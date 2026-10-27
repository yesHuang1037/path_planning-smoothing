#include "path_searching/dynamic_obstacle_manager.h"

#include <algorithm>
#include <cmath>

namespace path_searching
{

void DynamicObstacleManager::configure(const Config &config)
{
  config_ = config;
}

void DynamicObstacleManager::setMap(const GridGeometry &geometry, const ESDF_enviroment::Ptr &esdf)
{
  geometry_ = geometry;
  esdf_ = esdf;
  applyMergedDynamicObstacles();
}

void DynamicObstacleManager::clear()
{
  pointcloud_cells_.clear();
  costmap_cells_.clear();
  blocked_memory_cells_.clear();
  merged_dynamic_cells_.clear();
  costmap_cell_keys_.clear();
  blocked_memory_expire_time_.clear();
  has_costmap_info_ = false;
  applyMergedDynamicObstacles();
}

ObstacleCounts DynamicObstacleManager::counts() const
{
  return ObstacleCounts{
    pointcloud_cells_.size(),
    costmap_cells_.size(),
    blocked_memory_cells_.size(),
    merged_dynamic_cells_.size()};
}

ObstacleUpdateReport DynamicObstacleManager::updateFromPointCloudMapFrame(
  const std::vector<Eigen::Vector2d> &points_map_frame)
{
  ObstacleUpdateReport report;
  report.source = "pointcloud";
  if (!geometry_.valid() || !esdf_)
  {
    report.message = "静态地图尚未初始化，跳过点云动态障碍更新";
    report.counts = counts();
    return report;
  }
  if (!config_.use_pointcloud_dynamic_obstacles)
  {
    report.message = "点云动态障碍已禁用";
    report.counts = counts();
    return report;
  }

  const int expand_cells = geometry_.metersToCellsRound(config_.obstacle_expand_radius);
  const int expand_cells_sq = expand_cells * expand_cells;
  std::unordered_set<uint64_t> unique_cells;
  unique_cells.reserve(
    points_map_frame.size() *
    static_cast<size_t>((2 * expand_cells + 1) * (2 * expand_cells + 1)));

  for (const auto &pt : points_map_frame)
  {
    const Eigen::Vector2i center_idx = geometry_.worldToIndex(pt);
    for (int dr = -expand_cells; dr <= expand_cells; ++dr)
    {
      for (int dc = -expand_cells; dc <= expand_cells; ++dc)
      {
        if (dr * dr + dc * dc > expand_cells_sq)
        {
          continue;
        }
        const Eigen::Vector2i idx(center_idx[0] + dr, center_idx[1] + dc);
        if (!geometry_.inBounds(idx))
        {
          ++report.out_of_map_cells;
          continue;
        }
        unique_cells.insert(geometry_.packKey(idx));
      }
    }
  }

  std::vector<Eigen::Vector2i> next_cells;
  next_cells.reserve(unique_cells.size());
  for (const auto &key : unique_cells)
  {
    next_cells.push_back(geometry_.unpackKey(key));
  }

  report.accepted = true;
  report.changed = next_cells.size() != pointcloud_cells_.size();
  report.accepted_cells = static_cast<int>(next_cells.size());
  pointcloud_cells_.swap(next_cells);
  if (mergeDynamicSources())
  {
    report.changed = true;
  }
  applyMergedDynamicObstacles();
  report.counts = counts();
  report.message = "点云动态障碍已更新";
  return report;
}

ObstacleUpdateReport DynamicObstacleManager::updateFromCostmapFull(
  const nav_msgs::msg::OccupancyGrid &msg)
{
  latest_costmap_info_ = msg.info;
  has_costmap_info_ = true;

  return consumeCostmapRegion(
    msg.info,
    0,
    0,
    static_cast<int>(msg.info.width),
    static_cast<int>(msg.info.height),
    msg.data,
    "costmap_full");
}

ObstacleUpdateReport DynamicObstacleManager::updateFromCostmapUpdate(
  const map_msgs::msg::OccupancyGridUpdate &msg)
{
  ObstacleUpdateReport report;
  report.source = "costmap_update";
  if (!has_costmap_info_)
  {
    report.message = "尚未收到 costmap 全量地图，跳过增量更新";
    report.counts = counts();
    return report;
  }

  return consumeCostmapRegion(
    latest_costmap_info_,
    static_cast<int>(msg.y),
    static_cast<int>(msg.x),
    static_cast<int>(msg.width),
    static_cast<int>(msg.height),
    msg.data,
    "costmap_update");
}

ObstacleUpdateReport DynamicObstacleManager::addBlockedZone(
  const Eigen::Vector2i &blocked_idx,
  double now_sec,
  const std::string &reason)
{
  ObstacleUpdateReport report;
  report.source = "blocked_memory";
  if (!geometry_.valid() || !esdf_)
  {
    report.message = "静态地图尚未初始化，跳过阻塞记忆更新";
    report.counts = counts();
    return report;
  }
  if (!config_.enable_blocked_zone_memory || config_.blocked_zone_memory_ttl_sec <= 0.0)
  {
    report.message = "阻塞区域记忆已禁用";
    report.counts = counts();
    return report;
  }

  const int radius_cells = std::max(1, geometry_.metersToCellsCeil(config_.blocked_zone_memory_radius));
  const int radius_sq = radius_cells * radius_cells;
  const double expire_time = now_sec + config_.blocked_zone_memory_ttl_sec;
  for (int dr = -radius_cells; dr <= radius_cells; ++dr)
  {
    for (int dc = -radius_cells; dc <= radius_cells; ++dc)
    {
      if (dr * dr + dc * dc > radius_sq)
      {
        continue;
      }
      const Eigen::Vector2i idx(blocked_idx[0] + dr, blocked_idx[1] + dc);
      if (!geometry_.inBounds(idx))
      {
        ++report.out_of_map_cells;
        continue;
      }

      const uint64_t key = geometry_.packKey(idx);
      auto it = blocked_memory_expire_time_.find(key);
      if (it == blocked_memory_expire_time_.end())
      {
        blocked_memory_expire_time_.emplace(key, expire_time);
        ++report.inserted_cells;
      }
      else if (it->second < expire_time)
      {
        it->second = expire_time;
        ++report.accepted_cells;
      }
    }
  }

  report.accepted = true;
  report.changed = report.inserted_cells > 0 || report.accepted_cells > 0;
  rebuildBlockedMemoryCells();
  if (mergeDynamicSources())
  {
    report.changed = true;
  }
  applyMergedDynamicObstacles();
  report.counts = counts();
  report.message = "已记录临时阻塞区域：" + reason;
  return report;
}

bool DynamicObstacleManager::pruneExpiredBlockedZones(double now_sec)
{
  if (!config_.enable_blocked_zone_memory)
  {
    const bool had_any =
      !blocked_memory_expire_time_.empty() || !blocked_memory_cells_.empty();
    blocked_memory_expire_time_.clear();
    blocked_memory_cells_.clear();
    if (had_any)
    {
      mergeDynamicSources();
      applyMergedDynamicObstacles();
    }
    return had_any;
  }

  bool changed = false;
  for (auto it = blocked_memory_expire_time_.begin(); it != blocked_memory_expire_time_.end();)
  {
    if (it->second <= now_sec)
    {
      it = blocked_memory_expire_time_.erase(it);
      changed = true;
    }
    else
    {
      ++it;
    }
  }

  if (changed)
  {
    rebuildBlockedMemoryCells();
    mergeDynamicSources();
    applyMergedDynamicObstacles();
  }
  return changed;
}

double DynamicObstacleManager::costmapClearanceMeters(
  const Eigen::Vector2i &map_idx,
  double radius_m) const
{
  if (
    !config_.use_costmap_dynamic_obstacles ||
    !config_.use_costmap_clearance_fusion ||
    costmap_cell_keys_.empty() ||
    !geometry_.valid())
  {
    return std::numeric_limits<double>::infinity();
  }

  const uint64_t center_key = geometry_.packKey(map_idx);
  if (costmap_cell_keys_.find(center_key) != costmap_cell_keys_.end())
  {
    return 0.0;
  }

  const int radius_cells = std::max(1, geometry_.metersToCellsCeil(radius_m));
  const int radius_sq = radius_cells * radius_cells;
  double min_dist_cells = std::numeric_limits<double>::infinity();

  for (int dr = -radius_cells; dr <= radius_cells; ++dr)
  {
    for (int dc = -radius_cells; dc <= radius_cells; ++dc)
    {
      const int dist_sq = dr * dr + dc * dc;
      if (dist_sq > radius_sq)
      {
        continue;
      }

      const Eigen::Vector2i idx(map_idx[0] + dr, map_idx[1] + dc);
      if (!geometry_.inBounds(idx))
      {
        continue;
      }

      if (costmap_cell_keys_.find(geometry_.packKey(idx)) == costmap_cell_keys_.end())
      {
        continue;
      }

      min_dist_cells = std::min(min_dist_cells, std::sqrt(static_cast<double>(dist_sq)));
    }
  }

  if (!std::isfinite(min_dist_cells))
  {
    return std::numeric_limits<double>::infinity();
  }
  return min_dist_cells * geometry_.resolution();
}

int DynamicObstacleManager::normalizeCostmapCellToOcc100(int8_t raw_cell) const
{
  if (raw_cell >= 0 && raw_cell <= 100)
  {
    return static_cast<int>(raw_cell);
  }

  const int cell_u8 = static_cast<int>(static_cast<uint8_t>(raw_cell));
  if (cell_u8 == 255)
  {
    return -1;
  }
  return static_cast<int>(std::round(static_cast<double>(cell_u8) * 100.0 / 254.0));
}

bool DynamicObstacleManager::convertCostmapCellToStaticMapIndex(
  int costmap_row,
  int costmap_col,
  const nav_msgs::msg::MapMetaData &info,
  Eigen::Vector2i &map_idx_out) const
{
  Eigen::Vector2d world_pt;
  world_pt[0] = info.origin.position.x +
    (static_cast<double>(costmap_col) + 0.5) * static_cast<double>(info.resolution);
  world_pt[1] = info.origin.position.y +
    (static_cast<double>(costmap_row) + 0.5) * static_cast<double>(info.resolution);
  map_idx_out = geometry_.worldToIndex(world_pt);
  return geometry_.inBounds(map_idx_out);
}

ObstacleUpdateReport DynamicObstacleManager::consumeCostmapRegion(
  const nav_msgs::msg::MapMetaData &info,
  int start_row,
  int start_col,
  int region_width,
  int region_height,
  const std::vector<int8_t> &data,
  const std::string &source)
{
  ObstacleUpdateReport report;
  report.source = source;
  if (!geometry_.valid() || !esdf_)
  {
    report.message = "静态地图尚未初始化，跳过 costmap 动态障碍更新";
    report.counts = counts();
    return report;
  }
  if (!config_.use_costmap_dynamic_obstacles)
  {
    report.message = "costmap 动态障碍已禁用";
    report.counts = counts();
    return report;
  }

  const int map_width = static_cast<int>(info.width);
  const int map_height = static_cast<int>(info.height);
  if (map_width <= 0 || map_height <= 0 || data.empty())
  {
    report.message = "costmap 消息无效：地图尺寸或数据为空";
    report.counts = counts();
    return report;
  }
  if (start_row < 0 || start_col < 0 || region_width <= 0 || region_height <= 0)
  {
    report.message = "costmap 更新区域无效";
    report.counts = counts();
    return report;
  }
  if (start_row + region_height > map_height || start_col + region_width > map_width)
  {
    report.message = "costmap 更新区域超出地图范围";
    report.counts = counts();
    return report;
  }

  report.accepted = true;
  const bool full_region =
    start_row == 0 && start_col == 0 && region_width == map_width && region_height == map_height;
  if (full_region)
  {
    costmap_cell_keys_.clear();
  }

  const int threshold = std::max(0, std::min(100, config_.costmap_occupied_threshold));
  const int expected_cells = region_width * region_height;
  const int total_cells = std::min(static_cast<int>(data.size()), expected_cells);
  for (int idx = 0; idx < total_cells; ++idx)
  {
    const int local_row = idx / region_width;
    const int local_col = idx % region_width;
    const int row = start_row + local_row;
    const int col = start_col + local_col;

    const int occ = normalizeCostmapCellToOcc100(data[idx]);
    if (occ < 0)
    {
      ++report.unknown_cells;
    }
    if (occ >= threshold)
    {
      ++report.occupied_cells;
    }

    const bool should_insert = occ >= threshold;
    if (!should_insert && full_region)
    {
      continue;
    }

    Eigen::Vector2i map_idx;
    if (!convertCostmapCellToStaticMapIndex(row, col, info, map_idx))
    {
      ++report.out_of_map_cells;
      continue;
    }

    const uint64_t key = geometry_.packKey(map_idx);
    if (should_insert)
    {
      ++report.accepted_cells;
      if (costmap_cell_keys_.insert(key).second)
      {
        ++report.inserted_cells;
      }
    }
    else if (costmap_cell_keys_.erase(key) > 0)
    {
      ++report.removed_cells;
    }
  }

  report.changed = rebuildCostmapCellsFromKeySet();
  if (mergeDynamicSources())
  {
    report.changed = true;
  }
  applyMergedDynamicObstacles();
  report.counts = counts();
  report.message = "costmap 动态障碍已更新";
  return report;
}

void DynamicObstacleManager::rebuildBlockedMemoryCells()
{
  blocked_memory_cells_.clear();
  blocked_memory_cells_.reserve(blocked_memory_expire_time_.size());
  for (const auto &kv : blocked_memory_expire_time_)
  {
    blocked_memory_cells_.push_back(geometry_.unpackKey(kv.first));
  }
}

bool DynamicObstacleManager::rebuildCostmapCellsFromKeySet()
{
  const size_t old_size = costmap_cells_.size();
  costmap_cells_.clear();
  costmap_cells_.reserve(costmap_cell_keys_.size());
  for (const auto &key : costmap_cell_keys_)
  {
    costmap_cells_.push_back(geometry_.unpackKey(key));
  }
  return old_size != costmap_cells_.size();
}

bool DynamicObstacleManager::mergeDynamicSources()
{
  std::unordered_set<uint64_t> merged_keys;
  merged_keys.reserve(pointcloud_cells_.size() + costmap_cells_.size() + blocked_memory_cells_.size());

  auto add_cells = [&](const std::vector<Eigen::Vector2i> &cells)
  {
    for (const auto &idx : cells)
    {
      if (geometry_.inBounds(idx))
      {
        merged_keys.insert(geometry_.packKey(idx));
      }
    }
  };

  add_cells(pointcloud_cells_);
  add_cells(costmap_cells_);
  add_cells(blocked_memory_cells_);

  std::vector<Eigen::Vector2i> next_merged;
  next_merged.reserve(merged_keys.size());
  for (const auto &key : merged_keys)
  {
    next_merged.push_back(geometry_.unpackKey(key));
  }

  const bool changed = next_merged.size() != merged_dynamic_cells_.size();
  merged_dynamic_cells_.swap(next_merged);
  return changed;
}

void DynamicObstacleManager::applyMergedDynamicObstacles()
{
  if (esdf_)
  {
    esdf_->setDynamicObstacles(merged_dynamic_cells_);
  }
}

}  // namespace path_searching
