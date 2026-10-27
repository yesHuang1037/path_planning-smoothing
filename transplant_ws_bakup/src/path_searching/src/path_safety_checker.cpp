#include "path_searching/path_safety_checker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace path_searching
{

const char *pathBlockReasonToChinese(PathBlockReason reason)
{
  switch (reason)
  {
    case PathBlockReason::None:
      return "路径可通行";
    case PathBlockReason::OutOfMap:
      return "路径点超出地图范围";
    case PathBlockReason::InvalidClearance:
      return "路径点安全距离无效";
    case PathBlockReason::ClearanceTooLow:
      return "路径安全距离不足";
    case PathBlockReason::Collision:
      return "路径发生碰撞";
    default:
      return "未知路径阻塞原因";
  }
}

void PathSafetyChecker::configure(const Config &config)
{
  config_ = config;
}

void PathSafetyChecker::setMap(
  const GridGeometry &geometry,
  const ESDF_enviroment::Ptr &esdf,
  const DynamicObstacleManager *dynamic_obstacles)
{
  geometry_ = geometry;
  esdf_ = esdf;
  dynamic_obstacles_ = dynamic_obstacles;
}

double PathSafetyChecker::requiredClearanceMeters() const
{
  return std::max(0.0, config_.robot_inscribed_radius + config_.clearance_margin);
}

ClearanceResult PathSafetyChecker::clearanceAt(
  const Eigen::Vector2d &world_pt,
  double costmap_search_radius_m) const
{
  ClearanceResult result;
  result.required_clearance_m = requiredClearanceMeters();

  if (!geometry_.valid() || !esdf_)
  {
    result.index = Eigen::Vector2i::Zero();
    return result;
  }

  result.index = geometry_.worldToIndex(world_pt);
  if (!geometry_.inBounds(result.index))
  {
    return result;
  }

  const double dist_cells = esdf_->getDist(result.index);
  if (!std::isfinite(dist_cells))
  {
    return result;
  }

  const double esdf_clearance_m = dist_cells * geometry_.resolution();
  const double costmap_radius =
    (costmap_search_radius_m > 0.0) ? costmap_search_radius_m : result.required_clearance_m;
  const double costmap_clearance_m =
    dynamic_obstacles_ ? dynamic_obstacles_->costmapClearanceMeters(result.index, costmap_radius) :
    std::numeric_limits<double>::infinity();

  result.valid = true;
  result.clearance_m = std::min(esdf_clearance_m, costmap_clearance_m);
  result.safe = result.clearance_m > result.required_clearance_m;
  return result;
}

PathCheckReport PathSafetyChecker::checkForward(
  const std::vector<Eigen::Vector2d> &path,
  const Eigen::Vector2d &current_pose) const
{
  PathCheckReport report;
  report.passable = true;
  report.min_clearance_m = std::numeric_limits<double>::infinity();
  report.required_clearance_m = requiredClearanceMeters();

  if (!geometry_.valid() || !esdf_ || path.size() < 2 || config_.forward_check_distance <= 0.0)
  {
    return report;
  }
  if (report.required_clearance_m <= 0.0)
  {
    return report;
  }

  const size_t nearest_idx = nearestPathIndex(path, current_pose);
  const double sample_step = std::max(config_.forward_sample_step, geometry_.resolution());
  double checked_distance = 0.0;
  bool sampled_any = false;

  auto sample_and_check = [&](const Eigen::Vector2d &pt) -> bool
  {
    const ClearanceResult clearance = clearanceAt(pt);
    const double clearance_m = clearance.valid ? clearance.clearance_m : 0.0;
    sampled_any = true;
    report.min_clearance_m = std::min(report.min_clearance_m, clearance_m);
    if (!clearance.valid)
    {
      report = blockedReport(
        geometry_.inBounds(clearance.index) ? PathBlockReason::InvalidClearance : PathBlockReason::OutOfMap,
        pt,
        clearance_m,
        checked_distance,
        report.required_clearance_m);
      return false;
    }
    if (clearance_m <= report.required_clearance_m)
    {
      report = blockedReport(
        PathBlockReason::ClearanceTooLow,
        pt,
        clearance_m,
        checked_distance,
        report.required_clearance_m);
      return false;
    }
    return true;
  };

  if (!sample_and_check(current_pose))
  {
    return report;
  }

  for (size_t i = nearest_idx; i + 1 < path.size(); ++i)
  {
    const Eigen::Vector2d seg_start = path[i];
    const Eigen::Vector2d seg_end = path[i + 1];
    const Eigen::Vector2d seg_vec = seg_end - seg_start;
    const double seg_len = seg_vec.norm();
    if (seg_len < 1e-6)
    {
      continue;
    }

    const double seg_budget = config_.forward_check_distance - checked_distance;
    if (seg_budget <= 0.0)
    {
      break;
    }

    for (double s = 0.0; s <= seg_len; s += sample_step)
    {
      if (s > seg_budget)
      {
        break;
      }
      const Eigen::Vector2d pt = seg_start + seg_vec * (s / seg_len);
      if (!sample_and_check(pt))
      {
        return report;
      }
    }

    checked_distance += seg_len;
    report.checked_distance_m = checked_distance;
    if (checked_distance >= config_.forward_check_distance)
    {
      break;
    }
  }

  report.passable = sampled_any;
  return report;
}

PathCheckReport PathSafetyChecker::checkWholePath(const std::vector<Eigen::Vector2d> &path) const
{
  PathCheckReport report;
  report.passable = true;
  report.min_clearance_m = std::numeric_limits<double>::infinity();
  report.required_clearance_m =
    std::max(config_.collision_radius, requiredClearanceMeters());

  if (!geometry_.valid() || !esdf_ || path.empty())
  {
    return report;
  }

  for (size_t i = 1; i < path.size(); ++i)
  {
    const ClearanceResult clearance = clearanceAt(path[i], report.required_clearance_m);
    const double clearance_m = clearance.valid ? clearance.clearance_m : 0.0;
    report.min_clearance_m = std::min(report.min_clearance_m, clearance_m);
    if (!clearance.valid)
    {
      return blockedReport(
        geometry_.inBounds(clearance.index) ? PathBlockReason::InvalidClearance : PathBlockReason::OutOfMap,
        path[i],
        clearance_m,
        0.0,
        report.required_clearance_m);
    }
    if (clearance_m <= report.required_clearance_m)
    {
      return blockedReport(
        PathBlockReason::Collision,
        path[i],
        clearance_m,
        0.0,
        report.required_clearance_m);
    }
  }

  return report;
}

PathTruncationResult PathSafetyChecker::truncateBeforeBlockedPoint(
  const std::vector<Eigen::Vector2d> &path,
  const Eigen::Vector2d &current_pose) const
{
  PathTruncationResult result;
  result.path = path;
  if (
    !config_.enable_blocked_path_truncation ||
    !geometry_.valid() ||
    !esdf_ ||
    path.size() < 2)
  {
    return result;
  }

  const double truncation_extra_margin_m = std::max(0.0, config_.blocked_path_truncation_margin);
  const double required_clearance = requiredClearanceMeters();
  const double safe_endpoint_clearance = required_clearance + truncation_extra_margin_m;
  const size_t nearest_idx = nearestPathIndex(path, current_pose);
  const double sample_step = std::max(config_.forward_sample_step, geometry_.resolution());
  const double max_forward_distance =
    (config_.forward_check_distance > 0.0) ?
    config_.forward_check_distance : std::numeric_limits<double>::infinity();

  double checked_distance = 0.0;
  bool found_block = false;
  bool has_safe_endpoint = false;
  Eigen::Vector2d safe_endpoint = current_pose;
  Eigen::Vector2d blocked_point = current_pose;
  size_t safe_segment_index = nearest_idx;

  const ClearanceResult start_clearance = clearanceAt(current_pose, safe_endpoint_clearance);
  if (start_clearance.valid && start_clearance.clearance_m > safe_endpoint_clearance)
  {
    has_safe_endpoint = true;
    safe_endpoint = current_pose;
    safe_segment_index = nearest_idx;
  }

  for (size_t i = nearest_idx; i + 1 < path.size(); ++i)
  {
    const Eigen::Vector2d seg_start = path[i];
    const Eigen::Vector2d seg_end = path[i + 1];
    const Eigen::Vector2d seg_vec = seg_end - seg_start;
    const double seg_len = seg_vec.norm();
    if (seg_len < 1e-6)
    {
      continue;
    }

    const double seg_budget = max_forward_distance - checked_distance;
    if (seg_budget <= 0.0)
    {
      break;
    }
    const double seg_iter_limit = std::min(seg_len, seg_budget);

    for (double s = 0.0; s <= seg_iter_limit; s += sample_step)
    {
      const Eigen::Vector2d pt = seg_start + seg_vec * (s / seg_len);
      const ClearanceResult required = clearanceAt(pt, required_clearance);
      const double required_clearance_m = required.valid ? required.clearance_m : 0.0;
      if (required_clearance_m <= required_clearance)
      {
        found_block = true;
        blocked_point = pt;
        break;
      }

      ClearanceResult safe = required;
      if (safe_endpoint_clearance > required_clearance + 1e-6)
      {
        safe = clearanceAt(pt, safe_endpoint_clearance);
      }
      if (safe.valid && safe.clearance_m > safe_endpoint_clearance)
      {
        has_safe_endpoint = true;
        safe_endpoint = pt;
        safe_segment_index = i;
      }
    }

    if (found_block)
    {
      break;
    }

    checked_distance += seg_len;
    if (checked_distance >= max_forward_distance)
    {
      break;
    }
  }

  if (!found_block)
  {
    return result;
  }

  std::vector<Eigen::Vector2d> truncated_path;
  truncated_path.reserve(path.size());
  truncated_path.push_back(current_pose);

  if (has_safe_endpoint)
  {
    for (size_t i = nearest_idx + 1; i <= safe_segment_index && i < path.size(); ++i)
    {
      if ((path[i] - truncated_path.back()).norm() > 1e-6)
      {
        truncated_path.push_back(path[i]);
      }
    }
    if ((safe_endpoint - truncated_path.back()).norm() > 1e-6)
    {
      truncated_path.push_back(safe_endpoint);
    }
  }

  if (truncated_path.size() < 2)
  {
    truncated_path.push_back(current_pose);
  }

  result.truncated = true;
  result.path = std::move(truncated_path);
  result.blocked_point = blocked_point;
  result.blocked_index = geometry_.worldToIndex(blocked_point);
  const ClearanceResult endpoint_clearance = clearanceAt(result.path.back(), safe_endpoint_clearance);
  result.endpoint_clearance_m = endpoint_clearance.valid ? endpoint_clearance.clearance_m : 0.0;
  return result;
}

size_t PathSafetyChecker::nearestPathIndex(
  const std::vector<Eigen::Vector2d> &path,
  const Eigen::Vector2d &current_pose) const
{
  size_t nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.size(); ++i)
  {
    const double d = (path[i] - current_pose).norm();
    if (d < nearest_dist)
    {
      nearest_dist = d;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

PathCheckReport PathSafetyChecker::blockedReport(
  PathBlockReason reason,
  const Eigen::Vector2d &point,
  double clearance_m,
  double checked_distance_m,
  double required_clearance_m) const
{
  PathCheckReport report;
  report.passable = false;
  report.reason = reason;
  report.min_clearance_m = clearance_m;
  report.required_clearance_m = required_clearance_m;
  report.checked_distance_m = checked_distance_m;
  report.blocked_point = point;
  report.blocked_index = geometry_.valid() ? geometry_.worldToIndex(point) : Eigen::Vector2i::Zero();
  return report;
}

}  // namespace path_searching
