#ifndef PATH_SEARCHING_DYNAMIC_OBSTACLE_MANAGER_H_
#define PATH_SEARCHING_DYNAMIC_OBSTACLE_MANAGER_H_

#include "esdf_map.h"
#include "path_searching/grid_geometry.h"

#include <Eigen/Core>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace path_searching
{

struct ObstacleCounts
{
  size_t pointcloud_cells{0};
  size_t costmap_cells{0};
  size_t blocked_memory_cells{0};
  size_t merged_cells{0};
};

struct ObstacleUpdateReport
{
  bool accepted{false};
  bool changed{false};
  std::string source;
  std::string message;
  int occupied_cells{0};
  int unknown_cells{0};
  int accepted_cells{0};
  int inserted_cells{0};
  int removed_cells{0};
  int out_of_map_cells{0};
  ObstacleCounts counts;
};

class DynamicObstacleManager
{
public:
  struct Config
  {
    bool use_pointcloud_dynamic_obstacles{true};
    bool use_costmap_dynamic_obstacles{false};
    bool use_costmap_clearance_fusion{true};
    bool enable_blocked_zone_memory{true};
    double blocked_zone_memory_radius{0.45};
    double blocked_zone_memory_ttl_sec{2.0};
    int costmap_occupied_threshold{50};
    double obstacle_expand_radius{0.40};
  };

  void configure(const Config &config);
  void setMap(const GridGeometry &geometry, const ESDF_enviroment::Ptr &esdf);
  void clear();

  ObstacleUpdateReport updateFromPointCloudMapFrame(
    const std::vector<Eigen::Vector2d> &points_map_frame);
  ObstacleUpdateReport updateFromCostmapFull(const nav_msgs::msg::OccupancyGrid &msg);
  ObstacleUpdateReport updateFromCostmapUpdate(const map_msgs::msg::OccupancyGridUpdate &msg);
  ObstacleUpdateReport addBlockedZone(
    const Eigen::Vector2i &blocked_idx,
    double now_sec,
    const std::string &reason);
  bool pruneExpiredBlockedZones(double now_sec);

  double costmapClearanceMeters(const Eigen::Vector2i &map_idx, double radius_m) const;
  ObstacleCounts counts() const;
  bool hasCostmapInfo() const { return has_costmap_info_; }

private:
  int normalizeCostmapCellToOcc100(int8_t raw_cell) const;
  bool convertCostmapCellToStaticMapIndex(
    int costmap_row,
    int costmap_col,
    const nav_msgs::msg::MapMetaData &info,
    Eigen::Vector2i &map_idx_out) const;
  ObstacleUpdateReport consumeCostmapRegion(
    const nav_msgs::msg::MapMetaData &info,
    int start_row,
    int start_col,
    int region_width,
    int region_height,
    const std::vector<int8_t> &data,
    const std::string &source);

  void rebuildBlockedMemoryCells();
  bool rebuildCostmapCellsFromKeySet();
  bool mergeDynamicSources();
  void applyMergedDynamicObstacles();

  Config config_;
  GridGeometry geometry_;
  ESDF_enviroment::Ptr esdf_;
  std::vector<Eigen::Vector2i> pointcloud_cells_;
  std::vector<Eigen::Vector2i> costmap_cells_;
  std::vector<Eigen::Vector2i> blocked_memory_cells_;
  std::vector<Eigen::Vector2i> merged_dynamic_cells_;
  std::unordered_set<uint64_t> costmap_cell_keys_;
  std::unordered_map<uint64_t, double> blocked_memory_expire_time_;
  nav_msgs::msg::MapMetaData latest_costmap_info_;
  bool has_costmap_info_{false};
};

}  // namespace path_searching

#endif  // PATH_SEARCHING_DYNAMIC_OBSTACLE_MANAGER_H_
