#ifndef PATH_SEARCHING_PATH_SAFETY_CHECKER_H_
#define PATH_SEARCHING_PATH_SAFETY_CHECKER_H_

#include "esdf_map.h"
#include "path_searching/dynamic_obstacle_manager.h"
#include "path_searching/grid_geometry.h"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace path_searching
{

enum class PathBlockReason
{
  None,
  OutOfMap,
  InvalidClearance,
  ClearanceTooLow,
  Collision
};

const char *pathBlockReasonToChinese(PathBlockReason reason);

struct ClearanceResult
{
  bool valid{false};
  bool safe{false};
  double clearance_m{0.0};
  double required_clearance_m{0.0};
  Eigen::Vector2i index{Eigen::Vector2i::Zero()};
};

struct PathCheckReport
{
  bool passable{true};
  PathBlockReason reason{PathBlockReason::None};
  double min_clearance_m{0.0};
  double required_clearance_m{0.0};
  double checked_distance_m{0.0};
  Eigen::Vector2d blocked_point{Eigen::Vector2d::Zero()};
  Eigen::Vector2i blocked_index{Eigen::Vector2i::Zero()};
};

struct PathTruncationResult
{
  bool truncated{false};
  std::vector<Eigen::Vector2d> path;
  Eigen::Vector2d blocked_point{Eigen::Vector2d::Zero()};
  Eigen::Vector2i blocked_index{Eigen::Vector2i::Zero()};
  double endpoint_clearance_m{0.0};
};

class PathSafetyChecker
{
public:
  struct Config
  {
    double robot_inscribed_radius{0.25};
    double clearance_margin{0.05};
    double collision_radius{0.50};
    double forward_check_distance{2.0};
    double forward_sample_step{0.10};
    double blocked_path_truncation_margin{0.20};
    bool enable_blocked_path_truncation{true};
  };

  void configure(const Config &config);
  void setMap(
    const GridGeometry &geometry,
    const ESDF_enviroment::Ptr &esdf,
    const DynamicObstacleManager *dynamic_obstacles);

  double requiredClearanceMeters() const;
  ClearanceResult clearanceAt(
    const Eigen::Vector2d &world_pt,
    double costmap_search_radius_m = -1.0) const;
  PathCheckReport checkForward(
    const std::vector<Eigen::Vector2d> &path,
    const Eigen::Vector2d &current_pose) const;
  PathCheckReport checkWholePath(const std::vector<Eigen::Vector2d> &path) const;
  PathTruncationResult truncateBeforeBlockedPoint(
    const std::vector<Eigen::Vector2d> &path,
    const Eigen::Vector2d &current_pose) const;

private:
  size_t nearestPathIndex(
    const std::vector<Eigen::Vector2d> &path,
    const Eigen::Vector2d &current_pose) const;
  PathCheckReport blockedReport(
    PathBlockReason reason,
    const Eigen::Vector2d &point,
    double clearance_m,
    double checked_distance_m,
    double required_clearance_m) const;

  Config config_;
  GridGeometry geometry_;
  ESDF_enviroment::Ptr esdf_;
  const DynamicObstacleManager *dynamic_obstacles_{nullptr};
};

}  // namespace path_searching

#endif  // PATH_SEARCHING_PATH_SAFETY_CHECKER_H_
