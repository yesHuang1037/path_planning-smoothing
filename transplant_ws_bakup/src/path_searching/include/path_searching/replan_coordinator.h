#ifndef PATH_SEARCHING_REPLAN_COORDINATOR_H_
#define PATH_SEARCHING_REPLAN_COORDINATOR_H_

#include <Eigen/Core>

#include <string>

namespace path_searching
{

enum class AvoidanceMode
{
  GlobalReplan,
  GlobalMonitor,
  GlobalOnly
};

enum class ReplanReason
{
  NewGoal,
  Manual,
  MapUpdate,
  CostmapBlocked,
  PointcloudBlocked,
  PathCollision,
  Stall,
  Periodic
};

const char *avoidanceModeToString(AvoidanceMode mode);
const char *avoidanceModeToChinese(AvoidanceMode mode);
AvoidanceMode parseAvoidanceMode(const std::string &mode);
const char *replanReasonToString(ReplanReason reason);
const char *replanReasonToChinese(ReplanReason reason);

struct ReplanTriggerConfig
{
  bool new_goal{true};
  bool manual{true};
  bool map_update{false};
  bool costmap_blocked{true};
  bool pointcloud_blocked{true};
  bool path_collision{true};
  bool stall{true};
  bool periodic{true};
};

struct ReplanDecision
{
  bool should_replan{false};
  bool trigger_disabled{false};
  bool throttled{false};
  bool goal_too_close{false};
  ReplanReason reason{ReplanReason::Periodic};
  std::string message;
};

class ReplanCoordinator
{
public:
  struct Config
  {
    AvoidanceMode avoidance_mode{AvoidanceMode::GlobalReplan};
    ReplanTriggerConfig triggers;
    double min_replan_interval_sec{0.5};
    bool enable_stall_trigger{true};
    double stall_window_sec{8.0};
    double stall_move_threshold{0.05};
    double stall_goal_distance_threshold{0.30};
  };

  static ReplanTriggerConfig defaultTriggersForMode(AvoidanceMode mode);

  void configure(const Config &config);
  const Config &config() const { return config_; }
  AvoidanceMode mode() const { return config_.avoidance_mode; }

  bool triggerEnabled(ReplanReason reason) const;
  ReplanDecision request(
    ReplanReason reason,
    double now_sec,
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &goal);

  bool detectStall(double now_sec, const Eigen::Vector2d &start, const Eigen::Vector2d &goal);
  void resetStallAnchor(double now_sec, const Eigen::Vector2d &start);

private:
  bool shouldThrottle(ReplanReason reason) const;

  Config config_;
  double last_replan_time_sec_{-3600.0};
  double stall_anchor_time_sec_{0.0};
  Eigen::Vector2d stall_anchor_pose_{Eigen::Vector2d::Zero()};
  bool stall_anchor_initialized_{false};
};

}  // namespace path_searching

#endif  // PATH_SEARCHING_REPLAN_COORDINATOR_H_
