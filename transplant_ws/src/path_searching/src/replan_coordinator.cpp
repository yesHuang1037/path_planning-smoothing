#include "path_searching/replan_coordinator.h"

#include <algorithm>

namespace path_searching
{

const char *avoidanceModeToString(AvoidanceMode mode)
{
  switch (mode)
  {
    case AvoidanceMode::GlobalReplan:
      return "global_replan";
    case AvoidanceMode::GlobalMonitor:
      return "global_monitor";
    case AvoidanceMode::GlobalOnly:
      return "global_only";
    default:
      return "global_replan";
  }
}

const char *avoidanceModeToChinese(AvoidanceMode mode)
{
  switch (mode)
  {
    case AvoidanceMode::GlobalReplan:
      return "全局规划承担动态避障重规划";
    case AvoidanceMode::GlobalMonitor:
      return "只监控动态障碍，不主动避障重规划";
    case AvoidanceMode::GlobalOnly:
      return "纯全局规划";
    default:
      return "未知规划模式";
  }
}

AvoidanceMode parseAvoidanceMode(const std::string &mode)
{
  if (mode == "global_only")
  {
    return AvoidanceMode::GlobalOnly;
  }
  if (mode == "global_monitor")
  {
    return AvoidanceMode::GlobalMonitor;
  }
  return AvoidanceMode::GlobalReplan;
}

const char *replanReasonToString(ReplanReason reason)
{
  switch (reason)
  {
    case ReplanReason::NewGoal:
      return "new_goal";
    case ReplanReason::Manual:
      return "manual";
    case ReplanReason::MapUpdate:
      return "map_update";
    case ReplanReason::CostmapBlocked:
      return "costmap_blocked";
    case ReplanReason::PointcloudBlocked:
      return "pointcloud_blocked";
    case ReplanReason::PathCollision:
      return "path_collision";
    case ReplanReason::Stall:
      return "stall";
    case ReplanReason::Periodic:
      return "periodic";
    default:
      return "unknown";
  }
}

const char *replanReasonToChinese(ReplanReason reason)
{
  switch (reason)
  {
    case ReplanReason::NewGoal:
      return "收到新目标";
    case ReplanReason::Manual:
      return "手动重规划";
    case ReplanReason::MapUpdate:
      return "静态地图更新";
    case ReplanReason::CostmapBlocked:
      return "costmap 检测到当前路径被阻塞";
    case ReplanReason::PointcloudBlocked:
      return "点云检测到当前路径被阻塞";
    case ReplanReason::PathCollision:
      return "当前路径碰撞或安全距离不足";
    case ReplanReason::Stall:
      return "机器人停滞";
    case ReplanReason::Periodic:
      return "周期性重规划";
    default:
      return "未知重规划原因";
  }
}

ReplanTriggerConfig ReplanCoordinator::defaultTriggersForMode(AvoidanceMode mode)
{
  ReplanTriggerConfig triggers;
  switch (mode)
  {
    case AvoidanceMode::GlobalReplan:
      triggers.new_goal = true;
      triggers.manual = true;
      triggers.map_update = false;
      triggers.costmap_blocked = true;
      triggers.pointcloud_blocked = true;
      triggers.path_collision = true;
      triggers.stall = true;
      triggers.periodic = true;
      break;
    case AvoidanceMode::GlobalMonitor:
      triggers.new_goal = true;
      triggers.manual = true;
      triggers.map_update = false;
      triggers.costmap_blocked = false;
      triggers.pointcloud_blocked = false;
      triggers.path_collision = false;
      triggers.stall = true;
      triggers.periodic = true;
      break;
    case AvoidanceMode::GlobalOnly:
      triggers.new_goal = true;
      triggers.manual = true;
      triggers.map_update = false;
      triggers.costmap_blocked = false;
      triggers.pointcloud_blocked = false;
      triggers.path_collision = false;
      triggers.stall = false;
      triggers.periodic = true;
      break;
  }
  return triggers;
}

void ReplanCoordinator::configure(const Config &config)
{
  config_ = config;
  if (config_.avoidance_mode == AvoidanceMode::GlobalOnly)
  {
    config_.triggers.costmap_blocked = false;
    config_.triggers.pointcloud_blocked = false;
    config_.triggers.path_collision = false;
    config_.triggers.stall = false;
    config_.enable_stall_trigger = false;
  }
  else if (config_.avoidance_mode == AvoidanceMode::GlobalMonitor)
  {
    config_.triggers.costmap_blocked = false;
    config_.triggers.pointcloud_blocked = false;
    config_.triggers.path_collision = false;
  }
}

bool ReplanCoordinator::triggerEnabled(ReplanReason reason) const
{
  switch (reason)
  {
    case ReplanReason::NewGoal:
      return config_.triggers.new_goal;
    case ReplanReason::Manual:
      return config_.triggers.manual;
    case ReplanReason::MapUpdate:
      return config_.triggers.map_update;
    case ReplanReason::CostmapBlocked:
      return config_.triggers.costmap_blocked;
    case ReplanReason::PointcloudBlocked:
      return config_.triggers.pointcloud_blocked;
    case ReplanReason::PathCollision:
      return config_.triggers.path_collision;
    case ReplanReason::Stall:
      return config_.triggers.stall;
    case ReplanReason::Periodic:
      return config_.triggers.periodic;
    default:
      return false;
  }
}

ReplanDecision ReplanCoordinator::request(
  ReplanReason reason,
  double now_sec,
  const Eigen::Vector2d &start,
  const Eigen::Vector2d &goal)
{
  ReplanDecision decision;
  decision.reason = reason;

  if (!triggerEnabled(reason))
  {
    decision.trigger_disabled = true;
    decision.message = "重规划触发已被参数禁用";
    return decision;
  }

  if ((start - goal).norm() < 0.1)
  {
    decision.goal_too_close = true;
    decision.message = "机器人距离目标过近，跳过重规划";
    return decision;
  }

  if (shouldThrottle(reason) && now_sec - last_replan_time_sec_ < config_.min_replan_interval_sec)
  {
    decision.throttled = true;
    decision.message = "距离上次重规划时间过短，跳过本次触发";
    return decision;
  }

  last_replan_time_sec_ = now_sec;
  decision.should_replan = true;
  decision.message = "允许重规划";
  return decision;
}

bool ReplanCoordinator::detectStall(
  double now_sec,
  const Eigen::Vector2d &start,
  const Eigen::Vector2d &goal)
{
  if (!config_.enable_stall_trigger || !triggerEnabled(ReplanReason::Stall))
  {
    return false;
  }
  if ((start - goal).norm() < config_.stall_goal_distance_threshold)
  {
    return false;
  }

  if (!stall_anchor_initialized_)
  {
    resetStallAnchor(now_sec, start);
    return false;
  }

  const double moved = (start - stall_anchor_pose_).norm();
  if (moved > config_.stall_move_threshold)
  {
    resetStallAnchor(now_sec, start);
    return false;
  }

  return now_sec - stall_anchor_time_sec_ >= config_.stall_window_sec;
}

void ReplanCoordinator::resetStallAnchor(double now_sec, const Eigen::Vector2d &start)
{
  stall_anchor_pose_ = start;
  stall_anchor_time_sec_ = now_sec;
  stall_anchor_initialized_ = true;
}

bool ReplanCoordinator::shouldThrottle(ReplanReason reason) const
{
  return reason != ReplanReason::NewGoal && reason != ReplanReason::Manual;
}

}  // namespace path_searching
