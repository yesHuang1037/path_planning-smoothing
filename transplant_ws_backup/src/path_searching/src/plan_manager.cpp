#include <rclcpp/rclcpp.hpp>
#include "esdf_map.h"
#include "Astar.h"
#include "path_searching/dynamic_obstacle_manager.h"
#include "path_searching/grid_geometry.h"
#include "path_searching/path_safety_checker.h"
#include "path_searching/replan_coordinator.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Core>
#include <cmath>
#include "smoother.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <algorithm>
#include <array>
#include <sstream>
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/bool.hpp>

class PlanManager : public rclcpp::Node
{
public:
  PlanManager()
  : Node("navi_planner"),
    map_geted(false),
    esdf_1(new ESDF_enviroment::esdf),
    planner_1(),
    smoother_1()
  {  
	    this->declare_parameter<std::string>("path_simplification_mode", "legacy");
	    this->declare_parameter<double>("path_simplification_tolerance_m", 0.1);
	    smoother_1.configureSimplification(
	      this->get_parameter("path_simplification_mode").as_string(),
	      this->get_parameter("path_simplification_tolerance_m").as_double());
	    // 初始化ROS2相关参数
	    this->declare_parameter<bool>("enable_downstairs", false);
	    this->declare_parameter<std::string>("avoidance_mode", "global_replan");
	    this->get_parameter("avoidance_mode", avoidance_mode_param_);
	    avoidance_mode_ = path_searching::parseAvoidanceMode(avoidance_mode_param_);
	    replan_trigger_config_ = path_searching::ReplanCoordinator::defaultTriggersForMode(avoidance_mode_);
	    this->declare_parameter<double>("obstacle_expand_radius", 0.40);
    // ROS1 legacy compatibility: old param name used in navigation_ws.
    this->declare_parameter<double>("voronoi_radius", -1.0);
    this->declare_parameter<double>("check_collision_radius", 0.50);
    this->declare_parameter<double>("obstacle_cost_weight", 0.0);
    this->declare_parameter<double>("dynamic_penalty_weight",100.0);
    // [DYN_GRID_REPLAN_V1] dynamic source + replan trigger params
    this->declare_parameter<bool>("use_pointcloud_dynamic_obstacles", true);
	    this->declare_parameter<bool>("use_costmap_dynamic_obstacles", false);
	    this->declare_parameter<bool>("use_costmap_clearance_fusion", true);
	    this->declare_parameter<bool>("enable_forward_path_check", avoidance_mode_ != path_searching::AvoidanceMode::GlobalOnly);
	    this->declare_parameter<bool>("enable_path_collision_check", avoidance_mode_ != path_searching::AvoidanceMode::GlobalOnly);
	    this->declare_parameter<bool>("enable_blocked_path_truncation", true);
    this->declare_parameter<double>("blocked_path_truncation_margin", 0.20);
    this->declare_parameter<bool>("enable_blocked_zone_memory", true);
    this->declare_parameter<double>("blocked_zone_memory_radius", 0.45);
    this->declare_parameter<double>("blocked_zone_memory_ttl_sec", 2.0);
    this->declare_parameter<std::string>("costmap_topic", "/costmap/costmap");
    this->declare_parameter<std::string>("costmap_update_topic", "/costmap/costmap_updates");
    this->declare_parameter<int>("costmap_occupied_threshold", 50);
    this->declare_parameter<double>("monitor_period_sec", 0.2);
    this->declare_parameter<double>("min_replan_interval_sec", 0.5);
	    this->declare_parameter<bool>("enable_stall_trigger", true);
	    this->declare_parameter<double>("stall_window_sec", 8.0);
	    this->declare_parameter<double>("stall_move_threshold", 0.05);
	    this->declare_parameter<double>("stall_goal_distance_threshold", 0.30);
	    this->declare_parameter<bool>("replan_triggers.new_goal", replan_trigger_config_.new_goal);
	    this->declare_parameter<bool>("replan_triggers.manual", replan_trigger_config_.manual);
	    this->declare_parameter<bool>("replan_triggers.map_update", replan_trigger_config_.map_update);
	    this->declare_parameter<bool>("replan_triggers.costmap_blocked", replan_trigger_config_.costmap_blocked);
	    this->declare_parameter<bool>("replan_triggers.pointcloud_blocked", replan_trigger_config_.pointcloud_blocked);
	    this->declare_parameter<bool>("replan_triggers.path_collision", replan_trigger_config_.path_collision);
	    this->declare_parameter<bool>("replan_triggers.stall", replan_trigger_config_.stall);
	    this->declare_parameter<bool>("replan_triggers.periodic", replan_trigger_config_.periodic);
	    this->declare_parameter<double>("robot_inscribed_radius", 0.25);
    this->declare_parameter<double>("clearance_margin", 0.05);
    this->declare_parameter<double>("forward_check_distance", 2.0);
    this->declare_parameter<double>("forward_sample_step", 0.10);
    this->declare_parameter<bool>("enable_start_escape_mode", true);
    this->declare_parameter<int>("start_escape_steps", 16);
    this->declare_parameter<bool>("enable_goal_escape_mode", true);
    this->declare_parameter<double>("goal_escape_search_radius", 0.80);
    this->declare_parameter<double>("goal_escape_clearance_tolerance", 0.08);

    this->get_parameter("enable_downstairs", enable_downstaris);
    this->get_parameter("obstacle_expand_radius", obstacle_expand_radius);
    double legacy_voronoi_radius = -1.0;
    this->get_parameter("voronoi_radius", legacy_voronoi_radius);
    if (legacy_voronoi_radius > 0.0) {
      obstacle_expand_radius = legacy_voronoi_radius;
    }
    this->get_parameter("check_collision_radius", collision_radius);
    this->get_parameter("obstacle_cost_weight", obstacle_cost_weight);
    this->get_parameter("dynamic_penalty_weight", dynamic_penalty_weight);
    // [DYN_GRID_REPLAN_V1] load dynamic/replan params
	    this->get_parameter("use_pointcloud_dynamic_obstacles", use_pointcloud_dynamic_obstacles_);
	    this->get_parameter("use_costmap_dynamic_obstacles", use_costmap_dynamic_obstacles_);
	    this->get_parameter("use_costmap_clearance_fusion", use_costmap_clearance_fusion_);
	    this->get_parameter("enable_forward_path_check", enable_forward_path_check_);
	    this->get_parameter("enable_path_collision_check", enable_path_collision_check_);
	    this->get_parameter("enable_blocked_path_truncation", enable_blocked_path_truncation_);
    this->get_parameter("blocked_path_truncation_margin", blocked_path_truncation_margin_);
    this->get_parameter("enable_blocked_zone_memory", enable_blocked_zone_memory_);
    this->get_parameter("blocked_zone_memory_radius", blocked_zone_memory_radius_);
    this->get_parameter("blocked_zone_memory_ttl_sec", blocked_zone_memory_ttl_sec_);
    this->get_parameter("costmap_topic", costmap_topic_);
    this->get_parameter("costmap_update_topic", costmap_update_topic_);
    this->get_parameter("costmap_occupied_threshold", costmap_occupied_threshold_);
    this->get_parameter("monitor_period_sec", monitor_period_sec_);
    this->get_parameter("min_replan_interval_sec", min_replan_interval_sec_);
	    this->get_parameter("enable_stall_trigger", enable_stall_trigger_);
	    this->get_parameter("stall_window_sec", stall_window_sec_);
	    this->get_parameter("stall_move_threshold", stall_move_threshold_);
	    this->get_parameter("stall_goal_distance_threshold", stall_goal_distance_threshold_);
	    this->get_parameter("replan_triggers.new_goal", replan_trigger_config_.new_goal);
	    this->get_parameter("replan_triggers.manual", replan_trigger_config_.manual);
	    this->get_parameter("replan_triggers.map_update", replan_trigger_config_.map_update);
	    this->get_parameter("replan_triggers.costmap_blocked", replan_trigger_config_.costmap_blocked);
	    this->get_parameter("replan_triggers.pointcloud_blocked", replan_trigger_config_.pointcloud_blocked);
	    this->get_parameter("replan_triggers.path_collision", replan_trigger_config_.path_collision);
	    this->get_parameter("replan_triggers.stall", replan_trigger_config_.stall);
	    this->get_parameter("replan_triggers.periodic", replan_trigger_config_.periodic);
	    this->get_parameter("robot_inscribed_radius", robot_inscribed_radius_);
    this->get_parameter("clearance_margin", clearance_margin_);
    this->get_parameter("forward_check_distance", forward_check_distance_);
    this->get_parameter("forward_sample_step", forward_sample_step_);
    this->get_parameter("enable_start_escape_mode", enable_start_escape_mode_);
    this->get_parameter("start_escape_steps", start_escape_steps_);
	    this->get_parameter("enable_goal_escape_mode", enable_goal_escape_mode_);
	    this->get_parameter("goal_escape_search_radius", goal_escape_search_radius_);
	    this->get_parameter("goal_escape_clearance_tolerance", goal_escape_clearance_tolerance_);

	    applyAvoidanceModeConstraints();
	    configureDynamicObstacleManager();
	    configurePathSafetyChecker();
	    configureReplanCoordinator();

	    RCLCPP_INFO(this->get_logger(), "[Params] [enable_downstairs] : %s", enable_downstaris ? "true" : "false");
	    RCLCPP_INFO(
	      this->get_logger(), "[Params] [avoidance_mode] : %s (%s)",
	      path_searching::avoidanceModeToString(avoidance_mode_),
	      path_searching::avoidanceModeToChinese(avoidance_mode_));
    RCLCPP_INFO(this->get_logger(), "[Params] [obstacle_expand_radius] : %f", obstacle_expand_radius);
    if (legacy_voronoi_radius > 0.0) {
      RCLCPP_INFO(this->get_logger(), "[Compat] use legacy param voronoi_radius: %f", legacy_voronoi_radius);
    }
    RCLCPP_INFO(this->get_logger(), "[Params] [check_collision_radius] : %f", collision_radius);
    RCLCPP_INFO(this->get_logger(), "[Params] [obstacle_cost_weight] : %f", obstacle_cost_weight);
	    RCLCPP_INFO(this->get_logger(), "[Params] [use_pointcloud_dynamic_obstacles] : %s", use_pointcloud_dynamic_obstacles_ ? "true" : "false");
	    RCLCPP_INFO(this->get_logger(), "[Params] [use_costmap_dynamic_obstacles] : %s", use_costmap_dynamic_obstacles_ ? "true" : "false");
	    RCLCPP_INFO(this->get_logger(), "[Params] [use_costmap_clearance_fusion] : %s", use_costmap_clearance_fusion_ ? "true" : "false");
	    RCLCPP_INFO(this->get_logger(), "[Params] [enable_forward_path_check] : %s", enable_forward_path_check_ ? "true" : "false");
	    RCLCPP_INFO(this->get_logger(), "[Params] [enable_path_collision_check] : %s", enable_path_collision_check_ ? "true" : "false");
	    RCLCPP_INFO(this->get_logger(), "[Params] [enable_blocked_path_truncation] : %s", enable_blocked_path_truncation_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [blocked_path_truncation_margin] : %f", blocked_path_truncation_margin_);
    RCLCPP_INFO(this->get_logger(), "[Params] [enable_blocked_zone_memory] : %s", enable_blocked_zone_memory_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [blocked_zone_memory_radius] : %f", blocked_zone_memory_radius_);
    RCLCPP_INFO(this->get_logger(), "[Params] [blocked_zone_memory_ttl_sec] : %f", blocked_zone_memory_ttl_sec_);
    RCLCPP_INFO(this->get_logger(), "[Params] [costmap_topic] : %s", costmap_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "[Params] [costmap_update_topic] : %s", costmap_update_topic_.c_str());
	    RCLCPP_INFO(this->get_logger(), "[Params] [monitor_period_sec] : %f", monitor_period_sec_);
	    RCLCPP_INFO(
	      this->get_logger(),
	      "[Params] [replan_triggers] new_goal=%s manual=%s map_update=%s costmap_blocked=%s pointcloud_blocked=%s path_collision=%s stall=%s periodic=%s",
	      replan_trigger_config_.new_goal ? "true" : "false",
	      replan_trigger_config_.manual ? "true" : "false",
	      replan_trigger_config_.map_update ? "true" : "false",
	      replan_trigger_config_.costmap_blocked ? "true" : "false",
	      replan_trigger_config_.pointcloud_blocked ? "true" : "false",
	      replan_trigger_config_.path_collision ? "true" : "false",
	      replan_trigger_config_.stall ? "true" : "false",
	      replan_trigger_config_.periodic ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [robot_inscribed_radius] : %f", robot_inscribed_radius_);
    RCLCPP_INFO(this->get_logger(), "[Params] [clearance_margin] : %f", clearance_margin_);
    RCLCPP_INFO(this->get_logger(), "[Params] [forward_check_distance] : %f", forward_check_distance_);
    RCLCPP_INFO(this->get_logger(), "[Params] [forward_sample_step] : %f", forward_sample_step_);
    RCLCPP_INFO(this->get_logger(), "[Params] [enable_start_escape_mode] : %s", enable_start_escape_mode_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [start_escape_steps] : %d", start_escape_steps_);
    RCLCPP_INFO(this->get_logger(), "[Params] [enable_goal_escape_mode] : %s", enable_goal_escape_mode_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [goal_escape_search_radius] : %f", goal_escape_search_radius_);
    RCLCPP_INFO(this->get_logger(), "[Params] [goal_escape_clearance_tolerance] : %f", goal_escape_clearance_tolerance_);

    // map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //   "/map", 1, std::bind(&PlanManager::map_callback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  "/map", 
  rclcpp::QoS(1).reliable().transient_local(),
  std::bind(&PlanManager::map_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1, std::bind(&PlanManager::goal_callback, this, std::placeholders::_1));
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_livox_obs", 1, std::bind(&PlanManager::setObstacle, this,std::placeholders::_1));
    cost_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PlanManager::cost_map_callback,this,std::placeholders::_1));
    cost_map_update_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      costmap_update_topic_,
      rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&PlanManager::cost_map_update_callback, this, std::placeholders::_1));
    
    //publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sPath", 1);
    reachable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ly/navi/reachable", 1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    start = Eigen::Vector2d::Zero();
    goal = Eigen::Vector2d::Zero();
    has_goal_ = false;
    //TODO
    is_first_goal_ = true;
    last_goal_ = Eigen::Vector2d::Zero();

    const double monitor_period = std::max(0.02, monitor_period_sec_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(monitor_period)),
      std::bind(&PlanManager::timer_callback, this));
	  }

private:
  bool map_geted;
  Eigen::Vector2i map_size;
  Eigen::Vector2d map_offset;
  double resolution;
	  ESDF_enviroment::Ptr esdf_1;
	  navi_planner::Astar planner_1;
	  navi_planner::smoother smoother_1;
	  path_searching::GridGeometry grid_geometry_;
	  path_searching::DynamicObstacleManager dynamic_obstacle_manager_;
	  path_searching::PathSafetyChecker path_safety_checker_;
	  path_searching::ReplanCoordinator replan_coordinator_;
	  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr cost_map_update_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reachable_pub_;
  Eigen::Vector2d start, goal;
  std::vector<Eigen::Vector2d> path_now;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  bool enable_downstaris;
  double obstacle_expand_radius, collision_radius;
  double obstacle_cost_weight;
  //新增的stvl动态障碍参数
  double dynamic_penalty_weight;
  bool has_goal_;
  const std::string global_frame_{"map"};
  const std::array<std::string, 2> robot_frame_candidates_{{"base_link", "baselink"}};
  // [DYN_GRID_REPLAN_V1] dynamic obstacle source and replan/stall params
  bool use_pointcloud_dynamic_obstacles_{true};
  bool use_costmap_dynamic_obstacles_{false};
	  bool use_costmap_clearance_fusion_{true};
	  path_searching::AvoidanceMode avoidance_mode_{path_searching::AvoidanceMode::GlobalReplan};
	  std::string avoidance_mode_param_{"global_replan"};
	  path_searching::ReplanTriggerConfig replan_trigger_config_;
	  bool enable_forward_path_check_{true};
	  bool enable_path_collision_check_{true};
	  bool enable_blocked_path_truncation_{true};
  double blocked_path_truncation_margin_{0.20};
  bool enable_blocked_zone_memory_{true};
  double blocked_zone_memory_radius_{0.45};
  double blocked_zone_memory_ttl_sec_{2.0};
  std::string costmap_topic_{"/costmap/costmap"};
  std::string costmap_update_topic_{"/costmap/costmap_updates"};
  int costmap_occupied_threshold_{50};
  double monitor_period_sec_{0.2};
  double min_replan_interval_sec_{0.5};
  bool enable_stall_trigger_{true};
  double stall_window_sec_{8.0};
  double stall_move_threshold_{0.05};
  double stall_goal_distance_threshold_{0.30};
  double robot_inscribed_radius_{0.25};
  double clearance_margin_{0.05};
  double forward_check_distance_{2.0};
  double forward_sample_step_{0.10};
  bool enable_start_escape_mode_{true};
  int start_escape_steps_{16};
  bool enable_goal_escape_mode_{true};
  double goal_escape_search_radius_{0.80};
  double goal_escape_clearance_tolerance_{0.08};
  //TODO
  rclcpp::TimerBase::SharedPtr timer_;
	  Eigen::Vector2d last_goal_;
	  bool is_first_goal_;
	  bool pending_truncation_replan_{false};
	  path_searching::ReplanReason pending_truncation_replan_reason_{path_searching::ReplanReason::PathCollision};
	  std::string pending_truncation_replan_detail_;

	  void applyAvoidanceModeConstraints()
	  {
	    if (avoidance_mode_ == path_searching::AvoidanceMode::GlobalOnly)
	    {
	      use_pointcloud_dynamic_obstacles_ = false;
	      use_costmap_dynamic_obstacles_ = false;
	      use_costmap_clearance_fusion_ = false;
	      enable_blocked_zone_memory_ = false;
	      enable_forward_path_check_ = false;
	      enable_path_collision_check_ = false;
	      enable_blocked_path_truncation_ = false;
	      enable_stall_trigger_ = false;
	      replan_trigger_config_.costmap_blocked = false;
	      replan_trigger_config_.pointcloud_blocked = false;
	      replan_trigger_config_.path_collision = false;
	      replan_trigger_config_.stall = false;
	    }
	    else if (avoidance_mode_ == path_searching::AvoidanceMode::GlobalMonitor)
	    {
	      enable_blocked_path_truncation_ = false;
	      replan_trigger_config_.costmap_blocked = false;
	      replan_trigger_config_.pointcloud_blocked = false;
	      replan_trigger_config_.path_collision = false;
	    }
	  }

	  void configureDynamicObstacleManager()
	  {
	    path_searching::DynamicObstacleManager::Config config;
	    config.use_pointcloud_dynamic_obstacles = use_pointcloud_dynamic_obstacles_;
	    config.use_costmap_dynamic_obstacles = use_costmap_dynamic_obstacles_;
	    config.use_costmap_clearance_fusion = use_costmap_clearance_fusion_;
	    config.enable_blocked_zone_memory = enable_blocked_zone_memory_;
	    config.blocked_zone_memory_radius = blocked_zone_memory_radius_;
	    config.blocked_zone_memory_ttl_sec = blocked_zone_memory_ttl_sec_;
	    config.costmap_occupied_threshold = costmap_occupied_threshold_;
	    config.obstacle_expand_radius = obstacle_expand_radius;
	    dynamic_obstacle_manager_.configure(config);
	  }

	  void configurePathSafetyChecker()
	  {
	    path_searching::PathSafetyChecker::Config config;
	    config.robot_inscribed_radius = robot_inscribed_radius_;
	    config.clearance_margin = clearance_margin_;
	    config.collision_radius = collision_radius;
	    config.forward_check_distance = forward_check_distance_;
	    config.forward_sample_step = forward_sample_step_;
	    config.blocked_path_truncation_margin = blocked_path_truncation_margin_;
	    config.enable_blocked_path_truncation = enable_blocked_path_truncation_;
	    path_safety_checker_.configure(config);
	  }

	  void configureReplanCoordinator()
	  {
	    path_searching::ReplanCoordinator::Config config;
	    config.avoidance_mode = avoidance_mode_;
	    config.triggers = replan_trigger_config_;
	    config.min_replan_interval_sec = min_replan_interval_sec_;
	    config.enable_stall_trigger = enable_stall_trigger_;
	    config.stall_window_sec = stall_window_sec_;
	    config.stall_move_threshold = stall_move_threshold_;
	    config.stall_goal_distance_threshold = stall_goal_distance_threshold_;
	    replan_coordinator_.configure(config);
	  }

	  // [DYN_GRID_REPLAN_V1] gate replan frequency and centralize replan entry
	  bool triggerReplan(
	    path_searching::ReplanReason reason,
	    const char *detail,
	    bool reset_stall_anchor = false)
	  {
	    if (!has_goal_) return false;
	    if (!getStart(false))
	    {
	      RCLCPP_WARN_THROTTLE(
	        this->get_logger(), *this->get_clock(), 1000,
	        "跳过重规划：无法获取当前机器人位姿");
	      return false;
	    }

	    const auto now = this->now();
	    const auto decision = replan_coordinator_.request(reason, now.seconds(), start, goal);
	    if (!decision.should_replan)
	    {
	      RCLCPP_DEBUG_THROTTLE(
	        this->get_logger(), *this->get_clock(), 1000,
	        "跳过重规划：原因=%s 详情=%s 判定=%s",
	        path_searching::replanReasonToChinese(reason),
	        detail,
	        decision.message.c_str());
	      return false;
	    }

	    RCLCPP_INFO(
	      this->get_logger(),
	      "触发重规划：原因=%s 详情=%s",
	      path_searching::replanReasonToChinese(reason),
	      detail);
	    plan(start, goal);
	    pending_truncation_replan_ = false;
	    pending_truncation_replan_detail_.clear();
	    if (reset_stall_anchor)
	    {
	      replan_coordinator_.resetStallAnchor(now.seconds(), start);
	    }
	    return true;
	  }

	  bool triggerReplan(const char *reason, bool reset_stall_anchor = false)
	  {
	    return triggerReplan(path_searching::ReplanReason::PathCollision, reason, reset_stall_anchor);
	  }

	  void scheduleReplanAfterTruncation(
	    path_searching::ReplanReason reason,
	    const char *detail)
	  {
	    pending_truncation_replan_ = true;
	    pending_truncation_replan_reason_ = reason;
	    pending_truncation_replan_detail_ = detail;
	    RCLCPP_INFO(
	      this->get_logger(),
	      "截断路径已发布，重规划延后到下一次监控周期：原因=%s 详情=%s",
	      path_searching::replanReasonToChinese(reason),
	      detail);
	  }

	  void logDynamicObstacleUpdate(const path_searching::ObstacleUpdateReport &report)
	  {
	    if (!report.accepted)
	    {
	      RCLCPP_DEBUG_THROTTLE(
	        this->get_logger(), *this->get_clock(), 2000,
	        "动态障碍更新跳过：来源=%s 原因=%s",
	        report.source.c_str(),
	        report.message.c_str());
	      return;
	    }

	    RCLCPP_INFO_THROTTLE(
	      this->get_logger(), *this->get_clock(), 1000,
	      "动态障碍更新：来源=%s %s occupied=%d unknown=%d accepted=%d inserted=%d removed=%d out_of_map=%d "
	      "动态障碍[点云=%zu costmap=%zu 阻塞记忆=%zu 合并=%zu]",
	      report.source.c_str(),
	      report.message.c_str(),
	      report.occupied_cells,
	      report.unknown_cells,
	      report.accepted_cells,
	      report.inserted_cells,
	      report.removed_cells,
	      report.out_of_map_cells,
	      report.counts.pointcloud_cells,
	      report.counts.costmap_cells,
	      report.counts.blocked_memory_cells,
	      report.counts.merged_cells);
	  }

	  void checkPathAfterDynamicObstacleUpdate(
	    path_searching::ReplanReason blocked_reason,
	    const char *source_label)
	  {
	    if (!has_goal_)
	    {
	      return;
	    }
	    if (!enable_forward_path_check_ && !enable_path_collision_check_)
	    {
	      return;
	    }
	    if (!getStart(false))
	    {
	      RCLCPP_WARN_THROTTLE(
	        this->get_logger(), *this->get_clock(), 1000,
	        "跳过动态障碍后的路径检查：无法获取当前机器人位姿");
	      return;
	    }

	    if (enable_forward_path_check_)
	    {
	      const auto report = path_safety_checker_.checkForward(path_now, start);
	      if (!report.passable)
	      {
	        RCLCPP_WARN(
	          this->get_logger(),
	          "动态障碍更新后前方路径不可通行：来源=%s 原因=%s 阻塞栅格=(%d,%d) min_clearance=%.3f m required=%.3f m",
	          source_label,
	          path_searching::pathBlockReasonToChinese(report.reason),
	          report.blocked_index[0], report.blocked_index[1],
	          report.min_clearance_m,
	          report.required_clearance_m);
	        if (truncatePathOnBlockingObstacle(source_label))
	        {
	          scheduleReplanAfterTruncation(blocked_reason, source_label);
	        }
	        else
	        {
	          triggerReplan(blocked_reason, source_label);
	        }
	        return;
	      }
	    }

	    if (enable_path_collision_check_)
	    {
	      const auto report = path_safety_checker_.checkWholePath(path_now);
	      if (!report.passable)
	      {
	        RCLCPP_WARN(
	          this->get_logger(),
	          "动态障碍更新后路径碰撞：来源=%s 原因=%s 阻塞栅格=(%d,%d) min_clearance=%.3f m required=%.3f m",
	          source_label,
	          path_searching::pathBlockReasonToChinese(report.reason),
	          report.blocked_index[0], report.blocked_index[1],
	          report.min_clearance_m,
	          report.required_clearance_m);
	        if (truncatePathOnBlockingObstacle(source_label))
	        {
	          scheduleReplanAfterTruncation(path_searching::ReplanReason::PathCollision, source_label);
	        }
	        else
	        {
	          triggerReplan(path_searching::ReplanReason::PathCollision, source_label);
	        }
	      }
	    }
	  }

  // [DYN_GRID_REPLAN_V1] stall detector: goal exists but robot hardly moved for a time window
  //检测机器人是否停滞，即在有路径情况下却并未移动，触发重规划
	  bool detectStall()
	  {
	    if (!has_goal_)
	    {
	      return false;
	    }
	    return replan_coordinator_.detectStall(this->now().seconds(), start, goal);
	  }
  // [修复]: 动态使用地图 resolution，不再硬编码除以 20.0
	  Eigen::Vector2d Index2pos(Eigen::Vector2i index_)
	  {
	    return grid_geometry_.valid() ? grid_geometry_.indexToWorld(index_) : Eigen::Vector2d::Zero();
	  }

  // [修复]: 动态使用地图 resolution，并使用 std::floor 确保坐标轴负半区转换正确
	  Eigen::Vector2i Pos2index(Eigen::Vector2d pos)
	  {
	    return grid_geometry_.valid() ? grid_geometry_.worldToIndex(pos) : Eigen::Vector2i::Zero();
	  }

	  double requiredClearanceMeters() const
	  {
	    return path_safety_checker_.requiredClearanceMeters();
	  }

	  double getCostmapClearanceMeters(
	    const Eigen::Vector2i &map_idx,
	    double required_clearance_m) const
	  {
	    return dynamic_obstacle_manager_.costmapClearanceMeters(map_idx, required_clearance_m);
	  }

	  bool getClearanceAtWorldPoint(
	    const Eigen::Vector2d &world_pt,
	    double &clearance_m,
	    double costmap_search_radius_m = -1.0)
	  {
	    const auto clearance = path_safety_checker_.clearanceAt(world_pt, costmap_search_radius_m);
	    clearance_m = clearance.valid ? clearance.clearance_m : 0.0;
	    return clearance.valid;
	  }

  void applyGoalEscapeIfNeeded(
    const Eigen::Vector2d &start_world,
    const Eigen::Vector2d &goal_world,
    Eigen::Vector2i &goal_index_io,
    Eigen::Vector2d &goal_world_used_io)
  {
    goal_world_used_io = goal_world;
    if (!enable_goal_escape_mode_ || !map_geted)
    {
      return;
    }

    const double required_clearance = requiredClearanceMeters();
    const double relaxed_clearance =
      std::max(0.0, required_clearance - std::max(0.0, goal_escape_clearance_tolerance_));

    double goal_clearance = 0.0;
    const bool goal_clearance_ok = getClearanceAtWorldPoint(goal_world, goal_clearance);
    const bool goal_collision = !goal_clearance_ok || esdf_1->checkCollision(goal_index_io);
    const bool goal_below_relaxed = !goal_clearance_ok || goal_clearance <= relaxed_clearance;
    if (!goal_collision && !goal_below_relaxed)
    {
      return;
    }

    const int radius_cells = std::max(1, static_cast<int>(std::ceil(goal_escape_search_radius_ / resolution)));
    const int radius_sq = radius_cells * radius_cells;

    auto select_candidate =
      [&](double clearance_threshold, Eigen::Vector2i &best_idx, double &best_clearance) -> bool
      {
        bool found = false;
        int best_goal_dist_sq = std::numeric_limits<int>::max();
        double best_start_dist_sq = std::numeric_limits<double>::infinity();
        best_clearance = 0.0;

        for (int dr = -radius_cells; dr <= radius_cells; ++dr)
        {
          for (int dc = -radius_cells; dc <= radius_cells; ++dc)
          {
            const int dist_sq = dr * dr + dc * dc;
            if (dist_sq > radius_sq)
            {
              continue;
            }

            Eigen::Vector2i idx(goal_index_io[0] + dr, goal_index_io[1] + dc);
            if (idx[0] < 0 || idx[1] < 0 || idx[0] >= map_size[0] || idx[1] >= map_size[1])
            {
              continue;
            }
            if (esdf_1->checkCollision(idx))
            {
              continue;
            }

            const Eigen::Vector2d cand_world = Index2pos(idx);
            double cand_clearance = 0.0;
            if (!getClearanceAtWorldPoint(cand_world, cand_clearance))
            {
              continue;
            }
            if (cand_clearance < clearance_threshold)
            {
              continue;
            }

            const double start_dist_sq = (cand_world - start_world).squaredNorm();
            const bool better =
              !found ||
              dist_sq < best_goal_dist_sq ||
              (dist_sq == best_goal_dist_sq && cand_clearance > best_clearance + 1e-6) ||
              (dist_sq == best_goal_dist_sq &&
               std::abs(cand_clearance - best_clearance) <= 1e-6 &&
               start_dist_sq < best_start_dist_sq);

            if (better)
            {
              found = true;
              best_goal_dist_sq = dist_sq;
              best_clearance = cand_clearance;
              best_start_dist_sq = start_dist_sq;
              best_idx = idx;
            }
          }
        }
        return found;
      };

    Eigen::Vector2i best_idx = goal_index_io;
    double best_clearance = 0.0;
    bool found = select_candidate(required_clearance, best_idx, best_clearance);
    if (!found)
    {
      found = select_candidate(relaxed_clearance, best_idx, best_clearance);
    }
    if (!found)
    {
      found = select_candidate(0.0, best_idx, best_clearance);
    }

    if (!found)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Goal escape failed: occupied/blocked goal at (%d,%d), radius=%.2f m, keep original goal.",
        goal_index_io[0], goal_index_io[1], goal_escape_search_radius_);
      return;
    }

    const Eigen::Vector2i old_idx = goal_index_io;
    goal_index_io = best_idx;
    goal_world_used_io = Index2pos(best_idx);
    RCLCPP_WARN(
      this->get_logger(),
      "Goal adjusted for escape: idx (%d,%d)->(%d,%d), clearance goal=%.3f used=%.3f required=%.3f relaxed=%.3f",
      old_idx[0], old_idx[1],
      best_idx[0], best_idx[1],
      goal_clearance,
      best_clearance,
      required_clearance,
      relaxed_clearance);
  }

	  bool checkForwardPassability(double &min_clearance_m)
	  {
	    if (!enable_forward_path_check_)
	    {
	      min_clearance_m = std::numeric_limits<double>::infinity();
	      return true;
	    }
	    const auto report = path_safety_checker_.checkForward(path_now, start);
	    min_clearance_m = report.min_clearance_m;
	    return report.passable;
	  }

	  bool truncatePathOnBlockingObstacle(const char *reason)
	  {
	    if (!enable_blocked_path_truncation_ || !map_geted || path_now.size() < 2)
	    {
	      return false;
	    }
	    if (!getStart(false))
	    {
	      return false;
	    }
	    const auto truncation = path_safety_checker_.truncateBeforeBlockedPoint(path_now, start);
	    if (!truncation.truncated)
	    {
	      return false;
	    }

	    path_now = truncation.path;
	    Path_pub(path_now);
	    const auto memory_report = dynamic_obstacle_manager_.addBlockedZone(
	      truncation.blocked_index,
	      this->now().seconds(),
	      reason);
	    RCLCPP_WARN(
	      this->get_logger(),
	      "路径已在阻塞点前截断：原因=%s 阻塞栅格=(%d,%d) 截断终点安全距离=%.3f m 动态障碍[点云=%zu costmap=%zu 阻塞记忆=%zu 合并=%zu]",
	      reason,
	      truncation.blocked_index[0], truncation.blocked_index[1],
	      truncation.endpoint_clearance_m,
	      memory_report.counts.pointcloud_cells,
	      memory_report.counts.costmap_cells,
	      memory_report.counts.blocked_memory_cells,
	      memory_report.counts.merged_cells);
	    return true;
	  }

  void Path_pub(const std::vector<Eigen::Vector2d> &path2pub)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = this->now();

    for (auto p : path2pub)
    {
      geometry_msgs::msg::PoseStamped vertex;
      vertex.header = path.header;
      vertex.pose.position.x = p[0];
      vertex.pose.position.y = p[1];
      vertex.pose.orientation.w = 1.0;
      path.poses.push_back(vertex);
	    }
	    path_pub_->publish(path);
	    RCLCPP_INFO(this->get_logger(), "全局路径已发布");
  }

  bool lookupLatestTransform(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &transform,
    bool verbose = true)
  {
    try
    {
      transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      if (verbose)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "TF lookup failed (%s <- %s): %s",
          target_frame.c_str(),
          source_frame.c_str(),
          ex.what());
      }
      else
      {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "TF lookup failed (%s <- %s): %s",
          target_frame.c_str(),
          source_frame.c_str(),
          ex.what());
      }
      return false;
    }
  }

  bool lookupLatestTransformEitherDirection(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &transform,
    bool verbose = true)
  {
    if (lookupLatestTransform(target_frame, source_frame, transform, verbose))
    {
      return true;
    }

    geometry_msgs::msg::TransformStamped inverse_transform;
    if (!lookupLatestTransform(source_frame, target_frame, inverse_transform, verbose))
    {
      return false;
    }

    Eigen::Isometry3d T_source_target = tf2::transformToEigen(inverse_transform.transform);
    Eigen::Isometry3d T_target_source = T_source_target.inverse();
    transform.header.stamp = inverse_transform.header.stamp;
    transform.header.frame_id = target_frame;
    transform.child_frame_id = source_frame;
    transform.transform = tf2::eigenToTransform(T_target_source).transform;
    return true;
  }

  bool poseToGlobal(
    const geometry_msgs::msg::PoseStamped &pose_in,
    Eigen::Vector2d &pose_xy_out,
    std::string *resolved_source_frame = nullptr)
  {
    const std::string source_frame = pose_in.header.frame_id.empty() ? global_frame_ : pose_in.header.frame_id;
    if (resolved_source_frame != nullptr)
    {
      *resolved_source_frame = source_frame;
    }

    geometry_msgs::msg::PoseStamped pose_latest = pose_in;
    pose_latest.header.frame_id = source_frame;
    pose_latest.header.stamp = builtin_interfaces::msg::Time();

    if (source_frame == global_frame_)
    {
      pose_xy_out[0] = pose_latest.pose.position.x;
      pose_xy_out[1] = pose_latest.pose.position.y;
      return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupLatestTransform(global_frame_, source_frame, transform))
    {
      return false;
    }

    geometry_msgs::msg::PoseStamped pose_in_global;
    tf2::doTransform(pose_latest, pose_in_global, transform);
    pose_xy_out[0] = pose_in_global.pose.position.x;
    pose_xy_out[1] = pose_in_global.pose.position.y;
    return true;
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    if (map_geted) { return; }
    RCLCPP_INFO(this->get_logger(), "Map received");
    
    int size = map->info.width * map->info.height;
    map_size[0] = map->info.height;
	    map_size[1] = map->info.width;
	    map_offset[0] = map->info.origin.position.x;
	    map_offset[1] = map->info.origin.position.y;
	    resolution = map->info.resolution;
	    grid_geometry_ = path_searching::GridGeometry(map_size, map_offset, resolution);
    
    std::cout << "map_size: " << map_size.transpose() << std::endl;
    std::cout << "map_offset: " << map_offset.transpose() << " resolution: " << resolution << std::endl;
    
    bool * bin_map_ = new bool[size];
    for (int i = 0; i < size; i++)
    {
      bin_map_[i] = map->data[i] ? true : false;
    }
    
    esdf_1->esdf_init(bin_map_, map->info.height, map->info.width, map_offset, enable_downstaris);
    
    // [修复]: 防止内存泄漏，初始化ESDF后释放临时分配的内存
    delete[] bin_map_; 
    
    RCLCPP_INFO(this->get_logger(), "ESDF map initialized");
    planner_1.setEnvironment(esdf_1);
    const double min_clearance_cells = requiredClearanceMeters() / resolution;
    planner_1.setParam(
      obstacle_cost_weight,
      dynamic_penalty_weight,
      min_clearance_cells,
      enable_start_escape_mode_,
      start_escape_steps_);
	    planner_1.init();
	    smoother_1.smoother_setEnvironment(esdf_1);
	    dynamic_obstacle_manager_.setMap(grid_geometry_, esdf_1);
	    path_safety_checker_.setMap(grid_geometry_, esdf_1, &dynamic_obstacle_manager_);
	    smoother_1.setSimplificationEnvironment(grid_geometry_, [this](const Eigen::Vector2i &index) {
	      if (!grid_geometry_.inBounds(index) || esdf_1->checkCollision(index)) return false;
	      const Eigen::Vector2d center = grid_geometry_.indexToWorld(index) +
	        Eigen::Vector2d::Constant(0.5 * grid_geometry_.resolution());
	      const auto clearance = path_safety_checker_.clearanceAt(center);
	      return clearance.valid && clearance.safe;
	    });

	    map_geted = true;
  }

	  void logNoPathDiagnostics(
	    const char *stage,
	    const Eigen::Vector2d &start_world,
	    const Eigen::Vector2d &goal_world_raw,
    const Eigen::Vector2d &goal_world_used,
    const Eigen::Vector2i &start_idx,
    const Eigen::Vector2i &goal_idx)
	  {
	    double start_clearance_m = 0.0;
	    double goal_clearance_m = 0.0;
	    const bool start_clearance_ok = getClearanceAtWorldPoint(start_world, start_clearance_m);
	    const bool goal_clearance_ok = getClearanceAtWorldPoint(goal_world_used, goal_clearance_m);
	    const bool start_in_map = grid_geometry_.inBounds(start_idx);
	    const bool goal_in_map = grid_geometry_.inBounds(goal_idx);
	    const bool start_collision = esdf_1->checkCollision(start_idx);
	    const bool goal_collision = esdf_1->checkCollision(goal_idx);
	    const double required_clearance = requiredClearanceMeters();

	    std::ostringstream cause;
	    if (!start_in_map || !goal_in_map)
	    {
	      cause << "起点或目标超出地图范围"
	            << "，起点栅格=(" << start_idx[0] << "," << start_idx[1] << ")"
	            << "，目标栅格=(" << goal_idx[0] << "," << goal_idx[1] << ")"
	            << "，地图尺寸=(" << map_size[0] << "," << map_size[1] << ")";
	    }
	    else if (start_collision)
	    {
	      cause << "起点栅格被占用，起点栅格=(" << start_idx[0] << "," << start_idx[1] << ")";
	    }
	    else if (goal_collision)
	    {
	      cause << "目标栅格被占用，目标栅格=(" << goal_idx[0] << "," << goal_idx[1] << ")";
	    }
	    else if (!start_clearance_ok)
	    {
	      cause << "起点安全距离无效，起点栅格=(" << start_idx[0] << "," << start_idx[1] << ")";
	    }
	    else if (!goal_clearance_ok)
	    {
	      cause << "目标安全距离无效，目标栅格=(" << goal_idx[0] << "," << goal_idx[1] << ")";
	    }
	    else if (start_clearance_m <= required_clearance)
	    {
	      cause << "起点安全距离不足，当前=" << start_clearance_m
	            << " m，要求>" << required_clearance << " m";
	    }
	    else if (goal_clearance_m <= required_clearance)
	    {
	      cause << "目标安全距离不足，当前=" << goal_clearance_m
	            << " m，要求>" << required_clearance << " m";
	    }
	    else
	    {
	      cause << planner_1.formatLastSearchFailureCauseChinese();
	    }

	    RCLCPP_ERROR(
	      this->get_logger(),
	      "路径规划失败：%s",
	      cause.str().c_str());

	    RCLCPP_DEBUG(
	      this->get_logger(),
	      "路径规划失败调试信息：阶段=%s 起点世界=(%.3f,%.3f) 原始目标世界=(%.3f,%.3f) 实际目标世界=(%.3f,%.3f)",
	      stage,
	      start_world[0], start_world[1],
	      goal_world_raw[0], goal_world_raw[1],
	      goal_world_used[0], goal_world_used[1]);
		  }

  void plan(Eigen::Vector2d start_, Eigen::Vector2d goal_)
  {
    if (!map_geted)
    {
	      RCLCPP_WARN(
	        this->get_logger(),
	        "路径规划跳过：静态地图尚未初始化 start=(%.3f,%.3f) goal=(%.3f,%.3f)",
	        start_[0], start_[1], goal_[0], goal_[1]);
      return;
    }

    // [修复]: 彻底清理多余的手动计算 sx, sy 逻辑，统一调用 Pos2index
    Eigen::Vector2i start_index = Pos2index(start_);
    Eigen::Vector2i end_index = Pos2index(goal_);
    Eigen::Vector2d planning_goal = goal_;

    // 越界保护
	    if (start_index[0] < 0 || start_index[0] >= map_size[0] ||
	        start_index[1] < 0 || start_index[1] >= map_size[1] ||
	        end_index[0] < 0 || end_index[0] >= map_size[0] ||
	        end_index[1] < 0 || end_index[1] >= map_size[1])
	    {
	        logNoPathDiagnostics(
	          "input_out_of_map",
	          start_,
          goal_,
          planning_goal,
          start_index,
          end_index);
		        return;
		    }

    applyGoalEscapeIfNeeded(start_, goal_, end_index, planning_goal);

			    RCLCPP_INFO(this->get_logger(), "开始全局路径规划...");
		    
		    auto beforeTime = std::chrono::steady_clock::now();
		    auto result = planner_1.search(start_index, end_index);
    bool retried_start_escape = false;
	    const bool start_in_collision = esdf_1->checkCollision(start_index);
	    if (result == navi_planner::Astar::NO_PATH && enable_start_escape_mode_ && start_in_collision)
	    {
	      const int retry_escape_steps = std::max(start_escape_steps_, 40);
	      if (retry_escape_steps > start_escape_steps_)
      {
        retried_start_escape = true;
	        RCLCPP_WARN(
	          this->get_logger(),
	          "A* 首次搜索失败且起点被占用，扩大起点脱困窗口后重试：%d -> %d 步",
	          start_escape_steps_,
	          retry_escape_steps);
        planner_1.reset();
        const double min_clearance_cells = requiredClearanceMeters() / resolution;
        planner_1.setParam(
          obstacle_cost_weight,
          dynamic_penalty_weight,
          min_clearance_cells,
          enable_start_escape_mode_,
          retry_escape_steps);
        result = planner_1.search(start_index, end_index);
        planner_1.setParam(
          obstacle_cost_weight,
          dynamic_penalty_weight,
          min_clearance_cells,
          enable_start_escape_mode_,
          start_escape_steps_);
      }
    }

    auto msg = std_msgs::msg::Bool();
    if (result == navi_planner::Astar::NO_PATH)
    {
      logNoPathDiagnostics(
        retried_start_escape ? "astar_no_path_after_retry" : "astar_no_path",
        start_,
        goal_,
        planning_goal,
        start_index,
        end_index);
      planner_1.reset();
      msg.data = false;
      reachable_pub_->publish(msg);
      return;
    }
    else
    {
        msg.data = true;
        reachable_pub_->publish(msg);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    double duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
    std::cout << "A* searching cost: " << duration_millsecond << "ms" << std::endl;
    
    std::vector<Eigen::Vector2i> Path_2i = planner_1.getPath();
    std::vector<Eigen::Vector2d> Path_2d;

    Path_2d.reserve(Path_2i.size() + 2);
    Path_2d.push_back(start_);
    for (auto p : Path_2i)
    {
      Eigen::Vector2d path_point = Index2pos(p);
      if ((path_point - Path_2d.back()).norm() < 1e-6)
      {
        continue;
      }
      Path_2d.push_back(path_point);
    }
    if ((planning_goal - Path_2d.back()).norm() >= 1e-6)
    {
      Path_2d.push_back(planning_goal);
    }

    beforeTime = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector2d> raw_path = Path_2d;
    try
    {
      Path_2d = smoother_1.smooth(Path_2d, 0.3f, 0.04f);
    }
    catch (const std::exception &error)
    {
      RCLCPP_ERROR(this->get_logger(), "Path processing rejected: %s", error.what());
      planner_1.reset();
      msg.data = false;
      reachable_pub_->publish(msg);
      return;
    }
    endTime = std::chrono::steady_clock::now();
    duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
    std::cout << "Smooth cost: " << duration_millsecond << "ms" << std::endl;

    if (Path_2d.empty())
    {
	      RCLCPP_WARN(this->get_logger(), "平滑后路径为空，回退使用原始 A* 路径");
      Path_2d = std::move(raw_path);
    }

    Path_2d.front() = start_;
    if ((Path_2d.back() - planning_goal).norm() < 1e-6)
    {
      Path_2d.back() = planning_goal;
    }
    else
    {
      Path_2d.push_back(planning_goal);
    }
    
    planner_1.reset();
    path_now = Path_2d;
    Path_pub(Path_2d);
  }

  bool getStart(bool verbose = true)
  {
    std::ostringstream tried_frames;
    for (const auto &robot_frame : robot_frame_candidates_)
    {
      geometry_msgs::msg::TransformStamped robot_global_pose;
      if (!lookupLatestTransform(global_frame_, robot_frame, robot_global_pose, verbose))
      {
        tried_frames << robot_frame << " ";
        continue;
      }

      double pose_x = robot_global_pose.transform.translation.x;
      double pose_y = robot_global_pose.transform.translation.y;
      start = Eigen::Vector2d(pose_x, pose_y);
      if (verbose)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Plan start resolved from frame '%s' at (%.3f, %.3f)",
          robot_frame.c_str(),
          pose_x,
          pose_y);
      }
      return true;
    }

    const std::array<std::string, 2> anchor_frame_candidates{{"odom", "lidar"}};
    for (const auto &robot_frame : robot_frame_candidates_)
    {
      for (const auto &anchor_frame : anchor_frame_candidates)
      {
        geometry_msgs::msg::TransformStamped map_to_anchor;
        geometry_msgs::msg::TransformStamped anchor_to_robot;
        if (!lookupLatestTransform(global_frame_, anchor_frame, map_to_anchor, verbose))
        {
          tried_frames << global_frame_ << "<-" << anchor_frame << " ";
          continue;
        }
        if (!lookupLatestTransformEitherDirection(anchor_frame, robot_frame, anchor_to_robot, verbose))
        {
          tried_frames << anchor_frame << "<->" << robot_frame << " ";
          continue;
        }

        Eigen::Isometry3d T_map_anchor = tf2::transformToEigen(map_to_anchor.transform);
        Eigen::Isometry3d T_anchor_robot = tf2::transformToEigen(anchor_to_robot.transform);
        Eigen::Isometry3d T_map_robot = T_map_anchor * T_anchor_robot;
        start = T_map_robot.translation().head<2>();
        if (verbose)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Plan start resolved from frame '%s' via anchor '%s' at (%.3f, %.3f)",
            robot_frame.c_str(),
            anchor_frame.c_str(),
            start[0],
            start[1]);
        }
        return true;
      }
    }

    if (verbose)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Unable to resolve robot start pose in %s. Tried frames: %s",
        global_frame_.c_str(),
        tried_frames.str().c_str());
    }
    else
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Unable to resolve robot start pose in %s. Tried frames: %s",
        global_frame_.c_str(),
        tried_frames.str().c_str());
    }
    return false;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr end)
  {
    if (!map_geted) {
	      RCLCPP_WARN(this->get_logger(), "尚未收到静态地图，跳过目标点规划");
      return;
    }

    Eigen::Vector2d goal_pt;
    std::string goal_source_frame;
    if (!poseToGlobal(*end, goal_pt, &goal_source_frame))
    {
	      RCLCPP_ERROR(this->get_logger(), "目标点坐标转换失败：无法转换到 %s 坐标系", global_frame_.c_str());
      return;
    }

    this->goal = goal_pt;
    has_goal_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Goal received in frame '%s', planned goal in %s: (%.3f, %.3f)",
      goal_source_frame.c_str(),
      global_frame_.c_str(),
      goal[0],
      goal[1]);

    double dist_to_last_goal = (goal_pt - last_goal_).norm();
    //TODO
    if (is_first_goal_ || dist_to_last_goal > 0.05) 
    {
      RCLCPP_INFO(this->get_logger(), "新目标点到达！立即触发规划。距离偏差: %f", dist_to_last_goal);
      last_goal_ = goal_pt;
      is_first_goal_ = false;

      if (!getStart()) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前位姿，跳过规划");
        return;
      }
      
      if ((start - goal).norm() < 0.2) {
        RCLCPP_INFO(this->get_logger(), "目标点离起点太近，忽略。");
        return;
      }

	      // 立即规划
	      triggerReplan(path_searching::ReplanReason::NewGoal, "新目标点到达", true);

      // 更新上一次的目标点和标志位
      
    }
    else 
    {
      // 目标点没变，直接 return，不阻塞回调！交由定时器处理
      // RCLCPP_DEBUG(this->get_logger(), "目标点未改变，交由定时器处理。");
      return; 
    }
    //BAK
    // if (!getStart())
    // {
    //   RCLCPP_WARN(this->get_logger(), "Skip planning because current robot pose is unavailable");
    //   return;
    // }

    // if ((start - goal).norm() < 0.3) {
    //   RCLCPP_INFO(this->get_logger(), "Goal too close to start, ignoring.");
    //   return;
    // }
    
    // plan(start, goal);
  }
		  void cost_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
		  {
		    const auto report = dynamic_obstacle_manager_.updateFromCostmapFull(*msg);
	    logDynamicObstacleUpdate(report);
	    checkPathAfterDynamicObstacleUpdate(
	      path_searching::ReplanReason::CostmapBlocked,
	      "costmap 全量更新");
	  }

		  void cost_map_update_callback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
		  {
		    const auto report = dynamic_obstacle_manager_.updateFromCostmapUpdate(*msg);
	    logDynamicObstacleUpdate(report);
	    checkPathAfterDynamicObstacleUpdate(
	      path_searching::ReplanReason::CostmapBlocked,
	      "costmap 增量更新");
	  }

	  bool detectCollision()
	  {
	    if (!enable_path_collision_check_)
	    {
	      return false;
	    }
	    const auto report = path_safety_checker_.checkWholePath(path_now);
	    if (report.passable)
	    {
	      return false;
	    }
	    RCLCPP_INFO(
	      this->get_logger(),
	      "检测到路径不可通行：%s 阻塞栅格=(%d,%d) min_clearance=%.3f m required=%.3f m",
	      path_searching::pathBlockReasonToChinese(report.reason),
	      report.blocked_index[0], report.blocked_index[1],
	      report.min_clearance_m,
	      report.required_clearance_m);
	    return true;
	  }

	  void timer_callback()
	  {
	    if (map_geted)
	    {
	      dynamic_obstacle_manager_.pruneExpiredBlockedZones(this->now().seconds());
	    }

    // 如果还没接到过目标，直接跳过
    if (!has_goal_) {
      return;
    }

    // 获取当前最新位姿
    if (!getStart(false)) {
      return;
    }

    // 防抖：如果你离目标已经很近了，就不必一直规划了
    if ((start - goal).norm() < 0.1) {
      return;
    }
	    if (pending_truncation_replan_)
	    {
	      const auto pending_reason = pending_truncation_replan_reason_;
	      const std::string pending_detail = pending_truncation_replan_detail_;
	      if (triggerReplan(pending_reason, pending_detail.c_str()))
	      {
	        pending_truncation_replan_ = false;
	        pending_truncation_replan_detail_.clear();
	      }
	      return;
	    }
	    if (enable_forward_path_check_)
	    {
	      const auto report = path_safety_checker_.checkForward(path_now, start);
	      if (!report.passable)
	      {
	        RCLCPP_WARN(
	          this->get_logger(),
	          "定时检查发现前方路径不可通行：%s 阻塞栅格=(%d,%d) min_clearance=%.3f m required=%.3f m",
	          path_searching::pathBlockReasonToChinese(report.reason),
	          report.blocked_index[0], report.blocked_index[1],
	          report.min_clearance_m,
	          report.required_clearance_m);
	        bool replan_scheduled = false;
	        if (truncatePathOnBlockingObstacle("定时前视检查"))
	        {
	          scheduleReplanAfterTruncation(
	            path_searching::ReplanReason::PathCollision,
	            "定时前视检查");
	          replan_scheduled = true;
	        }
	        else
	        {
	          replan_scheduled = triggerReplan(
	            path_searching::ReplanReason::PathCollision,
	            "定时前视检查");
	        }
	        if (replan_scheduled)
	        {
	          return;
	        }
	        // A mode may reject PathCollision while still allowing periodic replanning.
	      }
	    }
	    if (detectCollision()) {
	      // [DYN_GRID_REPLAN_V1] collision trigger from periodic timer
	      bool replan_scheduled = false;
	      if (truncatePathOnBlockingObstacle("定时碰撞检查"))
	      {
	        scheduleReplanAfterTruncation(
	          path_searching::ReplanReason::PathCollision,
	          "定时碰撞检查");
	        replan_scheduled = true;
	      }
	      else
	      {
	        replan_scheduled = triggerReplan(
	          path_searching::ReplanReason::PathCollision,
	          "定时碰撞检查");
	      }
	      if (replan_scheduled)
	      {
	        return;
	      }
	      // A mode may reject PathCollision while still allowing periodic replanning.
	    }
	    if (detectStall())
	    {
	      // [DYN_GRID_REPLAN_V1] stall trigger from periodic timer
	      triggerReplan(path_searching::ReplanReason::Stall, "机器人停滞", true);
	      return;
	    }

	    if (replan_coordinator_.triggerEnabled(path_searching::ReplanReason::Periodic))
	    {
	      triggerReplan(path_searching::ReplanReason::Periodic, "定时周期触发");
	    }
	  }

  void setObstacle(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!map_geted) return;
    if (!use_pointcloud_dynamic_obstacles_) return;
    pcl::PointCloud<pcl::PointXYZ> buffer;
    pcl::fromROSMsg(*cloud_msg, buffer);

    const std::string cloud_frame = cloud_msg->header.frame_id.empty() ? global_frame_ : cloud_msg->header.frame_id;
    if (cloud_frame != global_frame_)
    {
      geometry_msgs::msg::TransformStamped transform;
      if (!lookupLatestTransform(global_frame_, cloud_frame, transform))
      {
        RCLCPP_WARN(
          this->get_logger(),
	          "跳过点云障碍更新：点云坐标系 '%s' 无法转换到 %s",
          cloud_frame.c_str(),
          global_frame_.c_str());
        return;
      }

      Eigen::Matrix4f tf_matrix = tf2::transformToEigen(transform.transform).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZ> transformed_buffer;
      pcl::transformPointCloud(buffer, transformed_buffer, tf_matrix);
      buffer.swap(transformed_buffer);
    }

	    std::vector<Eigen::Vector2d> points_map_frame;
	    points_map_frame.reserve(buffer.points.size());
	    for (const auto &p : buffer.points)
	    {
	      points_map_frame.emplace_back(p.x, p.y);
	    }

	    const auto report = dynamic_obstacle_manager_.updateFromPointCloudMapFrame(points_map_frame);
	    logDynamicObstacleUpdate(report);
	    checkPathAfterDynamicObstacleUpdate(
	      path_searching::ReplanReason::PointcloudBlocked,
	      "点云动态障碍更新");
	  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
