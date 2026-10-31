#include "tdt_path_planning/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <future>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <utility>

namespace {
class SnapshotPlanner : public rclcpp::Node {
public:
  SnapshotPlanner() : Node("snapshot_planner"), buffer_(get_clock()), listener_(buffer_) {
    if (declare_parameter<bool>("enable_execution", false))
      throw std::invalid_argument("execution is not implemented; debug node cannot enable execution");
#define LOAD_DOUBLE(name) config_.name = declare_parameter<double>(#name, config_.name)
    const auto integer = [this](const char *name, int initial) {
      const auto value = declare_parameter<int64_t>(name, initial);
      if (value < 0 || value > std::numeric_limits<int>::max()) throw std::invalid_argument(name);
      return static_cast<int>(value);
    };
#define LOAD_INT(name) config_.name = integer(#name, config_.name)
    LOAD_DOUBLE(body_length); LOAD_DOUBLE(body_width); LOAD_DOUBLE(clearance_margin);
    LOAD_DOUBLE(obstacle_weight); LOAD_DOUBLE(simplify_epsilon); LOAD_DOUBLE(corridor_range);
    LOAD_DOUBLE(allocation_speed); LOAD_DOUBLE(allocation_acceleration); LOAD_DOUBLE(min_segment_time);
    LOAD_DOUBLE(max_chord); LOAD_DOUBLE(curve_error); LOAD_DOUBLE(output_spacing);
    LOAD_DOUBLE(search_budget_ms); LOAD_DOUBLE(qp_budget_ms); LOAD_DOUBLE(total_budget_ms);
    LOAD_INT(occupied_threshold); LOAD_INT(max_refine_rounds); LOAD_INT(max_control_points);
    LOAD_INT(max_check_samples); LOAD_INT(max_search_expansions); LOAD_INT(qp_max_iterations);
#undef LOAD_DOUBLE
#undef LOAD_INT
    config_.validate();
    static_only_ = declare_parameter<bool>("static_only", false);
    snapshot_file_ = declare_parameter<std::string>("snapshot_file", "");
    frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    max_age_ = declare_parameter<double>("max_input_age_s", 0.5);
    max_start_drift_ = declare_parameter<double>("max_start_drift_m", 0.05);
    if (frame_.empty() || robot_frame_.empty() || !std::isfinite(max_age_) || max_age_ <= 0 ||
      !std::isfinite(max_start_drift_) || max_start_drift_ <= 0)
      throw std::invalid_argument("invalid frame or freshness limit");
    raw_ = create_publisher<nav_msgs::msg::Path>("~/raw_path", 1);
    simplified_ = create_publisher<nav_msgs::msg::Path>("~/simplified_path", 1);
    optimized_ = create_publisher<nav_msgs::msg::Path>("~/optimized_path", 1);
    corridors_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/corridors", 1);
    status_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local());
    map_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        try {
          auto grid = convert(*message);
          base_ = std::move(grid);
          base_error_.clear();
        } catch (const std::exception &error) {
          base_.reset();
          base_error_ = std::string("static map rejected: ") + error.what();
        }
      });
    dynamic_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap/costmap",
      rclcpp::QoS(1).transient_local(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        if (static_only_) return;
        try {
          auto grid = convert(*message);
          const rclcpp::Time stamp(message->header.stamp);
          dynamic_ = std::move(grid);
          dynamic_received_ = tdt::Clock::now();
          dynamic_stamp_ = stamp;
          dynamic_error_.clear();
        } catch (const std::exception &error) {
          dynamic_.reset();
          dynamic_error_ = std::string("dynamic map rejected: ") + error.what();
        }
      });
    goal_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        ++generation_;
        pending_ = *message;
        clearOutput();
        status("goal received: previous result cleared");
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {poll();});
    parameter_guard_ = add_on_set_parameters_callback([](const auto &) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "restart debug node to change configuration";
      return result;
    });
    RCLCPP_INFO(get_logger(), "debug only; clearance=%.9f m, static_only=%s",
      config_.requiredClearance(), static_only_ ? "true" : "false");
    status("ready: execution disabled");
  }
private:
  tdt::Grid convert(const nav_msgs::msg::OccupancyGrid &message) const {
    const auto &orientation = message.info.origin.orientation;
    if (message.header.frame_id != frame_ || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) || std::abs(orientation.x) > 1e-9 ||
      std::abs(orientation.y) > 1e-9 || std::abs(orientation.z) > 1e-9 ||
      !std::isfinite(orientation.w) || std::abs(std::abs(orientation.w) - 1.0) > 1e-9 ||
      !std::isfinite(message.info.origin.position.z))
      throw std::invalid_argument("expected axis-aligned grid in global_frame");
    tdt::Grid grid;
    grid.width = message.info.width;
    grid.height = message.info.height;
    grid.resolution = message.info.resolution;
    grid.origin = {message.info.origin.position.x, message.info.origin.position.y};
    grid.cells = message.data;
    grid.validate();
    return grid;
  }
  struct InputState {
    std::string reason;
    std::string detail;
  };
  InputState inputState() const {
    if (!base_) return {base_error_, ""};
    if (static_only_) return {};
    if (!dynamic_) return {dynamic_error_, ""};
    const double received_age = std::chrono::duration<double>(tdt::Clock::now() - dynamic_received_).count();
    if (received_age > max_age_)
      return {"dynamic receive age exceeded", "; receive_age_s=" + std::to_string(received_age) +
        "; max_age_s=" + std::to_string(max_age_) + "; clock=steady"};
    if (dynamic_stamp_.nanoseconds() == 0) return {"dynamic timestamp is zero", "; clock=ROS"};
    const double age = (now() - dynamic_stamp_).seconds();
    const std::string detail = "; stamp_age_s=" + std::to_string(age) +
      "; allowed_age_s=[-0.05," + std::to_string(max_age_) + "]; clock=ROS";
    if (age < -0.05) return {"dynamic timestamp is in future", detail};
    if (age > max_age_) return {"dynamic timestamp age exceeded", detail};
    return {};
  }
  void reportInputState() {
    const auto state = inputState();
    if (state.reason == last_input_reason_) return;
    last_input_reason_ = state.reason;
    if (state.reason.empty()) {
      RCLCPP_INFO(get_logger(), "map inputs ready; no automatic replanning");
    } else {
      RCLCPP_WARN(get_logger(), "map input unavailable: %s%s; any displayed result is historical",
        state.reason.c_str(), state.detail.c_str());
    }
  }
  void status(const std::string &text) {
    std_msgs::msg::String message;
    message.data = text;
    status_->publish(message);
  }
  void publish(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher, const tdt::Path &path) {
    nav_msgs::msg::Path message;
    message.header.frame_id = frame_;
    message.header.stamp = now();
    for (const auto &point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.orientation.w = 1.0;
      message.poses.push_back(pose);
    }
    publisher->publish(message);
  }
  void publishCorridors(const std::vector<Eigen::Vector4d> &boxes) {
    visualization_msgs::msg::MarkerArray message;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    message.markers.push_back(clear);
    for (size_t index = 0; index < boxes.size(); ++index) {
      const auto &box = boxes[index];
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_;
      marker.header.stamp = now();
      marker.ns = "waypoint_corridors";
      marker.id = index;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1;
      marker.pose.position.x = (box[0] + box[2]) * 0.5;
      marker.pose.position.y = (box[1] + box[3]) * 0.5;
      marker.scale.x = std::max(0.001, box[2] - box[0]);
      marker.scale.y = std::max(0.001, box[3] - box[1]);
      marker.scale.z = 0.02;
      marker.color.g = 1; marker.color.a = 0.3;
      message.markers.push_back(marker);
    }
    corridors_->publish(message);
  }
  void clearOutput() {publish(raw_, {}); publish(simplified_, {}); publish(optimized_, {}); publishCorridors({});}
  void poll() {
    reportInputState();
    if (work_.valid()) {
      if (work_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
      try {
        const auto result = work_.get();
        if (running_generation_ == generation_) {
          const auto inputs = inputState();
          if (!inputs.reason.empty())
            throw std::runtime_error("result rejected: " + inputs.reason + inputs.detail + "; new goal required");
          if (!result.success)
            throw std::runtime_error("planning failed: " + result.reason + "; new goal required");
          const auto validation_begin = tdt::Clock::now();
          const auto transform = buffer_.lookupTransform(frame_, robot_frame_, tf2::TimePointZero);
          const rclcpp::Time stamp(transform.header.stamp);
          const double age = (now() - stamp).seconds();
          if (stamp.nanoseconds() && (age < -0.05 || age > max_age_)) throw std::runtime_error("stale robot TF");
          const tdt::Point current(transform.transform.translation.x, transform.transform.translation.y);
          if (!current.allFinite() || (current - running_start_).norm() > max_start_drift_)
            throw std::runtime_error("start moved; explicit new goal required");
          const tdt::Environment latest(*base_, static_only_ ? nullptr : &*dynamic_, config_);
          if (!latest.pathSafe(result.path)) throw std::runtime_error("latest map rejects candidate path");
          if (std::chrono::duration<double, std::milli>(tdt::Clock::now() - validation_begin).count() +
            result.total_ms > config_.total_budget_ms) throw std::runtime_error("final validation budget exceeded");
          const auto final_inputs = inputState();
          if (!final_inputs.reason.empty())
            throw std::runtime_error("result rejected: " + final_inputs.reason + final_inputs.detail);
          publish(raw_, result.raw);
          publish(simplified_, result.simplified);
          publish(optimized_, result.path);
          publishCorridors(result.corridors);
          status(result.reason + "; qp_solves=" + std::to_string(result.qp_solves) +
            "; total_ms=" + std::to_string(result.total_ms));
        }
      } catch (const std::exception &error) {
        if (running_generation_ == generation_) status(error.what());
      }
    }
    if (!pending_) return;
    const auto goal = *pending_;
    pending_.reset();
    try {
      const auto inputs = inputState();
      if (!inputs.reason.empty())
        throw std::runtime_error("goal rejected: " + inputs.reason + inputs.detail + "; new goal required");
      if (goal.header.frame_id != frame_) throw std::invalid_argument("goal must use global_frame");
      const auto transform = buffer_.lookupTransform(frame_, robot_frame_, tf2::TimePointZero);
      const rclcpp::Time stamp(transform.header.stamp);
      const double age = (now() - stamp).seconds();
      if (stamp.nanoseconds() && (age < -0.05 || age > max_age_)) throw std::runtime_error("stale robot TF");
      tdt::Snapshot snapshot;
      snapshot.config = config_;
      snapshot.base = *base_;
      snapshot.has_dynamic = !static_only_;
      if (snapshot.has_dynamic) snapshot.dynamic = *dynamic_;
      snapshot.start = {transform.transform.translation.x, transform.transform.translation.y};
      snapshot.goal = {goal.pose.position.x, goal.pose.position.y};
      if (!snapshot.start.allFinite() || !snapshot.goal.allFinite()) throw std::invalid_argument("nonfinite endpoint");
      if (!snapshot_file_.empty()) tdt::saveSnapshot(snapshot, snapshot_file_);
      running_generation_ = generation_;
      running_start_ = snapshot.start;
      work_ = std::async(std::launch::async, [snapshot] {
        const tdt::Environment environment(snapshot.base, snapshot.has_dynamic ? &snapshot.dynamic : nullptr, snapshot.config);
        return tdt::plan(environment, snapshot.start, snapshot.goal);
      });
      status("planning snapshot; execution disabled");
    } catch (const std::exception &error) {status(error.what());}
  }
  tdt::Config config_;
  bool static_only_;
  std::string snapshot_file_, frame_, robot_frame_;
  std::string base_error_ = "static map not received";
  std::string dynamic_error_ = "dynamic full map not received";
  std::string last_input_reason_;
  double max_age_, max_start_drift_;
  tdt::Point running_start_ = tdt::Point::Zero();
  std::optional<tdt::Grid> base_, dynamic_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_;
  tdt::Clock::time_point dynamic_received_;
  rclcpp::Time dynamic_stamp_{0, 0, RCL_ROS_TIME};
  uint64_t generation_ = 0, running_generation_ = 0;
  std::future<tdt::Result> work_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_, dynamic_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_, simplified_, optimized_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridors_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<SnapshotPlanner>());} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
