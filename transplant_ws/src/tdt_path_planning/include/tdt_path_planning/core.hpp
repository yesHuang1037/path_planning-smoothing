#pragma once

#include <Eigen/Core>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tdt {

using Point = Eigen::Vector2d;
using Path = std::vector<Point>;
using Clock = std::chrono::steady_clock;

struct Config {
  double body_length = 0.60;
  double body_width = 0.60;
  double clearance_margin = 0.08;
  int occupied_threshold = 65;
  double obstacle_weight = 0.10;
  double simplify_epsilon = 0.10;
  double corridor_range = 0.30;
  double allocation_speed = 0.20;
  double allocation_acceleration = 0.20;
  double min_segment_time = 0.05;
  double max_chord = 0.05;
  double curve_error = 0.001;
  double output_spacing = 0.10;
  double search_budget_ms = 1000.0;
  double qp_budget_ms = 300.0;
  double total_budget_ms = 3000.0;
  int max_refine_rounds = 6;
  int max_control_points = 128;
  int max_check_samples = 200000;
  int max_search_expansions = 500000;
  int qp_max_iterations = 20000;
  void validate() const;
  double requiredClearance() const;
};

struct Grid {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  Point origin = Point::Zero();
  std::vector<int8_t> cells;
  void validate() const;
  bool contains(const Point &point) const;
  Eigen::Vector2i index(const Point &point) const;
  Point center(int col, int row) const;
};

class DynamicGrid {
public:
  void replace(const Grid &grid);
  void patch(int col, int row, int width, int height,
    const std::vector<int8_t> &cells);
  void invalidate();
  bool synchronized() const;
  const Grid &grid() const;
private:
  Grid grid_;
  bool synchronized_ = false;
};

class Environment {
public:
  Environment(const Grid &base, const Grid *dynamic, const Config &config);
  ~Environment();
  Environment(const Environment &) = delete;
  Environment &operator=(const Environment &) = delete;
  double clearance(const Point &point) const;
  bool safe(const Point &point, double extra = 0.0) const;
  bool segmentSafe(const Point &start, const Point &end, double extra = 0.0,
    Clock::time_point deadline = Clock::time_point::max()) const;
  bool pathSafe(const Path &path, Clock::time_point deadline = Clock::time_point::max()) const;
  bool occupied(int col, int row) const;
  const Grid &geometry() const;
  const Config &config() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct Result {
  bool success = false;
  std::string reason;
  Path raw;
  Path simplified;
  Path path;
  std::vector<Eigen::Matrix<double, 6, 2>> coefficients;
  std::vector<double> segment_times;
  std::vector<Eigen::Vector4d> corridors;
  int rounds = 0;
  int qp_solves = 0;
  double search_ms = 0.0;
  double optimize_ms = 0.0;
  double total_ms = 0.0;
};

Path search(const Environment &environment, const Point &start, const Point &goal,
  Clock::time_point deadline);
Path simplify(const Environment &environment, const Path &path,
  Clock::time_point deadline = Clock::time_point::max());
Result plan(const Environment &environment, const Point &start, const Point &goal);

struct Snapshot {
  Config config;
  Grid base;
  Grid dynamic;
  bool has_dynamic = false;
  Point start = Point::Zero();
  Point goal = Point::Zero();
};
void saveSnapshot(const Snapshot &snapshot, const std::string &filename);
Snapshot loadSnapshot(const std::string &filename);

}
