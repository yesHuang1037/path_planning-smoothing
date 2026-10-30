#include "tdt_path_planning/core.hpp"
#include "dynamicvoronoi.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tdt {
namespace {
void positive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name);
}
void nonnegative(double value, const char *name) {
  if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument(name);
}
}

void Config::validate() const {
  positive(body_length, "body_length");
  positive(body_width, "body_width");
  nonnegative(clearance_margin, "clearance_margin");
  nonnegative(obstacle_weight, "obstacle_weight");
  nonnegative(simplify_epsilon, "simplify_epsilon");
  nonnegative(corridor_range, "corridor_range");
  positive(allocation_speed, "allocation_speed");
  positive(allocation_acceleration, "allocation_acceleration");
  positive(min_segment_time, "min_segment_time");
  positive(max_chord, "max_chord");
  positive(curve_error, "curve_error");
  positive(output_spacing, "output_spacing");
  positive(search_budget_ms, "search_budget_ms");
  positive(qp_budget_ms, "qp_budget_ms");
  positive(total_budget_ms, "total_budget_ms");
  for (double value : {body_length, body_width, clearance_margin, obstacle_weight,
    simplify_epsilon, corridor_range, allocation_speed, allocation_acceleration,
    min_segment_time, max_chord, curve_error, output_spacing, search_budget_ms,
    qp_budget_ms, total_budget_ms})
    if (value > 60000.0 || (value > 0.0 && value < 1e-6))
      throw std::invalid_argument("configuration exceeds numerical bounds [1e-6,60000]");
  if (occupied_threshold < 1 || occupied_threshold > 100 || max_refine_rounds < 1 ||
    max_refine_rounds > 64 || max_control_points < 2 || max_control_points > 256 ||
    max_check_samples < 2 || max_check_samples > 1000000 ||
    max_search_expansions < 1 || qp_max_iterations < 1 ||
    !std::isfinite(requiredClearance())) throw std::invalid_argument("invalid configuration bounds");
}

double Config::requiredClearance() const {
  return std::hypot(body_length * 0.5, body_width * 0.5) + clearance_margin;
}

void Grid::validate() const {
  positive(resolution, "grid resolution");
  if (width < 1 || height < 1 || width > 4096 || height > 4096 ||
    static_cast<size_t>(width) * height > 4000000 || !origin.allFinite() ||
    cells.size() != static_cast<size_t>(width) * height ||
    !(origin + Point(width * resolution, height * resolution)).allFinite())
    throw std::invalid_argument("grid geometry or data length");
  for (auto value : cells) if (value < -1 || value > 100)
    throw std::invalid_argument("expected OccupancyGrid -1..100 encoding");
}

bool Grid::contains(const Point &point) const {
  const Point local = (point - origin) / resolution;
  return point.allFinite() && local.allFinite() && local.x() >= 0 && local.y() >= 0 &&
    local.x() < width && local.y() < height;
}
Eigen::Vector2i Grid::index(const Point &point) const {
  if (!contains(point)) throw std::out_of_range("point outside grid");
  const Point local = (point - origin) / resolution;
  return {static_cast<int>(std::floor(local.x())), static_cast<int>(std::floor(local.y()))};
}
Point Grid::center(int col, int row) const {
  return origin + resolution * Point(col + 0.5, row + 0.5);
}

void DynamicGrid::replace(const Grid &grid) {
  synchronized_ = false;
  grid.validate();
  grid_ = grid;
  synchronized_ = true;
}
void DynamicGrid::patch(int col, int row, int width, int height,
  const std::vector<int8_t> &cells) {
  if (!synchronized_ || col < 0 || row < 0 || width < 1 || height < 1 ||
    width > grid_.width || height > grid_.height || col > grid_.width - width ||
    row > grid_.height - height || cells.size() != static_cast<size_t>(width) * height ||
    std::any_of(cells.begin(), cells.end(), [](int8_t value) {return value < -1 || value > 100;})) {
    synchronized_ = false;
    throw std::invalid_argument("invalid or unsynchronized costmap update");
  }
  for (int offset = 0; offset < height; ++offset)
    std::copy_n(cells.begin() + offset * width, width,
      grid_.cells.begin() + (row + offset) * grid_.width + col);
}
void DynamicGrid::invalidate() {synchronized_ = false;}
bool DynamicGrid::synchronized() const {return synchronized_;}
const Grid &DynamicGrid::grid() const {
  if (!synchronized_) throw std::logic_error("costmap requires a full snapshot");
  return grid_;
}

struct Environment::Impl {
  Grid grid;
  Config config;
  std::vector<std::vector<int>> occupied_columns;
  std::unique_ptr<navi_planner::DynamicVoronoi> distance;
};

Environment::Environment(const Grid &base, const Grid *dynamic, const Config &config)
: impl_(std::make_unique<Impl>()) {
  config.validate();
  base.validate();
  impl_->grid = base;
  impl_->config = config;
  auto &grid = impl_->grid;
  for (auto &value : grid.cells) value = value == 0 ? 0 : 100;
  if (dynamic) {
    dynamic->validate();
    for (int row = 0; row < base.height; ++row) {
      for (int col = 0; col < base.width; ++col) {
        const Point low = base.origin + base.resolution * Point(col, row);
        const Point high = low + Point::Constant(base.resolution);
        const Point first = (low - dynamic->origin) / dynamic->resolution;
        const Point last = (high - dynamic->origin) / dynamic->resolution;
        if (last.x() <= 0 || last.y() <= 0 || first.x() >= dynamic->width || first.y() >= dynamic->height)
          continue;
        const int first_col = static_cast<int>(std::floor(std::clamp(first.x(), 0.0, double(dynamic->width))));
        const int first_row = static_cast<int>(std::floor(std::clamp(first.y(), 0.0, double(dynamic->height))));
        const int last_col = std::min(dynamic->width - 1,
          static_cast<int>(std::ceil(std::clamp(last.x(), 0.0, double(dynamic->width)))) - 1);
        const int last_row = std::min(dynamic->height - 1,
          static_cast<int>(std::ceil(std::clamp(last.y(), 0.0, double(dynamic->height)))) - 1);
        for (int source_row = first_row; source_row <= last_row; ++source_row)
          for (int source_col = first_col; source_col <= last_col; ++source_col) {
            const int value = dynamic->cells[source_row * dynamic->width + source_col];
            if (value < 0 || value >= config.occupied_threshold) grid.cells[row * grid.width + col] = 100;
          }
      }
    }
  }
  impl_->occupied_columns.resize(grid.height);
  impl_->distance = std::make_unique<navi_planner::DynamicVoronoi>();
  impl_->distance->initializeEmpty(grid.width, grid.height);
  for (int row = 0; row < grid.height; ++row)
    for (int col = 0; col < grid.width; ++col)
      if (occupied(col, row)) {
        impl_->occupied_columns[row].push_back(col);
        impl_->distance->occupyCell(col, row);
      }
  impl_->distance->update();
}
Environment::~Environment() = default;
const Grid &Environment::geometry() const {return impl_->grid;}
const Config &Environment::config() const {return impl_->config;}
bool Environment::occupied(int col, int row) const {
  const auto &grid = impl_->grid;
  return col < 0 || row < 0 || col >= grid.width || row >= grid.height ||
    grid.cells[row * grid.width + col] != 0;
}

double Environment::clearance(const Point &point) const {
  const auto &grid = impl_->grid;
  if (!grid.contains(point)) return 0.0;
  const auto cell = grid.index(point);
  if (occupied(cell.x(), cell.y())) return 0.0;
  const Point local = point - grid.origin;
  double lower = std::min({local.x(), local.y(), grid.width * grid.resolution - local.x(),
    grid.height * grid.resolution - local.y(), config().requiredClearance() + 0.25});
  const double radius = lower;
  const int first_row = int(std::floor(std::clamp((local.y() - radius) / grid.resolution, 0.0, double(grid.height - 1))));
  const int last_row = int(std::floor(std::clamp((local.y() + radius) / grid.resolution, 0.0, double(grid.height - 1))));
  const int first_col = int(std::floor(std::clamp((local.x() - radius) / grid.resolution, 0.0, double(grid.width - 1))));
  const int last_col = int(std::floor(std::clamp((local.x() + radius) / grid.resolution, 0.0, double(grid.width - 1))));
  for (int row = first_row; row <= last_row; ++row) {
    const auto &columns = impl_->occupied_columns[row];
    for (auto iterator = std::lower_bound(columns.begin(), columns.end(), first_col);
      iterator != columns.end() && *iterator <= last_col; ++iterator) {
      const Point low = grid.resolution * Point(*iterator, row);
      const Point delta = (low - local).cwiseMax(local - low - Point::Constant(grid.resolution)).cwiseMax(0.0);
      lower = std::min(lower, delta.norm());
    }
  }
  const double center_distance = impl_->distance->getDistance(cell.x(), cell.y()) * grid.resolution;
  if (std::isfinite(center_distance)) lower = std::min(lower, std::max(0.0,
    center_distance - (point - grid.center(cell.x(), cell.y())).norm() - grid.resolution / std::sqrt(2.0)));
  return lower;
}

bool Environment::safe(const Point &point, double extra) const {
  return std::isfinite(extra) && extra >= 0.0 && clearance(point) > config().requiredClearance() + extra;
}
bool Environment::segmentSafe(const Point &start, const Point &end, double extra, Clock::time_point deadline) const {
  if (!start.allFinite() || !end.allFinite() || !std::isfinite(extra) || extra < 0) return false;
  const double length = (end - start).norm();
  const double step = std::min(geometry().resolution * 0.5, config().max_chord);
  const double count = std::ceil(length / step);
  if (!std::isfinite(count) || count + 1 > config().max_check_samples) return false;
  const int intervals = std::max(1, static_cast<int>(count));
  const double allowance = length / intervals * 0.5;
  for (int sample = 0; sample <= intervals; ++sample)
    if (Clock::now() >= deadline ||
      !safe(start + (end - start) * (double(sample) / intervals), extra + allowance)) return false;
  return true;
}
bool Environment::pathSafe(const Path &path, Clock::time_point deadline) const {
  if (path.size() < 2 || path.size() > static_cast<size_t>(config().max_check_samples)) return false;
  for (size_t index = 1; index < path.size(); ++index)
    if (!segmentSafe(path[index - 1], path[index], 0.0, deadline)) return false;
  return true;
}

}
