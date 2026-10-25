#include "path_searching/path_simplifier.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace path_searching
{
namespace
{

bool validGeometry(const GridGeometry &geometry)
{
  return geometry.valid() && std::isfinite(geometry.resolution()) &&
    geometry.offset().allFinite();
}

bool gridPoint(
  const Eigen::Vector2d &point, const GridGeometry &geometry, Eigen::Vector2d &grid)
{
  if (!point.allFinite())
  {
    return false;
  }
  grid = (point - geometry.offset()) / geometry.resolution();
  return grid.allFinite() && grid.x() >= 0.0 && grid.y() >= 0.0 &&
    grid.x() < geometry.size().y() && grid.y() < geometry.size().x();
}

bool onGridLine(double value)
{
  return std::abs(value - std::round(value)) <= 1e-10;
}

double segmentDistance(
  const Eigen::Vector2d &point, const Eigen::Vector2d &start, const Eigen::Vector2d &end)
{
  const auto delta = (end - start).eval();
  const double length = std::hypot(delta.x(), delta.y());
  if (length == 0.0)
  {
    return std::hypot(point.x() - start.x(), point.y() - start.y());
  }
  const auto direction = (delta / length).eval();
  const double distance = std::clamp((point - start).dot(direction), 0.0, length);
  const auto error = (point - start - distance * direction).eval();
  return std::hypot(error.x(), error.y());
}

}

PathSimplifier::PathSimplifier(double tolerance_m) : tolerance_m_(tolerance_m)
{
  if (!std::isfinite(tolerance_m) || tolerance_m < 0.0)
  {
    throw std::invalid_argument("simplification tolerance must be finite and nonnegative");
  }
}

bool PathSimplifier::segmentPassable(
  const Eigen::Vector2d &start, const Eigen::Vector2d &end,
  const GridGeometry &geometry, const CellQuery &query) const
{
  if (!validGeometry(geometry) || !query)
  {
    return false;
  }
  Eigen::Vector2d first;
  Eigen::Vector2d last;
  if (!gridPoint(start, geometry, first) || !gridPoint(end, geometry, last))
  {
    return false;
  }
  for (int axis = 0; axis < 2; ++axis)
  {
    if (onGridLine(first[axis])) first[axis] = std::round(first[axis]);
    if (onGridLine(last[axis])) last[axis] = std::round(last[axis]);
  }
  const auto visit = [&](int row, int column) {
    const Eigen::Vector2i cell(row, column);
    return geometry.inBounds(cell) && query(cell);
  };
  const auto endpoint = [&](const Eigen::Vector2d &point) {
    const int column = static_cast<int>(std::floor(point.x()));
    const int row = static_cast<int>(std::floor(point.y()));
    const int column_count = onGridLine(point.x()) ? 2 : 1;
    const int row_count = onGridLine(point.y()) ? 2 : 1;
    for (int row_offset = 0; row_offset < row_count; ++row_offset)
    {
      for (int column_offset = 0; column_offset < column_count; ++column_offset)
      {
        if (!visit(row - row_offset, column - column_offset)) return false;
      }
    }
    return true;
  };
  if (!endpoint(first) || !endpoint(last)) return false;

  int column = static_cast<int>(std::floor(first.x()));
  int row = static_cast<int>(std::floor(first.y()));
  const int end_column = static_cast<int>(std::floor(last.x()));
  const int end_row = static_cast<int>(std::floor(last.y()));
  const double delta_x = last.x() - first.x();
  const double delta_y = last.y() - first.y();
  const int step_column = (delta_x > 0.0) - (delta_x < 0.0);
  const int step_row = (delta_y > 0.0) - (delta_y < 0.0);
  const double infinity = std::numeric_limits<double>::infinity();
  const double interval_x = step_column == 0 ? infinity : 1.0 / std::abs(delta_x);
  const double interval_y = step_row == 0 ? infinity : 1.0 / std::abs(delta_y);
  double crossing_x = step_column == 0 ? infinity :
    ((step_column > 0 ? column + 1.0 : column) - first.x()) / delta_x;
  double crossing_y = step_row == 0 ? infinity :
    ((step_row > 0 ? row + 1.0 : row) - first.y()) / delta_y;
  const bool vertical_boundary = step_column == 0 && onGridLine(first.x());
  const bool horizontal_boundary = step_row == 0 && onGridLine(first.y());
  const auto visit_segment_cell = [&](int cell_row, int cell_column) {
    return visit(cell_row, cell_column) &&
      (!vertical_boundary || visit(cell_row, cell_column - 1)) &&
      (!horizontal_boundary || visit(cell_row - 1, cell_column)) &&
      (!vertical_boundary || !horizontal_boundary || visit(cell_row - 1, cell_column - 1));
  };
  const int64_t limit = static_cast<int64_t>(geometry.size().x()) + geometry.size().y() + 4;
  for (int64_t iteration = 0; iteration < limit; ++iteration)
  {
    if (!visit_segment_cell(row, column)) return false;
    if (column == end_column && row == end_row) return true;
    const bool corner = std::isfinite(crossing_x) && std::isfinite(crossing_y) &&
      std::abs(crossing_x - crossing_y) <= 1e-12;
    if (corner)
    {
      if (!visit(row, column + step_column) || !visit(row + step_row, column)) return false;
      column += step_column;
      row += step_row;
      crossing_x += interval_x;
      crossing_y += interval_y;
    }
    else if (crossing_x < crossing_y)
    {
      column += step_column;
      crossing_x += interval_x;
    }
    else
    {
      row += step_row;
      crossing_y += interval_y;
    }
  }
  return false;
}

PathSimplifier::Result PathSimplifier::simplify(
  const Path &path, const GridGeometry &geometry, const CellQuery &query) const
{
  if (!validGeometry(geometry) || !query)
  {
    return {Status::InvalidEnvironment, {}, "invalid grid geometry or missing cell query"};
  }
  if (path.empty()) return {Status::InvalidInput, {}, "empty input path"};
  Path clean;
  clean.reserve(path.size());
  for (const auto &point : path)
  {
    Eigen::Vector2d grid;
    if (!gridPoint(point, geometry, grid) ||
      !(point - path.front()).allFinite() ||
      !std::isfinite(std::hypot(point.x() - path.front().x(), point.y() - path.front().y())))
    {
      return {Status::InvalidInput, {}, "nonfinite, unrepresentable or out-of-map path point"};
    }
    if (clean.empty() || point != clean.back()) clean.push_back(point);
  }
  const auto finish = [&](Path output) {
    const auto status = output.size() < path.size() ? Status::Simplified : Status::Unchanged;
    return Result{status, std::move(output), "only new shortcuts are certified; retained edges are unchanged"};
  };
  if (clean.size() < 3) return finish(std::move(clean));

  std::vector<bool> keep(clean.size(), false);
  keep.front() = keep.back() = true;
  std::vector<std::pair<size_t, size_t>> pending{{0, clean.size() - 1}};
  while (!pending.empty())
  {
    const auto range = pending.back();
    pending.pop_back();
    if (range.second - range.first < 2) continue;
    double maximum_distance = -1.0;
    size_t split = range.first + 1;
    for (size_t index = range.first + 1; index < range.second; ++index)
    {
      const double distance = segmentDistance(clean[index], clean[range.first], clean[range.second]);
      if (!std::isfinite(distance)) return {Status::InvalidInput, {}, "path arithmetic overflow"};
      if (distance > maximum_distance)
      {
        maximum_distance = distance;
        split = index;
      }
    }
    if (maximum_distance <= tolerance_m_ &&
      segmentPassable(clean[range.first], clean[range.second], geometry, query)) continue;
    if (maximum_distance <= 1e-12) split = range.first + (range.second - range.first) / 2;
    keep[split] = true;
    pending.emplace_back(range.first, split);
    pending.emplace_back(split, range.second);
  }
  Path reduced;
  for (size_t index = 0; index < clean.size(); ++index)
  {
    if (keep[index]) reduced.push_back(clean[index]);
  }
  Path output{reduced.front()};
  size_t current = 0;
  while (current + 1 < reduced.size())
  {
    size_t next = current + 1;
    for (size_t candidate = current + 2; candidate < reduced.size(); ++candidate)
    {
      if (!segmentPassable(reduced[current], reduced[candidate], geometry, query)) break;
      next = candidate;
    }
    output.push_back(reduced[next]);
    current = next;
  }
  return finish(std::move(output));
}

}
