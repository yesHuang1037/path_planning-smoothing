#include "tdt_path_planning/core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace tdt {

Path search(const Environment &environment, const Point &start, const Point &goal,
  Clock::time_point deadline) {
  const auto &grid = environment.geometry();
  if (!environment.safe(start) || !environment.safe(goal)) throw std::runtime_error("invalid start or goal clearance");
  if ((start - goal).norm() < 1e-9) throw std::runtime_error("goal already reached");
  const auto start_cell = grid.index(start);
  const auto goal_cell = grid.index(goal);
  const int start_index = start_cell.y() * grid.width + start_cell.x();
  const int goal_index = goal_cell.y() * grid.width + goal_cell.x();
  const Point start_center = grid.center(start_cell.x(), start_cell.y());
  const Point goal_center = grid.center(goal_cell.x(), goal_cell.y());
  if (!environment.segmentSafe(start, start_center, 0.0, deadline) ||
    !environment.segmentSafe(goal_center, goal, 0.0, deadline))
    throw std::runtime_error("unsafe endpoint connection");
  struct Entry {double priority; double distance; int index;};
  struct Later {bool operator()(const Entry &first, const Entry &second) const {
    return first.priority > second.priority;
  }};
  std::priority_queue<Entry, std::vector<Entry>, Later> queue;
  std::vector<double> distance(grid.cells.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(grid.cells.size(), -1);
  distance[start_index] = 0;
  queue.push({(start_center - goal_center).norm(), 0, start_index});
  int expanded = 0;
  while (!queue.empty()) {
    if (Clock::now() >= deadline || ++expanded > environment.config().max_search_expansions)
      throw std::runtime_error("search budget exceeded");
    const auto current = queue.top();
    queue.pop();
    if (current.distance != distance[current.index]) continue;
    if (current.index == goal_index) {
      Path path{goal};
      for (int index = goal_index; index != -1; index = parent[index])
        path.push_back(grid.center(index % grid.width, index / grid.width));
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      path.erase(std::unique(path.begin(), path.end(), [](const Point &first, const Point &second) {
        return (first - second).norm() < 1e-10;
      }), path.end());
      path.front() = start;
      path.back() = goal;
      if (!environment.pathSafe(path, deadline)) throw std::runtime_error("unsafe or over-budget search result");
      return path;
    }
    const int col = current.index % grid.width;
    const int row = current.index / grid.width;
    const Point from = grid.center(col, row);
    for (int row_delta = -1; row_delta <= 1; ++row_delta)
      for (int col_delta = -1; col_delta <= 1; ++col_delta) {
        if (col_delta == 0 && row_delta == 0) continue;
        const int next_col = col + col_delta;
        const int next_row = row + row_delta;
        if (environment.occupied(next_col, next_row)) continue;
        if (col_delta && row_delta && (environment.occupied(next_col, row) || environment.occupied(col, next_row))) continue;
        const Point to = grid.center(next_col, next_row);
        if (!environment.segmentSafe(from, to, 0.0, deadline)) continue;
        const double clearance = environment.clearance(to);
        const double edge = (to - from).norm() *
          (1.0 + environment.config().obstacle_weight / (1.0 + clearance / grid.resolution));
        const double candidate = current.distance + edge;
        const int next_index = next_row * grid.width + next_col;
        if (candidate < distance[next_index]) {
          distance[next_index] = candidate;
          parent[next_index] = current.index;
          queue.push({candidate + (to - goal_center).norm(), candidate, next_index});
        }
      }
  }
  throw std::runtime_error("no path");
}

Path simplify(const Environment &environment, const Path &path, Clock::time_point deadline) {
  if (Clock::now() >= deadline) throw std::runtime_error("simplification deadline exceeded");
  if (!environment.pathSafe(path, deadline)) throw std::invalid_argument("invalid or over-budget simplifier input");
  std::vector<bool> keep(path.size(), false);
  keep.front() = keep.back() = true;
  std::vector<std::pair<size_t, size_t>> stack{{0, path.size() - 1}};
  while (!stack.empty()) {
    if (Clock::now() >= deadline) throw std::runtime_error("simplification deadline exceeded");
    const auto range = stack.back();
    stack.pop_back();
    if (range.second <= range.first + 1) continue;
    const Point delta = path[range.second] - path[range.first];
    double maximum = -1;
    size_t split = range.first + 1;
    for (size_t index = range.first + 1; index < range.second; ++index) {
      if (Clock::now() >= deadline) throw std::runtime_error("simplification deadline exceeded");
      const double ratio = delta.squaredNorm() > 0 ?
        std::clamp((path[index] - path[range.first]).dot(delta) / delta.squaredNorm(), 0.0, 1.0) : 0;
      const double error = (path[index] - path[range.first] - ratio * delta).norm();
      if (error > maximum) {maximum = error; split = index;}
    }
    if (maximum > environment.config().simplify_epsilon ||
      !environment.segmentSafe(path[range.first], path[range.second], 0.0, deadline)) {
      keep[split] = true;
      stack.emplace_back(range.first, split);
      stack.emplace_back(split, range.second);
    }
  }
  Path reduced;
  for (size_t index = 0; index < path.size(); ++index) if (keep[index]) reduced.push_back(path[index]);
  Path result{reduced.front()};
  size_t anchor = 0;
  while (anchor + 1 < reduced.size()) {
    size_t next = reduced.size() - 1;
    while (next > anchor + 1 && !environment.segmentSafe(reduced[anchor], reduced[next], 0.0, deadline)) {
      if (Clock::now() >= deadline) throw std::runtime_error("simplification deadline exceeded");
      --next;
    }
    result.push_back(reduced[next]);
    anchor = next;
  }
  return result;
}

}
