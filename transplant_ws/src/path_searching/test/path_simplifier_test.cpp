#include "path_searching/path_simplifier.h"

#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>

void require(bool condition, const std::string &message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

using Simplifier = path_searching::PathSimplifier;
using Path = Simplifier::Path;

bool intersectsCell(
  const Eigen::Vector2d &start, const Eigen::Vector2d &end, int row, int column)
{
  double lower = 0.0;
  double upper = 1.0;
  for (int axis = 0; axis < 2; ++axis)
  {
    const double minimum = axis == 0 ? column : row;
    const double delta = end[axis] - start[axis];
    if (delta == 0.0)
    {
      if (start[axis] < minimum || start[axis] > minimum + 1.0) return false;
      continue;
    }
    double entry = (minimum - start[axis]) / delta;
    double exit = (minimum + 1.0 - start[axis]) / delta;
    if (entry > exit) std::swap(entry, exit);
    lower = std::max(lower, entry);
    upper = std::min(upper, exit);
    if (lower > upper + 1e-12) return false;
  }
  return true;
}

void checkContract(const Path &input, const Simplifier::Result &result)
{
  require(result.valid(), "expected valid simplification result");
  require(!result.path.empty() && result.path.size() <= input.size(), "invalid output size");
  require(result.path.front() == input.front() && result.path.back() == input.back(), "endpoint changed");
  size_t cursor = 0;
  for (const auto &point : result.path)
  {
    require(point.allFinite(), "nonfinite output");
    while (cursor < input.size() && input[cursor] != point) ++cursor;
    require(cursor < input.size(), "output must be an ordered input subsequence");
    ++cursor;
  }
}

int main(int argc, char **argv)
{
  try
  {
  const path_searching::GridGeometry geometry({20, 20}, {-1.0, -1.0}, 0.05);
  const path_searching::PathSimplifier::Path input{
    {-0.875, -0.875}, {-0.775, -0.875}, {-0.675, -0.875}, {-0.575, -0.875}};
  const auto result = path_searching::PathSimplifier().simplify(
    input, geometry, [](const Eigen::Vector2i &) { return true; });
  require(result.valid(), "straight path must be valid");
  require(result.path.size() == 2, "straight path must reduce to endpoints");
  require(result.path.front() == input.front(), "start must be preserved");
  require(result.path.back() == input.back(), "end must be preserved");
  std::cout << "PASS straight path\n";
  const path_searching::PathSimplifier::Path detour{
    {-0.775, -0.775}, {-0.575, -0.775}, {-0.575, -0.575}};
  const auto blocked = path_searching::PathSimplifier().simplify(
    detour, geometry, [](const Eigen::Vector2i &cell) {
      return cell != Eigen::Vector2i(6, 6);
    });
  require(blocked.path.size() == 3, "shortcut through obstacle must retain detour");
  std::cout << "PASS blocked shortcut\n";
  const Simplifier simplifier;
  const auto free_cell = [](const Eigen::Vector2i &) { return true; };
  const auto center = [&](int row, int column) {
    return (geometry.indexToWorld({row, column}) + Eigen::Vector2d::Constant(0.025)).eval();
  };
  const Path direct{center(4, 4), center(8, 8)};
  require(!simplifier.segmentPassable(direct.front(), direct.back(), geometry,
    [](const Eigen::Vector2i &cell) { return cell != Eigen::Vector2i(5, 4); }),
    "diagonal corner must include touching side cell");
  const Eigen::Vector2d boundary_start(-0.75, -0.875);
  const Eigen::Vector2d boundary_end(-0.75, -0.575);
  require(!simplifier.segmentPassable(boundary_start, boundary_end, geometry,
    [](const Eigen::Vector2i &cell) { return cell != Eigen::Vector2i(6, 4); }),
    "grid-line segment must check both sides");
  require(!simplifier.segmentPassable(center(5, 5), center(5, 5) + Eigen::Vector2d(0.001, 0),
    geometry, [](const Eigen::Vector2i &) { return false; }), "short blocked line must fail");
  require(!simplifier.segmentPassable(center(4, 4), center(8, 8), geometry,
    [](const Eigen::Vector2i &cell) { return cell != Eigen::Vector2i(4, 4); }),
    "occupied endpoint must fail");
  require(!simplifier.segmentPassable({-1.0, -0.8}, {-1.0, -0.6}, geometry, free_cell),
    "touching outside map must fail");
  std::cout << "PASS corners, grid lines, endpoints and short segments\n";

  for (double resolution : {0.025, 0.1})
  {
    const path_searching::GridGeometry alternate({20, 20}, {-2.0, -3.0}, resolution);
    const auto alternate_center = [&](int column) {
      return (alternate.indexToWorld({10, column}) +
        Eigen::Vector2d::Constant(resolution / 2.0)).eval();
    };
    const Path corridor{alternate_center(3), alternate_center(8), alternate_center(14)};
    const auto narrow = simplifier.simplify(corridor, alternate,
      [](const Eigen::Vector2i &cell) { return cell.x() == 10; });
    checkContract(corridor, narrow);
    require(narrow.path.size() == 2, "free narrow corridor rejected at alternate resolution");
    require(!simplifier.segmentPassable(corridor.front(), corridor.back(), alternate,
      [](const Eigen::Vector2i &cell) { return cell.x() == 10 && cell.y() != 9; }),
      "blocked narrow corridor accepted");
  }
  std::cout << "PASS narrow corridor at 0.025m/0.1m (simplifier module only)\n";

  for (double tolerance : {-1.0, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    bool rejected = false;
    try { Simplifier invalid(tolerance); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "invalid tolerance accepted");
  }
  require(!simplifier.simplify({}, geometry, free_cell).valid(), "empty input accepted");
  for (double coordinate : {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max(), -1.1, 0.0})
  {
    require(!simplifier.simplify({{coordinate, -0.8}}, geometry, free_cell).valid(),
      "invalid path coordinate accepted");
  }
  require(!simplifier.simplify(input, {}, free_cell).valid(), "missing geometry accepted");
  require(!simplifier.simplify(input, geometry, {}).valid(), "missing query accepted");
  require(!simplifier.simplify(input,
    {{20, 20}, {-1, -1}, std::numeric_limits<double>::infinity()}, free_cell).valid(),
    "nonfinite resolution accepted");
  require(!simplifier.simplify(input,
    {{20, 20}, {std::numeric_limits<double>::quiet_NaN(), -1}, 0.05}, free_cell).valid(),
    "nonfinite origin accepted");
  std::cout << "PASS invalid parameters, inputs and environments\n";

  checkContract({input.front()}, simplifier.simplify({input.front()}, geometry, free_cell));
  checkContract(direct, simplifier.simplify(direct, geometry, free_cell));
  const Path duplicates{input.front(), input.front(), input.back(), input.back()};
  checkContract(duplicates, simplifier.simplify(duplicates, geometry, free_cell));
  require(simplifier.simplify({input.front(), input.front()}, geometry, free_cell).path.size() == 1,
    "all duplicates must reduce to one point");
  checkContract(input, Simplifier(0.0).simplify(input, geometry, free_cell));
  std::cout << "PASS short paths, duplicates and zero tolerance\n";

  bool dynamic_obstacle = true;
  const auto dynamic_query = [&](const Eigen::Vector2i &cell) {
    return !dynamic_obstacle || cell != Eigen::Vector2i(6, 6);
  };
  require(simplifier.simplify(detour, geometry, dynamic_query).path.size() == 3,
    "dynamic obstacle ignored");
  dynamic_obstacle = false;
  require(simplifier.simplify(detour, geometry, dynamic_query).path.size() == 2,
    "cleared obstacle remains cached");
  dynamic_obstacle = true;
  require(simplifier.simplify(detour, geometry, dynamic_query).path.size() == 3,
    "reappearing obstacle ignored");
  const auto escape = simplifier.simplify(input, geometry,
    [](const Eigen::Vector2i &) { return false; });
  require(escape.valid() && escape.path.size() == input.size(),
    "unsafe original edges must be retained without new shortcuts");
  std::cout << "PASS dynamic appearance/clearance and escape preservation\n";

  std::mt19937 generator(20261024);
  std::uniform_int_distribution<int> coordinate(2, 17);
  for (int trial = 0; trial < 160; ++trial)
  {
    Eigen::Vector2d first(coordinate(generator) + 0.5, coordinate(generator) + 0.5);
    Eigen::Vector2d last(coordinate(generator) + 0.5, coordinate(generator) + 0.5);
    if (trial % 4 == 0) first.x() = std::floor(first.x());
    if (trial % 5 == 0) last.y() = std::floor(last.y());
    std::set<std::pair<int, int>> touched;
    const auto collect = [&](const Eigen::Vector2i &cell) {
      touched.emplace(cell.x(), cell.y());
      return true;
    };
    require(simplifier.segmentPassable((first * 0.05 + geometry.offset()).eval(),
      (last * 0.05 + geometry.offset()).eval(), geometry, collect), "free segment rejected");
    for (int row = 0; row < 20; ++row)
    {
      for (int column = 0; column < 20; ++column)
      {
        if (intersectsCell(first, last, row, column))
          require(touched.count({row, column}) > 0, "supercover missed an oracle cell");
      }
    }
    std::set<std::pair<int, int>> reversed;
    require(simplifier.segmentPassable((last * 0.05 + geometry.offset()).eval(),
      (first * 0.05 + geometry.offset()).eval(), geometry,
      [&](const Eigen::Vector2i &cell) { reversed.emplace(cell.x(), cell.y()); return true; }),
      "reversed free segment rejected");
    require(touched == reversed, "segment coverage must be direction independent");
  }
  std::cout << "PASS 160 seeded coverage cases against closed-cell intersection oracle\n";

  Path long_path;
  for (int index = 0; index < 5000; ++index)
    long_path.emplace_back(-0.875 + 0.5 * index / 4999.0, -0.875);
  const auto begin = std::chrono::steady_clock::now();
  const auto long_result = simplifier.simplify(long_path, geometry, free_cell);
  checkContract(long_path, long_result);
  require(long_result.path.size() == 2, "long straight path not simplified");
  std::cout << "PASS long path: 5000 -> 2, elapsed_ms=" <<
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count() << '\n';

  if (argc == 3 && std::string(argv[1]) == "--export")
  {
    std::ofstream output(argv[2]);
    if (!output) throw std::runtime_error("cannot write observation CSV");
    output << "case,kind,x,y\n";
    const auto export_path = [&](const std::string &name, const std::string &kind, const Path &points) {
      for (const auto &point : points)
        output << name << ',' << kind << ',' << point.x() << ',' << point.y() << '\n';
    };
    export_path("free", "input", input);
    export_path("free", "output", result.path);
    export_path("blocked", "input", detour);
    export_path("blocked", "output", blocked.path);
    export_path("blocked", "obstacle", {center(6, 6)});
    export_path("cleared", "input", detour);
    export_path("cleared", "output", simplifier.simplify(detour, geometry, free_cell).path);
    std::cout << "EXPORTED " << argv[2] << '\n';
  }
  return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
