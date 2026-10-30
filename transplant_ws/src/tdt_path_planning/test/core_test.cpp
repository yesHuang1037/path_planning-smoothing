#include "tdt_path_planning/core.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <unistd.h>

namespace {
tdt::Grid grid(int width = 80, int height = 60, double resolution = 0.1) {
  tdt::Grid result;
  result.width = width;
  result.height = height;
  result.resolution = resolution;
  result.cells.assign(size_t(width) * height, 0);
  return result;
}
double oracle(const tdt::Grid &map, const tdt::Point &point) {
  const auto local = point - map.origin;
  double distance = std::min({local.x(), local.y(), map.width * map.resolution - local.x(),
    map.height * map.resolution - local.y()});
  for (int row = 0; row < map.height; ++row)
    for (int col = 0; col < map.width; ++col) if (map.cells[row * map.width + col] != 0) {
      const tdt::Point low = map.resolution * tdt::Point(col, row);
      const tdt::Point delta = (low - local).cwiseMax(local - low - tdt::Point::Constant(map.resolution)).cwiseMax(0.0);
      distance = std::min(distance, delta.norm());
    }
  return distance;
}
TEST(Configuration, RejectsInvalidAndComputesEnvelope) {
  tdt::Config config;
  EXPECT_NEAR(config.requiredClearance(), std::sqrt(0.18) + 0.08, 1e-12);
  config.validate();
  config.allocation_speed = 0;
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config = {};
  config.clearance_margin = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config = {};
  config.max_control_points = 257;
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config = {};
  config.body_length = 1e100;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}
TEST(Grid, BoundsEncodingAndNegativeOrigin) {
  auto map = grid(10, 10);
  map.origin = {-1, -2};
  map.validate();
  EXPECT_EQ(map.index({-0.99, -1.99}), Eigen::Vector2i(0, 0));
  EXPECT_FALSE(map.contains({-1.001, -2}));
  EXPECT_FALSE(map.contains({0, -1}));
  EXPECT_THROW(map.index({NAN, 0}), std::out_of_range);
  map.cells.pop_back();
  EXPECT_THROW(map.validate(), std::invalid_argument);
  map = grid(); map.cells[0] = -2;
  EXPECT_THROW(map.validate(), std::invalid_argument);
  map = grid(); map.resolution = INFINITY;
  EXPECT_THROW(map.validate(), std::invalid_argument);
}
TEST(DynamicGrid, FullPatchAndInvalidation) {
  tdt::DynamicGrid dynamic;
  EXPECT_THROW(dynamic.patch(0, 0, 1, 1, {100}), std::invalid_argument);
  dynamic.replace(grid(5, 5));
  dynamic.patch(1, 1, 2, 1, {100, -1});
  EXPECT_EQ(dynamic.grid().cells[6], 100);
  EXPECT_EQ(dynamic.grid().cells[7], -1);
  EXPECT_THROW(dynamic.patch(4, 0, 2, 1, {0, 0}), std::invalid_argument);
  EXPECT_FALSE(dynamic.synchronized());
  EXPECT_THROW(dynamic.grid(), std::logic_error);
  dynamic.replace(grid(3, 3));
  EXPECT_TRUE(dynamic.synchronized());
  EXPECT_EQ(dynamic.grid().width, 3);
}
TEST(Fusion, CoarseAreaUnknownWindowAndStaticPersistence) {
  auto base = grid(10, 10, 0.05);
  base.cells[0] = 1;
  auto local = grid(1, 1, 0.1);
  local.origin = {0.075, 0.075}; local.cells[0] = -1;
  const tdt::Environment environment(base, &local, {});
  for (int row = 1; row <= 3; ++row)
    for (int col = 1; col <= 3; ++col) EXPECT_TRUE(environment.occupied(col, row));
  EXPECT_TRUE(environment.occupied(0, 0));
  EXPECT_FALSE(environment.occupied(5, 5));
  local.cells[0] = 0;
  const tdt::Environment cleared(base, &local, {});
  EXPECT_FALSE(cleared.occupied(2, 2));
  EXPECT_TRUE(cleared.occupied(0, 0));
  local.origin = {0.3, 0.3}; local.cells[0] = 100;
  const tdt::Environment moved(base, &local, {});
  EXPECT_FALSE(moved.occupied(2, 2));
  EXPECT_TRUE(moved.occupied(6, 6));
}
TEST(Fusion, FineToCoarseThreshold) {
  auto base = grid(10, 10, 0.2);
  auto local = grid(4, 4, 0.05);
  local.origin = {0.21, 0.21}; local.cells[0] = 64;
  const tdt::Environment free(base, &local, {});
  EXPECT_FALSE(free.occupied(1, 1));
  local.cells[0] = 65;
  const tdt::Environment blocked(base, &local, {});
  EXPECT_TRUE(blocked.occupied(1, 1));
}
TEST(Distance, ConservativeAgainstAreaOracleAndLifecycle) {
  std::mt19937 random(42);
  std::uniform_real_distribution<double> position(0, 3.2);
  for (int repeat = 0; repeat < 10; ++repeat) {
    auto map = grid(32, 32);
    map.origin = {-1.3, -0.7};
    for (auto &value : map.cells) value = random() % 31 == 0 ? 100 : 0;
    const tdt::Environment environment(map, nullptr, {});
    for (int query = 0; query < 200; ++query) {
      const tdt::Point point = map.origin + tdt::Point(position(random), position(random));
      EXPECT_GE(environment.clearance(point), 0);
      EXPECT_LE(environment.clearance(point), oracle(map, point) + 1e-12);
    }
  }
}
TEST(Distance, LongQueueAndBoundaryEnvelope) {
  auto map = grid(1105, 4);
  map.cells[map.width + 1] = 100;
  const tdt::Environment long_map(map, nullptr, {});
  EXPECT_GE(long_map.clearance({109, 0.25}), 0);
  const tdt::Environment environment(grid(), nullptr, {});
  EXPECT_FALSE(environment.safe({0.50, 2}));
  EXPECT_TRUE(environment.safe({0.51, 2}));
  EXPECT_FALSE(environment.safe({NAN, 2}));
  EXPECT_FALSE(environment.segmentSafe({1, 1}, {-1, 1}));
  EXPECT_FALSE(environment.segmentSafe({1, 1}, {2, 2}, 0.0, tdt::Clock::now()));
}
TEST(Search, ExactEndpointsSimplificationAndDeterminism) {
  const tdt::Environment environment(grid(), nullptr, {});
  const tdt::Point start(1.03, 1.04), goal(6.21, 4.35);
  const auto deadline = tdt::Clock::now() + std::chrono::seconds(3);
  const auto path = tdt::search(environment, start, goal, deadline);
  EXPECT_EQ(path.front(), start); EXPECT_EQ(path.back(), goal);
  EXPECT_TRUE(environment.pathSafe(path));
  const auto reduced = tdt::simplify(environment, path);
  EXPECT_EQ(reduced.size(), 2u);
  EXPECT_TRUE(environment.pathSafe(reduced));
  EXPECT_EQ(path, tdt::search(environment, start, goal, deadline));
  EXPECT_THROW(tdt::simplify(environment, path, tdt::Clock::now()), std::runtime_error);
}
TEST(Search, NoCornerCutNoSolutionAndBudget) {
  auto map = grid(4, 4, 1.0);
  map.cells[1] = 100; map.cells[4] = 100;
  tdt::Config config; config.body_length = config.body_width = 0.01; config.clearance_margin = 0;
  const tdt::Environment environment(map, nullptr, config);
  EXPECT_THROW(tdt::search(environment, {0.5, 0.5}, {2.5, 2.5},
    tdt::Clock::now() + std::chrono::seconds(1)), std::runtime_error);
  const tdt::Environment free(grid(), nullptr, {});
  EXPECT_THROW(tdt::search(free, {1, 1}, {6, 4}, tdt::Clock::now()), std::runtime_error);
  const auto same = tdt::plan(free, {1, 1}, {1, 1});
  EXPECT_FALSE(same.success); EXPECT_TRUE(same.path.empty());
}
TEST(Search, DuplicatePointsAndNarrowPassage) {
  const tdt::Environment free(grid(), nullptr, {});
  const tdt::Path duplicated{{1, 1}, {1, 1}, {2, 2}, {2, 2}, {5, 4}};
  const auto reduced = tdt::simplify(free, duplicated);
  ASSERT_EQ(reduced.size(), 2u);
  EXPECT_EQ(reduced.front(), duplicated.front());
  EXPECT_EQ(reduced.back(), duplicated.back());
  auto narrow = grid();
  for (int row = 0; row < narrow.height; ++row)
    if (row < 26 || row >= 34) narrow.cells[row * narrow.width + 40] = 100;
  const tdt::Environment blocked(narrow, nullptr, {});
  const auto result = tdt::plan(blocked, {1, 3}, {7, 3});
  EXPECT_FALSE(result.success); EXPECT_TRUE(result.path.empty());
}
TEST(Optimizer, RealQpStraightAndDiagonal) {
  const tdt::Environment environment(grid(), nullptr, {});
  for (const auto &goal : {tdt::Point(6, 1), tdt::Point(6, 4)}) {
    const auto result = tdt::plan(environment, {1, 1}, goal);
    ASSERT_TRUE(result.success) << result.reason;
    EXPECT_EQ(result.qp_solves, 2);
    EXPECT_EQ(result.path.front(), tdt::Point(1, 1));
    EXPECT_EQ(result.path.back(), goal);
    EXPECT_TRUE(environment.pathSafe(result.path));
    ASSERT_EQ(result.coefficients.size(), result.segment_times.size());
    const auto &first = result.coefficients.front();
    const auto &last = result.coefficients.back();
    EXPECT_LT(first.row(1).norm() / result.segment_times.front(), 1e-5);
    const Eigen::RowVector2d final_velocity = (last.row(1) + 2 * last.row(2) +
      3 * last.row(3) + 4 * last.row(4) + 5 * last.row(5)) / result.segment_times.back();
    EXPECT_LT(final_velocity.norm(), 1e-5);
    for (size_t index = 1; index < result.path.size(); ++index)
      EXPECT_LE((result.path[index] - result.path[index - 1]).norm(), 0.100001);
  }
}
TEST(Optimizer, ObstacleTurnNeverFallsBackToUnsafePath) {
  auto map = grid(100, 80);
  for (int row = 25; row <= 54; ++row)
    for (int col = 40; col <= 59; ++col) map.cells[row * map.width + col] = 100;
  tdt::Config config;
  config.search_budget_ms = 5000;
  config.total_budget_ms = 15000;
  const tdt::Environment environment(map, nullptr, config);
  const auto result = tdt::plan(environment, {1, 4}, {9, 4});
  ASSERT_FALSE(result.raw.empty()) << result.reason;
  EXPECT_TRUE(environment.pathSafe(result.raw));
  EXPECT_TRUE(environment.pathSafe(result.simplified));
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_TRUE(environment.pathSafe(result.path));
  ASSERT_GT(result.coefficients.size(), 1u);
  for (size_t segment = 1; segment < result.coefficients.size(); ++segment) {
    const auto &previous = result.coefficients[segment - 1];
    const auto &current = result.coefficients[segment];
    const double before = result.segment_times[segment - 1], after = result.segment_times[segment];
    EXPECT_LT((previous.colwise().sum() - current.row(0)).norm(), 1e-4);
    const Eigen::RowVector2d velocity = previous.row(1) + 2 * previous.row(2) +
      3 * previous.row(3) + 4 * previous.row(4) + 5 * previous.row(5);
    const Eigen::RowVector2d acceleration = 2 * previous.row(2) + 6 * previous.row(3) +
      12 * previous.row(4) + 20 * previous.row(5);
    EXPECT_LT((velocity / before - current.row(1) / after).norm(), 1e-4);
    EXPECT_LT((acceleration / (before * before) - 2 * current.row(2) / (after * after)).norm(), 1e-4);
  }
  for (const auto &coefficients : result.coefficients)
    for (int sample = 0; sample <= 1000; ++sample) {
      const double parameter = sample / 1000.0;
      tdt::Point point = coefficients.row(5).transpose();
      for (int power = 4; power >= 0; --power) point = point * parameter + coefficients.row(power).transpose();
      EXPECT_TRUE(environment.safe(point));
    }
}
TEST(Optimizer, SCurveIsCertifiedOrFailsWithoutFallback) {
  auto map = grid(60, 50, 0.2);
  for (int row = 0; row < 33; ++row) map.cells[row * map.width + 20] = 100;
  for (int row = 17; row < map.height; ++row) map.cells[row * map.width + 38] = 100;
  tdt::Config config;
  config.search_budget_ms = 5000;
  config.total_budget_ms = 15000;
  const tdt::Environment environment(map, nullptr, config);
  const auto result = tdt::plan(environment, {1, 5}, {11, 5});
  ASSERT_FALSE(result.raw.empty()) << result.reason;
  EXPECT_TRUE(environment.pathSafe(result.raw));
  EXPECT_TRUE(environment.pathSafe(result.simplified));
  if (result.success) EXPECT_TRUE(environment.pathSafe(result.path));
  else {
    EXPECT_TRUE(result.path.empty());
    EXPECT_FALSE(result.reason.empty());
  }
}
TEST(Optimizer, LimitsFailClosed) {
  tdt::Config config; config.qp_max_iterations = 1;
  const tdt::Environment environment(grid(), nullptr, config);
  const auto failed = tdt::plan(environment, {1, 1}, {6, 4});
  EXPECT_FALSE(failed.success); EXPECT_TRUE(failed.path.empty());
  config = {}; config.max_check_samples = 2;
  const tdt::Environment tiny(grid(), nullptr, config);
  const auto limited = tdt::plan(tiny, {1, 1}, {6, 4});
  EXPECT_FALSE(limited.success); EXPECT_TRUE(limited.path.empty());
  config = {}; config.qp_budget_ms = 1e-6;
  const tdt::Environment timeout(grid(), nullptr, config);
  const auto timed = tdt::plan(timeout, {1, 1}, {6, 4});
  EXPECT_FALSE(timed.success); EXPECT_TRUE(timed.path.empty());
  EXPECT_GT(timed.qp_solves, 0);
}
TEST(Snapshot, RoundTripAndMalformed) {
  tdt::Snapshot snapshot;
  snapshot.base = grid(); snapshot.dynamic = grid(4, 4, 0.2);
  snapshot.has_dynamic = true; snapshot.start = {1, 1}; snapshot.goal = {6, 4};
  const std::string filename = "/tmp/tdt-snapshot-test-" + std::to_string(getpid());
  tdt::saveSnapshot(snapshot, filename);
  const auto loaded = tdt::loadSnapshot(filename);
  EXPECT_EQ(loaded.base.cells, snapshot.base.cells);
  EXPECT_EQ(loaded.dynamic.cells, snapshot.dynamic.cells);
  EXPECT_EQ(loaded.goal, snapshot.goal);
  EXPECT_EQ(loaded.config.max_check_samples, snapshot.config.max_check_samples);
  const tdt::Environment environment(loaded.base, &loaded.dynamic, loaded.config);
  EXPECT_TRUE(tdt::plan(environment, loaded.start, loaded.goal).success);
  std::ofstream(filename) << "TDT_SNAPSHOT 2\n";
  EXPECT_THROW(tdt::loadSnapshot(filename), std::invalid_argument);
  std::remove(filename.c_str());
}
}
