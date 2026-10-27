#include "esdf_map.h"
#include "path_searching/grid_geometry.h"
#include "path_searching/path_safety_checker.h"

#include <Eigen/Core>

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace
{

ESDF_enviroment::Ptr makeMapWithObstacleOnPath()
{
  constexpr int rows = 40;
  constexpr int cols = 40;
  auto map = std::make_unique<bool[]>(rows * cols);
  for (int i = 0; i < rows * cols; ++i)
  {
    map[i] = false;
  }

  const path_searching::GridGeometry geometry(
    Eigen::Vector2i(rows, cols),
    Eigen::Vector2d(-1.0, -1.0),
    0.1);
  const Eigen::Vector2i obstacle_idx = geometry.worldToIndex(Eigen::Vector2d(1.0, 0.0));
  map[obstacle_idx[0] * cols + obstacle_idx[1]] = true;

  ESDF_enviroment::Ptr esdf(
    new ESDF_enviroment::esdf(),
    [](ESDF_enviroment::esdf *) {});
  esdf->esdf_init(map.get(), rows, cols, geometry.offset(), false);
  return esdf;
}

path_searching::PathSafetyChecker makeChecker(const ESDF_enviroment::Ptr &esdf)
{
  path_searching::PathSafetyChecker checker;
  path_searching::PathSafetyChecker::Config config;
  config.robot_inscribed_radius = 0.05;
  config.clearance_margin = 0.0;
  config.forward_check_distance = 1.5;
  config.forward_sample_step = 0.1;
  config.blocked_path_truncation_margin = 0.2;
  config.enable_blocked_path_truncation = true;
  checker.configure(config);
  checker.setMap(
    path_searching::GridGeometry(
      Eigen::Vector2i(40, 40),
      Eigen::Vector2d(-1.0, -1.0),
      0.1),
    esdf,
    nullptr);
  return checker;
}

}  // namespace

int main()
{
  const auto esdf = makeMapWithObstacleOnPath();
  auto checker = makeChecker(esdf);
  const std::vector<Eigen::Vector2d> path{
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(2.0, 0.0)};
  const Eigen::Vector2d current_pose(0.0, 0.0);

  const auto forward_report = checker.checkForward(path, current_pose);
  assert(!forward_report.passable);

  const auto truncation = checker.truncateBeforeBlockedPoint(path, current_pose);
  assert(truncation.truncated);
  assert(truncation.path.size() >= 2);
  assert(truncation.path.back()[0] < 1.0);
  assert(truncation.endpoint_clearance_m > checker.requiredClearanceMeters());

  path_searching::PathSafetyChecker disabled_checker = makeChecker(esdf);
  path_searching::PathSafetyChecker::Config disabled_config;
  disabled_config.robot_inscribed_radius = 0.05;
  disabled_config.forward_check_distance = 1.5;
  disabled_config.forward_sample_step = 0.1;
  disabled_config.blocked_path_truncation_margin = 0.2;
  disabled_config.enable_blocked_path_truncation = false;
  disabled_checker.configure(disabled_config);
  disabled_checker.setMap(
    path_searching::GridGeometry(
      Eigen::Vector2i(40, 40),
      Eigen::Vector2d(-1.0, -1.0),
      0.1),
    esdf,
    nullptr);

  const auto disabled_truncation =
    disabled_checker.truncateBeforeBlockedPoint(path, current_pose);
  assert(!disabled_truncation.truncated);
  assert(disabled_truncation.path.size() == path.size());

  std::cout << "path_safety_checker_truncation_test passed\n";
  return 0;
}
