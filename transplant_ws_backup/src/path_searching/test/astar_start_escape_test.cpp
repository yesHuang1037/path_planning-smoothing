#include "Astar.h"
#include "esdf_map.h"

#include <Eigen/Core>

#include <cassert>
#include <iostream>
#include <memory>

namespace
{

ESDF_enviroment::Ptr makeMapWithObstacleNearStart()
{
  constexpr int rows = 30;
  constexpr int cols = 30;
  auto map = std::make_unique<bool[]>(rows * cols);
  for (int i = 0; i < rows * cols; ++i)
  {
    map[i] = false;
  }

  map[15 * cols + 15] = true;

  ESDF_enviroment::Ptr esdf(
    new ESDF_enviroment::esdf(),
    [](ESDF_enviroment::esdf *) {});
  esdf->esdf_init(map.get(), rows, cols, Eigen::Vector2d::Zero(), false);
  return esdf;
}

}  // namespace

int main()
{
  const auto esdf = makeMapWithObstacleNearStart();
  const Eigen::Vector2i start(15, 14);
  const Eigen::Vector2i goal(15, 25);
  constexpr double min_clearance_cells = 3.0;

  assert(!esdf->checkCollision(start));
  assert(esdf->getDist(start) < min_clearance_cells);
  assert(esdf->getDist(goal) > min_clearance_cells);

  navi_planner::Astar planner;
  planner.setEnvironment(esdf);
  planner.setParam(
    10.0,
    0.0,
    min_clearance_cells,
    true,
    8);
  planner.init();

  const int result = planner.search(start, goal);
  assert(result == navi_planner::Astar::REACH_END);
  assert(planner.formatLastSearchDebugInfo().find("escape_mode=true") != std::string::npos);
  assert(planner.getPath().size() > 1);

  std::cout << "astar_start_escape_test passed\n";
  return 0;
}
