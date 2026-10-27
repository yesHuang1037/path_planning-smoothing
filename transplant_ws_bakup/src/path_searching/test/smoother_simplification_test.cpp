#include "smoother.h"
#include "path_searching/path_safety_checker.h"

#include <cmath>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace
{
void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

template<typename Operation>
void mustReject(Operation operation)
{
  bool rejected = false;
  try { operation(); }
  catch (const std::exception &) { rejected = true; }
  require(rejected, "invalid request was silently accepted");
}
}

int main()
{
  try
  {
    const path_searching::GridGeometry geometry(
      Eigen::Vector2i(80, 80), Eigen::Vector2d(-2.0, -2.0), 0.05);
    auto occupancy = std::make_unique<bool[]>(80 * 80);
    for (int index = 0; index < 80 * 80; ++index) occupancy[index] = false;
    occupancy[40 * 80 + 40] = true;
    ESDF_enviroment::Ptr esdf(new ESDF_enviroment::esdf(), [](ESDF_enviroment::esdf *) {});
    esdf->esdf_init(occupancy.get(), 80, 80, geometry.offset(), false);

    path_searching::PathSafetyChecker checker;
    path_searching::PathSafetyChecker::Config config;
    config.robot_inscribed_radius = 0.08;
    config.clearance_margin = 0.0;
    checker.configure(config);
    checker.setMap(geometry, esdf, nullptr);
    const auto cell_query = [&](const Eigen::Vector2i &cell) {
      if (!geometry.inBounds(cell) || esdf->checkCollision(cell)) return false;
      const auto clearance = checker.clearanceAt(
        geometry.indexToWorld(cell) + Eigen::Vector2d::Constant(0.025));
      return clearance.valid && clearance.safe;
    };
    navi_planner::smoother default_legacy;
    navi_planner::smoother explicit_legacy;
    default_legacy.smoother_setEnvironment(esdf);
    explicit_legacy.smoother_setEnvironment(esdf);
    explicit_legacy.configureSimplification("legacy", 0.1);
    const navi_planner::smoother::VV2D path{
      {-0.975, -0.975}, {-0.725, -0.725}, {-0.475, -0.475}, {-0.225, -0.225}};
    const auto default_result = default_legacy.pathResampling(path);
    const auto explicit_result = explicit_legacy.pathResampling(path);
    require(default_result == explicit_result, "legacy preprocessing changed");
    const auto default_smooth = default_legacy.smooth(path, 0.3, 0.04);
    require(default_smooth == explicit_legacy.smooth(path, 0.3, 0.04),
      "default and explicit legacy smoothing differ");
    std::cout << "PASS default/explicit legacy preprocessing and smoothing equivalence\n";

    navi_planner::smoother candidate;
    mustReject([&] { candidate.configureSimplification("unknown", 0.1); });
    mustReject([&] { candidate.configureSimplification("legacy", -1.0); });
    mustReject([&] { candidate.configureSimplification("tdt", std::numeric_limits<double>::infinity()); });
    mustReject([&] { candidate.configureSimplification("tdt", std::numeric_limits<double>::quiet_NaN()); });
    candidate.configureSimplification("tdt", 0.1);
    mustReject([&] { candidate.pathResampling(path); });
    candidate.setSimplificationEnvironment(geometry, cell_query);
    mustReject([&] { candidate.smooth(path); });
    candidate.smoother_setEnvironment(esdf);
    mustReject([&] { candidate.smooth({}); });
    mustReject([&] { candidate.pathResampling({{3.0, 3.0}}); });
    mustReject([&] { candidate.pathResampling({{std::numeric_limits<double>::quiet_NaN(), 0.0}}); });
    mustReject([&] { candidate.smooth(path, -0.1, 0.1); });
    require(candidate.pathResampling(path).size() == 2, "free chord not integrated");
    const navi_planner::smoother::VV2D singleton{path.front()};
    require(candidate.smooth({path.front(), path.front()}) == singleton,
      "duplicate singleton handling failed");
    const auto smoothed = candidate.smooth(path, 0.3, 0.04);
    require(!smoothed.empty(), "tdt smoothing output empty");
    for (const auto &point : smoothed) require(point.allFinite(), "tdt smoothing output nonfinite");
    std::cout << "PASS configuration, explicit failures, duplicates and retained spline integration\n";

    const navi_planner::smoother::VV2D detour{
      {-0.475, 0.025}, {0.025, -0.475}, {0.525, 0.025}};
    require(candidate.pathResampling(detour).size() == 3, "occupied shortcut accepted");
    const navi_planner::smoother::VV2D clearance_detour{
      {-0.475, 0.075}, {0.025, 0.525}, {0.525, 0.075}};
    require(candidate.pathResampling(clearance_detour).size() == 3, "unsafe clearance shortcut accepted");
    config.robot_inscribed_radius = 0.0;
    checker.configure(config);
    require(candidate.pathResampling(clearance_detour).size() == 2,
      "current clearance configuration not observed");
    std::cout << "PASS real ESDF collision and current PathSafetyChecker clearance reuse\n";
    navi_planner::smoother::VV2D benchmark_path;
    for (int index = 0; index < 5000; ++index)
      benchmark_path.emplace_back(-1.475 + index / 4999.0, -1.475);
    const auto measure = [&](navi_planner::smoother &implementation, const char *label) {
      const auto start = std::chrono::steady_clock::now();
      size_t output_size = 0;
      for (int repeat = 0; repeat < 30; ++repeat)
        output_size = implementation.pathResampling(benchmark_path).size();
      const double mean_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count() / 30.0;
      std::cout << "OBSERVE " << label << " points=5000->" << output_size <<
        " mean_ms=" << mean_ms << " repeats=30 (preprocessing only)\n";
    };
    measure(default_legacy, "legacy");
    measure(candidate, "tdt");
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
