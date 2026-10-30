#include "tdt_path_planning/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  try {
    if (argc < 2 || argc > 3) {
      std::cerr << "usage: offline_planner SNAPSHOT [OUTPUT_CSV]\n";
      return 2;
    }
    const auto snapshot = tdt::loadSnapshot(argv[1]);
    const tdt::Environment environment(snapshot.base, snapshot.has_dynamic ? &snapshot.dynamic : nullptr, snapshot.config);
    const auto result = tdt::plan(environment, snapshot.start, snapshot.goal);
    std::cout << "status=" << result.reason << " raw=" << result.raw.size()
      << " simplified=" << result.simplified.size() << " output=" << result.path.size()
      << " rounds=" << result.rounds << " qp_solves=" << result.qp_solves
      << " search_ms=" << result.search_ms << " optimize_ms=" << result.optimize_ms
      << " total_ms=" << result.total_ms << '\n';
    if (argc == 3) {
      std::ofstream output(argv[2]);
      output << std::setprecision(17) << "x_m,y_m\n";
      for (const auto &point : result.path) output << point.x() << ',' << point.y() << '\n';
      output.flush();
      if (!output) throw std::runtime_error("path write failed");
    }
    return result.success ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
