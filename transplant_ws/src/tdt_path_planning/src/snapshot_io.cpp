#include "tdt_path_planning/core.hpp"
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace tdt {
namespace {
template<class Stream, class ConfigType>
void configuration(Stream &stream, ConfigType &config) {
  stream(config.body_length, config.body_width, config.clearance_margin,
    config.occupied_threshold, config.obstacle_weight, config.simplify_epsilon,
    config.corridor_range, config.allocation_speed, config.allocation_acceleration,
    config.min_segment_time, config.max_chord, config.curve_error, config.output_spacing,
    config.search_budget_ms, config.qp_budget_ms, config.total_budget_ms,
    config.max_refine_rounds, config.max_control_points, config.max_check_samples,
    config.max_search_expansions, config.qp_max_iterations);
}
void writeGrid(std::ostream &stream, const Grid &grid) {
  grid.validate();
  stream << grid.width << ' ' << grid.height << ' ' << grid.resolution << ' '
    << grid.origin.x() << ' ' << grid.origin.y() << '\n';
  for (auto value : grid.cells) stream << int(value) << ' ';
  stream << '\n';
}
Grid readGrid(std::istream &stream) {
  Grid grid;
  if (!(stream >> grid.width >> grid.height >> grid.resolution >> grid.origin.x() >> grid.origin.y()) ||
    grid.width < 1 || grid.height < 1 || grid.width > 4096 || grid.height > 4096 ||
    size_t(grid.width) * grid.height > 4000000)
    throw std::invalid_argument("invalid snapshot grid header");
  grid.cells.resize(size_t(grid.width) * grid.height);
  for (auto &value : grid.cells) {
    int encoded;
    if (!(stream >> encoded) || encoded < -1 || encoded > 100)
      throw std::invalid_argument("invalid snapshot cell");
    value = static_cast<int8_t>(encoded);
  }
  grid.validate();
  return grid;
}
}
void saveSnapshot(const Snapshot &snapshot, const std::string &filename) {
  snapshot.config.validate();
  snapshot.base.validate();
  if (snapshot.has_dynamic) snapshot.dynamic.validate();
  if (!snapshot.start.allFinite() || !snapshot.goal.allFinite())
    throw std::invalid_argument("invalid snapshot endpoints");
  std::ofstream output(filename);
  output << std::setprecision(std::numeric_limits<double>::max_digits10);
  output << "TDT_SNAPSHOT 1\n";
  auto writer = [&output](const auto &...values) {((output << values << ' '), ...); output << '\n';};
  configuration(writer, snapshot.config);
  output << snapshot.start.x() << ' ' << snapshot.start.y() << ' '
    << snapshot.goal.x() << ' ' << snapshot.goal.y() << ' ' << snapshot.has_dynamic << '\n';
  writeGrid(output, snapshot.base);
  if (snapshot.has_dynamic) writeGrid(output, snapshot.dynamic);
  output.flush();
  if (!output) throw std::runtime_error("snapshot write failed");
}
Snapshot loadSnapshot(const std::string &filename) {
  std::ifstream input(filename);
  std::string magic;
  int version = 0;
  if (!(input >> magic >> version) || magic != "TDT_SNAPSHOT" || version != 1)
    throw std::invalid_argument("unsupported snapshot header");
  Snapshot snapshot;
  auto reader = [&input](auto &...values) {((input >> values), ...);};
  configuration(reader, snapshot.config);
  snapshot.config.validate();
  int dynamic;
  if (!(input >> snapshot.start.x() >> snapshot.start.y() >> snapshot.goal.x() >> snapshot.goal.y() >> dynamic) ||
    (dynamic != 0 && dynamic != 1) || !snapshot.start.allFinite() || !snapshot.goal.allFinite())
    throw std::invalid_argument("invalid snapshot metadata");
  snapshot.has_dynamic = dynamic;
  snapshot.base = readGrid(input);
  if (snapshot.has_dynamic) snapshot.dynamic = readGrid(input);
  input >> std::ws;
  if (!input.eof()) throw std::invalid_argument("unexpected trailing snapshot data");
  return snapshot;
}
}
