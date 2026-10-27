#ifndef PATH_SEARCHING_PATH_SIMPLIFIER_H_
#define PATH_SEARCHING_PATH_SIMPLIFIER_H_

#include "path_searching/grid_geometry.h"

#include <functional>
#include <string>
#include <vector>

namespace path_searching
{

class PathSimplifier
{
public:
  using Path = std::vector<Eigen::Vector2d>;
  using CellQuery = std::function<bool(const Eigen::Vector2i &)>;

  enum class Status { Simplified, Unchanged, InvalidInput, InvalidEnvironment };

  struct Result
  {
    Status status{Status::InvalidInput};
    Path path;
    std::string message;
    bool valid() const
    {
      return status == Status::Simplified || status == Status::Unchanged;
    }
  };

  explicit PathSimplifier(double tolerance_m = 0.1);
  Result simplify(const Path &path, const GridGeometry &geometry, const CellQuery &query) const;
  bool segmentPassable(
    const Eigen::Vector2d &start, const Eigen::Vector2d &end,
    const GridGeometry &geometry, const CellQuery &query) const;

private:
  double tolerance_m_;
};

}

#endif
