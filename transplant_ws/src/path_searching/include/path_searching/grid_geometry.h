#ifndef PATH_SEARCHING_GRID_GEOMETRY_H_
#define PATH_SEARCHING_GRID_GEOMETRY_H_

#include <Eigen/Core>

#include <cstdint>
#include <cmath>

namespace path_searching
{

class GridGeometry
{
public:
  GridGeometry() = default;

  GridGeometry(
    const Eigen::Vector2i &size,
    const Eigen::Vector2d &offset,
    double resolution)
  : size_(size), offset_(offset), resolution_(resolution)
  {
  }

  bool valid() const
  {
    return size_[0] > 0 && size_[1] > 0 && resolution_ > 0.0;
  }

  const Eigen::Vector2i &size() const { return size_; }
  const Eigen::Vector2d &offset() const { return offset_; }
  double resolution() const { return resolution_; }

  bool inBounds(const Eigen::Vector2i &idx) const
  {
    return
      idx[0] >= 0 && idx[1] >= 0 &&
      idx[0] < size_[0] && idx[1] < size_[1];
  }

  Eigen::Vector2i worldToIndex(const Eigen::Vector2d &pos) const
  {
    Eigen::Vector2i index;
    index[1] = static_cast<int>(std::floor((pos[0] - offset_[0]) / resolution_));
    index[0] = static_cast<int>(std::floor((pos[1] - offset_[1]) / resolution_));
    return index;
  }

  Eigen::Vector2d indexToWorld(const Eigen::Vector2i &index) const
  {
    Eigen::Vector2d pos;
    pos[0] = static_cast<double>(index[1]) * resolution_ + offset_[0];
    pos[1] = static_cast<double>(index[0]) * resolution_ + offset_[1];
    return pos;
  }

  int metersToCellsCeil(double meters) const
  {
    if (resolution_ <= 0.0)
    {
      return 0;
    }
    return static_cast<int>(std::ceil(std::max(0.0, meters) / resolution_));
  }

  int metersToCellsRound(double meters) const
  {
    if (resolution_ <= 0.0)
    {
      return 0;
    }
    return static_cast<int>(std::round(std::max(0.0, meters) / resolution_));
  }

  uint64_t packKey(const Eigen::Vector2i &idx) const
  {
    return
      (static_cast<uint64_t>(static_cast<uint32_t>(idx[0])) << 32) |
      static_cast<uint32_t>(idx[1]);
  }

  Eigen::Vector2i unpackKey(uint64_t key) const
  {
    return Eigen::Vector2i(
      static_cast<int>(key >> 32),
      static_cast<int>(key & 0xffffffffu));
  }

private:
  Eigen::Vector2i size_{Eigen::Vector2i::Zero()};
  Eigen::Vector2d offset_{Eigen::Vector2d::Zero()};
  double resolution_{0.0};
};

}  // namespace path_searching

#endif  // PATH_SEARCHING_GRID_GEOMETRY_H_
