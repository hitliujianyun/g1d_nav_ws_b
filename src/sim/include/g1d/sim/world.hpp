#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace g1d::sim
{

/// Axis-aligned box obstacle in the world frame.
struct Box
{
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  std::string name;
};

/// Indoor test world: floor (z = 0), optional ceiling, axis-aligned boxes (walls, furniture).
/// Moving boxes can be updated between steps to emulate pedestrians.
class World
{
public:
  World() = default;

  void addBox(const Eigen::Vector3d & min, const Eigen::Vector3d & max, std::string name = "");
  /// Four walls enclosing [x0, x1] x [y0, y1], height h, thickness t.
  void addRoom(double x0, double y0, double x1, double y1, double h = 2.8, double t = 0.2);
  void setCeiling(std::optional<double> z) {ceiling_ = z;}
  std::vector<Box> & boxes() {return boxes_;}
  const std::vector<Box> & boxes() const {return boxes_;}

  /// Distance along unit `dir` from `origin` to the first hit within max_range, if any.
  std::optional<double> raycast(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & dir, double max_range) const;

  /// True if the planar footprint polygon (world xy) intersects any box whose vertical extent
  /// overlaps [z_min, z_max].
  bool footprintCollides(
    const std::vector<Eigen::Vector2d> & polygon, double z_min, double z_max) const;

  /// Smallest planar distance between the footprint polygon and any box overlapping [z_min, z_max].
  double footprintClearance(
    const std::vector<Eigen::Vector2d> & polygon, double z_min, double z_max) const;

  /// A furnished room used by several tests: 12 x 10 m with boxes, a table (overhang) and pillars.
  static World makeTestRoom();

private:
  std::vector<Box> boxes_;
  std::optional<double> ceiling_{2.8};
};

}  // namespace g1d::sim
