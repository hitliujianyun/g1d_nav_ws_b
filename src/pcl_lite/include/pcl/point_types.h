#pragma once

// pcl_lite: the subset of PCL point types used by FAST-LIO2 and ikd-Tree.

#include <cstdint>

#include <Eigen/Core>  // real PCL headers provide Eigen; ikd-Tree relies on it

namespace pcl
{

struct PointXYZ
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct PointXYZI
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
};

/// FAST-LIO stores per-point time offset (ms) in `curvature`.
struct PointXYZINormal
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
  float normal_x{0.0f};
  float normal_y{0.0f};
  float normal_z{0.0f};
  float curvature{0.0f};
};

}  // namespace pcl
