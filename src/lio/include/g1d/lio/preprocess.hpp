#pragma once

#include <array>
#include <vector>

#include <Eigen/Geometry>

#include "fast_lio_api.h"
#include "g1d/lio/livox_frame.hpp"

namespace g1d::lio
{

/// Axis-aligned box in some frame: [xmin, xmax, ymin, ymax, zmin, zmax].
using Box = std::array<double, 6>;

inline bool inBox(const Box & b, const Eigen::Vector3d & p)
{
  return p.x() >= b[0] && p.x() <= b[1] && p.y() >= b[2] && p.y() <= b[3] &&
         p.z() >= b[4] && p.z() <= b[5];
}

/// Removes points on the robot itself. The Mid-360S is rigidly mounted on the head, so torso /
/// head / column (and the tucked arms) occupy fixed boxes in the head frame.
class SelfFilter
{
public:
  SelfFilter() = default;
  /// `T_head_lidar`: LiDAR pose in the head frame; `boxes`: boxes in the head frame.
  SelfFilter(const Eigen::Isometry3d & T_head_lidar, std::vector<Box> boxes)
  : T_head_lidar_(T_head_lidar), boxes_(std::move(boxes)) {}

  bool isSelf(float x, float y, float z) const
  {
    if (boxes_.empty()) {return false;}
    const Eigen::Vector3d p = T_head_lidar_ * Eigen::Vector3d(x, y, z);
    for (const auto & b : boxes_) {
      if (inBox(b, p)) {return true;}
    }
    return false;
  }

private:
  Eigen::Isometry3d T_head_lidar_{Eigen::Isometry3d::Identity()};
  std::vector<Box> boxes_;
};

struct PreprocessParams
{
  double blind{0.6};        // m; points closer than this are dropped
  int point_filter_num{1};  // keep every N-th valid point
  double max_range{60.0};   // m
};

struct PreprocessStats
{
  int input{0};
  int tag_rejected{0};
  int blind_rejected{0};
  int self_rejected{0};
  int output{0};
};

/// Port of FAST-LIO Preprocess::avia_handler (non-feature branch) for Livox frames, plus the
/// self filter. Output points carry `curvature` = time offset in ms, as FAST-LIO expects.
fast_lio_api::Cloud::Ptr preprocessFrame(
  const LidarFrame & frame, const PreprocessParams & params, const SelfFilter & self_filter,
  PreprocessStats * stats = nullptr);

}  // namespace g1d::lio
