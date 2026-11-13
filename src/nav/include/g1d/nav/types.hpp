#pragma once

#include <array>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "g1d/common/geometry.hpp"

namespace g1d::nav
{

struct Twist2D
{
  double v{0.0};  // m/s, forward
  double w{0.0};  // rad/s, CCW
};

/// G1-D upper-body joint state relevant to the head-mounted LiDAR pose.
struct JointState
{
  double stamp{0.0};
  double lift_height{0.0};    // rt/hispeed_state.y (m)
  double lift_speed{0.0};     // rt/hispeed_state.z (m/s)
  double waist_pitch{0.0};    // rt/waistpitchstate.q (rad), URDF Yaw_Joint (axis y)
  double waist_yaw{0.0};      // rt/lowstate motor 12 (rad), URDF torso_Joint (axis z)
  std::array<double, 14> arms{};  // rt/lowstate motors 15..28
  bool has_lift{false};
  bool has_waist_pitch{false};
  bool has_lowstate{false};
};

/// Chassis 2D scan (rt/slamware_ros_sdk_server_node/scan) in the scanner frame.
struct ScanData
{
  double stamp{0.0};
  float angle_min{0.0f};
  float angle_increment{0.0f};
  float range_min{0.0f};
  float range_max{std::numeric_limits<float>::infinity()};
  std::vector<float> ranges;
};

/// One LIO update, already mapped to host time.
struct LioInput
{
  double stamp{0.0};
  Eigen::Isometry3d T_lio_imu{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d lidar_origin_lio{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_lio{Eigen::Vector3d::Zero()};
  bool ok{false};
  std::vector<Eigen::Vector3f> cloud_lio;  // registered points in the LIO world frame
};

struct StampedPose
{
  double stamp{0.0};
  Pose2D pose;
};

}  // namespace g1d::nav
