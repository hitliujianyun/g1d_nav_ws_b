#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "g1d/nav/types.hpp"

namespace g1d::nav
{

/// Kinematic chain base_link -> head -> livox -> livox IMU, following g1_d_with_dex1_1.urdf
/// (AGV_link -> LZ_ot/mt/it lift stages -> Pitching -> Yaw_Joint (waist pitch, y axis) ->
/// torso_Joint (waist yaw, z axis) -> head_link).
struct KinematicsParams
{
  double base_to_agv_z{0.0};     // AGV_link origin height above the floor (base_link)
  double lift_offset{0.0};       // lift total = lift_scale * hispeed_state.y + lift_offset
  double lift_scale{1.0};
  Eigen::Isometry3d T_head_livox{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_livox_imu{Eigen::Isometry3d::Identity()};  // IMU pose in the LiDAR frame
  double nav_waist_pitch{0.0};
  std::vector<double> arm_tuck;  // 14 values; empty disables the arm check
  double lock_waist_deg{2.0};
  double lock_arm_deg{5.0};
  double lock_lift_m{0.01};
  double lock_lift_speed{0.005};
  bool require_lock{true};

  static KinematicsParams fromYaml(const YAML::Node & root);
};

struct LockStatus
{
  bool locked{true};
  std::vector<std::string> reasons;
};

class BodyKinematics
{
public:
  explicit BodyKinematics(KinematicsParams p = {})
  : p_(std::move(p)) {}

  Eigen::Isometry3d baseToHead(const JointState & q) const;
  Eigen::Isometry3d baseToLivox(const JointState & q) const {return baseToHead(q) * p_.T_head_livox;}
  Eigen::Isometry3d baseToImu(const JointState & q) const {return baseToLivox(q) * p_.T_livox_imu;}

  /// Navigation requires waist / lift / arms to stay at the reference posture.
  LockStatus checkLock(const JointState & q, const JointState & reference) const;

  const KinematicsParams & params() const {return p_;}

private:
  KinematicsParams p_;
};

}  // namespace g1d::nav
