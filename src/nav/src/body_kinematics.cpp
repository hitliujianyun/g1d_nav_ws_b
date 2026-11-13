#include "g1d/nav/body_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "g1d/common/config.hpp"

namespace g1d::nav
{
namespace
{
Eigen::Isometry3d trans(double x, double y, double z)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

Eigen::Isometry3d isoFromYaml(const YAML::Node & n, const Eigen::Isometry3d & fallback)
{
  if (!n) {return fallback;}
  const auto xyz = n["xyz"] ? n["xyz"].as<std::vector<double>>() : std::vector<double>{0, 0, 0};
  const auto rpy = n["rpy"] ? n["rpy"].as<std::vector<double>>() : std::vector<double>{0, 0, 0};
  return makeIsometry(xyz.at(0), xyz.at(1), xyz.at(2), rpy.at(0), rpy.at(1), rpy.at(2));
}

constexpr double kDeg = M_PI / 180.0;
}  // namespace

KinematicsParams KinematicsParams::fromYaml(const YAML::Node & root)
{
  KinematicsParams p;
  p.base_to_agv_z = get(root, "kinematics.base_to_agv_z", p.base_to_agv_z);
  p.lift_offset = get(root, "kinematics.lift_offset", p.lift_offset);
  p.lift_scale = get(root, "kinematics.lift_scale", p.lift_scale);
  p.T_head_livox = isoFromYaml(findPath(root, "kinematics.head_to_livox"), p.T_head_livox);
  // FAST-LIO extrinsic_T is the LiDAR origin in the IMU frame (rotation identity on Mid-360).
  const auto ext_T = get(root, "lio.extrinsic_T", std::vector<double>{-0.011, -0.02329, 0.04412});
  Eigen::Isometry3d T_imu_livox = Eigen::Isometry3d::Identity();
  T_imu_livox.translation() = Eigen::Vector3d(ext_T.at(0), ext_T.at(1), ext_T.at(2));
  const auto ext_R = get(root, "lio.extrinsic_R", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
  T_imu_livox.linear() << ext_R.at(0), ext_R.at(1), ext_R.at(2), ext_R.at(3), ext_R.at(4),
    ext_R.at(5), ext_R.at(6), ext_R.at(7), ext_R.at(8);
  p.T_livox_imu = T_imu_livox.inverse();
  p.nav_waist_pitch = get(root, "kinematics.nav_pose.waist_pitch", p.nav_waist_pitch);
  p.arm_tuck = get(root, "kinematics.nav_pose.arm_tuck", p.arm_tuck);
  p.lock_waist_deg = get(root, "kinematics.lock_tolerance.waist_deg", p.lock_waist_deg);
  p.lock_arm_deg = get(root, "kinematics.lock_tolerance.arm_deg", p.lock_arm_deg);
  p.lock_lift_m = get(root, "kinematics.lock_tolerance.lift_m", p.lock_lift_m);
  p.lock_lift_speed = get(root, "kinematics.lock_tolerance.lift_speed", p.lock_lift_speed);
  p.require_lock = get(root, "kinematics.require_lock", p.require_lock);
  return p;
}

Eigen::Isometry3d BodyKinematics::baseToHead(const JointState & q) const
{
  const double total = std::clamp(p_.lift_scale * q.lift_height + p_.lift_offset, 0.0, 0.42);
  const double stage = 0.5 * total;  // the two lift stages extend equally
  Eigen::Isometry3d T = trans(0.0, 0.0, p_.base_to_agv_z);
  T = T * trans(-0.000162639876214998, 0.0, 0.5275);                 // LZ_ot_Joint
  T = T * trans(0.000387669942120007, 0.0, 0.00999999999999995 + stage);  // LZ_mt_Joint
  T = T * trans(0.0013627, 0.00020167, 0.018 + stage);               // LZ_it_Joint
  T = T * trans(-0.0013627, -0.00020167, 0.008);                     // Pitching_Joint
  Eigen::Isometry3d pitch = trans(0.0555, 0.0002165, 0.066);         // Yaw_Joint (axis y)
  pitch.linear() = Eigen::AngleAxisd(q.waist_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  T = T * pitch;
  Eigen::Isometry3d yaw = trans(-0.0023536, -0.000048782, 0.104008566);  // torso_Joint (axis z)
  yaw.linear() = Eigen::AngleAxisd(q.waist_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T = T * yaw;
  T = T * trans(0.0039635, 0.0, -0.054);                             // head_joint
  return T;
}

LockStatus BodyKinematics::checkLock(const JointState & q, const JointState & reference) const
{
  LockStatus s;
  auto fail = [&](const std::string & why) {s.locked = false; s.reasons.push_back(why);};
  if (q.has_lowstate && std::abs(q.waist_yaw) > p_.lock_waist_deg * kDeg) {
    fail("waist_yaw");
  }
  if (q.has_waist_pitch && std::abs(q.waist_pitch - p_.nav_waist_pitch) > p_.lock_waist_deg * kDeg) {
    fail("waist_pitch");
  }
  if (q.has_lift) {
    if (std::abs(q.lift_speed) > p_.lock_lift_speed) {fail("lift_moving");}
    if (reference.has_lift && std::abs(q.lift_height - reference.lift_height) > p_.lock_lift_m) {
      fail("lift_height");
    }
  }
  if (q.has_lowstate && p_.arm_tuck.size() == q.arms.size()) {
    for (size_t i = 0; i < q.arms.size(); ++i) {
      if (std::abs(q.arms[i] - p_.arm_tuck[i]) > p_.lock_arm_deg * kDeg) {
        std::ostringstream ss;
        ss << "arm_" << (15 + i);
        fail(ss.str());
        break;
      }
    }
  }
  return s;
}

}  // namespace g1d::nav
