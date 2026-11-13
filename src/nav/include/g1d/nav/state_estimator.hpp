#pragma once

#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "g1d/common/pose_buffer.hpp"
#include "g1d/nav/body_kinematics.hpp"
#include "g1d/nav/types.hpp"

namespace g1d::nav
{

struct StateEstimatorParams
{
  double wheel_timeout_s{0.3};
  double lio_timeout_s{0.5};
  double max_extrapolation_s{0.3};
  double consistency_window_s{2.0};
  double max_lio_wheel_translation_m{0.3};
  double max_lio_wheel_yaw_deg{10.0};
  bool use_wheel_odom{true};
  bool allow_wheel_only{false};   // keep navigating on wheel odometry if LIO is lost (not default)

  static StateEstimatorParams fromYaml(const YAML::Node & root);
};

struct NavState
{
  double stamp{0.0};
  Pose2D pose;          // base_link in odom
  Twist2D velocity;     // base frame
  bool valid{false};
  bool lio_ok{false};
  bool wheel_ok{false};
  bool consistent{true};
  std::string source;   // "lio+wheel" | "lio" | "wheel"
};

/// Fuses FAST-LIO poses (10 Hz, head-mounted IMU) and chassis wheel odometry into a continuous
/// base_link pose in `odom` (origin = base at LIO start, gravity aligned).
class StateEstimator
{
public:
  StateEstimator(StateEstimatorParams p, BodyKinematics kin);

  void onJointState(const JointState & q) {joints_ = q; has_joints_ = true;}
  void onLio(const LioInput & in);
  void onWheelOdom(double stamp, const Pose2D & wheel_pose, double v, double w);

  NavState estimate(double now) const;
  std::optional<Pose2D> poseAt(double t) const {return history_.lookup(t, p_.max_extrapolation_s);}
  /// Records `estimate(now)` into the pose history (call from the control loop).
  void record(const NavState & s) {if (s.valid) {history_.add(s.stamp, s.pose);}}

  bool initialized() const {return T_odom_lio_.has_value();}
  /// LIO world -> odom (valid after the first LIO update).
  Eigen::Isometry3d T_odom_lio() const {return T_odom_lio_.value_or(Eigen::Isometry3d::Identity());}
  /// Angle between LIO gravity mapped to odom and -z (extrinsic / mounting sanity check).
  double gravityTiltDeg() const {return gravity_tilt_deg_;}
  const JointState & joints() const {return joints_;}
  void setGravity(const Eigen::Vector3d & g_lio);
  void reset();

private:
  StateEstimatorParams p_;
  BodyKinematics kin_;
  JointState joints_;
  bool has_joints_{false};
  std::optional<Eigen::Isometry3d> T_odom_lio_;
  std::optional<StampedPose> lio_anchor_;
  std::optional<StampedPose> lio_prev_;
  Twist2D lio_twist_;
  bool lio_ok_{false};
  double gravity_tilt_deg_{0.0};
  PoseBuffer wheel_{10.0};
  PoseBuffer lio_hist_{10.0};
  PoseBuffer history_{10.0};
  Twist2D wheel_twist_;
  double wheel_stamp_{-1e9};
  bool consistent_{true};
};

}  // namespace g1d::nav
