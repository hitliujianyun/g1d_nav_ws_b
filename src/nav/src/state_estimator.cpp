#include "g1d/nav/state_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "g1d/common/config.hpp"

namespace g1d::nav
{

StateEstimatorParams StateEstimatorParams::fromYaml(const YAML::Node & root)
{
  StateEstimatorParams p;
  p.wheel_timeout_s = get(root, "state.wheel_timeout_s", p.wheel_timeout_s);
  p.lio_timeout_s = get(root, "state.lio_timeout_s", p.lio_timeout_s);
  p.max_extrapolation_s = get(root, "state.max_extrapolation_s", p.max_extrapolation_s);
  p.consistency_window_s = get(root, "state.consistency_window_s", p.consistency_window_s);
  p.max_lio_wheel_translation_m =
    get(root, "state.max_lio_wheel_translation_m", p.max_lio_wheel_translation_m);
  p.max_lio_wheel_yaw_deg = get(root, "state.max_lio_wheel_yaw_deg", p.max_lio_wheel_yaw_deg);
  p.use_wheel_odom = get(root, "state.use_wheel_odom", p.use_wheel_odom);
  p.allow_wheel_only = get(root, "state.allow_wheel_only", p.allow_wheel_only);
  return p;
}

StateEstimator::StateEstimator(StateEstimatorParams p, BodyKinematics kin)
: p_(p), kin_(std::move(kin)) {}

void StateEstimator::reset()
{
  T_odom_lio_.reset();
  lio_anchor_.reset();
  lio_prev_.reset();
  lio_ok_ = false;
  wheel_.clear();
  lio_hist_.clear();
  history_.clear();
  wheel_stamp_ = -1e9;
  consistent_ = true;
}

void StateEstimator::setGravity(const Eigen::Vector3d & g_lio)
{
  if (!T_odom_lio_ || g_lio.norm() < 1e-6) {return;}
  const Eigen::Vector3d g = T_odom_lio_->linear() * g_lio.normalized();
  gravity_tilt_deg_ = std::acos(std::clamp(-g.z(), -1.0, 1.0)) * 180.0 / M_PI;
}

void StateEstimator::onLio(const LioInput & in)
{
  lio_ok_ = in.ok;
  if (!in.ok) {return;}
  const Eigen::Isometry3d T_base_imu = kin_.baseToImu(joints_);
  if (!T_odom_lio_) {
    // odom origin = base_link at this instant (base assumed level on the floor).
    T_odom_lio_ = T_base_imu * in.T_lio_imu.inverse();
  }
  const Eigen::Isometry3d T_odom_base = (*T_odom_lio_) * in.T_lio_imu * T_base_imu.inverse();
  const Pose2D pose = toPose2D(T_odom_base);

  if (lio_prev_ && in.stamp > lio_prev_->stamp) {
    const double dt = in.stamp - lio_prev_->stamp;
    const Pose2D rel = lio_prev_->pose.relative(pose);
    lio_twist_.v = rel.x / dt;
    lio_twist_.w = rel.yaw / dt;
  }
  lio_prev_ = StampedPose{in.stamp, pose};
  lio_anchor_ = StampedPose{in.stamp, pose};
  lio_hist_.add(in.stamp, pose);

  // LIO vs wheel increments over the consistency window.
  if (p_.use_wheel_odom) {
    const double t0 = in.stamp - p_.consistency_window_s;
    const auto l0 = lio_hist_.lookup(t0, 0.0);
    const auto w0 = wheel_.lookup(t0, 0.0);
    const auto w1 = wheel_.lookup(in.stamp, p_.max_extrapolation_s);
    if (l0 && w0 && w1) {
      const Pose2D dl = l0->relative(pose);
      const Pose2D dw = w0->relative(*w1);
      const double dtrans = std::hypot(dl.x - dw.x, dl.y - dw.y);
      const double dyaw = std::abs(wrapAngle(dl.yaw - dw.yaw)) * 180.0 / M_PI;
      consistent_ = dtrans <= p_.max_lio_wheel_translation_m && dyaw <= p_.max_lio_wheel_yaw_deg;
    }
  }
}

void StateEstimator::onWheelOdom(double stamp, const Pose2D & wheel_pose, double v, double w)
{
  wheel_.add(stamp, wheel_pose);
  wheel_twist_ = Twist2D{v, w};
  wheel_stamp_ = stamp;
}

NavState StateEstimator::estimate(double now) const
{
  NavState s;
  s.stamp = now;
  s.wheel_ok = p_.use_wheel_odom && (now - wheel_stamp_) <= p_.wheel_timeout_s;
  s.lio_ok = lio_ok_ && lio_anchor_ && (now - lio_anchor_->stamp) <= p_.lio_timeout_s;
  s.consistent = consistent_;

  if (s.lio_ok) {
    Pose2D pose = lio_anchor_->pose;
    double t_from = lio_anchor_->stamp;
    if (s.wheel_ok) {
      const auto w_anchor = wheel_.lookup(lio_anchor_->stamp, p_.max_extrapolation_s);
      const auto w_last = wheel_.latest();
      if (w_anchor && w_last && w_last->first >= lio_anchor_->stamp) {
        pose = pose.compose(w_anchor->relative(w_last->second));
        t_from = w_last->first;
      }
      s.velocity = wheel_twist_;
      s.source = "lio+wheel";
    } else {
      s.velocity = lio_twist_;
      s.source = "lio";
    }
    const double dt = std::clamp(now - t_from, 0.0, p_.max_extrapolation_s);
    s.pose = pose.compose(Pose2D(s.velocity.v * dt, 0.0, s.velocity.w * dt));
    s.valid = true;
  } else if (s.wheel_ok && p_.allow_wheel_only && lio_anchor_) {
    const auto w_anchor = wheel_.lookup(lio_anchor_->stamp, 1e9);
    const auto w_last = wheel_.latest();
    if (w_anchor && w_last) {
      s.pose = lio_anchor_->pose.compose(w_anchor->relative(w_last->second));
      s.velocity = wheel_twist_;
      s.source = "wheel";
      s.valid = true;
    }
  }
  return s;
}

}  // namespace g1d::nav
