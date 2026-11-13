#include "g1d/sim/robot.hpp"

#include <algorithm>
#include <cmath>

namespace g1d::sim
{

double EllipseTrajectory::phase(double t) const
{
  const double phi0 = -M_PI / 2.0;
  if (t <= still_s) {return phi0;}
  const double tr = t - still_s;
  // Rate ramps with a smoothstep; integral of omega * smoothstep(tr / ramp_s).
  if (tr < ramp_s) {
    const double u = tr / ramp_s;
    const double integral = ramp_s * (u * u * u - 0.5 * u * u * u * u);  // ∫(3u²-2u³)
    return phi0 + omega * integral;
  }
  return phi0 + omega * (0.5 * ramp_s + (tr - ramp_s));
}

Eigen::Isometry3d EllipseTrajectory::basePose(double t) const
{
  const double phi = phase(t);
  const double x = center.x() + a * std::cos(phi);
  const double y = center.y() + b * std::sin(phi);
  // Tangent direction for increasing phi.
  const double yaw = std::atan2(b * std::cos(phi), -a * std::sin(phi));
  return toIsometry(Pose2D(x, y, yaw));
}

void DiffDriveModel::step(double dt)
{
  const double cv = std::clamp(cmd_v_, -p_.max_v, p_.max_v);
  const double cw = std::clamp(cmd_w_, -p_.max_w, p_.max_w);
  const double a = std::clamp((cv - v_) / std::max(p_.tau, dt), -p_.max_acc, p_.max_acc);
  const double aw = std::clamp((cw - w_) / std::max(p_.tau, dt), -p_.max_ang_acc, p_.max_ang_acc);
  v_ += a * dt;
  w_ += aw * dt;
  // Exact arc integration.
  const double th = pose_.yaw;
  if (std::abs(w_) < 1e-6) {
    pose_.x += v_ * dt * std::cos(th);
    pose_.y += v_ * dt * std::sin(th);
  } else {
    const double th2 = th + w_ * dt;
    pose_.x += v_ / w_ * (std::sin(th2) - std::sin(th));
    pose_.y += -v_ / w_ * (std::cos(th2) - std::cos(th));
  }
  pose_.yaw = wrapAngle(th + w_ * dt);
}

std::vector<Eigen::Vector2d> footprintAt(
  const Pose2D & pose, const std::vector<Eigen::Vector2d> & footprint_base)
{
  std::vector<Eigen::Vector2d> out;
  out.reserve(footprint_base.size());
  for (const auto & p : footprint_base) {out.push_back(pose.transformPoint(p));}
  return out;
}

}  // namespace g1d::sim
