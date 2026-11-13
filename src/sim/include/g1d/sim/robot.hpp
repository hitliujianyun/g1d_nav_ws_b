#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "g1d/common/geometry.hpp"

namespace g1d::sim
{

/// Smooth closed loop on an ellipse, driven like a differential-drive base (heading = tangent).
/// Stationary for `still_s`, then ramps up to the cruise angular rate over `ramp_s`.
struct EllipseTrajectory
{
  Eigen::Vector2d center{4.2, 0.0};
  double a{2.2};
  double b{1.2};
  double omega{0.035 * 2.0 * M_PI};  // rad/s along the ellipse parameter at cruise
  double still_s{1.5};
  double ramp_s{2.0};

  double phase(double t) const;
  Eigen::Isometry3d basePose(double t) const;  // base_link in world (z = 0)
};

/// First-order differential-drive model with acceleration limits (chassis dynamics stand-in).
class DiffDriveModel
{
public:
  struct Params
  {
    double tau{0.12};          // velocity response time constant (s)
    double max_acc{1.0};       // m/s^2
    double max_ang_acc{2.0};   // rad/s^2
    double max_v{1.5};
    double max_w{0.6};
  };

  DiffDriveModel() = default;
  explicit DiffDriveModel(Params p, Pose2D start = {})
  : p_(p), pose_(start) {}

  void setCommand(double v, double w) {cmd_v_ = v; cmd_w_ = w;}
  void step(double dt);

  const Pose2D & pose() const {return pose_;}
  void setPose(const Pose2D & p) {pose_ = p;}
  double v() const {return v_;}
  double w() const {return w_;}

private:
  Params p_;
  Pose2D pose_;
  double v_{0.0}, w_{0.0};
  double cmd_v_{0.0}, cmd_w_{0.0};
};

/// Footprint polygon (base frame) transformed to world.
std::vector<Eigen::Vector2d> footprintAt(
  const Pose2D & pose, const std::vector<Eigen::Vector2d> & footprint_base);

}  // namespace g1d::sim
