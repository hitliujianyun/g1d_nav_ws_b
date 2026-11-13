#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g1d
{

inline double wrapAngle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) {a += 2.0 * M_PI;}
  return a - M_PI;
}

/// Planar pose (x, y, yaw) — the navigation state of the chassis `base_link` in `odom`.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  Pose2D() = default;
  Pose2D(double x_, double y_, double yaw_)
  : x(x_), y(y_), yaw(yaw_) {}

  /// this ∘ rel  (rel expressed in this pose's frame)
  Pose2D compose(const Pose2D & rel) const
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return Pose2D(x + c * rel.x - s * rel.y, y + s * rel.x + c * rel.y, wrapAngle(yaw + rel.yaw));
  }
  Pose2D inverse() const
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return Pose2D(-c * x - s * y, s * x - c * y, wrapAngle(-yaw));
  }
  /// Pose of `other` expressed in this frame.
  Pose2D relative(const Pose2D & other) const {return inverse().compose(other);}
  Eigen::Vector2d transformPoint(const Eigen::Vector2d & p) const
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return Eigen::Vector2d(x + c * p.x() - s * p.y(), y + s * p.x() + c * p.y());
  }
  double distanceTo(const Pose2D & o) const {return std::hypot(o.x - x, o.y - y);}
};

/// Linear interpolation (yaw along the shortest arc), t in [0, 1].
inline Pose2D interpolate(const Pose2D & a, const Pose2D & b, double t)
{
  return Pose2D(
    a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
    wrapAngle(a.yaw + wrapAngle(b.yaw - a.yaw) * t));
}

inline double yawOf(const Eigen::Matrix3d & R) {return std::atan2(R(1, 0), R(0, 0));}

inline Pose2D toPose2D(const Eigen::Isometry3d & T)
{
  return Pose2D(T.translation().x(), T.translation().y(), yawOf(T.rotation()));
}

inline Eigen::Isometry3d toIsometry(const Pose2D & p, double z = 0.0)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(p.x, p.y, z);
  return T;
}

/// Roll-pitch-yaw (fixed-axis XYZ, i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll)) to rotation matrix.
inline Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

inline Eigen::Isometry3d makeIsometry(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = rpyToMatrix(roll, pitch, yaw);
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

/// Rotation that maps unit vector `from` onto unit vector `to` (minimal rotation).
inline Eigen::Matrix3d alignVectors(const Eigen::Vector3d & from, const Eigen::Vector3d & to)
{
  return Eigen::Quaterniond::FromTwoVectors(from, to).toRotationMatrix();
}

}  // namespace g1d
