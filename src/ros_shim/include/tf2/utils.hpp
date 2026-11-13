#pragma once

#include <cmath>

#include "ros_shim/msgs.hpp"

namespace tf2
{

/// Minimal tf2::Quaternion (x, y, z, w) with the operations used by Nav2.
class Quaternion
{
public:
  Quaternion() = default;
  Quaternion(double x, double y, double z, double w)
  : x_(x), y_(y), z_(z), w_(w) {}

  void setRPY(double roll, double pitch, double yaw)
  {
    const double hr = roll * 0.5, hp = pitch * 0.5, hy = yaw * 0.5;
    const double cr = std::cos(hr), sr = std::sin(hr);
    const double cp = std::cos(hp), sp = std::sin(hp);
    const double cy = std::cos(hy), sy = std::sin(hy);
    x_ = sr * cp * cy - cr * sp * sy;
    y_ = cr * sp * cy + sr * cp * sy;
    z_ = cr * cp * sy - sr * sp * cy;
    w_ = cr * cp * cy + sr * sp * sy;
  }
  void setValue(double x, double y, double z, double w) {x_ = x; y_ = y; z_ = z; w_ = w;}
  double x() const {return x_;}
  double y() const {return y_;}
  double z() const {return z_;}
  double w() const {return w_;}
  double getX() const {return x_;}
  double getY() const {return y_;}
  double getZ() const {return z_;}
  double getW() const {return w_;}
  Quaternion normalized() const
  {
    const double n = std::sqrt(x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_);
    return n > 0.0 ? Quaternion(x_ / n, y_ / n, z_ / n, w_ / n) : Quaternion();
  }

private:
  double x_{0.0}, y_{0.0}, z_{0.0}, w_{1.0};
};

inline double getYaw(double x, double y, double z, double w)
{
  // Same convention as tf2::getYaw (ZYX Euler, returns yaw).
  const double sqx = x * x, sqy = y * y, sqz = z * z, sqw = w * w;
  const double sarg = -2.0 * (x * z - w * y) / (sqx + sqy + sqz + sqw);
  if (sarg <= -0.99999) {
    return -2.0 * std::atan2(y, x);
  }
  if (sarg >= 0.99999) {
    return 2.0 * std::atan2(y, x);
  }
  return std::atan2(2.0 * (x * y + w * z), sqw + sqx - sqy - sqz);
}

inline double getYaw(const geometry_msgs::msg::Quaternion & q) {return getYaw(q.x, q.y, q.z, q.w);}
inline double getYaw(const Quaternion & q) {return getYaw(q.x(), q.y(), q.z(), q.w());}

inline geometry_msgs::msg::Quaternion toMsg(const Quaternion & q)
{
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}

inline void fromMsg(const geometry_msgs::msg::Quaternion & m, Quaternion & q)
{
  q.setValue(m.x, m.y, m.z, m.w);
}

}  // namespace tf2
