#pragma once

#include <cmath>
#include <cstdint>

namespace rclcpp
{

class Duration
{
public:
  Duration() = default;
  Duration(int32_t sec, uint32_t nsec)
  : ns_(static_cast<int64_t>(sec) * 1000000000LL + nsec) {}

  static Duration from_seconds(double s)
  {
    Duration d;
    d.ns_ = static_cast<int64_t>(std::llround(s * 1e9));
    return d;
  }
  static Duration from_nanoseconds(int64_t ns)
  {
    Duration d;
    d.ns_ = ns;
    return d;
  }
  double seconds() const {return static_cast<double>(ns_) * 1e-9;}
  int64_t nanoseconds() const {return ns_;}
  bool operator<(const Duration & o) const {return ns_ < o.ns_;}
  bool operator>(const Duration & o) const {return ns_ > o.ns_;}

private:
  int64_t ns_{0};
};

}  // namespace rclcpp
