#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/duration.hpp"

namespace rclcpp
{

class Time
{
public:
  Time() = default;
  Time(int32_t sec, uint32_t nsec)
  : ns_(static_cast<int64_t>(sec) * 1000000000LL + nsec) {}
  explicit Time(int64_t nanoseconds)
  : ns_(nanoseconds) {}
  Time(const builtin_interfaces::msg::Time & t)  // NOLINT implicit like rclcpp
  : ns_(static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec) {}

  double seconds() const {return static_cast<double>(ns_) * 1e-9;}
  int64_t nanoseconds() const {return ns_;}

  operator builtin_interfaces::msg::Time() const  // NOLINT
  {
    builtin_interfaces::msg::Time t;
    int64_t s = ns_ / 1000000000LL;
    int64_t r = ns_ % 1000000000LL;
    if (r < 0) {r += 1000000000LL; --s;}
    t.sec = static_cast<int32_t>(s);
    t.nanosec = static_cast<uint32_t>(r);
    return t;
  }

  Time operator+(const Duration & d) const {return Time(ns_ + d.nanoseconds());}
  Time operator-(const Duration & d) const {return Time(ns_ - d.nanoseconds());}
  Duration operator-(const Time & o) const {return Duration::from_nanoseconds(ns_ - o.ns_);}
  bool operator<(const Time & o) const {return ns_ < o.ns_;}
  bool operator>(const Time & o) const {return ns_ > o.ns_;}
  bool operator<=(const Time & o) const {return ns_ <= o.ns_;}
  bool operator>=(const Time & o) const {return ns_ >= o.ns_;}
  bool operator==(const Time & o) const {return ns_ == o.ns_;}

private:
  int64_t ns_{0};
};

inline builtin_interfaces::msg::Time operator+(
  const builtin_interfaces::msg::Time & t, const Duration & d)
{
  return Time(t) + d;
}

class Clock
{
public:
  using SharedPtr = std::shared_ptr<Clock>;
  Time now() const
  {
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    return Time(static_cast<int64_t>(ns));
  }
};

}  // namespace rclcpp
