#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>

namespace g1d
{

/// Host wall-clock seconds (CLOCK_REALTIME). All cross-process stamps use this clock.
inline double nowSec()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

/// Monotonic seconds, for timeouts and rate control inside one process.
inline double monoSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// Maps a sensor clock (e.g. Mid-360 time since boot) to host time.
/// offset = min over a sliding window of (host_receive - sensor_stamp); the minimum removes
/// transport/queueing delay, which is always positive.
class ClockMapper
{
public:
  explicit ClockMapper(double window_s = 10.0)
  : window_s_(window_s) {}

  void observe(double sensor_t, double host_t)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    const double off = host_t - sensor_t;
    while (!samples_.empty() && samples_.back().second >= off) {samples_.pop_back();}
    samples_.emplace_back(host_t, off);
    while (!samples_.empty() && samples_.front().first < host_t - window_s_) {
      samples_.pop_front();
    }
    if (!has_offset_) {
      has_offset_ = true;
    } else if (std::abs(samples_.front().second - offset_) > jump_threshold_s_) {
      ++jumps_;
    }
    offset_ = samples_.front().second;
  }

  bool valid() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return has_offset_;
  }
  double toHost(double sensor_t) const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return sensor_t + offset_;
  }
  double offset() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return offset_;
  }
  /// Number of times the offset changed by more than the jump threshold (clock reset/drift).
  int jumps() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return jumps_;
  }

private:
  double window_s_;
  double jump_threshold_s_{0.05};
  mutable std::mutex mutex_;
  std::deque<std::pair<double, double>> samples_;  // (host_t, offset), offsets increasing
  double offset_{0.0};
  bool has_offset_{false};
  int jumps_{0};
};

/// Fixed-rate loop helper.
class Rate
{
public:
  explicit Rate(double hz)
  : period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / hz))),
    next_(std::chrono::steady_clock::now() + period_) {}
  void sleep();

private:
  std::chrono::steady_clock::duration period_;
  std::chrono::steady_clock::time_point next_;
};

}  // namespace g1d
