#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include "g1d/common/geometry.hpp"

namespace g1d
{

/// Time-indexed history of planar poses (host time). Used to express NavGPT waypoints relative
/// to the pose at image capture time.
class PoseBuffer
{
public:
  explicit PoseBuffer(double horizon_s = 5.0)
  : horizon_s_(horizon_s) {}

  void add(double t, const Pose2D & pose)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!buf_.empty() && t <= buf_.back().first) {
      if (t == buf_.back().first) {buf_.back().second = pose;}
      return;
    }
    buf_.emplace_back(t, pose);
    while (!buf_.empty() && buf_.front().first < t - horizon_s_) {buf_.pop_front();}
  }

  /// Interpolated pose at time t. Extrapolation is not performed: queries newer than the last
  /// sample return the last sample if within `max_extrapolation_s`, otherwise nothing.
  std::optional<Pose2D> lookup(double t, double max_extrapolation_s = 0.2) const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (buf_.empty()) {return std::nullopt;}
    if (t >= buf_.back().first) {
      if (t - buf_.back().first <= max_extrapolation_s) {return buf_.back().second;}
      return std::nullopt;
    }
    if (t < buf_.front().first) {return std::nullopt;}
    for (size_t i = buf_.size() - 1; i > 0; --i) {
      const auto & a = buf_[i - 1];
      const auto & b = buf_[i];
      if (t >= a.first) {
        const double span = b.first - a.first;
        const double u = span > 0.0 ? (t - a.first) / span : 0.0;
        return interpolate(a.second, b.second, u);
      }
    }
    return buf_.front().second;
  }

  std::optional<std::pair<double, Pose2D>> latest() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (buf_.empty()) {return std::nullopt;}
    return buf_.back();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    buf_.clear();
  }

private:
  double horizon_s_;
  mutable std::mutex mutex_;
  std::deque<std::pair<double, Pose2D>> buf_;
};

}  // namespace g1d
