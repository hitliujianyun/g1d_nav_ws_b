#include "g1d/lio/livox_frame.hpp"

namespace g1d::lio
{

FrameAggregator::FrameAggregator(double frame_ms)
: frame_ns_(static_cast<uint64_t>(frame_ms * 1e6)) {}

void FrameAggregator::reset()
{
  active_ = false;
  current_ = LidarFrame();
}

void FrameAggregator::startFrame(uint64_t t_ns)
{
  active_ = true;
  frame_start_ns_ = t_ns;
  current_ = LidarFrame();
  current_.t_begin = static_cast<double>(t_ns) * 1e-9;
  current_.points.reserve(24000);
}

void FrameAggregator::push(const LivoxPacket & packet, std::vector<LidarFrame> & out)
{
  const size_t n = packet.points.size();
  if (n == 0) {return;}
  const uint64_t point_interval_ns = static_cast<uint64_t>(packet.time_interval) * 100ULL / n;

  // A timestamp going backwards (sensor restart / clock change) drops the partial frame.
  if (active_ && packet.timestamp_ns + frame_ns_ < last_point_ns_) {
    reset();
  }

  for (size_t i = 0; i < n; ++i) {
    const uint64_t t = packet.timestamp_ns + i * point_interval_ns;
    if (!active_) {
      startFrame(t);
    } else if (t >= frame_start_ns_ + frame_ns_) {
      current_.t_end = static_cast<double>(last_point_ns_) * 1e-9;
      out.push_back(std::move(current_));
      // Contiguous frames: the next frame starts exactly where the previous ended.
      uint64_t next = frame_start_ns_ + frame_ns_;
      while (t >= next + frame_ns_) {next += frame_ns_;}
      startFrame(next);
    }
    const auto & p = packet.points[i];
    FramePoint fp;
    fp.x = static_cast<float>(p.x_mm) * 1e-3f;
    fp.y = static_cast<float>(p.y_mm) * 1e-3f;
    fp.z = static_cast<float>(p.z_mm) * 1e-3f;
    fp.intensity = static_cast<float>(p.reflectivity);
    fp.tag = p.tag;
    fp.offset_ms = static_cast<float>(static_cast<double>(t - frame_start_ns_) * 1e-6);
    current_.points.push_back(fp);
    last_point_ns_ = t;
  }
}

}  // namespace g1d::lio
