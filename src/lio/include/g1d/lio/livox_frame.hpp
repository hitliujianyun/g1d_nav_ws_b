#pragma once

#include <cstdint>
#include <vector>

namespace g1d::lio
{

/// One Livox SDK2 point packet (kLivoxLidarCartesianCoordinateHighData), already decoded.
struct LivoxPacket
{
  uint64_t timestamp_ns{0};    // packet timestamp (first point), LiDAR clock
  uint32_t time_interval{0};   // packet time span in units of 0.1 us (SDK2 header field)
  struct Point
  {
    int32_t x_mm{0};
    int32_t y_mm{0};
    int32_t z_mm{0};
    uint8_t reflectivity{0};
    uint8_t tag{0};
  };
  std::vector<Point> points;
};

struct FramePoint
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
  float offset_ms{0.0f};  // time since frame start
  uint8_t tag{0};
};

/// One aggregated scan (default 100 ms). Times are LiDAR clock seconds.
struct LidarFrame
{
  double t_begin{0.0};
  double t_end{0.0};
  std::vector<FramePoint> points;
};

/// Groups packets into fixed-duration frames and assigns per-point timestamps.
/// Per-point time follows livox_ros_driver2: t_i = packet_ts + i * (time_interval * 100 / N) ns.
class FrameAggregator
{
public:
  explicit FrameAggregator(double frame_ms = 100.0);

  /// Adds a packet; completed frames (if any) are appended to `out`.
  void push(const LivoxPacket & packet, std::vector<LidarFrame> & out);
  void reset();

private:
  void startFrame(uint64_t t_ns);

  uint64_t frame_ns_;
  bool active_{false};
  uint64_t frame_start_ns_{0};
  uint64_t last_point_ns_{0};
  LidarFrame current_;
};

}  // namespace g1d::lio
