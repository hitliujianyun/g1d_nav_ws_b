#include <cmath>

#include "doctest.h"
#include "g1d/lio/livox_frame.hpp"
#include "g1d/lio/preprocess.hpp"

using namespace g1d::lio;

namespace
{
LivoxPacket makePacket(uint64_t t_ns, int n, uint32_t span_01us, int start_index = 0)
{
  LivoxPacket p;
  p.timestamp_ns = t_ns;
  p.time_interval = span_01us;
  for (int i = 0; i < n; ++i) {
    LivoxPacket::Point q;
    q.x_mm = 1000 + start_index + i;
    q.y_mm = 0;
    q.z_mm = 100;
    q.reflectivity = 10;
    q.tag = 0;
    p.points.push_back(q);
  }
  return p;
}
}  // namespace

TEST_CASE("FrameAggregator assigns per-point time and splits 100 ms frames")
{
  FrameAggregator agg(100.0);
  std::vector<LidarFrame> frames;
  // 96 points per packet, 480 us per packet (200k pts/s), 0.1 us units -> 4800
  const uint64_t t0 = 5'000'000'000ULL;
  int k = 0;
  for (uint64_t t = t0; t < t0 + 250'000'000ULL; t += 480'000ULL, ++k) {
    agg.push(makePacket(t, 96, 4800, k * 96), frames);
  }
  REQUIRE(frames.size() == 2);
  CHECK(frames[0].t_begin == doctest::Approx(5.0));
  CHECK(frames[1].t_begin == doctest::Approx(5.1));
  // 100 ms at 200k pts/s = 20000 points
  CHECK(std::abs(static_cast<int>(frames[0].points.size()) - 20000) <= 96);
  // offsets increase monotonically from 0 and stay below 100 ms
  float last = -1.0f;
  for (const auto & p : frames[0].points) {
    CHECK(p.offset_ms >= last);
    last = p.offset_ms;
  }
  CHECK(frames[0].points.front().offset_ms == doctest::Approx(0.0f));
  CHECK(last < 100.0f);
  // the 2nd point of the first packet is 5 us later
  CHECK(frames[0].points[1].offset_ms == doctest::Approx(0.005f).epsilon(1e-3));
}

TEST_CASE("FrameAggregator drops partial frame on time reset")
{
  FrameAggregator agg(100.0);
  std::vector<LidarFrame> frames;
  agg.push(makePacket(10'000'000'000ULL, 96, 4800), frames);
  agg.push(makePacket(1'000'000'000ULL, 96, 4800), frames);  // clock reset
  agg.push(makePacket(1'150'000'000ULL, 96, 4800), frames);
  REQUIRE(frames.size() == 1);
  CHECK(frames[0].t_begin == doctest::Approx(1.0));
}

TEST_CASE("preprocessFrame applies tag, blind, filter and self filter")
{
  LidarFrame f;
  f.t_begin = 1.0;
  auto add = [&](float x, float y, float z, uint8_t tag, float off) {
      FramePoint p;
      p.x = x; p.y = y; p.z = z; p.tag = tag; p.offset_ms = off; p.intensity = 5;
      f.points.push_back(p);
    };
  add(5, 0, 0, 0x00, 0.0f);   // keep
  add(5, 0, 0, 0x00, 0.1f);   // duplicate -> drop
  add(0.2f, 0, 0, 0x00, 0.2f);   // blind -> drop
  add(3, 1, 0, 0x20, 0.3f);   // bad tag -> drop
  add(0, 0, -1.0f, 0x10, 0.4f);   // inside self box -> drop
  add(2, 2, 1, 0x10, 0.5f);   // keep
  PreprocessParams pp;
  pp.blind = 0.5;
  SelfFilter sf(Eigen::Isometry3d::Identity(), {Box{-0.3, 0.3, -0.3, 0.3, -1.2, -0.5}});
  PreprocessStats st;
  auto c = preprocessFrame(f, pp, sf, &st);
  REQUIRE(c->size() == 2);
  CHECK(c->points[0].x == doctest::Approx(5.0f));
  CHECK(c->points[1].curvature == doctest::Approx(0.5f));
  CHECK(st.tag_rejected == 1);
  CHECK(st.blind_rejected == 1);
  CHECK(st.self_rejected == 1);
}
