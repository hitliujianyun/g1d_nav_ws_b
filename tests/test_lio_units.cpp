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

// ------------------------------------------------------------------ M-detector (de-ROSed port)
#include <random>

#include "g1d/lio/dynamic_detector.hpp"

namespace
{
/// Static sensor at the origin facing +x: wall at x = 6 m, floor at z = -1.2 m, and a 0.4 x 0.4 x
/// 1.7 m box crossing at x = 2.5 m with speed `v` (m/s). Rays are uniform in the Mid-360 FOV.
fast_lio_api::Cloud::Ptr crossingScan(double t, double v, int n, std::mt19937 & rng,
  std::vector<uint8_t> * on_box, const Eigen::Isometry3d & T_world_sensor)
{
  std::uniform_real_distribution<double> az(-M_PI, M_PI), el(-7.0 * M_PI / 180, 52.0 * M_PI / 180);
  const double by = -2.0 + v * t;
  const Eigen::Vector3d o = T_world_sensor.translation();
  fast_lio_api::Cloud::Ptr c(new fast_lio_api::Cloud());
  if (on_box) {on_box->clear();}
  for (int i = 0; i < n; ++i) {
    const double a = az(rng), e = el(rng);
    const Eigen::Vector3d ds(std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e));
    const Eigen::Vector3d d = T_world_sensor.linear() * ds;
    double r = 1e9;
    bool box = false;
    if (d.x() > 1e-6) {r = (6.0 - o.x()) / d.x();}                       // wall
    if (d.z() < -1e-6) {r = std::min(r, (-1.2 - o.z()) / d.z());}        // floor
    if (d.z() > 1e-6) {r = std::min(r, (1.6 - o.z()) / d.z());}          // ceiling
    if (d.x() > 1e-6) {                                                  // box front face
      const double rb = (2.5 - o.x()) / d.x();
      const Eigen::Vector3d q = o + rb * d;
      if (std::abs(q.y() - by) < 0.2 && q.z() > -1.2 && q.z() < 0.5 && rb < r) {r = rb; box = true;}
    }
    if (r > 40.0) {continue;}
    fast_lio_api::Point p;
    p.x = static_cast<float>(r * ds.x());
    p.y = static_cast<float>(r * ds.y());
    p.z = static_cast<float>(r * ds.z());
    c->push_back(p);
    if (on_box) {on_box->push_back(box);}
  }
  return c;
}
}  // namespace

TEST_CASE("DynamicDetector (M-detector) labels a crossing box and nothing static")
{
  struct Variant {
    int n; double vx; double wz; const char * name; double still_s = 0.0; double roll = 0.0;
  };
  for (const Variant var : {Variant{20000, 0, 0, "static"}, Variant{5000, 0, 0, "static"},
      Variant{20000, 0.4, 0, "translating"}, Variant{20000, 0, 0.4, "rotating"},
      Variant{20000, 0, 0, "starts_still", 2.0}, Variant{20000, 0, 0, "inverted", 0.0, M_PI}})
  {
  const int n = var.n;
  DynamicDetectorParams p;
  p.enabled = true;
  DynamicDetector det(p);
  std::mt19937 rng(3);
  long box_pts = 0, box_dyn = 0, bg_pts = 0, bg_dyn = 0, raw = 0;
  std::vector<uint8_t> on_box, dyn;
  const int frames = 40 + static_cast<int>(var.still_s * 10);
  for (int k = 0; k < frames; ++k) {
    const double t = 0.1 * k;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation().x() = var.vx * t;
    T.linear() = (Eigen::AngleAxisd(var.wz * t, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(var.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    auto scan = crossingScan(std::max(0.0, t - var.still_s), 1.0, n, rng, &on_box, T);
    det.label(*scan, T.linear(), T.translation(), t, dyn);
    REQUIRE(dyn.size() == scan->size());
    raw += det.lastStats().raw_dynamic;
    if (t < 2.0 + var.still_s) {continue;}
    for (size_t i = 0; i < dyn.size(); ++i) {
      if (on_box[i]) {++box_pts; box_dyn += dyn[i];} else {++bg_pts; bg_dyn += dyn[i];}
    }
  }
  std::printf("[m_detector] %-11s n=%d crossing box recall=%.2f (%ld/%ld) false_pos=%.4f raw=%ld "
    "last=%.1fms\n", var.name, n, double(box_dyn) / box_pts, box_dyn, box_pts,
    double(bg_dyn) / bg_pts, raw, det.lastStats().time_ms);
  if (n == 20000 && var.still_s == 0.0) {
    CHECK(double(box_dyn) / box_pts > 0.8);
    CHECK(double(bg_dyn) / bg_pts < 0.005);
  }
  }
}
