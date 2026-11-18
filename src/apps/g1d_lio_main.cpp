// g1d_lio: Mid-360S -> FAST-LIO2 -> rt/nav/lio_odom, rt/nav/registered_cloud, rt/nav/lio_health
//          (+ rt/nav/livox_cloud in the LiDAR frame when debug.publish_lidar_cloud is set).
// registered_cloud carries a float32 "dynamic" field (1 = labelled moving by M-detector when
// lio.dynamic.enabled). Subscribes rt/nav/map_delete (JSON {"boxes": [[x0,x1,y0,y1,z0,z1], ...]}
// in odom_lio) to remove late-confirmed moving objects from the estimator map.
//
// lidar.source = livox_sdk2   : direct Livox-SDK2 connection (build with G1D_WITH_LIVOX=ON)
// lidar.source = unitree_topic: rt/utlidar/cloud_livox_mid360 + rt/utlidar/imu_livox_mid360
//                               (only if Unitree's own lidar service owns the sensor)

#include <atomic>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <thread>

#include <nlohmann/json.hpp>
#include <unitree/idl/ros2/Imu_.hpp>

#include "dds_util.hpp"
#include "g1d/common/config.hpp"
#include "g1d/common/time.hpp"
#include "g1d/lio/lio_estimator.hpp"
#ifdef G1D_HAVE_LIVOX
#include "livox_sdk_source.hpp"
#endif

using namespace g1d;

namespace
{
std::atomic<bool> g_run{true};
void onSignal(int) {g_run = false;}

/// Converts a Unitree-published PointCloud2 (Livox driver layout) to a LidarFrame. Per-point
/// time is taken from a "timestamp" (float64, s or ns) / "time" (float32, s) / "offset_time"
/// (uint32, ns) field when present; otherwise all points get offset 0 (no deskew).
bool cloudToFrame(const bus::PointCloud2 & m, lio::LidarFrame & f)
{
  int ox = -1, oy = -1, oz = -1, oi = -1, ot = -1, otag = -1;
  uint8_t ttype = 0;
  for (const auto & fld : m.fields()) {
    const auto & n = fld.name();
    if (n == "x") {ox = fld.offset();}
    if (n == "y") {oy = fld.offset();}
    if (n == "z") {oz = fld.offset();}
    if (n == "intensity" || n == "reflectivity") {oi = fld.offset();}
    if (n == "tag") {otag = fld.offset();}
    if (n == "timestamp" || n == "time" || n == "offset_time" || n == "t") {
      ot = fld.offset();
      ttype = fld.datatype();
    }
  }
  if (ox < 0 || oy < 0 || oz < 0 || m.point_step() == 0) {return false;}
  const size_t n = m.data().size() / m.point_step();
  const uint8_t * d = m.data().data();
  f = lio::LidarFrame();
  f.t_begin = bus::toSec(m.header().stamp());
  f.points.reserve(n);
  double t0 = -1.0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * p = d + i * m.point_step();
    lio::FramePoint q;
    std::memcpy(&q.x, p + ox, 4);
    std::memcpy(&q.y, p + oy, 4);
    std::memcpy(&q.z, p + oz, 4);
    if (oi >= 0) {float v; std::memcpy(&v, p + oi, 4); q.intensity = v;}
    if (otag >= 0) {q.tag = p[otag];}
    if (ot >= 0) {
      double t = 0.0;
      if (ttype == 8) {double v; std::memcpy(&v, p + ot, 8); t = v > 1e12 ? v * 1e-9 : v;}  // FLOAT64
      if (ttype == 7) {float v; std::memcpy(&v, p + ot, 4); t = v;}                          // FLOAT32
      if (ttype == 6) {uint32_t v; std::memcpy(&v, p + ot, 4); t = v * 1e-9;}                // UINT32 ns
      if (t0 < 0.0) {t0 = t;}
      q.offset_ms = static_cast<float>((t - t0) * 1e3);
    }
    f.points.push_back(q);
  }
  if (ttype == 8 && t0 > 1e3) {f.t_begin = t0;}  // absolute point timestamps
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string config_path = argc > 1 ? argv[1] : "config/g1d_nav.yaml";
  const YAML::Node cfg = loadYamlFile(config_path);
  const std::string net_if = get(cfg, "system.net_if", std::string("eth0"));
  const int domain = get(cfg, "system.dds_domain", 0);
  const std::string source = get(cfg, "lidar.source", std::string("livox_sdk2"));
  const double frame_ms = get(cfg, "lidar.frame_ms", 100.0);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  unitree::robot::ChannelFactory::Instance()->Init(domain, net_if);
  auto odom_pub = bus::advertise<bus::Odometry>("rt/nav/lio_odom");
  auto cloud_pub = bus::advertise<bus::PointCloud2>("rt/nav/registered_cloud");
  auto health_pub = bus::advertise<bus::String>("rt/nav/lio_health");

  // Optional raw LiDAR-frame cloud for calibration (tools/calib_floor.py), decimated, <= 5 Hz.
  const bool publish_lidar_cloud = get(cfg, "debug.publish_lidar_cloud", false);
  std::shared_ptr<bus::Publisher<bus::PointCloud2>> lidar_cloud_pub;
  if (publish_lidar_cloud) {lidar_cloud_pub = bus::advertise<bus::PointCloud2>("rt/nav/livox_cloud");}
  std::atomic<double> last_lidar_cloud{0.0};
  auto maybe_publish_lidar = [&](const lio::LidarFrame & f) {
      if (!lidar_cloud_pub || monoSec() - last_lidar_cloud < 0.2) {return;}
      last_lidar_cloud = monoSec();
      const size_t stride = std::max<size_t>(1, f.points.size() / 20000);
      std::vector<Eigen::Vector3f> pts;
      pts.reserve(f.points.size() / stride + 1);
      for (size_t i = 0; i < f.points.size(); i += stride) {
        pts.emplace_back(f.points[i].x, f.points[i].y, f.points[i].z);
      }
      lidar_cloud_pub->Write(bus::makeCloudXYZ(pts, "livox", nowSec()));
    };

  lio::LioEstimator est(lio::LioConfig::fromYaml(cfg));
  bus::Mailbox<std::string> delete_box(16);
  auto delete_sub = bus::subscribe<bus::String>("rt/nav/map_delete",
      [&](const bus::String & m) {delete_box.put(m.data());}, 16);
  long map_deleted = 0;
  std::mutex agg_mutex;
  lio::FrameAggregator agg(frame_ms);
  std::atomic<long> packets{0}, imus{0};

  auto on_imu = [&](double t, const double g[3], const double a[3]) {
      est.observeClock(t, nowSec());
      est.pushImu(t, g, a);
      ++imus;
    };
  auto on_packet = [&](const lio::LivoxPacket & pkt) {
      std::vector<lio::LidarFrame> frames;
      {
        std::lock_guard<std::mutex> lk(agg_mutex);
        agg.push(pkt, frames);
      }
      for (const auto & f : frames) {
        maybe_publish_lidar(f);
        est.pushFrame(f);
      }
      ++packets;
    };

  std::vector<std::shared_ptr<void>> subs;
#ifdef G1D_HAVE_LIVOX
  std::unique_ptr<apps::LivoxSdkSource> livox;
#endif
  if (source == "livox_sdk2") {
#ifdef G1D_HAVE_LIVOX
    livox = std::make_unique<apps::LivoxSdkSource>(on_packet, on_imu);
    const std::string sdk_cfg = get(cfg, "lidar.sdk_config", std::string("config/mid360s_config.json"));
    const std::string host_ip = get(cfg, "lidar.host_ip", std::string("192.168.123.164"));
    if (!livox->start(sdk_cfg, host_ip)) {return 2;}
#else
    std::fprintf(stderr, "[g1d_lio] built without Livox-SDK2 (G1D_WITH_LIVOX=OFF)\n");
    return 2;
#endif
  } else if (source == "unitree_topic") {
    const std::string cloud_topic = get(cfg, "lidar.unitree_cloud_topic", std::string("rt/utlidar/cloud_livox_mid360"));
    const std::string imu_topic = get(cfg, "lidar.unitree_imu_topic", std::string("rt/utlidar/imu_livox_mid360"));
    subs.push_back(bus::subscribe<bus::PointCloud2>(cloud_topic, [&](const bus::PointCloud2 & m) {
        lio::LidarFrame f;
        if (cloudToFrame(m, f)) {
          maybe_publish_lidar(f);
          est.pushFrame(f);
          ++packets;
        }
      }, 4));
    subs.push_back(bus::subscribe<::sensor_msgs::msg::dds_::Imu_>(imu_topic,
      [&](const ::sensor_msgs::msg::dds_::Imu_ & m) {
        const double g[3] = {m.angular_velocity().x(), m.angular_velocity().y(), m.angular_velocity().z()};
        const double a[3] = {m.linear_acceleration().x(), m.linear_acceleration().y(), m.linear_acceleration().z()};
        on_imu(bus::toSec(m.header().stamp()), g, a);
      }, 64));
  } else {
    std::fprintf(stderr, "[g1d_lio] unknown lidar.source '%s'\n", source.c_str());
    return 2;
  }

  std::fprintf(stderr, "[g1d_lio] running, source=%s\n", source.c_str());
  double last_report = monoSec();
  while (g_run) {
    for (const auto & s : delete_box.drain()) {
      try {
        std::vector<fast_lio_api::Box> boxes;
        for (const auto & b : nlohmann::json::parse(s).at("boxes")) {
          const auto v = b.get<std::vector<float>>();
          if (v.size() == 6) {boxes.push_back({v[0], v[1], v[2], v[3], v[4], v[5]});}
        }
        map_deleted += est.deleteMapBoxes(boxes);
      } catch (const std::exception & e) {
        std::fprintf(stderr, "[g1d_lio] bad map_delete message: %s\n", e.what());
      }
    }
    lio::LioOutput out;
    bool any = false;
    while (est.process(&out)) {
      any = true;
      const double t = out.stamp_host;
      odom_pub->Write(bus::makeOdometry(out.T_world_imu, out.velocity_world, Eigen::Vector3d::Zero(),
        "odom_lio", "livox_imu", t));
      cloud_pub->Write(bus::makeCloudXYZL(out.cloud_world, out.cloud_dynamic, "odom_lio", t));
      nlohmann::json h;
      h["stamp"] = t;
      h["ok"] = out.health.ok;
      h["degenerate"] = out.health.degenerate;
      h["errors"] = out.health.errors;
      h["effective_points"] = out.stats.effective_points;
      h["input_points"] = out.stats.input_points;
      h["residual"] = out.stats.residual_mean;
      h["normal_eig_min"] = out.stats.normal_eig_min;
      h["process_ms"] = out.stats.process_time_s * 1e3;
      h["clock_offset"] = est.clock().offset();
      h["clock_jumps"] = est.clock().jumps();
      if (est.dynamicEnabled()) {
        h["dynamic_points"] = out.stats.dynamic_points;
        h["dynamic_raw"] = out.dynamic_stats.raw_dynamic;
        h["map_skipped"] = out.stats.map_skipped_points;
        h["label_ms"] = out.dynamic_stats.time_ms;
        h["map_deleted"] = map_deleted;
      }
      health_pub->Write(bus::makeString(h.dump()));
    }
    if (monoSec() - last_report > 5.0) {
      last_report = monoSec();
      std::fprintf(stderr, "[g1d_lio] packets=%ld imu=%ld initialized=%d\n", packets.load(),
        imus.load(), est.initialized());
    }
    if (!any) {std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  }
  return 0;
}
