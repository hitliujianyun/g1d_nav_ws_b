// FAST-LIO2 accuracy on simulated Mid-360 + IMU data with known ground truth.

#include <cmath>
#include <cstdio>

#include "doctest.h"
#include "g1d/common/geometry.hpp"
#include "g1d/lio/lio_estimator.hpp"
#include "g1d/sim/nav_sim.hpp"
#include "g1d/sim/robot.hpp"
#include "g1d/sim/sensors.hpp"
#include "g1d/sim/world.hpp"

using namespace g1d;

namespace
{

struct RunResult
{
  int estimates{0};
  double max_pos_err{0.0};
  double final_pos_err{0.0};
  double max_yaw_err_deg{0.0};
  double path_length{0.0};
  bool health_ok_at_end{false};
  double mean_process_ms{0.0};
};

RunResult runScenario(const Eigen::Isometry3d & T_base_imu, double duration_s)
{
  const sim::World world = sim::World::makeTestRoom();
  sim::EllipseTrajectory traj;
  const Eigen::Vector3d lidar_in_imu(-0.011, -0.02329, 0.04412);
  Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
  T_imu_lidar.translation() = lidar_in_imu;

  auto T_world_imu = [&](double t) {return traj.basePose(t) * T_base_imu;};
  auto T_world_lidar = [&](double t) {return T_world_imu(t) * T_imu_lidar;};

  lio::LioConfig cfg;
  cfg.fastlio.extrinsic_T = {lidar_in_imu.x(), lidar_in_imu.y(), lidar_in_imu.z()};
  cfg.preprocess.blind = 0.5;
  cfg.preprocess.point_filter_num = 2;
  cfg.health.max_process_time_s = 2.0;  // debug builds on CI machines are slow
  lio::LioEstimator est(cfg);

  sim::Mid360Params mp;
  mp.points_per_second = 100000;  // half rate keeps the test fast; geometry is still rich
  sim::Mid360Sim lidar(world, mp);
  const auto imu = sim::simulateImu(T_world_imu, 0.0, duration_s + 0.2);
  lio::FrameAggregator agg(100.0);

  RunResult res;
  const Eigen::Isometry3d T0_inv = T_world_imu(0.0).inverse();
  size_t imu_idx = 0;
  double proc_ms_sum = 0.0;
  Eigen::Vector3d last_true = T_world_imu(0.0).translation();
  std::vector<lio::LidarFrame> frames;
  for (double t = 0.0; t < duration_s; t += 0.1) {
    for (const auto & pkt : lidar.scan(T_world_lidar, t, t + 0.1)) {agg.push(pkt, frames);}
    while (imu_idx < imu.size() && imu[imu_idx].t < t + 0.12) {
      const auto & m = imu[imu_idx++];
      const double g[3] = {m.gyr.x(), m.gyr.y(), m.gyr.z()};
      const double a[3] = {m.acc.x(), m.acc.y(), m.acc.z()};
      est.pushImu(m.t, g, a);
    }
    for (const auto & f : frames) {est.pushFrame(f);}
    frames.clear();

    lio::LioOutput out;
    while (est.process(&out)) {
      if (!out.stats.ekf_inited) {continue;}
      ++res.estimates;
      proc_ms_sum += out.stats.process_time_s * 1e3;
      const Eigen::Isometry3d T_true = T0_inv * T_world_imu(out.stamp_sensor);
      const double pos_err = (T_true.translation() - out.T_world_imu.translation()).norm();
      const double yaw_err = std::abs(wrapAngle(
          yawOf(T_true.linear()) - yawOf(out.T_world_imu.linear()))) * 180.0 / M_PI;
      res.max_pos_err = std::max(res.max_pos_err, pos_err);
      res.max_yaw_err_deg = std::max(res.max_yaw_err_deg, yaw_err);
      res.final_pos_err = pos_err;
      res.health_ok_at_end = out.health.ok;
    }
    const Eigen::Vector3d cur = T_world_imu(t).translation();
    res.path_length += (cur - last_true).norm();
    last_true = cur;
  }
  res.mean_process_ms = res.estimates ? proc_ms_sum / res.estimates : 0.0;
  std::printf(
    "[lio_sim] estimates=%d path=%.1fm max_pos_err=%.3fm final=%.3fm max_yaw_err=%.2fdeg "
    "mean_proc=%.1fms\n", res.estimates, res.path_length, res.max_pos_err, res.final_pos_err,
    res.max_yaw_err_deg, res.mean_process_ms);
  return res;
}

}  // namespace

TEST_CASE("FAST-LIO2 tracks a loop with the Mid-360 mounted upright on the head")
{
  // head height ~1.35 m, slight forward pitch
  const Eigen::Isometry3d T_base_imu = makeIsometry(0.05, 0.0, 1.35, 0.0, 0.04, 0.0);
  const RunResult r = runScenario(T_base_imu, 20.0);
  CHECK(r.estimates > 150);
  CHECK(r.path_length > 5.0);
  CHECK(r.max_pos_err < 0.15);
  CHECK(r.final_pos_err < 0.10);
  CHECK(r.max_yaw_err_deg < 2.0);
  CHECK(r.health_ok_at_end);
}

namespace
{
struct DynamicRun
{
  RunResult lio;
  long ped_points{0};        // registered returns on the pedestrian (after warm-up)
  long ped_dynamic{0};       // ... of which labelled moving
  long static_points{0};     // returns away from the pedestrian
  long static_dynamic{0};    // ... of which labelled moving (false positives)
  long map_in_corridor{0};   // ikd-Tree map points inside the pedestrian's corridor at the end
  long map_in_corridor_late{0};  // ... in the part first walked after the detector warm-up
  long raw_dynamic{0};       // M-detector labels before / after voxel refinement (all frames)
  long refined_dynamic{0};
  double mean_label_ms{0.0};
};

/// Robot loops as above (inverted Mid-360); a pedestrian walks up and down x = 8 m at 1 m/s.
DynamicRun runPedestrian(bool detector, double duration_s)
{
  sim::World world = sim::World::makeTestRoom();
  world.addBox({7.8, -3.5, 0.0}, {8.2, -3.1, 1.7}, "ped");
  auto ped_y = [](double t) {
      const double s = std::fmod(t, 12.0);       // -3.3 -> 2.7 -> -3.3
      return s < 6.0 ? -3.3 + s : 2.7 - (s - 6.0);
    };
  const Eigen::Isometry3d T_base_imu = makeIsometry(0.05, 0.0, 1.35, M_PI, -0.04, 0.0);
  sim::EllipseTrajectory traj;
  const Eigen::Vector3d lidar_in_imu(-0.011, -0.02329, 0.04412);
  Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
  T_imu_lidar.translation() = lidar_in_imu;
  auto T_world_imu = [&](double t) {return traj.basePose(t) * T_base_imu;};
  auto T_world_lidar = [&](double t) {return T_world_imu(t) * T_imu_lidar;};

  lio::LioConfig cfg;
  cfg.fastlio.extrinsic_T = {lidar_in_imu.x(), lidar_in_imu.y(), lidar_in_imu.z()};
  cfg.preprocess.blind = 0.5;
  cfg.preprocess.point_filter_num = 2;
  cfg.health.max_process_time_s = 2.0;
  cfg.dynamic.enabled = detector;
  if (std::getenv("G1D_DENSE")) {cfg.preprocess.point_filter_num = 1;}
  lio::LioEstimator est(cfg);

  sim::Mid360Params mp;
  mp.points_per_second = std::getenv("G1D_DENSE") ? 200000 : 100000;
  sim::Mid360Sim lidar(world, mp);
  const auto imu = sim::simulateImu(T_world_imu, 0.0, duration_s + 0.2);
  lio::FrameAggregator agg(100.0);

  DynamicRun res;
  const Eigen::Isometry3d T0 = T_world_imu(0.0);
  const Eigen::Isometry3d T0_inv = T0.inverse();
  size_t imu_idx = 0;
  double label_ms = 0.0;
  int labelled = 0;
  std::vector<lio::LidarFrame> frames;
  for (double t = 0.0; t < duration_s; t += 0.1) {
    sim::moveBox(world, "ped", 8.0, ped_y(t + 0.05));
    for (const auto & pkt : lidar.scan(T_world_lidar, t, t + 0.1)) {agg.push(pkt, frames);}
    while (imu_idx < imu.size() && imu[imu_idx].t < t + 0.12) {
      const auto & m = imu[imu_idx++];
      const double g[3] = {m.gyr.x(), m.gyr.y(), m.gyr.z()};
      const double a[3] = {m.acc.x(), m.acc.y(), m.acc.z()};
      est.pushImu(m.t, g, a);
    }
    for (const auto & f : frames) {est.pushFrame(f);}
    frames.clear();

    lio::LioOutput out;
    while (est.process(&out)) {
      if (!out.stats.ekf_inited) {continue;}
      ++res.lio.estimates;
      const Eigen::Isometry3d T_true = T0_inv * T_world_imu(out.stamp_sensor);
      const double pos_err = (T_true.translation() - out.T_world_imu.translation()).norm();
      res.lio.max_pos_err = std::max(res.lio.max_pos_err, pos_err);
      res.lio.final_pos_err = pos_err;
      res.lio.health_ok_at_end = out.health.ok;
      if (detector) {
        label_ms += out.dynamic_stats.time_ms;
        ++labelled;
        res.raw_dynamic += out.dynamic_stats.raw_dynamic;
        res.refined_dynamic += out.dynamic_stats.dynamic;
      }
      if (out.stamp_sensor < 4.0) {continue;}   // depth-map history warm-up
      const double py = ped_y(out.stamp_sensor - 0.05);
      auto classify = [&](const std::vector<Eigen::Vector3f> & pts, bool dyn) {
          for (const auto & p : pts) {
            const Eigen::Vector3d w = T0 * p.cast<double>();
            const double dx = std::abs(w.x() - 8.0), dy = std::abs(w.y() - py);
            if (dx < 0.3 && dy < 0.3 && w.z() > 0.1 && w.z() < 1.75) {
              ++res.ped_points;
              res.ped_dynamic += dyn;
            } else if (dx > 0.6 || dy > 0.6) {
              ++res.static_points;
              res.static_dynamic += dyn;
            }
          }
        };
      classify(out.cloud_world, false);
      classify(out.cloud_dynamic, true);
    }
  }
  for (const auto & p : est.mapPoints()) {
    const Eigen::Vector3d w = T0 * p.cast<double>();
    if (std::abs(w.x() - 8.0) < 0.35 && w.y() > -3.6 && w.y() < 3.0 && w.z() > 0.15 &&
      w.z() < 1.7)
    {
      ++res.map_in_corridor;
      // The pedestrian first reaches y > -0.7 after 2.6 s, once the detector has history.
      if (w.y() > -0.7) {++res.map_in_corridor_late;}
    }
  }
  res.mean_label_ms = labelled ? label_ms / labelled : 0.0;
  std::printf(
    "[lio_sim] pedestrian detector=%d recall=%.2f (%ld/%ld) false_pos=%.4f (%ld/%ld) "
    "map_in_corridor=%ld (late %ld) max_pos_err=%.3fm label=%.1fms raw=%ld refined=%ld\n",
    detector, res.ped_points ? double(res.ped_dynamic) / res.ped_points : 0.0, res.ped_dynamic,
    res.ped_points, res.static_points ? double(res.static_dynamic) / res.static_points : 0.0,
    res.static_dynamic, res.static_points, res.map_in_corridor, res.map_in_corridor_late,
    res.lio.max_pos_err,
    res.mean_label_ms, res.raw_dynamic, res.refined_dynamic);
  return res;
}
}  // namespace

TEST_CASE("DynamicDetector on simulated Mid-360 frames with true poses")
{
  // Detector alone (no LIO): recall by height / range for the inverted head mount.
  sim::World world = sim::World::makeTestRoom();
  world.addBox({7.8, -3.5, 0.0}, {8.2, -3.1, 1.7}, "ped");
  auto ped_y = [](double t) {
      const double s = std::fmod(t, 12.0);
      return s < 6.0 ? -3.3 + s : 2.7 - (s - 6.0);
    };
  const Eigen::Isometry3d T_base_lidar = makeIsometry(0.05, 0.0, 1.35, M_PI, -0.04, 0.0);
  sim::EllipseTrajectory traj;
  for (const int n : {5000, 20000}) {
    for (const bool moving : {false, true}) {
      lio::DynamicDetectorParams p;
      p.enabled = true;
      lio::DynamicDetector det(p);
      sim::Mid360Sim lidar(world);
      long ped = 0, ped_dyn = 0, bg = 0, bg_dyn = 0;
      long zb[3] = {0, 0, 0}, zd[3] = {0, 0, 0}, rb[3] = {0, 0, 0}, rd[3] = {0, 0, 0};
      for (double t = 0.0; t < 12.0; t += 0.1) {
        sim::moveBox(world, "ped", 8.0, ped_y(t));
        const Eigen::Isometry3d T = (moving ? traj.basePose(t) : traj.basePose(0.0)) * T_base_lidar;
        const auto pts_w = lidar.worldPoints(T, n);
        fast_lio_api::Cloud scan;
        const Eigen::Isometry3d Ti = T.inverse();
        for (const auto & q : pts_w) {
          const Eigen::Vector3d l = Ti * q.cast<double>();
          fast_lio_api::Point pt;
          pt.x = l.x(); pt.y = l.y(); pt.z = l.z();
          scan.push_back(pt);
        }
        std::vector<uint8_t> dyn;
        det.label(scan, T.linear(), T.translation(), t, dyn);
        if (t < 2.0) {continue;}
        for (size_t i = 0; i < pts_w.size(); ++i) {
          const double dx = std::abs(pts_w[i].x() - 8.0), dy = std::abs(pts_w[i].y() - ped_y(t));
          if (dx < 0.25 && dy < 0.25 && pts_w[i].z() > 0.1) {
            ++ped; ped_dyn += dyn[i];
            const int z = std::min(2, static_cast<int>(pts_w[i].z() / 0.6));
            const double r = (pts_w[i].cast<double>() - T.translation()).norm();
            const int ri = r < 3.0 ? 0 : (r < 5.0 ? 1 : 2);
            ++zb[z]; zd[z] += dyn[i]; ++rb[ri]; rd[ri] += dyn[i];
          } else if (dx > 0.6 || dy > 0.6) {++bg; bg_dyn += dyn[i];}
        }
      }
      auto f = [](long a, long b) {return b ? double(a) / b : 0.0;};
      std::printf("[lio_sim] truth-pose detector n=%d moving=%d recall=%.2f (%ld/%ld) fp=%.4f "
        "z[0-.6,.6-1.2,>1.2]=%.2f/%.2f/%.2f r[<3,3-5,>5]=%.2f/%.2f/%.2f (%ld/%ld/%ld)\n", n, moving,
        f(ped_dyn, ped), ped_dyn, ped, f(bg_dyn, bg), f(zd[0], zb[0]), f(zd[1], zb[1]),
        f(zd[2], zb[2]), f(rd[0], rb[0]), f(rd[1], rb[1]), f(rd[2], rb[2]), rb[0], rb[1], rb[2]);
      CHECK(f(ped_dyn, ped) > 0.75);
      CHECK(f(bg_dyn, bg) < 0.001);
    }
  }
}

TEST_CASE("M-detector keeps a walking pedestrian out of the FAST-LIO2 map")
{
  const DynamicRun off = runPedestrian(false, 20.0);
  const DynamicRun on = runPedestrian(true, 20.0);
  // Measured (half-rate scans): recall 0.67, no false positives, steady-state corridor points
  // 73 -> 24. At the real 200k pts/s (G1D_DENSE=1): recall 0.74, corridor 72 -> 12. Points
  // inserted before the detector has history are left to the offline DUFOMap cleaning.
  REQUIRE(on.ped_points > 200);
  CHECK(on.lio.max_pos_err < 0.15);
  CHECK(on.lio.health_ok_at_end);
  CHECK(double(on.ped_dynamic) / on.ped_points > 0.6);
  CHECK(double(on.static_dynamic) / on.static_points < 0.001);
  CHECK(off.map_in_corridor_late > 30);
  CHECK(on.map_in_corridor_late * 2 < off.map_in_corridor_late);
  CHECK(on.map_in_corridor < off.map_in_corridor);
}

TEST_CASE("FAST-LIO2 tracks a loop with the Mid-360 mounted inverted (as on G1)")
{
  const Eigen::Isometry3d T_base_imu = makeIsometry(0.05, 0.0, 1.35, M_PI, -0.04, 0.0);
  const RunResult r = runScenario(T_base_imu, 20.0);
  CHECK(r.estimates > 150);
  CHECK(r.max_pos_err < 0.15);
  CHECK(r.final_pos_err < 0.10);
  CHECK(r.max_yaw_err_deg < 2.0);
  CHECK(r.health_ok_at_end);
}
