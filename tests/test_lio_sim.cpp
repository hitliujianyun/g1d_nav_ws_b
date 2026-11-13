// FAST-LIO2 accuracy on simulated Mid-360 + IMU data with known ground truth.

#include <cmath>
#include <cstdio>

#include "doctest.h"
#include "g1d/common/geometry.hpp"
#include "g1d/lio/lio_estimator.hpp"
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
