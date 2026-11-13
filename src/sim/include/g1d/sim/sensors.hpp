#pragma once

#include <cstdint>
#include <functional>
#include <random>
#include <vector>

#include <Eigen/Geometry>

#include "g1d/lio/livox_frame.hpp"
#include "g1d/sim/world.hpp"

namespace g1d::sim
{

/// Pose of a frame in the world as a function of time.
using PoseFn = std::function<Eigen::Isometry3d(double)>;

struct Mid360Params
{
  double points_per_second{200000.0};
  int points_per_packet{96};
  double fov_min_deg{-7.0};   // sensor frame elevation range
  double fov_max_deg{52.0};
  double max_range{40.0};
  double min_range{0.1};
  double range_noise{0.01};   // 1 sigma, m
  uint32_t seed{1};
};

/// Non-repetitive scanner emulation producing Livox SDK2-style packets with per-packet timing.
class Mid360Sim
{
public:
  Mid360Sim(const World & world, Mid360Params p = {});
  /// Generates all packets covering [t0, t1) for a LiDAR moving along `T_world_lidar`.
  std::vector<g1d::lio::LivoxPacket> scan(const PoseFn & T_world_lidar, double t0, double t1);

private:
  const World & world_;
  Mid360Params p_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  uint64_t k_{0};
};

struct ImuParams
{
  double rate_hz{200.0};
  double gyro_noise{0.002};   // rad/s, 1 sigma
  double acc_noise{0.01};     // m/s^2, 1 sigma
  Eigen::Vector3d gyro_bias{0.002, -0.001, 0.0015};
  Eigen::Vector3d acc_bias{0.02, -0.01, 0.015};
  bool output_in_g{true};     // Mid-360 reports acceleration in g
  uint32_t seed{2};
};

struct ImuMeasurement
{
  double t{0.0};
  Eigen::Vector3d gyr{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
};

/// IMU readings derived from the analytic pose function by numerical differentiation.
std::vector<ImuMeasurement> simulateImu(
  const PoseFn & T_world_imu, double t0, double t1, const ImuParams & p = {});

/// Horizontal single-beam scanner (chassis LiDAR): `beams` rays at mount height.
struct ScanSimParams
{
  int beams{360};
  double max_range{12.0};
  double min_range{0.05};
  double noise{0.01};
};

struct LaserScan
{
  double angle_min{0.0};
  double angle_increment{0.0};
  std::vector<float> ranges;  // inf when no return
};

LaserScan simulateScan(
  const World & world, const Eigen::Isometry3d & T_world_scanner, const ScanSimParams & p = {},
  std::mt19937 * rng = nullptr);

}  // namespace g1d::sim
