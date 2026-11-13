#include "g1d/sim/sensors.hpp"

#include <cmath>
#include <limits>

namespace g1d::sim
{

Mid360Sim::Mid360Sim(const World & world, Mid360Params p)
: world_(world), p_(p), rng_(p.seed), noise_(0.0, p.range_noise) {}

std::vector<g1d::lio::LivoxPacket> Mid360Sim::scan(
  const PoseFn & T_world_lidar, double t0, double t1)
{
  std::vector<g1d::lio::LivoxPacket> packets;
  const double dt_point = 1.0 / p_.points_per_second;
  const double dt_packet = dt_point * p_.points_per_packet;
  const double sin_lo = std::sin(p_.fov_min_deg * M_PI / 180.0);
  const double sin_hi = std::sin(p_.fov_max_deg * M_PI / 180.0);
  constexpr double kGolden = 0.6180339887498949;
  constexpr double kSqrt2Frac = 0.4142135623730951;

  for (double tp = t0; tp < t1 - 1e-12; tp += dt_packet) {
    g1d::lio::LivoxPacket pkt;
    pkt.timestamp_ns = static_cast<uint64_t>(std::llround(tp * 1e9));
    pkt.time_interval = static_cast<uint32_t>(std::llround(dt_packet * 1e7));  // 0.1 us units
    pkt.points.resize(p_.points_per_packet);
    for (int i = 0; i < p_.points_per_packet; ++i, ++k_) {
      const double t = tp + i * dt_point;
      const Eigen::Isometry3d T = T_world_lidar(t);
      // Low-discrepancy (rosette-like) coverage: azimuth by golden ratio, elevation uniform in
      // sin(el) so the density per solid angle is uniform.
      const double az = 2.0 * M_PI * std::fmod(k_ * kGolden, 1.0);
      const double u = std::fmod(k_ * kSqrt2Frac + 0.5 * std::fmod(k_ * 0.000137, 1.0), 1.0);
      const double sel = sin_lo + (sin_hi - sin_lo) * u;
      const double cel = std::sqrt(std::max(0.0, 1.0 - sel * sel));
      const Eigen::Vector3d dir_s(cel * std::cos(az), cel * std::sin(az), sel);
      const Eigen::Vector3d dir_w = T.linear() * dir_s;
      auto & out = pkt.points[i];
      const auto hit = world_.raycast(T.translation(), dir_w, p_.max_range);
      if (!hit || *hit < p_.min_range) {
        out = {};  // no return: SDK reports zeros
        continue;
      }
      const double r = *hit + noise_(rng_);
      const Eigen::Vector3d ps = dir_s * r;
      out.x_mm = static_cast<int32_t>(std::lround(ps.x() * 1000.0));
      out.y_mm = static_cast<int32_t>(std::lround(ps.y() * 1000.0));
      out.z_mm = static_cast<int32_t>(std::lround(ps.z() * 1000.0));
      out.reflectivity = 80;
      out.tag = 0;
    }
    packets.push_back(std::move(pkt));
  }
  return packets;
}

std::vector<Eigen::Vector3f> Mid360Sim::worldPoints(const Eigen::Isometry3d & T, int n)
{
  std::vector<Eigen::Vector3f> out;
  out.reserve(n);
  const double sin_lo = std::sin(p_.fov_min_deg * M_PI / 180.0);
  const double sin_hi = std::sin(p_.fov_max_deg * M_PI / 180.0);
  constexpr double kGolden = 0.6180339887498949;
  constexpr double kSqrt2Frac = 0.4142135623730951;
  for (int i = 0; i < n; ++i, ++k_) {
    const double az = 2.0 * M_PI * std::fmod(k_ * kGolden, 1.0);
    const double u = std::fmod(k_ * kSqrt2Frac + 0.5 * std::fmod(k_ * 0.000137, 1.0), 1.0);
    const double sel = sin_lo + (sin_hi - sin_lo) * u;
    const double cel = std::sqrt(std::max(0.0, 1.0 - sel * sel));
    const Eigen::Vector3d dir_w = T.linear() * Eigen::Vector3d(cel * std::cos(az), cel * std::sin(az), sel);
    const auto hit = world_.raycast(T.translation(), dir_w, p_.max_range);
    if (!hit || *hit < p_.min_range) {continue;}
    out.push_back((T.translation() + dir_w * (*hit + noise_(rng_))).cast<float>());
  }
  return out;
}

std::vector<ImuMeasurement> simulateImu(
  const PoseFn & T_world_imu, double t0, double t1, const ImuParams & p)
{
  std::mt19937 rng(p.seed);
  std::normal_distribution<double> ng(0.0, p.gyro_noise), na(0.0, p.acc_noise);
  const Eigen::Vector3d g_world(0.0, 0.0, -9.81);
  const double h = 1e-3;
  std::vector<ImuMeasurement> out;
  const double dt = 1.0 / p.rate_hz;
  for (double t = t0; t < t1 - 1e-12; t += dt) {
    const Eigen::Isometry3d Tm = T_world_imu(t - h);
    const Eigen::Isometry3d T0 = T_world_imu(t);
    const Eigen::Isometry3d Tp = T_world_imu(t + h);
    const Eigen::Vector3d a_world =
      (Tp.translation() - 2.0 * T0.translation() + Tm.translation()) / (h * h);
    // Body angular velocity from R^T * dR/dt (skew-symmetric part).
    const Eigen::Matrix3d W = T0.linear().transpose() * (Tp.linear() - Tm.linear()) / (2.0 * h);
    const Eigen::Vector3d w(
      0.5 * (W(2, 1) - W(1, 2)), 0.5 * (W(0, 2) - W(2, 0)), 0.5 * (W(1, 0) - W(0, 1)));
    ImuMeasurement m;
    m.t = t;
    m.gyr = w + p.gyro_bias + Eigen::Vector3d(ng(rng), ng(rng), ng(rng));
    m.acc = T0.linear().transpose() * (a_world - g_world) + p.acc_bias +
      Eigen::Vector3d(na(rng), na(rng), na(rng));
    if (p.output_in_g) {m.acc /= 9.81;}
    out.push_back(m);
  }
  return out;
}

LaserScan simulateScan(
  const World & world, const Eigen::Isometry3d & T_world_scanner, const ScanSimParams & p,
  std::mt19937 * rng)
{
  LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / p.beams;
  s.ranges.resize(p.beams, std::numeric_limits<float>::infinity());
  std::normal_distribution<double> n(0.0, p.noise);
  for (int i = 0; i < p.beams; ++i) {
    const double a = s.angle_min + i * s.angle_increment;
    const Eigen::Vector3d dir = T_world_scanner.linear() * Eigen::Vector3d(std::cos(a), std::sin(a), 0);
    const auto hit = world.raycast(T_world_scanner.translation(), dir, p.max_range);
    if (hit && *hit >= p.min_range) {
      s.ranges[i] = static_cast<float>(*hit + (rng ? n(*rng) : 0.0));
    }
  }
  return s;
}

}  // namespace g1d::sim
