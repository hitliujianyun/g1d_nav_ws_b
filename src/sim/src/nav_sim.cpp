#include "g1d/sim/nav_sim.hpp"

#include <chrono>
#include <cmath>

namespace g1d::sim
{

PointLabeler truthLabeler(std::string prefix, double margin)
{
  return [prefix, margin](
    const World & world, const std::vector<Eigen::Vector3f> & pts, const Eigen::Isometry3d &,
    double, std::vector<uint8_t> & dynamic) {
           std::vector<const Box *> movers;
           for (const auto & b : world.boxes()) {
             if (b.name.rfind(prefix, 0) == 0) {movers.push_back(&b);}
           }
           for (size_t i = 0; i < pts.size(); ++i) {
             const Eigen::Vector3d p = pts[i].cast<double>();
             for (const Box * b : movers) {
               if ((p.array() >= b->min.array() - margin).all() &&
                 (p.array() <= b->max.array() + margin).all())
               {
                 dynamic[i] = 1;
                 break;
               }
             }
           }
         };
}

bool moveBox(World & world, const std::string & name, double x, double y)
{
  for (auto & b : world.boxes()) {
    if (b.name != name) {continue;}
    const Eigen::Vector2d half = 0.5 * (b.max - b.min).head<2>();
    b.min.head<2>() = Eigen::Vector2d(x, y) - half;
    b.max.head<2>() = Eigen::Vector2d(x, y) + half;
    return true;
  }
  return false;
}

NavSimulation::NavSimulation(World world, const YAML::Node & config, Pose2D start, NavSimParams p)
: world_(std::move(world)), p_(std::move(p)), robot_(p_.dynamics, start)
{
  YAML::Node cfg = YAML::Clone(config);
  // The simulated head mount is the source of truth for both sides.
  const Eigen::Vector3d t = p_.T_head_livox.translation();
  const Eigen::Vector3d rpy = p_.T_head_livox.linear().eulerAngles(2, 1, 0).reverse();
  cfg["kinematics"]["head_to_livox"]["xyz"] = std::vector<double>{t.x(), t.y(), t.z()};
  cfg["kinematics"]["head_to_livox"]["rpy"] = std::vector<double>{rpy.x(), rpy.y(), rpy.z()};
  const nav::NavConfig nc = nav::NavConfig::fromYaml(cfg);
  kin_ = nc.kinematics;
  core_ = std::make_unique<nav::NavCore>(nc);
  Mid360Params mp;
  mp.range_noise = 0.01;
  lidar_ = std::make_unique<Mid360Sim>(world_, mp);
  nav::JointState q;
  q.stamp = t_;
  core_->onJointState(q);
}

Eigen::Isometry3d NavSimulation::worldImu() const
{
  const nav::BodyKinematics kin(kin_);
  return toIsometry(robot_.pose()) * kin.baseToImu(nav::JointState{});
}

void NavSimulation::sendGoal(const nlohmann::json & goal)
{
  core_->onGoalJson(goal.dump(), t_);
}

void NavSimulation::step()
{
  // Sensors
  if (lio_enabled_ && t_ >= next_lidar_) {
    next_lidar_ = t_ + p_.lidar_period;
    const Eigen::Isometry3d T_w_imu = worldImu();
    if (!lio_started_) {T_world_lio_ = T_w_imu; lio_started_ = true;}
    const nav::BodyKinematics kin(kin_);
    const Eigen::Isometry3d T_w_lidar = toIsometry(robot_.pose()) * kin.baseToLivox(nav::JointState{});
    const auto pts_w = lidar_->worldPoints(T_w_lidar, p_.lidar_points_per_frame);
    nav::LioInput in;
    in.stamp = t_;
    in.ok = true;
    const Eigen::Isometry3d T_lio_w = T_world_lio_.inverse();
    in.T_lio_imu = T_lio_w * T_w_imu;
    in.lidar_origin_lio = T_lio_w * T_w_lidar.translation();
    in.cloud_lio.reserve(pts_w.size());
    std::vector<uint8_t> dynamic(pts_w.size(), 0);
    if (p_.labeler) {p_.labeler(world_, pts_w, T_w_lidar, t_, dynamic);}
    const Eigen::Matrix3f R = T_lio_w.linear().cast<float>();
    const Eigen::Vector3f tr = T_lio_w.translation().cast<float>();
    for (size_t i = 0; i < pts_w.size(); ++i) {
      (dynamic[i] ? in.dynamic_lio : in.cloud_lio).push_back(R * pts_w[i] + tr);
    }
    core_->onLio(in);
  }
  if (t_ >= next_scan_) {
    next_scan_ = t_ + p_.scan_period;
    const Eigen::Isometry3d T_base_scanner =
      makeIsometry(0.1871, -0.0161, 0.1009 + kin_.base_to_agv_z, 0, 0, 0);
    const auto s = simulateScan(world_, toIsometry(robot_.pose()) * T_base_scanner, ScanSimParams{}, &rng_);
    nav::ScanData sd;
    sd.stamp = t_;
    sd.angle_min = static_cast<float>(s.angle_min);
    sd.angle_increment = static_cast<float>(s.angle_increment);
    sd.range_min = 0.05f;
    sd.range_max = 12.0f;
    sd.ranges = s.ranges;
    core_->onScan(sd);
  }
  core_->onWheelOdom(t_, robot_.pose(), robot_.v(), robot_.w());

  const auto t0 = std::chrono::steady_clock::now();
  const auto out = core_->tick(t_);
  tick_ms_sum_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  ++result_.ticks;
  if (out.status_json) {result_.last_status = nlohmann::json::parse(*out.status_json);}
  robot_.setCommand(out.publish_cmd ? out.cmd.v : 0.0, out.publish_cmd ? out.cmd.w : 0.0);

  const Pose2D before = robot_.pose();
  robot_.step(p_.control_dt);
  result_.path_length += before.distanceTo(robot_.pose());
  result_.max_speed = std::max(result_.max_speed, std::abs(robot_.v()));
  const double clear = world_.footprintClearance(
    footprintAt(robot_.pose(), p_.body_footprint), 0.0, p_.collision_z_max);
  result_.min_clearance = std::min(result_.min_clearance, clear);
  if (clear <= 0.0) {result_.collided = true;}
  t_ += p_.control_dt;
}

NavSimResult NavSimulation::runUntilDone(
  double max_time, const std::function<void(NavSimulation &)> & on_step)
{
  const double t_start = t_;
  // Let sensors populate the maps before acting on the goal state.
  while (t_ - t_start < max_time) {
    step();
    if (on_step) {on_step(*this);}
    const auto s = core_->state();
    if (s == nav::NavStateId::Succeeded || s == nav::NavStateId::Failed ||
      s == nav::NavStateId::Canceled)
    {
      // Let the robot come to rest so final pose / collision are evaluated after stopping.
      for (int i = 0; i < 20; ++i) {step();}
      break;
    }
  }
  result_.final_state = nav::toString(core_->state());
  result_.reason = core_->reason();
  result_.time_s = t_ - t_start;
  result_.final_pose = robot_.pose();
  result_.mean_tick_ms = result_.ticks ? tick_ms_sum_ / result_.ticks : 0.0;
  return result_;
}

}  // namespace g1d::sim
