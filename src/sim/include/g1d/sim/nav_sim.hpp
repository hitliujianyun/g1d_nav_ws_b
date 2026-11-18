#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "g1d/nav/nav_core.hpp"
#include "g1d/sim/robot.hpp"
#include "g1d/sim/sensors.hpp"
#include "g1d/sim/world.hpp"

namespace g1d::sim
{

/// Marks returns as moving (`dynamic[i] = 1`) given world points and the LiDAR pose at time t.
using PointLabeler = std::function<void(
      const World & world, const std::vector<Eigen::Vector3f> & points_world,
      const Eigen::Isometry3d & T_world_lidar, double t, std::vector<uint8_t> & dynamic)>;

/// Ground-truth labels: returns within `margin` of a box whose name starts with `prefix`.
PointLabeler truthLabeler(std::string prefix = "ped", double margin = 0.05);

/// Moves box `name` so that its footprint centre is at (x, y); returns false if absent.
bool moveBox(World & world, const std::string & name, double x, double y);

struct NavSimParams
{
  double control_dt{0.05};
  double lidar_period{0.1};
  double scan_period{0.05};
  int lidar_points_per_frame{5000};
  double collision_z_max{1.75};       // whole robot height (head, arms) for collision checks
  std::vector<Eigen::Vector2d> body_footprint{
    {0.28, 0.265}, {0.28, -0.265}, {-0.28, -0.265}, {-0.28, 0.265}};  // 557 x 525 mm chassis
  Eigen::Isometry3d T_head_livox{makeIsometry(0.05, 0.0, 0.55, M_PI, -0.04, 0.0)};
  DiffDriveModel::Params dynamics;
  PointLabeler labeler;               // empty: every return is static
};

struct NavSimResult
{
  std::string final_state;
  std::string reason;
  double time_s{0.0};
  bool collided{false};
  double min_clearance{1e9};
  Pose2D final_pose;
  double path_length{0.0};
  double max_speed{0.0};
  int ticks{0};
  double mean_tick_ms{0.0};
  nlohmann::json last_status;
};

/// Closed loop: NavCore <-> diff-drive robot in a box world with simulated Mid-360S (truth
/// LIO pose), chassis scan and wheel odometry. Collision checks use the whole robot height.
class NavSimulation
{
public:
  NavSimulation(World world, const YAML::Node & config, Pose2D start, NavSimParams p = {});

  nav::NavCore & core() {return *core_;}
  World & world() {return world_;}
  const DiffDriveModel & robot() const {return robot_;}
  double time() const {return t_;}

  /// Advances the simulation by one control period.
  void step();
  /// Runs until the goal reaches a terminal state or `max_time` passes.
  NavSimResult runUntilDone(double max_time, const std::function<void(NavSimulation &)> & on_step = {});
  void sendGoal(const nlohmann::json & goal);
  /// Stops / resumes LIO updates (to emulate LiDAR loss).
  void setLioEnabled(bool on) {lio_enabled_ = on;}
  /// Runs `seconds` of simulation without checking for termination (map warm-up).
  void warmup(double seconds) {const double t_end = t_ + seconds; while (t_ < t_end) {step();}}

private:
  bool lio_enabled_{true};
  Eigen::Isometry3d worldImu() const;

  World world_;
  NavSimParams p_;
  std::unique_ptr<nav::NavCore> core_;
  nav::KinematicsParams kin_;
  DiffDriveModel robot_;
  std::unique_ptr<Mid360Sim> lidar_;
  std::mt19937 rng_{7};
  double t_{1000.0};
  double next_lidar_{0.0};
  double next_scan_{0.0};
  Eigen::Isometry3d T_world_lio_{Eigen::Isometry3d::Identity()};
  bool lio_started_{false};
  NavSimResult result_;
  double tick_ms_sum_{0.0};
};

}  // namespace g1d::sim
