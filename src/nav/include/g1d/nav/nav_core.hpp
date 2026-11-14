#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "g1d/nav/body_kinematics.hpp"
#include "g1d/nav/controller.hpp"
#include "g1d/nav/costmap/layered_costmap.hpp"
#include "g1d/nav/planner.hpp"
#include "g1d/nav/state_estimator.hpp"
#include "g1d/nav/types.hpp"

namespace g1d::nav
{

enum class NavStateId {Idle, Navigating, Rotating, Recovering, Succeeded, Failed, Canceled};
const char * toString(NavStateId s);

struct GoalRequest
{
  std::string id;
  std::string mode{"navigate"};   // navigate | rotate | simple | stop
  std::string frame{"base_at_stamp"};  // base_at_stamp | odom
  double stamp{0.0};
  double x{0.0}, y{0.0}, yaw{0.0};
  bool use_yaw{false};
  double xy_tolerance{0.2};
  double yaw_tolerance{0.25};
  double max_speed{0.0};          // 0 = controller default
  double timeout_s{120.0};

  static GoalRequest fromJson(const nlohmann::json & j);
};

struct ManagerParams
{
  double replan_period_s{1.0};
  double progress_window_s{8.0};
  double progress_min_m{0.2};
  double mppi_fail_tolerance_s{1.0};
  double goal_stamp_max_age_s{5.0};
  double final_rotation_kp{1.5};
  double final_rotation_wmax{0.5};
  double simple_v_max{0.3};
  double simple_kp_yaw{1.5};
  double approach_distance{0.6};   // switch from MPPI to the straight final approach (m)
  double approach_v_max{0.2};
  double free_sector_range_m{1.5};
  double status_period_s{0.1};
  std::vector<std::string> recovery{"wait", "clear", "rotate"};
  double recovery_wait_s{2.0};
  double recovery_clear_radius_m{1.5};
  double recovery_rotate_rad{0.52};
  bool require_lio{true};

  static ManagerParams fromYaml(const YAML::Node & root);
};

struct NavConfig
{
  KinematicsParams kinematics;
  StateEstimatorParams state;
  costmap::LayeredCostmapParams local;
  costmap::LayeredCostmapParams global;
  PlannerParams planner;
  ControllerParams controller;
  ManagerParams manager;
  Eigen::Isometry3d T_base_scanner{Eigen::Isometry3d::Identity()};  // chassis LiDAR mount
  double obstacle_points_range{3.0};   // for rt/nav/obstacle_points (guard)
  double obstacle_points_voxel{0.05};

  static NavConfig fromYaml(const YAML::Node & root);
};

struct TickOutput
{
  Twist2D cmd;
  bool publish_cmd{false};         // false while idle (guard then times out to zero)
  std::optional<std::string> status_json;
  NavState state;
};

/// Navigation core: state estimation, costmaps, NavFn, MPPI and the goal state machine.
/// Not thread-safe: call every method from one thread (the DDS app queues inputs).
class NavCore
{
public:
  explicit NavCore(const NavConfig & cfg);

  // ---- inputs
  void onLio(const LioInput & in);
  void onWheelOdom(double stamp, const Pose2D & wheel_pose, double v, double w);
  void onScan(const ScanData & scan);
  void onJointState(const JointState & q);
  void onGoal(const GoalRequest & goal, double now);
  void onGoalJson(const std::string & json, double now);
  void onEstop(bool on, double now);
  void onGuardState(bool blocked, bool estop, bool manual);

  // ---- control step (call at the controller rate)
  TickOutput tick(double now);

  // ---- accessors (debug / simulation)
  const StateEstimator & estimator() const {return estimator_;}
  costmap::LayeredCostmap & localCostmap() {return *local_;}
  costmap::LayeredCostmap & globalCostmap() {return *global_;}
  const std::vector<Pose2D> & globalPath() const {return path_;}
  NavStateId state() const {return state_;}
  const std::string & reason() const {return reason_;}
  /// Latest Mid-360S obstacle points in base_link (height band, within range) for the guard.
  const std::vector<Eigen::Vector3f> & obstaclePointsBase() const {return obstacle_points_base_;}
  double lastObstaclePointsStamp() const {return obstacle_points_stamp_;}
  const MppiController & controller() const {return *controller_;}
  nlohmann::json statusJson(double now) const;
  std::map<int, bool> freeSectors(const Pose2D & pose) const;
  /// Web / debug view: T_odom_lio, active goal, global + local path, MPPI optimal trajectory.
  nlohmann::json debugJson(double now) const;

  // ---- runtime parameters (rt/nav/param_set); see paramsJson() for the names
  bool setParam(const std::string & name, double value, std::string * error = nullptr);
  nlohmann::json paramsJson() const;

private:
  void finish(NavStateId s, const std::string & reason, double now);
  bool startGoal(const NavState & st, double now);
  bool plan(const NavState & st, std::string * error);
  bool pathBlocked(const Pose2D & pose) const;
  Twist2D rotateTowards(const NavState & st, double target_yaw, bool * done) const;
  bool rotationSafe(const Pose2D & pose, double dyaw) const;
  Twist2D stepNavigate(const NavState & st, double now);
  Twist2D stepSimple(const NavState & st, double now);
  /// Straight final approach near the goal; returns nullopt if the segment is not free.
  std::optional<Twist2D> stepApproach(const NavState & st);
  bool segmentFree(const Pose2D & from, double heading, double length) const;
  Twist2D stepRecovery(const NavState & st, double now);
  void beginRecovery(const NavState & st, double now, const std::string & why);
  double effectiveSpeedLimit() const;
  std::map<std::string, double *> tunables();

  NavConfig cfg_;
  double default_max_speed_{0.0};  // 0 = MPPI vx_max
  BodyKinematics kin_;
  StateEstimator estimator_;
  std::unique_ptr<costmap::LayeredCostmap> local_;
  std::unique_ptr<costmap::LayeredCostmap> global_;
  GlobalPlanner planner_;
  std::unique_ptr<MppiController> controller_;

  NavStateId state_{NavStateId::Idle};
  std::string reason_;
  std::optional<GoalRequest> goal_req_;
  Pose2D goal_;              // odom
  bool goal_adjusted_{false};
  bool goal_partial_{false};
  std::vector<Pose2D> path_;
  double goal_start_t_{0.0};
  double last_plan_t_{-1e9};
  double best_dist_{1e9};
  double best_dist_t_{0.0};
  double mppi_fail_since_{-1.0};
  bool final_rotation_{false};
  size_t recovery_idx_{0};
  double recovery_t0_{0.0};
  double recovery_target_yaw_{0.0};
  std::string recovery_reason_;
  int recoveries_{0};
  bool estop_{false};
  bool guard_blocked_{false};
  bool guard_estop_{false};
  bool guard_manual_{false};
  JointState reference_joints_;
  bool has_reference_joints_{false};
  LockStatus lock_;
  double finish_t_{-1e9};
  double rotate_blocked_since_{-1.0};
  double last_global_update_t_{-1e9};
  double last_status_t_{-1e9};
  NavStateId last_status_state_{NavStateId::Idle};
  std::vector<Eigen::Vector3f> obstacle_points_base_;
  double obstacle_points_stamp_{0.0};
  NavState last_state_;
};

}  // namespace g1d::nav
