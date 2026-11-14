#include "g1d/nav/nav_core.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "g1d/common/config.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace g1d::nav
{

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;

const char * toString(NavStateId s)
{
  switch (s) {
    case NavStateId::Idle: return "IDLE";
    case NavStateId::Navigating: return "NAVIGATING";
    case NavStateId::Rotating: return "ROTATING";
    case NavStateId::Recovering: return "RECOVERING";
    case NavStateId::Succeeded: return "SUCCEEDED";
    case NavStateId::Failed: return "FAILED";
    case NavStateId::Canceled: return "CANCELED";
  }
  return "UNKNOWN";
}

namespace
{
bool isActive(NavStateId s)
{
  return s == NavStateId::Navigating || s == NavStateId::Rotating || s == NavStateId::Recovering;
}
}  // namespace

// ------------------------------------------------------------------------------- config
GoalRequest GoalRequest::fromJson(const nlohmann::json & j)
{
  GoalRequest g;
  g.id = j.value("id", std::string());
  g.mode = j.value("mode", g.mode);
  g.frame = j.value("frame", g.frame);
  g.stamp = j.value("stamp", 0.0);
  g.x = j.value("x", 0.0);
  g.y = j.value("y", 0.0);
  if (j.contains("yaw") && j["yaw"].is_number()) {
    g.yaw = j["yaw"].get<double>();
    g.use_yaw = j.value("use_yaw", true);
  } else {
    g.use_yaw = false;
  }
  g.xy_tolerance = j.value("xy_tolerance", g.xy_tolerance);
  g.yaw_tolerance = j.value("yaw_tolerance", g.yaw_tolerance);
  g.max_speed = j.value("max_speed", g.max_speed);
  g.timeout_s = j.value("timeout_s", g.timeout_s);
  if (g.mode != "navigate" && g.mode != "rotate" && g.mode != "simple" && g.mode != "stop") {
    throw std::invalid_argument("unknown mode '" + g.mode + "'");
  }
  if (g.frame != "base_at_stamp" && g.frame != "odom") {
    throw std::invalid_argument("unknown frame '" + g.frame + "'");
  }
  return g;
}

ManagerParams ManagerParams::fromYaml(const YAML::Node & root)
{
  ManagerParams p;
  p.replan_period_s = 1.0 / get(root, "planner.replan_hz", 1.0 / p.replan_period_s);
  p.progress_window_s = get(root, "manager.progress_window_s", p.progress_window_s);
  p.progress_min_m = get(root, "manager.progress_min_m", p.progress_min_m);
  p.mppi_fail_tolerance_s = get(root, "manager.mppi_fail_tolerance_s", p.mppi_fail_tolerance_s);
  p.goal_stamp_max_age_s = get(root, "manager.goal_stamp_max_age_s", p.goal_stamp_max_age_s);
  p.final_rotation_kp = get(root, "manager.final_rotation_kp", p.final_rotation_kp);
  p.final_rotation_wmax = get(root, "manager.final_rotation_wmax", p.final_rotation_wmax);
  p.simple_v_max = get(root, "manager.simple_v_max", p.simple_v_max);
  p.simple_kp_yaw = get(root, "manager.simple_kp_yaw", p.simple_kp_yaw);
  p.approach_distance = get(root, "manager.approach_distance", p.approach_distance);
  p.approach_v_max = get(root, "manager.approach_v_max", p.approach_v_max);
  p.free_sector_range_m = get(root, "manager.free_sector_range_m", p.free_sector_range_m);
  p.status_period_s = get(root, "manager.status_period_s", p.status_period_s);
  p.recovery = get(root, "manager.recovery", p.recovery);
  p.recovery_wait_s = get(root, "manager.recovery_wait_s", p.recovery_wait_s);
  p.recovery_clear_radius_m = get(root, "manager.recovery_clear_radius_m", p.recovery_clear_radius_m);
  p.recovery_rotate_rad = get(root, "manager.recovery_rotate_rad", p.recovery_rotate_rad);
  p.require_lio = get(root, "manager.require_lio", p.require_lio);
  return p;
}

NavConfig NavConfig::fromYaml(const YAML::Node & root)
{
  NavConfig c;
  c.kinematics = KinematicsParams::fromYaml(root);
  c.state = StateEstimatorParams::fromYaml(root);
  c.local = costmap::LayeredCostmapParams::fromYaml(root, "local");
  c.global = costmap::LayeredCostmapParams::fromYaml(root, "global");
  c.planner = PlannerParams::fromYaml(root);
  c.controller = ControllerParams::fromYaml(root);
  c.manager = ManagerParams::fromYaml(root);
  const auto xyz = get(root, "robot.chassis_laser.xyz",
      std::vector<double>{0.1871, -0.0161, 0.1009 + c.kinematics.base_to_agv_z});
  const auto rpy = get(root, "robot.chassis_laser.rpy", std::vector<double>{0.0, 0.0, 0.0});
  c.T_base_scanner = makeIsometry(xyz.at(0), xyz.at(1), xyz.at(2), rpy.at(0), rpy.at(1), rpy.at(2));
  c.obstacle_points_range = get(root, "guard.obstacle_points_range", c.obstacle_points_range);
  c.obstacle_points_voxel = get(root, "guard.obstacle_points_voxel", c.obstacle_points_voxel);
  return c;
}

// --------------------------------------------------------------------------------- inputs
NavCore::NavCore(const NavConfig & cfg)
: cfg_(cfg), kin_(cfg.kinematics), estimator_(cfg.state, kin_), planner_(cfg.planner)
{
  local_ = std::make_unique<costmap::LayeredCostmap>(cfg.local);
  global_ = std::make_unique<costmap::LayeredCostmap>(cfg.global);
  controller_ = std::make_unique<MppiController>(local_->costmapRos(), cfg.controller);
}

void NavCore::onLio(const LioInput & in)
{
  estimator_.onLio(in);
  if (!in.ok || !estimator_.initialized()) {return;}
  const Eigen::Isometry3d T_odom_lio = estimator_.T_odom_lio();
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(in.cloud_lio.size());
  const Eigen::Matrix3f R = T_odom_lio.linear().cast<float>();
  const Eigen::Vector3f t = T_odom_lio.translation().cast<float>();
  for (const auto & p : in.cloud_lio) {pts.push_back(R * p + t);}
  const Eigen::Vector3d origin = T_odom_lio * in.lidar_origin_lio;
  local_->addCloud(pts, origin, in.stamp);
  global_->addCloud(pts, origin, in.stamp);

  // Obstacle points in base_link for the guard reflex layer.
  const Eigen::Isometry3d T_odom_base =
    T_odom_lio * in.T_lio_imu * kin_.baseToImu(estimator_.joints()).inverse();
  const Eigen::Isometry3d T_base_odom = T_odom_base.inverse();
  const auto & vp = cfg_.local.voxel;
  const double r2 = cfg_.obstacle_points_range * cfg_.obstacle_points_range;
  const double inv = 1.0 / cfg_.obstacle_points_voxel;
  std::unordered_set<int64_t> seen;
  obstacle_points_base_.clear();
  for (const auto & p : pts) {
    const Eigen::Vector3d b = T_base_odom * p.cast<double>();
    // Heights are relative to the floor: odom z == base z for a level base.
    if (p.z() < vp.z_min || p.z() >= vp.z_max) {continue;}
    if (b.x() * b.x() + b.y() * b.y() > r2) {continue;}
    if (costmap::pointInPolygon(b.head<2>(), cfg_.local.footprint)) {continue;}
    const int64_t key = (static_cast<int64_t>(std::floor(b.x() * inv)) & 0xFFFFF) << 40 |
      (static_cast<int64_t>(std::floor(b.y() * inv)) & 0xFFFFF) << 20 |
      (static_cast<int64_t>(std::floor(b.z() * inv)) & 0xFFFFF);
    if (seen.insert(key).second) {obstacle_points_base_.push_back(b.cast<float>());}
  }
  obstacle_points_stamp_ = in.stamp;
}

void NavCore::onWheelOdom(double stamp, const Pose2D & wheel_pose, double v, double w)
{
  estimator_.onWheelOdom(stamp, wheel_pose, v, w);
}

void NavCore::onScan(const ScanData & scan)
{
  const NavState st = estimator_.estimate(scan.stamp);
  if (!st.valid) {return;}
  const Pose2D scanner = st.pose.compose(toPose2D(cfg_.T_base_scanner));
  local_->addScan(scan, scanner);
  global_->addScan(scan, scanner);
}

void NavCore::onJointState(const JointState & q)
{
  estimator_.onJointState(q);
  if (!isActive(state_)) {
    reference_joints_ = q;
    has_reference_joints_ = true;
  }
}

void NavCore::onEstop(bool on, double now)
{
  estop_ = on;
  if (on && isActive(state_)) {finish(NavStateId::Failed, "guard_estop", now);}
}

void NavCore::onGuardState(bool blocked, bool estop, bool manual)
{
  guard_blocked_ = blocked;
  guard_estop_ = estop;
  guard_manual_ = manual;
}

void NavCore::onGoalJson(const std::string & json, double now)
{
  try {
    onGoal(GoalRequest::fromJson(nlohmann::json::parse(json)), now);
  } catch (const std::exception & e) {
    goal_req_ = GoalRequest{};
    finish(NavStateId::Failed, std::string("invalid_goal: ") + e.what(), now);
  }
}

void NavCore::onGoal(const GoalRequest & goal, double now)
{
  if (goal.mode == "stop") {
    if (isActive(state_)) {finish(NavStateId::Canceled, "stop", now);}
    return;
  }
  goal_req_ = goal;
  const NavState st = estimator_.estimate(now);
  startGoal(st, now);
}

void NavCore::finish(NavStateId s, const std::string & reason, double now)
{
  state_ = s;
  reason_ = reason;
  finish_t_ = now;
  final_rotation_ = false;
  controller_->reset();
}

// ------------------------------------------------------------------------------ goal start
bool NavCore::startGoal(const NavState & st, double now)
{
  const GoalRequest & g = *goal_req_;
  reason_.clear();
  goal_adjusted_ = goal_partial_ = false;
  recovery_idx_ = 0;
  recoveries_ = 0;
  path_.clear();
  if (cfg_.manager.require_lio && !st.valid) {
    finish(NavStateId::Failed, "lio_lost", now);
    return false;
  }
  if (estop_ || guard_estop_) {
    finish(NavStateId::Failed, "guard_estop", now);
    return false;
  }
  if (cfg_.kinematics.require_lock) {
    lock_ = kin_.checkLock(estimator_.joints(), estimator_.joints());
    if (!lock_.locked) {
      finish(NavStateId::Failed, "body_not_locked", now);
      return false;
    }
  }
  reference_joints_ = estimator_.joints();
  has_reference_joints_ = true;

  Pose2D base = st.pose;
  if (g.frame == "base_at_stamp" && g.stamp > 0.0) {
    if (auto p = estimator_.poseAt(g.stamp)) {
      base = *p;
    } else if (now - g.stamp > cfg_.manager.goal_stamp_max_age_s || g.stamp > now + 1.0) {
      finish(NavStateId::Failed, "goal_stamp_unavailable", now);
      return false;
    }
  }
  if (g.mode == "rotate") {
    const double target = g.frame == "odom" ? g.yaw : base.yaw + g.yaw;
    goal_ = Pose2D(st.pose.x, st.pose.y, wrapAngle(target));
    state_ = NavStateId::Rotating;
    goal_start_t_ = now;
    rotate_blocked_since_ = -1.0;
    return true;
  }
  Pose2D target = g.frame == "odom" ? Pose2D(g.x, g.y, g.yaw) : base.compose(Pose2D(g.x, g.y, g.yaw));
  if (!g.use_yaw) {target.yaw = std::atan2(target.y - st.pose.y, target.x - st.pose.x);}

  global_->update(st.pose, now);
  last_global_update_t_ = now;
  const GoalCheck check = planner_.sanitizeGoal(*global_->costmap(), st.pose, target);
  if (!check.ok) {
    finish(NavStateId::Failed, check.reason, now);
    return false;
  }
  goal_ = check.goal;
  goal_adjusted_ = check.adjusted;
  goal_partial_ = check.partial;

  if (g.mode == "navigate") {
    std::string err;
    if (!plan(st, &err)) {
      finish(NavStateId::Failed, err.empty() ? "no_path" : err, now);
      return false;
    }
    controller_->reset();
    controller_->setPlan(path_);
    controller_->setSpeedLimit(effectiveSpeedLimit());
  } else {
    path_ = {st.pose, goal_};
  }
  state_ = NavStateId::Navigating;
  goal_start_t_ = now;
  last_plan_t_ = now;
  best_dist_ = st.pose.distanceTo(goal_);
  best_dist_t_ = now;
  mppi_fail_since_ = -1.0;
  final_rotation_ = false;
  rotate_blocked_since_ = -1.0;
  return true;
}

bool NavCore::plan(const NavState & st, std::string * error)
{
  std::vector<Pose2D> p;
  if (!planner_.makePlan(*global_->costmap(), st.pose, goal_, goal_req_->use_yaw, p, error)) {
    return false;
  }
  path_ = std::move(p);
  return true;
}

// ---------------------------------------------------------------------------- helpers
bool NavCore::pathBlocked(const Pose2D & pose) const
{
  (void)pose;
  for (const auto & p : controller_->localPath()) {
    if (local_->costAt(p.x, p.y) >= INSCRIBED_INFLATED_OBSTACLE &&
      local_->costAt(p.x, p.y) != nav2_costmap_2d::NO_INFORMATION)
    {
      return true;
    }
  }
  return false;
}

bool NavCore::rotationSafe(const Pose2D & pose, double dyaw) const
{
  const int n = std::max(2, static_cast<int>(std::ceil(std::abs(dyaw) / 0.1)));
  for (int i = 1; i <= n; ++i) {
    const Pose2D q(pose.x, pose.y, wrapAngle(pose.yaw + dyaw * i / n));
    if (local_->footprintCost(q) >= LETHAL_OBSTACLE) {return false;}
  }
  return true;
}

Twist2D NavCore::rotateTowards(const NavState & st, double target_yaw, bool * done) const
{
  const double err = wrapAngle(target_yaw - st.pose.yaw);
  const double tol = goal_req_ ? goal_req_->yaw_tolerance : 0.1;
  *done = std::abs(err) <= tol && std::abs(st.velocity.w) < 0.2;
  if (std::abs(err) <= tol) {return Twist2D{};}
  double w = std::clamp(cfg_.manager.final_rotation_kp * err,
      -cfg_.manager.final_rotation_wmax, cfg_.manager.final_rotation_wmax);
  if (std::abs(w) < 0.15) {w = std::copysign(0.15, err);}
  if (!rotationSafe(st.pose, std::copysign(std::min(std::abs(err), 0.3), err))) {
    return Twist2D{};
  }
  return Twist2D{0.0, w};
}

std::map<int, bool> NavCore::freeSectors(const Pose2D & pose) const
{
  std::map<int, bool> out;
  for (int deg = -90; deg <= 90; deg += 30) {
    const double a = deg * M_PI / 180.0;
    bool free = true;
    for (double d = 0.3; d <= cfg_.manager.free_sector_range_m + 1e-9; d += 0.15) {
      const Pose2D q = pose.compose(Pose2D(d * std::cos(a), d * std::sin(a), a));
      if (local_->footprintCost(q) >= LETHAL_OBSTACLE) {free = false; break;}
    }
    out[deg] = free;
  }
  return out;
}

void NavCore::beginRecovery(const NavState & st, double now, const std::string & why)
{
  const auto & seq = cfg_.manager.recovery;
  if (recovery_idx_ >= seq.size()) {
    finish(NavStateId::Failed, why == "no_progress" ? "blocked" : why, now);
    return;
  }
  const std::string action = seq[recovery_idx_++];
  ++recoveries_;
  recovery_reason_ = why;
  recovery_t0_ = now;
  state_ = NavStateId::Recovering;
  if (action == "clear") {
    local_->clearOutsideRadius(st.pose, cfg_.manager.recovery_clear_radius_m);
    global_->clearOutsideRadius(st.pose, cfg_.manager.recovery_clear_radius_m);
  } else if (action == "rotate") {
    const auto fs = freeSectors(st.pose);
    int left = 0, right = 0;
    for (const auto & kv : fs) {
      if (kv.second && kv.first > 0) {++left;}
      if (kv.second && kv.first < 0) {++right;}
    }
    const double dir = left >= right ? 1.0 : -1.0;
    recovery_target_yaw_ = wrapAngle(st.pose.yaw + dir * cfg_.manager.recovery_rotate_rad);
  }
}

Twist2D NavCore::stepRecovery(const NavState & st, double now)
{
  const std::string & action = cfg_.manager.recovery[recovery_idx_ - 1];
  bool done = false;
  Twist2D cmd;
  if (action == "wait") {
    done = now - recovery_t0_ >= cfg_.manager.recovery_wait_s;
  } else if (action == "rotate") {
    const double err = wrapAngle(recovery_target_yaw_ - st.pose.yaw);
    if (std::abs(err) < 0.08 || now - recovery_t0_ > 5.0 ||
      !rotationSafe(st.pose, std::copysign(std::min(std::abs(err), 0.3), err)))
    {
      done = true;
    } else {
      cmd.w = std::clamp(cfg_.manager.final_rotation_kp * err, -cfg_.manager.final_rotation_wmax,
          cfg_.manager.final_rotation_wmax);
    }
  } else {
    done = true;  // "clear" acts immediately
  }
  if (done) {
    state_ = NavStateId::Navigating;
    best_dist_ = st.pose.distanceTo(goal_);
    best_dist_t_ = now;
    mppi_fail_since_ = -1.0;
    last_plan_t_ = -1e9;  // force a replan
    controller_->reset();
    return Twist2D{};
  }
  return cmd;
}

Twist2D NavCore::stepNavigate(const NavState & st, double now)
{
  const GoalRequest & g = *goal_req_;
  const double dist = st.pose.distanceTo(goal_);
  bool done = false;
  if (final_rotation_) {
    const Twist2D cmd = rotateTowards(st, goal_.yaw, &done);
    if (done) {finish(NavStateId::Succeeded, "", now); return Twist2D{};}
    if (cmd.w == 0.0) {
      // Position reached but turning in place would hit something: report it instead of waiting.
      if (rotate_blocked_since_ < 0.0) {rotate_blocked_since_ = now;}
      if (now - rotate_blocked_since_ > 3.0) {
        finish(NavStateId::Succeeded, "yaw_not_reached", now);
      }
    } else {
      rotate_blocked_since_ = -1.0;
    }
    return cmd;
  }
  if (dist <= g.xy_tolerance) {
    if (g.use_yaw && std::abs(wrapAngle(goal_.yaw - st.pose.yaw)) > g.yaw_tolerance) {
      final_rotation_ = true;
      return rotateTowards(st, goal_.yaw, &done);
    }
    finish(NavStateId::Succeeded, "", now);
    return Twist2D{};
  }
  // Required progress shrinks near the goal (0.2 m cannot be gained when 0.25 m away).
  const double needed = std::clamp(0.5 * (dist - g.xy_tolerance), 0.02, cfg_.manager.progress_min_m);
  if (dist < best_dist_ - needed) {
    best_dist_ = dist;
    best_dist_t_ = now;
  } else if (now - best_dist_t_ > cfg_.manager.progress_window_s) {
    beginRecovery(st, now, "no_progress");
    return Twist2D{};
  }

  if (g.mode == "simple") {return stepSimple(st, now);}

  if (dist < cfg_.manager.approach_distance) {
    if (auto cmd = stepApproach(st)) {
      mppi_fail_since_ = -1.0;
      return *cmd;
    }
  }

  if (now - last_plan_t_ >= cfg_.manager.replan_period_s || pathBlocked(st.pose)) {
    if (now - last_global_update_t_ > 0.2) {
      global_->update(st.pose, now);
      last_global_update_t_ = now;
    }
    std::string err;
    if (plan(st, &err)) {
      controller_->setPlan(path_);
    } else {
      beginRecovery(st, now, "no_path");
      return Twist2D{};
    }
    last_plan_t_ = now;
  }

  Twist2D cmd;
  std::string err;
  if (!controller_->computeCommand(st.pose, st.velocity, goal_, cmd, &err)) {
    if (mppi_fail_since_ < 0.0) {mppi_fail_since_ = now;}
    if (now - mppi_fail_since_ > cfg_.manager.mppi_fail_tolerance_s) {
      beginRecovery(st, now, "blocked");
    }
    return Twist2D{};
  }
  mppi_fail_since_ = -1.0;
  return cmd;
}

bool NavCore::segmentFree(const Pose2D & from, double heading, double length) const
{
  for (double d = 0.0; d <= length + 1e-9; d += 0.05) {
    const Pose2D q(from.x + d * std::cos(heading), from.y + d * std::sin(heading), heading);
    if (local_->footprintCost(q) >= LETHAL_OBSTACLE) {return false;}
  }
  return true;
}

std::optional<Twist2D> NavCore::stepApproach(const NavState & st)
{
  const double dist = st.pose.distanceTo(goal_);
  const double heading = std::atan2(goal_.y - st.pose.y, goal_.x - st.pose.x);
  const double err = wrapAngle(heading - st.pose.yaw);
  if (!segmentFree(st.pose, heading, dist)) {return std::nullopt;}
  if (std::abs(err) > 0.35) {
    if (!rotationSafe(st.pose, std::copysign(std::min(std::abs(err), 0.3), err))) {
      return std::nullopt;
    }
    double w = std::clamp(cfg_.manager.final_rotation_kp * err, -cfg_.manager.final_rotation_wmax,
        cfg_.manager.final_rotation_wmax);
    if (std::abs(w) < 0.15) {w = std::copysign(0.15, err);}
    return Twist2D{0.0, w};
  }
  const double v = std::min(cfg_.manager.approach_v_max, 0.8 * dist + 0.03);
  return Twist2D{v * std::cos(err), cfg_.manager.simple_kp_yaw * err};
}

Twist2D NavCore::stepSimple(const NavState & st, double now)
{
  // Legacy "rotate to face the goal, then drive straight", closed-loop on the fused pose,
  // with a footprint sweep check of the straight segment.
  const GoalRequest & g = *goal_req_;
  const double dist = st.pose.distanceTo(goal_);
  const double heading = std::atan2(goal_.y - st.pose.y, goal_.x - st.pose.x);
  const double err = wrapAngle(heading - st.pose.yaw);
  const double vmax = g.max_speed > 0.0 ? std::min(g.max_speed, cfg_.manager.simple_v_max) :
    cfg_.manager.simple_v_max;
  if (std::abs(err) > 0.15) {
    double w = std::clamp(cfg_.manager.simple_kp_yaw * err, -cfg_.manager.final_rotation_wmax,
        cfg_.manager.final_rotation_wmax);
    if (std::abs(w) < 0.15) {w = std::copysign(0.15, err);}
    if (!rotationSafe(st.pose, std::copysign(std::min(std::abs(err), 0.3), err))) {
      finish(NavStateId::Failed, "blocked", now);
      return Twist2D{};
    }
    return Twist2D{0.0, w};
  }
  const double check = std::min(dist, 1.5);
  for (double d = 0.1; d <= check + 1e-9; d += 0.1) {
    const Pose2D q = st.pose.compose(Pose2D(d * std::cos(err), d * std::sin(err), err));
    if (local_->footprintCost(q) >= LETHAL_OBSTACLE) {
      finish(NavStateId::Failed, "blocked", now);
      return Twist2D{};
    }
  }
  return Twist2D{std::min(vmax, 0.6 * dist + 0.05), cfg_.manager.simple_kp_yaw * err};
}

// -------------------------------------------------------------------------------- tick
TickOutput NavCore::tick(double now)
{
  TickOutput out;
  const NavState st = estimator_.estimate(now);
  estimator_.record(st);
  last_state_ = st;
  out.state = st;
  if (st.valid) {local_->update(st.pose, now);}

  if (isActive(state_)) {
    lock_ = cfg_.kinematics.require_lock ?
      kin_.checkLock(estimator_.joints(), reference_joints_) : LockStatus{};
    if (estop_ || guard_estop_) {
      finish(NavStateId::Failed, "guard_estop", now);
    } else if (guard_manual_) {
      finish(NavStateId::Canceled, "manual_override", now);
    } else if (cfg_.manager.require_lio && !st.valid) {
      finish(NavStateId::Failed, "lio_lost", now);
    } else if (!lock_.locked) {
      finish(NavStateId::Failed, "body_moved", now);
    } else if (now - goal_start_t_ > goal_req_->timeout_s) {
      finish(NavStateId::Failed, "timeout", now);
    } else if (state_ == NavStateId::Rotating) {
      bool done = false;
      out.cmd = rotateTowards(st, goal_.yaw, &done);
      if (done) {
        finish(NavStateId::Succeeded, "", now);
        out.cmd = Twist2D{};
      } else if (out.cmd.w == 0.0) {
        if (rotate_blocked_since_ < 0.0) {rotate_blocked_since_ = now;}
        if (now - rotate_blocked_since_ > 3.0) {finish(NavStateId::Failed, "blocked", now);}
      } else {
        rotate_blocked_since_ = -1.0;
      }
    } else if (state_ == NavStateId::Recovering) {
      out.cmd = stepRecovery(st, now);
    } else {
      out.cmd = stepNavigate(st, now);
    }
    if (!isActive(state_)) {out.cmd = Twist2D{};}
  }
  out.publish_cmd = isActive(state_) || (now - finish_t_) < 1.0;

  if (now - last_status_t_ >= cfg_.manager.status_period_s || state_ != last_status_state_) {
    out.status_json = statusJson(now).dump();
    last_status_t_ = now;
    last_status_state_ = state_;
  }
  return out;
}

nlohmann::json NavCore::statusJson(double now) const
{
  nlohmann::json j;
  j["goal_id"] = goal_req_ ? goal_req_->id : "";
  j["state"] = toString(state_);
  j["reason"] = reason_;
  j["partial"] = goal_partial_;
  j["adjusted"] = goal_adjusted_;
  const NavState & st = last_state_;
  const bool has_goal = goal_req_.has_value() && state_ != NavStateId::Idle;
  j["distance_remaining"] = has_goal ? st.pose.distanceTo(goal_) : 0.0;
  j["yaw_error"] = has_goal ? wrapAngle(goal_.yaw - st.pose.yaw) : 0.0;
  j["goal"] = {{"x", goal_.x}, {"y", goal_.y}, {"yaw", goal_.yaw}};
  j["pose"] = {{"x", st.pose.x}, {"y", st.pose.y}, {"yaw", st.pose.yaw}};
  j["velocity"] = {{"v", st.velocity.v}, {"w", st.velocity.w}};
  if (st.valid) {
    nlohmann::json fs;
    for (const auto & kv : freeSectors(st.pose)) {fs[std::to_string(kv.first)] = kv.second;}
    j["free_sectors"] = fs;
  }
  j["lio_ok"] = st.lio_ok;
  j["state_valid"] = st.valid;
  j["state_source"] = st.source;
  j["odom_consistent"] = st.consistent;
  j["body_locked"] = lock_.locked;
  j["lock_reasons"] = lock_.reasons;
  j["guard"] = {{"blocked", guard_blocked_}, {"estop", guard_estop_ || estop_}, {"manual", guard_manual_}};
  j["recoveries"] = recoveries_;
  j["path_points"] = path_.size();
  j["stamp"] = now;
  return j;
}

namespace
{
nlohmann::json posesXY(const std::vector<Pose2D> & poses, size_t max_points)
{
  nlohmann::json a = nlohmann::json::array();
  if (poses.empty()) {return a;}
  const size_t step = std::max<size_t>(1, poses.size() / max_points);
  for (size_t i = 0; i < poses.size(); i += step) {a.push_back({poses[i].x, poses[i].y});}
  if ((poses.size() - 1) % step != 0) {a.push_back({poses.back().x, poses.back().y});}
  return a;
}
}  // namespace

nlohmann::json NavCore::debugJson(double now) const
{
  nlohmann::json j;
  j["stamp"] = now;
  j["state"] = toString(state_);
  const Eigen::Isometry3d T = estimator_.T_odom_lio();
  const Eigen::Quaterniond q(T.linear());
  j["T_odom_lio"] = {
    {"valid", estimator_.initialized()},
    {"t", {T.translation().x(), T.translation().y(), T.translation().z()}},
    {"q", {q.x(), q.y(), q.z(), q.w()}}};
  const bool active = isActive(state_);
  if (goal_req_ && state_ != NavStateId::Idle) {
    j["goal"] = {{"id", goal_req_->id}, {"mode", goal_req_->mode}, {"x", goal_.x}, {"y", goal_.y},
      {"yaw", goal_.yaw}, {"use_yaw", goal_req_->use_yaw}, {"adjusted", goal_adjusted_},
      {"xy_tolerance", goal_req_->xy_tolerance}};
  } else {
    j["goal"] = nullptr;
  }
  j["global_path"] = active ? posesXY(path_, 300) : nlohmann::json::array();
  j["local_path"] = active ? posesXY(controller_->localPath(), 100) : nlohmann::json::array();
  nlohmann::json traj = nlohmann::json::array();
  const Eigen::ArrayXXf & o = controller_->optimalTrajectory();
  if (active && o.cols() >= 2) {
    for (Eigen::Index i = 0; i < o.rows(); i += 2) {traj.push_back({o(i, 0), o(i, 1)});}
  }
  j["mppi_traj"] = traj;
  j["speed_limit"] = effectiveSpeedLimit();
  return j;
}

double NavCore::effectiveSpeedLimit() const
{
  double v = default_max_speed_;
  if (goal_req_ && goal_req_->max_speed > 0.0) {
    v = v > 0.0 ? std::min(v, goal_req_->max_speed) : goal_req_->max_speed;
  }
  return v;
}

std::map<std::string, double *> NavCore::tunables()
{
  auto & m = cfg_.manager;
  return {
    {"approach_v_max", &m.approach_v_max}, {"simple_v_max", &m.simple_v_max},
    {"final_rotation_wmax", &m.final_rotation_wmax}, {"final_rotation_kp", &m.final_rotation_kp},
    {"progress_window_s", &m.progress_window_s}, {"progress_min_m", &m.progress_min_m},
    {"recovery_wait_s", &m.recovery_wait_s}, {"recovery_clear_radius_m", &m.recovery_clear_radius_m},
    {"recovery_rotate_rad", &m.recovery_rotate_rad}};
}

bool NavCore::setParam(const std::string & name, double value, std::string * error)
{
  auto fail = [&](const std::string & e) {
      if (error) {*error = e;}
      return false;
    };
  if (!std::isfinite(value) || value < 0.0) {return fail("value must be finite and >= 0");}
  if (name == "max_speed") {
    if (value > controller_->maxVx() + 1e-9) {
      return fail("max_speed above mppi.vx_max (" + std::to_string(controller_->maxVx()) + ")");
    }
    default_max_speed_ = value;
    if (isActive(state_)) {controller_->setSpeedLimit(effectiveSpeedLimit());}
    return true;
  }
  auto t = tunables();
  const auto it = t.find(name);
  if (it == t.end()) {return fail("unknown parameter '" + name + "'");}
  *it->second = value;
  return true;
}

nlohmann::json NavCore::paramsJson() const
{
  nlohmann::json j;
  for (const auto & kv : const_cast<NavCore *>(this)->tunables()) {j[kv.first] = *kv.second;}
  j["max_speed"] = default_max_speed_;
  j["mppi_vx_max"] = controller_->maxVx();
  j["mppi_wz_max"] = controller_->maxWz();
  return j;
}

}  // namespace g1d::nav
