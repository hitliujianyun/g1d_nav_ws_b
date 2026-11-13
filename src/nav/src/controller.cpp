#include "g1d/nav/controller.hpp"

#include <cmath>
#include <limits>

#include "g1d/common/config.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

// pluginlib XML alias used as the upstream default validator name.
namespace
{
const bool kValidatorAlias = []() {
    ::pluginlib::impl::registry<mppi::OptimalTrajectoryValidator>()[
      "mppi::DefaultOptimalTrajectoryValidator"] =
      []() -> mppi::OptimalTrajectoryValidator * {return new mppi::OptimalTrajectoryValidator();};
    return true;
  }();
}  // namespace

namespace g1d::nav
{
namespace
{
constexpr const char * kName = "FollowPath";

geometry_msgs::msg::PoseStamped toPoseStamped(const Pose2D & p)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "odom";
  ps.pose.position.x = p.x;
  ps.pose.position.y = p.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p.yaw);
  ps.pose.orientation = tf2::toMsg(q);
  return ps;
}
}  // namespace

ControllerParams ControllerParams::fromYaml(const YAML::Node & root)
{
  ControllerParams p;
  p.controller_frequency = get(root, "mppi.rate_hz", p.controller_frequency);
  p.prune_distance = get(root, "mppi.prune_distance", p.prune_distance);
  p.search_window = get(root, "controller.search_window", p.search_window);
  p.mppi = findPath(root, "mppi");
  return p;
}

MppiController::MppiController(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, const ControllerParams & params)
: p_(params), costmap_(std::move(costmap))
{
  (void)kValidatorAlias;
  node_ = std::make_shared<nav2::LifecycleNode>("g1d_mppi");
  // Defaults that a Nav2 controller_server configuration would normally provide.
  node_->set_parameter("controller_frequency", rclcpp::ParameterValue(p_.controller_frequency));
  node_->set_parameter(std::string(kName) + ".motion_model", rclcpp::ParameterValue("DiffDrive"));
  node_->set_parameter(
    std::string(kName) + ".DiffDrive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node_->set_parameter(
    std::string(kName) + ".TrajectoryValidator.plugin",
    rclcpp::ParameterValue("mppi::DefaultOptimalTrajectoryValidator"));
  node_->set_parameter(std::string(kName) + ".critics", rclcpp::ParameterValue(
      std::vector<std::string>{"ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
        "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"}));
  if (p_.mppi) {
    g1d::loadParameters(*node_, p_.mppi, kName);
  }
  // `rate_hz` in our YAML is the controller frequency, not an MPPI parameter.
  node_->set_parameter("controller_frequency", rclcpp::ParameterValue(p_.controller_frequency));

  std::string name = kName;
  param_handler_ = std::make_unique<mppi::ParametersHandler>(node_, name);
  optimizer_ = std::make_unique<mppi::Optimizer>();
  optimizer_->initialize(node_, name, costmap_, nullptr, param_handler_.get());
  param_handler_->start();

  node_->get_parameter(std::string(kName) + ".vx_max", vx_max_);
  node_->get_parameter(std::string(kName) + ".wz_max", wz_max_);
}

MppiController::~MppiController()
{
  if (optimizer_) {optimizer_->shutdown();}
}

void MppiController::reset()
{
  optimizer_->reset(false);
  closest_idx_ = 0;
}

void MppiController::setSpeedLimit(double v)
{
  if (v <= 0.0) {
    optimizer_->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  } else {
    optimizer_->setSpeedLimit(std::min(v, vx_max_), false);
  }
}

void MppiController::setPlan(const std::vector<Pose2D> & global_path)
{
  global_path_ = global_path;
  closest_idx_ = 0;
}

void MppiController::prunePath(const Pose2D & pose)
{
  // Nav2 controller_server path handling: find the closest pose within a search window ahead
  // of the last closest pose, then keep `prune_distance` of path inside the local costmap.
  local_path_.clear();
  if (global_path_.empty()) {return;}
  double best = std::numeric_limits<double>::max();
  size_t best_i = closest_idx_;
  double travelled = 0.0;
  for (size_t i = closest_idx_; i < global_path_.size(); ++i) {
    if (i > closest_idx_) {travelled += global_path_[i - 1].distanceTo(global_path_[i]);}
    if (travelled > p_.search_window) {break;}
    const double d = pose.distanceTo(global_path_[i]);
    if (d < best) {best = d; best_i = i;}
  }
  closest_idx_ = best_i;
  const auto * cm = costmap_->getCostmap();
  double length = 0.0;
  for (size_t i = closest_idx_; i < global_path_.size(); ++i) {
    if (i > closest_idx_) {length += global_path_[i - 1].distanceTo(global_path_[i]);}
    unsigned int mx, my;
    if (!cm->worldToMap(global_path_[i].x, global_path_[i].y, mx, my)) {break;}
    local_path_.push_back(global_path_[i]);
    if (length > p_.prune_distance) {break;}
  }
  if (local_path_.empty()) {local_path_.push_back(global_path_[closest_idx_]);}
}

bool MppiController::computeCommand(
  const Pose2D & pose, const Twist2D & velocity, const Pose2D & goal, Twist2D & cmd,
  std::string * error)
{
  prunePath(pose);
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "odom";
  for (const auto & p : local_path_) {plan.poses.push_back(toPoseStamped(p));}
  geometry_msgs::msg::Twist speed;
  speed.linear.x = velocity.v;
  speed.angular.z = velocity.w;
  const auto robot = toPoseStamped(pose);
  const auto goal_ps = toPoseStamped(goal);

  std::lock_guard<std::mutex> param_lock(*param_handler_->getLock());
  auto * cm = costmap_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lk(*cm->getMutex());
  try {
    auto [twist, traj] = optimizer_->evalControl(robot, speed, plan, goal_ps.pose, nullptr);
    cmd.v = twist.twist.linear.x;
    cmd.w = twist.twist.angular.z;
    optimal_ = traj;
    return true;
  } catch (const nav2_core::NoValidControl & e) {
    if (error) {*error = e.what();}
  } catch (const std::exception & e) {
    if (error) {*error = e.what();}
  }
  cmd = Twist2D{};
  return false;
}

}  // namespace g1d::nav
