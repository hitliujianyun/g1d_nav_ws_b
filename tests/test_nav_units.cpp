#include <cmath>

#include "doctest.h"
#include "g1d/common/config.hpp"
#include "g1d/nav/body_kinematics.hpp"
#include "g1d/nav/controller.hpp"
#include "g1d/nav/costmap/layered_costmap.hpp"
#include "g1d/nav/planner.hpp"
#include "g1d/nav/state_estimator.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace g1d;
using namespace g1d::nav;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace
{
YAML::Node projectConfig() {return loadYamlFile(G1D_SOURCE_DIR "/config/g1d_nav.yaml");}

void addBoxPoints(
  std::vector<Eigen::Vector3f> & pts, double x0, double x1, double y0, double y1, double z0,
  double z1, double step = 0.04)
{
  for (double x = x0; x <= x1; x += step) {
    for (double y = y0; y <= y1; y += step) {
      for (double z = z0; z <= z1; z += step) {pts.emplace_back(x, y, z);}
    }
  }
}
}  // namespace

TEST_CASE("BodyKinematics follows the URDF chain")
{
  KinematicsParams p;
  BodyKinematics kin(p);
  JointState q;
  const Eigen::Isometry3d T0 = kin.baseToHead(q);
  // Sum of URDF z offsets from AGV_link to head_link.
  CHECK(T0.translation().z() == doctest::Approx(0.5275 + 0.01 + 0.018 + 0.008 + 0.066 + 0.104008566 - 0.054).epsilon(1e-6));
  CHECK(T0.translation().x() == doctest::Approx(0.0573).epsilon(0.01));
  q.lift_height = 0.30;
  CHECK(kin.baseToHead(q).translation().z() == doctest::Approx(T0.translation().z() + 0.30).epsilon(1e-6));
  q.lift_height = 1.0;  // clamped to the 0.42 m stroke
  CHECK(kin.baseToHead(q).translation().z() == doctest::Approx(T0.translation().z() + 0.42).epsilon(1e-6));
  JointState pitched;
  pitched.waist_pitch = 0.5;
  CHECK(kin.baseToHead(pitched).translation().x() > T0.translation().x());
}

TEST_CASE("BodyKinematics lock check")
{
  KinematicsParams p;
  BodyKinematics kin(p);
  JointState ref;
  ref.has_lift = ref.has_lowstate = ref.has_waist_pitch = true;
  JointState q = ref;
  CHECK(kin.checkLock(q, ref).locked);
  q.waist_yaw = 3.0 * M_PI / 180.0;
  auto s = kin.checkLock(q, ref);
  CHECK_FALSE(s.locked);
  CHECK(s.reasons.front() == "waist_yaw");
  q = ref;
  q.lift_speed = 0.02;
  CHECK_FALSE(kin.checkLock(q, ref).locked);
}

TEST_CASE("StateEstimator anchors odom at the first LIO pose and fuses wheel odometry")
{
  KinematicsParams kp;
  kp.T_head_livox = makeIsometry(0.05, 0, 0.55, M_PI, 0, 0);
  BodyKinematics kin(kp);
  StateEstimator est(StateEstimatorParams{}, kin);
  const Eigen::Isometry3d T_base_imu = kin.baseToImu(JointState{});
  // Robot starts at world (2, 1, 0.5 rad) and drives straight at 0.4 m/s.
  const Pose2D start(2.0, 1.0, 0.5);
  auto world_base = [&](double t) {return start.compose(Pose2D(0.4 * t, 0.0, 0.0));};
  const Eigen::Isometry3d T_w_lio = toIsometry(world_base(0.0)) * T_base_imu;
  for (int i = 0; i <= 20; ++i) {
    const double t = 0.1 * i;
    const Pose2D wb = world_base(t);
    est.onWheelOdom(100.0 + t, Pose2D(10 + 0.4 * t, 0, 0), 0.4, 0.0);  // wheel frame arbitrary
    LioInput in;
    in.stamp = 100.0 + t;
    in.ok = true;
    in.T_lio_imu = T_w_lio.inverse() * toIsometry(wb) * T_base_imu;
    est.onLio(in);
  }
  NavState s = est.estimate(102.05);
  REQUIRE(s.valid);
  CHECK(s.lio_ok);
  CHECK(s.source == "lio+wheel");
  CHECK(s.pose.x == doctest::Approx(0.4 * 2.05).epsilon(0.01));
  CHECK(std::abs(s.pose.y) < 1e-3);
  CHECK(std::abs(s.pose.yaw) < 1e-3);
  CHECK(s.velocity.v == doctest::Approx(0.4));
  // Stale LIO -> invalid unless wheel-only is allowed.
  CHECK_FALSE(est.estimate(104.0).valid);
}

TEST_CASE("VoxelLayer: mark, 3D ray clearing keeps overhangs, inflation")
{
  costmap::LayeredCostmapParams p = costmap::LayeredCostmapParams::fromYaml(projectConfig(), "local");
  costmap::LayeredCostmap cm(p);
  const Pose2D robot(0, 0, 0);
  const Eigen::Vector3d sensor(0.05, 0.0, 1.23);
  std::vector<Eigen::Vector3f> pts;
  addBoxPoints(pts, 1.5, 1.7, -0.2, 0.2, 0.1, 0.5);            // low box
  addBoxPoints(pts, 2.2, 2.8, 0.6, 1.2, 0.72, 0.76, 0.03);     // table top (overhang)
  cm.addCloud(pts, sensor, 1.0);
  cm.update(robot, 1.0);
  CHECK(cm.costAt(1.6, 0.0) == LETHAL_OBSTACLE);
  CHECK(cm.costAt(2.5, 0.9) == LETHAL_OBSTACLE);
  // Inflation decreases with distance and stays below INSCRIBED outside the inscribed radius.
  const unsigned char c1 = cm.costAt(1.45, 0.0);
  const unsigned char c2 = cm.costAt(1.2, 0.0);
  CHECK(c1 > c2);
  CHECK(c2 > 0);
  CHECK(cm.costAt(0.0, -2.0) == 0);

  // Floor returns behind the table: rays pass under the table top (z < 0.72) -> it survives.
  std::vector<Eigen::Vector3f> floor;
  for (double x = 3.0; x < 4.0; x += 0.05) {floor.emplace_back(x, 0.95, 0.0);}
  cm.addCloud(floor, sensor, 1.1);
  cm.update(robot, 1.1);
  CHECK(cm.costAt(2.5, 0.9) == LETHAL_OBSTACLE);

  // The box is gone: from 1.23 m the sensor now sees the floor behind it; those rays cross the
  // box volume (wall returns would pass above a 0.5 m box) and clear the column.
  std::vector<Eigen::Vector3f> behind;
  for (double x = 1.55; x <= 2.8; x += 0.02) {
    for (double y = -0.2; y <= 0.2; y += 0.02) {behind.emplace_back(x, y, 0.0);}
  }
  for (double y = -0.2; y <= 0.2; y += 0.02) {
    for (double z = 0.1; z <= 0.5; z += 0.05) {behind.emplace_back(2.9, y, z);}  // wall
  }
  cm.addCloud(behind, sensor, 1.2);
  cm.update(robot, 1.2);
  CHECK(cm.costAt(1.6, 0.0) < LETHAL_OBSTACLE);
  CHECK(cm.costAt(2.9, 0.0) == LETHAL_OBSTACLE);
}

TEST_CASE("InflationLayer cost function matches Nav2")
{
  costmap::InflationLayer inf;
  inf.configure(costmap::InflationParams{0.55, 3.0}, 0.05, 0.28);
  CHECK(inf.computeCost(0) == LETHAL_OBSTACLE);
  CHECK(inf.computeCost(5) == INSCRIBED_INFLATED_OBSTACLE);   // 0.25 m < inscribed
  const double expect = 252.0 * std::exp(-3.0 * (0.5 - 0.28));
  CHECK(std::abs(inf.computeCost(10) - static_cast<unsigned char>(expect)) <= 1);
  CHECK(inf.getInflationRadius() == doctest::Approx(0.55));
}

TEST_CASE("GlobalPlanner plans around a wall and sanitises goals")
{
  costmap::LayeredCostmapParams p = costmap::LayeredCostmapParams::fromYaml(projectConfig(), "global");
  costmap::LayeredCostmap cm(p);
  std::vector<Eigen::Vector3f> pts;
  addBoxPoints(pts, 2.0, 2.2, -3.0, 1.0, 0.2, 0.6);   // wall with a gap above y = 1
  addBoxPoints(pts, 5.0, 5.6, -0.3, 0.3, 0.2, 0.6);   // box at the goal
  cm.addCloud(pts, Eigen::Vector3d(0, 0, 1.2), 1.0);
  cm.update(Pose2D(0, 0, 0), 1.0);

  GlobalPlanner planner(PlannerParams::fromYaml(projectConfig()));
  std::vector<Pose2D> path;
  std::string err;
  REQUIRE(planner.makePlan(*cm.costmap(), Pose2D(0, 0, 0), Pose2D(4.0, 0, 0), false, path, &err));
  double max_y = -1e9;
  for (const auto & q : path) {
    CHECK(cm.costAt(q.x, q.y) < LETHAL_OBSTACLE);
    max_y = std::max(max_y, q.y);
  }
  CHECK(max_y > 1.0);   // went around the wall end
  CHECK(path.back().distanceTo(Pose2D(4.0, 0, 0)) < 0.15);
  CHECK(pathLength(path) > 4.0);

  const GoalCheck g = planner.sanitizeGoal(*cm.costmap(), Pose2D(3.5, 0, 0), Pose2D(5.3, 0, 0));
  CHECK(g.ok);
  CHECK(g.adjusted);
  CHECK(cm.costAt(g.goal.x, g.goal.y) <= 128);
  CHECK(g.goal.x < 5.0);

  const GoalCheck far = planner.sanitizeGoal(*cm.costmap(), Pose2D(0, 0, 0), Pose2D(0, -25, 0));
  CHECK(far.partial);
  CHECK(far.ok);
}

TEST_CASE("MppiController drives a straight path on an empty map")
{
  const YAML::Node cfg = projectConfig();
  costmap::LayeredCostmap cm(costmap::LayeredCostmapParams::fromYaml(cfg, "local"));
  MppiController mppi(cm.costmapRos(), ControllerParams::fromYaml(cfg));
  std::vector<Pose2D> path;
  for (double x = 0; x <= 3.0; x += 0.05) {path.emplace_back(x, 0.0, 0.0);}
  mppi.setPlan(path);
  Pose2D pose(0, 0, 0);
  Twist2D vel;
  double max_v = 0.0;
  for (int i = 0; i < 160; ++i) {
    cm.update(pose, 0.05 * i);
    Twist2D cmd;
    std::string err;
    REQUIRE(mppi.computeCommand(pose, vel, path.back(), cmd, &err));
    vel = cmd;
    pose = pose.compose(Pose2D(cmd.v * 0.05, 0.0, cmd.w * 0.05));
    max_v = std::max(max_v, cmd.v);
  }
  CHECK(max_v <= mppi.maxVx() + 1e-3);
  CHECK(max_v > 0.25);
  CHECK(pose.x > 2.6);
  CHECK(std::abs(pose.y) < 0.15);
}
