#include <cmath>

#include "doctest.h"
#include "g1d/common/config.hpp"
#include "g1d/common/geometry.hpp"
#include "g1d/common/pose_buffer.hpp"
#include "g1d/common/time.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

using g1d::Pose2D;

TEST_CASE("wrapAngle")
{
  CHECK(std::abs(std::abs(g1d::wrapAngle(3 * M_PI)) - M_PI) < 1e-9);
  CHECK(g1d::wrapAngle(3 * M_PI) < M_PI);
  CHECK(g1d::wrapAngle(-3 * M_PI / 2) == doctest::Approx(M_PI / 2));
  CHECK(g1d::wrapAngle(0.3) == doctest::Approx(0.3));
}

TEST_CASE("Pose2D compose / inverse / relative")
{
  const Pose2D a(1.0, 2.0, M_PI / 2);
  const Pose2D rel(1.0, 0.0, M_PI / 2);
  const Pose2D b = a.compose(rel);
  CHECK(b.x == doctest::Approx(1.0));
  CHECK(b.y == doctest::Approx(3.0));
  CHECK(std::abs(g1d::wrapAngle(b.yaw - M_PI)) < 1e-9);
  const Pose2D back = a.relative(b);
  CHECK(back.x == doctest::Approx(rel.x));
  CHECK(back.y == doctest::Approx(rel.y));
  CHECK(std::abs(g1d::wrapAngle(back.yaw - rel.yaw)) < 1e-9);
  const Pose2D id = a.compose(a.inverse());
  CHECK(std::abs(id.x) < 1e-12);
  CHECK(std::abs(id.y) < 1e-12);
}

TEST_CASE("PoseBuffer interpolation and bounds")
{
  g1d::PoseBuffer buf(5.0);
  buf.add(10.0, Pose2D(0, 0, 0));
  buf.add(11.0, Pose2D(1, 0, M_PI / 2));
  auto mid = buf.lookup(10.5);
  REQUIRE(mid);
  CHECK(mid->x == doctest::Approx(0.5));
  CHECK(mid->yaw == doctest::Approx(M_PI / 4));
  CHECK_FALSE(buf.lookup(9.0));
  CHECK(buf.lookup(11.1, 0.2));
  CHECK_FALSE(buf.lookup(11.5, 0.2));
}

TEST_CASE("ClockMapper tracks minimum delay")
{
  g1d::ClockMapper m(10.0);
  // sensor clock = host - 100 s, with 1-5 ms transport jitter
  for (int i = 0; i < 100; ++i) {
    const double host = 1000.0 + i * 0.01;
    const double jitter = 0.001 + 0.004 * ((i * 37) % 11) / 10.0;
    m.observe(host - 100.0 - jitter, host);
  }
  CHECK(m.valid());
  CHECK(m.offset() == doctest::Approx(100.001).epsilon(1e-6));
  CHECK(m.toHost(5.0) == doctest::Approx(105.001).epsilon(1e-6));
}

TEST_CASE("YAML findPath / get / loadParameters")
{
  const YAML::Node root = YAML::Load(R"(
mppi:
  vx_max: 0.4
  time_steps: 56
  critics: [CostCritic, GoalCritic]
  CostCritic: {cost_weight: 3.81, consider_footprint: true}
  DiffDrive: {plugin: "mppi::DiffDriveMotionModel"}
)");
  CHECK(g1d::get(root, "mppi.vx_max", 0.0) == doctest::Approx(0.4));
  CHECK(g1d::get(root, "mppi.missing", 7) == 7);
  nav2::LifecycleNode node("t");
  g1d::loadParameters(node, root["mppi"], "FollowPath");
  float vx = 0;
  int steps = 0;
  std::vector<std::string> critics;
  bool fp = false;
  std::string plugin;
  CHECK(node.get_parameter("FollowPath.vx_max", vx));
  CHECK(vx == doctest::Approx(0.4f));
  CHECK(node.get_parameter("FollowPath.time_steps", steps));
  CHECK(steps == 56);
  CHECK(node.get_parameter("FollowPath.critics", critics));
  CHECK(critics.size() == 2);
  CHECK(node.get_parameter("FollowPath.CostCritic.consider_footprint", fp));
  CHECK(fp);
  CHECK(node.get_parameter("FollowPath.DiffDrive.plugin", plugin));
  CHECK(plugin == "mppi::DiffDriveMotionModel");
}
