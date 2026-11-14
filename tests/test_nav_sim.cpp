// Closed-loop navigation scenarios: NavCore + simulated G1-D (Mid-360S on the head, chassis
// LiDAR, wheel odometry) in box worlds. Collision checks use the whole robot height.

#include <cstdio>

#include "doctest.h"
#include "g1d/common/config.hpp"
#include "g1d/sim/nav_sim.hpp"

using namespace g1d;
using namespace g1d::sim;
using nlohmann::json;

namespace
{
YAML::Node projectConfig() {return loadYamlFile(G1D_SOURCE_DIR "/config/g1d_nav.yaml");}

void report(const char * name, const NavSimResult & r)
{
  std::printf(
    "[nav_sim] %-28s state=%s reason='%s' t=%.1fs collided=%d min_clear=%.2fm "
    "final=(%.2f,%.2f,%.2f) path=%.1fm vmax=%.2f tick=%.1fms\n",
    name, r.final_state.c_str(), r.reason.c_str(), r.time_s, r.collided, r.min_clearance,
    r.final_pose.x, r.final_pose.y, r.final_pose.yaw, r.path_length, r.max_speed, r.mean_tick_ms);
}

World emptyRoom()
{
  World w;
  w.addRoom(-2.0, -4.0, 10.0, 4.0, 2.8);
  return w;
}
}  // namespace

TEST_CASE("navigate around a box on the straight line")
{
  World w = emptyRoom();
  w.addBox({2.8, -0.45, 0.0}, {3.4, 0.45, 0.7}, "box");
  NavSimulation sim(w, projectConfig(), Pose2D(0, 0, 0));
  sim.warmup(1.0);
  sim.sendGoal({{"id", "g1"}, {"x", 6.0}, {"y", 0.0}, {"yaw", 0.0}});
  const NavSimResult r = sim.runUntilDone(60.0);
  report("box_on_line", r);
  CHECK(r.final_state == "SUCCEEDED");
  CHECK_FALSE(r.collided);
  CHECK(r.final_pose.distanceTo(Pose2D(6.0, 0.0, 0.0)) < 0.3);
  CHECK(std::abs(wrapAngle(r.final_pose.yaw)) < 0.3);
}

TEST_CASE("table overhang invisible to the chassis LiDAR is avoided")
{
  World w = emptyRoom();
  // Table top 0.72-0.76 m across the straight line; only thin legs reach the 0.1 m scan plane.
  w.addBox({2.0, -0.7, 0.72}, {3.2, 0.7, 0.76}, "table_top");
  w.addBox({2.0, -0.7, 0.0}, {2.05, -0.65, 0.72}, "leg1");
  w.addBox({3.15, -0.7, 0.0}, {3.2, -0.65, 0.72}, "leg2");
  w.addBox({2.0, 0.65, 0.0}, {2.05, 0.7, 0.72}, "leg3");
  w.addBox({3.15, 0.65, 0.0}, {3.2, 0.7, 0.72}, "leg4");
  NavSimulation sim(w, projectConfig(), Pose2D(0, 0, 0));
  sim.warmup(1.0);
  sim.sendGoal({{"id", "g2"}, {"x", 5.5}, {"y", 0.0}});
  const NavSimResult r = sim.runUntilDone(60.0);
  report("table_overhang", r);
  CHECK(r.final_state == "SUCCEEDED");
  CHECK_FALSE(r.collided);
}

TEST_CASE("furnished room: two consecutive NavGPT-style relative waypoints")
{
  NavSimulation sim(World::makeTestRoom(), projectConfig(), Pose2D(0, 0, 0));
  sim.warmup(1.0);
  sim.sendGoal({{"id", "w1"}, {"frame", "base_at_stamp"}, {"stamp", sim.time()},
      {"x", 4.0}, {"y", -0.5}});
  int k = 0;
  NavSimResult r = sim.runUntilDone(60.0, [&](NavSimulation & s) {
        if (std::getenv("G1D_TRACE") && (++k % 20) == 0) {
          const auto & p = s.robot().pose();
          std::printf("  t=%.1f pose=(%.2f,%.2f,%.2f) v=%.2f w=%.2f state=%s\n", k * 0.05, p.x, p.y,
          p.yaw, s.robot().v(), s.robot().w(), nav::toString(s.core().state()));
        }
      });
  report("room_wp1", r);
  CHECK(r.final_state == "SUCCEEDED");
  CHECK_FALSE(r.collided);
  // Second waypoint relative to the pose 0.3 s ago (image capture latency).
  sim.sendGoal({{"id", "w2"}, {"frame", "base_at_stamp"}, {"stamp", sim.time() - 0.3},
      {"x", 2.5}, {"y", -2.0}, {"yaw", -1.2}});
  r = sim.runUntilDone(60.0);
  report("room_wp2", r);
  CHECK(r.final_state == "SUCCEEDED");
  CHECK_FALSE(r.collided);
}

TEST_CASE("goal inside an obstacle is moved out")
{
  World w = emptyRoom();
  w.addBox({4.0, -0.4, 0.0}, {4.8, 0.4, 1.0}, "crate");
  NavSimulation sim(w, projectConfig(), Pose2D(0, 0, 0));
  sim.warmup(1.5);
  sim.sendGoal({{"id", "g3"}, {"x", 4.4}, {"y", 0.0}});
  const NavSimResult r = sim.runUntilDone(60.0);
  report("goal_in_obstacle", r);
  CHECK_FALSE(r.collided);
  CHECK(r.final_state == "SUCCEEDED");
  CHECK(r.last_status.value("adjusted", false));
  CHECK(r.final_pose.x < 4.0);
}

TEST_CASE("rotate mode")
{
  NavSimulation sim(emptyRoom(), projectConfig(), Pose2D(1, 0, 0));
  sim.warmup(1.0);
  sim.sendGoal({{"id", "r1"}, {"mode", "rotate"}, {"yaw", 1.57}});
  const NavSimResult r = sim.runUntilDone(20.0);
  report("rotate", r);
  CHECK(r.final_state == "SUCCEEDED");
  CHECK(std::abs(wrapAngle(r.final_pose.yaw - 1.57)) < 0.3);
  CHECK(r.final_pose.distanceTo(Pose2D(1, 0, 0)) < 0.05);
}

TEST_CASE("simple mode: free path succeeds, blocked path fails with free sectors")
{
  {
    NavSimulation sim(emptyRoom(), projectConfig(), Pose2D(0, 0, 0));
    sim.warmup(1.0);
    sim.sendGoal({{"id", "s1"}, {"mode", "simple"}, {"x", 2.0}, {"y", 1.0}});
    const NavSimResult r = sim.runUntilDone(30.0);
    report("simple_free", r);
    CHECK(r.final_state == "SUCCEEDED");
    CHECK_FALSE(r.collided);
  }
  {
    World w = emptyRoom();
    w.addBox({1.2, -0.6, 0.0}, {1.6, 0.6, 1.0}, "block");
    NavSimulation sim(w, projectConfig(), Pose2D(0, 0, 0));
    sim.warmup(1.0);
    sim.sendGoal({{"id", "s2"}, {"mode", "simple"}, {"x", 3.0}, {"y", 0.0}});
    const NavSimResult r = sim.runUntilDone(30.0);
    report("simple_blocked", r);
    CHECK(r.final_state == "FAILED");
    CHECK(r.reason == "blocked");
    CHECK_FALSE(r.collided);
    REQUIRE(r.last_status.contains("free_sectors"));
    CHECK(r.last_status["free_sectors"]["0"] == false);
  }
}

TEST_CASE("LIO loss stops navigation")
{
  NavSimulation sim(emptyRoom(), projectConfig(), Pose2D(0, 0, 0));
  sim.warmup(1.0);
  sim.sendGoal({{"id", "l1"}, {"x", 6.0}, {"y", 0.0}});
  int steps = 0;
  const NavSimResult r = sim.runUntilDone(30.0, [&](NavSimulation & s) {
        if (++steps == 40) {s.setLioEnabled(false);}
      });
  report("lio_lost", r);
  CHECK(r.final_state == "FAILED");
  CHECK(r.reason == "lio_lost");
  CHECK(r.final_pose.x < 3.0);
}

TEST_CASE("dead end: no path to a goal behind a closed wall")
{
  World w = emptyRoom();
  w.addBox({3.0, -4.0, 0.0}, {3.3, 4.0, 2.0}, "closed_wall");
  NavSimulation sim(w, projectConfig(), Pose2D(0, 0, 0));
  sim.warmup(1.5);
  sim.sendGoal({{"id", "d1"}, {"x", 5.0}, {"y", 0.0}});
  const NavSimResult r = sim.runUntilDone(40.0);
  report("dead_end", r);
  CHECK(r.final_state == "FAILED");
  CHECK_FALSE(r.collided);
}

TEST_CASE("debug view and runtime parameters (web UI)")
{
  NavSimulation sim(emptyRoom(), projectConfig(), Pose2D(0, 0, 0));
  sim.warmup(1.0);
  std::string err;
  CHECK_FALSE(sim.core().setParam("no_such_param", 1.0, &err));
  CHECK(err.find("unknown") != std::string::npos);
  CHECK_FALSE(sim.core().setParam("max_speed", 5.0, &err));
  CHECK_FALSE(sim.core().setParam("approach_v_max", -1.0, &err));
  REQUIRE(sim.core().setParam("max_speed", 0.2, &err));
  REQUIRE(sim.core().setParam("approach_v_max", 0.15, &err));
  const json params = sim.core().paramsJson();
  CHECK(params["max_speed"].get<double>() == doctest::Approx(0.2));
  CHECK(params["approach_v_max"].get<double>() == doctest::Approx(0.15));

  sim.sendGoal({{"id", "p1"}, {"frame", "odom"}, {"x", 4.0}, {"y", 0.5}});
  json mid;
  const NavSimResult r = sim.runUntilDone(40.0, [&](NavSimulation & s) {
        if (mid.is_null() && s.time() > 1004.0) {mid = s.core().debugJson(s.time());}
      });
  report("debug_params", r);
  CHECK(r.final_state == "SUCCEEDED");
  CHECK(r.max_speed <= 0.2 + 0.02);
  REQUIRE_FALSE(mid.is_null());
  CHECK(mid["T_odom_lio"]["valid"].get<bool>());
  CHECK(mid["goal"]["x"].get<double>() == doctest::Approx(4.0));
  CHECK(mid["global_path"].size() >= 2);
  CHECK(mid["mppi_traj"].size() >= 5);
  CHECK(mid["speed_limit"].get<double>() == doctest::Approx(0.2));
  CHECK(sim.core().debugJson(sim.time())["global_path"].empty());
}
