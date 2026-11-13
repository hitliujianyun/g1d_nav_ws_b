// Offline closed-loop demo: runs NavCore against the simulated G1-D and dumps a JSON trace
// (world boxes, trajectory, commands, global path, final global costmap) for
// tools/plot_sim_trace.py. Runs on any host, no DDS required. The robot starts at the world
// origin, so frame "odom" goals are world coordinates.
//
//   g1d_sim_demo [--config g1d_nav.yaml] [--scenario room|box|table|deadend] [--out trace.json]

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#include "g1d/common/config.hpp"
#include "g1d/sim/nav_sim.hpp"

using namespace g1d;
using namespace g1d::sim;
using nlohmann::json;

namespace
{
struct Scenario
{
  World world;
  Pose2D start;
  std::vector<json> goals;
};

Scenario makeScenario(const std::string & name)
{
  Scenario s;
  if (name == "room") {
    s.world = World::makeTestRoom();
    s.goals.push_back({{"id", "w1"}, {"frame", "odom"}, {"x", 3.5}, {"y", 1.5}, {"yaw", 1.57}});
    s.goals.push_back({{"id", "w2"}, {"frame", "odom"}, {"x", 6.0}, {"y", -2.0}});
    s.goals.push_back({{"id", "w3"}, {"frame", "odom"}, {"x", 0.5}, {"y", 0.0}, {"yaw", 3.14}});
  } else if (name == "table") {
    s.world.addRoom(-2.0, -4.0, 10.0, 4.0, 2.8);
    s.world.addBox({2.0, -0.7, 0.72}, {3.2, 0.7, 0.76}, "table_top");
    s.world.addBox({2.0, -0.7, 0.0}, {2.05, -0.65, 0.72}, "leg1");
    s.world.addBox({3.15, -0.7, 0.0}, {3.2, -0.65, 0.72}, "leg2");
    s.world.addBox({2.0, 0.65, 0.0}, {2.05, 0.7, 0.72}, "leg3");
    s.world.addBox({3.15, 0.65, 0.0}, {3.2, 0.7, 0.72}, "leg4");
    s.goals.push_back({{"id", "t1"}, {"x", 5.5}, {"y", 0.0}});
  } else if (name == "deadend") {
    s.world.addRoom(-2.0, -4.0, 10.0, 4.0, 2.8);
    s.world.addBox({3.0, -4.0, 0.0}, {3.3, 4.0, 2.0}, "closed_wall");
    s.goals.push_back({{"id", "d1"}, {"x", 5.0}, {"y", 0.0}});
  } else {
    s.world.addRoom(-2.0, -4.0, 10.0, 4.0, 2.8);
    s.world.addBox({2.8, -0.45, 0.0}, {3.4, 0.45, 0.7}, "box");
    s.goals.push_back({{"id", "b1"}, {"x", 6.0}, {"y", 0.0}, {"yaw", 0.0}});
  }
  return s;
}

json costmapJson(const nav2_costmap_2d::Costmap2D & c)
{
  json j;
  j["origin"] = {c.getOriginX(), c.getOriginY()};
  j["resolution"] = c.getResolution();
  j["size"] = {c.getSizeInCellsX(), c.getSizeInCellsY()};
  const unsigned char * d = c.getCharMap();
  std::vector<int> cells(d, d + c.getSizeInCellsX() * c.getSizeInCellsY());
  j["data"] = cells;
  return j;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string config_path = "config/g1d_nav.yaml";
  std::string scenario = "room";
  std::string out_path = "sim_trace.json";
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--config")) {config_path = argv[i + 1];}
    else if (!std::strcmp(argv[i], "--scenario")) {scenario = argv[i + 1];}
    else if (!std::strcmp(argv[i], "--out")) {out_path = argv[i + 1];}
  }

  Scenario s = makeScenario(scenario);
  NavSimulation sim(s.world, loadYamlFile(config_path), s.start);
  sim.warmup(1.0);

  json trace;
  trace["scenario"] = scenario;
  for (const auto & b : s.world.boxes()) {
    trace["boxes"].push_back(
      {{"name", b.name}, {"min", {b.min.x(), b.min.y(), b.min.z()}},
        {"max", {b.max.x(), b.max.y(), b.max.z()}}});
  }

  bool all_ok = true;
  for (auto goal : s.goals) {
    if (!goal.contains("frame")) {goal["frame"] = "odom";}
    json gtrace;
    gtrace["goal"] = goal;
    sim.sendGoal(goal);
    double next_sample = sim.time();
    const NavSimResult r = sim.runUntilDone(90.0, [&](NavSimulation & ns) {
        const Pose2D & p = ns.robot().pose();
        gtrace["traj"].push_back({ns.time(), p.x, p.y, p.yaw, ns.robot().v(), ns.robot().w()});
        if (ns.time() >= next_sample) {
          next_sample += 1.0;
          json path = json::array();
          for (const auto & q : ns.core().globalPath()) {path.push_back({q.x, q.y});}
          gtrace["samples"].push_back(
            {{"t", ns.time()}, {"status", ns.core().statusJson(ns.time())}, {"path", path}});
          if (!path.empty() && !gtrace.contains("global_path")) {gtrace["global_path"] = path;}
        }
      });
    gtrace["result"] = {
      {"state", r.final_state}, {"reason", r.reason}, {"time_s", r.time_s},
      {"collided", r.collided}, {"min_clearance", r.min_clearance},
      {"path_length", r.path_length}, {"max_speed", r.max_speed},
      {"mean_tick_ms", r.mean_tick_ms}};
    std::printf(
      "[%s] goal %s -> state=%s reason='%s' t=%.1fs collided=%d min_clear=%.2f path=%.1fm\n",
      scenario.c_str(), goal.value("id", "").c_str(), r.final_state.c_str(), r.reason.c_str(),
      r.time_s, r.collided, r.min_clearance, r.path_length);
    all_ok = all_ok && !r.collided;
    trace["goals"].push_back(gtrace);
  }
  trace["global_costmap"] = costmapJson(*sim.core().globalCostmap().costmap());

  std::ofstream(out_path) << trace.dump();
  std::printf("trace written to %s\n", out_path.c_str());
  return all_ok ? 0 : 1;
}
