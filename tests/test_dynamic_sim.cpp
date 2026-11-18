// Moving obstacles in closed loop: ghost clearing after a pedestrian passes, navigation through a
// crossing pedestrian, and persistence of obstacles that drop out of the Mid-360S field of view.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

#include "doctest.h"
#include "g1d/common/config.hpp"
#include "g1d/sim/nav_sim.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace g1d;
using namespace g1d::sim;
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace
{
YAML::Node projectConfig() {return loadYamlFile(G1D_SOURCE_DIR "/config/g1d_nav.yaml");}

World roomWithPedestrian(double px, double py)
{
  World w;
  w.addRoom(-2.0, -4.0, 10.0, 4.0, 2.8);
  w.addBox({px - 0.2, py - 0.2, 0.0}, {px + 0.2, py + 0.2, 1.7}, "ped");
  return w;
}

struct GhostStats
{
  double max_age{0.0};       // longest time a cell stayed lethal after the pedestrian left it
  int ghost_samples{0};      // (cell, step) pairs lethal > 0.3 s after the pedestrian left
};

enum class Mode {Baseline, SeeThrough, SeeThroughLabels};

const char * modeName(Mode m)
{
  switch (m) {
    case Mode::Baseline: return "rays_only";
    case Mode::SeeThrough: return "see_through";
    case Mode::SeeThroughLabels: return "see_through+labels";
  }
  return "";
}

/// Stationary robot at the origin; a pedestrian walks across x = 2 m from y = -2.5 to +2.5 m.
GhostStats runCrossing(Mode mode)
{
  YAML::Node cfg = projectConfig();
  if (mode == Mode::Baseline) {cfg["costmap"]["common"]["voxel"]["see_through_clear"] = false;}
  NavSimParams sp;
  if (mode == Mode::SeeThroughLabels) {sp.labeler = truthLabeler();}
  NavSimulation sim(roomWithPedestrian(2.0, -2.5), cfg, Pose2D(0, 0, 0), sp);
  sim.warmup(1.0);

  const double speed = 1.0, x_ped = 2.0, half = 0.2, cover = half + 0.1;
  const double t0 = sim.time();
  std::map<std::pair<int, int>, double> t_left;
  GhostStats g;
  auto & cm = sim.core().localCostmap();
  while (sim.time() - t0 < 8.0) {
    const double y_ped = std::min(-2.5 + speed * (sim.time() - t0), 3.0);
    moveBox(sim.world(), "ped", x_ped, y_ped);
    sim.step();
    const double t = sim.time();
    for (int ix = 0; ix <= 16; ++ix) {
      for (int iy = 0; iy <= 88; ++iy) {
        const double x = 1.6 + 0.05 * ix, y = -2.2 + 0.05 * iy;
        const bool covered = std::abs(x - x_ped) <= cover && std::abs(y - y_ped) <= cover;
        if (covered) {t_left[{ix, iy}] = t; continue;}
        if (cm.costAt(x, y) != LETHAL_OBSTACLE) {continue;}
        const auto it = t_left.find({ix, iy});
        if (it == t_left.end()) {continue;}
        const double age = t - it->second;
        g.max_age = std::max(g.max_age, age);
        if (age > 0.3) {++g.ghost_samples;}
      }
    }
  }
  std::printf(
    "[dynamic_sim] crossing %-20s max_ghost_age=%.2fs ghost_samples=%d see_through_cleared=%llu\n",
    modeName(mode), g.max_age, g.ghost_samples,
    static_cast<unsigned long long>(cm.voxelLayer().seeThroughCleared()));
  return g;
}
}  // namespace

TEST_CASE("pedestrian crossing: ghost cells clear within 1 s once the person has left")
{
  const GhostStats base = runCrossing(Mode::Baseline);
  const GhostStats see = runCrossing(Mode::SeeThrough);
  const GhostStats lab = runCrossing(Mode::SeeThroughLabels);
  CHECK(see.max_age <= 1.0);
  CHECK(lab.max_age <= 1.0);
  CHECK(see.ghost_samples <= base.ghost_samples);
}

TEST_CASE("navigation yields to a pedestrian crossing the path")
{
  for (const bool labels : {false, true}) {
    NavSimParams sp;
    if (labels) {sp.labeler = truthLabeler();}
    NavSimulation sim(roomWithPedestrian(3.6, -3.0), projectConfig(), Pose2D(0, 0, 0), sp);
    sim.warmup(1.0);
    sim.sendGoal({{"id", "ped"}, {"frame", "odom"}, {"x", 7.0}, {"y", 0.0}});
    const double t0 = sim.time();
    const NavSimResult r = sim.runUntilDone(60.0, [&](NavSimulation & s) {
          const double dt = s.time() - t0 - 1.0;
          moveBox(s.world(), "ped", 3.6, std::clamp(-3.0 + 0.8 * dt, -3.0, 3.0));
        });
    std::printf(
      "[dynamic_sim] crossing_nav labels=%d state=%s t=%.1fs collided=%d min_clear=%.2fm\n", labels,
      r.final_state.c_str(), r.time_s, r.collided, r.min_clearance);
    CHECK(r.final_state == "SUCCEEDED");
    CHECK_FALSE(r.collided);
  }
}

TEST_CASE("obstacle below the field of view persists in the voxel layer")
{
  World w;
  w.addRoom(-2.0, -4.0, 10.0, 4.0, 2.8);
  w.addBox({1.4, -0.3, 0.0}, {1.7, 0.3, 0.2}, "low_box");
  NavSimulation sim(w, projectConfig(), Pose2D(0, 0, 0));
  sim.warmup(1.0);
  const auto & voxel = sim.core().localCostmap().voxelLayer();
  auto boxVoxels = [&]() {
      int n = 0;
      for (double x = 1.425; x < 1.7; x += 0.05) {
        for (double y = -0.275; y < 0.3; y += 0.05) {
          for (double z = 0.13; z < 0.3; z += 0.1) {n += voxel.voxelOccupied(x, y, z);}
        }
      }
      return n;
    };
  const int seen = boxVoxels();
  REQUIRE(seen > 10);
  sim.sendGoal({{"id", "near"}, {"mode", "simple"}, {"frame", "odom"}, {"x", 0.75}, {"y", 0.0}});
  const NavSimResult r = sim.runUntilDone(20.0);
  REQUIRE(r.final_state == "SUCCEEDED");
  REQUIRE_FALSE(r.collided);
  // From here the box top is ~55 deg below the LiDAR, outside the -52 deg FOV edge.
  sim.warmup(30.0);
  const int kept = boxVoxels();
  std::printf("[dynamic_sim] below_fov robot_x=%.2f box_voxels seen=%d kept_after_30s=%d\n",
    sim.robot().pose().x, seen, kept);
  CHECK(kept >= seen * 8 / 10);
  CHECK(sim.core().localCostmap().costAt(1.55, 0.0) == LETHAL_OBSTACLE);
}
