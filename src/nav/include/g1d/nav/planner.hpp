#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "g1d/common/geometry.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_navfn_planner
{
class NavFn;
}

namespace g1d::nav
{

struct PlannerParams
{
  bool use_astar{false};
  bool allow_unknown{true};
  double tolerance{0.3};           // NavFn goal tolerance (m)
  int max_cycles_factor{4};
  double path_spacing{0.05};       // resampled path spacing (m)
  double goal_search_radius{0.5};  // goal sanitisation spiral search radius (m)
  unsigned char goal_max_cost{128};
  double max_goal_distance{9.0};   // goals further than this are clamped (global map half-size)

  static PlannerParams fromYaml(const YAML::Node & root);
};

struct GoalCheck
{
  bool ok{false};
  bool adjusted{false};   // goal moved out of an obstacle
  bool partial{false};    // goal clamped to max_goal_distance
  Pose2D goal;
  std::string reason;
};

/// NavFn (Nav2 navfn core, Dijkstra / A* potential + gradient descent) on the global costmap.
class GlobalPlanner
{
public:
  explicit GlobalPlanner(PlannerParams p = {});
  ~GlobalPlanner();

  /// Returns the path start->goal in world coordinates with yaw set along the path; the last
  /// pose takes `goal.yaw` when `use_goal_yaw`.
  bool makePlan(
    const nav2_costmap_2d::Costmap2D & costmap, const Pose2D & start, const Pose2D & goal,
    bool use_goal_yaw, std::vector<Pose2D> & path, std::string * error = nullptr);

  /// Clamps distant goals and moves goals out of obstacles (ray back towards the robot, then a
  /// spiral search).
  GoalCheck sanitizeGoal(
    const nav2_costmap_2d::Costmap2D & costmap, const Pose2D & robot, const Pose2D & goal) const;

  const PlannerParams & params() const {return p_;}

private:
  PlannerParams p_;
  std::unique_ptr<nav2_navfn_planner::NavFn> navfn_;
  unsigned nx_{0}, ny_{0};
};

double pathLength(const std::vector<Pose2D> & path, size_t from = 0);

}  // namespace g1d::nav
