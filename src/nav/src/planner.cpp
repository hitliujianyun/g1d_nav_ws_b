#include "g1d/nav/planner.hpp"

#include <cmath>
#include <limits>

#include "g1d/common/config.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn.hpp"

namespace g1d::nav
{

PlannerParams PlannerParams::fromYaml(const YAML::Node & root)
{
  PlannerParams p;
  p.use_astar = get(root, "planner.use_astar", p.use_astar);
  p.allow_unknown = get(root, "planner.allow_unknown", p.allow_unknown);
  p.tolerance = get(root, "planner.tolerance", p.tolerance);
  p.max_cycles_factor = get(root, "planner.max_cycles_factor", p.max_cycles_factor);
  p.path_spacing = get(root, "planner.path_spacing", p.path_spacing);
  p.goal_search_radius = get(root, "planner.goal_search_radius", p.goal_search_radius);
  p.goal_max_cost = static_cast<unsigned char>(get(root, "planner.goal_max_cost", int(p.goal_max_cost)));
  p.max_goal_distance = get(root, "planner.max_goal_distance", p.max_goal_distance);
  return p;
}

GlobalPlanner::GlobalPlanner(PlannerParams p)
: p_(p) {}

GlobalPlanner::~GlobalPlanner() = default;

double pathLength(const std::vector<Pose2D> & path, size_t from)
{
  double l = 0.0;
  for (size_t i = from + 1; i < path.size(); ++i) {l += path[i - 1].distanceTo(path[i]);}
  return l;
}

bool GlobalPlanner::makePlan(
  const nav2_costmap_2d::Costmap2D & costmap, const Pose2D & start, const Pose2D & goal,
  bool use_goal_yaw, std::vector<Pose2D> & path, std::string * error)
{
  // Follows nav2_navfn_planner::NavfnPlanner::makePlan / getPlanFromPotential.
  path.clear();
  auto fail = [&](const char * why) {if (error) {*error = why;} return false;};
  unsigned int sx, sy, gx, gy;
  if (!costmap.worldToMap(start.x, start.y, sx, sy)) {return fail("start_off_map");}
  if (!costmap.worldToMap(goal.x, goal.y, gx, gy)) {return fail("goal_off_map");}

  const unsigned nx = costmap.getSizeInCellsX(), ny = costmap.getSizeInCellsY();
  if (!navfn_ || nx != nx_ || ny != ny_) {
    navfn_ = std::make_unique<nav2_navfn_planner::NavFn>(nx, ny);
    nx_ = nx;
    ny_ = ny;
  }
  // Work on a copy: the robot cell is cleared ("it can't be an obstacle").
  std::vector<unsigned char> costs(costmap.getCharMap(), costmap.getCharMap() + nx * ny);
  costs[sy * nx + sx] = nav2_costmap_2d::FREE_SPACE;
  navfn_->setNavArr(nx, ny);
  navfn_->setCostmap(costs.data(), true, p_.allow_unknown);
  int map_start[2] = {static_cast<int>(sx), static_cast<int>(sy)};
  int map_goal[2] = {static_cast<int>(gx), static_cast<int>(gy)};
  navfn_->setStart(map_goal);
  navfn_->setGoal(map_start);
  auto never_cancel = []() {return false;};
  if (p_.use_astar) {
    navfn_->calcNavFnAstar(never_cancel);
  } else {
    navfn_->calcNavFnDijkstra(never_cancel, true);
  }

  // Pick the goal or the closest reachable cell within tolerance.
  auto potential = [&](double wx, double wy) {
      unsigned int mx, my;
      if (!costmap.worldToMap(wx, wy, mx, my)) {return std::numeric_limits<double>::max();}
      return static_cast<double>(navfn_->potarr[my * nx + mx]);
    };
  Pose2D best = goal;
  bool found = potential(goal.x, goal.y) < POT_HIGH;
  if (!found) {
    double best_d = std::numeric_limits<double>::max();
    const double res = costmap.getResolution();
    for (double y = goal.y - p_.tolerance; y <= goal.y + p_.tolerance; y += res) {
      for (double x = goal.x - p_.tolerance; x <= goal.x + p_.tolerance; x += res) {
        if (potential(x, y) < POT_HIGH) {
          const double d = std::hypot(x - goal.x, y - goal.y);
          if (d < best_d) {best_d = d; best = Pose2D(x, y, goal.yaw); found = true;}
        }
      }
    }
  }
  if (!found) {return fail("no_path");}

  unsigned int bx, by;
  costmap.worldToMap(best.x, best.y, bx, by);
  int map_best[2] = {static_cast<int>(bx), static_cast<int>(by)};
  navfn_->setStart(map_best);
  const int max_cycles = static_cast<int>(std::max(nx, ny)) * p_.max_cycles_factor;
  if (navfn_->calcPath(max_cycles) == 0) {return fail("no_path");}
  const float * px = navfn_->getPathX();
  const float * py = navfn_->getPathY();
  const int len = navfn_->getPathLen();
  std::vector<Pose2D> raw;
  raw.reserve(len + 1);
  for (int i = len - 1; i >= 0; --i) {
    raw.emplace_back(
      costmap.getOriginX() + (px[i] + 0.5) * costmap.getResolution(),
      costmap.getOriginY() + (py[i] + 0.5) * costmap.getResolution(), 0.0);
  }
  if (raw.empty() || raw.back().distanceTo(best) > 1e-3) {raw.push_back(best);}
  raw.front() = Pose2D(start.x, start.y, start.yaw);

  // Resample at path_spacing and set yaw along the path.
  path.push_back(raw.front());
  double carry = 0.0;
  for (size_t i = 1; i < raw.size(); ++i) {
    const double seg = raw[i - 1].distanceTo(raw[i]);
    double s = p_.path_spacing - carry;
    while (s <= seg) {
      const double u = s / seg;
      path.emplace_back(raw[i - 1].x + (raw[i].x - raw[i - 1].x) * u,
        raw[i - 1].y + (raw[i].y - raw[i - 1].y) * u, 0.0);
      s += p_.path_spacing;
    }
    carry = seg - (s - p_.path_spacing);
  }
  if (path.back().distanceTo(best) > 1e-3) {path.push_back(best);}
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    path[i].yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  }
  path.front().yaw = path.size() > 1 ? path[0].yaw : start.yaw;
  path.back().yaw = use_goal_yaw ? goal.yaw : (path.size() > 1 ? path[path.size() - 2].yaw : goal.yaw);
  return true;
}

GoalCheck GlobalPlanner::sanitizeGoal(
  const nav2_costmap_2d::Costmap2D & costmap, const Pose2D & robot, const Pose2D & goal) const
{
  GoalCheck r;
  r.goal = goal;
  const double dist = robot.distanceTo(goal);
  if (dist > p_.max_goal_distance) {
    const double u = p_.max_goal_distance / dist;
    r.goal.x = robot.x + (goal.x - robot.x) * u;
    r.goal.y = robot.y + (goal.y - robot.y) * u;
    r.partial = true;
  }
  auto cost = [&](double x, double y) -> int {
      unsigned int mx, my;
      if (!costmap.worldToMap(x, y, mx, my)) {return 256;}
      return costmap.getCost(mx, my);
    };
  auto acceptable = [&](double x, double y) {
      const int c = cost(x, y);
      return c <= p_.goal_max_cost || c == nav2_costmap_2d::NO_INFORMATION;
    };
  if (cost(r.goal.x, r.goal.y) == 256) {
    r.reason = "goal_out_of_range";
    return r;
  }
  if (acceptable(r.goal.x, r.goal.y)) {
    r.ok = true;
    return r;
  }
  // 1) walk back along the ray goal -> robot
  const double res = costmap.getResolution();
  const double d = robot.distanceTo(r.goal);
  for (double s = res; s < d; s += res) {
    const double x = r.goal.x + (robot.x - r.goal.x) * (s / d);
    const double y = r.goal.y + (robot.y - r.goal.y) * (s / d);
    if (acceptable(x, y)) {
      // Keep at least one cell of margin further from the obstacle.
      r.goal.x = x;
      r.goal.y = y;
      r.ok = r.adjusted = true;
      return r;
    }
  }
  // 2) spiral (ring) search around the original goal
  for (double rad = res; rad <= p_.goal_search_radius; rad += res) {
    const int n = std::max(8, static_cast<int>(2 * M_PI * rad / res));
    for (int k = 0; k < n; ++k) {
      const double a = 2 * M_PI * k / n;
      const double x = r.goal.x + rad * std::cos(a), y = r.goal.y + rad * std::sin(a);
      if (acceptable(x, y)) {
        r.goal.x = x;
        r.goal.y = y;
        r.ok = r.adjusted = true;
        return r;
      }
    }
  }
  r.reason = "goal_in_obstacle";
  return r;
}

}  // namespace g1d::nav
