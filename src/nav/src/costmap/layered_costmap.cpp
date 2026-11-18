#include "g1d/nav/costmap/layered_costmap.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "g1d/common/config.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace g1d::nav::costmap
{

LayeredCostmapParams LayeredCostmapParams::fromYaml(const YAML::Node & root, const std::string & w)
{
  LayeredCostmapParams p;
  p.name = w + "_costmap";
  const std::string c = "costmap.common.";
  const std::string s = "costmap." + w + ".";
  auto pick = [&](const std::string & key, auto fallback) {
      return get(root, s + key, get(root, c + key, fallback));
    };
  if (w == "global") {
    p.size_x = p.size_y = 20.0;
    p.resolution = 0.1;
  }
  p.size_x = pick("size_x", p.size_x);
  p.size_y = pick("size_y", p.size_y);
  p.resolution = pick("resolution", p.resolution);
  p.rolling = pick("rolling", p.rolling);
  p.use_voxel = pick("use_voxel", p.use_voxel);
  p.use_scan = pick("use_scan", p.use_scan);
  p.clear_footprint = pick("clear_footprint", p.clear_footprint);
  p.voxel.z_min = pick("voxel.z_min", p.voxel.z_min);
  p.voxel.z_max = pick("voxel.z_max", p.voxel.z_max);
  p.voxel.z_resolution = pick("voxel.z_resolution", p.voxel.z_resolution);
  p.voxel.obstacle_max_range = pick("voxel.obstacle_max_range", p.voxel.obstacle_max_range);
  p.voxel.raytrace_max_range = pick("voxel.raytrace_max_range", p.voxel.raytrace_max_range);
  p.voxel.decay_s = pick("voxel.decay_s", p.voxel.decay_s);
  p.voxel.clear_every_n = pick("voxel.clear_every_n", p.voxel.clear_every_n);
  p.voxel.min_points_per_column = pick("voxel.min_points_per_column", p.voxel.min_points_per_column);
  p.voxel.see_through_clear = pick("voxel.see_through_clear", p.voxel.see_through_clear);
  p.voxel.see_through_bin_deg = pick("voxel.see_through_bin_deg", p.voxel.see_through_bin_deg);
  p.voxel.see_through_margin = pick("voxel.see_through_margin", p.voxel.see_through_margin);
  p.voxel.see_through_frames = pick("voxel.see_through_frames", p.voxel.see_through_frames);
  p.voxel.blind_radius = pick("voxel.blind_radius", p.voxel.blind_radius);
  p.dynamic.enabled = pick("dynamic.enabled", p.dynamic.enabled);
  p.dynamic.ttl_s = pick("dynamic.ttl_s", p.dynamic.ttl_s);
  p.dynamic.max_range = pick("dynamic.max_range", p.dynamic.max_range);
  p.dynamic.z_min = p.voxel.z_min;
  p.dynamic.z_max = p.voxel.z_max;
  p.scan.obstacle_max_range = pick("scan.obstacle_max_range", p.scan.obstacle_max_range);
  p.scan.raytrace_max_range = pick("scan.raytrace_max_range", p.scan.raytrace_max_range);
  p.scan.inf_is_valid = pick("scan.inf_is_valid", p.scan.inf_is_valid);
  p.inflation.inflation_radius = pick("inflation.radius", p.inflation.inflation_radius);
  p.inflation.cost_scaling_factor =
    pick("inflation.cost_scaling_factor", p.inflation.cost_scaling_factor);

  const auto fp = get(root, "robot.footprint", std::vector<std::vector<double>>{
      {0.30, 0.28}, {0.30, -0.28}, {-0.30, -0.28}, {-0.30, 0.28}});
  for (const auto & v : fp) {
    if (v.size() != 2) {throw std::runtime_error("robot.footprint entries need [x, y]");}
    p.footprint.emplace_back(v[0], v[1]);
  }
  return p;
}

LayeredCostmap::LayeredCostmap(const LayeredCostmapParams & p)
: p_(p)
{
  const unsigned sx = static_cast<unsigned>(std::lround(p.size_x / p.resolution));
  const unsigned sy = static_cast<unsigned>(std::lround(p.size_y / p.resolution));
  // Costmap2D() leaves default_value_ uninitialised (Costmap2DROS normally sets it).
  master_.setDefaultValue(nav2_costmap_2d::FREE_SPACE);
  master_.resizeMap(sx, sy, p.resolution, -0.5 * p.size_x, -0.5 * p.size_y);

  layered_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>(&master_);
  layered_->setFootprint(footprintMsg());
  inscribed_ = layered_->getInscribedRadius();
  circumscribed_ = layered_->getCircumscribedRadius();
  inflation_ = std::make_shared<InflationLayer>();
  inflation_->configure(p.inflation, p.resolution, inscribed_);
  layered_->addPlugin(inflation_);
  ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(layered_, "odom", "base_link");

  voxel_.configure(p.voxel, sx, sy, p.resolution, master_.getOriginX(), master_.getOriginY());
  dynamic_.configure(p.dynamic, sx, sy, p.resolution, master_.getOriginX(), master_.getOriginY());
  scan_.configure(p.scan, sx, sy, p.resolution, master_.getOriginX(), master_.getOriginY());
}

std::vector<geometry_msgs::msg::Point> LayeredCostmap::footprintMsg() const
{
  std::vector<geometry_msgs::msg::Point> out;
  for (const auto & v : p_.footprint) {
    geometry_msgs::msg::Point pt;
    pt.x = v.x();
    pt.y = v.y();
    out.push_back(pt);
  }
  return out;
}

std::vector<Eigen::Vector2d> LayeredCostmap::footprintAt(const Pose2D & pose) const
{
  std::vector<Eigen::Vector2d> out;
  for (const auto & v : p_.footprint) {out.push_back(pose.transformPoint(v));}
  return out;
}

void LayeredCostmap::addCloud(
  const std::vector<Eigen::Vector3f> & points, const Eigen::Vector3d & sensor, double stamp,
  const std::vector<Eigen::Vector3f> * dynamic)
{
  if (p_.use_voxel) {voxel_.addCloud(points, sensor, stamp, dynamic);}
  if (dynamic && !dynamic->empty()) {dynamic_.addPoints(*dynamic, sensor, stamp);}
}

void LayeredCostmap::addDynamicPoints(
  const std::vector<Eigen::Vector3f> & points, const Eigen::Vector3d & origin, double stamp,
  double clear_radius)
{
  if (points.empty()) {return;}
  if (p_.use_voxel) {voxel_.clearAround(points, clear_radius);}
  dynamic_.addPoints(points, origin, stamp);
}

void LayeredCostmap::addScan(const ScanData & scan, const Pose2D & scanner)
{
  if (p_.use_scan) {scan_.addScan(scan, scanner);}
}

void LayeredCostmap::clearOutsideRadius(const Pose2D & robot, double radius)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lk(*master_.getMutex());
  voxel_.clearOutsideRadius(robot.x, robot.y, radius);
  scan_.clearOutsideRadius(robot.x, robot.y, radius);
}

void LayeredCostmap::update(const Pose2D & robot, double now)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lk(*master_.getMutex());
  if (p_.rolling) {
    master_.updateOrigin(robot.x - 0.5 * p_.size_x, robot.y - 0.5 * p_.size_y);
    voxel_.updateOrigin(master_.getOriginX(), master_.getOriginY());
    dynamic_.updateOrigin(master_.getOriginX(), master_.getOriginY());
    scan_.updateOrigin(master_.getOriginX(), master_.getOriginY());
  }
  voxel_.decay(now);
  if (p_.clear_footprint) {
    const auto fp = footprintAt(robot);
    voxel_.clearPolygon(fp);
    dynamic_.clearPolygon(fp);
    scan_.clearPolygon(fp);
  }
  master_.resetMap(0, 0, master_.getSizeInCellsX(), master_.getSizeInCellsY());
  voxel_.writeTo(master_);
  dynamic_.writeTo(master_, now);
  scan_.writeTo(master_);
  inflation_->inflate(master_);
}

double LayeredCostmap::footprintCost(const Pose2D & pose) const
{
  // Only the Costmap2D* instantiation exists upstream; the checker does not modify the map.
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker;
  checker.setCostmap(const_cast<nav2_costmap_2d::Costmap2D *>(&master_));
  return checker.footprintCostAtPose(pose.x, pose.y, pose.yaw, footprintMsg());
}

unsigned char LayeredCostmap::costAt(double x, double y) const
{
  unsigned int mx, my;
  if (!master_.worldToMap(x, y, mx, my)) {return nav2_costmap_2d::NO_INFORMATION;}
  return master_.getCost(mx, my);
}

nav_msgs::msg::OccupancyGrid LayeredCostmap::toOccupancyGrid() const
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "odom";
  g.info.resolution = static_cast<float>(master_.getResolution());
  g.info.width = master_.getSizeInCellsX();
  g.info.height = master_.getSizeInCellsY();
  g.info.origin.position.x = master_.getOriginX();
  g.info.origin.position.y = master_.getOriginY();
  g.data.resize(static_cast<size_t>(g.info.width) * g.info.height);
  const unsigned char * c = master_.getCharMap();
  for (size_t i = 0; i < g.data.size(); ++i) {
    // Same mapping as nav2_costmap_2d::Costmap2DPublisher.
    if (c[i] == nav2_costmap_2d::NO_INFORMATION) {
      g.data[i] = -1;
    } else if (c[i] == nav2_costmap_2d::LETHAL_OBSTACLE) {
      g.data[i] = 100;
    } else if (c[i] == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      g.data[i] = 99;
    } else {
      g.data[i] = static_cast<int8_t>(1 + (97 * (c[i] - 1)) / 251);
      if (c[i] == 0) {g.data[i] = 0;}
    }
  }
  return g;
}

}  // namespace g1d::nav::costmap
