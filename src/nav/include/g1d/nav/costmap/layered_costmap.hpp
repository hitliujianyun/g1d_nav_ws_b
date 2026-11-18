#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "g1d/nav/costmap/layers.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace g1d::nav::costmap
{

struct LayeredCostmapParams
{
  std::string name{"local_costmap"};
  double size_x{6.0};
  double size_y{6.0};
  double resolution{0.05};
  bool rolling{true};
  bool use_voxel{true};
  bool use_scan{true};
  bool clear_footprint{true};
  VoxelLayerParams voxel;
  ScanLayerParams scan;
  DynamicLayerParams dynamic;
  InflationParams inflation;
  std::vector<Eigen::Vector2d> footprint;   // base frame polygon

  /// Reads `costmap.common` + `costmap.<which>` ("local" / "global").
  static LayeredCostmapParams fromYaml(const YAML::Node & root, const std::string & which);
};

/// Rolling-window costmap composed of VoxelLayer (Mid-360S static returns) + DynamicLayer
/// (short-lived moving obstacles) + ScanLayer (chassis LiDAR) + InflationLayer on a
/// nav2_costmap_2d::Costmap2D master grid, with a Costmap2DROS facade so MPPI critics and NavFn
/// can use it unchanged.
class LayeredCostmap
{
public:
  explicit LayeredCostmap(const LayeredCostmapParams & p);

  /// `points_odom` are static returns; `dynamic_odom` (optional) are returns labelled moving:
  /// they clear the voxel layer along their rays and mark only the dynamic layer.
  void addCloud(
    const std::vector<Eigen::Vector3f> & points_odom, const Eigen::Vector3d & sensor_odom,
    double stamp, const std::vector<Eigen::Vector3f> * dynamic_odom = nullptr);
  /// Late moving-object evidence (e.g. camera person points): removes them from the voxel
  /// layer and marks the dynamic layer.
  void addDynamicPoints(
    const std::vector<Eigen::Vector3f> & points_odom, const Eigen::Vector3d & origin_odom,
    double stamp, double clear_radius);
  void addScan(const ScanData & scan, const Pose2D & scanner_odom);
  /// Re-centres (rolling), clears the footprint, composes layers and inflates.
  void update(const Pose2D & robot, double now);
  /// Recovery: forget obstacles further than `radius` from the robot.
  void clearOutsideRadius(const Pose2D & robot, double radius);

  nav2_costmap_2d::Costmap2D * costmap() {return &master_;}
  const nav2_costmap_2d::Costmap2D * costmap() const {return &master_;}
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos() {return ros_;}
  std::shared_ptr<InflationLayer> inflation() {return inflation_;}
  std::vector<Eigen::Vector2d> footprintAt(const Pose2D & pose) const;
  std::vector<geometry_msgs::msg::Point> footprintMsg() const;
  /// Max cost under the footprint at pose (LETHAL if any cell is lethal); 255 if off-map.
  double footprintCost(const Pose2D & pose) const;
  /// Cost at a world point (255 if off-map).
  unsigned char costAt(double x, double y) const;
  nav_msgs::msg::OccupancyGrid toOccupancyGrid() const;
  const LayeredCostmapParams & params() const {return p_;}
  const VoxelLayer & voxelLayer() const {return voxel_;}
  const DynamicLayer & dynamicLayer() const {return dynamic_;}
  double inscribedRadius() const {return inscribed_;}
  double circumscribedRadius() const {return circumscribed_;}

private:
  LayeredCostmapParams p_;
  nav2_costmap_2d::Costmap2D master_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_;
  std::shared_ptr<InflationLayer> inflation_;
  VoxelLayer voxel_;
  DynamicLayer dynamic_;
  ScanLayer scan_;
  double inscribed_{0.0};
  double circumscribed_{0.0};
};

}  // namespace g1d::nav::costmap
