#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include <Eigen/Core>

#include "g1d/common/geometry.hpp"
#include "g1d/nav/costmap/rolling_grid.hpp"
#include "g1d/nav/types.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"

namespace g1d::nav::costmap
{

/// 3D obstacle layer for the head Mid-360S (Nav2 VoxelLayer semantics: persistent marking,
/// 3D ray-traced clearing), with optional time decay per column (STVL-like) and footprint
/// clearing. Heights are relative to the floor (odom z = 0).
struct VoxelLayerParams
{
  double z_min{0.08};               // below: floor
  double z_max{1.85};               // above: higher than the robot
  double z_resolution{0.1};
  double obstacle_max_range{5.0};   // mark only points closer than this (from the sensor)
  double raytrace_max_range{6.0};   // clear along rays up to this range
  double decay_s{0.0};              // 0 = no decay (cleared only by rays)
  int clear_every_n{1};             // use every N-th ray for clearing (CPU)
  int min_points_per_column{1};     // marks needed before a column becomes lethal
  // See-through clearing: an occupied voxel is cleared after `see_through_frames` scans in
  // which every return in its direction bin lies beyond it (the sensor saw past it). Voxels in
  // directions without returns (body occlusion, outside the FOV) or inside `blind_radius` never
  // collect evidence, so obstacles in blind zones persist.
  bool see_through_clear{true};
  double see_through_bin_deg{2.0};
  double see_through_margin{0.2};   // m the bin's nearest return must lie beyond the voxel
  int see_through_frames{2};        // 1..3
  double blind_radius{0.6};         // m from the sensor
};

class VoxelLayer
{
public:
  VoxelLayer() = default;
  void configure(const VoxelLayerParams & p, unsigned sx, unsigned sy, double res, double ox,
    double oy);
  void updateOrigin(double ox, double oy);
  /// Marks `points_odom` (static returns); `extra_returns` (e.g. points labelled dynamic) only
  /// clear: rays to them and their ranges count as free space up to the return.
  void addCloud(
    const std::vector<Eigen::Vector3f> & points_odom, const Eigen::Vector3d & sensor_odom,
    double stamp, const std::vector<Eigen::Vector3f> * extra_returns = nullptr);
  /// Clears the voxels containing `points_odom` and their xy neighbours within `radius`.
  void clearAround(const std::vector<Eigen::Vector3f> & points_odom, double radius);
  void clearPolygon(const std::vector<Eigen::Vector2d> & polygon_odom);
  void clearOutsideRadius(double cx, double cy, double radius);
  void decay(double now);
  void writeTo(nav2_costmap_2d::Costmap2D & master) const;
  int nz() const {return nz_;}
  bool occupied(int mx, int my) const;
  bool voxelOccupied(double x, double y, double z) const;
  /// Voxels removed by see-through evidence so far (diagnostics).
  uint64_t seeThroughCleared() const {return see_through_cleared_;}

private:
  void clearRay(const Eigen::Vector3d & a, const Eigen::Vector3d & b);
  void seeThrough(
    const std::vector<Eigen::Vector3f> & points, const std::vector<Eigen::Vector3f> * extra,
    const Eigen::Vector3d & sensor);
  void clearBit(int mx, int my, int z);

  VoxelLayerParams p_;
  int nz_{0};
  RollingGrid<uint32_t> bits_;
  RollingGrid<uint16_t> hits_;
  RollingGrid<float> stamp_;
  RollingGrid<uint64_t> evidence_;   // 2-bit see-through counter per voxel
  double t0_{0.0};
  bool has_t0_{false};
  uint64_t see_through_cleared_{0};
  std::vector<float> bin_min_;
};

/// Short-lived obstacles (points labelled dynamic, people): a cell stays lethal for `ttl_s`
/// after its last observation and is then forgotten, so departed objects leave no residue.
struct DynamicLayerParams
{
  bool enabled{true};
  double ttl_s{0.8};
  double z_min{0.08};
  double z_max{1.85};
  double max_range{6.0};            // from the sensor / robot
};

class DynamicLayer
{
public:
  void configure(const DynamicLayerParams & p, unsigned sx, unsigned sy, double res, double ox,
    double oy);
  void updateOrigin(double ox, double oy) {seen_.updateOrigin(ox, oy);}
  void addPoints(
    const std::vector<Eigen::Vector3f> & points_odom, const Eigen::Vector3d & origin_odom,
    double stamp);
  void clearPolygon(const std::vector<Eigen::Vector2d> & polygon_odom);
  void writeTo(nav2_costmap_2d::Costmap2D & master, double now) const;
  bool active(int mx, int my, double now) const;

private:
  DynamicLayerParams p_;
  RollingGrid<float> seen_;          // seconds since t0 of the last observation; <0 = never
  double t0_{0.0};
  bool has_t0_{false};
};

/// 2D obstacle layer for the chassis single-line LiDAR (Nav2 ObstacleLayer semantics).
struct ScanLayerParams
{
  double obstacle_max_range{4.0};
  double raytrace_max_range{5.0};
  bool inf_is_valid{false};
};

class ScanLayer
{
public:
  void configure(const ScanLayerParams & p, unsigned sx, unsigned sy, double res, double ox,
    double oy);
  void updateOrigin(double ox, double oy) {occ_.updateOrigin(ox, oy);}
  void addScan(const ScanData & scan, const Pose2D & scanner_odom);
  void clearPolygon(const std::vector<Eigen::Vector2d> & polygon_odom);
  void clearOutsideRadius(double cx, double cy, double radius);
  void writeTo(nav2_costmap_2d::Costmap2D & master) const;

private:
  ScanLayerParams p_;
  RollingGrid<uint8_t> occ_;
};

/// Inflation with Nav2's cost function; exposed to MPPI through InflationLayerInterface.
struct InflationParams
{
  double inflation_radius{0.55};
  double cost_scaling_factor{3.0};
};

class InflationLayer : public nav2_costmap_2d::InflationLayerInterface
{
public:
  InflationLayer() {name_ = "inflation_layer";}
  void configure(const InflationParams & p, double resolution, double inscribed_radius);
  void inflate(nav2_costmap_2d::Costmap2D & master) const;

  double getCostScalingFactor() override {return p_.cost_scaling_factor;}
  double getInflationRadius() override {return p_.inflation_radius;}
  mutex_t * getMutex() override {return &mutex_;}
  unsigned char computeCost(double distance_cells) const override;

private:
  InflationParams p_;
  double resolution_{0.05};
  double inscribed_radius_{0.0};
  int cell_radius_{0};
  struct KernelCell
  {
    int dx, dy;
    unsigned char cost;
  };
  std::vector<KernelCell> kernel_;
  mutable mutex_t mutex_;
};

/// Point-in-polygon test (polygon in world xy).
bool pointInPolygon(const Eigen::Vector2d & p, const std::vector<Eigen::Vector2d> & poly);

}  // namespace g1d::nav::costmap
