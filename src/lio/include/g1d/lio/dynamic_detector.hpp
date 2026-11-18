#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "fast_lio_api.h"

class DynObjFilter;

namespace g1d::lio
{

struct DynamicDetectorParams
{
  bool enabled{false};
  /// M-detector parameters by upstream name (without the "dyn_obj/" prefix). Unset keys use
  /// the Mid-360 defaults of `defaults()`.
  std::map<std::string, double> m_detector;
  int max_points{20000};          // per-frame capacity; larger scans are sub-sampled for detection
  // Voxel refinement (replaces the upstream DBSCAN cluster stage).
  double refine_voxel{0.15};      // m
  int refine_min_points{3};       // moving points needed in the 3x3x3 voxel neighbourhood
  double refine_min_ratio{0.4};   // moving share of detected points within the voxel
  int refine_dilate{1};           // grow moving voxels into occupied neighbours (iterations)
  double exclusion_voxel{0.2};    // map exclusion grid around moving points (m)
  int map_delay_scans{3};         // hold map insertions this many scans (temporal consistency)

  static std::map<std::string, double> defaults();
  /// Reads `lio.dynamic` (enabled, refine_*, exclusion_voxel, max_points, m_detector: {...}).
  static DynamicDetectorParams fromYaml(const YAML::Node & root);
};

struct DynamicDetectorStats
{
  int input_points{0};
  int detected_points{0};         // points passed to M-detector
  int raw_dynamic{0};             // M-detector labels before refinement
  int dynamic{0};                 // after refinement
  double time_ms{0.0};
};

/// M-detector (Wu et al., Nature Communications 2024) on the undistorted scan + voxel refinement.
/// Expects scans in time order; labels are delayed by nothing (case 1/2/3 tests use past maps).
class DynamicDetector
{
public:
  explicit DynamicDetector(const DynamicDetectorParams & p);
  ~DynamicDetector();
  DynamicDetector(const DynamicDetector &) = delete;
  DynamicDetector & operator=(const DynamicDetector &) = delete;

  /// `scan_lidar` in the LiDAR frame at scan end; `R`, `t` = LiDAR pose in the world.
  void label(
    const fast_lio_api::Cloud & scan_lidar, const Eigen::Matrix3d & R, const Eigen::Vector3d & t,
    double stamp, std::vector<uint8_t> & dynamic);
  /// Adapter for fast_lio_api::set_point_labeler.
  fast_lio_api::PointLabeler labeler();
  const DynamicDetectorStats & lastStats() const {return stats_;}
  const DynamicDetectorParams & params() const {return p_;}

private:
  DynamicDetectorParams p_;
  std::unique_ptr<DynObjFilter> filter_;
  DynamicDetectorStats stats_;
};

}  // namespace g1d::lio
