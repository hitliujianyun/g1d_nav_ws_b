#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "fast_lio_api.h"
#include "g1d/common/time.hpp"
#include "g1d/lio/dynamic_detector.hpp"
#include "g1d/lio/lio_health.hpp"
#include "g1d/lio/livox_frame.hpp"
#include "g1d/lio/preprocess.hpp"

namespace g1d::lio
{

struct LioConfig
{
  fast_lio_api::Params fastlio;
  PreprocessParams preprocess;
  Eigen::Isometry3d T_head_lidar{Eigen::Isometry3d::Identity()};
  std::vector<Box> self_boxes;          // head frame
  double output_voxel{0.05};            // registered cloud downsampling (m)
  HealthParams health;
  DynamicDetectorParams dynamic;        // M-detector moving-point labelling (lio.dynamic)

  /// Reads `lio`, `self_filter` and `kinematics.head_to_livox` from the project config.
  static LioConfig fromYaml(const YAML::Node & root);
};

struct LioOutput
{
  double stamp_sensor{0.0};             // scan end, LiDAR clock
  double stamp_host{0.0};               // scan end, host clock (via ClockMapper)
  Eigen::Isometry3d T_world_imu{Eigen::Isometry3d::Identity()};  // LIO world = initial IMU frame
  Eigen::Vector3d lidar_origin_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_world{0.0, 0.0, -9.81};
  Eigen::Matrix<double, 6, 6> pose_cov{Eigen::Matrix<double, 6, 6>::Identity()};
  HealthStatus health;
  fast_lio_api::Result stats;           // estimator diagnostics (cloud_world / dynamic cleared)
  std::vector<Eigen::Vector3f> cloud_world;    // registered static returns, voxel-downsampled
  std::vector<Eigen::Vector3f> cloud_dynamic;  // registered returns labelled moving, downsampled
  DynamicDetectorStats dynamic_stats;
};

/// Thread-safe facade over FAST-LIO2: preprocessing, estimation, health and clock mapping.
/// Only one instance may exist per process (FAST-LIO keeps global state).
class LioEstimator
{
public:
  explicit LioEstimator(const LioConfig & config);
  ~LioEstimator();
  LioEstimator(const LioEstimator &) = delete;
  LioEstimator & operator=(const LioEstimator &) = delete;

  void pushImu(double t_sensor, const double gyr[3], const double acc[3]);
  void pushFrame(const LidarFrame & frame, PreprocessStats * stats = nullptr);
  /// Feed (sensor stamp, host receive time) pairs to keep the clock mapping current.
  void observeClock(double t_sensor, double t_host) {clock_.observe(t_sensor, t_host);}
  /// Runs one estimator step if a complete scan + IMU window is buffered.
  bool process(LioOutput * out);
  void reset();
  /// Late moving-object evidence: removes map points inside world boxes [x0,x1,y0,y1,z0,z1].
  int deleteMapBoxes(const std::vector<fast_lio_api::Box> & boxes);
  /// Snapshot of the estimator map (LIO world).
  std::vector<Eigen::Vector3f> mapPoints() const;
  bool dynamicEnabled() const {return detector_ != nullptr;}

  const LioConfig & config() const {return config_;}
  const ClockMapper & clock() const {return clock_;}
  bool initialized() const {return fast_lio_api::initialized();}

private:
  LioConfig config_;
  SelfFilter self_filter_;
  LioHealth health_;
  ClockMapper clock_;
  std::mutex mutex_;
  Eigen::Isometry3d T_imu_lidar_{Eigen::Isometry3d::Identity()};
  std::unique_ptr<DynamicDetector> detector_;
};

}  // namespace g1d::lio
