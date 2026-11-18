#ifndef FAST_LIO_API_H
#define FAST_LIO_API_H

// g1d: embeddable interface to the FAST-LIO2 estimator (replaces the ROS node in laserMapping.cpp).
// The estimator keeps the upstream global state, so there is exactly one instance per process.
// push_imu()/push_scan() may be called from sensor threads; process() from one worker thread.

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_lio_api
{

using Point = pcl::PointXYZINormal;
using Cloud = pcl::PointCloud<Point>;

struct Params
{
  // mapping
  double gyr_cov{0.1};
  double acc_cov{0.1};
  double b_gyr_cov{0.0001};
  double b_acc_cov{0.0001};
  double filter_size_surf{0.3};
  double filter_size_map{0.4};
  double cube_side_length{1000.0};
  float det_range{60.0f};
  double fov_degree{360.0};
  int max_iteration{3};
  bool extrinsic_est_en{false};
  // LiDAR pose in the IMU frame (FAST-LIO "extrinsic_T/R": p_imu = R * p_lidar + T)
  std::vector<double> extrinsic_T{0.0, 0.0, 0.0};
  std::vector<double> extrinsic_R{1, 0, 0, 0, 1, 0, 0, 0, 1};
  double time_offset_lidar_to_imu{0.0};
};

struct ImuSample
{
  double t{0.0};            // seconds, same clock as the LiDAR
  double gyr[3]{0, 0, 0};   // rad/s
  double acc[3]{0, 0, 0};   // g or m/s^2 (rescaled internally)
};

struct Result
{
  double t_end{0.0};                 // scan end time (LiDAR clock)
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};      // IMU position in the LIO world frame
  Eigen::Quaterniond rot{Eigen::Quaterniond::Identity()};  // IMU orientation
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  Eigen::Vector3d grav{0.0, 0.0, -9.81};
  Eigen::Matrix<double, 6, 6> pose_cov{Eigen::Matrix<double, 6, 6>::Identity()};  // [pos, rot]
  int input_points{0};
  int downsampled_points{0};
  int effective_points{0};
  double residual_mean{0.0};
  double normal_eig_min{0.0};        // min eigenvalue of mean(n n^T) over matched planes (degeneracy)
  double process_time_s{0.0};
  bool ekf_inited{false};
  Cloud::Ptr cloud_world;            // undistorted scan in the LIO world frame
  std::vector<uint8_t> dynamic;      // per cloud_world point: 1 = labelled moving (empty: none)
  int dynamic_points{0};
  int map_skipped_points{0};         // downsampled points kept out of the map as moving
  double label_time_s{0.0};
};

/// Labels moving returns of the undistorted scan (LiDAR frame, scan end) given the LiDAR pose in
/// the LIO world; `dynamic` is pre-sized to the scan and zeroed. Runs after the EKF update and
/// before map insertion; points labelled moving are kept out of the ikd-Tree map.
using PointLabeler = std::function<void(
      const Cloud & scan_lidar, const Eigen::Matrix3d & R_world_lidar,
      const Eigen::Vector3d & t_world_lidar, double t_end, std::vector<uint8_t> & dynamic)>;

/// Axis-aligned world box [xmin, xmax, ymin, ymax, zmin, zmax].
using Box = std::array<float, 6>;

void init(const Params & params);
void reset();
/// Installs (or clears with an empty function) the moving-point labeller. Downsampled scan
/// points within `exclusion_voxel` (one voxel of dilation) of a moving point are not mapped;
/// insertions are held `map_delay_scans` scans and also dropped if a later scan in that window
/// labels a moving point next to them.
void set_point_labeler(PointLabeler labeler, double exclusion_voxel = 0.2, int map_delay_scans = 3);
/// Removes map points inside the boxes (late moving-object evidence); returns points deleted.
int delete_boxes(const std::vector<Box> & boxes);
/// Copy of the current ikd-Tree map (LIO world frame).
void map_points(Cloud & out);
void push_imu(const ImuSample & imu);
/// Points use `curvature` = time offset from `t_begin` in milliseconds.
void push_scan(double t_begin, const Cloud::Ptr & cloud);
/// Processes at most one buffered scan; returns true when `out` holds a new estimate.
bool process(Result * out);
/// True after the first scan has initialised the IMU and the map.
bool initialized();

}  // namespace fast_lio_api

#endif  // FAST_LIO_API_H
