#include "g1d/lio/lio_estimator.hpp"

#include <atomic>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include <pcl/filters/voxel_grid.h>

#include "g1d/common/config.hpp"
#include "g1d/common/geometry.hpp"

namespace g1d::lio
{
namespace
{
std::atomic<int> g_instances{0};

Eigen::Isometry3d isometryFromYaml(const YAML::Node & n)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (!n) {return T;}
  const auto xyz = n["xyz"] ? n["xyz"].as<std::vector<double>>() : std::vector<double>{0, 0, 0};
  const auto rpy = n["rpy"] ? n["rpy"].as<std::vector<double>>() : std::vector<double>{0, 0, 0};
  if (xyz.size() != 3 || rpy.size() != 3) {throw std::runtime_error("xyz/rpy need 3 values");}
  return makeIsometry(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
}
}  // namespace

LioConfig LioConfig::fromYaml(const YAML::Node & root)
{
  LioConfig c;
  auto & f = c.fastlio;
  f.gyr_cov = get(root, "lio.gyr_cov", f.gyr_cov);
  f.acc_cov = get(root, "lio.acc_cov", f.acc_cov);
  f.b_gyr_cov = get(root, "lio.b_gyr_cov", f.b_gyr_cov);
  f.b_acc_cov = get(root, "lio.b_acc_cov", f.b_acc_cov);
  f.filter_size_surf = get(root, "lio.filter_size_surf", f.filter_size_surf);
  f.filter_size_map = get(root, "lio.filter_size_map", f.filter_size_map);
  f.cube_side_length = get(root, "lio.cube_side_length", f.cube_side_length);
  f.det_range = get(root, "lio.det_range", f.det_range);
  f.fov_degree = get(root, "lio.fov_degree", f.fov_degree);
  f.max_iteration = get(root, "lio.max_iteration", f.max_iteration);
  f.extrinsic_est_en = get(root, "lio.extrinsic_est_en", f.extrinsic_est_en);
  f.extrinsic_T = get(root, "lio.extrinsic_T", f.extrinsic_T);
  f.extrinsic_R = get(root, "lio.extrinsic_R", f.extrinsic_R);
  f.time_offset_lidar_to_imu = get(root, "lio.time_offset_lidar_to_imu", f.time_offset_lidar_to_imu);

  c.preprocess.blind = get(root, "lio.blind", c.preprocess.blind);
  c.preprocess.point_filter_num = get(root, "lio.point_filter_num", c.preprocess.point_filter_num);
  c.preprocess.max_range = get(root, "lio.max_range", c.preprocess.max_range);
  c.output_voxel = get(root, "lio.publish_cloud_voxel", c.output_voxel);

  c.T_head_lidar = isometryFromYaml(findPath(root, "kinematics.head_to_livox"));
  const YAML::Node boxes = findPath(root, "self_filter.boxes");
  if (boxes && boxes.IsSequence()) {
    for (const auto & b : boxes) {
      const auto v = b.as<std::vector<double>>();
      if (v.size() != 6) {throw std::runtime_error("self_filter.boxes entries need 6 values");}
      c.self_boxes.push_back({v[0], v[1], v[2], v[3], v[4], v[5]});
    }
  }

  auto & h = c.health;
  h.min_effective_points = get(root, "lio.health.min_effective_points", h.min_effective_points);
  h.bad_frames_to_fail = get(root, "lio.health.bad_frames_to_fail", h.bad_frames_to_fail);
  h.max_step_translation_m = get(root, "lio.health.max_step_translation_m", h.max_step_translation_m);
  h.max_step_yaw_deg = get(root, "lio.health.max_step_yaw_deg", h.max_step_yaw_deg);
  h.max_process_time_s = get(root, "lio.health.max_process_time_s", h.max_process_time_s);
  h.max_z_drift_m = get(root, "lio.health.max_z_drift_m", h.max_z_drift_m);
  h.min_normal_eigenvalue = get(root, "lio.health.min_normal_eigenvalue", h.min_normal_eigenvalue);
  c.dynamic = DynamicDetectorParams::fromYaml(root);
  return c;
}

LioEstimator::LioEstimator(const LioConfig & config)
: config_(config), self_filter_(config.T_head_lidar, config.self_boxes), health_(config.health)
{
  if (g_instances.fetch_add(1) != 0) {
    g_instances.fetch_sub(1);
    throw std::logic_error("only one LioEstimator may exist per process (FAST-LIO global state)");
  }
  const auto & T = config_.fastlio.extrinsic_T;
  const auto & R = config_.fastlio.extrinsic_R;
  T_imu_lidar_.translation() = Eigen::Vector3d(T[0], T[1], T[2]);
  T_imu_lidar_.linear() << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
  fast_lio_api::init(config_.fastlio);
  if (config_.dynamic.enabled) {
    detector_ = std::make_unique<DynamicDetector>(config_.dynamic);
    fast_lio_api::set_point_labeler(
      detector_->labeler(), config_.dynamic.exclusion_voxel, config_.dynamic.map_delay_scans);
  } else {
    fast_lio_api::set_point_labeler({});
  }
}

LioEstimator::~LioEstimator()
{
  fast_lio_api::set_point_labeler({});
  fast_lio_api::reset();
  g_instances.fetch_sub(1);
}

int LioEstimator::deleteMapBoxes(const std::vector<fast_lio_api::Box> & boxes)
{
  return fast_lio_api::delete_boxes(boxes);
}

std::vector<Eigen::Vector3f> LioEstimator::mapPoints() const
{
  fast_lio_api::Cloud c;
  fast_lio_api::map_points(c);
  std::vector<Eigen::Vector3f> out;
  out.reserve(c.size());
  for (const auto & p : c.points) {out.emplace_back(p.x, p.y, p.z);}
  return out;
}

void LioEstimator::reset()
{
  std::lock_guard<std::mutex> lk(mutex_);
  fast_lio_api::reset();
  health_.reset();
}

void LioEstimator::pushImu(double t_sensor, const double gyr[3], const double acc[3])
{
  fast_lio_api::ImuSample s;
  s.t = t_sensor;
  for (int i = 0; i < 3; ++i) {
    s.gyr[i] = gyr[i];
    s.acc[i] = acc[i];
  }
  fast_lio_api::push_imu(s);
}

void LioEstimator::pushFrame(const LidarFrame & frame, PreprocessStats * stats)
{
  auto cloud = preprocessFrame(frame, config_.preprocess, self_filter_, stats);
  fast_lio_api::push_scan(frame.t_begin, cloud);
}

bool LioEstimator::process(LioOutput * out)
{
  std::lock_guard<std::mutex> lk(mutex_);
  fast_lio_api::Result r;
  if (!fast_lio_api::process(&r)) {return false;}
  if (!out) {return true;}

  out->stamp_sensor = r.t_end;
  out->stamp_host = clock_.valid() ? clock_.toHost(r.t_end) : r.t_end;
  out->T_world_imu = Eigen::Isometry3d::Identity();
  out->T_world_imu.linear() = r.rot.normalized().toRotationMatrix();
  out->T_world_imu.translation() = r.pos;
  out->lidar_origin_world = (out->T_world_imu * T_imu_lidar_).translation();
  out->velocity_world = r.vel;
  out->gravity_world = r.grav;
  out->pose_cov = r.pose_cov;
  out->health = health_.update(r);

  out->cloud_world.clear();
  out->cloud_dynamic.clear();
  out->dynamic_stats = detector_ ? detector_->lastStats() : DynamicDetectorStats{};
  if (r.cloud_world && !r.cloud_world->empty()) {
    const float leaf = static_cast<float>(config_.output_voxel);
    auto downsample = [leaf](const fast_lio_api::Cloud::Ptr & in, std::vector<Eigen::Vector3f> & dst) {
        if (in->empty()) {return;}
        pcl::VoxelGrid<fast_lio_api::Point> vg;
        vg.setLeafSize(leaf, leaf, leaf);
        vg.setInputCloud(in);
        fast_lio_api::Cloud ds;
        vg.filter(ds);
        dst.reserve(ds.size());
        for (const auto & p : ds.points) {dst.emplace_back(p.x, p.y, p.z);}
      };
    if (r.dynamic.size() == r.cloud_world->size() && r.dynamic_points > 0) {
      fast_lio_api::Cloud::Ptr st(new fast_lio_api::Cloud()), dy(new fast_lio_api::Cloud());
      st->reserve(r.cloud_world->size());
      dy->reserve(static_cast<size_t>(r.dynamic_points));
      for (size_t i = 0; i < r.dynamic.size(); ++i) {
        (r.dynamic[i] ? dy : st)->push_back(r.cloud_world->points[i]);
      }
      downsample(st, out->cloud_world);
      downsample(dy, out->cloud_dynamic);
    } else {
      downsample(r.cloud_world, out->cloud_world);
    }
  }
  r.cloud_world.reset();
  r.dynamic.clear();
  out->stats = r;
  return true;
}

}  // namespace g1d::lio
