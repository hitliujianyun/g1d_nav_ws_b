#include "g1d/lio/dynamic_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

#include <m-detector/DynObjFilter.h>

#include "g1d/common/config.hpp"

namespace g1d::lio
{

std::map<std::string, double> DynamicDetectorParams::defaults()
{
  // Upstream config/mid360/mid360.yaml (hku-mars/M-detector @ 0645dab), except: 10 Hz frames,
  // no cluster stage, no debug clouds, 4 depth maps (memory on the Orin NX), and case-1 map
  // consistency windows of 0.15 m instead of 0.3 / 0.6 m: with the inverted head mount the
  // floor next to a person's feet falls inside the larger windows, the legs stay "consistent
  // with the map" and the static label then spreads up the body (recall 0.2 -> 0.7 in sim).
  // Upstream quirk kept: "ver_resolution_max" sets the horizontal resolution and vice versa.
  return {
    {"dataset", 4}, {"buffer_delay", 0.1}, {"buffer_size", 100000}, {"depth_map_dur", 0.4},
    {"max_depth_map_num", 4}, {"max_pixel_points", 20}, {"frame_dur", 0.1},
    {"self_x_f", 0}, {"self_x_b", 0}, {"self_y_l", 0}, {"self_y_r", 0}, {"blind_dis", 0.3},
    {"fov_up", 52}, {"fov_down", -7}, {"fov_left", 180}, {"fov_right", -180}, {"fov_cut", -20},
    {"depth_thr1", 0.5}, {"enter_min_thr1", 0.5}, {"enter_max_thr1", 0.5},
    {"map_cons_depth_thr1", 0.5}, {"map_cons_hor_thr1", 0.05}, {"map_cons_ver_thr1", 0.05},
    {"map_cons_hor_dis1", 0.15}, {"map_cons_ver_dis1", 0.15}, {"depth_cons_depth_thr1", 0.5},
    {"depth_cons_depth_max_thr1", 1.0}, {"depth_cons_hor_thr1", 0.05},
    {"depth_cons_ver_thr1", 0.05}, {"enlarge_z_thr1", -2.5}, {"enlarge_angle", 2.0},
    {"enlarge_depth", 3.0}, {"occluded_map_thr1", 3}, {"case1_interp_en", 1},
    {"v_min_thr2", 0.5}, {"acc_thr2", 7.0}, {"map_cons_depth_thr2", 0.3},
    {"map_cons_hor_thr2", 0.02}, {"map_cons_ver_thr2", 0.06}, {"occ_depth_thr2", 0.3},
    {"occ_hor_thr2", 0.02}, {"occ_ver_thr2", 0.06}, {"depth_cons_depth_thr2", 0.1},
    {"depth_cons_depth_max_thr2", 0.5}, {"depth_cons_hor_thr2", 0.03},
    {"depth_cons_ver_thr2", 0.06}, {"occluded_times_thr2", 3}, {"k_depth2", 0.005},
    {"case2_interp_en", 0},
    {"v_min_thr3", 0.5}, {"acc_thr3", 15.0}, {"map_cons_depth_thr3", 0.1},
    {"map_cons_hor_thr3", 0.03}, {"map_cons_ver_thr3", 0.03}, {"occ_depth_thr3", 0.15},
    {"occ_hor_thr3", 0.03}, {"occ_ver_thr3", 0.03}, {"depth_cons_depth_thr3", 0.3},
    {"depth_cons_depth_max_thr3", 1.0}, {"depth_cons_hor_thr3", 0.03},
    {"depth_cons_ver_thr3", 0.03}, {"occluding_times_thr3", 3}, {"case3_interp_en", 0},
    {"interp_hor_thr", 0.03}, {"interp_ver_thr", 0.03}, {"interp_thr1", 0.5},
    {"interp_static_max", 10.0}, {"interp_start_depth1", 10}, {"interp_kp1", 0.1},
    {"interp_kd1", 0.5}, {"interp_thr2", 0.25}, {"interp_thr3", 0.05},
    {"dyn_filter_en", 1}, {"debug_publish", 0}, {"cluster_coupled", 0}, {"cluster_future", 0},
    {"ver_resolution_max", 0.04}, {"hor_resolution_max", 0.025}, {"point_index", -1},
  };
}

DynamicDetectorParams DynamicDetectorParams::fromYaml(const YAML::Node & root)
{
  DynamicDetectorParams p;
  p.enabled = get(root, "lio.dynamic.enabled", p.enabled);
  p.max_points = get(root, "lio.dynamic.max_points", p.max_points);
  p.refine_voxel = get(root, "lio.dynamic.refine_voxel", p.refine_voxel);
  p.refine_min_points = get(root, "lio.dynamic.refine_min_points", p.refine_min_points);
  p.refine_min_ratio = get(root, "lio.dynamic.refine_min_ratio", p.refine_min_ratio);
  p.refine_dilate = get(root, "lio.dynamic.refine_dilate", p.refine_dilate);
  p.exclusion_voxel = get(root, "lio.dynamic.exclusion_voxel", p.exclusion_voxel);
  p.map_delay_scans = get(root, "lio.dynamic.map_delay_scans", p.map_delay_scans);
  const YAML::Node md = findPath(root, "lio.dynamic.m_detector");
  if (md && md.IsMap()) {
    for (const auto & kv : md) {p.m_detector[kv.first.as<std::string>()] = kv.second.as<double>();}
  }
  return p;
}

DynamicDetector::DynamicDetector(const DynamicDetectorParams & p)
: p_(p), filter_(std::make_unique<DynObjFilter>())
{
  auto values = DynamicDetectorParams::defaults();
  for (const auto & kv : p_.m_detector) {values[kv.first] = kv.second;}
  values["points_num_perframe"] = p_.max_points;
  values["buffer_size"] = std::max(values["buffer_size"], 4.0 * p_.max_points);
  const std::string prefix = "dyn_obj/";
  filter_->init(m_detector::ParamSource([values, prefix](const std::string & name, double & v) {
      if (name.rfind(prefix, 0) != 0) {return false;}
      const auto it = values.find(name.substr(prefix.size()));
      if (it == values.end()) {return false;}
      v = it->second;
      return true;
    }));
}

DynamicDetector::~DynamicDetector() = default;

namespace
{
int64_t voxelKey(int x, int y, int z)
{
  return (static_cast<int64_t>(x) & 0x1FFFFF) << 42 | (static_cast<int64_t>(y) & 0x1FFFFF) << 21 |
         (static_cast<int64_t>(z) & 0x1FFFFF);
}
}  // namespace

void DynamicDetector::label(
  const fast_lio_api::Cloud & scan, const Eigen::Matrix3d & R, const Eigen::Vector3d & t,
  double stamp, std::vector<uint8_t> & dynamic)
{
  const auto t0 = std::chrono::steady_clock::now();
  const int n = static_cast<int>(scan.size());
  dynamic.assign(n, 0);
  stats_ = DynamicDetectorStats{};
  stats_.input_points = n;
  if (n == 0) {return;}

  const int stride = std::max(1, (n + p_.max_points - 1) / p_.max_points);
  PointCloudXYZI::Ptr fed(new PointCloudXYZI());
  fed->reserve(n / stride + 1);
  for (int i = 0; i < n; i += stride) {fed->push_back(scan.points[i]);}
  stats_.detected_points = static_cast<int>(fed->size());
  filter_->filter(fed, R, t, stamp);
  const std::vector<int> & raw = filter_->dyn_tag_origin;

  // Voxel vote in the world frame: isolated labels are dropped, labels spread to the whole voxel
  // (including points that were sub-sampled away).
  struct Cell {int dyn{0}; int total{0};};
  const double inv = 1.0 / p_.refine_voxel;
  std::vector<Eigen::Vector3i> cell(n);
  std::unordered_map<int64_t, Cell> cells;
  cells.reserve(static_cast<size_t>(n) / 2 + 1);
  for (int i = 0; i < n; ++i) {
    const auto & q = scan.points[i];
    const Eigen::Vector3d w = R * Eigen::Vector3d(q.x, q.y, q.z) + t;
    cell[i] = Eigen::Vector3i(static_cast<int>(std::floor(w.x() * inv)),
        static_cast<int>(std::floor(w.y() * inv)), static_cast<int>(std::floor(w.z() * inv)));
  }
  for (int j = 0, i = 0; i < n && j < static_cast<int>(raw.size()); i += stride, ++j) {
    Cell & c = cells[voxelKey(cell[i].x(), cell[i].y(), cell[i].z())];
    ++c.total;
    if (raw[j] == 1) {++c.dyn; ++stats_.raw_dynamic;}
  }
  std::unordered_map<int64_t, bool> verdict;
  auto isDynamic = [&](const Eigen::Vector3i & v) {
      const int64_t key = voxelKey(v.x(), v.y(), v.z());
      const auto known = verdict.find(key);
      if (known != verdict.end()) {return known->second;}
      bool d = false;
      const auto it = cells.find(key);
      if (it != cells.end() && it->second.dyn > 0 &&
        it->second.dyn >= p_.refine_min_ratio * it->second.total)
      {
        int support = 0;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              const auto nb = cells.find(voxelKey(v.x() + dx, v.y() + dy, v.z() + dz));
              if (nb != cells.end()) {support += nb->second.dyn;}
            }
          }
        }
        d = support >= p_.refine_min_points;
      }
      verdict.emplace(key, d);
      return d;
    };
  std::vector<int64_t> keys(n);
  std::unordered_map<int64_t, bool> grown;
  for (int i = 0; i < n; ++i) {
    keys[i] = voxelKey(cell[i].x(), cell[i].y(), cell[i].z());
    if (isDynamic(cell[i])) {grown[keys[i]] = true;}
  }
  // Grow by `refine_dilate` voxels into occupied neighbours (legs next to the floor, edges).
  for (int it = 0; it < p_.refine_dilate; ++it) {
    std::vector<int64_t> add;
    for (int i = 0; i < n; ++i) {
      if (grown.count(keys[i])) {continue;}
      const Eigen::Vector3i & v = cell[i];
      bool hit = false;
      for (int dx = -1; dx <= 1 && !hit; ++dx) {
        for (int dy = -1; dy <= 1 && !hit; ++dy) {
          for (int dz = -1; dz <= 1 && !hit; ++dz) {
            hit = grown.count(voxelKey(v.x() + dx, v.y() + dy, v.z() + dz)) > 0;
          }
        }
      }
      if (hit) {add.push_back(keys[i]);}
    }
    for (int64_t k : add) {grown[k] = true;}
  }
  for (int i = 0; i < n; ++i) {
    if (grown.count(keys[i])) {dynamic[i] = 1; ++stats_.dynamic;}
  }
  stats_.time_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

fast_lio_api::PointLabeler DynamicDetector::labeler()
{
  return [this](const fast_lio_api::Cloud & scan, const Eigen::Matrix3d & R,
           const Eigen::Vector3d & t, double stamp, std::vector<uint8_t> & dynamic) {
           label(scan, R, t, stamp, dynamic);
         };
}

}  // namespace g1d::lio
