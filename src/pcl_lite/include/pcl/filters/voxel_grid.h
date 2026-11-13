#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pcl/point_cloud.h"

namespace pcl
{

namespace detail
{
// Field-wise accumulation, specialised per point type so the centroid covers all fields
// (PCL's VoxelGrid also averages every field).
template<typename PointT>
struct Accum;

template<>
struct Accum<PointXYZ>
{
  double s[3]{0, 0, 0};
  void add(const PointXYZ & p) {s[0] += p.x; s[1] += p.y; s[2] += p.z;}
  PointXYZ mean(double n) const
  {
    PointXYZ p; p.x = float(s[0] / n); p.y = float(s[1] / n); p.z = float(s[2] / n); return p;
  }
};

template<>
struct Accum<PointXYZI>
{
  double s[4]{0, 0, 0, 0};
  void add(const PointXYZI & p) {s[0] += p.x; s[1] += p.y; s[2] += p.z; s[3] += p.intensity;}
  PointXYZI mean(double n) const
  {
    PointXYZI p;
    p.x = float(s[0] / n); p.y = float(s[1] / n); p.z = float(s[2] / n);
    p.intensity = float(s[3] / n);
    return p;
  }
};

template<>
struct Accum<PointXYZINormal>
{
  double s[8]{0, 0, 0, 0, 0, 0, 0, 0};
  void add(const PointXYZINormal & p)
  {
    s[0] += p.x; s[1] += p.y; s[2] += p.z; s[3] += p.intensity;
    s[4] += p.normal_x; s[5] += p.normal_y; s[6] += p.normal_z; s[7] += p.curvature;
  }
  PointXYZINormal mean(double n) const
  {
    PointXYZINormal p;
    p.x = float(s[0] / n); p.y = float(s[1] / n); p.z = float(s[2] / n);
    p.intensity = float(s[3] / n);
    p.normal_x = float(s[4] / n); p.normal_y = float(s[5] / n); p.normal_z = float(s[6] / n);
    p.curvature = float(s[7] / n);
    return p;
  }
};
}  // namespace detail

/// Centroid voxel-grid downsampling (same output semantics as pcl::VoxelGrid).
template<typename PointT>
class VoxelGrid
{
public:
  void setLeafSize(float lx, float ly, float lz) {leaf_[0] = lx; leaf_[1] = ly; leaf_[2] = lz;}
  void setInputCloud(const typename PointCloud<PointT>::ConstPtr & cloud) {input_ = cloud;}

  void filter(PointCloud<PointT> & output) const
  {
    if (!input_) {output.clear(); return;}
    const auto & in = input_->points;
    const float inv[3] = {1.0f / leaf_[0], 1.0f / leaf_[1], 1.0f / leaf_[2]};

    std::vector<std::pair<uint64_t, uint32_t>> keyed;
    keyed.reserve(in.size());
    for (uint32_t i = 0; i < in.size(); ++i) {
      const auto & p = in[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
      const int64_t ix = static_cast<int64_t>(std::floor(p.x * inv[0])) + (1 << 20);
      const int64_t iy = static_cast<int64_t>(std::floor(p.y * inv[1])) + (1 << 20);
      const int64_t iz = static_cast<int64_t>(std::floor(p.z * inv[2])) + (1 << 20);
      const uint64_t key = (static_cast<uint64_t>(ix & 0x1FFFFF) << 42) |
        (static_cast<uint64_t>(iy & 0x1FFFFF) << 21) | static_cast<uint64_t>(iz & 0x1FFFFF);
      keyed.emplace_back(key, i);
    }
    std::sort(keyed.begin(), keyed.end());

    PointCloud<PointT> out;
    out.reserve(keyed.size() / 2 + 1);
    size_t i = 0;
    while (i < keyed.size()) {
      detail::Accum<PointT> acc;
      size_t j = i;
      for (; j < keyed.size() && keyed[j].first == keyed[i].first; ++j) {
        acc.add(in[keyed[j].second]);
      }
      out.push_back(acc.mean(static_cast<double>(j - i)));
      i = j;
    }
    output = std::move(out);
  }

private:
  float leaf_[3]{0.1f, 0.1f, 0.1f};
  typename PointCloud<PointT>::ConstPtr input_;
};

}  // namespace pcl
