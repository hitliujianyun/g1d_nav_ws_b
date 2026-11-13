#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "pcl/point_types.h"

namespace pcl
{

template<typename PointT>
class PointCloud
{
public:
  using Ptr = std::shared_ptr<PointCloud<PointT>>;
  using ConstPtr = std::shared_ptr<const PointCloud<PointT>>;
  using VectorType = std::vector<PointT, Eigen::aligned_allocator<PointT>>;

  PointCloud() = default;
  PointCloud(uint32_t width_, uint32_t height_)
  : points(static_cast<size_t>(width_) * height_), width(width_), height(height_) {}

  size_t size() const {return points.size();}
  bool empty() const {return points.empty();}
  void clear() {points.clear(); width = 0; height = 1;}
  void resize(size_t n) {points.resize(n); width = static_cast<uint32_t>(n); height = 1;}
  void reserve(size_t n) {points.reserve(n);}
  void push_back(const PointT & p) {points.push_back(p); width = static_cast<uint32_t>(points.size()); height = 1;}

  PointT & operator[](size_t i) {return points[i];}
  const PointT & operator[](size_t i) const {return points[i];}

  PointCloud & operator+=(const PointCloud & other)
  {
    points.insert(points.end(), other.points.begin(), other.points.end());
    width = static_cast<uint32_t>(points.size());
    height = 1;
    return *this;
  }

  VectorType points;
  uint32_t width{0};
  uint32_t height{1};
  bool is_dense{true};
};

}  // namespace pcl
