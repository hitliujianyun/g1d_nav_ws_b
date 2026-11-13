#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace g1d::nav::costmap
{

/// Dense 2D grid whose origin can move by whole cells (same semantics as
/// nav2_costmap_2d::Costmap2D::updateOrigin: overlapping cells are kept, new cells reset).
template<typename T>
class RollingGrid
{
public:
  void resize(unsigned sx, unsigned sy, double res, double ox, double oy, T init)
  {
    sx_ = sx;
    sy_ = sy;
    res_ = res;
    ox_ = ox;
    oy_ = oy;
    init_ = init;
    data_.assign(static_cast<size_t>(sx) * sy, init);
  }

  unsigned sizeX() const {return sx_;}
  unsigned sizeY() const {return sy_;}
  double resolution() const {return res_;}
  double originX() const {return ox_;}
  double originY() const {return oy_;}

  bool worldToMap(double wx, double wy, int & mx, int & my) const
  {
    mx = static_cast<int>(std::floor((wx - ox_) / res_));
    my = static_cast<int>(std::floor((wy - oy_) / res_));
    return inBounds(mx, my);
  }
  bool inBounds(int mx, int my) const
  {
    return mx >= 0 && my >= 0 && mx < static_cast<int>(sx_) && my < static_cast<int>(sy_);
  }
  size_t index(int mx, int my) const {return static_cast<size_t>(my) * sx_ + mx;}
  T & at(int mx, int my) {return data_[index(mx, my)];}
  const T & at(int mx, int my) const {return data_[index(mx, my)];}
  std::vector<T> & data() {return data_;}
  const std::vector<T> & data() const {return data_;}
  void fill(T v) {std::fill(data_.begin(), data_.end(), v);}

  void updateOrigin(double new_ox, double new_oy)
  {
    const int cell_ox = static_cast<int>(std::floor((new_ox - ox_) / res_ + 1e-6));
    const int cell_oy = static_cast<int>(std::floor((new_oy - oy_) / res_ + 1e-6));
    if (cell_ox == 0 && cell_oy == 0) {return;}
    std::vector<T> next(data_.size(), init_);
    const int sx = static_cast<int>(sx_), sy = static_cast<int>(sy_);
    for (int y = 0; y < sy; ++y) {
      const int src_y = y + cell_oy;
      if (src_y < 0 || src_y >= sy) {continue;}
      for (int x = 0; x < sx; ++x) {
        const int src_x = x + cell_ox;
        if (src_x < 0 || src_x >= sx) {continue;}
        next[static_cast<size_t>(y) * sx_ + x] = data_[static_cast<size_t>(src_y) * sx_ + src_x];
      }
    }
    data_.swap(next);
    ox_ += cell_ox * res_;
    oy_ += cell_oy * res_;
  }

private:
  unsigned sx_{0}, sy_{0};
  double res_{0.05};
  double ox_{0.0}, oy_{0.0};
  T init_{};
  std::vector<T> data_;
};

}  // namespace g1d::nav::costmap
