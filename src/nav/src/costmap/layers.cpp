#include "g1d/nav/costmap/layers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace g1d::nav::costmap
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;

bool pointInPolygon(const Eigen::Vector2d & p, const std::vector<Eigen::Vector2d> & poly)
{
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (((poly[i].y() > p.y()) != (poly[j].y() > p.y())) &&
      (p.x() < (poly[j].x() - poly[i].x()) * (p.y() - poly[i].y()) /
      (poly[j].y() - poly[i].y()) + poly[i].x()))
    {
      inside = !inside;
    }
  }
  return inside;
}

namespace
{
template<typename Grid, typename Fn>
void forCellsInPolygon(const Grid & g, const std::vector<Eigen::Vector2d> & poly, Fn && fn)
{
  if (poly.size() < 3) {return;}
  double minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
  for (const auto & p : poly) {
    minx = std::min(minx, p.x()); maxx = std::max(maxx, p.x());
    miny = std::min(miny, p.y()); maxy = std::max(maxy, p.y());
  }
  int x0, y0, x1, y1;
  g.worldToMap(minx, miny, x0, y0);
  g.worldToMap(maxx, maxy, x1, y1);
  x0 = std::max(x0, 0); y0 = std::max(y0, 0);
  x1 = std::min(x1, static_cast<int>(g.sizeX()) - 1);
  y1 = std::min(y1, static_cast<int>(g.sizeY()) - 1);
  const double res = g.resolution();
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const Eigen::Vector2d c(g.originX() + (x + 0.5) * res, g.originY() + (y + 0.5) * res);
      if (pointInPolygon(c, poly)) {fn(x, y);}
    }
  }
}

template<typename Grid, typename Fn>
void forCellsOutsideRadius(const Grid & g, double cx, double cy, double r, Fn && fn)
{
  const double res = g.resolution();
  const double r2 = r * r;
  for (int y = 0; y < static_cast<int>(g.sizeY()); ++y) {
    for (int x = 0; x < static_cast<int>(g.sizeX()); ++x) {
      const double dx = g.originX() + (x + 0.5) * res - cx;
      const double dy = g.originY() + (y + 0.5) * res - cy;
      if (dx * dx + dy * dy > r2) {fn(x, y);}
    }
  }
}
}  // namespace

// ------------------------------------------------------------------------------- VoxelLayer
void VoxelLayer::configure(
  const VoxelLayerParams & p, unsigned sx, unsigned sy, double res, double ox, double oy)
{
  p_ = p;
  nz_ = static_cast<int>(std::ceil((p.z_max - p.z_min) / p.z_resolution));
  nz_ = std::clamp(nz_, 1, 32);
  bits_.resize(sx, sy, res, ox, oy, 0u);
  hits_.resize(sx, sy, res, ox, oy, 0u);
  stamp_.resize(sx, sy, res, ox, oy, 0.0f);
}

void VoxelLayer::updateOrigin(double ox, double oy)
{
  bits_.updateOrigin(ox, oy);
  hits_.updateOrigin(ox, oy);
  stamp_.updateOrigin(ox, oy);
}

bool VoxelLayer::occupied(int mx, int my) const
{
  return bits_.inBounds(mx, my) && bits_.at(mx, my) != 0u &&
         hits_.at(mx, my) >= static_cast<uint16_t>(p_.min_points_per_column);
}

void VoxelLayer::clearRay(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  // Amanatides-Woo traversal in cell units (xy: res, z: z_resolution); the end cell is kept.
  const double res = bits_.resolution();
  const double fx0 = (a.x() - bits_.originX()) / res, fy0 = (a.y() - bits_.originY()) / res;
  const double fz0 = (a.z() - p_.z_min) / p_.z_resolution;
  const double fx1 = (b.x() - bits_.originX()) / res, fy1 = (b.y() - bits_.originY()) / res;
  const double fz1 = (b.z() - p_.z_min) / p_.z_resolution;
  int x = static_cast<int>(std::floor(fx0)), y = static_cast<int>(std::floor(fy0));
  int z = static_cast<int>(std::floor(fz0));
  const int xe = static_cast<int>(std::floor(fx1)), ye = static_cast<int>(std::floor(fy1));
  const int ze = static_cast<int>(std::floor(fz1));
  const double dx = fx1 - fx0, dy = fy1 - fy0, dz = fz1 - fz0;
  const int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1, sz = dz > 0 ? 1 : -1;
  const double inf = std::numeric_limits<double>::infinity();
  const double tdx = dx != 0 ? std::abs(1.0 / dx) : inf;
  const double tdy = dy != 0 ? std::abs(1.0 / dy) : inf;
  const double tdz = dz != 0 ? std::abs(1.0 / dz) : inf;
  double tmx = dx != 0 ? ((sx > 0 ? (x + 1 - fx0) : (fx0 - x)) * tdx) : inf;
  double tmy = dy != 0 ? ((sy > 0 ? (y + 1 - fy0) : (fy0 - y)) * tdy) : inf;
  double tmz = dz != 0 ? ((sz > 0 ? (z + 1 - fz0) : (fz0 - z)) * tdz) : inf;
  const int max_steps = std::abs(xe - x) + std::abs(ye - y) + std::abs(ze - z) + 2;
  for (int i = 0; i < max_steps; ++i) {
    if (x == xe && y == ye && z == ze) {break;}
    if (bits_.inBounds(x, y) && z >= 0 && z < nz_) {
      uint32_t & col = bits_.at(x, y);
      if (col & (1u << z)) {
        col &= ~(1u << z);
        if (col == 0u) {hits_.at(x, y) = 0;}
      }
    }
    if (tmx < tmy && tmx < tmz) {x += sx; tmx += tdx;} else if (tmy < tmz) {y += sy; tmy += tdy;} else {
      z += sz; tmz += tdz;
    }
  }
}

void VoxelLayer::addCloud(
  const std::vector<Eigen::Vector3f> & points_odom, const Eigen::Vector3d & sensor, double stamp)
{
  if (!has_t0_) {t0_ = stamp; has_t0_ = true;}
  const float ts = static_cast<float>(stamp - t0_);
  const double mark2 = p_.obstacle_max_range * p_.obstacle_max_range;
  const int every = std::max(1, p_.clear_every_n);
  // Clear first, then mark, so points of this scan are never erased by its own rays.
  for (size_t i = 0; i < points_odom.size(); i += every) {
    const Eigen::Vector3d q = points_odom[i].cast<double>();
    Eigen::Vector3d d = q - sensor;
    const double n = d.norm();
    if (n < 1e-3) {continue;}
    const Eigen::Vector3d end = n > p_.raytrace_max_range ? sensor + d * (p_.raytrace_max_range / n) : q;
    clearRay(sensor, end);
  }
  for (const auto & pf : points_odom) {
    const Eigen::Vector3d q = pf.cast<double>();
    if (q.z() < p_.z_min || q.z() >= p_.z_max) {continue;}
    if ((q - sensor).squaredNorm() > mark2) {continue;}
    int mx, my;
    if (!bits_.worldToMap(q.x(), q.y(), mx, my)) {continue;}
    const int z = static_cast<int>(std::floor((q.z() - p_.z_min) / p_.z_resolution));
    if (z < 0 || z >= nz_) {continue;}
    bits_.at(mx, my) |= (1u << z);
    uint16_t & h = hits_.at(mx, my);
    if (h < 0xFFFF) {++h;}
    stamp_.at(mx, my) = ts;
  }
}

void VoxelLayer::clearPolygon(const std::vector<Eigen::Vector2d> & poly)
{
  forCellsInPolygon(bits_, poly, [&](int x, int y) {bits_.at(x, y) = 0u; hits_.at(x, y) = 0;});
}

void VoxelLayer::clearOutsideRadius(double cx, double cy, double r)
{
  forCellsOutsideRadius(bits_, cx, cy, r, [&](int x, int y) {bits_.at(x, y) = 0u; hits_.at(x, y) = 0;});
}

void VoxelLayer::decay(double now)
{
  if (p_.decay_s <= 0.0 || !has_t0_) {return;}
  const float limit = static_cast<float>(now - t0_ - p_.decay_s);
  auto & b = bits_.data();
  auto & h = hits_.data();
  const auto & s = stamp_.data();
  for (size_t i = 0; i < b.size(); ++i) {
    if (b[i] != 0u && s[i] < limit) {b[i] = 0u; h[i] = 0;}
  }
}

void VoxelLayer::writeTo(nav2_costmap_2d::Costmap2D & master) const
{
  const unsigned sx = std::min(master.getSizeInCellsX(), bits_.sizeX());
  const unsigned sy = std::min(master.getSizeInCellsY(), bits_.sizeY());
  for (unsigned y = 0; y < sy; ++y) {
    for (unsigned x = 0; x < sx; ++x) {
      if (occupied(static_cast<int>(x), static_cast<int>(y))) {
        master.setCost(x, y, LETHAL_OBSTACLE);
      }
    }
  }
}

// -------------------------------------------------------------------------------- ScanLayer
void ScanLayer::configure(
  const ScanLayerParams & p, unsigned sx, unsigned sy, double res, double ox, double oy)
{
  p_ = p;
  occ_.resize(sx, sy, res, ox, oy, 0u);
}

void ScanLayer::addScan(const ScanData & scan, const Pose2D & sensor)
{
  const double res = occ_.resolution();
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double r = scan.ranges[i];
    bool mark = true;
    if (!std::isfinite(r) || r > scan.range_max) {
      if (!p_.inf_is_valid) {continue;}
      r = p_.raytrace_max_range;
      mark = false;
    }
    if (r < scan.range_min) {continue;}
    const double a = sensor.yaw + scan.angle_min + i * scan.angle_increment;
    const double ca = std::cos(a), sa = std::sin(a);
    const double rc = std::min(r, p_.raytrace_max_range);
    // 2D clearing along the beam, end cell excluded.
    const int steps = static_cast<int>(rc / (0.5 * res));
    int last_x = std::numeric_limits<int>::min(), last_y = last_x;
    int ex, ey;
    occ_.worldToMap(sensor.x + ca * r, sensor.y + sa * r, ex, ey);
    for (int k = 0; k < steps; ++k) {
      const double d = k * 0.5 * res;
      int mx, my;
      if (!occ_.worldToMap(sensor.x + ca * d, sensor.y + sa * d, mx, my)) {continue;}
      if ((mx == last_x && my == last_y) || (mx == ex && my == ey)) {continue;}
      occ_.at(mx, my) = 0u;
      last_x = mx;
      last_y = my;
    }
    if (mark && r <= p_.obstacle_max_range && occ_.inBounds(ex, ey)) {
      occ_.at(ex, ey) = 1u;
    }
  }
}

void ScanLayer::clearPolygon(const std::vector<Eigen::Vector2d> & poly)
{
  forCellsInPolygon(occ_, poly, [&](int x, int y) {occ_.at(x, y) = 0u;});
}

void ScanLayer::clearOutsideRadius(double cx, double cy, double r)
{
  forCellsOutsideRadius(occ_, cx, cy, r, [&](int x, int y) {occ_.at(x, y) = 0u;});
}

void ScanLayer::writeTo(nav2_costmap_2d::Costmap2D & master) const
{
  const unsigned sx = std::min(master.getSizeInCellsX(), occ_.sizeX());
  const unsigned sy = std::min(master.getSizeInCellsY(), occ_.sizeY());
  for (unsigned y = 0; y < sy; ++y) {
    for (unsigned x = 0; x < sx; ++x) {
      if (occ_.at(static_cast<int>(x), static_cast<int>(y))) {master.setCost(x, y, LETHAL_OBSTACLE);}
    }
  }
}

// --------------------------------------------------------------------------- InflationLayer
void InflationLayer::configure(const InflationParams & p, double resolution, double inscribed)
{
  std::lock_guard<mutex_t> lk(mutex_);
  p_ = p;
  resolution_ = resolution;
  inscribed_radius_ = inscribed;
  cell_radius_ = static_cast<int>(std::ceil(p.inflation_radius / resolution));
  kernel_.clear();
  for (int dy = -cell_radius_; dy <= cell_radius_; ++dy) {
    for (int dx = -cell_radius_; dx <= cell_radius_; ++dx) {
      const double d = std::hypot(dx, dy);
      if (d > cell_radius_ || (dx == 0 && dy == 0)) {continue;}
      const unsigned char c = computeCost(d);
      if (c > FREE_SPACE) {kernel_.push_back({dx, dy, c});}
    }
  }
}

unsigned char InflationLayer::computeCost(double distance) const
{
  // Same as nav2_costmap_2d::InflationLayer::computeCost.
  if (distance == 0) {return LETHAL_OBSTACLE;}
  if (distance * resolution_ <= inscribed_radius_) {return INSCRIBED_INFLATED_OBSTACLE;}
  const double factor =
    std::exp(-1.0 * p_.cost_scaling_factor * (distance * resolution_ - inscribed_radius_));
  return static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
}

void InflationLayer::inflate(nav2_costmap_2d::Costmap2D & master) const
{
  std::lock_guard<mutex_t> lk(mutex_);
  const int sx = static_cast<int>(master.getSizeInCellsX());
  const int sy = static_cast<int>(master.getSizeInCellsY());
  unsigned char * grid = master.getCharMap();
  std::vector<int> sources;
  for (int y = 0; y < sy; ++y) {
    for (int x = 0; x < sx; ++x) {
      if (grid[y * sx + x] != LETHAL_OBSTACLE) {continue;}
      // Interior lethal cells add nothing beyond their boundary neighbours.
      const bool interior = x > 0 && y > 0 && x < sx - 1 && y < sy - 1 &&
        grid[y * sx + x - 1] == LETHAL_OBSTACLE && grid[y * sx + x + 1] == LETHAL_OBSTACLE &&
        grid[(y - 1) * sx + x] == LETHAL_OBSTACLE && grid[(y + 1) * sx + x] == LETHAL_OBSTACLE;
      if (!interior) {sources.push_back(y * sx + x);}
    }
  }
  for (int idx : sources) {
    const int x = idx % sx, y = idx / sx;
    for (const auto & k : kernel_) {
      const int nx = x + k.dx, ny = y + k.dy;
      if (nx < 0 || ny < 0 || nx >= sx || ny >= sy) {continue;}
      unsigned char & c = grid[ny * sx + nx];
      if (c < k.cost) {c = k.cost;}
    }
  }
}

}  // namespace g1d::nav::costmap
