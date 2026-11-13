#include "g1d/sim/world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace g1d::sim
{

void World::addBox(const Eigen::Vector3d & min, const Eigen::Vector3d & max, std::string name)
{
  boxes_.push_back({min.cwiseMin(max), min.cwiseMax(max), std::move(name)});
}

void World::addRoom(double x0, double y0, double x1, double y1, double h, double t)
{
  addBox({x0 - t, y0 - t, 0.0}, {x1 + t, y0, h}, "wall_s");
  addBox({x0 - t, y1, 0.0}, {x1 + t, y1 + t, h}, "wall_n");
  addBox({x0 - t, y0, 0.0}, {x0, y1, h}, "wall_w");
  addBox({x1, y0, 0.0}, {x1 + t, y1, h}, "wall_e");
}

namespace
{
bool rayBox(
  const Eigen::Vector3d & o, const Eigen::Vector3d & inv, const Box & b, double tmax, double & t)
{
  double t0 = 0.0, t1 = tmax;
  for (int i = 0; i < 3; ++i) {
    double ta = (b.min[i] - o[i]) * inv[i];
    double tb = (b.max[i] - o[i]) * inv[i];
    if (ta > tb) {std::swap(ta, tb);}
    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);
    if (t0 > t1) {return false;}
  }
  t = t0;
  return t0 > 1e-6;
}

double segDist(const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d ab = b - a;
  const double l2 = ab.squaredNorm();
  const double u = l2 > 0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
  return (a + u * ab - p).norm();
}

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

std::vector<Eigen::Vector2d> rect(const Box & b)
{
  return {{b.min.x(), b.min.y()}, {b.max.x(), b.min.y()}, {b.max.x(), b.max.y()},
    {b.min.x(), b.max.y()}};
}

double polyPolyDistance(const std::vector<Eigen::Vector2d> & a, const std::vector<Eigen::Vector2d> & b)
{
  for (const auto & p : a) {if (pointInPolygon(p, b)) {return 0.0;}}
  for (const auto & p : b) {if (pointInPolygon(p, a)) {return 0.0;}}
  double d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < a.size(); ++i) {
    const auto & a0 = a[i];
    const auto & a1 = a[(i + 1) % a.size()];
    for (size_t j = 0; j < b.size(); ++j) {
      const auto & b0 = b[j];
      const auto & b1 = b[(j + 1) % b.size()];
      d = std::min({d, segDist(a0, b0, b1), segDist(a1, b0, b1), segDist(b0, a0, a1),
          segDist(b1, a0, a1)});
    }
  }
  // Proper segment crossing implies 0 distance; the endpoint distances above already catch
  // touching cases, crossing segments are detected here.
  for (size_t i = 0; i < a.size(); ++i) {
    const auto & p = a[i];
    const auto & r = a[(i + 1) % a.size()] - p;
    for (size_t j = 0; j < b.size(); ++j) {
      const auto & q = b[j];
      const auto & s = b[(j + 1) % b.size()] - q;
      const double den = r.x() * s.y() - r.y() * s.x();
      if (std::abs(den) < 1e-12) {continue;}
      const Eigen::Vector2d qp = q - p;
      const double t = (qp.x() * s.y() - qp.y() * s.x()) / den;
      const double u = (qp.x() * r.y() - qp.y() * r.x()) / den;
      if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {return 0.0;}
    }
  }
  return d;
}
}  // namespace

std::optional<double> World::raycast(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & dir, double max_range) const
{
  double best = max_range;
  bool hit = false;
  const Eigen::Vector3d inv(
    1.0 / (std::abs(dir.x()) > 1e-12 ? dir.x() : 1e-12),
    1.0 / (std::abs(dir.y()) > 1e-12 ? dir.y() : 1e-12),
    1.0 / (std::abs(dir.z()) > 1e-12 ? dir.z() : 1e-12));
  if (dir.z() < -1e-9) {
    const double t = -origin.z() / dir.z();
    if (t > 0 && t < best) {best = t; hit = true;}
  }
  if (ceiling_ && dir.z() > 1e-9) {
    const double t = (*ceiling_ - origin.z()) / dir.z();
    if (t > 0 && t < best) {best = t; hit = true;}
  }
  for (const auto & b : boxes_) {
    double t;
    if (rayBox(origin, inv, b, best, t) && t < best) {best = t; hit = true;}
  }
  if (!hit) {return std::nullopt;}
  return best;
}

bool World::footprintCollides(
  const std::vector<Eigen::Vector2d> & polygon, double z_min, double z_max) const
{
  return footprintClearance(polygon, z_min, z_max) <= 0.0;
}

double World::footprintClearance(
  const std::vector<Eigen::Vector2d> & polygon, double z_min, double z_max) const
{
  double d = std::numeric_limits<double>::infinity();
  for (const auto & b : boxes_) {
    if (b.max.z() < z_min || b.min.z() > z_max) {continue;}
    d = std::min(d, polyPolyDistance(polygon, rect(b)));
    if (d <= 0.0) {return 0.0;}
  }
  return d;
}

World World::makeTestRoom()
{
  World w;
  w.addRoom(-2.0, -5.0, 10.0, 5.0, 2.8);
  w.addBox({2.0, 1.0, 0.0}, {2.6, 1.6, 0.8}, "box_a");
  w.addBox({4.5, -2.5, 0.0}, {5.3, -1.7, 1.0}, "box_b");
  w.addBox({6.5, 2.0, 0.0}, {7.0, 2.5, 1.8}, "pillar_a");
  w.addBox({-1.0, -4.0, 0.0}, {-0.5, -3.5, 2.8}, "pillar_b");
  w.addBox({7.5, -4.2, 0.0}, {9.5, -3.2, 1.2}, "shelf");
  // Table: top at 0.72-0.76 m on four thin legs (overhang test).
  w.addBox({3.0, -4.5, 0.72}, {4.2, -3.7, 0.76}, "table_top");
  w.addBox({3.0, -4.5, 0.0}, {3.05, -4.45, 0.72}, "table_leg1");
  w.addBox({4.15, -4.5, 0.0}, {4.2, -4.45, 0.72}, "table_leg2");
  w.addBox({3.0, -3.75, 0.0}, {3.05, -3.7, 0.72}, "table_leg3");
  w.addBox({4.15, -3.75, 0.0}, {4.2, -3.7, 0.72}, "table_leg4");
  w.addBox({8.5, 3.0, 0.0}, {9.8, 4.8, 0.45}, "low_bench");
  return w;
}

}  // namespace g1d::sim
