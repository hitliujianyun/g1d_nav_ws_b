#include "g1d/lio/preprocess.hpp"

#include <cmath>

namespace g1d::lio
{

fast_lio_api::Cloud::Ptr preprocessFrame(
  const LidarFrame & frame, const PreprocessParams & params, const SelfFilter & self_filter,
  PreprocessStats * stats)
{
  auto cloud = std::make_shared<fast_lio_api::Cloud>();
  cloud->reserve(frame.points.size());
  PreprocessStats st;
  st.input = static_cast<int>(frame.points.size());
  const double blind2 = params.blind * params.blind;
  const double max2 = params.max_range * params.max_range;
  const int filter_num = params.point_filter_num < 1 ? 1 : params.point_filter_num;

  unsigned valid_num = 0;
  const FramePoint * prev = nullptr;
  for (size_t i = 0; i < frame.points.size(); ++i) {
    const FramePoint & p = frame.points[i];
    // Livox tag bits 4-5: 0x00 / 0x10 are normal returns (same test as FAST-LIO).
    if (!((p.tag & 0x30) == 0x10 || (p.tag & 0x30) == 0x00)) {
      ++st.tag_rejected;
      prev = &p;
      continue;
    }
    ++valid_num;
    if (valid_num % filter_num != 0) {
      prev = &p;
      continue;
    }
    const bool duplicate = prev && std::abs(p.x - prev->x) <= 1e-7f &&
      std::abs(p.y - prev->y) <= 1e-7f && std::abs(p.z - prev->z) <= 1e-7f;
    prev = &p;
    if (duplicate) {continue;}
    const double r2 = double(p.x) * p.x + double(p.y) * p.y + double(p.z) * p.z;
    if (r2 <= blind2 || r2 > max2) {
      ++st.blind_rejected;
      continue;
    }
    if (self_filter.isSelf(p.x, p.y, p.z)) {
      ++st.self_rejected;
      continue;
    }
    fast_lio_api::Point q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = p.intensity;
    q.curvature = p.offset_ms;
    cloud->push_back(q);
  }
  st.output = static_cast<int>(cloud->size());
  if (stats) {*stats = st;}
  return cloud;
}

}  // namespace g1d::lio
