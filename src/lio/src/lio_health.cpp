#include "g1d/lio/lio_health.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "g1d/common/geometry.hpp"

namespace g1d::lio
{

std::string HealthStatus::summary() const
{
  std::ostringstream ss;
  ss << (ok ? "ok" : "bad");
  if (degenerate) {ss << ",degenerate";}
  for (const auto & e : errors) {ss << "," << e;}
  return ss.str();
}

void LioHealth::reset()
{
  status_ = HealthStatus();
  last_pos_.reset();
  last_rot_.reset();
  z0_.reset();
}

HealthStatus LioHealth::update(const fast_lio_api::Result & r)
{
  std::vector<std::string> errors;
  if (r.effective_points < p_.min_effective_points) {errors.push_back("few_points");}
  if (r.process_time_s > p_.max_process_time_s) {errors.push_back("slow");}
  if (!r.pos.allFinite() || !r.rot.coeffs().allFinite()) {errors.push_back("nan");}

  if (last_pos_ && errors.empty()) {
    const double step = (r.pos - *last_pos_).norm();
    const Eigen::Matrix3d dR = last_rot_->toRotationMatrix().transpose() * r.rot.toRotationMatrix();
    const double dyaw = std::abs(yawOf(dR)) * 180.0 / M_PI;
    if (step > p_.max_step_translation_m) {errors.push_back("jump_translation");}
    if (dyaw > p_.max_step_yaw_deg) {errors.push_back("jump_yaw");}
  }
  if (!z0_ && errors.empty()) {z0_ = r.pos.z();}
  if (z0_ && std::abs(r.pos.z() - *z0_) > p_.max_z_drift_m) {errors.push_back("z_drift");}

  status_.degenerate = r.normal_eig_min < p_.min_normal_eigenvalue;
  if (errors.empty()) {
    status_.consecutive_bad = 0;
    last_pos_ = r.pos;
    last_rot_ = r.rot;
  } else {
    ++status_.consecutive_bad;
  }
  status_.errors = errors;
  // A single bad frame is tolerated; `bad_frames_to_fail` consecutive ones declare loss.
  status_.ok = r.ekf_inited && status_.consecutive_bad < p_.bad_frames_to_fail &&
    std::find(errors.begin(), errors.end(), "nan") == errors.end();
  return status_;
}

}  // namespace g1d::lio
