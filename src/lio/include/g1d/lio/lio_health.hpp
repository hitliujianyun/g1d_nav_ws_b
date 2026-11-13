#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "fast_lio_api.h"

namespace g1d::lio
{

struct HealthParams
{
  int min_effective_points{300};
  int bad_frames_to_fail{3};
  double max_step_translation_m{0.3};   // per scan (10 Hz => 3 m/s)
  double max_step_yaw_deg{15.0};
  double max_process_time_s{0.15};
  double max_z_drift_m{0.3};            // indoor flat floor; column locked during navigation
  double min_normal_eigenvalue{0.02};   // below: geometry degenerate (warning only)
};

struct HealthStatus
{
  bool ok{false};
  bool degenerate{false};
  int consecutive_bad{0};
  std::vector<std::string> errors;     // reasons the current frame is bad
  std::string summary() const;
};

/// Per-frame plausibility checks on FAST-LIO output (see design doc §6.2).
class LioHealth
{
public:
  explicit LioHealth(HealthParams p = {})
  : p_(p) {}

  HealthStatus update(const fast_lio_api::Result & r);
  void reset();
  const HealthStatus & status() const {return status_;}

private:
  HealthParams p_;
  HealthStatus status_;
  std::optional<Eigen::Vector3d> last_pos_;
  std::optional<Eigen::Quaterniond> last_rot_;
  std::optional<double> z0_;
};

}  // namespace g1d::lio
