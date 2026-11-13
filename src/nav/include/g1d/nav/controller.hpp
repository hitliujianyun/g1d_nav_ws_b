#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "g1d/common/geometry.hpp"
#include "g1d/nav/types.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace mppi
{
class Optimizer;
class ParametersHandler;
}
namespace nav2
{
class LifecycleNode;
}

namespace g1d::nav
{

struct ControllerParams
{
  double controller_frequency{20.0};
  double prune_distance{1.7};        // local path length handed to MPPI (m)
  double search_window{2.0};         // closest-point search window along the path (m)
  YAML::Node mppi;                   // the `mppi:` subtree (Nav2 MPPI parameter names)

  static ControllerParams fromYaml(const YAML::Node & root);
};

/// Nav2 MPPI controller (Eigen version) running outside ROS: same Optimizer, critics and
/// motion model; parameters come from YAML under the `FollowPath` namespace.
class MppiController
{
public:
  MppiController(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, const ControllerParams & params);
  ~MppiController();

  void setPlan(const std::vector<Pose2D> & global_path);
  /// One control step. Returns false when MPPI cannot find a collision-free command
  /// (nav2_core::NoValidControl); `error` then holds the reason.
  bool computeCommand(
    const Pose2D & pose, const Twist2D & velocity, const Pose2D & goal, Twist2D & cmd,
    std::string * error = nullptr);
  /// Absolute linear speed limit (m/s); <= 0 removes the limit.
  void setSpeedLimit(double v);
  void reset();

  const std::vector<Pose2D> & localPath() const {return local_path_;}
  /// Optimal trajectory of the last step (rows: x, y, yaw).
  const Eigen::ArrayXXf & optimalTrajectory() const {return optimal_;}
  double maxVx() const {return vx_max_;}
  double maxWz() const {return wz_max_;}

private:
  void prunePath(const Pose2D & pose);

  ControllerParams p_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<nav2::LifecycleNode> node_;
  std::unique_ptr<mppi::ParametersHandler> param_handler_;
  std::unique_ptr<mppi::Optimizer> optimizer_;
  std::vector<Pose2D> global_path_;
  std::vector<Pose2D> local_path_;
  size_t closest_idx_{0};
  Eigen::ArrayXXf optimal_;
  double vx_max_{0.5};
  double wz_max_{1.9};
};

}  // namespace g1d::nav
