#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_costmap_2d
{

using Footprint = std::vector<geometry_msgs::msg::Point>;

/// Read-only facade over a g1d costmap, exposing the Costmap2DROS methods that MPPI uses.
class Costmap2DROS
{
public:
  Costmap2DROS(
    std::shared_ptr<LayeredCostmap> layered, std::string global_frame, std::string base_frame)
  : layered_(std::move(layered)), global_frame_(std::move(global_frame)),
    base_frame_(std::move(base_frame)) {}

  Costmap2D * getCostmap() {return layered_->getCostmap();}
  LayeredCostmap * getLayeredCostmap() {return layered_.get();}
  Footprint getRobotFootprint() const {return layered_->getFootprint();}
  bool getUseRadius() const {return use_radius_;}
  void setUseRadius(bool v) {use_radius_ = v;}
  double getRobotRadius() const {return layered_->getCircumscribedRadius();}
  const std::string & getGlobalFrameID() const {return global_frame_;}
  const std::string & getBaseFrameID() const {return base_frame_;}

private:
  std::shared_ptr<LayeredCostmap> layered_;
  std::string global_frame_;
  std::string base_frame_;
  bool use_radius_{false};
};

}  // namespace nav2_costmap_2d
