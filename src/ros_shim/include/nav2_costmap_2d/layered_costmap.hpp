#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace nav2_costmap_2d
{

/// Holds the master grid, the layer list and footprint radii, as read by MPPI critics.
/// Layer composition itself is done by g1d::costmap::LayeredCostmap.
class LayeredCostmap
{
public:
  explicit LayeredCostmap(Costmap2D * master = nullptr)
  : costmap_(master) {}

  Costmap2D * getCostmap() {return costmap_;}
  void setCostmap(Costmap2D * c) {costmap_ = c;}

  std::vector<std::shared_ptr<Layer>> * getPlugins() {return &plugins_;}
  void addPlugin(std::shared_ptr<Layer> layer) {plugins_.push_back(std::move(layer));}

  void setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint)
  {
    footprint_ = footprint;
    std::tie(inscribed_radius_, circumscribed_radius_) = calculateMinAndMaxDistances(footprint_);
  }
  const std::vector<geometry_msgs::msg::Point> & getFootprint() const {return footprint_;}
  double getCircumscribedRadius() const {return circumscribed_radius_;}
  double getInscribedRadius() const {return inscribed_radius_;}

  bool isTrackingUnknown() const {return track_unknown_;}
  void setTrackingUnknown(bool v) {track_unknown_ = v;}

private:
  Costmap2D * costmap_{nullptr};
  std::vector<std::shared_ptr<Layer>> plugins_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  double inscribed_radius_{0.0};
  double circumscribed_radius_{0.0};
  bool track_unknown_{false};
};

}  // namespace nav2_costmap_2d
