#pragma once

#include <string>

namespace nav2_costmap_2d
{

/// Minimal base for costmap layers; only what InflationLayerInterface lookups need.
class Layer
{
public:
  virtual ~Layer() = default;
  const std::string & getName() const {return name_;}
  void setName(const std::string & name) {name_ = name;}

protected:
  std::string name_;
};

}  // namespace nav2_costmap_2d
