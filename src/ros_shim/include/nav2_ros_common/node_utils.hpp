#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2
{

template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node, const std::string & name, const rclcpp::ParameterValue & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_value);
  }
}

/// Reads `<plugin_name>.plugin`; throws if it is missing or empty, like the Nav2 helper.
template<typename NodeT>
std::string get_plugin_type_param(NodeT node, const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::ParameterValue(""));
  std::string plugin_type;
  node->get_parameter(plugin_name + ".plugin", plugin_type);
  if (plugin_type.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Can not proceed without '%s.plugin' parameter",
      plugin_name.c_str());
    throw std::runtime_error("No 'plugin' parameter defined for " + plugin_name);
  }
  return plugin_type;
}

}  // namespace nav2
