#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"

namespace rclcpp
{
namespace node_interfaces
{

struct OnSetParametersCallbackHandle
{
  using SharedPtr = std::shared_ptr<OnSetParametersCallbackHandle>;
  using OnParametersSetCallbackType =
    std::function<rcl_interfaces::msg::SetParametersResult(const std::vector<rclcpp::Parameter> &)>;
  OnParametersSetCallbackType callback;
};

struct PostSetParametersCallbackHandle
{
  using SharedPtr = std::shared_ptr<PostSetParametersCallbackHandle>;
  using PostSetParametersCallbackType = std::function<void(const std::vector<rclcpp::Parameter> &)>;
  PostSetParametersCallbackType callback;
};

}  // namespace node_interfaces
}  // namespace rclcpp
