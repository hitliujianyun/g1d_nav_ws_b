#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"

namespace nav2
{

/// Publisher stand-in: Nav2 code may create publishers for visualization; nothing is sent.
template<typename MsgT>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<MsgT>>;
  void publish(const MsgT &) {}
  void publish(std::unique_ptr<MsgT>) {}
  void on_activate() {}
  void on_deactivate() {}
  size_t get_subscription_count() const {return 0;}
};

/// In-process parameter store and logger/clock provider replacing rclcpp_lifecycle nodes.
class LifecycleNode : public std::enable_shared_from_this<LifecycleNode>
{
public:
  using SharedPtr = std::shared_ptr<LifecycleNode>;
  using WeakPtr = std::weak_ptr<LifecycleNode>;

  explicit LifecycleNode(std::string name = "g1d_node")
  : name_(std::move(name)), logger_(rclcpp::get_logger(name_)),
    clock_(std::make_shared<rclcpp::Clock>()) {}

  const char * get_name() const {return name_.c_str();}
  rclcpp::Logger get_logger() const {return logger_;}
  rclcpp::Clock::SharedPtr get_clock() const {return clock_;}
  rclcpp::Time now() const {return clock_->now();}

  bool has_parameter(const std::string & name) const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return params_.count(name) > 0;
  }

  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = params_.find(name);
    if (it == params_.end()) {
      it = params_.emplace(name, default_value).first;
    }
    return it->second;
  }

  void set_parameter(const rclcpp::Parameter & p)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    params_[p.get_name()] = p.get_parameter_value();
  }

  void set_parameter(const std::string & name, const rclcpp::ParameterValue & v)
  {
    set_parameter(rclcpp::Parameter(name, v));
  }

  rclcpp::Parameter get_parameter(const std::string & name) const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = params_.find(name);
    if (it == params_.end()) {
      throw std::runtime_error("parameter not declared: " + name);
    }
    return rclcpp::Parameter(name, it->second);
  }

  template<typename T>
  bool get_parameter(const std::string & name, T & out) const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = params_.find(name);
    if (it == params_.end() || !it->second.is_set()) {
      return false;
    }
    out = it->second.template get<T>();
    return true;
  }

  std::vector<std::string> list_parameter_names() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<std::string> names;
    for (const auto & kv : params_) {names.push_back(kv.first);}
    return names;
  }

  template<typename MsgT, typename ... Args>
  typename Publisher<MsgT>::SharedPtr create_publisher(const std::string &, Args && ...)
  {
    return std::make_shared<Publisher<MsgT>>();
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
  add_on_set_parameters_callback(
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::OnParametersSetCallbackType cb)
  {
    auto h = std::make_shared<rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
    h->callback = std::move(cb);
    return h;
  }

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
  add_post_set_parameters_callback(
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::PostSetParametersCallbackType cb)
  {
    auto h = std::make_shared<rclcpp::node_interfaces::PostSetParametersCallbackHandle>();
    h->callback = std::move(cb);
    return h;
  }

  void remove_on_set_parameters_callback(
    rclcpp::node_interfaces::OnSetParametersCallbackHandle *) {}
  void remove_post_set_parameters_callback(
    rclcpp::node_interfaces::PostSetParametersCallbackHandle *) {}

private:
  std::string name_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  mutable std::mutex mutex_;
  std::map<std::string, rclcpp::ParameterValue> params_;
};

}  // namespace nav2
