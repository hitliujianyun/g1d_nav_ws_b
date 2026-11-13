#pragma once

#include <string>
#include <vector>

#include "rclcpp/parameter_value.hpp"

namespace rclcpp
{

class Parameter
{
public:
  Parameter() = default;
  Parameter(std::string name, ParameterValue value)
  : name_(std::move(name)), value_(std::move(value)) {}

  const std::string & get_name() const {return name_;}
  const ParameterValue & get_parameter_value() const {return value_;}
  ParameterType get_type() const {return value_.get_type();}

  bool as_bool() const {return value_.as_bool();}
  int64_t as_int() const {return value_.as_int();}
  double as_double() const {return value_.as_double();}
  const std::string & as_string() const {return value_.as_string();}
  std::vector<bool> as_bool_array() const {return value_.as_bool_array();}
  std::vector<int64_t> as_integer_array() const {return value_.as_integer_array();}
  std::vector<double> as_double_array() const {return value_.as_double_array();}
  std::vector<std::string> as_string_array() const {return value_.as_string_array();}

  std::string value_to_string() const {return value_.to_string();}

private:
  std::string name_;
  ParameterValue value_;
};

}  // namespace rclcpp

namespace std
{
inline std::string to_string(const rclcpp::Parameter & p)
{
  return p.get_name() + ": " + p.value_to_string();
}
}  // namespace std
