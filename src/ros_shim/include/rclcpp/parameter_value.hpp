#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace rclcpp
{

enum ParameterType
{
  PARAMETER_NOT_SET = 0,
  PARAMETER_BOOL,
  PARAMETER_INTEGER,
  PARAMETER_DOUBLE,
  PARAMETER_STRING,
  PARAMETER_BYTE_ARRAY,
  PARAMETER_BOOL_ARRAY,
  PARAMETER_INTEGER_ARRAY,
  PARAMETER_DOUBLE_ARRAY,
  PARAMETER_STRING_ARRAY,
};

class ParameterTypeException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Holds one parameter value. Numeric reads convert between integer and double so that YAML
/// values such as `1` can initialise float settings.
class ParameterValue
{
public:
  using Storage = std::variant<
    std::monostate, bool, int64_t, double, std::string,
    std::vector<uint8_t>, std::vector<bool>, std::vector<int64_t>, std::vector<double>,
    std::vector<std::string>>;

  ParameterValue() = default;
  ParameterValue(bool v) : v_(v) {}  // NOLINT
  ParameterValue(int v) : v_(static_cast<int64_t>(v)) {}  // NOLINT
  ParameterValue(unsigned int v) : v_(static_cast<int64_t>(v)) {}  // NOLINT
  ParameterValue(long v) : v_(static_cast<int64_t>(v)) {}  // NOLINT
  ParameterValue(unsigned long v) : v_(static_cast<int64_t>(v)) {}  // NOLINT
  ParameterValue(long long v) : v_(static_cast<int64_t>(v)) {}  // NOLINT
  ParameterValue(unsigned long long v) : v_(static_cast<int64_t>(v)) {}  // NOLINT
  ParameterValue(float v) : v_(static_cast<double>(v)) {}  // NOLINT
  ParameterValue(double v) : v_(v) {}  // NOLINT
  ParameterValue(const char * v) : v_(std::string(v)) {}  // NOLINT
  ParameterValue(std::string v) : v_(std::move(v)) {}  // NOLINT
  ParameterValue(std::vector<bool> v) : v_(std::move(v)) {}  // NOLINT
  ParameterValue(std::vector<uint8_t> v) : v_(std::move(v)) {}  // NOLINT
  ParameterValue(std::vector<int64_t> v) : v_(std::move(v)) {}  // NOLINT
  ParameterValue(const std::vector<int> & v) : v_(std::vector<int64_t>(v.begin(), v.end())) {}  // NOLINT
  ParameterValue(std::vector<double> v) : v_(std::move(v)) {}  // NOLINT
  ParameterValue(const std::vector<float> & v) : v_(std::vector<double>(v.begin(), v.end())) {}  // NOLINT
  ParameterValue(std::vector<std::string> v) : v_(std::move(v)) {}  // NOLINT

  ParameterType get_type() const {return static_cast<ParameterType>(v_.index());}
  bool is_set() const {return v_.index() != 0;}

  bool as_bool() const
  {
    if (auto p = std::get_if<bool>(&v_)) {return *p;}
    if (auto p = std::get_if<int64_t>(&v_)) {return *p != 0;}
    throw ParameterTypeException("parameter is not a bool");
  }
  int64_t as_int() const
  {
    if (auto p = std::get_if<int64_t>(&v_)) {return *p;}
    if (auto p = std::get_if<double>(&v_)) {return static_cast<int64_t>(*p);}
    if (auto p = std::get_if<bool>(&v_)) {return *p ? 1 : 0;}
    throw ParameterTypeException("parameter is not an integer");
  }
  double as_double() const
  {
    if (auto p = std::get_if<double>(&v_)) {return *p;}
    if (auto p = std::get_if<int64_t>(&v_)) {return static_cast<double>(*p);}
    throw ParameterTypeException("parameter is not a double");
  }
  const std::string & as_string() const
  {
    if (auto p = std::get_if<std::string>(&v_)) {return *p;}
    throw ParameterTypeException("parameter is not a string");
  }
  std::vector<bool> as_bool_array() const
  {
    if (auto p = std::get_if<std::vector<bool>>(&v_)) {return *p;}
    return {};
  }
  std::vector<int64_t> as_integer_array() const
  {
    if (auto p = std::get_if<std::vector<int64_t>>(&v_)) {return *p;}
    if (auto p = std::get_if<std::vector<double>>(&v_)) {
      return std::vector<int64_t>(p->begin(), p->end());
    }
    return {};
  }
  std::vector<double> as_double_array() const
  {
    if (auto p = std::get_if<std::vector<double>>(&v_)) {return *p;}
    if (auto p = std::get_if<std::vector<int64_t>>(&v_)) {
      return std::vector<double>(p->begin(), p->end());
    }
    return {};
  }
  std::vector<std::string> as_string_array() const
  {
    if (auto p = std::get_if<std::vector<std::string>>(&v_)) {return *p;}
    return {};
  }

  template<typename T>
  T get() const
  {
    if constexpr (std::is_same_v<T, bool>) {
      return as_bool();
    } else if constexpr (std::is_integral_v<T>) {
      return static_cast<T>(as_int());
    } else if constexpr (std::is_floating_point_v<T>) {
      return static_cast<T>(as_double());
    } else if constexpr (std::is_same_v<T, std::string>) {
      return as_string();
    } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
      return as_string_array();
    } else if constexpr (std::is_same_v<T, std::vector<double>>) {
      return as_double_array();
    } else if constexpr (std::is_same_v<T, std::vector<float>>) {
      auto d = as_double_array();
      return std::vector<float>(d.begin(), d.end());
    } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) {
      return as_integer_array();
    } else if constexpr (std::is_same_v<T, std::vector<int>>) {
      auto d = as_integer_array();
      return std::vector<int>(d.begin(), d.end());
    } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
      return as_bool_array();
    } else {
      static_assert(sizeof(T) == 0, "unsupported parameter type");
    }
  }

  std::string to_string() const
  {
    switch (v_.index()) {
      case 1: return as_bool() ? "true" : "false";
      case 2: return std::to_string(as_int());
      case 3: return std::to_string(as_double());
      case 4: return as_string();
      default: return "<array/unset>";
    }
  }

private:
  Storage v_;
};

}  // namespace rclcpp
