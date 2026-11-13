#pragma once

#include <string>

namespace rclcpp
{

class Logger
{
public:
  Logger() = default;
  explicit Logger(std::string name)
  : name_(std::move(name)) {}
  const char * get_name() const {return name_.c_str();}
  Logger get_child(const std::string & suffix) const {return Logger(name_ + "." + suffix);}

private:
  std::string name_{"g1d"};
};

inline Logger get_logger(const std::string & name) {return Logger(name);}

}  // namespace rclcpp
