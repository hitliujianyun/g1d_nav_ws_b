#pragma once

#include <string>

namespace rcl_interfaces
{
namespace msg
{

struct SetParametersResult
{
  bool successful{true};
  std::string reason;
};

}  // namespace msg
}  // namespace rcl_interfaces
