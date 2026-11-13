#pragma once

#include <cstdint>

namespace builtin_interfaces
{
namespace msg
{

struct Time
{
  int32_t sec{0};
  uint32_t nanosec{0};
  bool operator==(const Time & o) const {return sec == o.sec && nanosec == o.nanosec;}
  bool operator!=(const Time & o) const {return !(*this == o);}
};

}  // namespace msg
}  // namespace builtin_interfaces
