#pragma once

#include <memory>

namespace nav2
{

/// TF is not used: all g1d navigation data is already expressed in the `odom` frame.
class TransformBuffer
{
public:
  using SharedPtr = std::shared_ptr<TransformBuffer>;
};

}  // namespace nav2
