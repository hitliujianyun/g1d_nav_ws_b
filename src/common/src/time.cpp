#include "g1d/common/time.hpp"

#include <thread>

namespace g1d
{

void Rate::sleep()
{
  const auto now = std::chrono::steady_clock::now();
  if (next_ > now) {
    std::this_thread::sleep_until(next_);
    next_ += period_;
  } else {
    // Overran: restart the schedule instead of bursting to catch up.
    next_ = now + period_;
  }
}

}  // namespace g1d
