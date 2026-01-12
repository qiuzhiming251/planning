

#include "plan_common/util/hr_timer.h"

namespace ad_byd {
namespace planning {

void HRTimer::Restart() { start_ = std::chrono::high_resolution_clock::now(); }

// by default is nano seconds, no need to cast
long HRTimer::ElapsedNs() const {
  return (std::chrono::high_resolution_clock::now() - start_).count();
}

long HRTimer::ElapsedUs() const {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::high_resolution_clock::now() - start_)
      .count();
}

long HRTimer::ElapsedMs() const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now() - start_)
      .count();
}

long HRTimer::ElapsedS() const {
  return std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::high_resolution_clock::now() - start_)
      .count();
}

}  // namespace planning
}  // namespace ad_byd