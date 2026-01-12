

#ifndef AD_BYD_PLANNING_UTILS_HRTIMER_H
#define AD_BYD_PLANNING_UTILS_HRTIMER_H

#include <chrono>

namespace ad_byd {
namespace planning {

/**
 * @brief High resolution Timer
 */
class HRTimer {
 public:
  HRTimer() { Restart(); }

  void Restart();
  long ElapsedNs() const;
  long ElapsedUs() const;
  long ElapsedMs() const;
  long ElapsedS() const;

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_UTILS_HRTIMER_H