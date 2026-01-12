#ifndef PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_UTILS_H_
#define PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_UTILS_H_
#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {
namespace speed_profile_utils {

/// @brief calculate v and s at t by given const a, if stopped at given t,
/// output v = 0, a remain unchanged instead of output 0
SpeedPoint CalConstAccSpeedPoint(const SpeedPoint &start, const double a,
                                 const double t);

/// @brief calculate a, v and s at t by given const jerk, if stopped at given t,
/// output v = 0, a remain unchanged
SpeedPoint CalConstJerkSpeedPoint(const SpeedPoint &start, const double jerk,
                                  const double t);

}  // namespace speed_profile_utils

}  // namespace planning
}  // namespace ad_byd

#endif
