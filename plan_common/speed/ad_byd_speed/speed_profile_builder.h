//
// Created by xxx on 10/26/21.
//

#ifndef PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_BUILDER_H_
#define PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_BUILDER_H_
#include "plan_common/speed/ad_byd_speed/speed_profile.h"

namespace ad_byd {
namespace planning {
class SpeedProfileBuilder {
 public:
  SpeedProfileBuilder() = default;
  ~SpeedProfileBuilder() = default;

  /// @brief build speed profile according to start point and end point
  /// @param start start speed point
  /// @param end end speed point
  /// @param time_length speed profile time length
  /// @return nullptr if fail
  virtual SpeedProfilePtr Build(const SpeedPoint &start, const SpeedPoint &end,
                                const double &time_length) = 0;

 private:
};
}  // namespace planning
}  // namespace ad_byd

#endif  // PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_BUILDER_H_
