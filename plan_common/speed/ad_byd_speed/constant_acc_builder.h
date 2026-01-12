//
// Created by xxx on 10/26/21.
//

#ifndef PILOT_PLANNING_COMMON_SPEED_CONSTANT_ACC_BUILDER_H_
#define PILOT_PLANNING_COMMON_SPEED_CONSTANT_ACC_BUILDER_H_
#include "plan_common/speed/ad_byd_speed/speed_profile_builder.h"

namespace ad_byd {
namespace planning {
class ConstantAccBuilder : public SpeedProfileBuilder {
 public:
  ConstantAccBuilder() = default;
  ~ConstantAccBuilder() = default;

  /// @brief before reach target speed, the acceleration would be constant which
  /// is indicated by end.a and then the acceleration would be 0
  /// @param start t, s, v, a is useful
  /// @param end v, a is useful while t, s is useless.
  /// v:target speed, a:target acceleration before reach target speed
  /// @param time_length speed profile time length, point on profile will
  /// not exceed start.t + time_length
  /// @return nullptr if fail
  SpeedProfilePtr Build(const SpeedPoint &start, const SpeedPoint &end,
                        const double &time_length) override;

  void set_interval(const double &interval) { interval_ = interval; };

 private:
  double interval_ = 0.1;

  /// @brief compute jerk according to current a and target_a
  double ComputeJerk(const SpeedPoint &pt, const double &target_a,
                     const double &t_interval) const;
};
}  // namespace planning
}  // namespace ad_byd

#endif  // PILOT_PLANNING_COMMON_SPEED_CONSTANT_ACC_BUILDER_H_
