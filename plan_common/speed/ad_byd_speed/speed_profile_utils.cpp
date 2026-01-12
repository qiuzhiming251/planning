#include "plan_common/speed/ad_byd_speed/speed_profile_utils.h"

namespace ad_byd {
namespace planning {
namespace speed_profile_utils {

SpeedPoint CalConstAccSpeedPoint(const SpeedPoint &start, const double a,
                                 const double t) {
  if (t < start.t) {
    return {};
  }
  SpeedPoint result_point;
  const double delta_t = t - start.t;
  if (a >= -Constants::ZERO) {
    result_point.t = t;
    result_point.a = a;
    result_point.v = start.v + a * delta_t;
    result_point.s = start.s + 0.5 * (start.v + result_point.v) * delta_t;
    return result_point;
  }
  //
  const double stop_t = start.v / std::abs(a) + start.t;
  if (t <= stop_t - Constants::ZERO) {
    result_point.t = t;
    result_point.a = a;
    result_point.v = start.v + a * delta_t;
    result_point.s = start.s + 0.5 * (start.v + result_point.v) * delta_t;
  } else {
    result_point.t = t;
    result_point.a = a;
    result_point.v = 0.0;
    result_point.s = start.s + 0.5 * start.v * (stop_t - start.t);
  }
  return result_point;
}

SpeedPoint CalConstJerkSpeedPoint(const SpeedPoint &start, const double jerk,
                                  const double t) {
  //
  if (t < start.t) {
    return {};
  }
  if (std::abs(jerk) <= Constants::ZERO) {
    return CalConstAccSpeedPoint(start, start.a, t);
  }
  //
  SpeedPoint result_point;
  const double discriminant = start.a * start.a - 2 * jerk * start.v;

  auto cal_const_jerk = [](const SpeedPoint &start, const double jerk,
                           const double t) {
    SpeedPoint result_point;
    const double delta_t = t - start.t;
    result_point.t = t;
    result_point.j = jerk;
    result_point.a = start.a + jerk * delta_t;
    result_point.v =
        start.v + start.a * delta_t + 0.5 * jerk * delta_t * delta_t;
    result_point.s = start.s + start.v * delta_t +
                     0.5 * start.a * delta_t * delta_t +
                     jerk * delta_t * delta_t * delta_t / 6.0;
    return result_point;
  };

  if (jerk > 0 && (start.a >= 0 || (start.a < 0 && discriminant <= 0))) {
    result_point = cal_const_jerk(start, jerk, t);
  } else {
    const double stop_delta_t = -(start.a + std::sqrt(discriminant)) / jerk;
    if (t < start.t + stop_delta_t - Constants::ZERO) {
      result_point = cal_const_jerk(start, jerk, t);
    } else {
      result_point = cal_const_jerk(start, jerk, start.t + stop_delta_t);
      result_point.t = t;
      result_point.j = 0.0;
      result_point.v = 0.0;
    }
  }
  return result_point;
}

}  // namespace speed_profile_utils

}  // namespace planning
}  // namespace ad_byd
