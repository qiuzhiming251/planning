#ifndef ONBOARD_PLANNER_UTIL_SPEED_UTIL_H_
#define ONBOARD_PLANNER_UTIL_SPEED_UTIL_H_

#include "plan_common/math/piecewise_linear_function.h"

namespace st {
namespace planning {
/// @brief transform km/h to m/s aligin to instrument panel
/// @param set_speed_bias_plf speed_bias_plf, used to get speed offset
/// @param set_speed_kph instrument panel speed in km/h
/// @return speed followed by planner
double SpeedTransformation(
    const PiecewiseLinearFunction<double>& set_speed_bias_plf,
    const PiecewiseLinearFunction<double>& set_speed_gain_plf,
    double set_speed_kph);

double SpeedTransformationUsePI(
    const PiecewiseLinearFunction<double>& set_speed_bias_plf,
    const PiecewiseLinearFunction<double>& set_speed_gain_plf,
    double set_speed_kph, double act_speed_mps, double display_speed,
    double pid_pgain_factor, double pid_igain_factor,
    bool enable_use_pid_set_speed, double* integrator);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_UTIL_SPEED_UTIL_H_