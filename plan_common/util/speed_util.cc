#include "plan_common/util/speed_util.h"

namespace st {
namespace planning {
double SpeedTransformation(
    const PiecewiseLinearFunction<double>& set_speed_bias_plf,
    const PiecewiseLinearFunction<double>& set_speed_gain_plf,
    double set_speed_kph) {
  const double speed_offset = set_speed_bias_plf(set_speed_kph);
  const double speed_gain = set_speed_gain_plf(set_speed_kph);
  double act_speed_mps = set_speed_kph / speed_gain;
  act_speed_mps = Kph2Mps(act_speed_mps) + speed_offset;

  return std::max(0.0, act_speed_mps);
}

double SpeedTransformationUsePI(
    const PiecewiseLinearFunction<double>& set_speed_bias_plf,
    const PiecewiseLinearFunction<double>& set_speed_gain_plf,
    double set_speed_kph, double act_can_speed_mps, double display_speed,
    double p_gain, double i_gain, bool use_pi_set_speed_flag,
    double* integrator) {
  constexpr double kPiApplicationRange = 1.5;    // m/s
  constexpr double kMaxGain = 0.3;               // m/s
  constexpr double kIntergatorThreshold = 0.05;  // m/s
  constexpr double kSpeedBuffer = 0.05;          // m/s
  constexpr double kSetSpeedThreshold = 0.02;    // km/h 
  const double speed_offset = set_speed_bias_plf(set_speed_kph);
  const double speed_gain = set_speed_gain_plf(set_speed_kph);
  double speed_error = 0.0;
  double pi_portion = 0.0;
  double pi_integrator = *integrator;
  //add condition keep target speed to reach display speed 
  double act_speed_mps = set_speed_kph / speed_gain;
  act_speed_mps = Kph2Mps(act_speed_mps) + speed_offset;
  speed_error = act_speed_mps - Kph2Mps(act_can_speed_mps);
  if (use_pi_set_speed_flag && abs(speed_error) < kPiApplicationRange) {
    if (std::abs(speed_error) < kIntergatorThreshold &&
        std::abs(display_speed - set_speed_kph) < kSetSpeedThreshold) {
      pi_integrator = 0.0;
      return std::max(0.0, act_speed_mps - kSpeedBuffer);
    }
    pi_portion = p_gain * speed_error;
    pi_integrator += i_gain * speed_error;
    pi_integrator = std::clamp(pi_portion + pi_integrator, -kMaxGain, kMaxGain);
    act_speed_mps += pi_integrator + pi_portion;
  }
  *integrator = pi_integrator;
  return std::max(0.0, act_speed_mps);
}

}  // namespace planning
}  // namespace st