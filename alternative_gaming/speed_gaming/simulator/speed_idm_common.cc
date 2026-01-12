#include "speed_idm_common.h"

namespace st::planning {
namespace SpeedIdmCommon {
double CalcYieldIdmAcc(const SpeedIdmState &cur_state,
                       const SpeedIdmUpdateParams &idm_params) {
  // 计算加速度 a_out
  constexpr double kMinDominator = 0.5;
  double alpha = cur_state.ego_v < cur_state.agent_v ? 0.0 : idm_params.alpha;
  // 计算理想距离
  double d_I = idm_params.d0 +
               (idm_params.k0 + alpha * (cur_state.ego_v - cur_state.agent_v)) *
                   cur_state.ego_v;
  // 计算实际距离 s_alpha
  double s_alpha = std::max(0.0, cur_state.agent_s - cur_state.ego_s);

  double a_out = (cur_state.agent_v - cur_state.ego_v +
                  alpha * cur_state.ego_v * cur_state.agent_a +
                  idm_params.beta * (s_alpha - d_I)) /
                 std::max((idm_params.k0 +
                           alpha * (2 * cur_state.ego_v - cur_state.agent_v)),
                          kMinDominator);
  return a_out;
}

double CalcPassIdmAcc(const SpeedIdmState &cur_state,
                      SpeedIdmUpdateParams &idm_params, double ego_cross_angle,
                      double interactive_time, double interactive_dist) {
  const double cos_ego_cutin_theta = std::cos(ego_cross_angle);
  const double ego_v_cos_theta = cur_state.ego_v * cos_ego_cutin_theta;
  double alpha = cur_state.agent_v < ego_v_cos_theta ? 0.0 : idm_params.alpha;
  double K = interactive_time -
             (idm_params.k0 + alpha * (cur_state.agent_v - ego_v_cos_theta));
  idm_params.alpha = alpha;

  double ego_ideal_cutin_dist = K * cur_state.ego_v;
  double current_risk = interactive_dist - ego_ideal_cutin_dist;
  double risk_dot = -idm_params.beta * current_risk;
  double R_v = -alpha * ego_v_cos_theta - K;
  if (current_risk < 0.0) {
    return 1.0;
  }
  return std::max(risk_dot / (std::min(-1.0, R_v)), 1.0);
}

double CalcSpeedLimitAcc(
    double cur_s, double cur_vel, double virtual_time,
    const PiecewiseLinearFunction<double> &speed_limit_plf) {
  double virtual_s = cur_s + cur_vel * virtual_time;
  double speed_limit = speed_limit_plf(virtual_s);
  double target_acc = 1.5 * (speed_limit - cur_vel) / virtual_time;
  return target_acc;
}

double CalMaxFrictionCircleBoundTargetAcc(double max_friction_acc, double cur_v,
                                          double cur_kappa) {
  double center_acc = cur_v * cur_v * std::max(std::fabs(cur_kappa), 1.0e-4);
  double max_target_acc = std::sqrt(std::max(
      (max_friction_acc * max_friction_acc - center_acc * center_acc), 1.0e-4));
  return max_target_acc;
}
double CalcSpeedProfileYieldIdmAcc(const SpeedIdmState &cur_state,
                                   const SpeedIdmUpdateParams &idm_params) {
  // 计算加速度 a_out
  constexpr double kMinDominator = 0.5;
  // 计算理想距离
  double d_I = idm_params.d0 +
               (idm_params.k0 +
                idm_params.alpha * (cur_state.ego_v - cur_state.agent_v)) *
                   cur_state.ego_v;
  // 计算实际距离 s_alpha
  double s_alpha = std::max(0.0, cur_state.agent_s - cur_state.ego_s);

  double a_out =
      (cur_state.agent_v - cur_state.ego_v +
       idm_params.alpha * cur_state.ego_v * cur_state.agent_a +
       idm_params.beta * (s_alpha - d_I)) /
      std::max((idm_params.k0 +
                idm_params.alpha * (2 * cur_state.ego_v - cur_state.agent_v)),
               kMinDominator);
  return a_out;
}

}  // namespace SpeedIdmCommon
}  // namespace st::planning