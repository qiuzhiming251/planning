#include "speed_idm_base.h"
namespace st::planning {
namespace {
double kEps = 1.0e-6;
}

double SpeedGamingIdmBase::CalcUpperSpeedBoundAcc(
    double t, const SpeedIdmState &cur_state,
    const SpeedVector &upper_speed_bound) const {
  if (upper_speed_bound.empty()) {
    return 2.0;
  }
  const auto upper_speed_point = upper_speed_bound.EvaluateByTime(t);
  if (!upper_speed_point.has_value()) {
    return 2.0;
  }
  SpeedIdmUpdateParams idm_param;
  idm_param.alpha = 0.4;
  idm_param.k0 = 0.0;
  idm_param.d0 = 0.0;
  double ttc = (upper_speed_point->s() - cur_state.ego_s) /
               std::max((cur_state.ego_v - upper_speed_point->v()),
                        kEps);  // 这里应该是写错了
  ttc = std::max(1.0, ttc);
  idm_param.beta = std::max(0.5, 1.0 / ttc);
  SpeedIdmState virtual_state = cur_state;
  virtual_state.agent_s = upper_speed_point->s();
  virtual_state.agent_v = upper_speed_point->v();
  virtual_state.agent_a = upper_speed_point->a();

  return SpeedIdmCommon::CalcYieldIdmAcc(virtual_state, idm_param);
}

double SpeedGamingIdmBase::CalcNormalSpeedTrackAcc(
    double t, const SpeedIdmState &cur_state,
    const SpeedVector &upper_speed_bound) const {
  return CalcUpperSpeedBoundAcc(t, cur_state, upper_speed_bound);
}

double SpeedGamingIdmBase::CalcInertialTrackJerk(
    const SpeedVector &origin_speed_data, const SpeedPoint &speed_point,
    int index) const {
  const double &t = speed_point.t();
  const double &s = speed_point.s();
  const double &v = speed_point.v();
  const double &a = speed_point.a();
  const double &j = speed_point.j();

  double lookforwardtimegap = 10.0;
  double lookforwardtimegap2 = lookforwardtimegap * lookforwardtimegap;
  double lookforwardtimegap3 =
      lookforwardtimegap * lookforwardtimegap * lookforwardtimegap;
  double target_s = 0.0;
  double target_v = 0.0;
  double target_a = 0.0;
  double target_j = 0.0;
  auto target_speed_point = origin_speed_data[index - 1];
  // target_a = target_speed_point.a();
  target_j = target_speed_point.j();
  target_s = target_speed_point.s();
  target_v = target_speed_point.v();
  target_a = target_speed_point.a();
  double jerk = target_j + 6 * (target_s - s) / lookforwardtimegap3 +
                6 * (target_v - v) / lookforwardtimegap2 +
                3 * (target_a - a) / lookforwardtimegap;

  return jerk;
}

double SpeedGamingIdmBase::CalcInertialTrackAcc(
    const SpeedVector &origin_speed_data, const SpeedPoint &speed_point) const {
  double lookforwardtimegap = 1.5;
  double lookforwardtimegap2 = lookforwardtimegap * lookforwardtimegap;
  double target_s = 0.0;
  double target_v = 0.0;
  double target_a = 0.0;

  auto target_speed_point = origin_speed_data.EvaluateByTimeWithExtrapolation(
      speed_point.t() + lookforwardtimegap);
  if (target_speed_point.has_value()) {
    target_s = target_speed_point->s();
    target_v = target_speed_point->v();
    target_a = target_speed_point->a();
  } else {
    // inertia_a 设为0
    return 0;
  }

  double acc = target_a +
               2 * (target_s - speed_point.s()) / lookforwardtimegap2 +
               2 * (target_v - speed_point.v()) / lookforwardtimegap;
  return acc;
}

double SpeedGamingIdmBase::CalcJerkByAcc(double cur_acc, double target_acc,
                                         double pJerk) const {
  if (pJerk < 0.0) {
    return 0.0;
  }
  const auto &config = GetConfig();
  double time_step1 = GetStepTime();
  double jerk = 0.0;
  constexpr double kFastAccChangeThresh = 0.1;  // TODD(haojie): check this
                                                // value
  constexpr double kFastAccChangeGapThresh = -1.5;
  if (config.fast_acc_to_dec && cur_acc > kFastAccChangeThresh &&
      target_acc <= -kFastAccChangeThresh) {  // fast_acc_to_dec
    jerk = -cur_acc * time_step1 / (time_step1 * time_step1 + kEps);
  } else if (config.fast_dec_to_acc && cur_acc < -kFastAccChangeThresh &&
             target_acc > kFastAccChangeThresh) {  // fast_dec_to_acc
    jerk = -cur_acc * time_step1 / (time_step1 * time_step1 + kEps);
  } else if (config.fast_dec_to_acc && cur_acc < -kFastAccChangeThresh &&
             ((cur_acc - target_acc) <
              kFastAccChangeGapThresh)) {  // quick brake releasing
    jerk = (std::min(0.0, target_acc) - cur_acc) * time_step1 /
           (time_step1 * time_step1 + kEps);
  } else {
    jerk = pJerk * (target_acc - cur_acc);
  }
  return jerk;
}

SpeedIdmState SpeedGamingIdmBase::UpdateNextEgoState(
    const SpeedIdmState &cur_state, double jerk) const {
  // 根据当前的位置、速度、加速度和加速度更新新的状态
  SpeedIdmState next_state = cur_state;
  double time_step1 = GetStepTime();
  double time_step2 = GetStepTime2();

  double time_step3 = GetStepTime3();

  double next_s = cur_state.ego_s + cur_state.ego_v * time_step1 +
                  0.5 * cur_state.ego_a * time_step2 +
                  0.16666666 * jerk * time_step3;

  double next_v =
      cur_state.ego_v + cur_state.ego_a * time_step1 + 0.5 * jerk * time_step2;

  double next_a = cur_state.ego_a + jerk * time_step1;

  double next_j = jerk;

  // 保证不倒车
  if (next_s < cur_state.ego_s) {
    next_s = cur_state.ego_s;
    next_v = 0.0;
    next_a = 0.0;
    next_j = 0.0;
  }

  next_state.ego_s = next_s;
  next_state.ego_v = next_v;
  next_state.ego_a = next_a;
  next_state.ego_j = next_j;
  return next_state;
}
}  // namespace st::planning