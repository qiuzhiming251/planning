#include "speed_merge_idm.h"
namespace st::planning {
SpeedVector SpeedMergeIdm::CalcYieldIdmSpeedVector(
    const SpeedIdmState &init_state, const SpeedVector &origin_speed_data,
    const IdmInteractiveInfo &interactive_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    IdmSimulateState *simulate_state) const {
  const PiecewiseLinearFunction<double> k0_pif(
      {0.0, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f},
      {0.3f, 0.3f, 0.4f, 0.6f, 0.8f, 1.0f, 1.2f});
  const PiecewiseLinearFunction<double> alpha_pif(
      {0.0, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f},
      {0.2f, 0.2f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f});
  SpeedVector yield_speed_profile;
  const auto time_step = GetStepTime();
  const auto &config = GetConfig();
  yield_speed_profile.reserve(
      static_cast<int>(speed_gaming_params_.planning_horizon / time_step));
  SpeedIdmState cur_state = init_state;

  double max_v = speed_gaming_params_.max_vel;
  double min_v = speed_gaming_params_.min_vel;
  SpeedIdmUpdateParams idm_param;
  double agent_cutin_time = interactive_info.agent_to_conflict_zone_time;
  int start_index = 1;
  int end_index =
      std::floor(speed_gaming_params_.planning_horizon / time_step) + 1;
  yield_speed_profile.emplace_back(0.0, cur_state.ego_s, cur_state.ego_v,
                                   cur_state.ego_a, cur_state.ego_j);
  for (int index = start_index; index < end_index; ++index) {
    double simu_time = index * time_step;
    double cut_time = speed_gaming_params_.planning_horizon - simu_time;
    idm_param.alpha = alpha_pif(cut_time);
    idm_param.k0 = k0_pif(cut_time);
    idm_param.d0 = 2.0;
    double ttc_idm =
        std::max(0.3, (cur_state.agent_s - cur_state.ego_s - idm_param.d0) /
                          std::max((cur_state.ego_v - cur_state.agent_v), 0.1));
    idm_param.beta = std::max(0.5, 1.0 / ttc_idm);
    auto calcJerkforStep = [&origin_speed_data, &interactive_info, &cur_state,
                            &idm_param, &speed_limit_plf, &max_v, config,
                            yield_speed_profile, index, simu_time,
                            simulate_state, this]() -> double {
      double acc_out = SpeedIdmCommon::CalcYieldIdmAcc(cur_state, idm_param);
      if (!speed_limit_plf.x().empty()) {
        max_v = speed_limit_plf(cur_state.ego_s);
        max_v = std::min(max_v, speed_gaming_params_.max_vel);
        double virtual_s =
            cur_state.ego_s +
            cur_state.ego_v * config.speed_limit_look_forward_time;
        max_v = std::min(max_v, speed_limit_plf(virtual_s));
        double speed_limit_acc = 1.5 * (max_v - cur_state.ego_v) /
                                 config.speed_limit_look_forward_time;
        acc_out = std::min(acc_out, speed_limit_acc);
      }
      if (!interactive_info.av_path.empty()) {
        auto cur_line_point =
            interactive_info.av_path.Evaluate(cur_state.ego_s);
        double max_target_acc =
            SpeedIdmCommon::CalMaxFrictionCircleBoundTargetAcc(
                speed_gaming_params_.max_friction_circle_acc, cur_state.ego_v,
                cur_line_point.kappa());
        acc_out = std::min(acc_out, max_target_acc);
      }
      acc_out = std::clamp(acc_out, speed_gaming_params_.min_acc,
                           speed_gaming_params_.max_acc);
      if (simulate_state != nullptr && simulate_state->is_agent_state &&
          index == 1) {
        simulate_state->agent_ideal_acc_in_pass_mode = acc_out;
      }
      double inertial_jerk = 0.0;
      if (!origin_speed_data.empty()) {
        SpeedPoint ego_speed_point = yield_speed_profile.back();

        // 计算惯性acc

        inertial_jerk =
            CalcInertialTrackJerk(origin_speed_data, ego_speed_point, index);

        inertial_jerk = std::clamp(inertial_jerk, speed_gaming_params_.min_jerk,
                                   speed_gaming_params_.max_jerk);
      }
      double jerk =
          CalcJerkByAcc(cur_state.ego_a, acc_out, 2.0 * idm_param.beta);

      jerk = std::clamp(jerk, speed_gaming_params_.min_jerk,
                        speed_gaming_params_.max_jerk);
      jerk = config.update_coeff * jerk +
             (1.0 - config.update_coeff) * inertial_jerk;
      jerk = std::clamp(jerk, speed_gaming_params_.min_jerk,
                        speed_gaming_params_.max_jerk);
      return jerk;
    };

    cur_state = UpdateNextEgoState(cur_state, calcJerkforStep());
    // 更新前车状态，切入之后更新前车速度
    if (simu_time > agent_cutin_time) {
      auto agent_speed_point =
          interactive_info.agent_speed_profile->EvaluateByTime(simu_time);
      if (agent_speed_point.has_value()) {
        cur_state.agent_v = agent_speed_point->v();
        cur_state.agent_a = agent_speed_point->a();
      }
    }
    cur_state.agent_s = cur_state.agent_s + time_step * cur_state.agent_v;
    // 更新cur_state with next_state
    yield_speed_profile.emplace_back(simu_time, cur_state.ego_s,
                                     cur_state.ego_v, cur_state.ego_a,
                                     cur_state.ego_j);
  }
  return yield_speed_profile;
}

SpeedVector SpeedMergeIdm::CalcPassIdmSpeedVector(
    const SpeedIdmState &init_state, const SpeedVector &origin_speed_data,
    const SpeedVector &upper_speed_bound, const SpeedVector &normal_speed_data,
    const IdmInteractiveInfo &interactive_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    IdmSimulateState *simulate_state) const {
  SpeedVector pass_speed_profile;
  const double time_step = GetStepTime();
  const auto &config = GetConfig();
  pass_speed_profile.reserve(
      static_cast<int>(speed_gaming_params_.planning_horizon / time_step));
  double max_v = speed_gaming_params_.max_vel;
  double min_v = speed_gaming_params_.min_vel;
  SpeedIdmState cur_state = init_state;
  SpeedIdmUpdateParams idm_param;
  idm_param.alpha = 0.3;
  idm_param.k0 = 0.8;
  idm_param.d0 = 2.0;
  idm_param.beta = 0.5;

  const double agent_cutin_time = interactive_info.agent_to_conflict_zone_time;
  const double ego_cutin_dist = interactive_info.ego_to_conflict_zone_dist;
  auto agent_cutin_speed_point =
      interactive_info.agent_speed_profile->EvaluateByS(
          interactive_info.agent_to_conflict_zone_dist - idm_param.d0, true);
  const double agent_cutin_time_consider_d0 = agent_cutin_speed_point->t();
  int index = 1;
  int end_index =
      std::floor(speed_gaming_params_.planning_horizon / time_step) + 1;
  int agent_cutin_index = std::floor(agent_cutin_time / time_step) + 1;
  pass_speed_profile.emplace_back(0.0, cur_state.ego_s, cur_state.ego_v,
                                  cur_state.ego_a, cur_state.ego_j);
  for (; index <= agent_cutin_index && index <= end_index; ++index) {
    double simu_time = index * time_step;
    double agent_cur_cutin_time = agent_cutin_time - simu_time;
    double ego_cutin_time =
        (ego_cutin_dist - cur_state.ego_s) / cur_state.ego_v;
    double hwt_gap = agent_cur_cutin_time - ego_cutin_time;
    double ttc_idm = std::max(1.0, hwt_gap);
    idm_param.beta = std::max(0.5, 1.0 / ttc_idm);
    double ego_cur_cutint_dist = ego_cutin_dist - cur_state.ego_s;
    auto calcJerkforStep =
        [&origin_speed_data, &interactive_info, &cur_state, &idm_param,
         &speed_limit_plf, &max_v, config, index, &upper_speed_bound,
         &normal_speed_data, ego_cur_cutint_dist, agent_cur_cutin_time,
         simulate_state, &pass_speed_profile, simu_time, this]() -> double {
      double acc_out = SpeedIdmCommon::CalcPassIdmAcc(
          cur_state, idm_param, interactive_info.ego_cross_angle,
          agent_cur_cutin_time, ego_cur_cutint_dist);
      acc_out = std::clamp(acc_out, speed_gaming_params_.min_acc,
                           speed_gaming_params_.max_acc);
      if (!speed_limit_plf.x().empty()) {
        max_v = speed_limit_plf(cur_state.ego_s);
        max_v = std::min(max_v, speed_gaming_params_.max_vel);
        double virtual_s =
            cur_state.ego_s +
            cur_state.ego_v * config.speed_limit_look_forward_time;
        max_v = std::min(max_v, speed_limit_plf(virtual_s));
        double speed_limit_acc = 1.5 * (max_v - cur_state.ego_v) /
                                 config.speed_limit_look_forward_time;
        acc_out = std::min(max_v, speed_limit_acc);
      }
      if (!interactive_info.av_path.empty()) {
        auto cur_line_point =
            interactive_info.av_path.Evaluate(cur_state.ego_s);
        double max_target_acc =
            SpeedIdmCommon::CalMaxFrictionCircleBoundTargetAcc(
                speed_gaming_params_.max_friction_circle_acc, cur_state.ego_v,
                cur_line_point.kappa());
        acc_out = std::min(acc_out, max_target_acc);
      }
      acc_out = std::clamp(acc_out, speed_gaming_params_.min_acc,
                           speed_gaming_params_.max_acc);

      if (!origin_speed_data.empty()) {
        SpeedPoint ego_speed_point = pass_speed_profile.back();
        // 计算惯性acc
        double inertial_acc =
            CalcInertialTrackAcc(origin_speed_data, ego_speed_point);
        auto target_a =
            origin_speed_data.EvaluateByTime(ego_speed_point.t())->a();
        inertial_acc = std::clamp(inertial_acc, speed_gaming_params_.min_acc,
                                  speed_gaming_params_.max_acc);
        // debug_t = "t " + absl::StrCat(simu_time) + " pass idm, acc_out: " +
        // absl::StrCat(acc_out) + " inertial_acc: " +
        // absl::StrCat(inertial_acc) + "\n"; 权重组合
        acc_out = config.update_coeff * acc_out +
                  (1.0 - config.update_coeff) * inertial_acc;
      }
      if (!normal_speed_data.empty()) {
        double normal_acc =
            CalcNormalSpeedTrackAcc(simu_time, cur_state, normal_speed_data);
        acc_out = std::max(acc_out, normal_acc);
      }

      if (!upper_speed_bound.empty()) {
        double upper_acc =
            CalcUpperSpeedBoundAcc(simu_time, cur_state, upper_speed_bound);
        acc_out = std::min(acc_out, upper_acc);
      }

      acc_out = std::clamp(acc_out, speed_gaming_params_.min_acc,
                           speed_gaming_params_.max_acc);
      if (simulate_state != nullptr && simulate_state->is_agent_state &&
          index == 1) {
        simulate_state->agent_ideal_acc_in_yield_mode = acc_out;
      }
      // 状态迭代更新
      double pJerk = 2.0 * idm_param.beta;
      double jerk = CalcJerkByAcc(cur_state.ego_a, acc_out, pJerk);
      jerk = std::clamp(jerk, speed_gaming_params_.min_jerk,
                        speed_gaming_params_.max_jerk);
      return jerk;
    };

    cur_state = UpdateNextEgoState(cur_state, calcJerkforStep());
    // 更新agent状态
    auto agent_speed_point =
        interactive_info.agent_speed_profile->EvaluateByTime(simu_time);
    cur_state.agent_s = agent_speed_point->s();
    cur_state.agent_v = agent_speed_point->v();
    cur_state.agent_a = agent_speed_point->a();

    // 更新cur_state with next_state
    pass_speed_profile.emplace_back(simu_time, cur_state.ego_s, cur_state.ego_v,
                                    cur_state.ego_a, cur_state.ego_j);
  }
  // 主车在切入之后匀加速
  for (; index < end_index; ++index) {
    double simu_time = index * time_step;
    double target_acc = 1.0;
    auto cur_line_point = interactive_info.av_path.Evaluate(cur_state.ego_s);
    if (!speed_limit_plf.x().empty()) {
      max_v = speed_limit_plf(cur_state.ego_s);
      max_v = std::min(max_v, speed_gaming_params_.max_vel);
      double virtual_s = cur_state.ego_s +
                         cur_state.ego_v * config.speed_limit_look_forward_time;
      max_v = std::min(max_v, speed_limit_plf(virtual_s));
      double speed_limit_acc = 1.5 * (max_v - cur_state.ego_v) /
                               config.speed_limit_look_forward_time;
      target_acc = std::min(target_acc, speed_limit_acc);
    }
    if (!interactive_info.av_path.empty()) {
      double max_target_acc =
          SpeedIdmCommon::CalMaxFrictionCircleBoundTargetAcc(
              speed_gaming_params_.max_friction_circle_acc, cur_state.ego_v,
              cur_line_point.kappa());
      target_acc = std::min(target_acc, max_target_acc);
    }
    if (!upper_speed_bound.empty()) {
      double upper_acc =
          CalcUpperSpeedBoundAcc(simu_time, cur_state, upper_speed_bound);
      target_acc = std::min(target_acc, upper_acc);
    }

    target_acc = std::clamp(target_acc, speed_gaming_params_.min_acc,
                            speed_gaming_params_.max_acc);

    double jerk =
        CalcJerkByAcc(cur_state.ego_a, target_acc, 2.0 * idm_param.beta);
    jerk = std::clamp(jerk, speed_gaming_params_.min_jerk,
                      speed_gaming_params_.max_jerk);

    cur_state = UpdateNextEgoState(cur_state, jerk);
    pass_speed_profile.emplace_back(simu_time, cur_state.ego_s, cur_state.ego_v,
                                    cur_state.ego_a, cur_state.ego_j);
  }
  return pass_speed_profile;
}
}  // namespace st::planning