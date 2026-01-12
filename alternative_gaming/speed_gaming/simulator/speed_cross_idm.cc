#include "speed_cross_idm.h"
namespace st::planning {

SpeedVector SpeedCrossIdm::CalcYieldIdmSpeedVector(
    const SpeedIdmState &init_state, const SpeedVector &origin_speed_data,
    const IdmInteractiveInfo &interactive_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    IdmSimulateState *simulate_state) const {
  SpeedVector yield_speed_profile;
  PiecewiseLinearFunction<double> k0_plf(
      {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f},
      {0.3f, 0.3f, 0.4f, 0.6f, 0.8f, 1.0f, 1.2f});
  const PiecewiseLinearFunction<double> alpha_plf(
      {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f},
      {0.2f, 0.2f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f});
  const PiecewiseLinearFunction<double> d0_plf({3.0, 5.0, 7.0},
                                               {0.5, 1.5, 2.0});

  const auto &config = GetConfig();
  const auto time_step = GetStepTime();
  yield_speed_profile.reserve(
      static_cast<int>(speed_gaming_params_.planning_horizon / time_step));
  SpeedIdmState cur_state = init_state;

  double max_v = speed_gaming_params_.max_vel;
  double min_v = speed_gaming_params_.min_vel;

  double ego_cutin_dist = interactive_info.ego_to_conflict_zone_dist;

  SpeedIdmUpdateParams idm_param;
  idm_param.d0 = d0_plf(ego_cutin_dist);

  double agent_cutout_time = interactive_info.agent_leave_conflict_zone_time;
  int index = 1;
  int end_index =
      std::floor(speed_gaming_params_.planning_horizon / time_step) + 1;
  int cutout_index = std::floor(agent_cutout_time / time_step) + 1;
  yield_speed_profile.emplace_back(0.0, cur_state.ego_s, cur_state.ego_v,
                                   cur_state.ego_a, cur_state.ego_j);

  for (; index < cutout_index && index < end_index; ++index) {
    double simu_time = index * time_step;
    double cutout_time = agent_cutout_time - simu_time;
    idm_param.alpha = alpha_plf(cutout_time);
    idm_param.k0 = k0_plf(cutout_time);
    double ttc = (ego_cutin_dist - cur_state.ego_s - idm_param.d0) /
                 std::max(cur_state.ego_v - cur_state.agent_v, 0.2);
    double ttc_idm = std::max(0.3, ttc);
    idm_param.beta = std::max(0.5, 1.0 / ttc_idm);
    auto calcJerkforStep = [&origin_speed_data, &interactive_info, &cur_state,
                            &idm_param, &speed_limit_plf, &max_v, config,
                            yield_speed_profile, index, simulate_state,
                            this]() -> double {
      double acc_out = SpeedIdmCommon::CalcYieldIdmAcc(cur_state, idm_param);
      if (!speed_limit_plf.x().empty()) {
        // calc velocity limit
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
        auto cur_line_plant =
            interactive_info.av_path.Evaluate(cur_state.ego_s);
        double max_target_acc =
            SpeedIdmCommon::CalMaxFrictionCircleBoundTargetAcc(
                speed_gaming_params_.max_friction_circle_acc, cur_state.ego_v,
                cur_line_plant.kappa());
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
    yield_speed_profile.emplace_back(simu_time, cur_state.ego_s,
                                     cur_state.ego_v, cur_state.ego_a,
                                     cur_state.ego_j);
  }
  // 障碍车切出之后，主车直接加速
  for (; index <= end_index; ++index) {
    double target_acc = 1.0;
    double simu_time = index * time_step;
    auto calcJerkforStep = [&origin_speed_data, &interactive_info, &cur_state,
                            &idm_param, &speed_limit_plf, &max_v, &target_acc,
                            config, this]() -> double {
      if (!speed_limit_plf.x().empty()) {
        // calc velocity limit
        max_v = speed_limit_plf(cur_state.ego_s);
        max_v = std::min(max_v, speed_gaming_params_.max_vel);
        double virtual_s =
            cur_state.ego_s +
            cur_state.ego_v * config.speed_limit_look_forward_time;
        max_v = std::min(max_v, speed_limit_plf(virtual_s));
        double speed_limit_acc = 1.5 * (max_v - cur_state.ego_v) /
                                 config.speed_limit_look_forward_time;
        target_acc = std::min(target_acc, speed_limit_acc);
      }

      if (!interactive_info.av_path.empty()) {
        auto cur_line_plant =
            interactive_info.av_path.Evaluate(cur_state.ego_s);
        double max_target_acc =
            SpeedIdmCommon::CalMaxFrictionCircleBoundTargetAcc(
                speed_gaming_params_.max_friction_circle_acc, cur_state.ego_v,
                cur_line_plant.kappa());
        target_acc = std::min(target_acc, max_target_acc);
      }

      double jerk =
          CalcJerkByAcc(cur_state.ego_a, target_acc, 2.0 * idm_param.beta);
      jerk = std::clamp(jerk, speed_gaming_params_.min_jerk,
                        speed_gaming_params_.max_jerk);
      return jerk;
    };

    cur_state = UpdateNextEgoState(cur_state, calcJerkforStep());
    yield_speed_profile.emplace_back(simu_time, cur_state.ego_s,
                                     cur_state.ego_v, cur_state.ego_a,
                                     cur_state.ego_j);
  }

  return yield_speed_profile;
}
SpeedVector SpeedCrossIdm::CalcPassIdmSpeedVector(
    const SpeedIdmState &init_state, const SpeedVector &origin_speed_data,
    const SpeedVector &upper_speed_bound, const SpeedVector &normal_speed_data,
    const IdmInteractiveInfo &interactive_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    IdmSimulateState *simulate_state) const {
  SpeedVector pass_speed_profile;
  const double time_step_1 = GetStepTime();
  const auto &config = GetConfig();
  pass_speed_profile.reserve(
      static_cast<int>(speed_gaming_params_.planning_horizon / time_step_1));
  double max_v = speed_gaming_params_.max_vel;
  double min_v = speed_gaming_params_.min_vel;
  SpeedIdmState cur_state = init_state;
  SpeedIdmUpdateParams idm_param;
  idm_param.alpha = 0.3;
  idm_param.k0 = 0.8;
  idm_param.d0 = 2.0;
  idm_param.beta = 0.5;

  // 主车组车在往复车切入点之前超车
  const double agent_cutin_time = interactive_info.agent_to_conflict_zone_time;
  auto agent_cutin_speed_point =
      interactive_info.agent_speed_profile->EvaluateByS(
          interactive_info.agent_to_conflict_zone_dist - idm_param.d0, true);
  const double agent_cutin_time_consider_d0 = agent_cutin_speed_point->t();
  const double ego_cutout_dist = interactive_info.ego_leave_conflict_zone_dist;
  const double agent_cutin_dist = interactive_info.agent_to_conflict_zone_dist;
  double simu_time = time_step_1;
  int index = 1;
  int end_index =
      std::floor(speed_gaming_params_.planning_horizon / time_step_1) + 1;
  int agent_cutin_index =
      std::floor(agent_cutin_time_consider_d0 / time_step_1) + 1;
  pass_speed_profile.emplace_back(0.0, cur_state.ego_s, cur_state.ego_v,
                                  cur_state.ego_a, cur_state.ego_j);

  // debug += "agent_cutin_time_consider_d0: " +
  // absl::StrCat(agent_cutin_time_consider_d0) + "\n";
  for (; index <= agent_cutin_index && index <= end_index; ++index) {
    double simu_time = index * time_step_1;
    auto calcJerkforStep =
        [&origin_speed_data, &interactive_info, &cur_state, &idm_param,
         &speed_limit_plf, &max_v, &min_v, simu_time, ego_cutout_dist,
         ego_cutout_dist, agent_cutin_time_consider_d0, &pass_speed_profile,
         agent_cutin_dist, &upper_speed_bound, &normal_speed_data, config,
         index, simulate_state, this]() -> double {
      // if (!speed_limit_plf.x().empty())
      // { // speed_limit_ptf is empty, no speed_constraint
      //     max_v = speed_limit_plf(cur_state.ego_s);
      //     max_v = std::min(max_v, speed_gaming_params_.max_vel);
      // }

      double agent_cur_cutin_time = agent_cutin_time_consider_d0 - simu_time;
      double hwt_gap =
          agent_cur_cutin_time -
          (ego_cutout_dist - cur_state.ego_s) / std::max(cur_state.ego_v, 0.2);
      double ttc_idm = std::max(1.0, hwt_gap);
      idm_param.beta = std::max(0.6, 1.0 / ttc_idm);

      double ego_cur_cutout_dist = ego_cutout_dist - cur_state.ego_s;
      // 自车切入冲突区的一个虚拟时间，不考虑加速信息
      // TODO(tianrui): 以Cross的simulator输入，这里的virtual
      // simulation可以在上线稳定后去除
      double virtual_time_to_conflict_point_1 =
          ego_cur_cutout_dist / (cur_state.ego_v + 1e-6);

      double critical_s_2 = 0.0;
      double critical_v_2 = 0.0;
      auto agent2_speed_point =
          interactive_info.agent_speed_profile->EvaluateByTime(
              virtual_time_to_conflict_point_1 + simu_time);
      if (agent2_speed_point.has_value()) {
        critical_v_2 = agent2_speed_point->v();
        critical_s_2 = agent2_speed_point->s();
      }

      double virtual_time_to_conflict_point_2 =
          virtual_time_to_conflict_point_1 +
          (agent_cutin_dist - critical_s_2) / std::max(critical_v_2, 1.0);
      cur_state.agent_v = critical_v_2;

      double acc_out = SpeedIdmCommon::CalcPassIdmAcc(
          cur_state, idm_param, interactive_info.ego_cross_angle,
          virtual_time_to_conflict_point_2, ego_cur_cutout_dist);
      if (!speed_limit_plf.x().empty()) {
        // calc velocity limit
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

      auto cur_line_point = interactive_info.av_path.Evaluate(cur_state.ego_s);
      if (!interactive_info.av_path.empty()) {
        double max_target_acc =
            SpeedIdmCommon::CalMaxFrictionCircleBoundTargetAcc(
                speed_gaming_params_.max_friction_circle_acc, cur_state.ego_v,
                cur_line_point.kappa());
        acc_out = std::min(acc_out, max_target_acc);
      }

      acc_out = std::clamp(acc_out, speed_gaming_params_.min_acc,
                           speed_gaming_params_.max_acc);
      //
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
      double jerk =
          CalcJerkByAcc(cur_state.ego_a, acc_out, 2.0 * idm_param.beta);
      jerk = std::clamp(jerk, speed_gaming_params_.min_jerk,
                        speed_gaming_params_.max_jerk);
      return jerk;
    };

    cur_state = UpdateNextEgoState(cur_state, calcJerkforStep());
    auto agent_speed_point =
        interactive_info.agent_speed_profile->EvaluateByTime(simu_time);
    cur_state.agent_s = agent_speed_point->s();
    cur_state.agent_v = agent_speed_point->v();
    cur_state.agent_a = agent_speed_point->a();
    // 更新cur_state with next_state
    pass_speed_profile.emplace_back(simu_time, cur_state.ego_s, cur_state.ego_v,
                                    cur_state.ego_a, cur_state.ego_j);
  }
  // 主车在切入之后仍保持加速(减少没超过去但是没碰撞的paradox)
  for (; index < end_index; ++index) {
    double simu_time = index * time_step_1;
    double target_acc = 1.0;
    if (!speed_limit_plf.x()
             .empty()) {  // speed_limit_plf is empty, no speed_constraint
      max_v = speed_limit_plf(cur_state.ego_s);
      max_v = std::min(max_v, speed_gaming_params_.max_vel);
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

}  // func

}  // namespace st::planning