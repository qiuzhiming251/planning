/**
 * @file multi_agent_idm.h
 * @author liu.tianrui (liu.tianrui1@byd.com)
 * @brief gaming result post process
 * @version 0.1
 * @date 2025-04-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "multi_agent_idm.h"

namespace st::planning {
constexpr double kEps = 1.0e-6;

SpeedVector MultiAgentIdm::CalcYieldTrajSpeedDataOfMultiAgent(
    const TrajectoryPoint &init_point, const DiscretizedPath &av_path,
    const SpeedVector &upper_speed_data, const IDMConfigPlf &idm_config_plf,
    const IDMConfigPlf &follow_idm_config_plf,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedIdmUpdateParams &idm_param,
    const std::unordered_map<std::string, GamingSimResult>
        &ego_yield_sim_results,
    const std::unordered_map<std::string, GamingSimResult>
        &ego_follow_sim_results) {
  SpeedVector yield_speed_vector{};
  double ego_s = 0.0;
  double ego_v = init_point.v();
  double ego_a = init_point.a();
  double ego_jerk = 0.0;
  double agent_s = 10000.0;
  double agent_v = 10.0;
  double agent_a = 0.0;
  double agent_j = 0.0;

  SpeedIdmState idm_state(ego_s, ego_v, ego_a, ego_jerk, agent_s, agent_v,
                          agent_a, agent_j);
  const PiecewiseLinearFunction<double> ego_init_accel_plf({0.0, 4.0, 8.0},
                                                           {2.0, 1.5, 1.0});
  int start_index = 1;
  const auto planning_horizon = speed_gaming_params_->planning_horizon;
  const auto time_step = speed_gaming_params_->time_step;
  const auto min_jerk = speed_gaming_params_->min_jerk;
  const auto max_jerk = speed_gaming_params_->max_jerk;
  auto max_v = speed_gaming_params_->max_vel;

  std::unordered_map<std::string, LastInfos> follow_last_infos;
  std::unordered_map<std::string, LastInfos> yield_last_infos;

  int end_index = std::floor(planning_horizon / time_step) + 1;
  yield_speed_vector.emplace_back(0.0, ego_s, ego_v, ego_a, ego_jerk);

  for (const auto &follow_sim_result : ego_follow_sim_results) {
    LastInfos last_infos;
    const GamingSimResult &follow_sim_res = follow_sim_result.second;
    if (follow_sim_res.interaction_type == InteractionType::kStaticOccupy) {
      continue;
    }
    const double agent_cutin_t =
        follow_sim_res.sim_conflict_zone_in_ego_view.agent_cutin_time;
    const auto agent_theta =
        SpeedGamingCommon::FindYawByT(follow_sim_res.obj_traj, agent_cutin_t);
    if (!agent_theta.has_value()) {
      continue;
    }
    const double agent_cutin_s =
        follow_sim_res.sim_conflict_zone_in_ego_view.ego_cutin_s;
    auto ref_pt = av_path.Evaluate(agent_cutin_s);
    const double ref_theta = ref_pt.theta();
    const double delta_theta = NormalizeAngle(ref_theta - agent_theta.value());
    const auto agent_speed_pt =
        follow_sim_res.obj_speed_data.EvaluateByTime(agent_cutin_t);
    if (!agent_speed_pt.has_value()) {
      continue;
    }

    last_infos.last_s =
        follow_sim_res.sim_conflict_zone_in_ego_view.ego_cutin_s;
    last_infos.last_v = agent_speed_pt.value().v() * std::cos(delta_theta);
    last_infos.last_a = agent_speed_pt.value().a() * std::cos(delta_theta);
    last_infos.last_delta_theta = delta_theta;
    follow_last_infos.emplace(follow_sim_result.first, last_infos);
  }

  for (const auto &yield_sim_result : ego_yield_sim_results) {
    if (yield_sim_result.second.interaction_type ==
            InteractionType::kStraightMerge ||
        yield_sim_result.second.interaction_type ==
            InteractionType::kTurnMerge) {
      LastInfos last_infos;
      double beta = idm_param.beta;
      const GamingSimResult &yield_sim_res = yield_sim_result.second;
      const double agent_cutin_t =
          yield_sim_res.sim_conflict_zone_in_ego_view.agent_cutin_time;
      const auto agent_theta =
          SpeedGamingCommon::FindYawByT(yield_sim_res.obj_traj, agent_cutin_t);
      if (!agent_theta.has_value()) {
        continue;
      }
      const double agent_cutin_s =
          yield_sim_res.sim_conflict_zone_in_ego_view.ego_cutin_s;
      auto ref_pt = av_path.Evaluate(agent_cutin_s);
      const double ref_theta = ref_pt.theta();
      const double delta_theta =
          NormalizeAngle(ref_theta - agent_theta.value());
      const auto agent_speed_pt =
          yield_sim_res.obj_speed_data.EvaluateByTime(agent_cutin_t);
      if (!agent_speed_pt.has_value()) {
        continue;
      }
      last_infos.last_s =
          yield_sim_res.sim_conflict_zone_in_ego_view.ego_cutin_s;
      last_infos.last_v = agent_speed_pt.value().v() * std::cos(delta_theta);
      last_infos.last_a = agent_speed_pt.value().a() * std::cos(delta_theta);
      last_infos.last_delta_theta = delta_theta;
      yield_last_infos.emplace(yield_sim_result.first, last_infos);
    }
  }

  for (int index = start_index; index < end_index; ++index) {
    double simu_time = index * time_step;
    double target_acc = ego_init_accel_plf(idm_state.ego_v);
    double beta = idm_param.beta;
    double target_jerk = SpeedGamingCommon::CalcSaturatedJerkByAcc(
        idm_state.ego_a, target_acc, 1.0 / time_step, time_step, min_jerk,
        max_jerk);
    for (const auto &yield_sim_result : ego_yield_sim_results) {
      if (yield_sim_result.second.interaction_type ==
              InteractionType::kStraightMerge ||
          yield_sim_result.second.interaction_type ==
              InteractionType::kTurnMerge) {
        auto last_info = yield_last_infos.find(yield_sim_result.first);
        if (last_info == yield_last_infos.end()) {
          continue;
        }
        double beta = idm_param.beta;
        double merge_yield_acc =
            CalcMergeYieldAcc(idm_state, yield_sim_result.second, index - 1,
                              idm_param, beta, av_path, last_info->second);
        target_acc = std::min(target_acc, merge_yield_acc);
        target_jerk = std::min(SpeedGamingCommon::CalcSaturatedJerkByAcc(
                                   idm_state.ego_a, target_acc, 1.0 / time_step,
                                   time_step, min_jerk, max_jerk),
                               target_jerk);
      } else if (yield_sim_result.second.interaction_type ==
                 InteractionType::kCross) {
        if (simu_time > yield_sim_result.second.sim_conflict_zone_in_ego_view
                            .agent_cutout_time) {
          continue;
        }
        // Process cross conflict zones
        double beta = idm_param.beta;
        double cross_yield_acc =
            CalcCrossYieldAcc(idm_state, yield_sim_result.second, simu_time,
                              idm_param, idm_config_plf, beta);
        target_acc = std::min(target_acc, cross_yield_acc);
        target_jerk = std::min(SpeedGamingCommon::CalcSaturatedJerkByAcc(
                                   idm_state.ego_a, target_acc, 1.0 / time_step,
                                   time_step, min_jerk, max_jerk),
                               target_jerk);
        // End of processing cross conflict zones
        // TODO():**how to address kFollow type**
      }
    }

    for (const auto &follow_sim_result : ego_follow_sim_results) {
      if (follow_sim_result.second.interaction_type ==
              InteractionType::kStraightMerge ||
          follow_sim_result.second.interaction_type ==
              InteractionType::kTurnMerge) {
        auto last_info = follow_last_infos.find(follow_sim_result.first);
        if (last_info == follow_last_infos.end()) {
          continue;
        }
        double beta = idm_param.beta;
        SpeedIdmUpdateParams idm_param_current = idm_param;
        double cut_out_remain_time =
            follow_sim_result.second.sim_conflict_zone_in_ego_view
                .agent_cutout_time -
            simu_time;
        idm_param_current.k0 =
            follow_idm_config_plf.k0_plf(cut_out_remain_time);
        idm_param_current.d0 =
            follow_idm_config_plf.d0_plf(cut_out_remain_time);
        // double K_0_risk = idm_config_plf.k0_plf(cut_out_remain_time);
        // double d_0_risk = idm_config_plf.d0_plf(cut_out_remain_time);
        // const PiecewiseLinearFunction<double>
        // follow_k_exp_plf({0.0, 1.0, 2.0, 3.0},
        //                                                        {0.0, 0.3,
        //                                                        0.5, 0.8}); //
        //                                                        TODO: need
        //                                                        tuning or to
        //                                                        be removed
        // double follow_K_expansion = follow_k_exp_plf(-1.0 *
        // last_info->second.last_a); const PiecewiseLinearFunction<double>
        // distance_quasi_plf({0.0, 10.0, 15.0, 20.0},
        //                                                          {1.0, 1.0, 1.0,
        //                                                          0.0});
        // const PiecewiseLinearFunction<double>
        // hwt_quasi_plf({0.0, 1.2, 1.5, 2.0, 3.0},
        //                                                     {1.0, 1.0, 1.0, 1.0,
        //                                                     0.0});
        // const PiecewiseLinearFunction<double> dv_v_quasi_plf(
        //     {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 1.0, 1.1},
        //     {0.0, 0.0, 0.1, 0.5, 0.6, 0.8, 1.0, 0.0});
        // double hwt = std::max(last_info->second.last_s - idm_state.ego_s,
        // 0.0)
        //              / std::max(idm_state.ego_v, 1.0);
        // double dv_v = std::max(idm_state.ego_v - last_info->second.last_v,
        // 0.0)
        //               / std::max(idm_state.ego_v, 1.0);
        // double distance_near = distance_quasi_plf(last_info->second.last_s -
        // idm_state.ego_s); double hwt_short = hwt_quasi_plf(hwt); double
        // front_obj_relatively_slow = dv_v_quasi_plf(dv_v); double
        // follow_obj_quasi_stop =
        //     (1.0 - (1.0 - distance_near) * (1.0 - hwt_short)) *
        //     front_obj_relatively_slow;
        // double alpha =
        //     0.5 * follow_obj_quasi_stop
        //     + (1.0 - follow_obj_quasi_stop) * idm_param_current.alpha; //
        //     TODO: 0.5 need tuning
        // const PiecewiseLinearFunction<double> extrem_distance_quasi_plf(
        //     {0.0, 10.0, 15.0, 25.0, 30.0}, {1.0, 1.0, 0.8, 0.5, 0.0});
        // double extrem_near =
        // extrem_distance_quasi_plf(last_info->second.last_s -
        // idm_state.ego_s); double follow_obj_quasi_stop_and_extreme_near =
        // extrem_near * follow_obj_quasi_stop; double K_0 = K_0_risk * (1.0 -
        // follow_obj_quasi_stop_and_extreme_near)
        //              + 2.0 * follow_obj_quasi_stop_and_extreme_near
        //              + follow_K_expansion; // TODO: 2.0 need tuning
        // idm_param_current.alpha = alpha;
        // idm_param_current.k0 = K_0;
        // idm_param_current.d0 = d_0_risk;
        double merge_yield_acc = CalcMergeYieldAcc(
            idm_state, follow_sim_result.second, index - 1, idm_param_current,
            beta, av_path, last_info->second);
        target_acc = std::min(target_acc, merge_yield_acc);
        target_jerk = std::min(SpeedGamingCommon::CalcSaturatedJerkByAcc(
                                   idm_state.ego_a, target_acc, 1.0 / time_step,
                                   time_step, min_jerk, max_jerk),
                               target_jerk);
      }
      if (follow_sim_result.second.interaction_type ==
          InteractionType::kStaticOccupy) {
        SpeedIdmState idm_cur_state = idm_state;
        idm_cur_state.agent_s =
            follow_sim_result.second.sim_conflict_zone_in_ego_view.ego_cutin_s;
        idm_cur_state.agent_v = 0.0;
        idm_cur_state.agent_a = 0.0;
        const double ttc_idm = std::max(
            0.3,
            (idm_cur_state.agent_s - idm_cur_state.ego_s - idm_param.d0) /
                std::max(idm_cur_state.ego_v - idm_cur_state.agent_v, 0.1));
        beta = std::max(idm_param.beta, 1.0 / ttc_idm);
        SpeedIdmUpdateParams idm_param_current = idm_param;
        idm_param_current.beta = beta;
        idm_param_current.k0 = 1.5;
        idm_param_current.d0 = 3.0;
        double follow_static_acc =
            SpeedIdmCommon::CalcYieldIdmAcc(idm_cur_state, idm_param_current);
        target_acc = std::min(target_acc, follow_static_acc);
        target_jerk = std::min(SpeedGamingCommon::CalcSaturatedJerkByAcc(
                                   idm_state.ego_a, target_acc, 1.0 / time_step,
                                   time_step, min_jerk, max_jerk),
                               target_jerk);
      }
    }
    if (!upper_speed_data.empty()) {
      target_acc = std::min(
          target_acc,
          CalcUpperSpeedBoundAcc(simu_time, idm_state, upper_speed_data));
      target_jerk = SpeedGamingCommon::CalcSaturatedJerkByAcc(
          idm_state.ego_a, target_acc, 1.0 / time_step, time_step, min_jerk,
          max_jerk);
    }
    if (!av_path.empty()) {
      auto cur_line_plant =
          av_path.Evaluate(idm_state.ego_s);  // TODO: if needed, add preview
      target_acc = std::min(target_acc,
                            SpeedIdmCommon::CalMaxFrictionCircleBoundTargetAcc(
                                speed_gaming_params_->max_friction_circle_acc,
                                idm_state.ego_v, cur_line_plant.kappa()));
      target_jerk = SpeedGamingCommon::CalcSaturatedJerkByAcc(
          idm_state.ego_a, target_acc, 1.0 / time_step, time_step, min_jerk,
          max_jerk);
    }
    if (!speed_limit_plf.x()
             .empty()) {  // speed_limit_plf is empty, no speed_constraint
      max_v = std::min(max_v, speed_limit_plf(idm_state.ego_s));
      max_v = std::min(max_v,
                       speed_limit_plf(idm_state.ego_s +
                                       std::max(idm_state.ego_v, 6.0) * 2.0));
    }
    if (idm_state.ego_v > max_v) {
      double speed_error = max_v - idm_state.ego_v;
      double speed_limit_acc = 0.75 * speed_error;
      target_acc = std::min(std::min(target_acc, 0.0), speed_limit_acc);
      target_jerk = SpeedGamingCommon::CalcSaturatedJerkByAcc(
          idm_state.ego_a, target_acc, 1.0 / time_step, time_step, min_jerk,
          max_jerk);
    }
    idm_state = SpeedGamingCommon::UpdateNextEgoState(idm_state, target_jerk,
                                                      time_step);
    yield_speed_vector.emplace_back(simu_time, idm_state.ego_s, idm_state.ego_v,
                                    idm_state.ego_a, idm_state.ego_j);
    yield_speed_vector[index - 1].set_j(idm_state.ego_j);
  }
  return yield_speed_vector;
}

double MultiAgentIdm::CalcMergeYieldAcc(
    const SpeedIdmState &idm_state, const GamingSimResult &yield_conflict_zone,
    const int index, const SpeedIdmUpdateParams &idm_param, double &beta,
    const DiscretizedPath &av_path, LastInfos &last_info) {
  auto &simulation_conflict_zone =
      yield_conflict_zone.sim_conflict_zone_in_ego_view;
  const double simu_time = index * speed_gaming_params_->time_step;
  SpeedIdmState idm_cur_state = idm_state;
  if (simu_time < simulation_conflict_zone.agent_cutin_time) {
    idm_cur_state.agent_s =
        yield_conflict_zone.planner_object
            ? std::max(idm_cur_state.ego_s,
                       simulation_conflict_zone.ego_cutin_s -
                           yield_conflict_zone.planner_object->bounding_box()
                                   .length() *
                               cos(last_info.last_delta_theta))
            : simulation_conflict_zone.ego_cutin_s;
    // const double dt =
    //     simulation_conflict_zone.agent_cutout_time -
    //     simulation_conflict_zone.agent_cutin_time;
    // const double ds =
    //     simulation_conflict_zone.ego_cutout_s -
    //     simulation_conflict_zone.ego_cutin_s;
    // TODO: change to agent speed projection
    idm_cur_state.agent_v = yield_conflict_zone.interaction_type == kTurnMerge
                                ? 0
                                : last_info.last_v;
    idm_cur_state.agent_a = last_info.last_a;

    const double ttc_idm = std::max(
        0.3, (idm_cur_state.agent_s - idm_cur_state.ego_s - idm_param.d0) /
                 std::max(idm_cur_state.ego_v - idm_cur_state.agent_v, 0.1));
    beta = std::max(idm_param.beta, 1.0 / ttc_idm);

    SpeedIdmUpdateParams idm_param_current = idm_param;
    idm_param_current.beta = beta;
    return SpeedIdmCommon::CalcYieldIdmAcc(idm_cur_state, idm_param_current);
  } else if (simu_time < simulation_conflict_zone.agent_cutout_time) {
    int obj_traj_size = yield_conflict_zone.obj_traj.size() - 1;
    const auto cur_agent_pt =
        yield_conflict_zone.obj_traj[std::min(index, obj_traj_size)];
    const double agent_s = cur_agent_pt.s();
    const double agent_theta = cur_agent_pt.theta();
    const double agent_v = cur_agent_pt.v();
    const double agent_a = cur_agent_pt.a();
    const double last_agent_s =
        index > 0
            ? yield_conflict_zone.obj_traj[std::min(index, obj_traj_size) - 1]
                  .s()
            : 0.0;

    const double ref_s = av_path.XYToSL(cur_agent_pt.pos()).s;
    auto ref_pt = av_path.Evaluate(ref_s);
    const double ref_theta = ref_pt.theta();
    const double delta_theta = NormalizeAngle(ref_theta - agent_theta);

    idm_cur_state.agent_s =
        last_info.last_s +
        (agent_s - last_agent_s) *
            std::abs(std::cos(
                NormalizeAngle(delta_theta + last_info.last_delta_theta) / 2));
    idm_cur_state.agent_v = agent_v * std::cos(delta_theta);
    idm_cur_state.agent_a = agent_a * std::cos(delta_theta);

    const double ttc_idm = std::max(
        0.3, (idm_cur_state.agent_s - idm_cur_state.ego_s - idm_param.d0) /
                 std::max(idm_cur_state.ego_v - idm_cur_state.agent_v, 0.1));

    const double beta = std::max(idm_param.beta, 1.0 / ttc_idm);

    SpeedIdmUpdateParams idm_param_current = idm_param;
    last_info.last_s = idm_cur_state.agent_s;
    last_info.last_delta_theta = delta_theta;
    idm_param_current.beta = beta;
    return SpeedIdmCommon::CalcYieldIdmAcc(idm_cur_state, idm_param_current);
  }
  return 2.0;
}

double MultiAgentIdm::CalcCrossYieldAcc(
    SpeedIdmState &idm_state, const GamingSimResult &yield_conflict_zone,
    double simu_time, const SpeedIdmUpdateParams &idm_param,
    const IDMConfigPlf &idm_config_plf, double &beta) {
  const auto &simulation_conflict_zone =
      yield_conflict_zone.sim_conflict_zone_in_ego_view;
  SpeedIdmState idm_cur_state = idm_state;
  idm_cur_state.agent_s = simulation_conflict_zone.ego_cutin_s;
  idm_cur_state.agent_v = 0.0;
  idm_cur_state.agent_a = 0.0;
  double ttc_idm = std::max(
      0.3, (idm_cur_state.agent_s - idm_cur_state.ego_s - idm_param.d0) /
               std::max((idm_cur_state.ego_v - idm_cur_state.agent_v), 0.1));
  beta = std::max(idm_param.beta, 1.0 / ttc_idm);
  SpeedIdmUpdateParams idm_param_current = idm_param;
  double cut_out_remain_time =
      simulation_conflict_zone.agent_cutout_time - simu_time;
  double K_0_risk = idm_config_plf.k0_plf(cut_out_remain_time);
  double d_0_risk = idm_config_plf.d0_plf(cut_out_remain_time);
  idm_param_current.k0 = K_0_risk;
  idm_param_current.d0 = d_0_risk;
  idm_param_current.beta = beta;
  return SpeedIdmCommon::CalcYieldIdmAcc(idm_cur_state, idm_param_current);
}
double MultiAgentIdm::CalcMergeOvertakeAcc(
    const SpeedIdmState &idm_state, const GamingSimResult &yield_conflict_zone,
    double simu_time, const SpeedIdmUpdateParams &idm_param, double &beta) {
  auto &simulation_conflict_zone =
      yield_conflict_zone.sim_conflict_zone_in_ego_view;
  SpeedIdmState idm_cur_state = idm_state;
  idm_cur_state.agent_s = simulation_conflict_zone.ego_cutin_s;
  idm_cur_state.agent_v = 0.0;  // TODO: change to agent speed projection
  idm_cur_state.agent_a = 0.0;
  double agent_cur_cutin_time =
      simulation_conflict_zone.agent_cutin_time - simu_time;
  const double ego_cutin_dist = simulation_conflict_zone.ego_cutin_s;
  double ego_cur_cutin_dist = ego_cutin_dist - idm_cur_state.ego_s;
  double ego_cutin_time = ego_cur_cutin_dist / idm_cur_state.ego_v;
  double hwt_gap = agent_cur_cutin_time - ego_cutin_time;
  double ttc_idm = std::max(1.0, hwt_gap);
  beta = std::max(0.5, 1.0 / ttc_idm);
  SpeedIdmUpdateParams idm_param_current = idm_param;
  idm_param_current.beta = beta;
  return SpeedIdmCommon::CalcPassIdmAcc(
      idm_cur_state, idm_param_current,
      simulation_conflict_zone.ego_cross_angle, agent_cur_cutin_time,
      ego_cur_cutin_dist);
}

double MultiAgentIdm::CalcCrossOvertakeAcc(
    const SpeedIdmState &idm_state, const GamingSimResult &yield_conflict_zone,
    double simu_time, const SpeedIdmUpdateParams &idm_param, double &beta) {
  auto &simulation_conflict_zone =
      yield_conflict_zone.sim_conflict_zone_in_ego_view;
  SpeedIdmState idm_cur_state = idm_state;
  idm_cur_state.agent_s =
      simulation_conflict_zone.ego_cutout_s;  // Should be cut out?
  idm_cur_state.agent_v = 0.0;  // TODO: change to agent speed projection
  idm_cur_state.agent_a = 0.0;
  double agent_cur_cutin_time =
      simulation_conflict_zone.agent_cutin_time - simu_time;
  const double ego_cutout_dist = simulation_conflict_zone.ego_cutout_s;
  double ego_cur_cutout_dist = ego_cutout_dist - idm_cur_state.ego_s;
  double hwt_gap = agent_cur_cutin_time -
                   ego_cur_cutout_dist / std::max(idm_cur_state.ego_v, 0.2);
  double ttc_idm = std::max(1.0, hwt_gap);
  beta = std::max(0.6, 1.0 / ttc_idm);
  SpeedIdmUpdateParams idm_param_current = idm_param;
  idm_param_current.beta = beta;
  return SpeedIdmCommon::CalcPassIdmAcc(
      idm_cur_state, idm_param_current,
      simulation_conflict_zone.ego_cross_angle, agent_cur_cutin_time,
      ego_cur_cutout_dist);
}

// 重复造轮子，链一下
double MultiAgentIdm::CalcUpperSpeedBoundAcc(
    double t, const SpeedIdmState &cur_state,
    const SpeedVector &upper_speed_bound) {
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
               std::max((cur_state.ego_v - upper_speed_point->v()), kEps);
  ttc = std::max(1.0, ttc);
  idm_param.beta = std::max(0.5, 1.0 / ttc);
  SpeedIdmState virtual_state = cur_state;
  virtual_state.agent_s = upper_speed_point->s();
  virtual_state.agent_v = upper_speed_point->v();
  virtual_state.agent_a = upper_speed_point->a();

  return SpeedIdmCommon::CalcYieldIdmAcc(virtual_state, idm_param);
}

// 重复造轮子，链一下
double MultiAgentIdm::CalcNormalSpeedTrackAcc(
    double t, const SpeedIdmState &cur_state,
    const SpeedVector &upper_speed_bound) {
  return CalcUpperSpeedBoundAcc(t, cur_state, upper_speed_bound);
}

SpeedVector MultiAgentIdm::CalcTrajSpeedDataOfMultiAgent(
    const TrajectoryPoint &init_point, const DiscretizedPath &av_path,
    const SpeedVector &upper_speed_data, const SpeedVector &normal_speed_data,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedIdmUpdateParams &idm_param,
    const std::unordered_map<std::string, GamingSimResult>
        &ego_overtake_sim_results) {
  SpeedVector ego_speed_vector{};
  if (upper_speed_data.empty() || normal_speed_data.empty()) {
    return ego_speed_vector;
  }
  double ego_s = 0.0;
  double ego_v = init_point.v();
  double ego_a = init_point.a();
  double ego_jerk = 0.0;
  double agent_s = 10000.0;
  double agent_v = 10.0;
  double agent_a = 0.0;
  double agent_j = 0.0;
  SpeedIdmState idm_state(ego_s, ego_v, ego_a, ego_jerk, agent_s, agent_v,
                          agent_a, agent_j);
  int start_index = 1;
  const auto planning_horizon = speed_gaming_params_->planning_horizon;
  const auto time_step = speed_gaming_params_->time_step;
  const auto min_jerk = speed_gaming_params_->min_jerk;
  const auto max_jerk = speed_gaming_params_->max_jerk;
  auto max_v = speed_gaming_params_->max_vel;

  int end_index = std::floor(planning_horizon / time_step) + 1;
  ego_speed_vector.emplace_back(0.0, ego_s, ego_v, ego_a, ego_jerk);
  for (int index = start_index; index < end_index; ++index) {
    double simu_time = index * time_step;
    // double target_acc = normal_speed_data.a();
    double target_acc =
        CalcNormalSpeedTrackAcc(simu_time, idm_state, normal_speed_data);
    double beta = idm_param.beta;
    double target_jerk = SpeedGamingCommon::CalcSaturatedJerkByAcc(
        idm_state.ego_a, target_acc, 1.0 / time_step, time_step, min_jerk,
        max_jerk);

    double track_upper_bound_acc =
        CalcUpperSpeedBoundAcc(simu_time, idm_state, upper_speed_data);
    for (const auto &overtake_result : ego_overtake_sim_results) {
      if (simu_time > overtake_result.second.sim_conflict_zone_in_ego_view
                          .agent_cutin_time) {
        continue;
      }
      if (overtake_result.second.interaction_type ==
              InteractionType::kStraightMerge ||
          overtake_result.second.interaction_type ==
              InteractionType::kTurnMerge) {
        double merge_pass_target_acc = CalcMergeOvertakeAcc(
            idm_state, overtake_result.second, simu_time, idm_param, beta);
        target_acc = std::max(target_acc, merge_pass_target_acc);
        target_acc = std::min(target_acc, track_upper_bound_acc);
        target_jerk =
            std::max(SpeedGamingCommon::CalcSaturatedJerkByAcc(
                         idm_state.ego_a, target_acc, 1.0 / time_step,
                         time_step, min_jerk, max_jerk),
                     target_jerk);  // only pass conflict with bound, output max
      } else if (overtake_result.second.interaction_type ==
                 InteractionType::kCross) {
        double cross_pass_target_acc = CalcCrossOvertakeAcc(
            idm_state, overtake_result.second, simu_time, idm_param, beta);
        target_acc = std::max(target_acc, cross_pass_target_acc);
        target_acc = std::min(target_acc, track_upper_bound_acc);
        target_jerk =
            std::max(SpeedGamingCommon::CalcSaturatedJerkByAcc(
                         idm_state.ego_a, target_acc, 1.0 / time_step,
                         time_step, min_jerk, max_jerk),
                     target_jerk);  // only pass conflict with bound, output max
      }
    }
    if (!av_path.empty()) {
      auto cur_line_plant =
          av_path.Evaluate(idm_state.ego_s);  // TODO: if needed, add preview
      target_acc = std::min(target_acc,
                            SpeedIdmCommon::CalMaxFrictionCircleBoundTargetAcc(
                                speed_gaming_params_->max_friction_circle_acc,
                                idm_state.ego_v, cur_line_plant.kappa()));
      target_jerk = SpeedGamingCommon::CalcSaturatedJerkByAcc(
          idm_state.ego_a, target_acc, 1.0 / time_step, time_step, min_jerk,
          max_jerk);
    }
    if (!speed_limit_plf.x().empty()) {
      // speed_limit_ptf is empty, no speed_constraint
      max_v = std::min(max_v, speed_limit_plf(idm_state.ego_s));
      max_v = std::min(max_v,
                       speed_limit_plf(idm_state.ego_s +
                                       std::max(idm_state.ego_v, 6.0) * 2.0));
    }
    if (idm_state.ego_v > max_v) {
      double speed_error = max_v - idm_state.ego_v;
      double speed_limit_acc = 0.75 * speed_error;
      target_acc = std::min(std::min(target_acc, 0.0), speed_limit_acc);
      target_jerk = SpeedGamingCommon::CalcSaturatedJerkByAcc(
          idm_state.ego_a, target_acc, 1.0 / time_step, time_step, min_jerk,
          max_jerk);
    }
    idm_state = SpeedGamingCommon::UpdateNextEgoState(idm_state, target_jerk,
                                                      time_step);
    ego_speed_vector.emplace_back(simu_time, idm_state.ego_s, idm_state.ego_v,
                                  idm_state.ego_a, idm_state.ego_j);
    ego_speed_vector[index - 1].set_j(idm_state.ego_j);
  }
  return ego_speed_vector;
}
}  // namespace st::planning