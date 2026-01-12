/***********************************************************
 * @file     alternative_idm_simulator.cc
 * @author   tianrui.liu
 * @date     2025.03.15
 * @brief    The implementation of alternative-idm simulator
 * @version  1.0
 ***********************************************************/
#include "alternative_idm_simulator.h"
#include "speed_idm_base.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
#include <json/json.h>
#include "alternative_gaming/speed_gaming/test/test_util.h"
#endif

namespace st::planning {
inline bool IsLargeVehicle(const PlannerObject &gaming_object) {
  auto obs_type = gaming_object.type();
  double obs_length =
      !gaming_object.num_trajs() ? gaming_object.bounding_box().length() : 5.0;
  double obs_width =
      !gaming_object.num_trajs() ? gaming_object.bounding_box().width() : 2.0;
  bool is_big_vehicle =
      (obs_type == OT_LARGE_VEHICLE &&
       (obs_length > 6.0 || obs_width > 2.6 || obs_length * obs_width > 11.6));
  return is_big_vehicle;
}

void AlternativeIDMSimulator::RunEgoPassSimulation(
    const AlternativeSimulatorInfo &alter_simulator_info,
    const PlannerObject &gaming_object, GamingSimResult *gaming_sim_result,
    IdmSimulateState *simulate_state) const {
  // 推演10次
  auto &ego_pass_speed_data = gaming_sim_result->ego_speed_data;
  auto &obj_yield_speed_data = gaming_sim_result->obj_speed_data;
  const auto &ego_init_point = alter_simulator_info.ego_init_point;
  const auto &agent_init_point = alter_simulator_info.agent_init_point;
  gaming_sim_result->sim_conflict_zone_in_ego_view =
      alter_simulator_info.riskfield_conflict_zone_in_ego_view;
  gaming_sim_result->sim_conflict_zone_in_agent_view =
      alter_simulator_info.riskfield_conflict_zone_in_agent_view;
  double ego_cross_angle = alter_simulator_info.ego_cross_angle;
  const auto &ego_speed_limit_plf = alter_simulator_info.ego_speed_limit_plf;
  const auto &agent_speed_limit_plf =
      alter_simulator_info.agent_speed_limit_plf;
  const auto &ego_upper_speed_data = alter_simulator_info.upper_speed_data;
  const auto &ego_normal_yield_speed_data =
      alter_simulator_info.normal_yield_speed_data;
  const auto &ego_path = alter_simulator_info.av_path;
  const auto &agent_path = alter_simulator_info.agent_path;
  const double obj_speed =
      static_cast<double>(std::abs(gaming_object.pose().v()));

  const auto &config = this->GetConfig();
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
  // 打印推演前结果
  std::string simulation_type = "EgoPass";
  if (aidm_scene_ == AidmScene::kCross) {
    simulation_type.append("+Cross");
  } else {
    simulation_type.append("+Merge");
  }
  SaveUnitTestData(g_root_, 0, gaming_object.id(), simulation_type,
                   ego_pass_speed_data, obj_yield_speed_data,
                   gaming_sim_result->sim_conflict_zone_in_ego_view);
#endif
  for (int i = 0; i < config.iter_time; i++) {
    // 推自车抢行
    SpeedIdmConfig speed_idm_config;
    speed_idm_config.update_coeff = config.update_coeff;
    speed_idm_config.fast_acc_to_dec = false;
    speed_idm_config.no_inertial_update_in_first_frame = false;
    speed_idm_base_->SetConfig(speed_idm_config);
    speed_idm_base_->SetGamingParams(*speed_gaming_params_);
    if (simulate_state != nullptr) {
      simulate_state->is_agent_state = false;
    }
    if (!PassSimulation(ego_init_point, ego_path, ego_upper_speed_data,
                        ego_normal_yield_speed_data,
                        gaming_sim_result->sim_conflict_zone_in_ego_view,
                        obj_yield_speed_data,
                        gaming_sim_result->sim_conflict_zone_in_agent_view,
                        ego_cross_angle, ego_speed_limit_plf,
                        &ego_pass_speed_data, simulate_state)) {
      continue;
    }
    ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_ego_view));
    ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_agent_view));

    // 推社会车让行idm，用于让行主车
    speed_idm_config.update_coeff =
        config.update_coeff * config.cooperation_grade;
    speed_idm_config.fast_acc_to_dec = true;
    speed_idm_config.no_inertial_update_in_first_frame = false;
    auto gaming_param = *speed_gaming_params_;
    // 推演障碍车让行， 大车需要额外考虑
    if (IsLargeVehicle(gaming_object)) {
      gaming_param.min_acc = -3.0;
      gaming_param.min_jerk = -3.0;
    }
    // 推演障碍车抢行，额外考虑VRU
    if (gaming_object.type() == OT_CYCLIST) {
      gaming_param.max_acc = 1.0;
      gaming_param.min_acc = -3.0;
      gaming_param.max_vel = std::max(7.0, obj_speed);
    }

    if (gaming_object.type() == OT_TRICYCLIST) {
      gaming_param.max_acc = 1.0;
      gaming_param.min_acc = -3.0;
      gaming_param.max_vel = std::max(7.0, obj_speed);
    }

    if (gaming_object.type() == OT_PEDESTRIAN) {
      gaming_param.max_acc = 1.0;
      gaming_param.min_acc = -3.0;
      gaming_param.max_vel = std::max(2.0, obj_speed);
    }

    speed_idm_base_->SetConfig(speed_idm_config);
    speed_idm_base_->SetGamingParams(gaming_param);
    if (simulate_state != nullptr) {
      simulate_state->is_agent_state = true;
    }
    if (!YieldSimulation(agent_init_point, agent_path,
                         gaming_sim_result->sim_conflict_zone_in_agent_view,
                         ego_pass_speed_data,
                         gaming_sim_result->sim_conflict_zone_in_ego_view,
                         agent_speed_limit_plf, &obj_yield_speed_data,
                         gaming_object, simulate_state)) {
      ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_ego_view));
      ReverseEgoAgentInfo(
          &(gaming_sim_result->sim_conflict_zone_in_agent_view));
      continue;
    }
    ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_ego_view));
    ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_agent_view));

#ifdef ALTERNATIVE_GAMING_UNIT_TEST
    // 打印每个周期推算结果
    SaveUnitTestData(g_root_, i + 1, gaming_object.id(), simulation_type,
                     ego_pass_speed_data, obj_yield_speed_data,
                     gaming_sim_result->sim_conflict_zone_in_ego_view);
#endif
  }
}

void AlternativeIDMSimulator::RunEgoYieldSimulation(
    const AlternativeSimulatorInfo &alter_simulator_info,
    const PlannerObject &gaming_object, GamingSimResult *gaming_sim_result,
    IdmSimulateState *simulate_state) const {
  auto &ego_yield_speed_data = gaming_sim_result->ego_speed_data;
  auto &obj_pass_speed_data = gaming_sim_result->obj_speed_data;
  const auto &ego_init_point = alter_simulator_info.ego_init_point;
  const auto &agent_init_point = alter_simulator_info.agent_init_point;
  gaming_sim_result->sim_conflict_zone_in_ego_view =
      alter_simulator_info.riskfield_conflict_zone_in_ego_view;
  gaming_sim_result->sim_conflict_zone_in_agent_view =
      alter_simulator_info.riskfield_conflict_zone_in_agent_view;
  double agent_cross_angle = alter_simulator_info.agent_cross_angle;
  const auto &ego_speed_limit_plf = alter_simulator_info.ego_speed_limit_plf;
  const auto &agent_speed_limit_plf =
      alter_simulator_info.agent_speed_limit_plf;
  const auto &ego_path = alter_simulator_info.av_path;
  const auto &agent_path = alter_simulator_info.agent_path;

  const auto &config = this->GetConfig();

  double obj_speed = static_cast<double>(
      std::abs(gaming_object.pose().v()));  // TODO(haojie): Check this!
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
  // 打印推演前结果
  std::string simulation_type = "EgoYield";
  if (aidm_scene_ == AidmScene::kCross) {
    simulation_type.append("+Cross");
  } else {
    simulation_type.append("+Merge");
  }
  SaveUnitTestData(g_root_, 0, gaming_object.id(), simulation_type,
                   ego_yield_speed_data, obj_pass_speed_data,
                   gaming_sim_result->sim_conflict_zone_in_ego_view);
#endif
  for (int i = 0; i < config.iter_time; i++) {
    // 主车根据抢行idm算出新的自车的speed_profile,用于障碍车下一次的推演
    // 注意：算抢行的时候可能并不需要风险场信息
    // 推演自车让
    SpeedIdmConfig speed_idm_config;
    speed_idm_config.update_coeff = config.ego_yield_coeff;
    speed_idm_config.fast_acc_to_dec = true;
    speed_idm_config.no_inertial_update_in_first_frame = true;
    speed_idm_base_->SetConfig(speed_idm_config);
    speed_idm_base_->SetGamingParams(*speed_gaming_params_);
    if (simulate_state != nullptr) {
      simulate_state->is_agent_state = false;
    }
    if (!YieldSimulation(ego_init_point, ego_path,
                         gaming_sim_result->sim_conflict_zone_in_ego_view,
                         obj_pass_speed_data,
                         gaming_sim_result->sim_conflict_zone_in_agent_view,
                         ego_speed_limit_plf, &ego_yield_speed_data,
                         gaming_object, simulate_state)) {
      continue;
    }
    ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_ego_view));
    ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_agent_view));
    // WriteCutinCutoutInfoToCSV(filename2, riskfield_conflict_zone_in_ego_view,
    // true); 社会车抢行主车
    speed_idm_config.update_coeff = config.ego_yield_coeff;
    speed_idm_config.fast_acc_to_dec = false;
    speed_idm_config.no_inertial_update_in_first_frame = false;

    SpeedVector ego_upper_speed_data;
    SpeedVector ego_normal_yield_speed_data;
    auto gaming_param = *speed_gaming_params_;
    // 推演障碍车抢行，额外考虑VRU
    if (gaming_object.type() == OT_CYCLIST) {
      gaming_param.max_acc = 1.0;
      gaming_param.min_acc = -3.0;
      gaming_param.max_vel = std::max(7.0, obj_speed);
    }
    if (gaming_object.type() == OT_TRICYCLIST) {
      gaming_param.max_acc = 1.0;
      gaming_param.min_acc = -3.0;
      gaming_param.max_vel = std::max(7.0, obj_speed);
    }
    if (gaming_object.type() == OT_PEDESTRIAN) {
      gaming_param.max_acc = 1.0;
      gaming_param.min_acc = -3.0;
      gaming_param.max_vel = std::max(2.0, obj_speed);
    }

    speed_idm_base_->SetConfig(speed_idm_config);
    speed_idm_base_->SetGamingParams(gaming_param);
    if (simulate_state != nullptr) {
      simulate_state->is_agent_state = true;
    }
    if (!PassSimulation(agent_init_point, agent_path, ego_upper_speed_data,
                        ego_normal_yield_speed_data,
                        gaming_sim_result->sim_conflict_zone_in_agent_view,
                        ego_yield_speed_data,
                        gaming_sim_result->sim_conflict_zone_in_ego_view,
                        agent_cross_angle, agent_speed_limit_plf,
                        &obj_pass_speed_data, simulate_state)) {
      ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_ego_view));
      ReverseEgoAgentInfo(
          &(gaming_sim_result->sim_conflict_zone_in_agent_view));
      continue;
    }
    ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_ego_view));
    ReverseEgoAgentInfo(&(gaming_sim_result->sim_conflict_zone_in_agent_view));

#ifdef ALTERNATIVE_GAMING_UNIT_TEST
    // 打印每个周期推算结果
    SaveUnitTestData(g_root_, i + 1, gaming_object.id(), simulation_type,
                     ego_yield_speed_data, obj_pass_speed_data,
                     gaming_sim_result->sim_conflict_zone_in_ego_view);
#endif
  }
}

bool AlternativeIDMSimulator::PassSimulation(
    const TrajectoryPoint &init_point, const DiscretizedPath &av_path,
    const SpeedVector &ego_upper_speed_data,
    const SpeedVector &ego_normal_speed_data,
    GamingConflictZone &risk_field_conflict_zone_agent_1,
    SpeedVector &agent_2_speed_profile,
    GamingConflictZone &risk_field_conflict_zone_agent_2,
    double ego_cross_angle,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    SpeedVector *agent_1_speed_profile,
    IdmSimulateState *simulate_state) const {
  // agent1 代表pass推演车 agent2 代表交互车
  auto origin_speed_profile = *agent_1_speed_profile;

  // 博弈类型由外层绑定
  this->CalPassSpeedProfile(
      init_point, av_path, risk_field_conflict_zone_agent_1, ego_cross_angle,
      speed_limit_plf, ego_upper_speed_data, ego_normal_speed_data,
      agent_2_speed_profile, origin_speed_profile, agent_1_speed_profile,
      simulate_state);

  // Step 1: 使用 agent_1 的速度数据来更新冲突区信息
  std::optional<SpeedPoint> agent1_cutin_speed_point =
      agent_1_speed_profile->EvaluateByS(
          risk_field_conflict_zone_agent_1.ego_cutin_s, true);
  std::optional<SpeedPoint> agent1_cutout_speed_point =
      agent_1_speed_profile->EvaluateByS(
          risk_field_conflict_zone_agent_1.ego_cutout_s, true);
  if (agent1_cutin_speed_point == std::nullopt ||
      agent1_cutout_speed_point == std::nullopt) {
    return false;
  }
  // 计算并返回 ego_cutin_time_in_ego_view
  risk_field_conflict_zone_agent_1.ego_cutin_time =
      agent1_cutin_speed_point->t();
  risk_field_conflict_zone_agent_1.ego_cutout_time =
      agent1_cutout_speed_point->t();

  // 更新自车视角冲突区信息
  std::optional<SpeedPoint> agent2_cutin_speed_point =
      agent_1_speed_profile->EvaluateByS(
          risk_field_conflict_zone_agent_2.ego_cutin_s, true);
  std::optional<SpeedPoint> agent2_cutout_speed_point =
      agent_1_speed_profile->EvaluateByS(
          risk_field_conflict_zone_agent_2.ego_cutout_s, true);
  if (agent2_cutin_speed_point == std::nullopt ||
      agent2_cutout_speed_point == std::nullopt) {
    return false;
  }

  // 计算并返回 ego_cutin_time_in_agent_view
  risk_field_conflict_zone_agent_2.ego_cutin_time =
      agent2_cutin_speed_point->t();
  risk_field_conflict_zone_agent_2.ego_cutout_time =
      agent2_cutout_speed_point->t();

  return true;
}

bool AlternativeIDMSimulator::YieldSimulation(
    const TrajectoryPoint &init_point, const DiscretizedPath &av_path,
    GamingConflictZone &risk_field_conflict_zone_agent_1,
    SpeedVector &agent_2_speed_profile,
    GamingConflictZone &risk_field_conflict_zone_agent_2,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    SpeedVector *agent_1_speed_profile, const PlannerObject &gaming_object,
    IdmSimulateState *simulate_state) const {
  // agent1 代表yield推演车 agent2 代表交互车
  auto origin_speed_data = *agent_1_speed_profile;
  this->CalYieldSpeedProfile(
      init_point, av_path, risk_field_conflict_zone_agent_1,
      agent_2_speed_profile, speed_limit_plf, origin_speed_data,
      agent_1_speed_profile, gaming_object, simulate_state);

  // Step 1: 使用 agent_1 的速度数据来更新冲突区信息
  std::optional<SpeedPoint> agent1_cutin_speed_point =
      agent_1_speed_profile->EvaluateByS(
          risk_field_conflict_zone_agent_1.ego_cutin_s, true);
  std::optional<SpeedPoint> agent1_cutout_speed_point =
      agent_1_speed_profile->EvaluateByS(
          risk_field_conflict_zone_agent_1.ego_cutout_s, true);
  if (agent1_cutin_speed_point == std::nullopt ||
      agent1_cutout_speed_point == std::nullopt) {
    return false;
  }
  // 计算并返回 ego_cutin_time_in_ego_view
  risk_field_conflict_zone_agent_1.ego_cutin_time =
      agent1_cutin_speed_point->t();
  risk_field_conflict_zone_agent_1.ego_cutout_time =
      agent1_cutout_speed_point->t();

  // 更新自车视角冲突区信息
  std::optional<SpeedPoint> agent2_cutin_speed_point =
      agent_1_speed_profile->EvaluateByS(
          risk_field_conflict_zone_agent_2.ego_cutin_s, true);
  std::optional<SpeedPoint> agent2_cutout_speed_point =
      agent_1_speed_profile->EvaluateByS(
          risk_field_conflict_zone_agent_2.ego_cutout_s, true);
  if (agent2_cutin_speed_point == std::nullopt ||
      agent2_cutout_speed_point == std::nullopt) {
    return false;
  }
  // 计算并返回 ego_cutin_time_in_agent_view
  risk_field_conflict_zone_agent_2.ego_cutin_time =
      agent2_cutin_speed_point->t();
  risk_field_conflict_zone_agent_2.ego_cutout_time =
      agent2_cutout_speed_point->t();
  return true;
}

bool AlternativeIDMSimulator::CalPassSpeedProfile(
    const TrajectoryPoint &agent1_init_point, const DiscretizedPath &av_path,
    const GamingConflictZone &riskfield_conflict_zone_agent_1,
    double ego_cross_angle,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedVector &ego_upper_speed_data,
    const SpeedVector &ego_normal_yield_speed_data,
    const SpeedVector &agent2_decel_speed_profile,
    const SpeedVector &origin_speed_data, SpeedVector *pass_speed_data,
    IdmSimulateState *simulate_state) const {
  // agent_1是idm推演的车, agent_2是发生交互的车
  if (pass_speed_data == nullptr) {
    return false;
  }
  pass_speed_data->clear();
  const double &agent2_cutin_dist =
      riskfield_conflict_zone_agent_1.agent_cutin_s;
  const double &agent1_cutin_dist = riskfield_conflict_zone_agent_1.ego_cutin_s;

  // 获取交互车的cut-in时间和速度
  std::optional<SpeedPoint> speed_point =
      agent2_decel_speed_profile.EvaluateByS(agent2_cutin_dist, true);
  if (speed_point == std::nullopt) {
    return false;
  }

  double agent2_cutin_time = riskfield_conflict_zone_agent_1.agent_cutin_time;
  double agent2_cutin_speed = speed_point->v();

  // agent1 推演 初始状态
  double agent1_s = 0.0;
  double agent1_v = agent1_init_point.v();
  double agent1_a = agent1_init_point.a();
  double agent1_j = agent1_init_point.j();
  // agent2 推演 初始状态
  double agent2_s = 0.0;
  double agent2_v = agent2_cutin_speed;
  double agent2_a = 0.0;
  double agent2_j = 0.0;
  SpeedIdmState init_idm_state(agent1_s, agent1_v, agent1_a, agent1_j, agent2_s,
                               agent2_v, agent2_a, agent2_j);
  IdmInteractiveInfo idm_teractive_info;
  idm_teractive_info.agent_speed_profile = &agent2_decel_speed_profile;
  idm_teractive_info.agent_to_conflict_zone_time = agent2_cutin_time;
  idm_teractive_info.agent_to_conflict_zone_dist = agent2_cutin_dist;
  idm_teractive_info.agent_leave_conflict_zone_time =
      riskfield_conflict_zone_agent_1.agent_cutout_time;
  idm_teractive_info.agent_leave_conflict_zone_dist =
      riskfield_conflict_zone_agent_1.agent_cutout_s;
  idm_teractive_info.ego_cross_angle = ego_cross_angle;
  idm_teractive_info.ego_to_conflict_zone_time =
      riskfield_conflict_zone_agent_1.ego_cutin_time;
  idm_teractive_info.ego_to_conflict_zone_dist =
      riskfield_conflict_zone_agent_1.ego_cutin_s;
  idm_teractive_info.ego_leave_conflict_zone_time =
      riskfield_conflict_zone_agent_1.ego_cutout_time;
  idm_teractive_info.ego_leave_conflict_zone_dist =
      riskfield_conflict_zone_agent_1.ego_cutout_s;
  idm_teractive_info.av_path = av_path;
  // 以agent_1视角进行IDM推演

  *pass_speed_data = speed_idm_base_->CalcPassIdmSpeedVector(
      init_idm_state, origin_speed_data, ego_upper_speed_data,
      ego_normal_yield_speed_data, idm_teractive_info, speed_limit_plf,
      simulate_state);
  return true;
}

bool AlternativeIDMSimulator::CalYieldSpeedProfile(
    const TrajectoryPoint &init_point, const DiscretizedPath &av_path,
    const GamingConflictZone &riskfield_conflict_zone,
    const SpeedVector &agent_speed_profile,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedVector &origin_speed_data, SpeedVector *yield_speed_data,
    const PlannerObject &gaming_object,
    IdmSimulateState *simulate_state) const {
  if (yield_speed_data == nullptr) {
    return false;
  }
  yield_speed_data->clear();

  const double &agent2_cutin_time = riskfield_conflict_zone.agent_cutin_time;
  const double &agent2_cutin_dist = riskfield_conflict_zone.agent_cutin_s;
  const double &agent1_cutin_dist = riskfield_conflict_zone.ego_cutin_s;
  const double &agent1_cutin_time = riskfield_conflict_zone.ego_cutin_time;

  // agent_1状态，作为推演车
  double agent1_s = 0;
  double agent1_v = init_point.v();
  double agent1_a = init_point.a();
  double agent1_j = init_point.j();

  // agent_2状态，作为交互车
  double agent2_s = agent1_cutin_dist;
  double agent2_v = 0.0;
  double agent2_a = 0.0;
  double agent2_j = 0.0;
  SpeedIdmState init_idm_state(agent1_s, agent1_v, agent1_a, agent1_j, agent2_s,
                               agent2_v, agent2_a, agent2_j);
  IdmInteractiveInfo idm_teractive_info;
  idm_teractive_info.agent_speed_profile = &agent_speed_profile;
  idm_teractive_info.agent_to_conflict_zone_time = agent2_cutin_time;
  idm_teractive_info.agent_to_conflict_zone_dist = agent2_cutin_dist;
  idm_teractive_info.agent_leave_conflict_zone_time =
      riskfield_conflict_zone.agent_cutout_time;
  idm_teractive_info.agent_leave_conflict_zone_dist =
      riskfield_conflict_zone.agent_cutout_s;
  idm_teractive_info.ego_cross_angle = 0.0;
  idm_teractive_info.ego_to_conflict_zone_time = agent1_cutin_time;
  idm_teractive_info.ego_to_conflict_zone_dist = agent1_cutin_dist;
  idm_teractive_info.ego_leave_conflict_zone_time =
      riskfield_conflict_zone.ego_cutout_time;
  idm_teractive_info.ego_leave_conflict_zone_dist =
      riskfield_conflict_zone.ego_cutout_s;
  idm_teractive_info.gaming_object = gaming_object;
  idm_teractive_info.av_path = av_path;

  // 以agent_1视角进行IDM推演
  *yield_speed_data = speed_idm_base_->CalcYieldIdmSpeedVector(
      init_idm_state, origin_speed_data, idm_teractive_info, speed_limit_plf,
      simulate_state);
  return true;
}

void AlternativeIDMSimulator::ReverseEgoAgentInfo(
    GamingConflictZone *riskfiled_conflict_zone) const {
  const auto origin_conflict_zone = *riskfiled_conflict_zone;
  riskfiled_conflict_zone->ego_cutin_s = origin_conflict_zone.agent_cutin_s;
  riskfiled_conflict_zone->ego_cutin_time =
      origin_conflict_zone.agent_cutin_time;
  riskfiled_conflict_zone->ego_cutout_s = origin_conflict_zone.agent_cutout_s;
  riskfiled_conflict_zone->ego_cutout_time =
      origin_conflict_zone.agent_cutout_time;

  riskfiled_conflict_zone->agent_cutin_s = origin_conflict_zone.ego_cutin_s;
  riskfiled_conflict_zone->agent_cutin_time =
      origin_conflict_zone.ego_cutin_time;
  riskfiled_conflict_zone->agent_cutout_s = origin_conflict_zone.ego_cutout_s;
  riskfiled_conflict_zone->agent_cutout_time =
      origin_conflict_zone.ego_cutout_time;

  riskfiled_conflict_zone->agent_cross_angle =
      origin_conflict_zone.agent_cross_angle;
  riskfiled_conflict_zone->ego_cross_angle =
      origin_conflict_zone.ego_cross_angle;
}

}  // namespace st::planning
