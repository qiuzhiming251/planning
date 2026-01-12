/**
 * @file speed_gaming_decider.cc
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "speed_gaming_decider.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
#include <json/json.h>
#include "alternative_gaming/speed_gaming/test/test_util.h"
#endif
namespace st::planning {
constexpr double kSpeedLimit = 33.0;
constexpr double kMinimentStartTimeThresh = 0.001;
constexpr double kObstaclePathMaxLength = 100.0;
constexpr double kSimpleGamingD0 = 5.0;
constexpr double kDefaultSpeedLimit = 33.0;

absl::StatusOr<SpeedGamingOutput> SpeedGamingDecider::Execute(
    const SpeedGamingInput &input) {
  constexpr bool kGamingIsWorking = true;
  if (!kGamingIsWorking) {
    return absl::UnavailableError("gaming is not working.");
  }
  TIMELINE("SpeedGamingDecider::Execute");
  std::string name = Log2DDS::TaskPrefix(input.plan_id) + "Gaming_Execute";
  SCOPED_TRACE(name.c_str());
  std::string debug_info;
  std::string group_name;
  SpeedGamingOutput output;
  group_name = Log2DDS::TaskPrefix(input.plan_id) + "gaming/input";
  debug_info.clear();
  bool is_closed_junction = false;
  if (input.drive_passage->lane_seq_info() &&
      input.drive_passage->lane_seq_info()->lane_seq) {
    const auto &lanes = input.drive_passage->lane_seq_info()->lane_seq->lanes();
    const double dis_to_junction =
        input.drive_passage->lane_seq_info()->dist_to_junction;
    for (const auto &lane : lanes) {
      if (0 != lane->junction_id() && dis_to_junction < 50.0) {
        is_closed_junction = true;
        intersection_turn_type_ =
            (lane->turn_type() == ad_byd::planning::TurnType::LEFT_TURN)
                ? IntersectionTurnType::Left
                : IntersectionTurnType::None;
        break;
      }
    }
  }

  if (false) {
    debug_info = "not in junction,sleeping!";
    Log2DDS::LogDataV2(group_name, debug_info);
    return absl::UnavailableError("ego is not in junction.");
  } else {
    debug_info = "in junction,working!";
    Log2DDS::LogDataV2(group_name, debug_info);
  }
  debug_info.clear();

  /**
   * @brief step1: 障碍物预测轨迹前处理
   *
   */
  speed_obs_processor_->Init(input.traj_mgr, input.ego_path,
                             input.drive_passage, vehicle_geo_params_);
  speed_obs_processor_->ConvertStBoundaryToConflictZone(
      conflict_zone_infos_, *input.st_boundaries_with_decision,
      *input.processed_st_objects, input.gaming_lc_obs_set, input.plan_id);
  //增加风险场
  if (is_risk_field_enabled_) {
    speed_obs_processor_->RiskFieldGenerate(conflict_zone_infos_,
                                            *input.st_boundaries_with_decision);
  }
  object_cooperation_maps_.clear();
  if (input.last_speed_gaming_result) {
    for (auto obj_coop_info :
         input.last_speed_gaming_result->speed_gaming_objects()) {
      object_cooperation_maps_.insert(
          {obj_coop_info.obj_id(), obj_coop_info.cooperation_info()});
    }
  }
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
  SaveDebugData(conflict_zone_infos_, *input.ego_path, *input.traj_mgr);
#endif
  /**
   * @brief step2: construct ego_init_point/speed_limit_plf
   *
   */
  // TrajectoryPoint ego_init_point;
  ego_init_point_.set_pos(
      Vec2d(input.ego_path->front().x(), input.ego_path->front().y()));
  ego_init_point_.set_theta(input.ego_path->front().theta());
  ego_init_point_.set_s(input.ego_path->front().s());
  ego_init_point_.set_v(input.plan_start_v);
  ego_init_point_.set_a(input.plan_start_a);
  ego_init_point_.set_j(input.plan_start_j);

  // TO DO：当前只取静态限速，后续考虑是否与DP一致，加入动态限速，需要改限速接口
  std::vector<double> s_list(input.ego_path->size(), 0.0);
  std::vector<double> v_list(input.ego_path->size(), kDefaultSpeedLimit);
  for (size_t i = 0; i < s_list.size(); i++) {
    s_list[i] = input.ego_path->at(i).s();
    // auto iter = speed_limit_map.find(SpeedLimitTypeProto_Type_COMBINATION);
    // if (iter != speed_limit_map.end())
    // {
    auto tmp_v = input.speed_limit_provider->GetStaticSpeedLimitByS(s_list[i]);
    if (tmp_v.has_value()) {
      v_list[i] = std::min(v_list[i], tmp_v.value());
    }
    // }
  }
  speed_limit_plf_ = PiecewiseLinearFunction<double>(s_list, v_list);

  SpeedVector upper_speed_data;
  SpeedVector normal_yield_speed_data;
  multi_agent_idm_ = std::make_unique<MultiAgentIdm>(&speed_gaming_params_);
  SpeedIdmUpdateParams idm_param;
  std::unordered_set<std::string> ignore_objs;

  /**
   * @brief step3: merge Follow类型无需交替推演，直接生成upper_data
   *
   */
  for (auto iter = conflict_zone_infos_.begin();
       iter != conflict_zone_infos_.end(); iter++) {
    debug_info += iter->first;
    if (iter->second.is_ignore &&
        iter->second.interaction_type == InteractionType::kCross) {
      ignore_objs.insert(iter->first);
      debug_info += " is ignore obj. ";
    } else if (iter->second.is_follow ||
               (iter->second.is_pre_gaming &&
                (iter->second.interaction_type ==
                     InteractionType::kStraightMerge ||
                 iter->second.interaction_type == InteractionType::kTurnMerge ||
                 iter->second.interaction_type ==
                     InteractionType::kStaticOccupy))) {
      debug_info += " is follow obj. ";
      GamingSimResult sim_result;
      sim_result.ego_lon_type = LonGamingDecisionType::kYield;
      sim_result.interaction_type = iter->second.interaction_type;
      sim_result.sim_conflict_zone_in_ego_view =
          iter->second.conflict_zone_in_ego_view;
      sim_result.sim_conflict_zone_in_agent_view =
          iter->second.conflict_zone_in_agent_view;
      //优先从processed_st_objects中取轨迹，再从原始traj_mgr中取,只针对动态目标
      if (iter->second.interaction_type != InteractionType::kStaticOccupy) {
        auto processed_traj = input.processed_st_objects->find(iter->first);
        const auto obj_traj =
            processed_traj != input.processed_st_objects->end()
                ? &(processed_traj->second)
                : input.traj_mgr->FindTrajectoryById(
                      std::string_view(iter->first));
        auto gaming_processed_traj =
            SpeedGamingCommon::genThetaValidTraj(*obj_traj);
        sim_result.obj_speed_data =
            SpeedGamingCommon::ConvertPredTrajToSpeedData(
                &gaming_processed_traj, speed_gaming_params_.time_step,
                speed_gaming_params_.planning_horizon);
        const auto obj_path = SpeedGamingCommon::GeneratePathBasedTraj(
            &gaming_processed_traj, kObstaclePathMaxLength);
        sim_result.obj_traj = SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
            obj_path, sim_result.obj_speed_data);
      }
      // DumpSimResultGraph(group_name, "follow_obj_id" + iter->first,
      // sim_result);
      DumpConflictZoneGraph(
          group_name, "follow_" + iter->first, Log2DDS::kDarkBlue,
          sim_result.sim_conflict_zone_in_ego_view, sim_result.ego_lon_type);
      ego_follow_sim_results_.insert({iter->first, std::move(sim_result)});
    } else {
      debug_info += " is game obj. ";
      DumpConflictZoneGraph(group_name, "origin_" + iter->first, Log2DDS::kBlue,
                            iter->second.conflict_zone_in_ego_view,
                            LonGamingDecisionType::kNone);
    }
    debug_info += "\n";
  }
  for (auto it = ego_follow_sim_results_.begin();
       it != ego_follow_sim_results_.end(); it++) {
    DumpGamingSpeedVectorGraph(group_name, it->first, Log2DDS::kPink,
                               it->second.obj_speed_data);
    debug_info += absl::StrCat(
        it->first,
        " in_t: ", it->second.sim_conflict_zone_in_ego_view.agent_cutin_time,
        " in_s: ", it->second.sim_conflict_zone_in_ego_view.agent_cutin_s,
        " out_t: ", it->second.sim_conflict_zone_in_ego_view.agent_cutout_time,
        " out_s: ", it->second.sim_conflict_zone_in_ego_view.agent_cutout_s,
        "\n");
  }

  follow_upper_speed_data_ =
      multi_agent_idm_->CalcYieldTrajSpeedDataOfMultiAgent(
          ego_init_point_, *input.ego_path, upper_speed_data,
          normal_idm_config_plf_, follow_idm_config_plf_, speed_limit_plf_,
          idm_param, ego_yield_sim_results_, ego_follow_sim_results_);
  DumpGamingSpeedVectorGraph(group_name, "_follow_upper", Log2DDS::kBlue,
                             follow_upper_speed_data_);
  Log2DDS::LogDataV2(group_name, debug_info);

  /**
   * @brief step4: 单障碍物推演
   *
   */
  for (auto iter = conflict_zone_infos_.begin();
       iter != conflict_zone_infos_.end(); iter++) {
    if (ego_follow_sim_results_.count(iter->first) ||
        ignore_objs.count(iter->first)) {
      continue;
    }
    debug_info.clear();
    debug_info += "id:" + iter->second.obj_traj_id + ", type:";
    switch (iter->second.interaction_type) {
      case InteractionType::kCross:
        debug_info += "kCross\n";
        break;
      case InteractionType::kStraightMerge:
        debug_info += "kStraightMerge\n";
        break;
      case InteractionType::kTurnMerge:
        debug_info += "kTurnMerge\n";
        break;
      default:
        debug_info += "kNone\n";
        break;
    }
    debug_info += absl::StrCat(
        "ego_in_s:", iter->second.conflict_zone_in_ego_view.ego_cutin_s, "\n",
        "ego_out_s:", iter->second.conflict_zone_in_ego_view.ego_cutout_s, "\n",
        "agent_in_s:", iter->second.conflict_zone_in_ego_view.agent_cutin_s,
        "\n",
        "agent_out_s:", iter->second.conflict_zone_in_ego_view.agent_cutout_s,
        "\n");
    GamingSimResult sim_result;
    sim_result.interaction_type = iter->second.interaction_type;
    const auto &planner_object =
        input.traj_mgr->FindTrajectoryById(std::string_view(iter->first))
            ->planner_object();
    bool is_need_merge_process = (planner_object.type() == OT_CYCLIST ||
                                  planner_object.type() == OT_PEDESTRIAN ||
                                  planner_object.type() == OT_TRICYCLIST) ||
                                 iter->second.is_lc_pass;

    switch (iter->second.interaction_type)  // 只处理cross
    {
      case InteractionType::kCross:
        RunCrossGaming(*input.ego_path, ego_init_point_, *input.traj_mgr,
                       *input.processed_st_objects, iter->second,
                       speed_limit_plf_, follow_upper_speed_data_,
                       normal_yield_speed_data, &sim_result);
        break;
      case InteractionType::kStraightMerge:
        if (is_need_merge_process) {
          RunAgentMergeEgoGaming(
              *input.ego_path, ego_init_point_, *input.traj_mgr,
              *input.processed_st_objects, iter->second, speed_limit_plf_,
              follow_upper_speed_data_, normal_yield_speed_data, &sim_result);
        }
        break;
      case InteractionType::kTurnMerge:
        if (is_need_merge_process) {
          RunEgoMergeAgentGaming(
              *input.ego_path, ego_init_point_, *input.traj_mgr,
              *input.processed_st_objects, iter->second, speed_limit_plf_,
              follow_upper_speed_data_, normal_yield_speed_data, &sim_result);
        }
        break;
      default:
        break;
    }

    if (sim_result.ego_lon_type == LonGamingDecisionType::kYield) {
      ego_yield_sim_results_.insert({iter->first, sim_result});
    } else if (sim_result.ego_lon_type == LonGamingDecisionType::kOvertake) {
      ego_overtake_sim_results_.insert({iter->first, sim_result});
    }

    group_name = Log2DDS::TaskPrefix(input.plan_id) + "gaming/single_agent";
    debug_info += sim_result.debug_info;
    Log2DDS::LogDataV2(group_name, debug_info);
    // DumpSimResultGraph(group_name, "obj_id" + iter->second.obj_traj_id,
    // sim_result);
    DumpConflictZoneGraph(
        group_name, "obj_id" + iter->second.obj_traj_id, Log2DDS::kHotpink,
        sim_result.sim_conflict_zone_in_ego_view, sim_result.ego_lon_type);
  }
  if (!is_single_agent_gaming_) {
    /**
     * @brief step5: 多障碍物推演解冲突
     *
     */
    // 5.1 用aggressive idm param 走multiagent，只对所有yield障碍物生成上边界

    // 5.2 解冲突
    // 用生成的upper_speed_data，重刷所有overtake的障碍物，完成解冲突（不调用multiagent）
    // 举例：obj id 1 2 3 其中 1 重刷后由超转让了
    // 调用CalcYieldTrajSpeedDataofMultiAgent输入 现有的upper_speed_data
    // 只输入obj 1 新生成的conflict zone，更新upper_speed_data
    auto obj_ids = ConflictResolution(input, idm_param, normal_yield_speed_data,
                                      group_name, debug_info, upper_speed_data);

    // 5.4 考虑所有结果，先重新生成normal speed data,在基于此生成ref speed
    // data，处理成参考轨迹 upper_speed_data在3.2刷新过了
    SpeedVector empty_upper_speed_data{};
    normal_yield_speed_data =
        multi_agent_idm_->CalcYieldTrajSpeedDataOfMultiAgent(
            ego_init_point_, *input.ego_path, empty_upper_speed_data,
            normal_idm_config_plf_, follow_idm_config_plf_, speed_limit_plf_,
            idm_param, ego_yield_sim_results_, ego_follow_sim_results_);
    DumpGamingSpeedVectorGraph(group_name, "_normal_yield", Log2DDS::kBlue,
                               normal_yield_speed_data);
    auto ego_speed_data = multi_agent_idm_->CalcTrajSpeedDataOfMultiAgent(
        ego_init_point_, *input.ego_path, upper_speed_data,
        normal_yield_speed_data, speed_limit_plf_, idm_param,
        ego_overtake_sim_results_);
    output.gaming_ego_speed_profile = std::move(ego_speed_data);

#ifdef ALTERNATIVE_GAMING_UNIT_TEST
    for (auto &result : ego_yield_sim_results_)
      SaveTestData(g_root_, result.first, result.second,
                   output.gaming_ego_speed_profile);
    for (auto &result : ego_overtake_sim_results_)
      SaveTestData(g_root_, result.first, result.second,
                   output.gaming_ego_speed_profile);
#endif
  }
  /**
   * @brief step6: 构造输出
   *
   */
  auto *objects = output.speed_gaming_result.mutable_speed_gaming_objects();
  objects->Reserve(static_cast<int>(object_cooperation_maps_.size()));
  for (const auto &[obj_id, coop_info] : object_cooperation_maps_) {
    auto *obj = objects->Add();
    obj->set_obj_id(obj_id);
    *obj->mutable_cooperation_info() = coop_info;
    // oss << "coop_info at end of the frame!!!! id: " << obj_id
    //     << coop_info.DebugString() << "\n";
  }
  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      gaming_processed_st_objects;
  std::unordered_map<std::string, RiskObjInfo> gaming_risk_field_zoneinfos;
  std::unordered_map<std::string, StBoundaryProto::DecisionType>
      gaming_decision;
  speed_obs_processor_->PostPrecess(
      input.traj_mgr, input.processed_st_objects, ignore_objs,
      ego_follow_sim_results_, ego_yield_sim_results_,
      ego_overtake_sim_results_, &gaming_processed_st_objects, &gaming_decision,
      &gaming_risk_field_zoneinfos);
  output.gaming_decision = std::move(gaming_decision);
  output.gaming_processed_st_objects = std::move(gaming_processed_st_objects);
  output.gaming_risk_field_zoneinfos = std::move(gaming_risk_field_zoneinfos);

  /**
   * @brief step7: 输出debugframe
   *
   */
  group_name = Log2DDS::TaskPrefix(input.plan_id) + "gaming/output";
  debug_info.clear();
  // Log2DDS::LogDataV2(group_name, debug_info);
  // DumpGamingSpeedVectorGraph(group_name, "", Log2DDS::kRed,
  // output.gaming_ego_speed_profile);

  return output;
}

void SpeedGamingDecider::RunCrossGaming(
    const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
    const SpacetimeTrajectoryManager &speed_traj_manager,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        &processed_st_objects,
    const GamingConflictZoneInfo &conflict_zone_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedVector &upper_speed_data,
    const SpeedVector &normal_yield_speed_data,
    GamingSimResult *gaming_sim_result) {
  RunGaming(av_path, ego_init_point, speed_traj_manager, processed_st_objects,
            conflict_zone_info, speed_limit_plf, upper_speed_data,
            normal_yield_speed_data, AidmScene::kCross, gaming_sim_result);
}

void SpeedGamingDecider::RunAgentMergeEgoGaming(
    const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
    const SpacetimeTrajectoryManager &speed_traj_manager,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        &processed_st_objects,
    const GamingConflictZoneInfo &conflict_zone_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedVector &upper_speed_data,
    const SpeedVector &normal_yield_speed_data,
    GamingSimResult *gaming_sim_result) {
  RunGaming(av_path, ego_init_point, speed_traj_manager, processed_st_objects,
            conflict_zone_info, speed_limit_plf, upper_speed_data,
            normal_yield_speed_data, AidmScene::kMergeIn, gaming_sim_result);
}

void SpeedGamingDecider::RunEgoMergeAgentGaming(
    const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
    const SpacetimeTrajectoryManager &speed_traj_manager,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        &processed_st_objects,
    const GamingConflictZoneInfo &conflict_zone_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedVector &upper_speed_data,
    const SpeedVector &normal_yield_speed_data,
    GamingSimResult *gaming_sim_result) {
  RunGaming(av_path, ego_init_point, speed_traj_manager, processed_st_objects,
            conflict_zone_info, speed_limit_plf, upper_speed_data,
            normal_yield_speed_data, AidmScene::kMergeIn, gaming_sim_result);
}

void SpeedGamingDecider::RunGaming(
    const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
    const SpacetimeTrajectoryManager &speed_traj_manager,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        &processed_st_objects,
    const GamingConflictZoneInfo &conflict_zone_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedVector &upper_speed_data,
    const SpeedVector &normal_yield_speed_data, AidmScene aidm_scene,
    GamingSimResult *gaming_sim_result) {
  std::string cur_debug_info;
  gaming_sim_result->debug_info.clear();
  const PiecewiseLinearFunction<double> ego_init_accel_plf({0.0, 4.0, 6.0},
                                                           {0.1, 0.1, 0.0});
  auto processed_traj =
      processed_st_objects.find(conflict_zone_info.obj_traj_id);
  //优先从processed_st_objects中取轨迹，再从原始traj_mgr中取
  const auto *agent_pred_traj =
      processed_traj != processed_st_objects.end()
          ? &(processed_traj->second)
          : speed_traj_manager.FindTrajectoryById(
                std::string_view(conflict_zone_info.obj_traj_id));
  auto gaming_processed_traj =
      SpeedGamingCommon::genThetaValidTraj(*agent_pred_traj);

  GamingSimResult ego_pass_result{.planner_object =
                                      &(agent_pred_traj->planner_object())};
  GamingSimResult ego_yield_result{.planner_object =
                                       &(agent_pred_traj->planner_object())};
  ego_yield_result.interaction_type = conflict_zone_info.interaction_type;
  ego_pass_result.interaction_type = conflict_zone_info.interaction_type;
  ego_pass_result.ego_lon_type = LonGamingDecisionType::kOvertake;
  ego_yield_result.ego_lon_type = LonGamingDecisionType::kYield;
  /**
   * @brief step1: 生成自车和他车初始的speed profile
   *
   */
  ego_yield_result.ego_speed_data =
      SpeedGamingCommon::ConstructConstAccelSpeedProfile(
          ego_init_point, ego_init_accel_plf(ego_init_point.v()), kSpeedLimit,
          speed_limit_plf, speed_gaming_params_.time_step,
          /*time_horizon=*/15.0);
  ego_pass_result.ego_speed_data = ego_yield_result.ego_speed_data;

  SpeedVector obj_yield_speed_data =
      SpeedGamingCommon::ConvertPredTrajToSpeedData(
          &gaming_processed_traj, speed_gaming_params_.time_step,
          speed_gaming_params_.planning_horizon);
  // 修改obj_speed_data让他更符合运动学
  SpeedGamingCommon::ObjSpeedDataPostProcess(
      obj_yield_speed_data, speed_gaming_params_.time_step,
      speed_gaming_params_.min_jerk, speed_gaming_params_.max_jerk);
  ego_yield_result.obj_speed_data = obj_yield_speed_data;
  ego_pass_result.obj_speed_data = obj_yield_speed_data;

  /**
   * @brief step2: 设置AlternativeSimulator输入信息
   *
   */
  AlternativeSimulatorInfo alter_simulator_info;
  alter_simulator_info.av_path = av_path;
  alter_simulator_info.agent_path = SpeedGamingCommon::GeneratePathBasedTraj(
      &gaming_processed_traj, kObstaclePathMaxLength);
  alter_simulator_info.upper_speed_data = upper_speed_data;
  alter_simulator_info.normal_yield_speed_data = normal_yield_speed_data;
  // 设置冲突区信息
  GamingConflictZoneInfo risk_field_conflict_zone_info = conflict_zone_info;
  if (!SpeedGamingCommon::UpdateConflictZoneInfoByEgoSpeedProfile(
          ego_pass_result.ego_speed_data, &risk_field_conflict_zone_info)) {
    cur_debug_info.append("UpdateConflictZoneInfoByEgoSpeedProfile failed\n");
    gaming_sim_result->debug_info.append(cur_debug_info);
    return;
  }
  alter_simulator_info.riskfield_conflict_zone_in_ego_view =
      risk_field_conflict_zone_info.conflict_zone_in_ego_view;
  alter_simulator_info.riskfield_conflict_zone_in_agent_view =
      risk_field_conflict_zone_info.conflict_zone_in_agent_view;
  // 设置规划起点信息
  alter_simulator_info.ego_init_point = ego_init_point;
  alter_simulator_info.agent_init_point.set_s(gaming_processed_traj.pose().s());
  alter_simulator_info.agent_init_point.set_v(gaming_processed_traj.pose().v());
  alter_simulator_info.agent_init_point.set_a(gaming_processed_traj.pose().a());

  double jerk_sum = 0.0;
  int num_points =
      std::min(10, static_cast<int>(gaming_processed_traj.states().size()) - 1);

  for (int i = 0; i < num_points; ++i) {
    double delta_acc = gaming_processed_traj.states()[i + 1].traj_point->a() -
                       gaming_processed_traj.states()[i].traj_point->a();
    double delta_time = gaming_processed_traj.states()[i + 1].traj_point->t() -
                        gaming_processed_traj.states()[i].traj_point->t();
    double jerk = delta_acc / delta_time;
    jerk_sum += jerk;
  }

  double average_jerk = (num_points > 0) ? (jerk_sum / num_points) : 0.0;
  alter_simulator_info.agent_init_point.set_j(
      std::clamp(average_jerk, -5.0, 2.0));

  // 设置交叉角信息
  alter_simulator_info.ego_cross_angle = SpeedGamingCommon::CalcEgoCrossAngle(
      av_path, &gaming_processed_traj,
      alter_simulator_info.riskfield_conflict_zone_in_ego_view.agent_cutin_time,
      alter_simulator_info.riskfield_conflict_zone_in_ego_view.ego_cutout_s);

  alter_simulator_info.agent_cross_angle =
      SpeedGamingCommon::CalcAgentCrossAngle(
          av_path, &gaming_processed_traj,
          alter_simulator_info.riskfield_conflict_zone_in_ego_view
              .agent_cutout_time,
          alter_simulator_info.riskfield_conflict_zone_in_ego_view.ego_cutin_s);

  risk_field_conflict_zone_info.conflict_zone_in_ego_view.agent_cross_angle =
      alter_simulator_info.agent_cross_angle;
  risk_field_conflict_zone_info.conflict_zone_in_ego_view.ego_cross_angle =
      alter_simulator_info.ego_cross_angle;
  cur_debug_info.append(absl::StrCat(
      "ego_cross_angle: ", alter_simulator_info.ego_cross_angle,
      " agent_cross_angle: ", alter_simulator_info.ego_cross_angle, "\n\n"));

  // 设置限速信息
  alter_simulator_info.ego_speed_limit_plf = speed_limit_plf;
  alter_simulator_info.agent_speed_limit_plf =
      SpeedGamingCommon::GetObjCurveSpeedLimit(&gaming_processed_traj);

  /**
   * @brief step3: 设置AidmConfig
   *
   */
  // 计算配合度
  // 计算配合度--用上一帧的轨迹估算他车减速度TO DO，暂设为0.5
  // GamingConflictZoneInfo risk_field_conflict_zone_info_for_cooperation_degree
  // =
  //     risk_field_conflict_zones.find(st_boundary->id())->second;
  // std::string cooperation_grade_debug_msg;
  // const double cooperation_grade =
  // SpeedGamingCommon::CalculateCooperationGrade(
  //     his_maintence_ptr, *st_boundary->object_ptr(),
  //     last_ego_traj_speed_data,
  //     risk_field_conflict_zone_info_for_cooperation_degree,
  //     cooperation_grade_debug_msg);
  double cooperation_grade = 0.5;  // 默认值
  std::ostringstream oss;
  double agent_cutin_time =
      alter_simulator_info.riskfield_conflict_zone_in_ego_view.agent_cutin_time;
  auto is_left_turn_oncoming = [av_path, &gaming_processed_traj,
                                agent_cutin_time, &oss, this]() -> bool {
    prediction::PredictedTrajectoryPoint cutin_traj_point;
    if (!SpeedGamingCommon::GetPredictedPointByTimeFromTrajectory(
            &gaming_processed_traj, agent_cutin_time, &cutin_traj_point)) {
      return false;
    }
    PathPoint av_point = av_path.front();
    auto angle = NormalizeAngle(av_point.theta() - cutin_traj_point.theta());
    bool is_oncoming = (angle > 0.75 * M_PI || angle < -0.75 * M_PI);
    oss << "is_oncoming:" << is_oncoming << "  heading diff:" << angle << "\n";
    if (intersection_turn_type_ == IntersectionTurnType::Left && is_oncoming) {
      return true;
    } else {
      return false;
    }
  };

  if (is_left_turn_oncoming()) {
    auto it = object_cooperation_maps_.find(conflict_zone_info.obj_traj_id);
    if (it != object_cooperation_maps_.end()) {
      cooperation_grade = it->second.pass_cooperate_grade();
      oss << "Cooperation trigger!! obs id: " << conflict_zone_info.obj_traj_id
          << " it's cooperation grade is: " << cooperation_grade << "\n";
    }
  } else {
    oss << " not left concoming, Cooperation counting not been trigger."
        << "\n";
  }

  cur_debug_info += oss.str();
  const int aidm_iter_time = 10;
  const double update_coeff = 0.4;
  const double ego_yield_coeff = 0.7;
  IdmSimulateState simulate_state;
  AidmConfig aid_config(cooperation_grade, update_coeff, ego_yield_coeff,
                        speed_gaming_params_.time_step, aidm_iter_time);

  /**
   * @brief step4: 进行Simulation
   *
   */

  auto start_time = std::chrono::high_resolution_clock::now();
  std::shared_ptr<AlternativeIDMSimulator> AIDM_Simu_ptr =
      aidm_scene == AidmScene::kCross
          ? std::make_shared<AlternativeIDMSimulator>(AidmScene::kCross,
                                                      &speed_gaming_params_)
          : std::make_shared<AlternativeIDMSimulator>(AidmScene::kMergeIn,
                                                      &speed_gaming_params_);
  AIDM_Simu_ptr->SetConfig(aid_config);

  auto RunYield = [&AIDM_Simu_ptr, &ego_yield_result, &cur_debug_info,
                   &alter_simulator_info, av_path, agent_pred_traj,
                   &simulate_state,
                   this]() {  // step4-1: 进行yield simulation
    AIDM_Simu_ptr->RunEgoYieldSimulation(alter_simulator_info,
                                         agent_pred_traj->planner_object(),
                                         &ego_yield_result, &simulate_state);
    ego_yield_result.ego_traj =
        SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
            av_path, ego_yield_result.ego_speed_data);
    ego_yield_result.obj_traj =
        SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
            alter_simulator_info.agent_path, ego_yield_result.obj_speed_data);

    cur_debug_info.append("Simulation->ego_yield\n");
    cur_debug_info.append(absl::StrCat(
        "ego_in_t: ",
        ego_yield_result.sim_conflict_zone_in_ego_view.ego_cutin_time, "\n"));
    cur_debug_info.append(absl::StrCat(
        "ego_out_t: ",
        ego_yield_result.sim_conflict_zone_in_ego_view.ego_cutout_time, "\n"));
    cur_debug_info.append(absl::StrCat(
        "agent_in_t: ",
        ego_yield_result.sim_conflict_zone_in_ego_view.agent_cutin_time, "\n"));
    cur_debug_info.append(absl::StrCat(
        "agent_out_t: ",
        ego_yield_result.sim_conflict_zone_in_ego_view.agent_cutout_time,
        "\n\n"));
    cur_debug_info.append(ConvertEgoData2Log(ego_yield_result));
    cur_debug_info.append("\n");
    cur_debug_info.append(ConvertObjData2Log(ego_yield_result));
    cur_debug_info.append("\n");
  };

  auto RunPass = [&AIDM_Simu_ptr, &ego_pass_result, &cur_debug_info,
                  &alter_simulator_info, av_path, agent_pred_traj,
                  &simulate_state, this]() {
    // step4-2: 先进行pass simulation
    AIDM_Simu_ptr->RunEgoPassSimulation(alter_simulator_info,
                                        agent_pred_traj->planner_object(),
                                        &ego_pass_result, &simulate_state);
    ego_pass_result.ego_traj =
        SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
            av_path, ego_pass_result.ego_speed_data);
    auto &obj_path = alter_simulator_info.agent_path;
    ego_pass_result.obj_traj =
        SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
            obj_path, ego_pass_result.obj_speed_data);

    cur_debug_info.append("Simulation->ego_pass\n");
    cur_debug_info.append(absl::StrCat(
        "ego_in_t: ",
        ego_pass_result.sim_conflict_zone_in_ego_view.ego_cutin_time, "\n"));
    cur_debug_info.append(absl::StrCat(
        "ego_out_t: ",
        ego_pass_result.sim_conflict_zone_in_ego_view.ego_cutout_time, "\n"));
    cur_debug_info.append(absl::StrCat(
        "agent_in_t: ",
        ego_pass_result.sim_conflict_zone_in_ego_view.agent_cutin_time, "\n"));
    cur_debug_info.append(absl::StrCat(
        "agent_out_t: ",
        ego_pass_result.sim_conflict_zone_in_ego_view.agent_cutout_time,
        "\n\n"));
    cur_debug_info.append(ConvertEgoData2Log(ego_pass_result));
    cur_debug_info.append("\n");
    cur_debug_info.append(ConvertObjData2Log(ego_pass_result));
    cur_debug_info.append("\n");
  };
  // cross pre-gaming 只推演让行
  if (conflict_zone_info.is_pre_gaming && aidm_scene == AidmScene::kCross) {
    RunYield();
    *gaming_sim_result = ego_yield_result;
    cur_debug_info.append(
        "the case of is_pre_gaming_yeild and kCross, only cal yield\n");
    gaming_sim_result->debug_info.append(cur_debug_info);
    return;
  } else {
    RunYield();
    RunPass();
  }
  const auto iteration_end_time = std::chrono::high_resolution_clock::now();

  // calc yield/pass cooperation grad.
  auto [yield_coop, pass_coop] = calcObsCooperationGrad(
      &ego_yield_result, &ego_pass_result, conflict_zone_info.obj_traj_id,
      &gaming_processed_traj, object_cooperation_maps_, &cur_debug_info,
      &simulate_state);
  object_cooperation_maps_[conflict_zone_info.obj_traj_id]
      .set_yield_cooperate_grade(yield_coop);
  object_cooperation_maps_[conflict_zone_info.obj_traj_id]
      .set_pass_cooperate_grade(pass_coop);
  // std::ostringstream oss;
  // for (const auto &[obj_id, coop_info] : object_cooperation_maps_) {
  //   oss << "cooperation_map_obj_id: " << obj_id << "\n"
  //       << coop_info.DebugString() << "\n";
  // }
  // cur_debug_info += oss.str();
  // cur_debug_info.append(
  //     absl::StrCat("cooperation grade->yield_coop: ", yield_coop, "\n",
  //                  "cooperation grade->pass_coop: ", pass_coop, "\n"));
  /**
   * @brief step5: Evaluation
   *
   */
  const double ego_width = vehicle_geo_params_->width();
  const double ego_length = vehicle_geo_params_->length();
  const double ego_offset = vehicle_geo_params_->length() / 2.0 -
                            vehicle_geo_params_->back_edge_to_center();
  const auto &gaming_object = agent_pred_traj->planner_object();
  const double obj_width = gaming_object.bounding_box().width();
  const double obj_length = gaming_object.bounding_box().length();
  const double obj_offset = 0.0;
  const bool is_large_vehicle = gaming_object.is_large_vehicle();

  // TODO: Add evaluator here
  SpeedGamingEvaluator evaluator_ego_pass(&speed_gaming_params_);
  SpeedGamingEvaluator evaluator_ego_yield(&speed_gaming_params_);
  evaluator_ego_pass.Init(obj_length, obj_width, obj_offset, ego_length,
                          ego_width, ego_offset,
                          alter_simulator_info.ego_speed_limit_plf,
                          alter_simulator_info.agent_speed_limit_plf,
                          gaming_object.type(), ego_pass_result);
  evaluator_ego_yield.Init(obj_length, obj_width, obj_offset, ego_length,
                           ego_width, ego_offset,
                           alter_simulator_info.ego_speed_limit_plf,
                           alter_simulator_info.agent_speed_limit_plf,
                           gaming_object.type(), ego_yield_result);
  double cost_ego_pass = evaluator_ego_pass.Evaluate();
  double cost_ego_yield = evaluator_ego_yield.Evaluate();

  const auto end_time = std::chrono::high_resolution_clock::now();
  const double total_duration =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  const double iteration_duration =
      std::chrono::duration<double, std::milli>(iteration_end_time - start_time)
          .count();
  const double evaluation_duration =
      std::chrono::duration<double, std::milli>(end_time - iteration_end_time)
          .count();

  // TODO: 根据evaluator选取获胜的输出
  if (cost_ego_pass < cost_ego_yield) {
    *gaming_sim_result = ego_pass_result;
  } else {
    *gaming_sim_result = ego_yield_result;
  }

  cur_debug_info.append(
      absl::StrCat("Evaluation->ego_pass: total cost ", cost_ego_pass, "\n",
                   evaluator_ego_pass.GetDebugMessage(), "\n",
                   "Evaluation->ego_yield: total cost", cost_ego_yield, "\n",
                   evaluator_ego_yield.GetDebugMessage(), "\n"));
  cur_debug_info.append(
      absl::StrCat("simulator taken: ", iteration_duration, "ms\n"));
  cur_debug_info.append(
      absl::StrCat("Evaluator taken: ", evaluation_duration, "ms\n"));
  gaming_sim_result->debug_info.append(cur_debug_info);
}

std::unordered_set<std::string> SpeedGamingDecider::RunSimpleSpeedGamingDecider(
    const std::vector<const StBoundary *> &st_boundarys,
    const std::unordered_map<std::string, GamingConflictZoneInfo>
        &risk_field_conflict_zones,
    const std::optional<double> &traffic_light_stop_fence_dist,
    const SpeedVector &ego_upper_speed_data) const {
  std::unordered_set<std::string> simple_gaming_failed_ids;
  for (const auto &st : st_boundarys) {
    if (st->min_t() < kMinimentStartTimeThresh) {
      simple_gaming_failed_ids.insert(st->id());
      continue;
    }
    auto iter = risk_field_conflict_zones.find(st->id());
    if (traffic_light_stop_fence_dist.has_value() &&
        iter != risk_field_conflict_zones.end()) {
      double ego_cutin_dist =
          iter->second.conflict_zone_in_ego_view.ego_cutin_s;
      if (ego_cutin_dist > *traffic_light_stop_fence_dist) {
        simple_gaming_failed_ids.insert(st->id());
        continue;
      }
    }
    if (iter != risk_field_conflict_zones.end()) {
      double ego_cutin_dist =
          iter->second.conflict_zone_in_ego_view.ego_cutin_s;
      double agent_cutout_time =
          iter->second.conflict_zone_in_ego_view.agent_cutout_time;
      auto ego_drive_max_dist_when_agent_cutin =
          ego_upper_speed_data.EvaluateByTime(agent_cutout_time);
      if (ego_drive_max_dist_when_agent_cutin.has_value() &&
          ego_drive_max_dist_when_agent_cutin.value().s() <
              ego_cutin_dist - kSimpleGamingD0) {
        simple_gaming_failed_ids.insert(st->id());
        continue;
      }
    }
  }
  return simple_gaming_failed_ids;
}

std::optional<std::vector<std::string>> SpeedGamingDecider::ConflictResolution(
    const SpeedGamingInput &input, const SpeedIdmUpdateParams &idm_param,
    const SpeedVector &normal_yield_speed_data, std::string &group_name,
    std::string &debug_info, SpeedVector &upper_speed_data) {
  upper_speed_data = multi_agent_idm_->CalcYieldTrajSpeedDataOfMultiAgent(
      ego_init_point_, *input.ego_path, follow_upper_speed_data_,
      aggressive_idm_config_plf_, follow_idm_config_plf_, speed_limit_plf_,
      idm_param, ego_yield_sim_results_);

  bool has_overtake_to_yield = true;
  int index = 0;
  std::vector<std::string> convert_obj_id;
  convert_obj_id.reserve(ego_overtake_sim_results_.size());
  while (has_overtake_to_yield) {
    index++;
    has_overtake_to_yield = false;
    std::vector<std::string> overtake_remove_ids;
    overtake_remove_ids.reserve(ego_overtake_sim_results_.size());
    for (auto it = ego_overtake_sim_results_.begin();
         it != ego_overtake_sim_results_.end(); it++) {
      GamingSimResult sim_result{};
      sim_result.interaction_type = it->second.interaction_type;
      auto origin_conflict_zone_iter = conflict_zone_infos_.find(it->first);
      if (origin_conflict_zone_iter != conflict_zone_infos_.end()) {
        switch (it->second.interaction_type)  //只处理cross
        {
          case InteractionType::kCross:
            RunCrossGaming(*input.ego_path, ego_init_point_, *input.traj_mgr,
                           *input.processed_st_objects,
                           origin_conflict_zone_iter->second, speed_limit_plf_,
                           upper_speed_data, normal_yield_speed_data,
                           &sim_result);
            break;
          case InteractionType::kStraightMerge:
            RunAgentMergeEgoGaming(*input.ego_path, ego_init_point_,
                                   *input.traj_mgr, *input.processed_st_objects,
                                   origin_conflict_zone_iter->second,
                                   speed_limit_plf_, upper_speed_data,
                                   normal_yield_speed_data, &sim_result);
            break;
          case InteractionType::kTurnMerge:
            RunEgoMergeAgentGaming(*input.ego_path, ego_init_point_,
                                   *input.traj_mgr, *input.processed_st_objects,
                                   origin_conflict_zone_iter->second,
                                   speed_limit_plf_, upper_speed_data,
                                   normal_yield_speed_data, &sim_result);
            break;
          default:
            break;
        }
      }
      if (sim_result.ego_lon_type == LonGamingDecisionType::kYield) {
        has_overtake_to_yield = true;
        ego_yield_sim_results_.insert({it->first, sim_result});
        overtake_remove_ids.push_back(it->first);

        group_name = Log2DDS::TaskPrefix(input.plan_id) + "gaming/multi_agent";
        debug_info.clear();
        debug_info += absl::StrCat(
            "********* gaming/multi_agent/index-", index,
            "/obj_id: " + origin_conflict_zone_iter->second.obj_traj_id,
            "***********\n");
        debug_info += sim_result.debug_info;
        Log2DDS::LogDataV2(group_name, debug_info);
        // DumpSimResultGraph(group_name, "index-" + absl::StrCat(index) +
        // "/obj_id-" +
        //     origin_conflict_zone_iter->second.obj_traj_id, sim_result);
        DumpConflictZoneGraph(group_name,
                              "index-" + absl::StrCat(index) + "/obj_id-" +
                                  origin_conflict_zone_iter->second.obj_traj_id,
                              Log2DDS::kBlack,
                              sim_result.sim_conflict_zone_in_ego_view,
                              sim_result.ego_lon_type);
      } else if (sim_result.ego_lon_type == LonGamingDecisionType::kOvertake &&
                 ego_overtake_sim_results_.count(it->first)) {
        ego_overtake_sim_results_[it->first] = sim_result;
      }
    }
    for (auto &id : overtake_remove_ids) {
      // output.gaming_processed_st_objects.emplace(id, )//TODO overtake to
      // yield emplace
      ego_overtake_sim_results_.erase(id);
    }
    if (has_overtake_to_yield) {
      upper_speed_data = multi_agent_idm_->CalcYieldTrajSpeedDataOfMultiAgent(
          ego_init_point_, *input.ego_path, upper_speed_data,
          aggressive_idm_config_plf_, follow_idm_config_plf_, speed_limit_plf_,
          idm_param, ego_yield_sim_results_);
      std::move(overtake_remove_ids.begin(), overtake_remove_ids.end(),
                std::back_inserter(convert_obj_id));
    }
  }
  group_name = Log2DDS::TaskPrefix(input.plan_id) + "gaming/multi_agent";
  DumpGamingSpeedVectorGraph(group_name, "_yield_upper", Log2DDS::kBlue,
                             upper_speed_data);
  return convert_obj_id;
}

absl::StatusOr<SpeedGamingOutput> SpeedGamingDecider::ConflictResolutionAfterDP(
    const SpeedGamingInput &input,
    std::unordered_map<std::string, StBoundaryProto::DecisionType>
        &single_gaming_decision) {
  TIMELINE("SpeedGamingDecider::ConflictResolutionAfterDP");
  std::string name =
      Log2DDS::TaskPrefix(input.plan_id) + std::string(__FUNCTION__);
  SCOPED_TRACE(name.c_str());
  auto new_gaming_decision = single_gaming_decision;
  SpeedGamingOutput conflict_resolution_output;
  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      new_gaming_processed_st_objects;
  std::unordered_map<std::string, RiskObjInfo> new_gaming_risk_field_zoneinfos;
  const SpeedIdmUpdateParams idm_param;
  SpeedVector normal_yield_speed_data;
  SpeedVector upper_speed_data;
  std::string debug_info;
  std::string group_name;

  auto &st_boundaries_with_decision = *input.st_boundaries_with_decision;
  //从ST更新yield result和overtake result，再调用解冲突代码
  for (auto &st_boundarie_decision : st_boundaries_with_decision) {
    if (st_boundarie_decision.decision_type() == StBoundaryProto::YIELD &&
        !st_boundarie_decision.raw_st_boundary()->is_stationary()) {
      if (!st_boundarie_decision.raw_st_boundary()->traj_id().has_value()) {
        continue;
      }
      const auto st_boundary = st_boundarie_decision.st_boundary();
      size_t pos = st_boundary->traj_id()->find("|");
      std::string traj_id = pos != std::string::npos
                                ? st_boundary->traj_id()->substr(0, pos)
                                : st_boundary->traj_id().value();
      if (!ego_yield_sim_results_.count(traj_id) &&
          !ego_overtake_sim_results_.count(traj_id) &&
          !ego_follow_sim_results_.count(traj_id) &&
          !st_boundary->overlap_infos().empty()) {
        GamingSimResult yield_sim_result{};
        speed_obs_processor_->ConvertStBoundaryToSimResult(
            *st_boundary, *input.processed_st_objects, *input.traj_mgr,
            speed_gaming_params_, yield_sim_result);
        ego_yield_sim_results_.emplace(traj_id, yield_sim_result);
      }
    } else if (st_boundarie_decision.decision_type() ==
                   StBoundaryProto::OVERTAKE &&
               !st_boundarie_decision.raw_st_boundary()->is_stationary()) {
      if (!st_boundarie_decision.raw_st_boundary()->traj_id().has_value()) {
        continue;
      }
      const auto st_boundary = st_boundarie_decision.st_boundary();
      size_t pos = st_boundary->traj_id()->find("|");
      std::string traj_id = pos != std::string::npos
                                ? st_boundary->traj_id()->substr(0, pos)
                                : st_boundary->traj_id().value();
      if (ego_yield_sim_results_.count(traj_id)) {
        auto sim_result = ego_yield_sim_results_.find(traj_id);
        ego_overtake_sim_results_.insert({traj_id, sim_result->second});
        ego_yield_sim_results_.erase(traj_id);
        new_gaming_decision[traj_id] = StBoundaryProto::OVERTAKE;
      }
    }
  }

  auto obj_ids = ConflictResolution(input, idm_param, normal_yield_speed_data,
                                    group_name, debug_info, upper_speed_data);
  if (obj_ids.has_value()) {
    for (auto id : obj_ids.value()) {
      auto sim_result = ego_yield_sim_results_.find(id);
      if (sim_result != ego_yield_sim_results_.end()) {
        if (sim_result->second.sim_conflict_zone_in_ego_view.is_path_conflict) {
          auto st_traj =
              speed_obs_processor_->GenerateSpacetimeObjectTrajectory(
                  input.processed_st_objects, input.traj_mgr,
                  sim_result->second, sim_result->first);
          new_gaming_processed_st_objects.emplace(sim_result->first, st_traj);
        }
      } else if (sim_result->second.sim_conflict_zone_in_ego_view
                     .is_risk_field_conflict) {
        RiskObjInfo risk_obj_info;
        auto st_traj = input.traj_mgr->FindTrajectoryById(sim_result->first);
        risk_obj_info.risk_field_zoneinfo =
            std::move(sim_result->second.sim_conflict_zone_in_ego_view);
        risk_obj_info.planner_obj = &(st_traj->planner_object());
        new_gaming_risk_field_zoneinfos.emplace(sim_result->first,
                                                risk_obj_info);
      }
      auto obj_dec = new_gaming_decision.find(id);
      if (obj_dec != new_gaming_decision.end()) {
        obj_dec->second = StBoundaryProto::YIELD;
      }
    }
  }
  SpeedVector empty_upper_speed_data{};
  normal_yield_speed_data =
      multi_agent_idm_->CalcYieldTrajSpeedDataOfMultiAgent(
          ego_init_point_, *input.ego_path, empty_upper_speed_data,
          normal_idm_config_plf_, follow_idm_config_plf_, speed_limit_plf_,
          idm_param, ego_yield_sim_results_, ego_follow_sim_results_);
  DumpGamingSpeedVectorGraph(group_name, "_normal_yield", Log2DDS::kBlue,
                             normal_yield_speed_data);
  auto ego_speed_data = multi_agent_idm_->CalcTrajSpeedDataOfMultiAgent(
      ego_init_point_, *input.ego_path, upper_speed_data,
      normal_yield_speed_data, speed_limit_plf_, idm_param,
      ego_overtake_sim_results_);
  for (auto overtake_sim_result : ego_overtake_sim_results_) {
    if (overtake_sim_result.second.sim_conflict_zone_in_ego_view
            .is_path_conflict) {
      auto st_traj = speed_obs_processor_->GenerateSpacetimeObjectTrajectory(
          input.processed_st_objects, input.traj_mgr,
          overtake_sim_result.second, overtake_sim_result.first);
      new_gaming_processed_st_objects.emplace(overtake_sim_result.first,
                                              st_traj);
    } else if (overtake_sim_result.second.sim_conflict_zone_in_ego_view
                   .is_risk_field_conflict) {
      RiskObjInfo risk_obj_info;
      auto st_traj =
          input.traj_mgr->FindTrajectoryById(overtake_sim_result.first);
      risk_obj_info.risk_field_zoneinfo =
          std::move(overtake_sim_result.second.sim_conflict_zone_in_ego_view);
      risk_obj_info.planner_obj = &(st_traj->planner_object());
      new_gaming_risk_field_zoneinfos.emplace(overtake_sim_result.first,
                                              risk_obj_info);
    }
  }

  conflict_resolution_output.gaming_ego_speed_profile =
      std::move(ego_speed_data);
  conflict_resolution_output.gaming_decision = std::move(new_gaming_decision);
  conflict_resolution_output.gaming_processed_st_objects =
      std::move(new_gaming_processed_st_objects);
  conflict_resolution_output.gaming_risk_field_zoneinfos =
      std::move(new_gaming_risk_field_zoneinfos);

  group_name = Log2DDS::TaskPrefix(input.plan_id) + "gaming/output";
  DumpGamingSpeedVectorGraph(
      group_name, "", Log2DDS::kRed,
      conflict_resolution_output.gaming_ego_speed_profile);
  return conflict_resolution_output;
  // TODO: 额外补充构造解冲突仍能超和转让的ST信息
}
}  // namespace st::planning