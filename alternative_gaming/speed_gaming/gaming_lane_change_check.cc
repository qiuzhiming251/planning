#include "gaming_lane_change_check.h"

namespace st::planning {
constexpr double kSpeedLimit = 33.0;
constexpr double kObstaclePathMaxLength = 100.0;

absl::StatusOr<std::string> LaneChangeSafetyChecker::Execute(
    const LaneChangeSafetyGamingInput &input,
    const std::string &check_traj_id) {
  std::string debug_info;
  std::string group_name =
      Log2DDS::TaskPrefix(input.plan_id) + "gaming_lc/input";
  if (!first_trigger_) {
    debug_info.append("first trigger\n");
    first_trigger_ = true;
    speed_obs_processor_->Init(input.traj_mgr, input.ego_path,
                               input.drive_passage, vehicle_geo_params_);
    // step1 筛选让行前车计算冲突区并转换成simresult TODO
    std::unordered_map<std::string, GamingConflictZoneInfo>
        yield_conflict_zone_infos;
    speed_obs_processor_->GenerateYieldConflictZoneForLaneChangeSafety(
        yield_conflict_zone_infos);
    for (auto &yield_conflict_zone_info : yield_conflict_zone_infos) {
      GamingSimResult ego_sim_result;
      ego_sim_result.ego_lon_type = LonGamingDecisionType::kYield;
      ego_sim_result.sim_conflict_zone_in_agent_view =
          yield_conflict_zone_info.second.conflict_zone_in_agent_view;
      ego_sim_result.sim_conflict_zone_in_ego_view =
          yield_conflict_zone_info.second.conflict_zone_in_ego_view;
      ego_sim_result.interaction_type =
          yield_conflict_zone_info.second.interaction_type;
      if (yield_conflict_zone_info.second.interaction_type !=
          InteractionType::kStaticOccupy) {
        const auto &obj_traj = input.traj_mgr->FindTrajectoryById(
            std::string_view(yield_conflict_zone_info.first));
        ego_sim_result.obj_speed_data =
            SpeedGamingCommon::ConvertPredTrajToSpeedData(
                obj_traj, speed_gaming_params_.time_step,
                speed_gaming_params_.planning_horizon);
        const auto obj_path = SpeedGamingCommon::GeneratePathBasedTraj(
            obj_traj, kObstaclePathMaxLength);
        ego_sim_result.obj_traj =
            SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
                obj_path, ego_sim_result.obj_speed_data);
      }
      debug_info.append(
          absl::StrCat("front car: ", yield_conflict_zone_info.first, "\n"));
      DumpConflictZoneGraph(
          group_name, "follow_" + yield_conflict_zone_info.first,
          Log2DDS::kDarkBlue, ego_sim_result.sim_conflict_zone_in_ego_view,
          ego_sim_result.ego_lon_type);
      if (abs(yield_conflict_zone_info.second.conflict_zone_in_agent_view
                  .agent_cutin_time) < 1e-5) {
        ego_follow_sim_results_.insert(
            {yield_conflict_zone_info.second.obj_traj_id,
             std::move(ego_sim_result)});
      } else if (yield_conflict_zone_info.second.conflict_zone_in_agent_view
                     .agent_cutin_time > 0) {
        ego_yield_sim_results_.insert(
            {yield_conflict_zone_info.second.obj_traj_id,
             std::move(ego_sim_result)});
      }
    }

    // step2 设置起始点和限速信息
    ego_init_point_.set_pos(
        Vec2d(input.ego_path->front().x(), input.ego_path->front().y()));
    ego_init_point_.set_theta(input.ego_path->front().theta());
    ego_init_point_.set_s(input.ego_path->front().s());
    ego_init_point_.set_v(input.plan_start_v);
    ego_init_point_.set_a(input.plan_start_a);
    ego_init_point_.set_j(input.plan_start_j);

    std::vector<double> s_list(input.ego_path->size(), 0.0);
    std::vector<double> v_list(input.ego_path->size(), input.speed_limit);
    speed_limit_plf_ = PiecewiseLinearFunction<double>(s_list, v_list);

    // step3 生成upper speed
    SpeedIdmUpdateParams idm_param;
    multi_agent_idm_ = std::make_unique<MultiAgentIdm>(&speed_gaming_params_);
    upper_speed_data_ = multi_agent_idm_->CalcYieldTrajSpeedDataOfMultiAgent(
        ego_init_point_, *input.ego_path, upper_speed_data_,
        normal_idm_config_plf_, follow_idm_config_plf_, speed_limit_plf_,
        idm_param, ego_follow_sim_results_, ego_yield_sim_results_);
    DumpGamingSpeedVectorGraph(group_name, "_follow_upper", Log2DDS::kBlue,
                               upper_speed_data_);
    Log2DDS::LogDataV2(group_name, debug_info);
  }

  group_name = Log2DDS::TaskPrefix(input.plan_id) + "gaming_lc/single_check";
  debug_info.clear();
  // step4 对check traj 生成冲突区信息 TODO
  GamingConflictZoneInfo check_obj_conflict_zone;
  speed_obs_processor_->GenerateConflictZoneForOneAgent(
      check_obj_conflict_zone,
      input.traj_mgr->FindTrajectoryById(check_traj_id), false);
  if (!check_obj_conflict_zone.conflict_zone_in_ego_view.is_path_conflict) {
    debug_info.append(
        absl::StrCat(check_traj_id, "has no path conflict with ego path"));
    Log2DDS::LogDataV2(group_name, debug_info);
    return absl::UnavailableError("has no path conflict");
  }
  debug_info += absl::StrCat(
      check_traj_id, " ego_in_s:",
      check_obj_conflict_zone.conflict_zone_in_ego_view.ego_cutin_s, "\n",
      "ego_out_s:",
      check_obj_conflict_zone.conflict_zone_in_ego_view.ego_cutout_s, "\n",
      "agent_in_s:",
      check_obj_conflict_zone.conflict_zone_in_ego_view.agent_cutin_s, "\n",
      "agent_out_s:",
      check_obj_conflict_zone.conflict_zone_in_ego_view.agent_cutout_s, "\n");
  DumpConflictZoneGraph(
      group_name, "origin_" + check_obj_conflict_zone.obj_traj_id,
      Log2DDS::kDarkBlue, check_obj_conflict_zone.conflict_zone_in_ego_view,
      LonGamingDecisionType::kNone);
  // step5 对check traj 进行博弈推演
  GamingSimResult sim_result;
  auto check_status = LaneChangeSafetyCheck(
      *input.ego_path, ego_init_point_, *input.traj_mgr,
      check_obj_conflict_zone, speed_limit_plf_, upper_speed_data_, {},
      AidmScene::kMergeIn, &sim_result);
  debug_info += sim_result.debug_info;
  Log2DDS::LogDataV2(group_name, debug_info);

  // step5 输出检查结果
  if (check_status.ok()) {
    // 判断变道安全，可超，输出修正后的预测轨迹
    // auto st_traj =
    //     *input.traj_mgr->FindTrajectoryById(std::string_view(check_traj_id));
    // auto &modified_traj_points = sim_result.obj_traj;
    // prediction::PredictedTrajectory modified_traj = st_traj.trajectory();
    // auto points_ptr = modified_traj.mutable_points();
    // points_ptr->clear();
    // for (auto &pt : modified_traj_points) {
    //   prediction::PredictedTrajectoryPoint tpt(pt);
    //   points_ptr->emplace_back(tpt);
    // }
    // st_traj = st_traj.CreateTrajectoryMutatedInstance(modified_traj);
    // ego_overtake_sim_results_.insert({check_traj_id, std::move(sim_result)});
    // return st_traj;
    DumpConflictZoneGraph(
        group_name, "obj_id" + check_obj_conflict_zone.obj_traj_id,
        Log2DDS::kHotpink, sim_result.sim_conflict_zone_in_ego_view,
        sim_result.ego_lon_type);
    return check_traj_id;
  } else {
    return check_status;
  }
}

absl::Status LaneChangeSafetyChecker::LaneChangeSafetyCheck(
    const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
    const SpacetimeTrajectoryManager &speed_traj_manager,
    const GamingConflictZoneInfo &conflict_zone_info,
    const PiecewiseLinearFunction<double> &speed_limit_plf,
    const SpeedVector &upper_speed_data,
    const SpeedVector &normal_yield_speed_data, AidmScene aidm_scene,
    GamingSimResult *gaming_sim_result) {
  // TODO: 与rungaming 推演逻辑一致，只是增加判断：决策为超并且无碰才输出OK
  // 推演决策
  std::string cur_debug_info;
  gaming_sim_result->debug_info.clear();
  const PiecewiseLinearFunction<double> ego_init_accel_plf({0.0, 4.0, 6.0},
                                                           {0.1, 0.1, 0.0});
  const auto *agent_pred_traj = speed_traj_manager.FindTrajectoryById(
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
          agent_pred_traj, speed_gaming_params_.time_step,
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
      agent_pred_traj, kObstaclePathMaxLength);
  alter_simulator_info.upper_speed_data = upper_speed_data;
  alter_simulator_info.normal_yield_speed_data = normal_yield_speed_data;
  // 设置冲突区信息
  GamingConflictZoneInfo risk_field_conflict_zone_info = conflict_zone_info;
  if (!SpeedGamingCommon::UpdateConflictZoneInfoByEgoSpeedProfile(
          ego_pass_result.ego_speed_data, &risk_field_conflict_zone_info)) {
    cur_debug_info.append("UpdateConflictZoneInfoByEgoSpeedProfile failed\n");
    gaming_sim_result->debug_info.append(cur_debug_info);
    return absl::UnavailableError(
        "UpdateConflictZoneInfoByEgoSpeedProfile failed");
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
  const double cooperation_grade = 0.1;
  const int aidm_iter_time = 10;
  const double update_coeff = 0.4;
  const double ego_yield_coeff = 0.7;
  AidmConfig aid_config(cooperation_grade, update_coeff, ego_yield_coeff,
                        speed_gaming_params_.time_step, aidm_iter_time);

  /**
   * @brief step4: 进行Simulation
   *
   */
  // New TODO 统计时间先注释了
  //  auto start_time = std::chrono::high_resolution_clock::now();
  std::shared_ptr<AlternativeIDMSimulator> AIDM_Simu_ptr =
      aidm_scene == AidmScene::kCross
          ? std::make_shared<AlternativeIDMSimulator>(AidmScene::kCross,
                                                      &speed_gaming_params_)
          : std::make_shared<AlternativeIDMSimulator>(AidmScene::kMergeIn,
                                                      &speed_gaming_params_);
  AIDM_Simu_ptr->SetConfig(aid_config);

  // step4-1: 进行yield simulation
  AIDM_Simu_ptr->RunEgoYieldSimulation(alter_simulator_info,
                                       agent_pred_traj->planner_object(),
                                       &ego_yield_result);
  ego_yield_result.ego_traj = SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
      av_path, ego_yield_result.ego_speed_data);
  ego_yield_result.obj_traj = SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
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
  // New TODO这里应该只有merge场景了吧 这个判断还要加嘛

  // cross pre-gaming 只推演让行
  // if (conflict_zone_info.is_pre_gaming && aidm_scene == AidmScene::kCross) {
  //   *gaming_sim_result = ego_yield_result;
  //   cur_debug_info.append(
  //       "the case of is_pre_gaming and kCross, only cal yield\n");
  //   gaming_sim_result->debug_info.append(cur_debug_info);
  //   return;
  // }

  // step4-2: 先进行pass simulation
  AIDM_Simu_ptr->RunEgoPassSimulation(alter_simulator_info,
                                      agent_pred_traj->planner_object(),
                                      &ego_pass_result);
  ego_pass_result.ego_traj = SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
      av_path, ego_pass_result.ego_speed_data);
  auto &obj_path = alter_simulator_info.agent_path;
  ego_pass_result.obj_traj = SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
      obj_path, ego_pass_result.obj_speed_data);
  const auto iteration_end_time = std::chrono::high_resolution_clock::now();

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
      ego_pass_result.sim_conflict_zone_in_ego_view.agent_cutout_time, "\n\n"));
  cur_debug_info.append(ConvertEgoData2Log(ego_pass_result));
  cur_debug_info.append("\n");
  cur_debug_info.append(ConvertObjData2Log(ego_pass_result));
  cur_debug_info.append("\n");

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
  bool is_collision_result_yield = false;
  bool is_collision_result_pass = false;
  // TODO: Add evaluator here
  GamingLaneChangeEvaluator evaluator_ego_pass(&speed_gaming_params_);
  GamingLaneChangeEvaluator evaluator_ego_yield(&speed_gaming_params_);
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
  double cost_ego_pass =
      evaluator_ego_pass.LaneChangeSafetyEvaluate(is_collision_result_pass);
  double cost_ego_yield =
      evaluator_ego_yield.LaneChangeSafetyEvaluate(is_collision_result_yield);

  // New TODO 统计时间先注释了
  //  const auto end_time = std::chrono::high_resolution_clock::now();
  //  const double total_duration =
  //      std::chrono::duration<double, std::milli>(end_time -
  //      start_time).count();
  //  const double iteration_duration =
  //      std::chrono::duration<double, std::milli>(iteration_end_time -
  //      start_time)
  //          .count();
  //  const double evaluation_duration =
  //      std::chrono::duration<double, std::milli>(end_time -
  //      iteration_end_time)
  //          .count();

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
  if (cost_ego_pass > cost_ego_yield) {
    cur_debug_info.append("Ego not pass obj\n");
    gaming_sim_result->debug_info.append(cur_debug_info);
    return absl::UnavailableError("Ego not pass obj");
  }
  if (is_collision_result_pass) {
    cur_debug_info.append("is_collision_result_pass\n");
    gaming_sim_result->debug_info.append(cur_debug_info);
    return absl::UnavailableError("is_collision_result_pass");
  }
  gaming_sim_result->debug_info.append(cur_debug_info);
  return absl::OkStatus();
}

SpeedVector LaneChangeSafetyChecker::GenerateEgoSpeedProfile(
    const LaneChangeSafetyGamingInput &input) {
  const SpeedIdmUpdateParams idm_param;
  SpeedVector normal_yield_speed_data =
      multi_agent_idm_->CalcYieldTrajSpeedDataOfMultiAgent(
          ego_init_point_, *input.ego_path, {}, normal_idm_config_plf_,
          follow_idm_config_plf_, speed_limit_plf_, idm_param,
          ego_yield_sim_results_, ego_follow_sim_results_);

  return multi_agent_idm_->CalcTrajSpeedDataOfMultiAgent(
      ego_init_point_, *input.ego_path, upper_speed_data_,
      normal_yield_speed_data, speed_limit_plf_, idm_param,
      ego_overtake_sim_results_);
}
}  // namespace st::planning
