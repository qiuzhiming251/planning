
#include <optional>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
//
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_validation.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
//
#include "plan_common/async/async_util.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/planning_macros.h"
#include "plan_common/timer.h"
#include "plan_common/trajectory_util.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/util/planner_status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"
//
#include "decider/initializer/lane_change_style_decider.h"
#include "decider/initializer/st_path_planner.h"
#include "planner/planner_manager/planner_defs.h"
#include "planner/planner_manager/planner_flags.h"
#include "planner/planner_manager/trajectory_validation.h"
#include "predictor/predicted_trajectory.h"
#include "planner/speed_optimizer/speed_finder.h"
#include "planner/speed_optimizer/speed_finder_flags.h"
#include "planner/speed_optimizer/speed_finder_input.h"
#include "planner/speed_optimizer/speed_finder_output.h"
//
#include "object_manager/partial_spacetime_object_trajectory.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_planner_object_trajectories_builder.h"
#include "object_manager/st_inference/decider_input.h"
//
#include "planner/planner_manager/min_length_path_extension.h"
#include "planner/planner_manager/planner_util.h"
#include "planner/trajectory_optimizer/ddp/trajectory_optimizer.h"
//
#include "decider/decision_manager/constraint_builder.h"
#include "decider/initializer/lane_change_safety.h"
#include "node_manager/task_runner/est_planner.h"
//
#include <fstream>
#include <cereal/archives/json.hpp>
#include "plan_common/serialization/st_path_planner_input_serialize.h"
#include "plan_common/serialization/st_path_planner_output_serialization.h"

namespace st {
namespace planning {

namespace {
void ModifySpeedFinderParamsStyle(
    const SpeedFinderParamsProto& speed_finder_lc_radical_params,
    const SpeedFinderParamsProto& speed_finder_lc_normal_params,
    const SpeedFinderParamsProto& speed_finder_lc_conservative_params,
    const TrafficGapResult& traffic_gap, LaneChangeStage lc_stage,
    SpeedResponseStyle active_speed_response_style,
    SpeedFinderParamsProto* speed_finder_params) {
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    switch (active_speed_response_style) {
      case SPEED_RESPONSE_NORMAL:
        *speed_finder_params = speed_finder_lc_normal_params;
        break;
      case SPEED_RESPONSE_RADICAL:
        *speed_finder_params = speed_finder_lc_radical_params;
        break;
      case SPEED_RESPONSE_CONSERVATIVE:
        *speed_finder_params = speed_finder_lc_conservative_params;
        break;
      case SPEED_RESPONSE_FAST:
        break;
    }
  } else if (traffic_gap.acc_gap_target_speed.has_value()) {
    *speed_finder_params = speed_finder_lc_radical_params;
  }
}

void ModifySpeedFinderParamByBehavior(
    const Behavior* behavior, double start_point_v,
    SpeedFinderParamsProto* speed_finder_params) {
  if (behavior == nullptr) return;
  CHECK_NOTNULL(speed_finder_params);

  constexpr double kLargeVehicleExtraBuffer = 0.2;  // s.

  // Tips: Temporarily place the parameters here.
  // Highway curvature speed_limit param.
  constexpr double curvature_power = 0.71;
  constexpr double curvature_numerator = 0.47;
  constexpr double curvature_bias1 = 0.007;
  constexpr double curvature_bias2 = 2.1;

  PiecewiseLinearFunction<double> headway_speed_buffer_plf(
      PiecewiseLinearFunctionFromProto(
          speed_finder_params->headway_speed_buffer_plf()));
  const double dynamic_headway = behavior->dynamic_headway();
  speed_finder_params->set_follow_time_headway(
      dynamic_headway + headway_speed_buffer_plf(Mps2Kph(start_point_v)));
  speed_finder_params->set_large_vehicle_follow_time_headway(
      dynamic_headway + headway_speed_buffer_plf(Mps2Kph(start_point_v)) +
      kLargeVehicleExtraBuffer);

  if (behavior->function_id() == Behavior_FunctionId_HW_NOA) {
    auto* speed_limit_params =
        speed_finder_params->mutable_speed_limit_params();
    speed_limit_params->set_curvature_power(curvature_power);
    speed_limit_params->set_curvature_numerator(curvature_numerator);
    speed_limit_params->set_curvature_bias1(curvature_bias1);
    speed_limit_params->set_curvature_bias2(curvature_bias2);
  }
}

std::vector<ApolloTrajectoryPointProto> ResampleOptimizerTrajectory(
    const std::vector<ApolloTrajectoryPointProto>& opt_traj) {
  std::vector<ApolloTrajectoryPointProto> resampled(kTrajectorySteps);
  for (int i = 0; i < kTrajectorySteps; ++i) {
    resampled[i] = QueryApolloTrajectoryPointByT(
        opt_traj.begin(), opt_traj.end(), i * kTrajectoryTimeStep);
  }
  return resampled;
}

std::vector<ApolloTrajectoryPointProto> StitchPreviousTrajectoryAndStTrajectory(
    int stitch_index,
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    const std::vector<ApolloTrajectoryPointProto>& st_trajectory) {
  if (time_aligned_prev_traj.empty() || stitch_index <= 0) {
    return st_trajectory;
  }
  std::vector<ApolloTrajectoryPointProto> res_traj;
  res_traj.reserve(stitch_index + st_trajectory.size());
  for (int idx = 0; idx < stitch_index; ++idx) {
    res_traj.push_back(time_aligned_prev_traj[idx]);
  }
  const double s_offset = time_aligned_prev_traj[stitch_index].path_point().s();
  for (const auto& point : st_trajectory) {
    res_traj.push_back(point);
    res_traj.back().mutable_path_point()->set_s(point.path_point().s() +
                                                s_offset);
  }
  return res_traj;
}

PlannerStatus OptimizerPostProcess(StPathPlannerInput& st_path_input,
                                   StPathPlannerOutput& st_path_out,
                                   TrajectoryOptimizerOutput opt_output) {
  const auto& psmm = *st_path_input.planner_semantic_map_manager;
  const auto& plan_start_point = st_path_input.start_point_info->start_point;
  const auto& vehicle_params = *st_path_input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();

  if (FLAGS_planner_est_scheduler_seperate_lc_pause &&
      (st_path_out.scheduler_output.lane_change_state.stage() ==
           LaneChangeStage::LCS_EXECUTING ||
       st_path_out.scheduler_output.lane_change_state.stage() ==
           LaneChangeStage::LCS_RETURN)) {
    const auto target_lane_path_ext = BackwardExtendLanePath(
        psmm,
        st_path_out.scheduler_output.drive_passage.extend_lane_path()
            .BeforeArclength(kLaneChangeCheckForwardLength),
        kLaneChangeCheckBackwardLength);
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        const auto target_frenet_frame,
        BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path_ext),
                               /*down_sample_raw_points=*/true),
        PlannerStatusProto::LC_SAFETY_CHECK_FAILED);

    constexpr double kLaneSpeedLimitPreviewTime = 6.0;  // s.
    const double speed_limit = psmm.QueryLaneSpeedLimitById(
        st_path_out.scheduler_output.drive_passage.lane_path()
            .ArclengthToLanePoint(
                st_path_input.st_path_start_point_info->start_point.v() *
                kLaneSpeedLimitPreviewTime)
            .lane_id());

    const auto resampled_traj =
        ResampleOptimizerTrajectory(opt_output.trajectory_proto);

    TrajEvalInfo eval_info{};
    LaneChangeStylePostDeciderSceneInfo target_front_obj_scene_info{};
    LaneChangeStylePostDeciderSceneInfos scene_infos;

    std::vector<std::string> leading_traj_ids;
    for (const auto& [id, obj_proto] : st_path_out.leading_trajs) {
      leading_traj_ids.emplace_back(id);
    }

    const auto prev_response_style = std::make_pair(
        st_path_input.pre_lc_style_decider_result.active_path_response_style(),
        st_path_input.pre_lc_style_decider_result
            .active_speed_response_style());

    int scene_cones_riding_line_frames_result =
        st_path_input.pre_scene_cones_riding_line_frames_result;
    auto lc_safety_status = CheckLaneChangeSafety(
        *st_path_input.planner_semantic_map_manager,
        st_path_input.scheduler_output.drive_passage,
        st_path_input.scheduler_output.sl_boundary, plan_start_point,
        resampled_traj, leading_traj_ids, *st_path_input.stalled_objects,
        target_frenet_frame, speed_limit, *st_path_input.traj_mgr,
        *st_path_input.obs_history, target_lane_path_ext, vehicle_params,
        st_path_input.lane_change_style, prev_response_style,
        st_path_input.scheduler_output.lane_change_state.stage(),
        st_path_input.prev_lane_change_stage,
        st_path_input.pre_lc_style_decider_result.congestion_scene(),
        st_path_input.path_look_ahead_duration, st_path_input.plan_id,
        &eval_info, &target_front_obj_scene_info, &scene_infos,
        &scene_cones_riding_line_frames_result, st_path_out.gaming_lc_obs_set);
    if (!lc_safety_status.ok()) {
      return PlannerStatus(
          PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
          absl::StrCat("Lane change to ",
                       st_path_out.scheduler_output.drive_passage.lane_path()
                           .front()
                           .lane_id(),
                       " not safe: ", lc_safety_status.message()));
    } else {
      st_path_out.follower_set = eval_info.follower_set;
      st_path_out.leader_set = eval_info.leader_set;
      st_path_out.follower_max_decel = eval_info.follower_max_decel;
      st_path_out.unsafe_object_ids = {eval_info.unsafe_object_id};
    }
  }

  // Path Extension
  std::vector<ApolloTrajectoryPointProto> st_trajectory =
      StitchPreviousTrajectoryAndStTrajectory(
          st_path_input.st_path_start_point_info
              ->relative_index_from_plan_start_point,
          *st_path_input.time_aligned_prev_traj, opt_output.trajectory_proto);

  constexpr double kRequiredMinPathLength = 30.0;
  const double min_path_length =
      st_trajectory.empty()
          ? kRequiredMinPathLength
          : st_trajectory.back().path_point().s() + kRequiredMinPathLength;
  // This parameter is consistent with that in trajectory curvature check.
  constexpr double kCurvatureRelaxFactor = 1.05;
  const double max_curvature = ComputeRelaxedCenterMaxCurvature(
      vehicle_geom_params, vehicle_drive_params);
  //   LOG_ERROR << "st path planner";
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto raw_path_points,
      ExtendPathAndDeleteUnreasonablePart(
          st_path_out.scheduler_output.drive_passage,
          st_path_out.scheduler_output.sl_boundary,
          *st_path_input.motion_constraint_params, vehicle_geom_params,
          st_trajectory, min_path_length,
          kCurvatureRelaxFactor * max_curvature),
      PlannerStatusProto::PATH_EXTENSION_FAILED);

  st_path_out.path = DiscretizedPath::CreateResampledPath(raw_path_points,
                                                          kPathSampleInterval);
  st_path_out.st_path_points = std::move(raw_path_points);
  return OkPlannerStatus();
}

}  // namespace

PlannerStatus RunEstPlanner(const EstPlannerInput& input,
                            SchedulerOutput scheduler_output,
                            EstPlannerOutput* est_output,
                            EstPlannerDebug* debug_info,
                            ThreadPool* thread_pool) {
  // ("EstPlanner");
  TIMELINE("RunEstPlanner");
  std::string name =
      Log2DDS::TaskPrefix(input.plan_id) + std::string(__FUNCTION__);
  SCOPED_TRACE(name.c_str());
  CHECK_NOTNULL(est_output);
  CHECK_NOTNULL(debug_info);

  absl::Cleanup fill_scheduler_output = [&est_output, &scheduler_output] {
    est_output->scheduler_output = std::move(scheduler_output);
  };
  double saved_push_offset_in_scheduler_output =
      scheduler_output.saved_offset.pre_push_offset();
  const auto& motion_constraint_params =
      *CHECK_NOTNULL(input.motion_constraint_params);
  const auto& decision_constraint_config =
      *CHECK_NOTNULL(input.decision_constraint_config);
  const auto& vehicle_params = *input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();

  double left_navi_dist =
      scheduler_output.drive_passage.lane_seq_info()->dist_to_navi_end;
  if (input.planner_semantic_map_manager->map_ptr()->is_on_highway() &&
      input.planner_semantic_map_manager->map_ptr()->type() ==
          ad_byd::planning::MapType::BEV_MAP) {
    left_navi_dist = std::max(
        left_navi_dist,
        input.planner_semantic_map_manager->map_ptr()->v2_info().dist_to_ramp);
  }
  double distance_to_junction =
      scheduler_output.drive_passage.lane_seq_info()->dist_to_junction;
  int cur_navi_lc_num = scheduler_output.drive_passage.lane_seq_info()->lc_num;
  // Build constraint manager.
  // TODO: Serialize this decision context if it need to be persist
  // across iterations.
  DeciderInput decider_input{
      .plan_id = input.plan_id,
      .vehicle_geometry_params = &vehicle_geom_params,
      .motion_constraint_params = input.motion_constraint_params,
      .config = input.decision_constraint_config,
      .planner_semantic_map_manager = input.planner_semantic_map_manager,
      .lc_state = &scheduler_output.lane_change_state,
      .plan_start_point = &input.start_point_info->start_point,
      .lane_path_before_lc = &scheduler_output.lane_path_before_lc,
      .passage = &scheduler_output.drive_passage,
      .sl_boundary = &scheduler_output.sl_boundary,
      .borrow_lane_boundary = scheduler_output.borrow_lane,
      .obj_mgr = input.obj_mgr,
      .obs_history = input.obs_history,
      .st_traj_mgr = input.st_traj_mgr,
      //   .tl_info_map = input.tl_info_map,
      .traffic_light_status_map = input.traffic_light_status_map,
      .pre_decider_state = input.decider_state,
      //   .parking_brake_release_time = input.parking_brake_release_time,
      //.teleop_enable_traffic_light_stop = input.enable_traffic_light_stopping,
      .enable_tl_ok_btn = input.enable_tl_ok_btn,
      .override_passable = input.override_passable,
      //.enable_pull_over = input.enable_pull_over,
      //.brake_to_stop = input.brake_to_stop,
      // .max_reach_length = input.left_navi_dist_v2,
      .max_reach_length = left_navi_dist,
      .lc_num = cur_navi_lc_num,
      .leading_id = input.leading_id,
      .plan_time = input.start_point_info->plan_time,
      .route_target_info = input.route_target_info,
      .scene_reasoning = input.scene_reasoning,
      // .system_break_stop = input.system_break_stop,
      // .map_func_id = input.map_func_id,
      .behavior = input.behavior,
      .speed_state = input.speed_state,
      .cur_dist_to_junction = distance_to_junction,
      .lc_lead_obj_ids = input.lc_lead_obj_ids,
      .push_dir = input.push_dir,
      .last_turn_type_v2 = input.last_turn_type_v2,
      .stalled_objects = input.stalled_objects,
      .cruising_speed_limit = input.cruising_speed_limit,
      .cur_dist_to_prev_junction = input.cur_dist_to_prev_junction,
      .eie_braking_down_flag = input.eie_braking_down_flag,
      .dist_to_tunnel_entrance = input.dist_to_tunnel_entrance,
      .dist_to_tunnel_exitance = input.dist_to_tunnel_exitance,
      .eie_choice_type = input.eie_choice_type};
  st::Timer est_planner_timer("BuildConstraints");
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decider_output, BuildConstraints(decider_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);
  st::planning::Log2DDS::LogDataV0(
      Log2DDS::TaskPrefix(input.plan_id) + "BuildConstraints",
      est_planner_timer.TimeUs() / 1e3);
  est_planner_timer.Reset("BuildSpacetimePlannerObjectTrajectories");
  est_output->distance_to_traffic_light_stop_line =
      decider_output.distance_to_traffic_light_stop_line;
  est_output->tl_stop_interface = decider_output.tl_stop_interface;
  est_output->speed_state = decider_output.speed_state;
  est_output->tl_ind_info = decider_output.tl_ind_info;

  // force no nudge when MRM
  //   const bool force_no_nudge =
  //     input.eie_braking_down_flag;
  //   input.eie_choice_type ==
  //   ad_byd::planning::EIEChoiceType::CHOICE_BRAKING_DOWN;
  SpacetimePlannerObjectTrajectoriesBuilderInput
      st_planner_object_traj_builder_input{
          .psmm = input.planner_semantic_map_manager,
          .passage = &scheduler_output.drive_passage,
          .sl_boundary = &scheduler_output.sl_boundary,
          .lane_change_state = &scheduler_output.lane_change_state,
          .veh_geom = &vehicle_geom_params,
          .plan_start_point = &input.start_point_info->start_point,
          .st_planner_start_offset =
              input.st_path_start_point_info
                  ->relative_index_from_plan_start_point *
              kTrajectoryTimeStep,
          .prev_st_trajs = input.st_planner_object_trajectories,
          .time_aligned_prev_traj = input.time_aligned_prev_traj,
          .stop_lines = decider_output.constraint_manager.StopLine(),
          .spacetime_planner_object_trajectories_params =
              input.spacetime_planner_object_trajectories_params,
          .nudge_object_info = input.nudge_object_info,
          .plan_id = input.plan_id,
          .force_no_nudge = input.eie_braking_down_flag};
  est_output->truncated_back_traj_horizon =
      input.pre_truncated_back_traj_horizon;
  auto init_st_planner_object_traj = BuildSpacetimePlannerObjectTrajectories(
      st_planner_object_traj_builder_input, input.st_traj_mgr->trajectories(),
      kSpacetimePlannerTrajectoryHorizon, kTrajectoryTimeStep,
      kDefaultHalfLaneWidth, est_output->truncated_back_traj_horizon);

  std::string dp_objects_str = "";
  for (const auto& traj_info : init_st_planner_object_traj.trajectory_infos) {
    dp_objects_str = absl::StrCat(dp_objects_str, traj_info.object_id, ",");
  }
  Log2DDS::LogDataV0(
      "st_planner_object_traj",
      absl::StrCat(Log2DDS::TaskPrefix(input.plan_id), " dp/ddp obj num ",
                   init_st_planner_object_traj.trajectory_infos.size(), " : ",
                   dp_objects_str));
  Log2DDS::LogDataV0(absl::StrCat(Log2DDS::TaskPrefix(input.plan_id),
                                  "st_planner_object_traj_size"),
                     init_st_planner_object_traj.trajectory_infos.size());

  for (const auto& traj : init_st_planner_object_traj.trajectories) {
    Log2DDS::LogDataV0(
        "st_planner_object_traj",
        absl::StrCat(Log2DDS::TaskPrefix(input.plan_id), " obj_id ",
                     std::string(traj.object_id()),
                     " is_stationary:", traj.is_stationary(),
                     " traj_time: ", traj.states().back().traj_point->t(),
                     " size:", traj.states().size()));
  }
  st::planning::Log2DDS::LogDataV0(
      Log2DDS::TaskPrefix(input.plan_id) +
          "BuildSpacetimePlannerObjectTrajectories",
      est_planner_timer.TimeUs() / 1e3);

  StPathPlannerInput st_path_input{
      .plan_id = input.plan_id,
      .st_path_start_point_info = input.st_path_start_point_info,
      .path_look_ahead_duration = input.st_path_start_point_info->plan_time -
                                  input.start_point_info->plan_time,
      .vehicle_params = input.vehicle_params,
      .planner_semantic_map_manager = input.planner_semantic_map_manager,
      .smooth_result_map = input.smooth_result_map,
      .scheduler_output = std::move(scheduler_output),
      .traj_mgr = input.st_traj_mgr,
      .pnp_infos = input.pnp_infos,
      .lane_change_style = input.lane_change_style,

      // For rebuilding constraint manager on lc pause.
      .start_point_info = input.start_point_info,
      .route_target_info = input.route_target_info,
      .obj_mgr = input.obj_mgr,
      //   .tl_info_map = input.tl_info_map,
      .traffic_light_status_map = input.traffic_light_status_map,
      .prev_decider_state = input.decider_state,
      .init_st_planner_object_traj = std::move(init_st_planner_object_traj),
      .stalled_objects = input.stalled_objects,
      .scene_reasoning = input.scene_reasoning,
      .decider_output = std::move(decider_output),
      .prev_target_lane_path_from_start =
          input.prev_target_lane_path_from_start,
      .pre_large_vehicle_avoid_state = input.pre_large_vehicle_avoid_state,
      .time_aligned_prev_traj = input.time_aligned_prev_traj,
      .prev_initializer_state = input.initializer_state,
      .trajectory_optimizer_state_proto =
          input.trajectory_optimizer_state_proto,
      // Params.
      .decision_constraint_config = &decision_constraint_config,
      .initializer_params = input.initializer_params,
      .trajectory_optimizer_params = input.trajectory_optimizer_params,
      .motion_constraint_params = &motion_constraint_params,
      .planner_functions_params = input.planner_functions_params,
      .vehicle_models_params = input.vehicle_models_params,
      .trajectory_optimizer_lc_radical_params =
          input.trajectory_optimizer_lc_radical_params,
      .trajectory_optimizer_lc_normal_params =
          input.trajectory_optimizer_lc_normal_params,
      .trajectory_optimizer_lc_conservative_params =
          input.trajectory_optimizer_lc_conservative_params,
      // .map_func_id = input.map_func_id,
      .behavior = input.behavior,
      .speed_state = input.speed_state,
      .miss_navi_scenario = input.miss_navi_scenario,
      .obs_history = input.obs_history,
      .cur_navi_lc_num = cur_navi_lc_num,
      .leading_id = input.leading_id,
      .left_navi_dist = left_navi_dist,
      .prev_lane_change_stage = input.prev_lane_change_stage,
      .lc_cmd_state = input.lc_cmd_state,
      .push_dir = input.push_dir,
      .nudge_object_info = input.nudge_object_info,
      .cur_dist_to_junction = distance_to_junction,
      .saved_offset = input.saved_offset,
      .pre_lc_style_decider_result = input.pre_lc_style_decider_result,
      .pre_task_safety_evaluation_result =
          input.pre_task_safety_evaluation_result,
      .pre_scene_cones_riding_line_frames_result =
          input.pre_scene_cones_riding_line_frames_result,
      .eie_choice_type = input.eie_choice_type};

#ifdef SERIALIZE_MACRO
  std::ofstream os("st_path_input.json");
  cereal::JSONOutputArchive archive(os);
  archive(st_path_input);
#endif

  StPathPlannerOutput path_output;
  SetDefaultResponseStyle(input.lane_change_style,
                          &path_output.lc_style_decider_result);

  Log2DDS::LogDataV2("lc_safety",
                     absl::StrFormat("prev_lc_stage:%.3f lc_style:%.3f ",
                                     st_path_input.prev_lane_change_stage,
                                     st_path_input.lane_change_style));
  auto path_status = RunStPathPlanner(st_path_input, &path_output, thread_pool);
  est_output->lc_style_decider_result = path_output.lc_style_decider_result;
  est_output->task_safety_evaluation_result =
      path_output.task_safety_evaluation_result;
  est_output->scene_cones_riding_line_frames_result =
      path_output.scene_cones_riding_line_frames_result;

  absl::StatusOr<DRFDrivelineOutput> drf_driveline_output;

  // Run driveline generator
  est_planner_timer.Reset("DrivelineGenerate");
  if (path_status.ok()) {
    constexpr double kRequiredMinPathLength = 20.0;
    // This parameter is consistent with that in trajectory curvature check.
    constexpr double kCurvatureRelaxFactor = 1.05;
    std::vector<PathPoint> raw_path_points;
    raw_path_points.reserve(path_output.traj_points.size());
    for (const auto& pt : path_output.traj_points) {
      raw_path_points.push_back(pt.path_point());
    }
    raw_path_points.begin()->set_s(0.0);
    for (int index = 1; index < raw_path_points.size(); ++index) {
      double dx = raw_path_points[index - 1].x() - raw_path_points[index].x();
      double dy = raw_path_points[index - 1].y() - raw_path_points[index].y();
      const double d = std::sqrt(dx * dx + dy * dy);
      raw_path_points[index].set_s(raw_path_points[index - 1].s() + d);
    }

    if (raw_path_points.back().s() > 0.5) {
      const auto init_path = DiscretizedPath::CreateResampledPath(
          raw_path_points, kPathSampleInterval);

      DRFDynamicDrivelineGenerator drf_driveline_generator(
          &vehicle_geom_params);
      const DRFDrivelineInput drf_driveline_input{
          .traj_mgr = st_path_input.traj_mgr,
          .init_path = &init_path,
          .obs_history = input.obs_history,
          .plan_start_point = input.st_path_start_point_info,
          .plan_id = input.plan_id,
          .drive_passage = &path_output.scheduler_output.drive_passage,
          .last_driveline_result =
              &(input.last_gaming_result->driveline_result()),
      };
      // drf_driveline_output =
      //     drf_driveline_generator.RunRiskFieldDrivelineGenerator(
      //         drf_driveline_input);
      drf_driveline_output =
          drf_driveline_generator.RunRiskFieldDrivelineGenerator(
              drf_driveline_input);
      Log2DDS::LogDataV0(
          absl::StrCat("_task_", input.plan_id, "_driveline_debug"),
          drf_driveline_generator.get_debug());
      if (drf_driveline_output.ok() &&
          drf_driveline_output.value().driveline_result.driveline_status() ==
              DrivelineStatus::GENERATE_SUCCEED) {
        path_output.traj_points =
            drf_driveline_output.value().dynamic_drive_line;
        path_output.captain_traj_points =
            drf_driveline_output.value().dynamic_drive_line;
        std::vector<Vec2d> pos;
        pos.reserve(path_output.traj_points.size());
        for (auto pt : path_output.traj_points) {
          pos.push_back(Vec2d(pt.path_point().x(), pt.path_point().y()));
        }
        Log2DDS::LogLineV0(
            absl::StrCat("_task_", input.plan_id, "_driveline_debug"),
            Log2DDS::kTiffanyBlue, {}, pos);
        Log2DDS::LogDataV0(
            absl::StrCat("_task_", input.plan_id, "_driveline_debug"), "OK");
      } else {
        auto driveline_result_status = DrivelineStatus_Name(
            drf_driveline_output.value().driveline_result.driveline_status());
        Log2DDS::LogDataV0(
            absl::StrCat("_task_", input.plan_id, "_driveline_debug"),
            driveline_result_status);
      }
    }
  }
  if (drf_driveline_output.ok()) {
    est_output->gaming_result.mutable_driveline_result()->CopyFrom(
        drf_driveline_output.value().driveline_result);
  }
  st::planning::Log2DDS::LogDataV0(
      Log2DDS::TaskPrefix(input.plan_id) + "DrivelineGenerate",
      est_planner_timer.TimeUs() / 1e3);

#ifdef SERIALIZE_MACRO
  std::ofstream os2("st_path_output.json");
  cereal::JSONOutputArchive archive2(os2);
  archive2(path_output);
#endif

  // Run optimizer.
  if (path_status.ok()) {
    TrajectoryOptimizerOutput opt_output;
    path_status = RunOptimizeTrajectry(st_path_input, &path_output, opt_output,
                                       thread_pool);
    if (path_status.ok()) {
      // Run optimizer post process
      std::string postprocess_name =
          Log2DDS::TaskPrefix(input.plan_id) + "OptimizerPostProcess";
      SCOPED_TRACE(postprocess_name.c_str());
      path_status =
          OptimizerPostProcess(st_path_input, path_output, opt_output);
    }
  }
  if (std::abs(saved_push_offset_in_scheduler_output) > 0) {
    path_output.saved_offset.set_pre_push_offset(
        saved_push_offset_in_scheduler_output);
  }
  // Fill path out to est output.
  for (const auto& stop_line : path_output.constraint_manager.StopLine()) {
    if (stop_line.source().type_case() !=
            SourceProto::TypeCase::kEndOfPathBoundary &&
        !est_output->first_stop_s.has_value()) {
      est_output->first_stop_s = stop_line.s();
    }

    if (stop_line.source().type_case() ==
            SourceProto::TypeCase::kTrafficLight &&
        !est_output->redlight_lane_id.has_value()) {
      est_output->redlight_lane_id =
          mapping::ElementId(stop_line.source().traffic_light().lane_id());
    }
  }
  est_output->unsafe_object_ids = std::move(path_output.unsafe_object_ids);
  est_output->safety_check_failed_reason =
      path_output.safety_check_failed_reason;
  est_output->is_init_return_scene =
      path_output.task_safety_evaluation_result.task_is_init_return_scene();
  // Set cross-frame state of the decider.
  est_output->decider_state = std::move(path_output.decider_state);
  // Set cross-frame state of the initializer.
  est_output->initializer_state = std::move(path_output.initializer_state);
  est_output->pre_large_vehicle_avoid_state =
      path_output.pre_large_vehicle_avoid_state;
  est_output->saved_offset = std::move(path_output.saved_offset);
  debug_info->initializer_debug_proto =
      std::move(path_output.initializer_debug_proto);
  // Optimizer state.
  est_output->trajectory_optimizer_state_proto =
      std::move(path_output.trajectory_optimizer_state_proto);
  // Optimizer Auto Tuning
  //   est_output->candidate_auto_tuning_traj_proto =
  //       std::move(path_output.candidate_auto_tuning_traj_proto);
  //   est_output->expert_auto_tuning_traj_proto =
  //       std::move(path_output.expert_auto_tuning_traj_proto);
  //  Optimizer hmi
  est_output->traffic_gap =
      std::move(path_output.constraint_manager.TrafficGap());

  NudgeObjectInfo large_veh_avoid_nudge_obj_info;
  large_veh_avoid_nudge_obj_info.id =
      est_output->pre_large_vehicle_avoid_state.cur_avoid_veh_id();
  large_veh_avoid_nudge_obj_info.direction =
      est_output->pre_large_vehicle_avoid_state.avoid_dir() ==
              LargeVehicleAvoidStateProto_AvoidDir_DIR_LEFT
          ? 1
      : est_output->pre_large_vehicle_avoid_state.avoid_dir() ==
              LargeVehicleAvoidStateProto_AvoidDir_DIR_RIGHT
          ? -1
          : 0;
  large_veh_avoid_nudge_obj_info.arc_dist_to_object =
      est_output->pre_large_vehicle_avoid_state.dist_to_cur_avoid_veh();
  large_veh_avoid_nudge_obj_info.type = OT_LARGE_VEHICLE;
  large_veh_avoid_nudge_obj_info.nudge_state =
      NudgeObjectInfo::NudgeState::NUDGE;

  if (((path_output.nudge_object_info.has_value() &&
        path_output.nudge_object_info.value().id == "") ||
       !path_output.nudge_object_info.has_value()) &&
      large_veh_avoid_nudge_obj_info.id != "") {
    est_output->nudge_object_info = std::move(large_veh_avoid_nudge_obj_info);
  } else {
    est_output->nudge_object_info = std::move(path_output.nudge_object_info);
  }

  est_output->obj_lead = std::move(path_output.obs_leading);
  est_output->lc_status_code = std::move(path_output.lc_status_code);
  est_output->is_init_follow_scene = path_output.is_init_follow_scene;
  est_output->lc_lead_obj_id = path_output.lc_lead_obj_id;
  debug_info->optimizer_debug_proto =
      std::move(path_output.optimizer_debug_proto);

  est_output->st_path_points = std::move(path_output.st_path_points);
  // Set cross-frame state of spacetime planner objects.
  for (const auto& st_planner_traj_info :
       path_output.st_planner_object_traj.trajectory_infos) {
    auto* st_planner_traj_proto =
        est_output->st_planner_object_trajectories.add_trajectory();
    st_planner_traj_proto->set_reason(st_planner_traj_info.reason);
    st_planner_traj_proto->set_id(st_planner_traj_info.object_id);
    st_planner_traj_proto->set_index(st_planner_traj_info.traj_index);
  }
  debug_info->st_planner_object_trajectories =
      est_output->st_planner_object_trajectories;
  // Export filtered object to debug message.
  debug_info->filtered_prediction_trajectories.mutable_filtered()->Reserve(
      input.st_traj_mgr->ignored_trajectories().size());
  for (const auto& ignored : input.st_traj_mgr->ignored_trajectories()) {
    auto* filtered =
        debug_info->filtered_prediction_trajectories.add_filtered();
    filtered->set_reason(ignored.reason);
    filtered->set_id(ignored.object_id);
    filtered->set_index(ignored.traj->index());
  }
  est_output->scheduler_output = std::move(path_output.scheduler_output);
  std::move(fill_scheduler_output).Cancel();
  if (!path_status.ok()) {
    FillDecisionConstraintDebugInfo(path_output.constraint_manager,
                                    &debug_info->decision_constraints);
    return path_status;
  }

  est_output->path = std::move(path_output.path);
  est_output->leading_trajs = std::move(path_output.leading_trajs);
  est_output->follower_set = std::move(path_output.follower_set);
  est_output->leader_set = std::move(path_output.leader_set);
  est_output->follower_max_decel = path_output.follower_max_decel;
  for (auto it = est_output->leading_trajs.begin();
       it != est_output->leading_trajs.end();) {
    if (it->second.reason() ==
        ConstraintProto::LeadingObjectProto::AFTER_STOPLINE) {
      it = est_output->leading_trajs.erase(it);
    } else {
      ++it;
    }
  }

  // Run speed.
  const SpeedFinderInput speed_input{
      .base_name = "est",
      // .map_func_id = input.map_func_id,
      .behavior = input.behavior,
      .psmm = input.planner_semantic_map_manager,
      .traj_mgr = input.st_traj_mgr,
      .constraint_mgr = &path_output.constraint_manager,
      .leading_trajs = &est_output->leading_trajs,
      .follower_set = &est_output->follower_set,
      .leader_set = &est_output->leader_set,
      .consider_lane_change_gap = input.consider_lane_change_gap,
      .drive_passage = &est_output->scheduler_output.drive_passage,
      .path_sl_boundary = &est_output->scheduler_output.sl_boundary,
      .stalled_objects = input.stalled_objects,
      .path = &est_output->path,
      .st_path_points = &est_output->st_path_points,
      //   .safe_invariance_speed_info = input.safe_invariance_speed_info,
      .time_aligned_prev_traj = st_path_input.time_aligned_prev_traj,
      // speed planning always starts at plan start point
      .plan_start_v = input.start_point_info->start_point.v(),
      .plan_start_a = input.start_point_info->start_point.a(),
      .plan_start_j = input.start_point_info->start_point.j(),
      .plan_time = input.start_point_info->plan_time,
      //   .planner_av_context = input.planner_av_context,
      //   .objects_proto = input.objects_proto,
      //   .planner_model_pool = input.planner_model_pool,
      .plan_id = input.plan_id,
      .lc_stage = est_output->scheduler_output.lane_change_state.stage(),
      .lane_change_state = est_output->scheduler_output.lane_change_state,
      .attention_obj_id = input.speed_state->attention_obj_id,
      .nudge_object_info = input.nudge_object_info,
      .is_open_gap = input.is_open_gap,
      .ego_history = input.ego_history,
      .spdlimit_curvature_gain_prev = input.spdlimit_curvature_gain_prev,
      .active_speed_response_style =
          path_output.lc_style_decider_result.active_speed_response_style(),
      .obj_history = input.obs_history,
      .gaming_lc_obs_set = path_output.gaming_lc_obs_set,
      .last_speed_gaming_result =
          &(input.last_gaming_result->speed_gaming_result()),
  };

  // Modify style settings for StPathPlanner.
  const auto& traffic_gap = path_output.constraint_manager.TrafficGap();
  auto speed_finder_params = *input.speed_finder_params;
  if (FLAGS_planner_enable_lc_style_params) {
    ModifySpeedFinderParamsStyle(
        *input.speed_finder_lc_radical_params,
        *input.speed_finder_lc_normal_params,
        *input.speed_finder_lc_conservative_params, traffic_gap,
        est_output->scheduler_output.lane_change_state.stage(),
        path_output.lc_style_decider_result.active_speed_response_style(),
        &speed_finder_params);
  }

  ModifySpeedFinderParamByBehavior(input.behavior,
                                   input.start_point_info->start_point.v(),
                                   &speed_finder_params);

  CHECK_NOTNULL(input.speed_finder_state);
  SpeedFinderStateProto speed_finder_state = *input.speed_finder_state;

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      FindSpeed(speed_input, vehicle_geom_params, vehicle_drive_params,
                motion_constraint_params, speed_finder_params, thread_pool,
                &est_output->curr_ego_frame, &speed_finder_state),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);

  // Fill speed out to est output.
  debug_info->speed_finder_debug = std::move(speed_output.speed_finder_proto);
  FillDecisionConstraintDebugInfo(speed_output.constraint_mgr,
                                  &debug_info->decision_constraints);
  // Move final trajectory to output.
  est_output->traj_points = std::move(speed_output.trajectory_points);
  est_output->considered_st_objects =
      std::move(speed_output.considered_st_objects);
  est_output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  est_output->gaming_result.mutable_speed_gaming_result()->CopyFrom(
        speed_output.speed_gaming_result);

  est_output->alerted_front_vehicle =
      std::move(speed_output.alerted_front_vehicle);

  est_output->st_boundaries_with_decision =
      std::move(speed_output.st_boundaries_with_decision);

  est_output->obj_sl_map = speed_output.obj_sl_map;

  est_output->speed_state.attention_obj_id = speed_output.attention_obj_id;
  est_output->spdlimit_curvature_gain = speed_output.spdlimit_curvature_gain;
  est_output->speed_finder_state = std::move(speed_finder_state);
  est_output->cipv_obj_id = speed_output.cipv_obj_id;

  const bool valid = ValidateEstTrajectory(
      *input.planner_semantic_map_manager, est_output->considered_st_objects,
      input.start_point_info->full_stop, est_output->scheduler_output,
      vehicle_geom_params, vehicle_drive_params, motion_constraint_params,
      est_output->traj_points, &debug_info->traj_validation_result,
      thread_pool);
  if (!valid) {
    return PlannerStatus(
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        absl::StrCat("Validation failed: ",
                     debug_info->traj_validation_result.DebugString()));
  }

  DestroyContainerAsyncMarkSource(std::move(path_output),
                                  "est_planner:path_output");

  return OkPlannerStatus();
}

}  // namespace planning
}  // namespace st
