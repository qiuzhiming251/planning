

#include "node_manager/task_runner/city_planner/msg_proxy.h"

#include <functional>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/cleanup/cleanup.h"
#include "decider/scheduler/smooth_reference_line_builder.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/hmi_content.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/route_manager_output.pb.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/msg/orin_msgs/map_event.pb.h"
#include "object_manager/low_likelihood_filter.h"
#include "object_manager/motion_state_filter.h"
#include "object_manager/lane_attr_type_filter.h"
#include "object_manager/planner_object_manager_builder.h"
#include "object_manager/predicted_motion_filter.h"
#include "object_manager/reflected_object_in_proximity_filter.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections.h"
#include "plan_common/maps/route_sections_info.h"
#include "plan_common/planning_macros.h"
#include "plan_common/timer.h"
#include "plan_common/util/speed_util.h"
#include "planner/planner_manager/planner_flags.h"
#include "router/route_util.h"
#include "node_manager/task_runner/planner_main_loop_internal.h"

namespace st {
namespace planning {

std::unique_ptr<MultiTasksCruisePlannerInput> AdaptPlannerInput(
    const PlanningInputFrame* input_frame,
    const PlannerParamsProto& planner_params,
    const VehicleParamsProto& vehicle_params, absl::Time predicted_plan_time,
    ThreadPool* thread_pool, PlannerState* planner_state) {
  SCOPED_TRACE(__FUNCTION__);
  // rm_output_ = std::make_unique<RouteManagerOutput>();
  auto status_or_input =
      ComputeMembers(input_frame, planner_params, vehicle_params,
                     predicted_plan_time, thread_pool, planner_state);
  if (status_or_input.index() == 0) {
    // TODO: error should be reported
    // planner_state->planner_status_code = planner_status.status_code();
    LOG_ERROR << "ComputeMembers failed: "
              << std::get<PlannerStatus>(status_or_input).ToString();
    Log2DDS::LogDataV0("ComputeMembersStatus",
                       std::get<PlannerStatus>(status_or_input).ToString());
    return nullptr;
  }
  return std::move(std::get<1>(status_or_input));
}

namespace {
std::variant<PlannerStatus, std::unique_ptr<MultiTasksCruisePlannerInput>>
ComputeMembers(const PlanningInputFrame* input_frame,
               const PlannerParamsProto& planner_params,
               const VehicleParamsProto& vehicle_params,
               absl::Time predicted_plan_time, ThreadPool* thread_pool,
               PlannerState* planner_state) {
  DLOG(INFO) << __FUNCTION__;
  Timer timer(__FUNCTION__);
  st::Timer msg_proxy_timer("AdaptMap");
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();

  // 0. update map
  planner_state->planner_semantic_map_manager =
      std::make_shared<PlannerSemanticMapManager>(input_frame->map_ptr,
                                                  input_frame->lite_map_ptr);

  bool is_map_valid =
      planner_state->planner_semantic_map_manager->map_ptr() != nullptr &&
      planner_state->planner_semantic_map_manager->map_ptr()->IsValid();
  Log2DDS::LogDataV0("AdaptMap", msg_proxy_timer.TimeUs() / 1e3);
  // 1. Set start_point_info.
  const PoseProto pose_proto =
      AdaptPoseProto(*input_frame->odometry, input_frame->estimate_a);
  const AutonomyStateProto autonomy_state_proto =
      AdaptAutonomyStateProto(*input_frame->behavior);
  const double front_wheel_angle =
      GetFrontWheelAngle(*input_frame->vehicle_status,
                         vehicle_params.vehicle_drive_params().steer_ratio());

  std::ignore = AvHistory::instance()->PushPose(pose_proto);  // Copy pose

  LOG_INFO << "[acctask] prev_traj size: "
           << planner_state->previous_trajectory.trajectory_point_size();
  if (autonomy_state_proto.autonomy_state() != AutonomyStateProto::AUTO_DRIVE &&
      autonomy_state_proto.autonomy_state() !=
          AutonomyStateProto::AUTO_STEER_ONLY &&
      autonomy_state_proto.autonomy_state() !=
          AutonomyStateProto::AUTO_SPEED_ONLY) {
    planner_state->previous_trajectory.mutable_trajectory_point()->Clear();
    // planner_state->prev_route_sections.Clear();
    planner_state->planner_status_code = PlannerStatusProto::OK;
    LOG_INFO << "[acctask] clear prev traj!!!";
  }

  LOG_INFO << "[acctask] autonomy_state: "
           << AutonomyStateProto_State_Name(
                  autonomy_state_proto.autonomy_state());

  const auto& previous_trajectory = planner_state->previous_trajectory;
  // const bool previously_triggered_aeb = false;
  const bool aeb = false;
  const bool rerouted = false;  // TODO: Get from routing.
  const Vec2d ego_pos = {pose_proto.pos_smooth().x(),
                         pose_proto.pos_smooth().y()};
  planner_state->planner_status_code = PlannerStatusProto::OK;
  if (autonomy_state_proto.autonomy_state() == AutonomyStateProto::AUTO_DRIVE &&
      planner_state->planner_semantic_map_manager && is_map_valid &&
      !planner_state->planner_semantic_map_manager->GetNearestLaneWithHeading(
          ego_pos, pose_proto.yaw(), 5.0, M_PI / 6.0)) {
    planner_state->planner_status_code = PlannerStatusProto::EGO_LANE_LOSS_FAIL;
    is_map_valid = false;
  }
  // update behavior
  auto [current_behavior, current_Behavior_Choice] =
      AdaptBehavior(*input_frame->behavior, planner_params,
                    *input_frame->vehicle_status, planner_state);
  // lc command

  // 0:invalid, 1:preL, 2:preR, 3:L, 4:R, 5:abort, 6:mergeL, 7:mergeR,
  // 8:keep-path
  // ext_cmd_status->alc_state = ALCState::ALC_STANDBY;
  DriverAction::LaneChangeCommand lc_command_raw = DriverAction::LC_CMD_NONE;
  const auto behavior_action = input_frame->behavior->behavior_action();

  if (behavior_action ==
      byd::msg::planning::BehaviorAction::BEHAVIOR_ACTION_LEFT_CHANGE) {
    lc_command_raw = DriverAction::LC_CMD_LEFT;
  } else if (behavior_action ==
             byd::msg::planning::BehaviorAction::BEHAVIOR_ACTION_RIGHT_CHANGE) {
    lc_command_raw = DriverAction::LC_CMD_RIGHT;
  } else if (behavior_action ==
             byd::msg::planning::BehaviorAction::BEHAVIOR_ACTION_ABORT_CHANGE) {
    lc_command_raw = DriverAction::LC_CMD_CANCEL;
  }
  DriverAction::LaneChangeCommand new_lc_command = DriverAction::LC_CMD_NONE;
  if (planner_state->last_lc_command == DriverAction::LC_CMD_NONE &&
      lc_command_raw != DriverAction::LC_CMD_NONE) {
    LOG_ERROR << " set_lc_command: " << lc_command_raw;
    new_lc_command = lc_command_raw;
    planner_state->lc_command_number = 0;
    Log2DDS::LogDataV2(
        "lc_unable_reason",
        absl::StrCat("inti number:", planner_state->lc_command_number));
  } else if (planner_state->last_lc_command == lc_command_raw) {
    new_lc_command = lc_command_raw;
    if (lc_command_raw != DriverAction::LC_CMD_NONE) {
      planner_state->lc_command_number += 1;
    }
    Log2DDS::LogDataV2(
        "lc_unable_reason",
        absl::StrCat("+1 lcnumber:", planner_state->lc_command_number));
  } else {
    new_lc_command = DriverAction::LC_CMD_NONE;  // default setting
    planner_state->lc_command_number = 0;
    Log2DDS::LogDataV2(
        "lc_unable_reason",
        absl::StrCat("defalt number:", planner_state->lc_command_number));
    // allow manually interrupting the lane change activated by users
    if (lc_command_raw == DriverAction::LC_CMD_CANCEL) {
      new_lc_command = DriverAction::LC_CMD_CANCEL;
    }
  }
  planner_state->last_lc_command = lc_command_raw;

  // LOG_ERROR << " **********steer_ratio:"
  //            << vehicle_params.vehicle_drive_params().steer_ratio();
  // LOG_ERROR << " **********front_wheel_angle:" << front_wheel_angle;
  // FLAGS_planner_lookforward_time_ms is previous trajectory for trajectory
  // stable.
  const auto& vehicle_status = *input_frame->vehicle_status;
  bool override_passable =
      (vehicle_status.vcu_0x05a().vcu_drvraccrpedlovrd()) ||
      (vehicle_status.vcu_0x05a().vcu_accrpedlposnvld() &&
       vehicle_status.vcu_0x05a().act_acce_pedl_perc_hw() > 4.0);
  msg_proxy_timer.Reset("ComputePlanStartPoint");
  PlanStartPointInfo start_point_info;
  if (autonomy_state_proto.autonomy_state() ==
      AutonomyStateProto::AUTO_SPEED_ONLY) {
    start_point_info = ComputeAccPlanStartPoint(
        predicted_plan_time, previous_trajectory, pose_proto,
        autonomy_state_proto, planner_state->previous_autonomy_state, rerouted,
        aeb, front_wheel_angle, planner_params.motion_constraint_params(),
        vehicle_geom_params, vehicle_drive_params, override_passable);
  } else {
    start_point_info = ComputeEstPlanStartPoint(
        predicted_plan_time, previous_trajectory, pose_proto,
        autonomy_state_proto, planner_state->previous_autonomy_state, rerouted,
        aeb, front_wheel_angle, planner_params.motion_constraint_params(),
        vehicle_geom_params, vehicle_drive_params, override_passable,
        current_behavior.function_id());
  }
  Log2DDS::LogDataV0("StartPointInfo", start_point_info.DebugString());
  Log2DDS::LogDataV0("reset", start_point_info.reset);
  auto min_path_look_ahead_duration = GetStPathPlanLookAheadTime(
      start_point_info, pose_proto, absl::Seconds(0.0), previous_trajectory);

  bool use_reset_if_acc_or_to_lcc =
      (current_behavior.function_id() == Behavior_FunctionId_ACC &&
       start_point_info.reset_reason != ResetReasonProto::SPEED_ONLY) ||
      start_point_info.reset_reason != ResetReasonProto::SPEED_ONLY_ENGAGE;
  auto time_aligned_prev_traj = CreatePreviousTrajectory(
      start_point_info.plan_time, previous_trajectory,
      planner_params.motion_constraint_params(),
      start_point_info.reset && use_reset_if_acc_or_to_lcc);
  // mapping::LanePoint destination(mapping::ElementId(""), 1.0);
  // RouteSections route_section(
  //     0.0, 1.0,
  //     planner_state->planner_semantic_map_manager->map_ptr()->route()->GetNaviRouteIds(),
  //     destination);
  // route_section.set_topology_start_offset(
  //     planner_state->planner_semantic_map_manager->map_ptr()->route()->navi_start().s_offset);
  // rm_output_->route_sections_from_current = route_section;
  // // ----------------------Route Result------------------------
  // // Dynamic update route in NOA mode.
  // if (!rerouted && rm_output_ &&
  //     !rm_output_->route_sections_from_current.empty() &&
  //     !planner_state->prev_route_sections.empty()) {
  //   auto tailored_sections =
  //       AppendRouteSectionsToTail(planner_state->prev_route_sections,
  //                                 rm_output_->route_sections_from_current);
  //   tailored_sections = rm_output_->route_sections_from_current;
  //   if (!tailored_sections.ok()) {
  //     LOG_INFO << "Planner prev route sections does not match route, reset";
  //     planner_state->prev_route_sections.Clear();
  //   } else {
  //     planner_state->prev_route_sections =
  //     std::move(tailored_sections).value();
  //   }
  // }
  // est planner post process.
  // const absl::Cleanup fill_prev_target_lane_path = [this, &ego_pos,
  //                                                   &planner_state]() {
  //   // Check and fill prev_target_lane_path before the end of each
  //   // iteration.
  //   if (planner_state->prev_target_lane_path.IsEmpty()) {
  //     auto target_lane_path_or = FindClosestTargetLanePathOnReset(
  //         *planner_state->planner_semantic_map_manager,
  //         planner_state->prev_route_sections, ego_pos);
  //     if (target_lane_path_or.ok()) {
  //       // planner_state->prev_target_lane_path =
  //       // std::move(*target_lane_path_or);
  //     }
  //   }
  // };
  // Initialize route sections.

  // if (planner_state->prev_route_sections.empty()) {
  // RETURN_PLANNER_STATUS_OR_ASSIGN(
  //     planner_state->prev_route_sections,
  //     BackwardExtendRouteSectionsFromPos(
  //         *planner_state->planner_semantic_map_manager,
  //         rm_output_->route_sections_from_current, ego_pos,
  //         kDrivePassageKeepBehindLength),
  //     PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED);
  // auto target_lane_path_or = FindClosestTargetLanePathOnReset(
  //     *planner_state->planner_semantic_map_manager,
  //     planner_state->prev_route_sections, ego_pos);
  // if (target_lane_path_or.ok()) {
  //   planner_state->prev_target_lane_path = std::move(*target_lane_path_or);
  // } else {
  //   // BANDAID(SCENARIOS-689): Force to recalculate prev target lane path
  //   // by
  //   // clearing prev route section.
  //   planner_state->prev_route_sections.Clear();
  //   return PlannerStatus(
  //       PlannerStatusProto::RESET_PREV_TARGET_LANE_PATH_FAILED,
  //       target_lane_path_or.status().message());
  // }
  // }
  // if (planner_state->prev_target_lane_path.IsEmpty()) {
  // Should have been filled at the end of the last iteration.
  // return PlannerStatus(PlannerStatusProto::PLANNER_STATE_INCOMPLETE,
  //                      "Prev target lane path empty.");
  // }
  // Clear PLC related states on reset.
  if (start_point_info.reset &&
      ShouldResetAlcState(start_point_info.reset_reason)) {
    planner_state->preferred_lane_path = mapping::LanePath();
    // ext_cmd_status->alc_state = ALC_STANDBY_ENABLE;
    // ext_cmd_status->lane_change_command = DriverAction::LC_CMD_NONE;
  }
  Log2DDS::LogDataV0("ComputePlanStartPoint", msg_proxy_timer.TimeUs() / 1e3);
  // -------------------- Route Section -----------------------
  // auto route_sections_proj_or = ProjectPointToRouteSections(
  //     *planner_state->planner_semantic_map_manager,
  //     planner_state->prev_route_sections, ego_pos,
  //     kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
  //     kDrivePassageKeepBehindLength);
  // if (!route_sections_proj_or.ok()) {
  //   // Clear to force to switch route.
  //   planner_state->prev_route_sections.Clear();
  //   // Exit auto mode.
  //   return PlannerStatus(
  //       PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED,
  //       route_sections_proj_or.status().message());
  // }
  // auto [route_sections_from_start, route_sections_with_behind, ego_pos_proj]
  // =
  //     std::move(route_sections_proj_or).value();
  // planner_state->prev_route_sections = std::move(route_sections_with_behind);
  // ego_nearest_lane_id_ = ego_pos_proj.lane_id;
  // -------------------- Smooth Reference Line ---------------
  if (is_map_valid) {
    const double half_av_width =
        vehicle_params.vehicle_geometry_params().width() * 0.5;
    auto smooth_result_map_or = BuildSmoothedResultMapFromRouteSections(
        *planner_state->planner_semantic_map_manager, ego_pos, half_av_width,
        std::move(planner_state->smooth_result_map));
    if (smooth_result_map_or.ok()) {
      planner_state->smooth_result_map =
          std::move(smooth_result_map_or).value();
    }
  }

  // route_sections_from_start_ = std::make_unique<RouteSections>();
  // ----------- Traffic light info collect  ------------------
  // tl_info_map_ = std::make_unique<TrafficLightInfoMap>();
  // RETURN_PLANNER_STATUS_OR_ASSIGN(
  //     auto tl_info_collector_output,
  //     CollectTrafficLightInfo(
  //         TrafficLightInfoCollectorInput{
  //             .psmm =
  //             input.planner_input->planner_semantic_map_manager.get(),
  //             .traffic_light_states =
  //                 input.planner_input->traffic_light_states.get(),
  //             .route_sections = &route_sections_from_start,
  //             .plan_time = input.plan_time},
  //         std::move(planner_state->yellow_light_observations)),
  //     PlannerStatusProto::TRAFFIC_LIGHT_INFO_COLLECTOR_FAILED);
  // planner_state->yellow_light_observations =
  //     std::move(tl_info_collector_output.yellow_light_observations);
  // 3. Perception and Prediction
  // Preprocess planner objects.

  Log2DDS::LogDataV0("lc_debug",
                     absl::StrCat("new_lc_command: ", new_lc_command));
  msg_proxy_timer.Reset("AdaptPrediction");
  auto prediction_result = AdaptPredictionResult(*input_frame->prediction);
  ComputeObjectsStopTime(prediction_result,
                         planner_state->object_stop_time_map);
  auto objects_proto = ObjectsProto(std::move(prediction_result.objects_proto));
  const ObjectsPredictionProto& objects_prediction_proto =
      prediction_result.objects_prediction_proto;
  auto planner_objects = BuildPlannerObjects(
      &objects_proto, &objects_prediction_proto,
      ToUnixDoubleSeconds(start_point_info.plan_time), thread_pool);
  // Enabled low likelihood object filter.
  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      /*only_use_most_likely_traj=*/false);
  const MotionStateFilter motion_state_filter(pose_proto, vehicle_geom_params);
  // const PredictedMotionFilter predicted_motion_filter(planner_objects);
  // std::vector<const TrajectoryFilter*> filters = {&low_likelihood_filter,
  //                                                 &predicted_motion_filter};
  // Disable predicted_motion_filter
  bool is_on_highway = false;
  if (planner_state->planner_semantic_map_manager != nullptr &&
      planner_state->planner_semantic_map_manager->map_ptr() != nullptr &&
      planner_state->planner_semantic_map_manager->map_ptr()->IsValid()) {
    const auto& psmm = *(planner_state->planner_semantic_map_manager);
    is_on_highway = psmm.map_ptr()->is_on_highway();
  }
  const LaneAttrTypeFilter lane_attr_type_filter(
      pose_proto, vehicle_geom_params, planner_state->lane_change_state,
      is_on_highway);
  std::vector<const TrajectoryFilter*> filters = {&low_likelihood_filter};
  filters.push_back(&motion_state_filter);
  filters.push_back(&lane_attr_type_filter);
  // TODO: Delete the code after March 01, 2022
  const ReflectedObjectInProximityFilter object_in_proximity_filter(
      pose_proto, vehicle_geom_params,
      FLAGS_planner_filter_reflected_object_distance);
  if (FLAGS_planner_filter_reflected_object_distance > 0.0) {
    filters.push_back(&object_in_proximity_filter);
  }
  PlannerObjectManagerBuilder obj_mgr_builder;
  obj_mgr_builder.set_planner_objects(std::move(planner_objects))
      .set_filters(filters);
  FilteredTrajectories filtered_trajs;
  filtered_trajs.clear_filtered();
  auto object_manager_or = obj_mgr_builder.Build(&filtered_trajs, thread_pool);
  if (!object_manager_or.ok()) {
    // auto* planner_status =
    //     output->mutable_planner_debug()->mutable_planner_status();
    // planner_status->set_status(PlannerStatusProto::OBJECT_MANAGER_FAILED);
    // planner_status->set_message(
    //     std::string(object_manager_or.status().message()));
    return PlannerStatus(PlannerStatusProto::OBJECT_MANAGER_FAILED,
                         std::string(object_manager_or.status().message()));
  }
  std::unordered_map<std::string, FilterReason::Type> filtered_table;
  for (int i = 0; i < filtered_trajs.filtered_size(); i++) {
    filtered_table[filtered_trajs.filtered(i).id()] =
        filtered_trajs.filtered(i).reason();
  }
  auto object_manager = PlannerObjectManager(std::move(*object_manager_or));
  auto st_traj_mgr = SpacetimeTrajectoryManager(
      absl::Span<const TrajectoryFilter*>{}, object_manager.planner_objects(),
      object_manager.frame_dropped_objects(), thread_pool);
  Log2DDS::LogDataV0("AdaptPrediction", msg_proxy_timer.TimeUs() / 1e3);
  // 4. Others
  // set user set speed
  // const double odom_speed_scale = (*input_frame->odometry).spd_factor() >
  // 5e-3
  //                                     ? (*input_frame->odometry).spd_factor()
  //                                     : 1.0;
  msg_proxy_timer.Reset("SceneReasoning");
  const double odom_speed_scale = 1.0;
  double set_speed_kph_disp = (*input_frame->behavior).target_speed();
  const double act_speed_mps_disp =
      (*input_frame->vehicle_status).ipb_0x10c().vehicle_speed();
  const double display_speed =
      (*input_frame->vehicle_status).left_bcm_0x151().speed_signal_151_s();
  const auto& speed_limit_params =
      planner_params.speed_finder_params().speed_limit_params();
  const double pid_pgain_factor = speed_limit_params.pid_pgain_factor();
  const double pid_igain_factor = speed_limit_params.pid_igain_factor();
  bool enable_use_pid_set_speed = speed_limit_params.enable_use_pid_set_speed();

  PiecewiseLinearFunction<double> user_set_speed_bias_plf(
      PiecewiseLinearFunctionFromProto(
          speed_limit_params.user_set_speed_bias_plf()));

  PiecewiseLinearFunction<double> user_set_speed_gain_plf(
      PiecewiseLinearFunctionFromProto(
          speed_limit_params.user_set_speed_gain_plf()));
  const double cruising_speed_limit =
      odom_speed_scale *
      SpeedTransformationUsePI(
          user_set_speed_bias_plf, user_set_speed_gain_plf, set_speed_kph_disp,
          act_speed_mps_disp, display_speed, pid_pgain_factor, pid_igain_factor,
          enable_use_pid_set_speed, &planner_state->display_error_integrator);

  const double target_lane_speed_limit = GetLaneSpeedLimit(
      user_set_speed_bias_plf, user_set_speed_gain_plf, set_speed_kph_disp);
  LOG_ERROR << " lane_speed_limit : " << target_lane_speed_limit;

  Log2DDS::LogDataV2("target_lane_speed_limit-speed", target_lane_speed_limit);
  ad_byd::planning::TrafficLightStatusMap traffic_light_status_map =
      is_map_valid ? planner_state->planner_semantic_map_manager->map_ptr()
                         ->traffic_light_status_map()
                   : ad_byd::planning::TrafficLightStatusMap();
  auto scene_reasoning = SceneOutputProto();
  absl::flat_hash_set<std::string> stalled_objects;
  absl::flat_hash_set<std::string> in_queue_objects;

  if (is_map_valid) {
    const auto scene_reasoning_output = RunSceneReasoningAndFillDebug(
        *planner_state->planner_semantic_map_manager, traffic_light_status_map,
        object_manager, objects_prediction_proto, start_point_info.start_point,
        thread_pool, planner_state->object_history_manager);

    // if (ext_cmd_status->enable_lc_objects) {
    //   for (const auto& stalled_object : debug_proto.stalled_objects()) {
    //     stalled_objects.insert(stalled_object.id());
    //   }
    // }
    for (const auto& stalled_object :
         scene_reasoning_output.scene_output_proto.objects_annotation()) {
      auto ite = filtered_table.find(stalled_object.object_id());
      if (ite != filtered_table.end() &&
          (ite->second == FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE ||
           ite->second == FilterReason::TRAJECTORY_IS_FRAME_DROPPED)) {
        continue;
      }
      stalled_objects.insert(stalled_object.object_id());
    }
    // double dynamic_headway =
    //     input_frame->behavior->driving_preference().headway_time();

    for (const auto& queue :
         scene_reasoning_output.scene_output_proto.traffic_waiting_queue()) {
      for (const auto& obj_str : queue.object_id()) {
        in_queue_objects.insert(obj_str);
      }
    }
    scene_reasoning = scene_reasoning_output.scene_output_proto;
  }
  Log2DDS::LogDataV0("SceneReasoning", msg_proxy_timer.TimeUs() / 1e3);
  planner_state->ClearHMIInfo();
  for (const auto& object_pred : objects_prediction_proto.objects()) {
    const auto& object = object_pred.perception_object();
    // Fill the HMI info
    if (ObjectType::OT_VEHICLE == object.type() ||
        ObjectType::OT_LARGE_VEHICLE == object.type()) {
      if (stalled_objects.find(object.id()) != stalled_objects.end()) {
        planner_state->stalled_cars.push_back(object.id());
      } else if (in_queue_objects.find(object.id()) != in_queue_objects.end()) {
        planner_state->in_queue_cars.push_back(object.id());
      }
    }
  }

  // auto stalled_objects_ptr = absl::flat_hash_set<std::string>(
  //     std::move(stalled_objects));
  // traffic light ok button
  bool btn_noa_on = false;
  bool btn_ad_op_tb = false;
  bool enable_tl_ok_btn = btn_noa_on || btn_ad_op_tb;

  // std::optional<Behavior_FunctionId> function_id_opt =
  //     static_cast<Behavior_FunctionId>(
  //         input_frame->behavior->func_id().underlying());
  // 5. Update planner_state.
  planner_state->previous_autonomy_state = autonomy_state_proto;
  // return OkPlannerStatus();
  
  //6. Determine whether the VRU crosses(appears) in the rectangular area in front of the vehicle
  const double vru_detect_box_length = 8 , vru_detect_box_width = 3.5; //m
  const Vec2d egocar_pos = {pose_proto.pos_smooth().x(),
                         pose_proto.pos_smooth().y()};
  const double egocar_theta = pose_proto.yaw();
  const Vec2d tangent = Vec2d::FastUnitFromAngle(egocar_theta);
  
  const double rac_to_boxcenter = 0.5 * vru_detect_box_length + 
                                  vehicle_geom_params.front_edge_to_center();
  const Vec2d vru_detect_boxcenter = egocar_pos + tangent * rac_to_boxcenter;
  const auto vru_detect_box = Box2d(0.5 * vru_detect_box_length, 0.5 * vru_detect_box_width, 
                                    vru_detect_boxcenter, egocar_theta, tangent);
  bool vru_detected = false;
  if(!st_traj_mgr.trajectories().empty()){
      for (const SpacetimeObjectTrajectory& st_traj : st_traj_mgr.trajectories()) {
         if(st_traj.object_type() != OT_PEDESTRIAN && st_traj.object_type() != OT_CYCLIST &&
            st_traj.object_type() != OT_TRICYCLIST){
                continue;
          }
         const auto& obs_center_x = st_traj.trajectory().points().front().pos().x();
         const auto& obs_center_y = st_traj.trajectory().points().front().pos().y();
         vru_detected = vru_detect_box.IsPointIn(Vec2d(obs_center_x,obs_center_y));
      }
  }
  LOG_INFO << " is_vru_for_suppressing_auto_start: " << vru_detected;
  Log2DDS::LogDataV0("is_vru_for_suppressing_auto_start", vru_detected);

  std::unique_ptr<MultiTasksCruisePlannerInput> st_planner_input(
      new MultiTasksCruisePlannerInput{
          .planner_params = &planner_params,
          .vehicle_params = &vehicle_params,
          .start_point_info = std::move(start_point_info),
          .ego_pos = ego_pos,
          .min_path_look_ahead_duration = min_path_look_ahead_duration,
          .st_traj_mgr = std::move(st_traj_mgr),
          .object_manager = std::move(object_manager),
          .stalled_objects = std::move(stalled_objects),
          .consider_lane_change_gap = true,
          .scene_reasoning = std::move(scene_reasoning),
          .time_aligned_prev_traj = std::move(time_aligned_prev_traj),
          .traffic_light_status_map = std::move(traffic_light_status_map),
          .new_lc_command = new_lc_command,
          .auto_model = input_frame->behavior->automode_enable(),
          .selector_state = &planner_state->selector_state,
          .objects_proto = std::move(objects_proto),
          .enable_tl_ok_btn = enable_tl_ok_btn,
          .override_passable = override_passable,
          .cruising_speed_limit = cruising_speed_limit,
          .behavior = current_behavior,
          .pose_proto = std::move(pose_proto),
          .front_wheel_angle = front_wheel_angle,

          .av_context = AvHistory::instance(),
          .acc_autonomy_state_input =
              {
                  .standstill_wait =
                      input_frame->behavior->standstill_request() ==
                      byd::msg::planning::StandstillRequest::
                          STAND_STILL_REQUEST_REQUEST,
                  .curve_speed_limit_allowed =
                      input_frame->behavior->curvespeed_limit_switch() ==
                      byd::msg::planning::CurveSpeedLimitSwitch::
                          CURVE_SPEED_LIMIT_SWITCH_ON,
                  .acceleration_request =
                      input_frame->behavior->acceleration_request(),
              },
          .map_event = input_frame->map_event.get(),
          .is_vru_for_suppressing_auto_start = vru_detected,
          .behavior_choice = current_Behavior_Choice,
          .target_lane_speed_limit = target_lane_speed_limit,
      });
  return st_planner_input;
}

PoseProto AdaptPoseProto(const ad_byd::planning::odometry_type& odom,
                         const double estimate_a) {
  DLOG(INFO) << __FUNCTION__;
  PoseProto pose_proto;
  pose_proto.mutable_header()->set_timestamp(
      odom.header().measurement_timestamp() * 1e6);  // int64, us.
  pose_proto.mutable_header()->set_channel_seq_number(
      odom.header().sequence_num());
  pose_proto.set_timestamp(
      odom.header().measurement_timestamp());  // double, s.
  auto& odom_pos = odom.pose();
  pose_proto.mutable_pos_smooth()->set_x(odom_pos.position().x());
  pose_proto.mutable_pos_smooth()->set_y(odom_pos.position().y());
  pose_proto.mutable_pos_smooth()->set_z(odom_pos.position().z());

  Eigen::Quaterniond quat;
  quat.x() = odom_pos.orientation().qx();
  quat.y() = odom_pos.orientation().qy();
  quat.z() = odom_pos.orientation().qz();
  quat.w() = odom_pos.orientation().qw();
  quat.normalized();
  const Eigen::Matrix3d R = quat.toRotationMatrix();
  // const double yaw = R.eulerAngles(0, 1, 2)(2);
  // const double yaw = ad_byd::planning::math::QuaternionToYaw(
  //     odom_pos.orientation().qw(), odom_pos.orientation().qx(),
  //     odom_pos.orientation().qy(), odom_pos.orientation().qz());
  // LOG_INFO << "qx: " << odom_pos.orientation().qx()
  //           << " , qy: " << odom_pos.orientation().qy()
  //           << " , qz: " << odom_pos.orientation().qz()
  //           << " , qw: " << odom_pos.orientation().qw();
  // LOG_INFO << "plan yaw_0: " << yaw << " , plan yaw_1: " << yaw_1
  //           << " , dr heading: " << odom_pos.heading() * M_PI / 180.0;
  const double yaw = odom_pos.heading() * M_PI / 180.0;
  pose_proto.set_yaw(yaw);
  pose_proto.set_pitch(0.0);
  pose_proto.set_roll(0.0);
  // Set velocity, acceleration, angular rate in odom frame.
  pose_proto.mutable_vel_smooth()->set_x(odom_pos.linear_velocity().x());
  pose_proto.mutable_vel_smooth()->set_y(odom_pos.linear_velocity().y());
  pose_proto.mutable_vel_smooth()->set_z(odom_pos.linear_velocity().z());

  const double G = 9.80665;
  const auto& accel3d = odom_pos.linear_acceleration();
  pose_proto.mutable_accel_body()->set_x(estimate_a);
  pose_proto.mutable_accel_body()->set_y(0.0);
  pose_proto.mutable_accel_body()->set_z(0.0);
  const Eigen::Vector3d ego_acc_body(pose_proto.accel_body().x(),
                                     pose_proto.accel_body().y(),
                                     pose_proto.accel_body().z());
  const Eigen::Vector3d odom_acc = R * ego_acc_body;
  pose_proto.mutable_accel_smooth()->set_x(odom_acc[0]);
  pose_proto.mutable_accel_smooth()->set_y(odom_acc[1]);
  pose_proto.mutable_accel_smooth()->set_z(odom_acc[2]);
  const Eigen::Vector3d ego_angular_rate_body(odom_pos.angular_velocity().x(),
                                              odom_pos.angular_velocity().y(),
                                              odom_pos.angular_velocity().z());
  const Eigen::Vector3d odom_angular_rate = R * ego_angular_rate_body;
  pose_proto.mutable_ar_smooth()->set_x(odom_angular_rate[0]);
  pose_proto.mutable_ar_smooth()->set_y(odom_angular_rate[1]);
  pose_proto.mutable_ar_smooth()->set_z(odom_angular_rate[2]);
  // Set velocity, acceleration, angular rate in body frame.
  const Eigen::Vector3d odom_velocity = Eigen::Vector3d(
      odom_pos.linear_velocity().x(), odom_pos.linear_velocity().y(),
      odom_pos.linear_velocity().z());
  const Eigen::Vector3d ego_velocity = R.inverse() * odom_velocity;
  pose_proto.mutable_vel_body()->set_x(ego_velocity[0]);
  pose_proto.mutable_vel_body()->set_y(ego_velocity[1]);
  pose_proto.mutable_vel_body()->set_z(ego_velocity[2]);
  pose_proto.set_speed(std::sqrt(ego_velocity[0] * ego_velocity[0] +
                                 ego_velocity[1] * ego_velocity[1]));
  pose_proto.mutable_ar_body()->set_x(odom_pos.angular_velocity().x());
  pose_proto.mutable_ar_body()->set_y(odom_pos.angular_velocity().y());
  pose_proto.mutable_ar_body()->set_z(odom_pos.angular_velocity().z());
  return pose_proto;
}

AutonomyStateProto AdaptAutonomyStateProto(
    const ad_byd::planning::behavior_type& behavior) {
  DLOG(INFO) << __FUNCTION__;
  AutonomyStateProto now_autonomy_state;
  now_autonomy_state.set_autonomy_state(
      AutonomyStateProto::READY_TO_AUTO_DRIVE);
  const auto& function_id = behavior.function_id();

  if (function_id == byd::msg::planning::FunctionId::FUNCTION_CITY_NOA ||
      function_id == byd::msg::planning::FunctionId::FUNCTION_HW_NOA ||
      function_id == byd::msg::planning::FunctionId::FUNCTION_MAPLESS_NOA ||
      function_id == byd::msg::planning::FunctionId::FUNCTION_LKA) {
    now_autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);
  }

  if (FLAGS_planner_enable_acc &&
      (FLAGS_planner_task_init_type == /*ACC ONLY*/ 2 ||
       function_id == byd::msg::planning::FunctionId::FUNCTION_ACC)) {
    now_autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_SPEED_ONLY);
  }

  return now_autonomy_state;
}

std::pair<Behavior, BehaviorChoice> AdaptBehavior(
    const ad_byd::planning::behavior_type& behavior,
    const PlannerParamsProto& planner_params,
    const ad_byd::planning::vehicle_status_type& veh_status,
    PlannerState* planner_state) {
  DLOG(INFO) << __FUNCTION__;
  Behavior current_behavior;
  // set function id
  Behavior_FunctionId function_id =
      static_cast<Behavior_FunctionId>(behavior.function_id());
  current_behavior.set_function_id(function_id);
  // set tunnel status
  auto tunnel_status = behavior.tunnel();
  current_behavior.set_tunnel(tunnel_status);
  bool auto_navi_lc_enable_status = behavior.auto_navi_lc_enable_status();
  bool traffic_light_func_enable = behavior.traffic_light_func_enable();
  current_behavior.set_auto_navi_lc_enable_status(auto_navi_lc_enable_status);
  current_behavior.set_traffic_light_func_enable(traffic_light_func_enable);
  // set brake to stop command
  bool system_break_stop = false;
  // 0:invalid, 1:cruise, 2:LC, 3:merge, 4:split, 5:MRM, 6:MRC
  const auto behavior_choice = behavior.behavior_choice();
  if (function_id == Behavior_FunctionId_CITY_NOA ||
      function_id == Behavior_FunctionId_MAPLESS_NOA ||
      function_id == Behavior_FunctionId_LKA) {
    if (behavior_choice ==
            byd::msg::planning::BehaviorChoice::BEHAVIOR_CHOICE_MRM ||
        behavior_choice ==
            byd::msg::planning::BehaviorChoice::BEHAVIOR_CHOICE_MRC) {
      system_break_stop = true;
    } else {
      system_break_stop = false;
    }
  } else {
    system_break_stop = false;
  }
  current_behavior.set_system_break_stop(system_break_stop);
  // set dynamic follow headway
  double dynamic_headway = behavior.dynamic_headway();
  current_behavior.set_dynamic_headway(dynamic_headway);

  current_behavior.set_lane_change_style(
      static_cast<Behavior_LaneChangeStyle>(behavior.lane_change_style()));

  // set user set speed
  const auto& speed_limit_params =
      planner_params.speed_finder_params().speed_limit_params();
  const double set_speed_kph_disp = behavior.target_speed();
  const double act_speed_mps_disp = veh_status.ipb_0x10c().vehicle_speed();
  const double display_speed = veh_status.left_bcm_0x151().speed_signal_151_s();
  const double pid_pgain_factor = speed_limit_params.pid_pgain_factor();
  const double pid_igain_factor = speed_limit_params.pid_igain_factor();
  bool enable_use_pid_set_speed = speed_limit_params.enable_use_pid_set_speed();

  PiecewiseLinearFunction<double> user_set_speed_bias_plf(
      PiecewiseLinearFunctionFromProto(
          speed_limit_params.user_set_speed_bias_plf()));
  PiecewiseLinearFunction<double> user_set_speed_gain_plf(
      PiecewiseLinearFunctionFromProto(
          speed_limit_params.user_set_speed_gain_plf()));
  const double cruising_speed_limit = SpeedTransformationUsePI(
      user_set_speed_bias_plf, user_set_speed_gain_plf, set_speed_kph_disp,
      act_speed_mps_disp, display_speed, pid_pgain_factor, pid_igain_factor,
      enable_use_pid_set_speed, &planner_state->display_error_integrator);

  current_behavior.set_cruising_speed_limit(cruising_speed_limit);
  Behavior_FunctionId select_function_id =
      static_cast<Behavior_FunctionId>(behavior.driver_select_function_id());
  current_behavior.set_driver_select_function_id(select_function_id);
  return {current_behavior, behavior_choice};
}

constexpr double keps = 1e-3;
double GetFrontWheelAngle(
    const ad_byd::planning::vehicle_status_type& veh_status,
    double steer_ratio) {
  DLOG(INFO) << __FUNCTION__;
  const double front_wheel_angle = ad_byd::planning::Constants::DEG2RAD *
                                   veh_status.eps_0x06d().eps_steerwheelag() /
                                   std::max(steer_ratio, keps);

  return front_wheel_angle;
}

double GetLaneSpeedLimit(
    const PiecewiseLinearFunction<double>& set_speed_bias_plf,
    const PiecewiseLinearFunction<double>& set_speed_gain_plf,
    double set_speed_kph) {
  const double speed_offset = set_speed_bias_plf(set_speed_kph);
  const double speed_gain = set_speed_gain_plf(set_speed_kph);
  double act_speed_mps = set_speed_kph / speed_gain;
  act_speed_mps = Kph2Mps(act_speed_mps) + speed_offset;
  return act_speed_mps;
}

TrajectoryIntention AdaptTrajectoryIntention(
    const byd::msg::prediction::TrajectoryIntention& intention) {
  TrajectoryIntention trajectory_intention =
      TrajectoryIntention::INTENTION_UNKNOWN;
  if (intention ==
      byd::msg::prediction::TrajectoryIntention::INTENTION_PARALLEL) {
    trajectory_intention = TrajectoryIntention::INTENTION_PARALLEL;
  } else if (intention ==
             byd::msg::prediction::TrajectoryIntention::INTENTION_LC_LEFT) {
    trajectory_intention = TrajectoryIntention::INTENTION_LC_LEFT;
  } else if (intention ==
             byd::msg::prediction::TrajectoryIntention::INTENTION_LC_RIGHT) {
    trajectory_intention = TrajectoryIntention::INTENTION_LC_RIGHT;
  } else if (intention ==
             byd::msg::prediction::TrajectoryIntention::INTENTION_TURN_LEFT) {
    trajectory_intention = TrajectoryIntention::INTENTION_TURN_LEFT;
  } else if (intention ==
             byd::msg::prediction::TrajectoryIntention::INTENTION_TURN_RIGHT) {
    trajectory_intention = TrajectoryIntention::INTENTION_TURN_RIGHT;
  } else if (intention ==
             byd::msg::prediction::TrajectoryIntention::INTENTION_TURN_AROUND) {
    trajectory_intention = TrajectoryIntention::INTENTION_TURN_AROUND;
  } else if (intention ==
             byd::msg::prediction::TrajectoryIntention::INTENTION_CROSS) {
    trajectory_intention = TrajectoryIntention::INTENTION_CROSS;
  }
  return trajectory_intention;
}

st::OlaType AdaptObjectOlaType(byd::msg::perception::OlaType li_ola_type) {
  st::OlaType ola_type = st::OlaType::OLA_UNKNOWN;
  switch (li_ola_type) {
    case byd::msg::perception::OlaType::OLA_NO_OCCUPANCY:
      ola_type = st::OlaType::OLA_NO_OCCUPANCY;
      break;
    case byd::msg::perception::OlaType::OLA_RIGHT_SLIGHT_OCCUPANCY:
      ola_type = st::OlaType::OLA_RIGHT_SLIGHT_OCCUPANCY;
      break;
    case byd::msg::perception::OlaType::OLA_LEFT_SLIGHT_OCCUPANCY:
      ola_type = st::OlaType::OLA_LEFT_SLIGHT_OCCUPANCY;
      break;
    case byd::msg::perception::OlaType::OLA_SEVERE_OCCUPANCY:
      ola_type = st::OlaType::OLA_SEVERE_OCCUPANCY;
      break;
    default:
      ola_type = st::OlaType::OLA_UNKNOWN;
      break;
  }
  return ola_type;
}

st::LaneAttrType AdaptObjectLaneAttrType(
    byd::msg::perception::LaneAttrType li_lane_attr_type) {
  st::LaneAttrType lane_attr_type = st::LaneAttrType::LANEATTR_UNKNOWN;
  switch (li_lane_attr_type) {
    case byd::msg::perception::LaneAttrType::LANEATTR_SELF:
      lane_attr_type = st::LaneAttrType::LANEATTR_SELF;
      break;
    case byd::msg::perception::LaneAttrType::LANEATTR_LEFT:
      lane_attr_type = st::LaneAttrType::LANEATTR_LEFT;
      break;
    case byd::msg::perception::LaneAttrType::LANEATTR_RIGHT:
      lane_attr_type = st::LaneAttrType::LANEATTR_RIGHT;
      break;
    case byd::msg::perception::LaneAttrType::LANEATTR_ON_LINE:
      lane_attr_type = st::LaneAttrType::LANEATTR_ON_LINE;
      break;
    case byd::msg::perception::LaneAttrType::LANEATTR_OTHER:
      lane_attr_type = st::LaneAttrType::LANEATTR_OTHER;
      break;
    default:
      lane_attr_type = st::LaneAttrType::LANEATTR_UNKNOWN;
      break;
  }
  return lane_attr_type;
}

st::ObstacleLightType AdaptObjectAttrLeftLightType(
    byd::msg::perception::TurnLightType in_obj_attr_light_type) {
  st::ObstacleLightType out_obj_attr_light_type =
      st::ObstacleLightType::LIGHT_UNKNOWN;
  switch (in_obj_attr_light_type) {
    case byd::msg::perception::TurnLightType::TL_UNKNOWN:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_UNKNOWN;
      break;
    case byd::msg::perception::TurnLightType::TL_LEFT_LIGHT_ON:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_ON;
      break;
    case byd::msg::perception::TurnLightType::TL_RIGHT_LIGHT_ON:
    case byd::msg::perception::TurnLightType::TL_LEFT_RIGHT_LIGHT_ON:
    case byd::msg::perception::TurnLightType::TL_ALL_CLOSE:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_OFF;
      break;
    default:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_UNKNOWN;
      break;
  }
  return out_obj_attr_light_type;
}

st::ObstacleLightType AdaptObjectAttrRightLightType(
    byd::msg::perception::TurnLightType in_obj_attr_light_type) {
  st::ObstacleLightType out_obj_attr_light_type =
      st::ObstacleLightType::LIGHT_UNKNOWN;
  switch (in_obj_attr_light_type) {
    case byd::msg::perception::TurnLightType::TL_UNKNOWN:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_UNKNOWN;
      break;
    case byd::msg::perception::TurnLightType::TL_RIGHT_LIGHT_ON:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_ON;
      break;
    case byd::msg::perception::TurnLightType::TL_LEFT_LIGHT_ON:
    case byd::msg::perception::TurnLightType::TL_LEFT_RIGHT_LIGHT_ON:
    case byd::msg::perception::TurnLightType::TL_ALL_CLOSE:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_OFF;
      break;
    default:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_UNKNOWN;
      break;
  }
  return out_obj_attr_light_type;
}

st::ObstacleLightType AdaptObjectAttrHazardLightType(
    byd::msg::perception::TurnLightType in_obj_attr_light_type) {
  st::ObstacleLightType out_obj_attr_light_type =
      st::ObstacleLightType::LIGHT_UNKNOWN;
  switch (in_obj_attr_light_type) {
    case byd::msg::perception::TurnLightType::TL_UNKNOWN:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_UNKNOWN;
      break;
    case byd::msg::perception::TurnLightType::TL_LEFT_RIGHT_LIGHT_ON:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_ON;
      break;
    case byd::msg::perception::TurnLightType::TL_LEFT_LIGHT_ON:
    case byd::msg::perception::TurnLightType::TL_RIGHT_LIGHT_ON:
    case byd::msg::perception::TurnLightType::TL_ALL_CLOSE:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_OFF;
      break;
    default:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_UNKNOWN;
      break;
  }
  return out_obj_attr_light_type;
}

st::ObstacleLightType AdaptObjectAttrBrakeLightType(
    byd::msg::perception::BrakeLightType in_obj_attr_light_type) {
  st::ObstacleLightType out_obj_attr_light_type =
      st::ObstacleLightType::LIGHT_UNKNOWN;
  switch (in_obj_attr_light_type) {
    case byd::msg::perception::BrakeLightType::BL_UNKNOWN:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_UNKNOWN;
      break;
    case byd::msg::perception::BrakeLightType::BL_BRAKE_LIGHT_ON:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_ON;
      break;
    case byd::msg::perception::BrakeLightType::BL_BRAKE_LIGHT_OFF:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_OFF;
      break;
    default:
      out_obj_attr_light_type = st::ObstacleLightType::LIGHT_UNKNOWN;
      break;
  }
  return out_obj_attr_light_type;
}

// 障碍物类型的映射需要在对一下
ObjectType AdaptObjectType(byd::msg::perception::ObjectType obs_type,
                           bool is_traj_empty) {
  ObjectType obstacle_type = OT_UNKNOWN_MOVABLE;
  switch (obs_type) {
    case byd::msg::perception::ObjectType::UNKNOWN:
    case byd::msg::perception::ObjectType::GENERAL_OBJECT:
      if (is_traj_empty) {
        obstacle_type = OT_UNKNOWN_STATIC;
      } else {
        obstacle_type = OT_UNKNOWN_MOVABLE;
      }
      break;
    // case byd::msg::perception::ObjectType::OBJECT_MINIVEHICLE:
    case byd::msg::perception::ObjectType::CAR:
      obstacle_type = OT_VEHICLE;
      break;
    case byd::msg::perception::ObjectType::TRUCK:
    case byd::msg::perception::ObjectType::TRAM:
      // case byd::msg::perception::ObjectType::OBJECT_CONSTRUCTION_VEHICLE:
      // case byd::msg::perception::ObjectType::OBJECT_SPECIALVEHICLE:
      obstacle_type = OT_LARGE_VEHICLE;
      break;
    // case byd::msg::perception::ObjectType::OBJECT_MOTORCYCLE:
    //   obstacle_type = OT_MOTORCYCLIST;
    //   break;
    case byd::msg::perception::ObjectType::CYCLIST:
      obstacle_type = OT_CYCLIST;
      break;
    case byd::msg::perception::ObjectType::PEDESTRIAN:
      obstacle_type = OT_PEDESTRIAN;
      break;
    case byd::msg::perception::ObjectType::TRICYCLE:
      obstacle_type = OT_TRICYCLIST;
      break;
    case byd::msg::perception::ObjectType::TRAFFIC_CONE:
    case byd::msg::perception::ObjectType::TRAFFIC_WARNING_POLE:
      obstacle_type = OT_CONE;  // 锥桶
      break;
    case byd::msg::perception::ObjectType::CRASH_BARRELS:
    case byd::msg::perception::ObjectType::WATER_HORSE:
    case byd::msg::perception::ObjectType::FIRE_HYDRANT:
      obstacle_type = OT_BARRIER;  // 屏障
      break;
    case byd::msg::perception::ObjectType::SPEED_RESTRICTION_BOARD:
    case byd::msg::perception::ObjectType::CONSTRUCTION_SIGN:
    case byd::msg::perception::ObjectType::TRIANGLE_BOARD:
      obstacle_type = OT_WARNING_TRIANGLE;  // 三角警示牌
      break;
      //  case byd::msg::perception::ObjectType::OBJECT_UNKNOWN_MOVABLE:
    case byd::msg::perception::ObjectType::ANIMAL:
      obstacle_type = OT_UNKNOWN_MOVABLE;
      break;
    case byd::msg::perception::ObjectType::TRAILER_HEAD:
    case byd::msg::perception::ObjectType::TRAILER_REAR:
    case byd::msg::perception::ObjectType::TRAFFIC_LIGHT:
      obstacle_type = OT_UNKNOWN_STATIC;
      break;
    case byd::msg::perception::ObjectType::ROW_OBSTACLES:
      obstacle_type = OT_ROW_OBSTACLES;
      break;
    default:
      obstacle_type = OT_UNKNOWN_MOVABLE;
      break;
  }
  return obstacle_type;
}

bool SignalCheck(
    const byd::msg::perception::PerceptionObstacle& perception_obstacle) {
  if (!std::isfinite(perception_obstacle.position().x()) ||
      !std::isfinite(perception_obstacle.position().y())) {
    LOG_ERROR << "Infinite value in perception_obstacle position, obstacle id: "
              << perception_obstacle.id()
              << "\t position.x = " << perception_obstacle.position().x()
              << "\t position.y = " << perception_obstacle.position().y();
    return false;
  }

  if (!std::isfinite(perception_obstacle.velocity().x()) ||
      !std::isfinite(perception_obstacle.velocity().y())) {
    LOG_ERROR << "Infinite value in perception_obstacle velocity, obstacle id: "
              << perception_obstacle.id()
              << "\t velocity.x = " << perception_obstacle.velocity().x()
              << "\t velocity.y = " << perception_obstacle.velocity().y();
    return false;
  }

  if (!std::isfinite(perception_obstacle.acceleration().x()) ||
      !std::isfinite(perception_obstacle.acceleration().y())) {
    LOG_ERROR
        << "Infinite value in perception_obstacle acceleration, obstacle id: "
        << perception_obstacle.id()
        << "\t acceleration.x = " << perception_obstacle.acceleration().x()
        << "\t acceleration.y = " << perception_obstacle.acceleration().y();
    return false;
  }

  if (!std::isfinite(perception_obstacle.heading_angle())) {
    LOG_ERROR
        << "Infinite value in perception_obstacle heading_angle, obstacle id: "
        << perception_obstacle.id()
        << "\t heading_angle = " << perception_obstacle.heading_angle();
    return false;
  }

  if (perception_obstacle.type() ==
          byd::msg::perception::ObjectType::GENERAL_OBJECT &&
      perception_obstacle.shape().points().size() > 2 &&
      perception_obstacle.polygon_only() == false) {
    for (const auto& point : perception_obstacle.shape().points()) {
      if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
        LOG_ERROR << "Infinite value in perception_obstacle shape points, "
                     "obstacle id: "
                  << perception_obstacle.id() << "\t point.x = " << point.x()
                  << "\t point.y = " << point.y();
        return false;
      }
    }
  }

  return true;
}
PredictionResult AdaptPredictionResult(
    const ad_byd::planning::prediction_type& prediction) {
  constexpr double kMinMovingSpeed = 0.6;  // 0.45
  PredictionResult prediction_result;
  auto& objects_proto = prediction_result.objects_proto;
  auto& objects_prediction_proto = prediction_result.objects_prediction_proto;
  objects_prediction_proto.mutable_header()->set_timestamp(
      (prediction.header().measurement_timestamp()) * 1e6);  // int64, us.
  objects_prediction_proto.mutable_header()->set_channel_seq_number(
      prediction.header().sequence_num());
  objects_proto.mutable_header()->set_timestamp(
      (prediction.header().measurement_timestamp()) * 1e6);  // int64, us.
  objects_proto.mutable_header()->set_channel_seq_number(
      prediction.header().sequence_num());
  objects_proto.set_scope(ObjectsProto::SCOPE_REAL);
  // LOG_ERROR << "\n";
  // LOG_ERROR << "prediction obstacles size: " << prediction.objects().size();
  for (const auto& obstacle : prediction.objects()) {
    const auto obj_iter = std::find_if(
        objects_proto.objects().begin(), objects_proto.objects().end(),
        [&obstacle](const ObjectProto& obj) -> bool {
          return obj.id() == absl::StrCat(obstacle.perception_obstacle().id());
        });
    bool is_new_obstacle = false;
    auto& perception_obstacle = obstacle.perception_obstacle();
    if (!SignalCheck(perception_obstacle)) {
      LOG_ERROR << "IGNORE obj: " << perception_obstacle.id()
                << " CONTINUE HERE";
      Log2DDS::LogDataV2(
          "object_filters",
          absl::StrCat("[msg_proxy] filtered obj_id:", perception_obstacle.id(),
                       " reason: SignalCheck NaN"));
      continue;
    }
    // ignore the polygon only object with type GENERAL OBSTACLE currently
    // ignore the frame dropped object by prediction
    // ignore the GENERAL_OBJECT obj not config to cnoa
    const bool general_not_cnoa =
        perception_obstacle.type() ==
            byd::msg::perception::ObjectType::GENERAL_OBJECT &&
        !(perception_obstacle.scenario_type() & 1U << 0);
    if (perception_obstacle.polygon_only() == true ||
        (obstacle.is_frame_dropped() && obstacle.trajectories().empty()) ||
        general_not_cnoa) {
      Log2DDS::LogDataV2(
          "object_filters",
          absl::StrCat(
              "[msg_proxy] filtered obj_id:", perception_obstacle.id(),
              " reason: polygon_only=", perception_obstacle.polygon_only(),
              " no traj and is_frame_dropped =", obstacle.is_frame_dropped(),
              " general_not_cnoa", general_not_cnoa));
      continue;
    }
    if (obj_iter == objects_proto.objects().end()) {  // Add new obstacle.
      is_new_obstacle = true;
      // Add new obstacle to objects_proto.
      auto object = objects_proto.add_objects();
      if (perception_obstacle.has_vision_attribute() &&
          perception_obstacle.vision_attribute().has_vehicle_subtype()) {
        object->set_subtype(static_cast<st::ObjectSubType>(
            perception_obstacle.vision_attribute().vehicle_subtype()));
      }
      object->set_is_frame_dropped(obstacle.is_frame_dropped());
      object->set_id(absl::StrCat(perception_obstacle.id()));
      object->set_type(AdaptObjectType(perception_obstacle.type(),
                                       obstacle.trajectories().empty()));
      object->set_timestamp((prediction.header().measurement_timestamp()));
      object->mutable_pos()->set_x(perception_obstacle.position().x());
      object->mutable_pos()->set_y(perception_obstacle.position().y());
      object->mutable_vel()->set_x(perception_obstacle.velocity().x());
      object->mutable_vel()->set_y(perception_obstacle.velocity().y());
      object->mutable_accel()->set_x(perception_obstacle.acceleration().x());
      object->mutable_accel()->set_y(perception_obstacle.acceleration().y());
      object->set_yaw(perception_obstacle.heading_angle());
      object->set_yaw_rate(perception_obstacle.heading_rate());
      object->mutable_vision_attribute()->set_ola_type(AdaptObjectOlaType(
          perception_obstacle.vision_attribute().ola_type()));
      object->mutable_vision_attribute()->set_ola_conf(
          perception_obstacle.vision_attribute().ola_conf());
      object->mutable_vision_attribute()->set_lane_attr_type(
          AdaptObjectLaneAttrType(
              perception_obstacle.vision_attribute().lane_attr_type()));
      object->mutable_vision_attribute()->set_lane_attr_conf(
          perception_obstacle.vision_attribute().lane_attr_conf());
      object->set_fusion_source(
          static_cast<st::FusionSource>(perception_obstacle.fusion_source()));
      // add obstacle light, reserved
      if (object->type() == ObstacleType::OT_VEHICLE ||
          object->type() == ObstacleType::OT_LARGE_VEHICLE) {
        object->mutable_obstacle_light()->set_left_turn_lights(
            AdaptObjectAttrLeftLightType(
                perception_obstacle.vision_attribute().turn_light_state()));
        object->mutable_obstacle_light()->set_right_turn_lights(
            AdaptObjectAttrRightLightType(
                perception_obstacle.vision_attribute().turn_light_state()));
        if (object->type() == ObstacleType::OT_VEHICLE) {
          object->mutable_obstacle_light()->set_brake_lights(
              AdaptObjectAttrBrakeLightType(
                  perception_obstacle.vision_attribute().brake_light_state()));
        }
        if (perception_obstacle.type() !=
            byd::msg::perception::ObjectType::TRAM) {
          object->mutable_obstacle_light()->set_hazard_lights(
              AdaptObjectAttrHazardLightType(
                  perception_obstacle.vision_attribute().turn_light_state()));
        }
        object->mutable_obstacle_light()->set_left_turn_lights_conf(
            perception_obstacle.vision_attribute().turn_light_state_conf());
        object->mutable_obstacle_light()->set_right_turn_lights_conf(
            perception_obstacle.vision_attribute().turn_light_state_conf());
        object->mutable_obstacle_light()->set_brake_lights_conf(
            perception_obstacle.vision_attribute().brake_light_state_conf());
        // object->mutable_obstacle_light()->set_hazard_lights_conf(
        //     obstacle.obstacle_light().hazard_lights_conf());

        // LOG_ERROR << "object: " << object->id()
        //           << " obstacle light : " <<
        //           (int)obstacle.obstacle_light().left_turn_lights()
        //           << " " <<
        //           (int)obstacle.obstacle_light().right_turn_lights()
        //           << " " <<  (int)obstacle.obstacle_light().brake_lights()
        //           << " " << (int)obstacle.obstacle_light().hazard_lights();
      }
      // Set object bounding_box.
      object->mutable_bounding_box()->set_x(perception_obstacle.position().x());
      object->mutable_bounding_box()->set_y(perception_obstacle.position().y());
      object->mutable_bounding_box()->set_heading(
          perception_obstacle.heading_angle());
      object->set_parked(false);
      object->set_observation_state(OS_PARTIALLY_OBSERVED);

      // Set object height.
      object->set_max_z(perception_obstacle.height());
      object->set_min_z(0.0);
      object->set_ground_z(0.0);

      // use polygon of objtype == GENERAL_OBJECT currently
      if (perception_obstacle.shape().points().size() > 2 &&
          perception_obstacle.polygon_only() == false) {
        for (const auto& point : perception_obstacle.shape().points()) {
          auto contour = object->add_contour();
          contour->set_x(point.x());
          contour->set_y(point.y());
        }

        Polygon2d objcontour;
        std::vector<Vec2d> vertices;
        for (const auto& point : object->contour()) {
          vertices.push_back(Vec2d(point.x(), point.y()));
        }
        CHECK_GT(vertices.size(), 2);
        objcontour = Polygon2d(std::move(vertices), /*is_convex=*/true);
        Box2d bounding_box;
        bounding_box = objcontour.BoundingBoxWithHeading(
            perception_obstacle.heading_angle());
        double width = bounding_box.width() < 0.01 ? 0.2 : bounding_box.width();
        double length = bounding_box.length();
        if (length < 0.01) {
          length = object->type() == ObstacleType::OT_PEDESTRIAN ? 0.2 : 1.0;
        }
        object->mutable_bounding_box()->set_length(length);
        object->mutable_bounding_box()->set_width(width);
        
        Log2DDS::LogLineV2(absl::StrCat(obstacle.id(), "_polygon"),
                          Log2DDS::kOrange, {},
                          objcontour.points());
      } else {
        double width = perception_obstacle.width() < 0.01
                           ? 0.2
                           : perception_obstacle.width();
        double length = perception_obstacle.length();
        if (length < 0.01) {
          length = object->type() == ObstacleType::OT_PEDESTRIAN ? 0.2 : 1.0;
        }
        object->mutable_bounding_box()->set_length(length);
        object->mutable_bounding_box()->set_width(width);

        const Vec2d obstacle_center(perception_obstacle.position().x(),
                                    perception_obstacle.position().y());
        Box2d obstacle_box(obstacle_center, perception_obstacle.heading_angle(),
                           length, width);
        // obstacle.corner_points()  empty.
        const std::vector<Box2d::Corner> points_index{
            Box2d::FRONT_LEFT, Box2d::REAR_LEFT, Box2d::REAR_RIGHT,
            Box2d::FRONT_RIGHT};  //  counterclockwise.
        for (const Box2d::Corner& point_index : points_index) {
          auto contour = object->add_contour();
          const Vec2d corner_point = obstacle_box.GetCorner(point_index);
          contour->set_x(corner_point.x());
          contour->set_y(corner_point.y());
        }
      }
      /*
      bool use_corner_pt = false;
      bool is_convex = false;
      bool is_static =
          perception_obstacle.velocity().x() < 1e-6 &&
      perception_obstacle.velocity().y() < 1e-6;
      //is_static |= obstacle.obstacle_state() & 0x01;
      if ((object->type() ==
               ObstacleType::OBJECT_UNKNOWN_UNMOVABLE ||
           is_static) &&
          obstacle.corner_points().size() > 2u) {
        std::vector<Vec2d> points;
        for (const auto& corner_pt : obstacle.corner_points()) {
          Vec2d point(corner_pt.x(), corner_pt.y());
          points.emplace_back(std::move(point));
        }
        Polygon2d corner_polygon(points, 1);
        is_convex = corner_polygon.is_convex();
        if (is_convex) {
          Log2DDS::LogDataV0("convex_id", obstacle.id());
          for (const auto& point : points) {
            auto contour = object->add_contour();
            contour->set_x(point.x());
            contour->set_y(point.y());
          }
          use_corner_pt = true;
        }
      }
      if (!use_corner_pt || !is_convex) {
        for (const Box2d::Corner& point_index : points_index) {
          auto contour = object->add_contour();
          const Vec2d corner_point = obstacle_box.GetCorner(point_index);
          contour->set_x(corner_point.x());
          contour->set_y(corner_point.y());
        }
      }
      */
    }
    // Add object_prediction_proto.
    auto object_prediction_proto = objects_prediction_proto.add_objects();
    object_prediction_proto->set_id(
        absl::StrCat(obstacle.perception_obstacle().id()));
    ObjectProto perception_object =
        is_new_obstacle
            ? objects_proto.objects(objects_proto.objects().size() - 1)
            : *obj_iter;
    object_prediction_proto->mutable_perception_object()->CopyFrom(
        std::move(perception_object));
    // Set obstacle trajectory.
    const double obstacle_speed =
        std::sqrt(perception_obstacle.velocity().x() *
                      perception_obstacle.velocity().x() +
                  perception_obstacle.velocity().y() *
                      perception_obstacle.velocity().y());
    // const bool is_stationary = obstacle_speed < kMinMovingSpeed ? true :
    // false;
    // constexpr double kMinBicycleSpeed = 0.5;  //1e-6
    // constexpr double kMinPedestrainMovingSpeed = 0.5;  //1e-6

    // double min_stationary_speed = kMinMovingSpeed;

    // const auto& obstacle_type = perception_obstacle.type();
    // if (obstacle_type == byd::msg::perception::ObjectType::CYCLIST ||
    //     obstacle_type == byd::msg::perception::ObjectType::TRICYCLE) {
    //   min_stationary_speed = kMinBicycleSpeed;
    // } else if (obstacle_type == byd::msg::perception::ObjectType::PEDESTRIAN)
    // {
    //   min_stationary_speed = kMinPedestrainMovingSpeed;
    // }

    // bool is_stationary = obstacle_speed < min_stationary_speed ? true :
    // false;
    bool is_stationary = obstacle.trajectories().empty();
    // if (obstacle_type == ObstacleType::OBJECT_BICYCLE) {
    //   is_stationary = true;
    // }
    if (obstacle.trajectories().size() >= 1) {
      int i = 0;
      // for (int i = 0; i < obstacle.trajectories().size(); ++i) {
      const auto& trajectory = obstacle.trajectories()[i];
      auto obstacle_trajectory = object_prediction_proto->add_trajectories();
      obstacle_trajectory->set_intention(
          AdaptTrajectoryIntention(trajectory.intention()));
      // obstacle_trajectory->set_probability(trajectory.probability());
      obstacle_trajectory->set_probability(1.0);
      if (is_stationary) {
        obstacle_trajectory->set_type(PT_STATIONARY);
      } else {
        obstacle_trajectory->set_type(PT_VEHICLE_LANE_FOLLOW);
      }
      obstacle_trajectory->set_index(i);
      obstacle_trajectory->set_is_reversed(false);
      double s = 0.0;
      for (int i = 0; i < trajectory.trajectory_point().size(); ++i) {
        const auto& point = trajectory.trajectory_point()[i];
        if (!std::isfinite(point.path_point().x()) ||
            !std::isfinite(point.path_point().y())) {
          LOG_ERROR << "Infinite value in prediction trajectory, obstacle id: "
                    << obstacle.id()
                    << "\t point.x = " << point.path_point().x()
                    << "\t point.y = " << point.path_point().y();
          continue;
        }
        auto trajectory_point = obstacle_trajectory->add_points();
        trajectory_point->mutable_pos()->set_x(point.path_point().x());
        trajectory_point->mutable_pos()->set_y(point.path_point().y());
        if (i > 0) {
          const double dx =
              point.path_point().x() -
              trajectory.trajectory_point()[i - 1].path_point().x();
          const double dy =
              point.path_point().y() -
              trajectory.trajectory_point()[i - 1].path_point().y();
          const double dist = dx * dx + dy * dy;
          s += std::sqrt(std::max(dist, 0.0));
        }
        trajectory_point->set_s(s);
        trajectory_point->set_theta(point.path_point().theta());
        trajectory_point->set_kappa(point.path_point().kappa());
        // Relative to first point.
        trajectory_point->set_t(
            point.relative_time() -
            trajectory.trajectory_point()[0].relative_time());
        trajectory_point->set_v(point.v());
        trajectory_point->set_a(point.a());
      }
    }
    // If the stationary obstacle does not have any trajectory,
    // mock a trajectory with one point.
    if (obstacle.trajectories().empty()) {
      auto obstacle_trajectory = object_prediction_proto->add_trajectories();
      obstacle_trajectory->set_probability(1.0);
      if (is_stationary) {
        obstacle_trajectory->set_type(PT_STATIONARY);
      } else {
        obstacle_trajectory->set_type(PT_VEHICLE_LANE_FOLLOW);
      }
      obstacle_trajectory->set_intention(
          TrajectoryIntention::INTENTION_UNKNOWN);
      obstacle_trajectory->set_index(0);
      obstacle_trajectory->set_is_reversed(false);
      auto trajectory_point = obstacle_trajectory->add_points();
      trajectory_point->mutable_pos()->set_x(
          perception_obstacle.position().x());
      trajectory_point->mutable_pos()->set_y(
          perception_obstacle.position().y());
      trajectory_point->set_s(0);
      trajectory_point->set_theta(perception_obstacle.heading_angle());
      trajectory_point->set_kappa(0.0);
      trajectory_point->set_t(0.0);
      trajectory_point->set_v(0.0);
      trajectory_point->set_a(0.0);
    }
  }

  // LOG_ERROR << "prediction objects size: " <<
  // prediction_result.objects_prediction_proto.objects().size(); LOG_ERROR <<
  // "perception objects size: " <<
  // prediction_result.objects_proto.objects().size(); for (size_t i = 0; i <
  // prediction_result.objects_prediction_proto.objects().size(); ++i) {
  //   LOG_ERROR << "prediction object id: " <<
  //   prediction_result.objects_prediction_proto.objects().at(i).id();
  //   LOG_ERROR << "perception object id: " <<
  //   prediction_result.objects_proto.objects().at(i).id();
  //   // auto perception_object =
  //   prediction_result.objects_proto.objects().at(i);
  //   // LOG_ERROR << "prediction id: " << predition_object.id() << ",
  //   perception id: " << perception_object.id();
  //   // LOG_ERROR << "prediction trajectory size: " <<
  //   predition_object.trajectories().size() << ", "
  //   //            << predition_object.perception_object().vel().x() << ", "
  //   //            << predition_object.perception_object().vel().y();
  //   // if (predition_object.perception_object().vel().x() > 1.0) {
  //   //   LOG_ERROR << "input obstacle id: " <<
  //   obstacle.perception_object().id();
  //   //   LOG_ERROR << "input obstacle pos: " <<
  //   obstacle.perception_object().pos().x() << ", " <<
  //   obstacle.perception_object().pos().y();
  //   //   break;
  //   // }
  // }
  return prediction_result;
}

void ComputeObjectsStopTime(
    PredictionResult& prediction_result,
    absl::flat_hash_map<std::string, PlannerState::ObjectStopTimeResult>&
        object_stop_time_map) {
  constexpr double kMinMovingSpeedThes = 0.6;
  for (const auto& object :
       prediction_result.objects_prediction_proto.objects()) {
    const auto& obj_id = object.id();
    double speed = std::sqrt(object.perception_object().vel().x() *
                                 object.perception_object().vel().x() +
                             object.perception_object().vel().y() *
                                 object.perception_object().vel().y());
    std::string speed_debug = absl::StrCat("id: ", obj_id, " speed: ", speed);
    Log2DDS::LogDataV2("speed_debug", speed_debug);
    if (object_stop_time_map.find(object.id()) == object_stop_time_map.end()) {
      PlannerState::ObjectStopTimeResult stop_time;
      stop_time.last_time = object.perception_object().timestamp();
      stop_time.time_duration_brake_light = 0.0;
      stop_time.previous_time_duration_brake_light = 0.0;
      stop_time.time_duration_none_brake_light = 0.1;
      stop_time.last_hazard_light_time = 1000.0;
      if (object.has_perception_object() &&
          object.perception_object().has_obstacle_light()) {
        if (object.perception_object().obstacle_light().has_brake_lights()) {
          if (object.perception_object().obstacle_light().brake_lights() ==
              st::ObstacleLightType::LIGHT_ON) {
            stop_time.time_duration_brake_light = 0.1;
            stop_time.previous_time_duration_brake_light = 0.1;
            stop_time.time_duration_none_brake_light = 0.0;
          }
        }
        if (object.perception_object().obstacle_light().has_hazard_lights() &&
            object.perception_object().obstacle_light().hazard_lights() ==
                st::ObstacleLightType::LIGHT_ON) {
          stop_time.last_hazard_light_time = 0.0;
        }
      }
      // if (speed < kMinMovingSpeedThes) {
      if ((object.trajectories().size() >= 1 &&
           object.trajectories()[0].type() == PT_STATIONARY) ||
          speed < kMinMovingSpeedThes) {
        stop_time.time_duration_since_stop = 0.1;
        stop_time.previous_stop_time_duration = 0.1;
        stop_time.last_move_time_duration = 0.0;
        stop_time.accumulated_high_speed_duration = 0.0;
      } else {
        stop_time.time_duration_since_stop = 0.0;
        stop_time.previous_stop_time_duration = 0.0;
        stop_time.last_move_time_duration = 0.1;
        stop_time.accumulated_high_speed_duration = 0.1;
      }
      object_stop_time_map[object.id()] = stop_time;
    } else {
      if (object.has_perception_object() &&
          object.perception_object().has_obstacle_light()) {
        if (object.perception_object().obstacle_light().has_brake_lights()) {
          if (object.perception_object().obstacle_light().brake_lights() ==
              st::ObstacleLightType::LIGHT_ON) {
            double last_brake_light_duration_time =
                object_stop_time_map[object.id()]
                    .time_duration_brake_light;
            double previous_brake_light_duration_time =
                object_stop_time_map[object.id()]
                    .previous_time_duration_brake_light;
            object_stop_time_map[object.id()].time_duration_brake_light =
                last_brake_light_duration_time +
                std::fmax(0.0, object.perception_object().timestamp() -
                                   object_stop_time_map[object.id()].last_time);
            object_stop_time_map[object.id()]
                .previous_time_duration_brake_light =
                previous_brake_light_duration_time +
                std::fmax(0.0, object.perception_object().timestamp() -
                                   object_stop_time_map[object.id()].last_time);
          } else {
            double last_brake_light_duration_time =
                object_stop_time_map[object.id()]
                    .time_duration_none_brake_light;
            object_stop_time_map[object.id()].time_duration_brake_light = 0.0;
            object_stop_time_map[object.id()].time_duration_none_brake_light =
                last_brake_light_duration_time +
                std::fmax(0.0, object.perception_object().timestamp() -
                                   object_stop_time_map[object.id()].last_time);
          }
        }
        if (object.perception_object().obstacle_light().has_hazard_lights() &&
            object.perception_object().obstacle_light().hazard_lights() ==
                st::ObstacleLightType::LIGHT_ON) {
          object_stop_time_map[object.id()].last_hazard_light_time = 0.0;
        } else {
          double last_hazard_light_time =
              object_stop_time_map[object.id()].last_hazard_light_time;
          object_stop_time_map[object.id()].last_hazard_light_time =
              last_hazard_light_time +
              std::fmax(0.0, object.perception_object().timestamp() -
                                 object_stop_time_map[object.id()].last_time);
        }
      }
      if ((object.trajectories().size() >= 1 &&
           object.trajectories()[0].type() == PT_STATIONARY) ||
          speed < kMinMovingSpeedThes) {
        double last_stop_duration_time =
            object_stop_time_map[object.id()].time_duration_since_stop;
        double previous_stop_time_duration =
            object_stop_time_map[object.id()].previous_stop_time_duration;
        object_stop_time_map[object.id()].time_duration_since_stop =
            last_stop_duration_time +
            std::fmax(0.0, object.perception_object().timestamp() -
                               object_stop_time_map[object.id()].last_time);
        object_stop_time_map[object.id()].previous_stop_time_duration =
            previous_stop_time_duration +
            std::fmax(0.0, object.perception_object().timestamp() -
                               object_stop_time_map[object.id()].last_time);
      } else {
        double last_move_time_duration =
            object_stop_time_map[object.id()].last_move_time_duration;
        object_stop_time_map[object.id()].time_duration_since_stop = 0.0;
        object_stop_time_map[object.id()].last_move_time_duration =
            last_move_time_duration +
            std::fmax(0.0, object.perception_object().timestamp() -
                               object_stop_time_map[object.id()].last_time);
        if (object.has_perception_object() &&
            object.perception_object().has_vel() &&
            std::sqrt(object.perception_object().vel().x() *
                          object.perception_object().vel().x() +
                      object.perception_object().vel().y() *
                          object.perception_object().vel().y()) > 3.0) {
          double accumulated_high_speed_duration =
              object_stop_time_map[object.id()].accumulated_high_speed_duration;
          object_stop_time_map[object.id()].accumulated_high_speed_duration =
              accumulated_high_speed_duration +
              std::fmax(0.0, object.perception_object().timestamp() -
                                 object_stop_time_map[object.id()].last_time);
        }
      }
      object_stop_time_map[object.id()].last_time =
          object.perception_object().timestamp();
    }
  }
  // refresh object_stop_time_map
  auto& prediction_proto = prediction_result.objects_prediction_proto;
  for (auto itr = object_stop_time_map.begin();
       itr != object_stop_time_map.end();) {
    auto obj_iter = prediction_proto.mutable_objects()->begin();
    for (; obj_iter != prediction_proto.mutable_objects()->end(); obj_iter++) {
      if (obj_iter->id() == itr->first) {
        break;
      }
    }
    if (obj_iter == prediction_proto.mutable_objects()->end()) {
      object_stop_time_map.erase(itr++);
    } else {
      obj_iter->mutable_stop_time()->set_time_duration_since_stop(
          itr->second.time_duration_since_stop);
      obj_iter->mutable_stop_time()->set_previous_stop_time_duration(
          itr->second.previous_stop_time_duration);
      obj_iter->mutable_stop_time()->set_last_move_time_duration(
          itr->second.last_move_time_duration);
      obj_iter->mutable_stop_time()->set_accumulated_high_speed_duration(
          itr->second.accumulated_high_speed_duration);
      obj_iter->mutable_stop_time()->set_time_duration_brake_light(
          itr->second.time_duration_brake_light);
      obj_iter->mutable_stop_time()->set_previous_time_duration_brake_light(
          itr->second.previous_time_duration_brake_light);
      obj_iter->mutable_stop_time()->set_time_duration_none_brake_light(
          itr->second.time_duration_none_brake_light);
      obj_iter->mutable_stop_time()->set_last_hazard_light_time(
          itr->second.last_hazard_light_time);
      ++itr;
      std::string value =
          "obs-id: " + obj_iter->id() + " time_stop: " +
          absl::StrCat(obj_iter->stop_time().time_duration_since_stop())
              .substr(0, 4) +
          " previous_stop: " +
          absl::StrCat(obj_iter->stop_time().previous_stop_time_duration())
              .substr(0, 4) +
          " last_move: " +
          absl::StrCat(obj_iter->stop_time().last_move_time_duration())
              .substr(0, 4) +
          " last_high_speed: " +
          absl::StrCat(obj_iter->stop_time().accumulated_high_speed_duration())
              .substr(0, 4) +
          " last_brake_light: " +
          absl::StrCat(
              obj_iter->stop_time().previous_time_duration_brake_light())
              .substr(0, 4) +
          " brake_light_duration: " +
          absl::StrCat(obj_iter->stop_time().time_duration_brake_light())
              .substr(0, 4) +
          " none_brake_light_duration: " +
          absl::StrCat(obj_iter->stop_time().time_duration_none_brake_light())
              .substr(0, 4) +
          " hazard_time: " +
          absl::StrCat(obj_iter->stop_time().last_hazard_light_time())
              .substr(0, 4);
      Log2DDS::LogDataV0("obs_stop_time", value);
    }
  }
}

bool ShouldResetAlcState(ResetReasonProto::Reason reset_reason) {
  switch (reset_reason) {
    case ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND:
    case ResetReasonProto::PREV_NOW_POINT_NOT_FOUND:
    case ResetReasonProto::REROUTED:
    case ResetReasonProto::NON_AUTONOMY:
    case ResetReasonProto::FIRST_ENGAGE:
    case ResetReasonProto::FULL_STOP:
    case ResetReasonProto::RECOVER_FROM_AEB:
    case ResetReasonProto::NEW_FREESPACE_PATH:
    case ResetReasonProto::SPEED_ONLY:
    case ResetReasonProto::SPEED_ONLY_ENGAGE:
      return true;
    case ResetReasonProto::NONE:
    case ResetReasonProto::LON_ERROR_TOO_LARGE:
    case ResetReasonProto::LAT_ERROR_TOO_LARGE:
    case ResetReasonProto::STEER_ONLY_ENGAGE:
    case ResetReasonProto::STEER_ONLY:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

SceneReasoningOutput RunSceneReasoningAndFillDebug(
    const PlannerSemanticMapManager& psmm,
    const ad_byd::planning::TrafficLightStatusMap& tl_info_map,
    const PlannerObjectManager& obj_mgr,
    const ObjectsPredictionProto& prediction,
    const ApolloTrajectoryPointProto& plan_start_point, ThreadPool* thread_pool,
    ObjectHistoryManager& obj_his_manager) {
  // const auto lane_paths_or =
  //     BuildLocalMap(psmm, route_sections, route_navi_info);

  const auto lane_paths_or = BuildLanePathsFormPsmm(psmm, plan_start_point);
  if (!lane_paths_or.ok()) {
    LOG_WARN
        << "RunSceneReasoningAndFillDebug failed in BuildLanePathsFormPsmm: "
        << lane_paths_or.status();
    return SceneReasoningOutput();
  }
  auto scene_reasoning_output_or = RunSceneReasoning(
      SceneReasoningInput{.psmm = &psmm,
                          .prediction = &prediction,
                          .tl_info_map = &tl_info_map,
                          .lane_paths = &(*lane_paths_or),
                          .plan_start_point = &plan_start_point},
      thread_pool, obj_his_manager);

  if (!scene_reasoning_output_or.ok()) {
    LOG_WARN << "RunSceneReasoningAndFillDebug failed in RunSceneReasoning: "
             << scene_reasoning_output_or.status();
    return SceneReasoningOutput();
  }
  // const auto& scene_output_proto =
  //     scene_reasoning_output_or->scene_output_proto;

  // *debug_proto->mutable_scene_understanding_debug() = scene_output_proto;
  // Record stalled objects.
  //
  //
  // ParseObjectAnnotationToDebugProto(scene_output_proto.objects_annotation(),
  //                                   obj_mgr, debug_proto);
  // Record traffic waiting objects.
  // ParseTrafficWaitingQueueToDebugProto(
  //     scene_output_proto.traffic_waiting_queue(), obj_mgr, debug_proto);
  return *scene_reasoning_output_or;
}

absl::StatusOr<std::vector<mapping::LanePath>> BuildLanePathsFormPsmm(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point) {
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const auto& map = psmm.map_ptr();
  if (!map || !map->route()) {
    return absl::NotFoundError("map_ptr or route is nullptr.");
  }
  // 0. get lane ids in current section
  std::vector<uint64_t> navi_start_lanes;
  auto& navi_start = map->route()->navi_start();
  if (!(navi_start.section_id == 0)) {
    const auto& navi_section = map->GetSectionById(navi_start.section_id);
    if (navi_section) {
      for (auto& id : navi_section->lanes()) {
        navi_start_lanes.push_back(id);
      }
    }
  }
  // 1. Get all lane_squences
  std::vector<mapping::LanePath> all_lane_paths;
  for (int i = 0; i < navi_start_lanes.size(); i++) {
    const auto& lane = map->GetLaneById(navi_start_lanes[i]);
    if (lane && lane->is_navigation() && lane->IsValid() &&
        lane->type() != ad_byd::planning::LaneType::LANE_DIVERSION) {
      std::vector<std::vector<ad_byd::planning::LaneConstPtr>> lane_seqs;
      map->GetAllLaneSequences(lane, lane_seqs);
      double max_lane_seq_length = std::numeric_limits<double>::min();
      std::vector<ad_byd::planning::LaneConstPtr> max_lane_seq =
          *lane_seqs.begin();
      std::vector<uint64_t> lane_ids;
      for (const auto& lane_seq : lane_seqs) {
        double length = 0.0;
        for (const auto& lane_ptr : lane_seq) {
          if (lane_ptr->IsValid()) length += lane_ptr->topo_length();
        }
        if (length > max_lane_seq_length) {
          max_lane_seq_length = length;
          max_lane_seq = lane_seq;
        }
      }
      for (const auto& lane_ptr : max_lane_seq) {
        if (lane_ptr->IsValid()) lane_ids.emplace_back(lane_ptr->id());
      }
      // all_lane_seqs.insert(all_lane_seqs.end(), lane_seqs.begin(),
      //                     lane_seqs.end());
      st::mapping::LanePath lane_path(map, lane_ids, 0.0, 1.0);
      all_lane_paths.emplace_back(std::move(lane_path));
    }
  }
  if (all_lane_paths.empty()) {
    return absl::NotFoundError(" all_lane_seqs is nullptr.");
  }
  return all_lane_paths;
}

double GetLaneLength(const std::vector<Vec2d>& points) {
  if (points.empty()) return 0.0;
  double s = 0.0;
  std::vector<double> accumulated_s;
  accumulated_s.emplace_back(s);
  for (int i = 0; i < points.size(); ++i) {
    int last_index = std::max(0, i - 1);
    auto ds = std::hypot(points[i].x() - points[last_index].x(),
                         points[i].y() - points[last_index].y());
    s += ds;
    accumulated_s.emplace_back(s);
  }
  return accumulated_s.back();
}
}  // namespace
}  // namespace planning
}  // namespace st
