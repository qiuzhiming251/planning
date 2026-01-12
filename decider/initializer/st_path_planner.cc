
#include <algorithm>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "plan_common/log.h"

#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"

#include "plan_common/async/thread_pool.h"
#include "plan_common/drive_passage.h"
#include "plan_common/gflags.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/vec.h"
#include "plan_common/planner_status.h"
#include "plan_common/timer.h"
#include "plan_common/trajectory_optimizer_state.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/trajectory_util.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/util/file_util.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/util/planner_status_macros.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "plan_common/planning_macros.h"

#include "decider/initializer/initializer_input.h"
#include "decider/initializer/search_motion.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/decider_output.h"
#include "object_manager/st_inference/initializer_output.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "object_manager/st_inference/trajectory_optimizer_input.h"
#include "object_manager/st_inference/trajectory_optimizer_output.h"

#include "decider/initializer/st_path_planner.h"
#include "plan_common/util/format_numeric_string.h"

namespace st::planning {

namespace {

absl::Status AddSpaceTimePlannerTrajectoryById(
    const SpacetimeTrajectoryManager& traj_mgr, std::string traj_id,
    SpacetimePlannerObjectTrajectoryReason::Type reason,
    SpacetimePlannerObjectTrajectories* res) {
  if (!res->trajectory_ids.contains(traj_id)) {
    const auto* traj = traj_mgr.FindTrajectoryById(traj_id);
    if (traj == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("Could not find trajectory ", traj_id));
    }
    ASSIGN_OR_RETURN(
        auto truncated_traj,
        traj->CreateTruncatedCopy(res->st_start_offset,
                                  kSpacetimePlannerTrajectoryHorizon));
    res->trajectories.push_back(std::move(truncated_traj));
    res->trajectory_infos.push_back(
        {.traj_index = traj->traj_index(),
         .object_id = traj->planner_object().is_sim_agent()
                          ? traj->planner_object().base_id()
                          : traj->planner_object().id(),
         .reason = reason});
    res->trajectory_ids.insert(std::string(traj_id));
  }
  return absl::OkStatus();
}

// std::vector<TrajectoryPoint> ConvertCaptainTrajectoryToOptimizerInput(
//     int trajectory_steps, double trajectory_time_step, double shift_time,
//     absl::Span<const ApolloTrajectoryPointProto> captain_traj) {
//   CHECK_GT(trajectory_steps, 0);
//   CHECK_GT(trajectory_time_step, 0.0);
//   if (captain_traj.empty()) return {};
//   std::vector<ApolloTrajectoryPointProto> sampled_traj;
//   for (int i = 0; i < trajectory_steps; ++i) {
//     const double t = shift_time + trajectory_time_step * i;
//     if (t > captain_traj.back().relative_time()) break;
//     ApolloTrajectoryPointProto interp_point = QueryApolloTrajectoryPointByT(
//         captain_traj.begin(), captain_traj.end(), t);
//     // Time alignment.
//     interp_point.set_relative_time(trajectory_time_step * i);
//     sampled_traj.push_back(interp_point);
//   }
//   return ToTrajectoryPoint(sampled_traj);
// }

void ModifySLBoundaryByDPPath(
    const DrivePassage& passage, const FrenetBox& ego_frenet_box,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const std::vector<TrajectoryPoint>& dp_traj, PathSlBoundary* sl_boundary) {
  CHECK_NOTNULL(sl_boundary);
  // coordinate transform - dp path
  std::vector<FrenetCoordinate> dp_frenet_points;
  dp_frenet_points.reserve(dp_traj.size());
  for (const auto& dp_pt : dp_traj) {
    const auto frenet_pt =
        passage.QueryLaterallyUnboundedFrenetCoordinateAt(dp_pt.pos());
    if (!frenet_pt.ok()) return;
    dp_frenet_points.emplace_back(*frenet_pt);
  }
  // traverse obstacle set
  const double front_to_rear_axle = 4.0;
  const double rear_to_rear_axle = 1.1;
  const double half_ego_width = 1.0;
  for (const auto& trajectory : st_planner_object_traj.trajectories) {
    if (trajectory.is_stationary()) continue;
    ObjectType object_type = trajectory.object_type();
    // longitudinal condition
    const auto object_frenet_box =
        passage.QueryFrenetBoxAtContour(trajectory.planner_object().contour());
    if (!object_frenet_box.ok()) {
      // std::cout << "----- object frenet box invalid. object id: "
      //           << trajectory.planner_object().id() << std::endl;
      continue;
    }
    if (object_frenet_box->s_max < ego_frenet_box.s_min ||
        object_frenet_box->s_min > sl_boundary->end_s()) {
      continue;
    }
    // static object

    // vulnerable road user
    if (object_type == OT_MOTORCYCLIST || object_type == OT_TRICYCLIST ||
        object_type == OT_CYCLIST || object_type == OT_PEDESTRIAN) {
      // coordinate transform - object state
      // std::vector<FrenetBox> ob_fboxes;
      // ob_fboxes.reserve(trajectory.states().size());
      double ob_l_min = DBL_MAX, ob_l_max = -DBL_MAX;
      double s_catchup = 0.0, s_overtake = 0.0;
      bool is_bypass = true, is_catchup = false;
      int bypass_direction = 0, idx_dp_pt = 0;
      for (const auto& state : trajectory.states()) {
        const auto frenet_box = passage.QueryFrenetBoxAt(state.box);
        if (!frenet_box.ok() || !state.traj_point) continue;
        // ob_fboxes.emplace_back(frenet_box.value());
        const double ob_traj_t = state.traj_point->t();
        // time alignment
        while (idx_dp_pt < dp_traj.size()) {
          if (dp_traj[idx_dp_pt].t() > ob_traj_t) break;
          idx_dp_pt++;
        }
        if (idx_dp_pt >= dp_traj.size()) break;
        double s_dp_align = dp_frenet_points[idx_dp_pt].s;
        double l_dp_align = dp_frenet_points[idx_dp_pt].l;
        if (idx_dp_pt > 0) {
          double t_factor =
              (dp_traj[idx_dp_pt].t() - ob_traj_t) /
              (dp_traj[idx_dp_pt].t() - dp_traj[idx_dp_pt - 1].t());
          l_dp_align -= (dp_frenet_points[idx_dp_pt].l -
                         dp_frenet_points[idx_dp_pt - 1].l) *
                        t_factor;
          s_dp_align -= (dp_frenet_points[idx_dp_pt].s -
                         dp_frenet_points[idx_dp_pt - 1].s) *
                        t_factor;
        }
        // encounter check
        if (s_dp_align - rear_to_rear_axle > frenet_box->s_max) {
          s_overtake = s_dp_align + 1.0 * dp_traj[idx_dp_pt].v();
          break;
        }
        if (s_dp_align + front_to_rear_axle > frenet_box->s_min) {
          if (!is_catchup) {
            s_catchup = std::fmax(s_dp_align - 1.0 * dp_traj[idx_dp_pt].v(),
                                  object_frenet_box->s_min);
            s_overtake = s_dp_align + 1.0 * dp_traj[idx_dp_pt].v();
            is_catchup = true;
          }
          if (l_dp_align - half_ego_width > frenet_box->l_max &&
              object_frenet_box->center_l() < ego_frenet_box.center_l()) {
            ob_l_max = object_frenet_box->l_max;
            bypass_direction = 1;
          } else if (l_dp_align + half_ego_width < frenet_box->l_min &&
                     object_frenet_box->center_l() >
                         ego_frenet_box.center_l()) {
            ob_l_min = object_frenet_box->l_min;
            bypass_direction = -1;
          } else {
            is_bypass = false;
            break;
          }
        }
      }
      // update s-l boundary
      if (is_catchup && is_bypass) {
        const double bound_buffer = 0.1;
        if (bypass_direction == 1) {
          sl_boundary->ModifyHardBoundByDpLabel(passage, s_catchup, s_overtake,
                                                ob_l_max + bound_buffer, true);
        }
        // else if (bypass_direction == -1) {
        //   sl_boundary->ModifyHardBoundByDpLabel(passage, s_catchup,
        //   s_overtake,
        //                                         ob_l_min - bound_buffer,
        //                                         false);
        // }
      }
    }
  }
}

bool IfMissNaviScenario(const SchedulerOutput& scheduler_result, Vec2d ego_pos,
                        int plan_id) {
  auto lane_seq_info = scheduler_result.drive_passage.lane_seq_info();
  if (!lane_seq_info) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "no input");
    return false;
  }
  if (lane_seq_info->dist_to_junction < 5.0) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "near junction");
    return false;
  }
  if (lane_seq_info->lc_num != 1 && lane_seq_info->lc_num != 2) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) +
                           absl::StrCat("lc num: ", lane_seq_info->lc_num));
    return false;
  }
  if (lane_seq_info->lc_num == 1 && lane_seq_info->dist_to_navi_end < 5.0) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "can miss navi");
    return false;
  }
  if (lane_seq_info->lc_num == 2 && lane_seq_info->dist_to_navi_end < 10.0) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "can miss navi");
    return false;
  }
  if (lane_seq_info->dist_to_navi_end > 150.0) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "far to navi end");
    return false;
  }
  if (lane_seq_info->dist_to_navi_end > lane_seq_info->dist_to_junction + 5.0) {
    Log2DDS::LogDataV2("miss_navi_debug", Log2DDS::TaskPrefix(plan_id) +
                                              "has junction in middle!");
    return false;
  }
  auto tgt_lane_seq = lane_seq_info->lane_seq;
  if (!tgt_lane_seq) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "! tgt lane seq");
    return false;
  }
  auto nearest_lane = tgt_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
  if (!nearest_lane) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "! nearest lane");
    return false;
  }
  if (nearest_lane->type() != ad_byd::planning::LANE_NORMAL &&
      nearest_lane->type() != ad_byd::planning::LANE_BUS_NORMAL &&
      nearest_lane->type() != ad_byd::planning::LANE_HOV_NORMAL) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "illegal lane!");
    return false;
  }
  if (scheduler_result.lane_change_state.stage() == LCS_WAITING ||
      scheduler_result.lane_change_state.stage() == LCS_NONE ||
      scheduler_result.lane_change_state.stage() == LCS_PAUSE ||
      scheduler_result.lane_change_state.stage() == LCS_EXECUTING) {
    Log2DDS::LogDataV2("miss_navi_debug",
                       Log2DDS::TaskPrefix(plan_id) + "MissNaviScneario!");
    return true;
  }
  Log2DDS::LogDataV2("miss_navi_debug",
                     Log2DDS::TaskPrefix(plan_id) + "In LCS_EXECUTING!");
  return false;
}

bool GetDistanceToNearestNonSolidLineFromLaneSeqNaviEnd(
    const Vec2d& start_point,
    const ad_byd::planning::LaneSeqInfoPtr& target_lane_seq_info, bool is_left,
    double& result_distance) {
  result_distance = std::numeric_limits<double>::infinity();
  if (!target_lane_seq_info || !target_lane_seq_info->lane_seq) {
    return false;
  }
  const double dis_to_navi_end = target_lane_seq_info->dist_to_navi_end;
  if (dis_to_navi_end > std::numeric_limits<double>::infinity() - 0.1) {
    return false;
  }
  auto target_lane_seq = target_lane_seq_info->lane_seq;
  const auto nearest_lane = target_lane_seq_info->nearest_lane;
  if (!nearest_lane) {
    return false;
  }
  if (!target_lane_seq_info->navi_end_lane) {
    return false;
  }

  double start_point_offset_to_seq_start = 0.0;
  double temp_proj_ego = target_lane_seq->GetProjectionDistance(
      {start_point.x(), start_point.y()}, &start_point_offset_to_seq_start);
  // Log2DDS::LogDataV2("miss_navi_debug",
  //                    absl::StrCat("start_point_offset_to_seq_start: ",
  //                                 start_point_offset_to_seq_start));
  std::vector<ad_byd::planning::LaneBoundaryType> lane_boundary_types;
  bool has_found_poi = false;
  // search laneseq reversely
  for (auto target_lane = target_lane_seq->lanes().rbegin();
       target_lane != target_lane_seq->lanes().rend(); target_lane++) {
    lane_boundary_types.clear();

    // start search till poi lane
    if (target_lane->get()->id() == target_lane_seq_info->navi_end_lane->id()) {
      has_found_poi = true;
    }
    if (!has_found_poi) {
      continue;
    }
    auto lane_start_point = target_lane->get()->center_line().begin_point();
    double begin_point_offset_to_seq_start = 0.0;
    double temp_proj_lane = target_lane_seq->GetProjectionDistance(
        lane_start_point, &begin_point_offset_to_seq_start);
    const auto lane_boundary = is_left ? target_lane->get()->left_boundary()
                                       : target_lane->get()->right_boundary();
    if (!lane_boundary) {
      continue;
    }
    lane_boundary_types = lane_boundary->boundary_types();
    for (auto boundary_type = lane_boundary_types.rbegin();
         boundary_type != lane_boundary_types.rend(); boundary_type++) {
      auto line_type = (*boundary_type).line_type;
      // LOGDATA("lc_lane_boundary_debug",
      //         absl::StrCat("line type: ", line_type));
      // LOGDATA("lc_lane_boundary_debug",
      //         absl::StrCat("line type end s: ", (*boundary_type).s));
      if (line_type != ad_byd::planning::SOLID &&
          line_type != ad_byd::planning::SOLID_SOLID &&
          line_type !=
              ad_byd::planning::SHADED_AREA) {  // ToDo use is left check
                                                // solid-dash/dash-solid
        // LOGDATA("lc_lane_boundary_debug",
        //         absl::StrCat("begin_point_offset_to_seq_start: ",
        //                      begin_point_offset_to_seq_start));
        // LOGDATA("lc_lane_boundary_debug",
        //         absl::StrCat("start_point_offset_to_seq_start: ",
        //                      start_point_offset_to_seq_start));
        // LOGDATA("lc_lane_boundary_debug",
        //         absl::StrCat("(*boundary_type).s: ", (*boundary_type).s));
        Log2DDS::LogDataV2(
            "miss_navi_debug",
            absl::StrCat("(*boundary_type).s: ", (*boundary_type).s));
        result_distance = begin_point_offset_to_seq_start -
                          start_point_offset_to_seq_start + (*boundary_type).s;
        return true;
      }
    }

    // stop search when reach nearest lane
    if (target_lane->get()->id() == nearest_lane->id()) {
      break;
    }
  }
  return false;
}

}  // namespace

// NOLINTNEXTLINE(readability-function-size)
PlannerStatus RunStPathPlanner(StPathPlannerInput& input,
                               StPathPlannerOutput* out,
                               ThreadPool* thread_pool) {
  TIMELINE("RunStPathPlanner");
  std::string function_name =
      Log2DDS::TaskPrefix(input.plan_id) + std::string(__FUNCTION__);
  SCOPED_TRACE(function_name.c_str());
  CHECK_NOTNULL(input.st_path_start_point_info);
  CHECK_NOTNULL(input.vehicle_params);
  CHECK_NOTNULL(input.planner_semantic_map_manager);
  CHECK_NOTNULL(input.stalled_objects);
  CHECK_NOTNULL(input.traj_mgr);
  CHECK_NOTNULL(input.prev_target_lane_path_from_start);
  CHECK_NOTNULL(input.time_aligned_prev_traj);
  CHECK_NOTNULL(input.prev_initializer_state);

  // For rebuilding constraint manager on lc pause.
  CHECK_NOTNULL(input.start_point_info);
  CHECK_NOTNULL(input.obj_mgr);
  // CHECK_NOTNULL(input.tl_info_map);
  CHECK_NOTNULL(input.prev_decider_state);

  // Params.
  CHECK_NOTNULL(input.decision_constraint_config);
  CHECK_NOTNULL(input.initializer_params);
  CHECK_NOTNULL(input.trajectory_optimizer_params);
  CHECK_NOTNULL(input.motion_constraint_params);
  CHECK_NOTNULL(input.planner_functions_params);
  CHECK_NOTNULL(input.vehicle_models_params);
  CHECK_NOTNULL(input.trajectory_optimizer_lc_radical_params);
  CHECK_NOTNULL(input.trajectory_optimizer_lc_normal_params);
  CHECK_NOTNULL(input.trajectory_optimizer_lc_conservative_params);
  CHECK_NOTNULL(input.obs_history);
  CHECK_NOTNULL(input.behavior);
  CHECK_NOTNULL(input.speed_state);

  const auto& psmm = *input.planner_semantic_map_manager;
  const auto& vehicle_params = *input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();
  const auto& map_func_id = input.behavior->function_id();

  out->st_planner_object_traj = std::move(input.init_st_planner_object_traj);

  // const ml::captain_net::CaptainNetOutput empty_captain_net_output;
  // Run initializer.
  InitializerInput initializer_input{
      .planner_semantic_map_manager = &psmm,
      .path_start_point_info = input.st_path_start_point_info,
      .path_look_ahead_duration = input.path_look_ahead_duration,
      .lane_change_state = &input.scheduler_output.lane_change_state,
      .prev_lc_stage = input.prev_lane_change_stage,
      .lane_change_style = input.lane_change_style,
      .stalled_objects = input.stalled_objects,
      .drive_passage = &input.scheduler_output.drive_passage,
      .st_traj_mgr = input.traj_mgr,
      .sl_boundary = &input.scheduler_output.sl_boundary,
      .prev_initializer_state = input.prev_initializer_state,
      .decision_constraint_config = input.decision_constraint_config,
      .initializer_params = input.initializer_params,
      .motion_constraint_params = input.motion_constraint_params,
      .vehicle_params = input.vehicle_params,
      .st_planner_object_traj = &out->st_planner_object_traj,
      .plan_id = input.plan_id,
      //.log_av_trajectory = input.log_av_trajectory,
      .scene_reasoning = input.scene_reasoning,
      .borrow_lane = input.scheduler_output.borrow_lane,
      .av_frenet_box = &input.scheduler_output.av_frenet_box_on_drive_passage,

      .start_point_info = input.start_point_info,
      .route_target_info = input.route_target_info,
      .smooth_result_map = input.smooth_result_map,
      .obj_mgr = input.obj_mgr,
      .traffic_light_status_map = input.traffic_light_status_map,
      .prev_target_lane_path_from_start =
          input.prev_target_lane_path_from_start,
      .prev_decider_state = input.prev_decider_state,
      // .parking_brake_release_time = input.parking_brake_release_time,
      // .enable_pull_over = input.enable_pull_over,
      // .enable_traffic_light_stopping = input.enable_traffic_light_stopping,
      // .brake_to_stop = input.brake_to_stop,
      .obs_history = input.obs_history,
      .behavior = input.behavior,
      .speed_state = input.speed_state,
      .cur_lc_num = input.cur_navi_lc_num,
      .leading_id = input.leading_id,
      .left_navi_dist = input.left_navi_dist,
      .push_dir = input.push_dir,
      .nudge_object_info = input.nudge_object_info,
      .pre_large_vehicle_avoid_state = input.pre_large_vehicle_avoid_state,
      .saved_offset = input.saved_offset,
      .pre_lc_style_decider_result = input.pre_lc_style_decider_result,
      .pre_task_safety_evaluation_result =
          input.pre_task_safety_evaluation_result,
      .pre_scene_cones_riding_line_frames_result =
          input.pre_scene_cones_riding_line_frames_result};

  absl::flat_hash_set<std::string> unsafe_object_ids;
  auto initializer_output_or = RunInitializer(
      initializer_input, &unsafe_object_ids, &input.scheduler_output,
      &input.decider_output, &out->initializer_debug_proto, thread_pool,
      &out->obs_leading, &out->lc_status_code, &out->lc_style_decider_result,
      &out->task_safety_evaluation_result,
      &out->scene_cones_riding_line_frames_result);
  std::string unsafe_obs_debug = Log2DDS::TaskPrefix(initializer_input.plan_id);
  for (const auto& unsafe_obs_id : unsafe_object_ids) {
    unsafe_obs_debug += (unsafe_obs_id + ",");
  }
  Log2DDS::LogDataV2("unsafe_obs_debug", unsafe_obs_debug);
  // miss navi lc set stopline constraint
  const auto& plan_start_point = input.start_point_info->start_point;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  bool is_miss_navi = input.miss_navi_scenario;
  if (input.miss_navi_scenario && FLAGS_planner_enable_miss_navi_stop_line) {
    // Has checked nullptr in IfMissNaviScenario
    auto lane_seq_info = input.scheduler_output.drive_passage.lane_seq_info();
    if (lane_seq_info) {
      auto lane_seq = lane_seq_info->lane_seq;
      int navi_lc_num = lane_seq_info->lc_num;

      bool is_left =
          lane_seq_info->navi_lc_command ==
                  ad_byd::planning::BehaviorCommand::Command_LaneChangeLeft
              ? true
              : false;
      double dis_to_non_solidline = std::numeric_limits<double>::infinity();
      bool get_result = GetDistanceToNearestNonSolidLineFromLaneSeqNaviEnd(
          ego_pos, lane_seq_info, is_left, dis_to_non_solidline);
      // ToDo add dist to solid func
      Log2DDS::LogDataV2("miss_navi_debug",
                         Log2DDS::TaskPrefix(initializer_input.plan_id) +
                             absl::StrCat("dis2solid: ", dis_to_non_solidline));
      double dist_navi_end_on_seq = lane_seq_info->dist_to_navi_end;
      const auto guess_dist_to_junction =
          std::min(lane_seq_info->dist_to_junction, dist_navi_end_on_seq);
      double navi_end_to_non_solidline =
          dist_navi_end_on_seq - dis_to_non_solidline;
      if (map_func_id == Behavior_FunctionId_MAPLESS_NOA) {
        navi_end_to_non_solidline = 50.0;
      }
      // Set dis to virtual stopline
      // double navi_end_to_stopline = navi_end_to_non_solidline + 10.0;
      double navi_end_to_stopline = navi_end_to_non_solidline;
      navi_end_to_stopline = std::clamp(navi_end_to_stopline, 10.0, 60.0);
      // double min_dis_to_stopline = dist_navi_end_on_seq -
      // navi_end_to_stopline;
      double min_dis_to_stopline =
          guess_dist_to_junction - navi_end_to_stopline;
      min_dis_to_stopline = std::fmax(0.0, min_dis_to_stopline);
      min_dis_to_stopline = std::min(120.0, min_dis_to_stopline);
      Log2DDS::LogDataV2(
          "miss_navi_debug",
          Log2DDS::TaskPrefix(initializer_input.plan_id) +
              absl::StrCat("min_dis_to_stopline: ", min_dis_to_stopline));
      //----------- cope with heavy brake ---------------
      bool anti_heavy_brake = false;
      double reaction_distance = plan_start_point.v() * 0.3;
      double front_to_rear_axle = 4.0;  // ToDo replace with vehicle param
                                        // loader
      double distance =
          min_dis_to_stopline - reaction_distance - front_to_rear_axle;
      double brake_a = 1.0 * plan_start_point.v() * plan_start_point.v() / 2.0 /
                       std::max(distance, 0.5);
      if (brake_a > 2.5) {
        anti_heavy_brake = true;
      }

      double check_range = 60.0;
      const auto& passage = input.scheduler_output.drive_passage;
      double start_s = min_dis_to_stopline;
      double end_s = min_dis_to_stopline + 10.0;
      const auto start_point = passage.QueryPointXYAtS(start_s);
      const auto end_point = passage.QueryPointXYAtS(end_s);
      if (!start_point.ok())
        return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                             std::string(start_point.status().message()));
      if (!end_point.ok())
        return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                             std::string(end_point.status().message()));

      if (min_dis_to_stopline < check_range &&
          (navi_lc_num == 1 || navi_lc_num == 2) &&
          lane_seq_info->dist_to_navi_end < 100.0 &&
          input.scheduler_output.lane_change_state.stage() != LCS_EXECUTING &&
          input.scheduler_output.lane_change_state.stage() != LCS_PAUSE) {
        if (anti_heavy_brake) {
          Log2DDS::LogDataV2(
              "miss_navi_debug",
              Log2DDS::TaskPrefix(initializer_input.plan_id) + "slow dec");
          // ConstraintProto::SpeedRegionProto pre_dec_speed_region;
          // start_point.value().ToProto(pre_dec_speed_region.mutable_start_point());
          // end_point.value().ToProto(pre_dec_speed_region.mutable_end_point());
          // pre_dec_speed_region.set_start_s(start_s);
          // pre_dec_speed_region.set_end_s(end_s);
          // pre_dec_speed_region.set_max_speed(std::max(plan_start_point.v()-3.0,
          // 0.0)); pre_dec_speed_region.set_min_speed(0.0);
          // pre_dec_speed_region.mutable_source()->mutable_pedestrian_object()->set_id(
          //     absl::StrCat("miss navi dec", "1"));
          // pre_dec_speed_region.set_id(absl::StrCat("miss navi dec", "1"));
          // input.decider_output.constraint_manager.AddSpeedRegion(std::move(pre_dec_speed_region));
          input.decider_output.constraint_manager.AddVLimit(
              std::max(plan_start_point.v() - 3.0, 0.0), 2.0,
              "miss navi slow dec");
        } else {
          Log2DDS::LogDataV2(
              "miss_navi_debug",
              Log2DDS::TaskPrefix(initializer_input.plan_id) + "set stopline");
          ConstraintProto::StopLineProto stop_line;
          stop_line.set_s(min_dis_to_stopline + passage.lane_path_start_s());
          stop_line.set_standoff(0.0);
          stop_line.set_time(0.0);
          HalfPlane halfplane(start_point.value(), end_point.value());
          halfplane.ToProto(stop_line.mutable_half_plane());
          stop_line.set_id("miss_navi_stop_line");
          stop_line.mutable_source()->mutable_brake_to_stop()->set_reason(
              absl::StrCat("miss navi stop", "1"));
          input.decider_output.constraint_manager.AddStopLine(
              std::move(stop_line));
          // ConstraintProto::SpeedRegionProto pre_dec_speed_region;
          // start_point.value().ToProto(pre_dec_speed_region.mutable_start_point());
          // end_point.value().ToProto(pre_dec_speed_region.mutable_end_point());
          // pre_dec_speed_region.set_start_s(start_s);
          // pre_dec_speed_region.set_end_s(end_s);
          // pre_dec_speed_region.set_max_speed(std::max(plan_start_point.v()-4.0,
          // 0.0)); pre_dec_speed_region.set_min_speed(0.0);
          // pre_dec_speed_region.mutable_source()->mutable_pedestrian_object()->set_id(
          //     absl::StrCat("miss navi stop dec", "2"));
          // pre_dec_speed_region.set_id(absl::StrCat("miss navi stop dec",
          // "2"));
          // input.decider_output.constraint_manager.AddSpeedRegion(std::move(pre_dec_speed_region));
        }
      }
    }
  }
  out->scheduler_output = std::move(input.scheduler_output);
  out->constraint_manager = std::move(input.decider_output.constraint_manager);
  out->decider_state = std::move(input.decider_output.decider_state);
  out->unsafe_object_ids = std::move(unsafe_object_ids);
  // viz the message of initializer
  auto name = Log2DDS::TaskPrefix(input.plan_id) + "init_msg";
  Log2DDS::LogDataV2(
      name, "0:" + std::string(initializer_output_or.status().message()));
  if (!initializer_output_or.ok()) {
    return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                         std::string(initializer_output_or.status().message()));
  }
  auto initializer_output = std::move(initializer_output_or).value();
  out->leading_trajs = std::move(initializer_output.leading_trajs);
  out->lc_status_code = std::move(initializer_output.lc_status_code);
  out->is_init_follow_scene = initializer_output.is_init_follow_scene;
  out->lc_lead_obj_id = initializer_output.lc_lead_obj_id;
  out->pre_large_vehicle_avoid_state =
      initializer_output.pre_large_vehicle_avoid_state;
  out->speed_response_style =
      std::move(initializer_output.speed_response_style);
  out->saved_offset = initializer_output.saved_offset;
  Log2DDS::LogDataV2("lc_unable_reason",
                     absl::StrCat("initializer output code",
                                  initializer_output.lc_status_code));
  Log2DDS::LogDataV2("lc_safety",
                     Log2DDS::TaskPrefix(initializer_input.plan_id) +
                         absl::StrCat("StPath Planner speed_response_style:",
                                      out->speed_response_style));
  // if can cancel lc
  bool if_can_cancel_lc = true;
  const auto lat_offset =
      out->scheduler_output.drive_passage.QueryFrenetLatOffsetAt(ego_pos);
  if (!lat_offset.ok())
    return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                         std::string(lat_offset.status().message()));
  if (std::abs(lat_offset.value()) < 2.4) {
    if_can_cancel_lc = false;
  }
  Log2DDS::LogDataV2("miss_navi_debug",
                     Log2DDS::TaskPrefix(initializer_input.plan_id) +
                         absl::StrCat("lat_offset:", lat_offset.value()));

  // lc special lane type check
  if (!is_miss_navi &&
      (out->scheduler_output.lane_change_state.stage() == LCS_EXECUTING ||
       out->scheduler_output.lane_change_state.stage() == LCS_PAUSE) &&
      input.prev_lane_change_stage != LCS_EXECUTING) {
    const auto lane_seq_info =
        out->scheduler_output.drive_passage.lane_seq_info();
    if (lane_seq_info->nearest_lane->type() == ad_byd::planning::LANE_UNKNOWN ||
        lane_seq_info->nearest_lane->type() ==
            ad_byd::planning::LANE_NON_MOTOR) {
      return PlannerStatus(PlannerStatusProto::LC_SPECIAL_LANE_CHECK_FAILED,
                           "Unable to lane change for unknown/non-motor lane");
    }
    double dist_to_navi_end = lane_seq_info->dist_to_navi_end;
    Log2DDS::LogDataV2(
        "lane_check",
        Log2DDS::TaskPrefix(initializer_input.plan_id) +
            FormatNumericString(", dist_to_navi_end: ", dist_to_navi_end) +
            FormatNumericString(", dist to bus lane: ",
                                lane_seq_info->dist_to_bus_lane) +
            absl::StrCat(",lc num: ", lane_seq_info->lc_num));
    // if (dist_to_navi_end > 160.0 && lane_seq_info->lc_num <= 1 &&
    //     lane_seq_info->dist_to_bus_lane < dist_to_navi_end &&
    //     lane_seq_info->dist_to_bus_lane <
    //         std::max(5.0 * plan_start_point.v(), 50.0)) {
    //   return PlannerStatus(PlannerStatusProto::LC_SPECIAL_LANE_CHECK_FAILED,
    //                        "Unable to lane change for bus lane");
    // }
  }
  // lc line check
  if (
      // input.left_navi_dist > 120 && if_can_cancel_lc &&
      !is_miss_navi && input.lc_cmd_state != DriverAction::LC_CMD_NONE &&
      out->scheduler_output.lane_change_state.stage() == LCS_EXECUTING &&
      input.prev_lane_change_stage != LCS_EXECUTING &&
      input.prev_lane_change_stage != LCS_PAUSE) {
    const auto lane_seq_info =
        out->scheduler_output.drive_passage.lane_seq_info();
    double dist_to_lc_solidline = std::numeric_limits<double>::infinity();
    if (lane_seq_info &&
        !((lane_seq_info->nearest_lane->type() ==
           ad_byd::planning::LANE_EMERGENCY) &&
          (input.eie_choice_type ==
           ad_byd::planning::EIEChoiceType::CHOICE_RIGHT_LANE))) {
      dist_to_lc_solidline = out->scheduler_output.lane_change_state.lc_left()
                                 ? lane_seq_info->dist_to_right_solid_line
                                 : lane_seq_info->dist_to_left_solid_line;
    } else {
      Log2DDS::LogDataV2(
          "line_check",
          Log2DDS::TaskPrefix(initializer_input.plan_id) + "no lane seq info");
    }
    if (
        // (input.pnp_infos.has_value() &&
        //    !input.pnp_infos.value().infos().empty() &&
        //    input.pnp_infos.value().infos()[0].lc_reason() !=
        //        LcReason::LC_REASON_FOR_AVOID_LANE_BUS) ||
        input.lc_cmd_state != DriverAction::LC_CMD_NONE) {
      if (dist_to_lc_solidline < 15.0 + plan_start_point.v() * 2.0) {
        Log2DDS::LogDataV2(
            "line_check",
            Log2DDS::TaskPrefix(initializer_input.plan_id) +
                absl::StrCat("dist to solidline: ", dist_to_lc_solidline));
        LOG_WARN << absl::StrFormat(
            "Scheduler branch deleted: Target lane linetype solid");
        return PlannerStatus(PlannerStatusProto::LC_LINE_CHECK_FAILED,
                             "Unable to lane change for solid line");
      }
    }
  }
  auto lane_seq_info_miss_navi =
      out->scheduler_output.drive_passage.lane_seq_info();
  // //deviate navi before junction
  // if (input.behavior->function_id() != Behavior_FunctionId_LKA &&
  //     input.behavior->function_id() != Behavior_FunctionId_NONE &&
  //     lane_seq_info_miss_navi &&
  //     lane_seq_info_miss_navi->dist_to_junction < 25.0) {
  //   if((input.prev_lane_change_stage == LCS_NONE &&
  //   out->scheduler_output.lane_change_state.stage() != LCS_NONE) ||
  //      ((input.prev_lane_change_stage == LCS_EXECUTING ||
  //      input.prev_lane_change_stage == LCS_PAUSE) &&
  //      lane_seq_info_miss_navi->dist_to_junction < 15.0 &&
  //       (out->scheduler_output.lane_change_state.stage() == LCS_EXECUTING ||
  //       out->scheduler_output.lane_change_state.stage() == LCS_PAUSE) &&
  //       if_can_cancel_lc)){
  //     return PlannerStatus(
  //         PlannerStatusProto::LC_DEVIATE_NAVI,"Unable to lane change near
  //         junction");
  //   }
  // }

  // lc miss navi check

  if (map_func_id != Behavior_FunctionId_LKA &&
      map_func_id != Behavior_FunctionId_NONE &&
      input.prev_lane_change_stage == LCS_NONE &&
      input.lc_cmd_state == DriverAction::LC_CMD_NONE &&
      lane_seq_info_miss_navi &&
      lane_seq_info_miss_navi->dist_to_junction < 100.0 &&
      lane_seq_info_miss_navi->dist_to_junction > 1.0 &&
      std::abs(input.left_navi_dist -
               lane_seq_info_miss_navi->dist_to_navi_end) > 10.0 &&
      lane_seq_info_miss_navi->dist_to_navi_end - input.left_navi_dist < 10.0 &&
      input.cur_navi_lc_num < lane_seq_info_miss_navi->lc_num &&
      (out->scheduler_output.lane_change_state.stage() == LCS_EXECUTING ||
       out->scheduler_output.lane_change_state.stage() == LCS_PAUSE)) {
    return PlannerStatus(PlannerStatusProto::LC_MISS_NAVI_FAILED,
                         "Unable to lane change miss navi");
  }

  if (out->scheduler_output.lane_change_state.stage() ==
      LaneChangeStage::LCS_PAUSE) {
    const auto start_lane_id =
        out->scheduler_output.drive_passage.lane_path().front().lane_id();
    out->safety_check_failed_reason =
        SafetyCheckFailedReason::LC_UNSAFE_REASON_UNKNOWN;
    if (!out->unsafe_object_ids.empty()) {
      out->safety_check_failed_reason =
          SafetyCheckFailedReason::LC_UNSAFE_REASON_MOVING_OBJECT;
      if (input.stalled_objects != nullptr && !input.stalled_objects->empty()) {
        for (const auto& obj_id : out->unsafe_object_ids) {
          if (input.stalled_objects->contains(obj_id)) {
            out->safety_check_failed_reason =
                SafetyCheckFailedReason::LC_UNSAFE_REASON_STALLED_OBJECT;
            break;
          }
        }
      }
    }
    Log2DDS::LogDataV2(
        "lc_safety",
        Log2DDS::TaskPrefix(initializer_input.plan_id) +
            " safety_check_failed_reason: " +
            SafetyCheckFailedReason_Name(out->safety_check_failed_reason));
    if (input.prev_target_lane_path_from_start->IsEmpty() ||
        input.prev_target_lane_path_from_start->front().lane_id() !=
            start_lane_id) {
      LOG_WARN << absl::StrFormat(
          "Scheduler branch deleted: Target lane %d not safe", start_lane_id);
      return PlannerStatus(
          PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
          absl::StrCat("Unsafe to initiate lane change to ",
                       out->scheduler_output.drive_passage.lane_path()
                           .front()
                           .lane_id()));
    }
  }

  out->follower_set = std::move(initializer_output.follower_set);
  out->leader_set = std::move(initializer_output.leader_set);
  out->follower_max_decel = initializer_output.follower_max_decel;
  out->gaming_lc_obs_set = std::move(initializer_output.gaming_lc_obs_set);
  // Update spacetime planner trajectory from leading obj.
  for (const auto& [traj_id, _] : out->leading_trajs) {
    const auto status = AddSpaceTimePlannerTrajectoryById(
        *input.traj_mgr, traj_id,
        SpacetimePlannerObjectTrajectoryReason::LEADING,
        &out->st_planner_object_traj);
    VLOG_IF(3, !status.ok()) << status.ToString();
  }
  out->initializer_state = std::move(initializer_output.initializer_state);
  out->traj_points = std::move(initializer_output.traj_points);
  out->nudge_info = std::move(initializer_output.nudge_info);
  return OkPlannerStatus();
}

}  // namespace st::planning
