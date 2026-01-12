
#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>
//
#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/meta/type_traits.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "plan_common/gflags.h"
//
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_debug.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/msg/st_msgs/sm_behavior.pb.h"
//
#include "plan_common/assist_util.h"
#include "plan_common/async/async_util.h"
#include "plan_common/async/future.h"
#include "plan_common/async/parallel_for.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/drive_passage.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections.h"
#include "plan_common/maps/route_sections_info.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/planning_macros.h"
#include "plan_common/timer.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/trajectory_util.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/util/planner_status_macros.h"
#include "plan_common/util/time_util.h"
#include "plan_common/util/vehicle_geometry_util.h"
//
#include "predictor/prediction.h"
//
#include "router/route_manager_output.h"
//
#include "object_manager/object_vector.h"
#include "object_manager/partial_spacetime_object_trajectory.h"
#include "object_manager/planner_object.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/spacetime_trajectory_manager_builder.h"
#include "object_manager/st_inference/est_planner_output.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "object_manager/st_inference/selector_input.h"
//
#include "planner/planner_manager/min_length_path_extension.h"
#include "planner/planner_manager/planner_defs.h"
#include "planner/planner_manager/planner_flags.h"
#include "planner/planner_manager/planner_util.h"
#include "planner/planner_manager/pnp_util.h"
//
#include "decider/initializer/lane_change_style_decider.h"
#include "decider/decision_manager/rule_based_lc.h"
#include "decider/decision_manager/traffic_gap_finder.h"
#include "decider/scheduler/candidate_lane_sequences.h"
#include "decider/scheduler/fsd_lane_selector.h"
#include "decider/scheduler/multi_tasks_scheduler.h"
#include "decider/scheduler/scheduler_input.h"
#include "decider/scheduler/target_lane_path_filter.h"
#include "decider/selector/selector.h"
// #include "decider/scheduler/lane_graph/lane_path_finder.h"
// #include "decider/scheduler/lane_graph/lane_graph_builder.h"
#include "decider/scene_manager/construction_scene_identification.h"
//
#include "node_manager/task_runner/est_planner.h"
#include "node_manager/task_runner/fallback_planner.h"
#include "node_manager/task_runner/multi_tasks_cruise_planner.h"
#include "node_manager/task_runner/planner_main_loop_internal.h"
#include "plan_common/util/format_numeric_string.h"

namespace st::planning {

namespace {

using MapConstPtr = ad_byd::planning::MapConstPtr;
using LaneConstPtr = ad_byd::planning::LaneConstPtr;
using LaneSequencePtr = ad_byd::planning::LaneSequencePtr;
using LaneType = ad_byd::planning::LaneType;
// using StationaryObstacle = ad_byd::planning::StationaryObstacle;
bool IsValidLane(const LaneConstPtr& lane) {
  if (!lane || lane->type() == LaneType::LANE_NON_MOTOR ||
      lane->type() == LaneType::LANE_EMERGENCY) {
    return false;
  }
  return true;
}

void CountNaviAndEfficiencyInfo(const Vec2d& ego_pos,
                                const LaneSequencePtr& laneseq,
                                const PlannerObjectManager& obj_mgr,
                                int* navi_section_cnt, double* average_speed) {
  if (!laneseq) return;
  // navigation
  int navi_cnt = 0;
  for (const auto& lane : laneseq->lanes()) {
    if (!lane->is_navigation()) break;
    navi_cnt++;
  }
  if (navi_section_cnt != nullptr) *navi_section_cnt = navi_cnt;
  // efficiency
  double avg_speed = 0.0;
  int valid_obs_cnt = 0;
  double ego_s = 0.0;
  laneseq->GetProjectionDistance(ego_pos, &ego_s);
  for (const auto& obs : obj_mgr.planner_objects()) {
    const auto obs_pos = obs.pose().pos();
    double obs_s = 0.0;
    double obs_l = laneseq->GetProjectionDistance(obs_pos, &obs_s);
    if (obs_l > 2.0 || obs_s < ego_s) continue;
    avg_speed += obs.pose().v();
    valid_obs_cnt++;
  }
  if (valid_obs_cnt > 0 && average_speed != nullptr) {
    *average_speed = avg_speed / valid_obs_cnt;
  }
}

std::optional<double> ComputeCruisingSpeedLimit(
    const std::optional<double>& noa_cruising_speed_limit, double start_v) {
  if (!noa_cruising_speed_limit.has_value()) {
    return std::nullopt;
  }

  std::optional<double> speed_limit = noa_cruising_speed_limit;

  if (start_v >
      speed_limit.value() + FLAGS_planner_max_drop_cruising_speed_limit) {
    speed_limit = start_v - FLAGS_planner_max_drop_cruising_speed_limit;
  }

  return speed_limit;
}

std::optional<double> UpdateCruisingSpeedLimitByLabel(
    const ad_byd::planning::MapPtr& map, const double& start_v,
    const double& raw_cruising_speed_limit,
    const ad_byd::planning::LaneSequencePtr& prev_target_lane_seq) {
  std::optional<double> speed_limit = raw_cruising_speed_limit;
  if (!map || !prev_target_lane_seq) return speed_limit;

  const auto& navi_start = map->route()->navi_start();
  const double frame_time = 0.1;
  double custom_limit = 0.0;
  double dis_to_custom = prev_target_lane_seq->GetDistanceToCustomSpeedLimit(
      navi_start, &custom_limit);
  Log2DDS::LogDataV2(
      "user_set_speed_debug",
      FormatNumericString("raw_user_limit: ", raw_cruising_speed_limit) +
          FormatNumericString(", custom speed: ", custom_limit) +
          FormatNumericString(", dis_to_custom: ", dis_to_custom));
  if (dis_to_custom > 50.0 || custom_limit > raw_cruising_speed_limit) {
    Log2DDS::LogDataV2("user_set_speed_debug_curve", raw_cruising_speed_limit);
    return speed_limit;
  }

  //
  double new_user_limit = raw_cruising_speed_limit;
  if (start_v - custom_limit < 5.0 * ad_byd::planning::Constants::KPH2MPS) {
    new_user_limit = custom_limit;
  } else if (dis_to_custom < ad_byd::planning::Constants::ZERO) {
    new_user_limit = start_v - 1.5 * frame_time;
  } else {
    double a =
        (custom_limit * custom_limit - start_v * start_v) / 2.0 / dis_to_custom;
    a = ad_byd::planning::math::Clamp(a, -1.5, 1.5);
    new_user_limit = start_v + a * frame_time;
  }

  Log2DDS::LogDataV2("user_set_speed_debug_curve", new_user_limit);
  speed_limit = new_user_limit;
  return speed_limit;
}

absl::StatusOr<int> FindRouteTargetIndex(
    absl::Span<const SchedulerOutput> scheduler_outputs,
    const mapping::LanePath& preferred_lane_path, bool is_open_gap,
    const std::optional<bool>& begin_route_change_left,
    const std::optional<bool>& force) {
  if (!preferred_lane_path.IsEmpty()) {
    absl::flat_hash_set<mapping::ElementId> preferred_lanes(
        preferred_lane_path.lane_ids().begin(),
        preferred_lane_path.lane_ids().end());
    if (preferred_lane_path.lane_seq() &&
        preferred_lane_path.lane_seq()->IsValid()) {
      for (const auto lane : preferred_lane_path.lane_seq()->lanes()) {
        if (lane) preferred_lanes.insert(lane->id());
      }
    }
    for (int i = 0; i < scheduler_outputs.size(); ++i) {
      if (preferred_lanes.contains(scheduler_outputs[i]
                                       .drive_passage.lane_path()
                                       .front()
                                       .lane_id())) {
        return i;
      }
    }
  }
  if (!is_open_gap) {
    Log2DDS::LogDataV2("gap_debug", "abort gap select");
    return absl::AbortedError("Abort gap select.");
  }

  if (force.has_value()) {
    bool lane_change_left = force.value();
    for (int i = 0; i < scheduler_outputs.size(); i++) {
      if (scheduler_outputs[i].lane_change_state.stage() == LCS_EXECUTING &&
          scheduler_outputs[i].lane_change_state.lc_left() ==
              lane_change_left) {
        return i;
      }
    }
  } else if (begin_route_change_left.has_value()) {
    bool lane_change_left = begin_route_change_left.value();
    for (int i = 0; i < scheduler_outputs.size(); i++) {
      if (scheduler_outputs[i].lane_change_state.stage() == LCS_EXECUTING &&
          scheduler_outputs[i].lane_change_state.lc_left() ==
              lane_change_left) {
        return i;
      }
    }
  }
  return absl::AbortedError("Abort gap select no reasonable task");
}

std::optional<RouteTargetInfo> FindRouteTargetInfo(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& preferred_lane_path, const Box2d& ego_box,
    absl::Span<const SchedulerOutput> scheduler_outputs,
    absl::Span<const SpacetimeTrajectoryManager> st_traj_mgr_list,
    bool& is_open_gap, const Vec2d& ego_pos, const std::optional<bool>& force,
    const std::optional<bool>& begin_route_change_left) {
  ASSIGN_OR_RETURN(
      const auto route_target_index,
      FindRouteTargetIndex(scheduler_outputs, preferred_lane_path, is_open_gap,
                           begin_route_change_left, force),
      std::nullopt);
  Log2DDS::LogDataV2("gap_debug",
                     absl::StrCat("route_target_index: ", route_target_index));

  const auto& target_scheduler = scheduler_outputs[route_target_index];
  const auto target_lane_path_ext = BackwardExtendLanePath(
      psmm,
      target_scheduler.drive_passage.extend_lane_path().BeforeArclength(
          kLaneChangeCheckForwardLength),
      kLaneChangeCheckBackwardLength);
  auto target_frenet_frame_or =
      BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path_ext),
                             /*down_sample_raw_points=*/true);
  if (!target_frenet_frame_or.ok()) return std::nullopt;

  ASSIGN_OR_RETURN(const auto ego_frenet_box,
                   target_frenet_frame_or->QueryFrenetBoxAt(ego_box),
                   std::nullopt);

  // solid line no route_info
  if (!force.has_value()) {
    const bool lc_left = target_scheduler.lane_change_state.lc_left();
    const auto& lane_seq_ptr =
        target_scheduler.drive_passage.lane_seq_info()->lane_seq;
    if (lane_seq_ptr) {
      ad_byd::planning::LineType line_type =
          ad_byd::planning::LineType::UNKNOWN;
      const auto& lanes = lane_seq_ptr->lanes();
      for (auto lane_ptr : lanes) {
        if (!lane_ptr) continue;
        ad_byd::planning::LaneBoundariesConstPtr boundary_ptr = nullptr;
        if (lc_left) {
          boundary_ptr = lane_ptr->right_boundary();
        } else {
          boundary_ptr = lane_ptr->left_boundary();
        }
        if (boundary_ptr) {
          double project_s = 0.0, project_l = 0.0;
          boundary_ptr->line_curve().GetProjection(ego_pos, &project_s,
                                                   &project_l);
          int cur_idx = boundary_ptr->GetBoundarySegmentIndex(project_s);
          if (cur_idx < 0) continue;
          const auto& all_boundary_type = boundary_ptr->boundary_types();
          line_type = all_boundary_type.at(cur_idx).line_type;
          break;
        }
      }
      if (line_type == ad_byd::planning::LineType::SOLID ||
          line_type == ad_byd::planning::LineType::SOLID_SOLID ||
          line_type == ad_byd::planning::LineType::DASHED_SOLID) {
        is_open_gap = false;
        Log2DDS::LogDataV2(
            "gap_debug",
            absl::StrCat("no force, solid line, don't calculate gap"));
        return std::nullopt;
      }
    }
  }

  Log2DDS::LogDataV2(
      "gap_debug",
      absl::StrCat(
          "target_route_info: ",
          target_scheduler.drive_passage.lane_path().front().lane_id()));

  return RouteTargetInfo{
      .plan_id = route_target_index + 1,
      .frenet_frame = std::move(target_frenet_frame_or).value(),
      .ego_frenet_box = ego_frenet_box,
      .drive_passage = target_scheduler.drive_passage,
      .sl_boundary = target_scheduler.sl_boundary,
      .st_traj_mgr = st_traj_mgr_list[route_target_index]};
}

inline bool ShouldConsiderRouteTarget(
    int index, const SchedulerOutput& scheduler_output,
    const std::optional<RouteTargetInfo>& route_target_info) {
  return route_target_info.has_value() &&
         ((index + 1 != route_target_info->plan_id &&
           scheduler_output.lane_change_state.stage() ==
               LaneChangeStage::LCS_NONE) ||
          scheduler_output.lane_change_state.stage() ==
              LaneChangeStage::LCS_PAUSE);
}

void AppendFallbackToResultList(
    const PlannerStatus& fallback_status, FallbackPlannerOutput fallback_result,
    EstPlannerDebug fallback_debug,
    std::vector<SpacetimeTrajectoryManager>* st_traj_mgr_list,
    std::vector<PlannerStatus>* status_list,
    std::vector<EstPlannerOutput>* results,
    std::vector<EstPlannerDebug>* debug_list) {
  st_traj_mgr_list->push_back(std::move(fallback_result.filtered_traj_mgr));
  status_list->push_back(fallback_status);
  debug_list->push_back(std::move(fallback_debug));
  results->emplace_back(EstPlannerOutput{
      .scheduler_output = std::move(fallback_result.scheduler_output),
      .path = std::move(fallback_result.path),
      .traj_points = std::move(fallback_result.trajectory_points),
      .st_path_points = std::move(fallback_result.st_path_points),
      .decider_state = std::move(fallback_result.decider_state),
      .considered_st_objects = std::move(fallback_result.considered_st_objects),
      .trajectory_end_info = std::move(fallback_result.trajectory_end_info),
      .tl_stop_interface = std::move(fallback_result.tl_stop_interface),
      .tl_ind_info = std::move(fallback_result.tl_ind_info),
      .st_boundaries_with_decision =
          std::move(fallback_result.st_boundaries_with_decision),
      .speed_state = std::move(fallback_result.speed_state)});
}

void PreFilterByPreferred(const ALCState alc_state,
                          const mapping::LanePath& preferred_lane_path,
                          const std::vector<EstPlannerOutput>& est_results,
                          const VehicleGeometryParamsProto& vehicle_geom,
                          PlcInternalResult* plc_result,
                          std::vector<PlannerStatus>* status_list,
                          LcFeasibility& lc_unable_reason_manual) {
  Log2DDS::LogDataV2(
      "mlc_debug_unable_reason",
      absl::StrCat("init lc_feasibility", lc_unable_reason_manual));
  const auto& original_status_list = *status_list;
  plc_result->status = PlcInternalResult::kOk;
  absl::flat_hash_set<mapping::ElementId> preferred_lanes(
      preferred_lane_path.lane_ids().begin(),
      preferred_lane_path.lane_ids().end());
  int preferred_idx = -1;
  std::vector<int> match_cnts(original_status_list.size(), 0);
  int max_match_cnt = -1;

  bool disable_all_others = false;
  bool bus_lane_passable = true;
  const absl::Cleanup disable_other_status = [alc_state, &bus_lane_passable,
                                              &preferred_idx,
                                              &disable_all_others,
                                              &preferred_lanes, status_list]() {
    if (!disable_all_others &&
        !(alc_state == ALC_CROSSING_LANE || alc_state == ALC_RETURNING)) {
      return;
    }
    const auto& pref_status_list = *status_list;
    bool is_preferred_ok = pref_status_list[preferred_idx].ok();
    if (preferred_idx == -1 ||
        (!is_preferred_ok && !pref_status_list[0].ok())) {
      return;
    }
    const std::string err_msg = absl::StrCat(
        "Planner branch ignored: teleop to lanes (",
        absl::StrJoin(preferred_lanes, ", "), ")",
        alc_state == ALC_CROSSING_LANE || alc_state == ALC_RETURNING
            ? absl::StrCat(" with state ", ALCState_Name(alc_state), ".")
            : ".");
    for (int i = 0; i < status_list->size(); ++i) {
      if (i == preferred_idx) {
        continue;
      }
      if (i == 0 && (!is_preferred_ok || !bus_lane_passable)) {
        continue;
      }
      (*status_list)[i] =
          PlannerStatus(PlannerStatusProto::BRANCH_RESULT_IGNORED, err_msg);
    }
  };

  // compute match_counts for lps of est_results
  for (int i = 0; i < original_status_list.size(); ++i) {
    const auto& lane_path =
        est_results[i].scheduler_output.drive_passage.lane_path();
    for (const auto lane_id : lane_path.lane_ids()) {
      if (!preferred_lanes.contains(lane_id)) {
        break;
      }
      match_cnts[i]++;
    }
  }
  // get max_match_count and preferred_idx
  for (int j = 0; j < original_status_list.size(); ++j) {
    if (match_cnts[j] == 0) {
      continue;
    }
    if (preferred_idx == -1 || match_cnts[j] > max_match_cnt) {
      preferred_idx = j;
      max_match_cnt = match_cnts[j];
    } else if (match_cnts[j] == max_match_cnt) {
      if (original_status_list[j].ok() &&
          (!original_status_list[preferred_idx].ok() ||
           est_results[preferred_idx].scheduler_output.is_fallback)) {
        preferred_idx = j;
      }
    }
  }

  // no valid target lane for manual lc
  if (preferred_idx == -1) {
    plc_result->status = PlcInternalResult::kBranchNotFound;
    Log2DDS::LogDataV2("mlc_debug_unable_reason", "no valid target lane");
    lc_unable_reason_manual = LcFeasibility::FEASIBILITY_NO_LANE;
    return;
  }

  const auto& preferred_status = original_status_list[preferred_idx];
  const auto& preferred_result = est_results[preferred_idx];
  const auto& preferred_scheduler = preferred_result.scheduler_output;
  const auto& preferred_seq_info =
      preferred_scheduler.drive_passage.lane_seq_info();
  if (preferred_status.ok() && preferred_seq_info != nullptr) {
    // bus_lane_passable default is true. Set to false if preferred_lane_seq is
    // bus lane and impassable.
    if (!preferred_seq_info->dist_to_bus_lane_vec.empty() &&
        !preferred_seq_info->bus_lane_passable_mark.empty()) {
      double dist_to_bus_lane =
          preferred_seq_info->dist_to_bus_lane_vec.front().first;
      bool is_impassable =
          (preferred_seq_info->bus_lane_passable_mark.front() == 2);
      if (is_impassable && dist_to_bus_lane < 50.0) {
        bus_lane_passable = false;
      }
      Log2DDS::LogDataV2(
          "mlc_prefilter_debug",
          absl::StrCat("dist_to_bus_lane: ", dist_to_bus_lane,
                       ", bus_lane_passable: ", bus_lane_passable));
    }
  } else if (!preferred_status.ok()) {
    if (preferred_status.status_code() ==
        PlannerStatusProto::LC_LINE_CHECK_FAILED) {
      plc_result->status = PlcInternalResult::kSolidBoundary;
      Log2DDS::LogDataV2("mlc_debug_unable_reason", "solid line!");
      lc_unable_reason_manual = LcFeasibility::FEASIBILITY_LINE_TYPE;
    } else if (preferred_status.status_code() ==
               PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      plc_result->status = PlcInternalResult::kUnsafeObject;
      plc_result->unsafe_object_ids = {
          preferred_result.unsafe_object_ids.begin(),
          preferred_result.unsafe_object_ids.end()};
      // Also find possible solid boundary points since we have no trajectory
      // here to judge whether they also affect plc.
      plc_result->left_solid_boundary =
          preferred_scheduler.lane_change_state.lc_left();
      Log2DDS::LogDataV2("mlc_debug_unable_reason", "safety_check_failed");
      lc_unable_reason_manual = LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT;
    } else {
      plc_result->status = PlcInternalResult::kBranchFailedInternal;
    }
  }
  if (false) {
    const auto preferred_crossed_or = HasTrajectoryCrossedSolidBoundary(
        preferred_scheduler.drive_passage, preferred_scheduler.sl_boundary,
        preferred_result.traj_points, vehicle_geom,
        preferred_scheduler.lane_change_state.stage() ==
            LaneChangeStage::LCS_PAUSE,
        kRouteStationUnitStep);
    if (preferred_crossed_or.ok() && *preferred_crossed_or) {
      bool all_crossed_solid = true;
      for (int i = 0; i < original_status_list.size(); ++i) {
        if (i == preferred_idx || !original_status_list[i].ok()) continue;

        const auto& branch_scheduler = est_results[i].scheduler_output;
        const auto branch_crossed_or = HasTrajectoryCrossedSolidBoundary(
            branch_scheduler.drive_passage, branch_scheduler.sl_boundary,
            est_results[i].traj_points, vehicle_geom,
            branch_scheduler.lane_change_state.stage() ==
                LaneChangeStage::LCS_PAUSE,
            kRouteStationUnitStep);
        if (branch_crossed_or.ok() && !(*branch_crossed_or)) {
          all_crossed_solid = false;
          break;
        }
      }

      // do not check line typy modified on 2024 0303
      // if (!all_crossed_solid) {
      //   plc_result->status = PlcInternalResult::kSolidBoundary;
      //   plc_result->left_solid_boundary =
      //       preferred_scheduler.lane_change_state.lc_left();
      //   (*status_list)[preferred_idx] = PlannerStatus(
      //       PlannerStatusProto::BRANCH_RESULT_IGNORED,
      //       absl::StrCat("Planner branch ignored: crossing solid
      //       boundary."));
      //   return;
      // } else {
      //   Log2DDS::LogDataV2("mlc_debug_unable_reason", "solid line!!!");
      //   lc_unable_reason_manual = LcFeasibility::FEASIBILITY_LINE_TYPE;
      // }
    }
  }
  disable_all_others = true;
}

void PreFilterRedundant(const std::vector<EstPlannerOutput>& est_results,
                        std::vector<PlannerStatus>* status_list) {
  // If a normal branch (scheduled in this iteration) has ok status:
  //  - its fallback correspondence (if any) will not be considered,
  //  - its lc pause correspondence (if any) will not be considered,
  // otherwise:
  //  - if it has a lc pause correspondence, use it,
  //  - otherwise if it has a fallback correspondence, use it.

  const auto& original_status_list = *status_list;
  // Map succeeded normal branches' start lane id to whether executing lc.
  absl::flat_hash_map<mapping::ElementId, bool> id_stage_map;
  for (int i = 0; i < original_status_list.size(); ++i) {
    if (!original_status_list[i].ok() ||
        est_results[i].scheduler_output.is_fallback) {
      continue;
    }
    const auto start_lane_id = est_results[i]
                                   .scheduler_output.drive_passage.lane_path()
                                   .front()
                                   .lane_id();
    const auto [lc_executing, _] = id_stage_map.insert({start_lane_id, false});
    if (est_results[i].scheduler_output.lane_change_state.stage() ==
        LaneChangeStage::LCS_EXECUTING) {
      lc_executing->second = true;
    }
  }

  auto& considered_status_list = *status_list;
  for (int i = 0; i < considered_status_list.size(); ++i) {
    if (!considered_status_list[i].ok()) continue;

    const auto& scheduler_output = est_results[i].scheduler_output;
    const auto start_lane_id =
        scheduler_output.drive_passage.lane_path().front().lane_id();
    if (!id_stage_map.contains(start_lane_id)) continue;

    if (scheduler_output.is_fallback) {
      considered_status_list[i] = PlannerStatus(
          PlannerStatusProto::BRANCH_RESULT_IGNORED,
          "Fallback branch ignored: latest correspondence succeeded.");
    }
    if (scheduler_output.lane_change_state.stage() ==
            LaneChangeStage::LCS_PAUSE &&
        FindOrDie(id_stage_map, start_lane_id)) {
      considered_status_list[i] = PlannerStatus(
          PlannerStatusProto::BRANCH_RESULT_IGNORED,
          "LC pause branch ignored: lane change branch passed safety check.");
    }
  }
}

enum class lc_dir { none = 0, left = 1, right = 2 };

bool IfMissNaviScenarioForCur(
    const ad_byd::planning::LaneSeqInfo* lane_seq_info, Vec2d ego_pos,
    Behavior_FunctionId func_id) {
  if (!lane_seq_info) {
    Log2DDS::LogDataV2("miss_navi_debug", "no input");
    return false;
  }
  Log2DDS::LogDataV2("miss_navi_debug", absl::StrCat("[IfMissNavi] lc num: ",
                                                     lane_seq_info->lc_num));
  Log2DDS::LogDataV2("miss_navi_debug",
                     absl::StrCat("[IfMissNavi] dist_to_navi_end: ",
                                  lane_seq_info->dist_to_navi_end));
  Log2DDS::LogDataV2("miss_navi_debug",
                     FormatNumericString("[IfMissNavi] dist_to_junction: ",
                                         lane_seq_info->dist_to_junction));
  double min_distance_to_end_or_junction = lane_seq_info->dist_to_navi_end;
  if (lane_seq_info->dist_to_junction < 35.0) {
    Log2DDS::LogDataV2("miss_navi_debug", "near junction");
    return false;
  }
  if (lane_seq_info->lc_num == 0) {
    Log2DDS::LogDataV2("miss_navi_debug", "lc num0 ");
    return false;
  }
  if (lane_seq_info->lc_num == 1 && lane_seq_info->dist_to_navi_end < 5.0) {
    Log2DDS::LogDataV2("miss_navi_debug", "can miss navi");
    return false;
  }
  if (lane_seq_info->lc_num == 2 && lane_seq_info->dist_to_navi_end < 10.0) {
    Log2DDS::LogDataV2("miss_navi_debug", "can miss navi");
    return false;
  }
  if (min_distance_to_end_or_junction >
      std::max(120.0, lane_seq_info->lc_num * 100.0)) {
    Log2DDS::LogDataV2("miss_navi_debug", "far to v2 navi end");
    return false;
  }
  if (lane_seq_info->dist_to_navi_end >
          lane_seq_info->dist_to_junction + 20.0 &&
      func_id == Behavior_FunctionId_CITY_NOA) {
    Log2DDS::LogDataV2("miss_navi_debug", "has junction in middle!");
    return false;
  }
  auto tgt_lane_seq = lane_seq_info->lane_seq;
  if (!tgt_lane_seq) {
    Log2DDS::LogDataV2("miss_navi_debug", "! tgt lane seq");
    return false;
  }
  auto nearest_lane = tgt_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
  if (!nearest_lane) {
    Log2DDS::LogDataV2("miss_navi_debug", "! nearest lane");
    return false;
  }
  if (nearest_lane->type() != ad_byd::planning::LANE_NORMAL &&
      nearest_lane->type() != ad_byd::planning::LANE_BUS_NORMAL &&
      nearest_lane->type() != ad_byd::planning::LANE_HOV_NORMAL) {
    Log2DDS::LogDataV2("miss_navi_debug", "illegal lane!");
    return false;
  }
  Log2DDS::LogDataV2("miss_navi_debug", "MissNaviScenario!");
  return true;
}

bool GetInterestPointsOfLatSafetyCheck(const Vec2d& ego_pos,
                                       const double& ego_v,
                                       const LaneSequencePtr& tgt_seq,
                                       std::vector<double>* ipts) {
  if (!tgt_seq || !ipts) return false;

  double ego_s = 0.0;
  tgt_seq->GetProjectionDistance(ego_pos, &ego_s);

  std::vector<double> check_time{0.0, 1.0, 2.0, 3.0};
  for (const auto& t : check_time) {
    auto pt_0 = tgt_seq->GetPointAtS(ego_s + ego_v * t);
    auto pt_1 = tgt_seq->GetPointAtS(ego_s + ego_v * t + 1.0);
    if (pt_0 == pt_1 || ego_s + ego_v * t + 1.0 > tgt_seq->GetPointsLength()) {
      break;
    }
    ipts->emplace_back(
        ad_byd::planning::math::NormalizeAngle((pt_1 - pt_0).Angle()));

    Log2DDS::LogDataV0("Lat_unsafety", absl::StrCat(ipts->back()));
    std::vector<ad_byd::planning::Point2d> debug;
    debug.emplace_back(pt_0);
    debug.emplace_back(pt_1);
    Log2DDS::LogPointsV2("debug_lat_safety", Log2DDS::kYellow, {}, debug, 10.0);
  }
  return true;
}
}  // namespace

double UpdatePlanningHorizon(const ad_byd::planning::MapPtr& map,
                             const double& raw_cruising_speed_limit,
                             const double& start_v) {
  double planning_min_horizon = 150.0;
  double max_speed_limit = start_v;
  max_speed_limit = std::fmax(max_speed_limit, raw_cruising_speed_limit);
  if (map && map->route()) {
    const auto& route_sections = map->route()->GetRouteInfo().sections;
    for (const auto& section : route_sections) {
      for (const auto lane_id : section.lane_ids) {
        if (!map->GetLaneById(lane_id)) continue;
        double lane_speed_limit = map->GetLaneById(lane_id)->speed_limit();
        max_speed_limit = std::fmax(max_speed_limit, lane_speed_limit / 3.6);
      }
    }
  }
  max_speed_limit = std::fmin(max_speed_limit, 130.0 / 3.6);
  if (!map->is_on_highway()) planning_min_horizon = 100.0;
  return std::fmax(planning_min_horizon,
                   max_speed_limit * kPlanningTimeHorizon);
}  // namespace

absl::StatusOr<MergeTopology> CheckIfHasMerge(
    const PlannerSemanticMapManager& psmm, const Vec2d& ego_pos,
    std::vector<ad_byd::planning::LaneSequencePtr> candidate_lane_seqs) {
  // get current lane seq
  ad_byd::planning::LaneSequencePtr cur_seq;
  for (int i = 0; i < candidate_lane_seqs.size(); ++i) {
    const auto& lane_seq = candidate_lane_seqs.at(i);
    if (lane_seq->GetSequenceDirection() ==
        ad_byd::planning::SequenceDirection::Cur) {
      cur_seq = lane_seq;
      break;
    }
  }
  if (!cur_seq || !cur_seq->IsValid()) {
    return absl::CancelledError("Current lane seq is invalid!");
  }
  // get ego offset and start_idx on cur_seq
  double ego_s = 0.0;
  const double l = cur_seq->GetProjectionDistance(ego_pos, &ego_s);
  const auto projection_point = cur_seq->GetPointAtS(ego_s);
  const auto& nearest_lane = cur_seq->GetNearestLane(projection_point);
  if (!nearest_lane) {
    return absl::CancelledError("Cant find nearest lane in cur_seq!");
  }
  const auto& navi_start = psmm.map_ptr()->route()->navi_start();
  double dist_to_start_lane_end = std::numeric_limits<double>::infinity();
  int start_index = -1;
  cur_seq->GetStartDistAndIndex(navi_start, nearest_lane, ego_s,
                                dist_to_start_lane_end, start_index);
  if (start_index == -1) {
    return absl::CancelledError("Cant get start_idx in cur_seq!");
  }
  // return merge_topo if has merge in kICCPreviewDist
  constexpr double kICCPreviewDist = 300;  // m
  LaneConstPtr merge_lane = nullptr;
  double dist_to_merge = cur_seq->GetDistanceToPOI(
      ad_byd::planning::Poi_Merge, ego_pos.x(), ego_pos.y(), merge_lane,
      dist_to_start_lane_end, start_index);
  if (merge_lane && dist_to_merge < kICCPreviewDist) {
    const auto& merge_topo = merge_lane->merge_topology();
    if (merge_topo == MergeTopology::TOPOLOGY_MERGE_LEFT ||
        merge_topo == MergeTopology::TOPOLOGY_MERGE_RIGHT) {
      Log2DDS::LogDataV2("icc_task_debug",
                         absl::StrCat("merge_lane_id: ", merge_lane->id(),
                                      ", merge_topo: ", merge_topo));
      return merge_topo;
    }
  }
  return absl::CancelledError("Has no merge topo!");
}

// NOLINTNEXTLINE
PlannerStatus RunMultiTasksCruisePlanner(
    const MultiTasksCruisePlannerInput& input,
    const PlannerState& planner_state, PathBoundedEstPlannerOutput* output,
    ThreadPool* thread_pool) {
  SCOPED_TRACE(__FUNCTION__);
  TIMELINE("RunMultiTasksCruisePlanner");
  const auto& plan_start_point = input.start_point_info->start_point;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const double ego_theta = plan_start_point.path_point().theta();
  const auto& vehicle_geometry =
      input.vehicle_params->vehicle_geometry_params();
  const Box2d ego_box = ComputeAvBox(
      ego_pos, plan_start_point.path_point().theta(), vehicle_geometry);

  const auto& psmm = *planner_state.planner_semantic_map_manager;
  const auto& smm = *psmm.map_ptr();
  auto& lite_map = psmm.lite_map_ptr();

  // Set cruising speed limit
  auto cruising_speed_limit = input.cruising_speed_limit;
  const double planning_horizon = UpdatePlanningHorizon(
      psmm.map_ptr(), *cruising_speed_limit, plan_start_point.v());
  const mapping::LanePoint destination;

  // Set lc style
  LaneChangeStyle lc_style_param =
      psmm.map_ptr()->is_on_highway()
          ? input.planner_params->lc_decision_params()
                .lane_change_style_highway()
          : input.planner_params->lc_decision_params().lane_change_style_city();
  Log2DDS::LogDataV2("lane_change_style",
                     absl::StrCat("proto_lane_change_style: ", lc_style_param));

  auto lane_change_style = LC_STYLE_NORMAL;
  switch (lc_style_param) {
    case LaneChangeStyle::LC_STYLE_NORMAL:
      lane_change_style = LC_STYLE_NORMAL;
      break;
    case LaneChangeStyle::LC_STYLE_RADICAL:
      lane_change_style = LC_STYLE_RADICAL;
      break;
    case LaneChangeStyle::LC_STYLE_CONSERVATIVE:
      lane_change_style = LC_STYLE_CONSERVATIVE;
      break;
    default:
      lane_change_style = LC_STYLE_NORMAL;
      break;
  }
  Log2DDS::LogDataV2(
      "lane_change_style",
      absl::StrCat("last lc style: ", planner_state.last_lc_style));

  // ----------------------------------------------------------
  // ---------------- 1 Update prev_target_lane_path ----------
  // ----------------------------------------------------------
  st::Timer multitask_timer("UpdatePrevTarLanePath");
  const auto& function_id = input.behavior->function_id();
  const auto& behavior_choice = input.behavior_choice;
  const bool is_navi = function_id != Behavior_FunctionId_LKA;
  const bool is_lka = function_id == Behavior_FunctionId_LKA;
  // Get pre_target_lane_seq same lane sequence.
  const auto& map = psmm.map_ptr();
  ad_byd::planning::LaneSequencePtr pre_target_lane_seq = nullptr;
  std::vector<double> last_angles;
  Log2DDS::LogDataV0("Lat_unsafety", "start");
  if (!planner_state.prev_target_lane_path.IsEmpty()) {
    const auto& last_tgt_seq = planner_state.prev_target_lane_path.lane_seq();
    Log2DDS::LogDataV0("Lat_unsafety", "last_angles:");
    GetInterestPointsOfLatSafetyCheck(ego_pos, plan_start_point.v(),
                                      last_tgt_seq, &last_angles);

    std::vector<std::string> debug_pre_target_lane_seq;
    debug_pre_target_lane_seq.emplace_back("Get same pre_target_lane_seq:");
    if (map->type() == ad_byd::planning::MapType::BEV_MAP ||
        planner_state.prev_map_type == ad_byd::planning::MapType::BEV_MAP ||
        planner_state.prev_success_map_type ==
            ad_byd::planning::MapType::BEV_MAP) {
      std::vector<ad_byd::planning::Point2d> debug_match_point;
      pre_target_lane_seq = map->GetSameLaneSequenceV2(
          last_tgt_seq, ego_pos.x(), ego_pos.y(), ego_theta,
          debug_pre_target_lane_seq, debug_match_point);
      // Log2DDS::LogPointsV2("debug_match_point_tgt", Log2DDS::kGreen, {},
      //                      debug_match_point, 10.0);
    } else {
      pre_target_lane_seq =
          map->GetSameLaneSequence(last_tgt_seq, ego_pos.x(), ego_pos.y());
      if (pre_target_lane_seq && !pre_target_lane_seq->lanes().empty()) {
        std::ostringstream stream;
        for (const auto& lane_ptr : pre_target_lane_seq->lanes()) {
          if (!lane_ptr) break;
          stream << lane_ptr->id() << ", ";
        }
        debug_pre_target_lane_seq.emplace_back(stream.str());
      }
    }
    Log2DDS::LogDataV2("match_debug", debug_pre_target_lane_seq);
    DLOG(INFO) << (pre_target_lane_seq == nullptr);

    // forward extent
    if (pre_target_lane_seq && !pre_target_lane_seq->lanes().empty()) {
      std::vector<LaneConstPtr> lanes = pre_target_lane_seq->lanes();
      std::set<uint64_t> lane_set;
      for (const auto& lane : lanes) {
        if (lane) lane_set.insert(lane->id());
      }
      while (lanes.back()) {
        LaneConstPtr next_lane = map->GetOptimalNextLane(lanes.back(), true);
        if (!next_lane || lane_set.find(next_lane->id()) != lane_set.end()) {
          break;
        }
        lanes.emplace_back(next_lane);
        lane_set.insert(next_lane->id());
      }
      pre_target_lane_seq =
          std::make_shared<ad_byd::planning::LaneSequence>(lanes);
    }
  }
  if (!pre_target_lane_seq) {
    auto nearest_lane = map->GetNearestLane(ego_pos, ego_theta,
                                            /*LatDist =*/3.2, is_navi, false);
    if (!nearest_lane) {
      nearest_lane = map->GetNearestLane(ego_pos, ego_theta, /*LatDist =*/4.2,
                                         is_navi, false, 65.0 * M_PI / 180.0);
      if (!nearest_lane) {
        return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                             "Cant get nearest lane 1!");
      }
    }
    pre_target_lane_seq = map->GetLaneSequence(nearest_lane, is_navi);
    Log2DDS::LogDataV0("lc_pnp_prevent_jump",
                       "prev target obtained from nearest lane.");
  }
  // Check if target lane jumped
  const auto& last_tgt_seq = planner_state.prev_target_lane_path.lane_seq();
  if (last_tgt_seq && pre_target_lane_seq) {
    double prev_s, prev_l, s, l;
    const auto& prev_nearest_lane = last_tgt_seq->GetNearestLane(ego_pos);
    const auto& nearest_lane = pre_target_lane_seq->GetNearestLane(ego_pos);
    if (!prev_nearest_lane || !nearest_lane) {
      return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                           "Cant get nearest lane 2!");
    }
    if (prev_nearest_lane->center_line().GetProjection(ego_pos, &prev_s,
                                                       &prev_l) &&
        nearest_lane->center_line().GetProjection(ego_pos, &s, &l)) {
      const bool is_map_type_changed =
          (map->type() == ad_byd::planning::MapType::BEV_MAP &&
           planner_state.prev_map_type == ad_byd::planning::MapType::HD_MAP) ||
          (map->type() == ad_byd::planning::MapType::HD_MAP &&
           planner_state.prev_map_type == ad_byd::planning::MapType::BEV_MAP);
      const double threshold = is_map_type_changed ? 2.0 : 1.0;
      if (std::fabs(l - prev_l) > threshold) {
        return PlannerStatus(PlannerStatusProto::TARGET_LANE_JUMPED_FAIL,
                             "Match target lane jumped!");
      }
    }
  } else if (planner_state.prev_est_planner_status_code ==
                 PlannerStatusProto::TARGET_LANE_JUMPED_FAIL &&
             !last_tgt_seq && pre_target_lane_seq) {
    double s, l;
    const auto& nearest_lane = pre_target_lane_seq->GetNearestLane(ego_pos);
    if (!nearest_lane) {
      return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                           "Cant get nearest lane 2!");
    }
    if (nearest_lane->center_line().GetProjection(ego_pos, &s, &l)) {
      if (std::fabs(l) > 1.0) {
        return PlannerStatus(PlannerStatusProto::TARGET_LANE_JUMPED_FAIL,
                             "Depart from ego lane!");
      }
    }
  }
  // Update cruising_speed_limit by pre_target_lane_seq
  cruising_speed_limit = UpdateCruisingSpeedLimitByLabel(
      psmm.map_ptr(), plan_start_point.v(), *cruising_speed_limit,
      pre_target_lane_seq);
  // LOG_ERROR << "cruising_speed_limit: " << cruising_speed_limit.value();

  mapping::LanePath prev_target_lane_path_from_start;
  mapping::LanePath prev_lane_path_before_lc_from_start;
  if (pre_target_lane_seq) {
    std::vector<uint64_t> lane_ids;
    const auto& nearest_lane = pre_target_lane_seq->GetNearestLane(ego_pos);
    if (!nearest_lane) {
      return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                           "Cant get nearest lane 2!");
    }
    bool find_start = false;
    for (const auto& lane : pre_target_lane_seq->lanes()) {
      if (lane && nearest_lane && nearest_lane->id() == lane->id()) {
        find_start = true;
      }
      if (!find_start) continue;
      if (!lane || lane->id() == 0 || !lane->center_line().IsValid()) break;
      lane_ids.emplace_back(lane->id());
    }
    st::mapping::LanePath lane_path(map, lane_ids, 0.0, 1.0);
    LOG_ERROR << "use planner_state prev target lane path update!";
    prev_target_lane_path_from_start = lane_path;
    prev_lane_path_before_lc_from_start = lane_path;
  } else {
    return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                         "Get pre_target_lane_seq fail!");
  }
  ad_byd::planning::LaneSequencePtr pre_lane_seq_before_lc = nullptr;
  if (!planner_state.prev_lane_path_before_lc.IsEmpty()) {
    std::vector<ad_byd::planning::LaneConstPtr> lanes;
    for (const auto& id : planner_state.prev_lane_path_before_lc.lane_ids()) {
      const auto& lane = map->GetLaneById(id);
      if (lane) {
        lanes.emplace_back(lane);
      }
    }
    ad_byd::planning::LaneSequencePtr pre_lane_seq =
        planner_state.prev_lane_path_before_lc.lane_seq();
    std::vector<std::string> debug_pre_lane_seq_before_lc;
    debug_pre_lane_seq_before_lc.emplace_back(
        "Get same pre_lane_seq_before_lc:");
    if (map->type() == ad_byd::planning::MapType::BEV_MAP) {
      std::vector<ad_byd::planning::Point2d> debug_match_point;
      pre_lane_seq_before_lc = map->GetSameLaneSequenceV2(
          pre_lane_seq, ego_pos.x(), ego_pos.y(), ego_theta,
          debug_pre_lane_seq_before_lc, debug_match_point);
      // Log2DDS::LogPointsV2("debug_match_point_tgt", Log2DDS::kGreen, {},
      //                      debug_match_point, 10.0);
    } else {
      pre_lane_seq_before_lc =
          map->GetSameLaneSequence(pre_lane_seq, ego_pos.x(), ego_pos.y());
      if (pre_lane_seq_before_lc && !pre_lane_seq_before_lc->lanes().empty()) {
        std::ostringstream stream;
        for (const auto& lane_ptr : pre_lane_seq_before_lc->lanes()) {
          if (!lane_ptr) break;
          stream << lane_ptr->id() << ", ";
        }
        debug_pre_lane_seq_before_lc.emplace_back(stream.str());
      }
    }
    Log2DDS::LogDataV2("match_debug", debug_pre_lane_seq_before_lc);

    // forward extent
    if (pre_lane_seq_before_lc && !pre_lane_seq_before_lc->lanes().empty()) {
      std::vector<LaneConstPtr> lanes = pre_lane_seq_before_lc->lanes();
      std::set<uint64_t> lane_set;
      for (const auto& lane : lanes) {
        if (lane) lane_set.insert(lane->id());
      }
      while (lanes.back()) {
        LaneConstPtr next_lane = map->GetOptimalNextLane(lanes.back(), true);
        if (!next_lane || lane_set.find(next_lane->id()) != lane_set.end()) {
          break;
        }
        lanes.emplace_back(next_lane);
        lane_set.insert(next_lane->id());
      }
      pre_lane_seq_before_lc =
          std::make_shared<ad_byd::planning::LaneSequence>(lanes);
    }
    if (pre_lane_seq_before_lc) {
      std::vector<uint64_t> lane_ids;
      const auto& nearest_lane =
          pre_lane_seq_before_lc->GetNearestLane(ego_pos);
      if (!nearest_lane) {
        return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                             "Cant get nearest lane 3!");
      }
      bool find_start = false;
      for (const auto& lane : pre_lane_seq_before_lc->lanes()) {
        if (lane && nearest_lane && nearest_lane->id() == lane->id()) {
          find_start = true;
        }
        if (!find_start) continue;
        if (!lane || lane->id() == 0 || !lane->center_line().IsValid()) break;
        lane_ids.emplace_back(lane->id());
      }
      LOG_ERROR << "use planner_state_ pre lane path before lc update!";
      st::mapping::LanePath lane_path(map, lane_ids, 0.0, 1.0);
      prev_lane_path_before_lc_from_start = lane_path;
    }
  }
  Log2DDS::LogDataV0("UpdatePrevTarLanePath", multitask_timer.TimeUs() / 1e3);
  // -------------------------------------------------------
  //            2 ScheduleCandidateLaneSequences
  // -------------------------------------------------------
  multitask_timer.Reset("ScheduleCandidateLaneSequences");
  std::set<uint64_t> navi_start_lanes;
  auto& navi_start = map->route()->navi_start();
  // EIE funtion type judgement
  bool eie_braking_down_flag = false;
  ad_byd::planning::EIEChoiceType eie_choice_type =
      ad_byd::planning::EIEChoiceType::CHOICE_NONE;
  if (behavior_choice ==
      byd::msg::planning::BehaviorChoice::BEHAVIOR_CHOICE_MRM) {
    if (function_id == Behavior_FunctionId_LKA ||
        function_id == Behavior_FunctionId_CITY_NOA) {
      eie_choice_type = ad_byd::planning::EIEChoiceType::CHOICE_BRAKING_DOWN;
      eie_braking_down_flag = true;
    } else if (function_id == Behavior_FunctionId_HW_NOA) {
      eie_choice_type = ad_byd::planning::EIEChoiceType::CHOICE_RIGHT_LANE;
    }
  }
  // ad_byd::planning::SectionPtr section;
  if (!(navi_start.section_id == 0)) {
    const auto& navi_section = map->GetSectionById(navi_start.section_id);
    if (navi_section) {
      for (auto& id : navi_section->lanes()) {
        navi_start_lanes.insert(id);
      }
    }
  }
  ad_byd::planning::LaneConstPtr nearest_lane_on_pre_path = nullptr;
  if (pre_target_lane_seq) {
    nearest_lane_on_pre_path =
        pre_target_lane_seq->GetNearestLane(ego_pos, ego_theta);
  }
  auto nearest_lane =
      map->GetNearestLane(ego_pos, ego_theta, /*LatDist =*/3.2, is_navi, false);
  if (!nearest_lane) {
    TIMELINE("RunMultiTasksCruisePlanner.GetNearestLane");
    nearest_lane = map->GetNearestLane(ego_pos, ego_theta, /*LatDist =*/4.2,
                                       is_navi, false, 65.0 * M_PI / 180.0);
    if (!nearest_lane) {
      return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                           "Cant get nearest lane 4!");
    }
  }
  Log2DDS::LogDataV0(
      "nearest_lane",
      absl::StrCat("nearest lane in map: ",
                   (nearest_lane ? absl::StrCat(nearest_lane->id())
                                 : "cant get nearest lane in map")));
  ad_byd::planning::LaneConstPtr start_lane = nullptr;
  double min_dist = DBL_MAX, start_lane_offset = 0.0;
  for (const auto& lane_id : navi_start_lanes) {
    const auto& lane = map->GetLaneById(lane_id);
    if (!lane) continue;
    double s, l;
    if (!lane->center_line().GetProjection(ego_pos, &s, &l)) continue;
    if (std::abs(l) < min_dist) {
      if (lane->type() == LaneType::LANE_EMERGENCY) {
        const double clamp_width = vehicle_geometry.width() * 0.5;
        double current_lane_half_width =
            0.5 * lane->GetWidthAtPoint(ego_pos.x(), ego_pos.y());
        if ((abs(l) < current_lane_half_width - clamp_width) ||
            (navi_start_lanes.size() == 1)) {
          start_lane = lane;
          min_dist = std::abs(l);
          start_lane_offset = s;
        }
      } else {
        start_lane = lane;
        min_dist = std::abs(l);
        start_lane_offset = s;
      }
    }
  }
  constexpr double kNaviStartOffsetError = 10.0;
  if (nearest_lane && start_lane && nearest_lane->id() != start_lane->id() &&
      std::abs(navi_start.s_offset - start_lane_offset) >
          kNaviStartOffsetError) {
    Log2DDS::LogDataV0("nearest_lane",
                       absl::StrCat("navi start error from env!",
                                    " start lane: ", start_lane->id(),
                                    ", env offset: ", navi_start.s_offset,
                                    ", actual offset: ", start_lane_offset));
  }
  Log2DDS::LogDataV0(
      "nearest_lane",
      absl::StrCat("nearest lane in navi start: ",
                   (start_lane ? absl::StrCat(start_lane->id())
                               : "cant get nearest lane in navi start")));
  if (is_navi && start_lane == nullptr) {
    return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                         "Can't find nearest lane!");
  }
  if (!is_navi) {
    start_lane = nearest_lane;
  }
  Log2DDS::LogDataV0(
      "nearest_lane",
      absl::StrCat("nearest lane on pre target path: ",
                   (nearest_lane_on_pre_path
                        ? absl::StrCat(nearest_lane_on_pre_path->id())
                        : "None")));
  if (map) {
    TIMELINE("RMTCP.UpdateNaviPriorityLanes");
    if (nearest_lane_on_pre_path)
      map->UpdateNaviPriorityLanes(nearest_lane_on_pre_path);
    else
      map->UpdateNaviPriorityLanes(nearest_lane);
  }

  // Handle scene lc intent
  ad_byd::planning::LaneSequencePtr scene_target_lane_seq = nullptr;
  LaneChangeReason new_lane_change_reason = LaneChangeReason::NO_CHANGE;
  // when CHOICE_RIGHT_LANE,keep lk 6s
  auto new_lc_command = input.new_lc_command;
  double eie_time_counter_diff = 0.0;
  constexpr double eie_time_counter_threshold = 6.0;
  output->eie_time_counter_prev = planner_state.eie_time_counter_prev;
  output->eie_choice_type_prev = planner_state.eie_choice_type_prev;
  if (eie_choice_type == ad_byd::planning::EIEChoiceType::CHOICE_RIGHT_LANE) {
    const auto now_time = absl::Now();
    double eie_time_counter = ToUnixDoubleSeconds(now_time);
    if (output->eie_choice_type_prev !=
        ad_byd::planning::EIEChoiceType::CHOICE_RIGHT_LANE) {
      output->eie_time_counter_prev = eie_time_counter;
    }
    eie_time_counter_diff = eie_time_counter - output->eie_time_counter_prev;
  }
  output->eie_choice_type_prev = eie_choice_type;
  // EIEChoiceType::CHOICE_RIGHT_LANE
  if (eie_time_counter_diff > eie_time_counter_threshold) {
    new_lc_command = st::DriverAction::LC_CMD_RIGHT;
  }
  const std::optional<int> tunnel_status = input.behavior->tunnel();
  std::vector<ad_byd::planning::LaneSequencePtr> candidate_lane_seqs;
  Log2DDS::LogDataV0(
      "lc_pnp_prevent_jump",
      absl::StrCat("pre lc stage: ", planner_state.lane_change_state.stage()));
  // double ego_s_offset = 0.0;
  // double ego_l_offset = 0.0;
  // nearest_lane->center_line().GetProjection(ego_pos, &ego_s_offset,
  //                                           &ego_l_offset);
  bool if_continuous_lc_secnario = false;
  const ad_byd::planning::BehaviorCommand& intention_dir =
      planner_state.selector_state.intention_dir;
  Log2DDS::LogDataV2("intention_dir", intention_dir);

  bool is_on_highway = map->is_on_highway();
  bool is_hw_noa = function_id == Behavior_FunctionId_HW_NOA;
  bool is_city_noa = function_id == Behavior_FunctionId_CITY_NOA;
  if (map->is_on_highway() || is_hw_noa) {
    bool need_fix_hw_to_city =
        ((lite_map && lite_map->dist_to_junction() < 200.0) ||
         (map->route() && map->route()->GetDistToJunctionEndOnNavi() < 200.0));
    if (need_fix_hw_to_city) {
      is_on_highway = false;
      if (is_hw_noa) {
        is_hw_noa = false;
        is_city_noa = true;
      }
    }
  }

  if (is_on_highway || is_hw_noa ||
      function_id == Behavior_FunctionId_LKA_PLUS ||
      function_id == Behavior_FunctionId_LKA ||
      function_id == Behavior_FunctionId_NONE ||
      function_id == Behavior_FunctionId_ACC) {
    TIMELINE("RMTCP.FsdHighwayScheduleCandidateLaneSequences");
    FsdHighwayScheduleCandidateLaneSequences(
        ego_pos, ego_theta, map, start_lane, plan_start_point.v(),
        planner_state.lane_change_state, function_id, pre_target_lane_seq,
        pre_lane_seq_before_lc,
        planner_state.selector_state.selected_target_lane_state.is_borrow(),
        planner_state.nudge_object_info.has_value(), &candidate_lane_seqs,
        new_lc_command, intention_dir, eie_choice_type);
  } else if (is_city_noa) {
    TIMELINE("RMTCP.FsdCityScheduleCandidateLaneSequences");
    bool is_pre_seq_valid =
        (planner_state.pre_funciton_id == Behavior_FunctionId_CITY_NOA ||
         planner_state.pre_funciton_id == Behavior_FunctionId_HW_NOA) &&
        (planner_state.prev_success_map_type == map->type());
    ad_byd::planning::LaneSequencePtr pre_target_lane_seq_used =
        is_pre_seq_valid ? pre_target_lane_seq : nullptr;
    FsdCityScheduleCandidateLaneSequences(
        ego_pos, ego_theta, map, start_lane, plan_start_point.v(),
        planner_state.lane_change_state, pre_target_lane_seq_used,
        pre_lane_seq_before_lc, &candidate_lane_seqs, new_lc_command,
        intention_dir);
  } else {
    return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                         "Can't Get Candidate laneseqs!");
  }
  Log2DDS::LogDataV0("ScheduleCandidateLaneSequences",
                     multitask_timer.TimeUs() / 1e3);
  // CHOICE_RIGHT_LANE,braking down stage
  if (!candidate_lane_seqs.empty() &&
      (eie_choice_type == ad_byd::planning::EIEChoiceType::CHOICE_RIGHT_LANE) &&
      (planner_state.lane_change_state.stage() ==
       st::LaneChangeStage::LCS_NONE)) {
    bool has_right_lane_seqs = false;
    for (int i = 0; i < candidate_lane_seqs.size(); ++i) {
      auto lane_seq = candidate_lane_seqs.at(i);
      if (lane_seq->GetSequenceDirection() ==
          ad_byd::planning::SequenceDirection::Right) {
        has_right_lane_seqs = true;
      }
    }
    if (!has_right_lane_seqs) eie_braking_down_flag = true;
  }
  // target_lane_seq->nearest_lane->type() == LANE_RAMP ||
  //     target_lane_seq->nearest_lane->type() == LANE_DEC
  // tunnel and ramp
  if ((candidate_lane_seqs.size() == 1) && eie_braking_down_flag &&
      ((tunnel_status.has_value() && (tunnel_status.value() != 0)) ||
       (candidate_lane_seqs[0]->lanes().front()->type() ==
        LaneType::LANE_RAMP))) {
    eie_braking_down_flag = false;
  }

  // -------------------------------------------------------
  //                3 Handle Manual Lc Command
  // -------------------------------------------------------
  multitask_timer.Reset("HandleManualLc");
  auto updated_preferred_lane_path = planner_state.preferred_lane_path;
  auto new_alc_state = planner_state.alc_state;
  auto new_lc_cmd_state = planner_state.lane_change_command;
  auto last_lc_reason =
      planner_state.last_lc_reason;  // 上一次变道的原因（不是上一帧变道的原因）
  auto last_manual_lc_time = planner_state.last_manual_lc_time;
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("last_lc_reason", last_lc_reason));
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("last_manual_lc_time", last_manual_lc_time));
  const auto& alc_request =
      planner_state.selector_state.selector_lane_change_request;
  const bool alc_request_waiting =
      alc_request.has_lane_change_reason() &&
      alc_request.lane_change_reason() != LaneChangeReason::NO_CHANGE;
  const auto input_new_lc_cmd =
      alc_request_waiting ? DriverAction::LC_CMD_NONE : new_lc_command;
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("alc_wait_request: ", alc_request_waiting));
  output->input_lc_cmd = new_lc_command;

  if (input_new_lc_cmd != new_lc_command) {
    LOG_INFO << "Ignore driver action: "
             << DriverAction_LaneChangeCommand_Name(new_lc_command)
             << ", when alc request waits.";
  }

  {
    TIMELINE("RunMultiTasksCruisePlanner::HandleManualLcCommand");
    HandleManualLcCommand(
        plan_start_point, vehicle_geometry, psmm, input_new_lc_cmd,
        planner_state.prev_lane_path_before_lc, planner_state.lane_change_state,
        ego_pos, ego_theta, &updated_preferred_lane_path, &new_alc_state,
        &new_lc_cmd_state, &output->lc_unable_reason, &output->if_cancel_lc,
        &output->if_allow_cancel_lc, last_manual_lc_time, last_lc_reason,
        candidate_lane_seqs, eie_choice_type);
  }
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("raw_lc_cmd: ", input_new_lc_cmd));
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("new_alc_state: ", new_alc_state));
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("new_lc_cmd: ", new_lc_cmd_state));
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("if_cancel_lc: ", output->if_cancel_lc));
  Log2DDS::LogDataV2("mlc_debug", absl::StrCat("if_allow_cancel_lc: ",
                                               output->if_allow_cancel_lc));

  bool auto_model = input.auto_model;
  bool is_eie_cnoa_action = planner_state.speed_state.is_eie_cnoa_action;
  if (function_id == Behavior_FunctionId_CITY_NOA ||
      function_id == Behavior_FunctionId_HW_NOA &&
          eie_choice_type == ad_byd::planning::EIEChoiceType::CHOICE_NONE) {
    auto_model = input.auto_model;
  } else {
    auto_model = false;
  }

  if (planner_state.lane_change_state.stage() != LCS_NONE) {
    auto_model = true;
  }
  Log2DDS::LogDataV0("lc_debug", absl::StrCat("auto_model: ", auto_model));

  // check if ego_lane_seq has merge topo in ICC
  bool has_merge = false;
  MergeTopology merge_topo = MergeTopology::TOPOLOGY_MERGE_NONE;
  if (!candidate_lane_seqs.empty() && function_id == Behavior_FunctionId_LKA) {
    const auto merge_topo_status =
        CheckIfHasMerge(psmm, ego_pos, candidate_lane_seqs);
    has_merge = merge_topo_status.ok();
    if (has_merge) {
      merge_topo = merge_topo_status.value();
    }
  }
  Log2DDS::LogDataV0("icc_task_debug", absl::StrCat("has_merge: ", has_merge));

  std::vector<ad_byd::planning::LaneSequencePtr> candidate_lane_seqs_select;
  if (auto_model || !updated_preferred_lane_path.IsEmpty()) {
    for (const auto& candidate_lane_seq : candidate_lane_seqs) {
      candidate_lane_seqs_select.emplace_back(candidate_lane_seq);
    }
  } else if (!candidate_lane_seqs.empty() && !auto_model &&
             updated_preferred_lane_path.IsEmpty()) {
    if (has_merge) {
      // retain two lane_seqs if ego_lane_seq has merge topo in ICC
      std::unordered_set<ad_byd::planning::SequenceDirection> allowed_dirs{
          ad_byd::planning::SequenceDirection::Cur};
      if (merge_topo == MergeTopology::TOPOLOGY_MERGE_LEFT) {
        allowed_dirs = {ad_byd::planning::SequenceDirection::Left,
                        ad_byd::planning::SequenceDirection::Cur};
      } else if (merge_topo == MergeTopology::TOPOLOGY_MERGE_RIGHT) {
        allowed_dirs = {ad_byd::planning::SequenceDirection::Right,
                        ad_byd::planning::SequenceDirection::Cur};
      }
      for (const auto& candidate_lane_seq : candidate_lane_seqs) {
        const auto lane_seq_dir = candidate_lane_seq->GetSequenceDirection();
        if (allowed_dirs.find(lane_seq_dir) != allowed_dirs.end()) {
          candidate_lane_seqs_select.emplace_back(candidate_lane_seq);
        }
      }
      if (candidate_lane_seqs_select.empty()) {
        return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                             "Can't Get Candidate laneseqs!");
      }
    } else {
      const auto& pre_tgt_lane = pre_target_lane_seq->GetNearestLane(ego_pos);
      auto& updated_nearest_lane = nearest_lane;
      Log2DDS::LogDataV0(
          "lc_debug",
          absl::StrCat("auto_model_nearest_lane: ", nearest_lane->id(),
                       ", pre_tgt_lane: ", pre_tgt_lane->id()));
      auto temp_dist =
          pre_target_lane_seq->GetProjectionDistance(ego_pos, nullptr);
      if (temp_dist < 0.5) {
        updated_nearest_lane = pre_tgt_lane;
      }
      for (const auto& candidate_lane_seq : candidate_lane_seqs) {
        bool is_lc_seq = true;
        for (const auto& lane : candidate_lane_seq->lanes()) {
          if (lane && lane->id() == updated_nearest_lane->id()) {
            is_lc_seq = false;
            break;
          }
        }
        if (!is_lc_seq) {
          candidate_lane_seqs_select.emplace_back(candidate_lane_seq);
        }
      }
      if (candidate_lane_seqs_select.empty() && pre_target_lane_seq &&
          !pre_target_lane_seq->lanes().empty()) {
        candidate_lane_seqs_select.emplace_back(pre_target_lane_seq);
      }
    }
  } else {
    return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                         "Can't Get Candidate laneseqs!");
  }

  bool icc_lc_enable = false;
  if (!candidate_lane_seqs_select.empty() &&
      candidate_lane_seqs_select.size() > 1) {
    icc_lc_enable = has_merge;
  }
  Log2DDS::LogDataV0("icc_task_debug",
                     absl::StrCat("icc_lc_enable: ", icc_lc_enable));
  output->icc_lc_enable = icc_lc_enable;

  bool is_open_gap = false;
  if (!candidate_lane_seqs_select.empty() &&
      candidate_lane_seqs_select.size() > 1 &&
      (planner_state.begin_route_change_left.has_value() ||
       planner_state.is_going_force_route_change_left.has_value()) &&
      planner_state.lane_change_state.stage() == LCS_NONE) {
    is_open_gap = true;
  }
  // no gap when borrow
  if (planner_state.nudge_object_info.has_value() &&
      planner_state.nudge_object_info.value().nudge_state ==
          NudgeObjectInfo::NudgeState::BORROW) {
    is_open_gap = false;
  }

  std::vector<std::pair<uint64_t, double>> nearest_section_lanes;
  if (lite_map) {
    ad_byd::planning::LaneConstPtr nearest_section_target_lane =
        nearest_lane_on_pre_path ? nearest_lane_on_pre_path : nearest_lane;
    const auto& nearest_section =
        map->GetSectionById(nearest_section_target_lane->section_id());
    if (nearest_section) {
      nearest_section_lanes.reserve(nearest_section->lanes().size());
      for (const auto lane_id : nearest_section->lanes()) {
        const auto& lane = map->GetLaneById(lane_id);
        if (!lane) continue;
        double s, l;
        if (!lane->center_line().GetProjection(ego_pos, &s, &l)) continue;
        if (s < -1.0 || s > lane->curve_length() + 1.0) continue;
        nearest_section_lanes.emplace_back(lane_id, l);
      }
      std::sort(nearest_section_lanes.begin(), nearest_section_lanes.end(),
                [&](const std::pair<uint64_t, double>& a,
                    const std::pair<uint64_t, double>& b) {
                  return a.second < b.second;
                });
    }
    psmm.SectionLanesFilter(ego_pos, ego_theta, nearest_section_lanes);
    int nearest_section_target_lane_idx = 0;
    int valid_nearest_lane_nums = nearest_section_lanes.size();
    for (int i = 0; i < valid_nearest_lane_nums; i++) {
      Log2DDS::LogDataV2(
          "ld_lite_debug",
          absl::StrCat("lane_id: ", nearest_section_lanes[i].first, ", index: ",
                       i + 1, ", l: ", nearest_section_lanes[i].second));
      if (nearest_section_lanes[i].first == nearest_section_target_lane->id()) {
        nearest_section_target_lane_idx = i + 1;
        // break;
      }
    }
    lite_map->UpdateNaviStartPriority(nearest_section_target_lane_idx,
                                      valid_nearest_lane_nums);
  }
  // auto num_0 = psmm.GetValidLanesInRange(ego_pos, 0);
  // auto num_150 = psmm.GetValidLanesInRange(ego_pos, 150);
  Log2DDS::LogDataV0("HandleManualLc", multitask_timer.TimeUs() / 1e3);
  // ----------------------------------------------------------
  // ---------------- 4 Trans seq to lane path ----------------
  // ----------------------------------------------------------
  multitask_timer.Reset("TransSeqToLanePath");
  std::vector<st::mapping::LanePath> lane_paths;
  std::vector<LanePathInfo> target_lp_infos;
  std::vector<ad_byd::planning::LaneSeqInfo> lane_seq_infos;
  bool if_miss_navi_secnario = false;
  std::string leading_id = "";
  int cur_section_lane_num = 0;
  int cur_lane_position = -1;
  int cur_navi_lc_num = 0;
  double left_navi_dist_v2 = 999.0;
  double left_navi_dist = 999.0;
  double cur_dist_to_junction = DBL_MAX;
  double cur_dist_to_prev_junction = DBL_MAX;
  ad_byd::planning::V2TurnInfo::V2DetailTurnType cur_nearest_turn_type =
      ad_byd::planning::V2TurnInfo::V2DetailTurnType::NONE;
  ad_byd::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2 =
      ad_byd::planning::V2TurnInfo::V2DetailTurnType::NONE;

  for (const auto& candidate_lane_seq : candidate_lane_seqs_select) {
    std::vector<st::mapping::ElementId> lane_id_infos;
    double accm_s = 0.0;
    const ad_byd::planning::LaneConstPtr start_lane =
        *(candidate_lane_seq->lanes().begin());
    for (const auto& lane : candidate_lane_seq->lanes()) {
      if (!lane || lane->id() == 0 || !lane->center_line().IsValid() ||
          std::count(lane_id_infos.begin(), lane_id_infos.end(), lane->id())) {
        if (lane) {
          LOG_ERROR << "lane id = " << lane->id()
                    << " valid = " << lane->center_line().IsValid();
        } else {
          LOG_ERROR << "lane is null, fatal error";
        }
        break;
      }
      lane_id_infos.emplace_back(lane->id());
      accm_s += lane->curve_length();
    }

    if (lane_id_infos.empty()) {
      continue;
    }
    double start_fraction = 0.0;
    const auto ff = BuildBruteForceFrenetFrame(
        start_lane->points(), /*down_sample_raw_points=*/false);
    if (ff.ok()) {
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha;
      ff.value().XYToSL(ego_pos, &sl, &normal, &index_pair, &alpha);
      sl.s = std::max(0.0, sl.s - 30.0);
      start_fraction = std::clamp(
          sl.s / std::fmax(start_lane->curve_length(), 1e-2), 0.0, 1.0 - 1e-2);
    }

    st::mapping::LanePath lane_path(map, lane_id_infos, start_fraction, 1.0);
    auto lane_path_info = LanePathInfo(lane_path, accm_s, 0.0, psmm);
    lane_path_info.set_lane_seq(candidate_lane_seq);
    // Add lane seq info into lane path info
    ad_byd::planning::LaneSeqInfo temp_seq_info;
    bool result =
        GetLaneSeqInfo(nearest_lane, function_id, ego_pos, plan_start_point.v(),
                       candidate_lane_seq, map, lite_map, nearest_section_lanes,
                       &temp_seq_info);
    const Behavior behavior = input.behavior.value();
    const auto map_func_id = behavior.function_id();
    if (result) {
      if (temp_seq_info.is_current &&
          IfMissNaviScenarioForCur(&temp_seq_info, ego_pos, map_func_id)) {
        if_miss_navi_secnario = true;
        if (plan_start_point.v() < 16.67) {
          Log2DDS::LogDataV2("lane_change_style", "miss navi lc style");
          lane_change_style = LC_STYLE_RADICAL;
        }
      }
      if (temp_seq_info.is_current) {
        cur_navi_lc_num = temp_seq_info.lc_num;
        left_navi_dist = temp_seq_info.dist_to_navi_end;
        left_navi_dist_v2 = temp_seq_info.dist_to_navi_end_v2;
        cur_dist_to_junction = temp_seq_info.dist_to_junction;
        cur_nearest_turn_type = temp_seq_info.nearest_turn_type_v2;
        last_turn_type_v2 = temp_seq_info.last_turn_type_v2;
        cur_section_lane_num = temp_seq_info.cur_section_lane_num;
        cur_lane_position = temp_seq_info.cur_lane_position;
      }
      lane_path_info.set_lane_seq_info(temp_seq_info);
      lane_seq_infos.emplace_back(temp_seq_info);
    } else {
      temp_seq_info.lane_seq = candidate_lane_seq;
      lane_path_info.set_lane_seq_info(temp_seq_info);
    }
    target_lp_infos.emplace_back(lane_path_info);
  }
  cur_dist_to_prev_junction =
      planner_state.cur_max_dist_to_junction - cur_dist_to_junction;
  output->cur_max_dist_to_junction = planner_state.cur_max_dist_to_junction;
  if ((planner_state.prev_dist_to_junction < 0.1) &&
      (cur_dist_to_junction > 10.0)) {
    output->cur_max_dist_to_junction = cur_dist_to_junction;
    cur_dist_to_prev_junction = 0.0;
  }
  output->prev_dist_to_junction = cur_dist_to_junction;
  
  CalcuMinMergeLcNum(map, ego_pos, target_lp_infos, cur_navi_lc_num);

  double dist_to_tunnel_entrance = 10000.0;
  double dist_to_tunnel_exitance = 10000.0;

  if (input.map_event != nullptr) {
    const auto& remind_waypoints = input.map_event->reminder_way_info();

    for (int i = 0; i < remind_waypoints.size(); i++) {
      const auto& remind_waypoint = remind_waypoints.at(i);
      const auto& remind_type = remind_waypoint.reminder_waypoint_type();
      if (remind_type == byd::msg::orin::routing_map::ReminderWayInfo::TUNNEL) {
        dist_to_tunnel_entrance = remind_waypoint.reminder_waypoint_dis();
        dist_to_tunnel_exitance = remind_waypoint.reminder_waypoint_end_dis();
        break;
      }
    }
  }
  // turnning light decide
  // TurnSignal turn_type = TURN_SIGNAL_NONE;
  // switch (cur_nearest_turn_type) {
  //   case ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT:
  //     if (left_navi_dist_v2 < 30 && left_navi_dist_v2 > 0 &&
  //         cur_lane_position == 1) {
  //       turn_type = TURN_SIGNAL_LEFT;
  //     }
  //     break;
  //   case ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT:
  //     if (left_navi_dist_v2 < 30 && left_navi_dist_v2 > 0 &&
  //         cur_lane_position == cur_section_lane_num) {
  //       turn_type = TURN_SIGNAL_RIGHT;
  //     }
  //     break;
  //   case ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT_ONLY:
  //     if (left_navi_dist_v2 < 30 && left_navi_dist_v2 > 0 &&
  //         cur_lane_position == cur_section_lane_num) {
  //       turn_type = TURN_SIGNAL_RIGHT;
  //     }
  //     break;
  //   default:
  //     break;
  // }
  // output->turn_type_signal = turn_type;

  std::vector<LanePathInfo>& lp_infos = target_lp_infos;
  // LOG_ERROR << "lp_infos: " << lp_infos.size();
  // for (auto info : lp_infos) {
  //   LOG_ERROR << "llp info : " << info.lane_path().DebugString();
  //   LOG_ERROR << "lane path info: " << info.lane_seq_info()->lc_num << ", "
  //   << info.lane_seq_info()->dist_to_navi_end << ", " <<
  //   info.lane_seq_info()->dist_to_junction << "\n";
  // }

  if (target_lp_infos.empty()) {
    // In case all lane paths are blocked by stalled objects.
    return PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
                         "No valid target lane path from the current section.");
  }
  Log2DDS::LogDataV0("TransSeqToLanePath", multitask_timer.TimeUs() / 1e3);
  // --------------------------------------------------------------
  // ----------------- 5 Run Scheduler ----------------------------
  // --------------------------------------------------------------
  multitask_timer.Reset("RunSchedulerAll");
  MultiTasksSchedulerInput scheduler_input{
      .psmm = &psmm,
      .vehicle_geom = &vehicle_geometry,
      .st_traj_mgr = input.st_traj_mgr ? &input.st_traj_mgr.value() : nullptr,
      .obj_mgr = input.object_manager ? &input.object_manager.value() : nullptr,
      .obj_history_mgr = &planner_state.object_history_manager,
      .lane_path_infos = &lp_infos,
      .planning_horizon = planning_horizon,
      .destination = &destination,
      // .tl_info_map = input.tl_info_map,
      .prev_smooth_state = planner_state.prev_smooth_state,
      .plan_start_point = &plan_start_point,
      .station_anchor = &planner_state.station_anchor,
      .start_route_s = 0.0,
      .smooth_result_map = &planner_state.smooth_result_map,
      .prev_target_lane_path_from_start = &prev_target_lane_path_from_start,
      .prev_lane_path_before_lc_from_start =
          &prev_lane_path_before_lc_from_start,
      .preferred_lane_path = &updated_preferred_lane_path,
      .prev_lc_state = &planner_state.lane_change_state,
      .cruising_speed_limit = cruising_speed_limit,
      .planner_is_l4_mode = true,
      .ego_pos = ego_pos,
      .miss_navi_scenario = if_miss_navi_secnario,
      .continuous_lc_scenario = if_continuous_lc_secnario,
      //.ext_cmd_status = input.ext_cmd_status,
      .is_navi = is_navi,
      .lc_cmd_state = new_lc_cmd_state,
      .behavior = &input.behavior.value(),
      .lc_push_dir = planner_state.lc_push_dir,
      .is_going_force_route_change_left =
          planner_state.is_going_force_route_change_left,
      .pre_push_status = &planner_state.pre_push_status,
      .stalled_objects =
          input.stalled_objects ? &input.stalled_objects.value() : nullptr,
      .saved_offset = &planner_state.saved_offset,
      .construction_info = &planner_state.construction_info,
      .pre_lane_change_safety_info = &planner_state.pre_lane_change_safety_info,
      .intention_dir = intention_dir};
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto multi_tasks,
      ScheduleMultiplePlanTasks(scheduler_input, target_lp_infos, thread_pool),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);
  // timer.Mark("scheduler");

  // TODO: Get last_st_path_plan_start_time from elsewhere.
  std::optional<absl::Time> last_st_path_plan_start_time;
  if (planner_state.selected_trajectory_optimizer_state_proto.has_value()) {
    last_st_path_plan_start_time = st::FromProto(
        planner_state.selected_trajectory_optimizer_state_proto.value()
            .last_plan_start_time());
  }

  // Find st path planner path_plan_start_point with given look ahead duration.
  const auto path_start_point_info = GetStPathPlanStartPointInfo(
      input.min_path_look_ahead_duration, *input.start_point_info,
      planner_state.previous_trajectory,
      CHECK_NOTNULL(input.planner_params)
          ->trajectory_optimizer_params()
          .trajectory_time_step(),
      last_st_path_plan_start_time);

  Log2DDS::LogDataV2(
      "time_shift",
      absl::StrFormat(
          " path_look_ahead_duration: %.3f, path_start_point_time:%.3f, "
          "start_point_time:%.3f",
          ToUnixDoubleSeconds(path_start_point_info.plan_time) -
              ToUnixDoubleSeconds(input.start_point_info->plan_time),
          ToUnixDoubleSeconds(path_start_point_info.plan_time),
          ToUnixDoubleSeconds(input.start_point_info->plan_time)));

  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_list;
  // +2 for possible fallback and expert results.
  st_traj_mgr_list.reserve(multi_tasks.size() + 2);
  st_traj_mgr_list.resize(multi_tasks.size());

  {
    TIMELINE("RMTCP.SpacetimeTrajectoryManagerBuilderInput");
    ParallelFor(0, multi_tasks.size(), thread_pool,
                [&multi_tasks, &input, &st_traj_mgr_list, &thread_pool,
                 &ego_box, &map, &psmm](int i) {
                  SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
                      .passage = &multi_tasks[i].drive_passage,
                      .sl_boundary = &multi_tasks[i].sl_boundary,
                      .obj_mgr = input.object_manager
                                     ? &input.object_manager.value()
                                     : nullptr,
                      .lc_state = &multi_tasks[i].lane_change_state,
                      .is_on_highway = map->is_on_highway(),
                      .ego_box = ego_box,
                      .plan_id = i,
                      .psmm = &psmm};
                  st_traj_mgr_list[i] = BuildSpacetimeTrajectoryManager(
                      st_mgr_builder_input, thread_pool);
                });
    for (int i = 0; i < st_traj_mgr_list.size(); ++i) {
      std::string considered_object = "";
      std::string ignored_object = "";
      std::string frame_dropped_object = "";
      for (int j = 0; j < st_traj_mgr_list[i].trajectories().size(); ++j) {
        considered_object +=
            std::string(st_traj_mgr_list[i].trajectories().at(j).object_id()) +
            ", ";
      }
      for (int j = 0; j < st_traj_mgr_list[i].ignored_trajectories().size();
           ++j) {
        ignored_object +=
            st_traj_mgr_list[i].ignored_trajectories().at(j).object_id + ", ";
      }
      for (int j = 0;
           j < st_traj_mgr_list[i].frame_dropped_trajectories().size(); ++j) {
        frame_dropped_object += std::string(st_traj_mgr_list[i]
                                                .frame_dropped_trajectories()
                                                .at(j)
                                                .object_id()) +
                                ", ";
      }
      Log2DDS::LogDataV0("st_traj_mgr_list",
                         absl::StrCat(i, " considered: ", considered_object));
      Log2DDS::LogDataV0("st_traj_mgr_list",
                         absl::StrCat(i, " ignored: ", ignored_object));
      Log2DDS::LogDataV0(
          absl::StrCat(Log2DDS::TaskPrefix(i), "st_traj_mgr_size"),
          st_traj_mgr_list[i].trajectories().size());
      Log2DDS::LogDataV0(
          "st_traj_mgr_list",
          absl::StrCat(i, " frame_drpped: ", frame_dropped_object));
    }
  }
  // traffic flow speed
  std::map<double, LaneObsInfo> all_lanes_obstacles_info;
  absl::flat_hash_set<std::string> has_checked_set;
  for (int i = 0; i < multi_tasks.size(); ++i) {
    const auto lane_obstacles_info = multi_tasks[i].lane_obstacles_info;
    if (!lane_obstacles_info.empty()) {
      for (const auto& obs_pair : lane_obstacles_info) {
        const auto& obs = obs_pair.second;
        if (has_checked_set.contains(obs.id)) continue;
        has_checked_set.emplace(obs.id);
        auto obs_s_min = obs.frenet_box.s_min;
        all_lanes_obstacles_info[obs_s_min].id = obs.id;
        all_lanes_obstacles_info[obs_s_min].frenet_box = obs.frenet_box;
        all_lanes_obstacles_info[obs_s_min].obs_v = obs.obs_v;
      }
    }
  }
  double target_traffic_velocity = 0.0;
  if (all_lanes_obstacles_info.size() > 1) {
    const auto it_first = all_lanes_obstacles_info.begin();
    const auto it_second = std::next(it_first);
    const double ego_v = plan_start_point.v();
    auto min_obs_v = it_first->second.obs_v > it_second->second.obs_v
                         ? it_second->second.obs_v
                         : it_first->second.obs_v;
    if (min_obs_v > ego_v) {
      output->target_traffic_velocity = min_obs_v;
      target_traffic_velocity = min_obs_v;
    };
    Log2DDS::LogDataV0(
        "target_traffic_velocity",
        absl::StrCat("target_traffic_obs: id: ", it_first->second.id,
                     " s_min: ", it_first->second.frenet_box.s_min, " v: ",
                     it_first->second.obs_v, " id: ", it_second->second.id,
                     " s_min: ", it_second->second.frenet_box.s_min,
                     " v: ", it_second->second.obs_v));
  }

  auto route_target_info = FindRouteTargetInfo(
      psmm, updated_preferred_lane_path, ego_box, multi_tasks, st_traj_mgr_list,
      is_open_gap, ego_pos, planner_state.is_going_force_route_change_left,
      planner_state.begin_route_change_left);
  Log2DDS::LogDataV2(
      "gap_debug",
      absl::StrCat("is_open_gap: ", is_open_gap, ", begin_route_change_left: ",
                   planner_state.begin_route_change_left.has_value()
                       ? planner_state.begin_route_change_left.value()
                       : -1,
                   ", force: ",
                   planner_state.is_going_force_route_change_left.has_value()
                       ? planner_state.is_going_force_route_change_left.value()
                       : -1));
  // pause calculate gap, set is_open_gap
  if (FLAGS_planner_enable_pause_gap_cal &&
      planner_state.lane_change_state.stage() == LCS_PAUSE &&
      planner_state.decider_state.traffic_gap().is_gap_cal()) {
    is_open_gap = true;
    Log2DDS::LogDataV2("gap_debug",
                       absl::StrCat("pause gap: is_open_gap = ", is_open_gap));
  }
  // push intention
  PushDirection push_dir = ExtractSchedulerPushIntention(multi_tasks);

  // PushDirection push_dir = PushIntention(
  //     plan_start_point,
  //     input.object_manager ? &input.object_manager.value() : nullptr,
  //     multi_tasks, planner_state, if_miss_navi_secnario);
  output->lc_push_dir = push_dir;
  Log2DDS::LogDataV0("lc_debug",
                     absl::StrCat("last_push: ", planner_state.lc_push_dir));
  Log2DDS::LogDataV0("lc_debug", absl::StrCat("push_dir: ", push_dir));
  if ((push_dir == PushDirection::Push_Congestion_Left ||
       push_dir == PushDirection::Push_Congestion_Right) &&
      plan_start_point.v() < 13.8) {
    lane_change_style = LC_STYLE_RADICAL;
  }
  output->last_lc_style = lane_change_style;
  Log2DDS::LogDataV2(
      "lane_change_style",
      absl::StrCat("output for next lc style: ", lane_change_style));
  if (planner_state.lane_change_state.stage() != LCS_NONE) {
    lane_change_style = planner_state.last_lc_style;
  }
  Log2DDS::LogDataV2(
      "lane_change_style",
      absl::StrCat("set lane_change_style: ", lane_change_style));
  Log2DDS::LogDataV2("lc_debug",
                     absl::StrCat("lane_change_style: ", lane_change_style));
  Log2DDS::LogDataV0("RunSchedulerAll", multitask_timer.TimeUs() / 1e3);
  // --------------------------------------------------------------
  // -------- Run CaptainNet Inference If Configured---------------
  // --------------------------------------------------------------
  // Run CaptainNet for multiple scheduler outputs if configured.
  // std::vector<ml::captain_net::CaptainNetOutput> captain_net_results(
  //     multi_tasks.size());
  multitask_timer.Reset("RunEstPlannerAll");
  std::vector<PlannerStatus> status_list(multi_tasks.size());
  std::vector<EstPlannerOutput> results(multi_tasks.size());
  std::vector<EstPlannerDebug> est_debugs(multi_tasks.size());

  // set lc style decider results to default
  LaneChangeStyleDeciderResultProto default_lc_styel_decider_result;
  SetDefaultResponseStyle(lane_change_style, &default_lc_styel_decider_result);
  for (auto& style_res : output->lc_style_decider_results) {
    style_res.second = default_lc_styel_decider_result;
  }
  // set safety evaluation results to default
  for (auto& eval_res : output->task_safety_evaluation_results) {
    eval_res.second = TaskSafetyEvaluationProto();
  }
  // set frames results for the scene of cones riding line to default
  for (auto& frames_res : output->scene_cones_riding_line_frames_results) {
    frames_res.second = 0;
  }

  // get previous styles and evaluations according to the task infos
  std::vector<std::tuple<st::LaneChangeStage, bool, bool>> task_key_list(
      multi_tasks.size());
  std::vector<LaneChangeStyleDeciderResultProto>
      pre_lc_style_decider_result_list(multi_tasks.size());
  std::vector<TaskSafetyEvaluationProto> pre_task_safety_evaluation_result_list(
      multi_tasks.size());
  std::vector<int> pre_scene_cones_riding_line_frames_result_list(
      multi_tasks.size());
  for (int i = 0; i < multi_tasks.size(); ++i) {
    const auto task_key = std::make_tuple(
        multi_tasks[i].lane_change_state.stage(), multi_tasks[i].borrow_lane,
        multi_tasks[i].lane_change_state.lc_left());
    task_key_list[i] = task_key;

    const auto lc_style_decider_result_iter =
        planner_state.lc_style_decider_results.find(task_key);
    if (lc_style_decider_result_iter !=
        planner_state.lc_style_decider_results.end()) {
      pre_lc_style_decider_result_list[i] =
          lc_style_decider_result_iter->second;
    } else {
      SetDefaultResponseStyle(lane_change_style,
                              &pre_lc_style_decider_result_list[i]);
    }

    const auto task_safety_evaluation_result_iter =
        planner_state.task_safety_evaluation_results.find(task_key);
    if (task_safety_evaluation_result_iter !=
        planner_state.task_safety_evaluation_results.end()) {
      pre_task_safety_evaluation_result_list[i] =
          task_safety_evaluation_result_iter->second;
    } else {
      pre_task_safety_evaluation_result_list[i] = TaskSafetyEvaluationProto();
    }

    const auto scene_cones_riding_line_frames_result_iter =
        planner_state.scene_cones_riding_line_frames_results.find(task_key);
    if (scene_cones_riding_line_frames_result_iter !=
        planner_state.scene_cones_riding_line_frames_results.end()) {
      pre_scene_cones_riding_line_frames_result_list[i] =
          scene_cones_riding_line_frames_result_iter->second;
    } else {
      pre_scene_cones_riding_line_frames_result_list[i] = 0;
    }
  }

  PlannerStatus fallback_status(PlannerStatusProto::INPUT_INCORRECT,
                                "fallback init failed");
  FallbackPlannerOutput fallback_result;
  EstPlannerDebug fallback_debug;

  DLOG(INFO) << "Scheduler output task size without fallback planner: "
             << multi_tasks.size();

  if ((FLAGS_planner_enable_captain_net ||
       FLAGS_planner_enable_captain_net_onnx_trt) &&
      FLAGS_planner_use_ml_trajectory_end_to_end) {
    // -----------------------------------------------------------------
    // --------------- Use CaptainNet Trajs BYD ------------------------
    // -----------------------------------------------------------------
    // for (int i = 0; i < multi_tasks.size(); ++i) {
    //   constexpr double kRequiredMinPathLength = 20.0;
    //   constexpr double kCurvatureRelaxFactor = 1.05;
    //   const double max_curvature = ComputeRelaxedCenterMaxCurvature(
    //       vehicle_geometry, input.vehicle_params->vehicle_drive_params());
    //   RETURN_PLANNER_STATUS_OR_ASSIGN(
    //       auto path_extension_output,
    //       ExtendPathAndDeleteUnreasonablePart(
    //           captain_net_results[i].traj_points, kRequiredMinPathLength,
    //           kCurvatureRelaxFactor * max_curvature),
    //       PlannerStatusProto::PATH_EXTENSION_FAILED);
    //   results[i].path = DiscretizedPath::CreateResampledPath(
    //       std::move(path_extension_output), kPathSampleInterval);
    //   results[i].traj_points = std::move(captain_net_results[i].traj_points);
    //   results[i].scheduler_output = multi_tasks[i];
    //   *est_debugs[i].speed_finder_debug.mutable_trajectory() = {
    //       results[i].traj_points.begin(), results[i].traj_points.end()};
    //   est_debugs[i].speed_finder_debug.set_trajectory_start_timestamp(
    //       ToUnixDoubleSeconds(input.start_point_info->plan_time));
    //   est_debugs[i].capnet_ref_traj = {
    //       captain_net_results[i].traj_points.begin(),
    //       captain_net_results[i].traj_points.end()};
    // }
  } else {
    // ---------------------------------------
    // ------ Run Est Planner ----------------
    // ---------------------------------------
    TIMELINE("RMTCP.RunEstPlanner");
    ParallelFor(0, multi_tasks.size(), thread_pool, [&](int i) {
      bool consider =
          ShouldConsiderRouteTarget(i, multi_tasks[i], route_target_info);
      status_list[i] = RunEstPlanner(
          EstPlannerInput{
              .semantic_map_manager = &smm,
              .planner_semantic_map_manager = &psmm,
              .plan_id = i,
              .vehicle_params = input.vehicle_params,
              // .parking_brake_release_time =
              //     planner_state.parking_brake_release_time,
              .decider_state = &planner_state.decider_state,
              .initializer_state = &planner_state.initializer_state,
              .trajectory_optimizer_state_proto =
                  planner_state.selected_trajectory_optimizer_state_proto
                          .has_value()
                      ? &planner_state.selected_trajectory_optimizer_state_proto
                             .value()
                      : nullptr,
              .st_planner_object_trajectories =
                  &planner_state.st_planner_object_trajectories,
              .obj_mgr = input.object_manager ? &input.object_manager.value()
                                              : nullptr,
              .start_point_info = input.start_point_info
                                      ? &input.start_point_info.value()
                                      : nullptr,
              .st_path_start_point_info = &path_start_point_info,
              // .tl_info_map = input.tl_info_map,
              .traffic_light_status_map =
                  input.traffic_light_status_map
                      ? &input.traffic_light_status_map.value()
                      : nullptr,
              .smooth_result_map = &planner_state.smooth_result_map,
              .stalled_objects = input.stalled_objects
                                     ? &input.stalled_objects.value()
                                     : nullptr,
              .scene_reasoning = input.scene_reasoning
                                     ? &input.scene_reasoning.value()
                                     : nullptr,
              .prev_target_lane_path_from_start =
                  &prev_target_lane_path_from_start,
              .pre_large_vehicle_avoid_state =
                  &planner_state.pre_large_vehicle_avoid_state,
              .time_aligned_prev_traj =
                  input.time_aligned_prev_traj
                      ? &input.time_aligned_prev_traj.value()
                      : nullptr,
              .lane_change_style = lane_change_style,
              //.enable_pull_over = ext_cmd_status.enable_pull_over,
              //.enable_traffic_light_stopping =
              //    ext_cmd_status.enable_traffic_light_stopping,
              .enable_tl_ok_btn = input.enable_tl_ok_btn,
              .override_passable = input.override_passable,
              // .system_break_stop = input.system_break_stop,
              //.brake_to_stop = ext_cmd_status.brake_to_stop,
              // .dynamic_headway = input.dynamic_headway,
              .route_target_info =
                  consider ? &route_target_info.value() : nullptr,
              .consider_lane_change_gap = input.consider_lane_change_gap,
              .st_traj_mgr = &st_traj_mgr_list[i],
              //.log_av_trajectory = input.log_av_trajectory,
              // .captain_net_output = &captain_net_output,
              // .planner_av_context = input.planner_av_context,
              // .objects_proto = input.objects_proto.get(),
              // .planner_model_pool = input.planner_model_pool,
              .decision_constraint_config =
                  &input.planner_params->decision_constraint_config(),
              .initializer_params = &input.planner_params->initializer_params(),
              .trajectory_optimizer_params =
                  &input.planner_params->trajectory_optimizer_params(),
              .speed_finder_params =
                  &input.planner_params->speed_finder_params(),
              .motion_constraint_params =
                  &input.planner_params->motion_constraint_params(),
              .planner_functions_params =
                  &input.planner_params->planner_functions_params(),
              .vehicle_models_params =
                  &input.planner_params->vehicle_models_params(),
              .speed_finder_lc_radical_params =
                  &input.planner_params->speed_finder_lc_radical_params(),
              .speed_finder_lc_conservative_params =
                  &input.planner_params->speed_finder_lc_conservative_params(),
              .speed_finder_lc_normal_params =
                  &input.planner_params->speed_finder_lc_normal_params(),
              .trajectory_optimizer_lc_radical_params =
                  &input.planner_params
                       ->trajectory_optimizer_lc_radical_params(),
              .trajectory_optimizer_lc_normal_params =
                  &input.planner_params
                       ->trajectory_optimizer_lc_normal_params(),
              .trajectory_optimizer_lc_conservative_params =
                  &input.planner_params
                       ->trajectory_optimizer_lc_conservative_params(),
              .spacetime_planner_object_trajectories_params =
                  &input.planner_params
                       ->spacetime_planner_object_trajectories_params(),
              // .map_func_id = input.map_func_id,
              .behavior = &(input.behavior.value()),
              .miss_navi_scenario = if_miss_navi_secnario,
              .obs_history = &(planner_state.object_history_manager),
              .speed_state = &(planner_state.speed_state),
              .cur_navi_lc_num = cur_navi_lc_num,
              .leading_id = input.leading_id,
              .left_navi_dist = left_navi_dist,
              .left_navi_dist_v2 = left_navi_dist_v2,
              .last_turn_type_v2 = last_turn_type_v2,
              .prev_lane_change_stage = planner_state.lane_change_state.stage(),
              .lc_cmd_state = new_lc_cmd_state,
              .nudge_object_info =
                  planner_state.nudge_object_info.has_value()
                      ? &planner_state.nudge_object_info.value()
                      : nullptr,
              .cur_dist_to_junction = cur_dist_to_junction,
              .lc_lead_obj_ids = planner_state.lc_lead_obj_ids,
              .is_open_gap = is_open_gap,
              .spdlimit_curvature_gain_prev =
                  planner_state.spdlimit_curvature_gain_prev,
              .saved_offset = &planner_state.saved_offset,
              .pre_lc_style_decider_result =
                  pre_lc_style_decider_result_list[i],
              .pre_task_safety_evaluation_result =
                  pre_task_safety_evaluation_result_list[i],
              .pre_scene_cones_riding_line_frames_result =
                  pre_scene_cones_riding_line_frames_result_list[i],
              .speed_finder_state = &planner_state.speed_finder_state,
              .last_gaming_result = &(planner_state.last_gaming_result),
              .pre_truncated_back_traj_horizon =
                  planner_state.truncated_back_traj_horizon,
              .cruising_speed_limit = input.cruising_speed_limit,
              .eie_choice_type = eie_choice_type,
              .eie_braking_down_flag = eie_braking_down_flag,
              .cur_dist_to_prev_junction = cur_dist_to_prev_junction,
              .dist_to_tunnel_entrance = dist_to_tunnel_entrance,
              .dist_to_tunnel_exitance = dist_to_tunnel_exitance},
          std::move(multi_tasks[i]), &results[i], &est_debugs[i],
          FLAGS_planner_allow_multi_threads_in_est ? thread_pool : nullptr);
    });
  }
  for (int i = 0; i < multi_tasks.size(); ++i) {
    const auto task_key = task_key_list[i];

    auto lc_style_decider_result_iter =
        output->lc_style_decider_results.find(task_key);
    if (lc_style_decider_result_iter !=
        output->lc_style_decider_results.end()) {
      lc_style_decider_result_iter->second = results[i].lc_style_decider_result;
    } else {
      output->lc_style_decider_results.emplace(
          std::make_pair(task_key, results[i].lc_style_decider_result));
    }

    auto task_safety_evaluation_result_iter =
        output->task_safety_evaluation_results.find(task_key);
    if (task_safety_evaluation_result_iter !=
        output->task_safety_evaluation_results.end()) {
      task_safety_evaluation_result_iter->second =
          results[i].task_safety_evaluation_result;
    } else {
      output->task_safety_evaluation_results.emplace(
          std::make_pair(task_key, results[i].task_safety_evaluation_result));
    }

    auto scene_cones_riding_line_frames_result_iter =
        output->scene_cones_riding_line_frames_results.find(task_key);
    if (scene_cones_riding_line_frames_result_iter !=
        output->scene_cones_riding_line_frames_results.end()) {
      scene_cones_riding_line_frames_result_iter->second =
          results[i].scene_cones_riding_line_frames_result;
    } else {
      output->scene_cones_riding_line_frames_results.emplace(std::make_pair(
          task_key, results[i].scene_cones_riding_line_frames_result));
    }
  }
  bool is_all_tasks_failed = true;
  for (int i = 0; i < results.size(); ++i) {
    if (status_list[i].ok()) is_all_tasks_failed = false;
    LOG_ERROR << "Task num: " << i << "; status: " << status_list[i].ok()
              << "; message: " << status_list[i].message() << "; cost: "
              << est_debugs[i].optimizer_debug_proto.ddp().final_costs().cost();
  }
  // run fallback
  if (is_all_tasks_failed) {
    TIMELINE("RMTCP.RunFallbackPlanner");
    FallbackPlannerInput fallback_input{
        .psmm = &psmm,
        .start_point_info =
            input.start_point_info ? &input.start_point_info.value() : nullptr,
        .time_aligned_prev_trajectory =
            input.time_aligned_prev_traj ? &input.time_aligned_prev_traj.value()
                                         : nullptr,
        .prev_target_lane_path_from_start = &prev_target_lane_path_from_start,
        .prev_length_along_route = planner_state.prev_length_along_route,
        .prev_max_reach_length = planner_state.prev_max_reach_length,
        .station_anchor = &planner_state.station_anchor,
        .prev_smooth_state = planner_state.prev_smooth_state,
        .prev_lane_path_before_lc = &prev_lane_path_before_lc_from_start,
        .obj_mgr =
            input.object_manager ? &input.object_manager.value() : nullptr,
        .st_traj_mgr = input.st_traj_mgr ? &input.st_traj_mgr.value() : nullptr,
        .stalled_objects =
            input.stalled_objects ? &input.stalled_objects.value() : nullptr,
        .scene_reasoning =
            input.scene_reasoning ? &input.scene_reasoning.value() : nullptr,
        .prev_lc_state = &planner_state.lane_change_state,
        // .traffic_light_states = input.traffic_light_states,
        .pre_decider_state = &planner_state.decider_state,
        // .tl_info_map = input.tl_info_map,
        .traffic_light_status_map =
            input.traffic_light_status_map
                ? &input.traffic_light_status_map.value()
                : nullptr,
        .smooth_result_map = &planner_state.smooth_result_map,
        .cruising_speed_limit = cruising_speed_limit,
        .behavior = &(input.behavior.value()),
        .speed_state = &(planner_state.speed_state),
        .plan_id = (int)results.size() + 1,
        .ego_box = ego_box,
        .speed_finder_state = &planner_state.speed_finder_state};
    LOG_ERROR << "fallback input trajectory size: "
              << fallback_input.time_aligned_prev_trajectory->size();
    fallback_status = RunFallbackPlanner(
        fallback_input, *input.vehicle_params,
        input.planner_params->motion_constraint_params(),
        input.planner_params->decision_constraint_config(),
        input.planner_params->fallback_planner_params(), &fallback_result,
        &fallback_debug, /*thread_pool=*/nullptr);
  }
  Log2DDS::LogDataV1("task_info_fallback",
                     "Fallback planner status: " + fallback_status.ToString());
  DLOG(INFO) << "Fallback planner status: " << fallback_status.ok()
             << "; message: " << fallback_status.message() << "; cost: "
             << fallback_debug.optimizer_debug_proto.ddp().final_costs().cost();

  // timer.Mark("multi_trajectories");

  // Collect speed-considered object ids by all est planners and fallback
  // planner.
  std::set<std::string> speed_considered_object_ids;
  if (FLAGS_planner_export_all_prediction_to_speed_considered) {
    for (const auto& planner_object : input.object_manager->planner_objects()) {
      speed_considered_object_ids.insert(planner_object.id());
    }
  } else {
    for (int i = 0; i < results.size(); ++i) {
      for (const auto& part_st_traj : results[i].considered_st_objects) {
        speed_considered_object_ids.insert(
            std::string(part_st_traj.st_traj().object_id()));
      }
    }
    for (const auto& part_st_traj : fallback_result.considered_st_objects) {
      speed_considered_object_ids.insert(
          std::string(part_st_traj.st_traj().object_id()));
    }
  }

  // Fill the fallback result to be selected together if:
  // - fallback planner succeeds, and
  // - no teleop-required lane change is activated, and
  //   - it matches the start id of some est planner, or
  //   - no est planner succeeds.
  if (fallback_status.ok() && updated_preferred_lane_path.IsEmpty()) {
    const bool est_any_success =
        std::any_of(status_list.begin(), status_list.end(),
                    [](const PlannerStatus& status) { return status.ok(); });
    if (!est_any_success) {
      Log2DDS::LogDataV1("task_info_fallback", "AppendFB: est all fail");
      AppendFallbackToResultList(fallback_status, std::move(fallback_result),
                                 std::move(fallback_debug), &st_traj_mgr_list,
                                 &status_list, &results, &est_debugs);
    } else {
      const auto fallback_start_id =
          fallback_result.scheduler_output.drive_passage.lane_path()
              .front()
              .lane_id();
      Log2DDS::LogDataV1(
          "task_info_fallback",
          absl::StrCat("AppendFB: fb start_lane:", fallback_start_id));
      for (const auto& result : results) {
        if (!result.scheduler_output.drive_passage.empty() &&
            result.scheduler_output.drive_passage.lane_path()
                    .front()
                    .lane_id() == fallback_start_id) {
          AppendFallbackToResultList(
              fallback_status, std::move(fallback_result),
              std::move(fallback_debug), &st_traj_mgr_list, &status_list,
              &results, &est_debugs);
          Log2DDS::LogDataV1("task_info_fallback", "AppendFB: est success ok");
          break;
        }
      }
    }
  }

  std::vector<std::string> task_infos;
  task_infos.reserve(3);
  for (size_t i = 0; i < results.size(); i++) {
    const auto& output_lc = results[i].scheduler_output.lane_change_state;
    const std::string& prefix = Log2DDS::TaskPrefix(i);
    task_infos.emplace_back(absl::StrFormat(
        "%s:%s,is_fallback:%d,borrow:%d,lc_state:%s,push_state:%s,return_scene:"
        "%d,lc_left:%d,"
        "entered_target_lane:%d,force_merge:%d",
        prefix, status_list[i].ToString(),
        results[i].scheduler_output.is_fallback,
        results[i].scheduler_output.borrow_lane,
        LaneChangeStage_Name(output_lc.stage()),
        PushState_Name(output_lc.push_state()), results[i].is_init_return_scene,
        output_lc.lc_left(), output_lc.entered_target_lane(),
        output_lc.force_merge()));
  }
  Log2DDS::LogDataV1("task_infos", std::move(task_infos));

  for (size_t i = 0; i < results.size(); i++) {
    results[i].scheduler_output.sl_boundary.DumpToDebugFrame(i);
  }
  Log2DDS::LogDataV0("RunEstPlannerAll", multitask_timer.TimeUs() / 1e3);
  // ---------------------------------------
  // ------ Trajectory Selection ----------------
  // ---------------------------------------
  multitask_timer.Reset("RunSelectorAll");
  // 0. Pre-process status list to filter out est planner results that should
  // not be considered by selector.
  TIMELINE("RunMultiTasksCruisePlanner.TrajectorySelection");
  if (!planner_state.preferred_lane_path.IsEmpty() ||
      !updated_preferred_lane_path.IsEmpty()) {
    output->plc_result = PlcInternalResult();

    if (!updated_preferred_lane_path.IsEmpty()) {
      PreFilterByPreferred(new_alc_state, updated_preferred_lane_path, results,
                           vehicle_geometry, &output->plc_result.value(),
                           &status_list, output->lc_unable_reason);
    }
  }
  PreFilterRedundant(results, &status_list);

  // 1. Run trajectory selector.
  SelectorParamsProto tuned_selector_params;
  if (FLAGS_planner_use_tuned_selector_params) {
    tuned_selector_params =
        LoadSelectorParamsFromFile(FLAGS_planner_selector_params_file_address);
  }
  // Get noa need lane change confirmation from hmi.
  const bool noa_need_lane_change_confirmation = false;
  // ext_cmd_status.noa_need_lane_change_confirmation.has_value()
  //     ? *ext_cmd_status.noa_need_lane_change_confirmation
  //     : false;
  // set driving style gear
  int driving_style_gear = 1;
  if (input.behavior->dynamic_headway() > 1.4) {
    driving_style_gear = 1;
  } else if (input.behavior->dynamic_headway() > 0.9) {
    driving_style_gear = 2;
  } else {
    driving_style_gear = 3;
  }
  // default is LC_STYLE_RADICAL, unless HMI set the value to CONSERVATE
  LaneChangeStyle lc_style = LaneChangeStyle::LC_STYLE_RADICAL;
  if (input.behavior->lane_change_style() ==
      Behavior_LaneChangeStyle::
          Behavior_LaneChangeStyle_LANE_CHANGE_STYLE_CONSERVATE) {
    lc_style = LaneChangeStyle::LC_STYLE_CONSERVATIVE;
  }

  SelectorFlags selector_flags{
      .planner_enable_selector_scoring_net =
          FLAGS_planner_enable_selector_scoring_net,
      .planner_allow_lc_time_after_activate_selector =
          FLAGS_planner_allow_lc_time_after_activate_selector,
      .planner_allow_lc_time_after_give_up_lc =
          FLAGS_planner_allow_lc_time_after_give_up_lc,
      .planner_allow_opposite_lc_time_after_paddle_lc =
          FLAGS_planner_allow_opposite_lc_time_after_paddle_lc,
      .planner_begin_radical_lane_change_frame =
          FLAGS_planner_begin_radical_lane_change_frame,
      .planner_max_allow_lc_time_before_give_up =
          FLAGS_planner_max_allow_lc_time_before_give_up,
      .planner_miss_navi_length = FLAGS_planner_miss_navi_length,
      .planner_dumping_selector_features =
          FLAGS_planner_dumping_selector_features,
      .planner_begin_lane_change_frame = FLAGS_planner_begin_lane_change_frame,
      .planner_begin_signal_frame = FLAGS_planner_begin_signal_frame,
      .planner_begin_signal_frame_city_noa =
          FLAGS_planner_begin_signal_frame_city_noa,
      .planner_enable_lane_change_in_intersection =
          FLAGS_planner_enable_lane_change_in_intersection,
      .planner_enable_turn_light_when_open_gap =
          FLAGS_planner_enable_turn_light_when_open_gap,
      .planner_enable_cross_solid_boundary =
          FLAGS_planner_enable_cross_solid_boundary,
      .planner_lane_change_style = lc_style,
      .planner_need_to_lane_change_confirmation =
          noa_need_lane_change_confirmation,
      .planner_is_bus_model =
          input.planner_params->vehicle_models_params().is_vehicle_bus_model(),
      .planner_is_l4_mode = false,
      .planner_begin_lane_change_frame_progress =
          FLAGS_planner_begin_lane_change_frame_progress,
      .planner_begin_lane_change_frame_progress_city_noa =
          FLAGS_planner_begin_lane_change_frame_progress_city_noa,
      .planner_begin_change_best_lk_trajectory_frame_city_noa =
          FLAGS_planner_begin_change_best_lk_trajectory_frame_city_noa,
      .planner_begin_change_best_lk_trajectory_frame_highway_noa =
          FLAGS_planner_begin_change_best_lk_trajectory_frame_highway_noa};
  SelectorInput selector_input{
      .psmm = &psmm,
      .prev_lane_path_from_current = &prev_target_lane_path_from_start,
      .pre_lc_stage = planner_state.lane_change_state.stage(),
      .prev_traj = input.time_aligned_prev_traj
                       ? &input.time_aligned_prev_traj.value()
                       : nullptr,
      .motion_constraints = &input.planner_params->motion_constraint_params(),
      .vehicle_geom = &vehicle_geometry,
      .plan_start_point = &plan_start_point,
      .stalled_objects =
          input.stalled_objects ? &input.stalled_objects.value() : nullptr,
      // .avoid_lanes = &input.rm_output->avoid_lanes,
      .plan_time = input.start_point_info->plan_time,
      .alc_confirmation = std::
          nullopt,  // planner_state->async_planner_state.pending_alc_confirmation
      // .prediction_debug = input.prediction_debug,
      // .context_feature = input.context_feature.get(),
      // .planner_model_pool = input.planner_model_pool,
      .selector_state = &planner_state.selector_state,
      .selector_flags = &selector_flags,
      .config = FLAGS_planner_use_tuned_selector_params
                    ? &tuned_selector_params
                    : &input.planner_params->selector_params(),
      .planner_config = input.planner_params,
      .cruising_speed_limit = cruising_speed_limit,
      .target_lane_speed_limit = input.target_lane_speed_limit,
      .driving_style_gear = driving_style_gear,
      .is_open_gap = is_open_gap,
      .map_event = input.map_event,
      .begin_route_change_left = planner_state.begin_route_change_left,
      .is_going_force_route_change_left =
          planner_state.is_going_force_route_change_left,
      .navi_start_lanes = navi_start_lanes,
      .icc_lc_enable = icc_lc_enable};
  DeviateNaviInput deviate_navi_input{
      .psmm = &psmm,
      .func_id = function_id,
      .pre_lc_stage = planner_state.lane_change_state.stage(),
      .vehicle_geom = &vehicle_geometry,
      .plan_start_point = &plan_start_point,
      .pre_target_laneseq = pre_target_lane_seq,
      .pre_target_laneseq_before_lc = pre_lane_seq_before_lc};

  for (int idx = 0; idx < results.size(); ++idx) {
    if (results[idx].scheduler_output.is_expert) continue;
    if (results[idx].scheduler_output.drive_passage.lane_seq_info() !=
        nullptr) {
      double dist_to_junction =
          results[idx]
              .scheduler_output.drive_passage.lane_seq_info()
              ->dist_to_junction;
      for (int idx = 0; idx < results.size(); ++idx) {
        Log2DDS::LogDataV2(
            "selector_data",
            absl::StrCat(
                "idx:", idx,
                FormatNumericString(" dist_to_junction: ", dist_to_junction)));
      }
    }
    const auto& drive_passage = results[idx].scheduler_output.drive_passage;

    if (drive_passage.lane_seq_info()) {
      const auto& lane_seq = drive_passage.lane_seq_info()->lane_seq;
      if (lane_seq != nullptr) {
        Log2DDS::LogDataV2("selector_data", "input lane seq is not nullptr");
      } else {
        Log2DDS::LogDataV2("selector_data", "input lane seq is nullptr");
      }
    }
  }
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      output->selector_output,
      SelectTrajectoryV2(selector_input, status_list, st_traj_mgr_list, results,
                         deviate_navi_input, &output->selector_debug,
                         &output->selector_state),
      PlannerStatusProto::SELECTOR_FAILED);
  auto selected_idx_or = output->selector_output->selected_idx;

  // LOG_ERROR << "select path index: " << selected_idx_or;
  // publish debug info
  Log2DDS::LogDataV2("selector_debug_force_lk",
                     output->selector_output->force_lane_keep_info);
  std::vector<std::string> debug_string;
  for (const auto& str : output->selector_debug.debugstring()) {
    // LOG_ERROR << "select: " << str;
    debug_string.push_back(str);
  }
  Log2DDS::LogDataV2("selector_debug_string", debug_string);
  if (!icc_lc_enable && function_id == Behavior_FunctionId_LKA) {
    for (int idx = 0; idx < status_list.size(); ++idx) {
      if (!status_list[idx].ok()) continue;
      selected_idx_or = idx;
      break;
    }
    output->selector_state.lane_change_reason = LaneChangeReason::NO_CHANGE;
  }

  if (output->selector_output->may_miss_navi) {
    output->lc_notice = LaneChangeNotice::Notice_Miss_Navi;
  }

  DLOG(INFO) << "Use Selector Score Net (1: true; 0: false):"
             << selector_flags.planner_enable_selector_scoring_net;
  DLOG(INFO) << "Dump Selector Features (1: true; 0: false):"
             << selector_flags.planner_dumping_selector_features;
  DLOG(INFO) << "Selector Begin Lane Change Frame:"
             << selector_flags.planner_begin_lane_change_frame;
  DLOG(INFO) << "Selector Begin Signal Frame:"
             << selector_flags.planner_begin_signal_frame;
  DLOG(INFO) << "Selector Need LC Confirmation:"
             << selector_flags.planner_need_to_lane_change_confirmation;

  Log2DDS::LogDataV0("selected_idx", selected_idx_or);
  std::swap(results[0], results[selected_idx_or]);
  std::swap(status_list[0], status_list[selected_idx_or]);
  std::swap(est_debugs[0], est_debugs[selected_idx_or]);

  // Stitch path points and convert to global path points.
  const auto& start_point_info = *input.start_point_info;
  // StitchStPathTrajectoryWithPastTrajectory(
  //     *input.previous_trajectory,
  //     start_point_info.start_index_on_prev_traj, results[0].st_path_points,
  //     &output->st_path_points_global_including_past);

  const auto& scheduler_output = results[0].scheduler_output;
  Log2DDS::LogDataV0("RunSelectorAll", multitask_timer.TimeUs() / 1e3);
  // -----------------------------------------------------------------
  // ---------------------- Update PLC result ------------------------
  // -----------------------------------------------------------------
  multitask_timer.Reset("UpdatePlcResult");
  if (output->plc_result.has_value()) {
    const auto plc_status = output->plc_result->status;
    if (plc_status == PlcInternalResult::kBranchNotFound) {
      updated_preferred_lane_path = mapping::LanePath();
      new_alc_state = ALC_STANDBY_ENABLE;
      new_lc_cmd_state = DriverAction::LC_CMD_NONE;
    } else if (plc_status == PlcInternalResult::kSolidBoundary ||
               plc_status == PlcInternalResult::kUnsafeObject ||
               plc_status == PlcInternalResult::kBranchFailedInternal) {
      // ReportPlcEventSignal(init_alc_state,
      //                      /*new_state=*/ALC_PREPARE, new_lc_cmd_state);
      new_alc_state = ALC_PREPARE;
    } else {
      RETURN_PLANNER_STATUS_OR_ASSIGN(
          new_alc_state,
          UpdateAlcState(
              new_alc_state == ALC_PREPARE ? ALC_ONGOING : new_alc_state,
              ego_pos, ego_pos, plan_start_point.path_point().theta(),
              scheduler_output.drive_passage, kMaxLaneKeepLateralOffset),
          PlannerStatusProto::SCHEDULER_UNAVAILABLE);
      if (new_alc_state == ALC_STANDBY_ENABLE) {
        updated_preferred_lane_path = mapping::LanePath();
        new_lc_cmd_state = DriverAction::LC_CMD_NONE;
      }
      // ReportPlcEventSignal(init_alc_state, new_alc_state,
      // new_lc_cmd_state);
    }
    // Check if having been prepared for too long.
    if (new_alc_state == ALC_PREPARE &&
        planner_state.plc_prepare_start_time.has_value() &&
        absl::ToDoubleSeconds(input.start_point_info->plan_time -
                              *planner_state.plc_prepare_start_time) >=
            FLAGS_planner_paddle_lane_change_max_prepare_time) {
      updated_preferred_lane_path = mapping::LanePath();
      new_alc_state = ALC_STANDBY_ENABLE;
      new_lc_cmd_state = DriverAction::LC_CMD_NONE;
    }

    // manual lc reason
    if (!updated_preferred_lane_path.IsEmpty() &&
        new_lc_cmd_state != DriverAction::LC_CMD_NONE) {
      output->selector_state.lane_change_reason =
          LaneChangeReason::MANUAL_CHANGE;
    }
    output->alc_state = new_alc_state;
    output->plc_result->preferred_lane_path =
        std::move(updated_preferred_lane_path);
    output->plc_result->lane_change_command = new_lc_cmd_state;
    Log2DDS::LogDataV2("update_plc_result",
                       absl::StrCat("alc_state: ", output->alc_state));
    Log2DDS::LogDataV2(
        "update_plc_result",
        absl::StrCat("lc_cmd_run: ", output->plc_result->lane_change_command));
  }
  // if (input.function_id && *input.function_id == Behavior_FunctionId_LKA &&
  if (!icc_lc_enable &&
      input.behavior->function_id() == Behavior_FunctionId_LKA &&
      scene_target_lane_seq && scene_target_lane_seq->IsValid()) {
    output->selector_state.lane_change_reason = new_lane_change_reason;
  }
  // Hmi
  output->nudge_object_info = results[0].nudge_object_info;
  output->tl_stop_interface = results[0].tl_stop_interface;
  output->tl_ind_info = results[0].tl_ind_info;
  // speed decision history data
  output->speed_state = results[0].speed_state;
  // speed plan history data
  output->spdlimit_curvature_gain = results[0].spdlimit_curvature_gain;

  // computer ego pose and planning_start_point frenet
  if (input.start_point_info) {
    auto start_point_sl_pt =
        scheduler_output.drive_passage.QueryUnboundedFrenetCoordinateAt(
            Vec2dFromApolloTrajectoryPointProto(
                input.start_point_info->start_point));
    if (start_point_sl_pt.ok())
      output->start_point_frenet = std::move(start_point_sl_pt).value();
  }

  if (input.ego_pos) {
    auto ego_pose_sl_pt =
        scheduler_output.drive_passage.QueryUnboundedFrenetCoordinateAt(
            *input.ego_pos);
    if (ego_pose_sl_pt.ok())
      output->ego_pose_frenet = std::move(ego_pose_sl_pt).value();
  }

  if (!icc_lc_enable && is_lka &&
      output->selector_state.lane_change_reason !=
          LaneChangeReason::MANUAL_CHANGE &&
      output->selector_state.lane_change_reason !=
          LaneChangeReason::LCC_ROUTE_CHANGE &&
      output->selector_state.lane_change_reason !=
          LaneChangeReason::AVOID_ROADWORK_CHANGE &&
      output->selector_state.lane_change_reason !=
          LaneChangeReason::AVOID_VEHICLE_CHANGE) {
    output->selector_state.lane_change_reason = LaneChangeReason::NO_CHANGE;
  }

  // Ego lane loss check
  // bool ego_lane_loss_fail = false;
  // if (scheduler_output.lane_change_state.stage() == LaneChangeStage::LCS_NONE
  // &&
  //     planner_state.lane_change_state.stage() == LaneChangeStage::LCS_NONE &&
  //     output->ego_pose_frenet && planner_state.prev_ego_pose_frenet) {
  //   double delta_ego_l =
  //       output->ego_pose_frenet->l - planner_state.prev_ego_pose_frenet->l;
  //   if (std::abs(delta_ego_l) > 2.0 &&
  //       std::abs(output->ego_pose_frenet->l) > 1.0) {
  //     ego_lane_loss_fail = true;
  //   }
  // }
  // if (planner_state.prev_est_planner_status_code != PlannerStatusProto::OK &&
  //     // planner_state.prev_est_planner_status_code ==
  //     //       PlannerStatusProto::EGO_LANE_LOSS_FAIL &&
  //     output->ego_pose_frenet && std::abs(output->ego_pose_frenet->l) > 1.0)
  //     {
  //   ego_lane_loss_fail = true;
  // }
  // if (ego_lane_loss_fail) {
  //   return PlannerStatus(PlannerStatusProto::EGO_LANE_LOSS_FAIL,
  //                        "Ego lane loss!");
  // }

  // Lat safety check
  if (!results.empty() &&
      results[0].scheduler_output.drive_passage.lane_seq_info()) {
    std::vector<double> new_angles;
    const auto& new_tgt_seq =
        results[0].scheduler_output.drive_passage.lane_seq_info()->lane_seq;
    Log2DDS::LogDataV0("Lat_unsafety", "new_angles:");
    GetInterestPointsOfLatSafetyCheck(ego_pos, plan_start_point.v(),
                                      new_tgt_seq, &new_angles);
    if (last_angles.size() > 1 && last_angles.size() == new_angles.size()) {
      Log2DDS::LogDataV0("Lat_unsafety", "compare:");
      for (int i = 0; i < last_angles.size(); i++) {
        if (std::fabs(ad_byd::planning::math::NormalizeAngle(
                last_angles[i] - new_angles[i])) > M_PI / 2.0) {
          return PlannerStatus(PlannerStatusProto::TARGET_LANE_JUMPED_FAIL,
                               "Lat unsafety!");
        }
      }
    }
  }

  output->est_status_list = std::move(status_list);
  output->est_planner_output_list = std::move(results);
  output->est_planner_debug_list = std::move(est_debugs);
  /*
    double min_dis_to_junction_or_left_navi =
        std::min(left_navi_dist_v2, cur_dist_to_junction);
    if (min_dis_to_junction_or_left_navi < 1.0) {
      output->has_passed_this_junction = true;
    }
    Log2DDS::LogDataV2("lc_notice",
                       absl::StrCat("cur_navi_lc_num: ", cur_navi_lc_num));
    int cur_navi_lc_num_clamped =
        ad_byd::planning::math::Clamp(cur_navi_lc_num, 0, 2);
    int dist_threshold = 0;
    if (cur_navi_lc_num_clamped == 1) {
      dist_threshold = 70;
    } else if (cur_navi_lc_num_clamped == 2) {
      dist_threshold = 100;
    }
    Log2DDS::LogDataV2("lc_notice",
                       absl::StrCat("min_dis_to_junction_or_left_navi: ",
                                    min_dis_to_junction_or_left_navi));
    Log2DDS::LogDataV2("lc_notice", absl::StrCat("cur_nearest_turn_type: ",
                                                 cur_nearest_turn_type));
    if (cur_navi_lc_num_clamped > 0 &&
        dist_threshold > min_dis_to_junction_or_left_navi &&
        min_dis_to_junction_or_left_navi > 10.0 &&
        cur_nearest_turn_type !=
            ad_byd::planning::V2TurnInfo::V2DetailTurnType::CONTINUE) {
      output->if_lc_to_congestion = true;
      if (cur_nearest_turn_type ==
              ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT ||
          cur_nearest_turn_type ==
              ad_byd::planning::V2TurnInfo::V2DetailTurnType::SLIGHT_LEFT ||
          cur_nearest_turn_type ==
              ad_byd::planning::V2TurnInfo::V2DetailTurnType::UTURN) {
        output->lc_notice = Notice_LeTurn_Hard;
      } else if (cur_nearest_turn_type ==
                     ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT
    || cur_nearest_turn_type == ad_byd::planning::V2TurnInfo::
                                              V2DetailTurnType::SLIGHT_RIGHT ||
                 cur_nearest_turn_type == ad_byd::planning::V2TurnInfo::
                                              V2DetailTurnType::TURN_RIGHT_ONLY)
    { output->lc_notice = Notice_RiTurn_Hard; } else { output->lc_notice =
    Notice_General_Hard;
      }
    }
  */
  Log2DDS::LogDataV0("UpdatePlcResult", multitask_timer.TimeUs() / 1e3);
  return OkPlannerStatus();
}  // NOLINT

// No valid output but still contains some debug information.
bool IsNonEmptyPlannerResult(const PlannerStatus& status) {
  switch (status.status_code()) {
    case PlannerStatusProto::OK:
    case PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE:
    case PlannerStatusProto::INITIALIZER_FAILED:
    case PlannerStatusProto::OPTIMIZER_FAILED:
    case PlannerStatusProto::PATH_EXTENSION_FAILED:
    case PlannerStatusProto::SPEED_OPTIMIZER_FAILED:
    case PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED:
    case PlannerStatusProto::SELECTOR_FAILED:
    // case PlannerStatusProto::MODEL_PRECONDITION_CHECK_FAILED:
    // case PlannerStatusProto::MODEL_INFERENCE_FAILED:
    // case PlannerStatusProto::MODEL_OUTPUT_POSTPROCESS_FAILED:
    case PlannerStatusProto::BRANCH_RESULT_IGNORED:
    case PlannerStatusProto::LC_SAFETY_CHECK_FAILED:
    case PlannerStatusProto::DEVIATE_NAVI:
      return true;
    // case PlannerStatusProto::ROUTE_MSG_UNAVAILABLE:
    // case PlannerStatusProto::GEAR_NOT_READY:
    case PlannerStatusProto::INPUT_INCORRECT:
    // case PlannerStatusProto::ROUTING_FAILED:
    case PlannerStatusProto::TARGET_LANE_JUMPED_FAIL:
    case PlannerStatusProto::EGO_LANE_LOSS_FAIL:
    case PlannerStatusProto::OBJECT_MANAGER_FAILED:
    // case PlannerStatusProto::REFERENCE_PATH_UNAVAILABLE:
    case PlannerStatusProto::SCHEDULER_UNAVAILABLE:
    case PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE:
    case PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED:
    // case PlannerStatusProto::RESET_PREV_TARGET_LANE_PATH_FAILED:
    // case PlannerStatusProto::UPDATE_TARGET_LANE_PATH_FAILED:
    // case PlannerStatusProto::LOCAL_LANE_MAP_BUILDER_FAILED:
    case PlannerStatusProto::PLANNER_ABNORMAL_EXIT:
    // case PlannerStatusProto::TRAFFIC_LIGHT_INFO_COLLECTOR_FAILED:
    // case PlannerStatusProto::PLANNER_STATE_INCOMPLETE:
    // case PlannerStatusProto::EXPERT_TRAJ_UNAVAILABLE:
    // case PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED:
    // case PlannerStatusProto::EXPERT_SPEED_IN_CONTRADICTORY_WITH_SPEED_FINDER:
    // case PlannerStatusProto::PATH_PLANNER_FAILED:
    case PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE:
    // case PlannerStatusProto::OPTIMIZER_DIFF_INTENTION_EXPERT:
    // case PlannerStatusProto::LCC_ODC_CHECK_FAIL:
    // case PlannerStatusProto::BUILD_DRIVING_MAP_FAILED:
    case PlannerStatusProto::ACC_CORRIDOR_FAILED:
    case PlannerStatusProto::ACC_SPEED_FAILED:
    case PlannerStatusProto::ACC_TRAJECTORY_VALIDATION_FAILED:
      // case PlannerStatusProto::PROJECT_TO_ONLINE_MAP_FAILED:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

void ParseSchedulerOutputToPlannerState(const SchedulerOutput& scheduler_output,
                                        PlannerState* planner_state,
                                        const ad_byd::planning::MapPtr& map) {
  const auto& drive_passage = scheduler_output.drive_passage;
  planner_state->prev_target_lane_path =
      drive_passage.extend_lane_path()
          .BeforeLastOccurrenceOfLanePoint(drive_passage.lane_path().back())
          .AfterArclength(
              scheduler_output.av_frenet_box_on_drive_passage.s_min);

  // add lane sequence ptr if null
  if (planner_state->prev_target_lane_path.lane_seq() == nullptr) {
    std::vector<ad_byd::planning::LaneConstPtr> lanes;
    for (const auto& id : planner_state->prev_target_lane_path.lane_ids()) {
      const auto& lane = map->GetLaneById(id);
      if (lane) {
        lanes.emplace_back(lane);
      }
    }
    ad_byd::planning::LaneSequencePtr pre_lane_seq =
        std::make_shared<ad_byd::planning::LaneSequence>(lanes);
    planner_state->prev_target_lane_path.set_lane_seq(std::move(pre_lane_seq));
  }

  planner_state->station_anchor =
      drive_passage.FindNearestStationAtS(0.0).GetLanePoint();
  planner_state->prev_length_along_route = scheduler_output.length_along_route;
  planner_state->prev_max_reach_length = scheduler_output.max_reach_length;
  planner_state->prev_smooth_state = scheduler_output.should_smooth;
  // NOTE: The following fields are faked by multi-task scheduler and
  // should be removed in the future.
  planner_state->lane_change_state = scheduler_output.lane_change_state;
  planner_state->prev_lane_path_before_lc =
      scheduler_output.lane_path_before_lc;
}

void ParseEstPlannerOutputToPlannerState(
    const EstPlannerOutput& est_planner_output, PlannerState* planner_state) {
  planner_state->decider_state = est_planner_output.decider_state;
  planner_state->initializer_state = est_planner_output.initializer_state;
  planner_state->speed_finder_state = est_planner_output.speed_finder_state;
  planner_state->st_planner_object_trajectories =
      est_planner_output.st_planner_object_trajectories;
  // planner_state->prev_traj_end_info = est_planner_output.trajectory_end_info;
  planner_state->selected_trajectory_optimizer_state_proto =
      est_planner_output.trajectory_optimizer_state_proto;
  planner_state->pre_large_vehicle_avoid_state =
      est_planner_output.pre_large_vehicle_avoid_state;
}

bool IsCongestionLane(const ApolloTrajectoryPointProto plan_start_point,
                      const PlannerObjectManager* obj_mgr, const bool& lc_left,
                      const ad_byd::planning::LaneSeqInfoPtr lane_seq_info,
                      const bool& last_lc_push_state) {
  if (!obj_mgr || !lane_seq_info || !(lane_seq_info->lane_seq)) {
    Log2DDS::LogDataV0("lc_debug_congestion", "normal fail");
    return false;
  }
  double ego_s = DBL_MAX, ego_l = DBL_MAX;
  const auto laneseq = lane_seq_info->lane_seq;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  laneseq->GetProjectionDistance(ego_pos, &ego_s, &ego_l);
  const double required_s_max =
      std::fmin(lane_seq_info->dist_to_junction, 50.0);
  const int required_obs_cnt = last_lc_push_state ? 3 : 4;
  const double required_obs_avg_speed = last_lc_push_state ? 60.0 : 55.0;
  Log2DDS::LogDataV0("lc_debug_congestion",
                     absl::StrCat("required_s_max:", required_s_max));
  //
  std::vector<PlannerObject> considered_objects;
  for (const auto& obs : obj_mgr->planner_objects()) {
    if (obs.type() != OT_VEHICLE && obs.type() != OT_MOTORCYCLIST &&
        obs.type() != OT_TRICYCLIST && obs.type() != OT_LARGE_VEHICLE) {
      continue;
      // TODO take bicycle into consideration
    }
    if (obs.is_stationary() &&
        obs.prediction().perception_object().obstacle_light().brake_lights() ==
            ObstacleLightType::LIGHT_OFF) {
      // continue;
    }
    double obs_s = DBL_MAX, obs_l = DBL_MAX;
    double obs_length = obs.bounding_box().length();
    laneseq->GetProjectionDistance(obs.pose().pos(), &obs_s, &obs_l);
    if ((ego_s - 20.0 > obs_s + 0.5 * obs_length) ||
        (ego_s + required_s_max < obs_s - 0.5 * obs_length)) {
      continue;
    }
    if ((lc_left && ego_l > obs_l) || (!lc_left && obs_l > ego_l)) {
      continue;
    }
    double dl =
        std::fabs(obs_l - ego_l) - 0.5 * obs.bounding_box().width() - 1.0;
    if (dl > 3.0) {
      continue;
    }
    considered_objects.emplace_back(obs);
  }
  if (considered_objects.size() <= required_obs_cnt) {
    Log2DDS::LogDataV0("lc_debug_congestion",
                       absl::StrCat("cnt fail:", considered_objects.size()));
    return false;
  }
  //
  double avg_speed = 0.0;
  for (const auto& obs : considered_objects) {
    avg_speed += obs.pose().v();
    Log2DDS::LogDataV0("lc_debug_congestion", obs.id());
  }
  if (avg_speed / considered_objects.size() >
      required_obs_avg_speed * ad_byd::planning::Constants::KPH2MPS) {
    Log2DDS::LogDataV0(
        "lc_debug_congestion",
        absl::StrCat("v fail:", avg_speed / considered_objects.size()));
    return false;
  }
  return true;
}

PushDirection PushIntention(const ApolloTrajectoryPointProto plan_start_point,
                            const PlannerObjectManager* obj_mgr,
                            const std::vector<SchedulerOutput>& tasks,
                            const PlannerState& planner_state,
                            const bool& if_miss_navi_secnario) {
  bool last_lc_push_state =
      planner_state.lc_push_dir != PushDirection::Push_None ? true : false;
  if (last_lc_push_state && plan_start_point.v() > 18.1)
    return PushDirection::Push_None;
  if (!last_lc_push_state && plan_start_point.v() > 16.6)
    return PushDirection::Push_None;
  if (tasks.size() < 2) return PushDirection::Push_None;
  // if (tasks[0].lane_change_state.stage() != LaneChangeStage::LCS_EXECUTING &&
  //     tasks[0].lane_change_state.stage() != LaneChangeStage::LCS_PAUSE) {
  //   return PushDirection::Push_None;
  // }
  // if (tasks[1].lane_change_state.stage() != LaneChangeStage::LCS_NONE ||
  //     tasks[1].borrow_lane) {
  //   return PushDirection::Push_None;
  // }
  // const bool lc_left = tasks[0].lane_change_state.lc_left();
  // const auto lane_seq_info = tasks[0].drive_passage.lane_seq_info();
  if (planner_state.lane_change_state.stage() != LaneChangeStage::LCS_NONE)
    return PushDirection::Push_None;
  if (tasks.size() == 2 && tasks[1].borrow_lane)
    return PushDirection::Push_None;
  if (!planner_state.is_going_force_route_change_left.has_value())
    return PushDirection::Push_None;
  const bool lc_left = planner_state.is_going_force_route_change_left.value();
  ad_byd::planning::LaneSeqInfoPtr lane_seq_info = nullptr;
  for (const auto& task : tasks) {
    if (task.lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING ||
        lc_left == task.lane_change_state.lc_left()) {
      lane_seq_info = task.drive_passage.lane_seq_info();
    }
  }
  if (!lane_seq_info) return PushDirection::Push_None;

  // Congestion scenario check
  const bool is_congestion_scenario = IsCongestionLane(
      plan_start_point, obj_mgr, lc_left, lane_seq_info, last_lc_push_state);
  Log2DDS::LogDataV0("lc_debug", absl::StrCat("push_dir: is congestion:",
                                              is_congestion_scenario));

  // solid line check
  if (!if_miss_navi_secnario) {
    double dist_to_lc_solidline = lc_left
                                      ? lane_seq_info->dist_to_right_solid_line
                                      : lane_seq_info->dist_to_left_solid_line;
    if (dist_to_lc_solidline < 15.0 + plan_start_point.v() * 2.0) {
      Log2DDS::LogDataV0("lc_debug", absl::StrCat("push_dir: none, solid line:",
                                                  dist_to_lc_solidline));
      return PushDirection::Push_None;
    }
  }

  // special lane check

  double dist_to_navi_end = lane_seq_info->dist_to_navi_end;
  double dist_to_bus_lane = lane_seq_info->dist_to_bus_lane;
  if (dist_to_navi_end > 160.0 && lane_seq_info->lc_num <= 1 &&
      dist_to_bus_lane < dist_to_navi_end &&
      dist_to_bus_lane < std::max(5.0 * plan_start_point.v(), 50.0)) {
    Log2DDS::LogDataV0("lc_debug", absl::StrCat("push_dir: none, bus lane:",
                                                dist_to_bus_lane));
    return PushDirection::Push_None;
  }

  // close obs check
  if (obj_mgr && !is_congestion_scenario) {
    const auto laneseq = lane_seq_info->lane_seq;
    const double ttc = last_lc_push_state ? 1.5 : 2.0;
    if (laneseq) {
      const Vec2d ego_pos =
          Vec2dFromApolloTrajectoryPointProto(plan_start_point);
      double ego_s = DBL_MAX, ego_l = DBL_MAX;
      laneseq->GetProjectionDistance(ego_pos, &ego_s, &ego_l);
      for (const auto& obs : obj_mgr->planner_objects()) {
        if (obs.type() != OT_VEHICLE && obs.type() != OT_MOTORCYCLIST &&
            obs.type() != OT_TRICYCLIST && obs.type() != OT_LARGE_VEHICLE &&
            obs.type() != OT_CYCLIST) {
          continue;
        }
        double obs_s = DBL_MAX, obs_l = DBL_MAX;
        laneseq->GetProjectionDistance(obs.pose().pos(), &obs_s, &obs_l);
        double obs_tail_s = obs_s - 0.5 * obs.bounding_box().length();
        double obs_head_s = obs_s + 0.5 * obs.bounding_box().length();
        double ego_tail_s = ego_s - 2.0;
        double ego_head_s = ego_s + 5.0;
        double dv = obs.pose().v() - plan_start_point.v();
        // if (obs_tail_s > ego_head_s) {
        //   continue;
        // }
        const bool is_valid_large_veh =
            obs.type() == OT_LARGE_VEHICLE && obs.bounding_box().length() > 6.0;
        const double update_ttc = is_valid_large_veh ? ttc * 1.5 : ttc;
        // back obs
        if ((obs_head_s < ego_tail_s) &&
            ((ego_tail_s - obs_head_s) > (update_ttc * dv))) {
          Log2DDS::LogDataV0(
              "lc_debug_push",
              absl::StrCat("not danger obs:", obs.id(), " ego tail: ",
                           ego_tail_s, " obs head: ", obs_head_s, " dv: ", dv));
          continue;
        }
        // front obs
        if ((obs_tail_s > ego_head_s) &&
            ((ego_head_s - obs_tail_s) < (update_ttc * dv))) {
          Log2DDS::LogDataV0(
              "lc_debug_push",
              absl::StrCat("not danger obs:", obs.id(),
                           " ego_head_s: ", ego_head_s,
                           " obs_tail_s: ", obs_tail_s, " dv: ", dv));
          continue;
        }
        if ((lc_left && ego_l > obs_l) || (!lc_left && obs_l > ego_l)) {
          continue;
        }
        double dl =
            std::fabs(obs_l - ego_l) - 0.5 * obs.bounding_box().width() - 1.0;
        if (dl > 3.0) {
          continue;
        }
        // debug
        Log2DDS::LogDataV0(
            "lc_debug_push",
            absl::StrCat("danger obs:", obs.id(), " obs head: ", obs_head_s,
                         " obs tail: ", obs_tail_s, " ego head: ", ego_head_s,
                         " ego tail: ", ego_tail_s, " dv: ", dv));

        Log2DDS::LogDataV0("lc_debug",
                           "push_dir: none, danger obs:" + obs.id());
        return PushDirection::Push_None;
      }
    }
  }

  if (is_congestion_scenario) {
    return lc_left ? PushDirection::Push_Congestion_Left
                   : PushDirection::Push_Congestion_Right;
  }

  return lc_left ? PushDirection::Push_Normal_Left
                 : PushDirection::Push_Normal_Right;
}

PushDirection ExtractSchedulerPushIntention(
    const std::vector<SchedulerOutput>& multi_tasks) {
  PushDirection push_dir = PushDirection::Push_None;
  for (const auto& task : multi_tasks) {
    if (task.lane_change_state.stage() == LaneChangeStage::LCS_NONE &&
        task.lane_change_state.push_state() != PushState::NONE_PUSH) {
      switch (task.lane_change_state.push_state()) {
        case PushState::LEFT_PUSH:
          push_dir = ad_byd::planning::PushDirection::Push_Normal_Left;
          break;
        case PushState::RIGHT_PUSH:
          push_dir = ad_byd::planning::PushDirection::Push_Normal_Right;
          break;
        case PushState::CONGESTION_LEFT_PUSH:
          push_dir = ad_byd::planning::PushDirection::Push_Congestion_Left;
          break;
        case PushState::CONGESTION_RIGHT_PUSH:
          push_dir = ad_byd::planning::PushDirection::Push_Congestion_Right;
          break;
        case PushState::NONE_PUSH:
        default:
          push_dir = ad_byd::planning::PushDirection::Push_None;
          break;
      }
      break;
    }
  }
  Log2DDS::LogDataV2("lc_push", static_cast<int>(push_dir));
  return push_dir;
}

void UpdatePreLaneChangeSafetyInfo(
    const PathBoundedEstPlannerOutput* planner_output,
    LaneChangeSafetyInfo* pre_lane_change_safety_info) {
  if (pre_lane_change_safety_info == nullptr) return;

  if (planner_output == nullptr) {
    pre_lane_change_safety_info->set_have_check_lc_left_safe(false);
    pre_lane_change_safety_info->set_have_check_lc_right_safe(false);
    pre_lane_change_safety_info->set_lc_left_safe(false);
    pre_lane_change_safety_info->set_lc_right_safe(false);
    pre_lane_change_safety_info->set_ego_lane_block_by_obs_cnt(0);
    return;
  }
  // step1 : reset
  pre_lane_change_safety_info->set_have_check_lc_left_safe(false);
  pre_lane_change_safety_info->set_have_check_lc_right_safe(false);
  pre_lane_change_safety_info->set_lc_left_safe(false);
  pre_lane_change_safety_info->set_lc_right_safe(false);

  // bool find_lc_left_task_safety_fail{false},
  // find_lc_right_task_safety_fail{false};
  bool find_lc_left_task_safety_ok{false}, find_lc_right_task_safety_ok{false};
  bool find_lane_keep_task{false};

  for (int idx = 0; idx < planner_output->est_status_list.size(); ++idx) {
    if (idx < planner_output->est_planner_output_list.size()) {
      if (idx > 0) {
        pre_lane_change_safety_info->set_have_check_lc_left_safe(true);
        pre_lane_change_safety_info->set_have_check_lc_right_safe(true);
      }

      const auto status_code =
          planner_output->est_status_list[idx].status_code();
      const auto lc_left = planner_output->est_planner_output_list[idx]
                               .scheduler_output.lane_change_state.lc_left();
      const auto lc_stage = planner_output->est_planner_output_list[idx]
                                .scheduler_output.lane_change_state.stage();
      const auto traffic_static_obstacles_info =
          planner_output->est_planner_output_list[idx]
              .scheduler_output.drive_passage.traffic_static_obstacles_info();

      if (status_code == PlannerStatusProto::OK &&
          lc_stage == LaneChangeStage::LCS_NONE && !find_lane_keep_task) {
        find_lane_keep_task = true;
        if ((traffic_static_obstacles_info.enable_stop ||
             traffic_static_obstacles_info.enable_slow_down) &&
            traffic_static_obstacles_info.block_reason !=
                BlockReason::CURB_CROSS) {
          pre_lane_change_safety_info->set_ego_lane_block_by_obs_cnt(
              pre_lane_change_safety_info->ego_lane_block_by_obs_cnt() + 1);
        } else {
          pre_lane_change_safety_info->set_ego_lane_block_by_obs_cnt(0);
        }
      }

      if (status_code == PlannerStatusProto::OK &&
          lc_stage == LaneChangeStage::LCS_EXECUTING) {
        if (lc_left && !find_lc_left_task_safety_ok) {
          find_lc_left_task_safety_ok = true;
          pre_lane_change_safety_info->set_lc_left_safe(true);
          continue;
        } else if (!lc_left && !find_lc_right_task_safety_ok) {
          find_lc_right_task_safety_ok = true;
          pre_lane_change_safety_info->set_lc_right_safe(true);
          continue;
        }
      }
    }
  }

  Log2DDS::LogDataV2(
      "pre_safety_info_debug",
      absl::StrCat(
          "ego_lane_block_by_obs_cnt = ",
          pre_lane_change_safety_info->ego_lane_block_by_obs_cnt(),
          ", have_check_lc_left_safe = ",
          pre_lane_change_safety_info->have_check_lc_left_safe(),
          ", have_check_lc_right_safe = ",
          pre_lane_change_safety_info->have_check_lc_right_safe(),
          ", lc_left_safe = ", pre_lane_change_safety_info->lc_left_safe(),
          ", lc_right_safe = ", pre_lane_change_safety_info->lc_right_safe(),
          ", find_lc_left_task_safety_ok = ", find_lc_left_task_safety_ok,
          ", find_lc_right_task_safety_ok = ", find_lc_right_task_safety_ok,
          ", find_lane_keep_task = ", find_lane_keep_task));

  // for (int idx = 0; idx < planner_output->est_status_list.size(); ++idx) {
  //     if (idx < planner_output->est_planner_output_list.size()) {

  //       const auto status_code =
  //       planner_output->est_status_list[idx].status_code(); const auto
  //       lc_left =
  //       planner_output->est_planner_output_list[idx].scheduler_output.lane_change_state.lc_left();

  //       if (status_code == PlannerStatusProto::LC_SAFETY_CHECK_FAILED)
  //       {
  //         if(lc_left && !find_lc_left_task_safety_fail)
  //         {
  //           find_lc_left_task_safety_fail = true;
  //           pre_lane_change_safety_info->set_have_check_lc_left_safe(true);
  //           pre_lane_change_safety_info->set_lc_left_safe(false);
  //           continue;
  //         }
  //         else if(!lc_left && !find_lc_right_task_safety_fail)
  //         {
  //           find_lc_right_task_safety_fail = true;
  //           pre_lane_change_safety_info->set_have_check_lc_right_safe(true);
  //           pre_lane_change_safety_info->set_lc_right_safe(false);
  //           continue;
  //         }
  //       }
  //   }
  // }

  // for (int idx = 0; idx < planner_output->est_status_list.size(); ++idx) {
  //   if (idx < planner_output->est_planner_output_list.size())
  //   {

  //     const auto lc_stage =
  //     planner_output->est_planner_output_list[idx].scheduler_output.lane_change_state.stage();
  //     const auto lc_left =
  //     planner_output->est_planner_output_list[idx].scheduler_output.lane_change_state.lc_left();

  //     if (lc_stage == LaneChangeStage::LCS_PAUSE)
  //     {
  //       if(lc_left && !find_lc_left_task_safety_fail)
  //       {
  //         find_lc_left_task_safety_fail = true;
  //         pre_lane_change_safety_info->set_have_check_lc_left_safe(true);
  //         pre_lane_change_safety_info->set_lc_left_safe(false);
  //         continue;
  //       }
  //       else if(!lc_left && !find_lc_right_task_safety_fail)
  //       {
  //         find_lc_right_task_safety_fail = true;
  //         pre_lane_change_safety_info->set_have_check_lc_right_safe(true);
  //         pre_lane_change_safety_info->set_lc_right_safe(false);
  //         continue;
  //       }
  //     }
  //   }
  // }
}

void UpdatePrePushStatus(const PathBoundedEstPlannerOutput* planner_output,
                         PushStatusProto* pre_push_status) {
  if (pre_push_status == nullptr || planner_output == nullptr) return;

  bool is_going_force_route_change = false;
  bool force_lc_left = pre_push_status->force_lc_left();
  int force_invalid_count = 0;

  if (planner_output->selector_output.has_value() &&
      planner_output->selector_output->is_going_force_route_change_left
          .has_value()) {
    is_going_force_route_change = true;
    force_lc_left = planner_output->selector_output
                        ->is_going_force_route_change_left.value();
  } else {
    force_invalid_count =
        std::min(pre_push_status->force_invalid_count() + 1, 1000);
  }

  const int task_size =
      std::min(planner_output->est_status_list.size(),
               planner_output->est_planner_output_list.size());
  int target_idx = -1;
  for (int idx = 0; idx < task_size; ++idx) {
    const auto& lc_state = planner_output->est_planner_output_list[idx]
                               .scheduler_output.lane_change_state;
    if ((lc_state.stage() == LaneChangeStage::LCS_EXECUTING ||
         lc_state.stage() == LaneChangeStage::LCS_PAUSE) &&
        lc_state.lc_left() == force_lc_left) {
      target_idx = idx;
      break;
    }
  }

  int push_lane_safe_count = 0;
  int push_lane_unsafe_count = 0;
  int push_lane_solid_count = 0;
  int push_lane_dashed_count = 0;
  const bool pre_same_direction =
      force_lc_left == pre_push_status->force_lc_left();
  if (target_idx >= 0 && target_idx < task_size) {
    if (planner_output->est_status_list[target_idx].status_code() ==
        PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      push_lane_safe_count = 0;
      push_lane_unsafe_count =
          pre_same_direction
              ? std::min(pre_push_status->push_lane_unsafe_count() + 1, 1000)
              : 1;
    } else {
      push_lane_unsafe_count = 0;
      push_lane_safe_count =
          pre_same_direction
              ? std::min(pre_push_status->push_lane_safe_count() + 1, 1000)
              : 1;
    }

    constexpr double kNearSolidLineThres = 25.0;  // m
    const auto& target_lane_seq_info =
        planner_output->est_planner_output_list[target_idx]
            .scheduler_output.drive_passage.lane_seq_info();
    if (target_lane_seq_info) {
      const double dist_to_lc_solidline =
          force_lc_left ? target_lane_seq_info->dist_to_right_solid_line
                        : target_lane_seq_info->dist_to_left_solid_line;
      if (dist_to_lc_solidline < kNearSolidLineThres) {
        push_lane_dashed_count = 0;
        push_lane_solid_count =
            pre_same_direction
                ? std::min(pre_push_status->push_lane_solid_count() + 1, 1000)
                : 1;

      } else {
        push_lane_solid_count = 0;
        push_lane_dashed_count =
            pre_same_direction
                ? std::min(pre_push_status->push_lane_dashed_count() + 1, 1000)
                : 1;
      }
    }
  }

  pre_push_status->set_push_lane_safe_count(push_lane_safe_count);
  pre_push_status->set_push_lane_unsafe_count(push_lane_unsafe_count);
  pre_push_status->set_push_lane_solid_count(push_lane_solid_count);
  pre_push_status->set_push_lane_dashed_count(push_lane_dashed_count);
  pre_push_status->set_force_invalid_count(force_invalid_count);
  pre_push_status->set_force_lc_left(force_lc_left);
}

void UpdateSavedOffset(const PathBoundedEstPlannerOutput* planner_output,
                       PausePushSavedOffsetProto* saved_offset) {
  if (saved_offset == nullptr || planner_output == nullptr) return;
  saved_offset->set_pre_pause_offset(0.0);
  saved_offset->set_pre_push_offset(0.0);
  saved_offset->set_pre_pause_scene(
      PausePushSavedOffsetProto_PauseScene_UnknownScene);
  for (int idx = 0; idx < planner_output->est_planner_output_list.size();
       ++idx) {
    if (abs(planner_output->est_planner_output_list[idx]
                .saved_offset.pre_pause_offset()) > 0) {
      saved_offset->set_pre_pause_offset(
          planner_output->est_planner_output_list[idx]
              .saved_offset.pre_pause_offset());
      saved_offset->set_pre_pause_scene(
          planner_output->est_planner_output_list[idx]
              .saved_offset.pre_pause_scene());
    }
    if (abs(planner_output->est_planner_output_list[idx]
                .saved_offset.pre_push_offset()) > 0) {
      saved_offset->set_pre_push_offset(
          planner_output->est_planner_output_list[idx]
              .saved_offset.pre_push_offset());
    }
  }
}

void UpdateIsConstructionScene(
    const PathBoundedEstPlannerOutput* planner_output,
    ad_byd::planning::ConstructionInfo& construction_info) {
  if (planner_output == nullptr) return;

  bool current_is_construction_scene{false},
      current_construction_scene_debouncing{false}, current_is_lane_keep{true},
      construction_is_left{false};
  bool find_lc_left_task{false}, find_lc_right_task{false};
  bool find_lane_keep_task{false};
  bool pre_is_emergence_lane_scene{false};
  for (int idx = 0; idx < planner_output->est_status_list.size(); ++idx) {
    if (idx < planner_output->est_planner_output_list.size()) {
      current_is_lane_keep = planner_output->est_planner_output_list[0]
                                 .scheduler_output.lane_change_state.stage() ==
                             LaneChangeStage::LCS_NONE;
      pre_is_emergence_lane_scene =
          planner_output->est_planner_output_list[0]
              .scheduler_output.drive_passage.traffic_static_obstacles_info()
              .is_emergence_lane_scene;
      if (!current_is_construction_scene) {
        current_is_construction_scene =
            planner_output->est_planner_output_list[idx]
                .scheduler_output.drive_passage.traffic_static_obstacles_info()
                .is_construction_scene;
        construction_is_left =
            planner_output->est_planner_output_list[idx]
                .scheduler_output.drive_passage.traffic_static_obstacles_info()
                .construction_is_left;
        current_construction_scene_debouncing =
            planner_output->est_planner_output_list[idx]
                .scheduler_output.drive_passage.traffic_static_obstacles_info()
                .construction_scene_debouncing;
      } else {
        break;
      }
    }
  }

  for (int idx = 0; idx < planner_output->est_status_list.size(); ++idx) {
    if (idx < planner_output->est_planner_output_list.size()) {
      const auto status_code =
          planner_output->est_status_list[idx].status_code();
      const auto lc_left = planner_output->est_planner_output_list[idx]
                               .scheduler_output.lane_change_state.lc_left();
      const auto lc_stage = planner_output->est_planner_output_list[idx]
                                .scheduler_output.lane_change_state.stage();
      const auto traffic_static_obstacles_info =
          planner_output->est_planner_output_list[idx]
              .scheduler_output.drive_passage.traffic_static_obstacles_info();

      if (status_code == PlannerStatusProto::OK &&
          (lc_stage == LaneChangeStage::LCS_NONE ||
           lc_stage == LaneChangeStage::LCS_RETURN) &&
          !find_lane_keep_task) {
        find_lane_keep_task = true;
        if ((traffic_static_obstacles_info.enable_stop ||
             traffic_static_obstacles_info.enable_slow_down) &&
            traffic_static_obstacles_info.block_reason !=
                BlockReason::CURB_CROSS) {
          construction_info.ego_lane_block_info.pre_final_stop_s =
              traffic_static_obstacles_info.stop_s;
          construction_info.ego_lane_block_info.pre_obs_id =
              traffic_static_obstacles_info.obs_id;
        } else {
          construction_info.ego_lane_block_info.pre_final_stop_s = 1000;
          construction_info.ego_lane_block_info.pre_obs_id = "none";
        }
      }

      if (status_code == PlannerStatusProto::OK &&
          (lc_stage == LaneChangeStage::LCS_EXECUTING ||
           lc_stage == LaneChangeStage::LCS_PAUSE)) {
        if (lc_left && !find_lc_left_task) {
          find_lc_left_task = true;
          construction_info.left_lane_block_info.pre_final_stop_s =
              traffic_static_obstacles_info.stop_s;
          construction_info.left_lane_block_info.pre_obs_id =
              traffic_static_obstacles_info.obs_id;
          continue;
        } else if (!lc_left && !find_lc_right_task) {
          find_lc_right_task = true;
          construction_info.right_lane_block_info.pre_final_stop_s =
              traffic_static_obstacles_info.stop_s;
          construction_info.right_lane_block_info.pre_obs_id =
              traffic_static_obstacles_info.obs_id;
          continue;
        }
      }
    }
  }

  if (!find_lc_left_task) {
    construction_info.left_lane_block_info.pre_final_stop_s = 1000;
    construction_info.left_lane_block_info.pre_obs_id = "none";
  }

  if (!find_lc_right_task) {
    construction_info.right_lane_block_info.pre_final_stop_s = 1000;
    construction_info.right_lane_block_info.pre_obs_id = "none";
  }

  if (current_is_lane_keep && !construction_info.pre_current_is_lane_keep) {
    construction_info.pre_lc_change_switch_to_lc_keep = true;
  } else {
    construction_info.pre_lc_change_switch_to_lc_keep = false;
  }

  Log2DDS::LogDataV2(
      "construction_debug",
      absl::StrCat(
          "current_is_construction_scene = ", current_is_construction_scene,
          ", debouncing = ", current_construction_scene_debouncing,
          ", ego_stop_s = ",
          construction_info.ego_lane_block_info.pre_final_stop_s,
          ", pre_is_emergence_lane_scene = ", pre_is_emergence_lane_scene,
          ", ego_bloc_id =  ", construction_info.ego_lane_block_info.pre_obs_id,
          ", left_stop_s = ",
          construction_info.left_lane_block_info.pre_final_stop_s,
          ", left_obs_id = ", construction_info.left_lane_block_info.pre_obs_id,
          ", right_stop_s = ",
          construction_info.right_lane_block_info.pre_final_stop_s,
          ", right_obs_id = ",
          construction_info.right_lane_block_info.pre_obs_id,
          ", current_is_lane_keep = ", current_is_lane_keep,
          ", construction_is_left = ", construction_is_left,
          ", construction_info.pre_current_is_lane_keep = ",
          construction_info.pre_current_is_lane_keep,
          ", construction_info.pre_lc_change_switch_to_lc_keep = ",
          construction_info.pre_lc_change_switch_to_lc_keep));

  construction_info.pre_current_is_lane_keep = current_is_lane_keep;
  construction_info.pre_is_emergence_lane_scene = pre_is_emergence_lane_scene;

  if (current_is_construction_scene && !current_construction_scene_debouncing) {
    construction_info.pre_is_construction_scene = true;
    construction_info.none_construction_scene_cnt = 0;
    construction_info.pre_construction_is_left = construction_is_left;
  } else {
    construction_info.none_construction_scene_cnt =
        construction_info.none_construction_scene_cnt + 1;
  }

  if (construction_info.none_construction_scene_cnt >= 15 ||
      construction_info.pre_lc_change_switch_to_lc_keep) {
    construction_info.pre_is_construction_scene = false;
  }

  if (construction_info.none_construction_scene_cnt > 10000 ||
      construction_info.pre_lc_change_switch_to_lc_keep) {
    construction_info.none_construction_scene_cnt = 15;  // reset
  }
}

bool IsMergeBetweenLaneSeqs(
    const ad_byd::planning::MapPtr &map, const Vec2d &start_point,
    const ad_byd::planning::LaneSequencePtr first_lane_seq,
    const ad_byd::planning::LaneSequencePtr second_lane_seq) {
  if (!first_lane_seq || !second_lane_seq) {
    return false;
  }

  auto first_nearest_lane = first_lane_seq->GetNearestLane(start_point);
  auto second_nearest_lane = second_lane_seq->GetNearestLane(start_point);

  if (!first_nearest_lane || !second_nearest_lane) {
    return false;
  }

  auto first_nearest_lane_id = first_nearest_lane->id();
  auto second_nearest_lane_id = second_nearest_lane->id();

  auto first_lanes = first_lane_seq->lanes();
  auto second_lanes = second_lane_seq->lanes();

  auto first_nearest_lane_it = std::find_if(first_lanes.begin(), first_lanes.end(),
      [first_nearest_lane_id](const auto& lane) {
        return lane->id() == first_nearest_lane_id;
      });

  auto second_nearest_lane_it = std::find_if(second_lanes.begin(), second_lanes.end(),
      [second_nearest_lane_id](const auto& lane) {
        return lane->id() == second_nearest_lane_id;
      });

  if (first_nearest_lane_it == first_lanes.end() || second_nearest_lane_it == second_lanes.end()) {
    return false;
  }

  auto first_nearest_lane_index = std::distance(first_lanes.begin(), first_nearest_lane_it);
  auto second_nearest_lane_index = std::distance(second_lanes.begin(), second_nearest_lane_it);
  int lane_size = std::min(first_lanes.size() - first_nearest_lane_index, second_lanes.size() - second_nearest_lane_index);

  bool find_different_lane = false;
  int first_non_zero_num = 0;
  int second_non_zero_num = 0;
  for (int i = 0; i < lane_size; ++i) {
    auto first_lane = first_lanes[i + first_nearest_lane_index];
    auto second_lane = second_lanes[i + second_nearest_lane_index];
    if (!first_lane->is_navigation() || !second_lane->is_navigation()) {
      return false;
    }

    auto first_lane_lc_num = std::abs(map->GetPriorityLaneRelation(first_lane));
    auto second_lane_lc_num = std::abs(map->GetPriorityLaneRelation(second_lane));
    if (first_lane->id() != second_lane->id()) {
      find_different_lane = true;
      if (first_lane_lc_num != 0) {
        first_non_zero_num++;
      }
      if (second_lane_lc_num != 0) {
        second_non_zero_num++;
      }
      if (first_non_zero_num != 0 && second_non_zero_num != 0) {
        return false;
      }
    } else if (find_different_lane) {
      return true;
    }
  }

  return false;
}

void CalcuMinMergeLcNum(const ad_byd::planning::MapPtr &map, const Vec2d &start_point,
    std::vector<LanePathInfo> &lp_infos,int &cur_navi_lc_num) {
  auto size = lp_infos.size();
  if (size <= 1) {
    return;
  }
  for (int i = 0; i < size; i++) {
    auto &first_lane_seq_info = lp_infos.at(i).lane_seq_info();
    auto first_lane_seq = lp_infos.at(i).lane_seq();

    auto &second_lane_seq_info = lp_infos.at((i + 1) % size).lane_seq_info();
    auto second_lane_seq = lp_infos.at((i + 1) % size).lane_seq();

    if (size == 2 && i == 1) {
        continue;
    }

    if (!first_lane_seq_info || !second_lane_seq_info || !first_lane_seq || !second_lane_seq) {
        continue;
    }

    if (IsMergeBetweenLaneSeqs(map, start_point, first_lane_seq, second_lane_seq)) {
        int min_lc_num = std::min(first_lane_seq_info->lc_num, second_lane_seq_info->lc_num);
        Log2DDS::LogDataV2(
          "merge_lc_num_debug",
          absl::StrCat(
              "first_lane_id = ", first_lane_seq->lanes().front()->id(),
              ", first_lane_original_lc_num = ", first_lane_seq_info->lc_num,
              ", second_lane_id = ", second_lane_seq->lanes().front()->id(),
              ", second_lane_original_lc_num = ", second_lane_seq_info->lc_num,
              ", min_lc_num = ", min_lc_num));
        first_lane_seq_info->lc_num = min_lc_num;
        second_lane_seq_info->lc_num = min_lc_num;
        if (first_lane_seq_info->is_current || second_lane_seq_info->is_current) {
            cur_navi_lc_num = min_lc_num;
        }
    }
  }
  return;
}

}  // namespace st::planning
