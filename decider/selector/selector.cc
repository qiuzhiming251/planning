#include "decider/selector/selector.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <valarray>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "plan_common/log.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/log_data.h"
#include "plan_common/path/path.h"
// #include "global/trace.h"
// #include "hmi/events/run_event.h"
// #include "lite/check.h"
// #include "lite/logging.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/util.h"

#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
// #include "ml/selector_models/selector_feature_extractor.h"
// #include "ml/selector_models/selector_scorer.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "decider/selector/candidate_stats.h"
#include "decider/selector/common_feature.h"
#include "decider/selector/cost_feature_base.h"
#include "decider/selector/cost_feature_util.h"
#include "decider/selector/ensure_lc_feasibility.h"
#include "decider/selector/selector_defs.h"
#include "decider/selector/selector_util.h"
#include "decider/selector/traj_cost_features.h"
#include "plan_common/planning_macros.h"
#include "plan_common/util/format_numeric_string.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/scene_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/time_util.h"

#include "modules/cnoa_pnc/planning/proto/hmi_content.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"

namespace st::planning {
namespace {

constexpr double kPreviewDefaultLanePathLength = 40.0;
constexpr double kPreviewHDLanePathLengthForCity = 80.0;
constexpr double kPreviewHDLanePathLengthForHighWay = 110.0;
constexpr double kPreviewLanePathLengthForCity = 60.0;
constexpr double kPreviewLanePathLengthForHighWay = 80.0;
constexpr double kIgnoreBehindLanePathLength = 15.0;
constexpr int kGoingFroceChangeTransitThresold = 3;
constexpr int kBeginRouteChangeTransitThresold = 3;
constexpr int kPreviewLengthCalRightMost = 150.0;

bool IsPerformLaneChange(const LaneChangeStage& lc_stage) {
  return lc_stage == LaneChangeStage::LCS_PAUSE ||
         lc_stage == LaneChangeStage::LCS_EXECUTING;
}
/*|| lc_stage == LaneChangeStage::LCS_RETURN*/

LaneChangeGeneralType ConvertLaneChangeTypeToGeneralType(
    LaneChangeType lane_change_type) {
  switch (lane_change_type) {
    case LaneChangeType::TYPE_NO_CHANGE:
      return LaneChangeGeneralType::LCGT_NO_CHANGE;
    case LaneChangeType::TYPE_OVERTAKE_CHANGE:
    case LaneChangeType::TYPE_STALLED_VEHICLE_CHANGE:
    case LaneChangeType::TYPE_OBSTACLE_CHANGE:
    case LaneChangeType::TYPE_AVOID_CONES:
    case LaneChangeType::TYPE_INTERSECTION_OBS:
      return LaneChangeGeneralType::LCGT_OVERTAKE_CHANGE;
    case LaneChangeType::TYPE_ROAD_SPEED_LIMIT_CHANGE:
    case LaneChangeType::TYPE_MAINROAD_EXIT_CHANGE:
    case LaneChangeType::TYPE_ENTER_MAINROAD_CHANGE:
    case LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE:
    case LaneChangeType::TYPE_MERGE_CHANGE:
    case LaneChangeType::TYPE_CENTER_CHANGE:
    case LaneChangeType::TYPE_CURB_CUTOFF_CHANGE:
    case LaneChangeType::TYPE_AVOID_BUS_LANE:
      return LaneChangeGeneralType::LCGT_ROUTE_CHANGE;
    case LaneChangeType::TYPE_DEFAULT_CHANGE:
      return LaneChangeGeneralType::LCGT_DEFAULT_CHANGE;
    case LaneChangeType::TYPE_CONSTRUCTION_ZONE_CHANGE:
    case LaneChangeType::TYPE_EMERGENCY_CHANGE:
      return LaneChangeGeneralType::LCGT_EMERGENCY_CHANGE;
    case LaneChangeType::TYPE_PADDLE_CHANGE:
      return LaneChangeGeneralType::LCGT_PADDLE_CHANGE;
    case LaneChangeType::TYPE_AVOID_MERGE_AREA:
      return LaneChangeGeneralType::LCGT_AVOID_MERGE_AREA_CHANGE;
  }
  return LaneChangeGeneralType::LCGT_NO_CHANGE;
}

absl::StatusOr<uint64_t> GetLaneIdAtPreviewS(
    const PlannerSemanticMapManager& psmm, const Vec2d& ego_pos,
    const DrivePassage& drive_passage, const double preview_distance) {
  absl::StatusOr<uint64_t> lane_id = -1;
  if (drive_passage.lane_seq_info() == nullptr ||
      drive_passage.lane_seq_info()->lane_seq == nullptr)
    return lane_id;
  ASSIGN_OR_RETURN(
      const auto ego_sl,
      drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(ego_pos),
      _ << "Ego_pos not on drive passage.");
  const auto& lane_seq_ptr =
      drive_passage.lane_seq_info()->lane_seq;  // LaneSequencePtr
  for (auto lane_ptr : lane_seq_ptr->lanes()) {
    if (lane_ptr == nullptr || lane_ptr->points().empty()) {
      // is_all_lane_right_most = false;
      continue;
    }
    ASSIGN_OR_RETURN(const auto lane_first_point_sl,
                     drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                         lane_ptr->points().front()),
                     _ << "Boundary lane front point not on drive passage.");
    ASSIGN_OR_RETURN(const auto lane_end_point_sl,
                     drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                         lane_ptr->points().back()),
                     _ << "Boundary lane back point not on drive passage.");

    if (lane_end_point_sl.s < ego_sl.s) continue;
    if (lane_first_point_sl.s > ego_sl.s + preview_distance) {
      break;
    }
    lane_id = lane_ptr->id();
  }
  return lane_id;
}

void ProcessAutoLaneChangeRequest(
    absl::Time plan_time, const SelectorOutput& selector_output,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorState* selector_state) {
  // // Clear selector request.
  // if (NeedClearSelectorLaneChangeRequest(selector_output, *selector_state)) {
  //   selector_state->selector_lane_change_request.Clear();
  // }

  // // Return if no need to send selector request.
  // const bool already_send_request =
  //     selector_state->selector_lane_change_request.lane_change_type() !=
  //     LaneChangeType::NO_CHANGE;
  // const bool need_lane_change_request =
  //     selector_output.need_lane_change_request ||
  //     selector_state->last_turn_signal_reason ==
  //         TurnSignalReason::FORK_TURN_SIGNAL;
  // if (already_send_request || !need_lane_change_request) {
  //   return;
  // }

  // // Send lane change request for for signal.
  // if (selector_state->last_turn_signal_reason ==
  //     TurnSignalReason::FORK_TURN_SIGNAL) {
  //   selector_state->selector_lane_change_request.set_lane_change_type(
  //       LaneChangeType::DEFAULT_ROUTE_CHANGE);
  //   selector_state->selector_lane_change_request.set_lc_left(
  //       selector_state->last_turn_signal == TurnSignal::TURN_SIGNAL_LEFT);
  //   selector_state->selector_lane_change_request.set_state(
  //       RequestState::RECEIVED_RESPONSE);
  //   SendAutoLaneChangeRequestEvent(
  //       selector_state->selector_lane_change_request);
  //   return;
  // }

  // // Send lane change request for selector lane change.
  // const auto lane_change_request_type = AnalyzeLaneChangeType(
  //     idx_traj_feature_output_map, /*is_paddle_lane_change=*/false,
  //     selector_output.best_traj_idx,
  //     selector_state->selector_lane_change_request.lane_change_type());
  // if (selector_flags.planner_need_to_lane_change_confirmation) {
  //   // Avoid the same lane change request after being rejectd.
  //   const double time_since_last_reject_time =
  //       selector_state->last_user_reject_alc_time.has_value()
  //           ? absl::ToDoubleSeconds(plan_time -
  //                                   *selector_state->last_user_reject_alc_time)
  //           : DBL_MAX;
  //   const bool reach_cool_down_time =
  //       time_since_last_reject_time >
  //       selector_flags.planner_alc_request_reject_cool_down_time;
  //   if (ConvertLaneChangeTypeToGeneralType(lane_change_request_type) !=
  //           ConvertLaneChangeTypeToGeneralType(
  //               selector_state->last_user_reject_alc_type) ||
  //       reach_cool_down_time) {
  //     selector_state->selector_lane_change_request.set_lane_change_type(
  //         lane_change_request_type);
  //     selector_state->selector_lane_change_request.set_lc_left(
  //         idx_traj_feature_output_map.at(selector_output.best_traj_idx)
  //             .lane_change_left);
  //     selector_state->selector_lane_change_request.set_state(
  //         RequestState::WAITING_RESPONSE);
  //   }
  // } else {
  //   selector_state->selector_lane_change_request.set_lane_change_type(
  //       lane_change_request_type);
  //   selector_state->selector_lane_change_request.set_lc_left(
  //       idx_traj_feature_output_map.at(selector_output.best_traj_idx)
  //           .lane_change_left);
  //   selector_state->selector_lane_change_request.set_state(
  //       RequestState::RECEIVED_RESPONSE);
  // }
  // SendAutoLaneChangeRequestEvent(selector_state->selector_lane_change_request);
}

int FindLaneKeepIdx(const absl::flat_hash_map<int, TrajFeatureOutput>&
                        idx_traj_feature_output_map,
                    const int last_selected_idx) {
  int lane_keep_idx = -1;
  if (idx_traj_feature_output_map.contains(last_selected_idx) &&
      !idx_traj_feature_output_map.at(last_selected_idx)
           .is_perform_lane_change) {
    lane_keep_idx = last_selected_idx;
  } else {
    for (const auto& [idx, traj_feature_output] : idx_traj_feature_output_map) {
      if (!traj_feature_output.is_perform_lane_change) {
        lane_keep_idx = idx;
        break;
      }
    }
  }

  return lane_keep_idx;
}

struct LcNumInfo {
  int valid_lane_num = 0;
  int navigation_lane_num = 0;
  std::vector<uint64_t> lane_id_seq;  // from left to right, 0 : left most
};

LcNumInfo CalcLaneNumInfoButEmergency(const SelectorInput& input,
                                      const double s) {
  int valid_lane_num = 0, navigation_lane_num = 0;
  const auto lane_vec = input.psmm->GetValidLanesInRange(
      Vec2dFromApolloTrajectoryPointProto(*input.plan_start_point), s);
  if (!lane_vec.empty()) {
    valid_lane_num = lane_vec.size();
    for (auto lane_id : lane_vec) {
      auto lane_ptr = input.psmm->FindCurveLaneByIdOrNull(
          static_cast<mapping::ElementId>(lane_id));
      if (lane_ptr != nullptr && lane_ptr->is_navigation()) {
        navigation_lane_num++;
      }
    }
  }
  return LcNumInfo{valid_lane_num, navigation_lane_num, lane_vec};
}

int CalcLaneRightIndex(const LcNumInfo& lc_num_info,
                       const mapping::LanePath& lane_path) {
  int index = -1;  // right most, lc_num = 0
  int lane_id_seq_num = lc_num_info.lane_id_seq.size();
  for (int i = 0; i < lane_id_seq_num; i++) {
    auto lane_id = lc_num_info.lane_id_seq[lane_id_seq_num - i -
                                           1];  // lane_path from left to right
    for (auto id : lane_path.lane_ids()) {
      if (static_cast<uint64_t>(id) == lane_id) {
        index = i;
        break;
      }
    }
  }
  return index;
}

int CalcLaneRightIndex(const PlannerSemanticMapManager& psmm,
                       const uint64_t start_lane_id) {
  int index = 0;  // right most, lc_num = 0
  auto lane_ptr = psmm.FindCurveLaneByIdOrNull(start_lane_id);
  std::set<uint64_t> right_lanes;
  while (lane_ptr != nullptr) {
    const auto right_neighbor_lane =
        psmm.FindCurveLaneByIdOrNull(lane_ptr->right_lane_id());
    if ((lane_ptr->right_lane_id() == 0) ||
        right_lanes.count(lane_ptr->right_lane_id()) > 0 ||
        (right_neighbor_lane &&
         right_neighbor_lane->type() == LaneType::LANE_EMERGENCY)) {
      break;
    }
    right_lanes.insert(lane_ptr->right_lane_id());
    lane_ptr = right_neighbor_lane;
    index++;
  }
  return index;
}

void UpdateSelectorStateAfterSelection(
    absl::Time plan_time, const SelectorFlags& selector_flags,
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const int last_selected_idx, const int final_selected_idx,
    const int best_traj_idx, const bool is_paddle_lane_change,
    const SelectorOutput& selector_output, SelectorState* selector_state,
    SelectorDebugProto* selector_debug,
    const DeviateNaviInput& deviate_navi_input) {
  if (final_selected_idx < 0) {
    return;
  }
  // if (selector_output.turn_signal_reason ==
  //     TurnSignalReason::PREPARE_LANE_CHANGE_TURN_SIGNAL) {
  //   selector_state->lane_change_type = AnalyzeLaneChangeType(
  //       idx_traj_feature_output_map, is_paddle_lane_change, best_traj_idx,
  //       selector_state->lane_change_type);
  // } else {
  selector_state->lane_change_type = internal::AnalyzeLaneChangeType(
      idx_traj_feature_output_map, is_paddle_lane_change, last_selected_idx,
      final_selected_idx, selector_state->lane_change_type);
  // }

  const auto& final_lc_stage =
      results[final_selected_idx].scheduler_output.lane_change_state.stage();
  const auto& last_lc_info = selector_state->last_lc_info;

  selector_state->lane_change_general_type =
      ConvertLaneChangeTypeToGeneralType(selector_state->lane_change_type);
  // Store first frame time of noa active
  if (deviate_navi_input.func_id ==
          st::Behavior_FunctionId::Behavior_FunctionId_HW_NOA ||
      deviate_navi_input.func_id ==
          st::Behavior_FunctionId::Behavior_FunctionId_CITY_NOA) {
    if (!selector_state->activate_selector_time.has_value()) {
      selector_state->activate_selector_time = plan_time;
    }
  } else {
    selector_state->activate_selector_time.reset();
  }

  // Re AnalyzeLCreason

  selector_state->lane_change_reason = internal::ReAnalyzeLaneChangeReason(
      selector_state->lane_change_reason, selector_state->lane_change_type);

  const auto& final_scheduler = results[final_selected_idx].scheduler_output;
  if (IsPerformLaneChange(final_scheduler.lane_change_state.stage())) {
    // Store last lane change time and direction.
    selector_state->last_lc_info.set_lc_left(
        final_scheduler.lane_change_state.lc_left());
    ToProto(plan_time, selector_state->last_lc_info.mutable_lane_change_time());
    selector_state->last_lc_info.set_lane_change_type(
        selector_state->lane_change_type);
    // Store lane change start time.
    if (!selector_state->start_lane_change_time.has_value()) {
      selector_state->start_lane_change_time = plan_time;
    }

    // Store lane change start give up time.
    const double time_after_lc_start = absl::ToDoubleSeconds(
        plan_time - *selector_state->start_lane_change_time);
    if (time_after_lc_start >
        selector_flags.planner_max_allow_lc_time_before_give_up) {
      selector_state->give_up_lane_change_time = plan_time;
    }
    selector_state->last_lc_info.set_lane_change_stage(final_lc_stage);
    if (last_selected_idx == final_selected_idx &&
        last_lc_info.lc_left() == selector_state->last_lc_info.lc_left() &&
        last_lc_info.lane_change_type() ==
            selector_state->last_lc_info.lane_change_type() &&
        last_lc_info.lane_change_type() ==
            LaneChangeType::TYPE_OVERTAKE_CHANGE &&
        last_lc_info.lane_change_stage() ==
            selector_state->last_lc_info.lane_change_stage() &&
        last_lc_info.lane_change_stage() == LaneChangeStage::LCS_PAUSE) {
      selector_state->overtake_lc_pause_successive_count += 1;
    } else if (selector_state->last_lc_info.lane_change_type() ==
                   LaneChangeType::TYPE_OVERTAKE_CHANGE &&
               selector_state->last_lc_info.lane_change_stage() ==
                   LaneChangeStage::LCS_PAUSE) {
      selector_state->overtake_lc_pause_successive_count = 1;
    } else {
      selector_state->overtake_lc_pause_successive_count = 0;
    }
  } else {
    selector_state->start_lane_change_time.reset();
    selector_state->overtake_lc_pause_successive_count = 0;
  }

  constexpr double kForwardDistThresholdPassedSplit = 10.0;
  constexpr double kBackwardDistThresholdPassedSplit = -50.0;
  static std::pair<double, double> prev_dist_to_nearest_split = {
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::max()};
  const auto& lane_seq_info = final_scheduler.drive_passage.lane_seq_info();
  if (lane_seq_info != nullptr) {
    if (prev_dist_to_nearest_split.second >= kForwardDistThresholdPassedSplit &&
        lane_seq_info->dist_to_nearest_split.second <
            kForwardDistThresholdPassedSplit) {
      ToProto(
          plan_time,
          selector_state->last_passed_split_info.mutable_passed_split_time());
      selector_state->last_passed_split_info.set_is_valid_avoid_split_dis(true);
    }

    if (prev_dist_to_nearest_split.first >= kBackwardDistThresholdPassedSplit &&
        lane_seq_info->dist_to_nearest_split.first <
            kBackwardDistThresholdPassedSplit) {
      selector_state->last_passed_split_info.set_is_valid_avoid_split_dis(
          false);
    }
    prev_dist_to_nearest_split = lane_seq_info->dist_to_nearest_split;
    Log2DDS::LogDataV2("lane_seq_info->dist_to_nearest_split.first",
                       lane_seq_info->dist_to_nearest_split.first);
    Log2DDS::LogDataV2("lane_seq_info->dist_to_nearest_split.second",
                       lane_seq_info->dist_to_nearest_split.second);
  }

  // Update red light stop time.
  bool has_red_light_stop_s = true;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) {
      continue;
    }
    has_red_light_stop_s =
        has_red_light_stop_s && results[idx].redlight_lane_id.has_value();
  }
  if (has_red_light_stop_s) {
    selector_state->last_redlight_stop_time = plan_time;
  }

  // Update object cost history
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) {
      continue;
    }
    auto it = idx_traj_feature_output_map.find(idx);
    if (it != idx_traj_feature_output_map.end()) {
      selector_state->object_cost_history[idx] = it->second.object_cost_map;
    }
  }
  std::ignore =
      selector_state->object_cost_history.UpdateAndRemoveDecay(plan_time);

  selector_debug->mutable_debugstring()->Add(absl::StrCat(
      "Selector state: lane_change_type: ",
      LaneChangeType_Name(selector_state->lane_change_type),
      " lane_change_general_type: ",
      LaneChangeGeneralType_Name(selector_state->lane_change_general_type)));

  ProcessAutoLaneChangeRequest(plan_time, selector_output, selector_flags,
                               idx_traj_feature_output_map, selector_state);
}
std::vector<PlannerStatus> PreFilterEstResults(
    absl::Time plan_time, const PlannerSemanticMapManager& psmm,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results,
    /*const SelectorCommonFeature& common_feature,*/ int last_selected_idx,
    SelectorDebugProto* selector_debug, SelectorState* selector_state) {
  std::vector<PlannerStatus> filtered_est_status = est_status;
  // todo(xxx)
  return filtered_est_status;
}

int ForceLaneChangeDirectionByMinLaneChangeNumber(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status) {
  int best_lane_change_traj_idx = -1;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok() &&
        (est_status[idx].status_code() !=
             PlannerStatusProto::LC_SAFETY_CHECK_FAILED ||
         (est_status[idx].status_code() ==
              PlannerStatusProto::LC_SAFETY_CHECK_FAILED &&
          results[idx].safety_check_failed_reason !=
              SafetyCheckFailedReason::LC_UNSAFE_REASON_MOVING_OBJECT))) {
      continue;
    }
    if (!IsPerformLaneChange(
            results[idx].scheduler_output.lane_change_state.stage())) {
      continue;
    }

    if (best_lane_change_traj_idx == -1) {
      best_lane_change_traj_idx = idx;
    }

    if (results[idx].scheduler_output.drive_passage.lane_seq_info()->lc_num <
        results[best_lane_change_traj_idx]
            .scheduler_output.drive_passage.lane_seq_info()
            ->lc_num) {
      best_lane_change_traj_idx = idx;
    }
  }
  return best_lane_change_traj_idx;
}

int ForceLaneChangeDirectionByMinCost(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map) {
  int best_lane_change_traj_idx = -1;
  for (const auto& [idx, cost] : idx_traj_cost_map) {
    if (!est_status[idx].ok() &&
        (est_status[idx].status_code() !=
             PlannerStatusProto::LC_SAFETY_CHECK_FAILED ||
         (est_status[idx].status_code() ==
              PlannerStatusProto::LC_SAFETY_CHECK_FAILED &&
          results[idx].safety_check_failed_reason !=
              SafetyCheckFailedReason::LC_UNSAFE_REASON_MOVING_OBJECT))) {
      continue;
    }
    if (!IsPerformLaneChange(
            results[idx].scheduler_output.lane_change_state.stage())) {
      continue;
    }

    if (best_lane_change_traj_idx == -1) {
      best_lane_change_traj_idx = idx;
    }

    if (cost.static_cost <
        idx_traj_cost_map.at(best_lane_change_traj_idx).static_cost) {
      best_lane_change_traj_idx = idx;
    }
  }
  return best_lane_change_traj_idx;
}

std::optional<bool> IsGoingToForceRouteChangeLeft(
    int final_selected_idx, const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map,
    const SelectorCommonFeature& selector_common_feature,
    const bool use_begin_route_change) {
  int lane_keep_lane_idx = -1;
  bool lane_change_left = false;
  lane_keep_lane_idx =
      FindLaneKeepIdx(idx_traj_feature_output_map, final_selected_idx);
  if (lane_keep_lane_idx < 0) {
    return std::nullopt;
  }
  if (IsPerformLaneChange(results[final_selected_idx]
                              .scheduler_output.lane_change_state.stage())) {
    // already in lane change stage.
    return std::nullopt;
  }

  if ((!use_begin_route_change &&
       !idx_traj_feature_output_map.at(lane_keep_lane_idx)
            .has_obvious_route_cost) ||
      (use_begin_route_change &&
       !idx_traj_feature_output_map.at(lane_keep_lane_idx)
            .has_begin_route_change)) {
    // not in obvious route cost.
    return std::nullopt;
  }

  int best_lane_change_traj_idx = -1;
  const bool only_lane_change_for_navi =
      idx_traj_feature_output_map.at(lane_keep_lane_idx)
          .lane_change_for_navi_cost &&
      !idx_traj_feature_output_map.at(lane_keep_lane_idx)
           .lane_change_for_stalled_vehicle &&
      !idx_traj_feature_output_map.at(lane_keep_lane_idx)
           .lane_change_for_stationary_obj &&
      !idx_traj_feature_output_map.at(lane_keep_lane_idx)
           .lane_change_for_avoid_cones &&
      !idx_traj_feature_output_map.at(lane_keep_lane_idx)
           .lane_change_for_length_cutoff;
  if (only_lane_change_for_navi) {
    best_lane_change_traj_idx =
        ForceLaneChangeDirectionByMinLaneChangeNumber(results, est_status);
  } else {
    best_lane_change_traj_idx = ForceLaneChangeDirectionByMinCost(
        results, est_status, idx_traj_cost_map);
  }
  if (best_lane_change_traj_idx == -1) {
    return std::nullopt;
  }

  if (results[best_lane_change_traj_idx]
              .scheduler_output.drive_passage.lane_seq_info() == nullptr ||
      results[lane_keep_lane_idx]
              .scheduler_output.drive_passage.lane_seq_info() == nullptr) {
    return std::nullopt;
  }

  if (idx_traj_feature_output_map.at(lane_keep_lane_idx)
          .lane_change_for_navi_cost ||
      idx_traj_feature_output_map.at(lane_keep_lane_idx)
          .lane_change_for_merge_lane) {
    if (results[best_lane_change_traj_idx]
            .scheduler_output.drive_passage.lane_seq_info()
            ->lc_num > results[lane_keep_lane_idx]
                           .scheduler_output.drive_passage.lane_seq_info()
                           ->lc_num) {
      return std::nullopt;
    }
  }

  lane_change_left = results[best_lane_change_traj_idx]
                         .scheduler_output.lane_change_state.lc_left();

  bool is_pass_check = true;
  const LaneChangeType lane_change_type =
      use_begin_route_change
          ? LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE
          : internal::AnalyzeLaneChangeType(
                idx_traj_feature_output_map, false, lane_keep_lane_idx,
                best_lane_change_traj_idx, LaneChangeType::TYPE_NO_CHANGE);
  is_pass_check = PassPrepareLcCheck(
      results, est_status, idx_traj_feature_output_map, idx_traj_cost_map,
      selector_common_feature, best_lane_change_traj_idx, lane_keep_lane_idx,
      lane_change_type, use_begin_route_change);

  if (!is_pass_check) {
    return std::nullopt;
  }

  return lane_change_left;
}

void UpdateRouteIntentionInSeletorState(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const SelectorCommonFeature& selector_common_feature,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const int final_selected_idx,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map,
    const bool use_begin_route_change,
    RouteIntentionInfo* route_change_left_info) {
  if (route_change_left_info == nullptr) {
    return;
  }
  const auto& route_change_left = IsGoingToForceRouteChangeLeft(
      final_selected_idx, results, est_status, idx_traj_feature_output_map,
      idx_traj_cost_map, selector_common_feature, use_begin_route_change);

  route_change_left_info->successive_count =
      route_change_left_info->route_change_left == route_change_left
          ? route_change_left_info->successive_count + 1
          : 1;
  route_change_left_info->route_change_left = route_change_left;
}

const std::optional<bool> CalFinalRouteIntention(
    const RouteIntentionInfo& cur_route_change_left_info,
    const std::optional<bool>& last_route_change_left,
    const int route_change_frame_thresold) {
  if (cur_route_change_left_info.successive_count >=
      route_change_frame_thresold) {
    return cur_route_change_left_info.route_change_left;
  }
  return last_route_change_left;
}

void UpdateSelectorOutput(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const SelectorCommonFeature& selector_common_feature,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    bool in_high_way, int final_selected_idx, int last_selected_idx,
    const RouteIntentionInfo& begin_route_change_left_info,
    const RouteIntentionInfo& force_route_change_left_info,
    SelectorOutput* selector_output, SelectorDebugProto* selector_debug,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map,
    bool is_open_gap, const SelectorFlags& selector_flags) {
  selector_output->has_begin_route_change =
      idx_traj_feature_output_map.at(final_selected_idx).has_begin_route_change;
  selector_output->begin_route_change_left = CalFinalRouteIntention(
      begin_route_change_left_info, selector_output->begin_route_change_left,
      kBeginRouteChangeTransitThresold);

  selector_output->is_going_force_route_change_left =
      CalFinalRouteIntention(force_route_change_left_info,
                             selector_output->is_going_force_route_change_left,
                             kGoingFroceChangeTransitThresold);
  selector_debug->mutable_debugstring()->Add(absl::StrCat(
      "route_intention_force: ",
      force_route_change_left_info.route_change_left.has_value()
          ? force_route_change_left_info.route_change_left.value()
          : -1,
      " successive_count: ", force_route_change_left_info.successive_count));
  selector_debug->mutable_debugstring()->Add(absl::StrCat(
      "route_intention_begin: ",
      begin_route_change_left_info.route_change_left.has_value()
          ? begin_route_change_left_info.route_change_left.value()
          : -1,
      " successive_count: ", begin_route_change_left_info.successive_count));

  selector_output->selected_idx = final_selected_idx;
  selector_output->last_selected_idx = last_selected_idx;
  selector_output->in_high_way = in_high_way;
  selector_debug->mutable_debugstring()->Add(absl::StrCat(
      "has_begin_route_change: ", selector_output->has_begin_route_change,
      " begin_route_change_left: ",
      selector_output->begin_route_change_left.has_value()
          ? selector_output->begin_route_change_left.value()
          : -1,
      " is_going_force_route_change_left: ",
      selector_output->is_going_force_route_change_left.has_value()
          ? selector_output->is_going_force_route_change_left.value()
          : -1,
      " selected_idx: ", selector_output->selected_idx,
      " last_selected_idx: ", selector_output->last_selected_idx));
  // selector_output->merge_point =
  //     idx_traj_feature_output_map.at(final_selected_idx).merge_point;
  // Update turn signal and reason.
  if (final_selected_idx >= 0) {
    const auto& final_scheduler = results[final_selected_idx].scheduler_output;
    if (IsPerformLaneChange(final_scheduler.lane_change_state.stage())) {
      selector_output->turn_signal = final_scheduler.lane_change_state.lc_left()
                                         ? TurnSignal::TURN_SIGNAL_LEFT
                                         : TurnSignal::TURN_SIGNAL_RIGHT;
      selector_output->turn_signal_reason =
          TurnSignalReason::LANE_CHANGE_TURN_SIGNAL;
      selector_debug->mutable_debugstring()->Add(absl::StrCat(
          "TurnSignal: ", selector_output->turn_signal,
          " TurnSignalReason: ", selector_output->turn_signal_reason));
    } else if (selector_output->is_going_force_route_change_left.has_value() &&
               selector_flags.planner_enable_turn_light_when_open_gap) {  // gap
      if (is_open_gap) {
        selector_output->turn_signal =
            *selector_output->is_going_force_route_change_left
                ? TurnSignal::TURN_SIGNAL_LEFT
                : TurnSignal::TURN_SIGNAL_RIGHT;
        selector_output->turn_signal_reason =
            TurnSignalReason::OPEN_GAP_TURN_SIGNAL;
      } else {
        selector_output->turn_signal = TurnSignal::TURN_SIGNAL_NONE;
        selector_output->turn_signal_reason = TurnSignalReason::TURN_SIGNAL_OFF;
      }

      selector_debug->mutable_debugstring()->Add(absl::StrCat(
          "TurnSignal: ", selector_output->turn_signal,
          " TurnSignalReason: ", selector_output->turn_signal_reason));
    } else if (selector_output->begin_route_change_left.has_value() &&
               selector_flags.planner_enable_turn_light_when_open_gap) {  // gap
      if (is_open_gap) {
        selector_output->turn_signal = *selector_output->begin_route_change_left
                                           ? TurnSignal::TURN_SIGNAL_LEFT
                                           : TurnSignal::TURN_SIGNAL_RIGHT;
        selector_output->turn_signal_reason =
            TurnSignalReason::OPEN_GAP_TURN_SIGNAL;
      } else {
        selector_output->turn_signal = TurnSignal::TURN_SIGNAL_NONE;
        selector_output->turn_signal_reason = TurnSignalReason::TURN_SIGNAL_OFF;
      }

      selector_debug->mutable_debugstring()->Add(absl::StrCat(
          "TurnSignal: ", selector_output->turn_signal,
          " TurnSignalReason: ", selector_output->turn_signal_reason));
    }
  }

  // Check the route lane change failed or not.
  if (final_selected_idx >= 0) {
    const auto& final_scheduler = results[final_selected_idx].scheduler_output;
    const auto final_passage_lane_seq =
        final_scheduler.drive_passage.lane_seq_info();
    if (!IsPerformLaneChange(final_scheduler.lane_change_state.stage())) {
      const int lc_num = final_scheduler.lc_num;
      if (final_passage_lane_seq != nullptr &&
          (final_passage_lane_seq->dist_to_navi_end <
           selector_flags.planner_miss_navi_length * lc_num)) {
        selector_output->may_miss_navi = true;
      }
    }
  }
}

int ChooseBestCostTraj(
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map) {
  int best_traj_idx = -1;
  for (const auto& [idx, cost] : idx_traj_cost_map) {
    // Filter LaneChangeFailed branch
    if (!est_status[idx].ok()) {
      continue;
    }
    if (best_traj_idx == -1) {
      best_traj_idx = idx;
    }
    const auto best_start_lane_id =
        results[best_traj_idx]
            .scheduler_output.drive_passage.lane_path()
            .front()
            .lane_id();
    const auto start_lane_id = results[idx]
                                   .scheduler_output.drive_passage.lane_path()
                                   .front()
                                   .lane_id();
    if (start_lane_id == best_start_lane_id) {
      if (cost < idx_traj_cost_map.at(best_traj_idx)) {
        best_traj_idx = idx;
      }
    } else {
      if (cost.cost_common < idx_traj_cost_map.at(best_traj_idx).cost_common) {
        best_traj_idx = idx;
      }
    }
  }
  return best_traj_idx;
}

bool HasEgoCornerCrossLine(const VehicleGeometryParamsProto& vehicle_geom,
                           const ApolloTrajectoryPointProto& plan_start_point,
                           const SchedulerOutput& scheduler_output) {
  constexpr double kMaxTolerance = 0.1;
  const double length = vehicle_geom.length();
  const double width = vehicle_geom.width();
  const double ego_front_to_ra = vehicle_geom.front_edge_to_center();
  const double heading = plan_start_point.path_point().theta();
  const auto ego_center =
      Vec2dFromApolloTrajectoryPointProto(plan_start_point) +
      (ego_front_to_ra - 0.5 * length) * Vec2d::UnitFromAngle(heading);
  const Box2d ego_box(ego_center, heading, length, width);

  for (const auto& corner : ego_box.GetCornersCounterClockwise()) {
    ASSIGN_OR_CONTINUE(const auto corner_frenet,
                       scheduler_output.drive_passage
                           .QueryLaterallyUnboundedFrenetCoordinateAt(corner));
    const auto [right_boundary, left_boundary] =
        scheduler_output.drive_passage.QueryEnclosingLaneBoundariesAtS(
            corner_frenet.s);
    const double boundary_right_l =
        right_boundary.has_value()
            ? std::max(right_boundary->lat_offset, -kMaxHalfLaneWidth)
            : -kMaxHalfLaneWidth;
    const double boundary_left_l =
        left_boundary.has_value()
            ? std::min(left_boundary->lat_offset, kMaxHalfLaneWidth)
            : kMaxHalfLaneWidth;
    if (corner_frenet.l < boundary_left_l + kMaxTolerance &&
        corner_frenet.l > boundary_right_l - kMaxTolerance) {
      return true;
    }
  }
  return false;
}

int CalcBeginChangeFrameCount(
    LaneChangeType lc_type, const SelectorFlags& selector_flags,
    bool highway_mode, const TrajFeatureOutput* traj_feature_lane_keep_index,
    const TrajFeatureOutput* traj_feature_best_traj_index,
    ad_byd::planning::MapType map_type) {
  bool has_obvious_route_cost =
      traj_feature_lane_keep_index != nullptr &&
      traj_feature_lane_keep_index->has_obvious_route_cost;
  bool is_hd_map =
      (map_type == ad_byd::planning::MapType::HD_MAP) ? true : false;
  if (lc_type == LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE ||
      lc_type == LaneChangeType::TYPE_MERGE_CHANGE) {
    return has_obvious_route_cost && !highway_mode && is_hd_map
               ? selector_flags.planner_begin_emergency_frame
               : selector_flags.planner_begin_radical_lane_change_frame;
  } else if (lc_type == LaneChangeType::TYPE_STALLED_VEHICLE_CHANGE) {
    return has_obvious_route_cost
               ? selector_flags.planner_begin_emergency_frame
               : selector_flags.planner_begin_radical_lane_change_frame;
  } else if (lc_type == LaneChangeType::TYPE_OBSTACLE_CHANGE) {
    return selector_flags.planner_begin_emergency_frame;
  } else if (lc_type == LaneChangeType::TYPE_AVOID_CONES) {
    return highway_mode ? selector_flags.planner_begin_cones_frame
                        : selector_flags.planner_begin_emergency_frame;
  } else if (lc_type == LaneChangeType::TYPE_ROAD_SPEED_LIMIT_CHANGE) {
    return highway_mode
               ? selector_flags.planner_begin_lane_change_frame_progress
               : selector_flags
                     .planner_begin_lane_change_frame_progress_city_noa;
  } else if (lc_type == LaneChangeType::TYPE_OVERTAKE_CHANGE) {
    if (traj_feature_best_traj_index != nullptr) {
      return highway_mode
                 ? traj_feature_best_traj_index->overtake_begin_frame
                 : selector_flags
                       .planner_begin_lane_change_frame_progress_city_noa;
    }
    return highway_mode
               ? selector_flags.planner_begin_lane_change_frame_progress
               : selector_flags
                     .planner_begin_lane_change_frame_progress_city_noa;
  } else if (lc_type == LaneChangeType::TYPE_INTERSECTION_OBS) {
    return selector_flags.planner_begin_lane_change_frame_progress_city_noa;
    // CITY NOA dynamic lane select
  } else {
    return selector_flags.planner_begin_lane_change_frame;
  }
}

int TransitLaneChangeStage(
    int last_selected_idx, int best_traj_idx, int successive_count,
    const std::vector<EstPlannerOutput>& results,
    const SelectorFlags& selector_flags,
    const VehicleGeometryParamsProto& vehicle_geom,
    const ApolloTrajectoryPointProto& plan_start_point,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const double dist_to_tunnel, const bool ego_near_intersection,
    const bool highway_mode, SelectorState* selector_state,
    SelectorOutput* selector_output, int* begin_lane_change_frame,
    SelectorDebugProto* selector_debug,
    const ad_byd::planning::MapType& map_type) {
  int final_selected_idx = last_selected_idx;
  const auto last_lc_stage =
      results[last_selected_idx].scheduler_output.lane_change_state.stage();
  const auto best_lc_stage =
      results[best_traj_idx].scheduler_output.lane_change_state.stage();
  selector_debug->mutable_debugstring()->Add(absl::StrCat(
      "last_selected_idx: ", last_selected_idx, " best_traj_idx: ",
      best_traj_idx, " last_lc_stage: ", LaneChangeStage_Name(last_lc_stage),
      " best_lc_stage: ", LaneChangeStage_Name(best_lc_stage),
      " lock_intersection: ", selector_state->has_selected_intersection));
  if (selector_state->has_selected_intersection) {
    // Trick: forbiden lc/split in junction
    selector_debug->mutable_debugstring()->Add("forbiden lc in juntion");
    final_selected_idx = last_selected_idx;
  }

  int lane_keep_idx =
      FindLaneKeepIdx(idx_traj_feature_output_map, last_selected_idx);

  if (lane_keep_idx == -1) {
    lane_keep_idx = best_traj_idx;
  }

  switch (last_lc_stage) {
    case LaneChangeStage::LCS_NONE: {
      // LK -> LC
      const bool task_switch = last_selected_idx != best_traj_idx;
      const bool lc_or_taskswitch_in_cross =
          best_lc_stage == LaneChangeStage::LCS_EXECUTING ||
          (ego_near_intersection && task_switch);
      const bool is_best_traj_lk =
          idx_traj_feature_output_map.contains(best_traj_idx) &&
          !idx_traj_feature_output_map.at(best_traj_idx)
               .is_perform_lane_change &&
          !idx_traj_feature_output_map.at(last_selected_idx)
               .lane_change_for_stalled_vehicle &&
          !idx_traj_feature_output_map.at(last_selected_idx)
               .lane_change_for_length_cutoff;
      if (ego_near_intersection && task_switch && is_best_traj_lk) {
        selector_debug->mutable_debugstring()->Add(
            absl::StrCat("is_best_traj_lk: ", is_best_traj_lk));
        final_selected_idx = lane_keep_idx;
      } else if (lc_or_taskswitch_in_cross) {
        selector_debug->mutable_debugstring()->Add("lk 2 lc");
        const auto lc_type = internal::AnalyzeLaneChangeType(
            idx_traj_feature_output_map, /*is_paddle_lane_change=*/false,
            last_selected_idx, best_traj_idx,
            /*last_lane_change_type=*/LaneChangeType::TYPE_NO_CHANGE);
        selector_debug->mutable_debugstring()->Add(
            absl::StrCat("lc_type: ", LaneChangeType_Name(lc_type)));

        auto lc_feasibility = EnsureLcFeasibilty(
            results, idx_traj_feature_output_map, dist_to_tunnel, highway_mode,
            best_traj_idx, lane_keep_idx, lc_type);
        selector_output->best_traj_lc_unable_reason = lc_feasibility;
        if (idx_traj_feature_output_map.at(best_traj_idx)
                .cross_solid_boundary) {
          selector_output->ignore_cross_solid_line =
              (lc_feasibility != LcFeasibility::FEASIBILITY_LINE_TYPE);
        }
        bool lc_feasible = lc_feasibility == LcFeasibility::FEASIBILITY_OK;
        // Decide lane change frame for different lane change types.
        *begin_lane_change_frame = CalcBeginChangeFrameCount(
            lc_type, selector_flags, highway_mode,
            idx_traj_feature_output_map.contains(lane_keep_idx)
                ? &idx_traj_feature_output_map.at(lane_keep_idx)
                : nullptr,
            idx_traj_feature_output_map.contains(best_traj_idx)
                ? &idx_traj_feature_output_map.at(best_traj_idx)
                : nullptr,

            map_type);
        if (highway_mode &&
            results[best_traj_idx].scheduler_output.lane_change_state.lc_left())
          *begin_lane_change_frame = std::min(*begin_lane_change_frame, 10);

        selector_debug->mutable_debugstring()->Add(
            absl::StrCat("begin_lane_change_frame: ", *begin_lane_change_frame,
                         " successive_count: ", successive_count));
        // Final choose lane change or not.
        if (!lc_feasible) {
          final_selected_idx = lane_keep_idx;  // lane keep;
          selector_debug->mutable_debugstring()->Add(absl::StrCat(
              "best_traj_idx: ", best_traj_idx, ", lc_unable_reason: ",
              LcFeasibility_Name(selector_output->best_traj_lc_unable_reason)));
        }

        if (lc_feasible && successive_count >= *begin_lane_change_frame) {
          selector_output->need_lane_change_request = true;
          if (!selector_flags.planner_need_to_lane_change_confirmation) {
            final_selected_idx = best_traj_idx;
          }
        }
        // Send preturn signal.
        if (lc_feasible && successive_count < *begin_lane_change_frame &&
            ((successive_count >= selector_flags.planner_begin_signal_frame &&
              highway_mode) ||
             (successive_count >=
                  selector_flags.planner_begin_signal_frame_city_noa &&
              !highway_mode)) &&
            /*enable preturn light*/ 0) {
          // Begin lane change prepare.
          selector_output->turn_signal =
              results[best_traj_idx]
                      .scheduler_output.lane_change_state.lc_left()
                  ? TURN_SIGNAL_LEFT
                  : TURN_SIGNAL_RIGHT;
          selector_output->turn_signal_reason =
              TurnSignalReason::PREPARE_LANE_CHANGE_TURN_SIGNAL;
          selector_output->need_lane_change_request = true;
          selector_state->lc_prepare_stage_lane_path =
              results[best_traj_idx].scheduler_output.drive_passage.lane_path();
          selector_debug->mutable_debugstring()->Add(absl::StrCat(
              "preturn signal: turn_signal: ", selector_output->turn_signal,
              " turn_signal_reason: ", selector_output->turn_signal_reason));
        }
      } else if ((best_traj_idx != final_selected_idx) &&
                 (best_lc_stage == LaneChangeStage::LCS_NONE)) {
        // Trick: for split change in intersection
        selector_debug->mutable_debugstring()->Add(
            absl::StrCat("not lane change but have different task "));
        if (ego_near_intersection) {
          *begin_lane_change_frame =
              selector_flags
                  .planner_begin_change_best_lk_trajectory_frame_city_noa;
        } else {
          *begin_lane_change_frame =
              selector_flags
                  .planner_begin_change_best_lk_trajectory_frame_highway_noa;
        }
        selector_debug->mutable_debugstring()->Add(
            absl::StrCat("begin_lane_change_frame", *begin_lane_change_frame,
                         " successive_count: ", successive_count));
        if (successive_count >= *begin_lane_change_frame) {
          selector_output->need_lane_change_request = true;
          if (!selector_flags.planner_need_to_lane_change_confirmation) {
            selector_state->has_selected_intersection = true;
            selector_debug->mutable_debugstring()->Add(
                absl::StrCat("has_lane_changed_intersection: ",
                             selector_state->has_selected_intersection));
            final_selected_idx = best_traj_idx;
          }
        }
      }
      break;
    }
    case LaneChangeStage::LCS_PAUSE: {
      if (results[best_traj_idx].is_init_return_scene) {
        final_selected_idx = lane_keep_idx;
        selector_debug->mutable_debugstring()->Add(
            absl::StrCat("is_init_return_scene: ",
                         results[best_traj_idx].is_init_return_scene));
      }
      break;
    }
    case LaneChangeStage::LCS_RETURN:
    case LaneChangeStage::LCS_EXECUTING:
    case LaneChangeStage::LCS_WAITING: {
      // LC -> LK LCR
      selector_debug->mutable_debugstring()->Add(absl::StrCat(
          "lc to lk , last_lc_stage: ", LaneChangeStage_Name(last_lc_stage)));
      const bool ego_corner_cross_line =
          HasEgoCornerCrossLine(vehicle_geom, plan_start_point,
                                results[last_selected_idx].scheduler_output);
      if (auto cancel_lc_reason =
              CalcLcCancelReason(results, idx_traj_feature_output_map,
                                 last_selected_idx, best_traj_idx);
          cancel_lc_reason != LaneChangeCancelReason::NOT_CANCEL) {
        selector_debug->mutable_debugstring()->Add(
            absl::StrCat(LaneChangeCancelReason_Name(cancel_lc_reason)));
        *begin_lane_change_frame =
            selector_flags.planner_fast_abort_lane_change_frame;
      } else if (ego_corner_cross_line && !ego_near_intersection) {
        selector_debug->mutable_debugstring()->Add("ego_corner_has_cross_line");
        selector_output->ego_corner_cross_line = true;
        *begin_lane_change_frame = -1;
      } else {
        const auto lc_type = selector_state->lane_change_type;
        if (lc_type == LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE ||
            lc_type == LaneChangeType::TYPE_ROAD_SPEED_LIMIT_CHANGE ||
            lc_type == LaneChangeType::TYPE_MERGE_CHANGE ||
            lc_type == LaneChangeType::TYPE_CENTER_CHANGE ||
            lc_type == LaneChangeType::TYPE_AVOID_CONES) {
          selector_debug->mutable_debugstring()->Add(
              absl::StrCat("not cross line ,lc type is ",
                           LaneChangeType_Name(lc_type), " forbiden cancel"));
          *begin_lane_change_frame = -1;
        } else if (idx_traj_feature_output_map.at(best_traj_idx)
                       .cross_solid_boundary) {
          selector_debug->mutable_debugstring()->Add(
              "best_traj will cross solid line, continue lc");
          // if lc return cross solid boundary, continue lane change.
          *begin_lane_change_frame = -1;
        } else if (idx_traj_feature_output_map.at(last_selected_idx)
                       .cross_solid_boundary) {
          selector_debug->mutable_debugstring()->Add(
              "last_select_traj will cross solid line, abort lc");
          // if lc return not cross solid boundary and lc cross solid boundary,
          // abort lc as soon as possible.
          *begin_lane_change_frame =
              selector_flags.planner_fast_abort_lane_change_frame;
        } else {
          *begin_lane_change_frame =
              selector_flags.planner_abort_lane_change_frame;
          selector_debug->mutable_debugstring()->Add(absl::StrCat(
              "not cross line ,lc type is ", LaneChangeType_Name(lc_type),
              " ,can cancel after ", *begin_lane_change_frame, " frame"));
        }
      }
      selector_debug->mutable_debugstring()->Add(
          absl::StrCat("begin_lane_change_frame", *begin_lane_change_frame,
                       " successive_count: ", successive_count));
      if (*begin_lane_change_frame >= 0 &&
          successive_count >= *begin_lane_change_frame) {
        selector_debug->mutable_debugstring()->Add(
            " satisfy frame , change stage");
        final_selected_idx = best_traj_idx;
      } else {
        selector_debug->mutable_debugstring()->Add(
            " dont satisfy frame , keep last");
      }
      break;
    }
  }

  return final_selected_idx;
}

std::optional<NaviActionInfoType> FindNextActionInfo(
    const MapEventType& map_event) {
  return map_event.navi_action().empty()
             ? std::nullopt
             : std::make_optional(map_event.navi_action(0));
}

std::optional<NaviActionInfoType> FindNearestNonStraightActionInfo(
    const MapEventType& map_event) {
  if (map_event.navi_action().empty()) {
    return std::nullopt;
  }
  const auto it = std::find_if(
      map_event.navi_action().begin(), map_event.navi_action().end(),
      [](NaviActionInfoType action) {
        return action.main_action() != ns_routing_map::NMA_NONE &&
               action.main_action() != ns_routing_map::NMA_CONTINUE &&
               action.main_action() != ns_routing_map::NMA_STRAIGHT &&
               action.main_action() != ns_routing_map::NMA_UNKNOWN;
      });
  return (it != map_event.navi_action().end()) ? std ::make_optional(*it)
                                               : std::nullopt;
}

SelectorCommonFeature BuildSelectorCommonFeatureV2(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results, SelectorState* selector_state,
    const st::Behavior_FunctionId func_id, int last_selected_idx) {
  SelectorCommonFeature common_feature;
  if (last_selected_idx != -1) {
    common_feature.last_selected_stage =
        results[last_selected_idx].scheduler_output.lane_change_state.stage();
  }
  std::optional<double> max_navi_dist_keep;
  std::optional<int> lc_num_keep;
  std::vector<int> lc_num_vector;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) continue;
    const auto& lane_seq_info =
        results[idx].scheduler_output.drive_passage.lane_seq_info();
    if (lane_seq_info == nullptr) continue;
    if (!results[idx].scheduler_output.borrow_lane && est_status[idx].ok()) {
      lc_num_vector.emplace_back(lane_seq_info->lc_num);
    }
    const auto lc_stage =
        results[idx].scheduler_output.lane_change_state.stage();
    if (lc_stage != LaneChangeStage::LCS_EXECUTING &&
        lc_stage != LaneChangeStage::LCS_PAUSE) {
      lc_num_keep = lc_num_keep.has_value()
                        ? std::min(lc_num_keep.value(), lane_seq_info->lc_num)
                        : lane_seq_info->lc_num;
      max_navi_dist_keep = max_navi_dist_keep.has_value()
                               ? std::max(max_navi_dist_keep.value(),
                                          lane_seq_info->dist_to_navi_end)
                               : lane_seq_info->dist_to_navi_end;

      constexpr double kConsiderInRampDistance = 500.0;
      if (lane_seq_info->is_current && lane_seq_info->dist_to_merge > 0 &&
          lane_seq_info->dist_to_merge < kConsiderInRampDistance) {
        selector_state->time_ego_in_ramp = input.plan_time;
      }
    }
  }
  std::sort(lc_num_vector.begin(), lc_num_vector.end());
  const double cruising_speed_limit = input.cruising_speed_limit.has_value()
                                          ? input.cruising_speed_limit.value()
                                          : DBL_MAX;
  const double target_lane_speed_limit =
      input.target_lane_speed_limit.has_value()
          ? input.target_lane_speed_limit.value()
          : DBL_MAX;
  Log2DDS::LogDataV2("target_lane_speed_limit", target_lane_speed_limit);
  const double speed_limit =
      std::min(Mph2Mps(input.motion_constraints->default_speed_limit()),
               target_lane_speed_limit);
  const auto ego_pos =
      Vec2dFromApolloTrajectoryPointProto(*input.plan_start_point);
  constexpr double kConsiderHighWaySpeedLimit = 23.61;  // m/s.
  bool is_highway_from_speed_limit = false;

  common_feature.func_id = func_id;  // store function_id

  bool has_red_light_stop_s = true;
  std::unordered_map<int, double> dist_to_navi_map;
  std::optional<int> min_lc_num;
  std::optional<double> min_split_angle;

  LcNumInfo lc_num_info = CalcLaneNumInfoButEmergency(input, /*s*/ 0.0);
  LcNumInfo lc_num_info_preview =
      CalcLaneNumInfoButEmergency(input, /*s*/ kPreviewLengthCalRightMost);
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) continue;

    if (results[idx].scheduler_output.borrow_lane) {
      common_feature.is_borrow_case = true;
    }

    LaneFeatureInfo lane_feature;
    lane_feature.target_switched =
        last_selected_idx != -1 && last_selected_idx != idx;
    const auto& result = results[idx];
    const auto& passage = result.scheduler_output.drive_passage;

    lane_feature.valid_lane_num = lc_num_info.valid_lane_num;
    lane_feature.preview_valid_lane_num = lc_num_info_preview.valid_lane_num;
    lane_feature.right_index =
        CalcLaneRightIndex(lc_num_info, passage.lane_path());

    lane_feature.preivew_right_index =
        CalcLaneRightIndex(lc_num_info_preview, passage.lane_path());

    if (idx == 0 && (passage.lane_seq_info() != nullptr)) {
      common_feature.ego_lane_dist_to_navi_end =
          passage.lane_seq_info()->dist_to_navi_end;
      common_feature.min_dist_to_merge =
          std::min(passage.lane_seq_info()->dist_to_merge,
                   common_feature.min_dist_to_merge);
    }
    const auto lane_seq_info = passage.lane_seq_info();
    const auto lc_stage = result.scheduler_output.lane_change_state.stage();
    const bool is_lc = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                       lc_stage == LaneChangeStage::LCS_PAUSE;
    lane_feature.is_keep_lane = !is_lc;
    if (lane_seq_info != nullptr && lc_num_keep.has_value()) {
      lane_feature.is_lc_follow_navi =
          is_lc && lane_seq_info->lc_num < lc_num_keep.value();
      lane_feature.is_lc_against_navi =
          is_lc && lane_seq_info->lc_num > lc_num_keep.value();
    }
    if (lane_seq_info != nullptr && !lc_num_vector.empty()) {
      const int min_lc_num = lc_num_vector.front();
      lane_feature.lc_num = lane_seq_info->lc_num;
      lane_feature.is_only_min_lc_num =
          lane_seq_info->lc_num == min_lc_num &&
          count(lc_num_vector.begin(), lc_num_vector.end(), min_lc_num) == 1;
    }
    // lane_feature.speed_limit = std::min(
    //     input.psmm->QueryLaneSpeedLimitById(start_lane_id), speed_limit);
    lane_feature.speed_limit = speed_limit;
    is_highway_from_speed_limit =
        is_highway_from_speed_limit ||
        lane_feature.speed_limit >= kConsiderHighWaySpeedLimit;
    const bool is_lane_keep =
        result.scheduler_output.lane_change_state.stage() ==
        LaneChangeStage::LCS_NONE;
    const bool is_lc_safety_check_falied =
        est_status[idx].status_code() ==
        PlannerStatusProto::LC_SAFETY_CHECK_FAILED;
    lane_feature.block_obj_ids = FindBlockObjectIds(
        st_traj_mgr_list[idx], result, *input.vehicle_geom,
        common_feature.in_high_way, is_lane_keep, is_lc_safety_check_falied,
        (*input.plan_start_point).v());
    lane_feature.invade_static_obj_map = FindInvadeStaticObjects(
        st_traj_mgr_list[idx], result, lane_feature.block_obj_ids,
        *input.stalled_objects);
    lane_feature.nearest_leader = CalcNearestLeaderFromBlockObjects(
        st_traj_mgr_list[idx], result.scheduler_output.drive_passage,
        lane_feature.block_obj_ids);
    const auto ego_sl_or = passage.QueryFrenetCoordinateAt(ego_pos);

    // Update intersection info.
    if (ego_sl_or.ok()) {
      common_feature.ego_in_tl_controlled_intersection = false;
      if (passage.lane_seq_info()) {
        const double dist_to_junc =
            common_feature.road_horizon_info.dist_to_cross;
        common_feature.ego_in_tl_controlled_intersection =
            dist_to_junc < 10.0 ? true : false;
      }
      // IsInTlControlledIntersection(*input.psmm, passage, ego_sl_or->s);
    }

    has_red_light_stop_s =
        has_red_light_stop_s && result.redlight_lane_id.has_value();

    // Add lane neighbor cones
    const auto& av_frenet_box =
        result.scheduler_output.av_frenet_box_on_drive_passage;
    lane_feature.lane_cones_info = CalcNeighborConesOnLane(
        st_traj_mgr_list[idx], result.scheduler_output.drive_passage,
        av_frenet_box, lane_feature.block_obj_ids);
    common_feature.lane_feature_infos[idx] = std::move(lane_feature);

    if (lane_seq_info == nullptr) {
      common_feature.is_left_turn = false;
      common_feature.is_right_turn = false;
    } else {
      if (lane_seq_info->lane_seq != nullptr) {
        Log2DDS::LogDataV2("selector_common", "Perform lane_seq.");
        common_feature.is_left_turn =
            lane_seq_info->lane_seq->IsTurnLeft(ego_pos.x(), ego_pos.y());
        common_feature.is_right_turn =
            lane_seq_info->lane_seq->IsTurnRight(ego_pos.x(), ego_pos.y());
      }
      if (!min_lc_num.has_value()) {
        min_lc_num = lane_seq_info->lc_num;
      } else {
        min_lc_num = std::min(*min_lc_num, lane_seq_info->lc_num);
      }
      dist_to_navi_map.insert(
          {lane_seq_info->lc_num, lane_seq_info->dist_to_navi_end});
      // get min split angle
      if (lane_seq_info->split_task_state ==
              ad_byd::planning::SplitTasksState::Right_Task ||
          lane_seq_info->split_task_state ==
              ad_byd::planning::SplitTasksState::Left_Task) {
        const double split_angle =
            std::fabs(lane_seq_info->nearest_split_angle);
        min_split_angle = !min_split_angle.has_value()
                              ? split_angle
                              : std::min(min_split_angle.value(), split_angle);
      }
    }
  }
  if (max_navi_dist_keep.has_value()) {
    common_feature.max_navi_dist_keep = max_navi_dist_keep.value();
  }
  if (min_split_angle.has_value()) {
    common_feature.min_split_angle = min_split_angle.value();
  }
  // get navi aciton and distance
  if (input.map_event != nullptr) {
    common_feature.next_navi_action_info = FindNextActionInfo(*input.map_event);
    common_feature.next_non_straight_navi_action_info =
        FindNearestNonStraightActionInfo(*input.map_event);
    common_feature.traffic_jam_info = input.map_event->traffic_info();
  }
  if (func_id == Behavior_FunctionId::Behavior_FunctionId_MAPLESS_NOA ||
      func_id == Behavior_FunctionId::Behavior_FunctionId_CITY_NOA ||
      func_id == Behavior_FunctionId::Behavior_FunctionId_LKA) {
    common_feature.in_high_way = false;
    common_feature.preview_in_high_way = false;
  } else {
    common_feature.in_high_way = true;
    common_feature.preview_in_high_way = true;
  }
  if (func_id == st::Behavior_FunctionId::Behavior_FunctionId_MAPLESS_NOA) {
    common_feature.mapless = true;
  } else {
    common_feature.mapless = false;
  }

  if (!has_red_light_stop_s) {
    if (input.selector_state->last_redlight_stop_time.has_value()) {
      common_feature.time_since_last_red_light = absl::ToDoubleSeconds(
          input.plan_time - *input.selector_state->last_redlight_stop_time);
    } else {
      common_feature.time_since_last_red_light = DBL_MAX;
    }
  } else {
    common_feature.time_since_last_red_light = 0.0;
  }
  if (input.selector_state->last_lc_info.has_lane_change_time()) {
    common_feature.time_since_last_lane_change = absl::ToDoubleSeconds(
        input.plan_time -
        st::FromProto(input.selector_state->last_lc_info.lane_change_time()));
  } else {
    common_feature.time_since_last_lane_change = DBL_MAX;
  }
  if (common_feature.in_high_way &&
      input.selector_state->time_ego_in_ramp.has_value()) {
    common_feature.time_since_ego_leave_ramp = absl::ToDoubleSeconds(
        input.plan_time - input.selector_state->time_ego_in_ramp.value());
  } else {
    common_feature.time_since_ego_leave_ramp = DBL_MAX;
  }
  double dist_to_cross = std::numeric_limits<double>::infinity();
  constexpr double kMaxDistanceToJunctionCheck = 2000.0;

  if (input.selector_state->last_passed_split_info.has_passed_split_time()) {
    common_feature.time_since_last_passed_split = absl::ToDoubleSeconds(
        input.plan_time -
        st::FromProto(
            input.selector_state->last_passed_split_info.passed_split_time()));
  } else {
    common_feature.time_since_last_passed_split = DBL_MAX;
  }

  if (input.selector_state->last_passed_split_info
          .has_is_valid_avoid_split_dis()) {
    common_feature.is_valid_avoid_split_dis =
        input.selector_state->last_passed_split_info.is_valid_avoid_split_dis();
  } else {
    common_feature.is_valid_avoid_split_dis = false;
  }

  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) continue;
    const auto& passage = results[idx].scheduler_output.drive_passage;
    if (passage.lane_seq_info() == nullptr) continue;
    if (passage.lane_seq_info()->dist_to_virtual_lane <
        kMaxDistanceToJunctionCheck) {
      dist_to_cross = passage.lane_seq_info()->dist_to_virtual_lane;
      break;
    }
  }

  constexpr double kSwtichMapDistThresold = 140.0;
  if (dist_to_cross > kMaxDistanceToJunctionCheck &&
      !common_feature.in_high_way &&
      common_feature.next_navi_action_info.has_value() &&
      common_feature.next_navi_action_info->action_dis() >
          kSwtichMapDistThresold) {
    dist_to_cross = common_feature.next_navi_action_info->action_dis();
  }

  common_feature.road_horizon_info.dist_to_fork =
      (input.psmm->map_ptr() != nullptr)
          ? input.psmm->map_ptr()->v2_info().dist_to_subpath
          : DBL_MAX;
  common_feature.road_horizon_info.dist_to_cross = dist_to_cross;
  common_feature.road_horizon_info.dist_to_merge =
      common_feature.road_horizon_info.dist_to_fork;

  common_feature.road_horizon_info.dist_to_tunel =
      input.psmm->map_ptr() != nullptr
          ? input.psmm->map_ptr()->v2_info().dist_to_tunnel
          : DBL_MAX;
  if (!dist_to_navi_map.empty()) {
    std::optional<double> dist_to_navi_end_for_lc;
    for (const auto& [lc_num, dist_to_navi_end] : dist_to_navi_map) {
      if (lc_num == *min_lc_num) {
        continue;
      }
      if (!dist_to_navi_end_for_lc.has_value()) {
        dist_to_navi_end_for_lc = dist_to_navi_end;
      } else {
        dist_to_navi_end_for_lc =
            std::fmax(*dist_to_navi_end_for_lc, dist_to_navi_end);
      }
    }
    if (dist_to_navi_end_for_lc.has_value()) {
      common_feature.road_horizon_info.dist_to_navi_end_for_lc =
          *dist_to_navi_end_for_lc;
    }
  }
  common_feature.plan_time = input.plan_time;
  return common_feature;
}
CostFeatures BuildCostFeatures(
    const SelectorInput& input, const SelectorCommonFeature& common_feature,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results,
    const st::LaneChangeStage& pre_lc_stage) {
  CHECK(input.config->has_cost_config());

  CostFeatures cost_features;
  const auto& cost_config = input.config->cost_config();
  if (cost_config.enable_progress_cost()) {
    cost_features.emplace_back(std::make_unique<TrajProgressCost>(
        &common_feature, input.psmm,
        input.selector_flags->planner_lane_change_style,
        input.prev_lane_path_from_current,
        ProgressStats(common_feature, *input.psmm, *input.plan_start_point,
                      *input.vehicle_geom, est_status, st_traj_mgr_list,
                      results),
        input.driving_style_gear));
  }
  if (cost_config.enable_max_jerk_cost()) {
    cost_features.emplace_back(std::make_unique<TrajMaxJerkCost>(
        &common_feature, *input.motion_constraints,
        (*input.vehicle_geom).width()));
  }
  if (cost_config.enable_lane_change_cost()) {
    cost_features.emplace_back(std::make_unique<TrajLaneChangeCost>(
        &common_feature, input.psmm, input.prev_lane_path_from_current,
        input.prev_traj, *input.plan_start_point,
        input.selector_state->last_lc_info,
        input.selector_state->last_passed_split_info,
        input.selector_flags->planner_enable_lane_change_in_intersection,
        input.selector_state->last_turn_signal,
        input.selector_state->last_turn_signal_reason, pre_lc_stage,
        (*input.vehicle_geom).width()));
  }
  if (cost_config.enable_solid_boundary_cost()) {
    cost_features.emplace_back(std::make_unique<TrajCrossSolidBoundaryCost>(
        &common_feature, input.prev_lane_path_from_current, *input.vehicle_geom,
        *input.psmm, *input.plan_start_point,
        input.selector_flags->planner_enable_cross_solid_boundary));
  }
  if (cost_config.enable_route_look_ahead_cost()) {
    RouteLookAheadStats route_stats(common_feature, *input.psmm,
                                    *input.plan_start_point, *input.avoid_lanes,
                                    *input.stalled_objects, *input.vehicle_geom,
                                    est_status, st_traj_mgr_list, results);
    cost_features.emplace_back(std::make_unique<TrajRouteLookAheadCost>(
        &common_feature, route_stats, *input.plan_start_point, *input.psmm,
        input.selector_flags->planner_is_bus_model));
  }
  if (cost_config.enable_boundary_expansion_cost()) {
    cost_features.emplace_back(std::make_unique<TrajBoundaryExpansionCost>(
        &common_feature, *input.plan_start_point));
  }
  if (cost_config.enable_prepare_intention_cost()) {
    cost_features.emplace_back(
        std::make_unique<PrepareIntentionCost>(&common_feature));
  }
  if (cost_config.enable_defensive_driving_cost()) {
    cost_features.emplace_back(std::make_unique<DefensiveDrivingCost>(
        &common_feature, *input.plan_start_point));
  }
  return cost_features;
}

WeightTable BuildWeightTable(
    const SelectorParamsProto::TrajCostWeights& weight_config,
    const CostFeatures& cost_features) {
  WeightTable weight_table;
  const auto* reflection = weight_config.GetReflection();
  const auto* descriptor = weight_config.GetDescriptor();
  for (const auto& feature : cost_features) {
    auto* feature_desc = descriptor->FindFieldByName(feature->name());
    const auto& feature_conf =
        reflection->GetMessage(weight_config, feature_desc);
    const auto* feature_ref = feature_conf.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> feature_weights_desc;
    feature_ref->ListFields(feature_conf, &feature_weights_desc);
    auto& weight_vec = weight_table[feature->name()];
    weight_vec.resize(feature->size());
    int idx = 0;
    for (const auto* desc : feature_weights_desc) {
      weight_vec[idx++] = feature_ref->GetDouble(feature_conf, desc);
    }
  }
  return weight_table;
}
FeatureCostSum ComputeTrajCostV2(
    const CostFeatures& cost_features, const WeightTable& weights,
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, TrajectoryCost* traj_cost,
    TrajFeatureOutput* traj_feature_output, bool* collide_curb) {
  FeatureCostSum cost_res{0.0, 0.0};
  constexpr double kInvalidCost = 1e5;
  for (const auto& feature : cost_features) {
    std::vector<std::string> extra_info;
    const auto cost_vec_or = feature->ComputeCostV2(
        planner_output, idx, cost_input, &extra_info, traj_feature_output);
    if (!cost_vec_or.ok()) {
      const double feat_cost = kInvalidCost;
      const std::string error_msg =
          absl::StrFormat("%s when calculate %s cost.",
                          cost_vec_or.status().message(), feature->name());
      auto& feat_debug = *traj_cost->add_features();
      feat_debug.set_name(feature->name());
      feat_debug.set_cost(feat_cost);
      feat_debug.add_extra_info(error_msg);
      cost_res.AddWithStaticCost(feat_cost, /*is_common=*/true,
                                 /*is_static_cost=*/false);
      LOG_WARN << error_msg;
    } else {
      const auto& cost_vec = *cost_vec_or;
      const auto weighted_cost = FindOrDie(weights, feature->name()) * cost_vec;
      const double feat_cost = weighted_cost.sum();
      auto& feat_debug = *traj_cost->add_features();
      feat_debug.set_name(feature->name());
      feat_debug.set_cost(feat_cost);
      for (auto& info : extra_info) {
        *feat_debug.add_extra_info() = std::move(info);
      }
      for (int i = 0; i < feature->size(); ++i) {
        feat_debug.add_cost_value_info(absl::StrFormat(
            "%s: raw: %.3f, w: %.2f, res: %.3f", feature->sub_names()[i],
            cost_vec[i], FindOrDie(weights, feature->name())[i],
            weighted_cost[i]));
        if (feature->name() == "cross_solid_boundary" &&
            feature->sub_names()[i] == "curb") {
          if (cost_vec[i] != 0.0) {
            *collide_curb = true;
          }
        }
      }
      cost_res.AddWithStaticCost(feat_cost, feature->is_common(),
                                 feature->is_static_feature());
    }
  }
  traj_cost->set_start_lane_id(
      absl::StrCat(planner_output.scheduler_output.drive_passage.lane_path()
                       .front()
                       .lane_id()));
  traj_cost->set_sum_common(cost_res.cost_common);
  traj_cost->set_sum(cost_res.cost_sum());
  traj_cost->set_lc_stage(
      planner_output.scheduler_output.lane_change_state.stage());

  return cost_res;
}

absl::StatusOr<bool> CalculateScoreFromScoringNet(
    const SelectorInput& input, const SelectorCommonFeature& common_feature,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results,
    SelectorDebugProto* selector_debug,
    absl::flat_hash_map<int, FeatureCostSum>* idx_cost_map) {
  if (!input.selector_flags->planner_enable_selector_scoring_net ||
      !input.config->selector_model_config().enable_selector_model()) {
    return false;
  }

  // Do selector scoring net inference when it is configured and ready.
  // bool ready_to_score = true;
  std::map<int, float> score_map;

  for (int idx = 0; idx < results.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) continue;
    if (results[idx].scheduler_output.is_expert) continue;
    if (results[idx].scheduler_output.is_fallback) continue;

    const auto start_id = results[idx]
                              .scheduler_output.drive_passage.lane_path()
                              .front()
                              .lane_id();

    // Assign selector scoring net output as the only term in the cost function.
    FeatureCostSum cur_cost{0.0, 0.0};
    if (input.config->selector_model_config().selector_score_only() &&
        score_map.find(idx) != score_map.end()) {
      auto score_cost = score_map[idx];
      auto weighted_score_cost =
          input.config->selector_model_config().selector_score_cost() *
          score_cost;
      cur_cost.add(weighted_score_cost, /*is_common=*/true);
      auto* traj_score = selector_debug->add_traj_scores_by_net();
      traj_score->set_start_lane_id(start_id);
      traj_score->set_score(score_map[idx]);
      traj_score->set_score_to_cost(score_cost);
      traj_score->set_weighted_score_to_cost(weighted_score_cost);
    }
    // Assign selector scoring net output as part of the cost function.
    if (!input.config->selector_model_config().selector_score_only() &&
        score_map.find(idx) != score_map.end()) {
      auto score_cost = score_map[idx];
      auto weighted_score_cost =
          input.config->selector_model_config().selector_score_cost() *
          score_cost;
      cur_cost.add(weighted_score_cost, /*is_common=*/true);
      auto* traj_score = selector_debug->add_traj_scores_by_net();
      traj_score->set_start_lane_id(start_id);
      traj_score->set_score(score_map[idx]);
      traj_score->set_score_to_cost(score_cost);
      traj_score->set_weighted_score_to_cost(weighted_score_cost);
    }
    (*idx_cost_map)[idx] = cur_cost;
  }

  return true;
}

LaneChangeReason AnalyzeLaneChangeReason(
    const SelectorDebugProto& selector_debug, const int final_debug_chosen_idx,
    const LaneChangeReason last_lane_change_reason,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map) {
  if (selector_debug.traj_costs(final_debug_chosen_idx).lc_stage() ==
      LaneChangeStage::LCS_NONE) {
    return LaneChangeReason::NO_CHANGE;
  }
  constexpr double kMinLaneChangeReasonThreshold = 3.0;
  // Find another valid trajectory.
  int other_lane_keep_result_idx = -1;
  for (int i = 0; i < selector_debug.traj_costs_size(); ++i) {
    if (i == final_debug_chosen_idx) continue;
    if (selector_debug.traj_costs(i).lc_stage() == LaneChangeStage::LCS_NONE) {
      other_lane_keep_result_idx = i;
      break;
    }
  }

  // No valid lane keep result, so use last reason or default change.
  // Keep lane change state, so use last lane change reason.
  if (other_lane_keep_result_idx == -1 ||
      selector_debug.traj_costs(final_debug_chosen_idx)
              .selector_state()
              .best_traj_time() == 0) {
    return (last_lane_change_reason == LaneChangeReason::NO_CHANGE)
               ? LaneChangeReason::DEFAULT_CHANGE
               : last_lane_change_reason;
  }

  double max_cost_error = kMinLaneChangeReasonThreshold;
  LaneChangeReason lane_change_reason = LaneChangeReason::DEFAULT_CHANGE;
  absl::flat_hash_map<std::string, double> chosen_feature_costs,
      other_feature_costs;
  for (const auto& feature_cost :
       selector_debug.traj_costs(final_debug_chosen_idx).features()) {
    chosen_feature_costs[feature_cost.name()] = feature_cost.cost();
  }
  for (const auto& feature_cost :
       selector_debug.traj_costs(other_lane_keep_result_idx).features()) {
    other_feature_costs[feature_cost.name()] = feature_cost.cost();
  }

  bool lk_has_route_cost = false;
  if (idx_traj_feature_output_map.contains(other_lane_keep_result_idx)) {
    lk_has_route_cost =
        idx_traj_feature_output_map.at(other_lane_keep_result_idx)
            .lane_change_for_route_cost;
  }
  const std::string progress_name = "progress";
  if (chosen_feature_costs.contains(progress_name) &&
      other_feature_costs.contains(progress_name)) {
    const double cost_error = other_feature_costs[progress_name] -
                              chosen_feature_costs[progress_name];
    if (cost_error > max_cost_error) {
      max_cost_error = cost_error;
      lane_change_reason = lk_has_route_cost
                               ? LaneChangeReason::ROUTE_CHANGE
                               : LaneChangeReason::PROGRESS_CHANGE;
    }
  }

  const std::string route_name = "route_look_ahead";
  if (chosen_feature_costs.contains(route_name) &&
      other_feature_costs.contains(route_name)) {
    const double cost_error =
        other_feature_costs[route_name] - chosen_feature_costs[route_name];
    if (cost_error > max_cost_error) {
      max_cost_error = cost_error;
      lane_change_reason = LaneChangeReason::ROUTE_CHANGE;
    }
  }

  return lane_change_reason;
}

int GetSameTargetLaneIdSize(const TargetLaneStateProto& prev,
                            const TargetLaneStateProto& curr) {
  if (prev.lane_ids_size() == 0 || curr.lane_ids_size() == 0) {
    return 0;
  }
  auto start_it = std::find(prev.lane_ids().begin(), prev.lane_ids().end(),
                            curr.lane_ids().at(0));
  if (start_it == prev.lane_ids().end()) {
    return 0;
  }
  int j = 0;
  for (int i = start_it - prev.lane_ids().begin();
       i < prev.lane_ids_size() && j < curr.lane_ids_size(); ++i, ++j) {
    if (prev.lane_ids().at(i) != curr.lane_ids().at(j)) {
      return j;
    }
  }
  if (prev.is_borrow() == curr.is_borrow() &&
      prev.is_fallback() == curr.is_fallback()) {
    return j;
  } else {
    return 0;
  }
}

bool IsSameTargetLane(const TargetLaneStateProto& prev,
                      const TargetLaneStateProto& curr) {
  // if (prev.lane_ids_size() == 0 || curr.lane_ids_size() == 0) return false;
  // auto start_it = std::find(prev.lane_ids().begin(), prev.lane_ids().end(),
  //                           curr.lane_ids().at(0));
  // if (start_it == prev.lane_ids().end()) return false;
  // for (int i = start_it - prev.lane_ids().begin(), j = 0;
  //      i < prev.lane_ids_size() && j < curr.lane_ids_size(); ++i, ++j) {
  //   if (prev.lane_ids().at(i) != curr.lane_ids().at(j)) {
  //     return false;
  //   }
  // }
  // return prev.is_borrow() == curr.is_borrow() &&
  //        prev.is_fallback() == curr.is_fallback();

  // if (prev.lane_ids_size() == 0 || curr.lane_ids_size() == 0) {
  //   return false;
  // }
  // auto start_it = std::find(prev.lane_ids().begin(), prev.lane_ids().end(),
  //                           curr.lane_ids().at(0));
  // if (start_it == prev.lane_ids().end()) {
  //   return false;
  // }
  // for (int i = start_it - prev.lane_ids().begin(), j = 0;
  //      i < prev.lane_ids_size() && j < curr.lane_ids_size(); ++i, ++j) {
  //   if (prev.lane_ids().at(i) != curr.lane_ids().at(j)) {
  //     return false;
  //   }
  // }
  constexpr double kMaxLatTolerance = 0.8;
  std::vector<Vec2d> prev_center_points;
  prev_center_points.reserve(prev.center_points_size());
  for (const auto& center_point : prev.center_points()) {
    prev_center_points.push_back(Vec2d(center_point.x(), center_point.y()));
  }
  const auto prev_ff = BuildKdTreeFrenetFrame(
      prev_center_points, /*down_sample_raw_points = */ true);
  if (!prev_ff.ok()) {
    return false;
  }
  for (const auto& center_point : curr.center_points()) {
    Vec2d center_point_vec(center_point.x(), center_point.y());
    const auto center_point_ff = prev_ff->XYToSL(center_point_vec);
    if (center_point_ff.s > prev_ff->end_s()) {
      break;
    }
    if (center_point_ff.s < kIgnoreBehindLanePathLength) {
      continue;
    }
    if (std::fabs(center_point_ff.l) > kMaxLatTolerance) {
      return false;
    }
  }
  return prev.is_borrow() == curr.is_borrow();
}

bool IsHdMap(const PlannerSemanticMapManager& psmm) {
  if (psmm.map_ptr() == nullptr) {
    return false;
  }
  return (psmm.map_ptr()->type() == ad_byd::planning::MapType::HD_MAP);
}

double GetLaneCheckPreviewDistance(const bool highway_mode,
                                   const bool is_hd_map) {
  if (is_hd_map) {
    return highway_mode ? kPreviewHDLanePathLengthForHighWay
                        : kPreviewHDLanePathLengthForCity;
  } else {
    return highway_mode ? kPreviewLanePathLengthForHighWay
                        : kPreviewLanePathLengthForCity;
  }
}

int FindLastSelectedTrjectory(const SelectorFlags& selector_flags,
                              const PlannerSemanticMapManager& psmm,
                              const std::vector<PlannerStatus>& est_status,
                              const std::vector<EstPlannerOutput>& results,
                              const SelectorState& selector_state,
                              const bool in_high_way,
                              SelectorDebugProto* selector_debug) {
  int last_selected_idx = -1;
  std::vector<std::pair<double, int>> lane_id_num_and_idx_vec;
  lane_id_num_and_idx_vec.reserve(results.size());
  const double preview_distance =
      GetLaneCheckPreviewDistance(in_high_way, IsHdMap(psmm));
  constexpr double kMaxLatDiffTolerance = 0.8;
  constexpr double kLonLengthWeight = 4.0;
  constexpr double kSameIdsWeight = 3.5;
  constexpr double kLatDiffWeight = 2.5;
  constexpr double kBorrowTypeWeight = 8.0;
  const auto& prev = selector_state.selected_target_lane_state;
  std::vector<Vec2d> prev_center_points;
  prev_center_points.reserve(prev.center_points_size());
  for (const auto& center_point : prev.center_points()) {
    prev_center_points.push_back(Vec2d(center_point.x(), center_point.y()));
  }
  const auto prev_ff = BuildKdTreeFrenetFrame(
      prev_center_points, /*down_sample_raw_points = */ true);
  for (int idx = 0; idx < results.size() && prev_ff.ok(); ++idx) {
    if (!est_status[idx].ok()) {
      continue;
    }
    auto cur_target_lane_state = GenerateTargetLaneState(
        psmm, results[idx].scheduler_output, preview_distance);
    double total_lat_diff = 0.0;
    int total_point_nums = 0;
    double max_same_length = 0.0;
    bool lat_diff_check_ok = true;
    for (const auto& center_point : cur_target_lane_state.center_points()) {
      Vec2d center_point_vec(center_point.x(), center_point.y());
      const auto center_point_ff = prev_ff->XYToSL(center_point_vec);
      if (center_point_ff.s > prev_ff->end_s()) {
        break;
      }
      if (center_point_ff.s < kIgnoreBehindLanePathLength) {
        continue;
      }
      if (std::fabs(center_point_ff.l) > kMaxLatDiffTolerance) {
        lat_diff_check_ok = false;
        break;
      }
      total_point_nums++;
      total_lat_diff += std::fabs(center_point_ff.l);
      max_same_length = std::fmax(max_same_length, center_point_ff.s);
    }
    max_same_length =
        std::fmax(0.0, max_same_length - kIgnoreBehindLanePathLength);
    const double average_lat_diff = (total_point_nums < 1)
                                        ? kMaxLatDiffTolerance
                                        : (total_lat_diff / total_point_nums);
    const int same_id_size = GetSameTargetLaneIdSize(
        selector_state.selected_target_lane_state, cur_target_lane_state);
    const double lon_length_prob =
        (max_same_length /
         std::fmax(1e-2, (preview_distance - kIgnoreBehindLanePathLength)));
    const double same_ids_prob =
        (prev.lane_ids_size() < 1)
            ? 0.0
            : (1.0 * same_id_size / prev.lane_ids_size());
    const double lat_diff_prob =
        std::fmax(0.0, (1.0 - average_lat_diff / kMaxLatDiffTolerance));
    double same_borrow_type =
        (prev.is_borrow() == cur_target_lane_state.is_borrow()) ? 1.0 : 0.0;
    const double lane_idx_same_prob =
        (lon_length_prob * kLonLengthWeight + same_ids_prob * kSameIdsWeight +
         lat_diff_prob * kLatDiffWeight + same_borrow_type * kBorrowTypeWeight);
    if (lat_diff_check_ok) {
      lane_id_num_and_idx_vec.push_back({lane_idx_same_prob, idx});
    }
  }
  std::sort(lane_id_num_and_idx_vec.begin(), lane_id_num_and_idx_vec.end(),
            [](const auto& a, const auto& b) { return a < b; });
  if (lane_id_num_and_idx_vec.size() > 1) {
    LOG_INFO << "same_target_lane size: " << lane_id_num_and_idx_vec.size()
             << ", ["
             << absl::StrJoin(lane_id_num_and_idx_vec, ",",
                              absl::PairFormatter("="))
             << "]";
  }
  if (!lane_id_num_and_idx_vec.empty()) {
    last_selected_idx = lane_id_num_and_idx_vec.back().second;
  }
  selector_debug->mutable_debugstring()->Add(absl::StrCat(
      "FindLastSelectedTrjectory last_state: ", last_selected_idx));
  return last_selected_idx;
}

void SendAutoLaneChangeRequestEvent(
    const SelectorLaneChangeRequestProto& selector_lane_change_request) {
  LOG_INFO << "Send auto lane change request event: "
           << LaneChangeReason_Name(
                  selector_lane_change_request.lane_change_reason());
  switch (selector_lane_change_request.lane_change_reason()) {
    case LaneChangeReason::NO_CHANGE:
      break;
    case LaneChangeReason::MANUAL_CHANGE:
      break;
    case LaneChangeReason::CENTER_CHANGE:
      break;
    case LaneChangeReason::PROGRESS_CHANGE:
      break;
    case LaneChangeReason::ROUTE_CHANGE:
      break;
    case LaneChangeReason::DEFAULT_CHANGE:
      break;
    default:
      break;
  }
}

absl::flat_hash_map<int, FeatureCostSum> CalculateTrajectoryCostV2(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results,
    const SelectorState& selector_state,
    absl::flat_hash_map<int, TrajFeatureOutput>* idx_traj_feature_output_map,
    SelectorDebugProto* selector_debug,
    absl::flat_hash_map<int, bool>* task_curb_flag, bool* ego_near_intersection,
    const st::Behavior_FunctionId func_id, int last_selected_idx,
    bool* is_nearby_obs, const st::LaneChangeStage& pre_lc_stage,
    const SelectorCommonFeature& selector_common_feature) {
  absl::flat_hash_map<int, FeatureCostSum> traj_costs;
  absl::flat_hash_map<int, FeatureCostSum> net_idx_cost_map;

  const auto cost_features =
      BuildCostFeatures(input, selector_common_feature, est_status,
                        st_traj_mgr_list, results, pre_lc_stage);
  const auto weights =
      BuildWeightTable(input.config->cost_weights(), cost_features);
  const auto can_use_score_net_or = CalculateScoreFromScoringNet(
      input, selector_common_feature, est_status, st_traj_mgr_list, results,
      selector_debug, &net_idx_cost_map);
  const bool can_use_score_net =
      can_use_score_net_or.ok() && *can_use_score_net_or;

  // if (input.selector_flags->dumping_selector_features) {
  //   const auto status = DumpSelectorEvaluations(
  //       input, selector_common_feature, cost_features, weights, est_status,
  //       st_traj_mgr_list, results, selector_debug);
  //   if (!status.ok()) {
  //     return traj_costs;
  //   }
  // }
  traj_costs.reserve(results.size());
  task_curb_flag->clear();
  task_curb_flag->reserve(results.size());
  idx_traj_feature_output_map->reserve(results.size());
  std::string borrow_task_nudge_id;
  SelectorCostInput cost_input;
  cost_input.map_event = input.map_event;
  if (input.selector_state != nullptr) {
    cost_input.begin_route_change_successive_count =
        input.selector_state->begin_route_change_successive_count;
    cost_input.overtake_lc_pause_successive_count =
        input.selector_state->overtake_lc_pause_successive_count;
    cost_input.is_passed_split_lane_change =
        input.selector_state->is_passed_split_lane_change;
    cost_input.force_route_change_successive_count =
        input.selector_state->force_route_change_successive_count;
  }
  cost_input.begin_route_change_left = input.begin_route_change_left;
  cost_input.is_going_force_route_change_left =
      input.is_going_force_route_change_left;
  cost_input.last_selected_idx = last_selected_idx;
  for (int idx = 0; idx < results.size(); ++idx) {
    const auto nugde_info = results[idx].nudge_object_info;
    const auto is_borrow_task = results[idx].scheduler_output.borrow_lane;
    if (is_borrow_task && nugde_info.has_value()) {
      cost_input.borrow_task_nudge_id = nugde_info.value().id;
      break;
    }
  }

  if (!results.empty() &&
      results[0].scheduler_output.drive_passage.lane_seq_info()) {
    cost_input.is_ego_on_pref_lane =
        results[0].scheduler_output.drive_passage.lane_seq_info()->lc_num == 0;
    cost_input.ego_lane_dist_to_navi_end =
        results[0]
            .scheduler_output.drive_passage.lane_seq_info()
            ->dist_to_navi_end;
  }

  // get curr section and lanes, judge whether is all pref lane
  LcNumInfo lc_num_info = CalcLaneNumInfoButEmergency(input, /*s*/ 0.0);
  cost_input.valid_lane_num = lc_num_info.valid_lane_num;
  cost_input.is_all_lanes_pref =
      (lc_num_info.navigation_lane_num == lc_num_info.valid_lane_num) &&
      lc_num_info.navigation_lane_num > 2;
  const auto& lane_feature_infos = selector_common_feature.lane_feature_infos;
  for (int idx = 0; idx < results.size(); ++idx) {
    if (input.psmm == nullptr) break;
    if (!IsValidBranch(est_status[idx])) continue;
    if (results[idx].scheduler_output.is_expert) continue;
    const auto& passage = results[idx].scheduler_output.drive_passage;
    if (passage.lane_seq_info() == nullptr) continue;
    if (!lane_feature_infos.contains(idx)) continue;
    const auto& lane_feature_info = FindOrDieNoPrint(lane_feature_infos, idx);
    if (lane_feature_info.valid_lane_num ==
            lane_feature_info.preview_valid_lane_num &&
        lane_feature_info.valid_lane_num > 0) {
      int right_index = lane_feature_info.right_index;
      cost_input.task_id_lane_index_map.emplace(idx, right_index);
    }
    cost_input.is_all_lanes_pref =
        cost_input.is_all_lanes_pref && passage.lane_seq_info()->lc_num == 0;
  }

  bool cur_frame_has_begin_route_change = false;
  for (int idx = 0; idx < results.size(); ++idx) {
    auto* traj_cost_debug = selector_debug->add_traj_costs();
    if (!IsValidBranch(est_status[idx])) continue;
    if (results[idx].scheduler_output.is_expert) continue;
    TrajFeatureOutput traj_feature_output;
    cost_input.stm = &(st_traj_mgr_list.at(idx));

    bool collide_curb_flag = false;
    const bool ego_in_intersection =
        selector_common_feature.ego_in_tl_controlled_intersection;
    bool preview_in_intersection = ego_in_intersection;
    if (results[idx].scheduler_output.drive_passage.lane_seq_info() !=
        nullptr) {
      // Section level.
      const double dist_to_intersection =
          selector_common_feature.road_horizon_info.dist_to_cross;
      preview_in_intersection =
          preview_in_intersection || (dist_to_intersection < 10.0);
    }
    cost_input.preview_in_intersection = preview_in_intersection;
    if (preview_in_intersection) {
      *ego_near_intersection = true;
    }
    cost_input.is_nearby_obs = is_nearby_obs;
    // cost_input.has_lane_changed_intersection =
    //     selector_state.has_lane_changed_intersection;
    // cost_input.select_lock_number = select_lock_number;
    traj_feature_output.object_cost_map = FindWithDefault(
        selector_state.object_cost_history.path_object_costs(), idx,
        std::unordered_map<ObjectIdType, FollowCostInfo>());
    traj_feature_output.cur_frame_has_begin_route_change |=
        cur_frame_has_begin_route_change;
    auto cur_cost = ComputeTrajCostV2(cost_features, weights, results[idx], idx,
                                      cost_input, traj_cost_debug,
                                      &traj_feature_output, &collide_curb_flag);

    (*task_curb_flag)[idx] = collide_curb_flag;

    cur_frame_has_begin_route_change |=
        traj_feature_output.has_begin_route_change;
    // construct selector cost feature details here and publish to debug_frame
    std::vector<std::string> selector_feature_debug_details;  // for debugging

    selector_feature_debug_details.emplace_back(absl::StrFormat(
        "Traj idx: %d; Total cost: %f, Cost comp:( %f, %f). static: %.2f", idx,
        cur_cost.cost_sum(), cur_cost.cost_common, cur_cost.cost_same_start,
        cur_cost.static_cost));
    for (std::size_t feature_idx = 0;
         feature_idx < traj_cost_debug->features_size(); feature_idx++) {
      const auto& feature_infos = traj_cost_debug->features(feature_idx);
      // get feature name and cost value
      selector_feature_debug_details.emplace_back(
          absl::StrFormat("Feature: %s; Cost: %f.", feature_infos.name(),
                          feature_infos.cost()));
      // get feature sub cost details
      for (std::size_t i = 0; i < feature_infos.cost_value_info_size(); ++i) {
        selector_feature_debug_details.emplace_back(
            "  " + feature_infos.cost_value_info(i));
      }
      // get feature sub details
      for (std::size_t i = 0; i < feature_infos.extra_info_size(); ++i) {
        selector_feature_debug_details.emplace_back(
            "  " + feature_infos.extra_info(i));
      }
    }

    Log2DDS::LogDataV2(absl::StrFormat("SelectorCostInfo_Traj_%d", idx),
                       // for (std::size_t i = 0; i <
                       // selector_feature_debug_details.size(); ++i) {
                       //   DLOG(INFO) << selector_feature_debug_details[i];
                       // }
                       selector_feature_debug_details);
    // for (auto debug_detail : selector_feature_debug_details) {
    //   LOG_ERROR << "debug_detail: " << debug_detail << " \n";
    // }
    // Use result of scoring selector_net.
    if (can_use_score_net && net_idx_cost_map.contains(idx)) {
      if (input.config->selector_model_config().selector_score_only()) {
        cur_cost = net_idx_cost_map.at(idx);
        traj_cost_debug->set_sum_common(cur_cost.cost_common);
        traj_cost_debug->set_sum(cur_cost.cost_sum());
      } else {
        cur_cost.AddWithStaticCost(net_idx_cost_map.at(idx).cost_common,
                                   /*is_common=*/true,
                                   /*is_static_cost=*/false);
        traj_cost_debug->set_sum_common(cur_cost.cost_common);
        traj_cost_debug->set_sum(cur_cost.cost_sum());
        auto& feat_debug = *traj_cost_debug->add_features();
        feat_debug.set_name("ssnet");
        feat_debug.set_cost(net_idx_cost_map.at(idx).cost_common);
      }
    }
    traj_costs[idx] = cur_cost;
    idx_traj_feature_output_map->emplace(idx, std::move(traj_feature_output));
  }
  return traj_costs;
}

bool IsDeviateNaviInterfaceScene(const DeviateNaviInput& deviate_navi_input,
                                 const std::vector<PlannerStatus>& est_status,
                                 const std::vector<EstPlannerOutput>& results) {
  if (deviate_navi_input.func_id != Behavior_FunctionId_CITY_NOA) {
    Log2DDS::LogDataV2(
        "deviate_navi_debug",
        absl::StrCat("function check fail1:", deviate_navi_input.func_id));
    return false;
  }
  bool is_lanechange =
      deviate_navi_input.pre_lc_stage == st::LaneChangeStage::LCS_EXECUTING ||
      deviate_navi_input.pre_lc_stage == st::LaneChangeStage::LCS_PAUSE;
  LaneSequencePtr cur_seq =
      is_lanechange ? deviate_navi_input.pre_target_laneseq_before_lc
                    : deviate_navi_input.pre_target_laneseq;
  const auto ego_pos =
      Vec2dFromApolloTrajectoryPointProto(*deviate_navi_input.plan_start_point);
  const auto ego_theta =
      (*deviate_navi_input.plan_start_point).path_point().theta();
  const auto ego_length = (*deviate_navi_input.vehicle_geom).length();
  const auto ego_width = (*deviate_navi_input.vehicle_geom).width();
  if (!cur_seq) return false;
  double lane_width = 3.5;
  const auto& nearest_lane = cur_seq->GetNearestLane(ego_pos);
  if (nearest_lane) {
    lane_width = nearest_lane->GetWidthAtPoint(ego_pos.x(), ego_pos.y());
  }
  const double half_lane_width = lane_width * 0.5;

  // ego heading check
  double s_offset = 0.0;
  cur_seq->GetProjectionDistance(ego_pos, &s_offset);
  const auto point_0 = cur_seq->GetPointAtS(s_offset);
  const auto point_1 = cur_seq->GetPointAtS(s_offset + 2.0);
  double theta_temp =
      ad_byd::planning::math::NormalizeAngle((point_1 - point_0).Angle());
  double theta_diff =
      std::fabs(ad_byd::planning::math::NormalizeAngle(theta_temp - ego_theta));
  if (theta_diff > 0.1) {
    Log2DDS::LogDataV2("deviate_navi_debug",
                       absl::StrCat("heading check fail:", theta_diff));
    return false;
  }

  // ego pos check
  std::vector<ad_byd::planning::Point2d> points;
  cur_seq->SamplePoints(0.0, &points);
  ad_byd::planning::Path path(points);
  st::Box2d ego_box(ego_pos, ego_theta, ego_length, ego_width);
  ad_byd::planning::SLBoundary sl_bound;
  path.XYToSL(ego_box, &sl_bound);
  if (((sl_bound.l_min + half_lane_width) * (sl_bound.l_max + half_lane_width) <
       0.0) ||
      ((sl_bound.l_min - half_lane_width) * (sl_bound.l_max - half_lane_width) <
       0.0)) {
    Log2DDS::LogDataV2("deviate_navi_debug", "boundary check fail");
    return false;
  }
  Log2DDS::LogDataV2("deviate_navi_debug", "IsDeviateNaviInterfaceScene");
  return true;
}

bool IsDeviateNaviScene(const DeviateNaviInput& deviate_navi_input,
                        const std::vector<PlannerStatus>& est_status,
                        const std::vector<EstPlannerOutput>& results) {
  if (deviate_navi_input.func_id != Behavior_FunctionId_CITY_NOA &&
      deviate_navi_input.func_id != Behavior_FunctionId_MAPLESS_NOA) {
    Log2DDS::LogDataV2(
        "deviate_navi_debug",
        absl::StrCat("function check fail2:", deviate_navi_input.func_id));
    return false;
  }
  bool is_lanechange =
      deviate_navi_input.pre_lc_stage == st::LaneChangeStage::LCS_EXECUTING ||
      deviate_navi_input.pre_lc_stage == st::LaneChangeStage::LCS_PAUSE;
  const auto ego_pos =
      Vec2dFromApolloTrajectoryPointProto(*deviate_navi_input.plan_start_point);

  // dist to navi end check
  for (const auto& result : results) {
    const auto laneseq_info =
        result.scheduler_output.drive_passage.lane_seq_info();
    if (laneseq_info && laneseq_info->is_current && laneseq_info->lane_seq) {
      const auto lat_offset =
          result.scheduler_output.drive_passage.QueryFrenetLatOffsetAt(ego_pos);
      if (lat_offset.ok() && std::fabs(lat_offset.value()) > 2.1) {
        Log2DDS::LogDataV2(
            "deviate_navi_debug",
            absl::StrCat("offset check fail:", std::fabs(lat_offset.value())));
        break;
      }
      bool has_non_navi_lane = false;
      for (const auto& lane : laneseq_info->lane_seq->lanes()) {
        if (lane && !lane->is_navigation()) {
          has_non_navi_lane = true;
          break;
        }
      }
      if (!has_non_navi_lane &&
          deviate_navi_input.func_id == Behavior_FunctionId_CITY_NOA) {
        Log2DDS::LogDataV2("deviate_navi_debug", "navi check fail: all navi");
        break;
      }
      double dist_junc = laneseq_info->dist_to_virtual_lane;
      if (dist_junc > 50.0) {
        Log2DDS::LogDataV2(
            "deviate_navi_debug",
            FormatNumericString("failed, dist_to_junction: ", dist_junc));
        break;
      }
      double dist_left = laneseq_info->dist_to_navi_end;
      if ((dist_left < 35.0 && !is_lanechange) ||
          (dist_left < 20.0 && is_lanechange)) {
        Log2DDS::LogDataV2("deviate_navi_debug", "IsDeviateNaviScene");
        return true;
      }
      Log2DDS::LogDataV2("deviate_navi_debug",
                         absl::StrCat("dist left check fail:", dist_left));
      break;
    }
  }
  return false;
}

int FindBestLaneKeepTrajectory(
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map) {
  int best_lane_keep_idx = -1;
  double best_lane_keep_cost = std::numeric_limits<double>::max();
  for (const auto& [idx, cost] : idx_traj_cost_map) {
    if (IsPerformLaneChange(
            results[idx].scheduler_output.lane_change_state.stage()) ||
        results[idx].scheduler_output.borrow_lane) {
      continue;
    }
    if (cost.cost_common < best_lane_keep_cost) {
      best_lane_keep_idx = idx;
      best_lane_keep_cost = cost.cost_common;
    }
  }
  return best_lane_keep_idx;
}

int ChooseLaneKeepTrajDirectly(
    bool planner_is_l4_mode, absl::Time plan_time, int last_selected_idx,
    const SelectorState& selector_state, const SelectorFlags& selector_flags,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const std::vector<EstPlannerOutput>& results, const bool icc_lc_enable,
    std::string* force_lane_keep_info) {
  int best_lane_keep_idx =
      FindBestLaneKeepTrajectory(results, idx_traj_cost_map);
  if (best_lane_keep_idx == -1) {
    *force_lane_keep_info += "find Lk task fail, no lane keep task here!";
    return -1;
  }
  if (icc_lc_enable) {
    return -1;
  }
  // For the first frame in noa, we need to choose lane keep trajectory.
  const auto activate_selector_time =
      selector_state.activate_selector_time.has_value()
          ? *selector_state.activate_selector_time
          : absl::InfiniteFuture();
  const double time_after_activate_selector =
      absl::ToDoubleSeconds(plan_time - activate_selector_time);
  VLOG(3) << "planner_is_l4_mode: " << planner_is_l4_mode
          << ", time_after_activate_selector: " << time_after_activate_selector;
  if (!planner_is_l4_mode &&
      time_after_activate_selector <
          selector_flags.planner_allow_lc_time_after_activate_selector) {
    *force_lane_keep_info = absl::StrFormat("active_past_time:  %.2f  ",
                                            time_after_activate_selector);
    return best_lane_keep_idx;
  }

  // If there is a lane change request waitting, we need to choose lane keep
  // trajectory.
  // if (selector_state.selector_lane_change_request.state() ==
  //     RequestState::WAITING_RESPONSE) {
  //   *force_lane_keep_info =
  //       "Choose lane keep trajectory directly in noa, because "
  //       "there is a lane change request waitting.";
  //   return best_lane_keep_idx;
  // }

  // In noa mode and no need to force route change
  // when lane change was just given up, we need to choose lane keep.
  const auto give_up_lane_change_time =
      selector_state.give_up_lane_change_time.has_value()
          ? *selector_state.give_up_lane_change_time
          : absl::InfinitePast();
  const double time_after_give_up_lane_change =
      absl::ToDoubleSeconds(plan_time - give_up_lane_change_time);
  const bool need_to_force_route_change =
      idx_traj_feature_output_map.at(best_lane_keep_idx).has_obvious_route_cost;
  if (!planner_is_l4_mode && !need_to_force_route_change &&
      time_after_give_up_lane_change <
          selector_flags.planner_allow_lc_time_after_give_up_lc) {
    if (last_selected_idx != -1 &&
        results[last_selected_idx].scheduler_output.lane_change_state.stage() ==
            LaneChangeStage::LCS_EXECUTING) {
      DLOG(INFO) << "We can continue lane change";
      *force_lane_keep_info +=
          " last select is LC , We can continue lane change";
    } else {
      *force_lane_keep_info =
          "Choose lane keep trajectory directly in noa, because "
          "lane change was just given up.";
      return best_lane_keep_idx;
    }
  }

  // In noa mode disable opposite lane change after paddle lane change.
  const auto paddle_lane_change_time =
      selector_state.last_lc_info.has_lane_change_time() &&
              selector_state.last_lc_info.lane_change_type() ==
                  LaneChangeType::TYPE_PADDLE_CHANGE
          ? FromProto(selector_state.last_lc_info.lane_change_time())
          : absl::InfinitePast();
  const auto time_after_paddle_lane_change =
      absl::ToDoubleSeconds(plan_time - paddle_lane_change_time);
  if (!planner_is_l4_mode &&
      time_after_paddle_lane_change <
          selector_flags.planner_allow_opposite_lc_time_after_paddle_lc) {
    bool is_opposite_lc = false;
    for (const auto& [_, traj_feature_output] : idx_traj_feature_output_map) {
      if (traj_feature_output.is_perform_lane_change &&
          traj_feature_output.lane_change_left !=
              selector_state.last_lc_info.lc_left()) {
        is_opposite_lc = true;
        break;
      }
    }
    if (is_opposite_lc) {
      *force_lane_keep_info = absl::StrFormat(
          "Choose lane keep trajectory directly in noa, because "
          "opposite paddle lane change was just finished %.2f seconds ago.",
          time_after_paddle_lane_change);
      return best_lane_keep_idx;
    }
  }
  return -1;
}
std::optional<int> selectNonDiagonalLane(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const PlannerSemanticMapManager& psmm, int last_selected_idx,
    const bool& need_non_diagonal, SelectorState* selector_state,
    SelectorDebugProto* selector_debug) {
  selector_debug->mutable_debugstring()->Add(
      absl::StrCat("need_non_diagonal:  ", need_non_diagonal));
  if (!need_non_diagonal) {
    selector_debug->mutable_debugstring()->Add("not need_non_diagonal ,clear");
    selector_state->junction_out_lane = nullptr;
    return std::nullopt;
  }
  absl::flat_hash_map<int, int> match_map;
  absl::flat_hash_map<int, ad_byd::planning::LaneConstPtr> outlane_map;

  bool dont_use_likehood(false);
  for (int i = 0; i < results.size(); i++) {
    if (!IsValidBranch(est_status[i])) continue;
    if (IsPerformLaneChange(
            results[i].scheduler_output.lane_change_state.stage())) {
      selector_debug->mutable_debugstring()->Add(absl::StrCat(
          "task ", i,
          " is lc ongoing,donot consider in selectNonDiagonalLane"));
      continue;
    }
    selector_debug->mutable_debugstring()->Add(
        FormatNumericString("dist_to_junction: ",
                            results[i]
                                .scheduler_output.drive_passage.lane_seq_info()
                                ->dist_to_virtual_lane));
    if (selector_state->junction_out_lane) {
      auto out_id = selector_state->junction_out_lane->id();
      selector_debug->mutable_debugstring()->Add(
          absl::StrCat("in junction,locked id is ", out_id));
      for (const auto& lane :
           results[i]
               .scheduler_output.drive_passage.lane_seq_info()
               ->lane_seq->lanes()) {
        if (lane->id() == out_id) {
          selector_debug->mutable_debugstring()->Add(absl::StrCat(
              "has adapted lane in intersection,force choose last result ", i));
          return i;
        }
      }
    }
    const auto& lanes = results.at(i)
                            .scheduler_output.drive_passage.lane_seq_info()
                            ->lane_seq->lanes();
    const auto& lane_seq =
        results.at(i).scheduler_output.drive_passage.lane_seq_info()->lane_seq;
    const auto& drive_passage = results.at(i).scheduler_output.drive_passage;
    const auto& map = psmm.map_ptr();
    int in_num = -1;
    int out_num = -1;
    for (const auto& lane : lanes) {
      bool pre_junction = false;
      auto pre_lane = lane_seq->FindPreLaneOnLaneseq(*lane);
      ad_byd::planning::LaneConstPtr outlane;
      if (pre_lane.has_value()) {
        if ((lane->type() == LaneType::LANE_VIRTUAL_JUNCTION &&
             pre_lane.value()->type() != LaneType::LANE_VIRTUAL_JUNCTION) ||
            (lane->type() != LaneType::LANE_VIRTUAL_JUNCTION &&
             pre_lane.value()->type() == LaneType::LANE_VIRTUAL_JUNCTION)) {
          auto left_id = pre_lane.value()->left_lane_id();

          ad_byd::planning::LaneConstPtr lane_detect;
          if (lane->type() == LaneType::LANE_VIRTUAL_JUNCTION &&
              pre_lane.value()->type() != LaneType::LANE_VIRTUAL_JUNCTION) {
            lane_detect = pre_lane.value();
          } else {
            lane_detect = lane;
            outlane = lane_detect;
          }
          uint64_t left_id_detect = lane_detect->id();
          int left_num = 0;
          while (lane_detect) {
            auto left_lane = map->GetLeftLane(lane_detect);
            auto lane_check = lane_detect;
            lane_detect = left_lane;
            bool is_left_turn_all(true);
            for (const auto& next_check_id : lane_check->next_lane_ids()) {
              auto check_lane = map->GetLaneById(next_check_id);
              if (check_lane) {
                if ((check_lane->turn_type() == TurnType::LEFT_TURN) ||
                    (check_lane->type() == LaneType::LANE_LEFT_WAIT) ||
                    (!check_lane->is_navigation())) {
                  is_left_turn_all = true && is_left_turn_all;
                } else {
                  is_left_turn_all = false && is_left_turn_all;
                }
              }
            }
            if ((lane->type() == LaneType::LANE_VIRTUAL_JUNCTION &&
                 pre_lane.value()->type() != LaneType::LANE_VIRTUAL_JUNCTION) &&
                ((lane_check->next_lane_ids().empty()) || is_left_turn_all)) {
              continue;
            }
            left_num++;
          }
          if (lane->type() == LaneType::LANE_VIRTUAL_JUNCTION &&
              pre_lane.value()->type() != LaneType::LANE_VIRTUAL_JUNCTION) {
            in_num = left_num;
          } else {
            out_num = left_num;
          }
        }
        if (in_num != -1 && out_num != -1) {
          match_map[i] = std::abs(in_num - out_num);
          outlane_map[i] = outlane;
        } else {
          match_map[i] = INT_MAX;
          outlane_map[i] = nullptr;
        }

        auto dist_to_junction =
            results[i]
                .scheduler_output.drive_passage.lane_seq_info()
                ->dist_to_virtual_lane;
        selector_debug->mutable_debugstring()->Add(
            absl::StrCat("out_num: ", out_num, " in_num: ", in_num));

        if ((out_num == in_num) && out_num != -1 && dist_to_junction < 10 &&
            dist_to_junction > 1.0) {
          selector_state->has_selected_intersection = true;
          selector_state->junction_out_lane = outlane;
          selector_debug->mutable_debugstring()->Add(FormatNumericString(
              "dist_to_junction: ",
              results[i]
                  .scheduler_output.drive_passage.lane_seq_info()
                  ->dist_to_virtual_lane));
          selector_debug->mutable_debugstring()->Add(
              absl::StrCat("task ", i,
                           " is driectly chosen because of "
                           "selectNonDiagonalLane,lock the chosen"));
          return i;
        }
      }
    }
  }
  int min_dislike = INT_MAX;
  int most_like_index = -1;
  for (const auto& it : match_map) {
    if (it.second == INT_MAX) return std::nullopt;
    if (it.second < min_dislike) {
      min_dislike = it.second;
      most_like_index = it.first;
    }
  }
  selector_debug->mutable_debugstring()->Add(
      absl::StrCat("task ", most_like_index, " is most like"));
  selector_state->has_selected_intersection = true;
  selector_state->junction_out_lane = outlane_map[most_like_index];
  return most_like_index;
}

int FindBestTrajectory(
    absl::Time plan_time, int last_selected_idx,
    const PlannerSemanticMapManager& psmm,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results,
    const SelectorFlags& selector_flags,
    const VehicleGeometryParamsProto& vehicle_geom,
    const ApolloTrajectoryPointProto& plan_start_point,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorDebugProto* selector_debug, SelectorState* selector_state,
    SelectorOutput* selector_output, const bool highway_mode,
    const bool ego_near_intersection, const bool icc_lc_enable) {
  int best_traj_idx = ChooseLaneKeepTrajDirectly(
      selector_flags.planner_is_l4_mode, plan_time, last_selected_idx,
      *selector_state, selector_flags, idx_traj_cost_map,
      idx_traj_feature_output_map, results, icc_lc_enable,
      &selector_output->force_lane_keep_info);
  bool lane_change_for_intersection_obs = false;
  for (const auto& map : idx_traj_feature_output_map) {
    // trick: has obstacle and have no stalled object , use topo
    if (map.second.lane_change_for_intersection_obs &&
        !map.second.has_obvious_stalled_object) {
      lane_change_for_intersection_obs = true;
    }
  }
  bool need_non_diagonal =
      ego_near_intersection && lane_change_for_intersection_obs;
  selector_debug->mutable_debugstring()->Add(absl::StrCat(
      "ego_near_intersection: ", ego_near_intersection,
      " lane_change_for_intersection_obs: ", lane_change_for_intersection_obs));
  // auto non_diagonal_idx =
  //     selectNonDiagonalLane(results, est_status, psmm, last_selected_idx,
  //                           need_non_diagonal, selector_state,
  //                           selector_debug);
  // if (non_diagonal_idx.has_value()) {
  //   best_traj_idx = non_diagonal_idx.value();
  // }
  int final_selected_idx = -1;
  int successive_count = 1;
  int begin_lane_change_frame = -1;
  const double preview_distance =
      GetLaneCheckPreviewDistance(highway_mode, IsHdMap(psmm));
  if (best_traj_idx != -1) {
    // Force choose lane keep.
    selector_output->force_lane_keep_info += "force LK here!";
    final_selected_idx = best_traj_idx;
    auto best_target_lane_state = GenerateTargetLaneState(
        psmm, results[best_traj_idx].scheduler_output, preview_distance);
    selector_state->best_target_lane_state = best_target_lane_state;
    selector_state->selected_target_lane_state =
        std::move(best_target_lane_state);
    selector_state->lc_prepare_stage_lane_path = std::nullopt;
  } else {
    selector_output->force_lane_keep_info += "No need force LK";
    best_traj_idx = ChooseBestCostTraj(est_status, results, idx_traj_cost_map);
    selector_debug->mutable_debugstring()->Add(
        absl::StrCat("best_traj_idx by_cost is ", best_traj_idx));
    auto best_target_lane_state =
        GenerateTargetLaneState(psmm, results[best_traj_idx].scheduler_output,
                                kPreviewDefaultLanePathLength);
    if (IsSameTargetLane(selector_state->best_target_lane_state,
                         best_target_lane_state)) {
      successive_count =
          selector_state->best_target_lane_state.successive_count() + 1;
      selector_debug->mutable_debugstring()->Add(absl::StrCat(
          "cost_best_traj is same as last selector result, success count is ",
          successive_count));
    }
    best_target_lane_state.set_successive_count(successive_count);
    selector_state->best_target_lane_state = std::move(best_target_lane_state);
    final_selected_idx = best_traj_idx;
    selector_state->lc_prepare_stage_lane_path = std::nullopt;
    selector_debug->mutable_debugstring()->Add(
        absl::StrCat("last_selected_idx: ", last_selected_idx));
    const double dist_to_tunnel = psmm.map_ptr() != nullptr
                                      ? psmm.map_ptr()->v2_info().dist_to_tunnel
                                      : DBL_MAX;
    const ad_byd::planning::MapType map_type =
        psmm.map_ptr() != nullptr ? psmm.map_ptr()->type()
                                  : ad_byd::planning::MapType::UNKNOWN_MAP;
    if (last_selected_idx != -1) {
      final_selected_idx = TransitLaneChangeStage(
          last_selected_idx, best_traj_idx, successive_count, results,
          selector_flags, vehicle_geom, plan_start_point,
          idx_traj_feature_output_map, dist_to_tunnel, ego_near_intersection,
          highway_mode, selector_state, selector_output,
          &begin_lane_change_frame, selector_debug, map_type);
    }
    if (!ego_near_intersection) {
      selector_debug->mutable_debugstring()->Add(
          "not ego_near_intersection,clear lock");
      selector_state->has_selected_intersection = false;
    }
    selector_state->selected_target_lane_state = GenerateTargetLaneState(
        psmm, results[final_selected_idx].scheduler_output, preview_distance);
    selector_debug->mutable_debugstring()->Add(
        absl::StrCat("final_selected_idx: ", final_selected_idx));
  }
  selector_debug->mutable_debugstring()->Add(
      absl::StrCat("has_selected_intersection: ",
                   selector_state->has_selected_intersection));
  selector_output->best_traj_idx = best_traj_idx;
  if (best_traj_idx == -1) {
    if (final_selected_idx != -1 &&
        results[final_selected_idx]
                .scheduler_output.drive_passage.lane_seq_info() != nullptr) {
      selector_state->intention_dir =
          results[final_selected_idx]
              .scheduler_output.drive_passage.lane_seq_info()
              ->lc_dir;
    }
  } else {
    if (results[best_traj_idx].scheduler_output.drive_passage.lane_seq_info() !=
        nullptr) {
      selector_state->intention_dir =
          results[best_traj_idx]
              .scheduler_output.drive_passage.lane_seq_info()
              ->lc_dir;
    }
  }

  // Update selector debug.
  for (int idx = 0; idx < results.size(); ++idx) {
    auto& selector_state_debug =
        *(selector_debug->mutable_traj_costs(idx)->mutable_selector_state());
    selector_state_debug.set_best_traj_time(
        idx == best_traj_idx ? successive_count : 0);
    selector_state_debug.set_best_threshold(begin_lane_change_frame);
    selector_state_debug.set_is_last_traj(idx == last_selected_idx);
    selector_state_debug.set_is_final_traj(idx == final_selected_idx);
  }
  return final_selected_idx;
}
}  // namespace
namespace internal {
LaneChangeType AnalyzeLaneChangeType(
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const bool is_paddle_lane_change, const int last_selected_idx,
    const int final_chosen_idx, const LaneChangeType last_lane_change_type) {
  // Not in lane change.
  if (final_chosen_idx < 0 ||
      !idx_traj_feature_output_map.contains(final_chosen_idx) ||
      !idx_traj_feature_output_map.at(final_chosen_idx)
           .is_perform_lane_change) {
    return LaneChangeType::TYPE_NO_CHANGE;
  }
  // If the last lane change reason is not NO_CHANGE, return it directly.
  if (last_lane_change_type != LaneChangeType::TYPE_NO_CHANGE) {
    return last_lane_change_type;
  }
  if (is_paddle_lane_change) {
    return LaneChangeType::TYPE_PADDLE_CHANGE;
  }
  // Find lane keep idx.
  int lane_keep_idx =
      FindLaneKeepIdx(idx_traj_feature_output_map, last_selected_idx);

  if (lane_keep_idx < 0) {
    return LaneChangeType::TYPE_DEFAULT_CHANGE;
  }

  const auto& traj_feature_output =
      idx_traj_feature_output_map.at(lane_keep_idx);

  // Emergency lane change
  if (traj_feature_output.lane_change_for_moving_obj &&
      traj_feature_output.lane_change_for_emergency) {
    return LaneChangeType::TYPE_EMERGENCY_CHANGE;
  }

  // Stalled lane change
  if (traj_feature_output.lane_change_for_stalled_vehicle) {
    return LaneChangeType::TYPE_STALLED_VEHICLE_CHANGE;
  }
  // Obstacle lane change
  if (traj_feature_output.lane_change_for_stationary_obj) {
    return LaneChangeType::TYPE_OBSTACLE_CHANGE;
  }
  if (traj_feature_output.lane_change_for_merge_lane) {
    return LaneChangeType::TYPE_MERGE_CHANGE;
  }
  // Defensive lane change
  if (traj_feature_output.lane_change_for_avoid_cones) {
    return LaneChangeType::TYPE_AVOID_CONES;
  }
  // CutOff lange change
  if (traj_feature_output.lane_change_for_length_cutoff) {
    return LaneChangeType::TYPE_CURB_CUTOFF_CHANGE;
  }
  if (traj_feature_output.lane_change_for_avoid_bus_lane) {
    return LaneChangeType::TYPE_AVOID_BUS_LANE;
  }
  if (traj_feature_output.lane_change_for_avoid_merge_area) {
    return LaneChangeType::TYPE_AVOID_MERGE_AREA;
  }
  if (traj_feature_output.lane_change_for_right_most_lane) {
    return LaneChangeType::TYPE_CENTER_CHANGE;
  }
  if (traj_feature_output.lane_change_for_intersection_obs) {
    return LaneChangeType::TYPE_INTERSECTION_OBS;
  }
  // Route lane change.
  if (traj_feature_output.lane_change_for_route_cost) {
    return LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE;
  }
  if (traj_feature_output.lane_change_for_navi_cost) {
    return LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE;
  }
  if (traj_feature_output.lane_change_for_road_speed_limit) {
    return LaneChangeType::TYPE_ROAD_SPEED_LIMIT_CHANGE;
  }
  if (traj_feature_output.lane_change_for_moving_obj) {
    return LaneChangeType::TYPE_OVERTAKE_CHANGE;
  }
  return LaneChangeType::TYPE_DEFAULT_CHANGE;
}

LaneChangeReason ReAnalyzeLaneChangeReason(LaneChangeReason lc_reason,
                                           LaneChangeType lc_type) {
  if ((lc_type == LaneChangeType::TYPE_OBSTACLE_CHANGE ||
       lc_type == LaneChangeType::TYPE_STALLED_VEHICLE_CHANGE) &&
      lc_reason != LaneChangeReason::NO_CHANGE) {
    return LaneChangeReason::AVOID_STATIC_OBJECT;
  } else if (lc_type == LaneChangeType::TYPE_MERGE_CHANGE &&
             lc_reason != LaneChangeReason::NO_CHANGE) {
    return LaneChangeReason::MERGE_CHANGE;
  } else if (lc_type == LaneChangeType::TYPE_AVOID_BUS_LANE &&
             lc_reason != LaneChangeReason::NO_CHANGE) {
    return LaneChangeReason::AVOID_LANE_BUS;
  } else if (lc_type == LaneChangeType::TYPE_AVOID_MERGE_AREA &&
             lc_reason != LaneChangeReason::NO_CHANGE) {
    return LaneChangeReason::AVOID_MERGE_AREA;
  } else if ((lc_type == LaneChangeType::TYPE_CURB_CUTOFF_CHANGE ||
              lc_type == LaneChangeType::TYPE_EMERGENCY_CHANGE ||
              lc_type == LaneChangeType::TYPE_AVOID_CONES) &&
             lc_reason != LaneChangeReason::NO_CHANGE) {
    return LaneChangeReason::DEFAULT_CHANGE;
  } else if (lc_type == LaneChangeType::TYPE_CENTER_CHANGE &&
             lc_reason != LaneChangeReason::NO_CHANGE) {
    return LaneChangeReason::CENTER_CHANGE;
  }

  return lc_reason;
}

}  // namespace internal

absl::StatusOr<SelectorOutput> SelectTrajectoryV2(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results,
    const DeviateNaviInput& deviate_navi_input,
    SelectorDebugProto* selector_debug, SelectorState* selector_state) {
  SCOPED_TRACE("SelectTrajectoryV2");
  const bool highway_mode = deviate_navi_input.func_id ==
                            st::Behavior_FunctionId::Behavior_FunctionId_HW_NOA;
  const st::LaneChangeStage pre_lc_stage = deviate_navi_input.pre_lc_stage;
  // ("SelectTrajectory");
  if (input.selector_state != nullptr) {
    *selector_state = *input.selector_state;
  }
  selector_state->lane_change_reason = LaneChangeReason::NO_CHANGE;
  double passable_area = 0.0;
  double lane_width = 0.0;
  const double ego_width = input.vehicle_geom->width();
  const auto ego_pos =
      Vec2dFromApolloTrajectoryPointProto(*input.plan_start_point);
  if (results.empty()) return absl::NotFoundError("Input results empty!");
  if (est_status.empty()) return absl::NotFoundError("Input est status empty");

  // selector_state->lc_unable_reason = LcFeasibility::FEASIBILITY_OK;
  // if (est_status[0].ok()) {
  //   selector_state->lane_change_prepare_state =
  //       LaneChangePrepareState::Lane_Keeping;
  //   Log2DDS::LogDataV2(
  //       "prepare_msg",
  //       absl::StrCat("keeping", selector_state->lane_change_prepare_state));
  // }

  // 1.Calculate traj cost for each lane.
  absl::flat_hash_map<int, bool> tasks_curb_flags;
  absl::flat_hash_map<int, TrajFeatureOutput> idx_traj_feature_output_map;
  SelectorOutput selector_output;
  const int last_selected_idx = FindLastSelectedTrjectory(
      *input.selector_flags, *input.psmm, est_status, results, *selector_state,
      highway_mode, selector_debug);
  const auto prefilter_est_status = PreFilterEstResults(
      input.plan_time, *input.psmm, *input.selector_flags,
      *input.stalled_objects, est_status, results, /*selector_common_feature,*/
      last_selected_idx, selector_debug, selector_state);
  bool ego_near_intersection = false;
  bool is_nearby_obs = false;
  const auto selector_common_feature = BuildSelectorCommonFeatureV2(
      input, est_status, st_traj_mgr_list, results, selector_state,
      deviate_navi_input.func_id, last_selected_idx);
  constexpr double kAvoidMergedAreaCoolTimeMax = 10000.0; // s
  double time_since_ego_leave_ramp =
      std::min(selector_common_feature.time_since_ego_leave_ramp,
               kAvoidMergedAreaCoolTimeMax);
  selector_debug->mutable_debugstring()->Add(
      absl::StrFormat("time_since_ego_leave_ramp: %.2f",
                      time_since_ego_leave_ramp));
  auto all_trajectory_cost = CalculateTrajectoryCostV2(
      input, est_status, st_traj_mgr_list, results, *selector_state,
      &idx_traj_feature_output_map, selector_debug, &tasks_curb_flags,
      &ego_near_intersection, deviate_navi_input.func_id, last_selected_idx,
      &is_nearby_obs, pre_lc_stage, selector_common_feature);
  selector_output.is_going_force_route_change_left =
      input.is_going_force_route_change_left;
  selector_output.begin_route_change_left = input.begin_route_change_left;
  int valid_traj_count = 0;
  bool existing_borrow_flag = false;
  for (int idx = 0; idx < results.size(); ++idx) {
    if (!est_status[idx].ok()) continue;
    if (results[idx].scheduler_output.is_expert) continue;
    if (results[idx].scheduler_output.borrow_lane) {
      existing_borrow_flag = true;
    }
    valid_traj_count++;
  }
  if (valid_traj_count == 0) {
    std::stringstream fail_reason;
    fail_reason << "No valid trajectory added for selection:\n";
    for (int idx = 0; idx < est_status.size(); ++idx) {
      fail_reason << "Task " << idx << ": " << est_status[idx].message();
    }
    return absl::NotFoundError(fail_reason.str());
  }
  // 2. Find best trajectory.
  int final_selected_idx = -1;
  final_selected_idx = FindBestTrajectory(
      input.plan_time, last_selected_idx, *input.psmm, prefilter_est_status,
      results, *input.selector_flags, *input.vehicle_geom,
      *input.plan_start_point, all_trajectory_cost, idx_traj_feature_output_map,
      selector_debug, selector_state, &selector_output, highway_mode,
      ego_near_intersection, input.icc_lc_enable);

  UpdateRouteIntentionInSeletorState(
      results, prefilter_est_status, selector_common_feature,
      idx_traj_feature_output_map, final_selected_idx, all_trajectory_cost,
      /*use_begin_route_change*/ true,
      &selector_state->begin_route_change_left_info);

  UpdateRouteIntentionInSeletorState(
      results, prefilter_est_status, selector_common_feature,
      idx_traj_feature_output_map, final_selected_idx, all_trajectory_cost,
      /*use_begin_route_change*/ false,
      &selector_state->force_route_change_left_info);

  UpdateSelectorOutput(results, prefilter_est_status, selector_common_feature,
                       idx_traj_feature_output_map, highway_mode,
                       final_selected_idx, last_selected_idx,
                       selector_state->begin_route_change_left_info,
                       selector_state->force_route_change_left_info,
                       &selector_output, selector_debug, all_trajectory_cost,
                       input.is_open_gap, *input.selector_flags);
  // refresh the count
  if (input.is_open_gap &&
      selector_output.begin_route_change_left.has_value()) {
    if (input.begin_route_change_left.has_value() &&
        input.begin_route_change_left.value() ==
            selector_output.begin_route_change_left.value()) {
      selector_state->begin_route_change_successive_count += 1;
    } else {
      selector_state->begin_route_change_successive_count = 1;
    }
  } else {
    selector_state->begin_route_change_successive_count = 0;
  }

  if (input.is_open_gap &&
      selector_output.is_going_force_route_change_left.has_value()) {
    if (input.is_going_force_route_change_left.has_value() &&
        input.is_going_force_route_change_left.value() ==
            selector_output.is_going_force_route_change_left.value()) {
      selector_state->force_route_change_successive_count += 1;
    } else {
      selector_state->force_route_change_successive_count = 1;
    }
  } else {
    selector_state->force_route_change_successive_count = 0;
  }

  // Update is_passed_split_lane_change
  if (final_selected_idx >= 0) {
    selector_state->is_passed_split_lane_change =
        idx_traj_feature_output_map.at(final_selected_idx)
            .is_passed_split_lane_change;
  }

  // 3. Update selector state.
  bool has_red_light_stop_s = true;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok()) continue;
    if (results[idx].scheduler_output.is_expert) continue;

    has_red_light_stop_s =
        has_red_light_stop_s && results[idx].redlight_lane_id.has_value();
  }
  if (has_red_light_stop_s) {
    selector_state->last_redlight_stop_time = input.plan_time;
  }
  const auto& final_scheduler = results[final_selected_idx].scheduler_output;
  if (IsPerformLaneChange(final_scheduler.lane_change_state.stage())) {
    selector_state->last_lc_info.set_lc_left(
        final_scheduler.lane_change_state.lc_left());
    st::ToProto(input.plan_time,
                selector_state->last_lc_info.mutable_lane_change_time());
  }

  if (selector_debug->traj_costs(final_selected_idx).lc_stage() ==
          LaneChangeStage::LCS_NONE &&
      selector_state->lane_change_prepare_state ==
          LaneChangePrepareState::Lane_Keeping) {
    selector_state->last_lane_change_reason = LaneChangeReason::NO_CHANGE;
  }
  selector_state->lane_change_reason = AnalyzeLaneChangeReason(
      *selector_debug, final_selected_idx, selector_state->lane_change_reason,
      idx_traj_feature_output_map);
  if (results[final_selected_idx]
          .scheduler_output.lane_change_state.has_stage()) {
    if (results[final_selected_idx]
            .scheduler_output.lane_change_state.stage() ==
        LaneChangeStage::LCS_PAUSE) {
      // TODO(wx): Delete this!
    }
  }
  UpdateSelectorStateAfterSelection(
      input.plan_time, *input.selector_flags, results, prefilter_est_status,
      idx_traj_feature_output_map, last_selected_idx, final_selected_idx,
      selector_output.best_traj_idx, /*input.is_paddle_lane_change*/ false,
      selector_output, selector_state, selector_debug, deviate_navi_input);

  // reset last_lane_change time when ego enter turn junction
  if (!IsPerformLaneChange(final_scheduler.lane_change_state.stage()) &&
      final_scheduler.drive_passage.lane_seq_info() != nullptr) {
    const auto& lane_seq_info = final_scheduler.drive_passage.lane_seq_info();
    const auto& junction_lane = lane_seq_info->junction_lane;
    if (junction_lane != nullptr &&
        junction_lane->turn_type() != ad_byd::planning::NO_TURN &&
        lane_seq_info->dist_to_junction < 5.0) {
      ToProto(absl::InfinitePast(),
              selector_state->last_lc_info.mutable_lane_change_time());
    }
  }
  selector_state->gap_front_id = "";
  selector_state->gap_back_id = "";
  const auto& traffic_gap = results[final_selected_idx].traffic_gap;
  if (traffic_gap.leader_id.has_value()) {
    std::string leader_id = *traffic_gap.leader_id;
    selector_state->gap_front_id = leader_id;
  }
  if (traffic_gap.follower_id.has_value()) {
    std::string follower_id = *traffic_gap.follower_id;
    selector_state->gap_back_id = follower_id;
  }

  return selector_output;
}

}  // namespace st::planning
