#include "decider/selector/ensure_lc_feasibility.h"
#include "decider/selector/cost_feature_util.h"

namespace st::planning {

namespace {

constexpr double kOvertakeMinNaviDist = 1000.0;                 // m.
constexpr double kOvertakeMinCrossDist = 50.0;                  // m.
constexpr double kBeginAvoidDefaultRouteLaneChangeDist = 25.0;  // m.
constexpr double kOvertakeMaxCrossDist = 100.0;                 // m.
constexpr double kOvertakeMinSolidLineDist = 30.0;              // m.
constexpr double kMinLengthForCutoffLcBack = 20.0;              // m.
constexpr double kEnterPrepareIntentionCostDiffThresold = 2.0;
constexpr double kMaxCostDiffThresold = 6.0;
constexpr double kMissNaviDistBeginBaseCity = 200.0;
constexpr double kMissNaviDistBeginBaseHighway = 300.0;
constexpr double kMissNaviDistForceBaseCity = 150.0;
constexpr double kMissNaviDistForceBaseHighway = 200.0;

}  // namespace

bool DisallowOvertakeNearRampOrCross(
    bool on_highway, double dist_to_ramp, double dist_to_cross,
    const std::optional<double>& dist_to_solid_line) {
  return (on_highway && dist_to_ramp < kOvertakeMinNaviDist) ||
         (!on_highway &&
          (dist_to_cross < kOvertakeMinCrossDist ||
           (dist_to_cross < kOvertakeMaxCrossDist &&
            dist_to_solid_line.has_value() &&
            dist_to_solid_line.value() < kOvertakeMinSolidLineDist)));
}

LcFeasibility CanOvertakeLaneChange(
    const TrajFeatureOutput& traj_feature_output,
    const TrajFeatureOutput& lane_keep_feature_output, bool on_highway,
    bool is_in_tunnel, const RoadHorizonInfo& road_horizon,
    LaneChangeType lc_type, const std::optional<double>& dist_to_solid_line) {
  const bool lc_left = traj_feature_output.lane_change_left;
  double opposite_lc_interval_secs =
      traj_feature_output.opposite_lc_interval_secs.value_or(
          std::numeric_limits<double>::max());
  VLOG(3) << "opposite_lc_interval_secs: " << opposite_lc_interval_secs;
  if (lc_type != LaneChangeType::TYPE_OVERTAKE_CHANGE &&
      lc_type != LaneChangeType::TYPE_INTERSECTION_OBS) {
    return LcFeasibility::FEASIBILITY_OK;
  }
  // Check Obvious route cost
  if (traj_feature_output.has_obvious_route_cost) {
    return LcFeasibility::FEASIBILITY_OBVIOUSE_ROUTE;
  }

  if (traj_feature_output.has_obvious_invade_risk) {
    return LcFeasibility::FEASIBILITY_OBS_INVADE_RISK;
  }

  if (lane_keep_feature_output.is_accel_traj_start) {
    return LcFeasibility::FEASIBILITY_AVOID_OVERTAKE_IF_ACCEL;
  }

  constexpr double kAvoidOVertakeBeforeMergeInHighway = 500.0;  // m.
  const double dist_to_merge = road_horizon.dist_to_merge;
  // Lane feature
  if (on_highway && dist_to_merge < kAvoidOVertakeBeforeMergeInHighway) {
    return LcFeasibility::FEASIBILITY_MERGE_LANE;
  }

  // Section feature
  const double dist_to_ramp = road_horizon.dist_to_fork;
  const double dist_to_cross = road_horizon.dist_to_cross;
  if (DisallowOvertakeNearRampOrCross(on_highway, dist_to_ramp, dist_to_cross,
                                      dist_to_solid_line)) {
    return LcFeasibility::FEASIBILITY_ROUTE_INTENT;
  }

  // change ahead.
  constexpr double kAvoidOvertakeDistBeforeTunnel = -30.0;
  constexpr double kAvoidOvertakeDistAfterTunnelMinDist = -15.0;

  // Can opposite lc
  constexpr double kMinLcLeftOppositeTimeHighway = 8.0;    // s.
  constexpr double kMinLcRightOppositeTimeHighway = 10.0;  // s.

  constexpr double kMinLcLeftOppositeTime = 10.0;   // s.
  constexpr double kMinLcRightOppositeTime = 15.0;  // s.
  const auto left_oppsite_ttc =
      on_highway ? kMinLcLeftOppositeTimeHighway : kMinLcLeftOppositeTime;
  const auto right_oppsite_ttc =
      on_highway ? kMinLcRightOppositeTimeHighway : kMinLcRightOppositeTime;
  if (!on_highway &&
      ((lc_left && opposite_lc_interval_secs < left_oppsite_ttc) ||
       (!lc_left && opposite_lc_interval_secs < right_oppsite_ttc))) {
    return LcFeasibility::FEASIBILITY_OPPOSITE;
  }

  // Check curvature
  if (traj_feature_output.is_obvous_curvy_road) {
    return LcFeasibility::FEASIBILITY_CURVATURE;
  }

  // Can decel lc
  constexpr double kMinLcDecleration = -0.35;  // m/s2.
  if (traj_feature_output.min_a - lane_keep_feature_output.min_a <
      kMinLcDecleration) {
    return LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT;
  }

  return LcFeasibility::FEASIBILITY_OK;
}

[[maybe_unused]] bool CanOppositeDefensiveLaneChange(
    LaneChangeType lc_type, bool lc_left,
    const std::optional<double>& opposite_lc_interval_secs,
    bool last_lc_is_paddle) {
  if (!opposite_lc_interval_secs.has_value()) {
    return true;
  }
  if (!last_lc_is_paddle) {
    // Not a paddle lane change
    return true;
  }
  bool need_check = (lc_type == LaneChangeType::TYPE_CENTER_CHANGE);
  if (!need_check) {
    return true;
  }
  // Can opposite lc

  constexpr double kMinLcLeftOppositeTime = 8.0;    // secs.
  constexpr double kMinLcRightOppositeTime = 25.0;  // secs.
  return (lc_left && *opposite_lc_interval_secs < kMinLcLeftOppositeTime) ||
         (!lc_left && *opposite_lc_interval_secs < kMinLcRightOppositeTime);
}

LcFeasibility CanDefaultRouteLaneChange(
    const TrajFeatureOutput& lane_keep_traj_feature_output,
    const TrajFeatureOutput& best_traj_feature_output, bool on_highway,
    const RoadHorizonInfo& road_horizon, LaneChangeType lc_type,
    bool is_navi_dir) {
  const bool lc_left = lane_keep_traj_feature_output.lane_change_left;
  // Check Obvious route cost
  if (lane_keep_traj_feature_output.has_obvious_stalled_object ||
      lane_keep_traj_feature_output.has_obvious_cutoff_cost ||
      lane_keep_traj_feature_output.lane_change_for_stationary_obj) {
    return LcFeasibility::FEASIBILITY_OK;
  }
  if (best_traj_feature_output.has_obvious_invade_risk) {
    return LcFeasibility::FEASIBILITY_OBS_INVADE_RISK;
  }
  if (!is_navi_dir) {
    return LcFeasibility::FEASIBILITY_ROUTE_INTENT;
  }
  if (lane_keep_traj_feature_output.has_obvious_route_cost) {
    return LcFeasibility::FEASIBILITY_OK;
  }
  if (road_horizon.dist_to_cross < kBeginAvoidDefaultRouteLaneChangeDist) {
    return LcFeasibility::FEASIBILITY_GENERAL;
  }
  return LcFeasibility::FEASIBILITY_OK;
}

bool IsLaneChangeCanPassBorrowObj(
    const TrajFeatureOutput& lane_keep_traj_feature_output,
    const TrajFeatureOutput& best_traj_feature_output) {
  const auto& lane_keep_lat_away_objs =
      lane_keep_traj_feature_output.lat_away_obj_ids;
  const auto& best_traj_block_objs =
      best_traj_feature_output.traj_block_obj_ids;
  if (best_traj_block_objs.empty() || lane_keep_lat_away_objs.empty()) {
    return true;
  }
  for (const auto& block_obj_id : best_traj_block_objs) {
    if (lane_keep_lat_away_objs.contains(block_obj_id)) {
      return false;
    }
  }
  return true;
}

LcFeasibility EnsureLcFeasibilty(
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const double dist_to_tunnel, bool on_highway, int best_selected_idx,
    int lane_keep_idx, LaneChangeType lc_type) {
  if (best_selected_idx == lane_keep_idx) {
    return LcFeasibility::FEASIBILITY_OK;
  }
  const auto& last_scheduler = results[lane_keep_idx].scheduler_output;
  const auto& best_traj_feature_output =
      idx_traj_feature_output_map.at(best_selected_idx);
  const auto& lane_keep_traj_feature_output =
      idx_traj_feature_output_map.at(lane_keep_idx);
  const auto driving_dist =
      last_scheduler.drive_passage.lane_seq_info()->dist_to_navi_end;
  const auto lane_keep_dist_to_merge =
      last_scheduler.drive_passage.lane_seq_info() == nullptr
          ? DBL_MAX
          : last_scheduler.drive_passage.lane_seq_info()->dist_to_merge;
  // Check CrossSolidBoundary
  bool cross_solid_boundary = best_traj_feature_output.cross_solid_boundary;
  LOGINFO_EVERY(5) << "lc_type: " << LaneChangeType_Name(lc_type)
                   << ", best_scheduler.lc_num: " << last_scheduler.lc_num
                   << ", driving_dist:" << driving_dist
                   << ", max_reach_length:" << last_scheduler.max_reach_length
                   << ", cross_solid_boundary: " << cross_solid_boundary
                   << ", best_selected_idx: " << best_selected_idx
                   << ", lane_keep_idx: " << lane_keep_idx;
  constexpr double kDistThresholdEnterTunnel = 100.0;
  const bool is_in_tunnel = dist_to_tunnel < kDistThresholdEnterTunnel;
  if (cross_solid_boundary) {
    if (!CanIgnoreCrossSolidBoundary(
            lc_type,
            /*is_scecond_lane_from_right*/ last_scheduler.lc_num, driving_dist,
            best_traj_feature_output.lane_change_left, on_highway, is_in_tunnel,
            lane_keep_dist_to_merge)) {
      return LcFeasibility::FEASIBILITY_LINE_TYPE;
    }
  }

  const auto& lane_seq_info =
      results[best_selected_idx].scheduler_output.drive_passage.lane_seq_info();
  const auto dist_to_merge =
      lane_seq_info != nullptr ? lane_seq_info->dist_to_merge : DBL_MAX;
  const auto dist_to_cross =
      lane_seq_info != nullptr ? lane_seq_info->dist_to_virtual_lane : DBL_MAX;
  RoadHorizonInfo road_horizon_info = {
      .dist_to_fork = driving_dist,
      .dist_to_merge = dist_to_merge,
      .dist_to_cross = dist_to_cross,
  };
  const bool lc_left = best_traj_feature_output.lane_change_left;
  LcFeasibility check_lc_feasibility = LcFeasibility::FEASIBILITY_OK;
  if (lc_type == LaneChangeType::TYPE_OVERTAKE_CHANGE ||
      lc_type == LaneChangeType::TYPE_INTERSECTION_OBS) {
    check_lc_feasibility = CanOvertakeLaneChange(
        best_traj_feature_output, lane_keep_traj_feature_output, on_highway,
        is_in_tunnel, road_horizon_info, lc_type,
        /*dist_to_solid_line=*/std::nullopt);
  }

  if (lc_type == LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE) {
    bool is_navi_dir = results[best_selected_idx].scheduler_output.lc_num <=
                       last_scheduler.lc_num;
    check_lc_feasibility = CanDefaultRouteLaneChange(
        lane_keep_traj_feature_output, best_traj_feature_output, on_highway,
        road_horizon_info, lc_type, is_navi_dir);
  }

  if (lc_type == LaneChangeType::TYPE_AVOID_BUS_LANE &&
      best_traj_feature_output.has_obvious_invade_risk) {
    check_lc_feasibility = LcFeasibility::FEASIBILITY_OBS_INVADE_RISK;
  }

  if (lc_type == LaneChangeType::TYPE_OBSTACLE_CHANGE) {
    if (!IsLaneChangeCanPassBorrowObj(lane_keep_traj_feature_output,
                                      best_traj_feature_output)) {
      check_lc_feasibility = LcFeasibility::FEASIBILITY_OBS_CURRENT_FRONT;
    }
  }

  if (check_lc_feasibility == LcFeasibility::FEASIBILITY_OK) {
    if (lane_keep_traj_feature_output.has_obvious_cutoff_cost &&
        (best_traj_feature_output.feasible_length_along_route -
             lane_keep_traj_feature_output.feasible_length_along_route <
         kMinLengthForCutoffLcBack)) {
      check_lc_feasibility = LcFeasibility::FEASIBILITY_CUTOFF_TOO_CLOSE;
    }
  }
  VLOG(3) << "dist_to_fork: " << driving_dist
          << ", dist_to_merge: " << dist_to_merge
          << ", dist_to_cross: " << dist_to_cross
          << ", lc_type: " << LaneChangeType_Name(lc_type)
          << ", reason: " << LcFeasibility_Name(check_lc_feasibility);
  return check_lc_feasibility;
}

bool PassPrepareLcCheck(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map,
    const SelectorCommonFeature& selector_common_feature,
    const int best_selected_idx, const int lane_keep_idx,
    const LaneChangeType lc_type, const bool use_begin_route_change) {
  bool is_pass_check = true;
  if (best_selected_idx == -1 || lane_keep_idx == -1 ||
      best_selected_idx == lane_keep_idx) {
    return false;
  }

  const auto& last_scheduler = results[lane_keep_idx].scheduler_output;
  const auto& best_traj_feature_output =
      idx_traj_feature_output_map.at(best_selected_idx);
  const auto& lane_keep_traj_feature_output =
      idx_traj_feature_output_map.at(lane_keep_idx);
  const double dist_to_tunnel =
      selector_common_feature.road_horizon_info.dist_to_tunel;
  const bool on_highway = selector_common_feature.in_high_way;

  // solid line、tunnel、static vehicle
  const auto lc_feasibility =
      EnsureLcFeasibilty(results, idx_traj_feature_output_map, dist_to_tunnel,
                         on_highway, best_selected_idx, lane_keep_idx, lc_type);
  if (lc_feasibility != LcFeasibility::FEASIBILITY_OK) {
    return false;
  }

  // safty check
  if (!est_status[best_selected_idx].ok() &&
      (est_status[best_selected_idx].status_code() ==
           PlannerStatusProto::LC_SAFETY_CHECK_FAILED &&
       results[best_selected_idx].safety_check_failed_reason ==
           SafetyCheckFailedReason::LC_UNSAFE_REASON_STALLED_OBJECT)) {
    return false;
  } else {
    const double cost_diff_thresold =
        best_traj_feature_output.cur_frame_has_begin_route_change &&
                (use_begin_route_change
                     ? best_traj_feature_output
                               .begin_route_change_successive_count > 0
                     : best_traj_feature_output
                               .force_route_change_successive_count > 0)
            ? kMaxCostDiffThresold
            : kEnterPrepareIntentionCostDiffThresold;
    // when to be missing navi, ignore the non-static cost
    const double dist_to_navi_end =
        last_scheduler.drive_passage.lane_seq_info() != nullptr
            ? last_scheduler.drive_passage.lane_seq_info()->dist_to_navi_end
            : DBL_MAX;
    const bool is_missing_navi_begin =
        use_begin_route_change &&
        lane_keep_traj_feature_output.has_begin_route_change &&
        dist_to_navi_end < std::max(1, last_scheduler.lc_num) *
                               (on_highway ? kMissNaviDistBeginBaseHighway
                                           : kMissNaviDistBeginBaseCity);
    const bool is_missing_navi_force =
        !use_begin_route_change &&
        lane_keep_traj_feature_output.has_obvious_route_cost &&
        dist_to_navi_end < std::max(1, last_scheduler.lc_num) *
                               (on_highway ? kMissNaviDistForceBaseHighway
                                           : kMissNaviDistForceBaseCity);
    if ((use_begin_route_change ? !is_missing_navi_begin
                                : !is_missing_navi_force) &&
        idx_traj_cost_map.at(lane_keep_idx).cost_common + cost_diff_thresold <
            idx_traj_cost_map.at(best_selected_idx).cost_common) {
      return false;
    }
  }

  if (idx_traj_cost_map.at(lane_keep_idx).static_cost <
      idx_traj_cost_map.at(best_selected_idx).static_cost) {
    return false;
  }

  return is_pass_check;
}

LaneChangeCancelReason CalcLcCancelReason(
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    int last_selected_idx, int best_traj_idx) {
  const int last_selected_lc_num =
      results[last_selected_idx].scheduler_output.lc_num;
  const int best_traj_lc_num = results[best_traj_idx].scheduler_output.lc_num;
  const auto& last_idx_traj_map =
      idx_traj_feature_output_map.at(last_selected_idx);
  const auto& best_idx_traj_map = idx_traj_feature_output_map.at(best_traj_idx);
  if (last_idx_traj_map.has_obvious_cutoff_cost &&
      (best_idx_traj_map.feasible_length_along_route -
           last_idx_traj_map.feasible_length_along_route >
       kMinLengthForCutoffLcBack)) {
    return LaneChangeCancelReason::CANCEL_LC_CURB_CUTOFF;
  }

  if (last_idx_traj_map.lane_change_for_stalled_vehicle) {
    return LaneChangeCancelReason::CANCEL_LC_STALLED_OBJ;
  }

  if (last_idx_traj_map.has_obvious_invade_risk) {
    return LaneChangeCancelReason::CANCEL_LC_INVADE_RISK;
  }

  if (last_idx_traj_map.lane_change_for_emergency) {
    return LaneChangeCancelReason::CANCEL_LC_EMERGENCY;
  }

  if (last_idx_traj_map.has_obvious_route_cost &&
      last_selected_lc_num >= best_traj_lc_num &&
      !best_idx_traj_map.has_obvious_invade_risk) {
    return LaneChangeCancelReason::CANCEL_LC_OBVIOUS_ROUTE;
  }

  return LaneChangeCancelReason::NOT_CANCEL;
}

}  // namespace st::planning
