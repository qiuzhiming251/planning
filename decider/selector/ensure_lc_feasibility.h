#pragma once

#include <vector>

#include "decider/selector/selector_util.h"
#include "decider/selector/cost_feature_base.h"

#include "modules/cnoa_pnc/planning/proto/lane_change_type.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_state.pb.h"

#include "object_manager/st_inference/selector_output.h"
#include "decider/selector/selector_defs.h"

namespace st::planning {

inline constexpr bool IsDefensiveLaneChange(LaneChangeType lc_type) {
  return lc_type == LaneChangeType::TYPE_AVOID_CONES;
}

bool DisallowOvertakeNearRampOrCross(
    const bool on_highway, const double dist_to_ramp,
    const double dist_to_cross,
    const std::optional<double>& dist_to_solid_line);

LcFeasibility CanOvertakeLaneChange(
    const TrajFeatureOutput& traj_feature_output,
    const TrajFeatureOutput& lane_keep_feature_output, bool on_highway,
    bool is_in_tunnel, const RoadHorizonInfo& road_horizon,
    LaneChangeType lc_type, const std::optional<double>& dist_to_solid_line);

LcFeasibility CanDefaultRouteLaneChange(
    const TrajFeatureOutput& lk_traj_feature_output,
    const TrajFeatureOutput& best_traj_feature_output, bool on_highway,
    const RoadHorizonInfo& road_horizon, LaneChangeType lc_type,
    bool is_navi_dir);

bool CanOppositeDefensiveLaneChange(
    LaneChangeType lc_type, bool lc_left,
    const std::optional<double>& opposite_lc_interval_secs,
    bool last_lc_is_paddle);

bool IsLaneChangeCanPassBorrowObj(
    const TrajFeatureOutput& lane_keep_traj_feature_output,
    const TrajFeatureOutput& best_traj_feature_output);

LcFeasibility EnsureLcFeasibilty(
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const double dist_to_tunnel, bool on_highway, int best_selected_idx,
    int lane_keep_idx, LaneChangeType lc_type);

bool PassPrepareLcCheck(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map,
    const SelectorCommonFeature& selector_common_feature,
    const int best_selected_idx, const int lane_keep_idx,
    const LaneChangeType lc_type, const bool use_begin_route_change);

LaneChangeCancelReason CalcLcCancelReason(
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    int last_selected_idx, int best_traj_idx);

}  // namespace st::planning
