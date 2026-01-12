#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_H_

#include <vector>

#include "absl/status/statusor.h"

#include "modules/cnoa_pnc/planning/proto/selector_debug.pb.h"

#include "plan_common/planner_status.h"
#include "plan_common/selector_state.h"
#include "decider/selector/cost_feature_base.h"

#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/est_planner_output.h"
#include "object_manager/st_inference/selector_input.h"
#include "object_manager/st_inference/selector_output.h"
namespace st {
namespace planning {
using LcReason = ad_byd::planning::LcReason;

namespace internal {

LaneChangeType AnalyzeLaneChangeType(
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    bool is_paddle_lane_change, int last_selected_idx, int final_chosen_idx,
    LaneChangeType last_lane_change_type);

LaneChangeReason ReAnalyzeLaneChangeReason(LaneChangeReason origin_lc_reason,
                                           LaneChangeType lc_type);
}  // namespace internal

absl::StatusOr<SelectorOutput> SelectTrajectoryV2(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results,
    const DeviateNaviInput& deviate_navi_input,
    SelectorDebugProto* selector_debug, SelectorState* selector_state);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_H_
