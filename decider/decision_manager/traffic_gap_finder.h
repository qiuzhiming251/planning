

#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_FINDER_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_FINDER_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/type_def.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"

namespace st::planning {
struct TrafficGap {
  absl::Span<const SpacetimeObjectTrajectory* const> leader_trajectories;
  absl::Span<const SpacetimeObjectTrajectory* const> follower_trajectories;
  double s_start;
  double s_end;
  double avg_speed;
  bool ego_in_gap;
};
struct PlannerObjectProjectionInfo {
  absl::Span<const SpacetimeObjectTrajectory* const> st_trajectories;
  FrenetBox frenet_box;
};

std::vector<TrafficGap> FindCandidateTrafficGapsOnLanePath(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr, double ego_v,
    double dist_to_merge,
    const st::planning::DeciderStateProto& pre_decider_state,
    const std::vector<std::string>& lc_lead_obj_ids, double ego_heading,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const st::LaneChangeStateProto& lc_state);

absl::StatusOr<TrafficGapResult> EvaluateAndTakeBestTrafficGap(
    std::vector<TrafficGap>& candidate_gaps, const FrenetBox& ego_frenet_box,
    const FrenetFrame& target_frenet_frame, double ego_init_v,
    double speed_limit, double navi_dist, int lc_num, double dist_to_merge,
    const ConstraintProto::TrafficGapProto& last_traffic_gap,
    const st::Behavior_FunctionId& function_id, double ego_heading,
    const std::optional<PlannerObjectProjectionInfo>&);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_FINDER_H_
