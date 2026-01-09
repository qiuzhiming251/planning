

#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_

#include "plan_common/maps/lane_path.h"
#include "plan_common/math/frenet_common.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/scheduler.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
#include "object_manager/planner_object_manager.h"
#include "plan_common/maps/lane_path_info.h"
#include "plan_common/type_def.h"
#include "plan_common/planner_status.h"

namespace st::planning {
using PushDirection = ad_byd::planning::PushDirection;

// Creates a lane change state proto from a lane change stage. The rest fields
// use default settings.
LaneChangeStateProto MakeNoneLaneChangeState();

absl::StatusOr<LaneChangeStateProto> MakeLaneChangeState(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& start_point,
    const FrenetBox& ego_frenet_box,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const mapping::LanePath& preferred_lane_path,
    const LaneChangeStateProto& prev_lc_state,
    const SmoothedReferenceLineResultMap& smooth_res_map, bool should_smooth,
    bool if_miss_navi, bool is_continuous_lc, Behavior behavior,
    const int& target_lane_path_num);

void MakePushState(const DrivePassage& drive_passage,
                   const SpacetimeTrajectoryManager& st_traj_mgr,
                   const FrenetBox& ego_frenet_box,
                   const ad_byd::planning::MapPtr& map,
                   const PlannerObjectManager* obj_mgr,
                   const std::vector<LanePathInfo>& lp_infos,
                   const LanePathInfo& cur_lp_info,
                   const ApolloTrajectoryPointProto& plan_start_point,
                   const bool is_miss_navi, const bool borrow,
                   const std::optional<bool> is_going_force_route_change_left,
                   const ad_byd::planning::PushDirection pre_push_dir,
                   const PushStatusProto* pre_push_status,
                   const absl::flat_hash_set<std::string>* stalled_objects,
                   LaneChangeStateProto* lc_state);

void ExtractTrafficFlowInfo(
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlannerObjectManager* obj_mgr, const bool lc_left,
    const ad_byd::planning::LaneSeqInfoPtr lane_seq_info,
    const bool last_lc_push_state,
    const absl::flat_hash_set<std::string>* stalled_objects,
    const ad_byd::planning::PushDirection pre_push_dir,
    const double speed_limit, std::string* front_nearest_stall_id,
    double* front_nearest_stall_dis, int* traffic_cnt, double* traffic_speed,
    bool* exist_large_vehicle, bool* exist_stalled_vehicle,
    bool* is_congestion_scenario);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_
