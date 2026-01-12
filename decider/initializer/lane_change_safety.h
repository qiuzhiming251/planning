

#ifndef ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_H_
#define ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "decider/initializer/lane_change_style_decider.h"
#include "object_manager/object_history.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/util/decision_info.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/path_sl_boundary.h"

namespace st::planning {

bool HasEnteredTargetLane(const double center_l, const double half_width);

bool HasEnteredTargetLane(const FrenetBox& obj_box, const double lane_width);

bool HasFullyEnteredTargetLane(const double center_l, const double half_width);

bool HasFullyEnteredTargetLane(const FrenetBox& obj_box,
                               const double half_width);
bool HasFullyCenteredInTargetLane(const FrenetBox& obj_box,
                                  const double half_width);
absl::Status CheckLaneChangeSafety(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const PathSlBoundary& sl_boundary,
    const ApolloTrajectoryPointProto& start_point,
    const std::vector<ApolloTrajectoryPointProto>& ego_traj_pts,
    const std::vector<std::string>& leading_traj_ids,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const FrenetFrame& target_frenet_frame, double speed_limit,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ObjectHistoryManager& obs_history_mgr,
    const mapping::LanePath& target_lane_path_ext,
    const VehicleParamsProto& vehicle_params, LaneChangeStyle lc_style,
    const std::pair<PathResponseStyle, SpeedResponseStyle>& prev_resp_style,
    const st::LaneChangeStage& lc_state,
    const st::LaneChangeStage& prev_lc_stage, const bool is_congestion_scene,
    absl::Duration path_look_ahead_duration, int plan_id,
    TrajEvalInfo* eval_info,
    LaneChangeStylePostDeciderSceneInfo* target_front_obj_scene_info,
    LaneChangeStylePostDeciderSceneInfos* scene_infos,
    int* scene_cones_riding_line_frames_result,
    absl::flat_hash_set<std::string>& gaming_lc_obs_set,
    bool is_closed_ramp = false, bool is_on_highway = false,
    const std::vector<double>* stop_s_vec = nullptr);
}  // namespace st::planning

#endif  // ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_H_
