

#ifndef ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_
#define ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "plan_common/log_data.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/maps/map.h"
#include "plan_common/math/frenet_common.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/object_history.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

// Returns leading objects that we should not pass. Currently all leading
// objects are associated with our lane path.
std::vector<ConstraintProto::LeadingObjectProto> FindLeadingObjects(
    const ad_byd::planning::TrafficLightStatusMap& tl_status_map,
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const PathSlBoundary& sl_boundary, LaneChangeStage lc_stage,
    const SceneOutputProto& scene_reasoning,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ApolloTrajectoryPointProto& plan_start_point,
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetBox& ego_frenet_box, bool borrow_lane_boundary,
    const ObjectHistoryManager* obs_his_manager,
    std::map<std::string, bool>& obj_lead,
    const NudgeObjectInfo* nudge_object_info, const bool& is_lane_change,
    const double& nearest_stop_s, bool* is_first_lead, bool& must_borrow);
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_
