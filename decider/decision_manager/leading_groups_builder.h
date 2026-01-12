

#ifndef ONBOARD_PLANNER_DECISION_LEADING_GROUPS_BUILDER_H_
#define ONBOARD_PLANNER_DECISION_LEADING_GROUPS_BUILDER_H_

#include <map>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/math/frenet_common.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/util/decision_info.h"

namespace st::planning {

// Use trajectory id as key.

std::vector<LeadingGroup> FindMultipleLeadingGroups(
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    bool lc_left, const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects, double ego_heading,
    const FrenetBox& ego_frenet_box,
    const VehicleGeometryParamsProto& vehicle_geom,
    const st::planning::PlannerSemanticMapManager& psmm,
    const double path_start_time_offset);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_DECISION_LEADING_GROUPS_BUILDER_H_
