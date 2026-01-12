

#ifndef ONBOARD_PLANNER_DECISION_CAUTIOUS_BRAKE_H_
#define ONBOARD_PLANNER_DECISION_CAUTIOUS_BRAKE_H_

#include <vector>

#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

// This function return cautious brake constraint.
std::vector<ConstraintProto::SpeedRegionProto> BuildCautiousBrakeConstraints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const SpacetimeTrajectoryManager& st_traj_mgr);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_CAUTIOUS_BRAKE_H_
