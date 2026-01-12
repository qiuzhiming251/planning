

#ifndef ONBOARD_PLANNER_DECISION_SPEED_BUMP_H_
#define ONBOARD_PLANNER_DECISION_SPEED_BUMP_H_

#include <vector>

#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

// This function returns speed bump constraints.
std::vector<ConstraintProto::SpeedRegionProto> BuildSpeedBumpConstraints(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_SPEED_BUMP_H_
