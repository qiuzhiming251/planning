

#ifndef ONBOARD_PLANNER_DECISION_NO_BLOCK_H_
#define ONBOARD_PLANNER_DECISION_NO_BLOCK_H_

#include <vector>

#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

// This function return no block constraint. For more details see 'no_block.md'
std::vector<ConstraintProto::SpeedRegionProto> BuildNoBlockConstraints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_NO_BLOCK_H_
