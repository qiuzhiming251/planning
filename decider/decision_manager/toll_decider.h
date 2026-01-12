

#ifndef ONBOARD_PLANNER_DECISION_TOLL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TOLL_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"

namespace st {

namespace planning {
absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildTollConstraints(const PlannerSemanticMapManager& psmm,
                     const DrivePassage& passage,
                     const mapping::LanePath& lane_path_from_start,
                     double s_offset);
}
}  // namespace st

#endif
