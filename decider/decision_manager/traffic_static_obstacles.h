

#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_STATIC_OBSTACLES_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_STATIC_OBSTACLES_H_

#include "absl/status/statusor.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

// This function returns the end of path boundary constraint.
absl::StatusOr<ConstraintProto::StopLineProto>
BuildTrafficStaticObstaclesConstraint(const DrivePassage& passage);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_END_OF_PATH_BOUNDARY_H_
