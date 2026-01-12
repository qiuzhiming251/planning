

#ifndef ONBOARD_PLANNER_DECISION_END_OF_CURRENT_LANE_PATH_H_
#define ONBOARD_PLANNER_DECISION_END_OF_CURRENT_LANE_PATH_H_

#include "absl/status/statusor.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

// This function returns end of current lane path constraint: it signifies
// either the end of current trip, or the last lane point before lane change.
absl::StatusOr<ConstraintProto::StopLineProto>
BuildEndOfCurrentLanePathConstraint(const DrivePassage& passage);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_END_OF_CURRENT_LANE_PATH_H_
