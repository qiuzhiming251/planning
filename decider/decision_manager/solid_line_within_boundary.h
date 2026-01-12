

#ifndef ONBOARD_PLANNER_DECISION_SOLID_LINE_WITHIN_BOUNDARY_H_
#define ONBOARD_PLANNER_DECISION_SOLID_LINE_WITHIN_BOUNDARY_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

absl::StatusOr<std::vector<ConstraintProto::AvoidLineProto>>
BuildSolidLineWithinBoundaryConstraint(
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const ApolloTrajectoryPointProto& plan_start_point);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_SOLID_LINE_WITHIN_BOUNDARY_H_
