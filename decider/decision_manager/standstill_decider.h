

#ifndef ONBOARD_PLANNER_DECISION_STANDSTILL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_STANDSTILL_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {
absl::StatusOr<std::vector<ConstraintProto::StopLineProto>>
BuildStandstillConstraints(
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines);
}  // namespace planning
}  // namespace st
#endif
