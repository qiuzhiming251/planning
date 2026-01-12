

#ifndef ONBOARD_PLANNER_DECISION_PARKING_BRAKE_RELEASE_H_
#define ONBOARD_PLANNER_DECISION_PARKING_BRAKE_RELEASE_H_

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

// This function returns parking brake release constraint.
absl::StatusOr<ConstraintProto::StopLineProto>
BuildParkingBrakeReleaseConstraint(
    const st::VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& passage, const absl::Time parking_brake_release_time,
    const absl::Time plan_time);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_PARKING_BRAKE_RELEASE_H_
