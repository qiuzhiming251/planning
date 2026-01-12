

#ifndef ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_
#define ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/drive_passage.h"
#include "plan_common/trajectory_util.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/trajectory_point.h"

namespace st {
namespace planning {

std::vector<TrajectoryPoint> GetExtendStateByPurePursuit(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    double trajectory_time_step, const TrajectoryPoint& extend_start_point,
    int k_extend_steps, double target_v);

// Transform a trajectory to a path which extends the last trajectory point by
// an arc with a given minimum length, while considering deceleration distance.
absl::StatusOr<std::vector<PathPoint>> ExtendPathAndDeleteUnreasonablePart(
    const st::planning::DrivePassage& drive_passage,
    const st::planning::PathSlBoundary& path_sl_boundary,
    const st::MotionConstraintParamsProto& motion_constraint_params,
    const st::VehicleGeometryParamsProto& vehicle_geom_params,
    absl::Span<const ApolloTrajectoryPointProto> trajectory_points,
    double min_length, double max_curvature);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_
