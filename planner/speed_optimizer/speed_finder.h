

#ifndef ONBOARD_PLANNER_SPEED_SPEED_FINDER_H_
#define ONBOARD_PLANNER_SPEED_SPEED_FINDER_H_

#include "absl/status/statusor.h"
#include "plan_common/async/thread_pool.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "planner/speed_optimizer/speed_finder_input.h"
#include "planner/speed_optimizer/speed_finder_output.h"

namespace st {
namespace planning {
absl::StatusOr<SpeedFinderOutput> FindSpeed(
    const SpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params, ThreadPool* thread_pool,
    EgoFrame* curr_ego_frame, SpeedFinderStateProto* speed_finder_state);
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_SPEED_FINDER_H_
