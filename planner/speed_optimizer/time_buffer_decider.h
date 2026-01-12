

#ifndef ONBOARD_PLANNER_SPEED_TIME_BUFFER_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_TIME_BUFFER_DECIDER_H_

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"

namespace st {
namespace planning {

// Set pass time and yield time for st-boundary according to its type, right of
// way, etc.
void DecideTimeBuffersForStBoundary(
    StBoundaryWithDecision* st_boundary_wd, double init_v,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_params,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    bool disable_pass_time_buffer);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_TIME_BUFFER_DECIDER_H_
