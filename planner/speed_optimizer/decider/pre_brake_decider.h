

#ifndef ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_DECIDER_H_

#include <optional>
#include <vector>

#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "plan_common/speed/st_speed/vt_speed_limit.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

std::optional<VtSpeedLimit> MakePedestrainPreBrakeDecision(
    const SpeedFinderParamsProto::PreBrakeDeciderParamsProto& params,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, double max_v, double time_step, int step_num,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

std::optional<VtSpeedLimit> MakeUncertainVehiclePreBrakeDecision(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_params, double current_v,
    double max_v, double time_step, int step_num,
    const SpeedVector& preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

std::optional<VtSpeedLimit> MakeJunctionStraightSpeedDiffPreBrakeDecision(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_params, double current_v,
    double max_v, double time_step, int step_num,
    const SpeedVector& preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

std::optional<VtSpeedLimit> MakeCreepInteractionDecision(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_params, double current_v,
    double max_v, double time_step, int step_num,
    const SpeedVector& preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_PRE_BRAKE_DECIDER_H_
