

#ifndef ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_
#define ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_

#include <string>
#include <vector>

#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

std::vector<TrajectoryPoint> SmoothTrajectoryByMixedFourthOrderDdp(
    int plan_id, int trajectory_steps, double trajectory_time_step,
    const std::vector<TrajectoryPoint>& ref_traj,
    const DrivePassage& drive_passage, const std::string& owner,
    const TrajectorySmootherCostWeightParamsProto& smoother_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_
