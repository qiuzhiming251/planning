

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "predictor/prediction_object_state.h"
#include "planner/trajectory_optimizer/ddp/trajectory_optimizer_defs.h"
#include "plan_common/trajectory_point.h"

namespace st {
namespace planning {
namespace optimizer {

std::optional<std::vector<TrajectoryPoint>>
AdaptTrajectoryToGivenPlanStartPoint(int plan_id, int trajectory_steps,
                                     const Mfob& problem,
                                     const DdpOptimizerParamsProto& params,
                                     double max_adaption_cost,
                                     const TrajectoryPoint& plan_start_point,
                                     std::vector<TrajectoryPoint> trajectory);

absl::Status ValidateTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points,
    const TrajectoryOptimizerValidationParamsProto&
        trajectory_optimizer_validation_params,
    const TrajectoryOptimizerDebugProto& optimizer_debug);

// This function checks two trajectory have same decision using the following
bool HasSameDecisionOverSpacetimeObject(
    const std::vector<TrajectoryPoint>& traj_1,
    const std::vector<TrajectoryPoint>& traj_2,
    const std::vector<prediction::PredictionObjectState>&
        space_time_object_states);

}  // namespace optimizer
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
