

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_HMI_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_HMI_UTIL_H_

#include <vector>

#include "absl/types/span.h"
#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/drive_passage.h"
#include "plan_common/util/decision_info.h"

namespace st {
namespace planning {
namespace optimizer {
absl::StatusOr<std::optional<NudgeObjectInfo>> ExtractNudgeObjectId(
    int trajectory_steps, double trajectory_time_step, LaneChangeStage lc_stage,
    PushState push_dir, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const std::vector<TrajectoryPoint>& result_points,
    absl::Span<const ApolloTrajectoryPointProto> previous_trajectory,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<std::string>& final_cost_debug,
    const NudgeObjectInfo* previous_nudge_object_info);
}  // namespace optimizer
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_HMI_UTIL_H_
