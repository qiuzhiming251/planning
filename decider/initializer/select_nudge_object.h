

#ifndef ONBOARD_PLANNER_INITIALIZER_SELECT_NUDGE_OBJECT_H_
#define ONBOARD_PLANNER_INITIALIZER_SELECT_NUDGE_OBJECT_H_

#include <vector>

#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/drive_passage.h"
#include "plan_common/util/decision_info.h"

namespace st {
namespace planning {
namespace initializer {
absl::StatusOr<NudgeInfos> SelectNudgeObjectId(
    int trajectory_steps, double trajectory_time_step, bool is_lane_change,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const std::vector<TrajectoryPoint>& result_points,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const int plan_id);
}  // namespace initializer
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_INITIALIZER_SELECT_NUDGE_OBJECT_H_
