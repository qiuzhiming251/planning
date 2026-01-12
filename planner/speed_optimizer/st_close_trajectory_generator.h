

#ifndef ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_GENERATOR_H_
#define ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_GENERATOR_H_

#include <vector>

#include "object_manager/spacetime_object_trajectory.h"
#include "planner/speed_optimizer/st_close_trajectory.h"

namespace st::planning {
std::vector<StCloseTrajectory> GenerateMovingStCloseTrajectories(
    const SpacetimeObjectTrajectory& st_traj,
    std::vector<std::vector<StCloseTrajectory::StNearestPoint>>
        close_traj_points);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_GENERATOR_H_
