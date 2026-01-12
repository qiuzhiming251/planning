

#ifndef ONBOARD_PLANNER_MANUAL_TRAJECTORY_UTIL_H_
#define ONBOARD_PLANNER_MANUAL_TRAJECTORY_UTIL_H_

#include "absl/time/time.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"

namespace st {
namespace planning {

void CompleteTrajectoryPastPoints(double trajectory_time_step,
                                  TrajectoryProto* trajectory);

// Update manual trajectory iterationally.
void ShiftPreviousTrajectory(const absl::Duration shift_duration,
                             TrajectoryProto* trajectory);

void CompleteTrajectoryPastPoints(TrajectoryProto* trajectory);

// Recalculate trajectory point's acceleration.
void UpdateTrajectoryPointAccel(TrajectoryProto* trajectory);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_MANUAL_TRAJECTORY_UTIL_H_
