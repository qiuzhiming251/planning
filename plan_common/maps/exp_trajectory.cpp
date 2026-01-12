

#include "plan_common/maps/exp_trajectory.h"
namespace ad_byd {
namespace planning {
ExpTrajectory::ExpTrajectory(const ExpTrajectoryInfo& exp_trajectory_info) {
  id_ = exp_trajectory_info.id;
  lane_id_ = exp_trajectory_info.lane_id;
  start_lane_id_ = exp_trajectory_info.start_lane_id;
  end_lane_id_ = exp_trajectory_info.end_lane_id;
  relative_lane_id_.assign(exp_trajectory_info.relative_lane_id.begin(),
                           exp_trajectory_info.relative_lane_id.end());
  points_.assign(exp_trajectory_info.points.begin(),
                 exp_trajectory_info.points.end());
}

}  // namespace planning
}  // namespace ad_byd