

#ifndef ONBOARD_PLANNER_SPEED_EMPTY_ROAD_SPEED_H_
#define ONBOARD_PLANNER_SPEED_EMPTY_ROAD_SPEED_H_

#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_profile.h"
#include "plan_common/speed/st_speed/speed_limit.h"

namespace st {
namespace planning {

// Create speed profile not consider objects.
// Firstly, generate a v_s profile that AV accelerate at max accel. Secondly,
// treat adjacent speed limit points as a speed limit segment. Before that
// segment the speed follows a constant deceleration profile, and after that
// a constant acceleration profile. Thirdly, take the min value of v_s and all
// speed limit profiles along path s as the final speed profile.
SpeedProfile CreateEmptyRoadSpeedProfile(
    const MotionConstraintParamsProto& motion_constraint_params,
    const DiscretizedPath& path, const SpeedLimit& speed_limit, double v_now,
    int traj_steps);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_EMPTY_ROAD_SPEED_H_
