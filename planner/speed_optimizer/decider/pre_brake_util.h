

#ifndef ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_UTIL_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_UTIL_H_

#include <string>

#include "plan_common/speed/st_speed/vt_speed_limit.h"

namespace st {
namespace planning {

VtSpeedLimit GenerateConstAccSpeedLimit(double start_t, double end_t,
                                        double start_v, double min_v,
                                        double max_v, double acc,
                                        double time_step, int step_num,
                                        const std::string& info);

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_UTIL_H_
