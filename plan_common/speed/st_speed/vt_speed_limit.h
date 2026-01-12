

#ifndef ONBOARD_PLANNER_SPEED_VT_SPEED_LIMIT_H_
#define ONBOARD_PLANNER_SPEED_VT_SPEED_LIMIT_H_

#include <vector>

#include "plan_common/speed/st_speed/speed_limit.h"

namespace st::planning {

// v and t sequence).
using VtSpeedLimit = std::vector<SpeedLimit::SpeedLimitInfo>;

void MergeVtSpeedLimit(const VtSpeedLimit& source, VtSpeedLimit* target);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_VT_SPEED_LIMIT_H_
