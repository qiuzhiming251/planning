#ifndef ONBOARD_PLANNER_SPEED_DECIDER_SPEED_LIMIT_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_SPEED_LIMIT_DECIDER_H_

#include <vector>

#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/drive_passage.h"
#include "plan_common/speed/st_speed/open_loop_speed_limit.h"
#include "planner/speed_optimizer/st_graph_defs.h"

namespace st {
namespace planning {

void LaneChangeSpeedDecider(const double& av_speed, const double& max_acc,
                            OpenLoopSpeedLimit* open_loop_speed_limit);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_SPEED_LIMIT_DECIDER_H_
