

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_

#include <limits>

#include "planner/trajectory_optimizer/problem/mixed_fourth_order_bicycle.h"
#include "planner/planner_manager/planner_defs.h"

namespace st {
namespace planning {
namespace optimizer {

using Mfob = MixedFourthOrderBicycle;

struct LeadingInfo {
  double s = std::numeric_limits<double>::infinity();
  double v = std::numeric_limits<double>::infinity();
};

}  // namespace optimizer
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_
