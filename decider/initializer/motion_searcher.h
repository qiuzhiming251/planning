

#ifndef ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_
#define ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_

#include "absl/status/statusor.h"

#include "plan_common/async/thread_pool.h"

#include "object_manager/st_inference/initializer_output.h"

#include "decider/initializer/initializer_input.h"
#include "decider/initializer/motion_search_output.h"

namespace st::planning {
// The function searches space-time trajectory with dynamic programming. In
// order to accelerate computing, time and speed are discretized to several
// intervals, similar to the spirit of Hybrid A-Star. This function is only
// tested for curvy geometry graph. Do not use it for now for straight
// geometry graph.
absl::StatusOr<MotionSearchOutput> SearchForRawTrajectory(
    const MotionSearchInput& input, ThreadPool* thread_pool, int plan_id);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_
