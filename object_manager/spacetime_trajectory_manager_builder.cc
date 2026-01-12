

#include <vector>

#include "absl/types/span.h"
#include "spacetime_trajectory_manager_builder.h"

//#include "lite/check.h"
#include "plan_common/timer.h"
#include "plan_common/planning_macros.h"
#include "drive_passage_filter.h"
#include "low_likelihood_filter.h"
#include "trajectory_filter.h"
#include "lane_attr_type_filter.h"
//#include "planner/planner_manager/planner_flags.h"

namespace st {
namespace planning {

SpacetimeTrajectoryManager BuildSpacetimeTrajectoryManager(
    const SpacetimeTrajectoryManagerBuilderInput& input,
    ThreadPool* thread_pool) {
  TIMELINE("BuildSpacetimeTrajectoryManager");
  std::string function_name =
      Log2DDS::TaskPrefix(input.plan_id) + std::string(__FUNCTION__);
  SCOPED_TRACE(function_name.c_str());
  CHECK_NOTNULL(input.passage);
  CHECK_NOTNULL(input.sl_boundary);
  CHECK_NOTNULL(input.obj_mgr);
  CHECK_NOTNULL(input.lc_state);

  // Build spacetime object manager.
  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_only_use_most_likely_trajectory);
  const DrivePassageFilter drive_passage_filter(
      input.passage, input.sl_boundary, input.lc_state, input.ego_box,
      input.is_on_highway, input.psmm);

  std::vector<const TrajectoryFilter*> filters;
  filters.push_back(&low_likelihood_filter);
  filters.push_back(&drive_passage_filter);
  return SpacetimeTrajectoryManager(filters, input.obj_mgr->planner_objects(),
                                    input.obj_mgr->frame_dropped_objects(),
                                    thread_pool, input.plan_id);
}

}  // namespace planning
}  // namespace st
