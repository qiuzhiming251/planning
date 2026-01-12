

#ifndef ONBOARD_PLANNER_SPEED_INTERACTIVE_POST_PROCESSING_H_
#define ONBOARD_PLANNER_SPEED_INTERACTIVE_POST_PROCESSING_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "plan_common/async/thread_pool.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "planner/speed_optimizer/decider/post_st_boundary_modifier.h"
#include "planner/speed_optimizer/interactive_agent_process/game_theory/game_theory_environment.h"
#include "planner/speed_optimizer/st_graph.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

void InteractivePostProcessing(
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<gt_result_t>& gt_results, const DiscretizedPath& path,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    double ego_acc_interval = 0.2);

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_POST_PROCESSING_H_
