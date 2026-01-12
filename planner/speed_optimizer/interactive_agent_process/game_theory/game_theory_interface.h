

#ifndef ONBOARD_PLANNER_SPEED_GAME_THEORY_INTERFACE_H_
#define ONBOARD_PLANNER_SPEED_GAME_THEORY_INTERFACE_H_

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
#include "planner/speed_optimizer/interactive_agent_process/game_theory/game_theory_environment.h"
#include "planner/speed_optimizer/interactive_agent_process/interactive_agent.h"
#include "planner/speed_optimizer/st_graph.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

void GameTheoryEntry(std::vector<gt_result_t>& gt_results,
                     std::vector<InteractiveAgent>& interactive_agents,
                     InteractiveAgent& ego_interactive_info);
}
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
