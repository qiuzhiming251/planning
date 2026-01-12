

#include "planner/speed_optimizer/interactive_agent_process/game_theory/game_theory_interface.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/meta/type_traits.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "plan_common/log.h"
#include "gflags/gflags.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/intelligent_driver_model.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/planner_object.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/overlap_info.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_point.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/speed/st_speed/speed_point.h"
#include "plan_common/speed/st_speed/speed_profile.h"
#include "plan_common/timer.h"
#include "plan_common/trajectory_util.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_defs.h"
#include "predictor/prediction_object_state.h"
#include "planner/speed_optimizer/decider/interaction_util.h"
#include "planner/speed_optimizer/decider/post_st_boundary_modifier.h"
#include "planner/speed_optimizer/decider/pre_brake_decider.h"
#include "planner/speed_optimizer/decider/st_boundary_modifier_util.h"
#include "planner/speed_optimizer/empty_road_speed.h"
#include "planner/speed_optimizer/gridded_svt_graph.h"
#include "planner/speed_optimizer/interactive_agent_process/game_theory/game_theory_environment.h"
#include "planner/speed_optimizer/interactive_agent_process/interactive_agent.h"
#include "planner/speed_optimizer/st_graph_data.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {

void reset_game_theory_interaction_result(gt_result_t* gt_result) {
  if (nullptr == gt_result) return;

  gt_result->is_valid = false;
  gt_result->is_solution = false;
  gt_result->gt_interaction_matrix.col = 0;
  gt_result->gt_interaction_matrix.row = 0;
}

void GameTheoryEntry(std::vector<gt_result_t>& gt_results,
                     std::vector<InteractiveAgent>& interactive_agents,
                     InteractiveAgent& ego_interactive_info) {
  std::vector<Game_Theory_Interaction_Environment> gt_env;

  for (int i = 0; i < interactive_agents.size(); i++) {
    Game_Theory_Interaction_Environment tmp_gt_env;
    gt_env.emplace_back(tmp_gt_env);
  }

  for (int i = 0; i < interactive_agents.size(); i++) {
    auto& curr_gt_env = gt_env[i];
    curr_gt_env.InitGameTheoryEnvironment(&ego_interactive_info,
                                          &interactive_agents[i]);
    curr_gt_env.BuildUpGameEnvironment();
    gt_result_t tmp_gt_result;
    curr_gt_env.StartStackelbergGameProcess(&tmp_gt_result);
    gt_results.emplace_back(tmp_gt_result);
  }
}
}  // namespace st::planning
