
#include <algorithm>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>
#include "plan_common/async/parallel_for.h"

#include "plan_common/log.h"

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "decider/initializer/geometry/geometry_graph.h"
#include "decider/initializer/initializer_input.h"
#include "decider/initializer/motion_graph.h"
#include "decider/initializer/motion_graph_cache.h"
#include "decider/initializer/motion_state.h"
#include "decider/initializer/multi_traj_selector.h"
#include "plan_common/async/thread_pool.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"

namespace st::planning {
absl::StatusOr<std::vector<ApolloTrajectoryPointProto>> DPSearchMainLoop(
    const MotionState& sdc_motion, const MotionNodeIndex& sdc_node_idx,
    const InitializerSceneType init_scene_type,
    int start_node_idx_on_first_layer, const GeometryGraph& geom_graph,
    const MotionConstraintParamsProto motion_constraint_params,
    MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    MotionEdgeVector<IgnoreTrajMap>& ignored_trajs_vector,
    MotionGraphCache* cost_cache, ThreadPool* thread_pool,
    SingleTrajInfo& traj_output,
    std::vector<MotionEdgeIndex>& terminated_edge_idxes, const int plan_id);
}  // namespace st::planning