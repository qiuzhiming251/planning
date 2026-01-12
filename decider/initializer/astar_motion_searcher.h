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
absl::StatusOr<std::vector<ApolloTrajectoryPointProto>> AStarSearchMainLoop(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::string>& leading_trajs,
    const GeometryGraph& geom_graph,
    const ApolloTrajectoryPointProto& start_point,
    const MotionConstraintParamsProto motion_constraint_params,
    const InitializerConfig& initializer_params,
    const VehicleGeometryParamsProto& vehicle_geom,
    const MotionState& init_action_state, bool is_lane_change,
    const InitializerSceneType& init_scene_type,
    int start_node_idx_on_first_layer, const double min_stop_s,
    const int plan_id, SingleTrajInfo& traj_output,
    const double max_accumulated_s);
}  // namespace st::planning