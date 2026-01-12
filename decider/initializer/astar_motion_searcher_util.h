#ifndef _ST_PLANNING_DECIDER_INITIALIZER_ATAR_MOTION_SEARCHER_UTIL_H_
#define _ST_PLANNING_DECIDER_INITIALIZER_ATAR_MOTION_SEARCHER_UTIL_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/async/thread_pool.h"
#include "decider/initializer/cost_provider.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "decider/initializer/motion_graph.h"
#include "decider/initializer/motion_graph_cache.h"
#include "decider/initializer/motion_searcher_defs.h"
#include "decider/initializer/motion_state.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/initializer_output.h"
#include "router/drive_passage_builder.h"
#include "modules/cnoa_pnc/planning/proto/initializer_config.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {

// compute heuristic cost
void ComputeHCost(const CostProvider& cost_provider,
                  const PathSlBoundary& path_sl,
                  const RefSpeedTable& ref_speed_table, double heuristic_s,
                  AStarSearchNode* ptr_search_node);

// compute general cost
void ComputeGCost(const CostProvider& cost_provider,
                  AStarSearchNode* ptr_search_node);

// sample successor nodes based on current node
void CreateSuccessorNodes(
    const MotionState& init_action_state, const PathSlBoundary& path_sl,
    const AStarSearchNode& pred_search_node, const CostProvider& cost_provider,
    const GeometryGraph& geom_graph, const RefSpeedTable& ref_speed_table,
    const MotionConstraintParamsProto& motion_constraint_params,
    const InitializerConfig& initializer_params, double heurstic_s,
    const InitializerSceneType init_scene_type, bool is_lane_change,
    int plane_id, std::vector<std::unique_ptr<MotionForm>>& motion_forms,
    absl::flat_hash_map<uint64_t, AStarSearchNode>& candidate_nodes,
    uint64_t* node_index);

// sample actions based on different acceleration
void SampleDynamicActions(
    const MotionState& init_action_state, const PathSlBoundary& path_sl,
    const AStarSearchNode& pred_search_node, const GeometryEdge& geom_edge,
    const GeometryGraph& geom_graph, const RefSpeedTable& ref_speed_table,
    const IgnoreTrajMap& ignored_trajs,
    const MotionConstraintParamsProto& motion_constraint_params,
    const InitializerConfig& initializer_params,
    const CostProvider& cost_provider, bool sample_const_v, double heurstic_s,
    const InitializerSceneType init_scene_type, bool is_lane_change,
    int plan_id, std::vector<std::unique_ptr<MotionForm>>& motion_forms,
    absl::flat_hash_map<uint64_t, AStarSearchNode>& succ_search_nodes,
    uint64_t* node_index);

// check if the current node reach the goal
bool ReachGoal(const AStarSearchNode& current_node, const double goal_s,
               const double start_node_s, const double start_v,
               std::string* debug_str = nullptr);

// check if the current node is needed to be insert open queue
void CheckAndInsertOpenQueue(
    const GeometryGraph& geom_graph, const AStarSearchNode& node_pred,
    int plan_id,
    absl::flat_hash_map<uint64_t, AStarSearchNode>& candidate_nodes,
    absl::flat_hash_map<uint64_t, AStarSearchNode>& nodes_map,
    AstarPriorityQueue& open_queue);

// construct trajectory from the result index
bool ConstructTrajFromLastIndex(
    const absl::flat_hash_map<uint64_t, AStarSearchNode>& nodes_map,
    uint64_t result_index, std::string task_prefix,
    std::vector<ApolloTrajectoryPointProto>& traj_points);

}  // namespace st::planning

#endif  // _ST_PLANNING_DECIDER_INITIALIZER_ATAR_MOTION_SEARCHER_UTIL_H_
