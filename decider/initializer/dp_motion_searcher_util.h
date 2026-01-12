

#ifndef ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_UTIL_H_

#include <string>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/async/thread_pool.h"

#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/drive_passage.h"

#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/initializer_output.h"

#include "decider/initializer/motion_state.h"
#include "decider/initializer/motion_graph.h"
#include "decider/initializer/cost_provider.h"
#include "decider/initializer/motion_graph_cache.h"
#include "decider/initializer/motion_search_output.h"
#include "decider/initializer/motion_searcher_defs.h"
#include "decider/initializer/geometry/geometry_graph.h"

namespace st::planning {

std::vector<double> AddCost(absl::Span<const double> vec1,
                            absl::Span<const double> vec2);

void ComputeCost(const CostProvider& cost_provider,
                 DpMotionInfo* ptr_motion_info);

void ComputeDpLeadingObjCost(const CostProvider& cost_provider,
                             DpMotionInfo* ptr_motion_info);

// Sample by acceleration.
void SampleDynamicMotions(
    MotionNodeIndex motion_node_idx, MotionEdgeIndex prev_motion_edge_idx,
    const MotionGraph& motion_graph, const GeometryEdge& geom_edge,
    const IgnoreTrajMap& ignored_trajs,
    const PlannerParamsProto& planner_params, const CostProvider& cost_provider,
    const MotionGraphCache& cost_cache, bool sample_const_v,
    std::vector<DpMotionInfo>* ptr_motions,
    std::vector<NewCacheInfo>* new_motion_forms_cache, ThreadPool* thread_pool);

// Special handling for motion expansion from start node (planning start state).
std::vector<DpMotionInfo> ExpandStartMotionEdges(
    GeometryNodeIndex geom_node_idx, MotionNodeIndex motion_node_idx,
    const GeometryGraph& geometry, const MotionGraph& motion_graph,
    const MotionConstraintParamsProto& motion_constraint_params,
    const CostProvider& cost_provider, const MotionGraphCache& cost_cache,
    std::vector<NewCacheInfo>* new_motion_forms, ThreadPool* thread_pool);

// Expand motions from an intermediate geometry node.
std::vector<DpMotionInfo> ExpandMotionEdges(
    GeometryNodeIndex geom_node_idx,
    const std::vector<MotionEdgeIndex>& motion_edge_idxes,
    const MotionEdgeVector<IgnoreTrajMap>& ignored_trajs_vector,
    const GeometryGraph& geometry, const MotionGraph& motion_graph,
    const MotionConstraintParamsProto& motion_constraint_params,
    const CostProvider& cost_provider, bool sample_const_v,
    const MotionGraphCache& cost_cache,
    std::vector<NewCacheInfo>* new_motion_forms, ThreadPool* thread_pool);

std::vector<TrajInfo> TopKTrajectories(
    const MotionGraph& motion_graph,
    absl::Span<const MotionEdgeIndex> terminated_edge_idxes,
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    int k_top_trajectories);

BestEdgeInfo FindBestEdge(
    const CostProvider& cost_provider, const MotionState& sdc_motion,
    MotionNodeIndex sdc_node_index, const GeometryNodeIndex& sdc_geom_node,
    MotionGraph* mutable_motion_graph, MotionGraphCache* cost_cache,
    MotionEdgeVector<MotionSearchOutput::SearchCost>* ptr_mutable_search_costs,
    MotionEdgeVector<IgnoreTrajMap>* ptr_mutable_ignore_traj_map,
    std::vector<MotionEdgeIndex>* ptr_mutable_terminated_idxes);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_UTIL_H_
