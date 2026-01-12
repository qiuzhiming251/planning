#include "decider/initializer/astar_motion_searcher.h"
#include "decider/initializer/astar_motion_searcher_util.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/log_data.h"

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
    const double max_accumulated_s) {
  uint64_t result_index = 0;
  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  const std::string log_key = prefix + "search_result";
  const auto& astar_search_config = initializer_params.astar_search_config();
  const auto& nodes_layers = geom_graph.nodes_layers();
  // uint64_t node_index = astar_search_config.node_index();
  // prepare nodes map
  absl::flat_hash_map<uint64_t, AStarSearchNode>
      nodes_map;  // index from (x, y, v, t);
  std::vector<std::unique_ptr<MotionForm>> motion_forms;
  // prepare astar node & queue
  AstarPriorityQueue open_queue;
  AStarSearchNode start_search_node;
  start_search_node.feature_costs.resize(
      traj_output.cost_provider->cost_names().size(), 0.0);
  start_search_node.geometry_node_idx =
      nodes_layers[0][start_node_idx_on_first_layer];
  start_search_node.is_start_node = true;
  start_search_node.is_open = true;
  start_search_node.is_close = false;
  start_search_node.index = 1;
  start_search_node.h_cost = 0.0;
  start_search_node.g_cost = 0.0;
  start_search_node.total_cost = 0.0;
  start_search_node.v_limit = init_action_state.v;
  start_search_node.v_ref = init_action_state.v;

  // TODO: check AStar search goal s, whether need to be based on SceneType
  double goal_s = std::min(max_accumulated_s, min_stop_s);
  double heurstic_s = goal_s;
  const double ego_s =
      geom_graph.GetNode(start_search_node.geometry_node_idx).accumulated_s;
  Log2DDS::LogDataV2(log_key,
                     absl::StrCat("scene_type:", (int)init_scene_type,
                                  ", ego s:", ego_s, ", goal s:", goal_s));

  start_search_node.total_cost = start_search_node.h_cost;
  // begin to loop over astar search
  uint64_t index_pred = start_search_node.index;
  uint64_t index_succ;
  // // for debug
  // std::vector<Vec2d> debug_node_xy_vec;
  open_queue.push(
      std::pair<uint64_t, double>(index_pred, start_search_node.total_cost));
  nodes_map[index_pred] = start_search_node;
  int iterations = 0;
  std::set<int> candidate_edge_end_nodes;
  uint64_t candidate_node_index = start_search_node.index + 1;
  while (!open_queue.empty()) {
    const auto& node = open_queue.top();
    index_pred = node.first;
    auto& node_pred = nodes_map[index_pred];
    iterations++;
    // LOG_INFO << prefix + " node cost ::: " << node.second;
    const auto& space_node = geom_graph.GetNode(node_pred.geometry_node_idx);

    if (kEnableAstarSearchDebug) {
      Log2DDS::LogDataV2(
          log_key,
          absl::StrFormat(
              "#### iter %d, select node %d, index %d, motion a "
              "%.3f, end v %.3f, cost %.3f (h:%.3f, g:%.3f)",
              iterations, node_pred.geometry_node_idx.value(), node_pred.index,
              node_pred.is_start_node
                  ? init_action_state.v
                  : node_pred.motion_form->GetStartMotionState().a,
              node_pred.is_start_node
                  ? init_action_state.a
                  : node_pred.motion_form->GetEndMotionState().v,
              node_pred.total_cost, node_pred.h_cost, node_pred.g_cost));
    }

    if (iterations > astar_search_config.max_search_iteration()) {
      Log2DDS::LogDataV2(log_key, "Reach max iteration, quit...");
      break;
    }

    if (nodes_map[index_pred].is_close) {
      open_queue.pop();
      continue;
    }

    if (iterations != 1) {
      std::string reach_goal_debug_str = "";
      if (ReachGoal(node_pred, goal_s, ego_s, start_point.v(),
                    &reach_goal_debug_str)) {
        Log2DDS::LogDataV2(
            log_key, absl::StrFormat("Reach goal, iterations: %d, %s",
                                     iterations, reach_goal_debug_str.c_str()));
        result_index = index_pred;
        break;
      } else {
        if (kEnableAstarSearchDebug) {
          Log2DDS::LogDataV2(
              log_key,
              absl::StrFormat("  -- %s", reach_goal_debug_str.c_str()));
        }
      }
    }

    // Expansion of node
    nodes_map[index_pred].is_close = true;
    open_queue.pop();

    absl::flat_hash_map<uint64_t, AStarSearchNode> candidate_nodes;
    std::string open_string_debug;

    CreateSuccessorNodes(
        init_action_state, path_sl, node_pred, *traj_output.cost_provider,
        geom_graph, *traj_output.ref_speed_table, motion_constraint_params,
        initializer_params, heurstic_s, init_scene_type, is_lane_change,
        plan_id, motion_forms, candidate_nodes, &candidate_node_index);

    if (kEnableAstarSearchDebug) {
      for (const auto& [astar_node_index, astar_node] : candidate_nodes) {
        const auto end_node_idx = astar_node.geometry_node_idx;
        const auto end_node_idx_value = end_node_idx.value();
        if (candidate_edge_end_nodes.find(end_node_idx_value) ==
            candidate_edge_end_nodes.end()) {
          candidate_edge_end_nodes.insert(end_node_idx_value);
          std::vector<double> xs, ys;
          xs.emplace_back(geom_graph.GetNode(end_node_idx).xy.x());
          ys.emplace_back(geom_graph.GetNode(end_node_idx).xy.y());
          Log2DDS::LogPointsV3(
              absl::StrCat(prefix, "candidate_end_node_", end_node_idx_value),
              Log2DDS::kWhite, {}, xs, ys, 15.0);
        }
      }
    }

    CheckAndInsertOpenQueue(geom_graph, node_pred, plan_id, candidate_nodes,
                            nodes_map, open_queue);
  }
  if (open_queue.empty()) {
    Log2DDS::LogDataV2(
        log_key, absl::StrCat("OpenQueue empty, iterations: ", iterations));
  }

  if (result_index == 0) {
    return absl::NotFoundError("No trajectories found.");
  }

  traj_output.feature_costs = nodes_map[result_index].feature_costs;
  traj_output.total_cost = nodes_map[result_index].total_cost;

  std::vector<ApolloTrajectoryPointProto> traj_points;
  if (!ConstructTrajFromLastIndex(nodes_map, result_index, prefix,
                                  traj_points)) {
    return absl::NotFoundError("No trajectories found.");
  }

  if (kEnableAstarSearchDebug) {
    Log2DDS::LogDataV2(
        log_key,
        absl::StrFormat(
            "Final raw traj size:%u, end(s:%.3f, t:%.2f, v:%.3f, a:%.3f)",
            traj_points.size(), traj_points.back().path_point().s(),
            traj_points.back().relative_time(), traj_points.back().v(),
            traj_points.back().a()));
  }

  return traj_points;
}
}  // namespace st::planning
