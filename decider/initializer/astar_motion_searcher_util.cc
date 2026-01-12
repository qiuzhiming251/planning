#include "decider/initializer/astar_motion_searcher_util.h"
#include "decider/initializer/dp_motion_searcher_util.h"
#include "decider/initializer/motion_search_util.h"
#include "plan_common/log_data.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/util/path_util.h"

namespace st::planning {

void ComputeHCost(const CostProvider& cost_provider,
                  const PathSlBoundary& path_sl,
                  const RefSpeedTable& ref_speed_table, double heuristic_s,
                  AStarSearchNode* ptr_search_node) {
  const double diff_t =
      kInitializerTrajectoryTimeHorizon - ptr_search_node->accumulated_t;
  const auto& motion_form_end_state =
      ptr_search_node->motion_form->GetEndMotionState();
  const auto& end_state = (diff_t < kEpsilon)
                              ? ptr_search_node->motion_form->State(
                                    diff_t + motion_form_end_state.t)
                              : motion_form_end_state;

  const double start_accumulated_t =
      ptr_search_node->accumulated_t - motion_form_end_state.t;
  const auto [v_limit, v_ref] = ref_speed_table.LookUpRefSpeed(
      start_accumulated_t + end_state.t, end_state.accumulated_s);
  ptr_search_node->v_limit = v_limit;
  ptr_search_node->v_ref = v_ref;

  double diff_s = std::max(heuristic_s - end_state.accumulated_s, 0.0);
  double diff_l = std::fabs(
      end_state.l - path_sl.QueryReferenceCenterL(end_state.accumulated_s));
  double diff_speed =
      std::fabs(ptr_search_node->motion_form->GetEndMotionState().v - v_ref);

  constexpr double kLonOffsetHeuristicWeight = 20.0;
  constexpr double kLatOffsetHeuristicWeight = 60.0;
  constexpr double kRefSpeedHeuristicWeight = 10.0;

  ptr_search_node->heuristic_costs.clear();
  ptr_search_node->heuristic_costs.emplace_back(kLonOffsetHeuristicWeight *
                                                diff_s);
  ptr_search_node->heuristic_costs.emplace_back(kLatOffsetHeuristicWeight *
                                                diff_l);
  ptr_search_node->heuristic_costs.emplace_back(kRefSpeedHeuristicWeight *
                                                diff_speed);

  ptr_search_node->h_cost =
      absl::c_accumulate(ptr_search_node->heuristic_costs, 0.0);
}

void ComputeGCost(const CostProvider& cost_provider,
                  AStarSearchNode* ptr_search_node) {
  ptr_search_node->feature_costs.resize(cost_provider.weights().size());
  double start_t =
      std::max(0.0, ptr_search_node->accumulated_t -
                        ptr_search_node->motion_form->GetEndMotionState().t);
  ptr_search_node->ignored_trajs = cost_provider.ComputeInteractiveDpCost(
      start_t, ptr_search_node->motion_form, ptr_search_node->ignored_trajs,
      absl::MakeSpan(ptr_search_node->feature_costs));
}

void CreateSuccessorNodes(
    const MotionState& init_action_state, const PathSlBoundary& path_sl,
    const AStarSearchNode& pred_search_node, const CostProvider& cost_provider,
    const GeometryGraph& geom_graph, const RefSpeedTable& ref_speed_table,
    const MotionConstraintParamsProto& motion_constraint_params,
    const InitializerConfig& initializer_params, double heurstic_s,
    const InitializerSceneType init_scene_type, bool is_lane_change,
    int plan_id, std::vector<std::unique_ptr<MotionForm>>& motion_forms,
    absl::flat_hash_map<uint64_t, AStarSearchNode>& candidate_nodes,
    uint64_t* node_index) {
  const auto& outgoing_edge_idxs =
      geom_graph.GetOutgoingEdges(pred_search_node.geometry_node_idx);
  candidate_nodes.reserve((kAccelerationSamplePoints.size() + 5) *
                          outgoing_edge_idxs.size());
  for (const auto& outgoing_edge_idx : outgoing_edge_idxs) {
    const auto& geom_edge = geom_graph.GetEdge(outgoing_edge_idx);
    if (!geom_graph.IsActive(outgoing_edge_idx)) {
      continue;
    }
    // Sample start from this action node, based on this geom_edge, sampling
    // by different accelerations.
    SampleDynamicActions(init_action_state, path_sl, pred_search_node,
                         geom_edge, geom_graph, ref_speed_table,
                         /*ignored_trajs=*/{}, motion_constraint_params,
                         initializer_params, cost_provider,
                         /*sample_const_v=*/false, heurstic_s, init_scene_type,
                         is_lane_change, plan_id, motion_forms, candidate_nodes,
                         node_index);
  }
}

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
    uint64_t* node_index) {
  const auto& pred_action_state =
      pred_search_node.is_start_node
          ? init_action_state
          : pred_search_node.motion_form->GetEndMotionState();
  const double v0 = pred_action_state.v;
  const double t0 = pred_search_node.accumulated_t;
  const double remain_s_to_goal =
      heurstic_s - geom_graph.GetNode(geom_edge.start).accumulated_s;
  // expand action and get cost at the same time. if cannot get cost from cache,
  // record this idx and compute cost outside this function.

  // Create keys for the cost fetching.
  absl::flat_hash_set<MotionEdgeKey> unrepeated_keys;
  const auto expand_action_by_a = [&unrepeated_keys, &geom_edge, t0](
                                      double v0, double a0) {
    unrepeated_keys.emplace(MotionEdgeKey(a0, v0, t0, geom_edge.index));
  };

  CHECK_GE(v0, 0.0);
  const double a_max = motion_constraint_params.max_acceleration();
  const double a_min = motion_constraint_params.max_deceleration();
  CHECK_LT(a_min, 0.0);
  CHECK_GT(a_max, 0.0);
  const double v_limit =
      Mph2Mps(motion_constraint_params.default_speed_limit());     // m/s
  const double reciprocal_s = 1.0 / geom_edge.geometry->length();  // 1/m
  const double pos_a_limit = (Sqr(v_limit) - Sqr(v0)) * reciprocal_s * 0.5;
  // const double stop_a = -Sqr(v0) * reciprocal_s * 0.5;

  double a_lower = a_min;  // std::max(a_min, stop_a);
  double a_upper = std::min(pos_a_limit, a_max);
  std::set<double> acc_samples;
  if (!sample_const_v) {
    if (FLAGS_planner_initializer_enable_clip) {
      const double a0 = pred_action_state.a;
      constexpr double kAccVariationRange = 1.0;  // m/s^2
      a_lower = std::max(a_lower, a0 - kAccVariationRange);
      a_upper = std::min(a_upper, a0 + kAccVariationRange);
    }

    if (remain_s_to_goal < geom_edge.geometry->length() + kEpsilon) {
      const auto [v_limit, v_ref] = ref_speed_table.LookUpRefSpeed(
          kInitializerTrajectoryTimeHorizon, heurstic_s);
      const double acc_to_goal =
          (Sqr(v_ref) - Sqr(v0)) * 0.5 / remain_s_to_goal;
      if (acc_to_goal > a_min && acc_to_goal < a_upper) {
        acc_samples.insert(acc_to_goal);
        if (kEnableAstarSearchDebug) {
          const auto& prefix = Log2DDS::TaskPrefix(plan_id);
          Log2DDS::LogDataV2(
              prefix + "search_result",
              absl::StrFormat("==== s:%.3f -> %.3f, edge:%d->%d, "
                              "remain_s:%.3f, v0:%.3f, end_ref_v:%.3f, a:%.3f",
                              geom_graph.GetNode(geom_edge.start).accumulated_s,
                              heurstic_s, geom_edge.start.value(),
                              geom_edge.end.value(), remain_s_to_goal, v0,
                              v_ref, acc_to_goal));
        }
      } else {
        if (kEnableAstarSearchDebug) {
          const auto& prefix = Log2DDS::TaskPrefix(plan_id);
          Log2DDS::LogDataV2(
              prefix + "search_result",
              absl::StrFormat("==== fail to add, s:%.3f -> %.3f, edge:%d->%d, "
                              "remain_s:%.3f, v0:%.3f, end_ref_v:%.3f, a:%.3f",
                              geom_graph.GetNode(geom_edge.start).accumulated_s,
                              heurstic_s, geom_edge.start.value(),
                              geom_edge.end.value(), remain_s_to_goal, v0,
                              v_ref, acc_to_goal));
        }
      }
    }

    const auto a_begin =
        std::lower_bound(kAccelerationSamplePoints.begin(),
                         kAccelerationSamplePoints.end(), a_lower);
    const auto a_end =
        std::lower_bound(a_begin, kAccelerationSamplePoints.end(), a_upper);
    // Sample the valid accelerations on the sampling grid.
    for (auto it = a_begin; it != a_end; ++it) {
      acc_samples.insert(*it);
    }
  }
  acc_samples.insert(a_lower);
  acc_samples.insert(a_upper);
  acc_samples.insert(a_min);
  acc_samples.insert(0.0);
  for (const double acc : acc_samples) {
    expand_action_by_a(v0, acc);
  }

  // Prepare containers.
  std::vector<MotionEdgeKey> keys;
  keys.reserve(unrepeated_keys.size());
  for (auto& key : unrepeated_keys) {
    keys.push_back(key);
  }
  // succ_search_nodes.reserve(keys.size());
  for (int i = 0; i < keys.size(); ++i) {
    const auto& key = keys[i];
    motion_forms.emplace_back(std::make_unique<ConstAccelMotion>(
        key.v0(), key.a0(), geom_edge.geometry));
    double accumulated_t = pred_search_node.accumulated_t +
                           motion_forms.back()->GetEndMotionState().t;

    auto new_node = AStarSearchNode({.accumulated_t = accumulated_t,
                                     .s_to_init = pred_search_node.s_to_init,
                                     // .index = (*node_index)++,
                                     .pred_index = pred_search_node.index,
                                     .geometry_node_idx = geom_edge.end,
                                     .geometry_edge_idx = geom_edge.index,
                                     .motion_form = motion_forms.back().get(),
                                     .ignored_trajs = ignored_trajs});
    new_node.s_to_init += new_node.motion_form->GetEndMotionState().s;
    new_node.SetIndex(geom_graph, initializer_params);
    ComputeGCost(cost_provider, &new_node);
    new_node.g_cost =
        absl::c_accumulate(new_node.feature_costs, pred_search_node.g_cost);
    new_node.feature_costs =
        AddCost(new_node.feature_costs, pred_search_node.feature_costs);
    ComputeHCost(cost_provider, path_sl, ref_speed_table, heurstic_s,
                 &new_node);
    new_node.total_cost = new_node.g_cost + new_node.h_cost;

    if (kEnableAstarSearchDebug) {
      std::string hcost_info_str = "";
      for (int c = 0; c < new_node.heuristic_costs.size(); ++c) {
        hcost_info_str +=
            absl::StrCat(c == 0 ? "" : ", ", new_node.heuristic_costs[c]);
      }
      std::string gcost_info_str = "";
      const auto gcost_names = cost_provider.cost_names();
      for (int c = 0; c < gcost_names.size(); ++c) {
        gcost_info_str += absl::StrCat(c == 0 ? "" : ", ", gcost_names[c], ":",
                                       new_node.feature_costs[c]);
      }
      const auto& prefix = Log2DDS::TaskPrefix(plan_id);
      const auto& start_state = new_node.motion_form->GetStartMotionState();
      const auto& end_state = new_node.motion_form->GetEndMotionState();
      Log2DDS::LogDataV2(
          prefix + "search_result",
          absl::StrFormat(
              "edge:%d->%d, idx:%d, index:%u, a:%.3f(%.3f), "
              "end_v:%.3f(ref:%.3f, limit:%.3f), cost:%.3f, s_to_init:%.3f, "
              "accu[(t:%.3f, s:%.3f)->(t:%.3f, s:%.3f)], rel[(t:%.3f, "
              "s:%.3f)->(t:%.3f, s:%.3f)], edge_end[s_accu:%.3f, s_by_t:%.3f], "
              "h:%.3f(%s), g:%.3f, %s",
              geom_edge.start.value(), geom_edge.end.value(),
              succ_search_nodes.size(), new_node.index, start_state.a, key.a0(),
              end_state.v, new_node.v_ref, new_node.v_limit,
              new_node.total_cost, new_node.s_to_init,
              new_node.accumulated_t - end_state.t, start_state.accumulated_s,
              new_node.accumulated_t, end_state.accumulated_s, start_state.t,
              start_state.s, end_state.t, end_state.s,
              geom_graph.GetNode(geom_edge.end).accumulated_s,
              new_node.motion_form
                  ->State(kInitializerTrajectoryTimeHorizon -
                          new_node.accumulated_t)
                  .accumulated_s,
              new_node.h_cost, hcost_info_str.c_str(), new_node.g_cost,
              gcost_info_str.c_str()));
    }

    auto node_iter = succ_search_nodes.find(new_node.index);
    if (node_iter == succ_search_nodes.end()) {
      succ_search_nodes[new_node.index] = new_node;
    } else if (new_node.total_cost + kMathEpsilon <
               node_iter->second.total_cost) {
      node_iter->second = new_node;
    }
  }
}

bool ReachGoal(const AStarSearchNode& current_node, const double goal_s,
               const double start_node_s, const double start_v,
               std::string* debug_str) {
  const auto& cur_action_state = current_node.motion_form->GetEndMotionState();
  bool reach_goal =
      current_node.accumulated_t + kEpsilon > kInitializerTrajectoryTimeHorizon;
  if (debug_str != nullptr) {
    *debug_str = absl::StrFormat(
        "ego_s:%.3f, goal_s:%.3f, ego_v:%.3f, end_state(s:%.3f, l:%.3f, "
        "t:%.3f)",
        start_node_s, goal_s, start_v, cur_action_state.accumulated_s,
        cur_action_state.l, current_node.accumulated_t);
  }
  return reach_goal;
}

void CheckAndInsertOpenQueue(
    const GeometryGraph& geom_graph, const AStarSearchNode& node_pred,
    int plan_id,
    absl::flat_hash_map<uint64_t, AStarSearchNode>& candidate_nodes,
    absl::flat_hash_map<uint64_t, AStarSearchNode>& nodes_map,
    AstarPriorityQueue& open_queue) {
  for (auto& [succ_index, candidate_node] : candidate_nodes) {
    if (nodes_map.find(succ_index) != nodes_map.end()) {
      // check cost
      if (candidate_node.g_cost + 1e-5 > nodes_map[succ_index].g_cost) {
        if (kEnableAstarSearchDebug) {
          const auto& prefix = Log2DDS::TaskPrefix(plan_id);
          Log2DDS::LogDataV2(
              prefix + "search_result",
              absl::StrFormat(
                  "remove edge %d->%d, index %u, g_cost:%.3f > %.3f",
                  geom_graph.GetEdge(candidate_node.geometry_edge_idx)
                      .start.value(),
                  geom_graph.GetEdge(candidate_node.geometry_edge_idx)
                      .end.value(),
                  candidate_node.index, candidate_node.g_cost,
                  nodes_map[succ_index].g_cost));
        }
        continue;
      } else if (candidate_node.pred_index == candidate_node.index &&
                 candidate_node.g_cost <= node_pred.g_cost) {
        // if successor is in the same cell and the G value is lower,
        // set its predecessor to the predecessor of predecessor, renew
        // father

        // if (!node_pred.is_start_node) {
        //   candidate_node.pred_index = node_pred.pred_index;
        // }
      }
    }

    // put successor on open list
    candidate_node.is_open = true;
    // store in hashmap
    nodes_map[succ_index] = std::move(candidate_node);
    // push on priority_queue
    open_queue.emplace(std::pair<uint64_t, double>(
        succ_index, nodes_map[succ_index].total_cost));
  }
}

bool ConstructTrajFromLastIndex(
    const absl::flat_hash_map<uint64_t, AStarSearchNode>& nodes_map,
    uint64_t goal_index, std::string task_prefix,
    std::vector<ApolloTrajectoryPointProto>& traj_points) {
  std::vector<const MotionForm*> motions;
  uint64_t node_index = goal_index;
  LOG_INFO << "--------- ConstructTrajFromLastIndex ---------";
  std::string best_node;
  best_node += "best node is ";
  std::string motion_log;
  while (node_index != 1) {
    // LOG_INFO << "lhhh node_index: " << node_index;
    best_node += absl::StrFormat("%d ", node_index);
    const auto& node = nodes_map.at(node_index);
    motion_log +=
        absl::StrFormat("motion %d cost:%.3f, g_cost:%.3f, h_cost:%.3f | ",
                        node_index, node.total_cost, node.g_cost, node.h_cost);

    motions.emplace_back(node.motion_form);
    node_index = node.pred_index;
  }
  std::string log_key = absl::StrCat(task_prefix, "search_result");
  Log2DDS::LogDataV2(log_key, motion_log);
  Log2DDS::LogDataV2(log_key, best_node);
  std::reverse(motions.begin(), motions.end());
  if (motions.empty()) {
    LOG_INFO << "empty motions";
    Log2DDS::LogDataV2(log_key, "empty motions");
    return false;
  } else {
    LOG_INFO << "size of astarmotions: " << motions.size();
  }

  ResampleTrajectoryPoints(motions, traj_points);

  if (traj_points.empty()) {
    Log2DDS::LogDataV2(
        log_key,
        absl::StrFormat("resample fail, motions size: %d", motions.size()));
    return false;
  }

  // extend the trajectory to 8s if it will stop
  constexpr double kMinSpeed = 0.1;
  while (traj_points.size() < kInitializerTrajectorySteps &&
         traj_points.back().v() + 1e-6 < kMinSpeed) {
    const auto& last_pt = traj_points.back();
    auto& new_pt = traj_points.emplace_back();
    new_pt = last_pt;
    new_pt.set_v(0.0);
    new_pt.set_a(0.0);
    new_pt.set_j(0.0);
    new_pt.set_relative_time(last_pt.relative_time() + kTrajectoryTimeStep);
  }

  return true;
}

}  // namespace st::planning
