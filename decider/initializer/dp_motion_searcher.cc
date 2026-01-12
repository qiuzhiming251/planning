#include "decider/initializer/motion_search_util.h"
#include "decider/initializer/dp_motion_searcher.h"
#include "decider/initializer/motion_searcher_defs.h"
#include "decider/initializer/dp_motion_searcher_util.h"
#include "plan_common/log_data.h"

namespace st::planning {
void FillTrajDebugInfo(
    const MotionGraph& motion_graph,
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    std::vector<MotionEdgeIndex> terminated_edge_idxes,
    SingleTrajDebugInfo* debug_info) {
  constexpr int kMaxTopTrajectoryNum = 50;
  const int k_top_traj_num = std::min(
      static_cast<int>(terminated_edge_idxes.size()), kMaxTopTrajectoryNum);
  const auto top_k_traj_info = TopKTrajectories(
      motion_graph, terminated_edge_idxes, search_costs, k_top_traj_num);

  auto& top_k_trajs = debug_info->top_k_trajs;
  top_k_trajs.reserve(top_k_traj_info.size());
  auto& top_k_total_costs = debug_info->top_k_total_costs;
  top_k_total_costs.reserve(top_k_traj_info.size());
  auto& top_k_edges = debug_info->top_k_edges;
  top_k_edges.reserve(top_k_traj_info.size());
  for (const auto& traj_info : top_k_traj_info) {
    top_k_total_costs.push_back(traj_info.total_cost);
    top_k_edges.push_back(traj_info.idx);
    top_k_trajs.push_back(
        ConstructTrajFromLastEdge(motion_graph, traj_info.idx));
  }
  debug_info->terminated_edge_idxes = std::move(terminated_edge_idxes);
}

void InsertMotionNodes(
    const std::vector<GeometryNodeIndex>& nodes_layer,
    const MotionNodeIndex& sdc_node_idx,
    const GeometryNodeVector<absl::flat_hash_map<DpMotionSample, DpMotionInfo>>&
        opt_motion_samples,
    MotionGraph* motion_graph,
    std::vector<MotionEdgeIndex>* terminated_edge_idxes,
    GeometryNodeVector<std::vector<MotionEdgeIndex>>* motions_to_expand,
    MotionEdgeVector<MotionSearchOutput::SearchCost>* search_costs,
    MotionEdgeVector<IgnoreTrajMap>* ignored_trajs_vec) {
  for (int i = 0, n = nodes_layer.size(); i < n; ++i) {
    const auto geom_node_idx = nodes_layer[i];
    // best motion so far of a given geometry node.
    const auto& best_motions = opt_motion_samples[geom_node_idx];
    for (const auto& [_, dp_motion_info] : best_motions) {
      MotionState end_motion_state =
          dp_motion_info.motion_form->GetEndMotionState();
      MotionEdgeIndex motion_edge_index;
      const auto prev_motion_edge_index = dp_motion_info.prev_motion_edge_index;
      if (prev_motion_edge_index ==
          MotionEdgeVector<MotionEdge>::kInvalidIndex) {
        // This edge has no previous edge since it connects directly to
        // the start motion node.
        const auto end_node_index =
            motion_graph->AddMotionNode(end_motion_state, geom_node_idx);
        motion_edge_index = motion_graph->AddMotionEdge(
            sdc_node_idx, end_node_index, dp_motion_info.motion_form,
            geom_node_idx, MotionEdgeVector<MotionEdge>::kInvalidIndex);

        search_costs->push_back(MotionSearchOutput::SearchCost{
            .feature_cost = dp_motion_info.costs,
            .cost_to_come = dp_motion_info.sum_cost});
        ignored_trajs_vec->push_back(dp_motion_info.ignored_trajs);
      } else {
        // This edge has previous edge.
        const auto& prev_motion_edge =
            motion_graph->GetMotionEdge(prev_motion_edge_index);
        end_motion_state.t +=
            motion_graph->GetMotionNode(prev_motion_edge.end).state.t;
        const auto end_node_index =
            motion_graph->AddMotionNode(end_motion_state, geom_node_idx);
        motion_edge_index = motion_graph->AddMotionEdge(
            prev_motion_edge.end, end_node_index, dp_motion_info.motion_form,
            geom_node_idx, prev_motion_edge_index);

        const auto& prev_cost = (*search_costs)[prev_motion_edge_index];
        search_costs->push_back(MotionSearchOutput::SearchCost{
            .feature_cost =
                AddCost(dp_motion_info.costs, prev_cost.feature_cost),
            .cost_to_come = dp_motion_info.sum_cost + prev_cost.cost_to_come});
        ignored_trajs_vec->push_back(dp_motion_info.ignored_trajs);
      }
      // Discard super slow state if it fails to reach the end of time
      // horizon.
      if (end_motion_state.v < 0.1 &&
          end_motion_state.t < kInitializerTrajectoryTimeHorizon) {
        continue;
      }
      // We let the motion expansion stop if it reaches the end of time
      // horizon.
      if (end_motion_state.t >= kInitializerTrajectoryTimeHorizon) {
        terminated_edge_idxes->push_back(motion_edge_index);
      } else {
        // Otherwise we add it to the to-be-expanded motions.
        (*motions_to_expand)[geom_node_idx].push_back(motion_edge_index);
      }
    }
  }
}

int UpdateOptimalMotions(
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    const MotionGraphCache& cost_cache,
    std::vector<std::vector<DpMotionInfo>> candidate_motions,
    GeometryNodeVector<absl::flat_hash_map<DpMotionSample, DpMotionInfo>>*
        opt_motion_samples) {
  int motion_count = 0;
  for (auto& candidate_motions_per_node : candidate_motions) {
    motion_count += candidate_motions_per_node.size();

    for (auto& candidate_motion : candidate_motions_per_node) {
      const auto end_geometry_node_index =
          candidate_motion.end_geometry_node_index;
      // best_motion_so_far is motion terminating at this geometry node (at
      // the velocity and time determined by this motion).
      auto& best_motion_so_far = (*opt_motion_samples)[end_geometry_node_index];
      ASSIGN_OR_CONTINUE(candidate_motion.motion_form,
                         cost_cache.GetMotionForm(candidate_motion.key));
      const auto end_state = candidate_motion.motion_form->GetEndMotionState();
      const DpMotionSample dp_motion_sample(end_state.v, end_state.t);
      if (best_motion_so_far.find(dp_motion_sample) ==
          best_motion_so_far.end()) {
        // No motion is added yet to the hashmap, add it.
        // Only record some optimal edges costs to the cache.
        best_motion_so_far[dp_motion_sample] = std::move(candidate_motion);
      } else {
        // A motion is already in the hashmap, compare its cost with the
        // currently evaluating one and keep better.
        const auto& prev_best = best_motion_so_far[dp_motion_sample];
        auto prev_cost = prev_best.sum_cost;
        if (prev_best.prev_motion_edge_index !=
            MotionEdgeVector<MotionEdge>::kInvalidIndex) {
          prev_cost +=
              search_costs[prev_best.prev_motion_edge_index].cost_to_come;
        }
        auto cur_cost = candidate_motion.sum_cost;
        if (candidate_motion.prev_motion_edge_index !=
            MotionEdgeVector<MotionEdge>::kInvalidIndex) {
          cur_cost += search_costs[candidate_motion.prev_motion_edge_index]
                          .cost_to_come;
        }
        if (cur_cost < prev_cost) {
          best_motion_so_far[dp_motion_sample] = std::move(candidate_motion);
        }
      }
    }
  }
  return motion_count;
}

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>> DPSearchMainLoop(
    const MotionState& sdc_motion, const MotionNodeIndex& sdc_node_idx,
    const InitializerSceneType init_scene_type,
    int start_node_idx_on_first_layer, const GeometryGraph& geom_graph,
    const MotionConstraintParamsProto motion_constraint_params,
    MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    MotionEdgeVector<IgnoreTrajMap>& ignored_trajs_vector,
    MotionGraphCache* cost_cache, ThreadPool* thread_pool,
    SingleTrajInfo& traj_output,
    std::vector<MotionEdgeIndex>& terminated_edge_idxes, const int plan_id) {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  // Keep track on best motions per GeometryNode.
  GeometryNodeVector<absl::flat_hash_map<DpMotionSample, DpMotionInfo>>
      opt_motion_samples;
  opt_motion_samples.resize(geom_graph.nodes().size());
  // Keep track on "expandable" edges (currently best edges) per GeometryNode.
  GeometryNodeVector<std::vector<MotionEdgeIndex>> motions_to_expand;
  motions_to_expand.resize(geom_graph.nodes().size());
  // Main loop of DP search once a leading obj is specified.
  int motion_count = 0;
  const auto& nodes_layers = geom_graph.nodes_layers();
  for (int cur_layer_idx = 0, num_layer = nodes_layers.size();
       cur_layer_idx < num_layer; ++cur_layer_idx) {
    TIMELINE(absl::StrFormat("DpSearchOnLayer_%d", cur_layer_idx));

    const auto& nodes_layer = nodes_layers[cur_layer_idx];
    // 1. insert best so far motion nodes & edges of the current layer and
    // pick the motion edges to expand. The block is not executed for the
    // first layer since at the beginning we start to expand from the
    // planning_start_state.
    if (cur_layer_idx != 0) {
      InsertMotionNodes(nodes_layer, sdc_node_idx, opt_motion_samples,
                        traj_output.motion_graph.get(), &terminated_edge_idxes,
                        &motions_to_expand, &search_costs,
                        &ignored_trajs_vector);
    }

    // Reached the last layer, stop the iteration.
    if (cur_layer_idx == num_layer - 1) break;

    // 2. Expand motions from nodes.
    // candidate_motions records the candidate motions for each node per layer.
    std::vector<std::vector<DpMotionInfo>> candidate_motions;
    candidate_motions.resize(nodes_layer.size());
    std::vector<std::vector<NewCacheInfo>> new_motion_forms_container;
    new_motion_forms_container.resize(nodes_layer.size());

    if (cur_layer_idx == 0) {
      candidate_motions[start_node_idx_on_first_layer] = ExpandStartMotionEdges(
          nodes_layer[start_node_idx_on_first_layer], sdc_node_idx, geom_graph,
          *(traj_output.motion_graph), motion_constraint_params,
          *traj_output.cost_provider, *cost_cache,
          &new_motion_forms_container[start_node_idx_on_first_layer],
          thread_pool);
      // const std::string prefix = Log2DDS::TaskPrefix(plan_id);
      // for (const auto& motion_node :
      //      candidate_motions[start_node_idx_on_first_layer]) {
      //   const auto geom_edge =
      //       geom_graph.GetEdge(motion_node.geometry_edge_index);
      //   const auto cost_names = traj_output.cost_provider->cost_names();
      //   std::string cost_str = "";
      //   for (int i = 0; i < cost_names.size(); ++i) {
      //     cost_str += absl::StrCat((i == 0 ? "" : ", "), cost_names[i], ": ",
      //                              motion_node.costs[i]);
      //   }
      //   Log2DDS::LogDataV2(
      //       prefix + "search_result",
      //       absl::StrFormat("edge%d->%d, a:%.3f, cost:%.3f, %s",
      //                       geom_edge.start.value(), geom_edge.end.value(),
      //                       motion_node.motion_form->GetStartMotionState().a,
      //                       motion_node.sum_cost, cost_str.c_str()));

      //   const auto end_node_idx = geom_edge.end;
      //   const auto end_node_idx_value = end_node_idx.value();
      //   std::vector<double> xs, ys;
      //   xs.emplace_back(geom_graph.GetNode(end_node_idx).xy.x());
      //   ys.emplace_back(geom_graph.GetNode(end_node_idx).xy.y());
      //   Log2DDS::LogPointsV2(
      //       absl::StrCat(prefix, "candidate_node_", end_node_idx_value),
      //       Log2DDS::kWhite, {}, xs, ys, 15.0);
      // }
    } else {
      // This routine is the most computational heavy one, can be paralleled.
      // Create container per node to hold new motion forms temporarily.
      bool sample_const_v = false;
      if ((InitializerSceneType::INIT_SCENE_BORROW != init_scene_type) &&
          cur_layer_idx >= kConstVelSampleLayerSizeThreshold - 1) {
        // If current layer >= kConstVelSampleLayerSizeThreshold (currently
        // set at 5), only sample acc = 0 and a_min, a_max, a_stop motions.
        sample_const_v = true;
      }
      // do not use the thread pool
      ParallelFor(0, nodes_layer.size(), thread_pool, [&](int i) {
        // Parallel to calculate candidate motions and their cost for each
        // node on this layer.
        TIMELINE(absl::StrFormat("ExpandMotionEdges_%d", i));
        const auto geom_node_idx = nodes_layer[i];
        const auto& motion_edge_idxs = motions_to_expand[geom_node_idx];
        auto& new_motion_forms = new_motion_forms_container[i];
        candidate_motions[i] = ExpandMotionEdges(
            geom_node_idx, motion_edge_idxs, ignored_trajs_vector, geom_graph,
            *(traj_output.motion_graph), motion_constraint_params,
            *traj_output.cost_provider, sample_const_v, *cost_cache,
            &new_motion_forms, thread_pool);
      });
    }
    for (auto it = std::make_move_iterator(new_motion_forms_container.begin());
         it != std::make_move_iterator(new_motion_forms_container.end());
         ++it) {
      cost_cache->BatchInsert(*it);
    }
    // 3. Update optimal motions associated to each geometry node.
    motion_count +=
        UpdateOptimalMotions(search_costs, *cost_cache,
                             std::move(candidate_motions), &opt_motion_samples);
  }

  VLOG(2) << "Total evaluated motions of DP: " << motion_count;
  VLOG(2) << "Collected motion edge cache size: " << cost_cache->size();

  // Find the best edge considering final cost.
  const auto best_edge_info =
      FindBestEdge(*traj_output.cost_provider, sdc_motion, sdc_node_idx,
                   nodes_layers[0][start_node_idx_on_first_layer],
                   traj_output.motion_graph.get(), cost_cache, &search_costs,
                   &ignored_trajs_vector, &terminated_edge_idxes);
  const auto best_final_edge = best_edge_info.idx;
  if (best_final_edge == MotionEdgeVector<MotionEdge>::kInvalidIndex) {
    return absl::NotFoundError("No trajectories found.");
  }
  traj_output.last_edge_index = best_final_edge;
  traj_output.search_costs = search_costs;
  traj_output.total_cost = best_edge_info.total_cost;

  VLOG(2) << "best_final_edge: " << best_final_edge.value()
          << " Total cost: " << traj_output.total_cost;
  // if (FLAGS_planner_initializer_enable_post_evaluation) {
  //   PostEvaluatorInput post_evaluator_input{
  //       .form_builder = &form_builder,
  //       .pre_best = &best_edge_info,
  //       .drive_passage = &drive_passage,
  //       .stop_s = &stop_s_on_drive_passage,
  //       .st_traj_mgr = &st_traj_mgr,
  //       .leading_trajs = &leading_trajs,
  //       .vehicle_geom = &vehicle_geom,
  //       .collision_checker = &collision_checker,
  //       .path_sl = &path_sl,
  //       .initializer_params = &initializer_params,
  //       .motion_constraint_params = &motion_constraint_params,
  //       .ref_speed_table = traj_output.ref_speed_table.get(),
  //       .captain_net_output = &captain_net_output,
  //       .is_lane_change = is_lane_change,
  //       .max_accumulated_s = max_accumulated_s,
  //       .is_post_evaluation = true,
  //       .search_costs = &search_costs,
  //       .terminated_idxes = terminated_edge_idxes,
  //       .sdc_motion = &sdc_motion,
  //       .sdc_node_idx = sdc_node_idx,
  //       .sdc_geom_node = nodes_layers[0][start_node_idx_on_first_layer],
  //       .motion_graph = traj_output.motion_graph.get(),
  //   };
  //   PostEvaluateTrajs(post_evaluator_input, &traj_output, thread_pool);
  // }
  // Return ignorable trajectories.
  traj_output.ignored_trajs = ignored_trajs_vector[traj_output.last_edge_index];

  traj_points = ConstructTrajFromLastEdge(*traj_output.motion_graph,
                                          traj_output.last_edge_index);

  // For debug: show candidate trajectories
  if (FLAGS_planner_initializer_debug_level >= 1 ||
      FLAGS_planner_dumping_initializer_features) {
    FillTrajDebugInfo(*traj_output.motion_graph, traj_output.search_costs,
                      std::move(terminated_edge_idxes),
                      &traj_output.debug_info);
  }

  return traj_points;
}

}  // namespace st::planning
