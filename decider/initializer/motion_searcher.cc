

#include "decider/initializer/dp_motion_searcher.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "decider/initializer/astar_motion_searcher.h"
#include "decider/initializer/collision_checker.h"
#include "decider/initializer/cost_provider.h"
#include "decider/initializer/dp_motion_searcher.h"
#include "decider/initializer/geometry/geometry_form_builder.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "decider/initializer/motion_form.h"
#include "decider/initializer/motion_graph.h"
#include "decider/initializer/motion_graph_cache.h"
#include "decider/initializer/motion_search_util.h"
#include "decider/initializer/motion_state.h"
#include "decider/initializer/multi_traj_selector.h"
#include "decider/initializer/ref_speed_table.h"
#include "decider/initializer/select_nudge_object.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/util.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/timer.h"
#include "plan_common/util/decision_info.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_object_state.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "decider/initializer/initializer_util.h"

namespace st::planning {

namespace {

using LeadingTrajs = std::vector<std::string>;

std::vector<LeadingTrajs> BuildLeadingConfigs(
    const std::vector<LeadingGroup>& leading_groups,
    const ConstraintProto::LeadingObjectProto* blocking_static_traj) {
  // Construct leading config according to leading groups.
  std::vector<LeadingTrajs> leading_configs;
  leading_configs.reserve(leading_groups.size());
  for (const auto& leading_group : leading_groups) {
    auto& leading_trajs = leading_configs.emplace_back();
    for (const auto& [traj_id, lead_obj] : leading_group) {
      leading_trajs.push_back(traj_id);
      if (lead_obj.is_group_tail() && leading_trajs.size() > 1) {
        std::swap(leading_trajs.front(), leading_trajs.back());
      }
    }
  }

  if (blocking_static_traj != nullptr) {
    // Add non-stalled blocking static trajectory to all groups.
    for (auto& leading_config : leading_configs) {
      leading_config.push_back(blocking_static_traj->traj_id());
    }
  }

  return leading_configs;
}

absl::StatusOr<SingleTrajInfo> SearchForSingleTrajectory(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl,
    const ApolloTrajectoryPointProto& start_point,
    const st::FrenetCoordinate& ego_sl,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const LeadingTrajs& leading_trajs,
    const InitializerConfig& initializer_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom,
    const GeometryGraph& geom_graph, const GeometryFormBuilder& form_builder,
    const CollisionChecker& collision_checker,
    // const ml::captain_net::CaptainNetOutput& captain_net_output,
    const std::vector<double>& stop_s_on_drive_passage,
    const double passage_speed_limit,
    const InitializerSceneType init_scene_type, bool is_lane_change,
    MotionGraphCache* cost_cache, ThreadPool* thread_pool, const int plan_id) {
  TIMELINE("SearchForSingleTrajectory");
  // VLOG(2) << "--------- Start of One Search-----------";
  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  const double geom_graph_max_s = geom_graph.GetMaxAccumulatedS();
  const double speed_max_s =
      ego_sl.s +
      ((InitializerSceneType::INIT_SCENE_BORROW == init_scene_type)
           ? std::fmax(
                 kMinSpeedForFinalCost * kInitializerTrajectoryTimeHorizon,
                 start_point.v() > passage_speed_limit
                     ? start_point.v() * kInitializerTrajectoryTimeHorizon
                     : (passage_speed_limit - start_point.v() >
                                kInitializerTrajectoryTimeHorizon
                            ? start_point.v() *
                                      kInitializerTrajectoryTimeHorizon +
                                  0.5 * kInitializerTrajectoryTimeHorizon *
                                      kInitializerTrajectoryTimeHorizon
                            : 0.5 * (passage_speed_limit * passage_speed_limit -
                                     start_point.v() * start_point.v()) +
                                  passage_speed_limit *
                                      (kInitializerTrajectoryTimeHorizon -
                                       passage_speed_limit + start_point.v())))
           : std::fmax(start_point.v(), kMinSpeedForFinalCost) *
                 kInitializerTrajectoryTimeHorizon);

  Timer start_time;
  SingleTrajInfo traj_output;
  traj_output.leading_trajs = leading_trajs;
  std::string task_id = Log2DDS::TaskPrefix(plan_id);
  traj_output.ref_speed_table =
      std::make_unique<RefSpeedTable>(st_traj_mgr, leading_trajs, drive_passage,
                                      stop_s_on_drive_passage, task_id);
  traj_output.motion_graph = std::make_unique<XYTMotionGraph>(&geom_graph);

  const double leading_obj_min_s =
      GetLeadingObjectsEndMinS(st_traj_mgr, drive_passage, leading_trajs,
                               vehicle_geom.front_edge_to_center()) -
      kInitializerMinFollowDistance;
  const double max_accumulated_s =
      Min(leading_obj_min_s, geom_graph_max_s, speed_max_s);
  Log2DDS::LogDataV0(
      prefix + "search_result",
      absl::StrFormat("start_s:%.3f, leading_obj_min_s:%.3f, "
                      "geom_graph_max_s:%.3f, speed_max_s:%.3f",
                      ego_sl.s,
                      (leading_obj_min_s > 1e5 ? -1.0 : leading_obj_min_s),
                      geom_graph_max_s, speed_max_s));
  traj_output.cost_provider = std::make_unique<CostProvider>(
      drive_passage, initializer_params, motion_constraint_params,
      stop_s_on_drive_passage, st_traj_mgr, leading_trajs, vehicle_geom,
      &collision_checker, &path_sl, traj_output.ref_speed_table.get(),
      init_scene_type, is_lane_change, ego_sl.s, max_accumulated_s);
  // Keep track on finished motions (reached max trajectory time length).
  std::vector<MotionEdgeIndex> terminated_edge_idxes;
  // Keep track on optimal costs.
  MotionEdgeVector<MotionSearchOutput::SearchCost> search_costs;
  MotionEdgeVector<IgnoreTrajMap> ignored_trajs_vector;

  // Prepare start node
  const auto& nodes_layers = geom_graph.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  MotionState sdc_motion = PrepareStartMotionNode(
      geom_graph, nodes_layers[0], start_point, &start_node_idx_on_first_layer);
  const auto sdc_node_idx = traj_output.motion_graph->AddMotionNode(
      sdc_motion, nodes_layers[0][start_node_idx_on_first_layer]);
  // Min Stop S
  const double min_stop_s =
      stop_s_on_drive_passage.empty()
          ? std::numeric_limits<double>::max()
          : *std::min_element(stop_s_on_drive_passage.begin(),
                              stop_s_on_drive_passage.end());

  absl::StatusOr<std::vector<ApolloTrajectoryPointProto>> traj_points;
  switch (initializer_params.search_algorithm()) {
    case InitializerConfig::DP:
      traj_points = DPSearchMainLoop(
          sdc_motion, sdc_node_idx, init_scene_type,
          start_node_idx_on_first_layer, geom_graph, motion_constraint_params,
          search_costs, ignored_trajs_vector, cost_cache, thread_pool,
          traj_output, terminated_edge_idxes, plan_id);
      if (!traj_points.ok()) {
        traj_points = GenerateLatQuinticLonConstAccTrajToRefL(
            drive_passage, path_sl, st_traj_mgr, leading_trajs, start_point,
            vehicle_geom.front_edge_to_center(), min_stop_s, plan_id,
            absl::StrCat("DP search failed! ", traj_points.status().message()));
        const auto cost_size = traj_output.cost_provider->cost_names().size();
        traj_output.last_edge_index = MotionEdgeIndex(0);
        traj_output.feature_costs.resize(cost_size);
        traj_output.search_costs.resize(1);
        traj_output.search_costs[traj_output.last_edge_index]
            .feature_cost.resize(cost_size);
        if (!traj_points.ok()) return traj_points.status();
      }
      break;
    case InitializerConfig::AStar:
      traj_points = AStarSearchMainLoop(
          drive_passage, path_sl, st_traj_mgr, leading_trajs, geom_graph,
          start_point, motion_constraint_params, initializer_params,
          vehicle_geom, sdc_motion, is_lane_change, init_scene_type,
          start_node_idx_on_first_layer, min_stop_s, plan_id, traj_output,
          max_accumulated_s);
      if (!traj_points.ok()) {
        traj_points = GenerateLatQuinticLonConstAccTrajToRefL(
            drive_passage, path_sl, st_traj_mgr, leading_trajs, start_point,
            vehicle_geom.front_edge_to_center(), min_stop_s, plan_id,
            absl::StrCat("A* search failed! ", traj_points.status().message()));
        const auto cost_size = traj_output.cost_provider->cost_names().size();
        traj_output.last_edge_index = MotionEdgeIndex(0);
        traj_output.feature_costs.resize(cost_size);
        traj_output.search_costs.resize(1);
        traj_output.search_costs[traj_output.last_edge_index]
            .feature_cost.resize(cost_size);
        if (!traj_points.ok()) return traj_points.status();
      }
      break;
  }

  const auto& front_pt = traj_points.value().front();
  const auto& back_pt = traj_points.value().back();
  if (front_pt.v() < kCanSetToZeroSpeed && back_pt.v() < kCanSetToZeroSpeed &&
      back_pt.path_point().s() - front_pt.path_point().s() <
          kCanSetToZeroTrajLength) {
    traj_output.traj_points = ConstructStationaryTraj(sdc_motion);
  } else {
    traj_output.traj_points = std::move(traj_points.value());
  }
  NudgeInfos nudge_info;
  std::vector<TrajectoryPoint> init_traj_points;
  TrajectoryPoint traj_point;
  for (const auto& point : traj_output.traj_points) {
    traj_point.FromProto(point);
    init_traj_points.emplace_back(traj_point);
  }
  const auto nudge_object_infos = initializer::SelectNudgeObjectId(
      traj_output.traj_points.size(), kTrajectoryTimeStep, is_lane_change,
      drive_passage, path_sl, init_traj_points, st_planner_object_traj,
      vehicle_geom, plan_id);
  if (nudge_object_infos.ok()) {
    nudge_info = nudge_object_infos.value();
  }
  traj_output.nudge_info = nudge_info;

  VLOG(2) << "motion search for Time spent: " << start_time.Time();
  VLOG(2) << "--------- End of One Search-------------";
  return traj_output;
}

absl::Status CheckForImmediateCollision(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  const auto ego_box =
      ComputeAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
                   plan_start_point.path_point().theta(), vehicle_geom);

  // const double lat_overlap =
  //     std::min(ego_cur_frenet_box.l_max, obj_cur_frenet_box.l_max) -
  //     std::max(ego_cur_frenet_box.l_min, obj_cur_frenet_box.l_min);
  // if (lat_overlap > 0.3 * obj_cur_frenet_box.width()) {
  // }

  constexpr double kImmediateTimeThreshold = 0.3;  // s.
  for (const auto* traj_ptr : st_traj_mgr.moving_object_trajs()) {
    for (const auto& state : traj_ptr->states()) {
      if (state.traj_point->t() > kImmediateTimeThreshold) break;

      if (ego_box.HasOverlap(state.box)) {
        ASSIGN_OR_CONTINUE(
            const auto obj_sl,
            drive_passage.QueryFrenetCoordinateAt(state.box.center()));

        if (HasEnteredTargetLane(obj_sl.l, state.box.half_width())) {
          return absl::CancelledError(absl::StrFormat(
              "Trajectory %s will reach the ego vehicle\'s current "
              "position at %.2f s",
              traj_ptr->traj_id(), state.traj_point->t()));
        }
      }
    }
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<MotionSearchOutput> SearchForRawTrajectory(
    const MotionSearchInput& input, ThreadPool* thread_pool, int plan_id) {
  TIMELINE("SearchForRawTrajectory");
  const DrivePassage& drive_passage = *input.drive_passage;
  const ApolloTrajectoryPointProto& start_point = *input.start_point;
  const SpacetimeTrajectoryManager& st_traj_mgr = *input.st_traj_mgr;
  const VehicleGeometryParamsProto& vehicle_geom =
      input.vehicle_params->vehicle_geometry_params();
  const GeometryGraph& geom_graph = *input.geom_graph;
  const auto& prefix = Log2DDS::TaskPrefix(plan_id);

  const auto leading_traj_configs =
      BuildLeadingConfigs(*input.leading_groups, input.blocking_static_traj);
  if (leading_traj_configs.empty()) {
    return absl::NotFoundError("No leading group generated.");
  }

  // Prepare motion graph cache.
  MotionGraphCache cost_cache;

  std::vector<SingleTrajInfo> multi_trajs;
  multi_trajs.reserve(leading_traj_configs.size());

  // TODO: Change this loop into a parallel for loop.
  for (const auto& leading_trajs : leading_traj_configs) {
    auto single_traj_result = SearchForSingleTrajectory(
        drive_passage, *input.sl_boundary, start_point, input.ego_sl,
        st_traj_mgr, *input.st_planner_object_traj, leading_trajs,
        *input.initializer_params, *input.motion_constraint_params,
        vehicle_geom, geom_graph, *input.form_builder, *input.collision_checker,
        *input.stop_s_vec, input.passage_speed_limit, input.init_scene_type,
        input.is_lane_change, &cost_cache, thread_pool, plan_id);
    if (single_traj_result.ok()) {
      multi_trajs.push_back(std::move(single_traj_result).value());
    }
  }

  if (multi_trajs.empty() && !input.is_lane_change) {
    return absl::NotFoundError("No terminating motion edge found.");
  }

  const auto search_algorithm = (*input.initializer_params).search_algorithm();
  VLOG(3) << "---------- Leading Object Group ----------";
  // int i = 1;
  int traj_info_idx = 0;
  if (!multi_trajs.empty()) {
    for (const auto& traj_info : multi_trajs) {
      std::string cost_info = absl::StrCat("total: ", traj_info.total_cost);
      cost_info += absl::StrCat(", idx: ", traj_info_idx);
      cost_info +=
          absl::StrCat(", leading_id: ", traj_info.GetLeadingObjTrajId());
      const auto cost_names = traj_info.cost_provider->cost_names();
      if (search_algorithm == InitializerConfig::DP) {
        for (int i = 0; i < cost_names.size(); ++i) {
          cost_info +=
              absl::StrCat(", ", cost_names[i], ": ",
                           traj_info.search_costs[traj_info.last_edge_index]
                               .feature_cost[i]);
        }
      } else if (search_algorithm == InitializerConfig::AStar) {
        for (int i = 0; i < cost_names.size(); ++i) {
          cost_info += absl::StrCat(", ", cost_names[i], ": ",
                                    traj_info.feature_costs[i]);
        }
      }
      Log2DDS::LogDataV2(absl::StrCat(prefix, "dp_traj_info_"), cost_info);
      ++traj_info_idx;
    }
  }
  // Need to determine the final output.
  absl::flat_hash_set<std::string> follower_set;
  absl::flat_hash_set<std::string> leader_set;
  absl::flat_hash_set<std::string> gaming_lc_obs_set;
  double leader_max_decel = 0.0;
  double follower_max_decel = 0.0;
  absl::flat_hash_set<std::string> unsafe_object_ids;
  PlannerStatusProto::PlannerStatusCode lc_status_code = PlannerStatusProto::OK;

  int c_idx = 0;
  LaneChangeStyleDeciderResultProto lc_style_decider_result;
  TaskSafetyEvaluationProto task_safety_evaluation_result;
  int scene_cones_riding_line_frames_result = 0;
  const auto choice_or = EvaluateMultiTrajs(
      *input.planner_semantic_map_manager, drive_passage, *input.sl_boundary,
      start_point, st_traj_mgr, multi_trajs, *input.vehicle_params,
      input.passage_speed_limit, input.eval_safety,
      std::make_pair(input.standard_congestion_factor,
                     input.traffic_congestion_factor),
      input.lc_style, input.pre_lc_style_decider_result,
      input.pre_task_safety_evaluation_result,
      input.pre_scene_cones_riding_line_frames_result, input.lc_state,
      input.lc_left, input.prev_lc_stage, *input.stalled_objects,
      *input.obs_history, input.path_look_ahead_duration, &follower_set,
      &leader_set, &follower_max_decel, &leader_max_decel, &unsafe_object_ids,
      &lc_status_code, thread_pool, plan_id, input.borrow_lane, &c_idx,
      &lc_style_decider_result, &task_safety_evaluation_result,
      &scene_cones_riding_line_frames_result, leading_traj_configs[0],
      &gaming_lc_obs_set, input.stop_s_vec);

  // viz motion search result
  if (!multi_trajs.empty()) {
    const int chosen_idx = choice_or.ok() ? *choice_or : c_idx;
    const auto& chosen_traj = multi_trajs[chosen_idx];
    std::string cost_info = absl::StrCat("total: ", chosen_traj.total_cost);
    const auto cost_names = chosen_traj.cost_provider->cost_names();
    if (search_algorithm == InitializerConfig::DP) {
      for (int i = 0; i < cost_names.size(); ++i) {
        cost_info +=
            absl::StrCat(", ", cost_names[i], ": ",
                         chosen_traj.search_costs[chosen_traj.last_edge_index]
                             .feature_cost[i]);
      }
    } else if (search_algorithm == InitializerConfig::AStar) {
      for (int i = 0; i < cost_names.size(); ++i) {
        cost_info += absl::StrCat(", ", cost_names[i], ": ",
                                  chosen_traj.feature_costs[i]);
      }
    }

    Log2DDS::LogDataV2(absl::StrCat(prefix, "init_res_cost"), cost_info);
    std::vector<double> xs, ys;
    xs.reserve(chosen_traj.traj_points.size());
    ys.reserve(chosen_traj.traj_points.size());
    for (const auto& point : chosen_traj.traj_points) {
      xs.push_back(point.path_point().x());
      ys.push_back(point.path_point().y());
    }
    Log2DDS::LogLineV2(absl::StrCat(prefix, "init-res"), Log2DDS::kYellow, {},
                       xs, ys, 2.0);
    if (FLAGS_planner_initializer_debug_level >= 1) {
      int traj_info_idx = 0;
      for (const auto& traj_info : multi_trajs) {
        std::vector<double> xs, ys;
        xs.reserve(traj_info.traj_points.size());
        ys.reserve(traj_info.traj_points.size());
        for (const auto& point : traj_info.traj_points) {
          xs.push_back(point.path_point().x());
          ys.push_back(point.path_point().y());
        }
        Log2DDS::LogLineV2(absl::StrCat(prefix, "init-res_", traj_info_idx),
                           Log2DDS::kPink, {}, xs, ys, 2.0);
        ++traj_info_idx;
      }
    }
  }

  // Add all candidates to motion search output result, with the first one
  // always the selected.
  MotionSearchOutput output{};
  output.multi_traj_candidates.reserve(multi_trajs.size());
  for (const auto& traj : multi_trajs) {
    MotionSearchOutput::MultiTrajCandidate traj_candidate{};
    traj_candidate.leading_traj_ids = traj.leading_trajs;
    traj_candidate.trajectory = traj.traj_points;
    // if (traj.last_edge_index.value() < traj.search_costs.size()) {
    //   traj_candidate.feature_costs =
    //       traj.search_costs[traj.last_edge_index].feature_cost;
    // }
    // traj_candidate.last_edge_index = traj.last_edge_index;
    traj_candidate.total_cost = traj.total_cost;
    // traj_candidate.ignored_trajs = traj.ignored_trajs;
    output.multi_traj_candidates.push_back(std::move(traj_candidate));
  }

  output.lc_style_decider_result = lc_style_decider_result;
  output.task_safety_evaluation_result = task_safety_evaluation_result;
  output.scene_cones_riding_line_frames_result =
      scene_cones_riding_line_frames_result;
  output.speed_response_style = MappingLongResponseLevel(
      input.prev_lc_stage, follower_max_decel, leader_max_decel, prefix);
  output.lc_status_code = lc_status_code;
  Log2DDS::LogDataV2("lc_safety",
                     absl::StrCat(prefix, ":lc_status_code:", lc_status_code));

  if (!choice_or.ok()) {
    LOG_WARN << "No safe trajectory found for initializer, pausing "
                "lane change: "
             << choice_or.status().message();
    output.unsafe_object_ids = std::move(unsafe_object_ids);

    // const auto immediate_collision_status = CheckForImmediateCollision(
    //     drive_passage, start_point, vehicle_geom, st_traj_mgr);
    // if (!immediate_collision_status.ok()) {
    //   output.result_status = immediate_collision_status;

    //   return output;
    // }

    // No safe trajectory found, return a indicator to fake output outside.
    output.is_lc_pause = true;
    Log2DDS::LogDataV2("lc_pause_debug", "has unsafe obs");

    return output;
  }

  const int choice = *choice_or;
  if (choice != 0) {
    std::swap(output.multi_traj_candidates[0],
              output.multi_traj_candidates[choice]);
  }
  output.follower_set = std::move(follower_set);
  output.leader_set = std::move(leader_set);
  output.gaming_lc_obs_set = std::move(gaming_lc_obs_set);
  output.follower_max_decel = follower_max_decel;
  for (const auto& traj_id : multi_trajs[choice].leading_trajs) {
    for (const auto& leading_group : *input.leading_groups) {
      const auto it = leading_group.find(traj_id);
      if (it != leading_group.end()) {
        output.leading_trajs.emplace(traj_id, it->second);
        break;
      }
    }
  }
  output.nudge_info = std::move(multi_trajs[choice].nudge_info);
  output.traj_points = std::move(multi_trajs[choice].traj_points);
  output.search_costs = std::move(multi_trajs[choice].search_costs);
  output.best_last_edge_index = multi_trajs[choice].last_edge_index;
  output.min_cost = multi_trajs[choice].total_cost;
  output.motion_graph = std::move(multi_trajs[choice].motion_graph);
  output.ref_speed_table = std::move(multi_trajs[choice].ref_speed_table);
  output.cost_provider = std::move(multi_trajs[choice].cost_provider);
  VLOG(2) << absl::StrJoin(
      output.search_costs[output.best_last_edge_index].feature_cost, ", ");

  if (FLAGS_planner_initializer_debug_level >= 1 ||
      FLAGS_planner_dumping_initializer_features) {
    auto& debug_info = multi_trajs[choice].debug_info;
    output.terminated_edge_idxes = std::move(debug_info.terminated_edge_idxes);
    output.top_k_trajs = std::move(debug_info.top_k_trajs);
    output.top_k_total_costs = std::move(debug_info.top_k_total_costs);
    output.top_k_edges = std::move(debug_info.top_k_edges);
  }

  // For data dumping only.
  // if (FLAGS_planner_dumping_initializer_features) {
  //   RETURN_IF_ERROR(
  //       ExpertDpMotionEvaluation(input.plan_time, *input.form_builder,
  //                                *input.log_av_trajectory, &output));
  //   SampledDpMotionEvaluation(output.search_costs,
  //   output.terminated_edge_idxes,
  //                             &output);
  // }

  return output;
}

}  // namespace st::planning
