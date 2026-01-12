

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

#include "planner/speed_optimizer/interactive_agent_process/game_theory/game_theory_interface.h"
#include "planner/speed_optimizer/interactive_agent_process/interactive_agent.h"
#include "planner/speed_optimizer/interactive_agent_process/interactive_agent_process.h"
#include "planner/planner_manager/planner_flags.h"
#include "planner/speed_optimizer/interactive_agent_process/interactive_post_processing.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/utility.h"
namespace st::planning {

#define DEBUG_TIME (0)
#define DEBUG_EGO_INFO (0)
#define DEBUG_SELECTED_OBJ (0)

bool IsTwoAgentsInSameInteractionZone(
    const double agent_a_ego_dist_to_inter,
    const double agent_a_ego_dist_to_leave_enter,
    const double agent_b_ego_dist_to_inter,
    const double agent_b_ego_dist_to_leave_enter) {
  constexpr double kSameInteractionS = 2.0;

  if (lx_fless(agent_a_ego_dist_to_leave_enter + kSameInteractionS,
               agent_b_ego_dist_to_inter)) {
    return false;
  }
  if (lx_fgreater(agent_a_ego_dist_to_inter,
                  agent_b_ego_dist_to_leave_enter + kSameInteractionS)) {
    return false;
  }

  return true;
}

void UpdateProcessAgents(std::map<double, StBoundaryWithDecision*>& process_obj,
                         StBoundaryWithDecision* st_boundary_wd) {
  if (nullptr == st_boundary_wd || nullptr == st_boundary_wd->st_boundary()) {
    return;
  }

  bool is_need_insert = false;

  for (auto it = process_obj.begin(); it != process_obj.end();) {
    if (nullptr == it->second->st_boundary()) {
      ++it;
      continue;
    }
    if (IsTwoAgentsInSameInteractionZone(
            it->second->st_boundary()->min_s(),
            it->second->st_boundary()->max_s(),
            st_boundary_wd->st_boundary()->min_s(),
            st_boundary_wd->st_boundary()->max_s())) {
      if (lx_fless(st_boundary_wd->st_boundary()->min_t(),
                   it->second->st_boundary()->min_t())) {
        it = process_obj.erase(it);
        is_need_insert = true;
      } else {
        ++it;
      }
    } else {
      ++it;
      is_need_insert = true;
    }
  }

  if (is_need_insert) {
    double curr_ego_start_s = st_boundary_wd->st_boundary()->min_s();
    double curr_obj_min_t = st_boundary_wd->st_boundary()->min_t();
    double key_value = curr_ego_start_s * 100 + curr_obj_min_t;
    process_obj.insert(
        std::pair<double, StBoundaryWithDecision*>(key_value, st_boundary_wd));
  }

  return;
}

void SelectedInteractiveAgents(
    std::vector<InteractiveAgent>& interactive_agents,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage* drive_passage, const DiscretizedPath& path) {
  std::map<double, StBoundaryWithDecision*> process_obj;
  constexpr int max_process_obj_size = 3;

#if DEBUG_SELECTED_OBJ
  std::cout << "--------------------2. selected_obj " << std::endl;
#endif

  for (int i = 0; i < st_boundaries_with_decision->size(); i++) {
    /* todo: selected_obj */

    auto obj_scenario =
        (*st_boundaries_with_decision)[i].st_boundary()->obj_scenario_info();
#if DEBUG_SELECTED_OBJ
    if ((*st_boundaries_with_decision)[i].object_id().has_value()) {
      std::cout << "[ " << (*st_boundaries_with_decision)[i].object_id().value()
                << " ]: "
                << " interaction_zone "
                << static_cast<int>(obj_scenario.interaction_zone)
                << " relationship "
                << static_cast<int>(obj_scenario.relationship) << std::endl;
    }
#endif
    if (!(LaneSemantic::INTERSECTION_LEFT_TURN == obj_scenario.lane_semantic &&
          Relationship::Cross == obj_scenario.relationship)) {
      continue;
    }
    if ((*st_boundaries_with_decision)[i].raw_st_boundary()->is_protective()) {
      continue;
    }
    if (StBoundaryProto::FOLLOW ==
        (*st_boundaries_with_decision)[i].decision_type())
      continue;
    if (StBoundaryProto::IGNORE ==
        (*st_boundaries_with_decision)[i].decision_type()) {
      if (StBoundaryProto::IGNORE_DECIDER ==
              (*st_boundaries_with_decision)[i].decision_reason() ||
          StBoundaryProto::POSTPROCESS_PEDESTRIAN_LON_DECISION ==
              (*st_boundaries_with_decision)[i].decision_reason() ||
          StBoundaryProto::POSTPROCESS_STRAIGHT_TURN_RIGHT_SCENE_LON_DECISION ==
              (*st_boundaries_with_decision)[i].decision_reason()) {
        continue;
      }
    }
    if (process_obj.size() <= max_process_obj_size) {
      double curr_ego_start_s =
          (*st_boundaries_with_decision)[i].st_boundary()->min_s();
      double curr_obj_min_t =
          (*st_boundaries_with_decision)[i].st_boundary()->min_t();
      double key_value = curr_ego_start_s * 100 + curr_obj_min_t;
      process_obj.insert(std::pair<double, StBoundaryWithDecision*>(
          key_value, &(*st_boundaries_with_decision)[i]));
    } else {
      UpdateProcessAgents(process_obj, &(*st_boundaries_with_decision)[i]);
    }
  }

  bool ego_turn_left, ego_turn_right, ego_uturn;

  if (path.size() > 1) {
    auto ego_first_point = path.front();
    auto ego_end_point = path.back();

    SecondOrderTrajectoryPoint global_start, global_end;
    global_start.set_pos(Vec2d(ego_first_point.x(), ego_first_point.y()));
    global_start.set_theta(ego_first_point.theta());

    global_end.set_pos(Vec2d(ego_end_point.x(), ego_end_point.y()));
    global_end.set_theta(ego_end_point.theta());

    JudgeTurnInfo(&ego_turn_left, &ego_turn_right, &ego_uturn, &global_start,
                  &global_end);
  }

  int process_obj_num = 0;

  bool is_debug = false;

#if DEBUG_SELECTED_OBJ
  std::cout << " process_obj: size " << process_obj.size() << std::endl;
  is_debug = true;
#endif

  for (auto single_obj : process_obj) {
#if DEBUG_SELECTED_OBJ
    if (single_obj.second->object_id().has_value()) {
      std::cout << " [process_obj]: obj_id "
                << single_obj.second->object_id().value() << std::endl;
    }
#endif
    if (!single_obj.second->traj_id().has_value()) continue;
    const auto& traj_id = single_obj.second->traj_id();
    const auto* spacetime_obj =
        CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id.value()));

    InteractiveAgent tmp_interactive_agent;
    tmp_interactive_agent.SetAgentInitState(
        spacetime_obj, drive_passage, single_obj.second, ego_turn_left,
        ego_turn_right, ego_uturn, is_debug);
    interactive_agents.emplace_back(tmp_interactive_agent);
#if DEBUG_SELECTED_OBJ
    printf("\n");
#endif
    process_obj_num++;
    if (process_obj_num > max_process_obj_size) break;
  }

  return;
}

void UpdateEgoInteractiveInfo(InteractiveAgent& ego_interactive_info,
                              const VehicleGeometryParamsProto& vehicle_geom,
                              const DiscretizedPath& path, double ego_init_vel,
                              double ego_init_acc,
                              const std::vector<VehicleShapeBasePtr>& av_shapes,
                              const DrivePassage* drive_passage) {
  bool is_debug = false;
#if DEBUG_EGO_INFO
  std::cout << "--------------------1. update ego sim state " << std::endl;
  is_debug = true;
#endif

  ego_interactive_info.SetEgoInitState(vehicle_geom, path, ego_init_vel,
                                       ego_init_acc, av_shapes, drive_passage,
                                       is_debug);
}

absl::Status InteractiveAgentProcess(
    const VehicleGeometryParamsProto& vehicle_geom, const StGraph& st_graph,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, double current_a,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const DrivePassage* drive_passage,
    const LaneChangeStateProto lane_change_state) {
  CHECK_NOTNULL(st_boundaries_with_decision);
  CHECK_NOTNULL(processed_st_objects);
  std::vector<InteractiveAgent> interactive_agents;
  InteractiveAgent ego_interactive_info;
  std::vector<gt_result_t> gt_results;

  if (!FLAGS_lon_decision_enable_use_game_theory) return absl::OkStatus();

  if (LaneChangeStage::LCS_NONE != lane_change_state.stage()) {
    return absl::OkStatus();
  }

#if DEBUG_TIME
  std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<< game_theory_start "
               "<<<<<<<<<<<<<<<<<<<<<<<<<<<< "
            << std::endl;
#endif
  double start_time = ad_byd::planning::Utility::GetCurrentTime() / 1000.0;
  double ego_info_start_time = start_time;

  UpdateEgoInteractiveInfo(ego_interactive_info, vehicle_geom, path, current_v,
                           current_a, av_shapes, drive_passage);

  double ego_info_end_time =
      ad_byd::planning::Utility::GetCurrentTime() / 1000.0;

#if DEBUG_TIME
  std::cout << "-->-->-->-->-->-->-->-->->-->--> [ego_info_update_time]: "
            << (ego_info_end_time - ego_info_start_time) << "ms" << std::endl;
  std::cout << std::endl;
#endif

  double obj_info_start_time =
      ad_byd::planning::Utility::GetCurrentTime() / 1000.0;
  SelectedInteractiveAgents(interactive_agents, st_boundaries_with_decision,
                            st_traj_mgr, drive_passage, path);
  double obj_info_end_time =
      ad_byd::planning::Utility::GetCurrentTime() / 1000.0;

#if DEBUG_TIME
  std::cout << "-->-->-->-->-->-->-->-->->-->--> [obj_info_update_time]: "
            << (obj_info_end_time - obj_info_start_time) << "ms" << std::endl;
#endif

  double game_theory_start_time =
      ad_byd::planning::Utility::GetCurrentTime() / 1000.0;
  GameTheoryEntry(gt_results, interactive_agents, ego_interactive_info);
  double game_theory_end_time =
      ad_byd::planning::Utility::GetCurrentTime() / 1000.0;

#if DEBUG_TIME
  std::cout << "-->-->-->-->-->-->-->-->->-->--> [game_theory_update_time]: "
            << (game_theory_end_time - game_theory_start_time) << "ms"
            << std::endl;
#endif

  double postprocess_start_time =
      ad_byd::planning::Utility::GetCurrentTime() / 1000.0;
  InteractivePostProcessing(st_graph, st_traj_mgr, gt_results, path,
                            st_boundaries_with_decision);
  double postprocess_end_time =
      ad_byd::planning::Utility::GetCurrentTime() / 1000.0;

#if DEBUG_TIME
  std::cout << "-->-->-->-->-->-->-->-->->-->--> [postprocess_update_time]: "
            << (postprocess_end_time - postprocess_start_time) << "ms"
            << std::endl;
#endif

  double end_time = ad_byd::planning::Utility::GetCurrentTime() / 1000.0;

#if DEBUG_TIME
  std::cout << "-->-->-->-->-->-->-->-->->-->--> [total_game_theory_time]: "
            << end_time - start_time << "ms" << std::endl;
#endif

  return absl::OkStatus();
}

}  // namespace st::planning
