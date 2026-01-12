

#include "planner/speed_optimizer/interactive_agent_process/interactive_post_processing.h"
#include "plan_common/maps/st_boundary.h"

namespace st {
namespace planning {

#define DEBUG_BEST_GT_RES (0)

struct GameTheoryResSequence {
  double ego_acc = 0.0;
  std::vector<const GT_Interaction_Data*> res_sequence;
  double max_ego_score = 0.0;
  double max_agent_score = 0.0;
};

enum PostProcessingType {
  UPDATE_DECISION_ONLY = 0,
  UPDATE_DECISION_AND_TRAJECTORY = 1
};

std::pair<int, int> GetAccIdxRange(const double& acc, const double& min_acc,
                                   const double& acc_interval) {
  if (acc_interval < 1e-3) {
    return std::make_pair(0, -1);
  }

  int base_idx =
      std::round(std::max(0.0, acc - min_acc - acc_interval) / acc_interval);
  return std::make_pair(std::max(0, base_idx - 1), base_idx + 1);
}

std::vector<GameTheoryResSequence> GetOptimalActions(
    const std::vector<gt_result_t>& gt_results,
    const double& ego_acc_interval) {
  constexpr double kEgoInteractiveAccMin = -5.0;

  std::vector<GameTheoryResSequence> game_theory_res_sequences;
  double ego_min_acc = 0.0;

  // make a map to record optimal agent actions for each ego action
  std::map<int, std::map<std::string, const GT_Interaction_Data*>>
      optimal_agents_action;

  for (const auto& gt_result : gt_results) {
    for (int ego_idx = 0; ego_idx < gt_result.gt_interaction_matrix.row;
         ego_idx++) {
      for (int obj_idx = 0; obj_idx < gt_result.gt_interaction_matrix.col;
           obj_idx++) {
        const auto* inter_data =
            &gt_result.gt_interaction_matrix.Matrix[ego_idx][obj_idx];

        ego_min_acc = inter_data->ego_info.min_acc;
        // if (!inter_data->is_success) continue;
        // if (inter_data->is_collision) continue;

        const auto& idx_range =
            GetAccIdxRange(inter_data->ego_acc, inter_data->ego_info.min_acc,
                           inter_data->ego_info.acc_interval);

        for (int idx = idx_range.first; idx <= idx_range.second; idx++) {
          if (optimal_agents_action.find(idx) == optimal_agents_action.end()) {
            optimal_agents_action[idx] ==
                std::map<std::string, const GT_Interaction_Data*>();
          }

          auto& agents_action_map = optimal_agents_action[idx];
          std::string agent_id =
              std::string(inter_data->obj_info.obstacle_->id());
          if (agents_action_map.find(agent_id) == agents_action_map.end()) {
            agents_action_map[agent_id] = inter_data;
          } else {
            double original_total_score =
                agents_action_map[agent_id]->ego_info.average_cost +
                agents_action_map[agent_id]->obj_info.average_cost;
            double checking_total_score = inter_data->ego_info.average_cost +
                                          inter_data->obj_info.average_cost;
            if (checking_total_score < original_total_score) {
              agents_action_map[agent_id] = inter_data;
            }
          }
        }  // ego acc range for an agent action
      }    // checking every agent action from an ego action
    }      // checking every ego action
  }        // checking every agent

  for (const auto& [ego_acc_idx, agents_action] : optimal_agents_action) {
    if (agents_action.size() < gt_results.size()) {
      continue;
    }

    GameTheoryResSequence game_res_seq;
    for (const auto& [id, agent_action] : agents_action) {
      game_res_seq.res_sequence.push_back(agent_action);
      game_res_seq.max_ego_score += agent_action->ego_info.average_cost;
      game_res_seq.max_agent_score += agent_action->obj_info.average_cost;
    }
    game_res_seq.ego_acc = double(ego_acc_idx) * ego_acc_interval + ego_min_acc;

    game_theory_res_sequences.push_back(game_res_seq);
  }

  std::sort(
      game_theory_res_sequences.begin(), game_theory_res_sequences.end(),
      [](const GameTheoryResSequence& s1, const GameTheoryResSequence& s2) {
        return s1.max_ego_score + s1.max_agent_score <
               s2.max_ego_score + s2.max_agent_score;
      });

  return game_theory_res_sequences;
}

void UpdateStBoundaryWithDecision(
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    double path_end_s, const GT_Interaction_Data* agent_action,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  const std::string m_key = "|m";
  const std::string p_key = "|p";

  std::string agent_id = std::string(agent_action->obj_info.obstacle_->id());

#if DEBUG_BEST_GT_RES
  std::cout << " [post_process_traj]: agent_id " << agent_id << std::endl;
#endif

  StBoundaryWithDecision* target_st_boundary = nullptr;
  std::string target_id;

  for (auto& st_boundary_wd : *st_boundaries_with_decision) {
    if (!st_boundary_wd.object_id().has_value()) {
      continue;
    }
    if (st_boundary_wd.object_id().value() != agent_id) {
      continue;
    }

    if (st_boundary_wd.id().find(p_key) != std::string::npos) {
      continue;
    }
    if (st_boundary_wd.id().find(m_key) != std::string::npos) {
      /* may be do something, for example, record interaction decision */
    } else {
      target_st_boundary = &st_boundary_wd;
      target_id = st_boundary_wd.st_boundary()->id();
    }
    st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
    st_boundary_wd.set_decision_info(
        absl::StrCat(st_boundary_wd.decision_info(),
                     " and override decision in game theory"));
    st_boundary_wd.set_ignore_reason(
        StBoundaryProto::SAMPLY_INTERACTIVE_UPDATE);
  }

#if DEBUG_BEST_GT_RES
  std::cout << " target_st_boundary == nullptr "
            << (target_st_boundary == nullptr) << std::endl;
#endif

  if (target_st_boundary == nullptr) {
    return;
  }

  const auto& raw_st_boundary = target_st_boundary->raw_st_boundary();
  if (!raw_st_boundary->traj_id().has_value()) return;
  const auto& agent_raw_traj =
      st_traj_mgr.FindTrajectoryById(raw_st_boundary->traj_id().value());
#if DEBUG_BEST_GT_RES
  std::cout << " raw_traj_id: " << raw_st_boundary->traj_id().value()
            << std::endl;
#endif
  if (agent_raw_traj == nullptr) return;

  // Todo using different post processing type in different scenario
  PostProcessingType post_processing_type =
      PostProcessingType::UPDATE_DECISION_ONLY;

  std::vector<StBoundaryRef> new_st_boundaries;
  StBoundaryProto::DecisionType decision_type = StBoundaryProto::UNKNOWN;
  StBoundaryProto::DecisionReason decision_reason =
      StBoundaryProto::SAMPLING_INTERACTION;
  auto decision_info = absl::StrCat(target_st_boundary->decision_info(),
                                    " and override decision in game theory");

  if (post_processing_type == PostProcessingType::UPDATE_DECISION_ONLY) {
    if (agent_action->decision_type == StBoundaryProto::YIELD ||
        agent_action->decision_type == StBoundaryProto::FOLLOW) {
      decision_type = StBoundaryProto::YIELD;
    } else {
      decision_type = StBoundaryProto::IGNORE;
    }

    StBoundaryRef new_st_boundary = StBoundary::CopyInstance(*raw_st_boundary);
    new_st_boundaries.push_back(std::move(new_st_boundary));

  } else if (post_processing_type ==
             PostProcessingType::UPDATE_DECISION_AND_TRAJECTORY) {
    absl::Span<const AccelPoint> accel_point_list;
    /*  GenerateAccelPoints(agent_action, &accel_point_list);  */
    auto new_st_traj = CreateSpacetimeTrajectoryWithAccelPointList(
        accel_point_list, *agent_raw_traj);

    auto st_boundary_output = st_graph.MapMovingSpacetimeObject(
        new_st_traj, /*generate_lane_change_gap=*/false,
        /*calc_moving_close_traj=*/false, nullptr);

    for (auto& new_st_boundary : st_boundary_output.st_boundaries) {
      new_st_boundaries.push_back(std::move(new_st_boundary));
    }
  }
#if DEBUG_BEST_GT_RES
  std::cout << " [new_st_boundaries]: size " << new_st_boundaries.size()
            << std::endl;
  std::cout << " st_boundaries_with_decision: size "
            << st_boundaries_with_decision->size() << std::endl;
#endif

  for (auto& st_boundary : new_st_boundaries) {
    if (nullptr == st_boundary) continue;
    if (!st_boundary->is_protective()) {
      st_boundary->set_id(absl::StrCat(st_boundary->id(), "|gt"));

      StBoundaryWithDecision new_st_boundary_wd(
          std::move(st_boundary), decision_type, decision_reason, decision_info,
          target_st_boundary->follow_standstill_distance(),
          target_st_boundary->lead_standstill_distance(),
          target_st_boundary->pass_time(), target_st_boundary->yield_time(),
          path_end_s);
      if (decision_type == StBoundaryProto::IGNORE) {
        new_st_boundary_wd.set_ignore_reason(
            StBoundaryProto::SAMPLY_INTERACTIVE_UPDATE);
      }
      st_boundaries_with_decision->emplace_back(std::move(new_st_boundary_wd));
    }
  }

  for (auto iter = st_boundaries_with_decision->begin();
       iter != st_boundaries_with_decision->end(); iter++) {
    if (iter->st_boundary()->id() == target_id) {
      st_boundaries_with_decision->erase(iter);
      break;
    }
  }

#if DEBUG_BEST_GT_RES
  std::cout << " st_boundaries_with_decision-p: size "
            << st_boundaries_with_decision->size() << std::endl;
  std::cout << std::endl;
#endif

  return;
}

void InteractivePostProcessing(
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<gt_result_t>& gt_results, const DiscretizedPath& path,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    double ego_acc_interval) {
  if (st_boundaries_with_decision == nullptr) {
    return;
  }
  // 1.0 Get optimal actions
  std::vector<GameTheoryResSequence> game_theory_res_sequences;
  game_theory_res_sequences = GetOptimalActions(gt_results, ego_acc_interval);
  if (game_theory_res_sequences.empty()) {
    return;
  }
  GameTheoryResSequence& optimal_ego_action = game_theory_res_sequences.front();
  std::vector<const GT_Interaction_Data*> optimal_agents_action =
      optimal_ego_action.res_sequence;

#if DEBUG_BEST_GT_RES
  for (const auto& agent_action : optimal_agents_action) {
    std::cout << " [best_gt_result]: obj_id "
              << agent_action->obj_info.obstacle_->id() << " decision_type "
              << agent_action->decision_type << " ego_acc "
              << agent_action->ego_acc << " obj_acc " << agent_action->agent_acc
              << " ego_cost " << agent_action->ego_info.average_cost
              << " obj_cost " << agent_action->obj_info.average_cost
              << std::endl;
  }
  std::cout << std::endl;
#endif

  // 2.0 Make decision or Modify traj by optimal actions
  for (const auto* agent_action : optimal_agents_action) {
    UpdateStBoundaryWithDecision(st_graph, st_traj_mgr, path.length(),
                                 agent_action, st_boundaries_with_decision);
  }
}
}  // namespace planning
}  // namespace st
