

#include "planner/speed_optimizer/interactive_agent_process/game_theory/game_theory_environment.h"

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

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/meta/type_traits.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "plan_common/log.h"
#include "gflags/gflags.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/intelligent_driver_model.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/planner_object.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/overlap_info.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_point.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/speed/st_speed/speed_point.h"
#include "plan_common/speed/st_speed/speed_profile.h"
#include "plan_common/timer.h"
#include "plan_common/trajectory_util.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_defs.h"
#include "predictor/prediction_object_state.h"
#include "planner/speed_optimizer/decider/interaction_util.h"
#include "planner/speed_optimizer/decider/post_st_boundary_modifier.h"
#include "planner/speed_optimizer/decider/pre_brake_decider.h"
#include "planner/speed_optimizer/decider/st_boundary_modifier_util.h"
#include "planner/speed_optimizer/empty_road_speed.h"
#include "planner/speed_optimizer/gridded_svt_graph.h"
#include "planner/speed_optimizer/st_graph_data.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {

#define DEBUG_GT (0)
#define DEBUG_SINGLE (0)
#define DEBUG_DECISION_TYPE (0)

static char *st_decision_str[6] = {(char *)" None  ", (char *)" Follw ",
                                   (char *)" Overt ", (char *)" Ignor ",
                                   (char *)" Yield ", (char *)" Colls "};

const char *StDecisionToStr(int decision) {
  return (decision >= 0 && decision < 6) ? st_decision_str[decision]
                                         : (char *)"unknown";
}

void Game_Theory_Interaction_Environment::DumpCostMatrixSimple(
    const GT_Interaction_Matrix *gt_interaction_matrix) {
  int32_t ego_acc_size, obj_acc_size, i, j;
  const GT_Interaction_Data *inter_data;

  if (nullptr == gt_interaction_matrix) return;

  ego_acc_size = gt_interaction_matrix->row;
  obj_acc_size = gt_interaction_matrix->col;

  for (int i = 0; i < obj_acc_size; i++) {
    if (0 == i) {
      printf("  %30.2f| ", gt_interaction_matrix->Matrix[0][i].agent_acc);
    } else {
      printf("  %22.2f| ", gt_interaction_matrix->Matrix[0][i].agent_acc);
    }
  }
  printf("\n");

  for (j = 0; j < ego_acc_size; j++) {
    for (i = 0; i < obj_acc_size; i++) {
      inter_data = &gt_interaction_matrix->Matrix[j][i];

      if (0 == i) {
        if (!lx_fless(gt_interaction_matrix->Matrix[j][i].ego_acc, 0.0)) {
          printf("+");
        }
        printf("|%3.2f|", gt_interaction_matrix->Matrix[j][i].ego_acc);
      }
      double average_ego_cost = inter_data->ego_info.average_cost;
      double average_obj_cost = inter_data->obj_info.average_cost;
      if (gt_interaction_matrix->Matrix[j][i].is_collision) {
        printf("%6.2f * %6.2f * %s|", average_ego_cost, average_obj_cost,
               StDecisionToStr(5));
      } else {
        printf("%6.2f * %6.2f * %s|", average_ego_cost, average_obj_cost,
               StDecisionToStr(static_cast<int>(inter_data->decision_type)));
      }
    }
    printf("\n");
  }

  return;
}

Game_Theory_Interaction_Environment::Game_Theory_Interaction_Environment() {
  this->agent_array_.reserve(MAX_GT_ACC_NUM);
  this->ego_array_.reserve(MAX_GT_ACC_NUM);
  this->ego_traj_sim_state_.reserve(MAX_GT_ACC_NUM);
  this->obj_traj_sim_state_.reserve(MAX_GT_ACC_NUM);
  this->gt_inter_matrix_.col = 0;
  this->gt_inter_matrix_.row = 0;
  this->is_valid = true;
}

void Game_Theory_Interaction_Environment::InitGameTheoryEnvironment(
    InteractiveAgent *ego_interactive_info,
    InteractiveAgent *obj_interactive_info) {
  this->ego_interactive_info_ = ego_interactive_info;
  this->obj_interactive_info_ = obj_interactive_info;

  /* update gt params */

  gt_static_params_t *static_params = &this->static_params_;
  static_params->ego_sampling_interval = 0.2;
  static_params->agent_sampling_interval = 1.0;
  static_params->time_step = 0.5;
  static_params->max_simulation_time = 8.0;
  static_params->max_simulation_dist = 150.0;
  static_params->ego_acc_to_acc_delay_time = 0.2;
  static_params->ego_acc_to_dec_delay_time = 0.5;
  static_params->ego_dec_to_acc_delay_time = 0.2;
  static_params->ego_dec_to_dec_delay_time = 0.2;
  static_params->ego_acc_default_delay_time = 0.2;
  static_params->consider_ego_jerk = true;
  static_params->ego_max_acc_jerk = 2.0;
  static_params->ego_max_dec_jerk = 3.0;
  return;
}

void Game_Theory_Interaction_Environment::UpdateAccArrayForBothPlayer(
    std::vector<double> &agent_array, std::vector<double> &ego_array) {
  agent_array.clear();
  ego_array.clear();

  double ego_max_acc = this->ego_interactive_info_->param()->inter_max_acc();
  double ego_min_acc = this->ego_interactive_info_->param()->inter_min_acc();
  double agent_max_acc = this->obj_interactive_info_->param()->inter_max_acc();
  double agent_min_acc = this->obj_interactive_info_->param()->inter_min_acc();

  double ego_sampling_interval = this->static_params_.ego_sampling_interval;
  double agent_sampling_interval = this->static_params_.agent_sampling_interval;

  double col_range = agent_max_acc - agent_min_acc;
  double row_range = ego_max_acc - ego_min_acc;
  int total_col = std::min(
      static_cast<int>(std::ceil(col_range / agent_sampling_interval) + 1),
      MAX_GT_ACC_NUM);
  int total_row = std::min(
      static_cast<int>(std::ceil(row_range / ego_sampling_interval) + 1),
      MAX_GT_ACC_NUM);

  /* todo: here can according to scenario to dynamic gt sampling */

  /* update ego array */
  double tmp_sampling_acc = ego_min_acc;
  for (int i = 0; i < total_row; i++) {
    if (0 == i) {
      ego_array.emplace_back(tmp_sampling_acc);
    } else {
      tmp_sampling_acc += ego_sampling_interval;
      tmp_sampling_acc = std::min(tmp_sampling_acc, ego_max_acc);
      ego_array.emplace_back(tmp_sampling_acc);
    }
  }

  /* update agent array */
  tmp_sampling_acc = agent_min_acc;
  for (int i = 0; i < total_col; i++) {
    if (0 == i) {
      agent_array.emplace_back(tmp_sampling_acc);
    } else {
      tmp_sampling_acc += agent_sampling_interval;
      tmp_sampling_acc = std::min(tmp_sampling_acc, agent_max_acc);
      agent_array.emplace_back(tmp_sampling_acc);
    }
  }

  return;
}

void Game_Theory_Interaction_Environment::SetUpEgoInitState(
    gt_ego_sim_state_t &gt_ego_sim_state,
    const InteractiveAgent *ego_interactive_info) {
  if (nullptr == ego_interactive_info) return;

  gt_ego_state_t ego_state;

  ego_state.traj_point_idx = 0;
  ego_state.sl_center_pose = ego_interactive_info->state()[0].sl_center_pose;
  ego_state.frenet_box = ego_interactive_info->state()[0].frenet_box;
  ego_state.sl_vel = ego_interactive_info->state()[0].sl_vel;
  ego_state.global_center_pose =
      ego_interactive_info->state()[0].global_center_pose;
  ego_state.global_polygon = ego_interactive_info->state()[0].global_polygon;
  ego_state.vel = ego_interactive_info->init_vel();
  ego_state.acc = ego_interactive_info->init_acc();
  ego_state.theta = ego_interactive_info->init_theta();
  ego_state.time_stamp = 0.0;
  ego_state.dist = 0.0;
  ego_state.is_legal = true;

  gt_ego_sim_state.states.emplace_back(ego_state);

  return;
}

void Game_Theory_Interaction_Environment::SetUpObjInitState(
    gt_obj_sim_state_t &gt_obj_sim_state,
    const InteractiveAgent *obj_interactive_info) {
  if (nullptr == obj_interactive_info) return;

  gt_agent_state_t agent_state;

  agent_state.traj_point_idx = 0;
  agent_state.sl_center_pose = obj_interactive_info->state()[0].sl_center_pose;
  agent_state.frenet_box = obj_interactive_info->state()[0].frenet_box;
  agent_state.sl_vel = obj_interactive_info->state()[0].sl_vel;
  agent_state.global_center_pose =
      obj_interactive_info->state()[0].global_center_pose;
  agent_state.global_polygon = obj_interactive_info->state()[0].global_polygon;
  agent_state.vel = obj_interactive_info->init_vel();
  agent_state.acc = obj_interactive_info->init_acc();
  agent_state.theta = obj_interactive_info->init_theta();
  agent_state.time_stamp = 0.0;
  agent_state.dist = 0.0;
  agent_state.is_legal = true;

  gt_obj_sim_state.states.emplace_back(agent_state);

  return;
}

void Game_Theory_Interaction_Environment::SetUpInteractionMatrix() {
  this->gt_inter_matrix_.row = static_cast<int>(ego_array_.size());
  this->gt_inter_matrix_.col = static_cast<int>(agent_array_.size());

  for (int ego_idx = 0; ego_idx < this->gt_inter_matrix_.row; ego_idx++) {
    double ego_acc = ego_array_[ego_idx];
    for (int agent_idx = 0; agent_idx < this->gt_inter_matrix_.col;
         agent_idx++) {
      double obj_acc = agent_array_[agent_idx];
      auto &single_ceil = this->gt_inter_matrix_.Matrix[ego_idx][agent_idx];
      single_ceil.agent_acc = obj_acc;
      single_ceil.ego_acc = ego_acc;
      single_ceil.is_success = true;
      single_ceil.is_collision = false;
      single_ceil.decision_type = StBoundaryProto::UNKNOWN;

      /* update ego gt init state */
      single_ceil.ego_info.max_acc =
          this->ego_interactive_info_->param()->inter_max_acc();
      single_ceil.ego_info.min_acc =
          this->ego_interactive_info_->param()->inter_min_acc();
      single_ceil.ego_info.max_vel =
          this->ego_interactive_info_->param()->max_vel();

      /* update agent gt init state */
      single_ceil.obj_info.max_acc =
          this->obj_interactive_info_->param()->inter_max_acc();
      single_ceil.obj_info.min_acc =
          this->obj_interactive_info_->param()->inter_min_acc();
      single_ceil.obj_info.max_vel =
          this->obj_interactive_info_->param()->max_vel();
      single_ceil.obj_info.obstacle_ = this->obj_interactive_info_;
    }
  }

  return;
}

void Game_Theory_Interaction_Environment::FindSimPoseFromEgoTraj(
    gt_ego_state_t *next_state, const gt_ego_state_t *curr_state) {
  if (nullptr == next_state || nullptr == curr_state) return;

  int search_start_idx = curr_state->traj_point_idx;

  ego_interactive_info_->GetEgoStateInfoAccordingToDist(
      &next_state->frenet_box, &next_state->global_polygon,
      &next_state->global_center_pose, &next_state->traj_point_idx,
      next_state->dist, search_start_idx);

  ego_interactive_info_->CvtGlobalPoseToSlCenterPose(
      &next_state->sl_center_pose, &next_state->global_center_pose, false);

  next_state->theta = next_state->global_center_pose.theta();

  next_state->sl_vel.s_dt =
      next_state->vel * std::cos(next_state->sl_center_pose.theta());
  next_state->sl_vel.l_dt =
      next_state->vel * std::sin(next_state->sl_center_pose.theta());
  next_state->is_legal = true;
}

std::optional<gt_ego_sim_state_t>
Game_Theory_Interaction_Environment::UpdateEgoActionSimState(double ego_acc) {
  gt_ego_sim_state_t gt_ego_sim_state;

  double time_step = this->static_params_.time_step;

  gt_ego_sim_state.states.reserve(MAX_STATE_NUM);

  /* consider delay for ego */
  double delay_time = static_params_.ego_acc_default_delay_time;
  double ego_init_acc = this->ego_interactive_info_->init_acc();
  if (std::fabs(ego_init_acc - ego_acc) > 1.0) {
    if (ego_init_acc >= 0 && ego_acc >= 0) {
      delay_time = static_params_.ego_acc_to_acc_delay_time;
    } else if (ego_init_acc >= 0 && ego_acc < 0) {
      delay_time = static_params_.ego_acc_to_dec_delay_time;
    } else if (ego_init_acc < 0 && ego_acc >= 0) {
      delay_time = static_params_.ego_dec_to_acc_delay_time;
    } else if (ego_init_acc < 0 && ego_acc < 0) {
      delay_time = static_params_.ego_dec_to_dec_delay_time;
    }
  }

  SetUpEgoInitState(gt_ego_sim_state, this->ego_interactive_info_);

  for (int i = 0; i < (MAX_STATE_NUM - 1); i++) {
    gt_ego_state_t *curr_state = &gt_ego_sim_state.states[i];
    gt_ego_state_t next_state;
    next_state.time_stamp = curr_state->time_stamp + time_step;

    if (next_state.time_stamp <= delay_time) {
      next_state.acc = curr_state->acc;
    } else {
      if (static_params_.consider_ego_jerk) {
        next_state.acc = std::clamp(
            ego_acc,
            curr_state->acc - static_params_.ego_max_dec_jerk * time_step,
            curr_state->acc + static_params_.ego_max_acc_jerk * time_step);
        next_state.acc = std::clamp(next_state.acc,
                                    ego_interactive_info_->param()->min_acc(),
                                    ego_interactive_info_->param()->max_acc());
      } else {
        next_state.acc = ego_acc;
      }
    }

    next_state.vel = curr_state->vel + next_state.acc * time_step;

    /* todo: import GetSpeedLimitByTimeAndS function to update max_vel */
    double actual_max_vel = ego_interactive_info_->param()->max_vel();
    double actual_min_vel = 0.0;
    next_state.vel = std::clamp(next_state.vel, actual_min_vel, actual_max_vel);
    next_state.dist = curr_state->dist + time_step * next_state.vel;
    FindSimPoseFromEgoTraj(&next_state, curr_state);
    gt_ego_sim_state.states.emplace_back(next_state);
    if (!lx_fless(next_state.time_stamp,
                  this->static_params_.max_simulation_time) ||
        !lx_fless(next_state.dist, this->static_params_.max_simulation_dist)) {
      break;
    }
  }

  return gt_ego_sim_state;
}

void Game_Theory_Interaction_Environment::UpdateEgoAllActionSimState() {
  for (const auto &ego_acc : ego_array_) {
    std::optional<gt_ego_sim_state_t> gt_ego_sim_state =
        UpdateEgoActionSimState(ego_acc);
    if (gt_ego_sim_state.has_value()) {
      ego_traj_sim_state_.insert(
          {ego_acc, std::move(gt_ego_sim_state.value())});
    }
  }
}

void Game_Theory_Interaction_Environment::FindSimPoseFromObjTraj(
    gt_agent_state_t *next_state, const gt_agent_state_t *curr_state) {
  if (nullptr == curr_state || nullptr == next_state) return;

  int search_start_idx = curr_state->traj_point_idx;

  obj_interactive_info_->GetAgentStateInfoAccordingToDist(
      &next_state->frenet_box, &next_state->global_polygon,
      &next_state->global_center_pose, &next_state->traj_point_idx,
      next_state->dist, search_start_idx);

  obj_interactive_info_->CvtGlobalPoseToSlCenterPose(
      &next_state->sl_center_pose, &next_state->global_center_pose, false);

  next_state->theta = next_state->global_center_pose.theta();

  next_state->sl_vel.s_dt =
      next_state->vel * std::cos(next_state->sl_center_pose.theta());
  next_state->sl_vel.l_dt =
      next_state->vel * std::sin(next_state->sl_center_pose.theta());
  next_state->is_legal = true;

  return;
}

std::optional<gt_obj_sim_state_t>
Game_Theory_Interaction_Environment::UpdateObjActionSimState(double obj_acc) {
  gt_obj_sim_state_t gt_obj_sim_state;

  gt_obj_sim_state.states.reserve(MAX_STATE_NUM);

  double time_step = this->static_params_.time_step;

  /* consider delay for ego */

  SetUpObjInitState(gt_obj_sim_state, this->obj_interactive_info_);

  for (int i = 0; i < (MAX_STATE_NUM - 1); i++) {
    gt_agent_state_t *curr_state = &gt_obj_sim_state.states[i];
    gt_agent_state_t next_state;
    next_state.time_stamp = curr_state->time_stamp + time_step;
    next_state.acc =
        std::clamp(obj_acc, obj_interactive_info_->param()->min_acc(),
                   obj_interactive_info_->param()->max_acc());

    next_state.vel = curr_state->vel + next_state.acc * time_step;

    double actual_max_vel = obj_interactive_info_->param()->max_vel();
    double actual_min_vel = 0.0;
    next_state.vel = std::clamp(next_state.vel, actual_min_vel, actual_max_vel);
    next_state.dist = curr_state->dist + time_step * next_state.vel;
    FindSimPoseFromObjTraj(&next_state, curr_state);
    gt_obj_sim_state.states.emplace_back(next_state);
    if (!lx_fless(next_state.time_stamp,
                  this->static_params_.max_simulation_time) ||
        !lx_fless(next_state.dist, this->static_params_.max_simulation_dist)) {
      break;
    }
  }

  return gt_obj_sim_state;
}

void Game_Theory_Interaction_Environment::UpdateObjAllActionSimState() {
  for (const auto &obj_acc : agent_array_) {
    std::optional<gt_obj_sim_state_t> gt_obj_sim_state =
        UpdateObjActionSimState(obj_acc);
    if (gt_obj_sim_state.has_value()) {
      obj_traj_sim_state_.insert(
          {obj_acc, std::move(gt_obj_sim_state.value())});
    }
  }
}

void Game_Theory_Interaction_Environment::BuildUpGameEnvironment() {
  this->agent_array_.clear();
  this->ego_array_.clear();

  this->UpdateAccArrayForBothPlayer(agent_array_, ego_array_);
  this->SetUpInteractionMatrix();

  return;
}

void Game_Theory_Interaction_Environment::IgnoreUnrealizedStrategy(
    bool *is_realizable_strategy, const double ego_strategy_acc,
    const double ego_max_acc) {
  /* todo: ignore unreasonable strategy */

  *is_realizable_strategy = false;
  if (!lx_fgreater(
          ego_strategy_acc,
          ego_max_acc + this->static_params_.ego_sampling_interval / 2.0)) {
    *is_realizable_strategy = true;
  }

  return;
}

void Game_Theory_Interaction_Environment::UpdateDecisionResult(
    GT_Interaction_Data *inter_data) {
  if (nullptr == inter_data) return;

  inter_data->decision_type = StBoundaryProto::UNKNOWN;
  bool is_ego_first_approach_to_collision = false;
  bool is_obj_first_approach_to_collision = false;
  double obj_half_length = obj_interactive_info_->param()->length() / 2.0;
  double ego_half_length = ego_interactive_info_->param()->length() / 2.0;
  double obj_dist_to_enter_collision =
      obj_interactive_info_->collision_info()->obj_dist_to_enter_collision;
  double ego_dist_to_leave_collision =
      obj_interactive_info_->collision_info()->ego_dist_to_leave_collision;

  bool is_debug = false;
  double target_ego_acc = -2.2;
  double target_obj_acc = 0.0;

#if DEBUG_DECISION_TYPE
  if (lx_fequal(inter_data->ego_acc, target_ego_acc) &&
      lx_fequal(inter_data->agent_acc, target_obj_acc)) {
    is_debug = true;
  }
#endif
  if (is_debug) {
    std::cout << " ego_acc: " << inter_data->ego_acc
              << " obj_acc: " << inter_data->agent_acc << std::endl;
  }

  for (int i = 0; i < (MAX_STATE_NUM - 1); i++) {
    if (i >= inter_data->ego_info.states.size()) break;
    gt_ego_state_t *curr_ego_state = inter_data->ego_info.states[i];
    if (i >= inter_data->obj_info.states.size()) break;
    gt_agent_state_t *curr_obj_state = inter_data->obj_info.states[i];

    Box2d obj_box =
        Box2d(curr_obj_state->global_polygon.CircleCenter(),
              curr_obj_state->theta, obj_interactive_info_->param()->length(),
              obj_interactive_info_->param()->width());

    if (is_debug) {
      std::cout << " ego_global_polygon: size "
                << curr_ego_state->global_polygon.points().size() << std::endl;
      for (const auto &ego_point : curr_ego_state->global_polygon.points()) {
        std::cout << " x: " << ego_point.x() << " y: " << ego_point.y()
                  << std::endl;
      }
      std::cout << " obj_global_polygon: size "
                << curr_obj_state->global_polygon.points().size() << std::endl;
      for (const auto &obj_point : curr_obj_state->global_polygon.points()) {
        std::cout << " x: " << obj_point.x() << " y: " << obj_point.y()
                  << std::endl;
      }
    }

    bool has_overlap =
        curr_ego_state->global_polygon.HasOverlapWithBuffer(obj_box, 0.2, 0.2);
    if (is_debug) {
      std::cout << " time_step: " << inter_data->ego_info.states[i]->time_stamp
                << " has_overlap: " << has_overlap
                << " obj_dist: " << curr_obj_state->dist
                << " ego_dist: " << curr_ego_state->dist
                << " ego_dist_to_leave_collision "
                << ego_dist_to_leave_collision
                << " obj_dist_to_enter_collision "
                << obj_dist_to_enter_collision << std::endl;
    }
    if (has_overlap) {
      inter_data->is_collision = true;
      inter_data->decision_type = StBoundaryProto::YIELD;
      break;
    } else {
      if ((curr_obj_state->dist) >= obj_dist_to_enter_collision) {
        is_obj_first_approach_to_collision = true;
      }
      if ((curr_ego_state->dist) >= (ego_dist_to_leave_collision + 5.0)) {
        is_ego_first_approach_to_collision = true;
      }
      if (is_debug) {
        std::cout << " is_obj_first_approach_to_collision "
                  << is_obj_first_approach_to_collision
                  << " is_ego_first_approach_to_collision "
                  << is_ego_first_approach_to_collision << std::endl;
      }

      if (is_obj_first_approach_to_collision &&
          is_ego_first_approach_to_collision) {
        inter_data->is_collision = true;
        inter_data->decision_type = StBoundaryProto::YIELD;
        break;
      } else if (is_obj_first_approach_to_collision &&
                 !is_ego_first_approach_to_collision) {
        inter_data->is_collision = false;
        inter_data->decision_type = StBoundaryProto::YIELD;
        break;
      } else if (!is_obj_first_approach_to_collision &&
                 is_ego_first_approach_to_collision) {
        inter_data->is_collision = false;
        inter_data->decision_type = StBoundaryProto::OVERTAKE;
        break;
      }
    }
  }

  /* here Follow represent tow agents can not arrive collision */
  if (!is_obj_first_approach_to_collision &&
      !is_ego_first_approach_to_collision && !inter_data->is_collision) {
    inter_data->is_collision = false;
    inter_data->decision_type = StBoundaryProto::FOLLOW;
  }

  if (is_debug) {
    std::cout << " decision: " << StDecisionToStr(inter_data->decision_type)
              << "  " << inter_data->decision_type << std::endl;
    std::cout << std::endl;
  }

  return;
}

void Game_Theory_Interaction_Environment::SelectedCostFuncUseScenario(
    gt_agent_state_t *obj_state, gt_ego_state_t *ego_state,
    const game_theory_scenario_t game_theory_scenario,
    const StBoundaryProto::DecisionType decision_type,
    const double ego_sampling_acc, const double obj_sampling_acc,
    const int step, const bool is_debug_cost) {
  if (is_debug_cost) {
    std::cout << " game_theory_scenario " << game_theory_scenario << std::endl;
  }

  if (nullptr == obj_state || nullptr == ego_state) return;

  switch (game_theory_scenario) {
    case GAME_THEORY_SCENARIO_UNKNOWN:
    case GAME_THEORY_SCENARIO_CROSS_STR_STR:
    case GAME_THEORY_SCENARIO_CROSS_STR_TL:
    case GAME_THEORY_SCENARIO_CROSS_TL_STR:
    case GAME_THEORY_SCENARIO_CROSS_STR_TR:
    case GAME_THEORY_SCENARIO_CROSS_TR_STR:
    case GAME_THEORY_SCENARIO_CROSS_TL_TL:
    case GAME_THEORY_SCENARIO_CROSS_TR_TR:
    case GAME_THEORY_SCENARIO_CROSS_UTURN_STR:
    case GAME_THEORY_SCENARIO_CROSS_STR_UTURN:
    case GAME_THEORY_SCENARIO_MERGE_PARALLEL:
    case GAME_THEORY_SCENARIO_MERGE_STR_TL:
    case GAME_THEORY_SCENARIO_MERGE_TL_STR:
    case GAME_THEORY_SCENARIO_MERGE_STR_TR:
    case GAME_THEORY_SCENARIO_MERGE_TR_STR:
    case GAME_THEORY_SCENARIO_MERGE_TL_TL:
    case GAME_THEORY_SCENARIO_MERGE_TR_TR:
    case GAME_THEORY_SCENARIO_MERGE_UTURN_STR:
    case GAME_THEORY_SCENARIO_MERGE_STR_UTURN:
    case GAME_THEORY_SCENARIO_MERGE_LANE_CHANGE:
    case GAME_THEORY_SCENARIO_ONCOMMING:
    case GAME_THEORY_SCENARIO_FOLLOW:
      turn_left_cost_.GameTheoryCostUpdate(
          ego_state, obj_state, ego_interactive_info_->param()->length(),
          ego_interactive_info_->param()->width(),
          obj_interactive_info_->param()->length(),
          obj_interactive_info_->param()->width(),
          obj_interactive_info_->collision_info()->ego_dist_to_enter_collision,
          obj_interactive_info_->collision_info()->ego_dist_to_leave_collision,
          obj_interactive_info_->collision_info()->obj_dist_to_enter_collision,
          obj_interactive_info_->collision_info()->obj_dist_to_leave_collision,
          decision_type, ego_interactive_info_->init_acc(),
          obj_interactive_info_->init_acc(), ego_sampling_acc, obj_sampling_acc,
          step, is_debug_cost, game_theory_scenario);
      break;
    default:
      std::cout << " scene error " << std::endl;
  }

  return;
}

bool Game_Theory_Interaction_Environment::IsSimulationStop(
    const gt_ego_state_t *ego_state, const gt_agent_state_t *obj_state) {
  const auto scenario = this->obj_interactive_info_->st_boundary_decision()
                            ->st_boundary()
                            ->obj_scenario_info()
                            .relationship;
  double ego_dist_to_leave_collision =
      this->obj_interactive_info_->collision_info()
          ->ego_dist_to_leave_collision;
  double obj_dist_to_enter_collision =
      this->obj_interactive_info_->collision_info()
          ->obj_dist_to_enter_collision;
  if (Relationship::Cross == scenario) {
    if (ego_state->dist >= ego_dist_to_leave_collision ||
        obj_state->dist >= obj_dist_to_enter_collision) {
      return true;
    }
  }

  return false;
}

void Game_Theory_Interaction_Environment::UpdateGtSimStateByAction(
    GT_Interaction_Data *inter_data) {
  bool is_stop = false;

  if (nullptr == inter_data) {
    return;
  }

  int obj_state_size = obj_traj_sim_state_[inter_data->agent_acc].states.size();
  int ego_state_size = ego_traj_sim_state_[inter_data->ego_acc].states.size();

  inter_data->obj_info.states.reserve(obj_state_size);
  inter_data->ego_info.states.reserve(ego_state_size);

  for (int i = 0; i < ego_state_size; i++) {
    inter_data->ego_info.states.emplace_back(
        &ego_traj_sim_state_[inter_data->ego_acc].states[i]);
  }
  for (int i = 0; i < obj_state_size; i++) {
    inter_data->obj_info.states.emplace_back(
        &obj_traj_sim_state_[inter_data->agent_acc].states[i]);
  }
  inter_data->ego_info.size = 0;
  inter_data->obj_info.size = 0;

  UpdateDecisionResult(inter_data);

  if (!inter_data->is_success) return;

  bool is_debug_cost = false;
  double ego_target_acc = -0.6;
  double obj_target_acc = 0.0;

#if DEBUG_SINGLE
  if (lx_fequal(ego_target_acc, inter_data->ego_acc) &&
      lx_fequal(obj_target_acc, inter_data->agent_acc)) {
    is_debug_cost = true;
  }
#endif

  inter_data->ego_info.safety_cost = 0.0;
  inter_data->ego_info.law_cost = 0.0;
  inter_data->ego_info.acc_cost = 0.0;
  inter_data->obj_info.safety_cost = 0.0;
  inter_data->obj_info.law_cost = 0.0;
  inter_data->obj_info.acc_cost = 0.0;
  inter_data->ego_info.max_acc =
      this->ego_interactive_info_->param()->inter_max_acc();
  inter_data->ego_info.min_acc =
      this->ego_interactive_info_->param()->inter_min_acc();
  inter_data->ego_info.acc_interval =
      this->static_params_.ego_sampling_interval;

  inter_data->obj_info.max_acc =
      this->obj_interactive_info_->param()->inter_max_acc();
  inter_data->obj_info.min_acc =
      this->obj_interactive_info_->param()->inter_min_acc();
  inter_data->obj_info.acc_interval =
      this->static_params_.agent_sampling_interval;

  for (int i = 0; i < MAX_STATE_NUM; i++) {
    /* start at i, update to i + 1 */
    if (i >= inter_data->ego_info.states.size()) break;
    gt_ego_state_t *ego_state = inter_data->ego_info.states[i];
    if (i >= inter_data->obj_info.states.size()) break;
    gt_agent_state_t *obj_state = inter_data->obj_info.states[i];

    if (i > 0) {
      if (lx_fequal(obj_state->vel, 0.0)) {
        obj_state->wait_time = inter_data->obj_info.states[i - 1]->wait_time +
                               this->static_params_.time_step;
      }
      if (lx_fequal(ego_state->vel, 0.0)) {
        ego_state->wait_time = inter_data->ego_info.states[i - 1]->wait_time +
                               this->static_params_.time_step;
      }
    }

    SelectedCostFuncUseScenario(obj_state, ego_state,
                                obj_interactive_info_->game_theory_scenario(),
                                inter_data->decision_type, inter_data->ego_acc,
                                inter_data->agent_acc, i, is_debug_cost);

    inter_data->ego_info.safety_cost += ego_state->safety_cost;
    inter_data->ego_info.law_cost += ego_state->law_cost;
    inter_data->ego_info.acc_cost += ego_state->acc_cost;
    inter_data->ego_info.size++;

    inter_data->obj_info.safety_cost += obj_state->safety_cost;
    inter_data->obj_info.law_cost += obj_state->law_cost;
    inter_data->obj_info.acc_cost += obj_state->acc_cost;
    inter_data->obj_info.size++;
    if (IsSimulationStop(ego_state, obj_state)) break;
  }

  inter_data->ego_info.total_cost = inter_data->ego_info.safety_cost +
                                    inter_data->ego_info.law_cost +
                                    inter_data->ego_info.acc_cost;

  inter_data->obj_info.total_cost = inter_data->obj_info.safety_cost +
                                    inter_data->obj_info.law_cost +
                                    inter_data->obj_info.acc_cost;

  if (0 != inter_data->ego_info.size && 0 != inter_data->obj_info.size) {
    inter_data->ego_info.average_cost =
        inter_data->ego_info.total_cost / inter_data->ego_info.size;
    inter_data->obj_info.average_cost =
        inter_data->obj_info.total_cost / inter_data->obj_info.size;
  }

  if (is_debug_cost) {
    std::cout << " ego: [ " << inter_data->ego_info.safety_cost << "  "
              << inter_data->ego_info.law_cost << "  "
              << inter_data->ego_info.acc_cost << " ] "
              << " total_cost: " << inter_data->ego_info.total_cost
              << " average_cost: " << inter_data->ego_info.average_cost
              << " ego_size: " << inter_data->ego_info.size << std::endl;
    std::cout << " obj: [ " << inter_data->obj_info.safety_cost << "  "
              << inter_data->obj_info.law_cost << "  "
              << inter_data->obj_info.acc_cost << " ] "
              << " total_cost: " << inter_data->obj_info.total_cost
              << " average_cost: " << inter_data->obj_info.average_cost
              << " ego_size: " << inter_data->obj_info.size << std::endl;
  }

  return;
}

void Game_Theory_Interaction_Environment::ComputeExptAcc(double *acc,
                                                         double start_vel,
                                                         double end_vel,
                                                         double delta_dist) {
  if (nullptr == acc) return;

  *acc = 0.0;
  if (lx_fequal(delta_dist, 0.0)) {
    return;
  }

  *acc = (end_vel * end_vel - start_vel * start_vel) / (2 * delta_dist);

  return;
}

void Game_Theory_Interaction_Environment::CalcEgoMaxAccBySpeedLimit(
    double *ego_max_acc, const double ego_init_vel,
    const double ego_enter_collision_max_vel,
    const double ego_leave_collision_max_vel) {
  double ego_dist_to_collision_pt, ego_dist_leave_collision_pt;
  double ego_start_calc_acc, ego_end_calc_acc, ego_vel;

  ego_dist_to_collision_pt = this->obj_interactive_info_->collision_info()
                                 ->ego_dist_to_enter_collision;
  ego_dist_leave_collision_pt = this->obj_interactive_info_->collision_info()
                                    ->ego_dist_to_leave_collision;
  ego_vel = ego_init_vel;

  ego_start_calc_acc = std::numeric_limits<double>::max();
  if (lx_fgreater(ego_dist_to_collision_pt, 0.0)) {
    ComputeExptAcc(&ego_start_calc_acc, ego_vel, ego_enter_collision_max_vel,
                   ego_dist_to_collision_pt);
  }

  ego_end_calc_acc = std::numeric_limits<double>::max();
  if (lx_fgreater(ego_dist_leave_collision_pt, 0.0)) {
    ComputeExptAcc(&ego_end_calc_acc, ego_vel, ego_leave_collision_max_vel,
                   ego_dist_leave_collision_pt);
  }

  if (lx_fless(ego_end_calc_acc, std::numeric_limits<double>::max()) &&
      lx_fless(ego_start_calc_acc, std::numeric_limits<double>::max())) {
    *ego_max_acc = (ego_start_calc_acc + ego_end_calc_acc) / 2.0;
  } else if (!lx_fless(ego_end_calc_acc, std::numeric_limits<double>::max()) &&
             !lx_fless(ego_start_calc_acc,
                       std::numeric_limits<double>::max())) {
    *ego_max_acc = 0.0;
  } else {
    if (lx_fless(ego_end_calc_acc, std::numeric_limits<double>::max())) {
      *ego_max_acc = ego_end_calc_acc;
    } else {
      *ego_max_acc = ego_start_calc_acc;
    }
  }

  return;
}

void Game_Theory_Interaction_Environment::StartStackelbergGameProcess(
    gt_result_t *result) {
  /* start_simulation */

  UpdateEgoAllActionSimState();
  UpdateObjAllActionSimState();

  double ego_max_acc_by_speed_limit;
  const double ego_enter_collision_max_vel = 60.0;
  const double ego_leave_collision_max_vel = 60.0;

#if DEBUG_GT || DEBUG_SINGLE
  std::cout << "--------------------gt debug "
            << this->obj_interactive_info_->id() << " ----------------------"
            << std::endl;
#endif

  /* todo: use speed limit to calc */
  CalcEgoMaxAccBySpeedLimit(
      &ego_max_acc_by_speed_limit, this->ego_interactive_info_->init_vel(),
      ego_enter_collision_max_vel, ego_leave_collision_max_vel);

  for (int ego_idx = 0; ego_idx < this->gt_inter_matrix_.row; ego_idx++) {
    for (int obj_idx = 0; obj_idx < this->gt_inter_matrix_.col; obj_idx++) {
      bool is_realizable_strategy;
      GT_Interaction_Data *inter_data =
          &this->gt_inter_matrix_.Matrix[ego_idx][obj_idx];
      IgnoreUnrealizedStrategy(&is_realizable_strategy, inter_data->ego_acc,
                               ego_max_acc_by_speed_limit);
      if (!is_realizable_strategy) {
        inter_data->is_success = false;
      } else {
        UpdateGtSimStateByAction(inter_data);
      }
    }
  }

  result->gt_interaction_matrix = this->gt_inter_matrix_;
  result->is_valid = true;
  result->is_solution = true;

#if DEBUG_GT
  DumpCostMatrixSimple(&result->gt_interaction_matrix);
#endif
}

}  // namespace st::planning
