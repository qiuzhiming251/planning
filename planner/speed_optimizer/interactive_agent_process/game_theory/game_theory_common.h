

#ifndef ONBOARD_PLANNER_SPEED_GAME_THEORY_COMMON_H_
#define ONBOARD_PLANNER_SPEED_GAME_THEORY_COMMON_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "plan_common/async/thread_pool.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "planner/speed_optimizer/interactive_agent_process/interactive_agent.h"
#include "planner/speed_optimizer/st_graph.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

#define MAX_STATE_NUM (50)
#define MAX_GT_ACC_NUM (50)

typedef struct GT_Ego_State {
  int traj_point_idx;
  SecondOrderTrajectoryPoint sl_center_pose; /* drive_passage frenet */
  FrenetBox frenet_box;
  frenet_vel_t sl_vel; /* drive_passage frenet */
  SecondOrderTrajectoryPoint global_center_pose;
  Polygon2d global_polygon;

  double vel;
  double acc;
  double theta;
  double time_stamp;
  double dist = 0.0;
  double wait_time = 0.0;

  double safety_cost;
  double law_cost;
  double acc_cost;
  bool is_legal = true;
} gt_ego_state_t;

typedef struct GT_Ego_Info {
  double max_acc;
  double min_acc;
  double acc_interval;
  double max_vel;
  double safety_cost;
  double law_cost;
  double acc_cost;
  double total_cost;
  double average_cost;

  int size;
  std::vector<gt_ego_state_t*> states;

  void ResetGtEgoInfo();
  bool is_legal = true;

} gt_ego_info_t;

typedef struct GT_Agent_State {
  int traj_point_idx;
  SecondOrderTrajectoryPoint sl_center_pose; /* drive_passage frenet */
  FrenetBox frenet_box;
  frenet_vel_t sl_vel; /* drive_passage frenet */
  SecondOrderTrajectoryPoint global_center_pose;
  Polygon2d global_polygon;

  double vel; /* abs velocity */
  double acc;
  double theta; /* represent dir */
  double time_stamp;
  double dist = 0.0;
  double wait_time = 0.0;

  double safety_cost;
  double law_cost;
  double acc_cost;
  bool is_legal = true;
} gt_agent_state_t;

typedef struct GT_Agent_Info {
  const InteractiveAgent* obstacle_;
  double max_acc;
  double min_acc;
  double acc_interval;
  double max_vel;

  double safety_cost;
  double law_cost;
  double acc_cost;
  double total_cost;
  double average_cost;

  Polygon2d base_polygon;

  int size;
  std::vector<gt_agent_state_t*> states;

  void ResetGtAgentInfo();
  bool is_legal = true;

} gt_agent_info_t;

typedef struct GtEgoSimState {
  std::vector<gt_ego_state_t> states;
} gt_ego_sim_state_t;

typedef struct GtObjSimState {
  std::vector<gt_agent_state_t> states;
} gt_obj_sim_state_t;

struct GT_Interaction_Data {
  double ego_acc;
  double agent_acc;

  bool is_success;
  bool is_collision;

  gt_agent_info_t obj_info;
  gt_ego_info_t ego_info;
  StBoundaryProto::DecisionType decision_type;
};

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
