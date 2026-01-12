

#ifndef AD_BYD_PLANNING_PLAN_MULTI_TASKS_CRUISE_PLANNER_INPUT_H
#define AD_BYD_PLANNING_PLAN_MULTI_TASKS_CRUISE_PLANNER_INPUT_H

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "plan_common/av_history.h"
#include "plan_common/plan_start_point_info.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "modules/msg/orin_msgs/map_event.pb.h"
#include "router/route_manager_output.h"
#include "plan_common/selector_state.h"
#include "plan_common/maps/map_def.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/pnp_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/object_history.h"
#include "modules/msg/st_msgs/sm_behavior.pb.h"

namespace st {
namespace planning {

struct AccAutonomyStateInput {
  bool standstill_wait = false;
  bool curve_speed_limit_allowed = true;
  std::optional<double> acceleration_request = 0.0;  // m/s^2
};

struct MultiTasksCruisePlannerInput {
  // Meant to add one count to shared_ptr here for async planner.
  const PlannerParamsProto* planner_params = nullptr;
  const VehicleParamsProto* vehicle_params = nullptr;

  const std::optional<PlanStartPointInfo> start_point_info = std::nullopt;
  const std::optional<Vec2d> ego_pos = std::nullopt;
  absl::Duration min_path_look_ahead_duration;

  const std::optional<const SpacetimeTrajectoryManager> st_traj_mgr =
      std::nullopt;
  const std::optional<const PlannerObjectManager> object_manager = std::nullopt;
  const std::optional<absl::flat_hash_set<std::string>> stalled_objects =
      std::nullopt;
  bool consider_lane_change_gap = true;
  const std::optional<SceneOutputProto> scene_reasoning = std::nullopt;
  const std::optional<std::vector<ApolloTrajectoryPointProto>>
      time_aligned_prev_traj = std::nullopt;
  const std::optional<ad_byd::planning::TrafficLightStatusMap>
      traffic_light_status_map = std::nullopt;
  DriverAction::LaneChangeCommand new_lc_command = DriverAction::LC_CMD_NONE;
  bool auto_model = true;
  const SelectorState* selector_state = nullptr;
  const std::optional<const ObjectsProto> objects_proto = std::nullopt;
  // from ExternalCmdStatus
  bool enable_tl_ok_btn = false;
  bool override_passable = false;
  std::optional<double> cruising_speed_limit = std::nullopt;
  // behavior status
  const std::optional<Behavior> behavior = std::nullopt;
  std::string leading_id = "";
  const PoseProto pose_proto;
  double front_wheel_angle = 0.0;
  const AvHistory* av_context = nullptr;
  AccAutonomyStateInput acc_autonomy_state_input;
  const byd::msg::orin::routing_map::MapEvent* map_event = nullptr;
  bool is_vru_for_suppressing_auto_start = false;
  const byd::msg::planning::BehaviorChoice behavior_choice =
      byd::msg::planning::BehaviorChoice::BEHAVIOR_CHOICE_INVALID;
  double target_lane_speed_limit = 0.0;
};

}  // namespace planning
}  // namespace st

#endif  // AD_BYD_PLANNING_PLAN_MULTI_TASKS_CRUISE_PLANNER_INPUT_H
