

#ifndef ST_PLANNING_DECISION_DECIDER_INPUT
#define ST_PLANNING_DECISION_DECIDER_INPUT

#include <limits>
#include <string>

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"

#include "plan_common/type_def.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/drive_passage.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

#include "object_manager/object_history.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"

#include "scheduler_output.h"

namespace st::planning {
using PushDirection = ad_byd::planning::PushDirection;
struct DeciderInput {
  int plan_id = 0;
  const st::VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const DecisionConstraintConfigProto* config = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const LaneChangeStateProto* lc_state = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  double target_offset_from_start = 0.0;
  const mapping::LanePath* lane_path_before_lc = nullptr;
  const DrivePassage* passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  bool borrow_lane_boundary = false;
  const PlannerObjectManager* obj_mgr = nullptr;
  const ObjectHistoryManager* obs_history = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  // const TrafficLightInfoMap* tl_info_map = nullptr;
  const ad_byd::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const DeciderStateProto* pre_decider_state = nullptr;
  // absl::Time parking_brake_release_time;
  // bool teleop_enable_traffic_light_stop;
  bool enable_tl_ok_btn = false;
  bool override_passable = false;
  // bool enable_pull_over;
  // std::optional<double> brake_to_stop = std::nullopt;
  double max_reach_length = std::numeric_limits<double>::max();
  //   double left_navi_dist_map = std::numeric_limits<double>::max();
  int lc_num = 0;
  std::string leading_id = "";
  absl::Time plan_time{};
  // Null if no routed lane change is detected.
  const RouteTargetInfo* route_target_info = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  const Behavior* behavior = nullptr;
  const ad_byd::planning::SpeedState* speed_state = nullptr;
  double cur_dist_to_junction = std::numeric_limits<double>::max();
  std::vector<std::string> lc_lead_obj_ids{};
  PushDirection push_dir = PushDirection::Push_None;
  ad_byd::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2 =
      ad_byd::planning::V2TurnInfo::V2DetailTurnType::NONE;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  std::optional<double> cruising_speed_limit = std::nullopt;
  double cur_dist_to_prev_junction = std::numeric_limits<double>::max();
  bool eie_braking_down_flag = false;
  double dist_to_tunnel_entrance = std::numeric_limits<double>::max();
  double dist_to_tunnel_exitance = std::numeric_limits<double>::max();
  const ad_byd::planning::EIEChoiceType eie_choice_type =
      ad_byd::planning::EIEChoiceType::CHOICE_NONE;
};

}  // namespace st::planning

#endif  // ST_PLANNING_DECISION_DECIDER_INPUT
