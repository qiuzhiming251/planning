

#ifndef ST_PLANNING_INITIALIZER_INITIALIZER_INPUT
#define ST_PLANNING_INITIALIZER_INITIALIZER_INPUT

#include <memory>
#include <string>
#include <vector>

#include "plan_common/path_sl_boundary.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/util/decision_info.h"
#include "decider/initializer/collision_checker.h"
#include "decider/initializer/geometry/geometry_form_builder.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "plan_common/maps/map.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/object_history.h"

namespace st::planning {
using PushDirection = ad_byd::planning::PushDirection;

enum class InitializerSceneType {
  INIT_SCENE_NONE = 0,
  INIT_SCENE_FOLLOW = 1,
  INIT_SCENE_NUDGE = 2,
  INIT_SCENE_BORROW = 3,
  INIT_SCENE_LANE_CHANGE = 4
};

struct MotionSearchInput {
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const ApolloTrajectoryPointProto* start_point = nullptr;
  const st::FrenetCoordinate ego_sl;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  absl::Time plan_time;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const VehicleParamsProto* vehicle_params = nullptr;
  const GeometryGraph* geom_graph = nullptr;
  const GeometryFormBuilder* form_builder = nullptr;
  const CollisionChecker* collision_checker = nullptr;
  const std::vector<double>* stop_s_vec = nullptr;
  const std::vector<LeadingGroup>* leading_groups = nullptr;
  const ConstraintProto::LeadingObjectProto* blocking_static_traj = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const ObjectHistoryManager* obs_history = nullptr;
  const double standard_congestion_factor = 0.0;
  const double traffic_congestion_factor = 0.0;
  // const ml::captain_net::CaptainNetOutput* captain_net_output = nullptr;
  double passage_speed_limit = 10.0;
  const InitializerSceneType init_scene_type;
  bool is_lane_change = false;
  bool eval_safety = false;
  LaneChangeStyle lc_style = LC_STYLE_NORMAL;
  // const TrajectoryProto* log_av_trajectory = nullptr;
  LaneChangeStage lc_state = LCS_PAUSE;
  bool lc_left = true;
  LaneChangeStage prev_lc_stage = LCS_NONE;
  PushDirection push_dir = PushDirection::Push_None;
  bool borrow_lane = false;
  // Previous lane change style decider result
  LaneChangeStyleDeciderResultProto pre_lc_style_decider_result;
  // Previous task safety evaluation result
  TaskSafetyEvaluationProto pre_task_safety_evaluation_result;
  // Previous frames result for the scene of cones riding line
  int pre_scene_cones_riding_line_frames_result = 0;
};

struct InitializerInput {
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const StPathPlanStartPointInfo* path_start_point_info = nullptr;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  const LaneChangeStateProto* lane_change_state = nullptr;
  const LaneChangeStage prev_lc_stage = LaneChangeStage::LCS_NONE;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const InitializerStateProto* prev_initializer_state = nullptr;
  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const VehicleParamsProto* vehicle_params = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
  int plan_id = 0;
  // const TrajectoryProto* log_av_trajectory = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  bool borrow_lane = false;
  const FrenetBox* av_frenet_box = nullptr;

  // For rebuilding constraint manager on lc pause.
  const PlanStartPointInfo* start_point_info = nullptr;
  const RouteTargetInfo* route_target_info = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  const PlannerObjectManager* obj_mgr = nullptr;
  // const TrafficLightInfoMap* tl_info_map = nullptr;
  const ad_byd::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const DeciderStateProto* prev_decider_state = nullptr;
  // absl::Time parking_brake_release_time;
  // bool enable_pull_over = false;
  // bool enable_traffic_light_stopping = true;
  // std::optional<double> brake_to_stop = std::nullopt;
  const ObjectHistoryManager* obs_history = nullptr;
  const Behavior* behavior = nullptr;
  const ad_byd::planning::SpeedState* speed_state = nullptr;
  int cur_lc_num = 0;
  std::string leading_id = "";
  double left_navi_dist = 999.0;
  PushDirection push_dir = PushDirection::Push_None;
  const NudgeObjectInfo* nudge_object_info = nullptr;
  const LargeVehicleAvoidStateProto* pre_large_vehicle_avoid_state = nullptr;
  // const ml::captain_net::CaptainNetOutput* captain_net_output = nullptr;
  const PausePushSavedOffsetProto* saved_offset = nullptr;
  // Previous lane change style decider result
  LaneChangeStyleDeciderResultProto pre_lc_style_decider_result;
  // Previous task safety evaluation result
  TaskSafetyEvaluationProto pre_task_safety_evaluation_result;
  // Previous frames result for the scene of cones riding line
  int pre_scene_cones_riding_line_frames_result = 0;
};

struct ReferenceLineSearcherInput {
  const GeometryGraph* geometry_graph = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const VehicleDriveParamsProto* vehicle_drive = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
};

}  // namespace st::planning

#endif  // ST_PLANNING_INITIALIZER_INITIALIZER_INPUT
