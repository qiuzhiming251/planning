

#ifndef ONBOARD_ST_PATH_PANNER_INPUT_H_
#define ONBOARD_ST_PATH_PANNER_INPUT_H_

#include <map>
#include <vector>
#include <limits>
#include <string>
#include <optional>

#include "absl/time/time.h"
#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/pnp_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer_config.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"

#include "plan_common/type_def.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"

#include "object_manager/object_history.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/spacetime_planner_object_trajectories.h"

#include "decider_input.h"
#include "decider_output.h"
#include "scheduler_output.h"

namespace st::planning {
struct StPathPlannerInput {
  int plan_id = 0;
  const StPathPlanStartPointInfo* st_path_start_point_info = nullptr;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  const VehicleParamsProto* vehicle_params = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  // Not a const since it may be modified and will be moved.
  SchedulerOutput scheduler_output;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const std::optional<PNPInfos> pnp_infos = std::nullopt;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;

  // For rebuilding constraint manager on lc pause.
  const PlanStartPointInfo* start_point_info = nullptr;
  const RouteTargetInfo* route_target_info = nullptr;
  const PlannerObjectManager* obj_mgr = nullptr;
  //   const TrafficLightInfoMap* tl_info_map = nullptr;
  const ad_byd::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const DeciderStateProto* prev_decider_state = nullptr;
  //   absl::Time parking_brake_release_time;
  // bool enable_pull_over = false;
  // bool enable_traffic_light_stopping = true;
  // std::optional<double> brake_to_stop = std::nullopt;

  SpacetimePlannerObjectTrajectories init_st_planner_object_traj;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  DeciderOutput decider_output;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const LargeVehicleAvoidStateProto* pre_large_vehicle_avoid_state = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  const InitializerStateProto* prev_initializer_state = nullptr;
  const TrajectoryOptimizerStateProto* trajectory_optimizer_state_proto =
      nullptr;
  // const TrajectoryProto* log_av_trajectory = nullptr;

  //   const ml::captain_net::CaptainNetOutput* captain_net_output = nullptr;
  // Params.
  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;
  // Lane change style params.
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_radical_params =
      nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_normal_params =
      nullptr;
  const TrajectoryOptimizerParamsProto*
      trajectory_optimizer_lc_conservative_params = nullptr;
  const Behavior* behavior = nullptr;
  const ad_byd::planning::SpeedState* speed_state = nullptr;
  bool miss_navi_scenario = false;
  const ObjectHistoryManager* obs_history = nullptr;
  int cur_navi_lc_num = 0;
  std::string leading_id = "";
  double left_navi_dist = 999.0;
  double left_navi_dist_v2 = std::numeric_limits<double>::max();
  const st::LaneChangeStage& prev_lane_change_stage = LCS_NONE;
  const st::DriverAction::LaneChangeCommand lc_cmd_state =
      DriverAction::LC_CMD_NONE;
  PushDirection push_dir = PushDirection::Push_None;
  const NudgeObjectInfo* nudge_object_info = nullptr;
  double cur_dist_to_junction = std::numeric_limits<double>::max();
  const PausePushSavedOffsetProto* saved_offset = nullptr;
  // Previous lane change style decider result
  LaneChangeStyleDeciderResultProto pre_lc_style_decider_result;
  // Previous task safety evaluation result
  TaskSafetyEvaluationProto pre_task_safety_evaluation_result;
  // Previous frames result for the scene of cones riding line
  int pre_scene_cones_riding_line_frames_result = 0;
  const ad_byd::planning::EIEChoiceType eie_choice_type =
      ad_byd::planning::EIEChoiceType::CHOICE_NONE;
};
}  // namespace st::planning

#endif  // ONBOARD_ST_PATH_PANNER_INPUT_H_