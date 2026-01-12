

#ifndef ST_PLANNING_EST_PLANNER
#define ST_PLANNING_EST_PLANNER

#include <string>
#include <vector>

#include "absl/time/time.h"
#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"

#include "plan_common/type_def.h"
#include "plan_common/ego_history.h"
#include "plan_common/planner_status.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

#include "object_manager/object_history.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "object_manager/st_inference/est_planner_output.h"

#include "alternative_gaming/drf_driveline/drf_driveline_generator.h"

#include "planner/planner_manager/pnp_util.h"
#include "decider/scheduler/fsd_lane_selector.h"

namespace st {
namespace planning {
using PushDirection = ad_byd::planning::PushDirection;
struct EstPlannerInput {
  const ad_byd::planning::Map* semantic_map_manager = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  int plan_id = 1 /*for plot and debugging*/;
  const VehicleParamsProto* vehicle_params = nullptr;

  // Prev states:
  const DeciderStateProto* decider_state = nullptr;
  const InitializerStateProto* initializer_state = nullptr;
  const TrajectoryOptimizerStateProto* trajectory_optimizer_state_proto =
      nullptr;
  const SpacetimePlannerObjectTrajectoriesProto*
      st_planner_object_trajectories = nullptr;

  const PlannerObjectManager* obj_mgr = nullptr;
  const PlanStartPointInfo* start_point_info = nullptr;
  const StPathPlanStartPointInfo* st_path_start_point_info = nullptr;
  const ad_byd::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const LargeVehicleAvoidStateProto* pre_large_vehicle_avoid_state = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  const std::optional<PNPInfos> pnp_infos = std::nullopt;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  bool enable_tl_ok_btn = false;
  bool override_passable = false;
  const RouteTargetInfo* route_target_info = nullptr;
  bool consider_lane_change_gap = true;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  // Params.
  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;
  // Lane change style params.
  const SpeedFinderParamsProto* speed_finder_lc_radical_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_lc_conservative_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_lc_normal_params = nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_radical_params =
      nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_normal_params =
      nullptr;
  const TrajectoryOptimizerParamsProto*
      trajectory_optimizer_lc_conservative_params = nullptr;
  const SpacetimePlannerObjectTrajectoriesParamsProto*
      spacetime_planner_object_trajectories_params = nullptr;

  const Behavior* behavior = nullptr;
  bool miss_navi_scenario = false;
  const ObjectHistoryManager* obs_history = nullptr;
  const ad_byd::planning::SpeedState* speed_state = nullptr;
  int cur_navi_lc_num = 0;
  std::string leading_id = "";
  double left_navi_dist = std::numeric_limits<double>::max();
  double left_navi_dist_v2 = std::numeric_limits<double>::max();
  ad_byd::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2 =
      ad_byd::planning::V2TurnInfo::V2DetailTurnType::NONE;
  const st::LaneChangeStage& prev_lane_change_stage = LCS_NONE;
  const st::DriverAction::LaneChangeCommand lc_cmd_state =
      DriverAction::LC_CMD_NONE;
  PushDirection push_dir = PushDirection::Push_None;
  // nudge_info
  const NudgeObjectInfo* nudge_object_info = nullptr;
  double cur_dist_to_junction = std::numeric_limits<double>::max();
  std::vector<std::string> lc_lead_obj_ids{};
  bool is_open_gap = false;
  EgoFrame* curr_ego_frame = nullptr;
  const EgoHistory* ego_history = nullptr;
  double spdlimit_curvature_gain_prev = 1.0;
  const PausePushSavedOffsetProto* saved_offset = nullptr;
  // Previous lane change style decider result
  LaneChangeStyleDeciderResultProto pre_lc_style_decider_result{};
  // Previous task safety evaluation result
  TaskSafetyEvaluationProto pre_task_safety_evaluation_result{};
  // Previous frames result for the scene of cones riding line
  int pre_scene_cones_riding_line_frames_result = 0;

  const SpeedFinderStateProto* speed_finder_state = nullptr;
  const GamingResultProto* last_gaming_result = nullptr;
  const std::unordered_map<std::string, double> pre_truncated_back_traj_horizon{};
  std::optional<double> cruising_speed_limit = std::nullopt;
  const ad_byd::planning::EIEChoiceType eie_choice_type =
      ad_byd::planning::EIEChoiceType::CHOICE_NONE;
  const bool eie_braking_down_flag = false;
  double cur_dist_to_prev_junction = std::numeric_limits<double>::max();
  double dist_to_tunnel_entrance = std::numeric_limits<double>::max();
  double dist_to_tunnel_exitance = std::numeric_limits<double>::max();
};

PlannerStatus RunEstPlanner(const EstPlannerInput& input,
                            SchedulerOutput scheduler_output,
                            EstPlannerOutput* est_output,
                            EstPlannerDebug* debug_info,
                            ThreadPool* thread_pool);

}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_EST_PLANNER
