

#ifndef ST_PLANNING_EST_PLANNER_OUTPUT
#define ST_PLANNING_EST_PLANNER_OUTPUT

#include <string>
#include <vector>
#include <optional>

#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"

#include "plan_common/type_def.h"
#include "plan_common/ego_history.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/maps/st_boundary_with_decision.h"

#include "object_manager/planner_object.h"
#include "object_manager/partial_spacetime_object_trajectory.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_planner_object_trajectories.h"

#include "plan_common/trajectory_point.h"
#include "scheduler_output.h"

namespace st {
namespace planning {

struct EstPlannerDebug {
  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories{};
  FilteredTrajectories filtered_prediction_trajectories{};
  ConstraintProto decision_constraints{};
  InitializerDebugProto initializer_debug_proto{};
  TrajectoryOptimizerDebugProto optimizer_debug_proto{};
  // std::vector<ApolloTrajectoryPointProto> capnet_ref_traj;
  SpeedFinderDebugProto speed_finder_debug{};
  TrajectoryValidationResultProto traj_validation_result{};
};

struct EstPlannerOutput {
  SchedulerOutput scheduler_output{};
  DiscretizedPath path{};
  LeadingGroup leading_trajs{};
  PlannerStatusProto::PlannerStatusCode lc_status_code = PlannerStatusProto::OK;
  SafetyCheckFailedReason safety_check_failed_reason =
      SafetyCheckFailedReason::LC_UNSAFE_REASON_NONE;
  bool is_init_return_scene = false;
  bool is_init_follow_scene = false;
  std::string lc_lead_obj_id = "none";
  absl::flat_hash_set<std::string> follower_set{};
  absl::flat_hash_set<std::string> leader_set{};
  absl::flat_hash_set<std::string> unsafe_object_ids{};
  std::optional<std::string> cipv_obj_id = std::nullopt;
  double follower_max_decel = 0.0;  // Should be larger than or equal to 0.0;
  // Final trajectory.
  std::vector<ApolloTrajectoryPointProto> traj_points{};
  std::vector<PathPoint> st_path_points{};

  // The nearest stop line's s on existence.
  std::optional<double> first_stop_s = std::nullopt;
  std::optional<mapping::ElementId> redlight_lane_id = std::nullopt;

  // Return planner state that needs to persist here.
  DeciderStateProto decider_state{};

  // Return planner state that needs to persist here.
  InitializerStateProto initializer_state{};

  // State of trajectory optimizer.
  // Return planner state that needs to persist here.
  TrajectoryOptimizerStateProto trajectory_optimizer_state_proto{};

  // To be filled into planner_state.
  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories{};
  // Trajectories considered by speed.
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects{};
  std::optional<TrajectoryEndInfoProto> trajectory_end_info = std::nullopt;

  // Optimizer Auto Tuning
  // AutoTuningTrajectoryProto candidate_auto_tuning_traj_proto;
  // AutoTuningTrajectoryProto expert_auto_tuning_traj_proto;

  // For hmi display.
  std::optional<std::string> alerted_front_vehicle = std::nullopt;
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  std::optional<NudgeObjectInfo> nudge_object_info = std::nullopt;

  // Traffic light interface
  int tl_stop_interface = 0;
  TrafficLightIndicationInfoProto tl_ind_info{};

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision{};
  std::map<std::string, ObjectSlInfo> obj_sl_map{};
  std::map<std::string, bool> obj_lead{};

  // Gap info
  TrafficGapResult traffic_gap{};

  ad_byd::planning::SpeedState speed_state{};
  EgoFrame curr_ego_frame{};
  LargeVehicleAvoidStateProto pre_large_vehicle_avoid_state{};
  // Speed plan
  double spdlimit_curvature_gain = 1.0;
  PausePushSavedOffsetProto saved_offset{};

  // Lane change style decider result
  LaneChangeStyleDeciderResultProto lc_style_decider_result{};
  // Task evaluation result
  TaskSafetyEvaluationProto task_safety_evaluation_result{};
  // Frames result for the scene of cones riding line
  int scene_cones_riding_line_frames_result = 0;

  SpeedFinderStateProto speed_finder_state{};
  GamingResultProto gaming_result{};

  // truncated traj horizon of the obj in the back
  std::unordered_map<std::string, double> truncated_back_traj_horizon{};
};

}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_EST_PLANNER_OUTPUT
