
#ifndef ONBOARD_ST_PATH_PANNER_OUTPUT_H_
#define ONBOARD_ST_PATH_PANNER_OUTPUT_H_

#include <map>
#include <vector>
#include <string>

#include "absl/container/flat_hash_set.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/constraint_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/util/decision_info.h"

#include "object_manager/spacetime_planner_object_trajectories.h"

#include "scheduler_output.h"
#include "initializer_output.h"

namespace st::planning {
struct StPathPlannerOutput {
  InitializerOutput initializer_output;
  SchedulerOutput scheduler_output;
  DiscretizedPath path;
  std::vector<PathPoint> st_path_points;
  ConstraintManager constraint_manager;
  absl::flat_hash_set<std::string> follower_set;
  absl::flat_hash_set<std::string> leader_set;
  double follower_max_decel = 0.0;  // Should be larger than or equal to 0.0;
  absl::flat_hash_set<std::string> unsafe_object_ids;
  std::optional<NudgeObjectInfo> nudge_object_info;
  LeadingGroup leading_trajs;
  std::string leading_id;
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  InitializerDebugProto initializer_debug_proto;
  TrajectoryOptimizerDebugProto optimizer_debug_proto;
  DeciderStateProto decider_state;
  InitializerStateProto initializer_state;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  std::vector<ApolloTrajectoryPointProto> captain_traj_points;
  NudgeInfos nudge_info;
  TrajectoryOptimizerStateProto trajectory_optimizer_state_proto;
  std::map<std::string, bool> obs_leading;
  PlannerStatusProto::PlannerStatusCode lc_status_code;
  bool is_init_follow_scene = false;
  std::string lc_lead_obj_id = "none";
  // Optimizer Auto Tuning
  //   AutoTuningTrajectoryProto candidate_auto_tuning_traj_proto;
  //   AutoTuningTrajectoryProto expert_auto_tuning_traj_proto;
  SpeedResponseStyle speed_response_style = SPEED_RESPONSE_NORMAL;
  LargeVehicleAvoidStateProto pre_large_vehicle_avoid_state;
  SafetyCheckFailedReason safety_check_failed_reason =
      SafetyCheckFailedReason::LC_UNSAFE_REASON_NONE;
  PausePushSavedOffsetProto saved_offset;
  // Lane change style decider result
  LaneChangeStyleDeciderResultProto lc_style_decider_result;
  // Task evaluation result
  TaskSafetyEvaluationProto task_safety_evaluation_result;
  // Frames result for the scene of cones riding line
  int scene_cones_riding_line_frames_result = 0;
  // gaming lc obs id
  absl::flat_hash_set<std::string> gaming_lc_obs_set;
};
}  // namespace st::planning

#endif  // ONBOARD_ST_PATH_PANNER_OUTPUT_H_