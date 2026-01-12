

#ifndef ONBOARD_PLANNER_INITIALIZER_MULTI_TRAJ_SELECTOR_H_
#define ONBOARD_PLANNER_INITIALIZER_MULTI_TRAJ_SELECTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_join.h"
#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/drive_passage.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/initializer_output.h"

#include "decider/initializer/motion_graph.h"
#include "decider/initializer/cost_provider.h"
#include "decider/initializer/ref_speed_table.h"
#include "decider/initializer/lane_change_safety.h"
#include "decider/initializer/select_nudge_object.h"
#include "decider/initializer/motion_search_output.h"

namespace st::planning {

struct SingleTrajDebugInfo {
  std::vector<MotionEdgeIndex> terminated_edge_idxes;
  std::vector<std::vector<ApolloTrajectoryPointProto>> top_k_trajs;
  std::vector<double> top_k_total_costs;
  std::vector<MotionEdgeIndex> top_k_edges;
};

struct SingleTrajInfo {
  // Different trajectory should be determined by different grouping of leading
  // trajectories based on lane change state. Need logic to choose leading
  // trajectories while changing lanes.
  std::vector<std::string> leading_trajs;  // Leading objects' traj_ids.
  std::vector<ApolloTrajectoryPointProto> traj_points;
  NudgeInfos nudge_info;
  MotionEdgeIndex last_edge_index;
  MotionEdgeVector<MotionSearchOutput::SearchCost> search_costs;  // DP
  std::vector<double> feature_costs;                              // A*
  double total_cost;
  std::unique_ptr<MotionGraph> motion_graph;
  std::unique_ptr<RefSpeedTable> ref_speed_table;
  std::unique_ptr<CostProvider> cost_provider;
  IgnoreTrajMap ignored_trajs;
  SingleTrajDebugInfo debug_info;
  std::string GetLeadingObjTrajId() const {
    if (leading_trajs.empty()) {
      return "No leading trajectories.";
    } else {
      return absl::StrJoin(leading_trajs, ", ");
    }
  }
};

SpeedResponseStyle MappingLongResponseLevel(LaneChangeStage prev_lc_stage,
                                            double follower_max_decel,
                                            double leader_max_decel,
                                            const std::string& prefix);
absl::StatusOr<int> EvaluateMultiTrajs(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const PathSlBoundary& sl_boundary,
    const ApolloTrajectoryPointProto& start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<SingleTrajInfo>& multi_trajs,
    const VehicleParamsProto& vehicle_params, double speed_limit,
    bool eval_safety, const std::pair<double, double>& congestion_factor,
    LaneChangeStyle lc_style,
    const LaneChangeStyleDeciderResultProto& pre_lc_style_decider_result,
    const TaskSafetyEvaluationProto& pre_task_safety_evaluation_result,
    const int pre_scene_cones_riding_line_frames_result,
    const st::LaneChangeStage& lc_state, const bool lc_left,
    const st::LaneChangeStage& prev_lc_stage,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ObjectHistoryManager& obs_history_mgr,
    absl::Duration path_look_ahead_duration,
    absl::flat_hash_set<std::string>* follower_set,
    absl::flat_hash_set<std::string>* leader_set, double* follower_max_decel,
    double* leader_max_decel,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    PlannerStatusProto::PlannerStatusCode* status_code, ThreadPool* thread_pool,
    int plan_id, bool is_borrow, int* c_idx,
    LaneChangeStyleDeciderResultProto* lc_style_decider_result,
    TaskSafetyEvaluationProto* task_safety_evaluation_result,
    int* scene_cones_riding_line_frames_result,
    const std::vector<std::string>& leading_trajs,
    absl::flat_hash_set<std::string>* gaming_lc_obs_set,
    const std::vector<double>* stop_s_vec = nullptr);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_MULTI_TRAJ_SELECTOR_H_
