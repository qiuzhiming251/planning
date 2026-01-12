

#ifndef ST_PLANNING_SPEED_SPEED_FINDER_INPUT
#define ST_PLANNING_SPEED_SPEED_FINDER_INPUT

#include <map>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
//#include "plan_common/driving_map_topo.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/math/discretized_path.h"
//#include "ml/model_pool.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
//#include "predictor/container/av_context.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/route_sections.h"
//#include
//"plan_common/util/safe_invariance/safe_invariance_problem_interface.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "plan_common/ego_history.h"
//#include "plan_common/util/hmi_content_util.h"
#include "object_manager/object_history.h"
#include "object_manager/planner_object.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"

namespace st::planning {
struct SpeedFinderInput {
  std::string base_name = "";
  const Behavior* behavior = nullptr;
  // const DrivingMapTopo* driving_map_topo = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const absl::flat_hash_set<std::string>* follower_set = nullptr;
  const absl::flat_hash_set<std::string>* leader_set = nullptr;
  bool consider_lane_change_gap = true;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  // Path that has been resampled with kPathSampleInterval.
  const DiscretizedPath* path = nullptr;
  // Raw path points without resampling.
  const std::vector<PathPoint>* st_path_points = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  double plan_start_v = 0.0;
  double plan_start_a = 0.0;
  double plan_start_j = 0.0;
  absl::Time plan_time{};
  // const prediction::AvContext* planner_av_context = nullptr;
  const ObjectsProto* objects_proto = nullptr;
  // const ModelPool* planner_model_pool = nullptr;
  bool run_act_net_speed_decision = false;
  int plan_id = 0;
  LaneChangeStage lc_stage{};
  const LaneChangeStateProto lane_change_state{};
  std::string attention_obj_id = "";
  const NudgeObjectInfo* nudge_object_info = nullptr;
  bool is_open_gap = false;
  const EgoHistory* ego_history = nullptr;
  double spdlimit_curvature_gain_prev = 1.0;
  SpeedResponseStyle active_speed_response_style = SpeedResponseStyle::SPEED_RESPONSE_CONSERVATIVE;
  const ObjectHistoryManager* obj_history = nullptr;
  // gaming lc obs ids
  absl::flat_hash_set<std::string> gaming_lc_obs_set{};
  const SpeedGamingResultProto* last_speed_gaming_result = nullptr;
};

}  // namespace st::planning

#endif  // ST_PLANNING_SPEED_SPEED_FINDER_INPUT
