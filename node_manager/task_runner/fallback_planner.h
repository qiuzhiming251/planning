

#ifndef ONBOARD_PLANNER_PLAN_FALLBACK_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_FALLBACK_PLANNER_H_

#include <string>
#include <vector>
#include <optional>

#include "absl/time/time.h"
#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"

#include "plan_common/ego_history.h"
#include "plan_common/planner_status.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/route_sections.h"
#include "plan_common/maps/route_sections_info.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"

#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/partial_spacetime_object_trajectory.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "object_manager/st_inference/est_planner_output.h"

namespace st::planning {

struct FallbackPlannerInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const PlanStartPointInfo* start_point_info = nullptr;
  // The first point is assured to be plan start point.
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_trajectory = nullptr;
  // Only has meaningful value if est parallel main loop is enabled.
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  double prev_length_along_route = 0.0;
  double prev_max_reach_length = 0.0;
  const mapping::LanePoint* station_anchor = nullptr;
  bool prev_smooth_state = false;
  const mapping::LanePath* prev_lane_path_before_lc = nullptr;
  // const RouteSectionsInfo* route_sections_info_from_start;
  const PlannerObjectManager* obj_mgr = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  const LaneChangeStateProto* prev_lc_state = nullptr;
  // const TrafficLightStatesProto* traffic_light_states;
  const DeciderStateProto* pre_decider_state = nullptr;
  // const TrafficLightInfoMap* tl_info_map = nullptr;
  const ad_byd::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  // absl::Time parking_brake_release_time;
  // bool teleop_enable_traffic_light_stop = false;
  // bool enable_pull_over = false;
  // std::optional<double> brake_to_stop = std::nullopt;
  std::optional<double> cruising_speed_limit = std::nullopt;
  const Behavior* behavior = nullptr;
  const ad_byd::planning::SpeedState* speed_state = nullptr;
  int plan_id = 0;
  const EgoHistory* ego_history = nullptr;
  const Box2d ego_box{};
  const SpeedFinderStateProto* speed_finder_state = nullptr;
};

struct FallbackPlannerOutput {
  DeciderStateProto decider_state{};
  std::vector<ApolloTrajectoryPointProto> trajectory_points{};
  SpacetimeTrajectoryManager filtered_traj_mgr{};
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects{};
  std::optional<TrajectoryEndInfoProto> trajectory_end_info = std::nullopt;

  SchedulerOutput scheduler_output{};
  DiscretizedPath path{};
  std::vector<PathPoint> st_path_points{};
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision{};
  int tl_stop_interface = 0;
  ad_byd::planning::SpeedState speed_state{};
  EgoFrame curr_ego_frame{};
  TrafficLightIndicationInfoProto tl_ind_info{};
  SpeedFinderStateProto speed_finder_state{};
};

PlannerStatus RunFallbackPlanner(
    const FallbackPlannerInput& input, const VehicleParamsProto& vehicle_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const DecisionConstraintConfigProto& decision_constraint_config,
    const FallbackPlannerParamsProto& fallback_planner_params,
    FallbackPlannerOutput* output, EstPlannerDebug* debug,
    ThreadPool* thread_pool);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_PLAN_FALLBACK_PLANNER_H_
