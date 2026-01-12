

#ifndef ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
#define ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "plan_common/async/thread_pool.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "planner/speed_optimizer/st_graph.h"
#include "planner/speed_optimizer/object_scene_recognition.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

absl::Status MakeInteractiveSpeedDecision(
    std::string_view base_name, const VehicleGeometryParamsProto& vehicle_geom,
    const MotionConstraintParamsProto& motion_constraint_params,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& path, double current_v, double current_a,
    const SpeedFinderParamsProto& speed_finder_params, double speed_cap,
    int traj_steps,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const NudgeObjectInfo* nudge_object_info,
    SpeedLimitProvider* speed_limit_provider,
    const std::vector<DrivingProcess>& driving_process_seq,
    const bool is_on_highway, double dist_to_merge,
    SpeedVector* preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    InteractiveSpeedDebugProto* interactive_speed_debug,
    ThreadPool* thread_pool, LaneChangeStage lc_stage,
    const absl::flat_hash_set<std::string>* follower_set,
    SpeedResponseStyle active_speed_response_style);

std::optional<VtSpeedLimit> PostProcessYeildGapObsStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    double current_v);

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
