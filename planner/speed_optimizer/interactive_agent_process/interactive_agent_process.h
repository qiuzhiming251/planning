

#ifndef ONBOARD_PLANNER_SPEED_INTERACTIVE_AGENT_PROCESS_H_
#define ONBOARD_PLANNER_SPEED_INTERACTIVE_AGENT_PROCESS_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "planner/speed_optimizer/st_graph.h"

namespace st {
namespace planning {

absl::Status InteractiveAgentProcess(
    const VehicleGeometryParamsProto& vehicle_geom, const StGraph& st_graph,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, double current_a,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const DrivePassage* drive_passage,
    const LaneChangeStateProto lane_change_state);

}

}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
