

#ifndef ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_
#define ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_

#include <vector>

#include "absl/types/span.h"
#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/drive_passage.h"
#include "planner/speed_optimizer/path_semantic_analyzer.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "planner/speed_optimizer/st_graph.h"

namespace st {
namespace planning {

struct SpeedConstraintGeneratorOutput {
  std::vector<ConstraintProto::PathSpeedRegionProto> path_speed_regions;
  std::vector<ConstraintProto::PathStopLineProto> path_stop_lines;
};

SpeedConstraintGeneratorOutput GenerateStationaryCloseObjectConstraints(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& traj_mgr,
    const DiscretizedPath& path, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary, double av_speed,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const bool is_on_highway);

SpeedConstraintGeneratorOutput GenerateDenseTrafficFlowConstraint(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const DiscretizedPath& path, double plan_start_v,
    const VehicleGeometryParamsProto& vehicle_geometry_params);

}  // namespace planning

}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_
