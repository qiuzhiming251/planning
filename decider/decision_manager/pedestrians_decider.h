

#ifndef ONBOARD_PLANNER_DECISION_PEDESTRIANS_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_PEDESTRIANS_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildPedestriansConstraints(
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const PathSlBoundary& sl_boundary,
    const SpacetimeTrajectoryManager& st_traj_mgr);

}  // namespace planning
}  // namespace st

#endif
