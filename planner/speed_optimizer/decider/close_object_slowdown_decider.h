

#ifndef ONBOARD_PLANNER_SPEED_DECIDER_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_

#include <vector>

#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/drive_passage.h"
#include "planner/speed_optimizer/st_graph_defs.h"

namespace st {
namespace planning {

std::vector<ConstraintProto::PathSpeedRegionProto>
MakeCloseObjectSlowdownDecision(
    const std::vector<CloseSpaceTimeObject>& close_space_time_objects,
    const DrivePassage& drive_passage, const DiscretizedPath& path_points,
    double av_speed, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const bool is_on_highway);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_
