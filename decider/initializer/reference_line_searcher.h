

#ifndef ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_SEARCHER_H_
#define ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_SEARCHER_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/async/thread_pool.h"
#include "plan_common/path_sl_boundary.h"

#include "object_manager/st_inference/initializer_output.h"

#include "decider/initializer/initializer_input.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "decider/initializer/motion_search_output.h"

namespace st::planning {

absl::StatusOr<ReferenceLineSearcherOutput> SearchReferenceLine(
    const ReferenceLineSearcherInput& input, InitializerDebugProto* debug_proto,
    ThreadPool* thread_pool);

absl::Status DeactivateFarGeometries(
    const ReferenceLineSearcherOutput& searcher_result,
    const PathSlBoundary& path_sl, GeometryGraph* mutable_geom_graph);

absl::Status DeactivateFarGeometries(
    const std::vector<ApolloTrajectoryPointProto>& traj_points,
    const PathSlBoundary& path_sl, GeometryGraph* mutable_geometry_graph);

void ParseReferenceLineResultToProto(const ReferenceLineSearcherOutput& result,
                                     GeometryGraphProto* debug_proto);
}  // namespace st::planning
#endif
