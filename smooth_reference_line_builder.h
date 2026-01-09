

#ifndef ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_BUILDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/route_sections.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"

namespace st::planning {

absl::StatusOr<SmoothedReferenceCenterResult> SmoothLanePathByLaneIds(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width);

absl::StatusOr<SmoothedReferenceLineResultMap>
BuildSmoothedResultMapFromRouteSections(const PlannerSemanticMapManager& psmm,
                                        Vec2d ego_pos, double half_av_width,
                                        SmoothedReferenceLineResultMap results);

absl::StatusOr<std::vector<std::vector<mapping::ElementId>>>
FindLanesToSmoothFromRoute(const PlannerSemanticMapManager& psmm);

// TODO: Move it to anonymous namespace.
absl::StatusOr<SmoothedReferenceCenterResult>
SmoothLanePathBoundedByPathBoundary(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_BUILDER_H_
