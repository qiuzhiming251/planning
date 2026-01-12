

#ifndef ONBOARD_PLANNER_SCHEDULER_LOCAL_MAP_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_LOCAL_MAP_BUILDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/status/status.h"

#include "lane_path.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections.h"
#include "route_sections_info.h"
#include "plan_common/maps/map_or_die_macros.h"

#include "plan_common/util/map_util.h"

namespace st::planning {

mapping::LanePath FindLanePathFromLaneAlongRouteSections(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, mapping::ElementId start_lane_id,
    double start_frac, double extend_len);
// This function will return a lane paths vector sorted from left to right.
// Output lane paths num less than the lane num of first route section. The
// input param "route_sections" must start from AV pos.
absl::StatusOr<std::vector<mapping::LanePath>> BuildLocalMap(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const RouteNaviInfo& route_navi_info);
}  // namespace st::planning

#endif
