

#ifndef ONBOARD_PLANNER_SCHEDULER_DRIVING_MAP_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_DRIVING_MAP_BUILDER_H_

#include "absl/status/statusor.h"

//#include "online_semantic_map.pb.h"
#include "plan_common/driving_map_topo.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections.h"

namespace st::planning {

absl::StatusOr<DrivingMapTopo> BuildDrivingMapByRouteOnOfflineMap(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& sections_from_start);

// absl::StatusOr<DrivingMapTopo> BuildDrivingMapByOnlineMap(
//     const PlannerSemanticMapManager& psmm,
//     const mapping::OnlineSemanticMapProto& online_map, const Vec2d& ego_pos);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SCHEDULER_DRIVING_MAP_BUILDER_H_
