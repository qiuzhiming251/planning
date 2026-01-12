

#ifndef ONBOARD_PLANNER_UTIL_SCENE_UTIL_H_
#define ONBOARD_PLANNER_UTIL_SCENE_UTIL_H_

#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections.h"

namespace st {
namespace planning {

bool IsInHighWay(const PlannerSemanticMapManager& psmm,
                 const RouteSections& sections, const double preview_distance);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_UTIL_SCENE_UTIL_H_
