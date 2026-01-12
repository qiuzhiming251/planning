/*
 * @date:
 * @file:pnp info h
 */

#ifndef ST_PLANNING_PNP_UTIL
#define ST_PLANNING_PNP_UTIL

#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"
#include "modules/cnoa_pnc/planning/proto/pnp_info.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections.h"

namespace st {
namespace planning {

enum class PNP_LC_STATE {
  NONE = 0,
  KEEP = 1,
  LEFT = 2,
  RIGHT = 3,
};

DriverAction::LaneChangeCommand PNPIntention(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_section_in_horizon,
    const std::shared_ptr<PNPInfo> pnp_info,
    const mapping::LanePath& prev_target_lane_path);

std::vector<mapping::ElementId> PnpIds(const std::shared_ptr<PNPInfo> pnp_info);

}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_PNP_UTIL
