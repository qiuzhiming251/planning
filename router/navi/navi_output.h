

#ifndef ONBOARD_PLANNER_ROUTER_NAVI_NAVI_OUTPUT_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_NAVI_OUTPUT_H_

// IWYU pragma: no_include <ostream>

#include <cstdint>
#include <type_traits>
#include <utility>  // IWYU pragma: keep // for std::move
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "plan_common/container/strong_int.h"
//#include "lite/logging.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/hmi_content.pb.h"
#include "plan_common/maps/composite_lane_path.h"
#include "route_navigation.pb.h"
#include "router/route_util.h"

namespace st::planning::route {

struct NaviActionInfo {
  NaviActionProto navi_action;
  double length = 0.0;
};

struct NaviOutput {
  NaviPreviewProto navi_preview;
  NaviInstructionProto current_navi_instruction;
};

NaviActionProto MakeNaviAction(NaviActionProto::NaviActionType navi_action_type,
                               const Vec2d& point, double distance,
                               mapping::SectionId section_id);

}  // namespace st::planning::route

#endif  // ONBOARD_PLANNER_ROUTER_NAVI_NAVI_OUTPUT_H_
