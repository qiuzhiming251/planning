#pragma once

#include <limits>
#include <optional>

#include "plan_common/maps/map_def.h"
#include "plan_common/planner_status.h"
#include "modules/msg/orin_msgs/map_event.pb.h"

namespace st::planning {

// Need to move to proto if well defined;
struct RoadHorizonInfo {
  double dist_to_fork = std::numeric_limits<double>::max();
  double dist_to_merge = std::numeric_limits<double>::max();
  double dist_to_cross = std::numeric_limits<double>::max();
  double dist_to_tunel = std::numeric_limits<double>::max();
  double dist_to_navi_end_for_lc = std::numeric_limits<double>::max();
};

inline bool IsValidBranch(const PlannerStatus& status) {
  return status.ok() ||
         status.status_code() == PlannerStatusProto::LC_SAFETY_CHECK_FAILED;
}

inline bool IsTurnRightMainAction(
    byd::msg::orin::routing_map::NaviMainAction main_action) {
  return main_action == byd::msg::orin::routing_map::NMA_TURN_RIGHT ||
         main_action == byd::msg::orin::routing_map::NMA_SLIGHT_RIGHT ||
         main_action == byd::msg::orin::routing_map::NMA_TURN_HARD_RIGHT ||
         main_action == byd::msg::orin::routing_map::NMA_MERGE_RIGHT;
}

inline bool IsTurnRightAssistAction(
    byd::msg::orin::routing_map::NaviAssistantAction assist_action) {
  return assist_action == byd::msg::orin::routing_map::NAA_RIGHT ||
         assist_action == byd::msg::orin::routing_map::NAA_RIGHT_BACK ||
         assist_action == byd::msg::orin::routing_map::NAA_BRANCH_RIGHT ||
         assist_action == byd::msg::orin::routing_map::NAA_BACK_RIGHT;
}

inline bool IsTurnRightNaviAction(
    const byd::msg::orin::routing_map::NaviActionInfo& navi_action) {
  return IsTurnRightMainAction(navi_action.main_action()) ||
         IsTurnRightAssistAction(navi_action.assistant_action());
}

inline constexpr bool IsIgnoreLaneType(ad_byd::planning::LaneType lane_type) {
  return lane_type == ad_byd::planning::LaneType::LANE_EMERGENCY ||
         lane_type == ad_byd::planning::LaneType::LANE_NON_MOTOR ||
         lane_type == ad_byd::planning::LaneType::LANE_HARBOR_STOP;
}

}  // namespace st::planning
