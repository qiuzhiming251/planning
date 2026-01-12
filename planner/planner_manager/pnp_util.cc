/*
 * @date:
 * @file:pnp info cc
 */

#include "plan_common/log_data.h"
#include "planner/planner_manager/pnp_util.h"

namespace st {
namespace planning {

DriverAction::LaneChangeCommand PNPIntention(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_section_in_horizon,
    const std::shared_ptr<PNPInfo> pnp_info,
    const mapping::LanePath& prev_target_lane_path) {
  DriverAction::LaneChangeCommand pnp_lc_state = DriverAction::LC_CMD_NONE;
  if (pnp_info == nullptr) return pnp_lc_state;

  // front section info
  auto front_section_info =
      psmm.FindSectionByIdOrNull(route_section_in_horizon.front().id);
  std::vector<uint64_t> lane_ids;
  lane_ids.assign(front_section_info->lanes().begin(),
                  front_section_info->lanes().end());

  Log2DDS::LogDataV0("lc_debug",
                     absl::StrCat("route_section_in_horizon.front().id: ",
                                  route_section_in_horizon.front().id));

  // pnp id index
  std::vector<mapping::ElementId> pnp_lane_ids;
  for (auto id : pnp_info->target_lane_sequence_ids()) {
    // auto eid = static_cast<mapping::ElementId>(id);
    Log2DDS::LogDataV0("lc_debug", absl::StrCat("target id:", id));
    pnp_lane_ids.push_back(static_cast<mapping::ElementId>(id));
  }

  mapping::ElementId pnp_id;
  if (!pnp_lane_ids.empty()) {
    pnp_id = pnp_lane_ids.front();
  }
  const auto pnp_it = std::find(lane_ids.begin(), lane_ids.end(), pnp_id);
  if (pnp_it == lane_ids.end()) {
    Log2DDS::LogDataV0("lc_debug", "pnp not find");
    return DriverAction::LC_CMD_NONE;
  }
  const int pnp_index = pnp_it - lane_ids.begin();
  Log2DDS::LogDataV0("lc_debug", absl::StrCat("pnp_index: ", pnp_index));

  // 上一帧id index
  mapping::ElementId prev_id;
  if (!prev_target_lane_path.IsEmpty() &&
      !prev_target_lane_path.lane_path_data().lane_ids().empty()) {
    prev_id = prev_target_lane_path.lane_path_data().lane_ids().front();
  }
  const auto prev_it = std::find(lane_ids.begin(), lane_ids.end(), prev_id);
  if (prev_it == lane_ids.end()) {
    Log2DDS::LogDataV0("lc_debug", "prev not find");
    return DriverAction::LC_CMD_NONE;
  }
  int prev_index = prev_it - lane_ids.begin();
  Log2DDS::LogDataV0("lc_debug", absl::StrCat("prev_index: ", prev_index));

  // 对比 index 计算相对位置
  if (prev_index < pnp_index) {
    Log2DDS::LogDataV0("lc_debug", "pnp right");
    return DriverAction::LC_CMD_RIGHT;
  } else if (prev_index > pnp_index) {
    Log2DDS::LogDataV0("lc_debug", "pnp left");
    return DriverAction::LC_CMD_LEFT;
  } else {
    Log2DDS::LogDataV0("lc_debug", "pnp none");
    return DriverAction::LC_CMD_NONE;
  }
}

std::vector<mapping::ElementId> PnpIds(
    const std::shared_ptr<PNPInfo> pnp_info) {
  // pnp_lane_ids
  std::vector<mapping::ElementId> pnp_lane_ids;
  if (pnp_info == nullptr) {
    Log2DDS::LogDataV0("lc_debug", "pnp_info nullptr");
    return pnp_lane_ids;
  } else {
    Log2DDS::LogDataV0("lc_debug", "pnp_info not nullptr");
  }
  pnp_lane_ids.clear();
  for (auto id : pnp_info->target_lane_sequence_ids()) {
    Log2DDS::LogDataV0("lc_debug", absl::StrCat("target id: ", id));
    pnp_lane_ids.push_back(static_cast<mapping::ElementId>(id));
  }
  return pnp_lane_ids;
}

}  // namespace planning
}  // namespace st
