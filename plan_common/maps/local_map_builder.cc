

#include <algorithm>
#include <vector>

#include "plan_common/maps/local_map_builder.h"

namespace st::planning {

namespace {

// The max length of lane path is consistent with the effective range of
// perception temporarily.
constexpr double kDefaultLanePathLength = 150.0;

}  // namespace
mapping::LanePath FindLanePathFromLaneAlongRouteSections(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, mapping::ElementId start_lane_id,
    double start_frac, double extend_len) {
  if (extend_len <= 0.0) return {};

  const auto& sections = sections_info.section_segments();
  const auto& lane_navi_info_map = route_navi_info.route_lane_info_map;
  std::vector<mapping::ElementId> lane_ids;
  mapping::ElementId cur_lane_id = start_lane_id;
  double cur_frac = start_frac;
  if (cur_frac == 1.0) lane_ids.push_back(cur_lane_id);
  int sec_idx = 0;
  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, cur_lane_id);
    if (cur_frac == 1.0) {
      if (++sec_idx == sections.size()) break;
      const auto& next_sec = sections[sec_idx];
      mapping::ElementId outgoing_lane_id = mapping::kInvalidElementId;
      int min_lc_num = std::numeric_limits<int>::max();
      for (const auto next_lane_id : lane_info.next_lane_ids()) {
        if (next_sec.id_idx_map.contains(next_lane_id)) {
          const auto* lane_navi_info_ptr =
              FindOrNull(lane_navi_info_map, next_lane_id);
          const double lc_num = lane_navi_info_ptr == nullptr
                                    ? std::numeric_limits<int>::max()
                                    : lane_navi_info_ptr->min_lc_num_to_target;
          if (lc_num < min_lc_num) {
            outgoing_lane_id = next_lane_id;
            min_lc_num = lc_num;
          }
        }
      }
      if (outgoing_lane_id == mapping::kInvalidElementId) break;
      cur_lane_id = outgoing_lane_id;
      cur_frac = 0.0;
    } else {
      lane_ids.push_back(cur_lane_id);
      const double lane_len = lane_info.curve_length();
      if (sec_idx + 1 == sections.size()) {
        cur_frac = std::min(sections.back().end_fraction,
                            extend_len / lane_len + cur_frac);
        break;
      }
      const double len = lane_len * (sections[sec_idx].end_fraction - cur_frac);
      if (len >= extend_len) {
        cur_frac += extend_len / lane_len;
        break;
      }
      extend_len -= len;
      cur_frac = 1.0;
    }
  }
  return mapping::LanePath(psmm.map_ptr(), std::move(lane_ids), start_frac,
                           cur_frac);
}

absl::StatusOr<std::vector<mapping::LanePath>> BuildLocalMap(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const RouteNaviInfo& route_navi_info) {
  const RouteSectionsInfo route_sections_info(psmm, &route_sections);
  if (route_sections_info.empty() ||
      route_sections_info.front().lane_ids.empty()) {
    return absl::NotFoundError("Route section is empty.");
  }

  std::vector<mapping::LanePath> lane_paths;
  const auto& current_section_info = route_sections_info.front();
  lane_paths.reserve(current_section_info.lane_ids.size());

  // Build lane paths start from left most lane to right most lane in current
  // route section.
  for (const auto lane_id : current_section_info.lane_ids) {
    auto lane_path = FindLanePathFromLaneAlongRouteSections(
        psmm, route_sections_info, route_navi_info, lane_id,
        route_sections_info.start_fraction(), kDefaultLanePathLength);
    if (!lane_path.IsEmpty()) {
      lane_paths.emplace_back(lane_path);
    };
  }
  return lane_paths;
}

}  // namespace st::planning
