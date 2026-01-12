

#ifndef ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_H_

#include <limits>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "semantic_map_defs.h"
#include "modules/cnoa_pnc/planning/proto/hmi_content.pb.h"
#include "plan_common/maps/route_sections.h"

namespace st {
namespace planning {
struct RouteNaviInfo {
  // Distance is beggining from the lane, and ignore the start fraction.
  // Need to align when use this info.
  struct RouteLaneInfo {
    double max_driving_distance = 0.0;
    double max_reach_length = 0.0;
    double recommend_reach_length = 0.0;
    int min_lc_num_to_target = std::numeric_limits<int>::max();
    int lc_num_within_driving_dist = std::numeric_limits<int>::max();
    double len_before_merge_lane = 0.0;
    std::string DebugString() const {
      return absl::StrFormat(
          "{max_driving_distance: %f, max_reach_length: %f, "
          "recommend_reach_length: %f, min_lc_num_to_target: %d, "
          "lc_num_within_driving_dist: %d, len_before_merge_lane: %f}",
          max_driving_distance, max_reach_length, recommend_reach_length,
          min_lc_num_to_target, lc_num_within_driving_dist,
          len_before_merge_lane);
    }
  };

  struct NaviSectionInfo {
    double length_before_intersection = 0.0;
    NaviSectionInfoProto::Direction intersection_direction =
        NaviSectionInfoProto::STRAIGHT;
  };

  absl::flat_hash_map<mapping::ElementId, RouteLaneInfo> route_lane_info_map;
  absl::flat_hash_map<mapping::SectionId, NaviSectionInfo>
      navi_section_info_map;
  RouteSections back_extend_sections;

  std::string DebugString() const {
    return absl::StrJoin(
        route_lane_info_map, "\n", [](std::string* out, auto i) {
          out->append(absl::StrCat(i.first, i.second.DebugString()));
        });
  }

  void FromProto(const RouteNaviInfoProto& navi_info_proto);
  void ToProto(RouteNaviInfoProto* navi_info_proto) const;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_H_
