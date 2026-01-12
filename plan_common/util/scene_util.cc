

#include "plan_common/util/scene_util.h"

namespace st {
namespace planning {

bool IsInHighWay(const PlannerSemanticMapManager& psmm,
                 const RouteSections& sections, const double preview_distance) {
  bool in_high_way = false;
  // if (sections.empty()) {
  //   return in_high_way;
  // }
  // double accum_dist = 0.0;
  // for (int i = 0, n = sections.size(); i < n; ++i) {
  //   const auto& section_segment = sections.route_section_segment(i);
  //   const auto& section_ptr = psmm.FindSectionByIdOrNull(section_segment.id);
  //   if (section_ptr == nullptr || section_ptr->proto == nullptr) break;
  //   const auto road_class = section_ptr->proto->road_class();
  //   in_high_way = in_high_way ||
  //                 road_class == mapping::SectionProto_RoadClass_CITY_EXPRESS
  //                 || road_class == mapping::SectionProto_RoadClass_HIGHWAY;
  //   accum_dist +=
  //       (section_segment.end_fraction - section_segment.start_fraction) *
  //       section_ptr->average_length;
  //   if (in_high_way) break;
  //   if (accum_dist >= preview_distance) break;
  // }

  return in_high_way;
}

}  // namespace planning
}  // namespace st
