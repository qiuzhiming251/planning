

#ifndef ONBOARD_MAPS_SMM_PROTO_UTIL_H_
#define ONBOARD_MAPS_SMM_PROTO_UTIL_H_

#include <memory>

#include "plan_common/maps/map.h"
#include "plan_common/maps/semantic_map_defs.h"

namespace st::mapping {

template <typename SmmType>
ad_byd::planning::LaneConstPtr FindLanePtr(const SmmType& smm,
                                           mapping::ElementId id) {
  if constexpr (std::is_same_v<SmmType, ad_byd::planning::Map>) {
    return smm.GetLaneById(id);
  } else {
    return smm.FindLaneByIdOrNull(id);
  }
}

template <typename SmmType>
ad_byd::planning::SectionConstPtr FindSectionPtr(const SmmType& smm,
                                                 mapping::SectionId id) {
  if constexpr (std::is_same_v<SmmType, ad_byd::planning::Map>) {
    return smm.GetSectionById(id);
  } else {
    return smm.FindSectionByIdOrNull(id);
  }
}

}  // namespace st::mapping

#endif  // ONBOARD_MAPS_SMM_PROTO_UTIL_H_
