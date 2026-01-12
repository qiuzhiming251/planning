

#include "plan_common/ref_line/smooth_reference_line_result.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

namespace st::planning {

absl::StatusOr<std::vector<mapping::ElementId>>
SmoothedReferenceLineResultMap::FindOverlapSmoothedLaneIds(
    const std::vector<mapping::ElementId>& lane_ids) const {
  const auto it = smoothed_result_map_.find(lane_ids);
  if (it != smoothed_result_map_.end()) {
    return it->first;
  }

  // Return ok when lane_ids is a subset of one key in smoothed_result_map_.
  for (const auto& pair_it : smoothed_result_map_) {
    // NOTE: check first to avoid overflow.
    if (pair_it.first.size() < lane_ids.size()) {
      continue;
    }

    for (int i = 0; i <= pair_it.first.size() - lane_ids.size(); ++i) {
      // Find the first equal element.
      if (pair_it.first[i] == lane_ids[0]) {
        // Loop through the rest.
        bool find_succ = true;
        for (int j = 1; j < lane_ids.size(); ++j) {
          if (pair_it.first[i + j] != lane_ids[j]) {
            find_succ = false;
            break;
          }
        }

        if (find_succ) {
          return pair_it.first;
        } else {
          break;
        }
      }
    }
  }

  return absl::NotFoundError("");
}

}  // namespace st::planning
