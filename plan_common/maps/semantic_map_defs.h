

#ifndef ONBOARD_MAPS_SEMANTIC_MAP_DEFS_H_
#define ONBOARD_MAPS_SEMANTIC_MAP_DEFS_H_

#include <cstdint>
#include <limits>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/maps/type_defs.h"

namespace st {
namespace mapping {

// DEFINE_STRONG_INT_TYPE(ElementId, int64_t);
DEFINE_STRONG_INT_TYPE(SegmentId, int64_t);
// DEFINE_STRONG_INT_TYPE(SectionId, int64_t);
using ElementId = uint64_t;
using SectionId = uint64_t;

const ElementId kInvalidElementId = 0;
const SectionId kInvalidSectionId = 0;

using SectionConnection = std::pair<SectionId, SectionId>;
using SectionConnectionDistance =
    absl::flat_hash_map<SectionConnection, double>;
namespace v2 {
struct Segment {
  ElementId element_id;
  // The index [0, n) of polyline segment
  SegmentId segment_id;
};
}  // namespace v2

}  // namespace mapping
}  // namespace st

#endif  // ONBOARD_MAPS_SEMANTIC_MAP_DEFS_H_
