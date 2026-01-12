
#include "plan_common/gflags.h"
#include "plan_common/maps/road_boundaries.h"
namespace ad_byd {
namespace planning {
RoadBoundaries::RoadBoundaries(
    const std::vector<RoadBoundaryConstPtr>& boundaries) {
  for (auto boundary_ptr : boundaries) {
    // skip illegal boundary
    if (!boundary_ptr->IsValid()) {
      continue;
    }
    boundaries_.emplace_back(boundary_ptr);
  }
}
}  // namespace planning
}  // namespace ad_byd