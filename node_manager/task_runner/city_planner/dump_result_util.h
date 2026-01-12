#pragma once
#include "plan_common/path_sl_boundary.h"
#include "modules/msg/st_msgs/planning_result.pb.h"
namespace st::planning {

void DumpSlBoundaryToReferenceLine(const PathSlBoundary& sl_boundary,
                                   byd::msg::planning::PathInfo* reference_line,
                                   int sample_interval);

void DumpSlBoundaryToDrivingBoundary(
    const PathSlBoundary& sl_boundary,
    byd::msg::planning::DrivingBoundary* driving_boundary, int sample_interval);

}  // namespace st::planning
