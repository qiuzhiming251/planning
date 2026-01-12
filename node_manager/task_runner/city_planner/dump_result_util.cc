#include "dump_result_util.h"

#include <cmath>

namespace st::planning {

void DumpSlBoundaryToReferenceLine(const PathSlBoundary& sl_boundary,
                                   byd::msg::planning::PathInfo* reference_line,
                                   int sample_interval) {
  CHECK_NOTNULL(reference_line);
  reference_line->Clear();
  const auto& ref_center = sl_boundary.reference_center_xy_vector();
  sample_interval = std::max(1, sample_interval);
  reference_line->mutable_points()->Reserve(ref_center.size() /
                                            sample_interval);
  for (int i = 0; i < static_cast<int>(ref_center.size());
       i += sample_interval) {
    auto* ref_point = reference_line->add_points();
    ref_point->set_x(ref_center[i].x());
    ref_point->set_y(ref_center[i].y());
  }
}

void DumpSlBoundaryToDrivingBoundary(
    const PathSlBoundary& sl_boundary,
    byd::msg::planning::DrivingBoundary* driving_boundary,
    int sample_interval) {
  CHECK_NOTNULL(driving_boundary);
  driving_boundary->Clear();
  const auto& left_pts = sl_boundary.left_xy_vector();
  const auto& right_pts = sl_boundary.right_xy_vector();
  sample_interval = std::max(1, sample_interval);
  driving_boundary->mutable_left_boundary()->Reserve(left_pts.size() /
                                                     sample_interval);
  driving_boundary->mutable_right_boundary()->Reserve(right_pts.size() /
                                                      sample_interval);

  for (size_t i = 0; i < left_pts.size(); i += sample_interval) {
    auto* left_pos = driving_boundary->add_left_boundary();
    left_pos->set_x(left_pts[i].x());
    left_pos->set_y(left_pts[i].y());
  }
  for (size_t i = 0; i < right_pts.size(); i += sample_interval) {
    auto* right_pos = driving_boundary->add_right_boundary();
    right_pos->set_x(right_pts[i].x());
    right_pos->set_y(right_pts[i].y());
  }
}

}  // namespace st::planning
