
#include "plan_common/gflags.h"
#include "plan_common/maps/lane_boundary.h"
namespace ad_byd {
namespace planning {

LaneBoundary::LaneBoundary(const LaneBoundaryInfo& boundary_info) {
  points_.assign(boundary_info.points.begin(), boundary_info.points.end());
  id_ = boundary_info.id;
  type_ = boundary_info.boundary_type;
  InterpolatePoints();
}

void LaneBoundary::InterpolatePoints() {
  if (points_.size() < 2u) {
    return;
  }
  std::vector<Point2d> interpolate_points;
  interpolate_points.emplace_back(points_.front());
  const double map_gap_threshold_sqr =
      FLAGS_ad_byd_planning_map_point_distance_threshold *
      FLAGS_ad_byd_planning_map_point_distance_threshold;
  double total_length = 0.0;
  for (std::size_t i = 1; i < points_.size(); ++i) {
    total_length += points_.at(i).DistanceTo(points_.at(i - 1));
  }
  math::LineCurve2d tmp_curve(points_);
  tmp_curve.SamplePoints(0, total_length, &interpolate_points,
                         FLAGS_ad_byd_planning_boundary_interpolate_dist);
  if (interpolate_points.empty()) {
    interpolate_points.emplace_back(points_.front());
    interpolate_points.emplace_back(points_.back());
  } else if (interpolate_points.back().DistanceSquareTo(points_.back()) >
             map_gap_threshold_sqr) {
    interpolate_points.emplace_back(points_.back());
  }
  line_curve_.InitializePoints(interpolate_points);
  curve_length_ = line_curve_.length();
}

}  // namespace planning
}  // namespace ad_byd