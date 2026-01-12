

#include "plan_common/maps/speed_bump.h"
namespace ad_byd {
namespace planning {
SpeedBump::SpeedBump(const SpeedBumpInfo &stop_line_info) {
  id_ = stop_line_info.id;
  points_.assign(stop_line_info.points.begin(), stop_line_info.points.end());
  CHECK(math::Polygon2d::ComputeConvexHull(points_, &polygon_));
}

double SpeedBump::DistanceTo(const Point2d &point) const {
  if (polygon_.is_convex()) {
    return polygon_.DistanceTo(point);
  }
  return std::numeric_limits<double>::max();
}
}  // namespace planning
}  // namespace ad_byd