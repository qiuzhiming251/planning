

#include "plan_common/maps/crosswalk.h"
#include "plan_common/math/geometry/polygon2d.h"
namespace ad_byd {
namespace planning {
Crosswalk::Crosswalk(const CrossWalkInfo& cross_walk_info) {
  id_ = cross_walk_info.id;
  points_.assign(cross_walk_info.points.begin(), cross_walk_info.points.end());
  is_polygon_convex_ = math::Polygon2d::ComputeConvexHull(points_, &polygon_);
  bone_axis_smooth_ = st::Polygon2d(points_).GetPrincipalAxis();
}

double Crosswalk::DistanceTo(const Point2d& point) const {
  if (is_polygon_convex_) return polygon_.DistanceTo(point);
  return std::numeric_limits<double>::max();
}

}  // namespace planning
}  // namespace ad_byd