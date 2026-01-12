

#ifndef ST_PLANNING_MATH_GEOMETRY_POLYGON2D_UTIL
#define ST_PLANNING_MATH_GEOMETRY_POLYGON2D_UTIL

#include <string>

#include "absl/strings/str_cat.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/vec.h"

namespace st::polygon2d {

// Creates a convex regular polygon that is equiangular (all angles are equal in
// measure) and equilateral (all sides have the same length).
Polygon2d CreateRegularPolygon(int num_points, const Vec2d& center,
                               double radius, double first_point_angle);

// Returns the intersection range between a polygon and a half plane. The range
// is defined by the longitudinal range along the right side of the half plane.
//
//  Case A: When the polygon is completely at the left side of the half plane,
//  it returns the distance and the nearest point along the half plane. In this
//  case, in equals out.
//
//             ____
//            /   /
//           /   /
//          /   /
//          \ _/     _
//          ^        |
//        in = out   | dist positive.
//    ---------------------------> half plane direction.
//
//  Case B: When there is an overlap.
//             ____
//            /   / dist is negative.
//    -------/---/---------------> half plane direction.
//          /   /^
//         /___/ |
//        ^      |
//        |      |
//        in    out
//
//
//  Case C: When the polygon is completely at the right side of the half plane.
//  It returns the left most point and right most point's longitudinal
//  projection on half plane.
//
//    ---------------------------> half plane direction.
//             ____^   ^
//            /   /|   |
//           /   / |   |
//          /   /  |   | dist is negative.
//         /___/   |   _
//        ^        |
//        |        |
//        in      out
//
//
struct PolygonHalfPlaneOverlap {
  double dist;
  double in;
  double out;
  std::string DebugString() const {
    return absl::StrCat("dist: ", dist, ", in: ", in, ", out: ", out);
  }
};

PolygonHalfPlaneOverlap ComputePolygonHalfPlaneOverlap(
    const Polygon2d& polygon, const HalfPlane& halfplane);

struct PolygonBoxOverlap {
  // Positive when the polygon is completely at left side, and negative when the
  // polygon is completely at right side. 0.0 if the lateral distance is less
  // than half of the box's width. They may not overlap when lat_dist = 0.0.
  double lat_dist;
  // The start and end distance along the box's longitudinal direction that has
  // overlap with the polygon. The box's rear edge is 0.0 and the box's front
  // edge is at the box's `length`. When there is no overlap, they are equal to
  // the nearest place to the object.
  double in;
  double out;
  std::string DebugString() const {
    return absl::StrCat("lat_dist: ", lat_dist, ", in: ", in, ", out: ", out);
  }
};

PolygonBoxOverlap ComputePolygonBoxOverlap(const Polygon2d& polygon,
                                           const Box2d& box);

}  // namespace st::polygon2d

#endif  // ST_PLANNING_MATH_GEOMETRY_POLYGON2D_UTIL
