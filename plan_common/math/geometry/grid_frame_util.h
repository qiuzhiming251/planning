

#ifndef ST_PLANNING_MATH_GEOMETRY_GRID_FRAME_UTIL
#define ST_PLANNING_MATH_GEOMETRY_GRID_FRAME_UTIL

#include "absl/types/span.h"
#include "plan_common/math/geometry/circle2d.h"
#include "plan_common/math/geometry/grid_frame.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/vec.h"

namespace st {

// Polygon must be convex.
// Degenerated polygon not supported.
// Compute grid boxes that has overlap with the convex polygon region,
// using sweeping line.
// Time complexity O(max(y_range, n)), where y_range is the
// size of the of y-monotonic set along y axis, n is the number
// of polygon points.
YMonotonicGridSet2di CalculateGridsTouchingConvexPolygon(
    const GridFrame2d& grid_frame, const Polygon2d& polygon);

// Degenerated polygon not supported.
// Compute grid boxes that has overlap with the convex polygon
// defined points.
// Make sure points are from start to end in ccw order.
// using sweeping line.
// Time complexity O(max(y_range, n)), where y_range is the
// size of the of y-monotonic set along y axis, n is the number
// of polygon points.
YMonotonicGridSet2di CalculateGridsTouchingConvexPolygonPoints(
    const GridFrame2d& grid_frame, absl::Span<const Vec2d> points);

YMonotonicGridSet2di CalculateGridsTouchingCircle(const GridFrame2d& grid_frame,
                                                  const Circle2d& circle);

}  // namespace st

#endif  // ST_PLANNING_MATH_GEOMETRY_GRID_FRAME_UTIL
