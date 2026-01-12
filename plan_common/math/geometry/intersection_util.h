

#ifndef ST_PLANNING_MATH_GEOMETRY_INTERSECTION_UTIL
#define ST_PLANNING_MATH_GEOMETRY_INTERSECTION_UTIL

#include <optional>
#include <utility>

#include "plan_common/math/geometry/arc2d.h"
#include "plan_common/math/geometry/circle2d.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/geometry/polyline2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/vec.h"

namespace st {

// The basic routines for finding intersection between two 2D lines. It is more
// efficient than similar functions in Segment2d such as GetIntersect() because
// the functions here don't require constructing Segment2d objects, which
// performs operations such as vector normalization that are unnecessary for
// many use cases of line intersection.

// The lines are given by a point and a tangent. The intersection point x can be
// computed by `p0 + v0 * s0 = p1 + v1 * s1`.
bool FindIntersectionBetweenLinesWithTangents(const Vec2d& p0, const Vec2d& t0,
                                              const Vec2d& p1, const Vec2d& t1,
                                              double* s0, double* s1,
                                              Vec2d* inter_point);

// Lite version of FindIntersectionBetweenLinesWithTangents.
// No s returned.
bool FindIntersectionBetweenLinesWithTangents(const Vec2d& p0, const Vec2d& t0,
                                              const Vec2d& p1, const Vec2d& t1,
                                              Vec2d* inter_point);

// Find intersections between two curves interpreted by discrete point. Return
// false if no intersection was found otherwise return the first intersection
// on curve_1
bool FindFirstIntersectionBetweenCurves(const Polyline2d& curve1,
                                        const Polyline2d& curve2,
                                        Vec2d* inter_point, double* arc_len1,
                                        double* arc_len2);

// Clamp the given segment by half-plane in place.
// After clamp all points will be nearly inside the given plane.
// Return: false if the segment is fully clamped to nothing(or zero length),
// else true.
// When returning false, the inout segment won't be changed.
bool ClampSegment2dByHalfPlane(const HalfPlane& plane, Segment2d* segment);

// Define up-intersection:
// Intersection whose segment projection greater equal to
// that of circle center.
// Define down-intersection:
// Intersection whose segment projection lower equal to
// that of circle center.
//
// The function finds the up-intersection and down-intersection
// between the circle and given segment. std::nullopt will be filled
// when intersection absent.
std::pair<std::optional<Vec2d> /*down-intersection*/,
          std::optional<Vec2d> /*up-intersection*/>
FindIntersectionBetweenCircle2dSegment2d(const Circle2d& circle,
                                         const Segment2d& segment);

// Define up-intersection:
// Intersection whose segment projection greater equal to
// that of circle center.
// Define down-intersection:
// Intersection whose segment projection lower equal to
// that of circle center.
//
// The function finds the up-intersection and down-intersection
// between the arc and given segment. std::nullopt will be filled
// when intersection absent.
std::pair<std::optional<Vec2d> /*down-intersection*/,
          std::optional<Vec2d> /*up-intersection*/>
FindIntersectionBetweenArc2dHRSegment2d(const Arc2dHR& arc_hr,
                                        const Segment2d& segment);

// Arc and Arc please. today, you win
// Given arc a1 and a2. find their intersections.
// Define Segment2d(a1.circle().center(), a2.circle().center()) as seg0.
// Define left-intersection:
// Intersection such that its product onto seg0 greater equal to zero.
// Define right-intersection:
// Intersection such that its product onto seg0 smaller equal to zero.
//
// The function finds the left-intersection and right-intersection
// between the arc a1 and a2. std::nullopt will be filled
// when intersection absent.
std::pair<std::optional<Vec2d> /*left-intersection*/,
          std::optional<Vec2d> /*right-intersection*/>
FindIntersectionBetweenArc2dHRs(const Arc2dHR& a1, const Arc2dHR& a2);

}  // namespace st

#endif  // ST_PLANNING_MATH_GEOMETRY_INTERSECTION_UTIL
