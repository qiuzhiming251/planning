

#include <algorithm>
#include <cmath>
#include <ostream>
#include <utility>
#include <vector>

#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/polygon2d_util.h"

namespace st::polygon2d {

namespace {

// This is a specialized function with assumptions that . `hp` has an overlap
// with the polygon. `start` is a point on polygon at the left side of the half
// plane.
double FindHalfPlaneIntersection(const HalfPlane& hp, const Polygon2d& polygon,
                                 int start, int end) {
  const auto& points = polygon.points();
  const int n = polygon.num_points();
  const auto next = [n](int i) { return i + 1 < n ? i + 1 : 0; };

  double prev_lat = hp.lat_proj(points[start]);
  constexpr double kEpsilon = 1e-6;
  int i = start;
  while (i != end) {
    if (std::abs(prev_lat) < kEpsilon) return hp.lon_proj(points[i]);

    const int j = next(i);
    double next_lat = hp.lat_proj(points[j]);
    if (!((prev_lat > 0.0) ^ (next_lat > 0.0))) {
      // If they have the same sign, keep searching.
      prev_lat = next_lat;
      i = j;
      continue;
    } else {
      // They are not the same sign. The polygon edge crossed `hp`.
      const double i_lon = hp.lon_proj(points[i]);
      const double j_lon = hp.lon_proj(points[j]);
      return j_lon - (i_lon - j_lon) * next_lat / (prev_lat - next_lat);
    }
  }
  return 0.0;
}

// Compute polygon's left most point to to the half plane's distance. `left` is
// relative to the half plane's direction.
double ComputePolygonHalfPlaneLeftSideLateralDist(const Polygon2d& polygon,
                                                  const HalfPlane& hp) {
  const int highest = polygon.ExtremePoint(hp.tangent().Perp());
  return hp.lat_proj(polygon.points()[highest]);
}

PolygonHalfPlaneOverlap ComputeConvexPolygonHalfPlaneOverlap(
    const Polygon2d& polygon, const HalfPlane& hp) {
  PolygonHalfPlaneOverlap o;

  const auto& points = polygon.points();

  const int lowest = polygon.ExtremePoint(-hp.tangent().Perp());
  o.dist = hp.lat_proj(points[lowest]);

  // Case A: The polygon is at the left side of the half plane.
  if (o.dist > 0.0) {
    o.in = o.out = hp.lon_proj(points[lowest]);
    return o;
  }

  int left, right;
  polygon.ExtremePoints(hp.tangent(), &left, &right);

  if (const double left_depth = hp.lat_proj(points[left]); left_depth > 0.0) {
    // Case B.
    o.in = FindHalfPlaneIntersection(hp, polygon, left, lowest);
  } else {  // Case C.
    o.in = hp.lon_proj(points[left]);
  }

  if (const double right_depth = hp.lat_proj(points[right]);
      right_depth > 0.0) {
    // Case B.
    o.out = FindHalfPlaneIntersection(hp, polygon, lowest, right);
  } else {  // Case C.
    o.out = hp.lon_proj(points[right]);
  }

  return o;
}
PolygonHalfPlaneOverlap ComputeNonPolygonHalfPlaneOverlap(
    const Polygon2d& polygon, const HalfPlane& halfplane) {
  // Not implemented.
  PolygonHalfPlaneOverlap o{};
  CHECK(false) << "Not implemented yet.";
  return o;
}

}  // namespace

Polygon2d CreateRegularPolygon(int num_points, const Vec2d& center,
                               double radius, double first_point_angle) {
  CHECK_GE(num_points, 3);
  std::vector<Vec2d> points;
  points.reserve(num_points);
  const double angle_step = 2.0 * M_PI / num_points;
  double angle = first_point_angle;
  for (int i = 0; i < num_points; ++i, angle += angle_step) {
    const Vec2d dir = Vec2d::FastUnitFromAngle(angle);
    points.push_back(center + radius * dir);
  }
  return Polygon2d(std::move(points), /*is_convex=*/true);
}

PolygonHalfPlaneOverlap ComputePolygonHalfPlaneOverlap(const Polygon2d& polygon,
                                                       const HalfPlane& hp) {
  return polygon.is_convex() ? ComputeConvexPolygonHalfPlaneOverlap(polygon, hp)
                             : ComputeNonPolygonHalfPlaneOverlap(polygon, hp);
}

PolygonBoxOverlap ComputePolygonBoxOverlap(const Polygon2d& polygon,
                                           const Box2d& box) {
  const double box_length = box.length();

  const HalfPlane left_hp(box.GetCorner(Box2d::REAR_LEFT),
                          box.GetCorner(Box2d::FRONT_LEFT), box.tangent());

  PolygonBoxOverlap overlap;
  overlap.lat_dist = 0.0;

  const auto left_hp_overlap = ComputePolygonHalfPlaneOverlap(polygon, left_hp);
  if (left_hp_overlap.dist > 0.0) {
    overlap.lat_dist = left_hp_overlap.dist;
    overlap.in = overlap.out = std::clamp(left_hp_overlap.in, 0.0, box_length);
    return overlap;
  }
  if (left_hp_overlap.out < 0.0) {
    overlap.in = overlap.out = 0.0;
    overlap.lat_dist =
        std::min(0.0, box.width() + ComputePolygonHalfPlaneLeftSideLateralDist(
                                        polygon, left_hp));
    return overlap;
  } else if (left_hp_overlap.in > box_length) {
    overlap.in = overlap.out = box_length;
    overlap.lat_dist =
        std::min(0.0, box.width() + ComputePolygonHalfPlaneLeftSideLateralDist(
                                        polygon, left_hp));
    return overlap;
  }

  const HalfPlane right_hp(box.GetCorner(Box2d::FRONT_RIGHT),
                           box.GetCorner(Box2d::REAR_RIGHT), -box.tangent());
  const auto right_hp_overlap =
      ComputePolygonHalfPlaneOverlap(polygon, right_hp);
  if (right_hp_overlap.dist > 0.0) {
    overlap.lat_dist = -right_hp_overlap.dist;
    overlap.in = overlap.out =
        std::clamp(box.length() - right_hp_overlap.in, 0.0, box_length);
    return overlap;
  }

  overlap.lat_dist = 0.0;
  overlap.in = std::max(left_hp_overlap.in, box_length - right_hp_overlap.out);
  overlap.out = std::min(left_hp_overlap.out, box_length - right_hp_overlap.in);
  if (overlap.in > overlap.out) std::swap(overlap.in, overlap.out);

  if (overlap.in > box_length) {
    overlap.in = overlap.out = box_length;
  } else if (overlap.out < 0.0) {
    overlap.in = overlap.out = 0.0;
  }
  overlap.in = std::clamp(overlap.in, 0.0, box_length);
  overlap.out = std::clamp(overlap.out, 0.0, box_length);
  return overlap;
}

}  // namespace st::polygon2d
