

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "plan_common/log.h"
#include "plan_common/math/geometry/intersection_util.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/util.h"

namespace st {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}
bool FindIntersectionBetweenLinesWithTangents(const Vec2d& p0, const Vec2d& t0,
                                              const Vec2d& p1, const Vec2d& t1,
                                              double* s0, double* s1,
                                              Vec2d* inter_point) {
  DCHECK(s0 != nullptr);
  DCHECK(s1 != nullptr);
  DCHECK(inter_point != nullptr);
  // We have the following equation:
  // p0 + t0 * s0 = p1 + t1 * s1;
  // Expand it by xy we get the following two equations:
  // p0.x() + t0.x() * s0 = p1.x() + t1.x() * s1;
  // p0.y() + t0.y() * s0 = p1.y() + t1.y() * s1;
  //
  // Reshape the equations:
  // t0.x() * s0 - t1.x() * s1 = p1.x() - p0.x()
  // t0.y() * s0 - t1.y() * s1 = p1.y() - p0.y()
  //
  // Let p0p1 = p1 - p0;
  // The solution is three cross products.
  // det = t1.x() * t0.y() - t1.y() * t0.x() = t1.CrossProd(t0);
  // s0 = 1.0 / det * (-t1.y(), t1.x()).Dot(p1 - p0)
  // s0 = -p0p1.CrossProd(t1) * 1.0 / det;

  // s1 = 1.0 / det * (-t0.y(), t0.x()).Dot(p1 - p0)
  // s1 = -p0p1.CrossProd(t0) * 1.0 / det;
  const double det = t1.CrossProd(t0);
  constexpr double kEpsilon = 1e-20;
  if (std::abs(det) < kEpsilon) {
    *s0 = kInf;
    *s1 = kInf;
    return false;
  }
  const double inv_det = -1.0 / det;
  const Vec2d p0p1 = p1 - p0;
  *s0 = inv_det * p0p1.CrossProd(t1);
  *s1 = inv_det * p0p1.CrossProd(t0);
  *inter_point = p0 + *s0 * t0;
  return true;
}

bool FindIntersectionBetweenLinesWithTangents(const Vec2d& p0, const Vec2d& t0,
                                              const Vec2d& p1, const Vec2d& t1,
                                              Vec2d* inter_point) {
  double un_used_1, un_used_2;
  return FindIntersectionBetweenLinesWithTangents(p0, t0, p1, t1, &un_used_1,
                                                  &un_used_2, inter_point);
}

bool FindFirstIntersectionBetweenCurves(const Polyline2d& curve1,
                                        const Polyline2d& curve2,
                                        Vec2d* inter_point, double* arc_len1,
                                        double* arc_len2) {
  CHECK_NOTNULL(inter_point);

  for (int i = 0; i + 1 < curve1.points().size(); ++i) {
    const Segment2d seg1(curve1.points()[i], curve1.points()[i + 1]);
    for (int j = 0; j + 1 < curve2.points().size(); ++j) {
      const Segment2d seg2(curve2.points()[j], curve2.points()[j + 1]);
      if (seg1.GetIntersect(seg2, inter_point)) {
        *arc_len1 =
            curve1.point_s()[i] + inter_point->DistanceTo(curve1.points()[i]);

        *arc_len2 =
            curve2.point_s()[j] + inter_point->DistanceTo(curve2.points()[j]);

        return true;
      }
    }
  }
  return false;
}

bool ClampSegment2dByHalfPlane(const HalfPlane& plane, Segment2d* segment) {
  constexpr double kEpsilon = 1e-9;
  DCHECK_NOTNULL(segment);

  const Vec2d& u1 = segment->unit_direction();
  const Vec2d& u2 = plane.tangent();
  const Vec2d& s1 = segment->start();

  const double u1_prod = u2.CrossProd(u1);
  const double s1_proj = plane.lat_proj(s1);
  const bool s1_in_plane = s1_proj >= 0.0;
  if (u1_prod > -kEpsilon && u1_prod < kEpsilon) {
    // Parrallel.
    return s1_in_plane;
  }
  if (s1_in_plane && u1_prod > 0.0) {
    // Segment start in plane and pointing inward.
    return true;
  }
  if (!s1_in_plane && u1_prod < 0.0) {
    // Segment start outside plane and pointing outward.
    return false;
  }

  // It must ge 0.0, the max action is used to avoid CHECK.
  const double critical_seg_proj = std::max(0.0, -s1_proj / u1_prod);

  if (s1_in_plane) {
    // s1_proj >= 0.0 && u1_prod < 0.0
    segment->set_length(std::min(segment->length(), critical_seg_proj));
    return true;
  } else {
    // s1_proj < 0.0 && u1_prod > 0.0
    const double new_length = segment->length() - critical_seg_proj;
    if (new_length <= 0.0) {
      return false;
    } else {
      const Vec2d offset = u1 * critical_seg_proj;
      segment->Shift(offset);
      segment->set_length(new_length);
      return true;
    }
  }
}

std::pair<std::optional<Vec2d> /*down-intersection*/,
          std::optional<Vec2d> /*up-intersection*/>
FindIntersectionBetweenCircle2dSegment2d(const Circle2d& circle,
                                         const Segment2d& segment) {
  constexpr double kEpsilon = 1e-9;
  // Handle degenerated case.
  if (segment.length() < kEpsilon) {
    if (circle.IsPointOn(segment.start(), Sqr(kEpsilon))) {
      return {segment.start(), segment.start()};
    } else {
      return {std::nullopt, std::nullopt};
    }
  }

  // Line is laterally away from circle.
  const double center_prod = segment.ProductOntoUnit(circle.center());
  if (std::abs(center_prod) > circle.radius()) {
    return {std::nullopt, std::nullopt};
  }

  const double center_proj = segment.ProjectOntoUnit(circle.center());
  const double radius_sqr = Sqr(circle.radius());

  // Segment end proj smaller than down-intersection s.
  const double end_dist_sqr = segment.end().DistanceSquareTo(circle.center());
  if (center_proj > segment.length() && end_dist_sqr > radius_sqr) {
    return {std::nullopt, std::nullopt};
  }

  // Segment start proj greater than up-intersection s.
  const double start_dist_sqr =
      segment.start().DistanceSquareTo(circle.center());
  if (center_proj < 0.0 && start_dist_sqr > radius_sqr) {
    return {std::nullopt, std::nullopt};
  }

  const double half_chord_length =
      std::sqrt(std::max(0.0, radius_sqr - Sqr(center_prod)));
  std::pair<std::optional<Vec2d>, std::optional<Vec2d>> down_up;
  // Down intersection
  const double down_intersection_proj = center_proj - half_chord_length;
  if (down_intersection_proj >= 0.0 &&
      down_intersection_proj <= segment.length()) {
    down_up.first =
        segment.start() + down_intersection_proj * segment.unit_direction();
  }
  // Up intersection.
  const double up_intersection_proj = center_proj + half_chord_length;
  if (up_intersection_proj >= 0.0 && up_intersection_proj <= segment.length()) {
    down_up.second =
        segment.start() + up_intersection_proj * segment.unit_direction();
  }
  return down_up;
}

namespace {
void UpdatePairOptional(
    const std::pair<std::optional<Vec2d>, std::optional<Vec2d>>& input,
    std::pair<std::optional<Vec2d>, std::optional<Vec2d>>* buffer) {
  DCHECK_NOTNULL(buffer);
  if (input.first.has_value()) {
    buffer->first = *input.first;
  }
  if (input.second.has_value()) {
    buffer->second = *input.second;
  }
}
}  // namespace

std::pair<std::optional<Vec2d> /*down-intersection*/,
          std::optional<Vec2d> /*up-intersection*/>
FindIntersectionBetweenArc2dHRSegment2d(const Arc2dHR& arc_hr,
                                        const Segment2d& segment) {
  if (arc_hr.is_inferior_arc()) {
    // Clamp segment by start and end bound, then find segment intersection with
    // circle.
    Segment2d seg_in_bound = segment;
    if (!ClampSegment2dByHalfPlane(arc_hr.start_bound(), &seg_in_bound)) {
      return {std::nullopt, std::nullopt};
    }
    if (!ClampSegment2dByHalfPlane(arc_hr.end_bound().Inversed(),
                                   &seg_in_bound)) {
      return {std::nullopt, std::nullopt};
    }
    return FindIntersectionBetweenCircle2dSegment2d(arc_hr.circle(),
                                                    seg_in_bound);
  } else {
    // Merge result from two segment pieces, each clamped by one bound of the
    // arc.
    std::pair<std::optional<Vec2d>, std::optional<Vec2d>> down_up;
    Segment2d seg_part_1 = segment;
    if (ClampSegment2dByHalfPlane(arc_hr.start_bound(), &seg_part_1)) {
      UpdatePairOptional(
          FindIntersectionBetweenCircle2dSegment2d(arc_hr.circle(), seg_part_1),
          &down_up);
    }
    if (down_up.first.has_value() && down_up.second.has_value()) {
      return down_up;
    }
    Segment2d seg_part_2 = segment;
    if (ClampSegment2dByHalfPlane(arc_hr.end_bound().Inversed(), &seg_part_2)) {
      UpdatePairOptional(
          FindIntersectionBetweenCircle2dSegment2d(arc_hr.circle(), seg_part_2),
          &down_up);
    }
    return down_up;
  }
}

namespace {
// Say there is an inferior arc
// whose cricle intersects other circle.
// The arc starts from point s and ends
// at point e.
// Define arc circle center c1 to other circle
// center c2 as seg0.
// Also give whether s and e are on the left side of seg0.
// With information above, tell whether the arc-circle intersection
// exists on the left side of seg0.
bool InferiorArcHasLeftIntersectionWithCircle(
    double s_to_other_circle_center_sqr, double e_to_other_circle_center_sqr,
    double other_circle_radius_sqr, bool s_on_left, bool e_on_left) {
  if (s_on_left && s_to_other_circle_center_sqr > other_circle_radius_sqr) {
    return false;
  }
  if (e_on_left && e_to_other_circle_center_sqr < other_circle_radius_sqr) {
    return false;
  }
  return s_on_left || e_on_left;
}

// Definition similar as InferiorArcHasLeftIntersectionWithCircle.
bool InferiorArcHasRightIntersectionWithCircle(
    double s_to_other_circle_center_sqr, double e_to_other_circle_center_sqr,
    double other_circle_radius_sqr, bool s_on_right, bool e_on_right) {
  if (s_on_right && s_to_other_circle_center_sqr < other_circle_radius_sqr) {
    return false;
  }
  if (e_on_right && e_to_other_circle_center_sqr > other_circle_radius_sqr) {
    return false;
  }
  return s_on_right || e_on_right;
}

}  // namespace

std::pair<std::optional<Vec2d> /*left-intersection*/,
          std::optional<Vec2d> /*right-intersection*/>
FindIntersectionBetweenArc2dHRs(const Arc2dHR& a1, const Arc2dHR& a2) {
  constexpr double kEpsilon = 1e-9;
  const Vec2d& c1 = a1.circle().center();
  Vec2d c2 = a2.circle().center();

  Vec2d offset = c2 - c1;
  double offset_sqr = offset.Sqr();

  // Move a little bit to avoid zero offset.
  if (offset_sqr < Sqr(kEpsilon)) {
    offset_sqr = Sqr(kEpsilon);
    offset = Vec2d(kEpsilon, 0.0);
    c2 = c1 + offset;
  }

  const double r1 = a1.circle().radius();
  const double r2 = a2.circle().radius();

  if (offset_sqr > Sqr(r1 + r2)) {
    // Circles away.
    return {std::nullopt, std::nullopt};
  }
  if (offset_sqr < Sqr(r1 - r2)) {
    // Circles one in another.
    return {std::nullopt, std::nullopt};
  }

  const double r1_sqr = Sqr(r1);
  const double r2_sqr = Sqr(r2);

  bool has_left_intersection = true;
  bool has_right_intersection = true;
  // Wipe out intersection by a1's bounds.
  {
    const Vec2d& s1u = a1.start_bound().tangent();
    const Vec2d& e1u = a1.end_bound().tangent();
    const double s1_to_c2_sqr = (c1 + s1u * r1 - c2).Sqr();
    const double e1_to_c2_sqr = (c1 + e1u * r1 - c2).Sqr();
    const double s1_prod = offset.CrossProd(s1u);
    const double e1_prod = offset.CrossProd(e1u);

    if (a1.is_inferior_arc()) {
      has_left_intersection =
          has_left_intersection && InferiorArcHasLeftIntersectionWithCircle(
                                       s1_to_c2_sqr, e1_to_c2_sqr, r2_sqr,
                                       s1_prod >= 0.0, e1_prod >= 0.0);
      has_right_intersection =
          has_right_intersection && InferiorArcHasRightIntersectionWithCircle(
                                        s1_to_c2_sqr, e1_to_c2_sqr, r2_sqr,
                                        s1_prod <= 0.0, e1_prod <= 0.0);
    } else {
      // Simply let it rotates from end to start, tell whether the complimentary
      // arc has left/right intersection. (The two ends get open for superior
      // arc, user should take care of epsilon by himself/herself).
      has_left_intersection =
          has_left_intersection && !InferiorArcHasLeftIntersectionWithCircle(
                                       e1_to_c2_sqr, s1_to_c2_sqr, r2_sqr,
                                       e1_prod >= 0.0, s1_prod >= 0.0);
      has_right_intersection =
          has_right_intersection && !InferiorArcHasRightIntersectionWithCircle(
                                        e1_to_c2_sqr, s1_to_c2_sqr, r2_sqr,
                                        e1_prod <= 0.0, s1_prod <= 0.0);
    }
  }

  // Wipe out intersection by a2's bounds.
  if (has_left_intersection || has_right_intersection) {
    const Vec2d& s2u = a2.start_bound().tangent();
    const Vec2d& e2u = a2.end_bound().tangent();
    const double s2_to_c1_sqr = (c2 + s2u * r2 - c1).Sqr();
    const double e2_to_c1_sqr = (c2 + e2u * r2 - c1).Sqr();
    const double s2_prod = -offset.CrossProd(s2u);
    const double e2_prod = -offset.CrossProd(e2u);

    if (a2.is_inferior_arc()) {
      has_right_intersection =
          has_right_intersection && InferiorArcHasLeftIntersectionWithCircle(
                                        s2_to_c1_sqr, e2_to_c1_sqr, r1_sqr,
                                        s2_prod >= 0.0, e2_prod >= 0.0);
      has_left_intersection =
          has_left_intersection && InferiorArcHasRightIntersectionWithCircle(
                                       s2_to_c1_sqr, e2_to_c1_sqr, r1_sqr,
                                       s2_prod <= 0.0, e2_prod <= 0.0);
    } else {
      // Simply let it rotates from end to start, tell whether the complimentary
      // arc has left/right intersection. (The two ends get open for superior
      // arc, user should take care of epsilon by himself/herself).
      has_right_intersection =
          has_right_intersection && !InferiorArcHasLeftIntersectionWithCircle(
                                        e2_to_c1_sqr, s2_to_c1_sqr, r1_sqr,
                                        e2_prod >= 0.0, s2_prod >= 0.0);
      has_left_intersection =
          has_left_intersection && !InferiorArcHasRightIntersectionWithCircle(
                                       e2_to_c1_sqr, s2_to_c1_sqr, r1_sqr,
                                       e2_prod <= 0.0, s2_prod <= 0.0);
    }
  }

  // Early return to skip sqrt.
  if (!has_left_intersection && !has_right_intersection) {
    return {std::nullopt, std::nullopt};
  }

  // Get theta and offset unit.
  const double offset_length = offset.Length();
  const Vec2d offset_unit = offset * (1.0 / offset_length);

  double cos_theta =
      (offset_sqr + r1_sqr - r2_sqr) / (2.0 * offset_length * r1);
  cos_theta = std::clamp(cos_theta, -1.0, 1.0);
  const double sin_theta = std::sqrt(1.0 - Sqr(cos_theta));

  // Fill results.
  std::pair<std::optional<Vec2d>, std::optional<Vec2d>> left_right;
  if (has_left_intersection) {
    left_right.first = c1 + offset_unit.Rotate(cos_theta, sin_theta) * r1;
  }
  if (has_right_intersection) {
    left_right.second = c1 + offset_unit.Rotate(cos_theta, -sin_theta) * r1;
  }

  return left_right;
}

}  // namespace st
