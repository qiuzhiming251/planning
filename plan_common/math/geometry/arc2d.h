

#ifndef ST_PLANNING_MATH_GEOMETRY_ARC2D
#define ST_PLANNING_MATH_GEOMETRY_ARC2D

#include <algorithm>
#include <cmath>
#include <string>  // for string

#include "absl/strings/str_cat.h"  // for StrCat
#include "plan_common/math/geometry/circle2d.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/util.h"  // for NormalizeAngle
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/arc2d.pb.h"

namespace st {

/**
 * @class Arc2dHR
 * @brief Single piece of circle in 2-D, in halfplane-representation(HR).
 *
 * An arc is defined by a circle,
 * the starting angle of the arc, and the ending angle of the arc.
 * (From start to end, the point rotates counter-clockwise)
 *
 * The angle span is limited to be in [0.0, 2.0*M_PI).
 */
class Arc2dHR {
 public:
  Arc2dHR() = default;

  Arc2dHR(const Vec2d& center, double radius, const Vec2d& start_unit,
          const Vec2d& end_unit)
      : circle_(center, radius),
        start_bound_(center, center + radius * start_unit, start_unit),
        end_bound_(center, center + radius * end_unit, end_unit),
        is_inferior_arc_(start_bound_.IsPointInside(end_bound_.end())) {}

  const Circle2d& circle() const { return circle_; }

  const HalfPlane& start_bound() const { return start_bound_; }

  const HalfPlane& end_bound() const { return end_bound_; }

  bool is_inferior_arc() const { return is_inferior_arc_; }

  bool IsPointInBound(const Vec2d& point, double tolerance = 0.0) const {
    const bool in_start_bound = start_bound_.lat_proj(point) >= -tolerance;
    const bool in_end_bound = end_bound_.lat_proj(point) <= tolerance;
    if (is_inferior_arc_) {
      return in_start_bound && in_end_bound;
    } else {
      return in_start_bound || in_end_bound;
    }
  }

  bool IsPointIn(const Vec2d& point, double epsilon) const {
    return IsPointInBound(point, epsilon) && circle_.IsPointOn(point, epsilon);
  }

  /**
   * @brief Gets a human-readable description of the arc
   * @return A debug-string in full precision.
   */
  std::string DebugStringFullPrecision() const {
    return absl::StrFormat(
        "Arc2dHR(/*center=*/%s, /*radius=*/%s, "
        "/*start_unit=*/%s,/*end_unit=*/%s)",
        circle_.center().DebugStringFullPrecision(),
        absl::StrFormat("%.*e", DBL_DIG, circle_.radius()),
        start_bound_.tangent().DebugStringFullPrecision(),
        end_bound_.tangent().DebugStringFullPrecision());
  }

  static std::optional<Arc2dHR> FromProto(const Arc2dHRProto& proto) {
    const Vec2d circle_center = Vec2dFromProto(proto.circle_center());
    const double radius = proto.radius();

    if (radius <= 0.0) {
      return std::nullopt;
    }

    return Arc2dHR(circle_center, radius, Vec2dFromProto(proto.start_unit()),
                   Vec2dFromProto(proto.end_unit()));
  }

  void ToProto(Arc2dHRProto* proto) const {
    circle_.center().ToProto(proto->mutable_circle_center());
    proto->set_radius(circle_.radius());

    start_bound_.tangent().ToProto(proto->mutable_start_unit());
    end_bound_.tangent().ToProto(proto->mutable_end_unit());
  }

 private:
  Circle2d circle_;
  HalfPlane start_bound_;
  HalfPlane end_bound_;
  bool is_inferior_arc_ = false;
};

/**
 * @class Arc2dAR
 * @brief Single piece of circle in 2-D, in angle-representation.
 *
 * An arc is defined by a circle,
 * the starting angle of the arc, and the ending angle of the arc.
 * (From start to end, the point rotates counter-clockwise)
 *
 * The angle span is limited to be in [0.0, 2.0*M_PI).
 */
class Arc2dAR {
 public:
  static constexpr double kMinRadius = 1e-10;
  static constexpr double kMaxRadius = 1e7;
  static constexpr double kMaxRadiusUsingFastMath = 1e2;

  // Normalize to [0.0,2*pi)
  static double NormalizeAngleZeroToTwoPi(double angle) {
    return std::max(NormalizeAngle(angle - M_PI) + M_PI, 0.0);
  }

  Arc2dAR() = default;

  Arc2dAR(const Vec2d& center, double radius, double start_angle,
          double angle_span, bool use_fast_math = false)
      : circle_(center, radius),
        start_angle_(NormalizeAngle(start_angle)),
        angle_span_(NormalizeAngleZeroToTwoPi(angle_span)),
        arc_length_(angle_span_ * radius),
        radius_inv_(1.0 / radius),
        use_fast_math_(use_fast_math) {
    CHECK_GE(radius, kMinRadius);
    CHECK_LE(radius, use_fast_math_ ? kMaxRadiusUsingFastMath : kMaxRadius);
  }

  static Arc2dAR FromArc2dHR(const Arc2dHR& arc_hr,
                             bool use_fast_math = false) {
    const double start_angle = arc_hr.start_bound().tangent().Angle();
    const double end_angle = arc_hr.end_bound().tangent().Angle();
    const double angle_span =
        NormalizeAngleZeroToTwoPi(end_angle - start_angle);
    return Arc2dAR(arc_hr.circle().center(), arc_hr.circle().radius(),
                   start_angle, angle_span, use_fast_math);
  }

  double arc_length() const { return arc_length_; }

  double angle_span() const { return angle_span_; }

  const Circle2d& circle() const { return circle_; }

  /**
   * @brief find the unit() of "PointAtS(s) - center".
   * @param s: distance passed.
   * @return unit of "PointAtS(s) - center"
   */
  Vec2d UnitAtS(double s) const {
    const double angle_offset = s * radius_inv_;
    return use_fast_math_
               ? Vec2d::FastUnitFromAngle(start_angle_ + angle_offset)
               : Vec2d::UnitFromAngle(start_angle_ + angle_offset);
  }

  /**
   * @brief find the angle of "PointAtS(s) - center".
   * @param s: distance passed.
   * @return angle of "PointAtS(s) - center"
   */
  double AngleAtS(double s) const {
    return NormalizeAngle(start_angle_ + s * radius_inv_);
  }

  /**
   * @brief Given the unit() of "point - center" of a point on arc, find the
   * point.
   * @param unit: (point - center).unit()
   * @return The point.
   */
  Vec2d PointAtUnit(const Vec2d& unit) const {
    return circle().PointAtUnit(unit);
  }

  /**
   * @brief Move along arc by distance s from start point counter clockwise,
   * find the current point. If s is out of range, the point of the
   * circle extrapolation will be returned.
   * @param s: distance passed.
   * @return The current point.
   */
  Vec2d PointAtS(double s) const { return PointAtUnit(UnitAtS(s)); }

  /**
   * @brief Gets a human-readable description of the arc
   * @return A debug-string
   */
  std::string DebugString() const {
    return absl::StrCat("circle:(", circle_.DebugString(),
                        "),start_angle:", start_angle_,
                        ",angle_span:", angle_span_);
  }

  /**
   * @brief Gets a human-readable description of the arc
   * @return A debug-string in full precision.
   */
  std::string DebugStringFullPrecision() const {
    return absl::StrFormat(
        "Arc2dAR(/*center=*/%s, /*radius=*/%s, "
        "/*start_angle=*/%s,/*angle_span=*/%s,"
        "/*use_fast_math=*/%d)",
        circle_.center().DebugStringFullPrecision(),
        absl::StrFormat("%.*e", DBL_DIG, circle_.radius()),
        absl::StrFormat("%.*e", DBL_DIG, start_angle_),
        absl::StrFormat("%.*e", DBL_DIG, angle_span_), use_fast_math_);
  }

  static std::optional<Arc2dAR> FromProto(const Arc2dARProto& proto) {
    const Vec2d circle_center = Vec2dFromProto(proto.circle_center());
    const double radius = proto.radius();
    if (radius <= 0.0) {
      return std::nullopt;
    }

    return Arc2dAR(circle_center, radius, proto.start_angle(),
                   proto.angle_span(), proto.use_fast_math());
  }

  void ToProto(Arc2dARProto* proto) const {
    circle_.center().ToProto(proto->mutable_circle_center());
    proto->set_radius(circle_.radius());

    proto->set_start_angle(start_angle_);
    proto->set_angle_span(angle_span_);
    proto->set_use_fast_math(use_fast_math_);
  }

 private:
  Circle2d circle_;
  double start_angle_ = 0.0;
  double angle_span_ = 0.0;
  double arc_length_ = 0.0;
  double radius_inv_ = 0.0;
  bool use_fast_math_ = false;
};

}  // namespace st

#endif  // ST_PLANNING_MATH_GEOMETRY_ARC2D
