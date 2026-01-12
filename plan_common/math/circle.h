

#ifndef ONBOARD_MATH_CIRCLE_H_
#define ONBOARD_MATH_CIRCLE_H_

#include "plan_common/math/vec.h"

namespace st {

class Circle {
 public:
  Circle() = default;
  Circle(const Vec2d& center, double radius)
      : center_(center), radius_(radius) {}

  Vec2d EvaluateXY(double theta) const;
  double EvaluateTheta(const Vec2d& xy) const;
  double EvaluateTangent(double theta) const;
  double DistanceTo(const Vec2d& xy) const;

  const Vec2d& center() const { return center_; }
  double radius() const { return radius_; }

 private:
  Vec2d center_ = {0.0, 0.0};
  double radius_ = 0.0;
};

}  // namespace st

#endif  // ONBOARD_MATH_CIRCLE_H_
