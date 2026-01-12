

#ifndef ONBOARD_PLANNER_SPEED_VT_POINT_H_
#define ONBOARD_PLANNER_SPEED_VT_POINT_H_

#include <string>

#include "absl/strings/str_format.h"
#include "plan_common/math/vec.h"

namespace st::planning {

class VtPoint {
  // x-axis: t; y-axis: v.
 public:
  VtPoint() = default;

  VtPoint(double v, double t) : v_(v), t_(t) {}

  double v() const { return v_; }

  double t() const { return t_; }

  void set_v(double v) { v_ = v; }

  void set_t(double t) { t_ = t; }

  std::string DebugString() const {
    return absl::StrFormat("{ t : %.6f, v : %.6f }", t(), v());
  }

 private:
  double v_ = 0.0;
  double t_ = 0.0;
};

inline Vec2d ToVec2d(const VtPoint& point) {
  return Vec2d(point.t(), point.v());
}

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_VT_POINT_H_
