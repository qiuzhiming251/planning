

#ifndef ONBOARD_PLANNER_SPEED_ST_POINT_H_
#define ONBOARD_PLANNER_SPEED_ST_POINT_H_

#include <string>

#include "absl/strings/str_format.h"
#include "plan_common/math/vec.h"

namespace st::planning {

class StPoint {
  // x-axis: t; y-axis: s.
 public:
  StPoint() = default;

  StPoint(double s, double t) : s_(s), t_(t) {}

  double s() const { return s_; }

  double t() const { return t_; }

  void set_s(double s) { s_ = s; }

  void set_t(double t) { t_ = t; }

  std::string DebugString() const {
    return absl::StrFormat("{ t : %.6f, s : %.6f }", t(), s());
  }

 private:
  double s_ = 0.0;
  double t_ = 0.0;
};

inline Vec2d ToVec2d(const StPoint& point) {
  return Vec2d(point.t(), point.s());
}

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_ST_POINT_H_
