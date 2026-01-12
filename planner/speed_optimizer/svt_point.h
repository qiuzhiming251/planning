

#ifndef ONBOARD_PLANNER_SPEED_SVT_POINT_H_
#define ONBOARD_PLANNER_SPEED_SVT_POINT_H_

#include <string>

#include "absl/strings/str_format.h"
#include "plan_common/maps/st_point.h"

namespace st::planning {

class SvtPoint {
 public:
  SvtPoint() = default;

  SvtPoint(double s, double v, double t) : s_(s), v_(v), t_(t) {}

  double s() const { return s_; }

  double v() const { return v_; }

  double t() const { return t_; }

  void set_s(double s) { s_ = s; }

  void set_v(double v) { v_ = v; }

  void set_t(double t) { t_ = t; }

  StPoint GetStPoint() { return StPoint(s_, t_); }

  std::string DebugString() const {
    return absl::StrFormat("{ t : %.6f, v : %.6f, s : %.6f }", t_, v_, s_);
  }

 private:
  double s_ = 0.0;
  double v_ = 0.0;
  double t_ = 0.0;
};
}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_SVT_POINT_H_
