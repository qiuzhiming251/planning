

#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_STATE_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_STATE_H_

#include <string>

#include "absl/strings/str_cat.h"
#include "plan_common/math/vec.h"

namespace st {
namespace planning {

struct GeometryState {
  Vec2d xy;
  double h = 0.0;      // heading.
  double k = 0.0;      // curvature.
  double ref_k = 0.0;  // reference curvature on centerline.
  // Approximate frenet variable for cost computation later.
  double accumulated_s = 0.0;  // station s.
  double l = 0.0;              // lateral offset in drive passage.
  std::string DebugString() const {
    return absl::StrCat("xy: ", xy.DebugString(), " h: ", h, " k: ", k,
                        " accumulated_s:", accumulated_s, " l: ", l, " dk:", dk,
                        " ddk: ", ddk);
  }
  double dk = 0.0;  // derivative of curvature (for quintic spiral computation).
  double ddk = 0.0;  // second derivative of curvature.
};

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_STATE_H_
