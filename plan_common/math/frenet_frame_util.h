

#ifndef ONBOARD_MATH_FRENET_FRAME_UTIL_H_
#define ONBOARD_MATH_FRENET_FRAME_UTIL_H_

#include "absl/types/span.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/vec.h"

namespace st::frenet_frame_util {

// TODO: refactor frenet_frame.h to replace this util.
Vec2d SLToXY(const FrenetCoordinate& sl, absl::Span<const Vec2d> anchor_points,
             absl::Span<const double> anchor_s);
FrenetCoordinate XYToSLBruteForce(const Vec2d& xy,
                                  absl::Span<const Vec2d> anchor_points,
                                  absl::Span<const double> anchor_s);

}  // namespace st::frenet_frame_util

#endif  // ONBOARD_MATH_FRENET_FRAME_UTIL_H_
