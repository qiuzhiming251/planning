

#ifndef ONBOARD_MATH_CIRCLE_FITTER_H_
#define ONBOARD_MATH_CIRCLE_FITTER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/math/circle.h"
#include "plan_common/math/vec.h"

namespace st {

/*
  Solver for linear least squares systems (LS).
   - SVD: The SVD decomposition, generally the most accurate yet the slowest.
   - QR: The QR decomposition, moderately accurate and fast.
   - PINV: The pseudo-inverse, the fastest but least accurate.
 */
enum LS_SOLVER {
  SVD = 0,
  QR = 1,
  PINV = 2,
};

/*
  Fit a circle from input training data, expressed in the form of a center and a
  radius, via the weighted least squares (LS) methed.
*/
absl::StatusOr<Circle> FitCircleToData(
    const std::vector<Vec2d>& data,
    const std::vector<double>& weights = std::vector<double>(),
    LS_SOLVER solver = SVD, double* mse = nullptr);

}  // namespace st

#endif  // ONBOARD_MATH_CIRCLE_FITTER_H_
