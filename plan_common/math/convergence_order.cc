

#include <cmath>
#include <ostream>
#include <string>
#include <vector>

#include "absl/strings/str_join.h"
#include "plan_common/log.h"
#include "plan_common/math/convergence_order.h"
#include "plan_common/math/eigen.h"

namespace st {

int AssessConvergenceOrder(const std::function<double(double)>& f) {
  constexpr double kFpFloor = 1e-12;
  constexpr double kMinDynamicRange = 1e+3;
  constexpr double kXMin = 1e-3;
  constexpr double kXMax = 1.0;
  std::vector<double> xs, ys;
  double x = kXMin;
  double y = f(x);
  VLOG(4) << "x = " << x << " y = " << y;
  while (y < kFpFloor * kMinDynamicRange && x < kXMax) {
    x *= 2;
    y = f(x);
    VLOG(4) << "x = " << x << " y = " << y;
  }
  if (x >= kXMax) {
    // y is always numerically zero. No convergence property.
    VLOG(2) << "y is always numerically zero.";
    return -1;
  }

  while (y >= kFpFloor && x * 0.5 != x) {
    xs.push_back(x);
    ys.push_back(y);
    x *= 0.5;
    y = f(x);
    VLOG(4) << "x = " << x << " y = " << y;
  }

  VLOG(3) << "xs: " << absl::StrJoin(xs, " ");
  VLOG(3) << "ys: " << absl::StrJoin(ys, " ");
  if (xs.size() < 2) {
    // Not enough data points to assess order of convergence. Shouldn't happen.
    return -2;
  }

  CHECK_EQ(xs.size(), ys.size());
  const int n = xs.size();
  VecXd log_xs = VecXd::Zero(n);
  VecXd log_ys = VecXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    log_xs[i] = std::log(xs[i]);
    log_ys[i] = std::log(ys[i]);
  }
  log_xs -= MatXd::Ones(n, n) * log_xs / n;
  log_ys -= MatXd::Ones(n, n) * log_ys / n;
  VLOG(3) << "log_xs: " << log_xs.transpose();
  VLOG(3) << "log_ys: " << log_ys.transpose();

  // Linear regression for the order of convergence.
  const double k = log_xs.dot(log_ys) / log_xs.squaredNorm();
  VLOG(3) << "k = " << k;
  return std::round(k);
}

}  // namespace st
