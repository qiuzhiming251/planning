

#include <new>
#include <ostream>

#include "Eigen/SparseLU"
#include "plan_common/math/cubic_spline.h"
#include "plan_common/math/eigen.h"
#include "plan_common/math/util.h"

namespace st {

void CubicSpline::Solve() {
  const int n = x_.size();
  SMatXd A(n, n);  // NOLINT(readability-identifier-naming)
  VecXd b(n, 1);
  constexpr double kThreeInv = 1.0 / 3.0;
  for (int i = 1; i < n - 1; i++) {
    A.insert(i, i - 1) = 1.0 * kThreeInv * (x_[i] - x_[i - 1]);
    A.insert(i, i) = 2.0 * kThreeInv * (x_[i + 1] - x_[i - 1]);
    A.insert(i, i + 1) = 1.0 * kThreeInv * (x_[i + 1] - x_[i]);
    b(i) = (y_[i + 1] - y_[i]) / (x_[i + 1] - x_[i]) -
           (y_[i] - y_[i - 1]) / (x_[i] - x_[i - 1]);
  }
  // Set boundary conditions.
  if (left_.type == BoundaryType::SOD) {
    A.insert(0, 0) = 2.0;
    A.insert(0, 1) = 0.0;
    b(0) = left_.value;
  } else if (left_.type == BoundaryType::FOD) {
    A.insert(0, 0) = 2.0 * (x_[1] - x_[0]);
    A.insert(0, 1) = 1.0 * (x_[1] - x_[0]);
    b(0) = 3.0 * ((y_[1] - y_[0]) / (x_[1] - x_[0]) - left_.value);
  } else {
    LOG_FATAL << "Unknown left boundary type " << static_cast<int>(left_.type);
  }
  if (right_.type == BoundaryType::SOD) {
    A.insert(n - 1, n - 1) = 2.0;
    A.insert(n - 1, n - 2) = 0.0;
    b(n - 1) = right_.value;
  } else if (right_.type == BoundaryType::FOD) {
    A.insert(n - 1, n - 1) = 2.0 * (x_[n - 1] - x_[n - 2]);
    A.insert(n - 1, n - 2) = 1.0 * (x_[n - 1] - x_[n - 2]);
    b(n - 1) = 3.0 * (right_.value -
                      (y_[n - 1] - y_[n - 2]) / (x_[n - 1] - x_[n - 2]));
  } else {
    LOG_FATAL << "Unknown right boundary type "
              << static_cast<int>(right_.type);
  }

  A.makeCompressed();
  Eigen::SparseLU<SMatXd> solver;
  solver.analyzePattern(A);
  solver.factorize(A);
  CHECK(solver.info() == Eigen::Success);

  const auto c = solver.solve(b);
  c_.clear();
  c_.reserve(n);
  for (int i = 0; i < n; ++i) {
    c_.push_back(c(i));
  }

  d_.clear();
  d_.resize(n);
  b_.clear();
  b_.resize(n);
  for (int i = 0; i < n - 1; ++i) {
    d_[i] = 1.0 * kThreeInv * (c_[i + 1] - c_[i]) / (x_[i + 1] - x_[i]);
    b_[i] = (y_[i + 1] - y_[i]) / (x_[i + 1] - x_[i]) -
            1.0 * kThreeInv * (2.0 * c_[i] + c_[i + 1]) * (x_[i + 1] - x_[i]);
  }
  const double h = x_[n - 1] - x_[n - 2];
  d_[n - 1] = 0.0;
  b_[n - 1] = 3.0 * d_[n - 2] * Sqr(h) + 2.0 * c_[n - 2] * h + b_[n - 2];
  if (right_.type == BoundaryType::FOD) {
    c_[n - 1] = 0.0;
  }

  c0_ = (left_.type == BoundaryType::FOD) ? 0.0 : c_[0];
}

double CubicSpline::Evaluate(double x) const {
  const int n = x_.size();
  double value;
  if (x < x_[0]) {
    // Extrapolation to the left.
    const double h = x - x_[0];
    value = (c0_ * h + b_[0]) * h + y_[0];
  } else if (x > x_[n - 1]) {
    // Extrapolation to the right
    const double h = x - x_[n - 1];
    value = (c_[n - 1] * h + b_[n - 1]) * h + y_[n - 1];
  } else {
    // Interpolation.
    const int idx = FindNearestIndex(x);
    const double h = x - x_[idx];
    value = ((d_[idx] * h + c_[idx]) * h + b_[idx]) * h + y_[idx];
  }

  return value;
}

}  // namespace st
