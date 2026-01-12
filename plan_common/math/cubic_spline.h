

#ifndef ONBOARD_MATH_CUBIC_SPLINE_H_
#define ONBOARD_MATH_CUBIC_SPLINE_H_

#include <algorithm>
#include <utility>
#include <vector>

#include "plan_common/log.h"

namespace st {

class CubicSpline {
 public:
  enum class BoundaryType {
    FOD = 0,  // First-order derivative.
    SOD = 1   // Second-order derivative.
  };
  struct BoundaryCondition {
    BoundaryType type = BoundaryType::SOD;
    double value = 0.0;
  };

  CubicSpline(std::vector<double> x, std::vector<double> y)
      : x_(std::move(x)),
        y_(std::move(y)),
        left_({.type = BoundaryType::SOD, .value = 0.0}),
        right_({.type = BoundaryType::SOD, .value = 0.0}) {
    CheckInput(x_, y_);
    Solve();
  }

  CubicSpline(std::vector<double> x, std::vector<double> y,
              BoundaryCondition left, BoundaryCondition right)
      : x_(std::move(x)), y_(std::move(y)), left_(left), right_(right) {
    CheckInput(x_, y_);
    Solve();
  }

  // Evaluate value and derivatives at a given x.
  double Evaluate(double x) const;

  template <int Order, typename std::enable_if_t<(Order > 0), bool> = 0>
  double EvaluateDerivative(double x) const;

  const std::vector<double>& x() const { return x_; }
  const std::vector<double>& y() const { return y_; }

 protected:
  int FindNearestIndex(double x) const {
    const auto it = std::upper_bound(x_.begin(), x_.end(), x);
    const int idx = std::max(static_cast<int>(it - x_.begin() - 1), 0);
    return idx;
  }

  void CheckInput(const std::vector<double>& x, const std::vector<double>& y) {
    CHECK_EQ(x.size(), y.size());
    CHECK_GT(x.size(), 1);
    // Check strict monotonicity of x.
    for (int i = 0; i < x.size() - 1; ++i) {
      CHECK_LT(x[i], x[i + 1]);
    }
  }

  void Solve();

  std::vector<double> x_;  // x coordinates.
  std::vector<double> y_;  // y coordinates.

  // f(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3 where a_i = y_i
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;

  double c0_ = 0.0;
  BoundaryCondition left_;
  BoundaryCondition right_;
};

template <int Order, typename std::enable_if_t<(Order > 0), bool>>
double CubicSpline::EvaluateDerivative(double x) const {
  if constexpr (Order > 3) {
    return 0.0;
  } else {
    const int n = x_.size();
    double value = 0.0;
    if (x < x_[0]) {
      const double h = x - x_[0];
      if constexpr (Order == 1) {
        value = 2.0 * c0_ * h + b_[0];
      } else if constexpr (Order == 2) {
        value = 2.0 * c0_;
      } else {
        value = 0.0;
      }
    } else if (x > x_[n - 1]) {
      const double h = x - x_[n - 1];
      if constexpr (Order == 1) {
        value = 2.0 * c_[n - 1] * h + b_[n - 1];
      } else if constexpr (Order == 2) {
        value = 2.0 * c_[n - 1];
      } else {
        value = 0.0;
      }
    } else {
      const int idx = FindNearestIndex(x);
      const double h = x - x_[idx];
      if constexpr (Order == 1) {
        value = (3.0 * d_[idx] * h + 2.0 * c_[idx]) * h + b_[idx];
      } else if constexpr (Order == 2) {
        value = 6.0 * d_[idx] * h + 2.0 * c_[idx];
      } else if constexpr (Order == 3) {
        value = 6.0 * d_[idx];
      } else {
        value = 0.0;
      }
    }
    return value;
  }
}
}  // namespace st

#endif  // ONBOARD_MATH_CUBIC_SPLINE_H_
