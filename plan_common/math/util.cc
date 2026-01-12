

#include "plan_common/math/util.h"

namespace st {

std::vector<double> QuadraticRoot(double a, double b, double c) {
  constexpr double kEpsilon = 1e-10;
  if (std::abs(a) < kEpsilon) {
    if (std::abs(b) < kEpsilon) return {};
    return {-c / b};
  }
  const double d = b * b - 4 * a * c;
  if (d < 0.0) return {};
  const double sqrt_d = std::sqrt(d);
  const double t = 0.5 / a;
  std::vector<double> roots = {t * (-b + sqrt_d), t * (-b - sqrt_d)};
  if (roots[0] > roots[1]) std::swap(roots[0], roots[1]);
  return roots;
}

double FloorWithBase(double x, double base) {
  DCHECK_GT(base, 0.0);
  double residue = std::fmod(x, base);
  if (residue < 0.0) {
    residue += base;
  }
  return x - residue;
}

double QuadraticFunctionMinValue(double a, double b, double c, double x0,
                                 double x1) {
  DCHECK_GE(x1, x0);
  // Both ends.
  double min_y = a * Sqr(x0) + b * x0 + c;
  min_y = std::min(a * Sqr(x1) + b * x1 + c, min_y);

  if (a > 0.0) {
    const double xc = -b / (2.0 * a);
    if (xc > x0 && xc < x1) {
      min_y = std::min(a * Sqr(xc) + b * xc + c, min_y);
    }
  }
  return min_y;
}

}  // namespace st
