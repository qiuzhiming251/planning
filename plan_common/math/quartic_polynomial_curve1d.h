#ifndef QUARTIC_POLYNOMIAL_CURVE1D_H_
#define QUARTIC_POLYNOMIAL_CURVE1D_H_
#include <array>

#include "plan_common/math/polynomial_curve1d.h"
namespace ad_byd {
namespace planning {

// 1D quartic polynomial curve: (x0, dx0, ddx0) -- [0, param] --> (dx1, ddx1)
class QuarticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  QuarticPolynomialCurve1d() = default;

  QuarticPolynomialCurve1d(const std::array<double, 3>& start,
                           const std::array<double, 2>& end,
                           const double param);

  QuarticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                           const double dx1, const double ddx1,
                           const double param);

  QuarticPolynomialCurve1d(const QuarticPolynomialCurve1d& other);

  ~QuarticPolynomialCurve1d() override = default;

  double Evaluate(const std::uint32_t order, const double p) const override;

  double ParamLength() const override { return param_; }

  std::string ToString() const override;

  double Coef(const size_t order) const override;

  size_t Order() const override { return 4; }

  bool GetParams(double& s1, double& s2, double& s3, double& p) const;

 private:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double dx1, const double ddx1,
                           const double param);

  std::array<double, 5> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
  std::array<double, 2> end_condition_ = {{0.0, 0.0}};
};
}  // namespace planning
}  // namespace ad_byd
#endif