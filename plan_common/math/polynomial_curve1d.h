#ifndef POLYNOMIAL_CURVE1D_H_
#define POLYNOMIAL_CURVE1D_H_

#include "plan_common/math/curve1d.h"

namespace ad_byd {
namespace planning {

class PolynomialCurve1d : public Curve1d {
 public:
  PolynomialCurve1d() = default;
  ~PolynomialCurve1d() override = default;

  virtual double Coef(const size_t order) const = 0;
  virtual size_t Order() const = 0;

 protected:
  double param_ = 0.0;
};

}  // namespace planning
}  // namespace ad_byd
#endif