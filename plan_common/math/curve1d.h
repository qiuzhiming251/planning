//
// Created by xxx on 08/10/21.
//
#ifndef CURVE1D_H_
#define CURVE1D_H_

#include <string>

namespace ad_byd {
namespace planning {
class Curve1d {
 public:
  Curve1d() = default;

  virtual ~Curve1d() = default;

  virtual double Evaluate(const std::uint32_t order,
                          const double param) const = 0;
  // virtual double GetParams(double& s1, double& s2, double& s3, double& p)
  // const = 0; virtual double SearchX(double res) const;
  virtual double ParamLength() const = 0;

  virtual std::string ToString() const = 0;
};

}  // namespace planning
}  // namespace ad_byd
#endif
