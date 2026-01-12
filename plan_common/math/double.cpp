
#include <cmath>

#include "plan_common/log.h"

#include "plan_common/log.h"
#include "plan_common/math/double.h"

namespace ad_byd {
namespace planning {
namespace math {

double Double::epsilon_ = 1e-6;

Double::CompareType Double::Compare(const double& a, const double& b) {
  return Double::Compare(a, b, epsilon_);
}

Double::CompareType Double::Compare(const double& a, const double& b,
                                    const double& epsilon) {
  CHECK(!(std::isnan(a) && std::isnan(b) && std::isnan(epsilon)));
  if (LessThan(a, b, epsilon)) {
    return CompareType::LESS;
  } else if (GreaterThan(a, b, epsilon)) {
    return CompareType::GREATER;
  } else {
    return CompareType::EQUAL;
  }
}

bool Double::LessThan(const double& a, const double& b, const double& epsilon) {
  return b - a > epsilon;
}

bool Double::GreaterThan(const double& a, const double& b,
                         const double& epsilon) {
  return a - b > epsilon;
}

}  // namespace math
}  // namespace planning
}  // namespace ad_byd
