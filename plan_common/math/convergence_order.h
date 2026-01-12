

#ifndef ONBOARD_MATH_CONVERGENCE_ORDER_H_
#define ONBOARD_MATH_CONVERGENCE_ORDER_H_

#include <functional>

namespace st {

int AssessConvergenceOrder(const std::function<double(double)>& f);

}  // namespace st

#endif  // ONBOARD_MATH_CONVERGENCE_ORDER_H_
