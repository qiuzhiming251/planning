#pragma once

#include <vector>

#include "plan_common/math/piecewise_linear_function.h"

namespace st {
template <typename TY = double, typename TX = double,
          typename LERPER = Lerper<TY, TX>>
class PiecewiseBilinearFunction {
 public:
  PiecewiseBilinearFunction() = default;
  PiecewiseBilinearFunction(std::vector<TX> x,
                            std::vector<PiecewiseLinearFunction<TX, TY>> y)
      : x_(std::move(x)), y_(std::move(y)), lerper_(LERPER()) {
    CHECK_EQ(x_.size(), y_.size());
    CHECK_GT(x_.size(), 1);
    x_slope_.reserve(x_.size() - 1);
    for (size_t i = 1; i < x_.size(); ++i) {
      x_slope_.push_back(TX(1) / (x_[i] - x_[i - 1]));
    }
  }

  TY Evaluate(TX x, TY y) const {
    const int index = std::upper_bound(x_.begin(), x_.end(), x) - x_.begin();
    if (index == 0) return y_.front().Evaluate(y);
    if (index == static_cast<int>(x_.size())) return y_.back().Evaluate(y);
    DCHECK_GT(x_[index], x_[index - 1]);  // Guaranteed by std::upper_bound().
    const TX alpha = (x - x_[index - 1]) * x_slope_[index - 1];
    return lerper_(y_[index - 1].Evaluate(y), y_[index].Evaluate(y), alpha);
  }

  TY operator()(TX x, TY y) const { return Evaluate(x, y); }

 private:
  std::vector<TX> x_;
  std::vector<PiecewiseLinearFunction<TX, TY>> y_;
  std::vector<TX> x_slope_;
  LERPER lerper_;
};
}  // namespace st
