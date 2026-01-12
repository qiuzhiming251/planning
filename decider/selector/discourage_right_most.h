#pragma once
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "decider/selector/common_feature.h"
#include "decider/selector/selector_util.h"

namespace st::planning {

inline double CalcDiscourageRightCost(double av_speed) {
  static const PiecewiseLinearFunction<double, double> kMpsToHeadingFactorPlf =
      {{Kph2Mps(0.0), Kph2Mps(10.0), Kph2Mps(20.0), Kph2Mps(40.0),
        Kph2Mps(80.0)},
       {0.0, 0.3, 0.5, 0.8, 1.0}};
  return kMpsToHeadingFactorPlf(av_speed);
}

bool CalculateDiscourageRightMost(const bool planner_is_bus_model,
                                  const bool is_right_most_lane,
                                  const bool enable_discourage_right_most_cost);

double CalculateDiscourageDistFactor(const bool in_high_way,
                                     const double dist_to_virtual_lane);

double CalculateStraightPreviewfactor(
    const bool in_high_way, const SelectorCommonFeature* common_feature);

}  // namespace st::planning
