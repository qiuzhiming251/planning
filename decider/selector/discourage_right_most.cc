#include "decider/selector/discourage_right_most.h"
#include "decider/selector/cost_feature_util.h"
namespace st::planning {

bool CalculateDiscourageRightMost(
    const bool planner_is_bus_model, const bool is_right_most_lane,
    const bool enable_discourage_right_most_cost) {
  return !planner_is_bus_model && is_right_most_lane &&
         enable_discourage_right_most_cost;
}

double CalculateDiscourageDistFactor(const bool in_high_way,
                                     const double dist_to_virtual_lane) {
  double discourage_dist_factor = 1.0;
  if (!in_high_way) {
    constexpr double kMinJuncDistDiscourageRightMost = 150.0;  // m.
    const PiecewiseLinearFunction<double, double> kDiscourageFactorPlf = {
        {0.0, 20.0, 50.0, 80.0, 120.0, 150.0},  // length error
        {0.0, 0.1, 0.3, 0.5, 0.8, 1.0}};        // factor
    const double dist_error =
        dist_to_virtual_lane - kMinJuncDistDiscourageRightMost;
    discourage_dist_factor =
        dist_error > 0.0 ? kDiscourageFactorPlf(dist_error) : 0.0;
  }
  return discourage_dist_factor;
}

double CalculateStraightPreviewfactor(
    const bool in_high_way, const SelectorCommonFeature* common_feature) {
  double straight_preview_factor = 1.0;
  const auto& next_navi_op = common_feature->next_non_straight_navi_action_info;
  if (!in_high_way) {
    if (next_navi_op.has_value()) {
      if (!IsTurnRightNaviAction(*next_navi_op)) {
        constexpr double kMinDisCourageRightDist = 200.0;
        constexpr double kMaxDiscourageRightDist = 1000.0;
        const double may_go_straight_dist = std::min(
            /*InvaldLength=*/5000.0, next_navi_op->action_dis());
        straight_preview_factor =
            Lerp(0.0, 1.0,
                 (may_go_straight_dist - kMinDisCourageRightDist) /
                     (kMaxDiscourageRightDist - kMinDisCourageRightDist));
        straight_preview_factor = std::clamp(straight_preview_factor, 0.0, 1.0);
      } else {
      }
    }
  } else {
    if (next_navi_op.has_value()) {
      if (IsTurnRightNaviAction(*next_navi_op)) {
        constexpr double kHighWayMinDisCourageRightDist = 2000.0;
        constexpr double kHighWayMaxDiscourageRightDist = 3000.0;
        const double may_go_straight_dist = std::min(
            /*InvaldLength=*/5000.0, next_navi_op->action_dis());
        straight_preview_factor =
            Lerp(0.0, 1.0,
                 (may_go_straight_dist - kHighWayMinDisCourageRightDist) /
                     (kHighWayMaxDiscourageRightDist -
                      kHighWayMinDisCourageRightDist));
        straight_preview_factor = std::clamp(straight_preview_factor, 0.0, 1.0);
      }
    }
  }
  return straight_preview_factor;
}

}  // namespace st::planning
