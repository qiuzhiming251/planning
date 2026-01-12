#include "decider/selector/speed_cost_estimator.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <string>
#include <sstream>
#include <vector>

#include "absl/strings/str_format.h"
#include "decider/selector/cost_feature_util.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"

#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_debug.pb.h"

namespace st::planning {

namespace {
constexpr double kMinSpeedIncrementLowerBound = 1.0;  // mps
constexpr double kMinSpeedIncrementUpperBound = 4.0;  // m/s
constexpr double kSpeedIncrementLowerBoundRatio = 0.1;
constexpr double kSpeedIncrementUpperBoundRatio = 0.3;
constexpr double kMinLaneSpeedLimit = 10.0;           // m/s.
constexpr double kHighWayLowSpeedLowerBound = 5.56;   // m/s.
constexpr double kHighWayLowSpeedUpperBound = 11.11;  // m/s.
constexpr double kCutOffLaneSpeedDiffRatioCity = 0.2;
constexpr double kBeginLaneSpeedDiffRatioForHighway = 0.1;
constexpr double kGeneralLaneChangeDiffRatioForHighway = 0.2;
constexpr double kForceLaneChangeDiffRatioForHighway = 0.4;
constexpr double kHighwayConversativeProgressFactor = 1.0;
constexpr double kHighwayProgressFactor = 1.5;
constexpr double kBeginLaneSpeedDiffCostFactor = 0.15;
constexpr double kGeneralLaneChangeDiffCostFactor = 0.5;
constexpr double kForceLaneChangeDiffCostFactor = 0.8;
constexpr double kCityLaneSpeedDiffCutOffRatio = 0.13;
constexpr double kCityLaneSpeedDiffMaxRatio = 0.53;
constexpr double kCityNearIntersectionDis = 200.0;     // m.
constexpr double kCityFarFromIntersectionDis = 300.0;  // m.

constexpr double kCityLaneSpeedDiffMaxRatioCity = 1.0;
constexpr double kCityLaneSpeedDiffMaxRatioCityBeforeIntersection = 1.5;

double CalculateTransitionWeight(const double x, const double s1,
                                 const double s2) {
  if (std::fabs(s1 - s2) < 1e-6) return 0.0;

  const double lower = std::min(s1, s2);
  const double upper = std::max(s1, s2);
  if (x <= lower) return 0.0;
  if (x >= upper) return 1.0;
  const double t = (x - lower) / (upper - lower);
  return 6.0 * std::pow(t, 5) - 15.0 * std::pow(t, 4) + 10.0 * std::pow(t, 3);
}

double CalculateTranstionCost(const double s, const double c1,
                              const double c2) {
  const double w = CalculateTransitionWeight(s, kCityNearIntersectionDis,
                                             kCityFarFromIntersectionDis);
  return w * c1 + (1.0 - w) * c2;
}

}  // namespace

double GetDrivingStyleFactor(int driving_style_gear, bool is_highway) {
  if (driving_style_gear == 3) {
    return is_highway ? 1.0 : 2.0;  // 5.0;
  } else if (driving_style_gear == 2) {
    return is_highway ? 1.0 : 2.0;  // 3.0;
  } else {
    return 1.0;
  }
}

std::string DebugFormat(const SpeedLimitCostOutput& output) {
  return absl::StrFormat(
      "progress_factor: %.2f, slow_leader_factor: %.2f, "
      "lane_speed_diff_ratio: %.2f, driving_style_factor: %.2f, "
      "slow_factor: %.2f, speed_limit_cost: %.2f",
      output.progress_factor(), output.slow_leader_factor(),
      output.lane_speed_diff_ratio(), output.driving_style_factor(),
      output.slow_av_factor(), output.speed_limit_cost());
}

std::string SpeedLimitCostInput::DebugString() const {
  return absl::StrFormat(
      "is_highway: %d, driving_style_factor: %.1f, max_lane_speed: %.2f, "
      "lane_speed_limit: %.2f, max_init_leader_speed: %.2f, init_leader_speed: "
      "%.2f, lane_speed_limit_by_leader: %.2f, ego_v: %.2f, "
      "dist_to_virtual_lane: %.2f, valid_lane_num: %d, conversative_style: %d",
      is_highway, driving_style_factor, max_lane_speed, lane_speed_limit,
      max_init_leader_speed, init_leader_speed, lane_speed_limit_by_leader,
      ego_v, dist_to_virtual_lane, valid_lane_num, is_conversative_style);
}

SpeedLimitCostOutput CalculateSpeedLimitCost(
    const SpeedLimitCostInput& speed_limit_cost_input) {
  bool is_highway = speed_limit_cost_input.is_highway;

  auto driving_style_factor = speed_limit_cost_input.driving_style_factor;
  auto max_lane_speed = speed_limit_cost_input.max_lane_speed;
  auto lane_speed_limit = speed_limit_cost_input.lane_speed_limit;
  auto max_init_leader_speed = speed_limit_cost_input.max_init_leader_speed;
  auto init_leader_speed = speed_limit_cost_input.init_leader_speed;
  auto lane_speed_limit_by_leader =
      speed_limit_cost_input.lane_speed_limit_by_leader;
  auto ego_v = speed_limit_cost_input.ego_v;
  auto dist_to_virtual_lane = speed_limit_cost_input.dist_to_virtual_lane;
  auto valid_lane_num = speed_limit_cost_input.valid_lane_num;
  auto is_conversative_style = speed_limit_cost_input.is_conversative_style;
  // Avoid unnecessary lc when leader v is far higher than ego.
  const auto speed_lower_bound =
      std::max(kMinSpeedIncrementLowerBound,
               kSpeedIncrementLowerBoundRatio * lane_speed_limit);
  const auto speed_upper_bound =
      std::max(kMinSpeedIncrementUpperBound,
               kSpeedIncrementUpperBoundRatio * lane_speed_limit);
  const double speed_increment =
      std::min(std::max(speed_lower_bound, init_leader_speed - ego_v),
               speed_upper_bound);
  const double slow_leader_factor = (speed_upper_bound - speed_increment) /
                                    (speed_upper_bound - speed_lower_bound);
  // Avoid overtake lc in low speed in highway && city.
  static const PiecewiseLinearFunction<double, double> kSlowAvFactorPlf[2] = {
      {{kHighWayLowSpeedLowerBound, kHighWayLowSpeedUpperBound},
       {0.0, 1.0}},  // highway
      {{Kph2Mps(3.0), Kph2Mps(6.0), Kph2Mps(10.0), Kph2Mps(30.0)},
       {0.0, 0.2, 0.5, 1.0}}};
  double slow_factor = kSlowAvFactorPlf[is_highway ? 0 : 1](
      std::fmax(max_init_leader_speed, std::fabs(ego_v)));
  double leader_speed_diff = max_init_leader_speed - init_leader_speed;
  double lane_speed_diff_ratio = driving_style_factor * slow_leader_factor *
                                 slow_factor *
                                 (max_lane_speed - lane_speed_limit_by_leader) /
                                 std::max(lane_speed_limit, kMinLaneSpeedLimit);
  double cost = 0.0;
  if (is_highway) {
    if (leader_speed_diff < Kph2Mps(5.0) &&
        (lane_speed_diff_ratio < kCutOffLaneSpeedDiffRatio ||
         leader_speed_diff < kMinSpeedIncrementLowerBound)) {
      // Ignore 10% lower leader.
      cost = 0.0;
    } else if (lane_speed_diff_ratio < kBeginLaneSpeedDiffRatioForHighway) {
      // Smooth cost from 10% to 15%.
      cost = LinearInterpolate(
          0.0, kBeginLaneSpeedDiffCostFactor, kCutOffLaneSpeedDiffRatio,
          kBeginLaneSpeedDiffRatioForHighway, lane_speed_diff_ratio);
      if (leader_speed_diff > Kph2Mps(5.0) &&
          leader_speed_diff < Kph2Mps(10.0)) {
        cost = std::max(cost, 0.1);
      } else if (leader_speed_diff >= Kph2Mps(10.0)) {
        cost = std::max(cost, 0.15);
      }
      cost = std::clamp(cost, 0.08, kBeginLaneSpeedDiffCostFactor);
    } else {
      static const PiecewiseLinearFunction<double, double> kSpeedDiffRatioPlf =
          {{kBeginLaneSpeedDiffRatioForHighway,
            kGeneralLaneChangeDiffRatioForHighway,
            kForceLaneChangeDiffRatioForHighway, 1.0},
           {kBeginLaneSpeedDiffCostFactor, kGeneralLaneChangeDiffCostFactor,
            kForceLaneChangeDiffCostFactor, 1.0}};
      cost = kSpeedDiffRatioPlf(lane_speed_diff_ratio);
    }
    cost = std::clamp(cost, 0.0, 1.0) *
           (is_conversative_style ? kHighwayConversativeProgressFactor
                                  : kHighwayProgressFactor);
  } else {
    if (lane_speed_diff_ratio < kCutOffLaneSpeedDiffRatioCity) {
      cost = 0.0;
    } else {
      const double cost_far_from_intersection = LinearInterpolate(
          0.0, 1.0, kCutOffLaneSpeedDiffRatioCity,
          kCityLaneSpeedDiffMaxRatioCity, lane_speed_diff_ratio);
      const double cost_near_intersection =
          LinearInterpolate(0.0, 1.0, kCutOffLaneSpeedDiffRatioCity,
                            kCityLaneSpeedDiffMaxRatioCityBeforeIntersection,
                            lane_speed_diff_ratio);

      if (dist_to_virtual_lane > kCityFarFromIntersectionDis) {
        cost = cost_far_from_intersection;
      } else if (dist_to_virtual_lane < kCityNearIntersectionDis) {
        cost = cost_near_intersection;
      } else {
        cost = CalculateTranstionCost(dist_to_virtual_lane,
                                      cost_far_from_intersection,
                                      cost_near_intersection);
      }
      cost = std::clamp(cost, 0.0, 1.0);
    }
  }
  SpeedLimitCostOutput output;
  output.set_progress_factor(1 - lane_speed_diff_ratio);  // Maybe minus
  output.set_slow_leader_factor(slow_leader_factor);
  output.set_lane_speed_diff_ratio(lane_speed_diff_ratio);
  output.set_driving_style_factor(driving_style_factor);
  output.set_slow_av_factor(slow_factor);
  output.set_speed_limit_cost(cost);
  return output;
}

}  // namespace st::planning
