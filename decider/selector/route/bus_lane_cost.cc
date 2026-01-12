#include "decider/selector/route/bus_lane_cost.h"

#include <string>
#include <sstream>

#include "plan_common/math/piecewise_linear_function.h"

namespace st::planning {

namespace {
std::optional<std::pair<double, double>> GetNearestBusLaneDistRange(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info) {
  if (lane_seq_info == nullptr) {
    return std::nullopt;
  }
  if (lane_seq_info->dist_to_bus_lane_vec.empty()) {
    return std::nullopt;
  }
  constexpr double kMaxDistBusLaneConsidered = 1500.0;  // m.
  if (lane_seq_info->dist_to_bus_lane_vec.front().first <
      kMaxDistBusLaneConsidered) {
    return lane_seq_info->dist_to_bus_lane_vec.front();
  }
  return std::nullopt;
}

bool IsBusLaneAfterJunction(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    const double start_s_bus_lane) {
  if (lane_seq_info == nullptr) {
    return false;
  }
  const double dist_to_virtual_lane = lane_seq_info->dist_to_virtual_lane;
  return start_s_bus_lane > dist_to_virtual_lane;
}

bool IsIgnoreBusLaneOnNavi(const LaneFeatureInfo& lane_feature_info,
                           const double max_navi_dist_keep) {
  if (!lane_feature_info.is_only_min_lc_num) {
    return false;
  }
  if (lane_feature_info.is_keep_lane) {
    return true;
  } else {
    constexpr double kDistThreshIgnoreBusLaneOnNavi = 300.0;  // m.
    return max_navi_dist_keep < kDistThreshIgnoreBusLaneOnNavi;
  }
}

}  // namespace

double CalcLaneChangeToBusLaneCost(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    const LaneFeatureInfo& lane_feature_info, const double max_navi_dist_keep) {
  double lc_to_bus_lane_cost = 0.0;
  if (lane_seq_info == nullptr) {
    return lc_to_bus_lane_cost;
  }
  std::optional<std::pair<double, double>> dist_range =
      GetNearestBusLaneDistRange(lane_seq_info);
  if (!dist_range.has_value() ||
      lane_seq_info->bus_lane_passable_mark.empty()) {
    return lc_to_bus_lane_cost;
  }
  const uint32_t passable_state = lane_seq_info->bus_lane_passable_mark.front();
  if (passable_state == 1) {  // 可通行
    return lc_to_bus_lane_cost;
  }
  const double start_s_bus_lane = dist_range.value().first;
  const double end_s_bus_lane = dist_range.value().second;
  if (IsBusLaneAfterJunction(lane_seq_info, start_s_bus_lane)) {
    return lc_to_bus_lane_cost;
  }

  if (IsIgnoreBusLaneOnNavi(lane_feature_info, max_navi_dist_keep)) {
    return lc_to_bus_lane_cost;
  }

  constexpr double kMinDistAvoidBusLane = 50.0;         // m.
  constexpr double kDistThresholdAvoidBusLane = 400.0;  // m.
  if (start_s_bus_lane < kMinDistAvoidBusLane) {
    lc_to_bus_lane_cost = end_s_bus_lane > 5.0 ? 1.0 : 0.0;
  } else if (start_s_bus_lane < kDistThresholdAvoidBusLane) {
    const PiecewiseLinearFunction<double, double> kDecayFactorPlf = {
        {0.0, 100.0, 200.0, 300.0, 400.0},  // distance
        {1.0, 0.85, 0.6, 0.2, 0.0}};        // factor
    lc_to_bus_lane_cost = kDecayFactorPlf(start_s_bus_lane);
  } else {
    lc_to_bus_lane_cost = 0.0;
  }
  return lc_to_bus_lane_cost;
}

double CalcRouteBusLaneCost(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    const LaneFeatureInfo& lane_feature_info, const double max_navi_dist_keep,
    std::vector<std::string>* extra_info) {
  CHECK_NOTNULL(extra_info);
  double bus_lane_cost = 0.0;
  if (lane_seq_info == nullptr) {
    return bus_lane_cost;
  }

  std::optional<std::pair<double, double>> dist_range =
      GetNearestBusLaneDistRange(lane_seq_info);
  if (!dist_range.has_value() ||
      lane_seq_info->bus_lane_passable_mark.empty()) {
    return bus_lane_cost;
  }
  const uint32_t passable_state = lane_seq_info->bus_lane_passable_mark.front();
  const double start_s_bus_lane = dist_range.value().first;
  const double end_s_bus_lane = dist_range.value().second;
  extra_info->emplace_back(absl::StrFormat(
      "start_s_bus_lane: %.2f, end_s_bus_lane: %.2f, passable_state: %d",
      start_s_bus_lane, end_s_bus_lane, passable_state));

  const bool is_passable = lane_feature_info.is_keep_lane
                               ? passable_state == 0 || passable_state == 1
                               : passable_state == 1;  // 0:unknow; 1:可通行
  if (is_passable) {
    return bus_lane_cost;
  }
  if (IsBusLaneAfterJunction(lane_seq_info, start_s_bus_lane)) {
    return bus_lane_cost;
  }

  if (IsIgnoreBusLaneOnNavi(lane_feature_info, max_navi_dist_keep)) {
    return bus_lane_cost;
  }

  if (start_s_bus_lane < 5.0) {
    bus_lane_cost = end_s_bus_lane > 5.0 ? 1.0 : 0.0;
  } else if (start_s_bus_lane < 200.0) {
    const PiecewiseLinearFunction<double, double> kDecayFactorPlf = {
        {0.0, 30.0, 60.0, 100.0, 150.0, 200.0},  // distance
        {1.0, 0.95, 0.85, 0.6, 0.2, 0.0}};       // factor
    bus_lane_cost = kDecayFactorPlf(start_s_bus_lane);
  } else {
    bus_lane_cost = 0.0;
  }
  return bus_lane_cost;
}

}  // namespace st::planning
