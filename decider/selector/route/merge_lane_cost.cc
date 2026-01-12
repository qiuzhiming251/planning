#include "decider/selector/route/merge_lane_cost.h"

#include <string>
#include <sstream>

#include "plan_common/math/piecewise_linear_function.h"

namespace st::planning {

namespace {
constexpr double kMaxLengthCheckSplit = 150.0;  // m.

bool IsMergeAfterSplit(const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
                       double dist_to_split, double dist_to_merge) {
  if (lane_seq_info == nullptr || lane_seq_info->split_task_state ==
                                      ad_byd::planning::SplitTasksState::None) {
    return false;
  }
  return dist_to_split < kMaxLengthCheckSplit && dist_to_split < dist_to_merge;
}

bool IsMergeAfterTurnJunction(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    double len_before_merge, double dist_to_exit_junction) {
  if (lane_seq_info == nullptr || lane_seq_info->junction_lane == nullptr) {
    return false;
  }
  const auto& junction_lane = lane_seq_info->junction_lane;
  if (junction_lane->turn_type() != ad_byd::planning::RIGHT_TURN &&
      junction_lane->turn_type() != ad_byd::planning::LEFT_TURN &&
      junction_lane->turn_type() != ad_byd::planning::U_TURN) {
    return false;
  }
  return len_before_merge > dist_to_exit_junction;
}

bool IsMergeAfterStraightJunction(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    const double len_before_merge, const double dist_to_exit_junction) {
  if (lane_seq_info == nullptr || lane_seq_info->junction_lane == nullptr) {
    return false;
  }
  const auto& junction_lane = lane_seq_info->junction_lane;
  if (junction_lane->turn_type() != ad_byd::planning::NO_TURN) {
    return false;
  }
  return len_before_merge > dist_to_exit_junction;
}

double CalcHighwayMergeCost(double dist_to_merge,
                            const std::optional<double>& dist_to_split) {
  static const PiecewiseLinearFunction<double, double> kMergeLaneCostPlf = {
      {400.0, 800.0, 1000.0, 1200.0},  // length error
      {1.0, 0.5, 0.2, 0.0}};           // factor
  static const PiecewiseLinearFunction<double, double> kMergeAfterSplitCostPlf =
      {{200.0, 300.0, 600.0},  // length error
       {1.0, 0.7, 0.0}};       // factor
  if (dist_to_split.has_value() && *dist_to_split < dist_to_merge) {
    return kMergeAfterSplitCostPlf(dist_to_merge - *dist_to_split);
  }
  return kMergeLaneCostPlf(dist_to_merge);
}

double CalcCityMergeCost(double dist_to_merge) {
  static const PiecewiseLinearFunction<double, double> kMergeLaneCostPlf = {
      {200.0, 300.0, 600.0},  // length error
      {1.0, 0.7, 0.0}};       // factor
  return kMergeLaneCostPlf(dist_to_merge);
}

}  // namespace

std::string MergeLaneCostInput::DebugString() const {
  std::stringstream ss;
  ss << "on_highway: " << std::boolalpha << on_highway
     << ", dist_to_merge: " << dist_to_merge;
  ss << ", dist_to_split: " << dist_to_split.has_value()
      ? std::to_string(*dist_to_split)
      : "none";
  ss << ", dist_to_exit_junction: " << dist_to_exit_junction.has_value()
      ? std::to_string(*dist_to_exit_junction)
      : "none";
  return ss.str();
}

double CalceMergeLaneCost(const MergeLaneCostInput& input) {
  double merge_cross_decress_factor = 1.0;
  if (input.dist_to_exit_junction.has_value() &&
      IsMergeAfterStraightJunction(input.lane_seq_info, input.dist_to_merge,
                                   *input.dist_to_exit_junction)) {
    const double merge_length_after_juntion =
        std::max(input.dist_to_merge - *input.dist_to_exit_junction, 0.0);
    static const PiecewiseLinearFunction<double, double> kDampingFactor = {
        {0.0, 20.0, 40.0, 60.0, 70.0},  // length error
        {1.0, 0.9, 0.7, 0.3, 0.0}};     // factor
    merge_cross_decress_factor = kDampingFactor(merge_length_after_juntion);
  }
  return input.on_highway
             ? CalcHighwayMergeCost(input.dist_to_merge, input.dist_to_split)
             : CalcCityMergeCost(input.dist_to_merge) *
                   merge_cross_decress_factor;
}

double CalceMergedAreaCost(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    std::vector<std::string>* extra_info) {
  if (lane_seq_info == nullptr) {
    return 0.0;
  }
  const double start_dist_merged = lane_seq_info->dist_to_merged_zone.first;
  const double end_dist_merged = lane_seq_info->dist_to_merged_zone.second;
  const double dist_to_merge = lane_seq_info->dist_to_merge;
  constexpr double kMaxDistChecked = 2000.0;  // m.
  if (std::abs(start_dist_merged) > kMaxDistChecked) {
    return 0.0;
  }
  extra_info->emplace_back(
      absl::StrFormat("start_dist_merged: %.2f, end_dist_merged: %.2f",
                      start_dist_merged, end_dist_merged));
  if (dist_to_merge > start_dist_merged && dist_to_merge < end_dist_merged) {
    return 0.0;
  }
  constexpr double kDistThresholdAvoidMerged = 700.0;  // m.
  double merged_area_cost = 0.0;
  if (start_dist_merged < 0.0 && end_dist_merged > 0.0) {
    merged_area_cost = 1.0;
  } else if (start_dist_merged < kDistThresholdAvoidMerged) {
    const PiecewiseLinearFunction<double, double> kDecayFactorPlf = {
        {0.0, 200.0, 300.0, 600.0, 700.0},  // distance
        {1.0, 0.9, 0.8, 0.2, 0.0}};         // factor
    merged_area_cost = kDecayFactorPlf(start_dist_merged);
  } else {
    merged_area_cost = 0.0;
  }
  return merged_area_cost;
}

}  // namespace st::planning
