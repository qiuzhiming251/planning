#pragma once

#include <cfloat>
#include <string>
#include <vector>
#include <limits>

#include "plan_common/maps/lane_sequence.h"

namespace st::planning {

struct MergeLaneCostInput {
  bool on_highway = false;
  double dist_to_merge = DBL_MAX;
  ad_byd::planning::LaneSeqInfoPtr lane_seq_info = nullptr;
  std::optional<double> dist_to_split;
  std::optional<double> dist_to_exit_junction;

  std::string DebugString() const;
};

double CalceMergeLaneCost(const MergeLaneCostInput& input);

double CalceMergedAreaCost(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    std::vector<std::string>* extra_info);

}  // namespace st::planning
