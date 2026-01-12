#pragma once

#include <cfloat>
#include <string>
#include <limits>
#include <vector>

#include "plan_common/maps/lane_sequence.h"
#include "decider/selector/common_feature.h"

namespace st::planning {

double CalcLaneChangeToBusLaneCost(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    const LaneFeatureInfo& lane_feature_info, const double max_navi_dist_keep);

double CalcRouteBusLaneCost(
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    const LaneFeatureInfo& lane_feature_info, const double max_navi_dist_keep,
    std::vector<std::string>* extra_info);

}  // namespace st::planning
