#undef BOOST_TEST_MAIN

#include "decider/selector/selector.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <valarray>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "plan_common/log.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "gtest/gtest.h"

#include "plan_common/container/strong_int.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/drive_passage.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/path/path.h"
#include "decider/selector/candidate_stats.h"
#include "decider/selector/common_feature.h"
#include "decider/selector/cost_feature_base.h"
#include "decider/selector/cost_feature_util.h"
#include "decider/selector/ensure_lc_feasibility.h"
#include "decider/selector/selector_defs.h"
#include "decider/selector/selector_util.h"
#include "decider/selector/traj_cost_features.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/scene_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/time_util.h"
#include "modules/cnoa_pnc/planning/proto/hmi_content.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"

namespace st::planning {
namespace {
TEST(SelectorTest, AnalyzeLaneChangeType) {
  absl::flat_hash_map<int, TrajFeatureOutput> idx_traj_feature_output_map = {
      {0,
       {
           .has_obvious_route_cost = true,
           .is_perform_lane_change = false,
           .lane_change_left = true,
           .lane_change_for_route_cost = true,
       }},
      {1,
       {
           .is_perform_lane_change = true,
           .lane_change_for_moving_obj = true,
       }},
  };

  EXPECT_EQ(LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE,
            internal::AnalyzeLaneChangeType(idx_traj_feature_output_map,
                                            /*is_paddle_lane_change=*/false,
                                            /*last_selected_idx=*/0,
                                            /*final_chosen_idx=*/1,
                                            /*last_lane_change_type=*/
                                            LaneChangeType::TYPE_NO_CHANGE));
}

TEST(SelectorTest, ReAnalyzeLaneChangeReason) {
  EXPECT_EQ(LaneChangeReason::AVOID_STATIC_OBJECT,
            internal::ReAnalyzeLaneChangeReason(
                LaneChangeReason::ROUTE_CHANGE,
                LaneChangeType::TYPE_STALLED_VEHICLE_CHANGE));
  EXPECT_EQ(LaneChangeReason::DEFAULT_CHANGE,
            internal::ReAnalyzeLaneChangeReason(
                LaneChangeReason::ROUTE_CHANGE,
                LaneChangeType::TYPE_CURB_CUTOFF_CHANGE));
  EXPECT_EQ(LaneChangeReason::PROGRESS_CHANGE,
            internal::ReAnalyzeLaneChangeReason(
                LaneChangeReason::PROGRESS_CHANGE,
                LaneChangeType::TYPE_OVERTAKE_CHANGE));
}
}  // namespace

}  // namespace st::planning
