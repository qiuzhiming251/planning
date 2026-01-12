/*
 * This file contains proprietary code owned by xxx.
 * Unauthorized use, distribution, or modification is strictly prohibited.
 *
 * Copyright (c) 2023 xxx. All rights reserved.
 *
 */

#include "decider/selector/cost_feature_util.h"

#include "gtest/gtest.h"

namespace st::planning {
namespace {
TEST(CosteFeatureUtilTest, CanIgnoreCrossSolidBoundary) {
  ASSERT_FALSE(CanIgnoreCrossSolidBoundary(
      LaneChangeType::TYPE_OVERTAKE_CHANGE, /*lc_num=*/1,
      /*length_along_route=*/120.0, false,
      /*highway=*/true, /*in_tunnel=*/false,
      /*lane_keep_dist_to_merge=*/DBL_MAX));

  // TUNNEL
  ASSERT_FALSE(CanIgnoreCrossSolidBoundary(
      LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE, /*lc_num=*/1,
      /*length_along_route=*/120.0, false,
      /*highway=*/true, /*in_tunnel=*/true,
      /*lane_keep_dist_to_merge=*/DBL_MAX));

  ASSERT_TRUE(CanIgnoreCrossSolidBoundary(LaneChangeType::TYPE_EMERGENCY_CHANGE,
                                          /*lc_num=*/1,
                                          /*length_along_route=*/120.0, false,
                                          /*highway=*/true, /*in_tunnel=*/true,
                                          /*lane_keep_dist_to_merge=*/DBL_MAX));

  ASSERT_FALSE(CanIgnoreCrossSolidBoundary(
      LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE, /*lc_num=*/1,
      /*length_along_route=*/120.0, false,
      /*highway=*/true, /*in_tunnel=*/true,
      /*lane_keep_dist_to_merge=*/DBL_MAX));

  ASSERT_FALSE(CanIgnoreCrossSolidBoundary(
      LaneChangeType::TYPE_DEFAULT_ROUTE_CHANGE, /*lc_num=*/1,
      /*length_along_route=*/120.0, false,
      /*highway=*/true, /*in_tunnel=*/true,
      /*lane_keep_dist_to_merge=*/85.0));
}
}  // namespace

}  // namespace st::planning
