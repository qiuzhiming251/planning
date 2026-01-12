#include "decider/selector/ensure_lc_feasibility.h"

#include "gtest/gtest.h"
#include "decider/selector/cost_feature_util.h"
namespace st::planning {

namespace {
TEST(EnsureLcFeasibiltyTest, CanOvertakeLaneChange) {
  // Obvious route lane change.
  TrajFeatureOutput traj_feature_output = {
      .has_obvious_route_cost = false,
      .is_perform_lane_change = false,
      .lane_change_left = true,
      .lane_change_for_route_cost = true,
  };

  bool on_highway = false;
  bool is_in_tunnel = false;
  RoadHorizonInfo road_horizon = {
      .dist_to_fork = 100.0,
      .dist_to_merge = 100.0,
      .dist_to_cross = 60.0,
  };
  LaneChangeType lc_type = LaneChangeType::TYPE_OVERTAKE_CHANGE;
  double dist_to_solid_line = 40.0;
  {
    ASSERT_EQ(
        CanOvertakeLaneChange(traj_feature_output, {}, on_highway, is_in_tunnel,
                              road_horizon, lc_type, dist_to_solid_line),
        LcFeasibility::FEASIBILITY_OK);

    traj_feature_output.has_obvious_route_cost = true;
    ASSERT_EQ(
        CanOvertakeLaneChange(traj_feature_output, {}, on_highway, is_in_tunnel,
                              road_horizon, lc_type, dist_to_solid_line),
        LcFeasibility::FEASIBILITY_OBVIOUSE_ROUTE);

    TrajFeatureOutput lane_keep_feature_output = {
        .has_obvious_route_cost = false,
        .is_perform_lane_change = false,
        .lane_change_left = false,
        .lane_change_for_route_cost = false,
        .is_accel_traj_start = true,
    };

    traj_feature_output.has_obvious_route_cost = false;
    ASSERT_EQ(CanOvertakeLaneChange(
                  traj_feature_output, lane_keep_feature_output, on_highway,
                  is_in_tunnel, road_horizon, lc_type, dist_to_solid_line),
              LcFeasibility::FEASIBILITY_AVOID_OVERTAKE_IF_ACCEL);
  }
  {
    TrajFeatureOutput lane_keep_feature_output = {
        .opposite_lc_interval_secs = 0.0,
    };
    traj_feature_output.opposite_lc_interval_secs = 5.0;
    traj_feature_output.lane_change_left = true;
    ASSERT_EQ(CanOvertakeLaneChange(
                  traj_feature_output, lane_keep_feature_output, on_highway,
                  is_in_tunnel, road_horizon, lc_type, dist_to_solid_line),
              LcFeasibility::FEASIBILITY_OPPOSITE);
    traj_feature_output.opposite_lc_interval_secs = 10.0;
    ASSERT_EQ(CanOvertakeLaneChange(
                  traj_feature_output, lane_keep_feature_output, on_highway,
                  is_in_tunnel, road_horizon, lc_type, dist_to_solid_line),
              LcFeasibility::FEASIBILITY_OK);
  }
}

TEST(EnsureLcFeasibiltyTest, DisallowOvertakeNearRampOrCross) {
  {
    bool on_highway = false;
    double dist_to_ramp = 100.0;
    double dist_to_cross = 40.0;
    double dist_to_solid_line = 20.0;

    ASSERT_TRUE(DisallowOvertakeNearRampOrCross(
        /*on_highway=*/true, /*dist_to_ramp=*/300.0, dist_to_cross,
        dist_to_solid_line));
    ASSERT_FALSE(DisallowOvertakeNearRampOrCross(
        /*on_highway=*/true, /*dist_to_ramp=*/1200.0, dist_to_cross,
        dist_to_solid_line));
    dist_to_cross = 80.0;
    dist_to_solid_line = 50.0;

    ASSERT_FALSE(DisallowOvertakeNearRampOrCross(
        /*on_highway=*/false, dist_to_ramp, dist_to_cross, dist_to_solid_line));

    ASSERT_FALSE(DisallowOvertakeNearRampOrCross(
        /*on_highway=*/false, dist_to_ramp, dist_to_cross, dist_to_solid_line));
    ASSERT_TRUE(DisallowOvertakeNearRampOrCross(
        /*on_highway=*/false, dist_to_ramp, dist_to_cross,
        /*dist_to_solid_line*/ 20.0));
  }
}
}  // namespace

}  // namespace st::planning