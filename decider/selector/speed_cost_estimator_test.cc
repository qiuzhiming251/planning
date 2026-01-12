#include "decider/selector/speed_cost_estimator.h"

#include "gtest/gtest.h"
#include "decider/selector/cost_feature_util.h"
#include "decider/selector/lane_change_signal_frame.h"

namespace st::planning {

namespace {
TEST(SpeedCostEstimatorTest, CalcSpeedLimitCost) {
  {
    auto speed_limit_cost_output = CalculateSpeedLimitCost(SpeedLimitCostInput{
        .is_highway = false,
        .driving_style_factor = 1.0,
        .max_lane_speed = 15.0,
        .lane_speed_limit = 15.0,
        .max_init_leader_speed = 15.0,
        .init_leader_speed = 10.21,
        .lane_speed_limit_by_leader = 9.5,
        .ego_v = 11.0,
        .dist_to_virtual_lane = 300.0,
    });
    EXPECT_GE(speed_limit_cost_output.slow_av_factor(), 0.0);
    EXPECT_GT(speed_limit_cost_output.speed_limit_cost(), 0.0);
    EXPECT_NEAR(speed_limit_cost_output.driving_style_factor(), 1.0, 0.1);
  }

  {
    auto speed_limit_cost_output = CalculateSpeedLimitCost(SpeedLimitCostInput{
        .is_highway = false,
        .driving_style_factor = 3.0,
        .max_lane_speed = 8.17,
        .lane_speed_limit = 8.17,
        .max_init_leader_speed = 8.0,
        .init_leader_speed = 0.09,
        .lane_speed_limit_by_leader = 1.74,
        .ego_v = 7.4,
        .dist_to_virtual_lane = 69.6,
    });
    EXPECT_NEAR(speed_limit_cost_output.slow_av_factor(), 1.0, 0.1);
    EXPECT_NEAR(speed_limit_cost_output.driving_style_factor(), 3.0, 0.1);
    EXPECT_NEAR(speed_limit_cost_output.slow_leader_factor(), 1.0, 0.1);
    EXPECT_NEAR(speed_limit_cost_output.lane_speed_diff_ratio(), 1.76, 0.1);
  }
}

TEST(SpeedCostEstimatorTest, SpeedLimitCostInput) {
  auto input = SpeedLimitCostInput{
      .is_highway = true,
      .driving_style_factor = 1.0,
      .max_lane_speed = 2.0,
      .lane_speed_limit = 3.0,
      .max_init_leader_speed = 4.0,
      .init_leader_speed = 5.0,
      .lane_speed_limit_by_leader = 6.0,
      .ego_v = 7.0,
      .dist_to_virtual_lane = 8.0,
  };
  ASSERT_TRUE(input.DebugString().find("ego_v") != std::string::npos);
  // for (double v = 10.0; v <= 120.0; v += 5.0) {
  //   std::cout << "v=" << v;
  //   for (double diff = 5.0; diff <= 60.0; diff += 5.0)
  //     std::cout << ","
  //               << CalcOvertakeFrame({
  //                      .leader_speed_diff = Kph2Mps(diff),
  //                      .ego_v = Kph2Mps(v),
  //                      .on_highway = true,
  //                  });
  //   std::cout << std::endl;
  //}
}

}  // namespace st::planning
