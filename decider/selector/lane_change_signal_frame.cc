#include "decider/selector/lane_change_signal_frame.h"

#include <string>

#include "absl/strings/str_format.h"

#include "plan_common/math/piecewise_bilinear_function.h"
#include "plan_common/math/util.h"

namespace st::planning {

namespace {

constexpr double kLcLeftTwoLaneHighwayFrameFactor = 0.8;
constexpr double kLcRightTwoLaneHighwayFrameFactor = 4.0;
constexpr double kLcRightHighwayFrameFactor = 1.0;

}  // namespace

std::string OvertakeFrameInput::DebugString() const {
  return absl::StrFormat(
      "leader_speed_diff:%.2f, ego_v:%.2f, valid_lane_num: %d, lc_left: %d, "
      "on_highway: %d",
      leader_speed_diff, ego_v, valid_lane_num, lc_left, on_highway);
}

int CalcOvertakeFrame(const OvertakeFrameInput &overtake_begin_frame_input,
                      bool is_conversative_style) {
  const double leader_speed_diff = overtake_begin_frame_input.leader_speed_diff;
  const double ego_v = overtake_begin_frame_input.ego_v;
  // ego_v = 120
  PiecewiseLinearFunction<double, double> kLargeSpeedFramePlf = {
      {Kph2Mps(5.0), Kph2Mps(10.0), Kph2Mps(20.0), Kph2Mps(40.0),
       Kph2Mps(60.0)},
      {40.0, 25.0, 10.0, 8.0, 2.0}};
  // ego_v = 80
  PiecewiseLinearFunction<double, double> kHighSpeedFramePlf = {
      {Kph2Mps(5.0), Kph2Mps(10.0), Kph2Mps(20.0), Kph2Mps(40.0),
       Kph2Mps(60.0)},
      {40.0, 25.0, 10.0, 5.0, 2.0}};
  // ego_v = 40.
  PiecewiseLinearFunction<double, double> kMidSpeedFramePlf = {
      {Kph2Mps(5.0), Kph2Mps(10.0), Kph2Mps(20.0), Kph2Mps(40.0)},
      {30.0, 10.0, 6.0, 2.0}};
  // ego_v = 20.
  PiecewiseLinearFunction<double, double> kLowerSpeedFramePlf = {
      {Kph2Mps(5.0), Kph2Mps(10.0), Kph2Mps(20.0)}, {20.0, 8.0, 5.0}};

  static const PiecewiseBilinearFunction<double, double> kSpeedAndDiffPblf(
      {Kph2Mps(20.0), Kph2Mps(40.0), Kph2Mps(80.0), Kph2Mps(120.0)},
      {kLowerSpeedFramePlf, kMidSpeedFramePlf, kHighSpeedFramePlf,
       kLargeSpeedFramePlf});

  static const PiecewiseLinearFunction<double, double>
      kConservativePorgressFactor({0.0, 2.0, 10.0, 20.0, 40.0},
                                  {0.0, 2.0, 20.0, 50.0, 70.0});

  double radical_overtake_frame = 0.0, conservative_frame = 0.0;
  const bool is_two_lane_highway =
      overtake_begin_frame_input.valid_lane_num == 2 &&
      overtake_begin_frame_input.on_highway;
  if (is_two_lane_highway) {
    radical_overtake_frame =
        kSpeedAndDiffPblf.Evaluate(ego_v, leader_speed_diff) *
        (overtake_begin_frame_input.lc_left
             ? kLcLeftTwoLaneHighwayFrameFactor
             : kLcRightTwoLaneHighwayFrameFactor);
    conservative_frame =
        kConservativePorgressFactor.Evaluate(radical_overtake_frame);
    return is_conversative_style ? CeilToInt(conservative_frame)
                                 : CeilToInt(radical_overtake_frame);
  } else if (overtake_begin_frame_input.on_highway) {
    radical_overtake_frame =
        kSpeedAndDiffPblf.Evaluate(ego_v, leader_speed_diff) *
        (overtake_begin_frame_input.lc_left ? 1.0 : kLcRightHighwayFrameFactor);
    conservative_frame =
        kConservativePorgressFactor.Evaluate(radical_overtake_frame);
    return is_conversative_style ? CeilToInt(conservative_frame)
                                 : CeilToInt(radical_overtake_frame);
  }
  return CeilToInt(kSpeedAndDiffPblf.Evaluate(ego_v, leader_speed_diff));
}

}  // namespace st::planning
