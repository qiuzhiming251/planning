#pragma once

#include <string>

#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_debug.pb.h"

namespace st::planning {

constexpr double kCutOffLaneSpeedDiffRatio = 0.1;
constexpr double kTwoLaneCutOffRatioHighway = 0.15;
struct SpeedLimitCostInput {
  bool is_highway = false;
  double driving_style_factor = 1.0;
  double max_lane_speed = 0.0;              // mps
  double lane_speed_limit = 0.0;            // mps
  double max_init_leader_speed = 0.0;       // mps
  double init_leader_speed = 0.0;           // mps
  double lane_speed_limit_by_leader = 0.0;  // mps
  double ego_v = 0.0;                       // mps
  double dist_to_virtual_lane = 0.0;        // m
  int valid_lane_num = 1;                   // lane num;

  bool is_conversative_style = false;
  std::string DebugString() const;
};

std::string DebugFormat(const SpeedLimitCostOutput& output);

double GetDrivingStyleFactor(int driving_style_gear, bool is_highway);

SpeedLimitCostOutput CalculateSpeedLimitCost(
    const SpeedLimitCostInput& speed_limit_cost_input);
}  // namespace st::planning
