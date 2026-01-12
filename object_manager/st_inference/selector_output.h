#ifndef SELECTOR_OUTPUT_H_
#define SELECTOR_OUTPUT_H_

#include <string>

#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_state.pb.h"

namespace st::planning {

struct SelectorOutput {
  int selected_idx = -1;
  int best_traj_idx = -1;
  int last_selected_idx = -1;
  TurnSignal turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  TurnSignalReason turn_signal_reason = TurnSignalReason::TURN_SIGNAL_OFF;
  bool all_trajectories_blocked = false;
  std::optional<bool> is_going_force_route_change_left = std::nullopt;
  bool in_high_way = false;
  bool lane_change_for_obstacle_fail = false;
  std::optional<Vec2d> merge_point = std::nullopt;
  bool need_lane_change_request = false;
  std::string force_lane_keep_info = "";
  bool ego_corner_cross_line = false;
  bool ignore_cross_solid_line = false;
  LcFeasibility best_traj_lc_unable_reason = LcFeasibility::FEASIBILITY_OK;
  bool has_begin_route_change = false;
  bool may_miss_navi = false;
  std::optional<bool> begin_route_change_left = std::nullopt;
};

};  // namespace st::planning

#endif  // SELECTOR_OUTPUT_H_
