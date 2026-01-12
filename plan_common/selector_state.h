

#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_STATE_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_STATE_H_

#include <deque>
#include <vector>

#include "absl/time/time.h"
#include "boost/circular_buffer.hpp"

#include "plan_common/maps/lane_path.h"
#include "selector_cost_history.h"
#include "modules/cnoa_pnc/planning/proto/hmi_content.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change_type.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"
namespace st {
namespace planning {

struct RouteIntentionInfo {
  std::optional<bool> route_change_left = std::nullopt;
  int successive_count = 0;
  void Reset() {
    route_change_left = std::nullopt;
    successive_count = 0;
  }
};

struct SelectorState {
  void FromProto(const SelectorStateProto& proto);
  void ToProto(SelectorStateProto* proto) const;

  std::deque<TargetLaneStateProto> history_best_target_lane_states;
  TargetLaneStateProto selected_target_lane_state;
  TurnSignal pre_turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  std::optional<absl::Time> last_redlight_stop_time = std::nullopt;
  LaneChangeReason lane_change_reason = LaneChangeReason::NO_CHANGE;
  LastLcInfoProto last_lc_info;
  LastPassedSplitInfoProto last_passed_split_info;
  SelectorLaneChangeRequestProto selector_lane_change_request;
  std::optional<absl::Time> last_user_reject_alc_time = std::nullopt;
  LaneChangeReason last_user_reject_alc_reason = LaneChangeReason::NO_CHANGE;
  LaneChangePrepareState lane_change_prepare_state =
      LaneChangePrepareState::Lane_Keeping;
  LcFeasibility lc_unable_reason = LcFeasibility::FEASIBILITY_OK;
  LaneChangeReason last_lane_change_reason = LaneChangeReason::NO_CHANGE;
  LaneChangeState pre_lane_change_state = LaneChangeState::Lc_Keeping;
  st::LaneChangeStage prev_lc_stage = st::LaneChangeStage::LCS_NONE;
  std::string gap_front_id = "";
  std::string gap_back_id = "";
  bool is_deviate_navi = false;

  int begin_route_change_successive_count = 0;
  bool is_passed_split_lane_change = false;

  RouteIntentionInfo begin_route_change_left_info;
  RouteIntentionInfo force_route_change_left_info;

  TargetLaneStateProto best_target_lane_state;
  LaneChangeType lane_change_type = LaneChangeType::TYPE_NO_CHANGE;
  TurnSignal last_turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  TurnSignalReason last_turn_signal_reason = TurnSignalReason::TURN_SIGNAL_OFF;
  LaneChangeType last_user_reject_alc_type = LaneChangeType::TYPE_NO_CHANGE;
  RouteTtcSettingProto route_ttc_setting;
  std::optional<mapping::LanePath> lc_prepare_stage_lane_path = std::nullopt;
  std::optional<absl::Time> activate_selector_time = std::nullopt;
  std::optional<absl::Time> start_lane_change_time = std::nullopt;
  std::optional<absl::Time> give_up_lane_change_time = std::nullopt;
  LaneChangeGeneralType lane_change_general_type =
      LaneChangeGeneralType::LCGT_NO_CHANGE;
  PrefilterStateProto prefilter_state;
  bool has_selected_intersection = false;
  ad_byd::planning::LaneConstPtr junction_out_lane = nullptr;
  ad_byd::planning::BehaviorCommand intention_dir =
      ad_byd::planning::BehaviorCommand::Command_Invalid;
  // HistoryBufferAbslTime<PrefilterHistoryInfo> prefilter_history_infos;
  SelectorCostHistory object_cost_history;
  int overtake_lc_pause_successive_count = 0;
  int force_route_change_successive_count = 0;
  std::optional<absl::Time> time_ego_in_ramp = std::nullopt;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_STATE_H
