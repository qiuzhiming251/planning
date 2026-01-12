

#include "absl/time/time.h"
#include "selector_state.h"
#include "plan_common/util/time_util.h"

namespace st {
namespace planning {

void SelectorState::FromProto(const SelectorStateProto& proto) {
  history_best_target_lane_states.clear();
  for (const auto& lane_state_proto : proto.history_best_target_lane_states()) {
    history_best_target_lane_states.push_back(lane_state_proto);
  }
  selected_target_lane_state = proto.selected_target_lane_state();
  pre_turn_signal = proto.pre_turn_signal();

  if (proto.has_last_redlight_stop_time()) {
    last_redlight_stop_time = st::FromProto(proto.last_redlight_stop_time());
  } else {
    last_redlight_stop_time = std::nullopt;
  }

  lane_change_reason = proto.lane_change_reason();
  last_lc_info = proto.last_lc_info();
  last_passed_split_info = proto.last_passed_split_info();
  selector_lane_change_request = proto.selector_lane_change_request();

  if (proto.has_last_user_reject_alc_time()) {
    last_user_reject_alc_time =
        st::FromProto(proto.last_user_reject_alc_time());
  } else {
    last_user_reject_alc_time = std::nullopt;
  }
  last_user_reject_alc_reason = proto.last_user_reject_alc_reason();
  lane_change_prepare_state = proto.lane_change_prepare_state();
  lc_unable_reason = proto.lc_unable_reason();
  last_lane_change_reason = proto.last_lane_change_reason();
  pre_lane_change_state = proto.pre_lane_change_state();
}

void SelectorState::ToProto(SelectorStateProto* proto) const {
  proto->Clear();
  for (const auto& lane_state : history_best_target_lane_states) {
    *proto->add_history_best_target_lane_states() = lane_state;
  }
  *proto->mutable_selected_target_lane_state() = selected_target_lane_state;
  proto->set_pre_turn_signal(pre_turn_signal);
  if (last_redlight_stop_time.has_value()) {
    st::ToProto(*last_redlight_stop_time,
                proto->mutable_last_redlight_stop_time());
  }
  proto->set_lane_change_reason(lane_change_reason);
  *proto->mutable_last_lc_info() = last_lc_info;
  *proto->mutable_last_passed_split_info() = last_passed_split_info;
  *proto->mutable_selector_lane_change_request() = selector_lane_change_request;
  if (last_user_reject_alc_time.has_value()) {
    st::ToProto(*last_user_reject_alc_time,
                proto->mutable_last_user_reject_alc_time());
  }
  proto->set_last_user_reject_alc_reason(last_user_reject_alc_reason);
  proto->set_lane_change_prepare_state(lane_change_prepare_state);
  proto->set_lc_unable_reason(lc_unable_reason);
  proto->set_last_lane_change_reason(last_lane_change_reason);
  proto->set_pre_lane_change_state(pre_lane_change_state);
}

}  // namespace planning
}  // namespace st
