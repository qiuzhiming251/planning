#ifndef _PLAN_COMMON_SERIALIZATION_SELECTOR_STATE_SERIALIZATION_H_
#define _PLAN_COMMON_SERIALIZATION_SELECTOR_STATE_SERIALIZATION_H_

#include "plan_common/selector_state.h"
#include "plan_common/serialization/base_serialization.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_state.pb.h"

#include <cereal/types/deque.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/unordered_map.hpp>

namespace cereal {
template <typename Archive>
void serialize(Archive& ar, st::planning::LastLcInfoProto& last_lc_info) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    last_lc_info.ParseFromString(serialized_proto);
  } else {
    last_lc_info.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::TargetLaneStateProto& target_lane_state) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    target_lane_state.ParseFromString(serialized_proto);
  } else {
    target_lane_state.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(
    Archive& ar,
    st::planning::SelectorLaneChangeRequestProto& selector_lanechange_request) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    selector_lanechange_request.ParseFromString(serialized_proto);
  } else {
    selector_lanechange_request.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::RouteTtcSettingProto& route_ttc_setting) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    route_ttc_setting.ParseFromString(serialized_proto);
  } else {
    route_ttc_setting.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::PrefilterStateProto& prefilter_state) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    prefilter_state.ParseFromString(serialized_proto);
  } else {
    prefilter_state.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::planning::FollowCostInfo& follow_cost_info) {
  ar(CEREAL_NVP(follow_cost_info.obj_id));
  ar(CEREAL_NVP(follow_cost_info.obj_type));
  ar(CEREAL_NVP(follow_cost_info.speed_follow_cost));
  ar(CEREAL_NVP(follow_cost_info.final_follow_cost));
  ar(CEREAL_NVP(follow_cost_info.integration));
  ar(CEREAL_NVP(follow_cost_info.ts));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::SelectorState& selector_state) {
  ar(CEREAL_NVP(selector_state.history_best_target_lane_states));
  ar(CEREAL_NVP(selector_state.selected_target_lane_state));
  ar(CEREAL_NVP(selector_state.pre_turn_signal));
  ar(CEREAL_NVP(selector_state.last_redlight_stop_time));
  ar(CEREAL_NVP(selector_state.lane_change_reason));
  ar(CEREAL_NVP(selector_state.last_lc_info));
  ar(CEREAL_NVP(selector_state.selector_lane_change_request));
  ar(CEREAL_NVP(selector_state.last_user_reject_alc_time));
  ar(CEREAL_NVP(selector_state.last_user_reject_alc_reason));
  ar(CEREAL_NVP(selector_state.lane_change_prepare_state));
  ar(CEREAL_NVP(selector_state.lc_unable_reason));
  ar(CEREAL_NVP(selector_state.last_lane_change_reason));
  ar(CEREAL_NVP(selector_state.pre_lane_change_state));
  ar(CEREAL_NVP(selector_state.prev_lc_stage));
  ar(CEREAL_NVP(selector_state.gap_front_id));
  ar(CEREAL_NVP(selector_state.gap_back_id));
  ar(CEREAL_NVP(selector_state.is_deviate_navi));
  ar(CEREAL_NVP(selector_state.best_target_lane_state));
  ar(CEREAL_NVP(selector_state.lane_change_type));
  ar(CEREAL_NVP(selector_state.last_turn_signal));
  ar(CEREAL_NVP(selector_state.last_turn_signal_reason));
  ar(CEREAL_NVP(selector_state.last_user_reject_alc_type));
  ar(CEREAL_NVP(selector_state.route_ttc_setting));
  ar(CEREAL_NVP(selector_state.lc_prepare_stage_lane_path));
  ar(CEREAL_NVP(selector_state.activate_selector_time));
  ar(CEREAL_NVP(selector_state.start_lane_change_time));
  ar(CEREAL_NVP(selector_state.give_up_lane_change_time));
  ar(CEREAL_NVP(selector_state.lane_change_general_type));
  ar(CEREAL_NVP(selector_state.prefilter_state));
  ar(CEREAL_NVP(selector_state.has_selected_intersection));
  ar(CEREAL_NVP(selector_state.junction_out_lane));
  ar(CEREAL_NVP(selector_state.intention_dir));
}
}  // namespace cereal

namespace st::planning {
template <typename Archive>
void serialize(Archive& ar, SelectorCostHistory& selector_cost_history) {
  ar(CEREAL_NVP(selector_cost_history.cur_time_));
  ar(CEREAL_NVP(selector_cost_history.path_object_costs_));
  ar(CEREAL_NVP(selector_cost_history.decay_factor_));
  ar(CEREAL_NVP(selector_cost_history.expire_cost_threshold_));
}
}  // namespace st::planning

#endif