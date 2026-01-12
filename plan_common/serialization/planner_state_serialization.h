#ifndef _PLAN_COMMON_SERIALIZATION_PLANNER_STATE_SERIALIZATION_H_
#define _PLAN_COMMON_SERIALIZATION_PLANNER_STATE_SERIALIZATION_H_

#include "node_manager/task_runner/planner_state.h"
#include "node_manager/task_runner/decision.h"
#include "node_manager/task_runner/decision_supplement.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
#include "plan_common/ego_history.h"
#include "plan_common/type_def.h"
#include "object_manager/object_history.h"

#include "plan_common/serialization/base_serialization.h"
#include "plan_common/serialization/map_serialization.h"
#include "plan_common/serialization/selector_state_serialization.h"

#include <cereal/types/map.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/tuple.hpp>

namespace cereal {

template <typename Archive>
void serialize(Archive& ar, st::planning::PlannerState::PosePoint& pose_point) {
  ar(CEREAL_NVP(pose_point.pos));
  ar(CEREAL_NVP(pose_point.theta));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::SmoothedReferenceCenterResult&
                                smoothed_refline_center_result) {
  ar(CEREAL_NVP(
      smoothed_refline_center_result.lane_id_to_smoothed_lateral_offset));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::SpeedState& speedstate) {
  ar(CEREAL_NVP(speedstate.attention_obj_id));
  ar(CEREAL_NVP(speedstate.lcc_keep_brake));
  ar(CEREAL_NVP(speedstate.yield_to_vru));
  ar(CEREAL_NVP(speedstate.vru_interact_timer));
  ar(CEREAL_NVP(speedstate.infront_vru_ids));
  ar(CEREAL_NVP(speedstate.pre_fast_speed_limit));
  ar(CEREAL_NVP(speedstate.pre_lc_num));
  ar(CEREAL_NVP(speedstate.lc_num_has_up));
  ar(CEREAL_NVP(speedstate.pre_dynamic_acc_limit));
  ar(CEREAL_NVP(speedstate.pre_special_acc_gap_secnario));
  ar(CEREAL_NVP(speedstate.first_has_speed_limit));
}

template <typename Archive>
void serialize(
    Archive& ar,
    st::planning::PlannerState::ObjectStopTimeResult& object_stop_time_result) {
  ar(CEREAL_NVP(object_stop_time_result.time_duration_since_stop));
  ar(CEREAL_NVP(object_stop_time_result.previous_stop_time_duration));
  ar(CEREAL_NVP(object_stop_time_result.last_move_time_duration));
  ar(CEREAL_NVP(object_stop_time_result.last_time));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::NudgeObjectInfo& nudge_object_info) {
  ar(CEREAL_NVP(nudge_object_info.id));
  ar(CEREAL_NVP(nudge_object_info.direction));
  ar(CEREAL_NVP(nudge_object_info.arc_dist_to_object));
  ar(CEREAL_NVP(nudge_object_info.type));
  ar(CEREAL_NVP(nudge_object_info.nudge_state));
}

// template <class Archive>
// void serialize(Archive& ar, st::planning::LaneChangeSupplement&
// lanechange_supplement) {
//   ar(CEREAL_NVP(lanechange_supplement.last_lc_finish_time));
// }

// CEREAL_REGISTER_TYPE(st::planning::LaneChangeSupplement)
// CEREAL_REGISTER_POLYMORPHIC_RELATION(st::planning::DecisionSupplement,
// st::planning::LaneChangeSupplement)

// template <typename Archive>
// void serialize(Archive& ar, st::planning::Decision& decision_)
// {
//   ar(CEREAL_NVP(decision_.lane_follow_keep_time_));
//   // ar(CEREAL_NVP(decision_.supplements_map_));
// }

template <typename Archive>
void serialize(Archive& ar, st::planning::PlannerState& planner_state) {
  ar(CEREAL_NVP(planner_state.planner_status_code));
  ar(CEREAL_NVP(planner_state.prev_est_planner_status_code));
  ar(CEREAL_NVP(planner_state.previous_trajectory));
  ar(CEREAL_NVP(planner_state.prev_ego_pose_frenet));
  ar(CEREAL_NVP(planner_state.lane_change_state));
  ar(CEREAL_NVP(planner_state.tl_stop_interface));
  ar(CEREAL_NVP(planner_state.previous_autonomy_state));
  ar(CEREAL_NVP(planner_state.version));
  ar(CEREAL_NVP(planner_state.prev_lane_path_before_lc));
  ar(CEREAL_NVP(planner_state.decider_state));
  ar(CEREAL_NVP(planner_state.prev_map_type));
  ar(CEREAL_NVP(planner_state.prev_target_lane_path));
  ar(CEREAL_NVP(planner_state.prev_length_along_route));
  ar(CEREAL_NVP(planner_state.prev_max_reach_length));
  ar(CEREAL_NVP(planner_state.station_anchor));
  ar(CEREAL_NVP(planner_state.preferred_lane_path));
  ar(CEREAL_NVP(planner_state.prev_smooth_state));
  ar(CEREAL_NVP(planner_state.selector_state));
  ar(CEREAL_NVP(planner_state.is_going_force_route_change_left));
  ar(CEREAL_NVP(planner_state.selected_trajectory_optimizer_state_proto));
  ar(CEREAL_NVP(planner_state.smooth_result_map));
  // ar(CEREAL_NVP(planner_state.display_error_integrator));
  ar(CEREAL_NVP(planner_state.object_history_manager));
  ar(CEREAL_NVP(planner_state.ego_history));
  ar(CEREAL_NVP(planner_state.stalled_cars));
  ar(CEREAL_NVP(planner_state.in_queue_cars));
  ar(CEREAL_NVP(planner_state.alc_state));
  ar(CEREAL_NVP(planner_state.acc_state));
  ar(CEREAL_NVP(planner_state.lane_change_style));
  ar(CEREAL_NVP(planner_state.lane_change_command));
  ar(CEREAL_NVP(planner_state.plc_prepare_start_time));
  ar(CEREAL_NVP(planner_state.turn_signal_result));
  ar(CEREAL_NVP(planner_state.last_lc_command));
  ar(CEREAL_NVP(planner_state.last_lc_reason));
  ar(CEREAL_NVP(planner_state.planner_semantic_map_manager));
  ar(CEREAL_NVP(planner_state.lc_command_number));
  ar(CEREAL_NVP(planner_state.lc_command_number));
  ar(CEREAL_NVP(planner_state.lc_cancel_delay_number));
  ar(CEREAL_NVP(planner_state.lc_keeping_delay_number));
  ar(CEREAL_NVP(planner_state.last_manual_lc_time));
  ar(CEREAL_NVP(planner_state.speed_state));
  ar(CEREAL_NVP(planner_state.spdlimit_curvature_gain_prev));
  ar(CEREAL_NVP(planner_state.object_stop_time_map));
  ar(CEREAL_NVP(planner_state.nudge_object_info));
  ar(CEREAL_NVP(planner_state.has_triggered_lc_notice));
  ar(CEREAL_NVP(planner_state.lc_notice_pub_counter));
  ar(CEREAL_NVP(planner_state.lc_notice));
  ar(CEREAL_NVP(planner_state.if_continuous_lc));
  ar(CEREAL_NVP(planner_state.lc_push_dir));
  ar(CEREAL_NVP(planner_state.last_lc_style));
  ar(CEREAL_NVP(planner_state.lc_lead_obj_ids));

  // ar(CEREAL_NVP(planner_state.decision));
  ar(CEREAL_NVP(planner_state.pre_large_vehicle_avoid_state));
  ar(CEREAL_NVP(planner_state.pre_push_status));
  ar(CEREAL_NVP(planner_state.saved_offset));
  ar(CEREAL_NVP(planner_state.has_begin_route_change));
  ar(CEREAL_NVP(planner_state.lc_style_decider_results));
  ar(CEREAL_NVP(planner_state.task_safety_evaluation_results));
  ar(CEREAL_NVP(planner_state.scene_cones_riding_line_frames_results));
  ar(CEREAL_NVP(planner_state.begin_route_change_left));
  ar(CEREAL_NVP(planner_state.last_gaming_result));
}

}  // namespace cereal

namespace st::planning {
template <typename Archive>
void serialize(
    Archive& ar,
    st::planning::SmoothedReferenceLineResultMap& smoothed_refline_result_map) {
  ar(CEREAL_NVP(smoothed_refline_result_map.smoothed_result_map_));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::ObjectFrame& objectFrame) {
  ar(CEREAL_NVP(objectFrame.id));
  ar(CEREAL_NVP(objectFrame.timestamp));
  ar(CEREAL_NVP(objectFrame.object_proto));
  ar(CEREAL_NVP(objectFrame.is_leading));
  ar(CEREAL_NVP(objectFrame.is_stalled));
  ar(CEREAL_NVP(objectFrame.is_nudge));
  ar(CEREAL_NVP(objectFrame.lon_decision));
  ar(CEREAL_NVP(objectFrame.lat_decision));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::ObjectHistory& objectHistory) {
  ar(CEREAL_NVP(objectHistory.frames_));
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::ObjectHistoryManager& object_history_manager) {
  ar(CEREAL_NVP(object_history_manager.obj_history_map_));
  ar(CEREAL_NVP(object_history_manager.kMaxHistoryLengthUs));
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::V2SpeedLimitInfo& v2_speed_limit_info) {
  ar(CEREAL_NVP(v2_speed_limit_info.is_generate_small_speed_limit));
  ar(CEREAL_NVP(v2_speed_limit_info.is_close_curb));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::EgoFrame& ego_frame) {
  ar(CEREAL_NVP(ego_frame.v2_speed_limit_info));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::EgoHistory& ego_history) {
  ar(CEREAL_NVP(ego_history.frames_));
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::TurnSignalResult& turn_signal_result) {
  ar(CEREAL_NVP(turn_signal_result.signal));
  ar(CEREAL_NVP(turn_signal_result.reason));
}

// // abseil flat hash map
// template <typename Archive, typename Key, typename Value, typename Hash,
//           typename Eq, typename Allocator>
// void serialize(Archive& ar,
//                absl::flat_hash_map<Key, Value, Hash, Eq, Allocator>& map) {
//   size_t size = map.size();
//   // ar(make_size_tag(size));
//   if (ar.is_loading()) {
//     map.clear();
//     map.reserve(size);
//     for (size_t i = 0; i < size; ++i) {
//       Key key;
//       Value value;
//       ar(key, value);
//       map.emplace(std::move(key), std::move(value));
//     }
//   } else {
//     for (auto& [key, value] : map) {
//       ar(key, value);
//     }
//   }
// }
}  // namespace st::planning

#endif