

#include "node_manager/task_runner/planner_state.h"

#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "plan_common/container/strong_int.h"
#include "google/protobuf/message.h"
#include "plan_common/math/geometry/util.h"
#include "modules/cnoa_pnc/planning/proto/acc.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/smooth_reference_line.pb.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"
#include "plan_common/util/proto_util.h"
#include "plan_common/util/time_util.h"

namespace st::planning {

void PlannerState::Clear() {
  prev_target_lane_path = mapping::LanePath();
  station_anchor = mapping::LanePoint();
  prev_length_along_route = 0.0;
  prev_max_reach_length = 0.0;
  prev_smooth_state = false;
  decider_state.Clear();
  initializer_state.Clear();
  speed_finder_state.Clear();
  lane_change_state.Clear();
  prev_lane_path_before_lc = mapping::LanePath();
  st_planner_object_trajectories.Clear();
  // prev_traj_end_info = std::nullopt;
  // previously_triggered_aeb = false;
  nudge_object_info = std::nullopt;
  lc_lead_obj_ids.clear();
  is_going_force_route_change_left = std::nullopt;
  spdlimit_curvature_gain_prev = 1.0;
  acc_state = AccState::ACC_OFF;
}

void PlannerState::ClearforAccEnterandExit() {
  prev_target_lane_path = mapping::LanePath();
  station_anchor = mapping::LanePoint();
  prev_length_along_route = 0.0;
  prev_max_reach_length = 0.0;
  prev_smooth_state = false;
  decider_state.Clear();
  initializer_state.Clear();
  speed_finder_state.Clear();
  lane_change_state.Clear();
  selector_state = SelectorState();
  preferred_lane_path = mapping::LanePath();
  prev_lane_path_before_lc = mapping::LanePath();
  st_planner_object_trajectories.Clear();
  lane_change_command = DriverAction::LC_CMD_NONE;
  last_lc_command = DriverAction::LC_CMD_NONE;
  // prev_traj_end_info = std::nullopt;
  // previously_triggered_aeb = false;
  nudge_object_info = std::nullopt;
  prev_ego_pose_frenet = std::nullopt;
  lc_lead_obj_ids.clear();
  is_going_force_route_change_left = std::nullopt;
  spdlimit_curvature_gain_prev = 1.0;
  alc_state = ALCState::ALC_OFF;
  acc_state = AccState::ACC_OFF;
  lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  acc_path_corridor_type = AccPathCorridorType::NO_MAP;
  turn_signal_result.signal = TurnSignal::TURN_SIGNAL_NONE;
  speed_state.Reset();
  if_continuous_lc = false;
  lc_lead_obj_ids.clear();
  selected_trajectory_optimizer_state_proto = std::nullopt;
  pre_large_vehicle_avoid_state.Clear();
  smooth_result_map.Clear();
  begin_route_change_left = std::nullopt;
  truncated_back_traj_horizon.clear();
  has_begin_route_change = false;
  lc_style_decider_results.clear();
  task_safety_evaluation_results.clear();
  scene_cones_riding_line_frames_results.clear();
  last_gaming_result.Clear();
  saved_lane_width = 0.0;
  lane_width_invalid_cnt = 0;
  saved_offset.Clear();
  pre_push_status.Clear();
  lc_push_dir = ad_byd::planning::PushDirection::Push_None;
  pre_large_vehicle_avoid_state.Clear();
  decision.Reset();
  lc_lead_obj_ids.clear();
  speed_state.Reset();
  object_history_manager.Clear();
  display_error_integrator = 0.0;
  prev_success_map_type = ad_byd::planning::MapType::UNKNOWN_MAP;
}

// void PlannerState::ClearTrajectories() {
//   previous_trajectory_global.clear();
//   previous_past_trajectory_global.clear();
// }

void PlannerState::ClearHMIInfo() {
  stalled_cars.clear();
  in_queue_cars.clear();
}

void PlannerState::FromProto(const PlannerStateProto& proto) {
  header = proto.header();

  // planner_frame_seq_num = proto.planner_frame_seq_num();

  // for (const auto& e : proto.yellow_light_observations()) {
  //   yellow_light_observations[mapping::ElementId(e.first)].FromProto(e.second);
  // }

  // previous_trajectory_global.clear();
  // for (const auto& e : proto.previous_trajectory_global()) {
  //   previous_trajectory_global.push_back(
  //       {.pos = Vec2dFromProto(e.pos()), .theta = e.theta()});
  // }
  previous_trajectory = proto.previous_trajectory();

  // previous_past_trajectory_global.clear();
  // for (const auto& e : proto.previous_past_trajectory_global()) {
  //   previous_past_trajectory_global.push_back(
  //       {.pos = Vec2dFromProto(e.pos()), .theta = e.theta()});
  // }

  // previous_st_path_global_including_past.clear();
  // for (const auto& e : proto.previous_st_path_global_including_past()) {
  //   previous_st_path_global_including_past.push_back(e);
  // }

  // last_audio_alert_time = st::FromProto(proto.last_audio_alert_time());

  // parking_brake_release_time =
  //     st::FromProto(proto.parking_brake_release_time());

  lane_change_state = proto.lane_change_state();

  // planner_skip_counter = proto.planner_skip_counter();

  // input_seq_num = proto.input_seq_num();

  previous_autonomy_state = proto.previous_autonomy_state();

  // The number of consecutive uses of the previous frame's trajectory
  // previous_trajectory_plan_counter =
  // proto.previous_trajectory_plan_counter();

  version = proto.version();

  // last_door_override_time = proto.last_door_override_time();

  decider_state = proto.decider_state();
  initializer_state = proto.initializer_state();
  speed_finder_state = proto.speed_finder_state();

  if (proto.has_selected_trajectory_optimizer_state()) {
    selected_trajectory_optimizer_state_proto =
        proto.selected_trajectory_optimizer_state();
  } else {
    selected_trajectory_optimizer_state_proto = std::nullopt;
  }

  st_planner_object_trajectories = proto.st_planner_object_trajectories();
  // current_time = absl::FromUnixMicros(proto.current_time());

  // route_update_id = proto.route_update_id();

  // Freespace planner state.
  // freespace_planner_state = proto.freespace_planner_state();

  // mission_stage = proto.mission_stage();

  // Parallel planner state.
  // prev_target_lane_path restore later because it depends semantic map.
  // After map patch, it can restore here.
  prev_length_along_route = proto.prev_length_along_route();
  prev_max_reach_length = proto.prev_max_reach_length();
  station_anchor.FromProto(proto.station_anchor());

  // prev_route_sections =
  //     RouteSections::BuildFromProto(proto.prev_route_sections());

  // Reference line smooth.
  prev_smooth_state = proto.prev_smooth_state();

  // Plan task queue
  // for (const auto& task_proto : proto.plan_task_queue()) {
  //   plan_task_queue.emplace_back(task_proto);
  // }

  // Previous trajectory end info.
  // if (proto.has_previous_trajectory_end_info()) {
  //   prev_traj_end_info = proto.previous_trajectory_end_info();
  // } else {
  //   prev_traj_end_info = std::nullopt;
  // }

  // stopped_at_route_end = proto.stopped_at_route_end();

  // assist_plan_state = proto.assist_plan_state();

  selector_state.FromProto(proto.selector_state());

  // async_planner_state.FromProto(proto.async_planner_state());

  // previously_triggered_aeb = proto.previously_triggered_aeb();

  // if (proto.has_prev_online_map_id()) {
  //   prev_online_map_id = proto.prev_online_map_id();
  // }
  spdlimit_curvature_gain_prev = proto.spdlimit_curvature_gain_prev();
}

// If FromProto/ToProto slow, use move instead
void PlannerState::ToProto(PlannerStateProto* proto) const {
  // ("PlannerState::ToProto");

  proto->Clear();

  *proto->mutable_header() = header;

  // proto->set_planner_frame_seq_num(planner_frame_seq_num);

  // for (const auto& e : yellow_light_observations) {
  //   e.second.ToProto(&(*proto->mutable_yellow_light_observations())[e.first]);
  // }

  // for (const auto& e : previous_trajectory_global) {
  //   auto* point = proto->add_previous_trajectory_global();
  //   Vec2dToProto(e.pos, point->mutable_pos());
  //   point->set_theta(e.theta);
  // }
  *proto->mutable_previous_trajectory() = previous_trajectory;

  // for (const auto& e : previous_past_trajectory_global) {
  //   auto* point = proto->add_previous_past_trajectory_global();
  //   Vec2dToProto(e.pos, point->mutable_pos());
  //   point->set_theta(e.theta);
  // }

  // for (const auto& e : previous_st_path_global_including_past) {
  //   auto* point = proto->add_previous_st_path_global_including_past();
  //   *point = e;
  // }

  // st::ToProto(last_audio_alert_time, proto->mutable_last_audio_alert_time());

  // if (parking_brake_release_time < absl::UnixEpoch()) {
  //   st::ToProto(absl::UnixEpoch(),
  //   proto->mutable_parking_brake_release_time());
  // } else {
  //   st::ToProto(parking_brake_release_time,
  //               proto->mutable_parking_brake_release_time());
  // }

  *proto->mutable_lane_change_state() = lane_change_state;

  // proto->set_planner_skip_counter(planner_skip_counter);

  // *proto->mutable_input_seq_num() = input_seq_num;

  *proto->mutable_previous_autonomy_state() = previous_autonomy_state;

  // proto->set_previous_trajectory_plan_counter(previous_trajectory_plan_counter);

  proto->set_version(version);

  // proto->set_last_door_override_time(last_door_override_time);

  *proto->mutable_decider_state() = decider_state;

  *proto->mutable_initializer_state() = initializer_state;

  *proto->mutable_speed_finder_state() = speed_finder_state;

  prev_lane_path_before_lc.ToProto(proto->mutable_prev_lane_path_before_lc());
  *proto->mutable_st_planner_object_trajectories() =
      st_planner_object_trajectories;
  // proto->set_current_time(absl::ToUnixMicros(current_time));
  // proto->set_route_update_id(route_update_id);
  // *proto->mutable_freespace_planner_state() = freespace_planner_state;

  // *proto->mutable_mission_stage() = mission_stage;

  // Parallel planner state.
  prev_target_lane_path.ToProto(proto->mutable_prev_target_lane_path());
  proto->set_prev_length_along_route(prev_length_along_route);
  proto->set_prev_max_reach_length(prev_max_reach_length);
  station_anchor.ToProto(proto->mutable_station_anchor());
  // prev_route_sections.ToProto(proto->mutable_prev_route_sections());

  // Reference line smooth.
  proto->set_prev_smooth_state(prev_smooth_state);
  proto->mutable_smooth_result_map()->mutable_lane_id_vec()->Reserve(
      smooth_result_map.smoothed_result_map().size());
  for (const auto& it : smooth_result_map.smoothed_result_map()) {
    auto* lane_id_vec = proto->mutable_smooth_result_map()->add_lane_id_vec();
    for (const auto id : it.first) {
      lane_id_vec->add_lane_id(id);
    }
  }

  // For teleop lane change.
  preferred_lane_path.ToProto(proto->mutable_preferred_lane_path());

  // Plan task queue
  // proto->mutable_plan_task_queue()->Reserve(plan_task_queue.size());
  // int index = 0;
  // for (const auto& task : plan_task_queue) {
  //   task.ToProto(proto->add_plan_task_queue(), index);
  //   index++;
  // }

  // Selected trajectory optimizer state.
  if (selected_trajectory_optimizer_state_proto.has_value()) {
    *proto->mutable_selected_trajectory_optimizer_state() =
        (*selected_trajectory_optimizer_state_proto);
  } else {
    proto->mutable_selected_trajectory_optimizer_state()->Clear();
  }

  // Previous trajectory end info.
  // if (prev_traj_end_info.has_value()) {
  //   proto->mutable_previous_trajectory_end_info()->CopyFrom(
  //       *prev_traj_end_info);
  // } else {
  //   proto->mutable_previous_trajectory_end_info()->Clear();
  // }

  // proto->set_stopped_at_route_end(stopped_at_route_end);

  // *proto->mutable_assist_plan_state() = assist_plan_state;

  selector_state.ToProto(proto->mutable_selector_state());

  // async_planner_state.ToProto(proto->mutable_async_planner_state());

  // proto->set_previously_triggered_aeb(previously_triggered_aeb);

  // if (prev_online_map_id != kInvalidOnlineMapId) {
  //   proto->set_prev_online_map_id(prev_online_map_id);
  // }
  proto->set_spdlimit_curvature_gain_prev(spdlimit_curvature_gain_prev);
}

// This `operator==` could not be defined in anonymous namespace.
bool operator==(const PlannerState::PosePoint& lhs,
                const PlannerState::PosePoint& rhs) {
  return lhs.pos.x() == rhs.pos.x() && lhs.pos.y() == rhs.pos.y() &&
         lhs.theta == rhs.theta;
}

bool PlannerState::operator==(const PlannerState& other) const {
  if (
      // previous_trajectory_global != other.previous_trajectory_global ||
      // previous_past_trajectory_global !=
      //     other.previous_past_trajectory_global ||
      previous_trajectory != other.previous_trajectory ||
      // last_audio_alert_time != other.last_audio_alert_time ||
      // parking_brake_release_time != other.parking_brake_release_time ||
      // planner_skip_counter != other.planner_skip_counter ||
      !ProtoEquals(lane_change_state, other.lane_change_state) ||
      // !ProtoEquals(input_seq_num, other.input_seq_num) ||
      previous_autonomy_state != other.previous_autonomy_state ||
      // previous_trajectory_plan_counter !=
      //     other.previous_trajectory_plan_counter ||
      // last_door_override_time != other.last_door_override_time ||
      prev_lane_path_before_lc != other.prev_lane_path_before_lc ||
      !ProtoEquals(initializer_state, other.initializer_state) ||
      !ProtoEquals(st_planner_object_trajectories,
                   other.st_planner_object_trajectories) ||
      // !ProtoEquals(freespace_planner_state, other.freespace_planner_state) ||
      prev_target_lane_path != other.prev_target_lane_path ||
      prev_length_along_route != other.prev_length_along_route ||
      station_anchor != other.station_anchor ||
      preferred_lane_path != other.preferred_lane_path ||
      prev_smooth_state != other.prev_smooth_state ||
      spdlimit_curvature_gain_prev != other.spdlimit_curvature_gain_prev
      // current_time != other.current_time ||
      // async_planner_state != other.async_planner_state ||
      // previously_triggered_aeb != other.previously_triggered_aeb
  ) {
    return false;
  }

  return true;
}

bool PlannerState::Upgrade() {
  // trajectory_start_timestamp is set improperly, compatible for old runs";
  if (previous_trajectory.trajectory_start_timestamp() < 1E-6) {
    previous_trajectory.set_trajectory_start_timestamp(header.timestamp() /
                                                       1E6);
  }
  return true;
}

std::string PlannerState::DebugString() const {
  PlannerStateProto proto;
  ToProto(&proto);
  return proto.DebugString();
}

}  // namespace st::planning
