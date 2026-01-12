#ifndef _PLAN_COMMON_SERIALIZATION_ST_PATH_PLANNER_INPUT_SERIALIZE_H_
#define _PLAN_COMMON_SERIALIZATION_ST_PATH_PLANNER_INPUT_SERIALIZE_H_

#include "plan_common/plan_start_point_info.h"
#include "plan_common/drive_passage.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/type_def.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "object_manager/st_inference/st_path_planner_input.h"
#include "object_manager/planner_object_manager.h"
#include "node_manager/task_runner/est_planner.h"

#include "plan_common/serialization/base_serialization.h"
#include "plan_common/serialization/map_serialization.h"
#include "plan_common/serialization/selector_state_serialization.h"
#include "plan_common/serialization/planner_state_serialization.h"
#include <cereal/types/map.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/memory.hpp>

namespace cereal {
template <typename Archive>
void serialize(Archive& ar, st::FrenetBox& frenet_box) {
  ar(CEREAL_NVP(frenet_box.s_max));
  ar(CEREAL_NVP(frenet_box.s_min));
  ar(CEREAL_NVP(frenet_box.l_max));
  ar(CEREAL_NVP(frenet_box.l_min));
}

template <typename Archive>
void serialize(
    Archive* ar,
    st::planning::StPathPlanStartPointInfo& st_path_plan_start_point_info) {
  ar(CEREAL_NVP(st_path_plan_start_point_info.reset));
  ar(CEREAL_NVP(
      st_path_plan_start_point_info.relative_index_from_plan_start_point));
  ar(CEREAL_NVP(st_path_plan_start_point_info.start_point));
  ar(CEREAL_NVP(st_path_plan_start_point_info.plan_time));
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::StPathPlannerInput& st_path_planner_input) {
  ar(CEREAL_NVP(st_path_planner_input.plan_id));
  ar(CEREAL_NVP(*st_path_planner_input.st_path_start_point_info));
  // ar(CEREAL_NVP(st_path_planner_input.path_look_ahead_duration)); // TODO
  ar(CEREAL_NVP(*st_path_planner_input.vehicle_params));
  ar(CEREAL_NVP(*st_path_planner_input.planner_semantic_map_manager));
  ar(CEREAL_NVP(*st_path_planner_input.smooth_result_map));
  ar(CEREAL_NVP(st_path_planner_input.scheduler_output));
  ar(CEREAL_NVP(*st_path_planner_input.traj_mgr));
  ar(CEREAL_NVP(st_path_planner_input.pnp_infos));
  ar(CEREAL_NVP(st_path_planner_input.lane_change_style));
  ar(CEREAL_NVP(*st_path_planner_input.start_point_info));
  ar(CEREAL_NVP(*st_path_planner_input.route_target_info));
  // ar(CEREAL_NVP(*st_path_planner_input.obj_mgr));
  ar(CEREAL_NVP(*st_path_planner_input.traffic_light_status_map));
  ar(CEREAL_NVP(*st_path_planner_input.prev_decider_state));
  // ar(CEREAL_NVP(st_path_planner_input.init_st_planner_object_traj));
  // ar(CEREAL_NVP(*st_path_planner_input.stalled_objects));
  ar(CEREAL_NVP(*st_path_planner_input.scene_reasoning));
  // ar(CEREAL_NVP(st_path_planner_input.decider_output));
  ar(CEREAL_NVP(*st_path_planner_input.prev_target_lane_path_from_start));
  ar(CEREAL_NVP(*st_path_planner_input.pre_large_vehicle_avoid_state));
  // ar(CEREAL_NVP(*st_path_planner_input.time_aligned_prev_traj));
  ar(CEREAL_NVP(*st_path_planner_input.prev_initializer_state));
  ar(CEREAL_NVP(*st_path_planner_input.trajectory_optimizer_state_proto));
  ar(CEREAL_NVP(*st_path_planner_input.decision_constraint_config));
  // ar(CEREAL_NVP(*st_path_planner_input.initializer_params));
  ar(CEREAL_NVP(*st_path_planner_input.trajectory_optimizer_params));
  ar(CEREAL_NVP(*st_path_planner_input.motion_constraint_params));
  ar(CEREAL_NVP(*st_path_planner_input.planner_functions_params));
  ar(CEREAL_NVP(*st_path_planner_input.vehicle_models_params));
  ar(CEREAL_NVP(*st_path_planner_input.trajectory_optimizer_lc_radical_params));
  ar(CEREAL_NVP(*st_path_planner_input.trajectory_optimizer_lc_normal_params));
  ar(CEREAL_NVP(
      *st_path_planner_input.trajectory_optimizer_lc_conservative_params));
  ar(CEREAL_NVP(*st_path_planner_input.behavior));
  // ar(CEREAL_NVP(*st_path_planner_input.speed_state));
  ar(CEREAL_NVP(st_path_planner_input.miss_navi_scenario));
  ar(CEREAL_NVP(*st_path_planner_input.obs_history));
  ar(CEREAL_NVP(st_path_planner_input.cur_navi_lc_num));
  ar(CEREAL_NVP(st_path_planner_input.leading_id));
  ar(CEREAL_NVP(st_path_planner_input.left_navi_dist));
  ar(CEREAL_NVP(st_path_planner_input.left_navi_dist_v2));
  ar(CEREAL_NVP(st_path_planner_input.prev_lane_change_stage));
  ar(CEREAL_NVP(st_path_planner_input.lc_cmd_state));
  ar(CEREAL_NVP(st_path_planner_input.push_dir));
  ar(CEREAL_NVP(*st_path_planner_input.nudge_object_info));
  ar(CEREAL_NVP(st_path_planner_input.cur_dist_to_junction));
  ar(CEREAL_NVP(*st_path_planner_input.saved_offset));
  ar(CEREAL_NVP(st_path_planner_input.pre_lc_style_decider_result));
  ar(CEREAL_NVP(st_path_planner_input.pre_task_safety_evaluation_result));
}

}  // namespace cereal

namespace st::planning {

template <typename Archive>
void serialize(Archive& ar, ConstantAccLimit& constant_acclimit) {
  ar(CEREAL_NVP(constant_acclimit.acc));
  ar(CEREAL_NVP(constant_acclimit.speed));
  ar(CEREAL_NVP(constant_acclimit.source));
}

template <typename Archive>
void serialize(Archive& ar, ConstantSpeedLimit& constant_speedlimit) {
  ar(CEREAL_NVP(constant_speedlimit.speed));
  ar(CEREAL_NVP(constant_speedlimit.time));
  ar(CEREAL_NVP(constant_speedlimit.source));
}

template <typename Archive>
void serialize(Archive& ar, OpenLoopSpeedLimit& open_loop_speed_limit) {
  ar(CEREAL_NVP(open_loop_speed_limit.constant_acc_limits_));
  ar(CEREAL_NVP(open_loop_speed_limit.constant_speed_limits_));
}

template <typename Archive>
void serialize(Archive& ar, TrafficGapResult& traffic_gap_result) {
  ar(CEREAL_NVP(traffic_gap_result.leader_id));
  ar(CEREAL_NVP(traffic_gap_result.follower_id));
  ar(CEREAL_NVP(traffic_gap_result.dec_gap_target_speed));
  ar(CEREAL_NVP(traffic_gap_result.dec_gap_target_a));
  ar(CEREAL_NVP(traffic_gap_result.acc_gap_target_speed));
  ar(CEREAL_NVP(traffic_gap_result.acc_gap_target_a));
  ar(CEREAL_NVP(traffic_gap_result.gap_length));
  ar(CEREAL_NVP(traffic_gap_result.gap_ttc));
  ar(CEREAL_NVP(traffic_gap_result.gap_v));
  ar(CEREAL_NVP(traffic_gap_result.gap_a));
  ar(CEREAL_NVP(traffic_gap_result.gap_t));
  ar(CEREAL_NVP(traffic_gap_result.gap_total_cost));
}

template <typename Archive>
void serialize(Archive& ar, ConstraintManager& constraint_manager) {
  ar(CEREAL_NVP(constraint_manager.speed_region_));
  ar(CEREAL_NVP(constraint_manager.stop_line_));
  ar(CEREAL_NVP(constraint_manager.path_stop_line_));
  ar(CEREAL_NVP(constraint_manager.path_speed_region_));
  ar(CEREAL_NVP(constraint_manager.avoid_line_));
  ar(CEREAL_NVP(constraint_manager.speed_profiles_));
  ar(CEREAL_NVP(constraint_manager.traffic_gap_));
  ar(CEREAL_NVP(constraint_manager.open_loop_speed_limits_));
}

template <class Archive>
void serialize(Archive& ar, DeciderOutput& decider_output) {
  ar(CEREAL_NVP(decider_output.constraint_manager));
  ar(CEREAL_NVP(decider_output.decider_state));
  ar(CEREAL_NVP(decider_output.distance_to_traffic_light_stop_line));
  ar(CEREAL_NVP(decider_output.tl_stop_interface));
  ar(CEREAL_NVP(decider_output.speed_state));
}

template <class Archive>
void serialize(
    Archive& ar,
    SpacetimePlannerObjectTrajectories::TrajectoryInfo& trajectory_info) {
  ar(CEREAL_NVP(trajectory_info.traj_index));
  ar(CEREAL_NVP(trajectory_info.object_id));
  ar(CEREAL_NVP(trajectory_info.reason));
}

template <class Archive>
void serialize(
    Archive& ar,
    SpacetimePlannerObjectTrajectories& spacetime_planner_object_trajectories) {
  ar(CEREAL_NVP(spacetime_planner_object_trajectories.trajectories));
  ar(CEREAL_NVP(spacetime_planner_object_trajectories.trajectory_infos));
  ar(CEREAL_NVP(spacetime_planner_object_trajectories.trajectory_ids));
  ar(CEREAL_NVP(spacetime_planner_object_trajectories.st_start_offset));
}

template <class Archive>
void serialize(Archive& ar, PlannerObjectManager& planner_object_manager) {
  ar(CEREAL_NVP(planner_object_manager.planner_objects_));
  for (const auto* i : planner_object_manager.stationary_objects_) {
    ar(CEREAL_NVP(*i));
  }
  for (const auto* i : planner_object_manager.moving_objects_) {
    ar(CEREAL_NVP(*i));
  }

  // ar(CEREAL_NVP(planner_object_manager.object_map_));
}

template <typename Archive>
void serialize(Archive& ar, RouteTargetInfo& route_target_info) {
  ar(CEREAL_NVP(route_target_info.plan_id));
  // TODO
  // ar(CEREAL_NVP(route_target_info.frenet_frame));
  // ar(CEREAL_NVP(route_target_info.ego_frenet_box));
  // ar(CEREAL_NVP(route_target_info.drive_passage));
  // ar(CEREAL_NVP(route_target_info.sl_boundary));
  // ar(CEREAL_NVP(route_target_info.st_traj_mgr));
}

template <typename Archive>
void serialize(Archive& ar, PlanStartPointInfo& plan_start_point) {
  ar(CEREAL_NVP(plan_start_point.reset));
  ar(CEREAL_NVP(plan_start_point.start_index_on_prev_traj));
  ar(CEREAL_NVP(plan_start_point.start_point));
  ar(CEREAL_NVP(plan_start_point.path_s_increment_from_previous_frame));
  ar(CEREAL_NVP(plan_start_point.plan_time));
  ar(CEREAL_NVP(plan_start_point.full_stop));
  ar(CEREAL_NVP(plan_start_point.reset_reason));
}

template <typename Archive>
void serialize(Archive& ar,
               SecondOrderTrajectoryPoint& secondorder_trajectory_point) {
  ar(CEREAL_NVP(secondorder_trajectory_point.pos_));
  ar(CEREAL_NVP(secondorder_trajectory_point.s_));
  ar(CEREAL_NVP(secondorder_trajectory_point.theta_));
  ar(CEREAL_NVP(secondorder_trajectory_point.kappa_));
  ar(CEREAL_NVP(secondorder_trajectory_point.steer_angle_));
  ar(CEREAL_NVP(secondorder_trajectory_point.t_));
  ar(CEREAL_NVP(secondorder_trajectory_point.v_));
  ar(CEREAL_NVP(secondorder_trajectory_point.a_));
}

template <typename Archive>
void serialize(Archive& ar, PlannerObject& planner_object) {
  ar(CEREAL_NVP(planner_object.is_stationary_));
  ar(CEREAL_NVP(planner_object.pose_));
  ar(CEREAL_NVP(planner_object.velocity_));
  ar(CEREAL_NVP(planner_object.contour_));
  ar(CEREAL_NVP(planner_object.bounding_box_));
  ar(CEREAL_NVP(planner_object.perception_bbox_));
  ar(CEREAL_NVP(planner_object.aabox_));
  // ar(CEREAL_NVP(planner_object.prediction_));
  ar(CEREAL_NVP(planner_object.object_proto_));
  ar(CEREAL_NVP(planner_object.is_sim_agent_));
  ar(CEREAL_NVP(planner_object.is_large_vehicle_));
  ar(CEREAL_NVP(planner_object.base_id_));
}

template <typename Archive>
void serialize(Archive& ar,
               SpacetimeObjectTrajectory& spacetime_object_trajectory) {
  ar(CEREAL_NVP(spacetime_object_trajectory.traj_index_));
  ar(CEREAL_NVP(spacetime_object_trajectory.traj_id_));
  ar(CEREAL_NVP(spacetime_object_trajectory.is_stationary_));
  ar(CEREAL_NVP(spacetime_object_trajectory.planner_object_));
  ar(CEREAL_NVP(spacetime_object_trajectory.required_lateral_gap_));
  ar(CEREAL_NVP(spacetime_object_trajectory.states_));
  ar(CEREAL_NVP(spacetime_object_trajectory.trajectory_));
}

template <typename Archive>
void serialize(
    Archive& ar,
    SpacetimeTrajectoryManager::IgnoredTrajectory& ignored_trajectory) {
  ar(CEREAL_NVP(*ignored_trajectory.traj));
  ar(CEREAL_NVP(ignored_trajectory.object_id));
  ar(CEREAL_NVP(ignored_trajectory.reason));
}

template <typename Archive>
void serialize(
    Archive& ar,
    SpacetimeTrajectoryManager::StationaryObject& stationary_object) {
  ar(CEREAL_NVP(stationary_object.object_id));
  ar(CEREAL_NVP(stationary_object.planner_object));
}

template <typename Archive>
void serialize(Archive& ar,
               SpacetimeTrajectoryManager& spacetime_trajectory_manager) {
  // TODO
  // for (const auto& map_iter : spacetime_trajectory_manager.objects_id_map_) {
  //   for (const auto* iter : map_iter.second) {
  //     ar(CEREAL_NVP(map_iter.first, *iter));
  //   }
  // }
  // for (const auto& map_iter :
  // spacetime_trajectory_manager.trajectories_id_map_) {
  //   ar(CEREAL_NVP(map_iter.first, *map_iter.second));
  // }

  // // ar(CEREAL_NVP(spacetime_trajectory_manager.objects_id_map_));
  // // ar(CEREAL_NVP(spacetime_trajectory_manager.trajectories_id_map_));
  // ar(CEREAL_NVP(spacetime_trajectory_manager.considered_trajs_));
  // ar(CEREAL_NVP(spacetime_trajectory_manager.ignored_trajs_));
  // ar(CEREAL_NVP(spacetime_trajectory_manager.stationary_objs_));
  // //
  // ar(CEREAL_NVP(spacetime_trajectory_manager.considered_stationary_trajs_));
  // for (const auto* iter :
  // spacetime_trajectory_manager.considered_stationary_trajs_) {
  //   ar(CEREAL_NVP(*iter));
  // }
  // // ar(CEREAL_NVP(spacetime_trajectory_manager.considered_moving_trajs_));
  // for (const auto* iter :
  // spacetime_trajectory_manager.considered_moving_trajs_) {
  //   ar(CEREAL_NVP(*iter));
  // }
}

template <typename Archive>
void serialize(Archive* ar, StationInfo& station_info) {
  ar(CEREAL_NVP(station_info.is_in_intersection));
  ar(CEREAL_NVP(station_info.is_exclusive_right_turn));
  ar(CEREAL_NVP(station_info.is_in_roundabout));
  ar(CEREAL_NVP(station_info.speed_limit));
  ar(CEREAL_NVP(station_info.lane_type));
  ar(CEREAL_NVP(station_info.turn_type));
  ar(CEREAL_NVP(station_info.split_topo));
  ar(CEREAL_NVP(station_info.merge_topo));
}

template <typename Archive>
void serialize(Archive* ar, StationCenter& station_center) {
  ar(CEREAL_NVP(station_center.lane_id));
  ar(CEREAL_NVP(station_center.fraction));
  ar(CEREAL_NVP(station_center.xy));
  ar(CEREAL_NVP(station_center.tangent));
  ar(CEREAL_NVP(station_center.speed_limit));
  ar(CEREAL_NVP(station_center.is_virtual));
  ar(CEREAL_NVP(station_center.is_merging));
  ar(CEREAL_NVP(station_center.is_splitting));
  ar(CEREAL_NVP(station_center.is_in_intersection));
  ar(CEREAL_NVP(station_center.has_cross_curb));
  ar(CEREAL_NVP(station_center.turn_type));
  ar(CEREAL_NVP(station_center.station_info));
}

template <typename Archive>
void serialize(Archive* ar, StationBoundary& stationboundary) {
  ar(CEREAL_NVP(stationboundary.type));
  ar(CEREAL_NVP(stationboundary.lat_offset));
}

template <typename Archive>
void serialize(Archive* ar, Station& station) {
  ar(CEREAL_NVP(station.center_));
  ar(CEREAL_NVP(station.boundaries_));
}

template <typename Archive>
void serialize(Archive& ar, DrivePassage& drive_passage) {
  ar(CEREAL_NVP(drive_passage.stations_));
  ar(CEREAL_NVP(drive_passage.last_real_station_index_));
  ar(CEREAL_NVP(drive_passage.center_seg_inv_len_));
  ar(CEREAL_NVP(drive_passage.lane_path_));
  ar(CEREAL_NVP(drive_passage.extend_lane_path_));
  ar(CEREAL_NVP(drive_passage.beyond_lane_path_));
  ar(CEREAL_NVP(drive_passage.reach_destination_));
  ar(CEREAL_NVP(drive_passage.lane_path_start_s_));
  ar(CEREAL_NVP(drive_passage.segments_));
  ar(CEREAL_NVP(drive_passage.type_));
  ar(CEREAL_NVP(drive_passage.lane_seq_info_));
  ar(CEREAL_NVP(drive_passage.change_index_));
  // ar(CEREAL_NVP(drive_passage.frenet_frame_)); // TODO
}

template <typename Archive>
void serialize(Archive& ar, PathSlBoundary& path_sl_boundary) {
  ar(CEREAL_NVP(path_sl_boundary.s_vec_));
  ar(CEREAL_NVP(path_sl_boundary.ref_center_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.right_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.left_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.opt_right_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.opt_left_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.target_right_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.target_left_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.ref_center_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.right_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.left_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.opt_right_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.opt_left_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.target_right_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.target_left_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.right_type_vec_));
  ar(CEREAL_NVP(path_sl_boundary.left_type_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_s_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_ref_center_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_right_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_left_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_opt_right_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_opt_left_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_target_right_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_target_left_l_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_ref_center_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_right_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_left_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_opt_right_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_opt_left_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_target_right_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.resampled_target_left_xy_vec_));
  ar(CEREAL_NVP(path_sl_boundary.sample_interval_));
  ar(CEREAL_NVP(path_sl_boundary.offset_type_value_));
}

template <typename Archive>
void serialize(Archive* ar, st::planning::SchedulerOutput& scheduler_output) {
  ar(CEREAL_NVP(scheduler_output.is_fallback));
  ar(CEREAL_NVP(scheduler_output.is_expert));
  // ar(CEREAL_NVP(scheduler_output.drive_passage));
  ar(CEREAL_NVP(scheduler_output.sl_boundary));
  ar(CEREAL_NVP(scheduler_output.target_offset_from_start));
  ar(CEREAL_NVP(scheduler_output.lane_change_state));
  ar(CEREAL_NVP(scheduler_output.lc_reason));
  ar(CEREAL_NVP(scheduler_output.lane_path_before_lc));
  ar(CEREAL_NVP(scheduler_output.length_along_route));
  ar(CEREAL_NVP(scheduler_output.max_reach_length));
  ar(CEREAL_NVP(scheduler_output.lc_num));
  ar(CEREAL_NVP(scheduler_output.leading_id));
  ar(CEREAL_NVP(scheduler_output.standard_congestion_factor));
  ar(CEREAL_NVP(scheduler_output.traffic_congestion_factor));
  ar(CEREAL_NVP(scheduler_output.should_smooth));
  ar(CEREAL_NVP(scheduler_output.borrow_lane));
  ar(CEREAL_NVP(scheduler_output.av_frenet_box_on_drive_passage));
  ar(CEREAL_NVP(scheduler_output.request_help_lane_change_by_route));
  ar(CEREAL_NVP(scheduler_output.switch_alternate_route));
  ar(CEREAL_NVP(scheduler_output.planner_turn_signal));
  ar(CEREAL_NVP(scheduler_output.turn_signal_reason));
  ar(CEREAL_NVP(scheduler_output.miss_navi_scenario));
  ar(CEREAL_NVP(scheduler_output.saved_offset));
}

}  // namespace st::planning

namespace st::prediction {

template <typename Archive>
void serialize(Archive& ar, PredictionObjectState& prediction_object_state) {
  ar(CEREAL_NVP(*prediction_object_state.traj_point));
  ar(CEREAL_NVP(prediction_object_state.box));
  ar(CEREAL_NVP(prediction_object_state.contour));
}

template <typename Archive>
void serialize(Archive& ar, ObjectLongTermBehavior& object_longterm_behavior) {
  ar(CEREAL_NVP(object_longterm_behavior.avg_speed));
  ar(CEREAL_NVP(object_longterm_behavior.obs_duration));
  ar(CEREAL_NVP(object_longterm_behavior.accel_history));
}

template <typename Archive>
void serialize(Archive& ar, ObjectPrediction& object_prediction) {
  ar(CEREAL_NVP(object_prediction.trajectories_));
  ar(CEREAL_NVP(object_prediction.contour_));
  ar(CEREAL_NVP(object_prediction.perception_object_));
  ar(CEREAL_NVP(object_prediction.stop_time_));
  ar(CEREAL_NVP(object_prediction.long_term_behavior_));
}

template <typename Archive>
void serialize(Archive& ar, PredictedTrajectory& predicted_trajectory) {
  ar(CEREAL_NVP(predicted_trajectory.probability_));
  ar(CEREAL_NVP(predicted_trajectory.type_));
  ar(CEREAL_NVP(predicted_trajectory.index_));
  ar(CEREAL_NVP(predicted_trajectory.points_));
  ar(CEREAL_NVP(predicted_trajectory.is_reversed_));
  ar(CEREAL_NVP(predicted_trajectory.intention_));
}
}  // namespace st::prediction

namespace ad_byd::planning {
template <typename Archive>
void serialize(Archive& ar, SpeedState& speed_state) {
  ar(CEREAL_NVP(speed_state.attention_obj_id));
  ar(CEREAL_NVP(speed_state.lcc_keep_brake));
  ar(CEREAL_NVP(speed_state.yield_to_vru));
  ar(CEREAL_NVP(speed_state.vru_interact_timer));
  ar(CEREAL_NVP(speed_state.infront_vru_ids));
  ar(CEREAL_NVP(speed_state.pre_fast_speed_limit));
  ar(CEREAL_NVP(speed_state.pre_lc_num));
  ar(CEREAL_NVP(speed_state.lc_num_has_up));
  ar(CEREAL_NVP(speed_state.pre_dynamic_acc_limit));
  ar(CEREAL_NVP(speed_state.pre_special_acc_gap_secnario));
  ar(CEREAL_NVP(speed_state.first_has_speed_limit));
}

}  // namespace ad_byd::planning

#endif