#ifndef _PLAN_COMMON_SERIALIZATION_MAP_SERIALIZATION_
#define _PLAN_COMMON_SERIALIZATION_MAP_SERIALIZATION_

#include "plan_common/maps/lane.h"
#include "plan_common/maps/lane_boundaries.h"
#include "plan_common/maps/lane_boundary.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/lane_path_data.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/road_boundaries.h"
#include "plan_common/maps/road_boundary.h"
#include "plan_common/maps/clear_aera.h"
#include "plan_common/maps/junction.h"
#include "plan_common/maps/exp_trajectory.h"
#include "plan_common/maps/route.h"
#include "plan_common/maps/section.h"
#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/serialization/base_serialization.h"

#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/list.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/unordered_set.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

namespace cereal {
// MapInfo data serialization
template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::LaneMergeInfo& lane_merge_info) {
  ar(CEREAL_NVP(lane_merge_info.valid));
  ar(CEREAL_NVP(lane_merge_info.merge_source));
  ar(CEREAL_NVP(lane_merge_info.dist_to_merge));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::LaneInfo& lane_info) {
  ar(CEREAL_NVP(lane_info.id));
  ar(CEREAL_NVP(lane_info.section_id));
  ar(CEREAL_NVP(lane_info.junction_id));
  ar(CEREAL_NVP(lane_info.left_lane_id));
  ar(CEREAL_NVP(lane_info.right_lane_id));
  ar(CEREAL_NVP(lane_info.next_lane_ids));
  ar(CEREAL_NVP(lane_info.left_lane_boundary_ids));
  ar(CEREAL_NVP(lane_info.right_lane_boundary_ids));
  ar(CEREAL_NVP(lane_info.left_road_boundary_ids));
  ar(CEREAL_NVP(lane_info.right_road_boundary_ids));
  ar(CEREAL_NVP(lane_info.type));
  ar(CEREAL_NVP(lane_info.none_odd_type));
  ar(CEREAL_NVP(lane_info.turn_type));
  ar(CEREAL_NVP(lane_info.light_status));
  ar(CEREAL_NVP(lane_info.split_topology));
  ar(CEREAL_NVP(lane_info.merge_topology));
  ar(CEREAL_NVP(lane_info.lane_merge_info));
  ar(CEREAL_NVP(lane_info.points));
  ar(CEREAL_NVP(lane_info.length));
  ar(CEREAL_NVP(lane_info.speed_limit));
  ar(CEREAL_NVP(lane_info.is_virtual));
  ar(CEREAL_NVP(lane_info.is_navigation));
  ar(CEREAL_NVP(lane_info.is_virtual_navigation));
  ar(CEREAL_NVP(lane_info.stop_line));
  ar(CEREAL_NVP(lane_info.cross_walks));
  ar(CEREAL_NVP(lane_info.speed_bumps));
  ar(CEREAL_NVP(lane_info.parking_spaces));
  ar(CEREAL_NVP(lane_info.clear_areas));
  ar(CEREAL_NVP(lane_info.traffic_stop_lines));
  ar(CEREAL_NVP(lane_info.lane_operation_type));
  ar(CEREAL_NVP(lane_info.coeffs));
  ar(CEREAL_NVP(lane_info.arrow_type));
  ar(CEREAL_NVP(lane_info.light_countdown));
  ar(CEREAL_NVP(lane_info.is_exp_traj));
  ar(CEREAL_NVP(lane_info.is_split_topo_modify_));
  ar(CEREAL_NVP(lane_info.is_merge_topo_modify_));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::LaneBoundaryType& boundary_type) {
  ar(CEREAL_NVP(boundary_type.s));
  ar(CEREAL_NVP(boundary_type.line_type));
  ar(CEREAL_NVP(boundary_type.line_color));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::RoadBoundaryType& boundary_type) {
  ar(CEREAL_NVP(boundary_type.s));
  ar(CEREAL_NVP(boundary_type.width));
  ar(CEREAL_NVP(boundary_type.boundary_type));
}

template <typename Archive>
void serialize(Archive& ar,
               ad_byd::planning::LaneBoundaryInfo& lane_boundary_info) {
  ar(CEREAL_NVP(lane_boundary_info.id));
  ar(CEREAL_NVP(lane_boundary_info.points));
  ar(CEREAL_NVP(lane_boundary_info.boundary_type));
}

template <typename Archive>
void serialize(Archive& ar,
               ad_byd::planning::RoadBoundaryInfo& road_boundary_info) {
  ar(CEREAL_NVP(road_boundary_info.id));
  ar(CEREAL_NVP(road_boundary_info.width));
  ar(CEREAL_NVP(road_boundary_info.points));
  ar(CEREAL_NVP(road_boundary_info.boundary_type));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::StopLineInfo& stop_line_info) {
  ar(CEREAL_NVP(stop_line_info.id));
  ar(CEREAL_NVP(stop_line_info.points));
  ar(CEREAL_NVP(stop_line_info.type));
  ar(CEREAL_NVP(stop_line_info.light_type));
  ar(CEREAL_NVP(stop_line_info.sub_type));
  ar(CEREAL_NVP(stop_line_info.virtual_type));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::JunctionInfo& junction_info) {
  ar(CEREAL_NVP(junction_info.id));
  ar(CEREAL_NVP(junction_info.points));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::CrossWalkInfo& crosswalk_info) {
  ar(CEREAL_NVP(crosswalk_info.id));
  ar(CEREAL_NVP(crosswalk_info.points));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::SpeedBumpInfo& speedbump_info) {
  ar(CEREAL_NVP(speedbump_info.id));
  ar(CEREAL_NVP(speedbump_info.points));
}

template <typename Archive>
void serialize(Archive& ar,
               ad_byd::planning::ExpTrajectoryInfo& exp_trajectory_info) {
  ar(CEREAL_NVP(exp_trajectory_info.id));
  ar(CEREAL_NVP(exp_trajectory_info.lane_id));
  ar(CEREAL_NVP(exp_trajectory_info.start_lane_id));
  ar(CEREAL_NVP(exp_trajectory_info.end_lane_id));
  ar(CEREAL_NVP(exp_trajectory_info.relative_lane_id));
  ar(CEREAL_NVP(exp_trajectory_info.points));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::NaviPosition& navi_position) {
  ar(CEREAL_NVP(navi_position.section_id));
  ar(CEREAL_NVP(navi_position.s_offset));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::SectionInfo& section_info) {
  ar(CEREAL_NVP(section_info.id));
  ar(CEREAL_NVP(section_info.length));
  ar(CEREAL_NVP(section_info.none_odd_type));
  ar(CEREAL_NVP(section_info.lane_ids));
  ar(CEREAL_NVP(section_info.navi_priority_lane_id));
  ar(CEREAL_NVP(section_info.road_class));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::RouteInfo& route_info) {
  ar(CEREAL_NVP(route_info.id));
  ar(CEREAL_NVP(route_info.navi_start));
  ar(CEREAL_NVP(route_info.navi_end));
  ar(CEREAL_NVP(route_info.sections));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::V2RoadClass v2_road_class) {
  ar(CEREAL_NVP(v2_road_class.start_s));
  ar(CEREAL_NVP(v2_road_class.end_s));
  ar(CEREAL_NVP(v2_road_class.type));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::V2TrafficFlow& v2_traffic_flow) {
  ar(CEREAL_NVP(v2_traffic_flow.start_s));
  ar(CEREAL_NVP(v2_traffic_flow.end_s));
  ar(CEREAL_NVP(v2_traffic_flow.type));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::RoadInfo& road_info) {
  ar(CEREAL_NVP(road_info.road_class));
  ar(CEREAL_NVP(road_info.lane_num));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::PassInfo& pass_info) {
  ar(CEREAL_NVP(pass_info.curr_index));
  ar(CEREAL_NVP(pass_info.index_num));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::V2TurnInfo& v2_turn_info) {
  ar(CEREAL_NVP(v2_turn_info.id));
  ar(CEREAL_NVP(v2_turn_info.is_valid));
  ar(CEREAL_NVP(v2_turn_info.turn_type));
  ar(CEREAL_NVP(v2_turn_info.detail_turn_type));
  ar(CEREAL_NVP(v2_turn_info.dist));
  ar(CEREAL_NVP(v2_turn_info.original_dist));
  ar(CEREAL_NVP(v2_turn_info.before_turn));
  ar(CEREAL_NVP(v2_turn_info.after_turn));
  ar(CEREAL_NVP(v2_turn_info.infos));
  ar(CEREAL_NVP(v2_turn_info.straight_pass_info));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::V2Curvature& v2_curvature) {
  ar(CEREAL_NVP(v2_curvature.curvature));
  ar(CEREAL_NVP(v2_curvature.distance));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::NonODDInfo& non_odd_info) {
  ar(CEREAL_NVP(non_odd_info.id));
  ar(CEREAL_NVP(non_odd_info.reason));
  ar(CEREAL_NVP(non_odd_info.dist));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::EHPV2Info& v2_info) {
  ar(CEREAL_NVP(v2_info.is_valid));
  ar(CEREAL_NVP(v2_info.has_navigation));
  ar(CEREAL_NVP(v2_info.dist_to_ramp));
  ar(CEREAL_NVP(v2_info.dist_to_toll));
  ar(CEREAL_NVP(v2_info.dist_to_tunnel));
  ar(CEREAL_NVP(v2_info.dist_to_subpath));
  ar(CEREAL_NVP(v2_info.dist_to_route_split));
  ar(CEREAL_NVP(v2_info.road_class));
  ar(CEREAL_NVP(v2_info.traffic_flow));
  ar(CEREAL_NVP(v2_info.turn_info));
  ar(CEREAL_NVP(v2_info.pnp_turn_info));
  ar(CEREAL_NVP(v2_info.v2_curvatures));
  ar(CEREAL_NVP(v2_info.v2_nodd_info));
}

template <typename Archive>
void serialize(Archive& ar, ad_byd::planning::MapInfo& map_info) {
  ar(CEREAL_NVP(map_info.type));
  ar(CEREAL_NVP(map_info.sub_type));
  ar(CEREAL_NVP(map_info.timestamp));
  ar(CEREAL_NVP(map_info.seq));
  ar(CEREAL_NVP(map_info.is_on_highway));
  ar(CEREAL_NVP(map_info.all_lanes_vec));
  ar(CEREAL_NVP(map_info.all_lane_boundaries_vec));
  ar(CEREAL_NVP(map_info.all_road_boundaries_vec));
  ar(CEREAL_NVP(map_info.all_stop_lines_vec));
  ar(CEREAL_NVP(map_info.all_junctions_vec));
  ar(CEREAL_NVP(map_info.all_cross_walks_vec));
  ar(CEREAL_NVP(map_info.all_speed_bumps_vec));
  ar(CEREAL_NVP(map_info.all_exp_trajectories_vec));
  ar(CEREAL_NVP(map_info.route));
  ar(CEREAL_NVP(map_info.v2_info));
}

template <typename Archive>
void serialize(Archive& ar,
               ad_byd::planning::ValidationRecord& validation_record) {
  ar(CEREAL_NVP(validation_record.rule));
  ar(CEREAL_NVP(validation_record.note));
}

}  // namespace cereal

namespace ad_byd::planning {
template <typename Archive>
void serialize(Archive& ar, LaneSeqInfo& lane_seq_info) {
  ar(CEREAL_NVP(lane_seq_info.cur_section_ptr));
  ar(CEREAL_NVP(lane_seq_info.cur_section_lane_num));
  ar(CEREAL_NVP(lane_seq_info.cur_lane_position));
  ar(CEREAL_NVP(lane_seq_info.lane_seq));
  ar(CEREAL_NVP(lane_seq_info.nearest_lane));
  ar(CEREAL_NVP(lane_seq_info.navi_end_lane));
  ar(CEREAL_NVP(lane_seq_info.junction_lane));
  ar(CEREAL_NVP(lane_seq_info.dist_to_navi_end));
  ar(CEREAL_NVP(lane_seq_info.lane_seq_connect_navi_end));
  ar(CEREAL_NVP(lane_seq_info.is_current));
  ar(CEREAL_NVP(lane_seq_info.lc_dir));
  ar(CEREAL_NVP(lane_seq_info.dist_to_left_solid_line));
  ar(CEREAL_NVP(lane_seq_info.dist_to_right_solid_line));
  ar(CEREAL_NVP(lane_seq_info.merge_lane));
  ar(CEREAL_NVP(lane_seq_info.merge_command));
  ar(CEREAL_NVP(lane_seq_info.dist_to_junction));
  ar(CEREAL_NVP(lane_seq_info.dist_to_junction_id));
  ar(CEREAL_NVP(lane_seq_info.dist_to_virtual_lane));
  ar(CEREAL_NVP(lane_seq_info.navi_lc_command));
  ar(CEREAL_NVP(lane_seq_info.lc_num));
  ar(CEREAL_NVP(lane_seq_info.next_junction_turn));
  ar(CEREAL_NVP(lane_seq_info.overtake_lc_command));
  ar(CEREAL_NVP(lane_seq_info.dist_to_bus_lane));
  // ar(CEREAL_NVP(lane_seq_info.dist_to_bus_lane_vec));
  ar(CEREAL_NVP(lane_seq_info.pnp_top1_lc_reason));
  ar(CEREAL_NVP(lane_seq_info.lc_reason));
  ar(CEREAL_NVP(lane_seq_info.dist_to_merge));
  ar(CEREAL_NVP(lane_seq_info.dist_to_split));
  ar(CEREAL_NVP(lane_seq_info.lc_dir));
  ar(CEREAL_NVP(lane_seq_info.dist_to_left_solid_line));
  ar(CEREAL_NVP(lane_seq_info.dist_to_right_solid_line));
  ar(CEREAL_NVP(lane_seq_info.dist_to_nearest_split));
  ar(CEREAL_NVP(lane_seq_info.nearest_split_angle));
  ar(CEREAL_NVP(lane_seq_info.split_task_state));
  ar(CEREAL_NVP(lane_seq_info.dist_to_junction_v2));
  ar(CEREAL_NVP(lane_seq_info.dist_to_navi_end_v2));
  ar(CEREAL_NVP(lane_seq_info.nearest_turn_type_v2));
  ar(CEREAL_NVP(lane_seq_info.last_turn_type_v2));
  ar(CEREAL_NVP(lane_seq_info.dist_to_section_fork));
  ar(CEREAL_NVP(lane_seq_info.dist_to_section_merge));
  ar(CEREAL_NVP(lane_seq_info.dist_to_merged_zone));
}

template <typename Archive>
void serialize(Archive& ar, LaneSequence& lane_sequence) {
  ar(CEREAL_NVP(lane_sequence.lanes_));
}

template <typename Archive>
void serialize(Archive& ar, Lane::SampledWidth& sampled_width) {
  ar(CEREAL_NVP(sampled_width.s));
  ar(CEREAL_NVP(sampled_width.dist));
}

template <typename Archive>
void serialize(Archive& ar, Lane& lane) {
  ar(CEREAL_NVP(lane.sampled_left_width_));
  ar(CEREAL_NVP(lane.sampled_right_width_));
  ar(CEREAL_NVP(lane.lane_info_));
  ar(CEREAL_NVP(lane.prev_lane_ids_));
  ar(CEREAL_NVP(lane.valid_pre_lane_ids_));
  ar(CEREAL_NVP(lane.valid_next_lane_ids_));
  ar(CEREAL_NVP(lane.sorted_next_lane_ids_));
  ar(CEREAL_NVP(lane.left_boundary_));
  ar(CEREAL_NVP(lane.right_boundary_));
  ar(CEREAL_NVP(lane.left_road_boundary_));
  ar(CEREAL_NVP(lane.right_road_boundary_));
  ar(CEREAL_NVP(lane.center_line_));
  ar(CEREAL_NVP(lane.navi_distance_));
  ar(CEREAL_NVP(lane.navi_section_cnt_));
  ar(CEREAL_NVP(lane.next_turn_types_));
  ar(CEREAL_NVP(lane.lane_ind_in_section_));
  ar(CEREAL_NVP(lane.interactions_));
  ar(CEREAL_NVP(lane.endpoint_toll_));
}

template <typename Archive>
void serialize(Archive& ar, LaneInteraction& lane_interaction) {
  ar(CEREAL_NVP(lane_interaction.other_lane_fraction));
  ar(CEREAL_NVP(lane_interaction.geometric_configuration));
  ar(CEREAL_NVP(lane_interaction.reaction_rule));
  ar(CEREAL_NVP(lane_interaction.intersection_id));
  ar(CEREAL_NVP(lane_interaction.this_lane_fraction));
  ar(CEREAL_NVP(lane_interaction.this_lane_fraction));
  ar(CEREAL_NVP(lane_interaction.other_lane_fraction));
  ar(CEREAL_NVP(lane_interaction.belonging_levels));
  ar(CEREAL_NVP(lane_interaction.skip_records));
}

template <typename Archive>
void serialize(Archive& ar, RoadBoundaries& road_boundaries) {
  ar(CEREAL_NVP(road_boundaries.boundaries_));
}

template <typename Archive>
void serialize(Archive& ar, RoadBoundary& road_boundary) {
  ar(CEREAL_NVP(road_boundary.id_));
  ar(CEREAL_NVP(road_boundary.points_));
  ar(CEREAL_NVP(road_boundary.line_curve_));
  ar(CEREAL_NVP(road_boundary.type_));
  ar(CEREAL_NVP(road_boundary.length_));
  ar(CEREAL_NVP(road_boundary.curve_length_));
  ar(CEREAL_NVP(road_boundary.section_id_));
}

template <typename Archive>
void serialize(Archive& ar, LaneBoundaries& lane_boundaries) {
  ar(CEREAL_NVP(lane_boundaries.boundaries_));
  ar(CEREAL_NVP(lane_boundaries.boundary_types_));
  ar(CEREAL_NVP(lane_boundaries.line_curve_));
}

template <typename Archive>
void serialize(Archive& ar, LaneBoundary& lane_boundary) {
  ar(CEREAL_NVP(lane_boundary.id_));
  ar(CEREAL_NVP(lane_boundary.points_));
  ar(CEREAL_NVP(lane_boundary.left_lanes_));
  ar(CEREAL_NVP(lane_boundary.right_lanes_));
  ar(CEREAL_NVP(lane_boundary.line_curve_));
  ar(CEREAL_NVP(lane_boundary.type_));
  ar(CEREAL_NVP(lane_boundary.length_));
  ar(CEREAL_NVP(lane_boundary.curve_length_));
  ar(CEREAL_NVP(lane_boundary.section_id_));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::PlannerSemanticMapManager& psmm) {
  ar(CEREAL_NVP(psmm.map_ptr_));
  ar(CEREAL_NVP(psmm.modifier_));
}

template <typename Archive>
void serialize(Archive& ar, TransformInfo& transform_info) {
  ar(CEREAL_NVP(transform_info.delta_x));
  ar(CEREAL_NVP(transform_info.delta_y));
  ar(CEREAL_NVP(transform_info.delta_yaw));
}

template <typename Archive>
void serialize(Archive& ar, ClearAreaInfo& clear_area_info) {
  ar(CEREAL_NVP(clear_area_info.id));
  ar(CEREAL_NVP(clear_area_info.type));
  ar(CEREAL_NVP(clear_area_info.points));
}

template <typename Archive>
void serialize(Archive& ar, ClearArea& clear_area) {
  ar(CEREAL_NVP(clear_area.polygon_));
  ar(CEREAL_NVP(clear_area.points_));
  ar(CEREAL_NVP(clear_area.clear_area_info_));
}

template <typename Archive>
void serialize(Archive& ar, Crosswalk& crosswalk) {
  ar(CEREAL_NVP(crosswalk.points_));
  ar(CEREAL_NVP(crosswalk.polygon_));
  ar(CEREAL_NVP(crosswalk.id_));
  ar(CEREAL_NVP(crosswalk.is_polygon_convex_));
  ar(CEREAL_NVP(crosswalk.intersected_lanes_));
  ar(CEREAL_NVP(crosswalk.bone_axis_smooth_));
}

template <typename Archive>
void serialize(Archive& ar, Junction& junction) {
  ar(CEREAL_NVP(junction.crosswalks_));
  ar(CEREAL_NVP(junction.points_));
  ar(CEREAL_NVP(junction.entry_lanes_));
  ar(CEREAL_NVP(junction.exit_lanes_));
  ar(CEREAL_NVP(junction.overlap_lanes_));
  ar(CEREAL_NVP(junction.has_entry_lanes_));
  ar(CEREAL_NVP(junction.has_exit_lanes_));
  ar(CEREAL_NVP(junction.polygon_));
  ar(CEREAL_NVP(junction.id_));
  ar(CEREAL_NVP(junction.is_polygon_convex_));
  ar(CEREAL_NVP(junction.is_roundabout_));
}

template <typename Archive>
void serialize(Archive& ar, StopLine& stopline) {
  ar(CEREAL_NVP(stopline.id_));
  ar(CEREAL_NVP(stopline.points_));
  ar(CEREAL_NVP(stopline.type_));
  ar(CEREAL_NVP(stopline.light_type_));
  ar(CEREAL_NVP(stopline.sub_type_));
  ar(CEREAL_NVP(stopline.virtual_type_));
}

template <typename Archive>
void serialize(Archive& ar, SpeedBump& speedbump) {
  ar(CEREAL_NVP(speedbump.id_));
  ar(CEREAL_NVP(speedbump.points_));
  ar(CEREAL_NVP(speedbump.polygon_));
  ar(CEREAL_NVP(speedbump.intersected_lanes_));
}

template <typename Archive>
void serialize(Archive& ar, ExpTrajectory& exptrajectory) {
  ar(CEREAL_NVP(exptrajectory.id_));
  ar(CEREAL_NVP(exptrajectory.lane_id_));
  ar(CEREAL_NVP(exptrajectory.start_lane_id_));
  ar(CEREAL_NVP(exptrajectory.end_lane_id_));
  ar(CEREAL_NVP(exptrajectory.relative_lane_id_));
  ar(CEREAL_NVP(exptrajectory.points_));
}

template <typename Archive>
void serialize(Archive& ar, TrafficLightStatus& traffic_light_status) {
  ar(CEREAL_NVP(traffic_light_status.lane_id));
  ar(CEREAL_NVP(traffic_light_status.junction_id));
  ar(CEREAL_NVP(traffic_light_status.light_status));
  ar(CEREAL_NVP(traffic_light_status.stop_line));
  ar(CEREAL_NVP(traffic_light_status.is_left_wait_lane));
  ar(CEREAL_NVP(traffic_light_status.light_countdown));
}

template <typename Archive>
void serialize(Archive& ar, Route& route) {
  ar(CEREAL_NVP(route.route_info_));
  ar(CEREAL_NVP(route.map_));
  // ar(CEREAL_NVP(route.split_merge_shortdis_ids_));
}

template <typename Archive>
void serialize(Archive& ar, Section& section) {
  ar(CEREAL_NVP(section.id_));
  ar(CEREAL_NVP(section.length_));
  ar(CEREAL_NVP(section.curve_length_));
  ar(CEREAL_NVP(section.lanes_));
  ar(CEREAL_NVP(section.outgoing_sections_));
  ar(CEREAL_NVP(section.incoming_sections_));
  ar(CEREAL_NVP(section.speed_limit_));
  ar(CEREAL_NVP(section.average_limit_));
  ar(CEREAL_NVP(section.road_class_));
  ar(CEREAL_NVP(section.none_odd_type_));
  ar(CEREAL_NVP(section.navi_priority_lane_id_));
}

template <typename Archive>
void serialize(Archive& ar, Map::SmoothPoint& smooth_point) {
  ar(CEREAL_NVP(smooth_point.x));
  ar(CEREAL_NVP(smooth_point.y));
}

template <typename Archive>
void serialize(Archive& ar, Map::AABoxNode& aaboxnode) {
  ar(CEREAL_NVP(aaboxnode.element_id));
  ar(CEREAL_NVP(aaboxnode.segment_id));
  ar(CEREAL_NVP(aaboxnode.start));
  ar(CEREAL_NVP(aaboxnode.end));
}

template <typename Archive>
void serialize(Archive& ar, Map::AABoxTree& aaboxtree) {
  ar(CEREAL_NVP(aaboxtree.aabox_nodes));
  ar(CEREAL_NVP(aaboxtree.aabox_tree));
}

template <typename Archive>
void serialize(Archive& ar, Map& map) {
  ar(CEREAL_NVP(map.type_));
  ar(CEREAL_NVP(map.sub_type_));
  ar(CEREAL_NVP(map.is_on_highway_));
  ar(CEREAL_NVP(map.utm_to_cur_frame_));
  ar(CEREAL_NVP(map.lane_map_));
  ar(CEREAL_NVP(map.lanes_));
  ar(CEREAL_NVP(map.lane_boundary_map_));
  ar(CEREAL_NVP(map.road_boundary_map_));
  ar(CEREAL_NVP(map.clear_area_map_));
  ar(CEREAL_NVP(map.junction_map_));
  ar(CEREAL_NVP(map.crosswalk_map_));
  ar(CEREAL_NVP(map.stop_line_map_));
  ar(CEREAL_NVP(map.speed_bump_map_));
  ar(CEREAL_NVP(map.exp_trajectory_map_));
  ar(CEREAL_NVP(map.exp_traj_to_lane_map_));
  ar(CEREAL_NVP(map.junctions_));
  ar(CEREAL_NVP(map.traffic_light_status_map_));
  ar(CEREAL_NVP(map.timestamp_));
  ar(CEREAL_NVP(map.seq_));
  ar(CEREAL_NVP(map.route_));
  ar(CEREAL_NVP(map.section_map_));
  ar(CEREAL_NVP(map.route_section_seq_));
  ar(CEREAL_NVP(map.extend_section_seq_));
  ar(CEREAL_NVP(map.split_merge_shortdis_ids_));
  ar(CEREAL_NVP(map.v2_info_));
  ar(CEREAL_NVP(map.boundary_id_lane_id_map_));
  ar(CEREAL_NVP(map.feature_aabox_tree_));
  ar(CEREAL_NVP(map.thread_pool_));
}

}  // namespace ad_byd::planning

namespace st::mapping {

template <typename Archive>
void serialize(Archive& ar, st::mapping::LanePath& lane_path) {
  ar(CEREAL_NVP(lane_path.lane_path_data_));
  ar(CEREAL_NVP(lane_path.exp_traj_string_));
  ar(CEREAL_NVP(lane_path.split_topo_modified_string_));
  ar(CEREAL_NVP(lane_path.lane_end_s_));
  ar(CEREAL_NVP(lane_path.lane_lengths_));
  ar(CEREAL_NVP(lane_path.lane_seq_));
}

template <typename Archive>
void serialize(Archive& ar, LanePathData& lane_path_data) {
  ar(CEREAL_NVP(lane_path_data.start_fraction_));
  ar(CEREAL_NVP(lane_path_data.end_fraction_));
  ar(CEREAL_NVP(lane_path_data.lane_ids_));
  ar(CEREAL_NVP(lane_path_data.lane_path_in_forward_direction_));
}

template <typename Archive>
void serialize(Archive& ar, LanePoint& lane_point) {
  ar(CEREAL_NVP(lane_point.lane_id_));
  ar(CEREAL_NVP(lane_point.fraction_));
}

}  // namespace st::mapping

#endif