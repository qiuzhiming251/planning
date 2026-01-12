#include "decider/selector/cost_feature_base.h"

#include <sstream>

#include "absl/strings/str_format.h"

namespace st::planning {
std::string TrajFeatureOutput::DebugString() const {
  std::stringstream ss;
  ss << "{ progress_factor: " << absl::StrFormat("%.2f", progress_factor);
  ss << ", has_obvious_route_cost: " << has_obvious_route_cost;
  ss << ", is_perform_lane_change: " << is_perform_lane_change;
  ss << ", lane_change_left: " << lane_change_left;
  ss << ", lane_change_for_road_speed_limit: "
     << lane_change_for_road_speed_limit;
  ss << ", lane_change_for_right_most_lane: "
     << lane_change_for_right_most_lane;
  ss << ", lane_change_for_route_cost: " << lane_change_for_route_cost;
  ss << ", lane_change_for_merge_lane: " << lane_change_for_merge_lane;
  ss << ", lane_change_for_moving_obj:" << lane_change_for_moving_obj;
  ss << ", lane_change_for_stationary_vehicle: "
     << lane_change_for_stationary_vehicle;
  ss << ", lane_change_for_stalled_vehicle: "
     << lane_change_for_stalled_vehicle;
  ss << ", lane_change_for_stationary_obj: " << lane_change_for_stationary_obj;
  ss << ", lane_change_for_intersection_obs: "
     << lane_change_for_intersection_obs;
  ss << ", cross_solid_boundary: " << cross_solid_boundary;
  ss << ", has_obvious_cutoff_cost: " << has_obvious_cutoff_cost;
  ss << ", lane_change_for_length_cutoff: " << lane_change_for_length_cutoff;
  if (merge_point.has_value()) {
    ss << ", merge_point: ()";
  } else {
    ss << ", merge_point: "
       << absl::StrFormat("(%.2f,%.2f)", merge_point->x(), merge_point->y());
  }
  ss << ", has_obvious_stalled_object: " << has_obvious_stalled_object;
  ss << ", has_obvious_invade_risk: " << has_obvious_invade_risk;
  ss << ", has_begin_route_change: " << has_begin_route_change;
  ss << ", uncomfortable_decel: " << uncomfortable_decel;
  ss << ", is_accel_traj_start: " << is_accel_traj_start;
  ss << ", lat_away_obj_ids: " << absl::StrJoin(lat_away_obj_ids, ",");
  ss << ", traj_block_obj_ids: " << absl::StrJoin(traj_block_obj_ids, ",");
  ss << ", min_a: " << absl::StrFormat("%.2f", min_a);
  ss << ", is_passed_split_lane_change: " << is_passed_split_lane_change;
  if (is_going_force_route_change_left.has_value()) {
    ss << ", is_going_force_route_change_left: "
       << is_going_force_route_change_left.value();
  }
  ss << ", force_route_change_successive_count: "
     << force_route_change_successive_count;
  ss << "}";
  return ss.str();
}
}  // namespace st::planning
