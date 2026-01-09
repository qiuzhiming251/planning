#ifndef ONBOARD_PLANNER_SELECTOR_COMMON_FEATURE_H_
#define ONBOARD_PLANNER_SELECTOR_COMMON_FEATURE_H_

#include <map>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/selector_cost_history.h"
#include "decider/selector/selector_util.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/msg/orin_msgs/map_event.pb.h"
namespace st::planning {

namespace ns_routing_map = byd::msg::orin::routing_map;
using NaviActionInfoType = byd::msg::orin::routing_map::NaviActionInfo;
using MapEventType = byd::msg::orin::routing_map::MapEvent;
using TaskIndex = int;

using TrafficInfoNotifyType = byd::msg::orin::routing_map::TrafficInfoNotify;

inline bool IsConstructionObject(ObjectType obj_type) {
  return obj_type == ObjectType::OT_CONE ||
         obj_type == ObjectType::OT_BARRIER ||
         obj_type == ObjectType::OT_WARNING_TRIANGLE ||
         obj_type == ObjectType::OT_ROW_OBSTACLES;
}

struct LeaderObjectInfo {
  std::string obj_id;
  double obj_s;
  double obj_v;
  ObjectType obj_type;
  bool is_stationary = false;
};

struct LaneConesInfo {
  int left_cones_count = 0;
  int right_cones_count = 0;
  int blocking_cones_count = 0;
  std::string left_invade_cone_id = "Invalid";
  std::string right_invade_cone_id = "Invalid";
  int left_cones_invade_count = 0;
  int right_cones_invade_count = 0;
  int left_attribute_satisfied_count = 0;
  int right_attribute_satisfied_count = 0;
  int lc_left_direction_cones_count = 0;
  int lc_right_direction_cones_count = 0;
};

struct InvadeStaticObjInfo {
  double obj_s_min;
  double obj_s_max;
  double obj_l_min;
  double obj_l_max;
  ObjectType obj_type;
  double boundary_left_l;           // +
  double boundary_right_l;          // -
  double left_remain_width = 0.0;   // +
  double right_remain_width = 0.0;  // +
  bool is_stalled = false;
};

struct LaneFeatureInfo {
  // Route info.
  double speed_limit = 0.0;
  bool is_keep_lane = false;
  bool is_only_min_lc_num = false;
  bool is_lc_follow_navi = false;
  bool is_lc_against_navi = false;
  int right_index = -1;
  int valid_lane_num = -1;
  int preivew_right_index = -1;
  int preview_valid_lane_num = -1;
  std::optional<int> lc_num;
  // Obstacle info
  absl::flat_hash_set<std::string> block_obj_ids;
  bool target_switched = true;
  std::optional<LeaderObjectInfo> nearest_leader;
  LaneConesInfo lane_cones_info;
  // Invade objs, hash key is 'object_id'
  std::map<std::string, InvadeStaticObjInfo> invade_static_obj_map;
};

// TODO(xiang): should be defined in or before scheduler, because it is the
// route details other than the tasks. Identical with `SectionInfo` but with the
// deferent usage.
struct MppSectionInfo {
  std::vector<uint64_t> section_ids;
  std::vector<double> section_lengths;
  std::vector<std::vector<ad_byd::planning::LaneConstPtr>> section_lanes;
  double start_s;
};

struct SelectorCommonFeature {
  // Traffic info.
  bool mapless = false;
  bool in_high_way = false;
  double time_since_last_red_light = 0.0;
  bool ego_in_tl_controlled_intersection = false;
  bool preview_in_high_way = false;
  // Selector info.
  double time_since_last_lane_change = 0.0;
  double time_since_last_passed_split = 0.0;
  bool is_valid_avoid_split_dis = false;
  double time_since_ego_leave_ramp = 0.0;
  // Intersection info.
  double length_before_intersection = 0.0;
  bool is_left_turn = false;
  bool is_right_turn = false;
  bool is_borrow_case = false;
  // Lane feature info.
  absl::flat_hash_map<TaskIndex, LaneFeatureInfo> lane_feature_infos;
  std::optional<double> min_split_angle;
  // function id (only used for CityNOA with map)
  st::Behavior_FunctionId func_id = st::Behavior_FunctionId_NONE;
  // bool is_scene_follow = false;
  // aggregte from different tasks (except lane change task)
  std::string nudge_id = "Invalid";
  LaneChangeStage last_selected_stage = LaneChangeStage::LCS_NONE;
  // member, SHOULD not be recalc it here.
  RoadHorizonInfo road_horizon_info;
  // navi action info : action + distance
  std::optional<NaviActionInfoType> next_navi_action_info;
  std::optional<NaviActionInfoType> next_non_straight_navi_action_info;
  TrafficInfoNotifyType traffic_jam_info;
  absl::Time plan_time;
  double ego_lane_dist_to_navi_end = DBL_MAX;
  double max_navi_dist_keep = DBL_MAX;
  double min_dist_to_merge = DBL_MAX;
  MppSectionInfo mpp_section;
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SELECTOR_COMMON_FEATURE_H_
