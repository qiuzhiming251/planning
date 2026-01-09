#ifndef ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_
#define ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_

#include <string>
#include <utility>
#include <valarray>
#include <vector>
#include <unordered_map>

#include "object_manager/st_inference/est_planner_output.h"
#include "decider/selector/common_feature.h"
#include "modules/msg/orin_msgs/map_event.pb.h"

namespace st {
namespace planning {
struct SelectorCostInput {
  std::string borrow_task_nudge_id = "";
  st::prediction::PredictedTrajectory borrow_task_nudge_traj;
  const SpacetimeTrajectoryManager* stm = nullptr;
  bool preview_in_intersection = false;
  bool* is_nearby_obs = nullptr;
  size_t select_lock_number = 0;
  bool has_lane_changed_intersection = false;
  bool is_ego_on_pref_lane = false;
  double ego_lane_dist_to_navi_end = DBL_MAX;
  std::unordered_map<int, int> task_id_lane_index_map;
  const byd::msg::orin::routing_map::MapEvent* map_event = nullptr;
  std::optional<bool> begin_route_change_left = std::nullopt;
  int last_selected_idx = -1;
  int begin_route_change_successive_count = 0;
  bool is_passed_split_lane_change = false;
  int overtake_lc_pause_successive_count = 0;
  bool is_all_lanes_pref = false;
  int valid_lane_num = 0;
  std::optional<bool> is_going_force_route_change_left = std::nullopt;
  int force_route_change_successive_count = 0;
};

struct TrajFeatureOutput {
  double progress_factor = 1.0;
  bool has_obvious_route_cost = false;
  bool is_perform_lane_change = false;
  bool lane_change_left = false;
  bool lane_change_for_road_speed_limit = false;
  bool lane_change_for_right_most_lane = false;
  bool lane_change_for_route_cost = false;
  bool lane_change_for_merge_lane = false;
  bool lane_change_for_moving_obj = false;
  bool lane_change_for_stationary_vehicle = false;
  bool lane_change_for_stalled_vehicle = false;
  bool lane_change_for_stationary_obj = false;
  bool lane_change_for_intersection_obs = false;
  bool lane_change_for_avoid_cones = false;  // Defensive
  bool lane_change_for_avoid_bus_lane = false;
  bool lane_change_for_navi_cost = false;
  bool lane_change_for_avoid_merge_area = false;
  bool cross_solid_boundary = false;
  bool has_progress_cost = false;
  std::optional<Vec2d> merge_point = std::nullopt;
  bool has_obvious_stalled_object = false;
  bool has_obvious_invade_risk = false;
  bool is_obvous_curvy_road = false;
  std::optional<double> opposite_lc_interval_secs = std::nullopt;
  bool lane_change_for_emergency = false;
  bool has_obvious_cutoff_cost = false;
  bool lane_change_for_length_cutoff = false;
  bool has_begin_route_change = false;
  bool uncomfortable_decel = false;
  double feasible_length_along_route = DBL_MAX;
  bool is_accel_traj_start = false;
  std::optional<bool> begin_route_change_left = std::nullopt;
  absl::flat_hash_set<std::string> lat_away_obj_ids;
  absl::flat_hash_set<std::string> traj_block_obj_ids;
  object_cost_map_type object_cost_map;
  int overtake_begin_frame = 0;
  bool cur_frame_has_begin_route_change = false;
  double min_a = 0.0;
  bool is_passed_split_lane_change = false;
  bool lane_change_for_normal_emergency_overtake = false;
  bool lane_change_for_most_emergency_overtake = false;
  int begin_route_change_successive_count = 0;
  bool defensive_driving_lc_behind_truck = false;
  std::optional<bool> is_going_force_route_change_left = std::nullopt;
  int force_route_change_successive_count = 0;
  std::string DebugString() const;
};

class CostFeatureBase {
 public:
  using CostVec = std::valarray<double>;

  explicit CostFeatureBase(std::string&& name,
                           std::vector<std::string>&& sub_names, bool is_common,
                           const SelectorCommonFeature* common_feature)
      : name_(std::move(name)),
        sub_names_(std::move(sub_names)),
        size_(sub_names_.size()),
        is_common_(is_common),
        common_feature_(common_feature) {}
  virtual ~CostFeatureBase() {}

  const std::string& name() const { return name_; }
  const std::vector<std::string>& sub_names() const { return sub_names_; }
  int size() const { return size_; }
  bool is_common() const { return is_common_; }
  bool is_static_feature() const { return is_static_feature_; }
  const SelectorCommonFeature* common_feature() const {
    return common_feature_;
  }
  // virtual absl::StatusOr<CostVec> ComputeCost(
  //     const EstPlannerOutput& planner_output,
  //     std::vector<std::string>* extra_info) const = 0;
  virtual absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const = 0;

 protected:
  bool is_static_feature_;

 private:
  std::string name_;
  std::vector<std::string> sub_names_;
  int size_;
  // True if the cost applies for all trajs;
  // false if only applies for comparing trajs from the same start lane.
  bool is_common_;
  const SelectorCommonFeature* common_feature_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_
