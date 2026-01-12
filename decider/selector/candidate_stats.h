#ifndef ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_
#define ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_

#include <float.h>
#include <limits.h>

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "plan_common/planner_status.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/est_planner_output.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections_info.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "decider/selector/common_feature.h"

namespace st::planning {

// Sprinkler or sweeper.
struct SlowWorkingObject {
  std::string object_id;
  double object_s;  // min_s
  double object_l;  // center_l
  double object_len;
  double probability;
  int leader_count;
  bool is_large_vehicle = false;
  bool is_slow_working_object = false;
};

struct LaneSpeedInfo {
  double init_leader_speed;
  double max_leader_speed;
  double lane_speed_limit_by_leader;
  double lane_speed_limit;
  std::optional<std::string> block_obj;
  double nearest_obj_s;
};

struct StalledObjInfo {
  std::string stalled_obj_id;
  double stalled_obj_s;
  double punish_factor;
};
struct NudgeObjInfo {
  std::string nudge_obj_id;
  double object_s;  // min_s
  double object_l;  // center_l
  double object_len;
};

struct ProgressStats {
  ProgressStats(const SelectorCommonFeature& common_feature,
                const PlannerSemanticMapManager& psmm,
                const ApolloTrajectoryPointProto& plan_start_point,
                const VehicleGeometryParamsProto& vehicle_geom,
                const std::vector<PlannerStatus>& est_status,
                const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
                const std::vector<EstPlannerOutput>& results);

  double ego_v;
  double ego_width;
  absl::flat_hash_map<TaskIndex, LaneSpeedInfo> lane_speed_map;
  absl::flat_hash_map<TaskIndex, std::optional<SlowWorkingObject>>
      slow_working_objs_map;
  absl::flat_hash_map<TaskIndex, std::optional<NudgeObjInfo>> nudge_objs_map;
  double min_slow_working_object_s = DBL_MAX;
  double max_lane_speed = 0.0;
  double min_lane_speed = 33.4;  // 120 kmph.
  double max_init_leader_speed = 0.0;
  double lane_keep_init_leader_speed = 0.0;
};

struct RouteLookAheadStats {
  RouteLookAheadStats(
      const SelectorCommonFeature& common_feature,
      const PlannerSemanticMapManager& psmm,
      const ApolloTrajectoryPointProto& plan_start_point,
      const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
      const absl::flat_hash_set<std::string>& stalled_objects,
      const VehicleGeometryParamsProto& vehicle_geom,
      const std::vector<PlannerStatus>& est_status,
      const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
      const std::vector<EstPlannerOutput>& results);

  absl::flat_hash_map<TaskIndex, double> len_before_intersection_map,
      len_before_merge_lane_map;
  absl::flat_hash_map<TaskIndex, bool> is_right_most_lane_map;
  absl::flat_hash_map<TaskIndex, std::optional<StalledObjInfo>>
      front_stalled_obj_map;
  absl::flat_hash_map<TaskIndex, double> len_along_route_map,
      raw_len_along_route_map;
  absl::flat_hash_map<TaskIndex, bool> has_cross_curb_map;

  bool enable_discourage_right_most_cost = false;
  bool enable_encourage_right_most_cost = false;
  double max_length_along_route = 0.0;
  double min_length_along_route = DBL_MAX;
  double traffic_congestion_factor = 0.0;
  int min_lc_num = INT_MAX;
  int max_lc_num = 0;
  bool is_left_turn = false;
  bool is_right_turn = false;
  double max_length_before_merge_lane = -DBL_MAX;

  ad_byd::planning::NaviPosition navi_sart = {0, -1.0};
};
// ----------------------------------------------------------------

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_
