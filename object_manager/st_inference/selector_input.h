

#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/msg/orin_msgs/map_event.pb.h"

#include "plan_common/selector_state.h"
#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections_info.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st::planning {
using LaneSequencePtr = ad_byd::planning::LaneSequencePtr;
struct SelectorFlags {
  bool planner_enable_selector_scoring_net = false;
  double planner_allow_lc_time_after_activate_selector = 2.0;
  double planner_allow_lc_time_after_give_up_lc = 5.0;
  double planner_allow_opposite_lc_time_after_paddle_lc = 30.0;
  int planner_begin_radical_lane_change_frame = 1;
  double planner_max_allow_lc_time_before_give_up = 15.0;
  double planner_miss_navi_length = 50.0;
  bool planner_dumping_selector_features = false;
  int planner_begin_lane_change_frame = 1;
  int planner_begin_signal_frame = 1;
  int planner_begin_signal_frame_city_noa = 1;
  bool planner_enable_lane_change_in_intersection = false;
  bool planner_enable_turn_light_when_open_gap = false;
  bool planner_enable_cross_solid_boundary = true;
  LaneChangeStyle planner_lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  bool planner_need_to_lane_change_confirmation = false;
  bool planner_is_bus_model = false;
  bool planner_is_l4_mode = true;
  int planner_abort_lane_change_frame = 4;  // todo(xxx) add it into gflags
  int planner_fast_abort_lane_change_frame = 2;
  int planner_begin_lane_change_frame_progress = 20;
  int planner_begin_lane_change_frame_progress_city_noa = 5;
  int planner_begin_change_best_lk_trajectory_frame_city_noa = 2;
  int planner_begin_change_best_lk_trajectory_frame_highway_noa = 3;
  int planner_begin_emergency_frame = 1;
  int planner_begin_cones_frame = 3;
  int planner_begin_normal_emergency_overtake_frame_highway_noa = 4;
  int planner_begin_most_emergency_overtake_frame_highway_noa = 2;
};

struct SelectorInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const RouteSectionsInfo* sections_info = nullptr;
  const mapping::LanePath* prev_lane_path_from_current = nullptr;
  st::LaneChangeStage pre_lc_stage;
  const std::vector<ApolloTrajectoryPointProto>* prev_traj = nullptr;
  const MotionConstraintParamsProto* motion_constraints = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const RouteNaviInfo* route_navi_info = nullptr;
  const absl::flat_hash_set<mapping::ElementId>* avoid_lanes = nullptr;
  const absl::Time plan_time;
  const std::optional<bool> alc_confirmation;
  // const PredictionDebugProto* prediction_debug;
  // const ml::ContextFeature* context_feature;
  // const ModelPool* planner_model_pool;
  const SelectorState* selector_state = nullptr;
  const SelectorFlags* selector_flags = nullptr;
  const SelectorParamsProto* config = nullptr;
  const PlannerParamsProto* planner_config = nullptr;
  const std::optional<double> cruising_speed_limit;
  const std::optional<double> target_lane_speed_limit;
  // driving style
  const int driving_style_gear;
  const bool is_open_gap;
  const byd::msg::orin::routing_map::MapEvent* map_event = nullptr;
  std::optional<bool> begin_route_change_left = std::nullopt;
  std::optional<bool> is_going_force_route_change_left = std::nullopt;
  std::set<uint64_t> navi_start_lanes;
  bool icc_lc_enable;  // false, true if has merge in icc
};

struct DeviateNaviInput {
  const PlannerSemanticMapManager* psmm;
  const st::Behavior_FunctionId func_id;
  const st::LaneChangeStage& pre_lc_stage;
  const VehicleGeometryParamsProto* vehicle_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  const LaneSequencePtr& pre_target_laneseq;
  const LaneSequencePtr& pre_target_laneseq_before_lc;
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_
