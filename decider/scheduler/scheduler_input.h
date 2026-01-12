

#ifndef ST_PLANNING_SCHEDULER_SCHEDULER_INPUT
#define ST_PLANNING_SCHEDULER_SCHEDULER_INPUT

#include <memory>
#include <string>
#include <vector>

#include "plan_common/maps/lane_path_info.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "object_manager/object_history.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections_info.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
namespace st::planning {
struct MultiTasksSchedulerInput {
  const PlannerSemanticMapManager* psmm;
  const VehicleGeometryParamsProto* vehicle_geom;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const PlannerObjectManager* obj_mgr;
  const ObjectHistoryManager* obj_history_mgr;
  const std::vector<LanePathInfo>* lane_path_infos;
  double planning_horizon;
  const mapping::LanePoint* destination;
  // const TrafficLightInfoMap* tl_info_map;
  bool prev_smooth_state = false;
  const ApolloTrajectoryPointProto* plan_start_point;
  const mapping::LanePoint* station_anchor;
  double start_route_s;
  const SmoothedReferenceLineResultMap* smooth_result_map;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const mapping::LanePath* prev_lane_path_before_lc_from_start = nullptr;
  const mapping::LanePath* preferred_lane_path = nullptr;
  const LaneChangeStateProto* prev_lc_state = nullptr;
  const RouteNaviInfo* route_navi_info = nullptr;
  std::optional<double> cruising_speed_limit = std::nullopt;
  std::optional<double> target_lane_speed_limit = std::nullopt;
  bool planner_is_l4_mode = true;
  Vec2d ego_pos;
  bool miss_navi_scenario = false;
  bool continuous_lc_scenario = false;
  // const ExternalCommandStatus* ext_cmd_status = nullptr;
  const bool is_navi;
  const st::DriverAction::LaneChangeCommand lc_cmd_state =
      DriverAction::LC_CMD_NONE;
  const Behavior* behavior = nullptr;
  std::string leading_id = "";
  ad_byd::planning::PushDirection lc_push_dir =
      ad_byd::planning::PushDirection::Push_None;
  std::optional<bool> is_going_force_route_change_left = std::nullopt;
  const PushStatusProto* pre_push_status = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const PausePushSavedOffsetProto* saved_offset = nullptr;
  const ad_byd::planning::ConstructionInfo* construction_info = nullptr;
  const LaneChangeSafetyInfo* pre_lane_change_safety_info = nullptr;
  const ad_byd::planning::BehaviorCommand intention_dir =
      ad_byd::planning::BehaviorCommand::Command_Invalid;
};

}  // namespace st::planning

#endif  // ST_PLANNING_SCHEDULER_SCHEDULER_INPUT
