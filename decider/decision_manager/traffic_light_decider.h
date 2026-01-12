

#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
//#include "decider/decision_manager/tl_info.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/speed/st_speed/speed_profile.h"
#include "plan_common/drive_passage.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"

namespace st {
namespace planning {

struct TrafficLightDeciderOutput {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  std::vector<ConstraintProto::SpeedProfileProto> speed_profiles;
  TrafficLightDeciderStateProto traffic_light_decider_state;
  TrafficLightIndicationInfoProto tl_ind_info;
};

// absl::StatusOr<TrafficLightDeciderOutput> BuildTrafficLightConstraints(
//     const PlannerSemanticMapManager& psmm,
//     const st::VehicleGeometryParamsProto& vehicle_geometry_params,
//     const ApolloTrajectoryPointProto& plan_start_point,
//     const DrivePassage& passage, const mapping::LanePath&
//     lane_path_from_start, double s_offset, const TrafficLightInfoMap&
//     tl_info_map, const SpeedProfile& preliminary_speed_profile, const
//     TrafficLightDeciderStateProto& decider_state);

absl::StatusOr<TrafficLightDeciderOutput> BuildTrafficLightConstraints(
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    const ad_byd::planning::TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const TrafficLightDeciderStateProto& tld_state, bool enable_tl_ok_btn,
    bool override_passable, const DecisionConstraintConfigProto& config,
    const Behavior_FunctionId& map_func_id,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    bool traffic_light_fun_enable, bool& need_prebrake_for_lka,
    const absl::Time plan_start_time);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_
