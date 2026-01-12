

#ifndef ST_PLANNING_PLANNER_MAIN_LOOP_INTERNAL
#define ST_PLANNING_PLANNER_MAIN_LOOP_INTERNAL

#include <memory>
#include <tuple>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "plan_common/plan_start_point_info.h"

// #include "modules/cnoa_pnc/planning/proto/chassis.pb.h"
// #include "decision/tl_info.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/math/vec.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "node_manager/task_runner/planner_state.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections.h"
#include "router/route_sections_util.h"

#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/alc.pb.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_validation.pb.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

// std::shared_ptr<const ObjectsProto> GetAllObjects(
//     const std::shared_ptr<const ObjectsProto>& real_objects,
//     const std::shared_ptr<const ObjectsProto>& virtual_objects);

void FillTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto>& planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto>& past_points,
    const mapping::LanePath& target_lane_path_from_current,
    const LaneChangeStateProto& lane_change_state, TurnSignal turn_signal,
    // bool is_aeb_triggered,
    const TrajectoryValidationResultProto& validate_result,
    TrajectoryProto* trajectory);

// void ConvertTrajectoryToGlobalCoordinates(
//     const CoordinateConverter& coordinate_converter,
//     const TrajectoryProto& trajectory,
//     std::vector<PlannerState::PosePoint>* previous_trajectory_global,
//     std::vector<PlannerState::PosePoint>* previous_past_trajectory_global);

// void ConvertPreviousTrajectoryToCurrentSmoothLateral(
//     absl::Time predicted_plan_time,
//     const CoordinateConverter& coordinate_converter,
//     std::vector<PathPoint> previous_path,
//     const std::vector<PlannerState::PosePoint>& previous_trajectory_global,
//     const std::vector<PlannerState::PosePoint>&
//     previous_past_trajectory_global, TrajectoryProto* previous_trajectory);

// void ReportCandidateTrafficLightInfo(
//     const TrafficLightInfoMap& traffic_light_map, PlannerDebugProto* debug);

// void ReportSelectedTrafficLightInfo(
//     const TrafficLightInfoMap& traffic_light_map,
//     const mapping::LanePath& lane_path, TrafficLightInfoProto* proto);

// Prev traj includes past points. Prev traj will never be empty once the
// first successful planner iteration completes.
std::vector<ApolloTrajectoryPointProto> CreatePreviousTrajectory(
    absl::Time plan_time, const TrajectoryProto& previous_trajectory,
    const MotionConstraintParamsProto& motion_constraint_params, bool reset);

PlanStartPointInfo ComputeAccPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state, bool rerouted, bool aeb,
    double front_wheel_angle,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool override);

PlanStartPointInfo ComputeEstPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state, bool rerouted, bool aeb,
    double front_wheel_angle,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool override,
    Behavior_FunctionId function_id);

absl::Duration GetStPathPlanLookAheadTime(
    const PlanStartPointInfo& plan_start_point_info, const PoseProto& pose,
    absl::Duration planned_look_ahead_time,
    const TrajectoryProto& previous_trajectory);

StPathPlanStartPointInfo GetStPathPlanStartPointInfo(
    const absl::Duration look_ahead_time,
    const PlanStartPointInfo& plan_start_point_info,
    const TrajectoryProto& previous_trajectory,
    std::optional<double> trajectory_optimizer_time_step,
    std::optional<absl::Time> last_st_path_plan_start_time);

absl::StatusOr<mapping::LanePath> FindPreferredLanePathFromTeleop(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start,
    const RouteNaviInfo& route_navi_info, mapping::ElementId ego_proj_lane_id,
    DriverAction::LaneChangeCommand lc_cmd);

// // first: route sections from pos.
// // second: route sections including behind parts.
absl::StatusOr<std::tuple<RouteSections, RouteSections, PointOnRouteSections>>
ProjectPointToRouteSections(const PlannerSemanticMapManager& psmm,
                            const RouteSections& route_sections,
                            const Vec2d& pos, double projection_range,
                            double keep_behind_length);

bool CheckIfAllowCancel(const ApolloTrajectoryPointProto& plan_start_point,
                        const VehicleGeometryParamsProto& vehicle_geometry,
                        const Vec2d& ego_pos,
                        st::mapping::LanePath* preferred_lane_path,
                        const double& dist_buffer);

void HandleManualLcCommand(
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geometry,
    const PlannerSemanticMapManager& psmm,
    st::DriverAction::LaneChangeCommand new_lc_cmd,
    const st::mapping::LanePath& prev_lp_before_lc,
    const LaneChangeStateProto& prev_lc_state, const Vec2d& ego_pos,
    const double& ego_theta, st::mapping::LanePath* preferred_lane_path,
    ALCState* alc_state, st::DriverAction::LaneChangeCommand* lc_cmd_state,
    st::planning::LcFeasibility* lc_unable_reason, bool* if_cancel_lc,
    bool* if_allow_cancel_lc, int last_manual_lc_time,
    LaneChangeReason last_lc_reason,
    std::vector<ad_byd::planning::LaneSequencePtr> candidate_lane_seqs,
    const ad_byd::planning::EIEChoiceType& eie_choice_type);

}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_PLANNER_MAIN_LOOP_INTERNAL
