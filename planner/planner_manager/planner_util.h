

#ifndef ONBOARD_PLANNER_PLANNER_UTIL_H_
#define ONBOARD_PLANNER_PLANNER_UTIL_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/time/time.h"

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/trajectory_point.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/av_history.h"
namespace st {
namespace planning {

////////////////////////////////////////////////////////////////////////////////
// Vehicle kinematics related.
double ComputeLongitudinalJerk(const TrajectoryPoint& traj_point);
double ComputeLateralAcceleration(const TrajectoryPoint& traj_point);
double ComputeLateralJerk(const TrajectoryPoint& traj_point);

////////////////////////////////////////////////////////////////////////////////
// Semantic map and lane path related.

// Preprocess semantic map modifier proto. Convert it to
// PlannerSemanticMapModification data structure. Then it can be used to
// initialize planner semantic map manager directly.
// TODO: Delete smm v1 related later.
// PlannerSemanticMapModification CreateSemanticMapModification(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const mapping::SemanticMapModifierProto& modifier);

// PlannerSemanticMapModification CreateSemanticMapModification(
//     const ad_byd::planning::Map& semantic_map_manager,
//     const mapping::SemanticMapModifierProto& modifier);

// mapping::SemanticMapModifierProto PlannerSemanticMapModificationToProto(
//     const PlannerSemanticMapModification& modifier);

// Copied from planner module.
std::vector<ApolloTrajectoryPointProto> CreatePastPointsList(
    absl::Time plan_time, const TrajectoryProto& prev_traj, bool reset,
    int max_past_point_num);

std::vector<ApolloTrajectoryPointProto> CreateAccPastPointsFromAvHistory(
    const TrajectoryProto& prev_traj, const AvHistory& av_history,
    const ApolloTrajectoryPointProto& plan_start_point, absl::Time plan_time);

ApolloTrajectoryPointProto ComputePlanStartPointAfterReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool is_forward_task);

ApolloTrajectoryPointProto ComputePlanStartPointAfterLateralReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

ApolloTrajectoryPointProto ComputeAccPlanStartPointAfterLateralReset(
    const ApolloTrajectoryPointProto& prev_planned_now_point,
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params, double t_diff);

ApolloTrajectoryPointProto
ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
    const TrajectoryProto& prev_traj, const PoseProto& pose,
    double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

SelectorParamsProto LoadSelectorParamsFromFile(const std::string& file_address);

void FillDecisionConstraintDebugInfo(const ConstraintManager& constraint_mgr,
                                     ConstraintProto* constraint);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_PLANNER_UTIL_H_
