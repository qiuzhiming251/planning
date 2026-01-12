

#ifndef ONBOARD_PLANNER_EMERGENCY_STOP_H_
#define ONBOARD_PLANNER_EMERGENCY_STOP_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "plan_common/math/geometry/polygon2d.h"
//#include "planner_input.h"
#include "modules/cnoa_pnc/planning/proto/chassis.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/trajectory_point.h"

namespace st {
namespace planning {
namespace aeb {

struct EmergencyStopInfo {
  bool emergency_stop = false;
  Polygon2d risk_area;
  std::string object_id;
  st::ObjectType object_type;
};

// TODO: Refactor this function by creating another function to compute
// risk area.
// std::optional<EmergencyStopInfo> CheckEmergencyStopByCircularMotion(
//     const VehicleGeometryParamsProto& vehicle_geom,
//     const EmergencyStopParamsProto& emergency_stop_params,
//     const VehicleDriveParamsProto& vehicle_drive_params,
//     const PoseProto& vehicle_pose, const ObjectsProto& objects_proto,
//     const Chassis& chassis);

std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto& plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const EmergencyStopParamsProto& emergency_stop_params,
    const MotionConstraintParamsProto& motion_constraint_params);

// std::shared_ptr<const ObjectsProto> ExportObjectsForAEB(
//     const std::shared_ptr<const ObjectsProto>& real_objects,
//     const std::shared_ptr<const ObjectsProto>& virtual_objects);

// std::optional<EmergencyStopInfo> CheckEmergencyStop(
//     const PlannerParamsProto& planner_params, const PlannerInput& input);

std::vector<ApolloTrajectoryPointProto> GenerateStopTrajectory(
    double init_s, bool reset, bool forward, double max_deceleration,
    const MotionConstraintParamsProto& motion_constraint_params,
    const TrajectoryPoint& plan_start_traj_point,
    const std::vector<ApolloTrajectoryPointProto>& prev_trajectory);

}  // namespace aeb
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_EMERGENCY_STOP_H_
