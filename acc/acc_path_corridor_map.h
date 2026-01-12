#pragma once

#include "acc/acc_path_corridor.h"
#include "acc/acc_path_corridor_map.h"
#include "acc/acc_util.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
namespace st::planning {

AccPathCorridor BuildAccPathCorridorWithMap(
    const PlannerSemanticMapManager& psmm,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const PoseProto& pose, const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, double corridor_step_s,
    double total_preview_time, std::optional<double> cruising_speed_limit);

}
