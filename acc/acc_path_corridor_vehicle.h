#pragma once

#include "acc/acc_path_corridor.h"
#include "acc/acc_util.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/path_sl_boundary.h"

namespace st::planning {
AccPathCorridor BuildAccPathCorridorWithoutMap(
    const PoseProto& pose, const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, double corridor_step_s,
    double total_preview_time, double front_wheel_angle, double av_kappa);
}
