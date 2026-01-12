#pragma once

#include <string>
#include <utility>
#include <vector>

#include "acc/acc_path_corridor.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"

namespace st::planning {

std::vector<std::string> SelectAccTarget(
    const AccPathCorridor& path_corridor,
    const SpacetimeTrajectoryManager& traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point);

std::unique_ptr<SpacetimeTrajectoryManager> BuildSpacetimeTrajectoryManager(
    const std::vector<std::string>& considered_targets_id,
    const SpacetimeTrajectoryManager& traj_mgr);

}  // namespace st::planning
