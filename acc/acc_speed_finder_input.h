#pragma once

#include <string>
#include <vector>

#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_limit.h"
#include "plan_common/speed/st_speed/speed_vector.h"

#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st::planning {

struct AccSpeedFinderInput {
  std::string base_name;
  double plan_start_v = 0.0;
  double plan_start_a = 0.0;
  double user_speed_limit = 0.0;
  bool is_standwait = false;
  double loaded_map_dist = 0.0;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const DiscretizedPath* path = nullptr;
  const SpeedLimit* speed_limit = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const VehicleDriveParamsProto* vehicle_drive_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
};

}  // namespace st::planning
