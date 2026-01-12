#pragma once

#include <optional>
#include <set>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/path_approx.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/speed/st_speed/speed_vector.h"

#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st::planning {

struct AccSpeedDeciderInput {
  std::string base_name;
  int trajectory_steps = 0;
  double plan_start_v = 0.0;       // m/s.
  double plan_start_a = 0.0;       // m/s^2.
  double user_speed_limit = 0.0;   // m/s.
  double max_allowed_speed = 0.0;  // m/s.
  bool is_standwait = false;
  double loaded_map_dist = 0.0;

  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const DiscretizedPath* path = nullptr;
  const SegmentMatcherKdtree* path_kd_tree = nullptr;
  const PathApprox* path_approx = nullptr;

  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const VehicleDriveParamsProto* vehicle_drive_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
};

struct AccSpeedDeciderOutput {
  SpeedLimitProvider speed_limit_provider;
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  SpeedVector sampling_dp_speed;
};

absl::StatusOr<AccSpeedDeciderOutput> RunAccSpeedDecider(
    const AccSpeedDeciderInput& input);

}  // namespace st::planning
