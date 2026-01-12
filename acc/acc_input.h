#pragma once

#include <optional>
#include <string>
#include <vector>

#include "acc/acc_path_corridor.h"
// TODO: Move av_history to plan_common dir.
// #include "acc/av_history.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/av_history.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
#include "modules/cnoa_pnc/planning/proto/acc.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change_type.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st::planning {
struct AccInput {
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const VehicleDriveParamsProto* vehicle_drive_params = nullptr;
  const AccTaskParamsProto* acc_params = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  const PoseProto* pose = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const PlanStartPointInfo* start_point_info = nullptr;
  // PlannerState previous_trajectory
  const TrajectoryProto* prev_trajectory = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  // The same as lcc cruising_speed_limit
  std::optional<double> cruising_speed_limit = std::nullopt;
  LaneChangeType prev_lane_change_type = LaneChangeType::TYPE_NO_CHANGE;
  double front_wheel_angle = 0.0;
  const AvHistory* av_context = nullptr;
  bool acc_standwait = false;
  bool curve_speed_limit_allowed = true;
  std::optional<double> acceleration_request = std::nullopt;  // m/s^2
  std::string DebugString() const;
};
}  // namespace st::planning
