

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_

#include <map>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

#include "plan_common/type_def.h"
#include "plan_common/drive_passage.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/trajectory_optimizer_state.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"

namespace st::planning {

struct TrajectoryOptimizerInput {
  absl::Span<const ApolloTrajectoryPointProto> trajectory;
  NudgeInfos nudge_info;
  absl::Span<const ApolloTrajectoryPointProto> previous_trajectory;

  // std::nullopt when first time running trajectory optimizer.
  std::optional<TrajectoryOptimizerState> trajectory_optimizer_state;

  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_mgr = nullptr;
  ApolloTrajectoryPointProto plan_start_point;
  absl::Time plan_start_time;
  int plan_id = 0;
  bool borrow_lane = false;
  absl::Span<const ApolloTrajectoryPointProto> captain_trajectory;
  LaneChangeStage lc_stage;
  PushState push_dir = PushState::NONE_PUSH;
  const Behavior* behavior = nullptr;
  // Params.
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;
  const VehicleGeometryParamsProto* veh_geo_params = nullptr;
  const VehicleDriveParamsProto* veh_drive_params = nullptr;
  const NudgeObjectInfo* nudge_object_info = nullptr;
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_
