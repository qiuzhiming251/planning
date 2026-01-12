

#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_BUILDER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_BUILDER_H_

#include <vector>

#include "absl/types/span.h"
#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "spacetime_object_trajectory.h"
#include "spacetime_planner_object_trajectories.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/util/decision_info.h"
//#include "plan_common/util/hmi_content_util.h"
//#include "object_manager/planner_object.h"

namespace st {
namespace planning {

struct SpacetimePlannerObjectTrajectoriesBuilderInput {
  const PlannerSemanticMapManager* psmm;
  const DrivePassage* passage;
  const PathSlBoundary* sl_boundary;
  const LaneChangeStateProto* lane_change_state;
  const VehicleGeometryParamsProto* veh_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  double st_planner_start_offset;
  const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj;
  absl::Span<const ConstraintProto::StopLineProto> stop_lines;
  const SpacetimePlannerObjectTrajectoriesParamsProto*
      spacetime_planner_object_trajectories_params;
  const NudgeObjectInfo* nudge_object_info = nullptr;
  const int plan_id = 0;
  const bool force_no_nudge;
};

SpacetimePlannerObjectTrajectories BuildSpacetimePlannerObjectTrajectories(
    const SpacetimePlannerObjectTrajectoriesBuilderInput& input,
    absl::Span<const SpacetimeObjectTrajectory> trajectories,
    const double spacetime_planner_trajectory_horizon,
    const double trajectory_time_step, const double default_half_lane_width,
    std::unordered_map<std::string, double>& truncated_traj_map);

}  // namespace planning
}  // namespace st

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_BUILDER_H_
