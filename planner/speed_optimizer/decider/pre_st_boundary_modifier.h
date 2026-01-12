

#ifndef ONBOARD_PLANNER_SPEED_DECIDER_PRE_ST_BOUNDARY_MODIFIER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_PRE_ST_BOUNDARY_MODIFIER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/path_sl_boundary.h"
#include "planner/speed_optimizer/cipv_object_info.h"
#include "planner/speed_optimizer/interactive_laterally.h"
#include "planner/speed_optimizer/path_semantic_analyzer.h"
#include "planner/speed_optimizer/st_graph.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
namespace st {
namespace planning {

struct PreStboundaryModifierInput {
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const StGraph* st_graph = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  double current_v = 0.0;
  double current_a = 0.0;
  const DiscretizedPath* path = nullptr;
  const absl::flat_hash_map<std::string, StBoundaryLatModificationInfo>*
      lat_modification_info_map = nullptr;
  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const CipvObjectInfo* cipv_object_info = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_objs = nullptr;
};

// Modify prediction according to st-boundary overlap info prior to speed
// decision, and generate new st-boundaries. If a prediction trajectory is
// modified in this stage, it won't be modified in the following speed
// decision.

void PreModifyStBoundaries(
    const PreStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    OpenLoopSpeedLimit* open_loop_speed_limit);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_PRE_ST_BOUNDARY_MODIFIER_H_
