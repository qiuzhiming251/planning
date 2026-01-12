

#ifndef ST_PLANNING_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER
#define ST_PLANNING_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER

#include <string>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "planner/speed_optimizer/decider/st_boundary_modifier_util.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "planner/speed_optimizer/st_graph.h"

namespace st::planning {

// TODO: rename to StBoundaryLonModificationInfo.
struct StBoundaryModificationInfo {
  StBoundaryModifierProto::ModifierType modifier_type =
      StBoundaryModifierProto::UNKNOWN;
  StBoundaryProto::DecisionType decision = StBoundaryProto::UNKNOWN;
  bool is_decision_changed = false;
  // New trajectory accelerate at 'a' from 't'.
  std::vector<AccelPoint> accel_point_list;
};

struct PostStboundaryModifierInput {
  const StGraph* st_graph = nullptr;
  const DiscretizedPath* path = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
      modification_info_map = nullptr;
};

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryWithAccelPointList(
    absl::Span<const AccelPoint> accel_point_list,
    const SpacetimeObjectTrajectory& st_object);

void PostModifyStBoundaries(
    const PostStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects);

}  // namespace st::planning

#endif  // ST_PLANNING_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER
