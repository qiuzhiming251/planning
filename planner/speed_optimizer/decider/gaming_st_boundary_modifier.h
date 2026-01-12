

#ifndef ST_PLANNING_SPEED_DECIDER_GAMING_ST_BOUNDARY_MODIFIER
#define ST_PLANNING_SPEED_DECIDER_GAMING_ST_BOUNDARY_MODIFIER

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
#include "alternative_gaming/speed_gaming/speed_gaming_decider.h"

namespace st::planning {

struct GamingStboundaryModifierInput {
  const StGraph* st_graph = nullptr;
  const SpeedGamingOutput* speed_gaming_output = nullptr;
  const DiscretizedPath* path = nullptr;
};

void GamingModifyStBoundaries(
    const GamingStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects);

void GamingModifyStBoundariesAfterDP(
    const GamingStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    const int& plan_id);

bool CheckDecisionValid(const StBoundaryRef& st_boundary,
                        const SpeedVector& gaming_ego_speed_profile,
                        const int& plan_id);
}  // namespace st::planning

#endif  // ST_PLANNING_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER
