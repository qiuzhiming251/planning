

#ifndef ONBOARD_PLANNER_SPEED_STANDSTILL_DISTANCE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_STANDSTILL_DISTANCE_DECIDER_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "planner/speed_optimizer/cipv_object_info.h"

namespace st {
namespace planning {

struct StandstillDistanceDeciderInput {
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
  const absl::flat_hash_set<std::string>* stalled_object_ids = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const mapping::LanePath* lane_path = nullptr;  // Could be nullptr.
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  double plan_start_v = 0.0;
};

// Set standstill distance for st-boundary according to its type and some
// special rules.
void DecideStandstillDistanceForStBoundary(
    const StandstillDistanceDeciderInput& input,
    StBoundaryWithDecision* st_boundary_wd, bool is_open_gap,
    const CipvObjectInfo* cipv_object_info);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_STANDSTILL_DISTANCE_DECIDER_H_
