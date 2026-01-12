

#ifndef ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_
#define ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_

#include "plan_common/async/thread_pool.h"
#include "plan_common/path_sl_boundary.h"
#include "planner_object_manager.h"
#include "spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st {
namespace planning {
struct SpacetimeTrajectoryManagerBuilderInput {
  const DrivePassage* passage;
  const PathSlBoundary* sl_boundary;
  const PlannerObjectManager* obj_mgr;
  const st::LaneChangeStateProto* lc_state;
  const bool is_on_highway;
  const Box2d ego_box;
  const int plan_id;
  const PlannerSemanticMapManager* psmm;
};

SpacetimeTrajectoryManager BuildSpacetimeTrajectoryManager(
    const SpacetimeTrajectoryManagerBuilderInput& input,
    ThreadPool* thread_pool);

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_
