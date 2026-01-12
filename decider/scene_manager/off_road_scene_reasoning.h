

#ifndef ONBOARD_PLANNER_SCENE_OFFROAD_SCENE_REASONING_H_
#define ONBOARD_PLANNER_SCENE_OFFROAD_SCENE_REASONING_H_

#include "absl/status/statusor.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "object_manager/planner_object_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
namespace st::planning {

struct OffRoadSceneReasoningInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const PlannerObjectManager* object_mgr = nullptr;
};

// Scene reasoning on unstructured road.
absl::StatusOr<SceneOutputProto> RunOffRoadSceneReasoning(
    const OffRoadSceneReasoningInput& input);

}  // namespace st::planning
#endif
