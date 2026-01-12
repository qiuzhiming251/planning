

#include <utility>
#include <vector>

#include "decider/scene_manager/off_road_scene_reasoning.h"

//#include "global/trace.h"
//#include "lite/check.h"
#include "decider/scene_manager/objects_reasoning.h"
#include "plan_common/util/status_macros.h"

namespace st::planning {

absl::StatusOr<SceneOutputProto> RunOffRoadSceneReasoning(
    const OffRoadSceneReasoningInput& input) {
  SceneOutputProto output;
  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.object_mgr);

  const auto& object_mgr = *input.object_mgr;

  ASSIGN_OR_RETURN(auto objects_reasoning_output,
                   RunObjectsReasoning(object_mgr));

  output.mutable_objects_annotation()->Reserve(
      objects_reasoning_output.objects_annotation.size());
  for (auto& object_annotation : objects_reasoning_output.objects_annotation) {
    *output.add_objects_annotation() = std::move(object_annotation);
  }

  return output;
}
}  // namespace st::planning
