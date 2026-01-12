

#ifndef ONBOARD_PLANNER_SCENE_OBJECTS_REASONING_H_
#define ONBOARD_PLANNER_SCENE_OBJECTS_REASONING_H_
#include <vector>

#include "absl/status/statusor.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "object_manager/planner_object_manager.h"
namespace st::planning {

struct ObjectsReasoningOutput {
  std::vector<ObjectAnnotationProto> objects_annotation;
};

// Reasoning object annotation without structured road info(e.g. lane path).
absl::StatusOr<ObjectsReasoningOutput> RunObjectsReasoning(
    const PlannerObjectManager& object_mgr);
}  // namespace st::planning

#endif
