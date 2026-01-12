

#ifndef ST_PLANNING_SCENE_OCCLUDED_OBJECTS_REASONING
#define ST_PLANNING_SCENE_OCCLUDED_OBJECTS_REASONING

#include <vector>

#include "absl/status/statusor.h"

//#include "perception/lidar_pipeline/sensor_fov/sensor_fov.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections.h"

namespace st::planning {

struct OccludedObjectsReasoningInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  // Sensor fov is a necessary condition to reason occluded objects.
  // const sensor_fov::SensorFov* sensor_fov = nullptr;
  // Collect crosswalks along route sections.
  const RouteSections* route_sections = nullptr;
};

absl::StatusOr<std::vector<InferredObjectProto>> RunOccludedObjectsReasoning(
    const OccludedObjectsReasoningInput& input);

}  // namespace st::planning
#endif  // ST_PLANNING_SCENE_OCCLUDED_OBJECTS_REASONING
