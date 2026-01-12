

#ifndef ST_PLANNING_SCENE_OCCLUDED_INTERACTION_LANE_REASONING
#define ST_PLANNING_SCENE_OCCLUDED_INTERACTION_LANE_REASONING

#include <vector>

//#include "perception/lidar_pipeline/sensor_fov/sensor_fov.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections.h"

namespace st::planning {

std::vector<InferredObjectProto> InferOccludedObjectsOnInteractionLanes(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections);

}  // namespace st::planning

#endif  // ST_PLANNING_SCENE_OCCLUDED_INTERACTION_LANE_REASONING
