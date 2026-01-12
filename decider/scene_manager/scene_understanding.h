

#ifndef ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_H_
#define ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/map_def.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "object_manager/object_history.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections.h"

namespace st::planning {

struct SceneReasoningInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const ObjectsPredictionProto* prediction = nullptr;
  const ad_byd::planning::TrafficLightStatusMap* tl_info_map = nullptr;
  // The lane path must in order, from left to right.
  const std::vector<mapping::LanePath>* lane_paths = nullptr;
  // const SensorFovsProto* sensor_fovs = nullptr;
  // const RouteSections* route_sections = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
};

struct SceneReasoningOutput {
  SceneOutputProto scene_output_proto;
  std::optional<double> distance_to_roadblock = std::nullopt;
};

absl::StatusOr<SceneReasoningOutput> RunSceneReasoning(
    const SceneReasoningInput& input, ThreadPool* thread_pool,
    ObjectHistoryManager& obj_his_manager);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SCENE_SCENE_UNDERSTAND_H_
