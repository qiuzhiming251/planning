

#ifndef ONBOARD_PLANNER_SCENE_TRAFFIC_FLOW_REASONING_H_
#define ONBOARD_PLANNER_SCENE_TRAFFIC_FLOW_REASONING_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/map_def.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "object_manager/object_history.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "decider/scene_manager/scene_understanding_output.h"

namespace st::planning {

struct TrafficFlowReasoningInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const ObjectsPredictionProto* prediction = nullptr;
  const std::vector<mapping::LanePath>* lane_paths = nullptr;
  const ad_byd::planning::TrafficLightStatusMap* tl_info_map = nullptr;
  // const SensorFovsProto* sensor_fovs = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
};

absl::StatusOr<TrafficFlowReasoningOutput> RunTrafficFlowReasoning(
    const TrafficFlowReasoningInput& input, ThreadPool* thread_pool,
    ObjectHistoryManager& obj_manager);
}  // namespace st::planning

#endif
