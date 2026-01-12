#ifndef SCENE_CONSTRUCTION_SCENE_IDENTIFICATION_H_
#define SCENE_CONSTRUCTION_SCENE_IDENTIFICATION_H_
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/obstacle.h"
#include "plan_common/path/path.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/planner_object_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st::planning {
using LaneConstPtr = ad_byd::planning::LaneConstPtr;
using LaneSequencePtr = ad_byd::planning::LaneSequencePtr;
using StationaryObstacle = ad_byd::planning::StationaryObstacle;
using SLBoundary = ad_byd::planning::SLBoundary;
using LaneSeqInfoPtr = ad_byd::planning::LaneSeqInfo;
struct ConstructionSceneIdentificationInput {
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const VehicleParamsProto* vehicle_param = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  // const ExternalCommandStatus* ext_cmd_status = nullptr;
  const std::optional<double> lcc_cruising_speed_limit = std::nullopt;
  const std::optional<Behavior_FunctionId> function_id = std::nullopt;
  const std::optional<int> tunnel_status = std::nullopt;
  const PlannerObjectManager& obj_mgr;
  const LaneSequencePtr target_lane_seq = nullptr;
  const LaneChangeStateProto* lane_change_state = nullptr;
};
struct BlockerSceneIdentificationInput {
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const VehicleParamsProto* vehicle_param = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  // const ExternalCommandStatus* ext_cmd_status = nullptr;
  const std::optional<double> lcc_cruising_speed_limit = std::nullopt;
  const std::optional<Behavior_FunctionId> function_id = std::nullopt;
  const std::optional<int> tunnel_status = std::nullopt;
  const PlannerObjectManager& obj_mgr;
  const LaneSequencePtr target_lane_seq = nullptr;
  const LaneChangeStateProto* lane_change_state = nullptr;
  const LaneSequencePtr target_lane_seq_blocker = nullptr;
};

bool RunConstructionSceneIdentification(
    const ConstructionSceneIdentificationInput& input);
bool RunBlockerSceneIdentification(
    const BlockerSceneIdentificationInput& blockerinput);
bool ConstructionBlockerLcUUTrigger(
    const ConstructionSceneIdentificationInput& input);
bool BlockerLcUUTrigger(const BlockerSceneIdentificationInput& blockerinput,
                        double dist_to_junction);
bool ClassifyObstaclesByPositionAndSetSLBoundary(
    const LaneSequencePtr& lane_seq, std::vector<StationaryObstacle>& fsd_obs,
    std::vector<StationaryObstacle>& current_lane_obstacles,
    const double start_point_s);
void ConvertStObstaclesToFsdObs(
    absl::Span<const PlannerObject* const> stationary_objects,
    std::vector<StationaryObstacle>& fsd_obs);
double GetLeaderCarSOffset(const LaneSequencePtr& lane_seq,
                           const ObjectVector<PlannerObject>& obstacles,
                           const double& start_offset);
}  // namespace st::planning

#endif  // SCENE_CONSTRUCTION_SCENE_IDENTIFICATION_H_