

#ifndef AD_BYD_PLANNING_NODES_PLANNER_CITY_MSG_PROXY_H
#define AD_BYD_PLANNING_NODES_PLANNER_CITY_MSG_PROXY_H

#include <memory>
#include <vector>

#include "json/json.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/localization.pb.h"
#include "node_manager/msg_adapter/msg_manager.h"
#include "plan_common/input_frame.h"
#include "plan_common/planner_status.h"
#include "plan_common/type_def.h"
#include "decider/scene_manager/scene_understanding.h"
#include "node_manager/task_runner/multi_tasks_cruise_planner_input.h"
#include "node_manager/task_runner/planner_state.h"

namespace st {
namespace planning {

struct PredictionResult {
  ObjectsProto objects_proto;
  ObjectsPredictionProto objects_prediction_proto;
};

using PlanningInputFrame = ad_byd::planning::PlanningInputFrame;
// using ObstacleType = ad_byd::planning::ObstacleType;
using ObstacleType = st::ObjectType;

std::unique_ptr<MultiTasksCruisePlannerInput> AdaptPlannerInput(
    const PlanningInputFrame* input_frame,
    const PlannerParamsProto& planner_params,
    const VehicleParamsProto& vehicle_params, absl::Time predicted_plan_time,
    ThreadPool* thread_pool, PlannerState* planner_state);

namespace {
std::variant<PlannerStatus, std::unique_ptr<MultiTasksCruisePlannerInput>>
ComputeMembers(const PlanningInputFrame* input_frame,
               const PlannerParamsProto& planner_params,
               const VehicleParamsProto& vehicle_params,
               absl::Time predicted_plan_time, ThreadPool* thread_pool,
               PlannerState* planner_state);

PoseProto AdaptPoseProto(const ad_byd::planning::odometry_type& odom,
                         const double estimate_a);
AutonomyStateProto AdaptAutonomyStateProto(
    const ad_byd::planning::behavior_type& behavior);
std::pair<Behavior, BehaviorChoice> AdaptBehavior(
    const ad_byd::planning::behavior_type& behavior,
    const PlannerParamsProto& planner_params,
    const ad_byd::planning::vehicle_status_type& veh_status,
    PlannerState* planner_state);
double GetFrontWheelAngle(
    const ad_byd::planning::vehicle_status_type& veh_status,
    double steer_ratio);
double GetLaneSpeedLimit(
    const PiecewiseLinearFunction<double>& set_speed_bias_plf,
    const PiecewiseLinearFunction<double>& set_speed_gain_plf,
    double set_speed_kph);
ObjectType AdaptObjectType(ObstacleType li_obstacle_type, bool is_traj_empty);
PredictionResult AdaptPredictionResult(
    const byd::msg::prediction::ObjectsPrediction& prediction);

void ComputeObjectsStopTime(
    PredictionResult& prediction_result,
    absl::flat_hash_map<std::string, PlannerState::ObjectStopTimeResult>&
        object_stop_time_map);
TrajectoryIntention AdaptTrajectoryIntention(
    const byd::msg::prediction::TrajectoryIntention& intention);
PNPInfos AdaptPNPInfo(
    const byd::msg::prediction::ObjectsPrediction& prediction);
double GetLaneLength(const std::vector<Vec2d>& points);
SceneReasoningOutput RunSceneReasoningAndFillDebug(
    const PlannerSemanticMapManager& psmm,
    const ad_byd::planning::TrafficLightStatusMap& tl_info_map,
    const PlannerObjectManager& obj_mgr,
    const ObjectsPredictionProto& prediction,
    const ApolloTrajectoryPointProto& plan_start_point, ThreadPool* thread_pool,
    ObjectHistoryManager& obj_his_manager);

absl::StatusOr<std::vector<mapping::LanePath>> BuildLanePathsFormPsmm(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point);

bool ShouldResetAlcState(ResetReasonProto::Reason reset_reason);

}  // namespace

}  // namespace planning
}  // namespace st

#endif  // AD_BYD_PLANNING_NODES_PLANNER_CITY_MSG_PROXY_H
