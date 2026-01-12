

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_STATIC_BOUNDARY_COST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_STATIC_BOUNDARY_COST_UTIL_H_

#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "plan_common/path_sl_boundary.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/util/decision_info.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "planner/trajectory_optimizer/ddp/trajectory_optimizer_defs.h"
#include "planner/trajectory_optimizer/problem/center_line_query_helper.h"
#include "planner/trajectory_optimizer/problem/cost.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/drive_passage.h"
//#include "plan_common/util/hmi_content_util.h"
//#include "object_manager/planner_object.h"

namespace st {
namespace planning {
namespace optimizer {

void AddStaticBoundaryCosts(
    LaneChangeStage lc_stage, int trajectory_steps, std::string_view base_name,
    bool enable_three_point_turn, const TrajectoryPoint& plan_start_point,
    const DrivePassage& drive_passage, const PlannerSemanticMapManager& psmm,
    const NudgeInfos& nudge_info, const PathSlBoundary& path_sl_boundary,
    const std::vector<double>& inner_path_boundary_gains,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    const std::optional<double>& lane_width_curb_buffer_opt,
    const double lane_width, std::optional<double>* extra_curb_buffer_opt,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs);

void AddSolidWhiteLineCost(
    int trajectory_steps, std::string_view base_name,
    const std::vector<TrajectoryPoint>& solver_init_traj,
    const ConstraintManager& constraint_manager,
    const TrajectoryPoint& plan_start_point,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs);

void AddRoadBoundaryCost(
    int trajectory_steps, std::string_view base_name,
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* const costs);

}  // namespace optimizer
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_STATIC_BOUNDARY_COST_UTIL_H_
