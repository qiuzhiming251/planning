

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_SPEED_LIMIT_COST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_SPEED_LIMIT_COST_UTIL_H_

#include <memory>
#include <vector>

#include "plan_common/constraint_manager.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "planner/trajectory_optimizer/ddp/trajectory_optimizer_defs.h"
#include "planner/trajectory_optimizer/problem/center_line_query_helper.h"
#include "planner/trajectory_optimizer/problem/cost.h"
#include "planner/trajectory_optimizer/problem/segmented_speed_limit_cost_v2.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/drive_passage.h"
namespace st {
namespace planning {
namespace optimizer {

using SpeedlimitInfoPoint =
    SegmentedSpeedLimitCostV2<Mfob>::SpeedlimitInfoPoint;

struct SpeedZoneInfo {
  double s_start{0};
  double s_end{0};
  Vec2d x_start{};
  Vec2d x_end{};
  double target_speed{0};
};

namespace speedlimit {

void MergeSpeedLimitWithFirstStopLine(
    double first_stop_line_s, const Vec2d& first_stop_point,
    const std::vector<double>& station_points_s,
    const std::vector<Vec2d>& speed_limit_x,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits);

void MergeSpeedLimitWithSpeedZones(
    const std::vector<SpeedZoneInfo>& speed_zones,
    const std::vector<Vec2d>& station_points,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits);

std::vector<double> CreateSpatialRef(
    double trajectory_time_step, const TrajectoryPoint& plan_start_point,
    const std::vector<Vec2d>& station_points,
    const std::vector<double>& station_speed_limits,
    const std::vector<std::vector<SpeedlimitInfoPoint>>&
        additional_speed_limits,
    const std::vector<LeadingInfo>& leading_min_s, int step_count, double max_a,
    double min_a, double leading_s_offset);
}  // namespace speedlimit

void AddSpeedLimitCost(
    const LaneChangeStage lc_stage, int trajectory_steps,
    double trajectory_time_step, const TrajectoryPoint& plan_start_point,
    const DrivePassage& drive_passage,
    const ConstraintManager& constraint_manager,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    const std::vector<LeadingInfo>& leading_min_s, double* ref_end_state_s,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs,
    TrajectoryOptimizerDebugProto* traj_opt_debug_proto,
    int plan_id, int function_id);

}  // namespace optimizer
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_SPEED_LIMIT_COST_UTIL_H_
