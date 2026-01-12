

#ifndef ST_PLANNING_OPTIMIZATION_DDP_OBJECT_COST_UTIL
#define ST_PLANNING_OPTIMIZATION_DDP_OBJECT_COST_UTIL

#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "plan_common/async/thread_pool.h"
#include "plan_common/path_sl_boundary.h"
//#include "decider/initializer/select_nudge_object.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "planner/trajectory_optimizer/ddp/buffer_calc_util.h"
#include "planner/trajectory_optimizer/ddp/path_time_corridor.h"
#include "planner/trajectory_optimizer/ddp/trajectory_optimizer_defs.h"
#include "planner/trajectory_optimizer/problem/av_model_helper.h"
#include "planner/trajectory_optimizer/problem/cost.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/drive_passage.h"
//#include "plan_common/util/hmi_content_util.h"
//#include "object_manager/planner_object.h"

namespace st {
namespace planning {

namespace optimizer {

enum class TurnType {
  kUnknown = 0,
  kStraight = 1,
  kLeftTurn = 2,
  kRightTurn = 3,
  kUTurn = 4,
};

void AddObjectCosts(
    const int plan_id, const LaneChangeStage lc_stage, const bool ref_enhance,
    const bool borrow_lane, const NudgeInfos& nudge_info, int trajectory_steps,
    double trajectory_time_step, double avoid_dynamic_obj_early_time,
    std::string_view base_name, const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const PathTimeCorridor& path_time_corridor,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    const bool is_narrow_scene, std::vector<LeadingInfo>* leading_min_s,
    std::vector<double>* inner_path_boundary_gains,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs, ThreadPool* thread_pool,
    TurnType ego_turn_type, const std::vector<TrajectoryPoint>& prev_traj);

void CalcPartitionHalfContourInfo(const Vec2d& x, const Vec2d& obj_x,
                                  const Polygon2d& contour, double buffer,
                                  std::vector<Segment2d>* lines, Vec2d* ref_x,
                                  Vec2d* ref_tangent, double* offset);

// Re-sample spacetime object state to match with ddp optimizer time step.
std::vector<prediction::PredictionObjectState> SampleObjectStates(
    int trajectory_steps, double trajectory_time_step,
    absl::Span<const prediction::PredictionObjectState> states);

}  // namespace optimizer
}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_OPTIMIZATION_DDP_OBJECT_COST_UTIL
