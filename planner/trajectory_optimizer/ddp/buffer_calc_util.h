#ifndef ST_PLANNING_OPTIMIZATION_BUFFER_CALC_UTIL
#define ST_PLANNING_OPTIMIZATION_BUFFER_CALC_UTIL

#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "plan_common/async/thread_pool.h"
#include "plan_common/log_data.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "planner/trajectory_optimizer/ddp/path_time_corridor.h"
#include "planner/trajectory_optimizer/ddp/trajectory_optimizer_defs.h"
#include "planner/trajectory_optimizer/problem/av_model_helper.h"
#include "planner/trajectory_optimizer/problem/cost.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st {
namespace planning {
namespace optimizer {

enum class TurnType;

class NudgeBufferManager {
 public:
  void CalcNudgeBuffers(
      const PathTimeCorridor& path_time_corridor,
      const PlannerSemanticMapManager& psmm,
      const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
      const std::vector<TrajectoryPoint>& prev_traje,
      const std::vector<TrajectoryPoint>& init_traj,
      const DrivePassage& drive_passage);

  const std::vector<double> GenerateNudgeBufferDynamic(
      const int plan_id, const LaneChangeStage lc_stage, const bool borrow_lane,
      const std::vector<prediction::PredictionObjectState>& states,
      const std::vector<st::planning::TrajectoryPoint>& init_traj,
      bool is_camera_object, const Vec2d& object_velocity,
      const Polygon2d& object_contour, const TrajectoryPoint& plan_start_point,
      const DrivePassage& drive_passage, const double lane_width,
      const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
      const VehicleGeometryParamsProto& veh_geo_params,
      const SpacetimeObjectTrajectory& traj,
      const PathSlBoundary& path_boundary,
      const PathTimeCorridor& path_time_corridor,
      st::planning::optimizer::TurnType ego_turn_type,
      const std::vector<st::planning::TrajectoryPoint>& prev_traj);

  const std::vector<double> GenerateNudgeBufferStationary(
      const int plan_id, const LaneChangeStage lc_stage, const bool borrow_lane,
      const std::vector<prediction::PredictionObjectState>& states,
      bool is_camera_object, bool is_static, const Vec2d& object_velocity,
      const Polygon2d& object_contour,
      const std::vector<st::planning::TrajectoryPoint>& init_traj,
      const DrivePassage& drive_passage, const double lane_width,
      const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
      const VehicleGeometryParamsProto& veh_geo_params,
      const SpacetimeObjectTrajectory& traj,
      const PathSlBoundary& path_boundary,
      const PathTimeCorridor& path_time_corridor, TurnType ego_turn_type);

  bool GenerateNudgeBufferUU(
      const int plan_id, double trajectory_time_step,
      std::string_view base_name, const LaneChangeStage lc_stage,
      const TrajectoryPoint& plan_start_point,
      const PathTimeCorridor& path_time_corridor,
      absl::Span<const SpacetimeObjectTrajectory* const> spacetime_trajs,
      double min_mirror_height_avg, double max_mirror_height_avg,
      const double lane_width_l, const double lane_width_r,
      const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
      const VehicleCircleModelParamsProto&
          trajectory_optimizer_vehicle_model_params,
      const VehicleGeometryParamsProto& veh_geo_params);

  friend void AddObjectCosts(
      const int plan_id, const LaneChangeStage lc_stage, const bool ref_enhance,
      const bool borrow_lane, const NudgeInfos& nudge_info,
      int trajectory_steps, double trajectory_time_step,
      double avoid_dynamic_obj_early_time, std::string_view base_name,
      const std::vector<TrajectoryPoint>& init_traj,
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

 private:
  std::vector<std::map<mapping::ElementId, double>> uu_buffers_;
  std::vector<std::map<mapping::ElementId, double>> static_object_buffers_;
  std::vector<std::map<mapping::ElementId, std::vector<double>>>
      dynamic_object_buffers_;
};

}  // namespace optimizer
}  // namespace planning
}  // namespace st

#endif