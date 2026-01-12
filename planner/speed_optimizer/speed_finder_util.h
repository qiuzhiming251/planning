

#ifndef ONBOARD_PLANNER_SPEED_SPEED_FINDER_UTIL_H_
#define ONBOARD_PLANNER_SPEED_SPEED_FINDER_UTIL_H_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/math/segment_matcher/segment_matcher_kdtree.h"
#include "object_manager/partial_spacetime_object_trajectory.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_scene_recognition.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/path_approx.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "plan_common/speed/st_speed/vt_speed_limit.h"
#include "plan_common/vehicle_shape.h"
#include "planner/speed_optimizer/cipv_object_info.h"
#include "planner/speed_optimizer/speed_bound.h"
#include "plan_common/util/motion_util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {
constexpr double kSpeedLimitProviderTimeStep = 0.1;  // s.

void KeepNearestStationarySpacetimeTrajectoryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);

std::optional<std::string> GetNearestStationaryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);

void SetStBoundaryDebugInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    SpeedFinderDebugProto* speed_finder_proto);

std::vector<StBoundaryWithDecision> InitializeStBoundaryWithDecision(
    std::vector<StBoundaryRef> raw_st_boundaries);

int GetSpeedFinderTrajectorySteps(double init_v, double default_speed_limit);

void PostProcessSpeedByFullStop(
    const SpeedFinderParamsProto& speed_finder_params, SpeedVector* speed_data);

std::vector<Box2d> BuildAvHeadBoxes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<VehicleShapeBasePtr>& av_shapes);

std::vector<VehicleShapeBasePtr> BuildAvShapes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path_points);

std::unique_ptr<SegmentMatcherKdtree> BuildPathKdTree(
    const DiscretizedPath& path_points);

std::optional<PathApprox> BuildPathApproxForMirrors(
    const PathApprox& path_approx,
    const VehicleGeometryParamsProto& vehicle_geom);

std::vector<PartialSpacetimeObjectTrajectory> GetConsideredStObjects(
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& obj_mgr,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>
        processed_st_objects);

void CutoffSpeedByTimeHorizon(SpeedVector* speed_data);

SpeedLimit GenerateRawLaneSpeedLimit(const DrivePassage* drive_passage,
                                     const DiscretizedPath& path_points,
                                     double allowed_max_speed);

SpeedVector GenerateReferenceSpeed(
    const std::vector<SpeedBoundWithInfo>& min_speed_limit,
    const std::optional<SpeedLimit>& raw_lane_speed_limit, double init_v,
    double ref_speed_bias, double ref_speed_static_limit_bias, double max_accel,
    double max_decel, double total_time, double delta_t);

/// @brief generate Acceleration Soft Bound for speed optmize. Currently, only
/// used to chase acceleration lane change gap
std::vector<std::pair<double, double>> GenerateAccelerationBound(
    const st::MotionConstraintParamsProto& motion_constraint_params,
    const std::optional<double> acc_target_a,
    const std::optional<double> dec_target_a, double start_a, double delta_t,
    int knot_num);

bool IsConsiderOncomingObs(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const StBoundary& st_boundary, const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params, double current_v,
    const DiscretizedPath& path);

SpeedBoundMapType EstimateSpeedBound(
    const SpeedLimitProvider& speed_limit_provider,
    const SpeedVector& preliminary_speed, double init_v,
    double allowed_max_speed, int knot_num, double delta_t);

std::vector<SpeedBoundWithInfo> GenerateMinSpeedLimitWithLaneAndCurvature(
    const std::vector<SpeedBoundWithInfo>& lane_speed_limit,
    const std::vector<SpeedBoundWithInfo>& curvature_speed_limit);

void DumpStGraphBoundary(
    int plan_id,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const SpeedFinderDebugProto& speed_finder_proto);

std::vector<std::string> MakeStBoundaryDebugInfo(
    const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeTrajectoryManager& traj_mgr);

void DumpStGraphSpeed(int plan_id, double path_length, int trajectory_steps,
                      const SpeedVector& preliminary_speed,
                      const SpeedVector& optimized_speed,
                      const SpeedVector& comfortable_brake_speed,
                      const SpeedVector& max_brake_speed);

void DumpVtGraphSpeedLimit(int plan_id,
                           const SpeedFinderDebugProto& speed_finder_proto);

void DumpVtGraphSpeed(int plan_id, const SpeedVector& preliminary_speed,
                      const SpeedVector& optimized_speed,
                      const SpeedVector& ref_speed);

void DumpAccPath(int plan_id, const DiscretizedPath& path, const Box2d& av_box);

void DumpPredictionTrajectories(
    int plan_id,
    const std::unordered_map<std::string, const SpacetimeObjectTrajectory*>&
        st_trajs_map);

void DumpPrimaryBrakingTarget(const SpeedVector& optimized_speed,
                              const SpeedFinderDebugProto& speed_finder_proto,
                              int plan_id);

CipvObjectInfo ComputeCipvObjectInfo(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SegmentMatcherKdtree& path_kd_tree,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

void DumpInteractiveSetDebugInfoToDebugFrame(
    const InteractiveSpeedDebugProto& interactive_speed_debug, int plan_id,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_SPEED_FINDER_UTIL_H_
