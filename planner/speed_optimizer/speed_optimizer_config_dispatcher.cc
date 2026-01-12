

#include "planner/speed_optimizer/speed_optimizer_config_dispatcher.h"

#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "plan_common/path_approx_overlap.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "object_manager/planner_object.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/path_util.h"

namespace st::planning {
namespace {

using SpeedOptimizerParamsProto =
    SpeedFinderParamsProto::SpeedOptimizerParamsProto;

bool IsObjectExceedAvFrontEdge(
    const SpacetimeObjectTrajectory& traj, const PathPoint& av_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  CHECK_GT(traj.states().size(), 0);
  const auto& first_state = traj.states()[0];
  const Vec2d av_tan = Vec2d::FastUnitFromAngle(av_point.theta());
  const Vec2d obj_center = first_state.contour.CircleCenter();
  return av_tan.Dot(obj_center - ToVec2d(av_point)) +
             first_state.contour.CircleRadius() >
         vehicle_geometry_params.front_edge_to_center();
}

inline bool IsObjectParallelWithAv(const SpacetimeObjectTrajectory& traj,
                                   const PathPoint& av_point,
                                   double heading_threshold) {
  const auto& states = traj.states();
  if (states.empty()) return false;
  const auto& first_state = states[0];
  const double heading_diff =
      std::abs(AngleDifference(av_point.theta(), first_state.box.heading()));
  return heading_diff < heading_threshold;
}

// Determine whether the path collides with the linear predicted area of
// the object.
bool HasOverlapWithLinearPredictedObject(
    const SpacetimeObjectTrajectory& st_traj, const PathApprox& path_approx,
    const SegmentMatcherKdtree& path_kd_tree, double av_radius,
    int path_last_index, double path_step) {
  const auto& origin_contour = st_traj.planner_object().contour();
  constexpr double kForwardTime = 2.5;  // s.
  const auto transform_contour =
      origin_contour.Shift(kForwardTime * st_traj.planner_object().velocity());
  Polygon2d sweeped_area;
  if (st_traj.planner_object().velocity().norm() > 0.2) {
    const auto transform_contour = origin_contour.Shift(
        kForwardTime * st_traj.planner_object().velocity());
    sweeped_area =
        Polygon2d::MergeTwoPolygons(origin_contour, transform_contour);
  } else {
    sweeped_area = origin_contour;
  }
  constexpr double kLatBuffer = 0.5;  // m.
  const Polygon2d sweeped_area_with_buffer =
      sweeped_area.ExpandByDistance(kLatBuffer);

  constexpr double kSearchRadiusBuffer = 0.2;  // m.
  const double search_radius =
      av_radius + sweeped_area_with_buffer.CircleRadius() + kSearchRadiusBuffer;
  return HasPathApproxOverlapWithPolygon(path_approx, path_step,
                                         /*first_index=*/0, path_last_index,
                                         sweeped_area_with_buffer,
                                         search_radius);
}

std::optional<std::string> IsInSafetyMode(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr, const PathApprox& path_approx,
    const SegmentMatcherKdtree& path_kd_tree, double av_radius,
    double path_step_length, int path_last_index, double current_v,
    const PathPoint& current_path_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  constexpr double kSafetyCheckTime = 2.0;  // s.
  constexpr double kSafetyBuffer = 1.5;     // m.
  constexpr double kProbThres = 0.2;
  constexpr double kTimeStep = 0.1;  // s.

  absl::flat_hash_map<std::string, double> object_id_to_prob;
  absl::flat_hash_map<std::string, bool> obj_collision_ret;
  for (const auto& stb_wd : st_boundaries_with_decision) {
    const StBoundary* raw_st_boundary = stb_wd.raw_st_boundary();
    if (raw_st_boundary->source_type() !=
        StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (stb_wd.decision_type() != StBoundaryProto::FOLLOW &&
        stb_wd.decision_type() != StBoundaryProto::YIELD) {
      continue;
    }

    const auto& overlap_meta = raw_st_boundary->overlap_meta();
    const bool is_consider_back_obj =
        overlap_meta.has_value() &&
        (overlap_meta->source() == StOverlapMetaProto::LANE_MERGE ||
         overlap_meta->source() == StOverlapMetaProto::AV_CUTIN);

    const auto& traj_id = stb_wd.traj_id();
    CHECK(traj_id.has_value());
    const SpacetimeObjectTrajectory* st_traj =
        CHECK_NOTNULL(traj_mgr.FindTrajectoryById(*traj_id));
    if (!is_consider_back_obj &&
        !IsObjectExceedAvFrontEdge(*st_traj, current_path_point,
                                   vehicle_geometry_params)) {
      continue;
    }

    constexpr double kRelativeSpeedThres = 0.5;  // m/s.
    constexpr double kObjectAccelThres = -0.3;   // m/ss.
    const auto& speed_points = raw_st_boundary->speed_points();
    if (IsObjectParallelWithAv(*st_traj, current_path_point, d2r(20.0)) &&
        !speed_points.empty() &&
        speed_points.front().v() - current_v > kRelativeSpeedThres &&
        st_traj->pose().a() > kObjectAccelThres) {
      continue;
    }

    const auto& object_id = raw_st_boundary->object_id();
    CHECK(object_id.has_value());
    if (!obj_collision_ret.contains(*object_id)) {
      obj_collision_ret.emplace(
          *object_id, HasOverlapWithLinearPredictedObject(
                          *st_traj, path_approx, path_kd_tree, av_radius,
                          path_last_index, path_step_length));
    }

    if (!FindOrDie(obj_collision_ret, *object_id)) {
      continue;
    }

    for (double t = 0.0; t < kSafetyCheckTime; t += kTimeStep) {
      if (!InRange(t, raw_st_boundary->min_t(), raw_st_boundary->max_t())) {
        continue;
      }
      const auto s_range = raw_st_boundary->GetBoundarySRange(t);
      CHECK(s_range.has_value());
      if (current_v * t > s_range->second - kSafetyBuffer) {
        object_id_to_prob[*object_id] += raw_st_boundary->probability();
        if (FindOrDie(object_id_to_prob, *object_id) > kProbThres) {
          return *object_id;
        }
        break;
      }
    }
  }
  return std::nullopt;
}

void DispatchSpeedOptimizerConfigBySafetyMode(
    SpeedOptimizerParamsProto* speed_optimizer_param) {
  CHECK_NOTNULL(speed_optimizer_param);

  speed_optimizer_param->set_enable_comfort_brake_speed(false);
  speed_optimizer_param->set_enable_const_speed_ref_v(true);
  constexpr double kSafetyModePredImpactFactor = 1.0;
  speed_optimizer_param->set_prediction_impact_factor(
      kSafetyModePredImpactFactor);
  speed_optimizer_param->set_enable_lead_decision(false);
  constexpr double kSFollowWeakWeight = 1.5;
  speed_optimizer_param->set_s_follow_weak_weight(kSFollowWeakWeight);

  // Soft jerk weight is not correlated with av speed in safe mode.
  const PiecewiseLinearFunction<double> const_soft_jerk_lower_weight = {
      {0.0, 30.0}, {0.01, 0.01}};
  PiecewiseLinearFunctionToProto(
      const_soft_jerk_lower_weight,
      speed_optimizer_param->mutable_soft_jerk_lower_weight_plf());
}

}  // namespace

std::optional<SpeedOptimizerParamsProto> DispatchSpeedOptimizerConfig(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr, const PathApprox& path_approx,
    const SegmentMatcherKdtree& path_kd_tree, double av_radius,
    double path_step_length, int path_last_index, double current_v,
    const PathPoint& current_path_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedOptimizerParamsProto& raw_speed_optimizer_params,
    const SpeedFinderParamsProto::SpeedOptimizerConfigDispatcherParams&
        config_dispatcher_params) {
  std::optional<SpeedOptimizerParamsProto> speed_optimizer_params;

  if (config_dispatcher_params.enable_dispatch_by_safety_mode()) {
    if (!speed_optimizer_params.has_value()) {
      speed_optimizer_params = raw_speed_optimizer_params;
    }
    const auto safety_mode_obj_id = IsInSafetyMode(
        st_boundaries_with_decision, traj_mgr, path_approx, path_kd_tree,
        av_radius, path_step_length, path_last_index, current_v,
        current_path_point, vehicle_geometry_params);

    if (safety_mode_obj_id.has_value()) {
      DispatchSpeedOptimizerConfigBySafetyMode(&(*speed_optimizer_params));
    }
  }

  return speed_optimizer_params;
}

}  // namespace st::planning
