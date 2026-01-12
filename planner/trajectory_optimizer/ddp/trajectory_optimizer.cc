

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "plan_common/log.h"
#include "gflags/gflags.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "absl/status/status.h"
#include "absl/cleanup/cleanup.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/timer.h"
#include "plan_common/log_data.h"
#include "plan_common/drive_passage.h"
#include "plan_common/trajectory_util.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/async/async_util.h"
#include "plan_common/async/parallel_for.h"
#include "plan_common/math/vec.h"
#include "plan_common/math/util.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/util/file_util.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/time_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "plan_common/maps/semantic_map_util.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/planning_macros.h"

#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "planner/trajectory_optimizer/ddp/hmi_util.h"
#include "planner/trajectory_optimizer/ddp/ddp_optimizer.h"
#include "planner/trajectory_optimizer/ddp/object_cost_util.h"
#include "planner/trajectory_optimizer/ddp/path_time_corridor.h"
#include "planner/trajectory_optimizer/ddp/speed_limit_cost_util.h"
#include "planner/trajectory_optimizer/ddp/ddp_optimizer_debug_monitor.h"
#include "planner/trajectory_optimizer/ddp/static_boundary_cost_util.h"
#include "planner/trajectory_optimizer/ddp/trajectory_optimizer_defs.h"
#include "planner/trajectory_optimizer/ddp/trajectory_optimizer_util.h"
#include "planner/trajectory_optimizer/problem/cost.h"
#include "planner/trajectory_optimizer/problem/curvature_cost.h"
#include "planner/trajectory_optimizer/problem/av_model_helper.h"
#include "planner/trajectory_optimizer/problem/end_heading_cost.h"
#include "planner/trajectory_optimizer/problem/forward_speed_cost.h"
#include "planner/trajectory_optimizer/problem/intrinsic_jerk_cost.h"
#include "planner/trajectory_optimizer/problem/mfob_lateral_jerk_cost.h"
#include "planner/trajectory_optimizer/problem/mfob_curvature_rate_cost.h"
#include "planner/trajectory_optimizer/problem/center_line_query_helper.h"
#include "planner/trajectory_optimizer/problem/curvature_deviation_cost.h"
#include "planner/trajectory_optimizer/problem/lateral_acceleration_cost.h"
#include "planner/trajectory_optimizer/problem/mixed_fourth_order_bicycle.h"
#include "planner/trajectory_optimizer/problem/reference_line_deviation_cost.h"
#include "planner/trajectory_optimizer/problem/mfob_curvature_rate_rate_cost.h"
#include "planner/trajectory_optimizer/problem/longitudinal_acceleration_cost.h"
#include "planner/trajectory_optimizer/problem/reference_state_deviation_cost.h"
#include "planner/trajectory_optimizer/problem/mfob_intrinsic_lateral_snap_cost.h"
#include "planner/trajectory_optimizer/problem/reference_control_deviation_cost.h"
#include "planner/trajectory_optimizer/problem/reference_single_state_deviation_cost.h"
#include "planner/trajectory_optimizer/problem/reference_longitudinal_jerk_deviation_cost.h"  // NOLINT

#include "planner/planner_manager/planner_defs.h"
#include "planner/planner_manager/planner_flags.h"
#include "planner/planner_manager/planner_util.h"
#include "planner/trajectory_optimizer/mfob_trajectory_smoother.h"

#include "planner/trajectory_optimizer/ddp/trajectory_optimizer.h"

DEFINE_int32(traj_opt_verbosity_level, 2, "Traj opt verbosity level.");
DEFINE_double(auto_tuning_gamma, 0.99,
              "Only used in auto tuning mode, gamma is the discounted rate.");
DEFINE_bool(enable_ipopt_solver, false,
            "Whether to enable ipopt solver to be used for being compared with "
            "ddp optimizer.");
DEFINE_bool(traj_opt_draw_circle, false, "Whether to draw av model.");
DECLARE_bool(msd_static_boundary_cost_v2);

namespace st {
namespace planning {
namespace {

using Mfob = optimizer::Mfob;

void ModifyTrajOptParamsStyle(
    const TrajectoryOptimizerParamsProto&
        trajectory_optimizer_lc_radical_params,
    const TrajectoryOptimizerParamsProto& trajectory_optimizer_lc_normal_params,
    const TrajectoryOptimizerParamsProto&
        trajectory_optimizer_lc_conservative_params,
    LaneChangeStage lc_stage, PathResponseStyle lc_style,
    TrajectoryOptimizerParamsProto* trajectory_optimizer_params) {
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    switch (lc_style) {
      case PATH_RESPONSE_NORMAL:
        *trajectory_optimizer_params = trajectory_optimizer_lc_normal_params;
        break;
      case PATH_RESPONSE_RADICAL:
        *trajectory_optimizer_params = trajectory_optimizer_lc_radical_params;
        break;
      case PATH_RESPONSE_CONSERVATIVE:
        *trajectory_optimizer_params =
            trajectory_optimizer_lc_conservative_params;
        break;
      default:
        break;
    }
  }
}

// TODO: Use input instead of directly reading file.
void UpdateTrajOptParams(
    const std::string& traj_opt_params_address,
    TrajectoryOptimizerParamsProto* trajectory_optimizer_params) {
  VLOG(3) << "Trajectory optimizer params before update: ";
  VLOG(3) << trajectory_optimizer_params->DebugString();
  if (!file_util::TextFileToProto(traj_opt_params_address,
                                  trajectory_optimizer_params)) {
    CHECK(false) << "Read trajectory optimizer params as text file failed!!!!";
  }
  LOG_INFO << "New trajectory optimizer params are used.";
}

void ToDebugProto(const std::vector<TrajectoryPoint>& init_traj,
                  const std::vector<TrajectoryPoint>& solver_init_traj,
                  const std::vector<TrajectoryPoint>& result_traj,
                  const OptimizerSolverDebugMonitor<Mfob>& solver_debug_monitor,
                  const DdpOptimizerDebugProto::SolverInitialTrajectorySource&
                      solver_init_traj_source,
                  TrajectoryOptimizerDebugProto* traj_opt_debug_proto,
                  ThreadPool* thread_pool) {
  // ("TrajectoryOptimizer/ToDebugProto");
  //  Write DdpDebugProto data.
  TIMELINE("ToDebugProto");
  DdpOptimizerDebugProto* ddp_debug = traj_opt_debug_proto->mutable_ddp();
  ddp_debug->mutable_init_traj()->Reserve(init_traj.size());
  for (int k = 0; k < init_traj.size(); ++k) {
    init_traj[k].ToProto(ddp_debug->add_init_traj());
  }

  ddp_debug->mutable_solver_initial_trajectory()->Reserve(
      solver_init_traj.size());
  for (int k = 0; k < solver_init_traj.size(); ++k) {
    solver_init_traj[k].ToProto(ddp_debug->add_solver_initial_trajectory());
  }
  ddp_debug->set_solver_initial_trajectory_source(solver_init_traj_source);

  ddp_debug->mutable_final_traj()->Reserve(result_traj.size());
  for (int k = 0; k < result_traj.size(); ++k) {
    result_traj[k].ToProto(ddp_debug->add_final_traj());
  }

  const auto& init_costs = solver_debug_monitor.init_costs;
  ddp_debug->mutable_init_costs()->set_cost(init_costs.cost);
  for (int i = 0; i < init_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        ddp_debug->mutable_init_costs()->add_costs();
    cost_proto->set_name(init_costs.ddp_costs[i].name);
    cost_proto->set_cost(init_costs.ddp_costs[i].value);
  }
  const auto& final_costs = solver_debug_monitor.final_costs;
  ddp_debug->mutable_final_costs()->set_cost(final_costs.cost);
  for (int i = 0; i < final_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        ddp_debug->mutable_final_costs()->add_costs();
    cost_proto->set_name(final_costs.ddp_costs[i].name);
    cost_proto->set_cost(final_costs.ddp_costs[i].value);
  }

  const int num_iters = solver_debug_monitor.iterations.size();
  ddp_debug->mutable_iterations()->Reserve(num_iters);
  for (int i = 0; i < num_iters; ++i) {
    ddp_debug->add_iterations();
  }

  // do not use the thread pool
  ParallelFor(0, num_iters, thread_pool, [&](int i) {
    TIMELINE("ToDebugProto::ParallelProcess");
    const auto& iteration = solver_debug_monitor.iterations[i];
    DdpOptimizerDebugProto::Iteration* iteration_proto =
        ddp_debug->mutable_iterations()->Mutable(i);
    CHECK_EQ(iteration.alphas.size(), iteration.line_search_costs.size());
    for (int i = 0; i < iteration.alphas.size(); ++i) {
      iteration_proto->add_line_search_alphas(iteration.alphas[i]);
      iteration_proto->add_line_search_costs(iteration.line_search_costs[i]);
    }
    CHECK_EQ(iteration.k_s.size(), iteration.stepsize_adjustment_costs.size());
    for (int i = 0; i < iteration.k_s.size(); ++i) {
      iteration_proto->add_step_size_adjustment_ks(iteration.k_s[i]);
      iteration_proto->add_step_size_adjustment_costs(
          iteration.stepsize_adjustment_costs[i]);
    }
    iteration_proto->set_final_cost(iteration.final_cost);
    iteration_proto->set_js0(iteration.js0);
  });

  ddp_debug->set_num_iters(num_iters);

  for (const auto& response : solver_debug_monitor.object_responses) {
    *traj_opt_debug_proto->add_object_responses() = response;
  }
}

absl::Status CheckInputQuality(const TrajectoryOptimizerInput& input) {
  constexpr int kMinTrajectoryLength = 10;  // 1s.
  if (input.trajectory.size() < kMinTrajectoryLength) {
    return absl::FailedPreconditionError(
        absl::StrFormat("Input trajectory is not long enough: %d time steps.",
                        input.trajectory.size()));
  }
  return absl::OkStatus();
}

void AddRegularizersCost(
    const std::vector<TrajectoryPoint>& init_traj,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  const Mfob::StateType x0 = Mfob::FitInitialState(init_traj);
  const Mfob::ControlsType init_us = Mfob::FitControl(init_traj, x0);
  const Mfob::StatesType init_xs = Mfob::FitState(init_traj);

  std::vector<double> state_regularization_weights(init_xs.size(), 0.0);
  std::vector<double> control_regularization_weights(init_us.size(), 0.0);
  for (int i = 0; i < init_traj.size(); ++i) {
    for (int j = 0; j < Mfob::kStateSize; ++j) {
      state_regularization_weights[i * Mfob::kStateSize + j] =
          cost_weight_params.state_regularization_coeffs().w(j);
    }
    for (int j = 0; j < Mfob::kControlSize; ++j) {
      control_regularization_weights[i * Mfob::kControlSize + j] =
          cost_weight_params.control_regularization_coeffs().w(j);
    }
  }
  costs->emplace_back(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
      init_xs, std::move(state_regularization_weights),
      "MfobReferenceStateDeviationCost: StateRegularization",
      cost_weight_params.state_regularization_coeffs().multiplier(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  costs->emplace_back(std::make_unique<ReferenceControlDeviationCost<Mfob>>(
      init_us, std::move(control_regularization_weights),
      "MfobReferenceControlDeviationCost: ControlRegularization",
      cost_weight_params.control_regularization_coeffs().multiplier(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddAccelAndJerkCost(
    int trajectory_steps, double trajectory_time_step,
    const TrajectoryPoint& plan_start_point,
    const PlannerSemanticMapManager& psmm,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const TrajectoryOptimizerValidationParamsProto&
        trajectory_optimizer_validation_params,
    const std::optional<double>& extra_curb_buffer,
    const std::optional<double>& lane_width,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs, const bool in_uturn) {
  std::vector<double> accel_cascade_buffers;
  std::vector<double> accel_cascade_gains;
  std::vector<double> decel_cascade_buffers;
  std::vector<double> decel_cascade_gains;
  for (const auto& cascade :
       cost_weight_params.longitudinal_acceleration_cost_params()
           .accel_cascade()) {
    accel_cascade_buffers.push_back(cascade.buffer());
    accel_cascade_gains.push_back(cascade.gain());
  }
  for (const auto& cascade :
       cost_weight_params.longitudinal_acceleration_cost_params()
           .decel_cascade()) {
    decel_cascade_buffers.push_back(cascade.buffer());
    decel_cascade_gains.push_back(cascade.gain());
  }
  constexpr double kAccelerationBufferRatio = 1.0;
  constexpr double kJerkBufferRatio = 0.75;
  double longitudinal_acceleration_cost_weight =
      cost_weight_params.longitudinal_acceleration_cost_weight();
  if (in_uturn) {
    longitudinal_acceleration_cost_weight =
        cost_weight_params.uturn_longitudinal_acceleration_cost_weight();
  }
  costs->emplace_back(std::make_unique<LongitudinalAccelerationCost<Mfob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio,
      std::move(accel_cascade_buffers), std::move(accel_cascade_gains),
      std::move(decel_cascade_buffers), std::move(decel_cascade_gains),
      "MfobLongitudinalAccelerationCost", longitudinal_acceleration_cost_weight,
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));

  std::vector<double> lateral_acceleration_gains(trajectory_steps, 1.0);
  const PiecewiseLinearFunction<double>
      lateral_acceleration_gain_time_scale_plf =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.lateral_acceleration_gain_time_scale_plf());
  for (int i = 0; i < lateral_acceleration_gains.size(); ++i) {
    lateral_acceleration_gains.at(i) = lateral_acceleration_gain_time_scale_plf(
        static_cast<double>(i * trajectory_time_step));
  }
  costs->emplace_back(std::make_unique<LateralAccelerationCost<Mfob>>(
      lateral_acceleration_gains,
      /*using_hessian_approximate=*/true, "MfobLateralAccelerationCost",
      cost_weight_params.lateral_acceleration_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  // TODO: Try to use MfobLongitudinalJerkCost.
  double intrinsic_jerk_cost_weight =
      cost_weight_params.intrinsic_jerk_cost_weight();
  if (in_uturn) {
    intrinsic_jerk_cost_weight =
        cost_weight_params.uturn_intrinsic_jerk_cost_weight();
  }
  costs->emplace_back(std::make_unique<IntrinsicJerkCost<Mfob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio,
      "MfobIntrinsicJerkCost", intrinsic_jerk_cost_weight,
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));

  std::vector<double> lateral_jerk_gains(trajectory_steps, 1.0);
  const PiecewiseLinearFunction<double> lateral_jerk_gain_time_scale_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.lateral_jerk_gain_time_scale_plf());
  for (int i = 0; i < lateral_jerk_gains.size(); ++i) {
    lateral_jerk_gains.at(i) = lateral_jerk_gain_time_scale_plf(
        static_cast<double>(i * trajectory_time_step));
  }
  costs->emplace_back(std::make_unique<MfobLateralJerkCost<Mfob>>(
      lateral_jerk_gains,
      /*using_hessian_approximate=*/true, "MfobLateralJerkCost",
      cost_weight_params.lateral_jerk_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));

  // BANDAID(xxx): This is a hack to prevent fast steering change when AV
  // is close to curb due to control/localization error.
  std::vector<std::pair<double, Vec2d>> plan_start_circles;
  std::vector<std::pair<double, Vec2d>> plan_start_mirror_circles;
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(plan_start_point.theta() +
                                                   circle.angle_to_axis());
    plan_start_circles.emplace_back(
        circle.radius(),
        plan_start_point.pos() + circle.dist_to_rac() * tangent);
  }
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.mirror_circles()) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(plan_start_point.theta() +
                                                   circle.angle_to_axis());
    plan_start_mirror_circles.emplace_back(
        circle.radius(),
        plan_start_point.pos() + circle.dist_to_rac() * tangent);
  }
  const double min_mirror_height_avg =
      ComputeMinMaxMirrorAverageHeight(veh_geo_params).first;

  constexpr double kNearbyDistance = 10.0;  // m.
  double nearest_dist = kNearbyDistance;
  const std::vector<ImpassableBoundaryInfo> boundaries_info =
      psmm.GetImpassableBoundariesInfo(plan_start_point.pos(), kNearbyDistance);
  for (const auto& boundary_info : boundaries_info) {
    const bool consider_mirrors =
        boundary_info.height.has_value()
            ? (boundary_info.height.value() > min_mirror_height_avg)
            : trajectory_optimizer_vehicle_model_params
                  .consider_mirrors_by_default();
    for (const auto& circle : plan_start_circles) {
      nearest_dist = std::min(
          nearest_dist,
          boundary_info.segment.DistanceTo(circle.second) - circle.first);
    }
    if (consider_mirrors) {
      for (const auto& circle : plan_start_mirror_circles) {
        nearest_dist = std::min(
            nearest_dist,
            boundary_info.segment.DistanceTo(circle.second) - circle.first);
      }
    }
    // Log2DDS::LogDataV0("curb_debug", "curb_id: " + boundary_info.id);
  }

  double penetration_distance = 0.0;
  // static int curb_debounce = 0;
  const double extra_buffer =
      extra_curb_buffer.has_value() ? *extra_curb_buffer : 0.0;
  if (FLAGS_msd_static_boundary_cost_v2) {
    const auto speed_buffer_plf = PiecewiseLinearFunctionFromProto(
        cost_weight_params.speed_rel_hard_curb_clearance_plf());
    double speed_debug = speed_buffer_plf(plan_start_point.v());
    // double penetration_distance_max =
    //     std::max(extra_buffer + speed_buffer_plf(plan_start_point.v()),
    //              lane_width_curb_buffer_opt.value());
    double penetration_distance_max =
        extra_buffer + speed_buffer_plf(plan_start_point.v());
    penetration_distance =
        std::max(0.0, penetration_distance_max - nearest_dist);
    Log2DDS::LogDataV0("curb_debug", absl::StrCat("penetration_distance: ",
                                                  penetration_distance));
    Log2DDS::LogDataV0("curb_debug",
                       absl::StrCat("extra_buffer: ", extra_buffer));
    Log2DDS::LogDataV0("curb_debug",
                       absl::StrCat("speed_buffer_plf: ", speed_debug));
    Log2DDS::LogDataV0("curb_debug",
                       absl::StrCat("nearest_dist: ", nearest_dist));
    Log2DDS::LogDataV0("curb_debug", absl::StrCat("penetration_distance_max: ",
                                                  penetration_distance_max));
  }

  const auto penetration_gain_plf = PiecewiseLinearFunctionFromProto(
      cost_weight_params.curb_penetration_lateral_snap_gain_plf());
  const double lateral_snap_gain = penetration_gain_plf(penetration_distance);
  Log2DDS::LogDataV0("curb_debug",
                     absl::StrCat("lateral_snap_gain: ", lateral_snap_gain));
  const PiecewiseLinearFunction<double> speed_relative_gain =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.lateral_snap_speed_gain_plf());
  const double base_gain =
      lateral_snap_gain * speed_relative_gain(plan_start_point.v());
  std::vector<double> snap_gains(trajectory_steps, base_gain);
  const PiecewiseLinearFunction<double> snap_gain_time_scale_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.snap_gain_time_scale_plf());
  for (int i = 0; i < snap_gains.size(); ++i) {
    snap_gains.at(i) = snap_gain_time_scale_plf(
                           static_cast<double>(i * trajectory_time_step)) *
                       base_gain;
  }

  costs->emplace_back(std::make_unique<MfobIntrinsicLateralSnapCost<Mfob>>(
      snap_gains, "MfobIntrinsicLateralSnapCost",
      cost_weight_params.intrinsic_lateral_snap_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddCurvatureCost(
    double trajectory_time_step, const TrajectoryPoint& plan_start_point,
    const ConstraintManager& constraint_manager,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const DrivePassage& drive_passage,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs,
    bool& is_kappa_inhibit_scene, const bool in_uturn) {
  CHECK_GT(trajectory_time_step, 0.0);
  const int curvature_limit_index =
      static_cast<int>(kCurvatureLimitRange / trajectory_time_step);
  const int free_index =
      static_cast<int>(kTrajectoryTimeHorizon / trajectory_time_step);

  constexpr double kCurvatureBufferRatio = 0.98;
  double kCurvatureRateBufferRatio = 0.75;
  constexpr double curvature_scale = 1.0;
  double curvature_cost_weight = cost_weight_params.curvature_cost_weight();
  double curvature_rate_cost_weight =
      cost_weight_params.curvature_rate_cost_weight();
  double curvature_rate_rate_cost_weight =
      cost_weight_params.curvature_rate_rate_cost_weight();
  if (in_uturn) {
    curvature_cost_weight = cost_weight_params.uturn_curvature_cost_weight();
    curvature_rate_cost_weight =
        cost_weight_params.uturn_curvature_rate_cost_weight();
    curvature_rate_rate_cost_weight =
        cost_weight_params.uturn_curvature_rate_rate_cost_weight();
    kCurvatureRateBufferRatio = 0.98;
  }

  Log2DDS::LogDataV2("CurvatureCost", absl::StrCat("free_index ", free_index));

  costs->emplace_back(std::make_unique<CurvatureCost<Mfob>>(
      ComputeCenterMaxCurvature(veh_geo_params, vehicle_drive_params) *
          kCurvatureBufferRatio,
      curvature_limit_index, free_index, curvature_scale, "MfobCurvatureCost",
      curvature_cost_weight,
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  costs->emplace_back(std::make_unique<MfobCurvatureRateCost<Mfob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio,
      "MfobCurvatureRateCost", curvature_rate_cost_weight,
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  costs->emplace_back(std::make_unique<MfobCurvatureRateRateCost<Mfob>>(
      motion_constraint_params.max_chi(), "MfobCurvatureRateRateCost",
      curvature_rate_rate_cost_weight,
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddImmediateFutureCost(
    int trajectory_steps, double trajectory_time_step,
    const std::vector<TrajectoryPoint>& prev_traj, double v_now,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  if (prev_traj.size() < 2) return;
  const Mfob::StateType x0 = Mfob::FitInitialState(prev_traj);
  const Mfob::ControlsType ref_us = Mfob::FitControl(prev_traj, x0);
  const Mfob::StatesType ref_xs = Mfob::FitState(prev_traj);

  // Make reference variables
  std::vector<double> ref_jerk;
  std::vector<double> ref_s;
  std::vector<double> ref_kappa;
  ref_jerk.reserve(trajectory_steps);
  ref_s.reserve(trajectory_steps);
  ref_kappa.reserve(trajectory_steps);
  for (int i = 0; i < trajectory_steps; ++i) {
    ref_jerk.push_back(Mfob::j(ref_us, i));
    ref_s.push_back(Mfob::s(ref_xs, i));
    ref_kappa.push_back(Mfob::kappa(ref_xs, i));
  }

  constexpr double kImmediateJGain = 0.1;
  const PiecewiseLinearFunction<double> lon_weight_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.immediate_future_cost_params().lon_weight_plf());
  std::vector<double> lon_weights;
  lon_weights.reserve(trajectory_steps);
  CHECK_EQ(prev_traj.size(), trajectory_steps);
  for (int i = 0; i < trajectory_steps; ++i) {
    const auto& prev_traj_point = prev_traj[i];
    const double lon_weight_plf_t = lon_weight_plf(prev_traj_point.t());
    lon_weights.push_back(kImmediateJGain * lon_weight_plf_t);
  }
  costs->push_back(
      std::make_unique<ReferenceLongitudinalJerkDeviationCost<Mfob>>(
          std::move(ref_jerk), std::move(lon_weights),
          "MfobReferenceLongitudinalJerkDeviationCost: ImmediateFuture",
          cost_weight_params.immediate_future_cost_weight(),
          /*cost_type=*/Cost<Mfob>::CostType::GROUP_IMMEDIATE_FUTURE));

  constexpr double kCurvatureDeviationCostWeight = 500.0;
  const PiecewiseLinearFunction<double> lat_weight_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.immediate_future_cost_params().lat_weight_plf());
  const PiecewiseLinearFunction<double> intersection_lat_weight_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.immediate_future_cost_params()
              .intersection_lat_weight_plf());

  std::vector<double> lat_weights;
  lat_weights.reserve(trajectory_steps);

  int cur_station_index =
      drive_passage
          .FindNearestStationIndex(Vec2d(plan_start_point.path_point().x(),
                                         plan_start_point.path_point().y()))
          .value();
  const Station& cur_station =
      drive_passage.station(StationIndex(cur_station_index));
  if (cur_station.station_info().is_in_intersection &&
      (cur_station.station_info().turn_type ==
           ad_byd::planning::TurnType::LEFT_TURN ||
       cur_station.station_info().turn_type ==
           ad_byd::planning::TurnType::RIGHT_TURN)) {
    for (int i = 0; i < trajectory_steps; ++i) {
      // use intersection_lat_weight_plf to reduce steering wheel twitching in
      // intersections
      lat_weights.push_back(intersection_lat_weight_plf(
          static_cast<double>(i * trajectory_time_step)));
    }
  } else {
    for (int i = 0; i < trajectory_steps; ++i) {
      lat_weights.push_back(
          lat_weight_plf(static_cast<double>(i * trajectory_time_step)));
    }
  }
  costs->push_back(std::make_unique<CurvatureDeviationCost<Mfob>>(
      ref_s, ref_kappa, std::move(lat_weights),
      "MfobCurvatureDeviationCost: ImmediateFuture",
      cost_weight_params.curvature_deviation_immediate_future_cost_weight() *
          kCurvatureDeviationCostWeight));
}

void GetReferencePathGainForRouteDestinationStopLine(
    const DrivePassage& drive_passage,
    const ConstraintManager& constraint_manager,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<double>* ref_path_deviation_gains,
    std::vector<double>* ref_heading_deviation_gains) {
  const auto& stop_lines = constraint_manager.StopLine();
  CHECK_EQ(ref_path_deviation_gains->size(),
           ref_heading_deviation_gains->size());

  std::optional<double> route_destination_stop_line_s;
  for (const auto& stop_line : stop_lines) {
    if (stop_line.source().type_case() ==
        SourceProto::TypeCase::kRouteDestination) {
      route_destination_stop_line_s = stop_line.s() - stop_line.standoff() -
                                      veh_geo_params.front_edge_to_center();
      break;
    }
  }
  if (!route_destination_stop_line_s.has_value()) {
    return;
  }

  const int count = ref_path_deviation_gains->size();
  const StationIndex route_destination_index =
      drive_passage.FindNearestStationIndexAtS(*route_destination_stop_line_s);
  for (int k = route_destination_index.value(); k < count; ++k) {
    (*ref_path_deviation_gains)[k] =
        cost_weight_params.reference_path_cost_weight().destination_path_gain();
    (*ref_heading_deviation_gains)[k] =
        cost_weight_params.reference_path_cost_weight()
            .destination_theta_gain();
  }
}

bool IsDetourRequired(
    const DrivePassage& drive_passage,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& veh_geo_params) {
  constexpr double kSafetyBuffer = 0.3;
  constexpr double kMinForwardCheckDistance = 50.0;
  const double ego_half_width = veh_geo_params.width() / 2.0;
  const double plan_start_s = plan_start_point.path_point().s();
  const double plan_start_v = plan_start_point.v();
  for (const auto& obj_traj : st_planner_object_traj.trajectories) {
    ASSIGN_OR_CONTINUE(
        const auto obj_frenet_box,
        drive_passage.QueryFrenetBoxAtContour(obj_traj.contour()));
    if (leading_trajs.find(std::string(obj_traj.traj_id())) !=
        leading_trajs.end()) {
      continue;
    }
    if (obj_frenet_box.center_s() < plan_start_s ||
        obj_frenet_box.center_s() >
            std::max(3.0 * plan_start_v, kMinForwardCheckDistance)) {
      continue;
    }
    // Detour is required if any non-leading-object is too close laterally
    if (std::abs(obj_frenet_box.l_min) < ego_half_width + kSafetyBuffer ||
        std::abs(obj_frenet_box.l_max) < ego_half_width + kSafetyBuffer) {
      Log2DDS::LogDataV2("IsDetourRequired",
                         "object id: " + std::string(obj_traj.traj_id()));
      return true;
    }
  }
  return false;
}

void GetReferencePathGainForVirtualNormalLane(
    const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const LaneChangeStage lc_stage, const NudgeInfos& nudge_info,
    const bool is_detour_required,
    std::vector<double>* ref_path_deviation_gains,
    std::vector<double>* ref_heading_deviation_gains,
    bool& virtual_normal_gain) {
  std::vector<int> change_indexs = drive_passage.change_index();

  if (!(lc_stage == LaneChangeStage::LCS_NONE && !change_indexs.empty())) {
    return;
  }
  // Disabled when detour
  const bool detour_required =
      !nudge_info.nudgeInfos.empty() || is_detour_required;
  if (detour_required) {
    Log2DDS::LogDataV2("opt_debug", "detour is required");
    return;
  }

  // for (const int i : change_indexs) {
  //   Log2DDS::LogDataV0("opt_debug", absl::StrCat("index: ", i));
  // }

  int index =
      drive_passage
          .FindNearestStationIndex(Vec2d(plan_start_point.path_point().x(),
                                         plan_start_point.path_point().y()))
          .value();

  double plan_start_s = plan_start_point.path_point().s();

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("plan_start_s: ", plan_start_s));
  Log2DDS::LogDataV0("opt_debug", absl::StrCat("index: ", index));

  // find first lane type change
  const auto dp_size = drive_passage.size();
  bool first_normal_to_virtual = false;
  int first_normal_to_virtual_index = index;
  double first_normal_to_virtual_s = plan_start_s;
  bool first_virtual_to_normal = false;
  double first_virtual_to_normal_s = plan_start_s;
  int first_virtual_to_normal_index = index;

  Log2DDS::LogDataV0(
      "opt_debug", absl::StrCat("change_indexs size: ", change_indexs.size()));

  for (int i = 0; i < change_indexs.size(); i++) {
    // if (change_indexs[i] < index) continue;
    Log2DDS::LogDataV0("opt_debug", absl::StrCat("i: ", i, " change_indexs[i] ",
                                                 change_indexs[i]));
    const Station& cur_station =
        drive_passage.station(StationIndex(change_indexs[i]));
    const Station& pre_station =
        drive_passage.station(StationIndex(change_indexs[i] - 1));

    // if (cur_station.accumulated_s() < -0.5 * kVirtualToNormal) continue;
    // reference line in the intersection shakes violently,
    // ignored the weight of the reference line to increase the stability of the
    // steering wheel.
    if (cur_station.station_info().is_in_intersection &&
        (cur_station.station_info().turn_type ==
             ad_byd::planning::TurnType::LEFT_TURN ||
         cur_station.station_info().turn_type ==
             ad_byd::planning::TurnType::RIGHT_TURN)) {
      (*ref_path_deviation_gains)[change_indexs[i]] = 2.0 * 0.1;
      (*ref_heading_deviation_gains)[change_indexs[i]] = 0.1;
    }

    Log2DDS::LogDataV0("opt_debug", absl::StrCat("pre_station_s: ",
                                                 pre_station.accumulated_s()));
    Log2DDS::LogDataV0("opt_debug", absl::StrCat("cur_station_s: ",
                                                 cur_station.accumulated_s()));

    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("pre_station_type: ",
                                    pre_station.station_info().turn_type));
    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("cur_station_type: ",
                                    cur_station.station_info().turn_type));

    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("pre_station_type: ",
                                    pre_station.station_info().split_topo));
    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("cur_station_type: ",
                                    cur_station.station_info().split_topo));

    if ((pre_station.station_info().turn_type ==
             ad_byd::planning::TurnType::NO_TURN &&
         cur_station.station_info().turn_type !=
             ad_byd::planning::TurnType::NO_TURN) ||
        (pre_station.station_info().split_topo ==
             ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE &&
         cur_station.station_info().split_topo !=
             ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE)) {
      first_normal_to_virtual = true;
      first_normal_to_virtual_s =
          drive_passage.station(StationIndex(change_indexs[i])).accumulated_s();
      first_normal_to_virtual_index = change_indexs[i];
    } else if ((pre_station.station_info().turn_type !=
                    ad_byd::planning::TurnType::NO_TURN &&
                cur_station.station_info().turn_type ==
                    ad_byd::planning::TurnType::NO_TURN) ||
               (pre_station.station_info().split_topo !=
                    ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE &&
                cur_station.station_info().split_topo ==
                    ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE)) {
      first_virtual_to_normal = true;
      first_virtual_to_normal_s =
          drive_passage.station(StationIndex(change_indexs[i])).accumulated_s();
      first_virtual_to_normal_index = change_indexs[i];
    }

    if (first_virtual_to_normal && first_normal_to_virtual) {
      break;
    }
  }

  // Log2DDS::LogDataV0("opt_debug", absl::StrCat("dp_size: ", dp_size));

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("first_normal_to_virtual: ",
                                               first_normal_to_virtual));
  Log2DDS::LogDataV0("opt_debug",
                     absl::StrCat("first_normal_to_virtual_index: ",
                                  first_normal_to_virtual_index));
  Log2DDS::LogDataV0("opt_debug", absl::StrCat("first_normal_to_virtual_s: ",
                                               first_normal_to_virtual_s));

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("first_virtual_to_normal: ",
                                               first_virtual_to_normal));
  Log2DDS::LogDataV0("opt_debug",
                     absl::StrCat("first_virtual_to_normal_index: ",
                                  first_virtual_to_normal_index));
  Log2DDS::LogDataV0("opt_debug", absl::StrCat("first_virtual_to_normal_s: ",
                                               first_virtual_to_normal_s));

  virtual_normal_gain = first_normal_to_virtual || first_virtual_to_normal;

  const int count = ref_path_deviation_gains->size();

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("count: ", count));

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("lc_stage: ", lc_stage));

  if (first_normal_to_virtual && !first_virtual_to_normal) {
    Log2DDS::LogDataV0("opt_debug", "1 scen");
    for (int i = index; i < first_normal_to_virtual_index && i < count; i++) {
      if (first_normal_to_virtual_s -
              drive_passage.station(StationIndex(i)).accumulated_s() <
          kNormalToVirtual) {
        (*ref_path_deviation_gains)[i] =
            2.0 * cost_weight_params.reference_path_cost_weight()
                      .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
  } else if (first_normal_to_virtual && first_virtual_to_normal &&
             first_normal_to_virtual_s < first_virtual_to_normal_s) {
    Log2DDS::LogDataV0("opt_debug", "2 scen");
    for (int i = index; i < first_normal_to_virtual_index && i < count; i++) {
      if (first_normal_to_virtual_s -
              drive_passage.station(StationIndex(i)).accumulated_s() <
          kNormalToVirtual) {
        (*ref_path_deviation_gains)[i] =
            2.0 * cost_weight_params.reference_path_cost_weight()
                      .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
    for (int i = first_virtual_to_normal_index; i < count; i++) {
      if (drive_passage.station(StationIndex(i)).accumulated_s() -
              first_virtual_to_normal_s <
          kVirtualToNormal) {
        (*ref_path_deviation_gains)[i] =
            2.0 * cost_weight_params.reference_path_cost_weight()
                      .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
  } else if (!first_normal_to_virtual && first_virtual_to_normal) {
    Log2DDS::LogDataV0("opt_debug", "3 scen");
    for (int i = first_virtual_to_normal_index; i < count; i++) {
      if (drive_passage.station(StationIndex(i)).accumulated_s() -
              first_virtual_to_normal_s <
          kVirtualToNormal) {
        (*ref_path_deviation_gains)[i] =
            0.8 * /* lane line twitch, lower weight to increase steering
                     stability */
            2.0 *
            cost_weight_params.reference_path_cost_weight()
                .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
  } else if (first_normal_to_virtual && first_virtual_to_normal &&
             first_normal_to_virtual_s > first_virtual_to_normal_s) {
    Log2DDS::LogDataV0("opt_debug", "4 scen");
    for (int i = first_virtual_to_normal_index; i < count; i++) {
      if (drive_passage.station(StationIndex(i)).accumulated_s() -
              first_virtual_to_normal_s <
          kVirtualToNormal) {
        (*ref_path_deviation_gains)[i] =
            2.0 * cost_weight_params.reference_path_cost_weight()
                      .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
  } else {
    virtual_normal_gain = false;
  }
}

std::tuple<bool, int, double, bool, int, double, int> GetSplitInfo(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point) {
  std::vector<int> change_indexs = drive_passage.change_index();
  int index =
      drive_passage
          .FindNearestStationIndex(Vec2d(plan_start_point.path_point().x(),
                                         plan_start_point.path_point().y()))
          .value();

  double plan_start_s = plan_start_point.path_point().s();

  Log2DDS::LogDataV0("opt_debug", "split");

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("plan_start_s: ", plan_start_s));
  Log2DDS::LogDataV0("opt_debug", absl::StrCat("index: ", index));

  // find first lane type change
  const auto dp_size = drive_passage.size();
  bool first_normal_to_virtual = false;
  int first_normal_to_virtual_index = index;
  double first_normal_to_virtual_s = plan_start_s;
  bool first_virtual_to_normal = false;
  double first_virtual_to_normal_s = plan_start_s;
  int first_virtual_to_normal_index = index;

  Log2DDS::LogDataV0(
      "opt_debug", absl::StrCat("change_indexs size: ", change_indexs.size()));

  for (int i = 0; i < change_indexs.size(); i++) {
    // if (change_indexs[i] < index) continue;
    Log2DDS::LogDataV0("opt_debug", absl::StrCat("i: ", i, " change_indexs[i] ",
                                                 change_indexs[i]));
    const Station& cur_station =
        drive_passage.station(StationIndex(change_indexs[i]));
    const Station& pre_station =
        drive_passage.station(StationIndex(change_indexs[i] - 1));

    Log2DDS::LogDataV0("opt_debug", absl::StrCat("pre_station_s: ",
                                                 pre_station.accumulated_s()));
    Log2DDS::LogDataV0("opt_debug", absl::StrCat("cur_station_s: ",
                                                 cur_station.accumulated_s()));

    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("pre_station_type: ",
                                    pre_station.station_info().turn_type));
    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("cur_station_type: ",
                                    cur_station.station_info().turn_type));

    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("pre_station_type: ",
                                    pre_station.station_info().split_topo));
    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("cur_station_type: ",
                                    cur_station.station_info().split_topo));
    constexpr double kBackwardExtendLength = 9.0;
    constexpr double kForwardExtendLength = 11.0;
    if (pre_station.station_info().split_topo ==
            ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE &&
        cur_station.station_info().split_topo !=
            ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE) {
      first_normal_to_virtual = true;
      first_normal_to_virtual_s =
          drive_passage.station(StationIndex(change_indexs[i]))
              .accumulated_s() -
          kBackwardExtendLength;
      first_normal_to_virtual_index =
          drive_passage.FindNearestStationIndexAtS(first_normal_to_virtual_s)
              .value();
    } else if (pre_station.station_info().split_topo !=
                   ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE &&
               cur_station.station_info().split_topo ==
                   ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE) {
      first_virtual_to_normal = true;
      first_virtual_to_normal_s =
          drive_passage.station(StationIndex(change_indexs[i]))
              .accumulated_s() +
          kForwardExtendLength;
      first_virtual_to_normal_index =
          drive_passage.FindNearestStationIndexAtS(first_virtual_to_normal_s)
              .value();
    }

    if (first_virtual_to_normal && first_normal_to_virtual) {
      break;
    }
  }

  return std::tuple<bool, int, double, bool, int, double, int>(
      first_normal_to_virtual, first_normal_to_virtual_index,
      first_normal_to_virtual_s, first_virtual_to_normal,
      first_virtual_to_normal_s, first_virtual_to_normal_index, index);
}

std::tuple<bool, int, double, bool, int, double, int> GetTurnInfo(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point) {
  std::vector<int> change_indexs = drive_passage.change_index();
  int index =
      drive_passage
          .FindNearestStationIndex(Vec2d(plan_start_point.path_point().x(),
                                         plan_start_point.path_point().y()))
          .value();

  double plan_start_s = plan_start_point.path_point().s();

  Log2DDS::LogDataV0("opt_debug", "turn");
  Log2DDS::LogDataV0("opt_debug", absl::StrCat("plan_start_s: ", plan_start_s));
  Log2DDS::LogDataV0("opt_debug", absl::StrCat("index: ", index));

  // find first lane type change
  const auto dp_size = drive_passage.size();
  bool first_normal_to_virtual = false;
  int first_normal_to_virtual_index = index;
  double first_normal_to_virtual_s = plan_start_s;
  bool first_virtual_to_normal = false;
  double first_virtual_to_normal_s = plan_start_s;
  int first_virtual_to_normal_index = index;

  Log2DDS::LogDataV0(
      "opt_debug", absl::StrCat("change_indexs size: ", change_indexs.size()));

  for (int i = 0; i < change_indexs.size(); i++) {
    // if (change_indexs[i] < index) continue;
    Log2DDS::LogDataV0("opt_debug", absl::StrCat("i: ", i, " change_indexs[i] ",
                                                 change_indexs[i]));
    const Station& cur_station =
        drive_passage.station(StationIndex(change_indexs[i]));
    const Station& pre_station =
        drive_passage.station(StationIndex(change_indexs[i] - 1));

    Log2DDS::LogDataV0("opt_debug", absl::StrCat("pre_station_s: ",
                                                 pre_station.accumulated_s()));
    Log2DDS::LogDataV0("opt_debug", absl::StrCat("cur_station_s: ",
                                                 cur_station.accumulated_s()));

    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("pre_station_type: ",
                                    pre_station.station_info().turn_type));
    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("cur_station_type: ",
                                    cur_station.station_info().turn_type));

    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("pre_station_type: ",
                                    pre_station.station_info().split_topo));
    Log2DDS::LogDataV0("opt_debug",
                       absl::StrCat("cur_station_type: ",
                                    cur_station.station_info().split_topo));

    if ((pre_station.station_info().turn_type ==
             ad_byd::planning::TurnType::NO_TURN &&
         cur_station.station_info().turn_type ==
             ad_byd::planning::TurnType::LEFT_TURN) ||
        (pre_station.station_info().turn_type ==
             ad_byd::planning::TurnType::NO_TURN &&
         cur_station.station_info().turn_type ==
             ad_byd::planning::TurnType::RIGHT_TURN)) {
      first_normal_to_virtual = true;
      first_normal_to_virtual_s =
          drive_passage.station(StationIndex(change_indexs[i])).accumulated_s();
      first_normal_to_virtual_index = change_indexs[i];
    } else if ((pre_station.station_info().turn_type ==
                    ad_byd::planning::TurnType::LEFT_TURN &&
                cur_station.station_info().turn_type ==
                    ad_byd::planning::TurnType::NO_TURN) ||
               (pre_station.station_info().turn_type ==
                    ad_byd::planning::TurnType::RIGHT_TURN &&
                cur_station.station_info().turn_type ==
                    ad_byd::planning::TurnType::NO_TURN)) {
      first_virtual_to_normal = true;
      first_virtual_to_normal_s =
          drive_passage.station(StationIndex(change_indexs[i])).accumulated_s();
      first_virtual_to_normal_index = change_indexs[i];
    }

    if (first_virtual_to_normal && first_normal_to_virtual) {
      break;
    }
  }

  return std::tuple<bool, int, double, bool, int, double, int>(
      first_normal_to_virtual, first_normal_to_virtual_index,
      first_normal_to_virtual_s, first_virtual_to_normal,
      first_virtual_to_normal_s, first_virtual_to_normal_index, index);
}

bool IsInUturn(
    const DrivePassage& drive_passage, int plan_id,
    const PathPoint& plan_start_point,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params) {
  bool flag = false;
  const auto& prefix = Log2DDS::TaskPrefix(plan_id);

  std::vector<int> change_indexs = drive_passage.change_index();
  int index = drive_passage
                  .FindNearestStationIndex(
                      Vec2d(plan_start_point.x(), plan_start_point.y()))
                  .value();
  double start_point_s =
      drive_passage
          .FindNearestStation(Vec2d(plan_start_point.x(), plan_start_point.y()))
          .accumulated_s();

  double plan_start_s = plan_start_point.s();
  const auto uturn_enhance_distance =
      cost_weight_params.uturn_speed_limit_distance();

  // find first lane type change
  const auto dp_size = drive_passage.size();
  bool first_normal_to_virtual = false;
  int first_normal_to_virtual_index = index;
  double first_normal_to_virtual_s = plan_start_s;
  bool first_virtual_to_normal = false;
  double first_virtual_to_normal_s = plan_start_s;
  int first_virtual_to_normal_index = index;

  for (int i = 0; i < change_indexs.size(); i++) {
    // if (change_indexs[i] < index) continue;
    const Station& cur_station =
        drive_passage.station(StationIndex(change_indexs[i]));
    const Station& pre_station =
        drive_passage.station(StationIndex(change_indexs[i] - 1));

    if ((pre_station.station_info().turn_type ==
             ad_byd::planning::TurnType::NO_TURN &&
         cur_station.station_info().turn_type ==
             ad_byd::planning::TurnType::U_TURN)) {
      first_normal_to_virtual = true;
      first_normal_to_virtual_s =
          drive_passage.station(StationIndex(change_indexs[i])).accumulated_s();
      first_normal_to_virtual_index = change_indexs[i];
    } else if ((pre_station.station_info().turn_type ==
                    ad_byd::planning::TurnType::U_TURN &&
                cur_station.station_info().turn_type ==
                    ad_byd::planning::TurnType::NO_TURN)) {
      first_virtual_to_normal = true;
      first_virtual_to_normal_s =
          drive_passage.station(StationIndex(change_indexs[i])).accumulated_s();
      first_virtual_to_normal_index = change_indexs[i];
    }

    if (first_virtual_to_normal && first_normal_to_virtual) {
      break;
    }
  }

  int scene = -1;

  if (first_normal_to_virtual && first_virtual_to_normal &&
      first_normal_to_virtual_s < first_virtual_to_normal_s) {
    if (start_point_s > first_normal_to_virtual_s - uturn_enhance_distance &&
        start_point_s < first_virtual_to_normal_s + uturn_enhance_distance) {
      flag = true;
      scene = 0;
    }
  } else if (first_normal_to_virtual && first_virtual_to_normal &&
             first_normal_to_virtual_s >= first_virtual_to_normal_s) {
    if (start_point_s > first_normal_to_virtual_s - uturn_enhance_distance ||
        start_point_s < first_virtual_to_normal_s + uturn_enhance_distance) {
      flag = true;
      scene = 1;
    }
  } else if (first_normal_to_virtual &&
             start_point_s >
                 first_normal_to_virtual_s - uturn_enhance_distance) {
    flag = true;
    scene = 2;
  } else if (first_virtual_to_normal &&
             start_point_s <
                 first_virtual_to_normal_s + uturn_enhance_distance) {
    flag = true;
    scene = 3;
  }
  Log2DDS::LogDataV2(prefix + "is_inuturn", scene);
  return flag;
}

void ChangeReferencePathGain(
    const DrivePassage& drive_passage,
    std::tuple<bool, int, double, bool, int, double, int>& change_info,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<double>* ref_path_deviation_gains,
    std::vector<double>* ref_heading_deviation_gains,
    bool& virtual_normal_gain) {
  auto [first_normal_to_virtual, first_normal_to_virtual_index,
        first_normal_to_virtual_s, first_virtual_to_normal,
        first_virtual_to_normal_s, first_virtual_to_normal_index, index] =
      change_info;

  // Log2DDS::LogDataV0("opt_debug", absl::StrCat("dp_size: ", dp_size));

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("first_normal_to_virtual: ",
                                               first_normal_to_virtual));
  Log2DDS::LogDataV0("opt_debug",
                     absl::StrCat("first_normal_to_virtual_index: ",
                                  first_normal_to_virtual_index));
  Log2DDS::LogDataV0("opt_debug", absl::StrCat("first_normal_to_virtual_s: ",
                                               first_normal_to_virtual_s));

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("first_virtual_to_normal: ",
                                               first_virtual_to_normal));
  Log2DDS::LogDataV0("opt_debug",
                     absl::StrCat("first_virtual_to_normal_index: ",
                                  first_virtual_to_normal_index));
  Log2DDS::LogDataV0("opt_debug", absl::StrCat("first_virtual_to_normal_s: ",
                                               first_virtual_to_normal_s));

  virtual_normal_gain = first_normal_to_virtual || first_virtual_to_normal;

  const int count = ref_path_deviation_gains->size();

  Log2DDS::LogDataV0("opt_debug", absl::StrCat("count: ", count));

  if (first_normal_to_virtual && !first_virtual_to_normal) {
    Log2DDS::LogDataV0("opt_debug", "1 scen");
    for (int i = index; i < first_normal_to_virtual_index && i < count; i++) {
      if (first_normal_to_virtual_s -
              drive_passage.station(StationIndex(i)).accumulated_s() <
          kNormalToVirtual) {
        (*ref_path_deviation_gains)[i] =
            2.0 * cost_weight_params.reference_path_cost_weight()
                      .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
  } else if (first_normal_to_virtual && first_virtual_to_normal &&
             first_normal_to_virtual_s < first_virtual_to_normal_s) {
    Log2DDS::LogDataV0("opt_debug", "2 scen");
    for (int i = index; i < first_normal_to_virtual_index && i < count; i++) {
      if (first_normal_to_virtual_s -
              drive_passage.station(StationIndex(i)).accumulated_s() <
          kNormalToVirtual) {
        (*ref_path_deviation_gains)[i] =
            2.0 * cost_weight_params.reference_path_cost_weight()
                      .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
    for (int i = first_virtual_to_normal_index; i < count; i++) {
      if (drive_passage.station(StationIndex(i)).accumulated_s() -
                  first_virtual_to_normal_s <
              kVirtualToNormal &&
          drive_passage.station(StationIndex(i)).station_info().turn_type ==
              ad_byd::planning::TurnType::NO_TURN) {
        (*ref_path_deviation_gains)[i] =
            2.0 * cost_weight_params.reference_path_cost_weight()
                      .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
  } else if (!first_normal_to_virtual && first_virtual_to_normal) {
    Log2DDS::LogDataV0("opt_debug", "3 scen");
    for (int i = first_virtual_to_normal_index; i < count; i++) {
      if (drive_passage.station(StationIndex(i)).accumulated_s() -
                  first_virtual_to_normal_s <
              kVirtualToNormal &&
          drive_passage.station(StationIndex(i)).station_info().turn_type ==
              ad_byd::planning::TurnType::NO_TURN) {
        (*ref_path_deviation_gains)[i] =
            1.0 * /* lane line twitch, lower weight to increase steering
                     stability */
            2.0 *
            cost_weight_params.reference_path_cost_weight()
                .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
  } else if (first_normal_to_virtual && first_virtual_to_normal &&
             first_normal_to_virtual_s > first_virtual_to_normal_s) {
    Log2DDS::LogDataV0("opt_debug", "4 scen");
    for (int i = first_virtual_to_normal_index; i < count; i++) {
      if (drive_passage.station(StationIndex(i)).accumulated_s() -
              first_virtual_to_normal_s <
          kVirtualToNormal) {
        (*ref_path_deviation_gains)[i] =
            2.0 * cost_weight_params.reference_path_cost_weight()
                      .intersection_path_gain();
        (*ref_heading_deviation_gains)[i] =
            cost_weight_params.reference_path_cost_weight()
                .intersection_theta_gain();
      }
    }
  } else {
    virtual_normal_gain = false;
  }
}

void GetReferencePathGainForSplit(
    const DrivePassage& drive_passage, const NudgeInfos& nudge_info,
    const VehicleGeometryParamsProto& veh_geo_params,
    const bool is_detour_required,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const LaneChangeStage lc_stage,
    std::vector<double>* ref_path_deviation_gains,
    std::vector<double>* ref_heading_deviation_gains,
    bool& virtual_normal_gain) {
  std::vector<int> change_indexs = drive_passage.change_index();
  if (!(lc_stage == LaneChangeStage::LCS_NONE && !change_indexs.empty())) {
    return;
  }
  // Disabled when detour
  if (is_detour_required) {
    Log2DDS::LogDataV0("opt_debug", "detour is required");
    return;
  }
  auto split_change_info = GetSplitInfo(drive_passage, plan_start_point);

  ChangeReferencePathGain(drive_passage, split_change_info, cost_weight_params,
                          ref_path_deviation_gains, ref_heading_deviation_gains,
                          virtual_normal_gain);
}

void GetReferencePathGainForTurn(
    const DrivePassage& drive_passage, const NudgeInfos& nudge_info,
    const VehicleGeometryParamsProto& veh_geo_params,
    const bool is_detour_required,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const LaneChangeStage lc_stage,
    std::vector<double>* ref_path_deviation_gains,
    std::vector<double>* ref_heading_deviation_gains,
    bool& virtual_normal_gain) {
  std::vector<int> change_indexs = drive_passage.change_index();
  if (!(lc_stage == LaneChangeStage::LCS_NONE && !change_indexs.empty())) {
    return;
  }
  // Disabled when detour
  if (is_detour_required) {
    Log2DDS::LogDataV0("opt_debug", "detour is required");
    return;
  }
  auto turn_change_info = GetTurnInfo(drive_passage, plan_start_point);

  ChangeReferencePathGain(drive_passage, turn_change_info, cost_weight_params,
                          ref_path_deviation_gains, ref_heading_deviation_gains,
                          virtual_normal_gain);
}

bool NeedEnhanceCentralityCost(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const LaneChangeStage& lc_stage, const CenterLineOffsetType& offset_type,
    bool* const has_intersection) {
  CHECK_NOTNULL(has_intersection);
  *has_intersection = false;
  if (lc_stage != LaneChangeStage::LCS_NONE ||
      offset_type == CenterLineOffsetType::CENTER_LINE_OFFSET_PAUSE ||
      offset_type == CenterLineOffsetType::CENTER_LINE_OFFSET_PUSH ||
      offset_type ==
          CenterLineOffsetType::CENTER_LINE_OFFSET_LARGE_VEHICLE_AVOID) {
    Log2DDS::LogDataV2(
        "lc_or_push_or_pause_avoid",
        absl::StrCat("lc_stage: ", lc_stage, ", ref_type: ", offset_type));
    return false;
  }

  constexpr double kMaxIntersectionDsThreshold = 60.0;
  constexpr double kDsStepForIntersection = 8.0;
  const auto av_sl_pos_or = drive_passage.QueryFrenetCoordinateAt(Vec2d(
      plan_start_point.path_point().x(), plan_start_point.path_point().y()));
  if (!av_sl_pos_or.ok()) return false;
  for (double s = av_sl_pos_or->s;
       s < av_sl_pos_or->s + kMaxIntersectionDsThreshold;
       s += kDsStepForIntersection) {
    const auto& cur_station = drive_passage.FindNearestStationAtS(s);
    if (cur_station.is_in_intersection()) {
      *has_intersection = true;
      return false;
    }
    if (cur_station.station_info().turn_type ==
        ad_byd::planning::TurnType::U_TURN) {
      Log2DDS::LogDataV0("opt_debug", "has UTURN");
      return false;
    }
  }
  double max_kappa = std::numeric_limits<double>::min();
  constexpr double kDsStepForKappa = 4.0;
  constexpr double kMaxKappaDsThreshold = 33.0;
  for (double s = av_sl_pos_or->s; s < av_sl_pos_or->s + kMaxKappaDsThreshold;
       s += kDsStepForKappa) {
    const auto& cur_station = drive_passage.FindNearestStationAtS(s);
    const double cur_s = cur_station.accumulated_s();
    const double cur_theta = cur_station.tangent().FastAngle();
    const double next_s = cur_s + kDsStepForKappa;
    const auto& next_station = drive_passage.FindNearestStationAtS(next_s);
    const double next_theta = next_station.tangent().FastAngle();
    const double abs_kappa =
        std::fabs(NormalizeAngle(next_theta - cur_theta) / kDsStepForKappa);
    max_kappa = std::max(max_kappa, abs_kappa);
  }
  Log2DDS::LogDataV2("max_kappa", absl::StrCat("max_kappa: ", max_kappa));
  if (max_kappa <
      cost_weight_params.reference_path_cost_weight().enhance_gain_kappa()) {
    return false;
  }

  return true;
}

void AddReferencePathCost(
    int trajectory_steps, const PathSlBoundary& path_sl_boundary,
    std::vector<double> gains, std::vector<double> ref_path_deviation_gains,
    std::vector<double> ref_heading_deviation_gains,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>&
        reference_center_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  std::vector<double> ref_ls(
      path_sl_boundary.reference_center_l_vector().begin(),
      path_sl_boundary.reference_center_l_vector().end());
  // ref_ls.size() should be the same with ref_path_deviation_gains.size().
  ref_ls.pop_back();
  costs->emplace_back(std::make_unique<ReferenceLineDeviationCost<Mfob>>(
      trajectory_steps, std::move(ref_path_deviation_gains),
      cost_weight_params.reference_path_cost_weight().end_state_gain(),
      std::move(ref_ls), reference_center_query_helper->points(),
      reference_center_query_helper.get(), std::move(gains),
      "MfobReferenceLineDeviationCost",
      /*scale*/ 1.0, /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));

  // // Compute ref_thetas.
  // const std::vector<Vec2d> ref_path_vector(
  //     path_sl_boundary.reference_center_xy_vector().begin(),
  //     path_sl_boundary.reference_center_xy_vector().end());
  // std::vector<double> ref_thetas;
  // ref_thetas.reserve(ref_path_vector.size() - 1);
  // for (int i = 1; i < ref_path_vector.size(); ++i) {
  //   ref_thetas.push_back(
  //       (ref_path_vector[i] - ref_path_vector[i - 1]).FastAngle());
  // }
  // costs->emplace_back(std::make_unique<EndHeadingCost<Mfob>>(
  //     trajectory_steps, std::move(ref_thetas),
  //     reference_center_query_helper->points(),
  //     reference_center_query_helper.get(),
  //     std::move(ref_heading_deviation_gains), "MfobEndHeadingCost",
  //     cost_weight_params.reference_path_cost_weight()
  //         .reference_heading_cost_weight(),
  //     /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddCaptainReferenceTrajectoryCost(
    int trajectory_steps, std::string_view base_name,
    const std::vector<TrajectoryPoint>& captain_traj,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  if (captain_traj.empty()) return;
  // If captain trajectory size is smaller than ddp horizon, fill a pseudo
  // trajectory up to the horizon.
  auto captain_ref_traj = captain_traj;
  while (captain_ref_traj.size() < trajectory_steps) {
    captain_ref_traj.push_back(TrajectoryPoint());
  }
  auto ref_xs = Mfob::FitState(captain_ref_traj);
  std::vector<double> captain_ref_weights(ref_xs.size(), 0.0);
  // TODO: Move the weights to params and tune them.
  // TODO: Add more weights if captain trajectory provides more
  // quantities.
  constexpr double kCaptainRefPosWeight = 1.0;
  constexpr double kCaptainRefHeadingWeight = 5.0;
  for (int i = 0; i < captain_traj.size(); ++i) {
    captain_ref_weights[i * Mfob::kStateSize + Mfob::kStateXIndex] =
        kCaptainRefPosWeight;
    captain_ref_weights[i * Mfob::kStateSize + Mfob::kStateYIndex] =
        kCaptainRefPosWeight;
    captain_ref_weights[i * Mfob::kStateSize + Mfob::kStateThetaIndex] =
        kCaptainRefHeadingWeight;
  }
  costs->emplace_back(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
      std::move(ref_xs), std::move(captain_ref_weights),
      "MfobReferenceStateDeviationCost: CaptainReference",
      /*scale=*/1.0,
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddReferenceEndStateCost(
    int trajectory_steps, double ref_end_state_s,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs, const bool in_uturn) {
  auto end_state_cost_params = cost_weight_params.end_state_cost_params();
  if (in_uturn) {
    end_state_cost_params = cost_weight_params.uturn_end_state_cost_params();
  }
  Mfob::StateType end_state = Mfob::StateType::Zero();
  end_state[Mfob::kStateSIndex] = ref_end_state_s;

  std::vector<double> state_regularization_weights(end_state.size(), 0.0);
  std::vector<double> base_numbers(end_state.size(),
                                   std::numeric_limits<double>::infinity());
  for (int i = 0; i < Mfob::kStateSize; ++i) {
    state_regularization_weights[i] = end_state_cost_params.w(i);
    base_numbers[i] = end_state_cost_params.base_numbers(i);
    Log2DDS::LogDataV0("endstate", state_regularization_weights[i]);
  }
  costs->emplace_back(std::make_unique<ReferenceSingleStateDeviationCost<Mfob>>(
      std::move(end_state), /*ref_index=*/trajectory_steps - 1,
      std::move(state_regularization_weights), std::move(base_numbers),
      "MfobReferenceStateDeviationCost: EndStateAttraction",
      /*scale=*/end_state_cost_params.weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

bool NarrowScene(const DrivePassage& drive_passage,
                 const ApolloTrajectoryPointProto& plan_start_point,
                 double* lane_width) {
  const auto ego_lane_boundary_info =
      drive_passage.QueryEnclosingLaneBoundariesAtS(
          plan_start_point.path_point().s());
  *lane_width = ego_lane_boundary_info.left->lat_offset -
                ego_lane_boundary_info.right->lat_offset;
  constexpr double narrow_lane_width = 2.9;

  Log2DDS::LogDataV2(
      "ego_lane_boundary_info",
      absl::StrCat("left: ", ego_lane_boundary_info.left->lat_offset,
                   ", right: ", ego_lane_boundary_info.right->lat_offset));
  return (*lane_width) < narrow_lane_width ? true : false;
}

void ModifyPathGainsForSplitMerge(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const double split_first_gain_scale, const double split_second_gain_scale,
    const double split_first_distance, const double split_second_distance,
    const double split_backward_min_scale, const double split_forward_min_scale,
    std::vector<double>* const path_gains) {
  CHECK_NOTNULL(path_gains);
  std::vector<int> change_indexs = drive_passage.change_index();
  if (change_indexs.empty()) {
    Log2DDS::LogDataV0("split_merge_debug", "change_indexs.size is zero");
    return;
  }

  double plan_start_s = plan_start_point.path_point().s();
  Log2DDS::LogDataV0("split_merge_debug",
                     absl::StrCat("plan_start_s: ", plan_start_s));
  const double half_ignore_distance =
      std::max(split_first_distance, split_second_distance) / 2.0;
  for (int i = 0; i < change_indexs.size(); i++) {
    const Station& cur_station =
        drive_passage.station(StationIndex(change_indexs[i]));
    const Station& pre_station =
        drive_passage.station(StationIndex(std::max(change_indexs[i] - 1, 0)));
    const bool is_split =
        pre_station.station_info().split_topo ==
            ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE &&
        cur_station.station_info().split_topo !=
            ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE;
    const bool is_merge =
        pre_station.station_info().merge_topo !=
            ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_NONE &&
        cur_station.station_info().merge_topo ==
            ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_NONE;
    if (is_split || is_merge) {
      Log2DDS::LogDataV0("split_merge_debug",
                         absl::StrCat("is_split: ", is_split));
      Log2DDS::LogDataV0("split_merge_debug",
                         absl::StrCat("is_merge: ", is_merge));
      Log2DDS::LogDataV0(
          "split_merge_debug",
          absl::StrCat("pre_station_s: ", pre_station.accumulated_s()));
      Log2DDS::LogDataV0(
          "split_merge_debug",
          absl::StrCat("cur_station_s: ", cur_station.accumulated_s()));
      const double backward_gain_scale =
          is_split ? split_first_gain_scale : split_second_gain_scale;
      const double forward_gain_scale =
          is_split ? split_second_gain_scale : split_first_gain_scale;
      const double backward_ds_length =
          is_split ? split_first_distance : split_second_distance;
      const double forward_ds_length =
          is_split ? split_second_distance : split_first_distance;
      const int split_index = change_indexs[i];
      const double split_s =
          drive_passage.station(StationIndex(split_index)).accumulated_s();
      if (split_s < plan_start_s - half_ignore_distance) {
        continue;
      }
      for (int i = 0; i < split_index && i < path_gains->size(); i++) {
        const double ds =
            split_s - drive_passage.station(StationIndex(i)).accumulated_s();
        if (ds < backward_ds_length) {
          const double scale =
              std::max(backward_gain_scale * ds / backward_ds_length,
                       split_backward_min_scale);
          path_gains->at(i) *= scale;
        }
      }
      for (int i = split_index; i < path_gains->size(); i++) {
        const double ds =
            drive_passage.station(StationIndex(i)).accumulated_s() - split_s;
        if (ds < split_second_distance) {
          const double scale =
              std::max(forward_gain_scale * ds / split_second_distance,
                       split_forward_min_scale);
          path_gains->at(i) *= scale;
        } else {
          break;
        }
      }
      break;
    }
  }
}

void ModifyRefDeviationGains(
    const DrivePassage& drive_passage, const double trajectory_time_step,
    const ApolloTrajectoryPointProto& plan_start_point,
    const st::Behavior_FunctionId& function_id,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const LaneChangeStage lc_stage, const PathSlBoundary& path_sl_boundary,
    bool* const ref_enhance, std::vector<double>* const path_gains,
    std::vector<double>* const ref_deviation_gains) {
  CHECK_NOTNULL(ref_enhance);
  CHECK_NOTNULL(path_gains);
  CHECK_NOTNULL(ref_deviation_gains);
  *ref_enhance = false;
  bool has_intersection = false;
  const auto& ref_offset = path_sl_boundary.offset_type_value();
  const bool enhance_centrality = NeedEnhanceCentralityCost(
      drive_passage, plan_start_point, cost_weight_params, lc_stage,
      ref_offset.first, &has_intersection);
  constexpr double kCTLowSpeedMaxThreshold = 60.0 / 3.6;
  constexpr double kHWL2LowSpeedMaxThreshold = 30.0 / 3.6;
  const bool is_HW_L2 =
      (function_id == st::Behavior_FunctionId::Behavior_FunctionId_HW_NOA ||
       function_id == st::Behavior_FunctionId::Behavior_FunctionId_LKA_PLUS ||
       function_id == st::Behavior_FunctionId::Behavior_FunctionId_LKA);
  const double low_speed_max_threshold =
      is_HW_L2 ? kHWL2LowSpeedMaxThreshold : kCTLowSpeedMaxThreshold;
  bool enhance_final = false;
  if (plan_start_point.v() < low_speed_max_threshold) {
    enhance_final = enhance_centrality;
  } else {
    enhance_final = has_intersection ? false : true;
  }
  const PiecewiseLinearFunction<double> speed_path_gain_scale_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.reference_path_cost_weight()
              .speed_path_gain_scale_plf());
  const double speed_gain_scale =
      speed_path_gain_scale_plf(std::fabs(plan_start_point.v()));
  Log2DDS::LogDataV2(
      "centrality cost",
      absl::StrCat("enhance_tmp: ", enhance_centrality,
                   ", plan_start_point.v: ", plan_start_point.v(),
                   ", has_intersection: ", has_intersection,
                   ", enhance_final: ", enhance_final, ", is_HW_L2: ", is_HW_L2,
                   ", speed_gain_scale: ", speed_gain_scale));
  if (enhance_final) {
    const auto av_sl_pos_or = drive_passage.QueryFrenetCoordinateAt(Vec2d(
        plan_start_point.path_point().x(), plan_start_point.path_point().y()));
    if (!av_sl_pos_or.ok() || lc_stage != LaneChangeStage::LCS_NONE ||
        ref_offset.first == CenterLineOffsetType::CENTER_LINE_OFFSET_PAUSE ||
        ref_offset.first == CenterLineOffsetType::CENTER_LINE_OFFSET_PUSH) {
      const PiecewiseLinearFunction<double> progressive_path_gain_plf =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.reference_path_cost_weight()
                  .progressive_path_gain_plf());
      for (int i = 0; i < ref_deviation_gains->size(); ++i) {
        ref_deviation_gains->at(i) = progressive_path_gain_plf(
            static_cast<double>(i * trajectory_time_step));
      }
      return;
    }
    if (std::fabs(av_sl_pos_or->l) >
        cost_weight_params.reference_path_cost_weight()
            .hw_path_gain_threshold()) {
      const PiecewiseLinearFunction<double> hw_progressive_path_gain_plf =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.reference_path_cost_weight()
                  .hw_progressive_path_gain_plf());
      for (int i = 0; i < ref_deviation_gains->size(); ++i) {
        ref_deviation_gains->at(i) = hw_progressive_path_gain_plf(
            static_cast<double>(i * trajectory_time_step));
      }
    } else {
      *ref_enhance = true;
      const auto& ref_conf = cost_weight_params.reference_path_cost_weight();
      ModifyPathGainsForSplitMerge(
          drive_passage, plan_start_point, ref_conf.split_first_gain_scale(),
          ref_conf.split_second_gain_scale(), ref_conf.split_first_distance(),
          ref_conf.split_second_distance(), ref_conf.split_backward_min_scale(),
          ref_conf.split_forward_min_scale(), path_gains);
      const PiecewiseLinearFunction<double> hw_degressive_path_gain_plf =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.reference_path_cost_weight()
                  .hw_degressive_path_gain_plf());
      for (int i = 0; i < ref_deviation_gains->size(); ++i) {
        ref_deviation_gains->at(i) =
            hw_degressive_path_gain_plf(
                static_cast<double>(i * trajectory_time_step)) *
            speed_gain_scale;
      }
    }
  } else {
    const PiecewiseLinearFunction<double> progressive_path_gain_plf =
        PiecewiseLinearFunctionFromProto(
            cost_weight_params.reference_path_cost_weight()
                .progressive_path_gain_plf());
    for (int i = 0; i < ref_deviation_gains->size(); ++i) {
      ref_deviation_gains->at(i) = progressive_path_gain_plf(
          static_cast<double>(i * trajectory_time_step));
    }
  }
}

absl::Status AddCosts(
    const int plan_id, int trajectory_steps, double trajectory_time_step,
    double avoid_dynamic_obj_early_time, std::string_view base_name,
    const st::Behavior_FunctionId& function_id,
    const ApolloTrajectoryPointProto& plan_start_point,
    const LaneChangeStage lc_stage, PushState push_dir, const bool borrow_lane,
    const NudgeInfos& nudge_info,
    const std::vector<TrajectoryPoint>& initializer_traj,
    const std::vector<TrajectoryPoint>& prev_traj,
    const std::vector<TrajectoryPoint>& captain_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const ConstraintManager& constraint_manager,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const PlannerSemanticMapManager& psmm, double v_now,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const TrajectoryOptimizerCostConfigProto& cost_config,
    const TrajectoryOptimizerValidationParamsProto&
        trajectory_optimizer_validation_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const PlannerFunctionsParamsProto& planner_functions_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    const bool is_narrow_scene, std::vector<std::unique_ptr<Cost<Mfob>>>* costs,
    TrajectoryOptimizerDebugProto* traj_opt_debug_proto,
    ThreadPool* thread_pool, optimizer::TurnType ego_turn_type,
    const bool in_uturn) {
  SCOPED_TRACE(__FUNCTION__);
  TIMELINE("AddCosts");
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);
  if (cost_config.enable_regularizers_cost()) {
    AddRegularizersCost(initializer_traj, cost_weight_params, costs);
  }
  bool is_kappa_inhibit_scene = false;
  if (cost_config.enable_curvature_cost()) {
    AddCurvatureCost(trajectory_time_step, initializer_traj.front(),
                     constraint_manager, st_planner_object_traj, leading_trajs,
                     drive_passage, cost_weight_params, veh_geo_params,
                     vehicle_drive_params, motion_constraint_params, costs,
                     is_kappa_inhibit_scene, in_uturn);
  }
  if (cost_config.enable_forward_speed_cost()) {
    // Forward speed cost.
    costs->emplace_back(std::make_unique<ForwardSpeedCost<Mfob>>(
        "MfobForwardSpeedCost", cost_weight_params.forward_speed_cost_weight(),
        /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  }

  if (cost_config.enable_immediate_future_cost() && !in_uturn) {
    AddImmediateFutureCost(trajectory_steps, trajectory_time_step, prev_traj,
                           v_now, cost_weight_params, drive_passage,
                           plan_start_point, costs);
  }

  bool ref_enhance = false;
  if (cost_config.enable_reference_path_cost()) {
    std::vector<double> gains(
        stations_query_helper->points().size() - 1,
        cost_weight_params.reference_path_cost_weight().path_gain());
    std::vector<double> ref_path_deviation_gains(
        trajectory_steps, cost_weight_params.reference_path_cost_weight()
                              .reference_path_cost_weight());
    std::vector<double> ref_heading_deviation_gains(
        trajectory_steps, cost_weight_params.reference_path_cost_weight()
                              .reference_heading_cost_weight());
    if (captain_traj.empty() && !in_uturn) {
      ModifyRefDeviationGains(drive_passage, trajectory_time_step,
                              plan_start_point, function_id, cost_weight_params,
                              lc_stage, path_sl_boundary, &ref_enhance, &gains,
                              &ref_path_deviation_gains);
    }
    AddReferencePathCost(trajectory_steps, path_sl_boundary, std::move(gains),
                         std::move(ref_path_deviation_gains),
                         std::move(ref_heading_deviation_gains),
                         cost_weight_params, stations_query_helper, costs);
  }

  std::vector<double> inner_path_boudnary_gains(path_sl_boundary.size(), 1.0);
  std::vector<optimizer::LeadingInfo> leading_min_s(
      free_index + 1, {
                          std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(),
                      });
  const auto& ref_offset = path_sl_boundary.offset_type_value();
  bool keep_and_push_or_pause =
      ref_offset.first == CenterLineOffsetType::CENTER_LINE_OFFSET_PUSH ||
      ref_offset.first == CenterLineOffsetType::CENTER_LINE_OFFSET_PAUSE;
  Log2DDS::LogDataV2("push_or_pause",
                     absl::StrCat("push_dir: ", push_dir, ", lc_stage: ",
                                  lc_stage, ", ref_type: ", ref_offset.first,
                                  ", ref_value: ", ref_offset.second));
  if (cost_config.enable_object_cost() && !keep_and_push_or_pause) {
    const auto path_time_corridor = optimizer::BuildPathTimeCorridor(
        plan_id, base_name, initializer_traj, drive_passage, path_sl_boundary,
        leading_trajs, st_traj_mgr, st_planner_object_traj, veh_geo_params,
        trajectory_time_step);
    if (path_time_corridor.ok()) {
      optimizer::AddObjectCosts(
          plan_id, lc_stage, ref_enhance, borrow_lane, nudge_info,
          trajectory_steps, trajectory_time_step, avoid_dynamic_obj_early_time,
          base_name, initializer_traj, drive_passage, path_sl_boundary,
          *path_time_corridor, leading_trajs, st_traj_mgr,
          st_planner_object_traj, cost_weight_params, veh_geo_params,
          motion_constraint_params, trajectory_optimizer_vehicle_model_params,
          av_model_helpers, is_narrow_scene, &leading_min_s,
          &inner_path_boudnary_gains, costs, thread_pool, ego_turn_type,
          prev_traj);
    }
  }

  double ref_end_state_s = std::numeric_limits<double>::infinity();
  if (cost_config.enable_speed_limit_cost()) {
    optimizer::AddSpeedLimitCost(
        lc_stage, trajectory_steps, trajectory_time_step,
        initializer_traj.front(), drive_passage, constraint_manager,
        cost_weight_params, motion_constraint_params, veh_geo_params,
        stations_query_helper, leading_min_s, &ref_end_state_s, costs,
        traj_opt_debug_proto, plan_id, function_id);
  }
  ref_end_state_s = std::min(ref_end_state_s, initializer_traj.back().s());
  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  Log2DDS::LogDataV2(prefix + "ref_end_state_s", ref_end_state_s);

  if (cost_config.enable_reference_end_state_cost()) {
    AddReferenceEndStateCost(trajectory_steps, ref_end_state_s,
                             cost_weight_params, costs, in_uturn);
  }

  std::optional<double> extra_curb_buffer = std::nullopt;
  const auto ego_lane_boundary_info =
      drive_passage.QueryEnclosingLaneBoundariesAtS(
          plan_start_point.path_point().s());
  double lane_width = ego_lane_boundary_info.left->lat_offset -
                      ego_lane_boundary_info.right->lat_offset;
  const Station& ego_station =
      drive_passage.FindNearestStationAtS(plan_start_point.path_point().s());

  if (ego_station.is_in_intersection()) {
    const Station& normal_station =
        drive_passage.GetFirstNonIntersectionStation(
            Vec2d(plan_start_point.path_point().x(),
                  plan_start_point.path_point().y()));
    const auto first_non_intersection_lane_boundary_info =
        normal_station.QueryEnclosingLaneBoundariesAt(/*signed_lat=*/0.0)
            .value();
    lane_width = first_non_intersection_lane_boundary_info.left->lat_offset -
                 first_non_intersection_lane_boundary_info.right->lat_offset;
  }

  Log2DDS::LogDataV2(
      "lane_width",
      absl::StrCat(ego_station.is_in_intersection(), " ", lane_width));

  auto const left_lane_type = ego_lane_boundary_info.left->type;
  auto const right_lane_type = ego_lane_boundary_info.right->type;

  const auto lane_width_buffer_plf = PiecewiseLinearFunctionFromProto(
      cost_weight_params.lane_width_rel_hard_curb_clearance_plf());

  const std::optional<double> lane_width_curb_buffer_opt =
      lane_width_buffer_plf(lane_width);
  if (cost_config.enable_static_boundary_cost()) {
    const bool enable_three_point_turn =
        (FLAGS_planner_runtime_uturn_level == 2 ||
         (FLAGS_planner_runtime_uturn_level == 1 &&
          planner_functions_params.enable_three_point_turn()));
    // Curb and path boundary costs.
    optimizer::AddStaticBoundaryCosts(
        lc_stage, trajectory_steps, base_name, enable_three_point_turn,
        initializer_traj.front(), drive_passage, psmm, nudge_info,
        path_sl_boundary, inner_path_boudnary_gains, cost_weight_params,
        veh_geo_params, trajectory_optimizer_vehicle_model_params,
        stations_query_helper, lane_width_curb_buffer_opt, lane_width,
        &extra_curb_buffer, costs);
  }
  if (cost_config.enable_msd_road_boundary_cost()) {
    optimizer::AddRoadBoundaryCost(
        trajectory_steps, base_name, psmm, drive_passage, plan_start_point,
        trajectory_optimizer_vehicle_model_params, veh_geo_params,
        cost_weight_params, stations_query_helper, costs);
  }
  if (cost_config.enable_acceleration_and_jerk_cost()) {
    AddAccelAndJerkCost(
        trajectory_steps, trajectory_time_step, initializer_traj.front(), psmm,
        trajectory_optimizer_vehicle_model_params, veh_geo_params,
        cost_weight_params, motion_constraint_params,
        trajectory_optimizer_validation_params, extra_curb_buffer, lane_width,
        costs, in_uturn);
  }
  if (cost_config.enable_captain_reference_trajectory_cost()) {
    AddCaptainReferenceTrajectoryCost(trajectory_steps, base_name, captain_traj,
                                      costs);
  }
  return absl::OkStatus();
}

absl::Status AddStationQueryHelper(
    int trajectory_steps, std::string_view base_name,
    const DrivePassage& drive_passage,
    std::unique_ptr<CenterLineQueryHelper<Mfob>>* station_query_helper) {
  std::vector<Vec2d> station_points;
  station_points.reserve(drive_passage.size());
  for (const auto& station : drive_passage.stations()) {
    station_points.push_back(station.xy());
  }
  *station_query_helper = std::make_unique<CenterLineQueryHelper<Mfob>>(
      trajectory_steps, station_points,
      drive_passage.last_real_station_index().value(), "MfobStationQueryHelper",
      /*use_qtfm=*/true);
  return absl::OkStatus();
}

std::unique_ptr<AvModelHelper<Mfob>> AddAvModelHelpers(
    int trajectory_steps, std::string_view base_name,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params) {
  std::vector<double> dists_to_rac;
  std::vector<double> angles_to_axis;
  const int circle_size =
      trajectory_optimizer_vehicle_model_params.circles_size() +
      trajectory_optimizer_vehicle_model_params.mirror_circles_size();
  dists_to_rac.reserve(circle_size);
  angles_to_axis.reserve(circle_size);
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
  }
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.mirror_circles()) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
  }
  return std::make_unique<AvModelHelper<Mfob>>(
      trajectory_steps, dists_to_rac, angles_to_axis, "MfobAvModelHelper");
}

// Extend by PurePursuit with a constant speed.
std::vector<TrajectoryPoint> GetExtendStateByPurePursuit(
    double trajectory_time_step, const TrajectoryPoint& extend_start_point,
    int k_extend_steps, double target_v, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params) {
  CHECK_GT(trajectory_time_step, 0.0);
  // Extend by PurePursuit with a constant speed. Copied from
  // GetExtendStateByPurePursuit.
  constexpr double kLateralLookAhead = 1.0;
  Mfob::StateType x = Mfob::MakeState(
      extend_start_point.pos().x(), extend_start_point.pos().y(),
      extend_start_point.theta(), extend_start_point.v(),
      extend_start_point.kappa(), extend_start_point.a(),
      extend_start_point.psi(), extend_start_point.s());
  std::vector<TrajectoryPoint> res;
  const double time_base = extend_start_point.t();
  while (res.size() < k_extend_steps) {
    const double a = std::clamp<double>(
        (target_v - Mfob::StateGetV(x)) / trajectory_time_step,
        motion_constraint_params.max_deceleration(),
        motion_constraint_params.max_acceleration());
    const double j =
        std::clamp<double>((a - Mfob::StateGetA(x)) / trajectory_time_step,
                           motion_constraint_params.max_decel_jerk(),
                           motion_constraint_params.max_accel_jerk());
    Vec2d lateral_target_pos = drive_passage.stations().back().xy();
    const double lateral_look_ahead_dist =
        kLateralLookAhead * Mfob::StateGetV(x) + veh_geo_params.wheel_base();
    const auto lateral_nearest_point_status =
        drive_passage.QueryFrenetCoordinateAt(Mfob::StateGetPos(x));
    if (lateral_nearest_point_status.ok()) {
      lateral_target_pos = path_sl_boundary.QueryReferenceCenterXY(
          lateral_look_ahead_dist + lateral_nearest_point_status->s);
    }
    const double alpha =
        Vec2d(lateral_target_pos - Mfob::StateGetPos(x)).Angle() -
        Mfob::StateGetTheta(x);
    const double kappa = 2.0 * std::sin(alpha) / lateral_look_ahead_dist;
    double psi = (kappa - Mfob::StateGetKappa(x)) / trajectory_time_step;

    double chi = (psi - Mfob::StateGetPsi(x)) / trajectory_time_step;

    const auto u = Mfob::MakeControl(chi, j);
    x = Mfob::EvaluateF(res.size() - 1, x, u, trajectory_time_step);

    TrajectoryPoint next_pt;
    next_pt.set_pos(Mfob::StateGetPos(x));
    next_pt.set_theta(Mfob::StateGetTheta(x));
    next_pt.set_kappa(Mfob::StateGetKappa(x));
    next_pt.set_psi(Mfob::StateGetPsi(x));
    next_pt.set_v(Mfob::StateGetV(x));
    next_pt.set_a(Mfob::StateGetA(x));
    next_pt.set_s(Mfob::StateGetS(x));
    next_pt.set_t((res.empty() ? time_base : res.back().t()) +
                  trajectory_time_step);
    res.push_back(std::move(next_pt));
  }
  return res;
}

// Convert an input trajectory with time step kTrajectoryTimeStep to comply
// with trajectory optimizer input format: horizon target_trajectory_steps and
// time step target_trajectory_time_step.
std::vector<TrajectoryPoint> ToTrajectoryOptimizerInput(
    const std::vector<TrajectoryPoint>& input_traj, int target_trajectory_steps,
    double target_trajectory_time_step, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params, const bool in_uturn) {
  CHECK_GT(target_trajectory_time_step, 0.0);
  CHECK_GT(target_trajectory_steps, 0);
  const int sample_step =
      static_cast<int>(target_trajectory_time_step / kTrajectoryTimeStep + 0.5);

  if (input_traj.empty()) return input_traj;
  std::vector<TrajectoryPoint> res;
  res.reserve(target_trajectory_steps);
  for (int i = 0; i < target_trajectory_steps; ++i) {
    if (i * sample_step >= input_traj.size()) break;
    res.push_back(input_traj[i * sample_step]);
  }
  if (res.size() == target_trajectory_steps) return res;

  const double input_traj_end_v = input_traj.back().v();
  const auto traj_end_sl_status =
      drive_passage.QueryFrenetCoordinateAt(input_traj.back().pos());
  if (!traj_end_sl_status.ok() ||
      (traj_end_sl_status->s > path_sl_boundary.end_s())) {
    LOG_ERROR << "Input trajectory end is out of drive passage or trajectory "
                 "end if out of path boundary: "
              << input_traj.back().DebugString();
    // Extend by constant acceleration and curvature.
    constexpr double kMinSpeed = 1e-6;
    constexpr double kMinDist = 1e-6;
    while (res.size() < target_trajectory_steps) {
      auto& pt = res.back();
      const double dist = std::max(
          kMinDist, pt.v() * target_trajectory_time_step +
                        0.5 * pt.a() * Sqr(target_trajectory_time_step));
      PathPoint path_point;
      path_point.set_x(pt.pos().x());
      path_point.set_y(pt.pos().y());
      path_point.set_s(pt.s());
      path_point.set_theta(pt.theta());
      path_point.set_kappa(pt.kappa());
      path_point.set_lambda(pt.lambda());
      const auto next_path_point = GetPathPointAlongCircle(path_point, dist);
      auto next_pt = pt;
      next_pt.set_t(pt.t() + target_trajectory_time_step);
      next_pt.set_pos(ToVec2d(next_path_point));
      next_pt.set_s(next_path_point.s());
      next_pt.set_theta(next_path_point.theta());
      next_pt.set_kappa(next_path_point.kappa());
      next_pt.set_v(
          std::max(kMinSpeed, pt.v() + pt.a() * target_trajectory_time_step));
      next_pt.set_a(
          std::max(pt.a(), -1.0 * next_pt.v() / target_trajectory_time_step));
      pt.set_j(
          std::clamp((next_pt.a() - next_pt.a()) / target_trajectory_time_step,
                     motion_constraint_params.max_decel_jerk(),
                     motion_constraint_params.max_accel_jerk()));
      next_pt.set_psi(next_path_point.lambda() * next_pt.v());
      res.push_back(std::move(next_pt));
    }
    res.back().set_j((res.rbegin() + 1)->j());
    return res;
  }

  const double time_to_extend =
      target_trajectory_time_step * (target_trajectory_steps - res.size() + 1);

  double kMinEndSpeed = 3.6;  // m/s.
  if (in_uturn) {
    kMinEndSpeed = 1.0;
  }
  const double target_v = std::min(
      std::max(kMinEndSpeed, input_traj_end_v),
      (path_sl_boundary.end_s() - traj_end_sl_status->s) / time_to_extend);

  const auto extend_state = GetExtendStateByPurePursuit(
      target_trajectory_time_step, res.back(),
      target_trajectory_steps - res.size(), target_v, drive_passage,
      path_sl_boundary, veh_geo_params, motion_constraint_params);

  for (int k = 0; k < extend_state.size(); ++k) {
    res.push_back(extend_state[k]);
  }
  return res;
}

// Try to generate a solver init trajectory using
// last optimization result, according to the current
// problem to be solved.
// If such generation failed, std::nullopt will be returned.
std::optional<std::vector<TrajectoryPoint>> BuildUsablePrevInitTrajectory(
    int plan_id, int trajectory_steps, double trajectory_time_step,
    const Mfob& problem, const DdpOptimizerParamsProto& params,
    absl::Time plan_start_time, absl::Time last_plan_start_time,
    const std::vector<TrajectoryPoint>& initializer_traj,
    const std::vector<TrajectoryPoint>& last_optimized_traj,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  // Check input.
  if (initializer_traj.empty()) {
    return std::nullopt;
  }
  if (last_optimized_traj.size() < trajectory_steps) {
    return std::nullopt;
  }

  constexpr double kMaxShiftTime = 1.0;
  const absl::Duration dt = plan_start_time - last_plan_start_time;
  const double shift_time = absl::ToDoubleSeconds(dt);

  if (shift_time < 0 || shift_time > kMaxShiftTime) {
    // Long has passed since last optimization.
    return std::nullopt;
  }

  // Shift last trajectory by time.
  std::vector<TrajectoryPoint> shifted_last_optimized_traj =
      last_optimized_traj;
  ShiftTrajectoryByTime(shift_time, &shifted_last_optimized_traj);

  // Adapt last trajectory to plan start point.
  constexpr double kMaxAdaptionCost = 150.0;
  std::optional<std::vector<TrajectoryPoint>> prev_init_traj =
      optimizer::AdaptTrajectoryToGivenPlanStartPoint(
          plan_id, trajectory_steps, problem, params, kMaxAdaptionCost,
          initializer_traj.front(), shifted_last_optimized_traj);

  // Prev init is not usable when decision different.
  if (prev_init_traj.has_value()) {
    for (const auto& obj_traj : st_planner_object_traj.trajectories) {
      if (!optimizer::HasSameDecisionOverSpacetimeObject(
              initializer_traj, *prev_init_traj,
              optimizer::SampleObjectStates(
                  trajectory_steps, trajectory_time_step, obj_traj.states()))) {
        return std::nullopt;
      }
    }
  }

  return prev_init_traj;
}

void DumpTrajToDebugFrame(const std::string& traj_name,
                          const std::vector<TrajectoryPoint>& traj,
                          const int plan_id, const Log2DDS::Color& color) {
  const std::vector<std::string> postfixs = {
      "_t",         "_s",        "_v",        "_a",          "_j",
      "_lon-jerk",  "_theta",    "_kappa",    "_psi",        "_chi",
      "_lat-accel", "_lat-jerk", "_yaw-rate", "_steer-angle"};
  std::vector<std::vector<double>> values(postfixs.size());
  std::vector<Vec2d> pos;
  pos.reserve(traj.size());
  for (auto& v : values) {
    v.reserve(traj.size());
  }
  const double first_s = traj.front().s();
  for (size_t i = 0; i < traj.size(); ++i) {
    const TrajectoryPoint& traj_point = traj[i];
    int y_index = 0;
    values[y_index++].push_back(traj_point.t());
    values[y_index++].push_back(traj_point.s() - first_s);
    values[y_index++].push_back(traj_point.v());
    values[y_index++].push_back(traj_point.a());
    values[y_index++].push_back(traj_point.j());
    values[y_index++].push_back(ComputeLongitudinalJerk(traj_point));
    values[y_index++].push_back(traj_point.theta());
    values[y_index++].push_back(traj_point.kappa());
    values[y_index++].push_back(traj_point.psi());
    values[y_index++].push_back(traj_point.chi());
    values[y_index++].push_back(ComputeLateralAcceleration(traj_point));
    values[y_index++].push_back(ComputeLateralJerk(traj_point));
    values[y_index++].push_back(traj_point.v() * traj_point.kappa());
    values[y_index++].push_back(traj_point.steer_angle());
    pos.push_back(traj_point.pos());
  }

  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  for (size_t i = 0; i < values.size(); ++i) {
    Log2DDS::LogDataV2(prefix + traj_name + postfixs[i], std::move(values[i]));
  }
  Log2DDS::LogLineV2(prefix + traj_name, color, {}, pos);
}

void AddSteerAngleToPoints(double wheel_base, double steer_ratio,
                           std::vector<TrajectoryPoint>* result_points) {
  if (result_points == nullptr) {
    return;
  }
  for (TrajectoryPoint& point : *result_points) {
    point.set_steer_angle(fast_math::Atan2(point.kappa() * wheel_base, 1.0) *
                          steer_ratio);
  }
}

optimizer::TurnType GetEgoTurnType(
    const DrivePassage& drive_passage,
    const PlannerSemanticMapManager& planner_semantic_map_mgr) {
  const auto lane_id = drive_passage.lane_path().front().lane_id();
  const auto lane_info =
      planner_semantic_map_mgr.FindCurveLaneByIdOrNull(lane_id);
  optimizer::TurnType ego_turn_type = optimizer::TurnType::kStraight;
  if (lane_info != nullptr) {
    switch (lane_info->turn_type()) {
      case ad_byd::planning::NO_TURN:
        ego_turn_type = optimizer::TurnType::kStraight;
        break;
      case ad_byd::planning::LEFT_TURN:
        ego_turn_type = optimizer::TurnType::kLeftTurn;
        break;
      case ad_byd::planning::RIGHT_TURN:
        ego_turn_type = optimizer::TurnType::kRightTurn;
        break;
      case ad_byd::planning::U_TURN:
        ego_turn_type = optimizer::TurnType::kUTurn;
        break;
      default:
        ego_turn_type = optimizer::TurnType::kStraight;
    }
  }
  return ego_turn_type;
}

}  // namespace

// NOLINTNEXTLINE
absl::StatusOr<TrajectoryOptimizerOutput> OptimizeTrajectory(
    const TrajectoryOptimizerInput& input,
    TrajectoryOptimizerDebugProto* optimizer_debug, bool is_compare_weight,
    ThreadPool* thread_pool) {
  // ("EstPlanner/OptimizeTrajectory");
  std::string name =
      Log2DDS::TaskPrefix(input.plan_id) + std::string(__FUNCTION__);
  SCOPED_TRACE(name.c_str());
  TIMELINE("OptimizeTrajectory");
  const absl::Cleanup timeout_trigger = [start_time = Timer()]() {
    constexpr double kTimeoutLimitMs = 50.0;
    const double runtime = start_time.TimeNs() / 1e6;
    if (runtime > kTimeoutLimitMs) {
    }
  };
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.drive_passage);
  CHECK_NOTNULL(input.path_sl_boundary);
  CHECK_NOTNULL(input.constraint_mgr);
  CHECK_NOTNULL(input.leading_trajs);
  CHECK_NOTNULL(input.planner_semantic_map_mgr);
  CHECK_NOTNULL(input.st_planner_object_traj);
  // CHECK_NOTNULL(input.captain_trajectory);
  CHECK_NOTNULL(input.behavior);

  const auto& trajectory_optimizer_params =
      *CHECK_NOTNULL(input.trajectory_optimizer_params);
  const auto& motion_constraint_params =
      *CHECK_NOTNULL(input.motion_constraint_params);
  const auto& planner_functions_params =
      *CHECK_NOTNULL(input.planner_functions_params);
  const auto& vehicle_models_params =
      *CHECK_NOTNULL(input.vehicle_models_params);
  const auto& veh_geo_params = *CHECK_NOTNULL(input.veh_geo_params);
  const auto& veh_drive_params = *CHECK_NOTNULL(input.veh_drive_params);

  if (const auto input_status = CheckInputQuality(input); !input_status.ok()) {
    return input_status;
  }
  const auto& plan_start_point = input.plan_start_point;
  const PiecewiseLinearFunction<double> speed_trajectory_steps_scale_plf =
      PiecewiseLinearFunctionFromProto(
          trajectory_optimizer_params.speed_trajectory_steps_scale_plf());
  const int trajectory_steps =
      static_cast<int>(trajectory_optimizer_params.trajectory_steps() *
                       speed_trajectory_steps_scale_plf(plan_start_point.v()));
  const double trajectory_time_step =
      trajectory_optimizer_params.trajectory_time_step();
  const double avoid_dynamic_obj_early_time =
      trajectory_optimizer_params.avoid_dynamic_obj_early_time();
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  // The optimization length shall not be even longer than planning horizon.
  CHECK_LT(trajectory_time_step * trajectory_steps, kPlanningTimeHorizon);

  const auto& function_id = input.behavior->function_id();
  optimizer_debug->set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(input.plan_start_time));

  const bool in_uturn = IsInUturn(
      *input.drive_passage, input.plan_id, plan_start_point.path_point(),
      trajectory_optimizer_params.cost_weight_params());

  // Convert from ApolloTrajectoryPoint to TrajectoryPoint.
  // TODO: Replace with ToTrajectoryPoint once initializer can produce
  // third-order Apollo trajectory.
  std::vector<TrajectoryPoint> initializer_traj = ToTrajectoryOptimizerInput(
      ToTrajectoryPointFromSecondOrderApollo(input.trajectory),
      trajectory_steps, trajectory_time_step, *input.drive_passage,
      *input.path_sl_boundary, motion_constraint_params, veh_geo_params,
      in_uturn);

  std::vector<TrajectoryPoint> captain_traj = ToTrajectoryOptimizerInput(
      ToTrajectoryPointFromSecondOrderApollo(input.captain_trajectory),
      trajectory_steps, trajectory_time_step, *input.drive_passage,
      *input.path_sl_boundary, motion_constraint_params, veh_geo_params,
      in_uturn);

  // The size of prev_traj should be either 0 or equal to initializer_traj.
  // Note: make sure that prev_traj has been shifted in time to now.
  const std::vector<TrajectoryPoint> prev_traj = ToTrajectoryOptimizerInput(
      ToTrajectoryPoint(input.previous_trajectory), trajectory_steps,
      trajectory_time_step, *input.drive_passage, *input.path_sl_boundary,
      motion_constraint_params, veh_geo_params, in_uturn);
  CHECK(prev_traj.empty() || prev_traj.size() == initializer_traj.size());

  // ScopedMultiTimer timer("trajectory_optimizer");

  const std::string base_name_with_plan_id =
      absl::StrFormat("traj_opt_%d", input.plan_id);
  const std::string base_name = "traj_opt";

  constexpr double kStartSquareDistThreshold = 1.0;
  constexpr double kStartVDiffThreshold = 2.0;
  const auto& initializer_traj_start_point = initializer_traj.front();
  const double start_square_dist = (Vec2d(plan_start_point.path_point().x(),
                                          plan_start_point.path_point().y()) -
                                    initializer_traj_start_point.pos())
                                       .squaredNorm();
  const double start_v_diff =
      plan_start_point.v() - initializer_traj_start_point.v();
  if (start_square_dist < kStartSquareDistThreshold &&
      std::abs(start_v_diff) < kStartVDiffThreshold) {
    initializer_traj.begin()->FromProto(plan_start_point);
  } else {
    LOG_FATAL << "Initializer first state("
              << initializer_traj_start_point.pos().x() << ","
              << initializer_traj_start_point.pos().y() << ","
              << initializer_traj_start_point.v()
              << ") too far from plan start point("
              << plan_start_point.path_point().x() << ","
              << plan_start_point.path_point().y() << ","
              << plan_start_point.v() << ")!";
  }

  const auto& drive_passage = *input.drive_passage;
  const auto& path_sl_boundary = *input.path_sl_boundary;
  const auto& constraint_manager = *input.constraint_mgr;
  const auto& leading_trajs = *input.leading_trajs;
  const auto& st_traj_mgr = *input.st_traj_mgr;
  const double v_now = input.plan_start_point.v();
  const auto& st_planner_object_traj = *input.st_planner_object_traj;

  // 识别窄车道场景
  double lane_width = 0.0;
  const bool is_narrow_scene =
      NarrowScene(drive_passage, input.plan_start_point, &lane_width);

  // Build stations_query_helper and av_model_helpers.
  std::unique_ptr<CenterLineQueryHelper<Mfob>> stations_query_helper;
  absl::Status add_helper_status =
      AddStationQueryHelper(trajectory_steps, base_name_with_plan_id,
                            drive_passage, &stations_query_helper);
  CHECK_EQ(add_helper_status.ok(), true);

  std::unique_ptr<AvModelHelper<Mfob>> av_model_helpers = AddAvModelHelpers(
      trajectory_steps, base_name_with_plan_id, veh_geo_params,
      vehicle_models_params.trajectory_optimizer_vehicle_model_params());

  // Add problem costs.
  Timer add_cost_start_time;
  std::vector<std::unique_ptr<Cost<Mfob>>> costs;

  const optimizer::TurnType ego_turn_type =
      GetEgoTurnType(*input.drive_passage, *input.planner_semantic_map_mgr);
  auto lc_weight_debug_string =
      absl::StrCat("lateral_acceleration_cost_weight:",
                   trajectory_optimizer_params.cost_weight_params()
                       .lateral_acceleration_cost_weight());
  Log2DDS::LogDataV0(Log2DDS::TaskPrefix(input.plan_id) + "lc_weight",
                     lc_weight_debug_string);
  absl::Status add_cost_status = AddCosts(
      input.plan_id, trajectory_steps, trajectory_time_step,
      avoid_dynamic_obj_early_time, base_name_with_plan_id, function_id,
      input.plan_start_point, input.lc_stage, input.push_dir, input.borrow_lane,
      input.nudge_info, initializer_traj, prev_traj, captain_traj,
      drive_passage, path_sl_boundary, constraint_manager, leading_trajs,
      st_traj_mgr, st_planner_object_traj, *input.planner_semantic_map_mgr,
      v_now, trajectory_optimizer_params.cost_weight_params(),
      trajectory_optimizer_params.cost_config(),
      trajectory_optimizer_params.trajectory_optimizer_validation_params(),
      veh_geo_params, veh_drive_params, motion_constraint_params,
      vehicle_models_params.trajectory_optimizer_vehicle_model_params(),
      planner_functions_params, stations_query_helper, av_model_helpers,
      is_narrow_scene, &costs, optimizer_debug, thread_pool, ego_turn_type,
      in_uturn);
  CHECK_EQ(add_cost_status.ok(), true);

  const double add_cost_time = add_cost_start_time.TimeNs() / 1e6;

  // Build problem.
  auto problem = std::make_unique<Mfob>(
      &motion_constraint_params, &veh_geo_params, &veh_drive_params,
      trajectory_steps, trajectory_time_step,
      /*enable_dynamic_2nd_derivatives=*/false,
      /*enable_post_process=*/false);
  problem->AddCostHelper(std::move(stations_query_helper));
  problem->AddCostHelper(std::move(av_model_helpers));
  for (auto& cost : costs) {
    problem->AddCost(std::move(cost));
  }

  // Build solver init trajectory.
  std::string solver_init_traj_name = "none";
  DdpOptimizerDebugProto::SolverInitialTrajectorySource solver_init_traj_source;
  std::optional<std::vector<TrajectoryPoint>> solver_init_traj;
  // First try prev_init trajectory.
  if (FLAGS_planner_traj_opt_init_traj_uses_last_optimized_trajectory) {
    if (input.trajectory_optimizer_state.has_value() &&
        input.trajectory_optimizer_state->last_optimized_trajectory.size() >
            0) {
      TIMELINE("BuildUsablePrevInitTrajectory");
      solver_init_traj = BuildUsablePrevInitTrajectory(
          input.plan_id, trajectory_steps, trajectory_time_step, *problem,
          trajectory_optimizer_params.optimizer_params(), input.plan_start_time,
          input.trajectory_optimizer_state->last_plan_start_time,
          initializer_traj,
          input.trajectory_optimizer_state->last_optimized_trajectory,
          st_planner_object_traj);
      if (solver_init_traj.has_value()) {
        solver_init_traj_source = DdpOptimizerDebugProto::PREV_OPTIMIZATION;
        solver_init_traj_name = "prev-init";
      }
    }
  }

  // By default use smoothed initializer trajectory.
  if (!solver_init_traj.has_value()) {
    TIMELINE("SmoothTrajectoryByMixedFourthOrderDdp");
    solver_init_traj = SmoothTrajectoryByMixedFourthOrderDdp(
        input.plan_id, trajectory_steps, trajectory_time_step, initializer_traj,
        drive_passage, base_name_with_plan_id,
        trajectory_optimizer_params.smoother_params(), motion_constraint_params,
        veh_geo_params, veh_drive_params);
    solver_init_traj_source = DdpOptimizerDebugProto::SMOOTHED_INITIALIZER;
    solver_init_traj_name = "smooth-init";
  }

  Log2DDS::LogDataV2(
      "use_traj", Log2DDS::TaskPrefix(input.plan_id) + solver_init_traj_name);
  CHECK(solver_init_traj.has_value());

  // Add solver helper costs to problem.
  std::vector<std::unique_ptr<Cost<Mfob>>> solver_helper_costs;
  for (auto& cost : solver_helper_costs) {
    problem->AddCost(std::move(cost));
  }

  DdpOptimizer<Mfob> solver(input.plan_id, problem.get(), trajectory_steps,
                            /*owner=*/"trajectory_optimizer",
                            /*verbosity=*/FLAGS_traj_opt_verbosity_level,
                            trajectory_optimizer_params.optimizer_params());

  const auto& vehicle_model_params =
      vehicle_models_params.trajectory_optimizer_vehicle_model_params();
  std::vector<IterationAvModelVisualizerMonitor<Mfob>::circle> av_model_circle;
  av_model_circle.reserve(vehicle_model_params.circles_size());
  for (const auto& circle : vehicle_model_params.circles()) {
    av_model_circle.push_back(
        {circle.dist_to_rac(), circle.angle_to_axis(), circle.radius()});
  }
  std::vector<TrajectoryPoint> result_points;

  std::string error_code = "";
  bool optimizer_solve_success = true;

  Timer solve_start_time;
  double solve_time;
  double final_cost = std::numeric_limits<double>::max();
  std::vector<std::string> final_cost_debug_info;
  {
    TIMELINE("solver.Solve");
    auto solver_output = solver.Solve(
        *solver_init_traj, &final_cost_debug_info, &final_cost,
        DdpOptimizer<Mfob>::SolveConfig::Onboard(), is_compare_weight);
    solve_time = solve_start_time.TimeNs() / 1e6;

    if (!solver_output.ok()) {
      error_code = std::string(solver_output.status().message());
      optimizer_solve_success = false;
    } else {
      result_points = std::move(*solver_output);
      AddSteerAngleToPoints(veh_geo_params.wheel_base(),
                            veh_drive_params.steer_ratio(), &result_points);
    }
  }
  // timer.Mark("solve");

  TrajectoryOptimizerOutput output;
  optimizer_debug->mutable_ddp()->mutable_final_costs()->set_cost(final_cost);

  if (!is_compare_weight) {
    DumpTrajToDebugFrame("traj_init", initializer_traj, input.plan_id,
                         Log2DDS::kBlue);
    DumpTrajToDebugFrame("traj_" + solver_init_traj_name, *solver_init_traj,
                         input.plan_id, Log2DDS::kGreen);
    DumpTrajToDebugFrame("traj_res", result_points, input.plan_id,
                         Log2DDS::kMiddleBlueGreen);
  } else {
    DumpTrajToDebugFrame("traj_res_compare", result_points, input.plan_id,
                         Log2DDS::kPink);
  }

  // And this at the end in case some one did optimizer_debug->reset()
  // somewhere.
  optimizer_debug->mutable_ddp()->mutable_run_time_profile()->set_add_cost_time(
      add_cost_time);
  optimizer_debug->mutable_ddp()->mutable_run_time_profile()->set_solve_time(
      solve_time);

  DestroyContainerAsyncMarkSource(std::move(problem), std::string{});

  if (!optimizer_solve_success) {
    return absl::InternalError(error_code);
  }
  const auto validation_status = optimizer::ValidateTrajectory(
      result_points,
      trajectory_optimizer_params.trajectory_optimizer_validation_params(),
      *optimizer_debug);
  if (!validation_status.ok()) {
    LOG_ERROR << "TrajectoryOptimizer Failed: " << validation_status.message();
    return absl::InternalError(validation_status.message());
  }

  // Fill trajectory optimizer state.
  output.trajectory_optimizer_state.last_optimized_trajectory = result_points;
  output.trajectory_optimizer_state.last_plan_start_time =
      input.plan_start_time;

  // Export nudge object id
  const auto nudge_object_info = optimizer::ExtractNudgeObjectId(
      trajectory_steps, trajectory_time_step, input.lc_stage, input.push_dir,
      drive_passage, path_sl_boundary, result_points, input.previous_trajectory,
      st_planner_object_traj, veh_geo_params, final_cost_debug_info,
      input.nudge_object_info);
  if (nudge_object_info.ok()) {
    output.nudge_object_info = *nudge_object_info;
    if (output.nudge_object_info.has_value()) {
      const std::string& prefix = Log2DDS::TaskPrefix(input.plan_id);
      const std::string nudge_object_info_debug =
          prefix + ": nudge obstacle id = " + output.nudge_object_info->id +
          ", nudge_state: " +
          (output.nudge_object_info->nudge_state ==
                   NudgeObjectInfo::NudgeState::NUDGE
               ? "NUDGE"
               : "BORROW");
      Log2DDS::LogDataV2("nudge_object_info", nudge_object_info_debug);
    }
  }

  std::vector<ApolloTrajectoryPointProto> output_traj =
      ToApolloTrajectoryPointProto(result_points);
  output.trajectory_proto = ToApolloTrajectoryPointProto(result_points);
  output.trajectory = std::move(result_points);

  return output;
}

PlannerStatus RunOptimizeTrajectry(StPathPlannerInput& st_path_input,
                                   StPathPlannerOutput* st_path_out,
                                   TrajectoryOptimizerOutput& opt_output,
                                   ThreadPool* thread_pool) {
  auto mutable_trajectory_optimizer_params =
      *st_path_input.trajectory_optimizer_params;

  // Modify style settings for trajectory optimizer.
  if (FLAGS_planner_enable_lc_style_params) {
    ModifyTrajOptParamsStyle(
        *st_path_input.trajectory_optimizer_lc_radical_params,
        *st_path_input.trajectory_optimizer_lc_normal_params,
        *st_path_input.trajectory_optimizer_lc_conservative_params,
        st_path_out->scheduler_output.lane_change_state.stage(),
        st_path_out->lc_style_decider_result.active_path_response_style(),
        &mutable_trajectory_optimizer_params);
    auto lc_style_debug_string = absl::StrCat(
        "lc_style:",
        st_path_out->lc_style_decider_result.active_path_response_style(),
        ",lc_stage:", st_path_out->scheduler_output.lane_change_state.stage());
    Log2DDS::LogDataV0(
        absl::StrCat("_task_", st_path_input.plan_id, "_lc_style"),
        lc_style_debug_string);
  }

  // Setting autotuned trajectory optimizer params.
  // BANDAID: Refactor to solve code divergence in the future.
  if (FLAGS_planner_auto_tuning_mode) {
    UpdateTrajOptParams(FLAGS_planner_traj_opt_params_file_address,
                        &mutable_trajectory_optimizer_params);
    // For readability.
  } else if (FLAGS_planner_update_learned_alphas) {
    if (FLAGS_planner_update_learned_alphas_except_lane_change) {
      if (st_path_out->scheduler_output.lane_change_state.stage() ==
          LaneChangeStage::LCS_NONE) {
        UpdateTrajOptParams(FLAGS_planner_traj_opt_params_file_address,
                            &mutable_trajectory_optimizer_params);
      }
    } else {
      UpdateTrajOptParams(FLAGS_planner_traj_opt_params_file_address,
                          &mutable_trajectory_optimizer_params);
    }
  }

  VLOG(3) << "Actual cost weights used in trajectory optimizer: ";
  VLOG(3)
      << mutable_trajectory_optimizer_params.cost_weight_params().DebugString();

  // Time alignment.
  const std::vector<ApolloTrajectoryPointProto> previous_trajectory =
      ShiftTrajectoryByTime(
          st_path_input.st_path_start_point_info
                  ->relative_index_from_plan_start_point *
              kTrajectoryTimeStep,
          *st_path_input.time_aligned_prev_traj,
          st_path_input.motion_constraint_params->max_accel_jerk(),
          st_path_input.motion_constraint_params->max_decel_jerk());
  // const std::vector<TrajectoryPoint> captain_trajectory =
  //     ConvertCaptainTrajectoryToOptimizerInput(
  //         mutable_trajectory_optimizer_params.trajectory_steps(),
  //         mutable_trajectory_optimizer_params.trajectory_time_step(),
  //         st_path_input.st_path_start_point_info->relative_index_from_plan_start_point
  //         *
  //             kTrajectoryTimeStep,
  //         /*captain_traj=*/
  //         FLAGS_planner_use_ml_trajectory_as_optimizer_ref_traj
  //             ? st_path_input.captain_net_output->traj_points
  //             : empty_ref_traj);

  // s-l boundary
  // st_path_out->scheduler_output.sl_boundary.SwapOptBoundary();
  // if (st_path_input.scheduler_output.borrow_lane) {
  //   ModifySLBoundaryByDPPath(
  //       st_path_out->scheduler_output.drive_passage,
  //       st_path_input.scheduler_output.av_frenet_box_on_drive_passage,
  //       st_path_out->st_planner_object_traj,
  //       ToTrajectoryPointFromSecondOrderApollo(st_path_out.traj_points),
  //       &st_path_out->scheduler_output.sl_boundary);
  // }
  if (st_path_input.behavior->function_id() ==
      st::Behavior_FunctionId::Behavior_FunctionId_HW_NOA) {
    auto* mutable_cost_weight_params =
        mutable_trajectory_optimizer_params.mutable_cost_weight_params();
    const double kCurvaturePower = 0.74;
    const double kCurvatureNumerator = 0.47;
    const double kCurvatureBias1 = 0.008;
    const double kCurvatureBias2 = 2.1;
    mutable_cost_weight_params->set_curvature_power(kCurvaturePower);
    mutable_cost_weight_params->set_curvature_numerator(kCurvatureNumerator);
    mutable_cost_weight_params->set_curvature_bias1(kCurvatureBias1);
    mutable_cost_weight_params->set_curvature_bias2(kCurvatureBias2);
    // const double kLateralJerkCostWeight = 0.25;
    // const double kIntrinsicLateralSnapWeight = 0.01;
    // mutable_cost_weight_params->set_lateral_jerk_cost_weight(kLateralJerkCostWeight);
    // mutable_cost_weight_params->set_intrinsic_lateral_snap_weight(kIntrinsicLateralSnapWeight);
  }

  // Optional prev optimizer state.
  std::optional<TrajectoryOptimizerState> trajectory_optimizer_state;
  if (st_path_input.trajectory_optimizer_state_proto != nullptr) {
    trajectory_optimizer_state.emplace(
        *st_path_input.trajectory_optimizer_state_proto);
  }
  const auto& psmm = *st_path_input.planner_semantic_map_manager;
  const auto& vehicle_params = *st_path_input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();

  TrajectoryOptimizerInput opt_input{
      .trajectory = st_path_out->traj_points,
      .nudge_info = st_path_out->nudge_info,
      .previous_trajectory = previous_trajectory,
      .trajectory_optimizer_state = std::move(trajectory_optimizer_state),
      .st_traj_mgr = st_path_input.traj_mgr,
      .st_planner_object_traj = &st_path_out->st_planner_object_traj,
      .drive_passage = &st_path_out->scheduler_output.drive_passage,
      .path_sl_boundary = &st_path_out->scheduler_output.sl_boundary,
      .constraint_mgr = &st_path_out->constraint_manager,
      .leading_trajs = &st_path_out->leading_trajs,
      .planner_semantic_map_mgr = &psmm,
      .plan_start_point = st_path_input.st_path_start_point_info->start_point,
      .plan_start_time = st_path_input.st_path_start_point_info->plan_time,
      .plan_id = st_path_input.plan_id,
      .borrow_lane = st_path_input.scheduler_output.borrow_lane,
      .captain_trajectory = st_path_out->captain_traj_points,
      .lc_stage = st_path_out->scheduler_output.lane_change_state.stage(),
      .push_dir = st_path_out->scheduler_output.lane_change_state.push_state(),
      .behavior = st_path_input.behavior,
      .trajectory_optimizer_params = &mutable_trajectory_optimizer_params,
      .motion_constraint_params = st_path_input.motion_constraint_params,
      .planner_functions_params = st_path_input.planner_functions_params,
      .vehicle_models_params = st_path_input.vehicle_models_params,
      .veh_geo_params = &vehicle_geom_params,
      .veh_drive_params = &vehicle_drive_params,
      .nudge_object_info = st_path_input.nudge_object_info};
  const bool is_compare_weight = false;
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      opt_output,
      OptimizeTrajectory(opt_input, &st_path_out->optimizer_debug_proto,
                         is_compare_weight, thread_pool),
      PlannerStatusProto::OPTIMIZER_FAILED);
  // Optimizer state.
  st_path_out->trajectory_optimizer_state_proto =
      opt_output.trajectory_optimizer_state.ToProto();

  // Optimizer Auto Tuning
  // st_path_out->candidate_auto_tuning_traj_proto =
  //     std::move(opt_output.candidate_auto_tuning_traj_proto);
  // st_path_out->expert_auto_tuning_traj_proto =
  //     std::move(opt_output.expert_auto_tuning_traj_proto);
  st_path_out->nudge_object_info = std::move(opt_output.nudge_object_info);

  if (FLAGS_planner_compare_different_weight) {
    UpdateTrajOptParams(FLAGS_planner_traj_opt_params_file_address,
                        &mutable_trajectory_optimizer_params);
    if (FLAGS_planner_enable_lc_style_params) {
      ModifyTrajOptParamsStyle(
          *st_path_input.trajectory_optimizer_lc_radical_params,
          *st_path_input.trajectory_optimizer_lc_normal_params,
          *st_path_input.trajectory_optimizer_lc_conservative_params,
          st_path_out->scheduler_output.lane_change_state.stage(),
          st_path_out->lc_style_decider_result.active_path_response_style(),
          &mutable_trajectory_optimizer_params);
    }
    opt_input.trajectory_optimizer_params =
        &mutable_trajectory_optimizer_params;
    const bool is_compare_weight = true;
    TrajectoryOptimizerDebugProto original_optimizer_debug;
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        auto original_opt_output,
        OptimizeTrajectory(opt_input, &original_optimizer_debug,
                           is_compare_weight, thread_pool),
        PlannerStatusProto::PLANNER_ABNORMAL_EXIT);
    // const std::string base_name =
    //     absl::StrFormat("traj_opt_%d", opt_input.plan_id);
    if (FLAGS_planner_compare_based_on_original_weight) {
      opt_output = std::move(original_opt_output);
      st_path_out->optimizer_debug_proto = std::move(original_optimizer_debug);
    }
  }

  CHECK(!opt_output.trajectory_proto.empty());
  return OkPlannerStatus();
}

}  // namespace planning
}  // namespace st
