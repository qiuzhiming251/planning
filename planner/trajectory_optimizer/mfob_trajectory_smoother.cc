

#include "planner/trajectory_optimizer/mfob_trajectory_smoother.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <complex>  // IWYU pragma: keep
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "Eigen/Householder"  // IWYU pragma: keep
#include "Eigen/Jacobi"       // IWYU pragma: keep
#include "Eigen/LU"           // IWYU pragma: keep
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "plan_common/timer.h"
#include "plan_common/planning_macros.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "planner/trajectory_optimizer/ddp/ddp_optimizer.h"
#include "planner/trajectory_optimizer/problem/curvature_cost.h"
#include "planner/trajectory_optimizer/problem/forward_speed_cost.h"
#include "planner/trajectory_optimizer/problem/intrinsic_jerk_cost.h"
#include "planner/trajectory_optimizer/problem/longitudinal_acceleration_cost.h"
#include "planner/trajectory_optimizer/problem/mfob_curvature_rate_cost.h"
#include "planner/trajectory_optimizer/problem/mfob_curvature_rate_rate_cost.h"
#include "planner/trajectory_optimizer/problem/mixed_fourth_order_bicycle.h"
#include "planner/trajectory_optimizer/problem/reference_control_deviation_cost.h"
#include "planner/trajectory_optimizer/problem/reference_line_deviation_cost.h"
#include "planner/trajectory_optimizer/problem/reference_state_deviation_cost.h"
#include "planner/trajectory_optimizer/problem/static_boundary_cost_lite.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/util/vehicle_geometry_util.h"

DEFINE_int32(mfob_trajectory_smoother_verbosity_level, 0,
             "Mfob trajectory smoother verbosity level");

namespace st {
namespace planning {
namespace {

using Mfob = MixedFourthOrderBicycle;
using StatesType = Mfob::StatesType;
using StateType = Mfob::StateType;
using ControlsType = Mfob::ControlsType;
using ControlType = Mfob::ControlType;

std::vector<TrajectoryPoint> GeneratePurePursuitInitTraj(
    int trajectory_steps, double trajectory_time_step, const Mfob& problem,
    const std::vector<TrajectoryPoint>& init_traj,
    const VehicleGeometryParamsProto& veh_geo_params) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  // Pure pursuit on the initial trajectory within control bounds.
  constexpr double kLongitudinalLookAhead = 0.5;  // s.
  const int longitudinal_lookahead_steps =
      RoundToInt(kLongitudinalLookAhead / trajectory_time_step);

  StatesType init_xs = Mfob::FitState(init_traj);

  // Get longitudinal reference trajectory
  std::vector<StateType> longitudinal_extended_xs(trajectory_steps +
                                                  longitudinal_lookahead_steps);
  for (int k = 0; k < trajectory_steps; ++k) {
    longitudinal_extended_xs[k] = Mfob::GetStateAtStep(init_xs, k);
  }
  StateType x = Mfob::GetStateAtStep(init_xs, trajectory_steps - 1);
  for (int k = 0; k < longitudinal_lookahead_steps; ++k) {
    x = problem.EvaluateF(k, x, ControlType::Zero());
    longitudinal_extended_xs[trajectory_steps + k] = x;
  }

  const auto get_longitudinal_target =
      [&longitudinal_extended_xs,
       longitudinal_lookahead_steps](int index) -> StateType {
    CHECK_LT(index + longitudinal_lookahead_steps,
             longitudinal_extended_xs.size());
    return longitudinal_extended_xs[index + longitudinal_lookahead_steps];
  };

  // Get lateral reference trajectory
  constexpr double kLateralLookAhead = 0.5;  // s.
  std::vector<StateType> lateral_extended_xs(trajectory_steps);
  std::vector<double> lateral_extened_xs_s(trajectory_steps);
  for (int k = 0; k < trajectory_steps; ++k) {
    lateral_extended_xs[k] = Mfob::GetStateAtStep(init_xs, k);
    lateral_extened_xs_s[k] = Mfob::s(init_xs, k);
  }

  const double init_traj_s = Mfob::StateGetS(lateral_extended_xs.back());
  const PiecewiseLinearFunction<StateType, double> lateral_reference_plf(
      lateral_extened_xs_s, lateral_extended_xs);
  const auto get_lateral_target =
      [&lateral_reference_plf, &lateral_extended_xs, &init_traj_s](
          const StateType& state, double lateral_look_ahead_dist) -> StateType {
    const double target_s = lateral_look_ahead_dist + Mfob::StateGetS(state);
    if (target_s >= init_traj_s) {
      StateType target = lateral_extended_xs.back();
      Vec2d target_theta_tangent =
          Vec2d::UnitFromAngle(Mfob::StateGetTheta(target));
      Mfob::StateSetPos(Mfob::StateGetPos(target) +
                            (target_s - init_traj_s) * target_theta_tangent,
                        &target);
      return target;
    } else {
      return lateral_reference_plf(target_s);
    }
  };

  ControlsType us = ControlsType::Zero(trajectory_steps * Mfob::kControlSize);
  StatesType xs = StatesType::Zero(trajectory_steps * Mfob::kStateSize);
  x = Mfob::GetStateAtStep(init_xs, 0);
  for (int k = 0; k < trajectory_steps; ++k) {
    Mfob::SetStateAtStep(x, k, &xs);
    StateType longitudinal_target = get_longitudinal_target(k);
    const double lateral_look_ahead_dist =
        kLateralLookAhead * Mfob::StateGetV(x) + veh_geo_params.wheel_base();
    StateType lateral_target = get_lateral_target(x, lateral_look_ahead_dist);
    ControlType u = problem.PurePursuitController(
        x, longitudinal_target, lateral_target, longitudinal_lookahead_steps,
        lateral_look_ahead_dist);
    x = problem.EvaluateF(k, x, u);
    Mfob::SetControlAtStep(u, k, &us);
  }

  std::vector<TrajectoryPoint> res(trajectory_steps);
  for (int k = 0; k < trajectory_steps; ++k) {
    TrajectoryPoint& point = res[k];
    problem.ExtractTrajectoryPoint(k, Mfob::GetStateAtStep(xs, k),
                                   Mfob::GetControlAtStep(us, k), &point);
  }
  return res;
}

void AddStaticBoundaryCost(
    int trajectory_steps, const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectorySmootherCostWeightParamsProto& smoother_params,
    Mfob* problem) {
  CHECK_NOTNULL(problem);
  CHECK_GT(trajectory_steps, 0);
  const int drive_passage_size = drive_passage.stations().size();
  std::vector<Vec2d> path_points;
  std::vector<double> l_max;
  std::vector<double> l_min;

  path_points.reserve(drive_passage_size);
  l_min.reserve(drive_passage_size);
  l_max.reserve(drive_passage_size);

  for (int i = 0; i < drive_passage_size; ++i) {
    const Station& station = drive_passage.stations()[StationIndex(i)];
    const auto curb_pair_or = station.QueryCurbOffsetAt(/*signed_lat=*/0.0);
    // CHECK_OK(curb_pair_or.status());
    path_points.push_back(station.xy());
    l_min.push_back(curb_pair_or->first);
    l_max.push_back(curb_pair_or->second);
  }

  problem->AddCost(std::make_unique<StaticBoundaryCostLite<Mfob>>(
      trajectory_steps, std::move(path_points), std::move(l_min),
      std::move(l_max),
      veh_geo_params.width() * 0.5 +
          smoother_params.static_boundary_cost_buffer(),
      "StaticBoundaryCostLite", smoother_params.static_boundary_cost_weight()));
}

}  // namespace

std::vector<TrajectoryPoint> SmoothTrajectoryByMixedFourthOrderDdp(
    int plan_id, int trajectory_steps, double trajectory_time_step,
    const std::vector<TrajectoryPoint>& ref_traj,
    const DrivePassage& drive_passage, const std::string& owner,
    const TrajectorySmootherCostWeightParamsProto& smoother_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params) {
  std::string name = Log2DDS::TaskPrefix(plan_id) + std::string(__FUNCTION__);
  SCOPED_TRACE(name.c_str());
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);

  const Mfob::StateType x0 = Mfob::FitInitialState(ref_traj);
  ControlsType ref_us = Mfob::FitControl(ref_traj, x0);
  StatesType ref_xs = Mfob::FitState(ref_traj);

  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  std::vector<double> ref_us_chi, ref_us_j;
  ref_us_chi.reserve(ref_us.size());
  ref_us_j.reserve(ref_us.size());

  for (int i = 0; i < trajectory_steps; i++) {
    ref_us_chi.emplace_back(Mfob::chi(ref_us, i));
    ref_us_j.emplace_back(Mfob::j(ref_us, i));
  }
  Log2DDS::LogDataV2(prefix + "_ref_us_chi", std::move(ref_us_chi));
  Log2DDS::LogDataV2(prefix + "_ref_us_j", std::move(ref_us_j));

  Mfob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
               trajectory_steps, trajectory_time_step,
               /*enable_dynamic_2nd_derivatives=*/false,
               /*enable_post_process=*/false);

  std::vector<double> state_deviation_weights(
      trajectory_steps * Mfob::kStateSize,
      smoother_params.state_deviation_gain());
  std::vector<double> control_penalty_weights(
      trajectory_steps * Mfob::kControlSize,
      smoother_params.jerk_penalty_gain());
  // constexpr double kMaxChiPenaltyGain = 10.0;
  for (int i = 0; i < trajectory_steps; ++i) {
    state_deviation_weights[i * Mfob::kStateSize + Mfob::kStateXIndex] =
        smoother_params.end_pose_deviation_gain();
    state_deviation_weights[i * Mfob::kStateSize + Mfob::kStateYIndex] =
        smoother_params.end_pose_deviation_gain();
  }

  {
    TIMELINE("AddCost");
    problem.AddCost(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
        std::move(ref_xs), std::move(state_deviation_weights),
        "MfobTrajSmooth::StateRegularization", smoother_params.scale()));
    problem.AddCost(std::make_unique<ReferenceControlDeviationCost<Mfob>>(
        std::move(ref_us), std::move(control_penalty_weights),
        "MfobTrajSmooth::ControlRegularization", smoother_params.scale()));
    constexpr double kCurvatureBufferRatio = 1.0;
    problem.AddCost(std::make_unique<CurvatureCost<Mfob>>(
        ComputeCenterMaxCurvature(veh_geo_params, veh_drive_params) *
            kCurvatureBufferRatio,
        trajectory_steps, trajectory_steps, 1.0,
        "MfobTrajSmooth::MfobCurvatureCost",
        smoother_params.curvature_penalty()));
    constexpr double kCurvatureRateBufferRatio = 0.8;
    problem.AddCost(std::make_unique<MfobCurvatureRateCost<Mfob>>(
        motion_constraint_params.max_psi() * kCurvatureRateBufferRatio,
        "MfobTrajSmooth::MfobCurvatureRateCost",
        smoother_params.curvature_rate_penalty()));
    problem.AddCost(std::make_unique<MfobCurvatureRateRateCost<Mfob>>(
        motion_constraint_params.max_chi(),
        "MfobTrajSmooth::MfobCurvatureRateRateCost",
        smoother_params.curvature_rate_rate_penalty(),
        /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
    problem.AddCost(std::make_unique<ForwardSpeedCost<Mfob>>(
        "MfobTrajSmooth::MfobForwardSpeedCost",
        smoother_params.forward_speed_cost_weight()));
    // AddStaticBoundaryCost(trajectory_steps, drive_passage, veh_geo_params,
    //                       smoother_params, &problem);
  }

  constexpr double kAccelerationBufferRatio = 1.0;
  const std::vector<double> accel_cascade_buffers = {0.0};
  const std::vector<double> accel_cascade_gains = {
      smoother_params.accel_cascade_gains()};
  const std::vector<double> decel_cascade_buffers = {0.0};
  const std::vector<double> decel_cascade_gains = {
      smoother_params.decel_cascade_gains()};

  problem.AddCost(std::make_unique<LongitudinalAccelerationCost<Mfob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio,
      accel_cascade_buffers, accel_cascade_gains, decel_cascade_buffers,
      decel_cascade_gains, "MfobTrajSmooth::MfobLongitudinalAccelerationCost",
      smoother_params.longitudinal_acceleration_cost_weight()));
  constexpr double kJerkBufferRatio = 0.75;
  problem.AddCost(std::make_unique<IntrinsicJerkCost<Mfob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio,
      "MfobTrajSmooth::MfobIntrinsicJerkCost",
      smoother_params.intrinsic_jerk_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));

  DdpOptimizer<Mfob> smoother(
      plan_id, &problem, trajectory_steps, /*owner=*/"mfob_smoother",
      /*verbosity=*/FLAGS_mfob_trajectory_smoother_verbosity_level,
      smoother_params.optimizer_params());
  std::vector<TrajectoryPoint> init_vals =
      GeneratePurePursuitInitTraj(trajectory_steps, trajectory_time_step,
                                  problem, ref_traj, veh_geo_params);

  Timer solve_start_time;
  std::vector<TrajectoryPoint> res;
  {
    TIMELINE("smoother.Solve");
    std::vector<std::string> final_cost_debug_info;
    double final_cost = std::numeric_limits<double>::max();
    auto output = smoother.Solve(init_vals, &final_cost_debug_info, &final_cost,
                                 DdpOptimizer<Mfob>::SolveConfig::Onboard());
    if (!output.ok()) {
      LOG_WARN << "trajectory smoother solve failed: "
               << output.status().message();
      return ref_traj;
    }
    res = std::move(*output);
  }

  if (VLOG_IS_ON(4)) {
    for (int i = 0; i < ref_traj.size(); i++) {
      VLOG(4) << "ref_traj " << i << "  " << ref_traj[i].DebugString();
    }
    for (int i = 0; i < init_vals.size(); i++) {
      VLOG(4) << "pp_traj " << i << "  " << init_vals[i].DebugString();
    }
    for (int i = 0; i < res.size(); i++) {
      VLOG(4) << "res " << i << "  " << res[i].DebugString();
    }
  }
  return res;
}

}  // namespace planning
}  // namespace st
