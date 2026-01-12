

#include "planner/trajectory_optimizer/ddp/trajectory_optimizer_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <functional>
#include <limits>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/math/vec.h"
#include "planner/planner_manager/planner_util.h"
#include "planner/trajectory_optimizer/ddp/ddp_optimizer.h"
#include "planner/trajectory_optimizer/ipopt/ipopt_adapter.h"
#include "planner/trajectory_optimizer/ipopt/ipopt_optimizer_debug_monitor.h"
#include "plan_common/util/file_util.h"
//#include "vis/common/color.h"

DEFINE_string(
    traj_opt_compare_log_file_folder,
    "offboard/planner/optimizer/compare/data/",
    "Only used in comparison mode, it is the folder name of a log file."
    "Make sure the folder exists, see the default value as example.");

namespace st {
namespace planning {
namespace optimizer {

namespace {

void ToIpoptDebugProto(
    const std::vector<TrajectoryPoint>& init_traj,
    const std::vector<TrajectoryPoint>& smooth_init_traj,
    const std::vector<TrajectoryPoint>& result_traj,
    const IpoptOptimizerDebugMonitor<Mfob>& solver_debug_monitor,
    IpoptSolverDebugProto* ipopt_debug_proto) {
  // Write DdpDebugProto data.
  for (int k = 0; k < init_traj.size(); ++k) {
    init_traj[k].ToProto(ipopt_debug_proto->add_init_traj());
  }
  for (int k = 0; k < smooth_init_traj.size(); ++k) {
    smooth_init_traj[k].ToProto(ipopt_debug_proto->add_smooth_init_traj());
  }
  for (int k = 0; k < result_traj.size(); ++k) {
    result_traj[k].ToProto(ipopt_debug_proto->add_final_traj());
  }

  const auto& init_costs = solver_debug_monitor.init_costs();
  ipopt_debug_proto->mutable_init_costs()->set_cost(init_costs.cost);
  for (int i = 0; i < init_costs.costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        ipopt_debug_proto->mutable_init_costs()->add_costs();
    cost_proto->set_name(init_costs.costs[i].first);
    cost_proto->set_cost(init_costs.costs[i].second);
  }
  const auto& final_costs = solver_debug_monitor.final_costs();
  ipopt_debug_proto->mutable_final_costs()->set_cost(final_costs.cost);
  for (int i = 0; i < final_costs.costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        ipopt_debug_proto->mutable_final_costs()->add_costs();
    cost_proto->set_name(final_costs.costs[i].first);
    cost_proto->set_cost(final_costs.costs[i].second);
  }

  for (const auto& iteration : solver_debug_monitor.iterations()) {
    IpoptSolverDebugProto::Iteration* iteration_proto =
        ipopt_debug_proto->add_iterations();
    for (int i = 0; i < iteration.final_xs.size(); ++i) {
      iteration_proto->add_final_xs(iteration.final_xs[i]);
    }
    for (int i = 0; i < iteration.final_us.size(); ++i) {
      iteration_proto->add_final_us(iteration.final_us[i]);
    }
    iteration_proto->set_final_cost(iteration.cost_info.cost);
    for (int i = 0; i < iteration.cost_info.costs.size(); ++i) {
      TrajectoryOptimizerCost* cost_proto = iteration_proto->add_costs();
      cost_proto->set_name(iteration.cost_info.costs[i].first);
      cost_proto->set_cost(iteration.cost_info.costs[i].second);
    }
  }
  ipopt_debug_proto->set_num_iters(solver_debug_monitor.iterations().size());
}

absl::Status ExportToFile(const DdpOptimizerDebugProto& ddp_debug_proto,
                          const IpoptSolverDebugProto& ipopt_debug_proto) {
  TrajectoryOptimizerCompareProto debug_proto;
  *debug_proto.mutable_ddp() = ddp_debug_proto;
  *debug_proto.mutable_ipopt() = ipopt_debug_proto;
  const auto time_current = absl::Now();
  file_util::ProtoToTextFile(
      debug_proto,
      absl::StrFormat("%s%s_%d.pb.txt", FLAGS_traj_opt_compare_log_file_folder,
                      "data", absl::ToUnixMillis(time_current)));
  return absl::OkStatus();
}

}  // namespace

absl::Status ValidateTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points,
    const TrajectoryOptimizerValidationParamsProto&
        trajectory_optimizer_validation_params,
    const TrajectoryOptimizerDebugProto& optimizer_debug) {
  // Fisrt: Check final cost, if cost too large, we think result is abnormal.
  const double final_cost = optimizer_debug.ddp().final_costs().cost();
  if (final_cost > trajectory_optimizer_validation_params.max_final_cost()) {
    return absl::InternalError(
        absl::StrCat("Traj opt final cost too large, current cost is ",
                     final_cost, " max final cost is: ",
                     trajectory_optimizer_validation_params.max_final_cost()));
  }

  // Second: Check lateral acceleration.
  double max_abs_lateral_acc = 0.0;
  const double max_check_time =
      std::min({trajectory_points.crbegin()->t(), kTrajectoryTimeHorizon,
                kMaxLatAccCheckTime});
  for (const auto& pt : trajectory_points) {
    if (pt.t() > max_check_time) {
      break;
    }
    max_abs_lateral_acc =
        std::max(std::abs(ComputeLateralAcceleration(pt)), max_abs_lateral_acc);
  }
  if (max_abs_lateral_acc >
      trajectory_optimizer_validation_params.max_lateral_acc()) {
    return absl::InternalError(absl::StrCat(
        "Traj opt abs lateral acc to large, abs max lateral acc is ",
        max_abs_lateral_acc, " max abs lateral acc is: ",
        trajectory_optimizer_validation_params.max_lateral_acc()));
  }

  // Second: Check lateral jerk.
  double max_abs_lateral_jerk = 0.0;
  for (const auto& pt : trajectory_points) {
    if (pt.t() > max_check_time) {
      break;
    }
    max_abs_lateral_jerk =
        std::max(std::abs(ComputeLateralJerk(pt)), max_abs_lateral_jerk);
  }
  if (max_abs_lateral_jerk >
      trajectory_optimizer_validation_params.max_lateral_jerk()) {
    return absl::InternalError(absl::StrCat(
        "Traj opt abs lateral jerk to large, abs max lateral jerk is ",
        max_abs_lateral_jerk, " max abs lateral jerk is: ",
        trajectory_optimizer_validation_params.max_lateral_jerk()));
  }

  // Fourth: If av brakes sharply, steer changes sharply and heading changes
  // large, we think trajectory is abnormal.
  bool possible_twist = false;
  for (const auto& pt : trajectory_points) {
    if (pt.t() > max_check_time) {
      break;
    }
    if (pt.a() < trajectory_optimizer_validation_params.max_deceleration() &&
        std::abs(pt.psi()) > trajectory_optimizer_validation_params.max_psi()) {
      possible_twist = true;
      break;
    }
  }
  if (possible_twist) {
    double max_theta = -std::numeric_limits<double>::infinity();
    double min_theta = std::numeric_limits<double>::infinity();
    double min_jerk = std::numeric_limits<double>::infinity();
    double abs_max_psi = -std::numeric_limits<double>::infinity();
    double min_acceleration = std::numeric_limits<double>::infinity();
    const double min_jerk_check_time =
        trajectory_optimizer_validation_params.min_jerk_check_time();
    for (const auto& pt : trajectory_points) {
      if (pt.t() > max_check_time) {
        break;
      }
      max_theta = std::max(max_theta, pt.theta());
      min_theta = std::min(min_theta, pt.theta());
      abs_max_psi = std::min(abs_max_psi, std::abs(pt.psi()));
      min_acceleration = std::min(min_acceleration, pt.a());
      if (pt.t() < min_jerk_check_time) {
        min_jerk = std::min(min_jerk, pt.j());
      }
    }
    const double theta_diff = NormalizeAngle(min_theta - max_theta);
    if (std::abs(NormalizeAngle(min_theta - max_theta)) >
            trajectory_optimizer_validation_params.theta_diff() &&
        min_jerk < trajectory_optimizer_validation_params.min_jerk()) {
      return absl::InternalError(
          absl::StrCat("Traj opt result twist: theta_diff is ", theta_diff,
                       ", min_jerk is ", min_jerk));
    }
  }
  return absl::OkStatus();
}

std::optional<std::vector<TrajectoryPoint>>
AdaptTrajectoryToGivenPlanStartPoint(int plan_id, int trajectory_steps,
                                     const Mfob& problem,
                                     const DdpOptimizerParamsProto& params,
                                     double max_adaption_cost,
                                     const TrajectoryPoint& plan_start_point,
                                     std::vector<TrajectoryPoint> trajectory) {
  CHECK_GE(trajectory.size(), trajectory_steps);
  // replace trajectory first point with plan start point.
  trajectory.front() = plan_start_point;
  while (trajectory.size() > trajectory_steps) {
    trajectory.pop_back();
  }
  // Make a solver for the problem.
  DdpOptimizer<Mfob> solver(plan_id, &problem, trajectory_steps,
                            /*owner=*/"trajectory_optimizer_refit",
                            /*verbosity=*/0, params);

  // Solve for one iteration.
  constexpr int kAdaptTrajectorySolveIteration = 1;
  DdpOptimizer<Mfob>::SolveConfig config =
      DdpOptimizer<Mfob>::SolveConfig::Default();
  config.max_iteration = kAdaptTrajectorySolveIteration;
  std::vector<std::string> final_cost_debug_info;
  double final_cost = std::numeric_limits<double>::max();
  absl::StatusOr<std::vector<TrajectoryPoint>> refitted_trajectory =
      solver.Solve(trajectory, &final_cost_debug_info, &final_cost, config);

  // return the optimized trajectory.
  if (refitted_trajectory.ok()) {
    const double refit_cost =
        solver.EvaluateCostForTrajectory(refitted_trajectory.value());
    if (refit_cost <= max_adaption_cost) {
      return std::move(refitted_trajectory.value());
    }
  }

  return std::nullopt;
}

bool HasSameDecisionOverSpacetimeObject(
    const std::vector<TrajectoryPoint>& traj_1,
    const std::vector<TrajectoryPoint>& traj_2,
    const std::vector<prediction::PredictionObjectState>&
        spacetime_object_states) {
  // Check only the prediction exist part.
  // TODO: if the end 90 degree check is too strict. We may only do
  // the end check for leading objects.
  const int n = static_cast<int>(
      std::min({traj_1.size(), traj_2.size(), spacetime_object_states.size()}));
  if (n < 1) {
    return true;
  }

  // Compute angles of each trajectory point in object co-moving coordinate.
  std::vector<double> traj_1_angles;
  std::vector<double> traj_2_angles;
  traj_1_angles.reserve(n);
  traj_2_angles.reserve(n);
  for (int i = 0; i < n; ++i) {
    const Vec2d traj_1_offset =
        traj_1[i].pos() - spacetime_object_states[i].box.center();
    const Vec2d traj_2_offset =
        traj_2[i].pos() - spacetime_object_states[i].box.center();

    constexpr double kEpsilon = 1e-9;
    traj_1_angles.push_back(
        (traj_1_offset.Sqr() < kEpsilon) ? 0.0 : traj_1_offset.FastAngle());
    traj_2_angles.push_back(
        (traj_2_offset.Sqr() < kEpsilon) ? 0.0 : traj_2_offset.FastAngle());
  }

  // Start and end trajectory points shall have close angle.
  const double start_angle_diff =
      NormalizeAngle(traj_1_angles.front() - traj_2_angles.front());
  if (std::abs(start_angle_diff) > M_PI_2) {
    return false;
  }
  const double end_angle_diff =
      NormalizeAngle(traj_1_angles.back() - traj_2_angles.back());
  if (std::abs(end_angle_diff) > M_PI_2) {
    return false;
  }

  // Go through traj_1.front -> traj_1.back -> traj_2.back-> traj_2.front ->
  // traj_1.front. Sum up the accumulated angle.
  double angle_sum = start_angle_diff - end_angle_diff;
  for (int i = 0; i < n - 1; ++i) {
    angle_sum += NormalizeAngle(traj_1_angles[i + 1] - traj_1_angles[i]);
    angle_sum += NormalizeAngle(traj_2_angles[i] - traj_2_angles[i + 1]);
  }

  constexpr double kNoCirclingThreshold = M_PI;
  return std::abs(angle_sum) < kNoCirclingThreshold;
}

}  // namespace optimizer
}  // namespace planning
}  // namespace st
