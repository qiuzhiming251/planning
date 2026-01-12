#include "acc/acc_speed_finder.h"

#include <memory>
#include <utility>
#include <vector>

#include "acc/acc_speed_decider.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "plan_common/timer.h"
#include "planner/planner_manager/planner_defs.h"
#include "planner/speed_optimizer/path_speed_combiner.h"
#include "planner/speed_optimizer/speed_finder_util.h"
#include "planner/speed_optimizer/speed_optimizer.h"
#include "planner/speed_optimizer/speed_optimizer_config_dispatcher.h"
#include "planner/speed_optimizer/speed_optimizer_object_manager.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/time_util.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st::planning {

absl::StatusOr<AccSpeedFinderOutput> FindAccSpeed(
    const AccSpeedFinderInput& input) {
  CHECK_NOTNULL(input.traj_mgr);
  CHECK_NOTNULL(input.path);
  CHECK_NOTNULL(input.vehicle_geometry_params);
  CHECK_NOTNULL(input.vehicle_drive_params);
  CHECK_NOTNULL(input.motion_constraint_params);
  CHECK_NOTNULL(input.speed_finder_params);

  Timer timer(__FUNCTION__);

  int plan_id = 0;
  Log2DDS::LogDataV2("plan_task_id", plan_id);
  Log2DDS::LogDataV0("selected_idx", plan_id);

  const auto& vehicle_geometry_params = *input.vehicle_geometry_params;
  const auto& vehicle_drive_params = *input.vehicle_drive_params;
  const auto& motion_constraint_params = *input.motion_constraint_params;
  const auto& speed_finder_params = *input.speed_finder_params;

  AccSpeedFinderOutput output;

  // Speed decision.
  constexpr double kPathApproxTolerance = 0.05;  // m.
  const auto vehicle_rect =
      CreateOffsetRectFromVehicleGeometry(vehicle_geometry_params);
  const auto path_kd_tree = BuildPathKdTree(*input.path);
  const auto path_approx = BuildPathApprox(
      *input.path, vehicle_rect, kPathApproxTolerance, path_kd_tree.get());

  const double max_allowed_speed =
      Mph2Mps(motion_constraint_params.default_speed_limit());  // m/s.

  constexpr int kAccTrajectorySteps = 80;  // 8s.
  AccSpeedDeciderInput acc_speed_decider_input{
      .base_name = input.base_name,
      .trajectory_steps = kAccTrajectorySteps,
      .plan_start_v = input.plan_start_v,
      .plan_start_a = input.plan_start_a,
      .user_speed_limit = input.user_speed_limit,
      .max_allowed_speed = max_allowed_speed,
      .is_standwait = input.is_standwait,
      .loaded_map_dist = input.loaded_map_dist,
      .traj_mgr = input.traj_mgr,
      .path = input.path,
      .path_kd_tree = path_kd_tree.get(),
      .path_approx = &path_approx,
      .vehicle_geometry_params = &vehicle_geometry_params,
      .motion_constraint_params = &motion_constraint_params,
      .vehicle_drive_params = &vehicle_drive_params,
      .speed_finder_params = &speed_finder_params,
  };

  ASSIGN_OR_RETURN(const auto speed_decider_output,
                   RunAccSpeedDecider(acc_speed_decider_input));

  const auto& st_boundaries_with_decision =
      speed_decider_output.st_boundaries_with_decision;
  const SpeedLimitProvider& speed_limit_provider =
      speed_decider_output.speed_limit_provider;
  const SpeedVector& samplpling_dp_speed =
      speed_decider_output.sampling_dp_speed;

  // Speed optimizer.
  const double plan_total_time = kTrajectoryTimeStep * kAccTrajectorySteps;
  const int knot_num = speed_finder_params.speed_optimizer_params().knot_num();
  CHECK_GT(knot_num - 1, 0);
  const double plan_time_interval = plan_total_time / (knot_num - 1);
  // Assume the step length of the path is a fixed value.
  const double path_step_length = input.path->at(1).s() - input.path->at(0).s();
  CHECK_GT(path_step_length, 0.0);

  const auto now_time = absl::Now();
  const double coeffi_match_lc_style = 1.0;
  const SpeedOptimizerObjectManager opt_obj_mgr(
      /*plan_id=*/0, st_boundaries_with_decision, *input.traj_mgr, *input.path,
      input.plan_start_v, plan_total_time, plan_time_interval,
      speed_finder_params, samplpling_dp_speed, /*cutin_history=*/nullptr,
      ToUnixDoubleSeconds(now_time), LaneChangeStage::LCS_NONE,
      coeffi_match_lc_style, /*lc_finished_time=*/nullptr,
      /*need_accel_for_gap=*/false);

  const SpeedFinderParamsProto* new_speed_finder_params_ptr =
      &speed_finder_params;
  std::unique_ptr<SpeedFinderParamsProto> dispatched_speed_finder_params_ptr;
  auto dispatched_speed_optimizer_params = DispatchSpeedOptimizerConfig(
      st_boundaries_with_decision, *input.traj_mgr, path_approx, *path_kd_tree,
      vehicle_rect.radius(), path_step_length,
      static_cast<int>(input.path->size() - 1), input.plan_start_v,
      input.path->front(), vehicle_geometry_params,
      speed_finder_params.speed_optimizer_params(),
      speed_finder_params.speed_optimizer_config_dispatcher_params());
  if (dispatched_speed_optimizer_params.has_value()) {
    dispatched_speed_finder_params_ptr =
        std::make_unique<SpeedFinderParamsProto>(speed_finder_params);
    *dispatched_speed_finder_params_ptr->mutable_speed_optimizer_params() =
        std::move(*dispatched_speed_optimizer_params);
    new_speed_finder_params_ptr = dispatched_speed_finder_params_ptr.get();
  }
  const auto& new_speed_finder_params = *new_speed_finder_params_ptr;
  const auto speed_bound_map = EstimateSpeedBound(
      speed_limit_provider, samplpling_dp_speed, input.plan_start_v,
      max_allowed_speed, knot_num, plan_time_interval);

  const auto min_speed_limit = GenerateMinSpeedLimitWithLaneAndCurvature(
      FindOrDie(speed_bound_map, SpeedLimitTypeProto_Type_LANE),
      FindOrDie(speed_bound_map, SpeedLimitTypeProto_Type_CURVATURE));

  const auto reference_speed = GenerateReferenceSpeed(
      min_speed_limit, /*raw_lane_speed_limit=*/std::nullopt,
      input.plan_start_v,
      new_speed_finder_params.speed_optimizer_params().ref_speed_bias(),
      new_speed_finder_params.speed_optimizer_params()
          .ref_speed_static_limit_bias(),
      motion_constraint_params.max_acceleration(),
      motion_constraint_params.max_deceleration(), plan_total_time,
      plan_time_interval);
  std::vector<AccelBounds> accel_bounds;
  accel_bounds.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    accel_bounds.push_back(
        AccelBounds{.lower_bound = motion_constraint_params.max_deceleration(),
                    .upper_bound = motion_constraint_params.max_acceleration(),
                    .soft_lower_bound = std::nullopt,
                    .soft_upper_bound = std::nullopt});
  }

  SpeedVector optimized_speed;
  SpeedOptimizer speed_optimizer(
      input.base_name, input.plan_start_v, input.plan_start_a,
      &motion_constraint_params, &speed_finder_params, input.path->length(),
      motion_constraint_params.default_speed_limit(), plan_time_interval);
  RETURN_IF_ERROR(speed_optimizer.Optimize(
      opt_obj_mgr, speed_bound_map, reference_speed, accel_bounds,
      /*time_aligned_prev_traj=*/nullptr, &optimized_speed,
      &output.speed_finder_proto));

  CHECK(!optimized_speed.empty()) << "Optimized speed points is empty!";

  if (new_speed_finder_params.enable_full_stop()) {
    PostProcessSpeedByFullStop(new_speed_finder_params, &optimized_speed);
  }

  // Combine path and speed.
  std::vector<ApolloTrajectoryPointProto> output_trajectory_points;
  output_trajectory_points.reserve(optimized_speed.size());
  RETURN_IF_ERROR(CombinePathAndSpeed(*input.path, /*forward=*/true,
                                      optimized_speed,
                                      &output_trajectory_points));

  // Debug and plot.
  // st_graph
  DumpStGraphBoundary(plan_id, st_boundaries_with_decision, *input.traj_mgr,
                      output.speed_finder_proto);
  SpeedVector comfortable_brake_speed;
  SpeedVector max_brake_speed;

  DumpStGraphSpeed(plan_id, input.path->length(), kAccTrajectorySteps,
                   samplpling_dp_speed, optimized_speed,
                   comfortable_brake_speed, max_brake_speed);
  // vt-graph
  DumpVtGraphSpeedLimit(plan_id, output.speed_finder_proto);
  SpeedVector ref_speeds;
  DumpVtGraphSpeed(plan_id, samplpling_dp_speed, optimized_speed, ref_speeds);

  const auto av_box =
      ComputeAvBox(ToVec2d(input.path->front()), input.path->front().theta(),
                   vehicle_geometry_params);
  DumpAccPath(plan_id, *input.path, av_box);

  for (const auto& pt : output_trajectory_points) {
    *output.speed_finder_proto.add_trajectory() = pt;
  }
  output.trajectory_points = std::move(output_trajectory_points);
  return output;
}

}  // namespace st::planning
