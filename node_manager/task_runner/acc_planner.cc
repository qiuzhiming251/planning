#include "node_manager/task_runner/acc_planner.h"

#include "absl/container/flat_hash_map.h"
#include "acc/acc_input.h"
#include "acc/acc_output.h"
#include "acc/acc_path_corridor_map.h"
#include "acc/acc_path_corridor_vehicle.h"
#include "acc/acc_speed_finder.h"
#include "acc/acc_speed_finder_input.h"
#include "acc/acc_speed_finder_output.h"
#include "acc/acc_target_selector.h"
#include "acc/acc_util.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "object_manager/st_inference/est_planner_output.h"
#include "plan_common/planner_status.h"
#include "plan_common/timer.h"
#include "planner/planner_manager/planner_defs.h"
#include "planner/planner_manager/trajectory_validation.h"
#include "node_manager/task_runner/multi_tasks_cruise_planner_input.h"
#include "node_manager/task_runner/planner_state.h"
#include "plan_common/util/time_util.h"

#include "modules/cnoa_pnc/planning/proto/acc.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"

namespace st::planning {
namespace {

constexpr double kHighSpeedSamplePathMeters = 2.0;  // m.

PlannerStatus RunAccSpeedFinder(
    const ApolloTrajectoryPointProto& plan_start_point, double user_speed_limit,
    bool is_standwait, const AccPathCorridor& path_corridor,
    const SpacetimeTrajectoryManager* traj_mgr,
    const VehicleGeometryParamsProto* vehicle_geometry_params,
    const VehicleDriveParamsProto* vehicle_drive_params,
    const MotionConstraintParamsProto* motion_constraint_params,
    const SpeedFinderParamsProto* speed_finder_params,
    TrajectoryValidationResultProto* trajectory_validation_result,
    AccSpeedFinderOutput* output) {
  CHECK_NOTNULL(traj_mgr);
  CHECK_NOTNULL(vehicle_geometry_params);
  CHECK_NOTNULL(vehicle_drive_params);
  CHECK_NOTNULL(motion_constraint_params);
  CHECK_NOTNULL(speed_finder_params);
  CHECK_NOTNULL(trajectory_validation_result);
  CHECK_NOTNULL(output);

  const auto considered_traj_id = SelectAccTarget(
      path_corridor, *traj_mgr, *vehicle_geometry_params, plan_start_point);
  auto acc_traj_mgr =
      BuildSpacetimeTrajectoryManager(considered_traj_id, *traj_mgr);

  AccSpeedFinderInput acc_speed_finder_input{
      .base_name = "acc_speed",
      .plan_start_v = plan_start_point.v(),
      .plan_start_a = plan_start_point.a(),
      .user_speed_limit = user_speed_limit,
      .is_standwait = is_standwait,
      .loaded_map_dist = path_corridor.loaded_map_dist,
      .traj_mgr = acc_traj_mgr.get(),
      .path = &path_corridor.path,
      .vehicle_geometry_params = vehicle_geometry_params,
      .vehicle_drive_params = vehicle_drive_params,
      .motion_constraint_params = motion_constraint_params,
      .speed_finder_params = speed_finder_params,
  };

  auto acc_speed_output_or = FindAccSpeed(acc_speed_finder_input);
  if (!acc_speed_output_or.ok()) {
    return PlannerStatus(
        PlannerStatusProto::ACC_SPEED_FAILED,
        absl::StrFormat("Acc find speed runner for corridor %s fails: %s.",
                        AccPathCorridorType_Name(path_corridor.type),
                        acc_speed_output_or.status().message()));
  }
  std::vector<ApolloTrajectoryPointProto> past_points;  // not use.
  if (!ValidateAccTrajectory(*vehicle_geometry_params, *vehicle_drive_params,
                             *motion_constraint_params,
                             acc_speed_output_or->trajectory_points,
                             past_points, trajectory_validation_result)) {
    return PlannerStatus(
        PlannerStatusProto::ACC_TRAJECTORY_VALIDATION_FAILED,
        absl::StrFormat("Acc corridor %s trajectory validation fails.",
                        AccPathCorridorType_Name(path_corridor.type)));
  }
  *output = std::move(acc_speed_output_or).value();
  return PlannerStatus(
      PlannerStatusProto::OK,
      absl::StrFormat("Acc plan ok for corridor %s.",
                      AccPathCorridorType_Name(path_corridor.type)));
}

std::vector<PathPoint> ToPathPoints(const DiscretizedPath& smoothed_path,
                                    bool resample = false) {
  // The path should start at 0.
  return std::vector<PathPoint>(smoothed_path.begin(), smoothed_path.end());
}

void FillPlannerOutput(AccOutput acc_output,
                       PathBoundedEstPlannerOutput* output) {
  CHECK_NOTNULL(output);
  output->acc_state = AccState::ACC_OFF;
  if (acc_output.acc_path_corridors.empty()) {
    return;
  }

  output->acc_state = AccState::ACC_ON;
  output->est_status_list.push_back(
      std::move(acc_output.acc_path_corridors[0].build_status));
  auto path_points = ToPathPoints(acc_output.acc_path_corridors[0].path);
  // distance_to_traffic_light_stop_line
  // tl_stop_interface
  // speed_state

  EstPlannerOutput est_planner_output = {
      .path = std::move(acc_output.acc_path_corridors[0].path),
      .traj_points = std::move(acc_output.traj_points),
      .st_path_points = std::move(path_points),
  };
  output->est_planner_output_list.push_back(std::move(est_planner_output));

  EstPlannerDebug est_planner_debug = {
      .traj_validation_result = std::move(acc_output.traj_validation_result),
  };
  output->est_planner_debug_list.push_back(std::move(est_planner_debug));
  output->acc_sl_boundary =
      std::move(acc_output.acc_path_corridors[0].boundary);
  output->acc_path_corridor_type = acc_output.acc_path_corridors[0].type;
}

double CalcReliableAvCurvature(const AvHistory* av_context,
                               const PoseProto& pose, double front_wheel_angle,
                               double wheel_base) {
  const double steer_kappa =
      CalcVehicleCurvature(front_wheel_angle, wheel_base);
  if (av_context == nullptr) {
    return steer_kappa;
  }
  auto latest_kappa = av_context->GetKappa().value_or(steer_kappa);
  auto avg_curvature =
      av_context->GetAvKappaCacheAverage().value_or(steer_kappa);
  constexpr double kKappaCredibleMinSpeed = Kph2Mps(15.0);
  double av_kappa = latest_kappa;
  double kappa_error = std::fabs(av_kappa - steer_kappa) /
                       (std::fabs(steer_kappa) + kKappaEpsilon);
  constexpr double kMaxSteerKappaError = 0.5;
  if (GetLonSpeed(pose) < kKappaCredibleMinSpeed ||
      kappa_error >= kMaxSteerKappaError) {
    return steer_kappa;
  }
  // Add dead zone to avoid swing left and right.
  const double kappa_deadzone = MaybeKappaDeadZone(GetLonSpeed(pose));
  av_kappa = std::fabs(av_kappa) < kappa_deadzone &&
                     std::fabs(avg_curvature) < kappa_deadzone
                 ? 0.0
                 : av_kappa;
  // Maybe clamp with the av speed plf.
  av_kappa = std::clamp(av_kappa, -kMaxAllowedAccKappa, kMaxAllowedAccKappa);
  return av_kappa;
}

void UpdateHeadwayParamWithSpeed(double speed, double dynamic_headway,
                                 SpeedFinderParamsProto* speed_finder_params) {
  PiecewiseLinearFunction<double> headway_speed_buffer_plf(
      PiecewiseLinearFunctionFromProto(
          speed_finder_params->headway_speed_buffer_plf()));
  speed_finder_params->set_follow_time_headway(
      dynamic_headway + headway_speed_buffer_plf(Mps2Kph(speed)));
  speed_finder_params->set_large_vehicle_follow_time_headway(
      dynamic_headway + headway_speed_buffer_plf(Mps2Kph(speed)) + 0.2);
}

void FillAccOutput(AccPathCorridor acc_path_corridor,
                   std::vector<ApolloTrajectoryPointProto> trajectory_points,
                   TrajectoryValidationResultProto trajectory_validation_result,
                   AccOutput* acc_output) {
  CHECK_NOTNULL(acc_output);
  acc_output->acc_path_corridors.push_back(std::move(acc_path_corridor));
  acc_output->traj_points = std::move(trajectory_points);
  acc_output->traj_validation_result = std::move(trajectory_validation_result);
}

}  // namespace

AccInput CreateAccInput(const MultiTasksCruisePlannerInput& input,
                        const PlannerState& planner_state) {
  CHECK_NOTNULL(input.planner_params);
  CHECK_NOTNULL(input.vehicle_params);
  return AccInput{
      .vehicle_geometry_params =
          &input.vehicle_params->vehicle_geometry_params(),
      .vehicle_drive_params = &input.vehicle_params->vehicle_drive_params(),
      .acc_params = &input.planner_params->acc_params(),
      .psmm = planner_state.planner_semantic_map_manager.get(),
      .smooth_result_map = &planner_state.smooth_result_map,
      .pose = &input.pose_proto,
      .st_traj_mgr = input.st_traj_mgr.has_value()
                         ? &(input.st_traj_mgr.value())
                         : nullptr,
      .start_point_info = input.start_point_info.has_value()
                              ? &input.start_point_info.value()
                              : nullptr,
      .prev_trajectory = &planner_state.previous_trajectory,
      .time_aligned_prev_traj = input.time_aligned_prev_traj.has_value()
                                    ? &input.time_aligned_prev_traj.value()
                                    : nullptr,
      .cruising_speed_limit = input.cruising_speed_limit,
      .prev_lane_change_type = LaneChangeType::TYPE_NO_CHANGE,
      .front_wheel_angle = input.front_wheel_angle,
      .av_context = input.av_context,
      .acc_standwait = input.acc_autonomy_state_input.standstill_wait,
      .curve_speed_limit_allowed =
          input.acc_autonomy_state_input.curve_speed_limit_allowed,
      .acceleration_request =
          input.acc_autonomy_state_input.acceleration_request,
  };
}

PlannerStatus RunAccTask(const AccInput& input,
                         AccOutput* acc_output) {  // NOLINT

  // 0. preprocess, load last task status
  CHECK_NOTNULL(input.start_point_info);
  CHECK_NOTNULL(input.vehicle_geometry_params);
  CHECK_NOTNULL(input.vehicle_drive_params);
  CHECK_NOTNULL(input.pose);
  CHECK_NOTNULL(acc_output);

  const auto& vehicle_geometry_params = *input.vehicle_geometry_params;
  const auto& vehicle_drive_params = *input.vehicle_drive_params;
  const auto& plan_start_point_info = *input.start_point_info;
  const auto& plan_start_point = plan_start_point_info.start_point;
  const auto& motion_constraint_params =
      input.acc_params->motion_constraint_params();
  const auto* psmm = input.psmm;
  const auto& acc_params = input.acc_params;
  const auto& pose = *input.pose;
  const auto front_wheel_angle = input.front_wheel_angle;

  constexpr auto kLowSpeedForSample = 10.0;  // mps.
  auto corridor_step_s =
      std::fabs(GetLonSpeed(pose)) < kHighSpeedSamplePathMeters
          ? kHighSpeedSamplePathMeters / 2.0
          : kHighSpeedSamplePathMeters;

  double reliable_kappa =
      CalcReliableAvCurvature(input.av_context, pose, front_wheel_angle,
                              vehicle_geometry_params.wheel_base());
  auto total_preview_time = kAccTrajectoryTimeHorizon;
  const auto& cruise_speed_limit = input.cruising_speed_limit;
  const double max_speed_limit =
      Mph2Mps(motion_constraint_params.default_speed_limit());  // mps.
  double user_speed_limit = cruise_speed_limit.has_value()
                                ? std::min(max_speed_limit, *cruise_speed_limit)
                                : max_speed_limit;
  absl::flat_hash_map<AccPathCorridorType, PlannerStatus> corridor_status_map;
  AccPathCorridor acc_path_corridor =
      BuildErrorAccPathCorridorResult("No valid path corridor.");
  AccSpeedFinderOutput acc_speed_output;
  TrajectoryValidationResultProto trajectory_validation_result;
  bool is_valid_traj = false;
  {
    // 1.1 Try map path corridor.
    if (psmm != nullptr && psmm->map_ptr() != nullptr &&
        psmm->map_ptr()->IsValid()) {
      auto map_path_corridor = BuildAccPathCorridorWithMap(
          *psmm,
          input.smooth_result_map == nullptr ? SmoothedReferenceLineResultMap()
                                             : *input.smooth_result_map,
          pose, *input.st_traj_mgr, plan_start_point, vehicle_geometry_params,
          vehicle_drive_params, corridor_step_s, total_preview_time,
          input.cruising_speed_limit);
      if (map_path_corridor.build_status.ok()) {
        acc_speed_output.trajectory_points.clear();
        acc_speed_output.speed_finder_proto.Clear();
        trajectory_validation_result.Clear();
        // 2.1 Try acc speed finder
        auto map_acc_finder_status = RunAccSpeedFinder(
            plan_start_point, user_speed_limit, input.acc_standwait,
            map_path_corridor, input.st_traj_mgr, &vehicle_geometry_params,
            &vehicle_drive_params, &motion_constraint_params,
            &acc_params->speed_finder_params(), &trajectory_validation_result,
            &acc_speed_output);
        if (map_acc_finder_status.ok()) {
          is_valid_traj = true;
          acc_path_corridor = std::move(map_path_corridor);
        }
        corridor_status_map[AccPathCorridorType::MAP] =
            std::move(map_acc_finder_status);
      } else {
        corridor_status_map[AccPathCorridorType::NO_MAP] =
            std::move(map_path_corridor.build_status);
      }
    }
  }

  // No map
  if (!is_valid_traj) {
    auto estimate_path_corridor = BuildAccPathCorridorWithoutMap(
        pose, plan_start_point, vehicle_geometry_params, vehicle_drive_params,
        corridor_step_s, total_preview_time, front_wheel_angle, reliable_kappa);
    if (estimate_path_corridor.build_status.ok()) {
      acc_speed_output.trajectory_points.clear();
      acc_speed_output.speed_finder_proto.Clear();
      trajectory_validation_result.Clear();
      auto no_map_speed_finder_status = RunAccSpeedFinder(
          plan_start_point, user_speed_limit, input.acc_standwait,
          estimate_path_corridor, input.st_traj_mgr, &vehicle_geometry_params,
          &vehicle_drive_params, &motion_constraint_params,
          &acc_params->speed_finder_params(), &trajectory_validation_result,
          &acc_speed_output);
      if (no_map_speed_finder_status.ok()) {
        is_valid_traj = true;
        acc_path_corridor = std::move(estimate_path_corridor);
      }
      corridor_status_map[AccPathCorridorType::NO_MAP] =
          std::move(no_map_speed_finder_status);
    } else {
      corridor_status_map[AccPathCorridorType::NO_MAP] =
          std::move(estimate_path_corridor.build_status);
    }
  }
  if (is_valid_traj) {
    LOGINFO_EVERY(5) << "Acc cooridor success. type: "
                     << AccPathCorridorType_Name(acc_path_corridor.type);
    FillAccOutput(std::move(acc_path_corridor),
                  std::move(acc_speed_output.trajectory_points),
                  trajectory_validation_result, acc_output);
    return OkPlannerStatus();
  } else {
    for (const auto& [type, status] : corridor_status_map) {
      LOG_ERROR << "Acc path error: " << AccPathCorridorType_Name(type) << ", "
                << status.ToString();
    }
  }
  return PlannerStatus(PlannerStatusProto::ACC_CORRIDOR_FAILED,
                       "No available acc ");
}

PlannerStatus RunAccPlanner(const MultiTasksCruisePlannerInput& input,
                            const PlannerState& planner_state,
                            PathBoundedEstPlannerOutput* est_planner_output) {
  Timer timer(__FUNCTION__);

  if (input.behavior.has_value()) {
    PlannerParamsProto* param_proto =
        const_cast<PlannerParamsProto*>(input.planner_params);
    auto* speed_finder_params =
        param_proto->mutable_acc_params()->mutable_speed_finder_params();
    UpdateHeadwayParamWithSpeed(input.start_point_info->start_point.v(),
                                input.behavior->dynamic_headway(),
                                speed_finder_params);
  }

  AccInput acc_input = CreateAccInput(input, planner_state);
  AccOutput acc_output;
  auto planner_status = RunAccTask(acc_input, &acc_output);
  if (planner_status.ok()) {
    FillPlannerOutput(std::move(acc_output), est_planner_output);
  }
  return planner_status;
}

}  // namespace st::planning
