

#include "node_manager/task_runner/planner_main_loop_internal.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

// IWYU pragma: no_include "Eigen/Core"

#include "absl/algorithm/container.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"

#include "plan_common/log_data.h"
#include "plan_common/constants.h"
#include "plan_common/drive_passage.h"
#include "plan_common/trajectory_util.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/util/path_util.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/maps/local_map_builder.h"
#include "plan_common/maps/route_sections_info.h"
#include "plan_common/util/time_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/autonomy_state_util.h"
#include "plan_common/maps/lane_sequence.h"

#include "router/route_util.h"
#include "router/drive_passage_builder.h"

#include "planner/planner_manager/planner_defs.h"
#include "planner/planner_manager/planner_flags.h"
#include "planner/planner_manager/planner_util.h"

namespace st {
namespace planning {
// namespace {

bool MaybeReset(const ApolloTrajectoryPointProto& pre_reset_planned_point,
                const Vec2d& current_pos, double longitudinal_reset_error,
                double lateral_reset_error, const std::string& planner_name,
                ResetReasonProto::Reason* reset_reason) {
  const Vec2d pos_diff =
      current_pos - Vec2d(pre_reset_planned_point.path_point().x(),
                          pre_reset_planned_point.path_point().y());
  const Vec2d planned_tangent =
      Vec2d::FastUnitFromAngle(pre_reset_planned_point.path_point().theta());

  const double longitudinal_error = std::abs(pos_diff.Dot(planned_tangent));
  if (longitudinal_error > longitudinal_reset_error) {
    LOG_ERROR << "Resetting due to longitudinal error.";
    *reset_reason = ResetReasonProto::LON_ERROR_TOO_LARGE;
    return true;
  }

  const double lateral_error = std::abs(pos_diff.Dot(planned_tangent.Perp()));
  if (lateral_error > lateral_reset_error) {
    LOG_ERROR << "Resetting due to lateral error.";
    *reset_reason = ResetReasonProto::LAT_ERROR_TOO_LARGE;
    return true;
  }

  return false;
}

std::optional<int> InterpolatePointFromPrevTrajectory(
    absl::Time time, const TrajectoryProto& prev_traj) {
  const double t = ToUnixDoubleSeconds(time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      t > prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      t < prev_traj_start_time) {
    return std::nullopt;
  }

  // Previous trajectory should have equal time interval of
  return RoundToInt((t - prev_traj_start_time) / kTrajectoryTimeStep);
}

bool InterpolatePointFromPrevTrajectoryIncludingPast(
    absl::Time time, const TrajectoryProto& prev_traj,
    ApolloTrajectoryPointProto* point) {
  if (prev_traj.trajectory_point().empty()) {
    return false;
  }
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  const double prev_traj_end_time =
      prev_traj_start_time +
      prev_traj.trajectory_point().rbegin()->relative_time();
  const double prev_traj_begin_time =
      prev_traj.past_points().empty()
          ? prev_traj_start_time +
                prev_traj.trajectory_point().begin()->relative_time()
          : prev_traj_start_time +
                prev_traj.past_points().begin()->relative_time();
  const double t = ToUnixDoubleSeconds(time);
  if (t > prev_traj_end_time || t < prev_traj_begin_time) {
    return false;
  }

  // Both previous trajectory and previous past trajectory should have equal
  // time interval of kTrajectoryTimeStep.
  const int relative_time_index =
      RoundToInt((t - prev_traj_start_time) / kTrajectoryTimeStep);
  CHECK_GE(relative_time_index, -prev_traj.past_points_size());
  CHECK_LT(relative_time_index, prev_traj.trajectory_point_size());
  *point = relative_time_index < 0
               ? prev_traj.past_points(relative_time_index +
                                       prev_traj.past_points_size())
               : prev_traj.trajectory_point(relative_time_index);

  return true;
}

ResetReasonProto::Reason GetAccResetReasonByAutonomyState(
    const AutonomyStateProto& prev_autonomy_state,
    const AutonomyStateProto& now_autonomy_state) {
  const auto prev_state = prev_autonomy_state.autonomy_state();
  const auto cur_state = now_autonomy_state.autonomy_state();
  const bool is_non_autonomy = !IsAutoDrive(cur_state) &&
                               cur_state != AutonomyStateProto::AUTO_SPEED_ONLY;
  if (is_non_autonomy) {
    return ResetReasonProto::NON_AUTONOMY;
  }

  const bool is_first_engage =
      (prev_state == AutonomyStateProto::READY_TO_AUTO_DRIVE) &&
      (cur_state == AutonomyStateProto::AUTO_SPEED_ONLY);
  if (is_first_engage) {
    return ResetReasonProto::FIRST_ENGAGE;
  }

  return cur_state == AutonomyStateProto::AUTO_SPEED_ONLY
             ? ResetReasonProto::SPEED_ONLY
             : ResetReasonProto::NON_AUTONOMY;
}

bool MaybeResetByAutonomyState(const AutonomyStateProto& prev_autonomy_state,
                               const AutonomyStateProto& now_autonomy_state,
                               ResetReasonProto::Reason* reset_reason) {
  if (!IsAutoDrive(now_autonomy_state.autonomy_state()) &&
      now_autonomy_state.autonomy_state() !=
          AutonomyStateProto::AUTO_STEER_ONLY &&
      now_autonomy_state.autonomy_state() !=
          AutonomyStateProto::AUTO_SPEED_ONLY) {
    *reset_reason = ResetReasonProto::NON_AUTONOMY;
    return true;
  } else if (IS_ENGAGE(prev_autonomy_state.autonomy_state(),
                       now_autonomy_state.autonomy_state()) ||
             (prev_autonomy_state.autonomy_state() ==
                  AutonomyStateProto::READY_TO_AUTO_DRIVE &&
              now_autonomy_state.autonomy_state() ==
                  AutonomyStateProto::AUTO_SPEED_ONLY)) {
    *reset_reason = ResetReasonProto::FIRST_ENGAGE;
    return true;
  } else if (IsAutoSteerOnlyToAutoDrive(prev_autonomy_state.autonomy_state(),
                                        now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::STEER_ONLY_ENGAGE;
    return true;
  } else if (IsAutoSpeedOnlyToAutoDrive(prev_autonomy_state.autonomy_state(),
                                        now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::SPEED_ONLY_ENGAGE;
    return true;
  } else if (now_autonomy_state.autonomy_state() ==
             AutonomyStateProto::AUTO_STEER_ONLY) {
    *reset_reason = ResetReasonProto::STEER_ONLY;
    return true;
  }

  return false;
}

bool NeedForceResetEstPlanner(const TrajectoryProto& prev_trajectory,
                              // bool previously_triggered_aeb,
                              bool is_emergency_stop, bool rerouted,
                              bool full_stopped,
                              ResetReasonProto::Reason* reset_reason) {
  if (rerouted) {
    *reset_reason = ResetReasonProto::REROUTED;
    return true;
  }

  if (full_stopped) {
    *reset_reason = ResetReasonProto::FULL_STOP;
    return true;
  }

  return false;
}

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByDrivePassage(
    const PlannerSemanticMapManager& psmm, const RouteSections& sections,
    const Vec2d& query_point) {
  ASSIGN_OR_RETURN(const auto nearest_lane_path,
                   FindClosestLanePathOnRouteSectionsToSmoothPoint(
                       psmm, sections, query_point));

  const double step_s = std::min(1.0, 0.5 * nearest_lane_path.length());
  ASSIGN_OR_RETURN(const auto drive_passage,
                   BuildDrivePassageFromLanePath(
                       psmm, nearest_lane_path, step_s,
                       /*avoid_loop=*/false, /*avoid_notcontinuous=*/false,
                       /*backward_extend_len=*/0.0,
                       /*required_planning_horizon=*/0.0,
                       /*override_speed_limit_mps=*/std::nullopt),
                   _ << "FindSmoothPointOnRouteSectionsByDrivePassage: "
                        "BuildDrivePassageFromLanePath on "
                     << nearest_lane_path.DebugString() << " failed.");

  ASSIGN_OR_RETURN(
      const auto sl, drive_passage.QueryFrenetCoordinateAt(query_point),
      _ << "FindSmoothPointOnRouteSectionsByDrivePassage: Fail to project ego"
           "pos ("
        << query_point.x() << ", " << query_point.y()
        << ") on drive passage from lane path "
        << nearest_lane_path.DebugString());

  const auto start_lane_point = nearest_lane_path.AfterArclength(sl.s).front();
  for (int i = 0; i < sections.size(); ++i) {
    SMM_ASSIGN_SECTION_OR_CONTINUE_ISSUE(section_info, psmm,
                                         sections.section_ids()[i]);

    if (std::find(section_info.lanes().begin(), section_info.lanes().end(),
                  start_lane_point.lane_id()) != section_info.lanes().end()) {
      return PointOnRouteSections{.accum_s = sl.s,
                                  .section_idx = i,
                                  .fraction = start_lane_point.fraction(),
                                  .lane_id = start_lane_point.lane_id()};
    }
  }

  return absl::NotFoundError(
      absl::StrCat("FindSmoothPointOnRouteSectionsByDrivePassage: Point (",
                   query_point.x(), ", ", query_point.y(),
                   ") is not on route sections:", sections.DebugString()));
}

// PathPoint ResamplePathPoint(const DiscretizedPath& current_smooth_path,
//                             double sample_s) {
//   if (sample_s > current_smooth_path.length()) {
//     return GetPathPointAlongCircle(current_smooth_path.back(),
//                                    sample_s - current_smooth_path.length());
//   } else if (sample_s < 0.0) {
//     return GetPathPointAlongCircle(current_smooth_path.front(), sample_s);
//   } else {
//     return current_smooth_path.Evaluate(sample_s);
//   }
// }

// }  // namespace

// std::shared_ptr<const ObjectsProto> GetAllObjects(
//     const std::shared_ptr<const ObjectsProto>& real_objects,
//     const std::shared_ptr<const ObjectsProto>& virtual_objects) {
//   ("GetAllObjects");

//   ObjectsView objects_view;
//   if (real_objects != nullptr) {
//     objects_view.UpdateObjects(ObjectsProto::SCOPE_REAL, real_objects);
//   }
//   if (virtual_objects != nullptr) {
//     objects_view.UpdateObjects(ObjectsProto::SCOPE_VIRTUAL, virtual_objects);
//   }
//   return objects_view.ExportAllObjectsProto();
// }

bool MaybeResetEstPlanner(
    const ApolloTrajectoryPointProto& pre_reset_planned_point,
    const Vec2d& current_pos, ResetReasonProto::Reason* reset_reason) {
  double kLongitudinalErrorForReset = 2.0;  // m.
  kLongitudinalErrorForReset =
      std::max(kLongitudinalErrorForReset, pre_reset_planned_point.v() * 0.3);
  return MaybeReset(
      pre_reset_planned_point, current_pos, kLongitudinalErrorForReset,
      FLAGS_planner_lateral_reset_error, "est_planner", reset_reason);
}

void FillTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto>& planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto>& past_points,
    const mapping::LanePath& target_lane_path_from_current,
    const LaneChangeStateProto& lane_change_state, TurnSignal turn_signal,
    // bool is_aeb_triggered,
    const TrajectoryValidationResultProto& validate_result,
    TrajectoryProto* trajectory) {
  // ("FillTrajectoryProto");
  trajectory->set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));
  for (int i = 0; i < planned_trajectory.size(); ++i) {
    *trajectory->add_trajectory_point() = planned_trajectory[i];
  }
  // NOTE: past_points are designed for controller.
  for (const auto& past_point : past_points) {
    *trajectory->add_past_points() = past_point;
  }
  target_lane_path_from_current.ToProto(
      trajectory->mutable_target_lane_path_from_current());
  trajectory->set_turn_signal(turn_signal);

  // TODO： redesign it after onboard freespace planner.
  // trajectory->set_gear(Chassis::GEAR_DRIVE);
  // *(trajectory->mutable_door_decision()) = door_decision;
  // trajectory->set_aeb_triggered(is_aeb_triggered);
  // trajectory->mutable_driving_state()->CopyFrom(driving_state);

  trajectory->set_lane_change_stage(lane_change_state.stage());
  if (lane_change_state.stage() != LCS_NONE) {
    trajectory->set_lane_change_left(lane_change_state.lc_left());
  }
  // trajectory->set_low_speed_freespace(false);
  // trajectory->set_enable_stationary_steering(false);
  // TODO: Validate trajectories in planner_module.
  trajectory->mutable_traj_validation_result()->CopyFrom(validate_result);
}

// void ConvertTrajectoryToGlobalCoordinates(
//     const CoordinateConverter& coordinate_converter,
//     const TrajectoryProto& trajectory,
//     std::vector<PlannerState::PosePoint>* previous_trajectory_global,
//     std::vector<PlannerState::PosePoint>* previous_past_trajectory_global) {
//   // ---------------------------
//   // Copy from planner_module.cc
//   ("ConvertTrajectoryToGlobalCoordinates");
//   previous_trajectory_global->clear();
//   for (const auto& trajectory_point : trajectory.trajectory_point()) {
//     const auto& path_point = trajectory_point.path_point();
//     const Vec2d smooth_point(path_point.x(), path_point.y());
//     const Vec2d global_point =
//         coordinate_converter.SmoothToGlobal(smooth_point);
//     const double global_yaw =
//         coordinate_converter.SmoothYawToGlobalNoNormalize(path_point.theta());
//     previous_trajectory_global->push_back(
//         {.pos = Vec2d(global_point.x(), global_point.y()),
//          .theta = global_yaw});
//   }

//   // NOTE: trajectory.past_points() might be empty.
//   // TODO: check trajectory.past_points()
//   previous_past_trajectory_global->clear();
//   for (const auto& trajectory_point : trajectory.past_points()) {
//     const auto& path_point = trajectory_point.path_point();
//     const Vec2d smooth_point(path_point.x(), path_point.y());
//     const Vec2d global_point =
//         coordinate_converter.SmoothToGlobal(smooth_point);
//     const double global_yaw =
//         coordinate_converter.SmoothYawToGlobalNoNormalize(path_point.theta());
//     previous_past_trajectory_global->push_back(
//         {.pos = Vec2d(global_point.x(), global_point.y()),
//          .theta = global_yaw});
//   }
// }

// void ConvertPreviousTrajectoryToCurrentSmoothLateral(
//     absl::Time predicted_plan_time,
//     const CoordinateConverter& coordinate_converter,
//     std::vector<PathPoint> previous_path,
//     const std::vector<PlannerState::PosePoint>& previous_trajectory_global,
//     const std::vector<PlannerState::PosePoint>&
//     previous_past_trajectory_global, TrajectoryProto* previous_trajectory) {
//   std::optional<int> start_index_on_prev_traj = std::nullopt;
//   start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
//       predicted_plan_time, *previous_trajectory);
//   if (!start_index_on_prev_traj.has_value() || previous_path.empty()) {
//     return;
//   }

//   DiscretizedPath current_smooth_path(std::move(previous_path));

//   // previous trajectory resample.
//   std::vector<double> past_points_s;
//   std::vector<double> points_s;

//   if (!previous_trajectory->past_points().empty()) {
//     for (int i = 0; i < previous_trajectory->past_points_size(); ++i) {
//       const auto& current_path_point =
//           previous_trajectory->past_points(i).path_point();
//       past_points_s.push_back(current_path_point.s());
//     }
//   }

//   if (!previous_trajectory->trajectory_point().empty()) {
//     for (int i = 0; i < previous_trajectory->trajectory_point_size(); ++i) {
//       const auto& current_path_point =
//           previous_trajectory->trajectory_point(i).path_point();
//       points_s.push_back(current_path_point.s());
//     }
//   }

//   const auto& prev_planned_traj_point =
//       previous_trajectory->trajectory_point(*start_index_on_prev_traj);
//   const FrenetCoordinate prev_pt_sl = current_smooth_path.XYToSL(
//       Vec2d(prev_planned_traj_point.path_point().x(),
//             prev_planned_traj_point.path_point().y()));
//   const double prev_planned_s_current_smooth = prev_pt_sl.s;
//   const double delta_s =
//       points_s[*start_index_on_prev_traj] - prev_planned_s_current_smooth;

//   if (!previous_trajectory->trajectory_point().empty()) {
//     for (int i = 0; i < previous_trajectory->trajectory_point_size(); ++i) {
//       auto* path_point = previous_trajectory->mutable_trajectory_point(i)
//                              ->mutable_path_point();
//       const double sample_s = points_s[i] - delta_s;
//       *path_point = ResamplePathPoint(current_smooth_path, sample_s);
//       path_point->set_s(points_s[i]);
//     }
//   }
//   if (!previous_trajectory->past_points().empty()) {
//     for (int i = 0; i < previous_trajectory->past_points_size(); ++i) {
//       auto* path_point =
//           previous_trajectory->mutable_past_points(i)->mutable_path_point();
//       const double sample_s = past_points_s[i] - delta_s;
//       *path_point = ResamplePathPoint(current_smooth_path, sample_s);
//       path_point->set_s(past_points_s[i]);
//     }
//   }
// }

// void ReportCandidateTrafficLightInfo(
//     const TrafficLightInfoMap& traffic_light_map, PlannerDebugProto* debug) {
//   CHECK_NOTNULL(debug);
//   for (auto iter = traffic_light_map.begin(); iter !=
//   traffic_light_map.end();
//        ++iter) {
//     iter->second.ToProto(debug->add_candidate_traffic_light_info());
//   }
// }

// void ReportSelectedTrafficLightInfo(
//     const TrafficLightInfoMap& traffic_light_map,
//     const mapping::LanePath& lane_path, TrafficLightInfoProto* proto) {
//   for (const auto id : lane_path.lane_ids()) {
//     const auto iter = traffic_light_map.find(id);
//     if (iter != traffic_light_map.end()) {
//       iter->second.ToProto(proto);
//       return;
//     }
//   }
// }

std::vector<ApolloTrajectoryPointProto> CreatePreviousTrajectory(
    absl::Time plan_time, const TrajectoryProto& previous_trajectory,
    const MotionConstraintParamsProto& motion_constraint_params, bool reset) {
  // ("CreatePrevTrajectory");
  DLOG(INFO) << __FUNCTION__;
  if (previous_trajectory.trajectory_point().empty() || reset) return {};

  const double now_in_sec = ToUnixDoubleSeconds(plan_time);
  const double time_advancement = std::max(
      0.0, now_in_sec - previous_trajectory.trajectory_start_timestamp());
  const std::vector<ApolloTrajectoryPointProto> previous_trajectory_points(
      previous_trajectory.trajectory_point().begin(),
      previous_trajectory.trajectory_point().end());
  return ShiftTrajectoryByTime(time_advancement, previous_trajectory_points,
                               motion_constraint_params.max_decel_jerk(),
                               motion_constraint_params.max_accel_jerk());
}

PlanStartPointInfo ComputeAccPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state, bool rerouted, bool aeb,
    double front_wheel_angle,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool override) {
  absl::Time plan_start_time = predicted_plan_time;

  // The below variables may have meaningful values only in auto drive.
  std::optional<ApolloTrajectoryPointProto> prev_planned_traj_point;
  ApolloTrajectoryPointProto prev_planned_now_point;
  double path_s_increment_from_previous_frame = 0.0;
  bool full_stop = false;
  // Only has value if not reset.
  std::optional<int> start_index_on_prev_traj;
  // Autonomy state based reset logic.
  ResetReasonProto::Reason reset_reason =
      GetAccResetReasonByAutonomyState(prev_autonomy_state, now_autonomy_state);

  // We always have one reset reason.
  CHECK(reset_reason != ResetReasonProto::NONE);
  if (reset_reason == ResetReasonProto::SPEED_ONLY) {
    // Reset logic in AUTO_SPEED_ONLY drive.
    start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
        predicted_plan_time, prev_trajectory);
    if (!start_index_on_prev_traj.has_value()) {
      reset_reason = ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND;
    } else {
      prev_planned_traj_point = std::make_optional<ApolloTrajectoryPointProto>(
          prev_trajectory.trajectory_point(*start_index_on_prev_traj));
      plan_start_time =
          FromUnixDoubleSeconds(prev_planned_traj_point->relative_time() +
                                prev_trajectory.trajectory_start_timestamp());
      prev_planned_traj_point->set_relative_time(0.0);
      path_s_increment_from_previous_frame =
          prev_planned_traj_point->path_point().s();
      prev_planned_traj_point->mutable_path_point()->set_s(0.0);
      // Check if we need reset.
      // Trajectory point from previous trajectory at Clock::Now().
      if (!InterpolatePointFromPrevTrajectoryIncludingPast(
              FromUnixDoubleSeconds(pose.timestamp()), prev_trajectory,
              &prev_planned_now_point)) {
        // Reset if prev_planned_now_point not found.
        reset_reason = ResetReasonProto::PREV_NOW_POINT_NOT_FOUND;
      } else {
        const Vec2d current_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
        // Reset if control error too large.
        ResetReasonProto::Reason control_reset_reason = ResetReasonProto::NONE;
        constexpr double kAccLongitudinalErrorForReset = 5.0;  // m. const auto
        const auto control_reset = MaybeReset(
            prev_planned_now_point, current_pos, kAccLongitudinalErrorForReset,
            FLAGS_planner_lateral_reset_error, "acc_planner",
            &control_reset_reason);
        if (control_reset &&
            control_reset_reason == ResetReasonProto::LON_ERROR_TOO_LARGE) {
          reset_reason = control_reset_reason;
        }
      }
    }
    constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
    full_stop = prev_planned_traj_point.has_value() &&
                prev_planned_traj_point->v() == 0.0 &&
                std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;
    // Check if we need to force reset if current reset reason is SPEED_ONLY.
    if (reset_reason == ResetReasonProto::SPEED_ONLY) {
      ResetReasonProto::Reason force_reset_reason = ResetReasonProto::NONE;
      const bool need_force_reset = NeedForceResetEstPlanner(
          prev_trajectory, aeb, rerouted, full_stop, &force_reset_reason);
      if (need_force_reset) {
        reset_reason = force_reset_reason;
      }
    }
  }

  // We only need to perform lateral reset or full reset.
  const bool only_lat_reset = (reset_reason == ResetReasonProto::SPEED_ONLY);
  if (only_lat_reset) {
    CHECK(prev_planned_traj_point.has_value());
  } else {
    start_index_on_prev_traj = std::nullopt;
  }
  ApolloTrajectoryPointProto start_point;
  if (only_lat_reset && (!override)) {
    start_point = ComputeAccPlanStartPointAfterLateralReset(
        prev_planned_now_point, prev_planned_traj_point, pose,
        front_wheel_angle, vehicle_geom_params,
        ToUnixDoubleSeconds(plan_start_time) - pose.timestamp());
  } else {
    start_point = ComputePlanStartPointAfterReset(
        prev_planned_traj_point, pose, front_wheel_angle,
        motion_constraint_params, vehicle_geom_params, vehicle_drive_params,
        /*is_forward_task=*/true);
  }

  return PlanStartPointInfo{
      .reset = true,
      .start_index_on_prev_traj = start_index_on_prev_traj,
      .start_point = start_point,
      .path_s_increment_from_previous_frame =
          (only_lat_reset && !override) ? path_s_increment_from_previous_frame
                                        : 0.0,
      .plan_time = (only_lat_reset && !override)
                       ? plan_start_time
                       : FromUnixDoubleSeconds(pose.timestamp()),
      .full_stop = full_stop,
      .reset_reason = reset_reason};
}

PlanStartPointInfo ComputeEstPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state,
    // bool previously_triggered_aeb,
    bool rerouted, bool aeb, double front_wheel_angle,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool override,
    Behavior_FunctionId function_id) {
  // ("ComputeEstPlanStartPoint");
  DLOG(INFO) << __FUNCTION__;

  bool reset = false;
  ResetReasonProto::Reason reset_reason = ResetReasonProto::NONE;
  absl::Time plan_start_time = predicted_plan_time;
  // The below variables may have meaningful values only in auto drive.
  std::optional<ApolloTrajectoryPointProto> prev_planned_traj_point =
      std::nullopt;
  double path_s_increment_from_previous_frame = 0.0;
  bool full_stop = false;
  // Only has value if not reset.
  std::optional<int> start_index_on_prev_traj = std::nullopt;
  // Autonomy state based reset logic.
  reset = MaybeResetByAutonomyState(prev_autonomy_state, now_autonomy_state,
                                    &reset_reason);

  if (reset) {
    LOG_WARN
        << "reset reason: " << ResetReasonProto_Reason_Name(reset_reason)
        << ", prev: "
        << AutonomyStateProto_State_Name(prev_autonomy_state.autonomy_state())
        << ", now: "
        << AutonomyStateProto_State_Name(now_autonomy_state.autonomy_state())
        << ", function_id: " << Behavior_FunctionId_Name(function_id);
  }
  if (!reset) {
    // Reset logic in auto drive.
    start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
        predicted_plan_time, prev_trajectory);
    if (!start_index_on_prev_traj.has_value()) {
      reset = true;
      reset_reason = ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND;
    } else {
      prev_planned_traj_point = std::make_optional<ApolloTrajectoryPointProto>(
          prev_trajectory.trajectory_point(*start_index_on_prev_traj));
      plan_start_time =
          FromUnixDoubleSeconds(prev_planned_traj_point->relative_time() +
                                prev_trajectory.trajectory_start_timestamp());
      prev_planned_traj_point->set_relative_time(0.0);
      path_s_increment_from_previous_frame =
          prev_planned_traj_point->path_point().s();
      prev_planned_traj_point->mutable_path_point()->set_s(0.0);
      // Check if we need reset.
      // Trajectory point from previous trajectory at Clock::Now().
      ApolloTrajectoryPointProto prev_planned_now_point;
      if (!InterpolatePointFromPrevTrajectoryIncludingPast(
              FromUnixDoubleSeconds(pose.timestamp()), prev_trajectory,
              &prev_planned_now_point)) {
        // Reset if prev_planned_now_point not found.
        reset = true;
        reset_reason = ResetReasonProto::PREV_NOW_POINT_NOT_FOUND;
      } else {
        const Vec2d current_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
        // Reset if control error too large.
        reset = MaybeResetEstPlanner(prev_planned_now_point, current_pos,
                                     &reset_reason);
      }
    }
    constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
    constexpr double kLargeTimeErrorThreshold = 0.5;  // s.
    constexpr double kEps = 1e-4;
    const bool time_error_large =
        std::fabs(prev_trajectory.trajectory_start_timestamp() -
                  pose.timestamp()) > kLargeTimeErrorThreshold;
    full_stop = time_error_large && prev_planned_traj_point.has_value() &&
                prev_planned_traj_point->v() < kFullStopSpeedThreshold &&
                std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;

    // const std::vector<ApolloTrajectoryPointProto> previous_trajectory_points(
    //     prev_trajectory.trajectory_point().begin(),
    //     prev_trajectory.trajectory_point().end());
    // double avg_v = 0.0;
    // if (previous_trajectory_points.size() > 2) {
    //   for (size_t i = 0; i + 1 < previous_trajectory_points.size(); i++) {
    //     avg_v += std::abs(previous_trajectory_points[i].v());
    //   }
    //   avg_v = avg_v / previous_trajectory_points.size();
    // }

    // if ((!reset) && avg_v < kEps &&
    //     std::abs(pose.vel_body().x()) < kEps) {
    //   reset = true;
    //   reset_reason = ResetReasonProto::STEER_ONLY;
    // }

    if (!reset) {
      reset = NeedForceResetEstPlanner(prev_trajectory,
                                       // previously_triggered_aeb,
                                       aeb, rerouted, full_stop, &reset_reason);
    }
    if ((!reset) && override) {
      reset = true;
      reset_reason = ResetReasonProto::STEER_ONLY;
    }
  }

  if (!reset) {
    CHECK(prev_planned_traj_point.has_value());
  } else {
    start_index_on_prev_traj = std::nullopt;
  }
  ApolloTrajectoryPointProto start_point;

  if (reset && reset_reason == ResetReasonProto::STEER_ONLY) {
    start_point = ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
        prev_trajectory, pose, front_wheel_angle, vehicle_geom_params,
        vehicle_drive_params);
  } else if (reset && reset_reason == ResetReasonProto::LAT_ERROR_TOO_LARGE) {
    start_point = ComputePlanStartPointAfterLateralReset(
        prev_planned_traj_point, pose, front_wheel_angle, vehicle_geom_params,
        vehicle_drive_params);
  } else if (reset) {
    start_point = ComputePlanStartPointAfterReset(
        prev_planned_traj_point, pose, front_wheel_angle,
        motion_constraint_params, vehicle_geom_params, vehicle_drive_params,
        /*is_forward_task=*/true);
  } else {
    start_point = *prev_planned_traj_point;
  }

  return PlanStartPointInfo{
      .reset = reset,
      .start_index_on_prev_traj = start_index_on_prev_traj,
      .start_point = start_point,
      .path_s_increment_from_previous_frame =
          reset ? 0.0 : path_s_increment_from_previous_frame,
      .plan_time =
          reset ? FromUnixDoubleSeconds(pose.timestamp()) : plan_start_time,
      .full_stop = full_stop,
      .reset_reason = reset_reason};
}

absl::Duration GetStPathPlanLookAheadTime(
    const PlanStartPointInfo& plan_start_point_info, const PoseProto& pose,
    absl::Duration planned_look_ahead_time,
    const TrajectoryProto& previous_trajectory) {
  DLOG(INFO) << __FUNCTION__;
  absl::Duration look_ahead_time = planned_look_ahead_time;
  // Since we have spacetime planning but lateral and longitudinal control,
  // first reference point for lateral control may have large time diff with
  // plan start point. Notice that after getting spacetime trajectory, speed
  // planning using spacetime result as path will be running. If we start plan
  // path from point close to control ref point, there will be a performance
  // improvement on lateral control. So We check the closest point of pose on
  // previous trajectory and check the diff between it and plan start point,
  // if too large, set the path plan start point to this point.
  // https://docs.google.com/presentation/d/1oXnBZ95lRy_X1U9U_dPWJ_a8MPZSGTSRJuPeqzrrhiM/edit#slide=id.p1
  if (FLAGS_planner_enable_path_start_point_look_ahead &&
      plan_start_point_info.start_index_on_prev_traj.has_value() &&
      !previous_trajectory.trajectory_point().empty()) {
    const auto& previous_trajectory_points =
        previous_trajectory.trajectory_point();
    const Vec2d plan_start_pos =
        Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
    const auto closest_iter = absl::c_min_element(
        previous_trajectory_points,
        [&plan_start_pos](const ApolloTrajectoryPointProto& p1,
                          const ApolloTrajectoryPointProto& p2) {
          const Vec2d pos1(p1.path_point().x(), p1.path_point().y());
          const Vec2d pos2(p2.path_point().x(), p2.path_point().y());
          return (pos1 - plan_start_pos).squaredNorm() <
                 (pos2 - plan_start_pos).squaredNorm();
        });
    const int closest_index_on_prev_traj =
        std::distance(previous_trajectory_points.begin(), closest_iter);
    const auto diff_time =
        static_cast<double>((closest_index_on_prev_traj -
                             *plan_start_point_info.start_index_on_prev_traj) *
                            kTrajectoryTimeStep);
    if (diff_time > FLAGS_planner_path_start_point_time_diff_limit) {
      constexpr double kLookAheadTimeMaxTime = 5.0;  // s
      look_ahead_time +=
          absl::Seconds(std::min(kLookAheadTimeMaxTime, diff_time));
    }
  }
  return look_ahead_time;
}

StPathPlanStartPointInfo GetStPathPlanStartPointInfo(
    const absl::Duration look_ahead_time,
    const PlanStartPointInfo& plan_start_point_info,
    const TrajectoryProto& previous_trajectory,
    std::optional<double> trajectory_optimizer_time_step,
    std::optional<absl::Time> last_st_path_plan_start_time) {
  absl::Time path_planning_time =
      plan_start_point_info.plan_time + look_ahead_time;

  // Increase path_planning_time,
  // Making the time duration between last_st_path_plan_start_time
  // and path_planning_time to be integer times of
  // trajectory_optimizer_time_step.
  if (last_st_path_plan_start_time.has_value() &&
      FLAGS_planner_st_path_planner_lookahead_for_trajectory_optimizer_synchronization) {  // NOLINT
    CHECK(trajectory_optimizer_time_step.has_value());
    CHECK_GT(*trajectory_optimizer_time_step, 0.0);
    constexpr double kTimeEpsilon = 0.001;  // In seconds.
    const double delta_t =
        std::max(absl::ToDoubleSeconds(path_planning_time -
                                       *last_st_path_plan_start_time),
                 0.0);
    const double res = std::fmod(delta_t, *trajectory_optimizer_time_step);
    if (res > kTimeEpsilon &&
        res < (*trajectory_optimizer_time_step) - kTimeEpsilon) {
      path_planning_time +=
          absl::Seconds((*trajectory_optimizer_time_step) - res);
    }
  }

  const auto path_start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
      path_planning_time, previous_trajectory);
  if (plan_start_point_info.reset ||
      !path_start_index_on_prev_traj.has_value()) {
    // VLOG(3) << "Do not change path plan start time because
    // planner reset or "
    //            "path planning time not found in previous
    //            trajectory.";
    return {.reset = plan_start_point_info.reset,
            .relative_index_from_plan_start_point = 0,
            .start_point = plan_start_point_info.start_point,
            .plan_time = plan_start_point_info.plan_time};
  } else {
    // VLOG(3) << "path_planning_time:" << path_planning_time;
    ApolloTrajectoryPointProto path_plan_start_point =
        previous_trajectory.trajectory_point(*path_start_index_on_prev_traj);
    path_plan_start_point.set_relative_time(0.0);
    path_plan_start_point.mutable_path_point()->set_s(0.0);
    // If planner not reset,
    // plan_start_point_info.start_index_on_prev_traj should have
    // value.
    CHECK(plan_start_point_info.start_index_on_prev_traj.has_value());
    return {.reset = plan_start_point_info.reset,
            .relative_index_from_plan_start_point =
                *path_start_index_on_prev_traj -
                *plan_start_point_info.start_index_on_prev_traj,
            .start_point = path_plan_start_point,
            .plan_time = path_planning_time};
  }
}

absl::StatusOr<mapping::LanePath> FindPreferredLanePathFromTeleop(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start,
    const RouteNaviInfo& route_navi_info, mapping::ElementId ego_proj_lane_id,
    DriverAction::LaneChangeCommand lc_cmd) {
  if (lc_cmd == DriverAction::LC_CMD_NONE) {
    return absl::NotFoundError("Empty lane change command!");
  }

  SMM_ASSIGN_SECTION_OR_ERROR_ISSUE(section_info, psmm,
                                    route_sections_from_start.front().id);
  const auto& lane_ids = section_info.lanes();
  const auto current_it =
      std::find(lane_ids.begin(), lane_ids.end(), ego_proj_lane_id);
  if (current_it == lane_ids.end()) {
    return absl::NotFoundError(
        "Current lane not found in the current route section!");
  }

  constexpr double kForwardInitLength = 60.0;  // m.
  const auto short_route_sections = *ClampRouteSectionsBeforeArcLength(
      psmm, route_sections_from_start, kForwardInitLength);
  const RouteSectionsInfo short_sections_info(psmm, &short_route_sections);
  const double start_frac = short_sections_info.start_fraction();

  if (lc_cmd == DriverAction::LC_CMD_STRAIGHT) {
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *current_it, start_frac,
        short_sections_info.length());
  }

  if (lc_cmd == DriverAction::LC_CMD_LEFT) {
    if (current_it == lane_ids.begin()) {
      return absl::NotFoundError("Already on the leftmost lane!");
    }
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *std::prev(current_it),
        start_frac, short_sections_info.length());
  } else {
    if (std::next(current_it) == lane_ids.end()) {
      return absl::NotFoundError("Already on the rightmost lane!");
    }
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *std::next(current_it),
        start_frac, short_sections_info.length());
  }
}

absl::StatusOr<std::tuple<RouteSections, RouteSections, PointOnRouteSections>>
ProjectPointToRouteSections(const PlannerSemanticMapManager& psmm,
                            const RouteSections& route_sections,
                            const Vec2d& pos, double projection_range,
                            double keep_behind_length) {
  // ("ProjectPointToRouteSections");

  ASSIGN_OR_RETURN(const auto project_route_sections,
                   ClampRouteSectionsBeforeArcLength(psmm, route_sections,
                                                     projection_range));

  ASSIGN_OR_RETURN(auto point_proj,
                   FindSmoothPointOnRouteSectionsByDrivePassage(
                       psmm, project_route_sections, pos));

  std::vector<mapping::SectionId> sec_ids;
  for (int i = point_proj.section_idx; i < project_route_sections.size(); ++i) {
    sec_ids.push_back(project_route_sections.route_section_segment(i).id);
  }
  const RouteSections projected_route_sections_from_start(
      point_proj.fraction, project_route_sections.end_fraction(),
      std::move(sec_ids), project_route_sections.destination());

  ASSIGN_OR_RETURN(
      auto sections_from_start,
      AlignRouteSections(route_sections, projected_route_sections_from_start));

  RouteSections sections_with_behind;

  if (point_proj.accum_s > keep_behind_length) {
    ASSIGN_OR_RETURN(
        sections_with_behind,
        ClampRouteSectionsAfterArcLength(
            psmm, route_sections, point_proj.accum_s - keep_behind_length));
  } else {
    sections_with_behind = route_sections;
  }

  return std::make_tuple(std::move(sections_from_start),
                         std::move(sections_with_behind), point_proj);
}

bool CheckIfAllowCancel(const ApolloTrajectoryPointProto& plan_start_point,
                        const VehicleGeometryParamsProto& vehicle_geometry,
                        const Vec2d& ego_pos,
                        st::mapping::LanePath* preferred_lane_path,
                        const double& dist_buffer) {
  double buffer = std::clamp(dist_buffer, -0.2, 0.2);
  double s;
  if (!preferred_lane_path->lane_seq()) {
    return true;
  }
  auto neareast_lane = preferred_lane_path->lane_seq()->GetNearestLane(
      ad_byd::planning::Point2d(ego_pos.x(), ego_pos.y()));
  if (!neareast_lane) {
    return true;
  }
  neareast_lane->center_line().GetDistance(
      ad_byd::planning::Point2d(ego_pos.x(), ego_pos.y()), nullptr, &s);
  double half_lane_width =
      std::max(neareast_lane->GetWidthAtAccumS(s) * 0.5, kMinHalfLaneWidth);
  double min_dist = DBL_MAX;
  const auto ego_width = vehicle_geometry.width();
  const auto ego_length = vehicle_geometry.length();
  st::Box2d ego_box(ego_pos, plan_start_point.path_point().theta(), ego_length,
                    ego_width);

  const auto ego_corners =
      ego_box.GetCornersWithBufferCounterClockwise(/*lat_buffer=*/0.0,
                                                   /*lon_buffer=*/0.0);
  for (auto corner : ego_corners) {
    auto temp_dist =
        preferred_lane_path->lane_seq()->GetProjectionDistance(corner, nullptr);
    min_dist = min_dist < temp_dist ? min_dist : temp_dist;
  }
  Log2DDS::LogDataV2("mlc_debug", absl::StrCat("min_dist", min_dist));
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("half_lane_width", half_lane_width));
  Log2DDS::LogDataV2("mlc_debug", absl::StrCat("buffer", buffer));
  Log2DDS::LogDataV2("mlc_debug", absl::StrCat("half_lane_width - buffer: ",
                                               half_lane_width - buffer));
  return min_dist > half_lane_width - buffer;
}

bool CheckIfCrossLine(const ApolloTrajectoryPointProto& plan_start_point,
                      const VehicleGeometryParamsProto& vehicle_geometry,
                      const Vec2d& ego_pos,
                      const st::mapping::LanePath& prev_lp_before_lc,
                      const double& dist_buffer) {
  double buffer = std::clamp(dist_buffer, -0.2, 0.2);
  double s;
  std::vector<std::string> debug_string{};
  if (!prev_lp_before_lc.lane_seq()) {
    return true;
  }
  auto nearest_lane = prev_lp_before_lc.lane_seq()->GetNearestLane(
      ad_byd::planning::Point2d(ego_pos.x(), ego_pos.y()));
  if (!nearest_lane) {
    return true;
  }
  nearest_lane->center_line().GetDistance(
      ad_byd::planning::Point2d(ego_pos.x(), ego_pos.y()), nullptr, &s);
  double half_lane_width =
      std::max(nearest_lane->GetWidthAtAccumS(s) * 0.5, kMinHalfLaneWidth);
  debug_string.emplace_back(absl::StrCat("nearest_lane: ", nearest_lane->id()));
  debug_string.emplace_back(absl::StrCat("half_lane_width: ", half_lane_width));
  debug_string.emplace_back(absl::StrCat("buffer: ", buffer));
  double max_dist = DBL_MIN;
  const auto ego_width = vehicle_geometry.width();
  const auto ego_length = vehicle_geometry.length();
  st::Box2d ego_box(ego_pos, plan_start_point.path_point().theta(), ego_length,
                    ego_width);

  const auto ego_corners =
      ego_box.GetCornersWithBufferCounterClockwise(/*lat_buffer=*/0.0,
                                                   /*lon_buffer=*/0.0);
  for (auto corner : ego_corners) {
    auto temp_dist =
        prev_lp_before_lc.lane_seq()->GetProjectionDistance(corner, nullptr);
    max_dist = max_dist > temp_dist ? max_dist : temp_dist;
  }
  debug_string.emplace_back(absl::StrCat("max_dist: ", max_dist));
  Log2DDS::LogDataV2("mlc_debug", debug_string);
  return max_dist > half_lane_width + buffer;
}

void HandleManualLcCommand(
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geometry,
    const PlannerSemanticMapManager& psmm,
    st::DriverAction::LaneChangeCommand new_lc_cmd,
    const st::mapping::LanePath& prev_lp_before_lc,
    const LaneChangeStateProto& prev_lc_state, const Vec2d& ego_pos,
    const double& ego_theta, st::mapping::LanePath* preferred_lane_path,
    ALCState* alc_state, st::DriverAction::LaneChangeCommand* lc_cmd_state,
    st::planning::LcFeasibility* lc_unable_reason, bool* if_cancel_lc,
    bool* if_allow_cancel_lc, int last_manual_lc_time,
    LaneChangeReason last_lc_reason,
    std::vector<ad_byd::planning::LaneSequencePtr> candidate_lane_seqs,
    const ad_byd::planning::EIEChoiceType& eie_choice_type) {
  // 1. update preferred_lane_path
  if (!preferred_lane_path->IsEmpty()) {
    const auto& prev_pref_laneseq = preferred_lane_path->lane_seq();
    if (prev_pref_laneseq && prev_pref_laneseq->IsValid()) {
      ad_byd::planning::LaneSequencePtr update_seq;
      std::vector<std::string> debug;
      std::vector<ad_byd::planning::Point2d> debug_match_point;
      const auto seq = psmm.map_ptr()->GetSameLaneSequenceV2(
          prev_pref_laneseq, ego_pos.x(), ego_pos.y(), ego_theta, debug,
          debug_match_point);
      if (seq) {
        Log2DDS::LogDataV2("mlc_debug", "update preferred_lane_path");
        const auto& tgt_lane = seq->GetNearestLane(ego_pos);
        if (tgt_lane) {
          Log2DDS::LogDataV2("mlc_debug",
                             absl::StrCat("nearest lane on prev_pref_laneseq: ",
                                          tgt_lane->id()));
          for (int i = 0; i < candidate_lane_seqs.size(); ++i) {
            const auto& lane_seq = candidate_lane_seqs.at(i);
            if (lane_seq->IsOnLaneSequence(tgt_lane)) {
              Log2DDS::LogDataV2(
                  "mlc_debug", absl::StrCat("update seq direction: ",
                                            lane_seq->GetSequenceDirection()));
              update_seq = lane_seq;
              break;
            }
          }
        }
      }
      if (update_seq) {
        std::vector<uint64_t> lane_set;
        for (const auto& lane : update_seq->lanes()) {
          if (lane) lane_set.emplace_back(lane->id());
        }
        st::mapping::LanePath lane_path(psmm.map_ptr(), lane_set, 0.0, 1.0);
        *preferred_lane_path = std::move(lane_path);
      }
      // if (seq) {
      //   preferred_lane_path->set_lane_seq(seq);
      //   Log2DDS::LogDataV2("mlc_debug", "update preferred_lane_path");
      //   std::string str = "";
      //   for (const auto& lane : seq->lanes()) {
      //     str += absl::StrCat(lane->id(), ",");
      //   }
      //   Log2DDS::LogDataV2("mlc_debug", "update preferred_lane_path: " +
      //   str);
      // }
    }
  }

  if (!prev_lp_before_lc.IsEmpty()) {
    std::string ids = "";
    for (const auto lane : prev_lp_before_lc.lane_ids()) {
      ids += absl::StrCat(lane, "; ");
    }
    Log2DDS::LogDataV2("mlc_debug", "lane_path before lc: " + ids);
  }

  double dist_buffer = Lerp(0.3, 50.0 * ad_byd::planning::Constants::KPH2MPS,
                            0.1, 70.0 * ad_byd::planning::Constants::KPH2MPS,
                            plan_start_point.v(), true);
  if (!preferred_lane_path->IsEmpty()) {
    if (!CheckIfAllowCancel(plan_start_point, vehicle_geometry, ego_pos,
                            preferred_lane_path, dist_buffer)) {
      *if_allow_cancel_lc = false;
      LOG_ERROR << "enter here: " << *if_allow_cancel_lc;
      Log2DDS::LogDataV2("mlc_debug", "already crossed half width!");
    }
  }

  // 2. handle command
  if (new_lc_cmd == DriverAction::LC_CMD_NONE ||
      new_lc_cmd == DriverAction::LC_CMD_STRAIGHT) {
    *lc_cmd_state = DriverAction::LC_CMD_NONE;
    return;
  }
  // if last lc reason is manual change and we already finish it
  // we do not respond another lane change command if five frames
  if ((new_lc_cmd == DriverAction::LC_CMD_LEFT ||
       new_lc_cmd == DriverAction::LC_CMD_RIGHT) &&
      last_lc_reason == LaneChangeReason::MANUAL_CHANGE &&
      ((last_manual_lc_time <= 5 &&
        eie_choice_type == ad_byd::planning::EIEChoiceType::CHOICE_NONE) ||
       (last_manual_lc_time <= 20 &&
        eie_choice_type != ad_byd::planning::EIEChoiceType::CHOICE_NONE))) {
    return;
  }

  // get left_seq and right_seq from candidate_lane_seqs
  if (candidate_lane_seqs.empty()) {
    Log2DDS::LogDataV2("mlc_debug", "get no candidate_lane_seqs!");
    return;
  }
  std::vector<ad_byd::planning::LaneConstPtr> left_seq, right_seq, cur_seq;
  for (int i = 0; i < candidate_lane_seqs.size(); ++i) {
    auto lane_seq = candidate_lane_seqs.at(i);
    if (lane_seq->GetSequenceDirection() ==
        ad_byd::planning::SequenceDirection::Left) {
      left_seq = lane_seq->lanes();
    } else if (lane_seq->GetSequenceDirection() ==
               ad_byd::planning::SequenceDirection::Right) {
      right_seq = lane_seq->lanes();
    } else if (lane_seq->GetSequenceDirection() ==
               ad_byd::planning::SequenceDirection::Cur) {
      cur_seq = lane_seq->lanes();
    }
  }

  if (new_lc_cmd == DriverAction::LC_CMD_CANCEL) {
    // store previous ALC state
    ALCState prev_alc_state = *alc_state;
    // already return to original lane, then drive fully based on ego lane
    if (prev_lc_state.stage() == LCS_NONE) {
      Log2DDS::LogDataV2("mlc_debug", "Cleared teleop lane change state!");
      *preferred_lane_path = mapping::LanePath();
      *alc_state = ALC_STANDBY_ENABLE;
      *lc_cmd_state = DriverAction::LC_CMD_NONE;
      return;
    } else if (prev_lc_state.stage() == LCS_RETURN ||
               prev_alc_state == ALC_RETURNING) {
      Log2DDS::LogDataV2("mlc_debug",
                         "previous returning teleop not completed!");
      return;
    }
    // // If previously performing lane change, try going back.
    // if (*alc_state == ALC_CROSSING_LANE ||
    // prev_lc_state.crossed_boundary()) {
    //   Log2DDS::LogDataV2("mlc_debug", "already crossed boundary!");
    //   return;
    // }

    // if previously performing manual lane change, check if allow cancel by the
    // position of conners
    Log2DDS::LogDataV2("mlc_debug", absl::StrCat("dist_buffer: ", dist_buffer));
    if (!preferred_lane_path->IsEmpty()) {
      if (!CheckIfAllowCancel(plan_start_point, vehicle_geometry, ego_pos,
                              preferred_lane_path, dist_buffer)) {
        Log2DDS::LogDataV2("mlc_debug", "already crossed half width!");
        return;
      }
    }

    if (prev_lp_before_lc.IsEmpty()) {
      Log2DDS::LogDataV2("mlc_debug", "rejected by route!");
      return;
    }
    // if prev_lp_before_lc is not the ego, then use the nearest to replace
    // const auto& nearest_lane = psmm.map_ptr()->GetNearestLane(
    //     ego_pos, ego_theta, /*LatDist =*/3.2, true, false);
    // if (nearest_lane) {
    //   Log2DDS::LogDataV2("mlc_debug",
    //                      absl::StrCat("nearest_lane: ", nearest_lane->id()));
    //   bool ego_lane_in_lp_before_lc = false;
    //   for (const auto& lane_id : prev_lp_before_lc.lane_ids()) {
    //     if (lane_id == nearest_lane->id()) {
    //       ego_lane_in_lp_before_lc = true;
    //       break;
    //     }
    //   }
    //   // lp_before_lc does not contain the ego lane, replace with ego lane
    //   // because of the following reasons: case 1: ego lane is the lane
    //   // change target, then we finish this lane change behavior since ego
    //   // car already enters the target. case 2: ego lane is actually the
    //   // lane before lc, then we return.
    //   if (ego_lane_in_lp_before_lc == false) {
    //     *alc_state = ALC_RETURNING;
    //     if (prev_alc_state == ALC_RETURNING) {
    //       *lc_cmd_state = prev_lc_state.lc_left()
    //                           ? DriverAction::LC_CMD_LEFT    // 2
    //                           : DriverAction::LC_CMD_RIGHT;  // 2
    //     } else {
    //       *lc_cmd_state = prev_lc_state.lc_left()
    //                           ? DriverAction::LC_CMD_RIGHT  // 3
    //                           : DriverAction::LC_CMD_LEFT;  // 2
    //     }
    //     bool nearest_change_error = false;
    //     ad_byd::planning::SLPoint ego_sl;
    //     nearest_lane->GetSLWithLimit(ego_pos, &ego_sl);

    //     if (*lc_cmd_state == DriverAction::LC_CMD_RIGHT) {
    //       nearest_change_error = (ego_sl.l < -0.5);
    //     } else if (*lc_cmd_state == DriverAction::LC_CMD_LEFT) {
    //       nearest_change_error = (ego_sl.l > 0.5);
    //     }

    //     if (!nearest_change_error) {
    //       st::mapping::LanePath lane_path(psmm.map_ptr(),
    //       {nearest_lane->id()},
    //                                       0.0, 1.0);
    //       *preferred_lane_path = std::move(lane_path);
    //       Log2DDS::LogDataV2("mlc_debug",
    //                          "Driving back to the lane path before lane "
    //                          "change with nearest!");
    //       return;
    //     }
    //   }
    // }

    // if previously performing auto lane change, check if allow cancel by the
    // position of conners
    if (!CheckIfCrossLine(plan_start_point, vehicle_geometry, ego_pos,
                          prev_lp_before_lc, dist_buffer)) {
      *alc_state = ALC_RETURNING;
      *lc_cmd_state = prev_lc_state.lc_left() ? DriverAction::LC_CMD_RIGHT  // 3
                                              : DriverAction::LC_CMD_LEFT;  // 2
      *preferred_lane_path = prev_lp_before_lc;
      Log2DDS::LogDataV2("mlc_debug",
                         "Driving back to the lane path before lane change!");
      *if_cancel_lc = true;
      Log2DDS::LogDataV2("mlc_debug",
                         absl::StrCat("if_cancel_lc: ", *if_cancel_lc));
      return;
    } else {
      std::vector<ad_byd::planning::LaneConstPtr> target_seq;
      target_seq = prev_lc_state.lc_left() ? left_seq : right_seq;
      if (target_seq.empty()) {
        Log2DDS::LogDataV2("mlc_debug", "No valid lane!");
        return;
      }
      std::vector<uint64_t> lane_set;
      for (const auto lane : target_seq) {
        if (lane) lane_set.emplace_back(lane->id());
      }
      st::mapping::LanePath lane_path(psmm.map_ptr(), lane_set, 0.0, 1.0);
      *preferred_lane_path = std::move(lane_path);
      *alc_state = ALC_ONGOING;
      *lc_cmd_state = prev_lc_state.lc_left() ? DriverAction::LC_CMD_LEFT  // 2
                                              : DriverAction::LC_CMD_RIGHT;  // 3
      Log2DDS::LogDataV2("mlc_debug",
                         "Continue lane change if already crossing the line");
      return;
    }
  }
  if (!preferred_lane_path->IsEmpty()) {
    if ((new_lc_cmd == DriverAction::LC_CMD_LEFT && left_seq.empty()) ||
        (new_lc_cmd == DriverAction::LC_CMD_RIGHT && right_seq.empty())) {
      *lc_unable_reason = st::planning::LcFeasibility::FEASIBILITY_LINE_TYPE;
    }
    Log2DDS::LogDataV2("mlc_debug", "previous teleop not completed!");
    return;
  }
  if (prev_lc_state.stage() != LCS_NONE) {
    Log2DDS::LogDataV2("mlc_debug", "currently performing lane change!");
    return;
  }

  if (!left_seq.empty() && new_lc_cmd == DriverAction::LC_CMD_LEFT) {
    std::vector<uint64_t> lane_set;
    for (const auto lane : left_seq) {
      if (lane) lane_set.emplace_back(lane->id());
    }
    st::mapping::LanePath lane_path(psmm.map_ptr(), lane_set, 0.0, 1.0);
    *preferred_lane_path = std::move(lane_path);
    *lc_cmd_state = new_lc_cmd;
    Log2DDS::LogDataV2("mlc_debug", "DriverAction::LC_CMD_LEFT");
  } else if (!right_seq.empty() && new_lc_cmd == DriverAction::LC_CMD_RIGHT) {
    std::vector<uint64_t> lane_set;
    for (const auto lane : right_seq) {
      if (lane) lane_set.emplace_back(lane->id());
    }
    st::mapping::LanePath lane_path(psmm.map_ptr(), lane_set, 0.0, 1.0);
    *preferred_lane_path = std::move(lane_path);
    *lc_cmd_state = new_lc_cmd;
    Log2DDS::LogDataV2("mlc_debug", "DriverAction::LC_CMD_RIGHT");
  } else if ((new_lc_cmd == DriverAction::LC_CMD_LEFT && left_seq.empty()) ||
             (new_lc_cmd == DriverAction::LC_CMD_RIGHT && right_seq.empty())) {
    std::vector<uint64_t> lane_set;
    for (const auto lane : cur_seq) {
      if (lane) lane_set.emplace_back(lane->id());
    }
    st::mapping::LanePath lane_path(psmm.map_ptr(), lane_set, 0.0, 1.0);
    *preferred_lane_path = std::move(lane_path);
    *lc_cmd_state = DriverAction::LC_CMD_STRAIGHT;
    *lc_unable_reason = st::planning::LcFeasibility::FEASIBILITY_LINE_TYPE;
    Log2DDS::LogDataV2("mlc_debug", "no valid lane, use current seq");
  } else {
    *lc_unable_reason = st::planning::LcFeasibility::FEASIBILITY_NO_LANE;
    Log2DDS::LogDataV2("lc_unable_reason", "feasiblity no line!");
    return;
  }

  // New teleop command incoming, set lane path and state.
  *alc_state = ALC_ONGOING;
  // *lc_cmd_state = new_lc_cmd;
  LOG_INFO << "Processed new teleop lane change command: "
           << DriverAction_LaneChangeCommand_Name(new_lc_cmd);
  if (preferred_lane_path && preferred_lane_path->lane_seq() &&
      preferred_lane_path->lane_seq()->IsValid()) {
    std::string mlc_target_lanes = "";
    for (const auto lane : preferred_lane_path->lane_seq()->lanes()) {
      if (lane) {
        mlc_target_lanes += absl::StrCat(lane->id(), ",");
      }
    }
    Log2DDS::LogDataV2("mlc_debug", "mlc_target_lanes:" + mlc_target_lanes);
  }
}
}  // namespace planning
}  // namespace st
