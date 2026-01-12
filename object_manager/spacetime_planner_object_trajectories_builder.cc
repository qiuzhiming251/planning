

#include "spacetime_planner_object_trajectories_builder.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/log_data.h"
#include "plan_common/timer.h"

//#include "lite/check.h"
#include <limits>

#include "plan_common/math/linear_interpolation.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/vec.h"
#include "planner_object.h"
#include "spacetime_planner_object_trajectories_filter.h"
#include "spacetime_planner_object_trajectories_finder.h"
//#include "planner/planner_manager/planner_defs.h"
//#include "plan_common/util/hmi_content_util.h"
//#include "object_manager/planner_object.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st {
namespace planning {
namespace {
SpacetimePlannerObjectTrajectories GetSpacetimePlannerObjectTrajectories(
    const ApolloTrajectoryPointProto* plan_start_point,
    absl::Span<const SpacetimeObjectTrajectory> candidate_trajs,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFinder>>& finders,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFilter>>& filters,
    const DrivePassage* drive_passage,
    const LaneChangeStateProto* lane_change_state, double start_offset,
    const PathSlBoundary* sl_boundary,
    const VehicleGeometryParamsProto* veh_geom,
    const double spacetime_planner_trajectory_horizon,
    const double trajectory_time_step, const double default_half_lane_width,
    std::unordered_map<std::string, double>& truncated_traj_map,
    const Box2d& av_box, const std::vector<NudgeInfo>& nudgeinfos);

absl::StatusOr<double> Calculate_Obstacles_Predicted_Trajectory_Length(
    const DrivePassage* drive_passage, const PathSlBoundary* sl_boundary,
    const LaneChangeStateProto* lane_change_state,
    const VehicleGeometryParamsProto* veh_geom,
    const ApolloTrajectoryPointProto* plan_start_point,
    const SpacetimeObjectTrajectory& traj, bool special_avoidance_conditions,
    const double spacetime_planner_trajectory_horizon,
    const double trajectory_time_step, const double default_half_lane_width);

absl::StatusOr<double> truncate_back_traj_horizon(
    const DrivePassage* drive_passage, const Box2d& av_box,
    const SpacetimeObjectTrajectory& traj,
    const double spacetime_planner_trajectory_horizon,
    const SpacetimePlannerObjectTrajectoryReason::Type& selected_reason,
    const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs,
    std::unordered_map<std::string, double>& truncated_traj_map,
    const NudgeObjectInfo* nudge_object_info, bool special_condition,
    const std::vector<NudgeInfo>& nudgeinfos);

bool IsVru(const SpacetimeObjectTrajectory& traj) {
  return traj.object_type() == ObjectType::OT_MOTORCYCLIST ||
         traj.object_type() == ObjectType::OT_CYCLIST ||
         traj.object_type() == ObjectType::OT_TRICYCLIST ||
         traj.object_type() == ObjectType::OT_PEDESTRIAN;
}

absl::StatusOr<SpacetimeObjectTrajectory> CreateFakeTrajectoryByRelatedObstacle(
    const DrivePassage* drive_passage,
    const LaneChangeStateProto* lane_change_state,
    const VehicleGeometryParamsProto* veh_geom,
    const ApolloTrajectoryPointProto* plan_start_point,
    const SpacetimeObjectTrajectory& traj, double start_offset,
    const double spacetime_planner_trajectory_horizon);

SpacetimePlannerObjectTrajectories GetSpacetimePlannerObjectTrajectories(
    const SpacetimePlannerObjectTrajectoriesBuilderInput& input,
    absl::Span<const SpacetimeObjectTrajectory> candidate_trajs,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFinder>>& finders,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFilter>>& filters,
    const double spacetime_planner_trajectory_horizon,
    const double trajectory_time_step, const double default_half_lane_width,
    std::unordered_map<std::string, double>& truncated_traj_map,
    const Box2d& av_box, const std::vector<NudgeInfo>& nudgeinfos) {
  const ApolloTrajectoryPointProto* plan_start_point = input.plan_start_point;
  const DrivePassage* drive_passage = input.passage;
  const LaneChangeStateProto* lane_change_state = input.lane_change_state;
  double start_offset = std::clamp(input.st_planner_start_offset, -0.5, 0.5);
  const PathSlBoundary* sl_boundary = input.sl_boundary;
  const VehicleGeometryParamsProto* veh_geom = input.veh_geom;
  const std::string& prefix = Log2DDS::TaskPrefix(input.plan_id);

  SpacetimePlannerObjectTrajectories res;
  res.trajectories.reserve(candidate_trajs.size());
  res.trajectory_infos.reserve(candidate_trajs.size());
  res.st_start_offset = start_offset;

  const auto lane_id = drive_passage->lane_path().front().lane_id();
  const auto lane_info = input.psmm->FindCurveLaneByIdOrNull(lane_id);
  // is merge or split
  bool is_split = false, is_merge = false;
  // to recognize split earlier
  double dist_to_split = 0.0;
  const double kMaxheadway_to_split = 1.5;
  double maxdist_to_split =
      std::fmax(input.plan_start_point->v() * kMaxheadway_to_split, 3.0);
  if (drive_passage->lane_seq_info() != nullptr) {
    dist_to_split =
        drive_passage->lane_seq_info()->dist_to_nearest_split.second;
  }
  if (dist_to_split > 0.0 && dist_to_split < maxdist_to_split) {
    is_split = true;
  }

  if (lane_info && lane_info->pre_lane_ids().size() == 1) {
    const auto pre_lane_info =
        input.psmm->FindCurveLaneByIdOrNull(lane_info->pre_lane_ids()[0]);
    if (pre_lane_info && pre_lane_info->next_lane_ids().size() >= 2) {
      is_split = true;
    }
  }

  if (lane_info && lane_info->next_lane_ids().size() == 1) {
    const auto next_lane_info =
        input.psmm->FindCurveLaneByIdOrNull(lane_info->next_lane_ids()[0]);
    if (next_lane_info && next_lane_info->pre_lane_ids().size() >= 2) {
      is_merge = true;
    }
  }

  const bool special_avoidance_conditions =
      (lane_info && !(lane_info->junction_id() == 0)) || is_split || is_merge;

  // Pick trajectories for spacetime planner.
  std::vector<int> truncated_index;
  for (const auto& traj : candidate_trajs) {
    // auto fake_traj = CreateFakeTrajectoryByRelatedObstacle(
    //     drive_passage, lane_change_state, veh_geom, plan_start_point,
    //     traj_raw, start_offset, spacetime_planner_trajectory_horizon);
    // const auto& traj = fake_traj.ok() ? fake_traj.value() : traj_raw;
    for (const auto& finder : finders) {
      const auto selected_reason = finder->Find(traj);
      if (selected_reason != SpacetimePlannerObjectTrajectoryReason::NONE) {
        if (input.force_no_nudge &&
            selected_reason !=
                SpacetimePlannerObjectTrajectoryReason::STATIONARY) {
          continue;
        }
        bool is_filtered = false;
        for (const auto& filter : filters) {
          if (filter->Filter(traj)) {
            is_filtered = true;
            break;
          }
        }
        if (is_filtered) {
          break;
        }

        double st_planner_traj_horizon = spacetime_planner_trajectory_horizon;
        auto obs_traj_horizon = Calculate_Obstacles_Predicted_Trajectory_Length(
            drive_passage, sl_boundary, lane_change_state, veh_geom,
            plan_start_point, traj, special_avoidance_conditions,
            spacetime_planner_trajectory_horizon, trajectory_time_step,
            default_half_lane_width);
        bool truncated =
            obs_traj_horizon.ok() &&
            obs_traj_horizon.value() < st_planner_traj_horizon - 0.1;
        if (obs_traj_horizon.ok()) {
          st_planner_traj_horizon =
              std::min(st_planner_traj_horizon, obs_traj_horizon.value());
        } else {
          //   Log2DDS::LogDataV0(
          //       "st_planner_traj_horizon",
          //       absl::StrCat(
          //           traj.traj_id(),
          //           "-calculate_obstacles_predicted_trajectory_length_error_obs-",
          //           st_planner_traj_horizon));
        }
        if (truncated) {
          Log2DDS::LogDataV2(
              absl::StrCat(prefix, "st_planner_traj_horizon"),
              absl::StrCat(traj.traj_id(), " truncated_traj_horizon_cut: ",
                           st_planner_traj_horizon));
        }

        // truncate the obj traj of the back obj
        bool special_condition =
            (lane_info && !(lane_info->junction_id() == 0)) || is_split ||
            (is_merge && !IsVru(traj));
        auto truncate_traj_length = truncate_back_traj_horizon(
            drive_passage, av_box, traj, spacetime_planner_trajectory_horizon,
            selected_reason, input.prev_st_trajs, truncated_traj_map,
            input.nudge_object_info, special_condition, nudgeinfos);
        if (truncate_traj_length.ok() &&
            st_planner_traj_horizon > truncate_traj_length.value() + 0.1) {
          truncated = true;
          st_planner_traj_horizon =
              std::min(st_planner_traj_horizon, truncate_traj_length.value());
          Log2DDS::LogDataV2(
              absl::StrCat(prefix, "st_planner_traj_horizon"),
              absl::StrCat(traj.traj_id(), " truncated_traj_horizon_back: ",
                           st_planner_traj_horizon));
        }

        if (!traj.is_stationary() &&
            (traj.object_type() == ObjectType::OT_VEHICLE ||
             traj.object_type() == ObjectType::OT_LARGE_VEHICLE)) {
          st_planner_traj_horizon =
              std::min(st_planner_traj_horizon,
                       FLAGS_planner_pred_traj_horizon_for_dp_and_ddp);
        }

        ASSIGN_OR_CONTINUE(
            auto truncated_traj,
            traj.CreateTruncatedCopy(start_offset, st_planner_traj_horizon));

        res.trajectories.push_back(std::move(truncated_traj));
        if (truncated && res.trajectories.size() > 0) {
          truncated_index.push_back(res.trajectories.size() - 1);
        }

        res.trajectory_infos.push_back(
            {.traj_index = traj.traj_index(),
             .object_id = traj.planner_object().is_sim_agent()
                              ? traj.planner_object().base_id()
                              : traj.planner_object().id(),
             .reason = selected_reason});
        res.trajectory_ids.insert(std::string(traj.traj_id()));
        break;
      }
    }
  }
  // update truncate traj map
  std::vector<std::string> ids_to_erase;
  for (const auto& ele : truncated_traj_map) {
    auto ite = std::find_if(
        res.trajectory_infos.begin(), res.trajectory_infos.end(),
        [ele](const SpacetimePlannerObjectTrajectories::TrajectoryInfo&
                  traj_info) { return ele.first == traj_info.object_id; });
    if (ite == res.trajectory_infos.end()) {
      ids_to_erase.emplace_back(ele.first);
    }
  }
  for (const auto& id : ids_to_erase) {
    truncated_traj_map.erase(id);
  }

  for (const auto& index : truncated_index) {
    std::vector<double> xs, ys;
    const auto& traj = res.trajectories[index];
    xs.reserve(traj.states().size());
    ys.reserve(traj.states().size());
    for (const auto& state : traj.states()) {
      xs.emplace_back(state.traj_point->pos().x());
      ys.emplace_back(state.traj_point->pos().y());
    }
    Log2DDS::LogLineV2(absl::StrCat(prefix, "truncated_traj", traj.traj_id()),
                       Log2DDS::kRed, {}, xs, ys, 2.0);
  }

  return res;
}

absl::StatusOr<SpacetimeObjectTrajectory> CreateFakeTrajectoryByRelatedObstacle(
    const DrivePassage* drive_passage,
    const LaneChangeStateProto* lane_change_state,
    const VehicleGeometryParamsProto* veh_geom,
    const ApolloTrajectoryPointProto* plan_start_point,
    const SpacetimeObjectTrajectory& traj, double start_offset,
    const double spacetime_planner_trajectory_horizon) {
  if (traj.is_stationary()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("current lane change stage is:",
                        LaneChangeStage_Name(lane_change_state->stage()),
                        "traj.is_stationary() is:", traj.is_stationary()));
  }
  std::string_view object_id = traj.planner_object().id();

  constexpr double kLateralMinSafeBufferInFakeStatic = 0.3;   // m
  constexpr double kLateralMaxSpeedBufferInFakeStatic = 0.6;  // m/s
  const auto& curr_path_point = plan_start_point->path_point();
  const auto& av_start_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *veh_geom);
  const auto av_start_frenet_box =
      drive_passage->QueryFrenetBoxAt(av_start_box);
  if (!av_start_frenet_box.ok()) return av_start_frenet_box.status();

  const bool obj_car_type = traj.object_type() == ObjectType::OT_VEHICLE ||
                            traj.object_type() == ObjectType::OT_LARGE_VEHICLE;
  const auto object_start_frenet_box =
      drive_passage->QueryFrenetBoxAt(traj.bounding_box());
  if (!object_start_frenet_box.ok()) return object_start_frenet_box.status();

  if (std::fabs(traj.pose().v()) < kLateralMaxSpeedBufferInFakeStatic &&
      (obj_car_type ||
       (!(object_start_frenet_box->l_min >
              av_start_frenet_box->l_max + kLateralMinSafeBufferInFakeStatic ||
          av_start_frenet_box->l_min >
              av_start_frenet_box->l_max + kLateralMinSafeBufferInFakeStatic) &&
        traj.object_type() == ObjectType::OT_CYCLIST))) {
    Log2DDS::LogDataV2(
        "st_planner_traj_horizon",
        absl::StrCat(std::string(object_id), "-fake_to_static_obs"));
    if (traj.states().empty()) {
      return absl::FailedPreconditionError(absl::StrCat(
          "The trajectory ", std::string(object_id), " has no states."));
    }
    std::vector<prediction::PredictionObjectState> truncated_states;
    truncated_states.reserve(traj.states().size());

    constexpr double kEps = 1e-6;
    const double start_t =
        traj.states()[0].traj_point->t() + start_offset - kEps;
    for (int i = 0, n = traj.states().size(); i < n; ++i) {
      if (traj.states()[i].traj_point->t() - start_t >
          spacetime_planner_trajectory_horizon) {
        break;
      }
      if (traj.states()[i].traj_point->t() >= start_t) {
        truncated_states.push_back(traj.states()[i]);
      }
    }
    if (truncated_states.empty()) {
      return absl::NotFoundError(
          absl::StrCat("No trajectory for fake static trajectory:object_id",
                       std::string(object_id)));
    }
    PlannerObject planner_object = traj.planner_object();
    planner_object.set_stationary(true);
    return SpacetimeObjectTrajectory(
        planner_object, std::move(truncated_states), traj.traj_index(),
        traj.required_lateral_gap());
  }
  return absl::InvalidArgumentError(
      absl::StrCat("The current obstacle does not meet the fake static "
                   "obstacle conditions,object_id",
                   std::string(object_id)));
}

absl::StatusOr<double> truncate_back_traj_horizon(
    const DrivePassage* drive_passage, const Box2d& av_box,
    const SpacetimeObjectTrajectory& traj,
    const double spacetime_planner_trajectory_horizon,
    const SpacetimePlannerObjectTrajectoryReason::Type& selected_reason,
    const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs,
    std::unordered_map<std::string, double>& truncated_traj_map,
    const NudgeObjectInfo* nudge_object_info, bool special_condition,
    const std::vector<NudgeInfo>& nudgeinfos) {
  absl::flat_hash_set<std::string> prev_st_planner_obj_id;
  for (const auto& st_traj_proto : prev_st_trajs->trajectory()) {
    prev_st_planner_obj_id.insert(st_traj_proto.id());
  }
  const bool prev_st_planner_obj = prev_st_planner_obj_id.contains(
      traj.planner_object().is_sim_agent() ? traj.planner_object().base_id()
                                           : traj.planner_object().id());
  double tan_objspeed = traj.planner_object().velocity().Dot(av_box.tangent());

  double truncate_traj_horizon = spacetime_planner_trajectory_horizon;
  bool is_front = false;
  bool is_side = false;
  const auto& av_start_frenet_box =
      drive_passage->QueryFrenetBoxAt(av_box, false);
  if (!av_start_frenet_box.ok()) return av_start_frenet_box.status();
  const auto& object_start_box =
      drive_passage->QueryFrenetBoxAt(traj.bounding_box(), false);
  if (!object_start_box.ok()) return object_start_box.status();
  const double lon_buffer = 2.0;
  if (object_start_box->s_min > av_start_frenet_box->s_max + lon_buffer) {
    is_front = true;
  }
  if (object_start_box->s_min < av_start_frenet_box->s_max) {
    is_side = true;
  }
  // ignore back truncate when in nudge side
  bool ignore_back_trunc = false;
  const double lat_buffer = 0.2;
  const double nudge_buffer_ttc = 0.5;
  for (const auto& nudgeinfo : nudgeinfos) {
    if (nudgeinfo.nudge_back_ttc > nudge_buffer_ttc) {
      if (nudgeinfo.nudge_direction == 1 &&
          object_start_box->l_min > av_start_frenet_box->l_max - lat_buffer) {
        ignore_back_trunc = true;
      } else if (nudgeinfo.nudge_direction == -1 &&
                 object_start_box->l_max <
                     av_start_frenet_box->l_min + lat_buffer) {
        ignore_back_trunc = true;
      }
    }
  }
  const double release_step = IsVru(traj) ? 0.1 : 0.2;
  const double release_init = IsVru(traj) ? 0.3 : 0.5;

  if (selected_reason == SpacetimePlannerObjectTrajectoryReason::SIDE &&
      truncated_traj_map.find(traj.planner_object().id()) ==
          truncated_traj_map.end() &&
      is_side && !prev_st_planner_obj && tan_objspeed > 0 &&
      !ignore_back_trunc && !special_condition) {
    // add
    truncated_traj_map[traj.planner_object().id()] = release_init;
    truncate_traj_horizon = truncated_traj_map[traj.planner_object().id()];
  } else if ((selected_reason != SpacetimePlannerObjectTrajectoryReason::SIDE ||
              is_front || special_condition || ignore_back_trunc) &&
             truncated_traj_map.find(traj.planner_object().id()) !=
                 truncated_traj_map.end()) {
    // erase
    truncated_traj_map.erase(traj.planner_object().id());
  } else if (selected_reason == SpacetimePlannerObjectTrajectoryReason::SIDE &&
             truncated_traj_map.find(traj.planner_object().id()) !=
                 truncated_traj_map.end() &&
             !is_front) {
    // accumulate
    truncated_traj_map[traj.planner_object().id()] += release_step;
    truncated_traj_map[traj.planner_object().id()] =
        std::min(truncated_traj_map[traj.planner_object().id()],
                 spacetime_planner_trajectory_horizon);
    truncate_traj_horizon = truncated_traj_map[traj.planner_object().id()];
  }
  return truncate_traj_horizon;
}

absl::StatusOr<double> Calculate_Obstacles_Predicted_Trajectory_Length(
    const DrivePassage* drive_passage, const PathSlBoundary* sl_boundary,
    const LaneChangeStateProto* lane_change_state,
    const VehicleGeometryParamsProto* veh_geom,
    const ApolloTrajectoryPointProto* plan_start_point,
    const SpacetimeObjectTrajectory& traj, bool special_avoidance_conditions,
    const double spacetime_planner_trajectory_horizon,
    const double trajectory_time_step, const double default_half_lane_width) {
  double st_planner_traj_horizon = spacetime_planner_trajectory_horizon;
  if (traj.is_stationary() || (lane_change_state->stage() != LCS_NONE &&
                               lane_change_state->stage() != LCS_EXECUTING)) {
    return st_planner_traj_horizon;
  }
  // std::string_view object_id = traj.planner_object().id();
  const auto& traj_start_point = *traj.states().front().traj_point;
  const auto frenet_start_point =
      drive_passage->QueryUnboundedFrenetCoordinateAt(traj_start_point.pos());
  if (!frenet_start_point.ok()) return frenet_start_point.status();

  const auto lane_theta_at_pose =
      drive_passage->QueryTangentAngleAtS(frenet_start_point->s);
  if (!lane_theta_at_pose.ok()) return lane_theta_at_pose.status();

  constexpr double kSampleStepAlongS = 1.0;                        // m
  constexpr double kLateralMinSafeBufferToCurbDistance = 1.2;      // m
  constexpr double kLateralMinSafeReverseTrajectoryHorizon = 0.7;  // s
  const double kLateralMinSafeSameDirectionTrajectoryHorizon =
      special_avoidance_conditions ? 0.6 : 0.5;                // s
  const double kLateralMinReserveSafeTrajectoryHorizon = 0.8;  // s
  constexpr bool use_out_boundary = false;
  constexpr double kLateralTruncatingObstacleMinDistance = 0.6;  // m
  constexpr double kLateralTruncatingObstacleMinDistance_lowspeed = 0.3;  // m

  // computer obs position boundary
  auto computer_obs_boundary = [sl_boundary, default_half_lane_width](
                                   const FrenetBox& object_frenet_box,
                                   double& left_boundary,
                                   double& right_boundary) {
    left_boundary = std::numeric_limits<double>::infinity();
    right_boundary = -std::numeric_limits<double>::infinity();
    for (double sample_s = object_frenet_box.s_min;
         sample_s <= object_frenet_box.s_max; sample_s += kSampleStepAlongS) {
      const auto [right_l, left_l] =
          use_out_boundary ? sl_boundary->QueryBoundaryL(sample_s)
                           : sl_boundary->QueryTargetBoundaryL(sample_s);
      left_boundary =
          std::min({left_boundary, left_l, default_half_lane_width});
      right_boundary =
          std::max({right_boundary, right_l, -default_half_lane_width});
    }
  };

  // computer ego start point frenet box
  const auto& curr_path_point = plan_start_point->path_point();
  const Vec2d& av_tangent = Vec2d::FastUnitFromAngle(curr_path_point.theta());
  const auto& av_start_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *veh_geom);
  const auto& av_start_frenet_box =
      drive_passage->QueryFrenetBoxAt(av_start_box, false);
  if (!av_start_frenet_box.ok()) return av_start_frenet_box.status();
  const auto& object_start_box =
      drive_passage->QueryFrenetBoxAt(traj.bounding_box(), false);
  if (!object_start_box.ok()) return object_start_box.status();

  double left_curb_boundary = std::numeric_limits<double>::infinity();
  double right_curb_boundary = -std::numeric_limits<double>::infinity();
  for (double sample_s = av_start_frenet_box->s_max;
       sample_s <= av_start_frenet_box->s_max +
                       std::clamp(0.5 * plan_start_point->v(), 10.0, 30.0);
       sample_s += kSampleStepAlongS) {
    const auto curb_l = drive_passage->QueryCurbOffsetAtS(sample_s);
    if (!curb_l.ok()) continue;
    left_curb_boundary = std::min(left_curb_boundary, curb_l->second);
    right_curb_boundary = std::max(right_curb_boundary, curb_l->first);
  }

  const double kLateralMinSafeBufferInReverseSameCar =
      av_start_frenet_box->center_l() > object_start_box->center_l()
          ? (left_curb_boundary - av_start_frenet_box->l_max <
                     kLateralMinSafeBufferToCurbDistance
                 ? 1.1
                 : (special_avoidance_conditions ? 0.1 : 0.5))
          : (av_start_frenet_box->l_min - right_curb_boundary <
                     kLateralMinSafeBufferToCurbDistance
                 ? 1.1
                 : (special_avoidance_conditions ? 0.1 : 0.5));
  const double kkLateralMinSafeBufferLc = 0.6;
  double obs_frenet_l_min = std::numeric_limits<double>::infinity();
  double obs_frenet_l_max = -std::numeric_limits<double>::infinity();
  for (auto it = traj.states().begin(); it != traj.states().end(); ++it) {
    const auto& object_frenet_box = drive_passage->QueryFrenetBoxAt(it->box);
    if (!object_frenet_box.ok()) continue;
    obs_frenet_l_min = std::min(obs_frenet_l_min, object_frenet_box->l_min);
    obs_frenet_l_max = std::max(obs_frenet_l_max, object_frenet_box->l_max);
  }

  double LateralTruncatingObstacleMinDistance =
      plan_start_point->v() > 2.8
          ? kLateralTruncatingObstacleMinDistance
          : kLateralTruncatingObstacleMinDistance_lowspeed;
  const bool cut_in_intention =
      av_start_frenet_box->center_l() > object_start_box->center_l()
          ? obs_frenet_l_max - object_start_box->l_max >
                LateralTruncatingObstacleMinDistance
          : object_start_box->l_min - obs_frenet_l_min >
                LateralTruncatingObstacleMinDistance;
  const double behind_buffer = 1.0;
  const bool is_behind =
      object_start_box->s_max < av_start_frenet_box->s_max + behind_buffer;
  const bool is_vru = IsVru(traj);
  // reverse obstacle
  if (frenet_start_point->s - plan_start_point->path_point().s() > 0.0 &&
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj_start_point.theta())) < M_PI / 6 &&
      cut_in_intention) {
    // if (lane_change_state->stage() != LCS_NONE) {
    //   return st_planner_traj_horizon;
    // }
    for (auto it = traj.states().begin(); it != traj.states().end(); ++it) {
      if (it == traj.states().begin()) continue;
      const double time_count =
          std::distance(traj.states().begin(), it) * trajectory_time_step;
      const auto& object_frenet_box =
          drive_passage->QueryFrenetBoxAt(it->box, false);
      if (!object_frenet_box.ok()) {
        // Log2DDS::LogDataV0(
        //     "st_planner_traj_horizon",
        //     absl::StrCat(
        //         object_id, "-reverse_traj_invalid_obs-",
        //         std::clamp(time_count,
        //         kLateralMinSafeReverseTrajectoryHorizon,
        //                    spacetime_planner_trajectory_horizon)));
        return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                          spacetime_planner_trajectory_horizon);
      }
      double left_boundary = std::numeric_limits<double>::infinity();
      double right_boundary = -std::numeric_limits<double>::infinity();
      computer_obs_boundary(object_frenet_box.value(), left_boundary,
                            right_boundary);
      double ego_latmax = av_start_frenet_box->center_l() + 0.9;
      double ego_latmin = av_start_frenet_box->center_l() - 0.9;
      if (lane_change_state->stage() == LCS_NONE) {
        if (object_start_box->center_l() >
                av_start_frenet_box->center_l() + 0.5 &&
            std::max(std::min(right_boundary, av_start_frenet_box->l_min) +
                         veh_geom->width(),
                     std::min(av_start_frenet_box->l_max, ego_latmax)) +
                    kLateralMinSafeBufferInReverseSameCar >
                object_frenet_box.value().l_min) {
          // Log2DDS::LogDataV0(
          //     "st_planner_traj_horizon",
          //     absl::StrCat(
          //         object_id, "-reverse_left_obs-",
          //         std::clamp(time_count,
          //         kLateralMinSafeReverseTrajectoryHorizon,
          //                    spacetime_planner_trajectory_horizon)));
          return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                            spacetime_planner_trajectory_horizon);
        } else if (av_start_frenet_box->center_l() >
                       object_start_box->center_l() + 0.5 &&
                   std::min(
                       std::max(left_boundary, av_start_frenet_box->l_max) -
                           veh_geom->width(),
                       std::max(av_start_frenet_box->l_min, ego_latmin)) -
                           kLateralMinSafeBufferInReverseSameCar <
                       object_frenet_box.value().l_max) {
          // Log2DDS::LogDataV0(
          //     "st_planner_traj_horizon",
          //     absl::StrCat(
          //         object_id, "-reverse_right_obs-",
          //         std::clamp(time_count,
          //         kLateralMinSafeReverseTrajectoryHorizon,
          //                    spacetime_planner_trajectory_horizon)));
          return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                            spacetime_planner_trajectory_horizon);
        }
      } else if (lane_change_state->stage() == LCS_EXECUTING &&
                 object_start_box->center_l() > 1.0) {
        if (lane_change_state->lc_left() &&
            std::max(av_start_frenet_box->l_max,
                     left_boundary - kkLateralMinSafeBufferLc) +
                    kLateralMinSafeBufferInReverseSameCar >
                object_frenet_box.value().l_min) {
          return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                            spacetime_planner_trajectory_horizon);
        } else if (!lane_change_state->lc_left() &&
                   std::min(right_boundary + 2 * default_half_lane_width -
                                kkLateralMinSafeBufferLc,
                            av_start_frenet_box->l_max) +
                           kLateralMinSafeBufferInReverseSameCar >
                       object_frenet_box.value().l_min) {
          return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                            spacetime_planner_trajectory_horizon);
        }
      } else if (lane_change_state->stage() == LCS_EXECUTING &&
                 object_start_box->center_l() < -1.0) {
        if (lane_change_state->lc_left() &&
            std::max(left_boundary - 2 * default_half_lane_width +
                         kkLateralMinSafeBufferLc,
                     av_start_frenet_box->l_min) -
                    kLateralMinSafeBufferInReverseSameCar <
                object_frenet_box.value().l_max) {
          return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                            spacetime_planner_trajectory_horizon);
        } else if (!lane_change_state->lc_left() &&
                   std::min(right_boundary + kkLateralMinSafeBufferLc,
                            av_start_frenet_box->l_min) -
                           kLateralMinSafeBufferInReverseSameCar <
                       object_frenet_box.value().l_max) {
          return std::clamp(time_count, kLateralMinSafeReverseTrajectoryHorizon,
                            spacetime_planner_trajectory_horizon);
        }
      }
    }
  }

  // same direction car filter
  double safety_time = 0.0;
  bool find_safety_time = false;
  double left_boundary_init = std::numeric_limits<double>::infinity();
  double right_boundary_init = -std::numeric_limits<double>::infinity();
  computer_obs_boundary(object_start_box.value(), left_boundary_init,
                        right_boundary_init);
  bool is_inlane_vru =
      object_start_box->center_l() > right_boundary_init - 0.1 &&
      object_start_box->center_l() < left_boundary_init + 0.1 && is_vru &&
      object_start_box->s_min > av_start_frenet_box->s_max + 5.0;
  double LateralMinSafeBufferInReverseSameCar =
      is_behind
          ? is_vru
                ? std::max(std::min(kLateralMinSafeBufferInReverseSameCar + 0.2,
                                    0.9),
                           0.5)
                : std::max(std::min(kLateralMinSafeBufferInReverseSameCar + 0.1,
                                    0.9),
                           0.5)
      : is_inlane_vru && kLateralMinSafeBufferInReverseSameCar > 0.1 &&
              kLateralMinSafeBufferInReverseSameCar < 0.9
          ? kLateralMinSafeBufferInReverseSameCar - 0.5
          : kLateralMinSafeBufferInReverseSameCar;
  if (cut_in_intention &&
      frenet_start_point->s - plan_start_point->path_point().s() > -30.0 &&
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj_start_point.theta())) > M_PI / 6 - 1e-3) {
    double time_cross = 0.0;
    for (auto it = traj.states().begin(); it != traj.states().end(); ++it) {
      const double time_count =
          std::distance(traj.states().begin(), it) * trajectory_time_step;
      const auto& object_frenet_box =
          drive_passage->QueryFrenetBoxAt(it->box, false);
      if (!object_frenet_box.ok()) {
        // Log2DDS::LogDataV0(
        //     "st_planner_traj_horizon",
        //     absl::StrCat(
        //         object_id, "-same_direction_traj_invalid_obs-",
        //         std::clamp(time_count,
        //                    kLateralMinSafeSameDirectionTrajectoryHorizon,
        //                    spacetime_planner_trajectory_horizon)));
        return std::clamp(time_count,
                          kLateralMinSafeSameDirectionTrajectoryHorizon,
                          spacetime_planner_trajectory_horizon);
      }
      // check trajectory lat
      double left_boundary = std::numeric_limits<double>::infinity();
      double right_boundary = -std::numeric_limits<double>::infinity();
      computer_obs_boundary(object_frenet_box.value(), left_boundary,
                            right_boundary);
      const double ego_lat_max =
          std::clamp(0.5 * 0.5 * Sqr(time_count), 0.1, 0.3);
      if (!find_safety_time) {
        if (lane_change_state->stage() == LCS_NONE) {
          if (object_start_box->center_l() >
                  av_start_frenet_box->center_l() + 0.5 &&
              std::max(std::min(right_boundary, av_start_frenet_box->l_min) +
                           veh_geom->width(),
                       av_start_frenet_box->l_max - ego_lat_max) +
                      LateralMinSafeBufferInReverseSameCar >
                  object_frenet_box.value().l_min) {
            find_safety_time = true;
            safety_time = time_count;
          } else if (av_start_frenet_box->center_l() >
                         object_start_box->center_l() + 0.5 &&
                     std::min(
                         std::max(left_boundary, av_start_frenet_box->l_max) -
                             veh_geom->width(),
                         av_start_frenet_box->l_min + ego_lat_max) -
                             LateralMinSafeBufferInReverseSameCar <
                         object_frenet_box.value().l_max) {
            find_safety_time = true;
            safety_time = time_count;
          }
        } else if (lane_change_state->stage() == LCS_EXECUTING &&
                   object_start_box->center_l() > 1.0) {
          if (lane_change_state->lc_left() &&
              std::max(av_start_frenet_box->l_max,
                       left_boundary - kkLateralMinSafeBufferLc) +
                      kLateralMinSafeBufferInReverseSameCar >
                  object_frenet_box.value().l_min) {
            find_safety_time = true;
            safety_time = time_count;
          } else if (!lane_change_state->lc_left() &&
                     std::min(right_boundary + 2 * default_half_lane_width -
                                  kkLateralMinSafeBufferLc,
                              av_start_frenet_box->l_max) +
                             kLateralMinSafeBufferInReverseSameCar >
                         object_frenet_box.value().l_min) {
            find_safety_time = true;
            safety_time = time_count;
          }
        } else if (lane_change_state->stage() == LCS_EXECUTING &&
                   object_start_box->center_l() < -1.0) {
          if (lane_change_state->lc_left() &&
              std::max(left_boundary - 2 * default_half_lane_width +
                           kkLateralMinSafeBufferLc,
                       av_start_frenet_box->l_min) -
                      kLateralMinSafeBufferInReverseSameCar <
                  object_frenet_box.value().l_max) {
            find_safety_time = true;
            safety_time = time_count;
          } else if (!lane_change_state->lc_left() &&
                     std::min(right_boundary + kkLateralMinSafeBufferLc,
                              av_start_frenet_box->l_min) -
                             kLateralMinSafeBufferInReverseSameCar <
                         object_frenet_box.value().l_max) {
            find_safety_time = true;
            safety_time = time_count;
          }
        }
      }

      if (find_safety_time && lane_change_state->stage() == LCS_EXECUTING) {
        break;
      } else if (lane_change_state->stage() == LCS_EXECUTING) {
        continue;
      }
      const auto& av_time_box =
          ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()) +
                           av_tangent * (time_count * plan_start_point->v()),
                       curr_path_point.theta(), *veh_geom);
      const auto& av_frenet_box =
          drive_passage->QueryFrenetBoxAt(av_time_box, false);
      if (!av_frenet_box.ok()) return av_frenet_box.status();
      // if the traj cross
      bool if_cross =
          object_frenet_box.value().l_max * object_start_box->l_max < 0 ||
          object_frenet_box.value().l_min * object_start_box->l_min < 0 ||
          object_frenet_box.value().center_l() * object_start_box->center_l() <
              0;
      // check lon overlap
      if (object_frenet_box.value().s_max < av_frenet_box.value().s_min ||
          object_frenet_box.value().s_min > av_frenet_box.value().s_max) {
        // lon not overlap continue
        if (if_cross && time_cross < 0.1) {
          time_cross = time_count;
        }
        continue;
      }

      double time_temp = if_cross
                             ? std::min(std::max(time_cross, 0.0), time_count)
                             : time_count;

      if (object_start_box->center_l() >
              av_start_frenet_box->center_l() + 0.5 &&
          std::max(std::min(right_boundary, av_start_frenet_box->l_min) +
                       veh_geom->width(),
                   av_start_frenet_box->l_max - ego_lat_max) +
                  LateralMinSafeBufferInReverseSameCar >
              object_frenet_box.value().l_min) {
        return std::clamp(time_temp,
                          kLateralMinSafeSameDirectionTrajectoryHorizon,
                          spacetime_planner_trajectory_horizon);
      } else if (av_start_frenet_box->center_l() >
                     object_start_box->center_l() + 0.5 &&
                 std::min(std::max(left_boundary, av_start_frenet_box->l_max) -
                              veh_geom->width(),
                          av_start_frenet_box->l_min + ego_lat_max) -
                         LateralMinSafeBufferInReverseSameCar <
                     object_frenet_box.value().l_max) {
        return std::clamp(time_temp,
                          kLateralMinSafeSameDirectionTrajectoryHorizon,
                          spacetime_planner_trajectory_horizon);
      }
    }
  }

  if (find_safety_time) {
    // Log2DDS::LogDataV0(
    //     "st_planner_traj_horizon",
    //     absl::StrCat(
    //         object_id, "-mini_safety_",
    //         av_start_frenet_box->center_l() > object_start_box->center_l() +
    //         0.5
    //             ? "right"
    //             : "left",
    //         "_time_obs-",
    //         std::clamp(safety_time, kLateralMinReserveSafeTrajectoryHorizon,
    //                    spacetime_planner_trajectory_horizon)));
    return std::clamp(safety_time, kLateralMinReserveSafeTrajectoryHorizon,
                      spacetime_planner_trajectory_horizon);
  }

  //   Log2DDS::LogDataV0(
  //       "st_planner_traj_horizon",
  //       absl::StrCat(object_id, "-normal_obs-", st_planner_traj_horizon));
  return st_planner_traj_horizon;
}

}  // namespace

SpacetimePlannerObjectTrajectories BuildSpacetimePlannerObjectTrajectories(
    const SpacetimePlannerObjectTrajectoriesBuilderInput& input,
    absl::Span<const SpacetimeObjectTrajectory> trajectories,
    const double spacetime_planner_trajectory_horizon,
    const double trajectory_time_step, const double default_half_lane_width,
    std::unordered_map<std::string, double>& truncated_traj_map) {
  Timer timer(__FUNCTION__);
  CHECK_NOTNULL(input.passage);
  CHECK_NOTNULL(input.sl_boundary);
  CHECK_NOTNULL(input.veh_geom);
  CHECK_NOTNULL(input.plan_start_point);
  CHECK_NOTNULL(input.prev_st_trajs);
  CHECK_NOTNULL(input.spacetime_planner_object_trajectories_params);

  const auto& config = input.spacetime_planner_object_trajectories_params
                           ->spacetime_planner_object_trajectories_config();

  std::vector<NudgeInfo> nudgeinfos;
  nudgeinfos =
      CalNudgeinfo(input.time_aligned_prev_traj, input.lane_change_state,
                   *input.sl_boundary, *input.veh_geom, *input.passage);

  // Consider all objects in spacetime planner.
  std::vector<std::unique_ptr<SpacetimePlannerObjectTrajectoriesFinder>>
      finders;
  if (config.enable_all_finder()) {
    finders.push_back(
        std::make_unique<AllSpacetimePlannerObjectTrajectoriesFinder>());
  }
  const auto& curr_path_point = input.plan_start_point->path_point();
  const auto av_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *input.veh_geom);
  if (config.enable_stationary_finder()) {
    // Use customized finders for spacetime objects.
    finders.push_back(
        std::make_unique<StationarySpacetimePlannerObjectTrajectoriesFinder>(
            input.psmm, input.passage->lane_path(), av_box, input.veh_geom));
  }
  if (config.enable_front_side_moving_finder()) {  // NOLINT
    finders.push_back(std::make_unique<
                      FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder>(
        av_box, input.passage, input.sl_boundary, input.plan_start_point->v(),
        input.prev_st_trajs, input.time_aligned_prev_traj, input.veh_geom,
        input.psmm, input.nudge_object_info, input.lane_change_state,
        nudgeinfos, input.plan_id));
  }
  if (config.enable_dangerous_side_moving_finder()) {  // NOLINT
    finders.push_back(
        std::make_unique<
            DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder>(
            av_box, input.passage, input.plan_start_point->v()));
  }
  // Activate with caution, all front trajectories will be considered by
  // spacetime planner.
  if (config.enable_front_moving_finder()) {
    finders.push_back(
        std::make_unique<FrontMovingSpacetimePlannerObjectTrajectoriesFinder>(
            input.passage, input.plan_start_point, input.veh_geom->length()));
  }

  std::vector<std::unique_ptr<SpacetimePlannerObjectTrajectoriesFilter>>
      filters;
  // Cutin filter
  filters.push_back(
      std::make_unique<CutInSpacetimePlannerObjectTrajectoriesFilter>(
          input.passage, input.lane_change_state, av_box,
          input.plan_start_point->v()));
  // filters.push_back(
  //     std::make_unique<TargetBoundarySpacetimePlannerObjectTrajectoriesFilter>(
  //         input.passage, input.sl_boundary, input.lane_change_state, av_box,
  //         input.plan_start_point->v()));
  if (config.enable_cutin_vehicle_filter()) {  // NOLINT
    const auto& curr_path_point = input.plan_start_point->path_point();
    const auto av_box =
        ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                     curr_path_point.theta(), *input.veh_geom);
    filters.push_back(
        std::make_unique<CutInVehicleSpacetimePlannerObjectTrajectoriesFilter>(
            input.passage, input.lane_change_state, av_box,
            input.plan_start_point->v()));
  }
  if (config.enable_crossing_filter()) {
    filters.push_back(
        std::make_unique<CrossingSpacetimePlannerObjectTrajectoriesFilter>(
            input.passage, input.psmm));
  }
  if (config.enable_reverse_vehicle_filter()) {  // NOLINT
    const auto& curr_path_point = input.plan_start_point->path_point();
    auto av_box = ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                               curr_path_point.theta(), *input.veh_geom);
    filters.push_back(std::make_unique<
                      ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter>(
        input.passage, input.psmm, input.veh_geom, input.sl_boundary,
        input.nudge_object_info, std::move(av_box),
        input.plan_start_point->v()));
  }
  if (config.enable_beyond_stop_line_filter()) {  // NOLINT
    filters.push_back(std::make_unique<
                      BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter>(
        input.passage, input.stop_lines));
  }
  return GetSpacetimePlannerObjectTrajectories(
      input, trajectories, finders, filters,
      spacetime_planner_trajectory_horizon, trajectory_time_step,
      default_half_lane_width, truncated_traj_map, av_box, nudgeinfos);
}
}  // namespace planning
}  // namespace st
