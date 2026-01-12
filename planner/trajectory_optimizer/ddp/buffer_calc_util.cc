#include "planner/trajectory_optimizer/ddp/buffer_calc_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "planner/trajectory_optimizer/ddp/object_cost_util.h"
#include "planner/trajectory_optimizer/ddp/path_time_corridor.h"
#include "planner/planner_manager/planner_util.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/trajectory_util.h"

namespace st {
namespace planning {
namespace optimizer {

void NudgeBufferManager::CalcNudgeBuffers(
    const PathTimeCorridor& path_time_corridor,
    const PlannerSemanticMapManager& psmm,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const std::vector<TrajectoryPoint>& prev_traje,
    const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage) {
  auto trajs = st_planner_object_traj.trajectories;
  std::vector<PlannerObject> uu;
  std::vector<ad_byd::planning::RoadBoundaryConstPtr> curb;
  std::vector<PlannerObject> static_objects;
  std::vector<PlannerObject> dynmic_objects;
  for (const auto& traj : trajs) {
    if (traj.object_type() == ObjectType::OT_UNKNOWN_MOVABLE) {
      uu.emplace_back(traj.planner_object());
    } else if (traj.is_stationary()) {
      static_objects.emplace_back(traj.planner_object());
    } else if (!traj.is_stationary()) {
      dynmic_objects.emplace_back(traj.planner_object());
    }
  }
  std::unordered_set<mapping::ElementId> curb_boundaries;
  for (int i = 1; i < drive_passage.size(); ++i) {
    // Don't collect curb for extended drive passage.
    // TODO: Cut curb boundaries outside lane keeping drive passage.
    if (i > drive_passage.last_real_station_index().value()) {
      break;
    }
    const Vec2d p0 = drive_passage.station(StationIndex(i - 1)).xy();
    const Vec2d& p1 = drive_passage.station(StationIndex(i)).xy();
    const double search_radius = kMaxLateralOffset + (p1 - p0).Length() * 0.5;
    const Vec2d search_center = 0.5 * (p0 + p1);
    const std::vector<ad_byd::planning::RoadBoundaryConstPtr>
        candidate_boundaries =
            psmm.GetRoadBoundaries(search_center, search_radius);
    for (const auto& candidate_boundary : candidate_boundaries) {
      curb_boundaries.insert(candidate_boundary->id());
      curb.emplace_back(candidate_boundary);
    }
  }
}

const std::vector<double> NudgeBufferManager::GenerateNudgeBufferDynamic(
    const int plan_id, const LaneChangeStage lc_stage, const bool borrow_lane,
    const std::vector<prediction::PredictionObjectState>& states,
    const std::vector<st::planning::TrajectoryPoint>& init_traj,
    bool is_camera_object, const Vec2d& object_velocity,
    const Polygon2d& object_contour, const TrajectoryPoint& plan_start_point,
    const DrivePassage& drive_passage, const double lane_width,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const SpacetimeObjectTrajectory& traj, const PathSlBoundary& path_boundary,
    const PathTimeCorridor& path_time_corridor,
    st::planning::optimizer::TurnType ego_turn_type,
    const std::vector<st::planning::TrajectoryPoint>& prev_traj) {
  const PiecewiseLinearFunction<double> nudge_buffer_av_speed_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.object_cost_params()
              .nudge_front_buffer_object_speed_plf());
  const PiecewiseLinearFunction<double>
      nudge_buffer_gain_object_speed_diff_plf =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .nudge_buffer_gain_object_speed_diff_plf());
  const PiecewiseLinearFunction<double> min_nudge_buffer_speed_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.object_cost_params().min_nudge_buffer_speed_plf());
  const Vec2d av_local_dir = Vec2d::FastUnitFromAngle(plan_start_point.theta());
  const double obj_v_av_local = object_velocity.dot(av_local_dir);
  const double speed_diff = obj_v_av_local - plan_start_point.v();

  double nudge_buffer =
      (lc_stage == LaneChangeStage::LCS_NONE)
          ? std::clamp(0.4 + 0.45 * (lane_width - 2.7), 0.4, 0.85)
          : 0.85;
  const double heading_diff_cos = std::cos(
      NormalizeAngle(plan_start_point.theta() - traj.bounding_box().heading()));
  bool is_oppsite_obj = heading_diff_cos < -0.707;
  if (is_oppsite_obj) {
    nudge_buffer = std::max(0.75, nudge_buffer);
  }
  double min_nudge_buffer_dyn = 0.5;
  double max_nudge_buffer_dyn = 1.0;
  double nudge_buffer_base =
      std::max(min_nudge_buffer_dyn,
               nudge_buffer * nudge_buffer_av_speed_plf(plan_start_point.v()));

  if (traj.object_type() == OT_VEHICLE ||
      traj.object_type() == OT_LARGE_VEHICLE) {
    nudge_buffer_base += 0.1;
  }

  if (traj.object_type() == OT_MOTORCYCLIST ||
      traj.object_type() == OT_CYCLIST || traj.object_type() == OT_PEDESTRIAN ||
      traj.object_type() == OT_TRICYCLIST) {
    nudge_buffer_base += 0.1;
    min_nudge_buffer_dyn = 0.6;
    // max_nudge_buffer_dyn = 1.1;
  } else if (traj.object_type() == OT_LARGE_VEHICLE) {
    nudge_buffer_base += 0.10;
    min_nudge_buffer_dyn += 0.10;
    max_nudge_buffer_dyn += 0.10;
  }

  double nudge_buffer_consider_lane = nudge_buffer;
  const auto object_frenet_box_output =
      drive_passage.QueryFrenetBoxAtContour(object_contour);
  if (object_frenet_box_output.ok()) {
    const auto& object_frenet_box = object_frenet_box_output.value();
    const double object_s = object_frenet_box.center_s();
    const auto object_lane_boundary_info =
        drive_passage.QueryEnclosingLaneBoundariesAtS(object_s);
    constexpr double kNudgeBufferLaneWidthGain = 1.1;
    if (object_frenet_box.l_max < 0.0) {
      // If object on the right side of stations.
      if (object_lane_boundary_info.right.has_value()) {
        nudge_buffer_consider_lane =
            std::clamp((-object_lane_boundary_info.right->lat_offset -
                        veh_geo_params.width() * 0.5) *
                           kNudgeBufferLaneWidthGain,
                       min_nudge_buffer_dyn, nudge_buffer_base);
      }
    } else if (object_frenet_box.l_min > 0.0) {
      // If object on the left side of stations.
      if (object_lane_boundary_info.left.has_value()) {
        nudge_buffer_consider_lane =
            std::clamp((object_lane_boundary_info.left->lat_offset -
                        veh_geo_params.width() * 0.5) *
                           kNudgeBufferLaneWidthGain,
                       min_nudge_buffer_dyn, nudge_buffer_base);
      }
    }

    if (object_lane_boundary_info.right.has_value() &&
        object_lane_boundary_info.left.has_value()) {
      nudge_buffer_consider_lane =
          std::min(nudge_buffer_consider_lane,
                   std::clamp(((object_lane_boundary_info.left->lat_offset -
                                object_lane_boundary_info.right->lat_offset) *
                                   0.5 -
                               veh_geo_params.width() * 0.5) *
                                  kNudgeBufferLaneWidthGain,
                              min_nudge_buffer_dyn, nudge_buffer_base));
      const double nudge_buffer_min =
          nudge_buffer_base * min_nudge_buffer_speed_plf(std::max(
                                  obj_v_av_local, plan_start_point.v()));
      nudge_buffer = std::max(nudge_buffer_consider_lane, nudge_buffer_min);
      if (ego_turn_type != TurnType::kLeftTurn) {
        nudge_buffer *= nudge_buffer_gain_object_speed_diff_plf(speed_diff);
      }
      nudge_buffer_base = std::max(nudge_buffer, min_nudge_buffer_dyn);
    }
  } else {
    nudge_buffer_base = 0.8;
  }

  double nudge_buffer_base_fix = nudge_buffer_base;
  std::vector<double> buffer_value;
  std::vector<double> s_obj_states;
  std::vector<double> l_left_obj_states;
  std::vector<double> l_right_obj_states;
  const auto object_position_info =
      path_time_corridor.QueryObjectPositionInfo(std::string(traj.object_id()));
  for (int i = 0; i < states.size(); ++i) {
    const auto object_frenet_box_dt_output =
        drive_passage.QueryFrenetBoxAtContour(states[i].contour);
    if (!object_frenet_box_dt_output.ok()) {
      double attenuation_buffer = nudge_buffer_base;
      if (lc_stage == LaneChangeStage::LCS_NONE && !init_traj.empty() &&
          !prediction::IsVulnerableRoadUserType(traj.object_type())) {
        PiecewiseLinearFunction<double> attenuation_thr_cal_func =
            PiecewiseLinearFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .dynamic_obs_buffer_attenuation_threshold());
        double attenuation_thr =
            attenuation_thr_cal_func(std::fabs(init_traj.front().v()));
        constexpr double kMinObsV = 11.1;
        constexpr double kMinEgoV = 20.0;
        if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE &&
            !borrow_lane && std::fabs(obj_v_av_local) >= kMinObsV &&
            std::fabs(init_traj.front().v()) >= kMinEgoV) {
          const std::vector<double> vec_speed = {20.0, 22.2, 36.1};
          const std::vector<double> vec_base_buffer = {0.90, 0.90, 1.10};
          const PiecewiseLinearFunction<double> base_buffer_plf(
              vec_speed, vec_base_buffer);
          attenuation_thr = base_buffer_plf(std::fabs(init_traj.front().v()));
        }
        PiecewiseLinearFunction<double> attenuation_factor_func =
            PiecewiseLinearFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .dynamic_obs_buffer_attenuation_factor());
        double atten_factor =
            attenuation_factor_func(std::fabs(init_traj.front().v()));
        if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE &&
            !borrow_lane && std::fabs(obj_v_av_local) >= kMinObsV) {
          const std::vector<double> vec_speed = {20.0, 22.2, 36.1};
          const std::vector<double> vec_buffer_factor = {0.90, 0.30, 0.30};
          const PiecewiseLinearFunction<double> buffer_factor_plf(
              vec_speed, vec_buffer_factor);
          atten_factor = buffer_factor_plf(std::fabs(init_traj.front().v()));
        }
        attenuation_buffer =
            attenuation_buffer <= attenuation_thr
                ? attenuation_buffer
                : attenuation_thr +
                      (attenuation_buffer - attenuation_thr) * atten_factor;

        double large_veh_extra_buffer = 0.0;
        if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE &&
            !borrow_lane) {
          int cur_station_index =
              drive_passage
                  .FindNearestStationIndex(Vec2d(init_traj.front().pos().x(),
                                                 init_traj.front().pos().y()))
                  .value();
          const Station& cur_station =
              drive_passage.station(StationIndex(cur_station_index));
          if (!cur_station.station_info().is_in_intersection) {
            const std::vector<double> vec_speed = {0.0, 1.0, 2.0, 3.0};
            const std::vector<double> vec_extra_buffer = {-0.20, -0.1, -0.05,
                                                          0.0};
            const PiecewiseLinearFunction<double> extra_buffer_plf(
                vec_speed, vec_extra_buffer);
            large_veh_extra_buffer =
                extra_buffer_plf(std::fabs(init_traj.front().v()));

            const std::vector<double> vec_a = {-1.3, -0.9, 0.0, 0.5};
            const std::vector<double> vec_extra_buffer_gain = {0.0, 1.0, 1.0,
                                                               0.0};
            const PiecewiseLinearFunction<double> extra_buffer_gain_plf(
                vec_a, vec_extra_buffer_gain);
            large_veh_extra_buffer *=
                extra_buffer_gain_plf(init_traj.front().a());
          }
        }
        attenuation_buffer += large_veh_extra_buffer;
        attenuation_buffer = std::max(0.5, attenuation_buffer);
      }

      if (prediction::IsVulnerableRoadUserType(traj.object_type())) {
        PiecewiseLinearFunction<double> vru_extra_buffer_cal_func =
            PiecewiseLinearFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .vru_extra_buffer_speed_plf());
        double extra_buffer =
            vru_extra_buffer_cal_func(std::fabs(init_traj.front().v()));
        attenuation_buffer += extra_buffer;
      }

      buffer_value.push_back(attenuation_buffer);
      continue;
    }
    const auto& object_frenet_box_dt = object_frenet_box_dt_output.value();
    const double object_s_dt = object_frenet_box_dt.center_s();
    s_obj_states.push_back(object_s_dt);
    const auto& st_corridor = path_time_corridor.QueryBoundaryL(
        object_s_dt, states.at(i).traj_point->t());
    l_left_obj_states.push_back(st_corridor.second.l_object);
    l_right_obj_states.push_back(st_corridor.first.l_object);

    const auto object_station_point =
        drive_passage.FindNearestStation(states[i].traj_point->pos());
    const double heading_point_diff_cos = std::cos(
        NormalizeAngle(init_traj[i].theta() - states[i].box.heading()));
    bool is_oppsite_obj_point = heading_point_diff_cos < -0.707;
    bool is_obj_point_junction = object_station_point.is_in_intersection();
    if (object_station_point.station_info().turn_type ==
            ad_byd::planning::TurnType::RIGHT_TURN &&
        is_obj_point_junction && is_oppsite_obj_point) {
      Log2DDS::LogDataV2("heading_diff_cross, ",
                         "RIGHT_TURN_JUNCTION change buffers");
      if (traj.object_type() == OT_LARGE_VEHICLE) {
        nudge_buffer_base_fix = nudge_buffer_base + 0.5;
      } else {
        nudge_buffer_base_fix = nudge_buffer_base + 0.2;
      }
    }

    if (object_position_info.at(i).position_type ==
        PathTimeCorridor::PositionType::POSITION_LEFT) {
      double coef_space_obj = 0.5;
      // if (st_corridor.second.type ==
      // PathTimeCorridor::BoundaryInfo::oppsite_obj ||
      //   is_oppsite_obj) {
      //   coef_space_obj =
      // } else
      if (st_corridor.second.type == PathTimeCorridor::BoundaryInfo::VRU ||
          prediction::IsVulnerableRoadUserType(traj.object_type())) {
        switch (st_corridor.first.type) {
          case PathTimeCorridor::BoundaryInfo::VRU:
          case PathTimeCorridor::BoundaryInfo::CURB:
          case PathTimeCorridor::BoundaryInfo::STATIC:
            coef_space_obj = 0.5;
            break;
          case PathTimeCorridor::BoundaryInfo::LARGE_VEHICLE:
          case PathTimeCorridor::BoundaryInfo::VEHICLE:
            coef_space_obj = 0.7;
            break;
          case PathTimeCorridor::BoundaryInfo::LANE_BOUNDARY:
            coef_space_obj = 0.8;
            break;
          default:
            coef_space_obj = 0.5;
            break;
        }
      } else if (st_corridor.second.type ==
                     PathTimeCorridor::BoundaryInfo::LARGE_VEHICLE ||
                 st_corridor.second.type ==
                     PathTimeCorridor::BoundaryInfo::VEHICLE ||
                 traj.object_type() == OT_LARGE_VEHICLE ||
                 traj.object_type() == OT_VEHICLE) {
        switch (st_corridor.first.type) {
          case PathTimeCorridor::BoundaryInfo::VRU:
          case PathTimeCorridor::BoundaryInfo::CURB:
            coef_space_obj = 0.3;
            break;
          case PathTimeCorridor::BoundaryInfo::LARGE_VEHICLE:
          case PathTimeCorridor::BoundaryInfo::VEHICLE:
          case PathTimeCorridor::BoundaryInfo::STATIC:
            coef_space_obj = 0.5;
            break;
          case PathTimeCorridor::BoundaryInfo::LANE_BOUNDARY:
            coef_space_obj = 0.6;
            break;
          default:
            coef_space_obj = 0.5;
            break;
        }
      }
      // not exist below
      // else if (st_corridor.second.type ==
      //              PathTimeCorridor::BoundaryInfo::LANE_BOUNDARY) {
      // }
      double dis_space =
          st_corridor.second.l_object - st_corridor.first.l_object;
      nudge_buffer =
          std::max(0.0, (dis_space - veh_geo_params.width()) * coef_space_obj);
      nudge_buffer =
          std::clamp(nudge_buffer, min_nudge_buffer_dyn, max_nudge_buffer_dyn);
    } else if (object_position_info.at(i).position_type ==
               PathTimeCorridor::PositionType::POSITION_RIGHT) {
      double coef_space_obj = 0.5;
      if (st_corridor.first.type == PathTimeCorridor::BoundaryInfo::VRU ||
          prediction::IsVulnerableRoadUserType(traj.object_type())) {
        switch (st_corridor.second.type) {
          case PathTimeCorridor::BoundaryInfo::VRU:
          case PathTimeCorridor::BoundaryInfo::CURB:
          case PathTimeCorridor::BoundaryInfo::STATIC:
            coef_space_obj = 0.5;
            break;
          case PathTimeCorridor::BoundaryInfo::LARGE_VEHICLE:
          case PathTimeCorridor::BoundaryInfo::VEHICLE:
            coef_space_obj = 0.6;
            break;
          case PathTimeCorridor::BoundaryInfo::LANE_BOUNDARY:
            coef_space_obj = 0.8;
            break;
          default:
            coef_space_obj = 0.5;
            break;
        }
      } else if (st_corridor.first.type ==
                     PathTimeCorridor::BoundaryInfo::LARGE_VEHICLE ||
                 st_corridor.first.type ==
                     PathTimeCorridor::BoundaryInfo::VEHICLE ||
                 traj.object_type() == OT_LARGE_VEHICLE ||
                 traj.object_type() == OT_VEHICLE) {
        switch (st_corridor.second.type) {
          case PathTimeCorridor::BoundaryInfo::VRU:
          case PathTimeCorridor::BoundaryInfo::CURB:
            coef_space_obj = 0.4;
            break;
          case PathTimeCorridor::BoundaryInfo::LARGE_VEHICLE:
          case PathTimeCorridor::BoundaryInfo::VEHICLE:
          case PathTimeCorridor::BoundaryInfo::STATIC:
            coef_space_obj = 0.5;
            break;
          case PathTimeCorridor::BoundaryInfo::LANE_BOUNDARY:
            coef_space_obj = 0.6;
            break;
          default:
            coef_space_obj = 0.5;
            break;
        }
      }

      double dis_space =
          borrow_lane
              ? st_corridor.second.l_object - st_corridor.first.l_object - 0.5
              : st_corridor.second.l_object - st_corridor.first.l_object;
      nudge_buffer =
          std::max(0.0, (dis_space - veh_geo_params.width()) * coef_space_obj);
      nudge_buffer =
          std::clamp(nudge_buffer, min_nudge_buffer_dyn, max_nudge_buffer_dyn);
    } else {
      nudge_buffer = nudge_buffer_base_fix;
    }
    nudge_buffer = std::max(nudge_buffer_base_fix, nudge_buffer);
    constexpr double kMinBufferForEgoTurnRight = 0.7;
    if (ego_turn_type == TurnType::kRightTurn) {
      nudge_buffer = std::max(nudge_buffer, kMinBufferForEgoTurnRight);
    }

    double attenuation_buffer = nudge_buffer;
    if (lc_stage == LaneChangeStage::LCS_NONE && !init_traj.empty() &&
        !prediction::IsVulnerableRoadUserType(traj.object_type())) {
      PiecewiseLinearFunction<double> attenuation_thr_cal_func =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .dynamic_obs_buffer_attenuation_threshold());
      double attenuation_thr =
          attenuation_thr_cal_func(std::fabs(init_traj.front().v()));
      constexpr double kMinObsV = 11.1;
      constexpr double kMinEgoV = 20.0;
      if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE && !borrow_lane &&
          std::fabs(obj_v_av_local) >= kMinObsV &&
          std::fabs(init_traj.front().v()) >= kMinEgoV) {
        const std::vector<double> vec_speed = {20.0, 22.2, 36.1};
        const std::vector<double> vec_base_buffer = {0.90, 0.90, 1.10};
        const PiecewiseLinearFunction<double> base_buffer_plf(vec_speed,
                                                              vec_base_buffer);
        attenuation_thr = base_buffer_plf(std::fabs(init_traj.front().v()));
      }
      PiecewiseLinearFunction<double> attenuation_factor_func =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .dynamic_obs_buffer_attenuation_factor());
      double atten_factor =
          attenuation_factor_func(std::fabs(init_traj.front().v()));
      if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE && !borrow_lane &&
          std::fabs(obj_v_av_local) >= kMinObsV) {
        const std::vector<double> vec_speed = {20.0, 22.2, 36.1};
        const std::vector<double> vec_buffer_factor = {0.90, 0.30, 0.30};

        const PiecewiseLinearFunction<double> buffer_factor_plf(
            vec_speed, vec_buffer_factor);
        atten_factor = buffer_factor_plf(std::fabs(init_traj.front().v()));
      }
      attenuation_buffer =
          nudge_buffer <= attenuation_thr
              ? nudge_buffer
              : attenuation_thr +
                    (nudge_buffer - attenuation_thr) * atten_factor;

      constexpr double kMinDeltaSpeed = 0.0;
      constexpr double kMaxDeltaSpeed = 3.0;
      if (speed_diff > kMinDeltaSpeed) {
        PiecewiseLinearFunction<double> attenuation_thr_cal_func_fast =
            PiecewiseLinearFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .fast_obs_buffer_attenuation_threshold());
        double attenuation_thr_fast =
            attenuation_thr_cal_func_fast(std::fabs(init_traj.front().v()));
        PiecewiseLinearFunction<double> attenuation_factor_func =
            PiecewiseLinearFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .fast_obs_buffer_attenuation_factor());
        double atten_factor =
            attenuation_factor_func(std::fabs(init_traj.front().v()));
        double attenuation_buffer_fast =
            nudge_buffer <= attenuation_thr_fast
                ? nudge_buffer
                : attenuation_thr_fast +
                      (nudge_buffer - attenuation_thr_fast) * atten_factor;

        double transition_factor =
            (speed_diff >= kMaxDeltaSpeed) ? 1.0 : speed_diff / kMaxDeltaSpeed;
        attenuation_buffer = attenuation_buffer * (1.0 - transition_factor) +
                             attenuation_buffer_fast * transition_factor;
      }

      double large_veh_extra_buffer = 0.0;
      if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE && !borrow_lane) {
        int cur_station_index =
            drive_passage
                .FindNearestStationIndex(Vec2d(init_traj.front().pos().x(),
                                               init_traj.front().pos().y()))
                .value();
        const Station& cur_station =
            drive_passage.station(StationIndex(cur_station_index));
        if (!cur_station.station_info().is_in_intersection) {
          const std::vector<double> vec_speed = {0.0, 1.0, 2.0, 3.0};
          const std::vector<double> vec_extra_buffer = {-0.20, -0.1, -0.05,
                                                        0.0};
          const PiecewiseLinearFunction<double> extra_buffer_plf(
              vec_speed, vec_extra_buffer);
          large_veh_extra_buffer =
              extra_buffer_plf(std::fabs(init_traj.front().v()));

          const std::vector<double> vec_a = {-1.3, -0.9, 0.0, 0.5};
          const std::vector<double> vec_extra_buffer_gain = {0.0, 1.0, 1.0,
                                                             0.0};
          const PiecewiseLinearFunction<double> extra_buffer_gain_plf(
              vec_a, vec_extra_buffer_gain);
          large_veh_extra_buffer *=
              extra_buffer_gain_plf(init_traj.front().a());
        }
      }
      attenuation_buffer += large_veh_extra_buffer;
      attenuation_buffer = std::max(0.5, attenuation_buffer);
    }

    if (prediction::IsVulnerableRoadUserType(traj.object_type())) {
      PiecewiseLinearFunction<double> vru_extra_buffer_cal_func =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .vru_extra_buffer_speed_plf());
      double extra_buffer =
          vru_extra_buffer_cal_func(std::fabs(init_traj.front().v()));
      attenuation_buffer += extra_buffer;
    }

    buffer_value.push_back(attenuation_buffer);
  }
  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  Log2DDS::LogDataV2(prefix + std::string(traj.object_id()) + "_s",
                     s_obj_states);
  Log2DDS::LogDataV2(prefix + std::string(traj.object_id()) + "_buffer",
                     buffer_value);
  Log2DDS::LogDataV2(prefix + std::string(traj.object_id()) + "_lright",
                     l_right_obj_states);
  Log2DDS::LogDataV2(prefix + std::string(traj.object_id()) + "_lleft",
                     l_left_obj_states);

  const std::string base_name_with_plan_id =
      absl::StrFormat("opt_task_%d", plan_id);
  const std::string nudge_debug = absl::StrFormat(
      "%s,%s,buffer_dyn:%f,buffer_base:%f,states.size:%d,obj_type:%d,heading_"
      "diff_cos:%f,"
      "position_type:%d,turn_type:%d,"
      "speed_diff:%f,gain_object_speed_diff_plf:%f",
      base_name_with_plan_id, traj.object_id(), buffer_value.front(),
      nudge_buffer_base_fix, states.size(), traj.object_type(),
      heading_diff_cos, object_position_info.front().position_type,
      static_cast<int>(ego_turn_type), speed_diff,
      nudge_buffer_gain_object_speed_diff_plf(speed_diff));
  Log2DDS::LogDataV2("nudge_buffer_debug_dyn", nudge_debug);
  return buffer_value;
}

const std::vector<double> NudgeBufferManager::GenerateNudgeBufferStationary(
    const int plan_id, const LaneChangeStage lc_stage, const bool borrow_lane,
    const std::vector<prediction::PredictionObjectState>& states,
    bool is_camera_object, bool is_static, const Vec2d& object_velocity,
    const Polygon2d& object_contour,
    const std::vector<st::planning::TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const double lane_width,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const SpacetimeObjectTrajectory& traj, const PathSlBoundary& path_boundary,
    const PathTimeCorridor& path_time_corridor, TurnType ego_turn_type) {
  const auto object_frenet_box_output =
      drive_passage.QueryFrenetBoxAtContour(object_contour);
  if (!object_frenet_box_output.ok())
    return std::vector<double>(states.size(), 0.65);
  const auto& object_frenet_box = object_frenet_box_output.value();
  const double object_s = object_frenet_box.center_s();
  // find bypass time at previous trajectory to get corridor bound
  double bypass_time = 15.0;
  size_t bypass_traj_point_idx = 0;
  if (!init_traj.empty()) {
    size_t next_traj_point_idx = init_traj.size() - 1;
    while (next_traj_point_idx - bypass_traj_point_idx > 1) {
      size_t mid_idx = (next_traj_point_idx + bypass_traj_point_idx) >> 1;
      if (init_traj.at(mid_idx).s() < object_s) {
        bypass_traj_point_idx = mid_idx;
      } else {
        next_traj_point_idx = mid_idx;
      }
    }
    bypass_time = init_traj.at(bypass_traj_point_idx).t();
  }
  const double angle_diff = fabs(NormalizeAngle(init_traj.front().theta() -
                                                traj.bounding_box().heading()));
  const double heading_diff_sin =
      angle_diff < M_PI / 3 ? 0.0 : std::sin(angle_diff);
  double buffer_factor = 1.0 + 0.4 * heading_diff_sin;
  auto corridor_bound =
      path_time_corridor.QueryBoundaryL(object_s, bypass_time);
  for (size_t i = 0; i < bypass_traj_point_idx; ++i) {
    const double s = init_traj.at(i).s();
    const double t = init_traj.at(i).t();
    auto bound = path_time_corridor.QueryBoundaryL(s, t);
    corridor_bound.first.l_object =
        std::max(corridor_bound.first.l_object, bound.first.l_object);
    corridor_bound.second.l_object =
        std::min(corridor_bound.second.l_object, bound.second.l_object);
  }
  const auto object_position_info =
      path_time_corridor.QueryObjectPositionInfo(std::string(traj.object_id()));

  if (object_position_info.empty()) {
    return std::vector<double>(states.size(), 0.65 * buffer_factor);
  }
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    return std::vector<double>(states.size(), 0.8 * buffer_factor);
  }
  double nudge_buffer = 0.65;
  const auto object_sl_boundary_info = path_boundary.QueryBoundaryL(object_s);
  const auto object_sl_boundary_info_target =
      path_boundary.QueryTargetBoundaryL(object_s);
  double object_sl_boundary_info_target_first = std::max(
      object_sl_boundary_info_target.first, corridor_bound.first.l_object);
  double object_sl_boundary_info_target_second = std::min(
      object_sl_boundary_info_target.second, corridor_bound.second.l_object);
  double extra_buffer_left = 0.0;
  double extra_buffer_right = 0.0;
  constexpr double kMinNudgeBuffer = 0.65;
  double ego_speed_factor =
      std::clamp((init_traj.front().v() - 8.333) / 8.333, 0.0, 1.0);
  double kMaxNudgeBuffer =
      (((traj.object_type() == ObjectType::OT_VEHICLE ||
         traj.object_type() == ObjectType::OT_LARGE_VEHICLE)
            ? 1.0
            : 0.8) +
       0.2 * ego_speed_factor);
  if (object_position_info.front().position_type ==
      PathTimeCorridor::PositionType::POSITION_LEFT) {
    // If object on the left side of stations.
    // Object outside target boundary
    double object_invade_dist =
        object_sl_boundary_info_target.second - object_frenet_box.l_min;
    if (object_invade_dist < 0.0) {
      return std::vector<double>(states.size(), 0.65 * buffer_factor);
    }
    double extra_buffer_factor =
        std::clamp((object_invade_dist - 0.15) * 5.0, 0.0, 1.0);
    nudge_buffer = std::clamp(
        (object_frenet_box.l_min - object_sl_boundary_info_target_first -
         veh_geo_params.width()) *
            0.5,
        kMinNudgeBuffer, kMaxNudgeBuffer);
    extra_buffer_right = nudge_buffer < 0.75
                             ? (object_sl_boundary_info_target_first -
                                object_sl_boundary_info.first) *
                                   0.20 * extra_buffer_factor
                             : 0.0;
    nudge_buffer += std::min(extra_buffer_right, 0.4);
  } else if (object_position_info.front().position_type ==
             PathTimeCorridor::PositionType::POSITION_RIGHT) {
    // If object on the right side of stations.
    // Object outside target boundary
    double object_invade_dist =
        object_frenet_box.l_max - object_sl_boundary_info_target.first;
    if (object_invade_dist < 0.0) {
      return std::vector<double>(states.size(), 0.65 * buffer_factor);
    }
    double extra_buffer_factor =
        std::clamp((object_invade_dist - 0.15) * 5.0, 0.0, 1.0);
    nudge_buffer =
        std::clamp((object_sl_boundary_info_target_second -
                    object_frenet_box.l_max - veh_geo_params.width()) *
                       0.5,
                   kMinNudgeBuffer, kMaxNudgeBuffer);
    extra_buffer_left = nudge_buffer < 0.75
                            ? (object_sl_boundary_info.second -
                               object_sl_boundary_info_target_second) *
                                  0.20 * extra_buffer_factor
                            : 0.0;
    nudge_buffer += std::min(extra_buffer_left, 0.4);
  } else {
    nudge_buffer = 0.65;
  }
  if (traj.object_type() == ObjectType::OT_VEHICLE ||
      traj.object_type() == OT_LARGE_VEHICLE) {
    nudge_buffer =
        std::min(nudge_buffer * buffer_factor, kMaxNudgeBuffer) + 0.1;
  }
  nudge_buffer = std::min(nudge_buffer * buffer_factor, kMaxNudgeBuffer);

  double attenuation_buffer = nudge_buffer;
  if (lc_stage == LaneChangeStage::LCS_NONE && !init_traj.empty() &&
      !prediction::IsVulnerableRoadUserType(traj.object_type())) {
    PiecewiseLinearFunction<double> attenuation_thr_cal_func =
        PiecewiseLinearFunctionFromProto(
            cost_weight_params.object_cost_params()
                .stationary_obs_buffer_attenuation_threshold());
    double attenuation_thr =
        attenuation_thr_cal_func(std::fabs(init_traj.front().v()));
    PiecewiseLinearFunction<double> attenuation_factor_func =
        PiecewiseLinearFunctionFromProto(
            cost_weight_params.object_cost_params()
                .stationary_obs_buffer_attenuation_factor());
    double atten_factor =
        attenuation_factor_func(std::fabs(init_traj.front().v()));
    attenuation_buffer =
        attenuation_buffer <= attenuation_thr
            ? attenuation_buffer
            : attenuation_thr +
                  (attenuation_buffer - attenuation_thr) * atten_factor;

    if (!borrow_lane && (traj.object_type() == ObjectType::OT_VEHICLE ||
                         traj.object_type() == OT_LARGE_VEHICLE)) {
      const std::vector<double> vec_speed = {6.0, 12.0, 20.0, 35.0};
      const std::vector<double> vec_stationary_extra_gain = {0.0, 0.10, 0.15,
                                                             0.20};
      const PiecewiseLinearFunction<double> stationary_extra_buffer_plf(
          vec_speed, vec_stationary_extra_gain);
      attenuation_buffer +=
          stationary_extra_buffer_plf(std::fabs(init_traj.front().v()));
    }

    double large_veh_extra_buffer = 0.0;
    if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE && !borrow_lane) {
      int cur_station_index =
          drive_passage
              .FindNearestStationIndex(Vec2d(init_traj.front().pos().x(),
                                             init_traj.front().pos().y()))
              .value();
      const Station& cur_station =
          drive_passage.station(StationIndex(cur_station_index));
      if (!cur_station.station_info().is_in_intersection) {
        const std::vector<double> vec_speed = {0.0, 1.0, 2.0, 3.0};
        const std::vector<double> vec_extra_buffer = {-0.20, -0.1, -0.05, 0.0};
        const PiecewiseLinearFunction<double> extra_buffer_plf(
            vec_speed, vec_extra_buffer);
        large_veh_extra_buffer =
            extra_buffer_plf(std::fabs(init_traj.front().v()));

        const std::vector<double> vec_a = {-1.3, -0.9, 0.0, 0.5};
        const std::vector<double> vec_extra_buffer_gain = {0.0, 1.0, 1.0, 0.0};
        const PiecewiseLinearFunction<double> extra_buffer_gain_plf(
            vec_a, vec_extra_buffer_gain);
        large_veh_extra_buffer *= extra_buffer_gain_plf(init_traj.front().a());
      }
    }
    attenuation_buffer += large_veh_extra_buffer;
    attenuation_buffer = std::max(0.5, attenuation_buffer);
  }

  if (prediction::IsVulnerableRoadUserType(traj.object_type())) {
    PiecewiseLinearFunction<double> vru_extra_buffer_cal_func =
        PiecewiseLinearFunctionFromProto(cost_weight_params.object_cost_params()
                                             .vru_extra_buffer_speed_plf());
    double extra_buffer =
        vru_extra_buffer_cal_func(std::fabs(init_traj.front().v()));
    attenuation_buffer += extra_buffer;
  }

  const std::string base_name_with_plan_id =
      absl::StrFormat("opt_task_%d", plan_id);
  const std::string nudge_debug = absl::StrFormat(
      "%s,%s,nudge_buffer:%f,buffer_min:%f,buffer_max:%f,"
      "position_type:%d,nudge_space:%f,right_extra:%f,left_extra:%f,obj_min_"
      "l:%f,obj_max_l:%f,"
      "right_offset:%f,left_offset:%f",
      base_name_with_plan_id, traj.object_id(), attenuation_buffer,
      kMinNudgeBuffer, kMaxNudgeBuffer,
      static_cast<int>(object_position_info.front().position_type),
      object_frenet_box.l_min > 0.0
          ? (object_frenet_box.l_min - object_sl_boundary_info_target_first -
             veh_geo_params.width()) *
                0.5
          : (object_sl_boundary_info_target_second - object_frenet_box.l_max -
             veh_geo_params.width()) *
                0.5,
      extra_buffer_right, extra_buffer_left, object_frenet_box.l_min,
      object_frenet_box.l_max, object_sl_boundary_info_target_first,
      object_sl_boundary_info_target_second);
  Log2DDS::LogDataV2("nudge_buffer_debug", nudge_debug);
  return std::vector<double>(states.size(), attenuation_buffer);
}

bool NudgeBufferManager::GenerateNudgeBufferUU(
    const int plan_id, double trajectory_time_step, std::string_view base_name,
    const LaneChangeStage lc_stage, const TrajectoryPoint& plan_start_point,
    const PathTimeCorridor& path_time_corridor,
    absl::Span<const SpacetimeObjectTrajectory* const> spacetime_trajs,
    double min_mirror_height_avg, double max_mirror_height_avg,
    const double lane_width_l, const double lane_width_r,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const VehicleGeometryParamsProto& veh_geo_params) {
  std::vector<Segment2d> segments;
  std::vector<Segment2d> segments_consider_mirrors;
  constexpr int kEstimateLineCountsPerObject = 6;
  CHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);
  segments.reserve(kEstimateLineCountsPerObject * spacetime_trajs.size());
  segments_consider_mirrors.reserve(kEstimateLineCountsPerObject *
                                    spacetime_trajs.size());
  const double lane_width = lane_width_l - lane_width_r;
  std::vector<bool> obj_within_lane_vec;
  for (int idx = 0; idx < spacetime_trajs.size(); ++idx) {
    const auto& traj = *spacetime_trajs[idx];
    auto obejct_position = path_time_corridor.QueryObjectPositionInfo(
        std::string(traj.object_id()));
    auto obj_boundary_info =
        path_time_corridor.QueryBoundaryL(obejct_position[0].object_s, 0);
    auto lane_width_obj = obj_boundary_info.second.l_boundary -
                          obj_boundary_info.first.l_boundary;

    const bool obj_within_lane = obejct_position[0].object_l < lane_width_l &&
                                 obejct_position[0].object_l > lane_width_r;

    Log2DDS::LogDataV0("uu_debug",
                       absl::StrCat("lane_width_obj: ", lane_width_obj));
    Log2DDS::LogDataV0("uu_debug", absl::StrCat("obejct_position_l: ",
                                                obejct_position[0].object_l));
    Log2DDS::LogDataV0("uu_debug",
                       absl::StrCat("obj_within_lane: ", obj_within_lane));

    obj_within_lane_vec.emplace_back(obj_within_lane);
    /*     if (!obj_within_lane) {
          continue;
        } */

    Log2DDS::LogDataV0("object",
                       absl::StrCat("_s:", obejct_position[0].object_s));
    Log2DDS::LogDataV0("object",
                       absl::StrCat("_l:", obejct_position[0].object_l));
    Log2DDS::LogDataV0("object", absl::StrCat("_id:", traj.object_id()));
    Log2DDS::LogDataV0("object",
                       absl::StrCat("_dir:", obejct_position[0].position_type));
  }

  bool need_lane_width_buffer = false;
  for (bool objinlane : obj_within_lane_vec) {
    if (objinlane) {
      need_lane_width_buffer = true;
      break;
    }
  }

  if (segments.empty() && segments_consider_mirrors.empty()) {
    return true;
  }

  constexpr double kSafeBuffer = 0.6;
  // double nudge_buffer_soft =
  //     lc_stage == LaneChangeStage::LCS_NONE
  //         ? std::clamp(0.4 + 0.46 * (lane_width - 2.4), 0.4, 1.0)
  //         : 0.75;
  //  Copy from close_object_slowdown_decider, please modify at the same time.
  const std::vector<double> station_inside_sl_boundary_static_max_speed = {
      3.0, 10.0, 20.0, 30.0};  // m/s
  const std::vector<double> close_object_distance = {0.5, 0.65, 0.8, 1.0};  // m
  const std::vector<double> lane_width_vec = {2.7, 2.8, 2.9, 3.0, 4.0};
  const std::vector<double> nudge_buffers_vec = {0.05, 0.05, 0.05, 0.05, 0.3};

  const PiecewiseLinearFunction<double> nudge_buffer_lane_width_plf(
      lane_width_vec, nudge_buffers_vec);

  const PiecewiseLinearFunction<double> nudge_buffer_speed_plf(
      station_inside_sl_boundary_static_max_speed, close_object_distance);
  double nudge_buffer_soft =
      std::max(kSafeBuffer, nudge_buffer_speed_plf(plan_start_point.v()) + 0.1);
  if (need_lane_width_buffer) {
    nudge_buffer_soft = std::max(
        kSafeBuffer, nudge_buffer_speed_plf(plan_start_point.v()) +
                         nudge_buffer_lane_width_plf(lane_width) + 0.1);
  }
  Log2DDS::LogDataV0("uu_debug", absl::StrCat("need_lane_width_buffer: ",
                                              need_lane_width_buffer));
  const std::string nudge_buffer_soft_str =
      "StaticObject, task_" +
      absl::StrFormat("%d,nudge_buffer_soft:%f", plan_id, nudge_buffer_soft);
  Log2DDS::LogDataV2("object", nudge_buffer_soft_str);
  const double nudge_buffer_hard = kSafeBuffer;
  return true;
}

}  // namespace optimizer
}  // namespace planning
}  // namespace st
