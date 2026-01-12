

#include "planner/trajectory_optimizer/ddp/object_cost_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "plan_common/async/parallel_for.h"
#include "plan_common/log_data.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"
//#include "global/buffered_logger.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/piecewise_const_function.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "object_manager/planner_object.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "planner/trajectory_optimizer/problem/aggregate_static_object_cost.h"
#include "planner/trajectory_optimizer/problem/partitioned_object_cost.h"
#include "planner/trajectory_optimizer/problem/unidirectional_object_cost.h"
#include "planner/planner_manager/planner_defs.h"
#include "planner/planner_manager/planner_util.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/perception_util.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "plan_common/log_data.h"

namespace st {
namespace planning {
namespace optimizer {
namespace {
constexpr double kTrajVisZInc =
    kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep;

void CalculateNudgeGainForIntrusion(
    const int plan_id, const LaneChangeStage lc_stage,
    const DrivePassage& drive_passage,
    const std::vector<TrajectoryPoint>& init_traj,
    const double trajectory_time_step, const int trajectory_steps,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<double>* extra_gain_for_nudge) {
  constexpr double kHighSpeedThreshold = 60.0 / 3.6;
  constexpr double kSlowSpeedThreshold = 5.0 / 3.6;
  if (init_traj.empty() || init_traj.front().v() >= kHighSpeedThreshold ||
      init_traj.front().v() <= kSlowSpeedThreshold) {
    return;
  }

  constexpr double kLowSpeedThreshold = 45.0 / 3.6;
  constexpr double kFastSpeedThreshold = 15.0 / 3.6;
  double smooth_factor =
      std::min((kHighSpeedThreshold -
                std::max(kLowSpeedThreshold, init_traj.front().v())) /
                   (kHighSpeedThreshold - kLowSpeedThreshold),
               (std::min(kFastSpeedThreshold, init_traj.front().v()) -
                kSlowSpeedThreshold) /
                   (kFastSpeedThreshold - kSlowSpeedThreshold));

  int cur_station_index =
      drive_passage
          .FindNearestStationIndex(
              Vec2d(init_traj.front().pos().x(), init_traj.front().pos().y()))
          .value();
  const Station& cur_station =
      drive_passage.station(StationIndex(cur_station_index));
  PiecewiseLinearFunction<double> short_term_gain_cal_func;
  if (cur_station.station_info().is_in_intersection) {
    short_term_gain_cal_func = PiecewiseLinearFunctionFromProto(
        cost_weight_params.object_cost_params().short_term_gain_is());
  } else if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    short_term_gain_cal_func = PiecewiseLinearFunctionFromProto(
        cost_weight_params.object_cost_params().short_term_gain_lc());
  } else if (lc_stage == LaneChangeStage::LCS_NONE) {
    short_term_gain_cal_func = PiecewiseLinearFunctionFromProto(
        cost_weight_params.object_cost_params().short_term_gain_lk());
  } else {
    return;
  }

  constexpr double kMinGainValue = 1.0;
  for (int i = 0; i < trajectory_steps; ++i) {
    double time = trajectory_time_step * static_cast<double>(i);
    double gain_value = std::max(1.0, short_term_gain_cal_func(time));
    extra_gain_for_nudge->at(i) =
        kMinGainValue + (gain_value - kMinGainValue) * smooth_factor;
    if (i == 0) {
      Log2DDS::LogDataV0(
          "extra_gain_for_nudge: ",
          absl::StrCat("task_id ", plan_id, ", i ", i, ", gain_value ",
                       gain_value, ", smooth_factor ", smooth_factor,
                       ", extra_gain ", extra_gain_for_nudge->at(i)));
    }
  }

  return;
}

bool AddFakeObjectCost(
    const std::vector<double>& extra_gain_for_nudge, const bool ref_enhance,
    const int plan_id, std::string obj_id, const ObjectType& obj_type,
    const LaneChangeStage lc_stage, const bool borrow_lane,
    const NudgeInfos& nudge_info, double trajectory_time_step,
    const PathTimeCorridor& path_time_corridor,
    double avoid_dynamic_obj_early_time, std::string_view base_name,
    const std::vector<double>& nudge_buffer, bool consider_mirrors,
    const std::vector<prediction::PredictionObjectState>& states,
    const std::vector<TrajectoryPoint>& init_traj, bool is_stationary,
    const DrivePassage& drive_passage,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    std::string traj_id, double gain, const Vec2d& object_velocity,
    const VehicleGeometryParamsProto& veh_geo_params,
    const int fake_obs_amounts, const int seq_fake_obs,
    const int virtual_obs_idx,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  using ObjectCost = PartitionedObjectCost<Mfob>;
  CHECK_GT(trajectory_time_step, 0.0);

  constexpr double kSafeBuffer = 0.5;
  constexpr double kHighWayTTC = 9.0;
  const int num_points = static_cast<int>(states.size());

  std::vector<double> cascade_gains = {
      cost_weight_params.object_cost_params().object_b_cost_weight(),
      cost_weight_params.object_cost_params().object_a_cost_weight()};

  const double heading_diff_cos = std::cos(
      NormalizeAngle(init_traj.front().theta() - states.front().box.heading()));
  const auto object_frenet_box_output =
      drive_passage.QueryFrenetBoxAtContour(states[0].contour);
  if (!object_frenet_box_output.ok()) return false;
  const auto& object_frenet_box = object_frenet_box_output.value();
  const double object_s = object_frenet_box.center_s();
  const double object_l = object_frenet_box.center_l();
  const Vec2d av_local_dir =
      Vec2d::FastUnitFromAngle(init_traj.front().theta());
  const double obj_v_av_local = object_velocity.dot(av_local_dir);
  const double speed_diff = init_traj.front().v() - obj_v_av_local;
  double ds = std::max(
      object_frenet_box.s_min - veh_geo_params.front_edge_to_center(), 0.0);
  double ttc = ds / (std::abs(speed_diff) + 0.001);

  const auto object_position_info =
      path_time_corridor.QueryObjectPositionInfo(obj_id);
  const Vec2d obj_local_dir =
      Vec2d::FastUnitFromAngle(states.front().box.heading());
  double obj_av_cross = obj_local_dir.CrossProd(av_local_dir);

  const auto& plan_start_frenet_point =
      drive_passage.QueryFrenetCoordinateAt(init_traj.front().pos());
  const auto lane_heading =
      drive_passage.QueryTangentAngleAtS(plan_start_frenet_point->s);
  if (!lane_heading.ok()) {
    return false;
  }
  const Vec2d unit_cur_lane = Vec2d::FastUnitFromAngle(*lane_heading);
  double lane_av_cross = unit_cur_lane.CrossProd(av_local_dir);
  double lane_obj_cross = obj_local_dir.CrossProd(unit_cur_lane);
  const std::string plan_id_and_obj_id =
      absl::StrFormat("opt_task_%d,%s", plan_id, obj_id);

  if (!is_stationary && prediction::IsVulnerableRoadUserType(obj_type) &&
      ttc < 3.0 && heading_diff_cos > 0.0 && std::abs(object_l) < 5.0) {
    if (object_position_info.at(0).position_type ==
            PathTimeCorridor::PositionType::POSITION_RIGHT &&
        obj_av_cross < 0.0 && obj_av_cross > -0.5) {
      double cost_a_vru_wight = std::clamp(
          std::abs(obj_av_cross) * 600.0,
          cost_weight_params.object_cost_params().object_a_cost_weight(),
          300.0);
      cascade_gains = {200.0, cost_a_vru_wight};
    }
    if (object_position_info.at(0).position_type ==
            PathTimeCorridor::PositionType::POSITION_LEFT &&
        obj_av_cross > 0.0 && obj_av_cross < 0.5) {
      double cost_a_vru_wight = std::clamp(
          std::abs(obj_av_cross) * 600.0,
          cost_weight_params.object_cost_params().object_a_cost_weight(),
          300.0);
      cascade_gains = {200.0, cost_a_vru_wight};
    }
  }
  if (!is_stationary &&
      (obj_type == OT_LARGE_VEHICLE || obj_type == OT_VEHICLE) &&
      ttc < kHighWayTTC && heading_diff_cos > 0.0 && std::abs(object_l) < 5.0) {
    if (object_position_info.at(0).position_type ==
            PathTimeCorridor::PositionType::POSITION_RIGHT &&
        obj_av_cross < 0.0 && obj_av_cross > -0.5) {
      double cost_a_car_wight = std::clamp(
          std::abs(obj_av_cross) * 400.0,
          cost_weight_params.object_cost_params().object_a_cost_weight(),
          200.0);
      cascade_gains = {200.0, cost_a_car_wight};
    }
    if (object_position_info.at(0).position_type ==
            PathTimeCorridor::PositionType::POSITION_LEFT &&
        obj_av_cross > 0.0 && obj_av_cross < 0.5) {
      double cost_a_car_wight = std::clamp(
          std::abs(obj_av_cross) * 400.0,
          cost_weight_params.object_cost_params().object_a_cost_weight(),
          200.0);
      cascade_gains = {200.0, cost_a_car_wight};
    }
  }

  if (heading_diff_cos < -0.707 &&
      !prediction::IsVulnerableRoadUserType(obj_type) &&
      object_position_info.at(0).position_type ==
          PathTimeCorridor::PositionType::POSITION_LEFT &&
      ttc < 3.0 && std::abs(object_l) < 5.0) {
    double cost_a_oppsite_wight = std::clamp(
        speed_diff * 4.0 * std::max(1.0, lane_av_cross * 20.0),
        cost_weight_params.object_cost_params().object_a_cost_weight(), 300.0);
    cascade_gains = {
        cost_weight_params.object_cost_params().object_b_cost_weight(),
        cost_a_oppsite_wight};
  }

  google::protobuf::RepeatedPtrField<
      ::st::VehicleCircleModelParamsProto_CircleParams>
      circles = trajectory_optimizer_vehicle_model_params.circles();
  if (consider_mirrors) {
    for (const auto& circle :
         trajectory_optimizer_vehicle_model_params.mirror_circles()) {
      *circles.Add() = circle;
    }
  }

  const int circle_size = circles.size();
  std::vector<double> dists_to_rac;
  std::vector<double> angles_to_axis;
  std::vector<double> circles_radius;
  dists_to_rac.reserve(circle_size);
  angles_to_axis.reserve(circle_size);
  circles_radius.reserve(circle_size);
  double max_model_dist = 0.0;
  for (const auto& circle : circles) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
    circles_radius.push_back(circle.radius());
    max_model_dist = std::max(max_model_dist, circle.dist_to_rac());
  }

  std::vector<ObjectCost::filter> filters;
  std::vector<std::vector<ObjectCost::Object>> objects;
  objects.resize(num_points);
  filters.resize(num_points);

  const PiecewiseConstFunction<double, double> nudge_buffer_time_gain_pcf =
      prediction::IsVulnerableRoadUserType(obj_type)
          ? PiecewiseConstFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .vru_buffer_time_gain_pcf())
          : PiecewiseConstFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .nudge_buffer_time_gain_pcf());

  for (int k = 0; k < num_points; ++k) {
    double max_buffer_plus_radius = 0.0;
    const std::vector<double> nudge_buffers = {kSafeBuffer, nudge_buffer.at(k)};
    auto& objects_k = objects[k];
    Vec2d x;
    Vec2d obj_x;
    Polygon2d contour;
    x = init_traj[k].pos();
    const auto& traj_point = *states[virtual_obs_idx].traj_point;
    obj_x = traj_point.pos();
    contour = states[virtual_obs_idx].contour;

    const double t = static_cast<double>(k) * trajectory_time_step;
    const double gain = is_stationary ? 1.0 : nudge_buffer_time_gain_pcf(t);

    const auto object_station_point = drive_passage.FindNearestStation(obj_x);
    const double heading_point_diff_cos = std::cos(
        NormalizeAngle(init_traj[k].theta() - states[k].box.heading()));
    bool is_oppsite_obj_point = heading_point_diff_cos < -0.707;
    bool is_obj_point_junction = object_station_point.is_in_intersection();
    std::vector<double> gain_point = {1.0, 1.0};
    if (object_station_point.station_info().turn_type ==
            ad_byd::planning::TurnType::RIGHT_TURN &&
        is_obj_point_junction && is_oppsite_obj_point) {
      if (obj_type == OT_LARGE_VEHICLE) {
        gain_point = {10.0, 10.0};
      } else {
        gain_point = {6.0, 6.0};
      }
    }

    for (int idx = 0; idx < circle_size; ++idx) {
      max_buffer_plus_radius = std::max(
          max_buffer_plus_radius,
          *std::max_element(nudge_buffers.begin(), nudge_buffers.end()) +
              circles.at(idx).radius());

      const Vec2d tangent =
          Vec2d::FastUnitFromAngle(init_traj[k].theta() + angles_to_axis[idx]);
      const Vec2d x_center = x + tangent * dists_to_rac[idx];
      std::vector<Segment2d> lines;
      Vec2d ref_x;
      Vec2d ref_tangent;
      double offset = 0.0;
      const double circle_radius = circles_radius[idx];
      CalcPartitionHalfContourInfo(
          x_center, obj_x, contour,
          *std::max_element(nudge_buffers.begin(), nudge_buffers.end()) +
              circle_radius,
          &lines, &ref_x, &ref_tangent, &offset);
      CHECK(!lines.empty());
      std::vector<double> circle_buffers = nudge_buffers;
      for (int i = 0; i < circle_buffers.size(); ++i) {
        circle_buffers[i] = nudge_buffers[i] * gain + circle_radius;
      }
      objects_k.push_back(
          ObjectCost::Object{.lines = lines,
                             .buffers = std::move(circle_buffers),
                             .gains = gain_point,
                             .ref_x = ref_x,
                             .offset = offset,
                             .ref_tangent = ref_tangent,
                             .enable = true});
    }

    Vec2d ref_x = {0.0, 0.0};
    Vec2d ref_tangent = {0.0, 0.0};
    for (const auto& object : objects_k) {
      ref_x += object.ref_x;
      ref_tangent += object.ref_tangent;
    }
    ref_x /= static_cast<double>(objects_k.size());
    ref_tangent /= static_cast<double>(objects_k.size());
    ref_tangent = ref_tangent.normalized();

    Vec2d front, back;
    int front_index, back_index;
    contour.ExtremePoints(ref_tangent, &back_index, &front_index, &back,
                          &front);
    const double filter_offset = max_buffer_plus_radius + max_model_dist;
    const double offset = (front - ref_x).dot(ref_tangent) + filter_offset;

    ObjectCost::filter& filter = filters[k];
    filter.ref_x = std::move(ref_x);
    filter.ref_tangent = std::move(ref_tangent);
    filter.offset = offset;
  }

  std::vector<double> attenuation_cascade_gains = cascade_gains;
  if (lc_stage == LaneChangeStage::LCS_NONE) {
    if (prediction::IsVulnerableRoadUserType(obj_type)) {
      PiecewiseLinearFunction<double> vru_gain_vec_func =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .object_vru_a_cost_weight_attenuation_factor());
      double vru_gain = vru_gain_vec_func(std::fabs(init_traj.front().v()));

      attenuation_cascade_gains[1] *= vru_gain;
    } else {
      PiecewiseLinearFunction<double> dyn_obs_gain_vec_func =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .dynamic_object_a_cost_weight_attenuation_factor());
      double dyn_obs_gain =
          dyn_obs_gain_vec_func(std::fabs(init_traj.front().v()));
      if (is_stationary) {
        const std::vector<double> vec_speed = {5.0, 10.0, 20.0};
        const std::vector<double> vec_stationary_gain = {1.0, 1.2, 1.4};
        const PiecewiseLinearFunction<double> stationary_gain_plf(
            vec_speed, vec_stationary_gain);
        dyn_obs_gain *= stationary_gain_plf(std::fabs(init_traj.front().v()));
      }
      attenuation_cascade_gains[1] *= dyn_obs_gain;

      constexpr double kMaxDeltaSpeed = 0.0;
      constexpr double kMinDeltaSpeed = -3.0;
      if (speed_diff < kMaxDeltaSpeed) {
        PiecewiseLinearFunction<double> fast_obs_gain_vec_func =
            PiecewiseLinearFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .fast_object_a_cost_weight_attenuation_factor());
        double fast_obs_gain =
            fast_obs_gain_vec_func(std::fabs(init_traj.front().v()));
        double transition_factor =
            (speed_diff <= kMinDeltaSpeed) ? 1.0 : speed_diff / kMinDeltaSpeed;

        attenuation_cascade_gains[1] =
            attenuation_cascade_gains[1] * (1.0 - transition_factor) +
            cascade_gains[1] * fast_obs_gain * transition_factor;
      }
    }
  }

  if (ref_enhance && lc_stage == LaneChangeStage::LCS_NONE) {
    PiecewiseLinearFunction<double> weight_gain_for_ref_l_enhance_func =
        is_stationary ? PiecewiseLinearFunctionFromProto(
                            cost_weight_params.object_cost_params()
                                .weight_gain_for_enhance_stationary())
                      : PiecewiseLinearFunctionFromProto(
                            cost_weight_params.object_cost_params()
                                .weight_gain_for_enhance_dynamic());
    double extra_gain_for_enhance =
        weight_gain_for_ref_l_enhance_func(std::fabs(init_traj.front().v()));

    constexpr double kSpeedThr = 20.0 / 3.6;
    if (std::fabs(init_traj.front().v()) < kSpeedThr) {
      const std::vector<double> vec_speed = {4.17, 5.56};
      const std::vector<double> vec_extra_gain_atten_factor = {1.0, 0.0};
      const PiecewiseLinearFunction<double> extra_gain_atten_factor_plf(
          vec_speed, vec_extra_gain_atten_factor);
      double extra_gain_atten_factor =
          extra_gain_atten_factor_plf(std::fabs(init_traj.front().v()));

      const std::vector<double> vec_a = {-1.2, -0.6, 0.0};
      std::vector<double> vec_extra_gain_atten = {0.6, 0.3, 0.0};
      if (is_stationary && obj_type == OT_LARGE_VEHICLE) {
        vec_extra_gain_atten = {0.8, 0.4, 0.0};
      }
      const PiecewiseLinearFunction<double> extra_gain_atten_plf(
          vec_a, vec_extra_gain_atten);
      extra_gain_for_enhance *=
          (1.0 - extra_gain_atten_plf(init_traj.front().a()) *
                     extra_gain_atten_factor);
    }
    extra_gain_for_enhance = std::max(1.0, extra_gain_for_enhance);

    attenuation_cascade_gains[1] *= extra_gain_for_enhance;
  }

  int seq_fake_obstacle = seq_fake_obs;
  std::string fake_obs_id =
      traj_id + "_virtual_" + absl::StrCat(seq_fake_obstacle);
  costs->emplace_back(std::make_unique<ObjectCost>(
      extra_gain_for_nudge, std::move(objects), std::move(filters),
      std::move(dists_to_rac), std::move(angles_to_axis),
      std::move(attenuation_cascade_gains), av_model_helpers.get(),
      /*sub_names=*/std::vector<std::string>({"Inner", "Outer"}),
      /*using_hessian_approximate=*/true,
      absl::StrFormat("Partition AV Object: for %s", fake_obs_id),
      gain * cost_weight_params.object_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  Log2DDS::LogDataV2(
      "early_avoid_obs_infos: ",
      absl::StrCat(plan_id_and_obj_id, ", ", seq_fake_obstacle, ", ",
                   fake_obs_amounts, ", ", virtual_obs_idx, ", ", num_points));

  return true;
}

bool AddPartitionAvObjectCost(
    const std::vector<double>& extra_gain_for_nudge, const int plan_id,
    std::string obj_id, const ObjectType& obj_type, const bool ref_enhance,
    const LaneChangeStage lc_stage, const bool borrow_lane,
    const NudgeInfos& nudge_info, double trajectory_time_step,
    const PathTimeCorridor& path_time_corridor,
    double avoid_dynamic_obj_early_time, std::string_view base_name,
    const std::vector<double>& nudge_buffer, bool consider_mirrors,
    const std::vector<prediction::PredictionObjectState>& states,
    const std::vector<TrajectoryPoint>& init_traj, bool is_stationary,
    const DrivePassage& drive_passage,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    std::string traj_id, double gain, const Vec2d& object_velocity,
    const VehicleGeometryParamsProto& veh_geo_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  using ObjectCost = PartitionedObjectCost<Mfob>;
  CHECK_GT(trajectory_time_step, 0.0);

  double large_veh_hard_buffer = 0.5;
  if (lc_stage == LaneChangeStage::LCS_NONE && !init_traj.empty()) {
    if (obj_type == ObjectType::OT_LARGE_VEHICLE && !borrow_lane) {
      int cur_station_index =
          drive_passage
              .FindNearestStationIndex(Vec2d(init_traj.front().pos().x(),
                                             init_traj.front().pos().y()))
              .value();
      const Station& cur_station =
          drive_passage.station(StationIndex(cur_station_index));
      if (!cur_station.station_info().is_in_intersection) {
        const std::vector<double> vec_speed = {0.0, 1.0, 2.0, 3.0};
        const std::vector<double> vec_extra_buffer = {-0.08, -0.05, 0.0, 0.0};
        const PiecewiseLinearFunction<double> extra_buffer_plf(
            vec_speed, vec_extra_buffer);

        const std::vector<double> vec_a = {-1.2, -0.8, 0.0, 0.4};
        const std::vector<double> vec_extra_buffer_gain = {0.0, 1.0, 1.0, 0.0};
        const PiecewiseLinearFunction<double> extra_buffer_gain_plf(
            vec_a, vec_extra_buffer_gain);

        large_veh_hard_buffer +=
            extra_buffer_gain_plf(init_traj.front().a()) *
            extra_buffer_plf(std::fabs(init_traj.front().v()));
      }
    }
  }

  const double kSafeBuffer = (obj_type == ObjectType::OT_LARGE_VEHICLE)
                                 ? large_veh_hard_buffer
                                 : 0.5;  // m.
  constexpr double kHighWayTTC = 9.0;
  const int num_points = static_cast<int>(states.size());

  std::vector<double> cascade_gains = {
      cost_weight_params.object_cost_params().object_b_cost_weight(),
      cost_weight_params.object_cost_params().object_a_cost_weight()};

  const double heading_diff_cos = std::cos(
      NormalizeAngle(init_traj.front().theta() - states.front().box.heading()));
  const auto object_frenet_box_output =
      drive_passage.QueryFrenetBoxAtContour(states[0].contour);
  if (!object_frenet_box_output.ok()) return false;
  const auto& object_frenet_box = object_frenet_box_output.value();
  const double object_s = object_frenet_box.center_s();
  const double object_l = object_frenet_box.center_l();
  const Vec2d av_local_dir =
      Vec2d::FastUnitFromAngle(init_traj.front().theta());
  const double obj_v_av_local = object_velocity.dot(av_local_dir);
  const double speed_diff = init_traj.front().v() - obj_v_av_local;
  double ds = std::max(
      object_frenet_box.s_min - veh_geo_params.front_edge_to_center(), 0.0);
  double ttc = ds / (std::abs(speed_diff) + 0.001);

  const auto object_position_info =
      path_time_corridor.QueryObjectPositionInfo(obj_id);
  const Vec2d obj_local_dir =
      Vec2d::FastUnitFromAngle(states.front().box.heading());
  double obj_av_cross = obj_local_dir.CrossProd(av_local_dir);

  const auto& plan_start_frenet_point =
      drive_passage.QueryFrenetCoordinateAt(init_traj.front().pos());
  const auto lane_heading =
      drive_passage.QueryTangentAngleAtS(plan_start_frenet_point->s);
  if (!lane_heading.ok()) {
    return false;
  }
  const Vec2d unit_cur_lane = Vec2d::FastUnitFromAngle(*lane_heading);
  double lane_av_cross = unit_cur_lane.CrossProd(av_local_dir);
  double lane_obj_cross = obj_local_dir.CrossProd(unit_cur_lane);
  const std::string plan_id_and_obj_id =
      absl::StrFormat("opt_task_%d,%s", plan_id, obj_id);
  // Log2DDS::LogDataV2(
  //     "heading_diff_cross",
  //     absl::StrCat(plan_id_and_obj_id, ": ", obj_av_cross,
  //                  ", ttc: ", ttc,
  //                  ", cos: ", heading_diff_cos,
  //                  ", ego: ", init_traj.front().theta(),
  //                  ", obj: ", states.front().box.heading()));

  /*********************************************/
  // bool is_ego_ahead_obj =  // oppistie or same dir car
  /**********************************************/
  if (!is_stationary && prediction::IsVulnerableRoadUserType(obj_type) &&
      ttc < 3.0 && heading_diff_cos > 0.0 && std::abs(object_l) < 5.0) {
    if (object_position_info.at(0).position_type ==
            PathTimeCorridor::PositionType::POSITION_RIGHT &&
        obj_av_cross < 0.0 && obj_av_cross > -0.5) {
      double cost_a_vru_wight = std::clamp(
          std::abs(obj_av_cross) * 600.0,
          cost_weight_params.object_cost_params().object_a_cost_weight(),
          300.0);
      cascade_gains = {200.0, cost_a_vru_wight};
      Log2DDS::LogDataV2("heading_diff_cross",
                         absl::StrCat("vru_right, ", plan_id_and_obj_id,
                                      ", cost_a_vru_wight: ", cost_a_vru_wight,
                                      ", obj_av_cross: ", obj_av_cross));
    }
    if (object_position_info.at(0).position_type ==
            PathTimeCorridor::PositionType::POSITION_LEFT &&
        obj_av_cross > 0.0 && obj_av_cross < 0.5) {
      double cost_a_vru_wight = std::clamp(
          std::abs(obj_av_cross) * 600.0,
          cost_weight_params.object_cost_params().object_a_cost_weight(),
          300.0);
      cascade_gains = {200.0, cost_a_vru_wight};
      Log2DDS::LogDataV2("heading_diff_cross",
                         absl::StrCat("vru_left, ", plan_id_and_obj_id,
                                      ", cost_a_vru_wight: ", cost_a_vru_wight,
                                      ", obj_av_cross: ", obj_av_cross));
    }
  }
  if (!is_stationary &&
      (obj_type == OT_LARGE_VEHICLE || obj_type == OT_VEHICLE) &&
      ttc < kHighWayTTC && heading_diff_cos > 0.0 && std::abs(object_l) < 5.0) {
    if (object_position_info.at(0).position_type ==
            PathTimeCorridor::PositionType::POSITION_RIGHT &&
        obj_av_cross < 0.0 && obj_av_cross > -0.5) {
      double cost_a_car_wight = std::clamp(
          std::abs(obj_av_cross) * 400.0,
          cost_weight_params.object_cost_params().object_a_cost_weight(),
          200.0);
      cascade_gains = {200.0, cost_a_car_wight};
      Log2DDS::LogDataV2("heading_diff_cross",
                         absl::StrCat("car_right, ", plan_id_and_obj_id,
                                      ", cost_a_car_wight: ", cost_a_car_wight,
                                      ", obj_av_cross: ", obj_av_cross));
    }
    if (object_position_info.at(0).position_type ==
            PathTimeCorridor::PositionType::POSITION_LEFT &&
        obj_av_cross > 0.0 && obj_av_cross < 0.5) {
      double cost_a_car_wight = std::clamp(
          std::abs(obj_av_cross) * 400.0,
          cost_weight_params.object_cost_params().object_a_cost_weight(),
          200.0);
      cascade_gains = {200.0, cost_a_car_wight};
      Log2DDS::LogDataV2("heading_diff_cross",
                         absl::StrCat("car_left, ", plan_id_and_obj_id,
                                      ", cost_a_car_wight: ", cost_a_car_wight,
                                      ", obj_av_cross: ", obj_av_cross));
    }
  }

  if (heading_diff_cos < -0.707 &&
      !prediction::IsVulnerableRoadUserType(obj_type) &&
      object_position_info.at(0).position_type ==
          PathTimeCorridor::PositionType::POSITION_LEFT &&
      /* (lane_av_cross > 0.0 || lane_obj_cross > 0.0) && */ ttc < 3.0 &&
      std::abs(object_l) < 5.0) {
    double cost_a_oppsite_wight = std::clamp(
        speed_diff * 4.0 * std::max(1.0, lane_av_cross * 20.0),
        // std::max(1.0, std::max(lane_av_cross, lane_obj_cross) * 20.0),
        cost_weight_params.object_cost_params().object_a_cost_weight(), 300.0);
    cascade_gains = {
        cost_weight_params.object_cost_params().object_b_cost_weight(),
        cost_a_oppsite_wight};
    Log2DDS::LogDataV2(
        "heading_diff_cross",
        absl::StrCat("car_left_oppsite, ", plan_id_and_obj_id,
                     ", cost_a_oppsite_wight: ", cost_a_oppsite_wight,
                     ", lane_av_cross: ", lane_av_cross,
                     ", lane_obj_cross: ", lane_obj_cross));
  }

  google::protobuf::RepeatedPtrField<
      ::st::VehicleCircleModelParamsProto_CircleParams>
      circles = trajectory_optimizer_vehicle_model_params.circles();
  if (consider_mirrors) {
    for (const auto& circle :
         trajectory_optimizer_vehicle_model_params.mirror_circles()) {
      *circles.Add() = circle;
    }
  }
  // const std::vector<double> nudge_buffers = {kSafeBuffer, nudge_buffer};

  const int circle_size = circles.size();
  std::vector<double> dists_to_rac;
  std::vector<double> angles_to_axis;
  std::vector<double> circles_radius;
  dists_to_rac.reserve(circle_size);
  angles_to_axis.reserve(circle_size);
  circles_radius.reserve(circle_size);
  // double max_buffer_plus_radius = 0.0;
  double max_model_dist = 0.0;
  for (const auto& circle : circles) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
    circles_radius.push_back(circle.radius());
    // max_buffer_plus_radius =
    //     std::max(max_buffer_plus_radius,
    //              *std::max_element(nudge_buffers.begin(),
    //              nudge_buffers.end()) +
    //                  circle.radius());
    max_model_dist = std::max(max_model_dist, circle.dist_to_rac());
  }
  // const double filter_offset = max_buffer_plus_radius + max_model_dist;

  std::vector<ObjectCost::filter> filters;
  std::vector<std::vector<ObjectCost::Object>> objects;
  objects.resize(num_points);
  filters.resize(num_points);

  const PiecewiseConstFunction<double, double> nudge_buffer_time_gain_pcf =
      prediction::IsVulnerableRoadUserType(obj_type)
          ? PiecewiseConstFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .vru_buffer_time_gain_pcf())
          : PiecewiseConstFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .nudge_buffer_time_gain_pcf());

  struct FakePredictionObjectState {
    Vec2d traj_point;
    Polygon2d contour;
  };
  std::vector<FakePredictionObjectState> fake_states;
  bool is_num_points_add_fake = false;
  bool enable_add_fake_obs_cost_func = false;
  int fake_obj_num = 0;
  const double cal_dis =
      std::max(speed_diff * avoid_dynamic_obj_early_time,
               init_traj.front().v() * avoid_dynamic_obj_early_time);
  const double early_avoid_obj_dis = std::min(30.0, std::max(5.0, cal_dis));
  bool is_early_avoid_obj =
      (!nudge_info.nudgeInfos.empty()) &&
      (nudge_info.nudgeInfos.front().id == std::string(obj_id)) &&
      prediction::IsVulnerableRoadUserType(obj_type);

  const auto& dp_frenet_frame = *drive_passage.frenet_frame();
  const auto init_start_frenet_dp =
      dp_frenet_frame.XYToSL(init_traj.front().pos());
  auto obs_state_init = Vec2d(states[0].traj_point->pos());
  const auto init_start_frenet_dp_obs = dp_frenet_frame.XYToSL(obs_state_init);

  if (is_early_avoid_obj && prediction::IsVulnerableRoadUserType(obj_type) &&
      !cost_weight_params.object_cost_params()
           .enable_lane_keep_early_avoid_vru() &&
      !borrow_lane) {
    is_early_avoid_obj = false;
  }

  constexpr double kMinEarlyAvoidObsSpeed = 0.5;
  if (is_early_avoid_obj && prediction::IsVulnerableRoadUserType(obj_type) &&
      std::fabs(obj_v_av_local) <= kMinEarlyAvoidObsSpeed) {
    is_early_avoid_obj = false;
  }
  if (!cost_weight_params.object_cost_params().enable_early_avoid()) {
    is_early_avoid_obj = false;
  }

  if (is_early_avoid_obj && !borrow_lane) {
    constexpr double kMinAcc = -0.8;
    if (init_traj.front().a() <= kMinAcc) {
      is_early_avoid_obj = false;
    }

    int cur_station_index =
        drive_passage
            .FindNearestStationIndex(
                Vec2d(init_traj.front().pos().x(), init_traj.front().pos().y()))
            .value();
    Station cur_station =
        drive_passage.station(StationIndex(cur_station_index));
    if (cur_station.station_info().is_in_intersection) {
      is_early_avoid_obj = false;
      Log2DDS::LogDataV2("early_avoid_vru_fail: ", "ego is intersection: now");
    }

    constexpr double kForwardTime = 4.0;
    cur_station_index =
        drive_passage
            .FindNearestStationIndex(Vec2d(
                init_traj.front().pos().x() +
                    init_traj.front().v() *
                        std::cos(init_traj.front().theta()) * kForwardTime,
                init_traj.front().pos().y() +
                    init_traj.front().v() *
                        std::sin(init_traj.front().theta()) * kForwardTime))
            .value();
    cur_station = drive_passage.station(StationIndex(cur_station_index));
    if (cur_station.station_info().is_in_intersection) {
      is_early_avoid_obj = false;
      Log2DDS::LogDataV2("early_avoid_vru_fail: ",
                         "ego is intersection: backward");
    }

    cur_station_index =
        drive_passage
            .FindNearestStationIndex(Vec2d(
                init_traj.front().pos().x() -
                    init_traj.front().v() *
                        std::cos(init_traj.front().theta()) * kForwardTime,
                init_traj.front().pos().y() -
                    init_traj.front().v() *
                        std::sin(init_traj.front().theta()) * kForwardTime))
            .value();
    cur_station = drive_passage.station(StationIndex(cur_station_index));
    if (cur_station.station_info().is_in_intersection) {
      is_early_avoid_obj = false;
      Log2DDS::LogDataV2("early_avoid_vru_fail: ",
                         "ego is intersection: forward");
    }

    cur_station_index =
        drive_passage
            .FindNearestStationIndex(Vec2d(states[0].traj_point->pos().x(),
                                           states[0].traj_point->pos().y()))
            .value();
    cur_station = drive_passage.station(StationIndex(cur_station_index));
    if (cur_station.station_info().is_in_intersection) {
      is_early_avoid_obj = false;
      Log2DDS::LogDataV2("early_avoid_vru_fail: ", "vru is intersection: now");
    }

    cur_station_index =
        drive_passage
            .FindNearestStationIndex(Vec2d(
                states[0].traj_point->pos().x() +
                    states[0].traj_point->v() *
                        std::cos(states[0].traj_point->theta()) * kForwardTime,
                states[0].traj_point->pos().y() +
                    states[0].traj_point->v() *
                        std::sin(states[0].traj_point->theta()) * kForwardTime))
            .value();
    cur_station = drive_passage.station(StationIndex(cur_station_index));
    if (cur_station.station_info().is_in_intersection) {
      is_early_avoid_obj = false;
      Log2DDS::LogDataV2("early_avoid_vru_fail: ",
                         "vru is intersection: backward");
    }

    cur_station_index =
        drive_passage
            .FindNearestStationIndex(Vec2d(
                states[0].traj_point->pos().x() -
                    states[0].traj_point->v() *
                        std::cos(states[0].traj_point->theta()) * kForwardTime,
                states[0].traj_point->pos().y() -
                    states[0].traj_point->v() *
                        std::sin(states[0].traj_point->theta()) * kForwardTime))
            .value();
    cur_station = drive_passage.station(StationIndex(cur_station_index));
    if (cur_station.station_info().is_in_intersection) {
      is_early_avoid_obj = false;
      Log2DDS::LogDataV2("early_avoid_vru_fail: ",
                         "vru is intersection: forward");
    }

    constexpr double kMinRelDis = 6.0;
    if (init_start_frenet_dp.s + kMinRelDis >= init_start_frenet_dp_obs.s) {
      is_early_avoid_obj = false;
      Log2DDS::LogDataV2("early_avoid_vru_fail: ", "ego is too close to vru");
    }

    constexpr int kMinNumPoints = 15;
    if (num_points <= kMinNumPoints) {
      is_early_avoid_obj = false;
      Log2DDS::LogDataV2("early_avoid_vru_fail: ",
                         "vru's prediction traj is too short");
    }
  }
  Log2DDS::LogDataV2(
      "early_avoid_obj_infos: ",
      absl::StrCat("plan_id: ", plan_id, "id: ", obj_id, ", type: ", obj_type,
                   ", dis: ", early_avoid_obj_dis, ", obs_s: ", object_s,
                   ", is_early_avoid: ", is_early_avoid_obj,
                   ", nudge_info: ", !nudge_info.nudgeInfos.empty(),
                   ", is_vru: ", prediction::IsVulnerableRoadUserType(obj_type),
                   ", num_points: ", num_points,
                   ", ego_start_s: ", init_start_frenet_dp.s,
                   ", obs_start_s: ", init_start_frenet_dp_obs.s,
                   ", ego_start_x: ", init_traj.front().pos().x(),
                   ", ego_start_y: ", init_traj.front().pos().y(),
                   ", obs_start_x: ", states[0].traj_point->pos().x(),
                   ", obs_start_y: ", states[0].traj_point->pos().y(),
                   ", diff_theta_cos: ", heading_diff_cos,
                   ", speed_diff: ", speed_diff));

  if (is_early_avoid_obj && lc_stage == LaneChangeStage::LCS_NONE &&
      speed_diff > 0.5 && heading_diff_cos > 0.707 && obj_v_av_local > 1.0 &&
      object_s - object_frenet_box.length() / 2.0 >
          veh_geo_params.front_edge_to_center() &&
      object_s - object_frenet_box.length() / 2.0 <
          early_avoid_obj_dis + veh_geo_params.front_edge_to_center()) {
    fake_obj_num = num_points;
    fake_states.resize(fake_obj_num);
    int z = 0;
    bool nudge_left = false;
    bool nudge_right = false;
    if (!nudge_info.nudgeInfos.empty()) {
      nudge_left = nudge_info.nudgeInfos.front().direction == 1 ? true : false;
      nudge_right = nudge_info.nudgeInfos.front().direction == 2 ? true : false;
    }
    double min_l = object_frenet_box.l_min;
    double max_l = object_frenet_box.l_max;
    if (nudge_right) {
      min_l = std::max(min_l, 1.7);
      max_l = min_l + object_frenet_box.width();
    }
    if (nudge_left) {
      max_l = std::min(max_l, -1.7);
      min_l = max_l - object_frenet_box.width();
    }
    for (; z < fake_obj_num; ++z) {
      auto& fake_state = fake_states[z];
      double s_at_least_bigger_ego_back = 0.0 + z * init_traj.front().v() * 0.2;
      double s_at_least_bigger_ego_front =
          s_at_least_bigger_ego_back + object_frenet_box.length();

      if (s_at_least_bigger_ego_front >
          object_s - object_frenet_box.length() / 2.0) {
        break;
      }

      const auto fake_obj_front_right_point =
          drive_passage.QueryPointXYAtSL(s_at_least_bigger_ego_front, min_l);
      const auto fake_obj_back_right_point =
          drive_passage.QueryPointXYAtSL(s_at_least_bigger_ego_back, min_l);
      const auto fake_obj_front_left_point =
          drive_passage.QueryPointXYAtSL(s_at_least_bigger_ego_front, max_l);
      const auto fake_obj_back_left_point =
          drive_passage.QueryPointXYAtSL(s_at_least_bigger_ego_back, max_l);
      std::vector<Vec2d> fake_obj_points = {fake_obj_front_left_point.value(),
                                            fake_obj_back_left_point.value(),
                                            fake_obj_back_right_point.value(),
                                            fake_obj_front_right_point.value()};
      fake_state.contour = Polygon2d(fake_obj_points);

      fake_state.traj_point = {(fake_obj_front_left_point.value().x() +
                                fake_obj_back_right_point.value().x()) /
                                   2.0,
                               (fake_obj_front_left_point.value().y() +
                                fake_obj_back_right_point.value().y()) /
                                   2.0};
    }
    if (z > 0) {
      fake_obj_num = z;
      is_num_points_add_fake = false;
      enable_add_fake_obs_cost_func = true;
      const std::string early_avoid_obj = absl::StrFormat(
          "%s,fake_obj_num:%d,calc:%f,early_avoid_obj_dis:%f,"
          "object_l:%f",
          plan_id_and_obj_id, fake_obj_num, cal_dis, early_avoid_obj_dis,
          object_l);
      Log2DDS::LogDataV2("early_avoid_obj_debug", early_avoid_obj);
    }
  }

  for (int k = 0; k < num_points; ++k) {
    double max_buffer_plus_radius = 0.0;
    const std::vector<double> nudge_buffers = {kSafeBuffer, nudge_buffer.at(k)};
    auto& objects_k = objects[k];
    Vec2d x;
    Vec2d obj_x;
    Polygon2d contour;
    if (k < fake_obj_num && is_num_points_add_fake) {
      x = init_traj[k].pos();
      obj_x = fake_states[k].traj_point;
      contour = fake_states[k].contour;
    } else {
      x = init_traj[k].pos();
      const auto& traj_point = *states[k].traj_point;
      obj_x = traj_point.pos();
      contour = states[k].contour;
    }
    const double t = static_cast<double>(k) * trajectory_time_step;

    const double gain = is_stationary ? 1.0 : nudge_buffer_time_gain_pcf(t);

    const auto object_station_point = drive_passage.FindNearestStation(obj_x);
    const double heading_point_diff_cos = std::cos(
        NormalizeAngle(init_traj[k].theta() - states[k].box.heading()));
    bool is_oppsite_obj_point = heading_point_diff_cos < -0.707;
    bool is_obj_point_junction = object_station_point.is_in_intersection();
    std::vector<double> gain_point = {1.0, 1.0};
    if (object_station_point.station_info().turn_type ==
            ad_byd::planning::TurnType::RIGHT_TURN &&
        is_obj_point_junction && is_oppsite_obj_point) {
      Log2DDS::LogDataV2(
          "heading_diff_cross, ",
          absl::StrCat(plan_id_and_obj_id,
                       "RIGHT_TURN_JUNCTION change gains at index ", k));
      if (obj_type == OT_LARGE_VEHICLE) {
        gain_point = {10.0, 10.0};
      } else {
        gain_point = {6.0, 6.0};
      }
    }

    for (int idx = 0; idx < circle_size; ++idx) {
      max_buffer_plus_radius = std::max(
          max_buffer_plus_radius,
          *std::max_element(nudge_buffers.begin(), nudge_buffers.end()) +
              circles.at(idx).radius());
      // max_model_dist = std::max(max_model_dist,
      // circles.at(idx).dist_to_rac());

      const Vec2d tangent =
          Vec2d::FastUnitFromAngle(init_traj[k].theta() + angles_to_axis[idx]);
      const Vec2d x_center = x + tangent * dists_to_rac[idx];
      std::vector<Segment2d> lines;
      Vec2d ref_x;
      Vec2d ref_tangent;
      double offset = 0.0;
      const double circle_radius = circles_radius[idx];
      CalcPartitionHalfContourInfo(
          x_center, obj_x, contour,
          *std::max_element(nudge_buffers.begin(), nudge_buffers.end()) +
              circle_radius,
          &lines, &ref_x, &ref_tangent, &offset);
      CHECK(!lines.empty());
      std::vector<double> circle_buffers = nudge_buffers;
      for (int i = 0; i < circle_buffers.size(); ++i) {
        circle_buffers[i] = nudge_buffers[i] * gain + circle_radius;
      }
      objects_k.push_back(
          ObjectCost::Object{.lines = lines,
                             .buffers = std::move(circle_buffers),
                             .gains = gain_point,
                             .ref_x = ref_x,
                             .offset = offset,
                             .ref_tangent = ref_tangent,
                             .enable = true});
    }

    // Update filter.
    Vec2d ref_x = {0.0, 0.0};
    Vec2d ref_tangent = {0.0, 0.0};
    for (const auto& object : objects_k) {
      ref_x += object.ref_x;
      ref_tangent += object.ref_tangent;
    }
    ref_x /= static_cast<double>(objects_k.size());
    ref_tangent /= static_cast<double>(objects_k.size());
    ref_tangent = ref_tangent.normalized();

    Vec2d front, back;
    int front_index, back_index;
    contour.ExtremePoints(ref_tangent, &back_index, &front_index, &back,
                          &front);
    const double filter_offset = max_buffer_plus_radius + max_model_dist;
    const double offset = (front - ref_x).dot(ref_tangent) + filter_offset;

    ObjectCost::filter& filter = filters[k];
    filter.ref_x = std::move(ref_x);
    filter.ref_tangent = std::move(ref_tangent);
    filter.offset = offset;
  }

  std::vector<double> attenuation_cascade_gains = cascade_gains;
  if (lc_stage == LaneChangeStage::LCS_NONE) {
    if (prediction::IsVulnerableRoadUserType(obj_type)) {
      PiecewiseLinearFunction<double> vru_gain_vec_func =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .object_vru_a_cost_weight_attenuation_factor());
      double vru_gain = vru_gain_vec_func(std::fabs(init_traj.front().v()));

      attenuation_cascade_gains[1] *= vru_gain;

      if (!borrow_lane) {
        constexpr double kMaxDeltaSpeed = -0.5;
        constexpr double kMinDeltaSpeed = -1.5;
        if (speed_diff < kMaxDeltaSpeed) {
          // delta_v: atten_factor && smooth
          const std::vector<double> vec_ego_speed = {0.0, 4.17, 5.56, 6.94};
          std::vector<double> vec_fast_vru_a_cost_weight_atten_factor = {
              0.95, 0.85, 0.50, 0.0};
          std::vector<double> vec_fast_vru_b_cost_weight_atten_factor = {
              0.75, 0.65, 0.40, 0.0};
          // intersection_or_road: atten_factor
          int cur_station_index =
              drive_passage
                  .FindNearestStationIndex(Vec2d(init_traj.front().pos().x(),
                                                 init_traj.front().pos().y()))
                  .value();
          const Station& cur_station =
              drive_passage.station(StationIndex(cur_station_index));
          if (cur_station.station_info().is_in_intersection) {
            vec_fast_vru_a_cost_weight_atten_factor = {0.80, 0.40, 0.20, 0.0};
            vec_fast_vru_b_cost_weight_atten_factor = {0.70, 0.35, 0.15, 0.0};
          }
          const PiecewiseLinearFunction<double> fast_vru_gain_a_vec_func(
              vec_ego_speed, vec_fast_vru_a_cost_weight_atten_factor);
          const PiecewiseLinearFunction<double> fast_vru_gain_b_vec_func(
              vec_ego_speed, vec_fast_vru_b_cost_weight_atten_factor);
          double fast_vru_gain_a_atten =
              fast_vru_gain_a_vec_func(init_traj.front().v());
          double fast_vru_gain_b_atten =
              fast_vru_gain_b_vec_func(init_traj.front().v());
          double transition_factor =
              (speed_diff <= kMinDeltaSpeed)
                  ? 1.0
                  : std::pow((speed_diff - kMaxDeltaSpeed) /
                                 (kMinDeltaSpeed - kMaxDeltaSpeed),
                             0.2);

          // rel_dis: smooth
          const double dis_ego2vru =
              init_start_frenet_dp.s - init_start_frenet_dp_obs.s;
          const std::vector<double> vec_rel_dis = {-3.0, -1.5, 0.0};
          const std::vector<double> vec_extra_atten_for_rel_dis = {0.2, 0.8,
                                                                   1.0};
          const PiecewiseLinearFunction<double> extra_gain_smooth_plf(
              vec_rel_dis, vec_extra_atten_for_rel_dis);
          double gain_atten_for_rel_dis =
              std::pow(extra_gain_smooth_plf(dis_ego2vru), 0.12);
          attenuation_cascade_gains[1] =
              attenuation_cascade_gains[1] *
              (1.0 - fast_vru_gain_a_atten * gain_atten_for_rel_dis *
                         transition_factor);
          attenuation_cascade_gains[0] =
              attenuation_cascade_gains[0] *
              (1.0 - fast_vru_gain_b_atten * gain_atten_for_rel_dis *
                         transition_factor);

          // delta_theta: smooth
          constexpr double kMinCosDeltaTheta = 0.0;
          constexpr double kMaxCosDeltaTheta = 0.5;
          double smooth_factor_for_delta_theta = 1.0;
          if (heading_diff_cos <= kMinCosDeltaTheta) {
            smooth_factor_for_delta_theta = 0.0;
          } else if (heading_diff_cos <= kMaxCosDeltaTheta) {
            smooth_factor_for_delta_theta =
                (heading_diff_cos - kMinCosDeltaTheta) /
                (kMaxCosDeltaTheta - kMinCosDeltaTheta);
          }
          attenuation_cascade_gains[1] =
              attenuation_cascade_gains[1] * smooth_factor_for_delta_theta +
              cascade_gains[1] * (1.0 - smooth_factor_for_delta_theta);
          attenuation_cascade_gains[0] =
              attenuation_cascade_gains[0] * smooth_factor_for_delta_theta +
              cascade_gains[0] * (1.0 - smooth_factor_for_delta_theta);

          // vru_v: smooth
          constexpr double kMinVruSpeed = 1.2;
          constexpr double kMaxVruSpeed = 2.5;
          const double vru_v = std::fabs(obj_v_av_local);
          double smooth_factor_for_vru_v = 1.0;
          if (vru_v <= kMinVruSpeed) {
            smooth_factor_for_vru_v = 0.0;
          } else if (vru_v <= kMaxVruSpeed) {
            smooth_factor_for_vru_v =
                (vru_v - kMinVruSpeed) / (kMaxVruSpeed - kMinVruSpeed);
          }
          attenuation_cascade_gains[1] =
              attenuation_cascade_gains[1] * smooth_factor_for_vru_v +
              cascade_gains[1] * (1.0 - smooth_factor_for_vru_v);
          attenuation_cascade_gains[0] =
              attenuation_cascade_gains[0] * smooth_factor_for_vru_v +
              cascade_gains[0] * (1.0 - smooth_factor_for_vru_v);

          // value limit
          constexpr double kMinValueFactor = 0.1;
          attenuation_cascade_gains[1] = std::max(
              cascade_gains[1] * kMinValueFactor, attenuation_cascade_gains[1]);
          attenuation_cascade_gains[0] = std::max(
              cascade_gains[0] * kMinValueFactor, attenuation_cascade_gains[0]);

          Log2DDS::LogDataV2(
              "avoid_fast_vru_infos: ",
              absl::StrCat(
                  "plan_id: ", plan_id, "id: ", obj_id, ", type: ", obj_type,
                  ", num_points: ", num_points,
                  ", final_gain_a: ", attenuation_cascade_gains[1],
                  ", final_gain_b: ", attenuation_cascade_gains[0],
                  ", init_gain_a: ", cascade_gains[1],
                  ", init_gain_b: ", cascade_gains[0],
                  ", fast_vru_gain_a_atten: ", fast_vru_gain_a_atten,
                  ", fast_vru_gain_b_atten: ", fast_vru_gain_b_atten,
                  ", ego_v: ", init_traj.front().v(),
                  ", vru_v: ", obj_v_av_local, ", delta_v_smooth_factor: ",
                  transition_factor, ", ego_start_s: ", init_start_frenet_dp.s,
                  ", obs_start_s: ", init_start_frenet_dp_obs.s,
                  ", gain_atten_for_rel_dis: ", gain_atten_for_rel_dis,
                  ", smooth_factor_for_delta_theta: ",
                  smooth_factor_for_delta_theta,
                  ", smooth_factor_for_vru_v: ", smooth_factor_for_vru_v));
        }
      }
    } else {
      PiecewiseLinearFunction<double> dyn_obs_gain_vec_func =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .dynamic_object_a_cost_weight_attenuation_factor());
      double dyn_obs_gain =
          dyn_obs_gain_vec_func(std::fabs(init_traj.front().v()));
      if (is_stationary) {
        const std::vector<double> vec_speed = {5.0, 10.0, 20.0};
        const std::vector<double> vec_stationary_gain = {1.0, 1.2, 1.4};
        const PiecewiseLinearFunction<double> stationary_gain_plf(
            vec_speed, vec_stationary_gain);
        dyn_obs_gain *= stationary_gain_plf(std::fabs(init_traj.front().v()));
      }
      attenuation_cascade_gains[1] *= dyn_obs_gain;

      constexpr double kMaxDeltaSpeed = 0.0;
      constexpr double kMinDeltaSpeed = -3.0;
      if (speed_diff < kMaxDeltaSpeed) {
        PiecewiseLinearFunction<double> fast_obs_gain_vec_func =
            PiecewiseLinearFunctionFromProto(
                cost_weight_params.object_cost_params()
                    .fast_object_a_cost_weight_attenuation_factor());
        double fast_obs_gain =
            fast_obs_gain_vec_func(std::fabs(init_traj.front().v()));
        double transition_factor =
            (speed_diff <= kMinDeltaSpeed) ? 1.0 : speed_diff / kMinDeltaSpeed;

        attenuation_cascade_gains[1] =
            attenuation_cascade_gains[1] * (1.0 - transition_factor) +
            cascade_gains[1] * fast_obs_gain * transition_factor;
      }
    }
  }

  std::vector<double> extra_gain_for_nudge_new = extra_gain_for_nudge;
  if (is_stationary || (obj_type != ObjectType::OT_LARGE_VEHICLE &&
                        obj_type != ObjectType::OT_VEHICLE)) {
    for (int i = 0; i < extra_gain_for_nudge_new.size(); ++i) {
      extra_gain_for_nudge_new[i] = 1.0;
    }
  } else {
    const std::vector<double> vec_speed = {0.0, 2.5, 3.5, 18.0, 22.0, 33.3};
    const std::vector<double> vec_the = {1.0, 1.0, 3.0, 3.0, 1.5, 1.0};
    const PiecewiseLinearFunction<double> the_plf(vec_speed, vec_the);
    double the = the_plf(std::fabs(init_traj.front().v()));

    const std::vector<double> vec_a = {-2.0, -0.5, -0.2, 0.5};
    const std::vector<double> vec_gain = {0.0, 0.0, 1.0, 1.0};
    const PiecewiseLinearFunction<double> gain_plf(vec_a, vec_gain);
    double gain = gain_plf(std::fabs(init_traj.front().v()));

    for (int i = 0; i < extra_gain_for_nudge_new.size(); ++i) {
      extra_gain_for_nudge_new[i] =
          1.0 + (extra_gain_for_nudge[i] - 1.0) * gain;
      extra_gain_for_nudge_new[i] =
          std::max(1.0, std::min(the, extra_gain_for_nudge_new[i]));
    }
  }

  if (ref_enhance && lc_stage == LaneChangeStage::LCS_NONE) {
    PiecewiseLinearFunction<double> weight_gain_for_ref_l_enhance_func =
        is_stationary ? PiecewiseLinearFunctionFromProto(
                            cost_weight_params.object_cost_params()
                                .weight_gain_for_enhance_stationary())
                      : PiecewiseLinearFunctionFromProto(
                            cost_weight_params.object_cost_params()
                                .weight_gain_for_enhance_dynamic());
    double extra_gain_for_enhance =
        weight_gain_for_ref_l_enhance_func(std::fabs(init_traj.front().v()));

    constexpr double kSpeedThr = 20.0 / 3.6;
    if (std::fabs(init_traj.front().v()) < kSpeedThr) {
      const std::vector<double> vec_speed = {4.17, 5.56};
      const std::vector<double> vec_extra_gain_atten_factor = {1.0, 0.0};
      const PiecewiseLinearFunction<double> extra_gain_atten_factor_plf(
          vec_speed, vec_extra_gain_atten_factor);
      double extra_gain_atten_factor =
          extra_gain_atten_factor_plf(std::fabs(init_traj.front().v()));

      const std::vector<double> vec_a = {-1.2, -0.6, 0.0};
      std::vector<double> vec_extra_gain_atten = {0.6, 0.3, 0.0};
      if (is_stationary && obj_type == OT_LARGE_VEHICLE) {
        vec_extra_gain_atten = {0.8, 0.4, 0.0};
      }
      const PiecewiseLinearFunction<double> extra_gain_atten_plf(
          vec_a, vec_extra_gain_atten);
      extra_gain_for_enhance *=
          (1.0 - extra_gain_atten_plf(init_traj.front().a()) *
                     extra_gain_atten_factor);
    }
    extra_gain_for_enhance = std::max(1.0, extra_gain_for_enhance);

    attenuation_cascade_gains[1] *= extra_gain_for_enhance;
  }

  costs->emplace_back(std::make_unique<ObjectCost>(
      extra_gain_for_nudge, std::move(objects), std::move(filters),
      std::move(dists_to_rac), std::move(angles_to_axis),
      std::move(attenuation_cascade_gains), av_model_helpers.get(),
      /*sub_names=*/std::vector<std::string>({"Inner", "Outer"}),
      /*using_hessian_approximate=*/true,
      absl::StrFormat("Partition AV Object: for %s", traj_id),
      gain * cost_weight_params.object_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));

  if (enable_add_fake_obs_cost_func) {
    int kMaxFakeObsAmount = 2;
    constexpr double kMaxDeltaTime = 3.0;
    double first_vir_obs_s =
        (init_start_frenet_dp.s +
         init_traj.front().v() * cost_weight_params.object_cost_params()
                                     .virtual_obs_early_avoid_time());

    constexpr double kMaxTTC = 4.6;
    double time_to_collision = std::min(
        kMaxTTC,
        (init_start_frenet_dp_obs.s - init_start_frenet_dp.s) / speed_diff);
    const std::vector<double> vec_speed = {2.0, 4.0, 8.0};
    const std::vector<double> vec_forward_time_coff = {0.5, 0.5, 0.45};
    const PiecewiseLinearFunction<double> forward_time_coff_plf(
        vec_speed, vec_forward_time_coff);
    const std::vector<double> vec_acc = {-0.8, -0.4};
    const std::vector<double> vec_forward_time_atten = {0.0, 1.0};
    const PiecewiseLinearFunction<double> forward_time_atten_plf(
        vec_acc, vec_forward_time_atten);
    double forward_time =
        forward_time_coff_plf(std::fabs(init_traj.front().v())) *
        forward_time_atten_plf(init_traj.front().a()) *
        cost_weight_params.object_cost_params().virtual_obs_early_avoid_time();

    int kSizeSteps =
        init_traj.size() > num_points ? num_points : init_traj.size();
    constexpr double kRelDis = 6.0;
    for (int i = 0; i < kSizeSteps; ++i) {
      const auto& dp_frenet_frame = *drive_passage.frenet_frame();
      const auto ego_s = dp_frenet_frame.XYToSL(init_traj[i].pos());
      auto obs_state = Vec2d(states[i].traj_point->pos());
      const auto obs_s = dp_frenet_frame.XYToSL(obs_state);
      if (ego_s.s + kRelDis >= obs_s.s ||
          init_traj[i].t() >= time_to_collision) {
        time_to_collision = init_traj[i].t();
        first_vir_obs_s =
            std::max(init_start_frenet_dp_obs.s,
                     obs_s.s - states[i].traj_point->v() * forward_time);

        Log2DDS::LogDataV2(
            "ego_vru_collision_infos: ",
            absl::StrCat(
                "plan_id: ", plan_id, "id: ", obj_id,
                ", is_lane_borrow: ", borrow_lane, ", i: ", i,
                ", init_traj[i].t(): ", init_traj[i].t(),
                ", first_vir_obs_s: ", first_vir_obs_s,
                ", init_start_frenet_dp_obs.s: ", init_start_frenet_dp_obs.s,
                ", obs_s.s: ", obs_s.s, ", states[i].traj_point->v(): ",
                states[i].traj_point->v(), ", forward_time: ", forward_time));

        break;
      }
    }

    int first_vir_obs_idx = num_points - 1;
    for (int i = first_vir_obs_idx; i >= 0; --i) {
      const auto& dp_frenet_frame = *drive_passage.frenet_frame();
      auto obs_state = Vec2d(states[i].traj_point->pos());
      const auto obs_cur_s = dp_frenet_frame.XYToSL(obs_state);
      if (obs_cur_s.s < first_vir_obs_s) {
        break;
      }
      first_vir_obs_idx = i;
    }
    constexpr int kMinFirstVirObsIdx = 8;
    first_vir_obs_idx = std::max(kMinFirstVirObsIdx, first_vir_obs_idx);
    Log2DDS::LogDataV2(
        "early_avoid_vru_infos: ",
        absl::StrCat(
            "plan_id: ", plan_id, "id: ", obj_id,
            ", is_lane_borrow: ", borrow_lane,
            ", is_vru: ", prediction::IsVulnerableRoadUserType(obj_type),
            ", time_to_collision: ", time_to_collision,
            ", ego_v: ", std::fabs(init_traj.front().v()),
            ", ego_a: ", std::fabs(init_traj.front().a()),
            ", obs_v: ", obj_v_av_local, ", forward_time: ", forward_time,
            ", first_vir_obs_s: ", first_vir_obs_s,
            ", first_vir_obs_idx: ", first_vir_obs_idx));

    constexpr int kMaxDeltaStep = 5;
    int virtual_obs_idx = 0;
    for (int i = 0; i < kMaxFakeObsAmount; ++i) {
      if (num_points <= 0) break;
      int virtual_obs_time_steps =
          first_vir_obs_idx +
          static_cast<int>((kMaxDeltaTime * static_cast<double>(i)) /
                           trajectory_time_step);
      if (virtual_obs_time_steps + kMaxDeltaStep >= num_points) {
        break;
      }
      virtual_obs_idx =
          std::max(0, std::min(virtual_obs_time_steps,
                               static_cast<int>(states.size()) - 1));

      AddFakeObjectCost(
          extra_gain_for_nudge_new, ref_enhance, plan_id, obj_id, obj_type,
          lc_stage, borrow_lane, nudge_info, trajectory_time_step,
          path_time_corridor, avoid_dynamic_obj_early_time, base_name,
          nudge_buffer, consider_mirrors, states, init_traj, is_stationary,
          drive_passage, cost_weight_params,
          trajectory_optimizer_vehicle_model_params, av_model_helpers, traj_id,
          gain, object_velocity, veh_geo_params, kMaxFakeObsAmount, i,
          virtual_obs_idx, costs);
    }
  }

  return true;
}

bool AddUnidirectionalObjectCost(
    double trajectory_time_step, std::string_view base_name,
    const std::vector<prediction::PredictionObjectState>& states,
    const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    std::string traj_id, double gain, std::vector<LeadingInfo>* leading_min_s,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  CHECK_GT(trajectory_time_step, 0.0);

  CHECK_GE(leading_min_s->size(), states.size());

  using ObjectCost = UnidirectionalObjectCost<Mfob>;
  const auto& object_cost_params = cost_weight_params.object_cost_params();

  const double follow_buffer = object_cost_params.acc_standstill_standoff();
  const double half_width = 0.5 * veh_geo_params.width();
  const std::vector<double> buffers = {
      follow_buffer + half_width,
      object_cost_params.acc_safe_standstill_standoff() + half_width};

  std::vector<ObjectCost::Object> objects;
  const int num_points = states.size();
  objects.reserve(num_points);

  double t_range = kSpacetimePlannerTrajectoryHorizon;
  const Vec2d x = init_traj.front().pos();
  const Vec2d tangent = Vec2d::FastUnitFromAngle(init_traj.front().theta());
  const double dist_to_rac = veh_geo_params.front_edge_to_center() - half_width;
  const Vec2d circle_center = x + tangent * dist_to_rac;

  const auto& leading_object_proto =
      FindOrNull(leading_trajs, std::string(traj_id));
  if (leading_object_proto != nullptr &&
      !leading_object_proto->st_constraints().empty()) {
    const int st_constraints_length =
        leading_object_proto->st_constraints_size();
    t_range =
        leading_object_proto->st_constraints(st_constraints_length - 1).t();
  }

  const double plan_start_point_v = init_traj.front().v();
  const auto plan_start_point_sl_or =
      drive_passage.QueryFrenetCoordinateAt(circle_center);
  if (!plan_start_point_sl_or.ok()) {
    return false;
  }
  const double plan_start_point_s_on_drive_passage = plan_start_point_sl_or->s;
  double max_deceleration = motion_constraint_params.max_deceleration();
  // leading object
  if (!leading_object_proto->modified_trajectory().empty()) {
    const double object_a = leading_object_proto->modified_trajectory(0).a();
    if (object_a < 0.0) {
      max_deceleration = std::max(5.0 * (object_a - 1.0), -10.0);
    }
  }
  // Time of slowing down to zero. Assume that v > 0, so time shouldn't be
  // smaller than zero.
  const double slow_down_zero_time =
      std::max(0.0, plan_start_point_v / -max_deceleration);
  constexpr double kPenetrationOffset = 1.0;  // m.

  for (int k = 0; k < states.size(); ++k) {
    const auto& traj_point = *states[k].traj_point;
    if (traj_point.t() > t_range) {
      break;
    }
    const Polygon2d& contour = states[k].contour;

    const double t = std::min(static_cast<double>(k) * trajectory_time_step,
                              slow_down_zero_time);
    // Compute s that if av slow down with max deceleration from start s to now,
    // leading s should not be smaller than it to avoid abnormal braking traj.
    const double leading_min_s_on_drive_passage =
        plan_start_point_s_on_drive_passage + plan_start_point_v * t +
        0.5 * max_deceleration * Sqr(t) + follow_buffer + half_width -
        kPenetrationOffset;

    const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
    if (!frenet_box_or.ok()) {
      break;
    }

    const auto& frenet_box = *frenet_box_or;
    if (frenet_box.s_min > path_boundary.end_s()) {
      break;
    }

    const double leading_cost_s =
        std::max(leading_min_s_on_drive_passage, frenet_box.s_min);
    (*leading_min_s)[k] = LeadingInfo{.s = leading_cost_s, .v = traj_point.v()};

    const auto [right_boundary_point, left_boundary_point] =
        path_boundary.QueryBoundaryXY(leading_cost_s);
    Segment2d mid_line(left_boundary_point, right_boundary_point);

    objects.push_back(ObjectCost::Object{
        .dir = -mid_line.unit_direction().Perp(),
        .ref = (mid_line.start() + mid_line.end()) * 0.5,
        .lateral_extent = mid_line.length() * 0.5,
        .buffers = buffers,
        .gains = {object_cost_params.leading_object_a_cost_weight(),
                  object_cost_params.leading_object_b_cost_weight()},
        .enable = true});
  }

  std::vector<double> dist_to_rac_vec = {dist_to_rac};
  std::vector<double> angle_to_axis_vec = {0.0};

  costs->emplace_back(std::make_unique<ObjectCost>(
      std::move(objects), std::move(dist_to_rac_vec),
      std::move(angle_to_axis_vec),
      /*sub_names=*/std::vector<std::string>({SoftNameString, HardNameString}),
      /*using_hessian_approximate=*/true,
      absl::StrFormat("Leading Object (F): for %s", traj_id),
      gain * cost_weight_params.object_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  return true;
}

void GetClosestLeadingObjectInfo(
    int trajectory_steps, double trajectory_time_step,
    const DrivePassage& drive_passage,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    absl::Span<const SpacetimeObjectTrajectory* const> spacetime_trajs,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::optional<
        std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>*
        stationary_closest_leading_object_info,
    std::optional<
        std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>*
        moving_closest_leading_object_info) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  // End s can't be larger than max leading object s.
  // first: traj_id, second: closest_s.
  double stationary_closest_min_s = std::numeric_limits<double>::infinity();
  double moving_closest_min_s = std::numeric_limits<double>::infinity();
  const double acc_standstill_standoff =
      cost_weight_params.object_cost_params().acc_standstill_standoff();

  for (const auto& traj_ptr : spacetime_trajs) {
    const bool is_leading =
        leading_trajs.find(std::string(traj_ptr->traj_id())) !=
        leading_trajs.end();
    if (is_leading) {
      const Polygon2d& contour = traj_ptr->contour();
      const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
      if (!frenet_box_or.ok()) {
        continue;
      }
      if (traj_ptr->is_stationary()) {
        if (double leading_s = frenet_box_or->s_min - acc_standstill_standoff;
            leading_s < stationary_closest_min_s) {
          *stationary_closest_leading_object_info = {{*frenet_box_or},
                                                     traj_ptr};
          stationary_closest_min_s = leading_s;
        }
      } else {
        if (double leading_s = frenet_box_or->s_min - acc_standstill_standoff;
            leading_s < moving_closest_min_s) {
          *moving_closest_leading_object_info = {std::vector<FrenetBox>(),
                                                 traj_ptr};
          moving_closest_min_s = leading_s;
        }
      }
    }
  }
  if (moving_closest_leading_object_info->has_value() &&
      moving_closest_leading_object_info->value().second != nullptr) {
    const auto states = SampleObjectStates(
        trajectory_steps, trajectory_time_step,
        moving_closest_leading_object_info->value().second->states());
    std::vector<FrenetBox>& frenet_boxes =
        moving_closest_leading_object_info->value().first;
    frenet_boxes.reserve(states.size());
    for (const auto& state : states) {
      const Polygon2d& contour = state.contour;
      const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
      if (!frenet_box_or.ok()) {
        return;
      }
      frenet_boxes.push_back(frenet_box_or.value());
    }
  }
}

bool IgnoreObjectCost(
    std::string_view base_name, const SpacetimeObjectTrajectory& traj,
    const std::vector<prediction::PredictionObjectState>& sampled_states,
    const DrivePassage& drive_passage,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const std::optional<
        std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>&
        stationary_closest_leading_object_info,
    const std::optional<
        std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>&
        moving_closest_leading_object_info) {
  if (!stationary_closest_leading_object_info.has_value() &&
      !moving_closest_leading_object_info.has_value()) {
    return false;
  }
  // Don't ignore closest leading object itself.
  if (stationary_closest_leading_object_info.has_value() &&
      traj.traj_id() ==
          stationary_closest_leading_object_info->second->traj_id()) {
    return false;
  }
  // Ignore spacetime trajectories whose entire trajectory is behind the
  // closest leading object.
  const double acc_standstill_standoff =
      cost_weight_params.object_cost_params().acc_standstill_standoff();
  if (stationary_closest_leading_object_info.has_value()) {
    const double ignore_s =
        stationary_closest_leading_object_info->first.front().s_min;
    if (traj.is_stationary()) {
      const auto frenet_box_or =
          drive_passage.QueryFrenetBoxAtContour(traj.contour());
      if (!frenet_box_or.ok()) {
        VLOG(2) << base_name << " ignores stationary st-trajectory "
                << traj.traj_id()
                << " because we can't query its contour on drive passage.";
        return true;
      }
      if ((frenet_box_or->s_min - acc_standstill_standoff) < ignore_s) {
        return false;
      }
    } else {
      for (int k = 0; k < sampled_states.size(); ++k) {
        const Polygon2d& contour = sampled_states[k].contour;
        const auto frenet_box_or =
            drive_passage.QueryFrenetBoxAtContour(contour);
        if (!frenet_box_or.ok()) {
          return true;
        }
        if ((frenet_box_or->s_min - acc_standstill_standoff) < ignore_s) {
          return false;
        }
      }
    }
  }
  if (moving_closest_leading_object_info.has_value() &&
      traj.traj_id() == moving_closest_leading_object_info->second->traj_id()) {
    return false;
  }
  if (moving_closest_leading_object_info.has_value()) {
    const std::vector<FrenetBox>& leading_frenet_boxes =
        moving_closest_leading_object_info->first;
    if (sampled_states.size() > leading_frenet_boxes.size()) {
      return false;
    }
    for (int k = 0; k < sampled_states.size(); ++k) {
      const Polygon2d& contour = sampled_states[k].contour;
      const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
      if (!frenet_box_or.ok()) {
        return true;
      }
      const FrenetBox& leading_frenet_box = leading_frenet_boxes[k];
      if ((frenet_box_or->s_min - acc_standstill_standoff) <
          leading_frenet_box.s_min) {
        return false;
      }
    }
  }
  VLOG(2) << base_name << " ignores st-trajectory " << traj.traj_id()
          << " because its entire trajectory is behind closest leading object.";
  return true;
}

bool AddAggregateStaticObjectCost(
    const std::vector<double>& extra_gain_for_nudge, const int plan_id,
    double trajectory_time_step, std::string_view base_name,
    const LaneChangeStage lc_stage, const TrajectoryPoint& plan_start_point,
    const PathTimeCorridor& path_time_corridor, const bool ref_enhance,
    absl::Span<const SpacetimeObjectTrajectory* const> spacetime_trajs,
    double min_mirror_height_avg, double max_mirror_height_avg,
    const double lane_width_l, const double lane_width_r,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  std::vector<Segment2d> segments;
  std::vector<Segment2d> segments_consider_mirrors;
  constexpr int kEstimateLineCountsPerObject = 6;
  CHECK_GT(trajectory_time_step, 0.0);
  // const int free_index = static_cast<int>(
  //     (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);
  const int free_index = std::max(kTrajectorySteps - 1, 1);
  segments.reserve(kEstimateLineCountsPerObject * spacetime_trajs.size());
  segments_consider_mirrors.reserve(kEstimateLineCountsPerObject *
                                    spacetime_trajs.size());
  const double lane_width = lane_width_l - lane_width_r;
  std::vector<bool> obj_within_lane_vec;
  bool is_all_on_line_cone = true;
  for (int idx = 0; idx < spacetime_trajs.size(); ++idx) {
    const auto& traj = *spacetime_trajs[idx];
    auto obejct_position = path_time_corridor.QueryObjectPositionInfo(
        std::string(traj.object_id()));
    auto obj_boundary_info =
        path_time_corridor.QueryBoundaryL(obejct_position[0].object_s, 0);
    auto lane_width_obj = obj_boundary_info.second.l_boundary -
                          obj_boundary_info.first.l_boundary;

    auto obj_lane_attr_type = traj.planner_object()
                                  .object_proto()
                                  .vision_attribute()
                                  .lane_attr_type();
    bool is_on_line_cone =
        (traj.object_type() == ObjectType::OT_CONE &&
         obj_lane_attr_type == LaneAttrType::LANEATTR_ON_LINE);
    if (!is_on_line_cone) {
      is_all_on_line_cone = false;
    }

    const bool obj_within_lane =
        !is_on_line_cone &&
        ((lane_width_obj < 3.0 &&
          obejct_position[0].object_l < obj_boundary_info.second.l_boundary &&
          obejct_position[0].object_l > obj_boundary_info.first.l_boundary) ||
         ((lane_width_obj > 3.0 || lane_width_obj == 3.0) &&
          obejct_position[0].object_l <
              obj_boundary_info.second.l_boundary - 0.1 &&
          obejct_position[0].object_l >
              obj_boundary_info.first.l_boundary + 0.1));

    std::vector<std::string> debug_u;
    debug_u.emplace_back(absl::StrCat("lane_width_obj: ", lane_width_obj));
    debug_u.emplace_back(
        absl::StrCat("obejct_position_l: ", obejct_position[0].object_l));
    debug_u.emplace_back(absl::StrCat("obejct_id: ", traj.object_id()));
    debug_u.emplace_back(
        absl::StrCat("lane_l: ", obj_boundary_info.second.l_boundary));
    debug_u.emplace_back(
        absl::StrCat("lane_r: ", obj_boundary_info.first.l_boundary));
    debug_u.emplace_back(absl::StrCat("obj_within_lane: ", obj_within_lane));

    Log2DDS::LogDataV0("uu_debug", debug_u);
    obj_within_lane_vec.emplace_back(obj_within_lane);
    /*     if (!obj_within_lane) {
          continue;
        } */

    std::vector<std::string> debug_o;
    debug_o.emplace_back(absl::StrCat("_s:", obejct_position[0].object_s));
    debug_o.emplace_back(absl::StrCat("_l:", obejct_position[0].object_l));
    debug_o.emplace_back(absl::StrCat("_id:", traj.object_id()));
    debug_o.emplace_back(absl::StrCat("_obj_type:", traj.object_type()));
    debug_o.emplace_back(
        absl::StrCat("_dir:", obejct_position[0].position_type));
    Log2DDS::LogDataV0("object", debug_o);
    const auto& object_segments = traj.contour().line_segments();

    if (IsConsiderMirrorObject(traj.planner_object().object_proto(),
                               min_mirror_height_avg, max_mirror_height_avg)) {
      segments_consider_mirrors.insert(segments_consider_mirrors.end(),
                                       object_segments.begin(),
                                       object_segments.end());
    } else {
      segments.insert(segments.end(), object_segments.begin(),
                      object_segments.end());
    }
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

  double kSafeBuffer = is_all_on_line_cone ? 0.40 : 0.50;
  // double nudge_buffer_soft =
  //     lc_stage == LaneChangeStage::LCS_NONE
  //         ? std::clamp(0.4 + 0.46 * (lane_width - 2.4), 0.4, 1.0)
  //         : 0.75;
  //  Copy from close_object_slowdown_decider, please modify at the same time.
  const std::vector<double> station_inside_sl_boundary_static_max_speed = {
      3.0, 10.0, 20.0, 30.0};  // m/s
  const std::vector<double> close_object_distance = {0.5, 0.65, 0.8, 1.0};  // m
  const std::vector<double> on_line_object_distance = {0.4, 0.45, 0.7,
                                                       1.0};  // m
  const std::vector<double> lane_width_vec = {2.7, 2.8, 2.9, 3.0, 4.0};
  const std::vector<double> nudge_buffers_vec = {0.05, 0.05, 0.05, 0.05, 0.40};

  const PiecewiseLinearFunction<double> nudge_buffer_lane_width_plf(
      lane_width_vec, nudge_buffers_vec);

  const PiecewiseLinearFunction<double> nudge_buffer_speed_plf =
      PiecewiseLinearFunction<double>(
          station_inside_sl_boundary_static_max_speed,
          is_all_on_line_cone ? on_line_object_distance
                              : close_object_distance);
  double nudge_buffer_soft =
      std::max(kSafeBuffer, nudge_buffer_speed_plf(plan_start_point.v()));
  if (need_lane_width_buffer) {
    nudge_buffer_soft =
        std::max(kSafeBuffer, nudge_buffer_speed_plf(plan_start_point.v()) +
                                  nudge_buffer_lane_width_plf(lane_width));
  }

  double attenuation_buffer = nudge_buffer_soft;
  if (lc_stage == LaneChangeStage::LCS_NONE) {
    PiecewiseLinearFunction<double> attenuation_thr_cal_func =
        PiecewiseLinearFunctionFromProto(
            cost_weight_params.object_cost_params()
                .static_obs_buffer_attenuation_threshold());
    double attenuation_thr =
        attenuation_thr_cal_func(std::fabs(plan_start_point.v()));
    PiecewiseLinearFunction<double> attenuation_factor_func =
        PiecewiseLinearFunctionFromProto(
            cost_weight_params.object_cost_params()
                .static_obs_buffer_attenuation_factor());
    double atten_factor =
        attenuation_factor_func(std::fabs(plan_start_point.v()));
    attenuation_buffer =
        attenuation_buffer <= attenuation_thr
            ? attenuation_buffer
            : attenuation_thr +
                  (attenuation_buffer - attenuation_thr) * atten_factor;
  }

  Log2DDS::LogDataV0("uu_debug", absl::StrCat("need_lane_width_buffer: ",
                                              need_lane_width_buffer));
  const std::string nudge_buffer_soft_str =
      "StaticObject, task_" +
      absl::StrFormat("%d,nudge_buffer_soft:%f", plan_id, attenuation_buffer);
  Log2DDS::LogDataV2("object", nudge_buffer_soft_str);
  const double nudge_buffer_hard = kSafeBuffer;
  std::vector<double> gains = {
      cost_weight_params.object_cost_params().object_a_cost_weight(),
      cost_weight_params.object_cost_params().object_b_cost_weight()};
  if (lc_stage == LaneChangeStage::LCS_NONE) {
    PiecewiseLinearFunction<double> static_obs_gain_vec_func =
        PiecewiseLinearFunctionFromProto(
            cost_weight_params.object_cost_params()
                .static_object_a_cost_weight_attenuation_factor());
    double static_obs_gain =
        static_obs_gain_vec_func(std::fabs(plan_start_point.v()));
    gains[0] *= static_obs_gain;
  }

  if (ref_enhance && lc_stage == LaneChangeStage::LCS_NONE) {
    PiecewiseLinearFunction<double> weight_gain_for_ref_l_enhance_func =
        PiecewiseLinearFunctionFromProto(cost_weight_params.object_cost_params()
                                             .weight_gain_for_enhance_static());
    double extra_gain_for_enhance =
        weight_gain_for_ref_l_enhance_func(std::fabs(plan_start_point.v()));
    gains[0] *= extra_gain_for_enhance;
  }

  std::vector<std::string> sub_names = {"a", "b"};
  std::vector<double> dist_to_rac, angle_to_axis;
  std::vector<std::vector<double>> buffers;
  const auto& circles = trajectory_optimizer_vehicle_model_params.circles();
  dist_to_rac.reserve(circles.size());
  angle_to_axis.reserve(circles.size());
  buffers.reserve(circles.size());
  for (const auto& circle : circles) {
    dist_to_rac.push_back(circle.dist_to_rac());
    angle_to_axis.push_back(circle.angle_to_axis());
    buffers.push_back({attenuation_buffer + circle.radius(),
                       nudge_buffer_hard + circle.radius()});
  }
  if (!segments.empty()) {
    costs->push_back(std::make_unique<AggregateStaticObjectCost<Mfob>>(
        extra_gain_for_nudge, segments, dist_to_rac, angle_to_axis, buffers,
        gains, sub_names, free_index,
        /*using_hessian_approximate=*/
        true, absl::StrFormat("Aggregate Static Object"),
        cost_weight_params.object_cost_weight(),
        /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  }
  if (!segments_consider_mirrors.empty()) {
    const auto& mirror_circles =
        trajectory_optimizer_vehicle_model_params.mirror_circles();
    for (const auto& circle : mirror_circles) {
      dist_to_rac.push_back(circle.dist_to_rac());
      angle_to_axis.push_back(circle.angle_to_axis());
      buffers.push_back({attenuation_buffer + circle.radius(),
                         nudge_buffer_hard + circle.radius()});
    }
    costs->push_back(std::make_unique<AggregateStaticObjectCost<Mfob>>(
        extra_gain_for_nudge, segments_consider_mirrors, std::move(dist_to_rac),
        std::move(angle_to_axis), std::move(buffers), std::move(gains),
        std::move(sub_names), free_index,
        /*using_hessian_approximate=*/
        true, absl::StrFormat("Aggregate Static Object Consider Mirrors"),
        cost_weight_params.object_cost_weight(),
        /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  }
  return true;
}

void DecayInnerPathBoundaryGains(const DrivePassage& drive_passage,
                                 const PathSlBoundary& path_boundary,
                                 bool is_stationary,
                                 const TrajectoryPoint& plan_start_point,
                                 const SecondOrderTrajectoryPoint& object_pose,
                                 const Polygon2d& object_contour,
                                 std::vector<double>* path_boundary_gains,
                                 char* is_gain_update) {
  CHECK(path_boundary_gains != nullptr);
  CHECK_EQ(path_boundary_gains->size(), path_boundary.size());
  if (is_stationary) {
    bool contour_out_boundary = true;
    const auto& contour_points = object_contour.points();
    constexpr double kSafeBuffer = 0.5;
    for (const auto& pt : contour_points) {
      const auto frenet_pt = drive_passage.QueryFrenetCoordinateAt(pt);
      if (!frenet_pt.ok()) {
        return;
      }
      const auto boundary_l = path_boundary.QueryBoundaryL(frenet_pt->s);
      if (frenet_pt->l > (boundary_l.first - kSafeBuffer) &&
          frenet_pt->l < (boundary_l.second + kSafeBuffer)) {
        contour_out_boundary = false;
        break;
      }
    }
    if (contour_out_boundary) return;

    *is_gain_update = true;
    const auto obj_min_dist_dp_station_pt_index =
        drive_passage.FindNearestStationIndex(object_pose.pos());
    const auto& obj_min_dist_dp_station_pt =
        drive_passage.station(obj_min_dist_dp_station_pt_index);
    const Vec2d& min_dist_dp_station_pt_theta_tangent =
        obj_min_dist_dp_station_pt.tangent();
    const double min_dist_pt_s = obj_min_dist_dp_station_pt.accumulated_s();

    Vec2d front, back;
    object_contour.ExtremePoints(min_dist_dp_station_pt_theta_tangent, &back,
                                 &front);
    const double contour_length =
        (front - back).dot(min_dist_dp_station_pt_theta_tangent);

    constexpr double kGainSRangeBase = 5.0;
    constexpr double kGainSRangeCoeff = 0.5;
    const double gain_s = kGainSRangeBase +
                          kGainSRangeCoeff * Sqr(plan_start_point.v()) +
                          contour_length * 0.5;
    const double s_max = min_dist_pt_s + gain_s;
    const double s_min = min_dist_pt_s - gain_s;

    constexpr double kMinGain = 0.01;

    const auto& path_boundary_s_vector = path_boundary.s_vector();
    int mid_path_boundary_index = obj_min_dist_dp_station_pt_index.value();
    for (int i = mid_path_boundary_index;
         i < path_boundary.size() && path_boundary_s_vector[i] < s_max; ++i) {
      const double factor = (s_max - path_boundary_s_vector[i]) / gain_s;
      (*path_boundary_gains)[i] =
          std::min((*path_boundary_gains)[i], std::pow(kMinGain, factor));
    }

    for (int i = std::min(mid_path_boundary_index,
                          static_cast<int>(path_boundary.size()) - 1);
         i >= 0 && path_boundary_s_vector[i] > s_min; --i) {
      const double factor = (path_boundary_s_vector[i] - s_min) / gain_s;
      (*path_boundary_gains)[i] =
          std::min((*path_boundary_gains)[i], std::pow(kMinGain, factor));
    }
  }
}

}  // namespace

void AddObjectCosts(
    const int plan_id, const LaneChangeStage lc_stage, const bool ref_enhance,
    const bool borrow_lane, const NudgeInfos& nudge_info, int trajectory_steps,
    double trajectory_time_step, double avoid_dynamic_obj_early_time,
    std::string_view base_name, const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const PathTimeCorridor& path_time_corridor,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    const bool is_narrow_scene, std::vector<LeadingInfo>* leading_min_s,
    std::vector<double>* inner_path_boundary_gains,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs, ThreadPool* thread_pool,
    TurnType ego_turn_type, const std::vector<TrajectoryPoint>& prev_traj) {
  TIMELINE("AddObjectCosts");

  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);

  CHECK_GE(init_traj.size(), trajectory_steps);
  // Use a map-reduce strategy to parallelize the obstacle cost collection.
  // const auto spacetime_trajs = st_traj_mgr.spacetime_planner_trajs();
  const auto& spacetime_trajs = st_planner_object_traj.trajectories;
  const int num_trajs = spacetime_trajs.size();

  // Group st_trajs with different object cost types.
  // Trajs added to aggregate object cost.
  std::vector<const SpacetimeObjectTrajectory*> static_spacetime_trajs;
  // Trajs used to generate partition and leading object costs.
  std::vector<const SpacetimeObjectTrajectory*> generic_spacetime_trajs;

  static_spacetime_trajs.reserve(num_trajs);
  generic_spacetime_trajs.reserve(num_trajs);
  for (const auto& traj : spacetime_trajs) {
    if (prediction::IsStaticObjectType(traj.object_type()) &&
        leading_trajs.find(std::string(traj.traj_id())) ==
            leading_trajs.end()) {
      static_spacetime_trajs.push_back(&traj);
      // Log2DDS::LogDataV2(
      //     "object",
      //     absl::StrCat("static_spacetime_trajs: ", traj.traj_id(),
      //                  ", is_stationary: ", traj.is_stationary()));
    } else {
      generic_spacetime_trajs.push_back(&traj);
      // Log2DDS::LogDataV2(
      //     "object",
      //     absl::StrCat("generic_spacetime_trajs: ", traj.traj_id(),
      //                  ", is_stationary: ", traj.is_stationary()));
    }
  }

  // Compute AV min & max mirror average height.
  const auto min_max_mirror_average_height =
      ComputeMinMaxMirrorAverageHeight(veh_geo_params);
  // Note: Structured bindings cannot be captured by lambda expressions until
  // C++20.
  const double min_mirror_height_avg = min_max_mirror_average_height.first;
  const double max_mirror_height_avg = min_max_mirror_average_height.second;

  const auto ego_lane_boundary_info =
      drive_passage.QueryEnclosingLaneBoundariesAtS(init_traj.front().s());
  const double lane_width = ego_lane_boundary_info.left->lat_offset -
                            ego_lane_boundary_info.right->lat_offset;
  const double lane_width_l = ego_lane_boundary_info.left->lat_offset;
  const double lane_width_r = ego_lane_boundary_info.right->lat_offset;
  Log2DDS::LogDataV2("object", absl::StrCat("lanewidth : ", lane_width));

  std::vector<double> extra_gain_for_nudge(trajectory_steps, 1.0);
  CalculateNudgeGainForIntrusion(plan_id, lc_stage, drive_passage, init_traj,
                                 trajectory_time_step, trajectory_steps,
                                 cost_weight_params, &extra_gain_for_nudge);

  // Add aggregate static object cost first.
  AddAggregateStaticObjectCost(
      extra_gain_for_nudge, plan_id, trajectory_time_step, base_name, lc_stage,
      init_traj.front(), path_time_corridor, ref_enhance,
      static_spacetime_trajs, min_mirror_height_avg, max_mirror_height_avg,
      lane_width_l, lane_width_r, cost_weight_params,
      trajectory_optimizer_vehicle_model_params, veh_geo_params, costs);
  // Decay inner path boundary gain for static object.
  for (const auto& traj : static_spacetime_trajs) {
    char is_gain_update = false;
    DecayInnerPathBoundaryGains(drive_passage, path_boundary,
                                traj->is_stationary(), init_traj.front(),
                                traj->pose(), traj->contour(),
                                inner_path_boundary_gains, &is_gain_update);
  }

  // Next, add partition and leading object costs.
  const int num_generic_spacetime_trajs = generic_spacetime_trajs.size();
  std::vector<std::vector<double>> inner_path_boundary_gains_all_trajs(
      num_generic_spacetime_trajs,
      std::vector<double>(inner_path_boundary_gains->size(), 1.0));
  std::vector<std::vector<std::unique_ptr<Cost<Mfob>>>> costs_all_trajs(
      num_generic_spacetime_trajs);
  std::vector<char> is_gains_update(num_generic_spacetime_trajs, false);
  std::vector<std::vector<LeadingInfo>> leading_min_s_all_trajs(
      num_generic_spacetime_trajs, *leading_min_s);

  // Get Clostest leading object min_s, we will use the value to filter
  // objects whose prediction traj point s all larger than min_s.
  std::optional<
      std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>
      stationary_closest_leading_object_info,
      moving_closest_leading_object_info;
  GetClosestLeadingObjectInfo(trajectory_steps, trajectory_time_step,
                              drive_passage, leading_trajs,
                              generic_spacetime_trajs, cost_weight_params,
                              &stationary_closest_leading_object_info,
                              &moving_closest_leading_object_info);

  // Don't apply parallelism in debugging mode as is not
  // thread safe.
  ThreadPool* used_tp = thread_pool;
  // do not use the thread pool
  ParallelFor(0, num_generic_spacetime_trajs, thread_pool, [&](int i) {
    TIMELINE("AddObjectCosts::ProcessTrajObject");
    const auto& traj = *(generic_spacetime_trajs[i]);
    const auto states = SampleObjectStates(trajectory_steps,
                                           trajectory_time_step, traj.states());
    if (!IgnoreObjectCost(base_name, traj, states, drive_passage,
                          cost_weight_params,
                          stationary_closest_leading_object_info,
                          moving_closest_leading_object_info)) {
      const bool is_static = prediction::IsStaticObjectType(traj.object_type());
      const bool is_leading_object =
          leading_trajs.find(std::string(traj.traj_id())) !=
          leading_trajs.end();
      if (is_leading_object) {
        if (!cost_weight_params.object_cost_params().ignore_leading() &&
            lc_stage != LaneChangeStage::LCS_NONE) {
          AddUnidirectionalObjectCost(
              trajectory_time_step, base_name, states, init_traj, drive_passage,
              path_boundary, leading_trajs, cost_weight_params, veh_geo_params,
              motion_constraint_params, std::string(traj.traj_id()),
              traj.trajectory().probability(), &leading_min_s_all_trajs[i],
              &costs_all_trajs[i]);
        }
      } else {
        NudgeBufferManager nudge_buffer_manager;
        const auto& nudge_buffer =
            (traj.is_stationary() || is_static)
                ? nudge_buffer_manager.GenerateNudgeBufferStationary(
                      plan_id, lc_stage, borrow_lane, states,
                      IsCameraObject(traj.planner_object().object_proto()),
                      is_static, traj.planner_object().velocity(),
                      traj.contour(), init_traj, drive_passage, lane_width,
                      cost_weight_params, veh_geo_params, traj, path_boundary,
                      path_time_corridor, ego_turn_type)
                : nudge_buffer_manager.GenerateNudgeBufferDynamic(
                      plan_id, lc_stage, borrow_lane, states, init_traj,
                      IsCameraObject(traj.planner_object().object_proto()),
                      traj.planner_object().velocity(), traj.contour(),
                      init_traj.front(), drive_passage, lane_width,
                      cost_weight_params, veh_geo_params, traj, path_boundary,
                      path_time_corridor, ego_turn_type, prev_traj);

        const bool consider_mirrors = IsConsiderMirrorObject(
            traj.planner_object().object_proto(), min_mirror_height_avg,
            max_mirror_height_avg);

        AddPartitionAvObjectCost(
            extra_gain_for_nudge, plan_id, std::string(traj.object_id()),
            traj.object_type(), ref_enhance, lc_stage, borrow_lane, nudge_info,
            trajectory_time_step, path_time_corridor,
            avoid_dynamic_obj_early_time, base_name, nudge_buffer,
            consider_mirrors, states, init_traj, traj.is_stationary(),
            drive_passage, cost_weight_params,
            trajectory_optimizer_vehicle_model_params, av_model_helpers,
            std::string(traj.traj_id()), traj.trajectory().probability(),
            traj.planner_object().velocity(), veh_geo_params,
            &costs_all_trajs[i]);
      }
      DecayInnerPathBoundaryGains(
          drive_passage, path_boundary, traj.is_stationary(), init_traj.front(),
          traj.pose(), traj.contour(), &inner_path_boundary_gains_all_trajs[i],
          &is_gains_update[i]);
    }
  });

  // Collect results from each trajectory.
  for (int idx = 0; idx < num_generic_spacetime_trajs; ++idx) {
    const auto& leading_min_s_per_traj = leading_min_s_all_trajs[idx];
    for (int i = 0; i <= free_index; ++i) {
      if (leading_min_s_per_traj[i].s < (*leading_min_s)[i].s) {
        (*leading_min_s)[i] = leading_min_s_per_traj[i];
      }
    }
    const auto& gains_per_traj = inner_path_boundary_gains_all_trajs[idx];
    if (is_gains_update[idx]) {
      for (int i = 0; i < gains_per_traj.size(); ++i) {
        (*inner_path_boundary_gains)[i] =
            std::min((*inner_path_boundary_gains)[i], gains_per_traj[i]);
      }
    }
  }
  for (auto& costs_per_traj : costs_all_trajs) {
    std::move(costs_per_traj.begin(), costs_per_traj.end(),
              std::back_inserter(*costs));
  }
}

void CalcPartitionHalfContourInfo(const Vec2d& x, const Vec2d& obj_x,
                                  const Polygon2d& contour, double buffer,
                                  std::vector<Segment2d>* lines, Vec2d* ref_x,
                                  Vec2d* ref_tangent, double* offset) {
  const Vec2d force_dir = (x - obj_x).normalized();
  const Vec2d force_right = -force_dir.Perp();

  Vec2d left, right, front, back;
  int left_index, right_index, front_index, back_index;
  contour.ExtremePoints(force_dir, &back_index, &front_index, &back, &front);
  contour.ExtremePoints(force_right, &left_index, &right_index, &left, &right);

  const auto& contour_lines = contour.line_segments();
  lines->reserve(contour_lines.size());

  // Insert right border
  constexpr double kBorderExtent = 2.0;
  const Segment2d& right_line = contour_lines[right_index];
  lines->emplace_back(right_line.start() - force_dir * kBorderExtent,
                      right_line.start());
  if (front_index >= right_index && front_index <= left_index) {
    lines->insert(lines->end(), contour_lines.begin() + right_index,
                  contour_lines.begin() + left_index);
  } else {
    lines->insert(lines->end(), contour_lines.begin() + right_index,
                  contour_lines.end());
    if (left_index != 0) {
      lines->insert(lines->end(), contour_lines.begin(),
                    contour_lines.begin() + left_index);
    }
  }
  // Insert left border
  const Segment2d& left_line =
      contour_lines[left_index == 0 ? (contour_lines.size() - 1)
                                    : (left_index - 1)];
  lines->emplace_back(left_line.end(),
                      left_line.end() - force_dir * kBorderExtent);

  // Fill filter variables
  *ref_x = Vec2d((left.x() + right.x()) * 0.5, (front.y() + back.y()) * 0.5);
  *ref_tangent = force_dir;
  *offset = (front - (*ref_x)).dot(force_dir) + buffer;
}

std::vector<prediction::PredictionObjectState> SampleObjectStates(
    int trajectory_steps, double trajectory_time_step,
    absl::Span<const prediction::PredictionObjectState> states) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);

  const int sample_step =
      static_cast<int>(trajectory_time_step / kTrajectoryTimeStep + 0.5);

  // removed.
  std::vector<prediction::PredictionObjectState> sampled_states;
  sampled_states.reserve(states.size() / sample_step);

  Log2DDS::LogDataV2("obs_traj_sample_info: ",
                     absl::StrCat(trajectory_time_step, ", ",
                                  kTrajectoryTimeStep, ", ", sample_step, ", ",
                                  states.size(), ", ", sampled_states.size()));

  for (int i = 0; i < trajectory_steps; ++i) {
    if (i * sample_step >= states.size()) break;
    sampled_states.push_back(states[i * sample_step]);
  }
  return sampled_states;
}

}  // namespace optimizer
}  // namespace planning
}  // namespace st
