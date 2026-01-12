

#include "planner/planner_manager/planner_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "plan_common/container/strong_int.h"
//#include "lite/check.h"
//#include "lite/logging.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/maps/semantic_map_util.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
#include "plan_common/av_history.h"
#include "plan_common/util/path_util.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "planner/planner_manager/planner_defs.h"
#include "plan_common/util/file_util.h"
#include "plan_common/util/time_util.h"

namespace st {
namespace planning {

namespace {

void UpdateToMinSpeedLimit(std::map<mapping::ElementId, double>* map,
                           mapping::ElementId lane_id, double speed_limit) {
  auto pair_it = map->insert({lane_id, speed_limit});
  if (pair_it.second == false) {
    pair_it.first->second = std::min(pair_it.first->second, speed_limit);
  }
}

}  // namespace

double ComputeLongitudinalJerk(const TrajectoryPoint& traj_point) {
  return traj_point.j() - Cube(traj_point.v()) * Sqr(traj_point.kappa());
}

double ComputeLateralAcceleration(const TrajectoryPoint& traj_point) {
  return Sqr(traj_point.v()) * traj_point.kappa();
}

double ComputeLateralJerk(const TrajectoryPoint& traj_point) {
  return 3.0 * traj_point.v() * traj_point.a() * traj_point.kappa() +
         Sqr(traj_point.v()) * traj_point.psi();
}

// PlannerSemanticMapModification CreateSemanticMapModification(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const mapping::SemanticMapModifierProto& modifier) {
//   std::map<mapping::ElementId, double> lane_speed_limit_map;
//   double max_speed_limit = std::numeric_limits<double>::max();

//   if (modifier.has_speed_limit_modifier()) {
//     if (modifier.speed_limit_modifier().has_max_speed_limit()) {
//       max_speed_limit = modifier.speed_limit_modifier().max_speed_limit();
//     }

//     for (const auto& lane_id_mod :
//          modifier.speed_limit_modifier().lane_id_modifier()) {
//       UpdateToMinSpeedLimit(&lane_speed_limit_map,
//                             mapping::ElementId(lane_id_mod.lane_id()),
//                             lane_id_mod.override_speed_limit());
//     }

//     for (const auto& region_mod :
//          modifier.speed_limit_modifier().region_modifier()) {
//       const Polygon2d polygon = SmoothPolygon2dFromGeoPolygonProto(
//           region_mod.region(), semantic_map_manager.coordinate_converter());
//       for (const auto& mutable_lane_info : semantic_map_manager.lane_info())
//       {
//         bool in_polygon = true;
//         for (const auto& sp : mutable_lane_info.points_smooth) {
//           if (!polygon.IsPointIn(sp)) {
//             in_polygon = false;
//             break;
//           }
//         }
//         if (in_polygon) {
//           UpdateToMinSpeedLimit(&lane_speed_limit_map, mutable_lane_info.id,
//                                 region_mod.override_speed_limit());
//         }
//       }
//     }
//   }

//   return PlannerSemanticMapModification{
//       .lane_speed_limit_map = std::move(lane_speed_limit_map),
//       .max_speed_limit = max_speed_limit};
// }

// PlannerSemanticMapModification CreateSemanticMapModification(
//     const ad_byd::planning::Map& semantic_map_manager,
//     const mapping::SemanticMapModifierProto& modifier) {
//   std::map<mapping::ElementId, double> lane_speed_limit_map;
//   double max_speed_limit = std::numeric_limits<double>::max();

//   if (modifier.has_speed_limit_modifier()) {
//     if (modifier.speed_limit_modifier().has_max_speed_limit()) {
//       max_speed_limit = modifier.speed_limit_modifier().max_speed_limit();
//     }

//     for (const auto& lane_id_mod :
//          modifier.speed_limit_modifier().lane_id_modifier()) {
//       UpdateToMinSpeedLimit(&lane_speed_limit_map,
//                             mapping::ElementId(lane_id_mod.lane_id()),
//                             lane_id_mod.override_speed_limit());
//     }

//     for (const auto& region_mod :
//          modifier.speed_limit_modifier().region_modifier()) {
//       const Polygon2d polygon =
//           Polygon2d(mapping::ConverterGeoPoints(region_mod.region().points()));
//       for (const auto& lane_ptr : semantic_map_manager.semantic_map().lanes)
//       {
//         bool in_polygon = true;
//         for (const auto& point : lane_ptr->segment_points()) {
//           if (!polygon.IsPointIn(point)) {
//             in_polygon = false;
//             break;
//           }
//         }
//         if (in_polygon) {
//           UpdateToMinSpeedLimit(&lane_speed_limit_map,
//                                 mapping::ElementId(lane_ptr->proto().id()),
//                                 region_mod.override_speed_limit());
//         }
//       }
//     }
//   }

//   return PlannerSemanticMapModification{
//       .lane_speed_limit_map = std::move(lane_speed_limit_map),
//       .max_speed_limit = max_speed_limit};
// }

// mapping::SemanticMapModifierProto PlannerSemanticMapModificationToProto(
//     const PlannerSemanticMapModification& modifier) {
//   mapping::SemanticMapModifierProto modifier_proto;
//   modifier_proto.mutable_speed_limit_modifier()->set_max_speed_limit(
//       modifier.max_speed_limit);

//   for (const auto& it : modifier.lane_speed_limit_map) {
//     auto* lane_id_modifier =
//         modifier_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
//     lane_id_modifier->set_lane_id(it.first.value());
//     lane_id_modifier->set_override_speed_limit(it.second);
//   }

//   return modifier_proto;
// }

std::vector<ApolloTrajectoryPointProto> CreatePastPointsList(
    absl::Time plan_time, const TrajectoryProto& prev_traj, bool reset,
    int max_past_point_num) {
  std::vector<ApolloTrajectoryPointProto> past_points;
  const double curr_plan_time = ToUnixDoubleSeconds(plan_time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      curr_plan_time >
          prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      curr_plan_time < prev_traj_start_time || reset) {
    return past_points;
  }
  past_points.reserve(max_past_point_num);
  const int relative_time_index =
      RoundToInt((curr_plan_time - prev_traj_start_time) / kTrajectoryTimeStep);
  CHECK_LT(relative_time_index, prev_traj.trajectory_point_size());
  const double relative_s =
      -prev_traj.trajectory_point(relative_time_index).path_point().s();
  for (int i = max_past_point_num; i > 0; --i) {
    const int index = relative_time_index - i;
    if (index + prev_traj.past_points_size() < 0) {
      continue;
    }
    auto point =
        index < 0 ? prev_traj.past_points(index + prev_traj.past_points_size())
                  : prev_traj.trajectory_point(index);
    point.set_relative_time(-i * kTrajectoryTimeStep);
    point.mutable_path_point()->set_s(point.path_point().s() + relative_s);
    past_points.push_back(point);
  }
  past_points.shrink_to_fit();

  return past_points;
}

ApolloTrajectoryPointProto ComputePlanStartPointAfterReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool is_forward_task) {
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  const double pose_v = pose.vel_body().x();
  const double abs_pose_v = std::abs(pose_v);
  if (is_forward_task) {
    plan_start_point.set_v(std::max(0.0, pose_v));
  } else {
    plan_start_point.set_v(std::min(0.0, pose_v));
  }

  if (prev_reset_planned_point.has_value()) {
    constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
    const bool full_stop = prev_reset_planned_point->v() == 0.0 &&
                           abs_pose_v < kFullStopSpeedThreshold;
    if (full_stop) {
      plan_start_point.mutable_path_point()->set_kappa(
          prev_reset_planned_point->path_point().kappa());
      plan_start_point.mutable_path_point()->set_lambda(
          prev_reset_planned_point->path_point().lambda());
      plan_start_point.set_a(0.0);
      plan_start_point.set_j(0.0);
      return plan_start_point;
    }
  }

  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (abs_pose_v < kLowSpeedThreshold) {
    // At speed lower than this, we don't trust the measured acceleration and
    // angular velocity. Reset like this are mostly when we're stopped anyway.
    // const double steer_angle =
    //     chassis.has_steering_percentage()
    //         ? chassis.steering_percentage() * 0.01 *
    //               vehicle_drive_params.max_steer_angle() /
    //               vehicle_drive_params.steer_ratio()
    //         : 0.0;  // rad.
    const double kappa =
        std::tan(front_wheel_angle) / vehicle_geom_params.wheel_base();
    plan_start_point.mutable_path_point()->set_kappa(kappa);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
    plan_start_point.set_a(0.0);
    plan_start_point.set_j(0.0);
  } else {
    plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                     pose_v);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
    plan_start_point.set_a(std::clamp(
        pose.accel_body().x(), motion_constraint_params.max_deceleration(),
        motion_constraint_params.max_acceleration()));
    plan_start_point.set_j(0.0);
  }
  return plan_start_point;
}

std::vector<ApolloTrajectoryPointProto> CreateAccPastPointsFromAvHistory(
    const TrajectoryProto& prev_traj, const AvHistory& av_history,
    const ApolloTrajectoryPointProto& plan_start_point, absl::Time plan_time) {
  int past_pt_num = kMaxAccPastPointNum;
  const auto curr_time = ToUnixDoubleSeconds(plan_time);
  const auto earliest_ts = curr_time - (past_pt_num * 2) * kTrajectoryTimeStep;
  std::vector<ApolloTrajectoryPointProto> traj_pts;
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      curr_time > prev_traj_start_time +
                      prev_traj.trajectory_point().rbegin()->relative_time() ||
      curr_time < prev_traj_start_time) {
    return traj_pts;
  }
  traj_pts.reserve(past_pt_num);
  const auto& av_poses = av_history.GetAvPoseCache();
  std::vector<double> vec_t, vec_yaw, vec_v, vec_a, vec_kappa;
  std::vector<Vec3d> vec_pos;
  vec_t.reserve(av_poses.size());
  vec_yaw.reserve(av_poses.size());
  vec_v.reserve(av_poses.size());
  vec_a.reserve(av_poses.size());
  vec_kappa.reserve(av_poses.size());
  vec_pos.reserve(av_poses.size());
  double earliest_relative_t = std::numeric_limits<double>::max();
  for (const auto& pose : av_poses) {
    if (pose.timestamp() < earliest_ts) {
      continue;
    }
    vec_t.push_back(pose.timestamp() - curr_time);
    earliest_relative_t = std::min(earliest_relative_t, vec_t.back());
    vec_yaw.push_back(NormalizeAngle(pose.yaw()));
    vec_v.push_back(pose.speed());
    vec_a.push_back(pose.accel_body().x());
    const double kappa = av_history.GetAvKappaCacheAverage().value_or(0.0);
    vec_kappa.push_back(kappa);
    vec_pos.push_back(Vec3dFromProto(pose.pos_smooth()));
  }
  if (vec_t.size() <= 1) {
    // No pose history.
    return traj_pts;
  }
  if (vec_t.back() < 0.0) {
    vec_t.push_back(0.0);
    vec_yaw.push_back(NormalizeAngle(plan_start_point.path_point().theta()));
    vec_v.push_back(plan_start_point.v());
    vec_a.push_back(plan_start_point.a());
    vec_kappa.push_back(plan_start_point.path_point().kappa());
    vec_pos.push_back(Vec3d(plan_start_point.path_point().x(),
                            plan_start_point.path_point().y(),
                            plan_start_point.path_point().z()));
  }
  std::vector<double> vec_s(vec_t.size(), 0.0);
  for (int i = vec_t.size() - 2; i >= 0; --i) {
    vec_s[i] = vec_s[i + 1] - std::hypot(vec_pos[i].x() - vec_pos[i + 1].x(),
                                         vec_pos[i].y() - vec_pos[i + 1].y());
  }
  PiecewiseLinearFunction<Vec3d, double> vec3d_plf(vec_t, vec_pos);
  PiecewiseLinearFunction<double, double, NormalizedAngleLerper<double, double>>
      yaw_plf(vec_t, vec_yaw);
  PiecewiseLinearFunction<double, double> v_plf(vec_t, vec_v);
  PiecewiseLinearFunction<double, double> a_plf(vec_t, vec_a);
  PiecewiseLinearFunction<double, double> s_plf(vec_t, vec_s);
  PiecewiseLinearFunction<double, double> kappa_plf(vec_s, vec_kappa);
  for (int i = past_pt_num; i > 0; --i) {
    const double cur_t = -i * kTrajectoryTimeStep;
    if (cur_t < earliest_relative_t) {
      continue;
    }
    const auto pos = vec3d_plf(cur_t);
    const double s = s_plf(cur_t);
    const double kappa = kappa_plf(s);
    ApolloTrajectoryPointProto point;
    point.mutable_path_point()->set_x(pos.x());
    point.mutable_path_point()->set_y(pos.y());
    point.mutable_path_point()->set_z(pos.z());
    point.mutable_path_point()->set_theta(yaw_plf(cur_t));
    point.mutable_path_point()->set_kappa(kappa);
    point.mutable_path_point()->set_s(s);
    point.set_v(v_plf(cur_t));
    point.set_a(a_plf(cur_t));
    point.set_relative_time(cur_t);
    traj_pts.push_back(std::move(point));
  }
  return traj_pts;
}

ApolloTrajectoryPointProto ComputePlanStartPointAfterLateralReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  // Keep longitudinal quantities.
  CHECK(prev_reset_planned_point.has_value());
  plan_start_point.set_v(prev_reset_planned_point->v());
  plan_start_point.set_a(prev_reset_planned_point->a());
  plan_start_point.set_j(prev_reset_planned_point->j());

  const double pose_v = pose.vel_body().x();
  const double abs_pose_v = std::abs(pose_v);
  constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
  const bool full_stop = prev_reset_planned_point->v() == 0.0 &&
                         abs_pose_v < kFullStopSpeedThreshold;
  if (full_stop) {
    plan_start_point.mutable_path_point()->set_kappa(
        prev_reset_planned_point->path_point().kappa());
    plan_start_point.mutable_path_point()->set_lambda(
        prev_reset_planned_point->path_point().lambda());
    return plan_start_point;
  }

  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (abs_pose_v < kLowSpeedThreshold) {
    // At speed lower than this, we don't trust the measured acceleration and
    // angular velocity. Reset like this are mostly when we're stopped anyway.
    // const double steer_angle =
    //     chassis.has_steering_percentage()
    //         ? chassis.steering_percentage() * 0.01 *
    //               vehicle_drive_params.max_steer_angle() /
    //               vehicle_drive_params.steer_ratio()
    //         : 0.0;  // rad.
    const double kappa =
        std::tan(front_wheel_angle) / vehicle_geom_params.wheel_base();
    plan_start_point.mutable_path_point()->set_kappa(kappa);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
  } else {
    plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                     pose_v);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
  }
  return plan_start_point;
}

ApolloTrajectoryPointProto ComputeAccPlanStartPointAfterLateralReset(
    const ApolloTrajectoryPointProto& prev_planned_now_point,
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params, double t_diff) {
  CHECK(prev_reset_planned_point.has_value());
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  const double planned_cur_v = prev_planned_now_point.v();
  const double planned_cur_a = prev_planned_now_point.a();
  const double predicted_lon_distance =
      std::max(planned_cur_v * t_diff + 0.5 * planned_cur_a * Sqr(t_diff), 0.0);
  const double pose_v = pose.vel_body().x();
  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  const double kappa =
      std::abs(pose_v) < kLowSpeedThreshold
          ? std::tan(front_wheel_angle) / vehicle_geom_params.wheel_base()
          : pose.ar_smooth().z() / pose_v;
  PathPoint pose_pt;
  pose_pt.set_x(pose_pos.x());
  pose_pt.set_y(pose_pos.y());
  pose_pt.set_theta(pose.yaw());
  pose_pt.set_s(0.0);
  pose_pt.set_kappa(kappa);
  const auto lookahead_pose =
      GetPathPointAlongCircle(pose_pt, predicted_lon_distance);

  plan_start_point.mutable_path_point()->set_x(lookahead_pose.x());
  plan_start_point.mutable_path_point()->set_y(lookahead_pose.y());
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(lookahead_pose.theta());
  plan_start_point.set_v(prev_reset_planned_point->v());
  plan_start_point.set_a(prev_reset_planned_point->a());
  plan_start_point.set_j(prev_reset_planned_point->j());
  plan_start_point.mutable_path_point()->set_kappa(kappa);
  plan_start_point.mutable_path_point()->set_lambda(0.0);
  return plan_start_point;
}

ApolloTrajectoryPointProto
ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
    const TrajectoryProto& prev_traj, const PoseProto& pose,
    double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  ApolloTrajectoryPointProto plan_start_point;
  // Reset longitudinal quantities from pose.
  plan_start_point.set_v(std::max(0.0, pose.vel_body().x()));
  plan_start_point.set_a(pose.accel_body().x());
  plan_start_point.set_j(0.0);

  if (prev_traj.trajectory_point_size() < 2) {
    // Previous trajectory too short or empty, reset path point from pose.
    plan_start_point.mutable_path_point()->set_s(0.0);
    plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
    plan_start_point.mutable_path_point()->set_theta(pose.yaw());
    constexpr double kLowSpeedThreshold = 1.0;  // m/s.
    if (std::abs(pose.vel_body().x()) < kLowSpeedThreshold) {
      // At speed lower than this, we don't trust the measured angular velocity.
      // const double steer_angle =
      //     chassis.has_steering_percentage()
      //         ? chassis.steering_percentage() * 0.01 *
      //               vehicle_drive_params.max_steer_angle() /
      //               vehicle_drive_params.steer_ratio()
      //         : 0.0;  // rad.
      const double kappa =
          std::tan(front_wheel_angle) / vehicle_geom_params.wheel_base();
      plan_start_point.mutable_path_point()->set_kappa(kappa);
      plan_start_point.mutable_path_point()->set_lambda(0.0);
    } else {
      plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                       pose.vel_body().x());
      plan_start_point.mutable_path_point()->set_lambda(0.0);
    }
    return plan_start_point;
  }

  // Reset path point from previous trajectory.
  std::vector<PathPoint> prev_traj_path_points;
  prev_traj_path_points.reserve(prev_traj.trajectory_point_size());
  for (int i = 0; i < prev_traj.trajectory_point_size(); ++i) {
    prev_traj_path_points.push_back(prev_traj.trajectory_point(i).path_point());
    // Make sure s start from zero.
    prev_traj_path_points.back().set_s(
        prev_traj.trajectory_point(i).path_point().s() -
        prev_traj.trajectory_point(0).path_point().s());
  }
  DiscretizedPath prev_traj_path(std::move(prev_traj_path_points));
  const auto pose_sl = prev_traj_path.XYToSL(
      Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()));
  *plan_start_point.mutable_path_point() = prev_traj_path.Evaluate(pose_sl.s);
  plan_start_point.mutable_path_point()->set_s(0.0);
  return plan_start_point;
}

SelectorParamsProto LoadSelectorParamsFromFile(
    const std::string& file_address) {
  SelectorParamsProto selector_params_proto;
  if (!file_util::TextFileToProto(file_address, &selector_params_proto)) {
    CHECK(false) << "Read auto tuned selector params as text file failed!!!!";
  }
  LOG_INFO << "New auto tuned selector params are used.";
  return selector_params_proto;
}

void FillDecisionConstraintDebugInfo(const ConstraintManager& constraint_mgr,
                                     ConstraintProto* constraint) {
  CHECK_NOTNULL(constraint);
  constraint->Clear();
  for (const auto& stop_line : constraint_mgr.StopLine()) {
    constraint->add_stop_line()->CopyFrom(stop_line);
  }
  for (const auto& speed_region : constraint_mgr.SpeedRegion()) {
    constraint->add_speed_region()->CopyFrom(speed_region);
  }
  for (const auto& path_speed_region : constraint_mgr.PathSpeedRegion()) {
    constraint->add_path_speed_region()->CopyFrom(path_speed_region);
  }
  for (const auto& path_stop_line : constraint_mgr.PathStopLine()) {
    constraint->add_path_stop_line()->CopyFrom(path_stop_line);
  }
  for (const auto& avoid_line : constraint_mgr.AvoidLine()) {
    constraint->add_avoid_line()->CopyFrom(avoid_line);
  }
  for (const auto& speed_profile : constraint_mgr.SpeedProfiles()) {
    constraint->add_speed_profile()->CopyFrom(speed_profile);
  }

  const auto& traffic_gap = constraint_mgr.TrafficGap();
  if (traffic_gap.leader_id.has_value()) {
    constraint->mutable_traffic_gap()->set_leader_id(*traffic_gap.leader_id);
  }
  if (traffic_gap.follower_id.has_value()) {
    constraint->mutable_traffic_gap()->set_follower_id(
        *traffic_gap.follower_id);
  }
}

}  // namespace planning
}  // namespace st
