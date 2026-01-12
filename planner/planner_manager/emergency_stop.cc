

#include "planner/planner_manager/emergency_stop.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <utility>

#include "absl/status/statusor.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "planner/planner_manager/planner_defs.h"
#include "plan_common/trajectory_util.h"
#include "plan_common/util/path_util.h"
//#include "plan_common/util/objects_view.h"
#include "plan_common/util/status_macros.h"

namespace st {
namespace planning {
namespace aeb {
namespace {

void InsertLineSidePoints(double potential_risk_area_half_width,
                          const TrajectoryPoint& center_point,
                          double steer_angle,
                          std::vector<Vec2d>* right_side_points,
                          std::vector<Vec2d>* left_side_points) {
  CHECK_NOTNULL(right_side_points);
  CHECK_NOTNULL(left_side_points);
  Vec2d tangent = Vec2d::FastUnitFromAngle(
      NormalizeAngle(center_point.theta() - steer_angle));

  const Vec2d right_edge_point =
      center_point.pos() + tangent.Perp() * potential_risk_area_half_width;
  const Vec2d left_edge_point =
      center_point.pos() - tangent.Perp() * potential_risk_area_half_width;
  right_side_points->push_back(right_edge_point);
  left_side_points->push_back(left_edge_point);
}

}  // namespace

// std::optional<EmergencyStopInfo> CheckEmergencyStopByCircularMotion(
//     const VehicleGeometryParamsProto& vehicle_geom,
//     const EmergencyStopParamsProto& emergency_stop_params,
//     const VehicleDriveParamsProto& vehicle_drive_params,
//     const PoseProto& vehicle_pose, const ObjectsProto& objects_proto,
//     const Chassis& chassis) {
//   if (!chassis.has_steering_percentage()) {
//     return std::nullopt;
//   }
//   if (std::isnan(chassis.steering_percentage())) {
//     return std::nullopt;
//   }

//   auto generate_potential_collision_area =
//       [](double detect_distance, double width_buffer, const auto&
//       vehicle_point,
//          const auto& vehicle_param) {
//         Vec2d vec_to_center(
//             vehicle_param.front_edge_to_center() + detect_distance / 2.0,
//             0.0);
//         Vec2d center(vehicle_point.pos() +
//                      vec_to_center.FastRotate(vehicle_point.theta()));
//         return Polygon2d(Box2d(center, vehicle_point.theta(),
//         detect_distance,
//                                vehicle_param.width() + 2.0 * width_buffer));
//       };
//   auto to_object_polygon = [](const auto& object_proto) {
//     std::vector<Vec2d> vertices;
//     vertices.reserve(object_proto.contour_size());
//     for (int i = 0; i < object_proto.contour_size(); ++i) {
//       vertices.push_back(Vec2dFromProto(object_proto.contour(i)));
//     }
//     return Polygon2d(std::move(vertices));
//   };

//   TrajectoryPoint veh_center_point;
//   veh_center_point.set_pos(
//       Vec2d(vehicle_pose.pos_smooth().x(), vehicle_pose.pos_smooth().y()));
//   veh_center_point.set_theta(vehicle_pose.yaw());
//   veh_center_point.set_v(
//       Vec2d(vehicle_pose.vel_smooth().x(), vehicle_pose.vel_smooth().y())
//           .norm());
//   veh_center_point.set_t(0.0);
//   veh_center_point.set_kappa(0.0);
//   const Vec2d veh_center_pos(veh_center_point.pos()[0],
//                              veh_center_point.pos()[1]);

//   const double detect_distance =
//       std::clamp(0.5 * std::fabs(veh_center_point.v()),
//                  emergency_stop_params.min_detect_distance(),
//                  emergency_stop_params.max_detect_distance());
//   constexpr double kSingleDetectDistance = 0.25;
//   constexpr double kVehicleWidthBuffer = 0.1;
//   constexpr double kEps = 0.1;
//   std::vector<Vec2d> potential_risk_area_r_points;
//   std::vector<Vec2d> potential_risk_area_l_points;

//   const double steer_angle = chassis.steering_percentage() / 100.0 *
//                              vehicle_drive_params.max_steer_angle() /
//                              vehicle_drive_params.steer_ratio();  // rad.

//   const Vec2d vec_to_center(vehicle_geom.front_edge_to_center(), 0.0);
//   constexpr double kMaxVehCenterRadius = 1e6;

//   const double veh_center_radius =
//       std::clamp(vehicle_geom.wheel_base() / std::tan(steer_angle),
//                  -kMaxVehCenterRadius, kMaxVehCenterRadius);

//   const Vec2d veh_front_edge_pos(
//       veh_center_pos + vec_to_center.FastRotate(veh_center_point.theta()));
//   const double veh_front_edge_steer =
//       std::atan(vehicle_geom.front_edge_to_center() / veh_center_radius);

//   const double veh_front_edge_radius_abs =
//       Hypot(vehicle_geom.front_edge_to_center(), veh_center_radius);
//   const double veh_front_edge_radius =
//       std::copysign(veh_front_edge_radius_abs, veh_center_radius);
//   const double veh_front_edge_kappa = 1.0 / veh_front_edge_radius;
//   TrajectoryPoint veh_front_edge_point;  // s is not important.
//   veh_front_edge_point.set_pos(veh_front_edge_pos);
//   veh_front_edge_point.set_theta(veh_center_point.theta() +
//                                  veh_front_edge_steer);
//   if (std::isnan(veh_front_edge_point.theta())) {
//     return std::nullopt;
//   }

//   EmergencyStopInfo res;

//   veh_front_edge_point.set_kappa(veh_front_edge_kappa);
//   TrajectoryPoint curr_veh_front_edge_point = veh_front_edge_point;

//   const double potential_risk_area_half_width =
//       vehicle_geom.width() * 0.5 + kVehicleWidthBuffer;
//   std::vector<Vec2d> aeb_ref_path_pos;
//   std::vector<double> aeb_ref_path_theta;
//   double sum_discrete_point_length = 0.0;

//   PathPoint veh_front_edge_path_point;
//   veh_front_edge_path_point.set_x(veh_front_edge_point.pos().x());
//   veh_front_edge_path_point.set_y(veh_front_edge_point.pos().y());
//   veh_front_edge_path_point.set_theta(veh_front_edge_point.theta());
//   veh_front_edge_path_point.set_kappa(veh_front_edge_point.kappa());
//   while (sum_discrete_point_length < detect_distance) {
//     PathPoint curr_path_point = GetPathPointAlongCircle(
//         veh_front_edge_path_point, sum_discrete_point_length);
//     curr_veh_front_edge_point.set_pos(ToVec2d(curr_path_point));
//     curr_veh_front_edge_point.set_theta(curr_path_point.theta());
//     curr_veh_front_edge_point.set_kappa(curr_path_point.kappa());
//     InsertLineSidePoints(potential_risk_area_half_width,
//                          curr_veh_front_edge_point, veh_front_edge_steer,
//                          &potential_risk_area_r_points,
//                          &potential_risk_area_l_points);
//     aeb_ref_path_pos.push_back(ToVec2d(curr_path_point));
//     aeb_ref_path_theta.push_back(NormalizeAngle(
//         curr_veh_front_edge_point.theta() - veh_front_edge_steer));
//     sum_discrete_point_length += kSingleDetectDistance;
//   }
//   sum_discrete_point_length -= kSingleDetectDistance;
//   if (sum_discrete_point_length < detect_distance - kEps) {
//     const PathPoint terminal_path_point =
//         GetPathPointAlongCircle(veh_front_edge_path_point, detect_distance);
//     TrajectoryPoint terminal_point;
//     terminal_point.set_pos(ToVec2d(terminal_path_point));
//     terminal_point.set_theta(terminal_path_point.theta());
//     InsertLineSidePoints(potential_risk_area_half_width, terminal_point,
//                          veh_front_edge_steer, &potential_risk_area_r_points,
//                          &potential_risk_area_l_points);
//   }

//   std::reverse(potential_risk_area_l_points.begin(),
//                potential_risk_area_l_points.end());
//   std::vector<Vec2d> potential_risk_area_points =
//       std::move(potential_risk_area_r_points);
//   potential_risk_area_points.insert(potential_risk_area_points.end(),
//                                     potential_risk_area_l_points.begin(),
//                                     potential_risk_area_l_points.end());
//   Polygon2d potential_risk_area(std::move(potential_risk_area_points));
//   res.risk_area = potential_risk_area;

//   ASSIGN_OR_DIE(const auto aeb_ref_path,
//                 BuildBruteForceFrenetFrame(aeb_ref_path_pos,
//                                            /*down_sample_raw_points=*/true));
//   for (const auto& object_proto : objects_proto.objects()) {
//     if (object_proto.type() == ObjectType::OT_FOD ||
//         object_proto.type() == ObjectType::OT_VEGETATION) {
//       continue;
//     }
//     FrenetCoordinate object_sl;
//     Vec2d ref_normal;
//     std::pair<int, int> index_pair;
//     double alpha = 0.0;
//     aeb_ref_path.XYToSL(Vec2dFromProto(object_proto.pos()), &object_sl,
//                         &ref_normal, &index_pair, &alpha);
//     const double object_proj_point_theta =
//         LerpAngle(aeb_ref_path_theta[index_pair.first],
//                   aeb_ref_path_theta[index_pair.second], alpha);
//     const Vec2d unit_vec = Vec2d::FastUnitFromAngle(object_proj_point_theta);
//     const double veh_front_edge_center_v =
//         veh_center_point.v() * veh_front_edge_radius / veh_center_radius;
//     const double relative_v =
//     Vec2dFromProto(object_proto.vel()).dot(unit_vec) -
//                               veh_front_edge_center_v;
//     if (relative_v > -emergency_stop_params.vel_threshold()) {
//       continue;
//     }
//     auto risk_area = potential_risk_area;
//     const auto& object_polygon = to_object_polygon(object_proto);
//     if (object_proto.type() == ObjectType::OT_UNKNOWN_STATIC ||
//         object_proto.parked()) {
//       risk_area = generate_potential_collision_area(
//           emergency_stop_params.min_detect_distance(), 0.0, veh_center_point,
//           vehicle_geom);
//     }

//     if (risk_area.HasOverlap(object_polygon)) {
//       // LOG_EVERY_N_SEC(WARNING, 1.0) << "Ego vehicle potential risk area "
//       //                                   "has overlap with object id:"
//       //                                << object_proto.id();
//       res.emergency_stop = true;
//       res.object_id = object_proto.id();
//       res.object_type = object_proto.type();
//       return res;
//     }
//   }
//   return res;
// }

// For Planner 3.0
std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto& plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const EmergencyStopParamsProto& emergency_stop_params,
    const MotionConstraintParamsProto& motion_constraint_params) {
  VLOG(3) << "Generate emergency stop trajectory.";
  TrajectoryPoint plan_start_traj_point;
  plan_start_traj_point.FromProto(plan_start_point);

  return aeb::GenerateStopTrajectory(
      path_s_inc_from_prev, reset, /*forward=*/true,
      motion_constraint_params.max_deceleration(), motion_constraint_params,
      plan_start_traj_point, prev_traj_points);
}

std::vector<ApolloTrajectoryPointProto> GenerateStopTrajectory(
    double init_s, bool reset, bool forward, double max_deceleration,
    const MotionConstraintParamsProto& motion_constraint_params,
    const TrajectoryPoint& plan_start_traj_point,
    const std::vector<ApolloTrajectoryPointProto>& prev_trajectory) {
  ApolloTrajectoryPointProto curr_point;
  plan_start_traj_point.ToProto(&curr_point);
  auto prev_traj = prev_trajectory;

  if (!forward) {
    curr_point.mutable_path_point()->set_theta(
        NormalizeAngle(curr_point.path_point().theta() + M_PI));
    curr_point.mutable_path_point()->set_kappa(
        -curr_point.path_point().kappa());
    curr_point.set_v(-curr_point.v());
    curr_point.set_a(-curr_point.a());
    curr_point.set_j(-curr_point.j());
    curr_point.mutable_path_point()->set_s(-curr_point.path_point().s());
    for (auto& pt : prev_traj) {
      pt.mutable_path_point()->set_theta(
          NormalizeAngle(pt.path_point().theta() + M_PI));
      pt.mutable_path_point()->set_kappa(-pt.path_point().kappa());
      pt.set_v(-pt.v());
      pt.set_a(-pt.a());
      pt.set_j(-pt.j());
      pt.mutable_path_point()->set_s(-pt.path_point().s());
    }
  }

  std::vector<ApolloTrajectoryPointProto> output_trajectory;
  double accumulate_s = 0.0;
  constexpr double kMinSpeed = 1e-6;
  constexpr double kMinDist = 1e-6;
  for (int i = 0; i < kTrajectorySteps; ++i) {
    const double dist =
        std::max(kMinDist, curr_point.v() * kTrajectoryTimeStep +
                               0.5 * max_deceleration * kTrajectoryTimeStep *
                                   kTrajectoryTimeStep);
    accumulate_s += dist;
    const double accumulate_s_at_prev_traj = accumulate_s + init_s;
    ApolloTrajectoryPointProto next_point;
    const auto& curr_path_point = curr_point.path_point();
    if (prev_traj.empty() || reset ||
        accumulate_s_at_prev_traj >= prev_traj.rbegin()->path_point().s()) {
      *next_point.mutable_path_point() =
          GetPathPointAlongCircle(curr_path_point, dist);
    } else {
      next_point =
          QueryApolloTrajectoryPointByS(prev_traj, accumulate_s_at_prev_traj);
    }
    next_point.set_relative_time(curr_point.relative_time() +
                                 kTrajectoryTimeStep);
    next_point.set_v(std::max(
        kMinSpeed, curr_point.v() + kTrajectoryTimeStep * max_deceleration));
    next_point.set_a(std::max(max_deceleration,
                              -1.0 * next_point.v() / kTrajectoryTimeStep));
    curr_point.set_j(
        std::clamp((next_point.a() - curr_point.a()) / kTrajectoryTimeStep,
                   motion_constraint_params.max_decel_jerk(),
                   motion_constraint_params.max_accel_jerk()));
    output_trajectory.push_back(curr_point);
    curr_point = next_point;
  }

  if (!forward) {
    for (auto& pt : output_trajectory) {
      pt.mutable_path_point()->set_theta(
          NormalizeAngle(pt.path_point().theta() + M_PI));
      pt.mutable_path_point()->set_kappa(-pt.path_point().kappa());
      pt.set_v(-pt.v());
      pt.set_a(-pt.a());
      pt.set_j(-pt.j());
      pt.mutable_path_point()->set_s(-pt.path_point().s());
    }
  }

  return output_trajectory;
}

// std::shared_ptr<const ObjectsProto> ExportObjectsForAEB(
//     const std::shared_ptr<const ObjectsProto>& real_objects,
//     const std::shared_ptr<const ObjectsProto>& virtual_objects) {
//   ObjectsView objects_view;
//   if (real_objects != nullptr) {
//     objects_view.UpdateObjects(ObjectsProto::SCOPE_REAL, real_objects);
//   }
//   if (virtual_objects != nullptr) {
//     objects_view.UpdateObjects(ObjectsProto::SCOPE_VIRTUAL, virtual_objects);
//   }
//   return objects_view.ExportAllObjectsProto();
// }

// std::optional<EmergencyStopInfo> CheckEmergencyStop(
//     const PlannerParamsProto& planner_params, const PlannerInput& input) {
//   // ("CheckEmergencyStop");

//   const auto objects_proto =
//       aeb::ExportObjectsForAEB(input.real_objects, input.virtual_objects);

//   auto eme_stop_or = aeb::CheckEmergencyStopByCircularMotion(
//       input.vehicle_params.vehicle_geometry_params(),
//       planner_params.emergency_stop_params(),
//       input.vehicle_params.vehicle_drive_params(), *input.pose,
//       *objects_proto, *input.chassis);

//   return eme_stop_or;
// }

}  // namespace aeb
}  // namespace planning
}  // namespace st
