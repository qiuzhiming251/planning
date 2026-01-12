

#include "planner/speed_optimizer/ignore_decider.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "plan_common/log.h"

// #include "global/trace.h"
// #include "lite/check.h"
// #include "lite/logging.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/util/path_util.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_object_state.h"
// #include "semantic_map.pb.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/overlap_info.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_point.h"
#include "planner/speed_optimizer/decider/pre_brake_util.h"
#include "planner/speed_optimizer/speed_finder_util.h"
#include "plan_common/util/perception_util.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st {
namespace planning {
namespace {
const PiecewiseLinearFunction<double> kLaterFactorForFarawayObject(
    {80.0, 90.0, 130.0, 160.0}, {0.0, 0.1, 0.3, 0.5});

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 0.01;
constexpr double kEgoCreepInteractionVelThres = 7.5;
constexpr double kOtherCreepInteractionVelTrhes = 5.0;
constexpr double kMaxOncomingBrakeDecel = -0.5;          // mpss
constexpr double kMaxConfortOncomingBrakeDecel = -2.0;   // mpss
constexpr double kMaxCyclistCutoutDistacne = 25.0;       // m
constexpr double kMaxVehicleCutoutDistacne = 45.0;       // m
constexpr double kBufferlat = 0.35;                      // m
constexpr double kCutoutThetaThres = M_PI / 20;          // deg
constexpr double kMaxHalfLaneWidth = 2.7;                // m
constexpr double kFollowTimeHeadwayBias = 1.5;           // s
constexpr double kFollowHeadWayDistanceBias = 5.0;       // m
constexpr double kObjOncomingAngleThres = M_PI * 3 / 4;  // m
constexpr double kCutinCipvMaxTimeThres = 2.0;           // s
constexpr double kMaxFollowCipvBreakDecel = 3.0;         // mpss
constexpr double kLaneCrossCutoutAngleThres = M_PI / 8;
constexpr double kFarawayOncomingObjDistThres = 70.0;  // m
constexpr double kSearchBuffer = 0.2;                  // m.
constexpr double kRequiredLateralGap = 0.2;            // m.
constexpr double kLateralIgnoreDistThres = 80.0;       // m.
constexpr double kLaneParallCutinAngleThres = M_PI / 4;

// std::vector<StPoint> GenerateAvEmergencyStopStPoints(double init_s,
//                                                      double init_v,
//                                                      int traj_steps) {
//   CHECK_GE(init_v, 0.0);
//   constexpr double kEmergencyStopAccel = -2.0;  // m/s^2.
//   // In line with path resoluation but can differ from it.
//   constexpr double kCheckCollisionTime = 4.0;  // s.
//   double curr_s = init_s;
//   double curr_v = init_v;
//   std::vector<StPoint> emergency_stop_points;
//   emergency_stop_points.reserve(traj_steps);
//   double t = 0.0;
//   for (int i = 0; i < traj_steps && t < kCheckCollisionTime;
//        ++i, t += kTrajectoryTimeStep) {
//     emergency_stop_points.emplace_back(curr_s, t);
//     curr_s +=
//         std::max(0.0, curr_v * kTrajectoryTimeStep +
//                           0.5 * kEmergencyStopAccel *
//                           Sqr(kTrajectoryTimeStep));
//     curr_v = std::max(0.0, curr_v + kEmergencyStopAccel *
//     kTrajectoryTimeStep);
//   }
//   return emergency_stop_points;
// }

// bool IgnoreHitAvEmergencyStopTrajectoryStBoundary(
//     const std::vector<StPoint>& emergency_stop_points,
//     const StBoundary& st_boundary) {
//   // Don't ignore st-boundaries that would hit path in a short time.
//   constexpr double kShortTime = 1.0;        // s.
//   constexpr double kShortTimeForPed = 0.8;  // s.
//   const double short_time =
//       st_boundary.object_type() == StBoundaryProto::PEDESTRIAN
//           ? kShortTimeForPed
//           : kShortTime;
//   if (st_boundary.min_t() < short_time) return false;

//   constexpr double kCheckCollisionSBuffer = 0.5;  // m.
//   bool st_boundary_passed = false;
//   bool is_unprotected_left_turn =
//       st_boundary.overlap_meta().has_value() &&
//       st_boundary.overlap_meta()->is_unprotected_left_turn();
//   const auto start_iter = std::lower_bound(
//       emergency_stop_points.begin(), emergency_stop_points.end(),
//       st_boundary.min_t(),
//       [](const StPoint& point, double t) { return point.t() < t; });
//   for (auto it = start_iter; it < emergency_stop_points.end(); ++it) {
//     const auto& st_point = *it;
//     const double curr_s = st_point.s();
//     const double t = st_point.t();
//     if (t < 5.0 && (is_unprotected_left_turn ||
//                     st_boundary.object_type() ==
//                     StBoundaryProto::PEDESTRIAN)) {
//       VLOG(2) << "[speed finder] id: " << st_boundary.id()
//               << " is_unprotected_left_turn || PEDESTRIAN, t:" << t
//               << "st_boundary.min_t(): " << st_boundary.min_t();
//       return false;
//     }
//     const auto s_range = st_boundary.GetBoundarySRange(t);
//     if (!s_range.has_value()) {
//       if (!st_boundary_passed) {
//         continue;
//       } else {
//         return false;
//       }
//     }
//     st_boundary_passed = true;
//     if (curr_s > s_range->second + kCheckCollisionSBuffer) {
//       VLOG(2) << "[speed finder] St-boundary " << st_boundary.id()
//               << " is ignored because AV would hit it at t = " << t
//               << " s = " << curr_s << " by emergency stop.";
//       return true;
//     }
//   }
//   return false;
// }

bool IsCollisionHead(const VehicleShapeBasePtr& av_shape,
                     const Polygon2d& obj_contour, const PathPoint& path_point,
                     const VehicleGeometryParamsProto& vehicle_geometry_params,
                     double lat_buffer, double lon_buffer) {
  const Polygon2d av_polygon = Polygon2d(
      av_shape->GetCornersWithBufferCounterClockwise(lat_buffer, lon_buffer),
      /*is_convex=*/true);
  Polygon2d overlap_polygon;
  if (!av_polygon.ComputeOverlap(obj_contour, &overlap_polygon)) {
    return false;
  } else {
    const Vec2d av_tangent = Vec2d::FastUnitFromAngle(path_point.theta());
    Vec2d front, back;
    overlap_polygon.ExtremePoints(av_tangent, &back, &front);
    constexpr double kVehicleHeadRatio = 0.25;
    const double head_lower_limit =
        vehicle_geometry_params.front_edge_to_center() -
        vehicle_geometry_params.length() * kVehicleHeadRatio + lon_buffer;

    if ((front - Vec2d(path_point.x(), path_point.y())).dot(av_tangent) >
        head_lower_limit) {
      return true;
    }
  }
  return false;
}

bool IsCollisionMirrors(
    const VehicleShapeBasePtr& av_shape, const Polygon2d& obj_contour,
    const SpacetimeObjectTrajectory& st_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const double buffer = st_traj.required_lateral_gap();
  const auto [min_mirror_height, max_mirror_height] =
      ComputeMinMaxMirrorAverageHeight(vehicle_geometry_params);
  const bool consider_mirrors =
      IsConsiderMirrorObject(st_traj.planner_object().object_proto(),
                             min_mirror_height, max_mirror_height);
  return consider_mirrors &&
         (av_shape->LeftMirrorHasOverlapWithBuffer(obj_contour, buffer) ||
          av_shape->RightMirrorHasOverlapWithBuffer(obj_contour, buffer));
}

bool IsObjectBehindHead(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PathPoint& path_point, const Polygon2d& obj_contour) {
  const Vec2d av_tangent = Vec2d::FastUnitFromAngle(path_point.theta());
  Vec2d front, back;
  obj_contour.ExtremePoints(av_tangent, &back, &front);

  return (front - Vec2d(path_point.x(), path_point.y())).dot(av_tangent) <
         vehicle_geometry_params.front_edge_to_center();
}

bool IgnoreHitAvCurrentPositionStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PathPoint& current_path_point, const StBoundary& st_boundary,
    const std::vector<PathPointSemantic>& path_semantics, double current_v) {
  const double first_overlap_time = st_boundary.bottom_left_point().t();
  constexpr double kObjMinVel = Kph2Mps(5.0);
  constexpr double kAvMinVel = Kph2Mps(2.0);
  constexpr double kTimeThreshold = 2.0;

  const bool is_merge_scenario =
      st_boundary.obj_scenario_info().relationship == Relationship::Merge;
  const bool is_object_at_left_side =
      st_boundary.obj_scenario_info().bearing == Bearing::Left;
  const bool is_right_interaction =
      path_semantics.empty() ? false
                             : path_semantics.front().lane_semantic ==
                                   LaneSemantic::INTERSECTION_RIGHT_TURN;
  if (is_merge_scenario && is_object_at_left_side && is_right_interaction) {
    return false;
  }

  if (st_boundary.bottom_left_point().s() <= kEpsilon &&
      first_overlap_time >= kEpsilon) {
    return true;
  } else {
    CHECK(st_boundary.traj_id().has_value());
    const auto& traj_id = *st_boundary.traj_id();
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

    const auto& overlap_infos = st_boundary.overlap_infos();
    CHECK(!overlap_infos.empty());
    constexpr double kCheckTime = 0.5;  // s.
    const double check_time_limit = first_overlap_time + kCheckTime;
    for (auto& overlap_info : overlap_infos) {
      if (overlap_info.time > check_time_limit) {
        break;
      }
      if (overlap_info.av_start_idx == 0) {
        // NOTE: Buffer may be time-varying in the future.
        const bool is_stationary_object = traj->is_stationary();
        const double buffer =
            is_stationary_object ? 0.0 : traj->required_lateral_gap();
        const auto& obj_contour = traj->states()[overlap_info.obj_idx].contour;
        if (is_stationary_object) {
          return av_shapes[0]->MainBodyHasOverlapWithBuffer(obj_contour, buffer,
                                                            buffer) &&
                 !IsCollisionHead(av_shapes[0], obj_contour, current_path_point,
                                  vehicle_geometry_params, buffer, buffer);
        } else {
          return IsObjectBehindHead(vehicle_geometry_params, current_path_point,
                                    obj_contour);
        }
      }
    }
  }
  return false;
}

inline bool IsChangingSameDirWithAV(const StBoundary& st_boundary, bool lc_left,
                                    LaneChangeStage lc_stage) {
  const auto& obj_sl_info = st_boundary.obj_sl_info();
  if (LaneChangeStage::LCS_NONE == lc_stage || !obj_sl_info.has_value()) {
    return false;
  }

  return lc_left ? obj_sl_info->dl < 0.0 : obj_sl_info->dl > 0.0;
}

bool IgnoreBackCutInStBoundary(
    const StBoundary& st_boundary,
    const PathPointSemantic& current_path_semantic, const DiscretizedPath& path,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    bool on_left_turn_waiting_lane, bool lc_left, LaneChangeStage lc_stage) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  const auto current_lane_semantic = current_path_semantic.lane_semantic;
  if (current_lane_semantic == LaneSemantic::NONE ||
      current_lane_semantic == LaneSemantic::INTERSECTION_UTURN) {
    return false;
  }
  if ((st_boundary.overlap_meta()->source() == StOverlapMetaProto::AV_CUTIN &&
       !IsChangingSameDirWithAV(st_boundary, lc_left, lc_stage)) ||
      st_boundary.overlap_meta()->source() == StOverlapMetaProto::OTHER) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* st_traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

  const auto& last_overlap_info = st_boundary.overlap_infos().back();
  const double last_overlap_obj_heading =
      st_traj->states()[last_overlap_info.obj_idx].box.heading();
  const double last_overlap_ego_heading =
      path[(last_overlap_info.av_start_idx + last_overlap_info.av_end_idx) / 2]
          .theta();

  const double cut_out_angle = std::abs(
      NormalizeAngle(last_overlap_obj_heading - last_overlap_ego_heading));

  if (cut_out_angle > kLaneCrossCutoutAngleThres) {
    return false;
  }

  if (!st_boundary.overlap_meta()->has_front_most_projection_distance()) {
    return false;
  }
  const double front_most_projection_distance =
      st_boundary.overlap_meta()->front_most_projection_distance();
  // To imitate human drivers, objects that not pass the center of AV will be
  // ignored.
  constexpr double kDriverAwarenessFactor = 0.5;
  const double driver_awareness_area =
      vehicle_geometry_params.front_edge_to_center() -
      vehicle_geometry_params.length() * kDriverAwarenessFactor;
  const bool behind_current_path_point_center =
      front_most_projection_distance < driver_awareness_area;

  if (current_lane_semantic == LaneSemantic::ROAD) {
    return (st_boundary.overlap_meta()->source() ==
                StOverlapMetaProto::OBJECT_CUTIN ||
            (st_boundary.overlap_meta()->priority() ==
             StOverlapMetaProto::HIGH) ||
            IsChangingSameDirWithAV(st_boundary, lc_left, lc_stage)) &&
           behind_current_path_point_center;
  } else if (current_lane_semantic == LaneSemantic::INTERSECTION_STRAIGHT) {
    return behind_current_path_point_center;
  } else if (current_lane_semantic == LaneSemantic::INTERSECTION_LEFT_TURN ||
             current_lane_semantic == LaneSemantic::INTERSECTION_RIGHT_TURN) {
    // If AV is turning left/right at intersection, check if object current
    // contour is behind current path point. If so, ignore back cut-in object.
    // Not valid for LOW priority LANE-CROSS/LANE-MERGE & AV merging & crossing
    // straight lane.
    if ((st_boundary.overlap_meta()->source() !=
             StOverlapMetaProto::OBJECT_CUTIN &&
         st_boundary.overlap_meta()->priority() == StOverlapMetaProto::LOW) ||
        (st_boundary.overlap_meta()->has_obj_lane_direction() &&
         st_boundary.overlap_meta()->obj_lane_direction() ==
             StOverlapMetaProto::NO_TURN)) {
      return false;
    }
    if (behind_current_path_point_center) return true;
    // For left-turn waiting area.
    if (on_left_turn_waiting_lane) {
      // If AV is turning to non-innermost lane or on left turn waiting lane, we
      // ignore those behind AV front.
      const bool behind_current_path_point_front =
          front_most_projection_distance <
          vehicle_geometry_params.front_edge_to_center();
      if (behind_current_path_point_front) {
        return true;
      }
    }
    // Other cases, don't ignore front cut-in objects.
    return false;
  }

  LOG_FATAL << "Should not be here.";
  return false;
}

#define DEBUG_TURN_LEFT_BACK_IGNORE (0)

bool IgnoreTurnLeftBackCutInStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PathPoint& current_path_point, const StBoundary& st_boundary,
    const std::vector<DrivingProcess>& driving_process_seq,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  constexpr double kDeltaTheta = ad_byd::planning::Constants::DEG2RAD * 5.0;

  if (driving_process_seq.size() == 0) return false;
  if (!st_boundary.object_id().has_value()) return false;

  const auto& obj_sl_info = st_boundary.obj_sl_info();
  if (!obj_sl_info.has_value()) {
    return false;
  }

#if DEBUG_TURN_LEFT_BACK_IGNORE
  std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
            << " ]:  obj_max_s " << obj_frenet_polygon.s_max << " dl " << obj_dl
            << " ego_front_to_center "
            << vehicle_geometry_params.front_edge_to_center()
            << " ego_back_to_center "
            << vehicle_geometry_params.back_edge_to_center() << std::endl;
#endif

  bool is_av_turn_left = false;
  for (int i = 0; i < 2 && i < driving_process_seq.size(); i++) {
    if (driving_process_seq[i].lane_semantic ==
        LaneSemantic::INTERSECTION_LEFT_TURN) {
      is_av_turn_left = true;
      break;
    }
  }

  const PlannerObject* obj =
      st_traj_mgr.FindObjectByObjectId(st_boundary.object_id().value());

  if (nullptr == obj) return false;

  double delta_theta = std::fabs(ad_byd::planning::math::AngleDiff(
      current_path_point.theta(), obj->pose().theta()));

#if DEBUG_TURN_LEFT_BACK_IGNORE
  std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
            << " ]: is_av_turn_left " << is_av_turn_left << " delta_theta "
            << ad_byd::planning::Constants::RAD2DEG * delta_theta << std::endl;
#endif

  if (!is_av_turn_left) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " F]: reason:  is_av_not_turn_left " << std::endl;
    std::cout << std::endl;
#endif
    return false;
  }

  const double ego_front_to_center =
      vehicle_geometry_params.front_edge_to_center();
  const double ego_back_to_center =
      vehicle_geometry_params.back_edge_to_center();

  double ego_half_s =
      ego_front_to_center - vehicle_geometry_params.length() * 0.25;

  if (obj_sl_info->frenet_polygon.s_max > ego_half_s) {
    return false;
  }

  bool is_agent_back = false;
  bool is_agent_back_side = false;

  if (obj_sl_info->frenet_polygon.s_max < -ego_back_to_center) {
    is_agent_back = true;
  } else if (obj_sl_info->frenet_polygon.s_max <= ego_half_s) {
    is_agent_back_side = true;
  }

  if (obj_sl_info->dl >= 0.0) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " F]: reason:  obj_dl >= 0.0 " << std::endl;
    std::cout << std::endl;
#endif
    return false;
  }

#if DEBUG_TURN_LEFT_BACK_IGNORE
  std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
            << " ]: is_agent_back " << is_agent_back << " is_agent_back_side "
            << is_agent_back_side << std::endl;
#endif
  if (is_agent_back_side && delta_theta < kDeltaTheta) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " T]: reason: back_side ignore " << std::endl;
    std::cout << std::endl;
#endif
    return true;
  }

  if (is_agent_back) {
#if DEBUG_TURN_LEFT_BACK_IGNORE
    std::cout << "[turn_left_back_ignore-- " << st_boundary.object_id().value()
              << " T]: reason: back ignore " << std::endl;
    std::cout << std::endl;
#endif
    return true;
  }

  return false;
}

bool IgnoreOnRoadParallelCutInStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const FrenetBox& av_frenet_box, bool av_in_drive_passage_lane,
    const DrivePassage& drive_passage, double current_v,
    const PlannerSemanticMapManager& psmm,
    const std::vector<DrivingProcess>& driving_process_seq,
    const std::vector<mapping::ElementId>& upcoming_merge_lane_ids,
    bool ignore_late_parallel_cut_in_vehicle, const StBoundary& st_boundary) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }
  // Check if AV is on road and entirely enclosed by drive passage lane
  // boundaries.
  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& fo_info = overlap_infos.front();

  if (path_semantics[fo_info.av_start_idx].lane_semantic !=
      LaneSemantic::ROAD) {
    return false;
  }
  if (!av_in_drive_passage_lane) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto obj_frenet_box_or =
      drive_passage.QueryFrenetBoxAtContour(traj->contour());
  if (!obj_frenet_box_or.ok()) {
    return false;
  }
  const auto& obj_frenet_box = *obj_frenet_box_or;

  constexpr double kLateCutInTimeThres = 3.5;  // s.
  if (!ignore_late_parallel_cut_in_vehicle ||
      st_boundary.object_type() != StBoundaryProto::VEHICLE ||
      st_boundary.min_t() < kLateCutInTimeThres) {
    // Condition 1): Object s_max is among a relative speed related AV frenet s
    // range. The assumption here is: if object is faster than AV, it would
    // possibly cut in AV when not fully surpass it; otherwise, it would not cut
    // in AV if not fully supass it. The parameter here can be further fine
    // tuned.
    const double object_length = obj_frenet_box.s_max - obj_frenet_box.s_min;
    // TODO: Use different params for vehicle & cyclist.
    const PiecewiseLinearFunction rel_speed_surpass_dist_plf(
        std::vector<double>({-2.0, 0.0, 2.0}),
        std::vector<double>({0.8 * object_length, 0.3 * object_length, 0.0}));
    constexpr double kRearSRangeOffset = 0.5;  // m.
    if (obj_frenet_box.s_max < av_frenet_box.s_min - kRearSRangeOffset ||
        obj_frenet_box.s_max >
            av_frenet_box.s_max +
                rel_speed_surpass_dist_plf(traj->planner_object().pose().v() -
                                           current_v)) {
      return false;
    }
  }

  // Condition 2): Object heading is around its nearby drive passage center.
  if (!st_boundary.overlap_meta()->has_theta_diff()) return false;
  const auto& theta_diff = st_boundary.overlap_meta()->theta_diff();
  constexpr double kParallelHeadingDiff = 0.17453292519943295;  // 10 degree.
  if (std::abs(theta_diff) > kParallelHeadingDiff) {
    return false;
  }

  // Condition 3): Object is largely out of the lane boundaries that enclose
  // AV.
  const double obj_s_mean = 0.5 * (obj_frenet_box.s_min + obj_frenet_box.s_max);
  const auto obj_s_min_boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(obj_frenet_box.s_min);
  const auto obj_s_mean_boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(obj_s_mean);
  const auto obj_s_max_boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(obj_frenet_box.s_max);
  constexpr double kOutOfLaneOffset = 0.8;  // m.
  const double obj_l_min = obj_frenet_box.l_min + kOutOfLaneOffset;
  const double obj_l_max = obj_frenet_box.l_max - kOutOfLaneOffset;
  const bool left_of_left_boundary =
      obj_s_min_boundary.left.has_value() &&
      obj_s_mean_boundary.left.has_value() &&
      obj_s_max_boundary.left.has_value() &&
      obj_l_min > obj_s_min_boundary.left->lat_offset &&
      obj_l_min > obj_s_mean_boundary.left->lat_offset &&
      obj_l_min > obj_s_max_boundary.left->lat_offset;
  const bool right_of_right_boundary =
      obj_s_min_boundary.right.has_value() &&
      obj_s_mean_boundary.right.has_value() &&
      obj_s_max_boundary.right.has_value() &&
      obj_l_max < obj_s_min_boundary.right->lat_offset &&
      obj_l_max < obj_s_mean_boundary.right->lat_offset &&
      obj_l_max < obj_s_max_boundary.right->lat_offset;
  const bool out_of_lane = left_of_left_boundary || right_of_right_boundary;
  if (!out_of_lane) return false;

  // Condition 4): Object is not near an upcoming merge lane.
  double fraction = 0.0;
  double min_dist = 0.0;
  bool obj_near_upcoming_merge_lane = false;
  constexpr double kNearLaneDist = 2.0;  // m.
  for (const auto& merge_lane_id : upcoming_merge_lane_ids) {
    if (psmm.GetLaneProjection(traj->pose().pos(), merge_lane_id, &fraction,
                               /*point=*/nullptr, &min_dist) &&
        min_dist < kNearLaneDist) {
      obj_near_upcoming_merge_lane = true;
      break;
    }
  }
  if (obj_near_upcoming_merge_lane) return false;

  // Condition 4.1): Object is not near an merge lane
  for (const auto& driving_process : driving_process_seq) {
    if ((driving_process.merge_topology ==
         ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_LEFT) ||
        (driving_process.merge_topology ==
         ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_RIGHT)) {
      // const auto& lane = psmm.FindLaneByIdOrNull(driving_process.lane_id);
      // if (!lane) continue;
      // if (psmm.GetLaneProjection(traj->pose().pos(), driving_process.lane_id,
      //                            &fraction,
      //                            /*point=*/nullptr, &min_dist) &&
      //     min_dist < kNearLaneDist) {
      //   return false;
      // }
      return false;
    }
  }

  return true;
}

bool IgnoreReverseDrivingStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, const StBoundary& st_boundary) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::OBJECT_CUTIN &&
      st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::UNKNOWN_SOURCE) {
    return false;
  }
  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE ||
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  if (traj->planner_object().pose().v() >= -kEpsilon) {
    return false;
  }
  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      traj->states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kParallelDrivingThreshold = M_PI / 6.0;  // 30 deg.
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) >
      kParallelDrivingThreshold) {
    return false;
  }
  return true;
}

bool IgnoreUncomfortableBrakeOncomingStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, const StBoundary& st_boundary,
    const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  // The method to identify uncomfortable brake oncoming st-boundary here is
  // much similar with that in ModifyOncomingStBoundary of
  // pre_st_boundary_modifier but the parameters are a little different.
  // TODO： Refactor this part to reduce duplicate code with
  // ModifyOncomingStBoundary,
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }
  constexpr double kMinTimeLimit = 0.5;  // s.
  if (st_boundary.bottom_left_point().t() < kMinTimeLimit) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      traj->states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kOnComingThreshold = 2.9670597283903604;  // 170 deg.
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) <
      kOnComingThreshold) {
    return false;
  }

  // Only ignore the oncoming prediction if it would cause uncomfortable brake.
  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (st_boundary.bottom_right_point().s() > const_speed_s) {
    // No brake is needed.
    return false;
  }
  const double estimated_av_decel =
      2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
      Sqr(st_boundary.bottom_right_point().t());
  constexpr double kUncomfortableDecel = 0.3;  // m/s^2.
  if (estimated_av_decel < kUncomfortableDecel) {
    return false;
  }
  if (IsConsiderOncomingObs(st_traj_mgr, st_boundary, drive_passage,
                            vehicle_geometry_params, current_v, path)) {
    return false;
  }
  return true;
}

bool IgnoreOncomingStBoundaryWithoutObviousCutInIntention(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const DrivePassage& drive_passage, double current_v,
    const StBoundary& st_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const DiscretizedPath& path) {
  // Ignore the oncoming objects without obvious intention to cut in or crossing
  // AV's trajectory.
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& fo_info = overlap_infos.front();

  const auto fo_lane_semantic =
      path_semantics[fo_info.av_start_idx].lane_semantic;
  if (fo_lane_semantic != LaneSemantic::ROAD &&
      fo_lane_semantic != LaneSemantic::INTERSECTION_STRAIGHT) {
    return false;
  }

  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }
  constexpr double kMinTimeLimit = 0.1;  // s.
  if (st_boundary.bottom_left_point().t() < kMinTimeLimit) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

  // Here we use object info from planner object to recognize current object's
  // intention.
  const Vec2d& obj_pos = traj->planner_object().pose().pos();
  const auto obj_frenet_coord = drive_passage.QueryFrenetCoordinateAt(obj_pos);
  if (!obj_frenet_coord.ok()) {
    return false;
  }
  const auto obj_frenet_theta =
      drive_passage.QueryTangentAngleAtS(obj_frenet_coord->s);
  if (!obj_frenet_theta.ok()) {
    return false;
  }
  const double ref_theta = *obj_frenet_theta;
  const double theta_diff =
      NormalizeAngle(traj->planner_object().pose().theta() - ref_theta);

  // Larger threshold for cyclist.
  constexpr double kOncomingThresholdVehicle = 3.0543261909900768;  // 175 deg.
  constexpr double kOncomingThresholdCyclist = 2.9670597283903604;  // 170 deg.
  const double oncoming_threshold =
      st_boundary.object_type() == StBoundaryProto::VEHICLE
          ? kOncomingThresholdVehicle
          : kOncomingThresholdCyclist;
  if (std::abs(theta_diff) < oncoming_threshold) {
    return false;
  }
  if (IsConsiderOncomingObs(st_traj_mgr, st_boundary, drive_passage,
                            vehicle_geometry_params, current_v, path)) {
    return false;
  }
  // Only ignore the oncoming prediction if it would cause uncomfortable brake.
  constexpr double kFollowDistance = 3.0;  // m.
  const double brake_dis = st_boundary.min_s() - kFollowDistance;
  const double brake_time = st_boundary.bottom_right_point().t();
  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (brake_dis < kEpsilon) {
    // Not enough distance to brake.
    return true;
  }
  if (brake_dis > const_speed_s) {
    // No brake is needed.
    return false;
  }
  const bool brake_to_stop =
      0.5 * brake_dis / std::max(current_v, kEpsilon) < brake_time;
  const double estimated_av_decel =
      brake_to_stop
          ? 0.5 * Sqr(current_v) / brake_dis
          : 2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
                Sqr(st_boundary.bottom_right_point().t());
  constexpr double kUncomfortableDecel = 1.0;  // m/s^2.
  if (estimated_av_decel < kUncomfortableDecel) {
    return false;
  }
  return true;
}

std::optional<VtSpeedLimit> MakeOncomingPreBrakeDecisionForStBoundary(
    double current_v, double max_v, double time_step, int step_num,
    const StBoundaryWithDecision& st_boundary_wd) {
  constexpr double kMinTimeLimit = 0.1;  // s.
  constexpr double kMaxTimeLimit = 3.5;  // s.
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  const double pre_brake_time = st_boundary.bottom_left_point().t();
  if (pre_brake_time < kMinTimeLimit || pre_brake_time > kMaxTimeLimit) {
    return std::nullopt;
  }

  constexpr double kEmergencyStopAccel = -3.0;  // m/s^2.
  const double s_limit_lower = current_v * pre_brake_time +
                               0.5 * kEmergencyStopAccel * Sqr(pre_brake_time);
  if (st_boundary.bottom_left_point().s() < s_limit_lower) {
    return std::nullopt;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  constexpr double kMildDecel = -0.6;  // m/s^2.
  constexpr double kMinVel = 4.0;      // m/s.
  constexpr double kExtraTime = 1.0;   // s.
  const auto info = absl::StrCat("Pre brake for oncoming object ", *traj_id);
  return GenerateConstAccSpeedLimit(
      /*start_t=*/0.0, pre_brake_time + kExtraTime, current_v, kMinVel, max_v,
      kMildDecel, time_step, step_num, info);
}

#define DEBUG_PARALLEL_MERGE (0)

bool IgnoreBackParallelMergeAgent(
    const StBoundary& st_boundary,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    LaneChangeStage lc_stage, Merge_Direction merge_direction,
    double av_obj_ttc,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    SplitTopology split_topo) {
  constexpr double kConsiderMaxPathLength = 30.0;
  constexpr double kConsiderAvLengthRatio = 1.0;
  constexpr double kMaxCurvature = 0.002;
  constexpr double kTtcThreshold = 0.0;
  constexpr double kTtcThresholdMergeScene = 2.5;

  if (LaneChangeStage::LCS_NONE != lc_stage || merge_direction == MERGE_NONE)
    return false;

  const auto& obj_sl_info = st_boundary.obj_sl_info();
  if (!obj_sl_info.has_value()) {
    return false;
  }

  if (split_topo == SplitTopology::TOPOLOGY_SPLIT_LEFT ||
      split_topo == SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
    return false;
  }

  if (MERGE_LEFT == merge_direction && obj_sl_info->dl > 0) {
    return false;
  } else if (MERGE_RIGHT == merge_direction && obj_sl_info->dl < 0) {
    return false;
  }

  double ttc_threshold;

  if (MERGE_LEFT == merge_direction || MERGE_RIGHT == merge_direction) {
    ttc_threshold = kTtcThresholdMergeScene;
  } else {
    ttc_threshold = kTtcThreshold;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  double av_frenet_box_max_s = vehicle_geometry_params.front_edge_to_center();

  const auto& overlap_infos = st_boundary.overlap_infos();
  if (overlap_infos.empty()) {
    return false;
  }

  const auto& last_overlap_info = overlap_infos.back();
  const auto& first_overlap_info = overlap_infos.front();

  const double last_overlap_obj_heading =
      traj->states()[last_overlap_info.obj_idx].box.heading();
  const double last_overlap_ego_heading =
      path[(last_overlap_info.av_start_idx + last_overlap_info.av_end_idx) / 2]
          .theta();
  const double first_overlap_obj_heading =
      traj->states()[first_overlap_info.obj_idx].box.heading();
  const double first_overlap_ego_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();

  const double object_heading = traj->planner_object().pose().theta();
  const double ego_heading = path[0].theta();

  const double cut_in_angle = std::abs(
      NormalizeAngle(first_overlap_obj_heading - first_overlap_ego_heading));
  const double cut_out_angle = std::abs(
      NormalizeAngle(last_overlap_obj_heading - last_overlap_ego_heading));

  const double heading_diff = NormalizeAngle(object_heading - ego_heading);

  if (cut_in_angle > kLaneParallCutinAngleThres ||
      cut_out_angle > kLaneParallCutinAngleThres || heading_diff > M_PI / 8) {
    return false;
  }

  double ego_length = vehicle_geometry_params.length();

  if ((av_frenet_box_max_s - obj_sl_info->frenet_polygon.s_max) <
      kConsiderAvLengthRatio * ego_length) {
    return false;
  }

  if (av_obj_ttc < ttc_threshold) return false;

  double average_curvature = 0.0;
  int32_t point_nums = 0;

  for (const auto& point : path) {
    average_curvature += std::abs(point.kappa());
    point_nums++;
    if (point.s() > kConsiderMaxPathLength) {
      break;
    }
  }
  if (0 != point_nums) {
    average_curvature /= point_nums;
  }

  if (kMaxCurvature < average_curvature) return false;

  return true;
}

std::optional<double> GetPassageOccupyPercentage(
    const DrivePassage& drive_passage, const FrenetBox& av_frenet_box,
    double* occupy_width) {
  const auto av_s_min_boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(av_frenet_box.s_min);
  const auto av_s_max_boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(av_frenet_box.s_max);
  if (!av_s_min_boundary.left.has_value() ||
      !av_s_max_boundary.left.has_value() ||
      !av_s_min_boundary.right.has_value() ||
      !av_s_max_boundary.right.has_value()) {
    return std::nullopt;
  }
  // check if av fully in drive passage lane
  bool av_in_drive_passage_lane =
      (av_frenet_box.l_max < av_s_min_boundary.left->lat_offset) &&
      (av_frenet_box.l_max < av_s_max_boundary.left->lat_offset) &&
      (av_frenet_box.l_min > av_s_min_boundary.right->lat_offset) &&
      (av_frenet_box.l_min > av_s_max_boundary.right->lat_offset);
  // check if av fully out of drive passage lane
  bool av_out_of_drive_passage_lane =
      (av_frenet_box.l_max < av_s_min_boundary.right->lat_offset) &&
      (av_frenet_box.l_max < av_s_max_boundary.right->lat_offset) &&
      (av_frenet_box.l_min > av_s_min_boundary.left->lat_offset) &&
      (av_frenet_box.l_min > av_s_max_boundary.left->lat_offset);
  if (av_in_drive_passage_lane) {
    return 1.0;
  } else if (av_out_of_drive_passage_lane) {
    return 0.0;
  } else {
    const double ego_width = av_frenet_box.l_max - av_frenet_box.l_min;
    const double left_bound_mean_l = (av_s_max_boundary.left->lat_offset +
                                      av_s_min_boundary.left->lat_offset) /
                                     2.0;
    const double right_bound_mean_l = (av_s_max_boundary.right->lat_offset +
                                       av_s_min_boundary.right->lat_offset) /
                                      2.0;
    if (av_frenet_box.center_l() > kEpsilon) {
      *occupy_width = left_bound_mean_l - av_frenet_box.l_min;
    } else {
      *occupy_width = av_frenet_box.l_max - right_bound_mean_l;
    }
    return *occupy_width / ego_width;
  }
  return std::nullopt;
}

bool IgnoreBackWhenAVOccupyTargetLane(
    const StBoundaryWithDecision& st_boundary_wd,
    const DrivePassage& drive_passage, const FrenetBox& av_frenet_box,
    const st::VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto& st_boundary = st_boundary_wd.st_boundary();
  constexpr double kOccupyThreshold = 0.5;
  constexpr double kDefaultLatSpeed = 0.2;
  if (!st_boundary->overlap_meta().has_value()) return false;
  const auto& obj_sl_info = st_boundary->obj_sl_info();
  if (!obj_sl_info.has_value()) {
    return false;
  }
  if (st_boundary->overlap_meta()->source() == StOverlapMetaProto::AV_CUTIN &&
      obj_sl_info->ds < -kEpsilon) {
    bool is_av_occupy_target_lane = false;
    double occupy_width = 0.0;
    const auto occupy_percentage =
        GetPassageOccupyPercentage(drive_passage, av_frenet_box, &occupy_width);
    if (occupy_percentage.has_value() && occupy_percentage.value() > kEpsilon) {
      double ttc = st_boundary->min_t();
      is_av_occupy_target_lane = ((occupy_width > kOccupyThreshold) ||
                                  (occupy_width + ttc * kDefaultLatSpeed >
                                   vehicle_geometry_params.width() * 0.5));
    }
    return is_av_occupy_target_lane;
  }
  return false;
}

// need to be modifed
double CalcTtcConsiderRelativeAcc(double ds, double av_vel, double av_acc,
                                  const SpacetimeTrajectoryManager& st_traj_mgr,
                                  const StBoundary& st_boundary,
                                  bool consider_acc) {
  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id().value();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  double obj_vel = traj->planner_object().pose().v();
  double obj_acc = traj->planner_object().pose().a();
  double relative_acc = obj_acc - av_acc;
  double relative_vel = std::fabs(obj_vel - av_vel);

  double ttc = 0.0;
  if (ds >= 0) {
    relative_acc = -relative_acc;
  }
  ds = std::fabs(ds);

  double stop_acc = Sqr(relative_vel) / (2 * ds);

  if (!consider_acc) {
    relative_acc = 0.0;
  }

  if (relative_acc <= 0 && std::fabs(relative_acc) > stop_acc) {
    ttc = ds / (relative_vel + kEpsilon);
  } else {
    ttc =
        ds / (std::sqrt(Sqr(relative_vel) + 2 * relative_acc * ds) + kEpsilon);
  }

  return ttc;
}

bool IgnoreExceedActualMapRangeStBoundary(const StBoundary& st_boundary,
                                          const DrivePassage& drive_passage,
                                          const DiscretizedPath& path,
                                          double current_v) {
  if (st_boundary.overlap_infos().empty()) return false;

  constexpr double kDistanceDiffHysteresis = 15.0;   // m
  constexpr double kIgnoreFarObjTimeFactor = 3.0;    // s
  constexpr double kCutinObjConfirmTime = 1.0;       // s
  constexpr double kIgnoreSpeedlowerThreshold = 10;  // mps

  double min_dist_for_brake =
      Max(current_v, kIgnoreSpeedlowerThreshold) * kIgnoreFarObjTimeFactor;
  if (st_boundary.bottom_left_point().s() < min_dist_for_brake) return false;
  // step1. get cutin obj which could not be an oncoming obj from
  // overlap_infos
  const auto& obj_scenario_info = st_boundary.obj_scenario_info();

  if ((obj_scenario_info.relationship == Relationship::OnComing) ||
      (obj_scenario_info.relationship == Relationship::Unknown) ||
      (st_boundary.min_t() < kCutinObjConfirmTime)) {
    return false;
  }

  // step2. get length of actual drive_passage ,protect path_pt of first
  // overlap position noto drive_passage
  const st::planning::StationIndex last_real_station_idx =
      drive_passage.last_real_station_index();
  const st::planning::Station& last_real_station =
      drive_passage.station(last_real_station_idx);
  const auto& stations = drive_passage.stations();

  // find path point
  int first_overlap_index = st_boundary.overlap_infos().front().av_start_idx;
  const PathPoint& first_overlap_pt = path[first_overlap_index];

  Vec2d point;
  point.set_x(first_overlap_pt.x());
  point.set_y(first_overlap_pt.y());

  // project onto drive passage
  const auto& nearest_station = drive_passage.FindNearestStation(point);

  double first_overlap_station_s = nearest_station.accumulated_s();
  double last_real_station_s =
      last_real_station.accumulated_s() - kDistanceDiffHysteresis;

  return first_overlap_station_s > last_real_station_s;
}

bool IgnoreOncomingStBoundaryWithCutOutIntention(
    const StBoundary& st_boundary, const DrivePassage& drive_passage,
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& st_traj_mgr,
    double current_v) {
  const auto& overlap_meta = st_boundary.overlap_meta();

  if (!overlap_meta.has_value() || !overlap_meta->is_oncoming() ||
      overlap_meta->pattern() != StOverlapMetaProto::LEAVE) {
    return false;
  }

  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

  const auto& object_contour = traj->contour();

  const auto& obj_frenet_box_or = drive_passage.QueryFrenetBoxAtContour(
      object_contour, /*zone_checking*/ false);

  double obj_dist = 0.0;

  const auto& obj_sl_info = st_boundary.obj_sl_info();

  if (obj_sl_info.has_value()) {
    obj_dist = obj_sl_info->ds;
  } else {
    obj_dist = obj_frenet_box_or->center_s();
  }

  if (!obj_frenet_box_or.ok() || obj_dist <= kFarawayOncomingObjDistThres) {
    return false;
  }

  // make sure distance between firsr overlap area and ego is safe

  const double first_overlap_t =
      std::max(st_boundary.bottom_left_point().t(), kEpsilon);
  const double first_overlap_s = st_boundary.bottom_left_point().s();

  if (((first_overlap_s - current_v * first_overlap_t) * 2) /
          Sqr(first_overlap_t) <
      kMaxConfortOncomingBrakeDecel) {
    return false;
  }

  // If the deceleration required for the self  brake to stop at the
  // last_overlap position is less than the threshold, and ignore it
  const double last_overlap_t =
      std::max(st_boundary.bottom_right_point().t(), kEpsilon);
  const double last_overlap_s = st_boundary.bottom_right_point().s();

  if (((last_overlap_s - current_v * last_overlap_t) * 2) /
          Sqr(last_overlap_t) <=
      kMaxOncomingBrakeDecel) {
    return false;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  if (overlap_infos.empty()) {
    return false;
  };
  const auto& first_overlap_info = overlap_infos.front();
  const auto& last_overlap_info = overlap_infos.back();

  const auto& last_overlap_contour =
      traj->states()[last_overlap_info.obj_idx].contour;

  const auto& last_overlap_frenet_or = drive_passage.QueryFrenetBoxAtContour(
      last_overlap_contour, /*zone_checking*/ false);

  if (!last_overlap_frenet_or.ok()) {
    return false;
  }
  const double dist_between_obj_and_last_overlap = std::abs(
      obj_frenet_box_or->center_s() - last_overlap_frenet_or->center_s());

  // make sure object cut out quickly
  if ((st_boundary.object_type() == StBoundaryProto::CYCLIST &&
       dist_between_obj_and_last_overlap <= kMaxCyclistCutoutDistacne) ||
      dist_between_obj_and_last_overlap <= kMaxVehicleCutoutDistacne) {
    return true;
  }

  const auto& first_state = traj->states()[first_overlap_info.obj_idx];

  const auto& last_overlap_point =
      traj->states()[last_overlap_info.obj_idx].traj_point;
  const auto& first_overlap_point = first_state.traj_point;
  const double overlap_angle_diff = std::abs(NormalizeAngle(
      last_overlap_point->theta() - first_overlap_point->theta()));

  const auto& first_overlap_contour = first_state.contour;
  const auto& first_overlap_frenet_or = drive_passage.QueryFrenetBoxAtContour(
      first_overlap_contour, /*zone_checking*/ false);

  if (!first_overlap_frenet_or.ok()) {
    return false;
  }

  const double first_overlap_l = first_overlap_frenet_or->center_l();
  const double last_overlap_l = last_overlap_frenet_or->center_l();
  const double lateral_diff = std::abs(last_overlap_l - first_overlap_l);

  const auto [right_boundary, left_boundary] =
      drive_passage.QueryEnclosingLaneBoundariesAtS(
          last_overlap_frenet_or->center_s());
  const double boundary_right_l =
      right_boundary.has_value()
          ? std::max(right_boundary->lat_offset, -kMaxHalfLaneWidth)
          : -kMaxHalfLaneWidth;
  const double boundary_left_l =
      left_boundary.has_value()
          ? std::min(left_boundary->lat_offset, kMaxHalfLaneWidth)
          : kMaxHalfLaneWidth;

  const double lane_width = boundary_left_l - boundary_right_l;

  return (overlap_angle_diff >= kCutoutThetaThres) &&
         (lateral_diff >= lane_width * 0.5 - kBufferlat);
}

const StBoundary* GetCipvStBoundary(
    const std::vector<StBoundaryWithDecision>& st_boundaries_wd,
    const CipvObjectInfo& cipv_object_info) {
  const auto& cipv_obj_id = cipv_object_info.nearest_object_id;
  const auto& cipv_stay_obj_id = cipv_object_info.nearest_stay_object_id;
  if (!cipv_stay_obj_id.has_value() && !cipv_obj_id.has_value()) {
    return nullptr;
  }

  for (const auto& st_boundary_wd : st_boundaries_wd) {
    const StBoundary* st_boundary = st_boundary_wd.raw_st_boundary();
    if (st_boundary == nullptr ||
        st_boundary->object_type() != StBoundaryProto::VEHICLE) {
      return nullptr;
    }
    const auto& object_id = st_boundary->object_id();
    if (!object_id.has_value() || st_boundary->is_protective() ||
        st_boundary->min_t() > 0.0) {
      continue;
    }

    if (object_id == cipv_stay_obj_id) {
      return st_boundary;
    }
  }

  return nullptr;
}

bool IgnoreCrossBehindCipvStBoundary(
    const StBoundary& st_boundary, const StBoundary* cipv_st_boundary,
    const DrivePassage& drive_passage, const DiscretizedPath& path,
    const SpacetimeTrajectoryManager& st_traj_mgr, double current_v,
    double follow_time_headway) {
  const auto& overlap_meta = st_boundary.overlap_meta();
  if (!overlap_meta.has_value() ||
      overlap_meta->pattern() == StOverlapMetaProto::LEAVE ||
      overlap_meta->pattern() == StOverlapMetaProto::STAY ||
      st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT ||
      st_boundary.object_type() != StBoundaryProto::VEHICLE) {
    return false;
  }
  const double cipv_first_overlap_s = cipv_st_boundary->bottom_left_point().s();
  if (cipv_first_overlap_s >
      current_v * (follow_time_headway + kFollowTimeHeadwayBias) +
          kFollowHeadWayDistanceBias) {
    return false;
  }

  CHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

  const double obj_heading = traj->states()[0].box.heading();

  const auto& overlap_infos = st_boundary.overlap_infos();
  if (overlap_infos.empty()) {
    return false;
  }

  const auto& first_overlap_info = overlap_infos.front();

  const double first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();

  const double overlap_angle_diff =
      std::abs(NormalizeAngle(first_overlap_av_middle_heading - obj_heading));

  if (overlap_angle_diff < kObjOncomingAngleThres) {
    return false;
  }

  const double first_overlap_t = st_boundary.bottom_left_point().t();
  const double first_overlap_s = st_boundary.bottom_left_point().s();

  bool is_predict_cutin_behind_cipv = false;
  const auto cipv_s_range =
      cipv_st_boundary->GetBoundarySRange(first_overlap_t);
  if (cipv_s_range.has_value() &&
      first_overlap_s > cipv_s_range.value().second) {
    is_predict_cutin_behind_cipv = true;
  }

  return (first_overlap_t >= kCutinCipvMaxTimeThres) ||
         (Sqr(current_v) / (2 * kMaxFollowCipvBreakDecel) < first_overlap_s) ||
         is_predict_cutin_behind_cipv;
}

bool IgnoreFarAwayObjByShrinkLateralBuffer(
    const std::string& obj_id, const CipvObjectInfo& cipv_object_info,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SegmentMatcherKdtree& path_kd_tree, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geo_params) {
  const auto obj_distance_s =
      FindOrNull(cipv_object_info.object_distance_map, obj_id);

  if (obj_distance_s == nullptr) {
    return false;
  }

  if (*obj_distance_s < kLateralIgnoreDistThres) {
    return false;
  }

  const auto* planner_object =
      CHECK_NOTNULL(st_traj_mgr.FindObjectByObjectId(obj_id));
  const auto& obj_contour = planner_object->contour();
  const auto& obj_box = planner_object->bounding_box();

  const double obj_radius = obj_box.diagonal() * 0.5 + kRequiredLateralGap;

  const double ego_radius =
      Hypot(std::max(vehicle_geo_params.front_edge_to_center(),
                     vehicle_geo_params.back_edge_to_center()),
            vehicle_geo_params.right_edge_to_center());
  const double search_radius = obj_radius + ego_radius + kSearchBuffer;

  const auto indices = path_kd_tree.GetSegmentIndexInRadius(
      obj_contour.CircleCenter().x(), obj_contour.CircleCenter().y(),
      search_radius);

  const double lateral_buff =
      kRequiredLateralGap - vehicle_geo_params.width() *
                                kLaterFactorForFarawayObject(*obj_distance_s);

  for (const auto index : indices) {
    const auto& av_shape = av_shapes[index];
    if (av_shape->MainBodyHasOverlapWithBuffer(obj_contour, lateral_buff,
                                               /*lon_buffer */ 0.0)) {
      return false;
    }
  }

  return true;
}

void MakeIgnoreAndPreBrakeDecisionForStBoundary(
    const SpeedFinderParamsProto::IgnoreDeciderParamsProto& params,
    const DiscretizedPath& path,
    const std::vector<PathPointSemantic>& path_semantics,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double current_v,
    double max_v, double time_step, double trajectory_steps,
    const PlannerSemanticMapManager& psmm,
    // const std::vector<StPoint>& emergency_stop_points,
    bool on_left_turn_waiting_lane,
    const std::optional<FrenetBox>& av_frenet_box,
    bool av_in_drive_passage_lane,
    const std::vector<mapping::ElementId>& upcoming_merge_lane_ids,
    StBoundaryWithDecision* st_boundary_wd,
    std::optional<VtSpeedLimit>* speed_limit, LaneChangeStage lc_stage,
    Merge_Direction merge_direction, double current_a,
    const std::vector<DrivingProcess>& driving_process_seq,
    SplitTopology split_topo, const bool is_narrow_near_large_vehicle,
    const StBoundary* cipv_st_boundary, double follow_time_headway,
    const SegmentMatcherKdtree& path_kd_tree,
    const CipvObjectInfo& cipv_object_info, bool lc_left) {
  CHECK_NOTNULL(st_boundary_wd);

  if (st_boundary_wd->decision_type() != StBoundaryProto::UNKNOWN) {
    return;
  }
  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();
  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return;
  }

  double av_obj_ttc = std::numeric_limits<double>::max();
  const auto& obj_sl_info = st_boundary.obj_sl_info();
  if (obj_sl_info.has_value()) {
    av_obj_ttc = CalcTtcConsiderRelativeAcc(
        obj_sl_info->ds, current_v, current_a, st_traj_mgr, st_boundary, false);
  }

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         StBoundaryWithDecision* st_boundary_wd) {
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd->set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
        st_boundary_wd->set_ignore_reason(ignore_reason);
        st_boundary_wd->set_decision_info(decision_info);
      };

  // 1. Ignore st-boundaries hit AV current position.
  if (IgnoreHitAvCurrentPositionStBoundary(st_traj_mgr, vehicle_geometry_params,
                                           av_shapes, path.front(), st_boundary,
                                           path_semantics, current_v)) {
    make_ignore_decision("ignore hit current AV position",
                         StBoundaryProto::HIT_CURRENT_POS, st_boundary_wd);
    return;
  }

  if (st_boundary.is_stationary()) return;

  // 2. Ignore st-boundaries hit AV emergency stop trajectory.
  // if (IgnoreHitAvEmergencyStopTrajectoryStBoundary(emergency_stop_points,
  //                                                  st_boundary)) {
  //   make_ignore_decision("ignore hit AV emergency stop trajectory",
  //                        StBoundaryProto::HIT_EMERGENCY_TRAJ,
  //                        st_boundary_wd);
  //   return;
  // }

  // 3. Ignore back cut-in st-boundaries.
  if (!path_semantics.empty() &&
      IgnoreBackCutInStBoundary(st_boundary, path_semantics.front(), path,
                                st_traj_mgr, vehicle_geometry_params,
                                on_left_turn_waiting_lane, lc_left, lc_stage)) {
    make_ignore_decision("ignore back cutin", StBoundaryProto::BACK_CUT_IN,
                         st_boundary_wd);
    return;
  }

  if (!path_semantics.empty() &&
      IgnoreTurnLeftBackCutInStBoundary(st_traj_mgr, path.front(), st_boundary,
                                        driving_process_seq,
                                        vehicle_geometry_params)) {
    make_ignore_decision("ignore turn_left_back cutin",
                         StBoundaryProto::BACK_CUT_IN, st_boundary_wd);
    return;
  }

  // 4. Ignore on-road parallel cut-in st-boundaries.
  if (!is_narrow_near_large_vehicle && !path_semantics.empty() &&
      av_frenet_box.has_value() &&
      IgnoreOnRoadParallelCutInStBoundary(
          st_traj_mgr, path_semantics, *av_frenet_box, av_in_drive_passage_lane,
          drive_passage, current_v, psmm, driving_process_seq,
          upcoming_merge_lane_ids, params.ignore_late_parallel_cut_in_vehicle(),
          st_boundary)) {
    make_ignore_decision("ignore on-road parallel cutin",
                         StBoundaryProto::PARALLEL_CUT_IN, st_boundary_wd);
    return;
  }

  // HACK(bo): Ignore all OBJECT_CUTIN & UNKNOWN_SOURCE reverse-driving
  // vehicles/cyclists for vision-only demo.
  if (params.ignore_reverse_driving() &&
      IgnoreReverseDrivingStBoundary(st_traj_mgr, path, current_v,
                                     st_boundary)) {
    make_ignore_decision("ignore reverse-driving object",
                         StBoundaryProto::REVERSE_DRIVING, st_boundary_wd);
  }

  // 5. Ignore uncomfortable brake incoming cut-in st-boundaries.
  if (IgnoreUncomfortableBrakeOncomingStBoundary(st_traj_mgr, path, current_v,
                                                 st_boundary, drive_passage,
                                                 vehicle_geometry_params)) {
    make_ignore_decision("ignore uncomfortable brake oncoming",
                         StBoundaryProto::ONCOMING_OBJECT, st_boundary_wd);
    auto speed_limit_opt = MakeOncomingPreBrakeDecisionForStBoundary(
        current_v, max_v, time_step, trajectory_steps, *st_boundary_wd);
    if (speed_limit_opt.has_value()) {
      if (speed_limit->has_value()) {
        MergeVtSpeedLimit(speed_limit_opt.value(), &speed_limit->value());
      } else {
        *speed_limit = speed_limit_opt;
      }
    }
    return;
  }

  // 6. Ignore oncoming cut-in/crossing st-boundaries without obvious cut-in
  // intention.
  if (!path_semantics.empty() &&
      IgnoreOncomingStBoundaryWithoutObviousCutInIntention(
          st_traj_mgr, path_semantics, drive_passage, current_v, st_boundary,
          vehicle_geometry_params, path)) {
    make_ignore_decision(
        "ignore oncoming objects without obvious cut-in intention",
        StBoundaryProto::ONCOMING_OBJECT, st_boundary_wd);
    const auto speed_limit_opt = MakeOncomingPreBrakeDecisionForStBoundary(
        current_v, max_v, time_step, trajectory_steps, *st_boundary_wd);
    if (speed_limit_opt.has_value()) {
      if (speed_limit->has_value()) {
        MergeVtSpeedLimit(speed_limit_opt.value(), &speed_limit->value());
      } else {
        *speed_limit = speed_limit_opt;
      }
    }
    return;
  }

  // Ignore faraway oncoming obstacles which cut-out quickly
  if (IgnoreOncomingStBoundaryWithCutOutIntention(
          st_boundary, drive_passage, path, st_traj_mgr, current_v)) {
    make_ignore_decision("ignore faraway oncoming objects cut out quickly ",
                         StBoundaryProto::ONCOMING_OBJECT, st_boundary_wd);
  }

  // 7. Ignore back parallel merge obj
  if (params.ignore_back_parallel_merge_obj() &&
      IgnoreBackParallelMergeAgent(st_boundary, st_traj_mgr, path, lc_stage,
                                   merge_direction, av_obj_ttc,
                                   vehicle_geometry_params, split_topo)) {
    make_ignore_decision("ignore back parallel far merge obj",
                         StBoundaryProto::BACK_PARALLEL_MERGE, st_boundary_wd);
    return;
  }

  // 8. Ignore back vehicles when AV_CUTIN cross lane boundary
  if (IgnoreBackWhenAVOccupyTargetLane(*st_boundary_wd, drive_passage,
                                       av_frenet_box.value(),
                                       vehicle_geometry_params)) {
    make_ignore_decision("av cutin occupy enough drive passage lane",
                         StBoundaryProto::BACK_CUT_IN, st_boundary_wd);
  }

  // 9. Ignore CUTIN obstacles which locate out of actual drive_passage,and
  // create config
  if (params.ignore_beyond_actual_drive_passagr_obj()) {
    if (IgnoreExceedActualMapRangeStBoundary(st_boundary, drive_passage, path,
                                             current_v)) {
      make_ignore_decision(
          "obstacle locate out of reliable drive_passage",
          StBoundaryProto::EXCEED_ACTUAL_MAP_RANGE /*need update*/,
          st_boundary_wd);
    }
  }

  // 10. ignore cross ocstacles which loacted behind cipv within ego lane
  if (cipv_st_boundary != nullptr &&
      IgnoreCrossBehindCipvStBoundary(st_boundary, cipv_st_boundary,
                                      drive_passage, path, st_traj_mgr,
                                      current_v, follow_time_headway)) {
    make_ignore_decision(
        "first_overlap area loacted behind cipv within ego lane",
        StBoundaryProto::CROSS_EGO_AHEAD_CIPV, st_boundary_wd);
  }
}

void MakeIgnoreDecisionForNonNearestStationaryStBoundaries(
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  CHECK_NOTNULL(st_boundaries_wd);

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         StBoundaryWithDecision* st_boundary_wd) {
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd->set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
        st_boundary_wd->set_ignore_reason(ignore_reason);
        st_boundary_wd->set_decision_info(decision_info);
      };

  StBoundaryWithDecision* nearest_stationary_st_boundary = nullptr;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary()) continue;
    if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) continue;
    if (nullptr == nearest_stationary_st_boundary) {
      nearest_stationary_st_boundary = &st_boundary_wd;
    } else if (st_boundary.min_s() <
               nearest_stationary_st_boundary->raw_st_boundary()->min_s()) {
      make_ignore_decision("ignore non-nearest stationary st-boundary",
                           StBoundaryProto::NON_NEAREST_STATIONARY,
                           nearest_stationary_st_boundary);
      nearest_stationary_st_boundary = &st_boundary_wd;
    } else {
      make_ignore_decision("ignore non-nearest stationary st-boundary",
                           StBoundaryProto::NON_NEAREST_STATIONARY,
                           &st_boundary_wd);
    }
  }
  return;
}

void MakeIgnoreDecisionForFarAwaySignificantLateralStBoundaries(
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const StBoundary* cipv_st_boundary,
    const SegmentMatcherKdtree& path_kd_tree,
    const CipvObjectInfo& cipv_object_info) {
  CHECK_NOTNULL(st_boundaries_wd);

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         StBoundaryWithDecision* st_boundary_wd) {
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd->set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
        st_boundary_wd->set_ignore_reason(ignore_reason);
        st_boundary_wd->set_decision_info(decision_info);
      };

  std::set<std::string> cipv_obj_list;

  // ignore faraway cipv obj or static obj with significant lateral diff
  if (cipv_object_info.nearest_object_id.has_value() &&
      IgnoreFarAwayObjByShrinkLateralBuffer(
          *cipv_object_info.nearest_object_id, cipv_object_info, av_shapes,
          st_traj_mgr, path_kd_tree, path, vehicle_geometry_params)) {
    cipv_obj_list.emplace(*cipv_object_info.nearest_object_id);
  }

  if (cipv_object_info.nearest_stay_object_id.has_value() &&
      cipv_object_info.nearest_object_id !=
          *cipv_object_info.nearest_stay_object_id &&
      IgnoreFarAwayObjByShrinkLateralBuffer(
          *cipv_object_info.nearest_stay_object_id, cipv_object_info, av_shapes,
          st_traj_mgr, path_kd_tree, path, vehicle_geometry_params)) {
    cipv_obj_list.emplace(*cipv_object_info.nearest_stay_object_id);
  }

  if (cipv_object_info.second_nearest_stay_object_id.has_value() &&
      IgnoreFarAwayObjByShrinkLateralBuffer(
          *cipv_object_info.second_nearest_stay_object_id, cipv_object_info,
          av_shapes, st_traj_mgr, path_kd_tree, path,
          vehicle_geometry_params)) {
    cipv_obj_list.emplace(*cipv_object_info.second_nearest_stay_object_id);
  }

  if (cipv_obj_list.empty()) {
    return;
  }

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT ||
        !st_boundary.object_id().has_value()) {
      continue;
    }

    if (cipv_obj_list.count(*st_boundary.object_id())) {
      make_ignore_decision(
          "faraway cipv obj with significant lateral diff",
          StBoundaryProto::SIGNIFICANT_LATEAL_DIFF_FARAWAY_CIPV,
          &st_boundary_wd);
    }
  }
}

std::pair<HalfPlane, HalfPlane> ComputeAvFovByCipv(
    const PathPoint& av_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const Polygon2d& cipv_polygon, double nearest_object_s) {
  const Vec2d av_vec = Vec2d::FastUnitFromAngle(av_point.theta());
  const Vec2d av_front_center =
      Vec2d(av_point.x(), av_point.y()) +
      av_vec * vehicle_geometry_params.front_edge_to_center();
  const auto& cipv_points = cipv_polygon.points();
  CHECK_GT(cipv_points.size(), 0);
  HalfPlane left = HalfPlane(av_front_center, cipv_points[0]);
  HalfPlane right = HalfPlane(av_front_center, cipv_points[0]);
  for (int i = 1; i < cipv_points.size(); ++i) {
    if (left.IsPointInside(cipv_points[i])) {
      left = HalfPlane(av_front_center, cipv_points[i]);
    }
    if (!right.IsPointInside(cipv_points[i])) {
      right = HalfPlane(av_front_center, cipv_points[i]);
    }
  }
  const PiecewiseLinearFunction<double, double> kObjectDistFovBufferPlf = {
      {10.0, 100.0}, {0.5, 6.0}};
  double angle_buffer = 0.05;          // rad.
  constexpr double kMinSThres = 10.0;  // m.
  if (nearest_object_s > kMinSThres) {
    angle_buffer = kObjectDistFovBufferPlf(nearest_object_s) / nearest_object_s;
  }

  const Vec2d left_vec = Vec2d::FastUnitFromAngle(
      NormalizeAngle(left.tangent().FastAngle() + angle_buffer));
  const Vec2d right_vec = Vec2d::FastUnitFromAngle(
      NormalizeAngle(right.tangent().FastAngle() - angle_buffer));
  return std::make_pair(
      HalfPlane(av_front_center, left_vec + av_front_center),
      HalfPlane(av_front_center, right_vec + av_front_center));
}

bool IsLongTimeStayCipv(const DrivePassage& drive_passage,
                        const DiscretizedPath& path,
                        const StBoundary& st_boundary) {
  constexpr double kCutoutDistanceHysteresis = 10.0;
  constexpr double kCutoutStayTimeThres = 3.5;
  constexpr double kStayTimeThres = 5.0;  // s.

  if (st_boundary.max_t() < kCutoutStayTimeThres) {
    return false;
  } else if (st_boundary.max_t() > kStayTimeThres) {
    return true;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  if (overlap_infos.empty()) {
    return false;
  }

  int last_overlap_index = overlap_infos.back().av_end_idx;

  return drive_passage.FindNearestStation(ToVec2d(path[last_overlap_index]))
             .accumulated_s() >
         drive_passage.station(drive_passage.last_real_station_index())
                 .accumulated_s() +
             kCutoutDistanceHysteresis;
}

void MakeIgnoreForOccludedStBoundaries(
    int plan_id, const SpacetimeTrajectoryManager& st_traj_mgr,
    const PathPoint& path_point, const DrivePassage& drive_passage,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::optional<std::string>& nearest_object_id,
    const std::optional<std::string>& nearest_stay_object_id,
    const absl::flat_hash_map<std::string, double>& on_path_object_dist_map,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  CHECK_NOTNULL(st_boundaries_wd);

  if (!nearest_object_id.has_value()) return;

  auto nearest_st_boundary =
      std::find_if(st_boundaries_wd->begin(), st_boundaries_wd->end(),
                   [&nearest_object_id](const auto& st_boundary) {
                     const auto& obj_id = st_boundary.object_id();
                     return obj_id.has_value() && *obj_id == *nearest_object_id;
                   });
  if (nearest_st_boundary == st_boundaries_wd->end()) return;
  const StBoundary* nearest_raw_boundary =
      nearest_st_boundary->raw_st_boundary();

  if (nearest_raw_boundary->object_type() == StBoundaryProto::CYCLIST ||
      nearest_raw_boundary->object_type() == StBoundaryProto::PEDESTRIAN) {
    return;
  }

  const double nearest_stay_object_s =
      nearest_stay_object_id.has_value()
          ? *CHECK_NOTNULL(
                FindOrNull(on_path_object_dist_map, *nearest_stay_object_id)) +
                kEpsilon
          : kInf;
  const double on_path_object_filter_s = std::max(nearest_stay_object_s, 80.0);

  const double nearest_object_s =
      *CHECK_NOTNULL(FindOrNull(on_path_object_dist_map, *nearest_object_id));

  HalfPlane left, right;
  bool success_to_generate_fov = false;
  for (const StBoundaryWithDecision& stb_wd : *st_boundaries_wd) {
    const auto& obj_id = stb_wd.object_id();
    if (!obj_id.has_value() || *obj_id != nearest_object_id) {
      continue;
    }
    const auto& raw_stb = *stb_wd.raw_st_boundary();
    CHECK(raw_stb.traj_id().has_value());
    const auto& traj_id = *raw_stb.traj_id();
    const auto* st_traj =
        CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
    const auto av_fov =
        ComputeAvFovByCipv(path_point, vehicle_geometry_params,
                           st_traj->contour(), nearest_object_s);
    left = av_fov.first;
    right = av_fov.second;
    success_to_generate_fov = true;
    break;
  }
  if (!success_to_generate_fov) {
    return;
  }

  const auto is_occluded_by_cipv =
      [&left, &right](const Polygon2d& object_polygon, auto type) {
        if (type != StBoundaryProto::VEHICLE &&
            type != StBoundaryProto::UNKNOWN_OBJECT) {
          return false;
        }
        const auto& object_points = object_polygon.points();
        for (const auto& point : object_points) {
          if (left.IsPointInside(point) || !right.IsPointInside(point)) {
            return false;
          }
        }
        return true;
      };

  const auto is_oncoming = [](const auto& overlap_meta, auto type) {
    if (type != StBoundaryProto::VEHICLE &&
        type != StBoundaryProto::UNKNOWN_OBJECT) {
      return false;
    }
    if (!overlap_meta.has_value() || !overlap_meta->has_is_oncoming()) {
      return false;
    }
    return overlap_meta->is_oncoming();
  };

  const auto is_further_than_second = [&on_path_object_dist_map](double obj_s) {
    if (on_path_object_dist_map.size() < 3) return false;
    std::vector<double> distances;
    distances.reserve(on_path_object_dist_map.size());
    for (const auto& [_, dist] : on_path_object_dist_map)
      distances.push_back(dist);
    std::sort(distances.begin(), distances.end());
    return obj_s > distances[1];
  };

  const auto make_ignore_decision = [](const std::string& decision_info,
                                       auto* stb_wd) {
    stb_wd->set_decision_type(StBoundaryProto::IGNORE);
    stb_wd->set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
    stb_wd->set_ignore_reason(StBoundaryProto::OCCLUDED_OBJECT);
    stb_wd->set_decision_info(decision_info);
  };

  bool long_time_stay_cipv =
      IsLongTimeStayCipv(drive_passage, path, *nearest_raw_boundary);

  absl::flat_hash_set<std::string> occluded_object_id_set;
  for (StBoundaryWithDecision& stb_wd : *st_boundaries_wd) {
    const StBoundary& raw_stb = *stb_wd.raw_st_boundary();

    const auto& obj_id = raw_stb.object_id();

    if (raw_stb.source_type() != StBoundarySourceTypeProto::ST_OBJECT ||
        !obj_id.has_value() ||
        stb_wd.decision_type() == StBoundaryProto::IGNORE ||
        raw_stb.min_t() > 0.0) {
      continue;
    }
    if (*obj_id == *nearest_object_id) {
      continue;
    }
    const auto object_s = FindOrNull(on_path_object_dist_map, *obj_id);
    if (object_s != nullptr && *object_s > nearest_object_s) {
      if (ContainsKey(occluded_object_id_set, *obj_id)) {
        continue;
      }
      const SpacetimeObjectTrajectory* st_traj =
          CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*raw_stb.traj_id()));
      if (long_time_stay_cipv && *object_s > on_path_object_filter_s) {
        const auto decision_info =
            "ignore non-nearest on-path beyond 80m st-boundary";
        make_ignore_decision(decision_info, &stb_wd);
        occluded_object_id_set.insert(*obj_id);
        continue;
      }

      if (long_time_stay_cipv &&
          is_occluded_by_cipv(st_traj->contour(), raw_stb.object_type())) {
        const auto decision_info =
            "ignore non-nearest on-path fov occluded st-boundary";
        make_ignore_decision(decision_info, &stb_wd);
        occluded_object_id_set.insert(*obj_id);
        continue;
      }

      if (is_oncoming(raw_stb.overlap_meta(), raw_stb.object_type())) {
        const auto decision_info =
            "ignore non-nearest on-path oncoming st-boundary";
        make_ignore_decision(decision_info, &stb_wd);
        occluded_object_id_set.insert(*obj_id);
        continue;
      }

      if (long_time_stay_cipv && is_further_than_second(*object_s)) {
        const auto decision_info =
            "ignore non-nearest on-path further than second st-boundary";
        make_ignore_decision(decision_info, &stb_wd);
        occluded_object_id_set.insert(*obj_id);
        continue;
      }
    }
  }
  return;
}

}  // namespace

void MakeIgnoreAndPreBrakeDecisionForStBoundaries(
    const IgnoreDeciderInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::optional<VtSpeedLimit>* speed_limit,
    CipvObjectInfo* cipv_object_info) {
  CHECK_NOTNULL(input.params);
  CHECK_NOTNULL(input.path);
  CHECK_NOTNULL(input.path_semantics);
  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.drive_passage);
  CHECK_NOTNULL(input.vehicle_geometry_params);
  CHECK_NOTNULL(input.av_shapes);
  CHECK_NOTNULL(cipv_object_info);
  CHECK_NOTNULL(input.lane_change_state);
  CHECK_GT(input.max_v, 0.0);
  CHECK_GT(input.time_step, 0.0);
  CHECK_GT(input.trajectory_steps, 0);

  // Prepare some common input.
  // Calculate the AV emergency stop st points.
  // const auto emergency_stop_points = GenerateAvEmergencyStopStPoints(
  //     input.path->front().s(), input.current_v, input.trajectory_steps);
  // Calculate the travel length for left turn to be used in ignore decider.
  const auto& path_semantics = *input.path_semantics;
  bool on_left_turn_waiting_lane = false;
  if (!path_semantics.empty() && path_semantics.front().lane_semantic ==
                                     LaneSemantic::INTERSECTION_LEFT_TURN) {
    CHECK_NOTNULL(path_semantics.front().lane_info);
    const auto& current_lane_info = *path_semantics.front().lane_info;
    CHECK(current_lane_info.turn_type() ==
          ad_byd::planning::TurnType::LEFT_TURN);
    // Check if on left turn waiting lane.
    on_left_turn_waiting_lane = true;
    if (!current_lane_info.next_lane_ids().empty()) {
      for (const auto& lane_id : current_lane_info.next_lane_ids()) {
        const auto& outgoing_lane_info_ptr =
            input.psmm->FindCurveLaneByIdOrNull(lane_id);
        if (outgoing_lane_info_ptr == nullptr) continue;
        if (outgoing_lane_info_ptr->turn_type() !=
            ad_byd::planning::TurnType::LEFT_TURN) {
          on_left_turn_waiting_lane = false;
          break;
        }
      }
    } else {
      on_left_turn_waiting_lane = false;
    }
  }

  Merge_Direction merge_direction = MERGE_NONE;

  for (const auto& driving_process : input.driving_process_seq) {
    if (ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_LEFT ==
        driving_process.merge_topology) {
      merge_direction = MERGE_LEFT;
      break;
    } else if (ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_RIGHT ==
               driving_process.merge_topology) {
      merge_direction = MERGE_RIGHT;
      break;
    }
  }

  SplitTopology split_direction = SplitTopology::TOPOLOGY_SPLIT_NONE;
  for (const auto& driving_process : input.driving_process_seq) {
    if (ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_LEFT ==
        driving_process.split_topology) {
      split_direction = SplitTopology::TOPOLOGY_SPLIT_LEFT;
      break;
    } else if (ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_RIGHT ==
               driving_process.split_topology) {
      split_direction = SplitTopology::TOPOLOGY_SPLIT_RIGHT;
      break;
    }
  }

  // Calculate AV frenet box info on drive passage.
  const auto& curr_path_point = input.path->front();
  const auto av_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *input.vehicle_geometry_params);
  const auto av_frenet_box_or = input.drive_passage->QueryFrenetBoxAt(av_box);
  std::optional<FrenetBox> av_frenet_box = std::nullopt;
  bool av_in_drive_passage_lane = false;
  if (av_frenet_box_or.ok()) {
    av_frenet_box = *av_frenet_box_or;
    const auto av_s_min_boundary =
        input.drive_passage->QueryEnclosingLaneBoundariesAtS(
            av_frenet_box_or->s_min);
    const auto av_s_max_boundary =
        input.drive_passage->QueryEnclosingLaneBoundariesAtS(
            av_frenet_box_or->s_max);
    // Check if AV is entirely in drive passage lane.
    av_in_drive_passage_lane =
        av_s_min_boundary.left.has_value() &&
        av_s_max_boundary.left.has_value() &&
        av_s_min_boundary.right.has_value() &&
        av_s_max_boundary.right.has_value() &&
        av_frenet_box->l_max < av_s_min_boundary.left->lat_offset &&
        av_frenet_box->l_max < av_s_max_boundary.left->lat_offset &&
        av_frenet_box->l_min > av_s_min_boundary.right->lat_offset &&
        av_frenet_box->l_min > av_s_max_boundary.right->lat_offset;
  }

  // Get upcoming merge lane ids.
  std::vector<mapping::ElementId> upcoming_merge_lane_ids;
  // if (!path_semantics.empty() &&
  //     path_semantics.front().lane_semantic == LaneSemantic::ROAD &&
  //     av_in_drive_passage_lane) {
  //   const auto& current_closest_lane_point =
  //       path_semantics.front().closest_lane_point;
  //   CHECK_NOTNULL(path_semantics.front().lane_info);
  //   const auto& current_lane_info = *path_semantics.front().lane_info;
  //   const auto& current_lane_proto = *current_lane_info.proto;
  //   for (const auto& interaction : current_lane_proto.interactions()) {
  //     // Only consider lane interactions beyond current AV position.
  //     if (interaction.this_lane_fraction() <
  //         current_closest_lane_point.fraction()) {
  //       continue;
  //     }
  //     constexpr double kUpcomingMergeThres = 30.0;  // m.
  //     if (interaction.geometric_configuration() ==
  //             mapping::LaneInteractionProto::MERGE &&
  //         current_lane_info.length() * (interaction.this_lane_fraction() -
  //                                       current_closest_lane_point.fraction())
  //                                       <
  //             kUpcomingMergeThres) {
  //       upcoming_merge_lane_ids.push_back(
  //           mapping::ElementId(interaction.other_lane_id()));
  //     }
  //   }
  // }

  // get cipv st_boundary
  const StBoundary* cipv_st_boundary =
      GetCipvStBoundary(*st_boundaries_wd, *cipv_object_info);

  // <id of the corresponding original st-boundary, protective st-boundary
  // pointer>
  absl::flat_hash_map<std::string, StBoundaryWithDecision*>
      protective_st_boundary_wd_map;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (!st_boundary_wd.raw_st_boundary()->is_protective() ||
        st_boundary_wd.raw_st_boundary()->protection_type() ==
            StBoundaryProto::LANE_CHANGE_GAP) {
      continue;
    }
    const auto& protected_st_boundary_id =
        st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
    if (!protected_st_boundary_id.has_value()) continue;
    protective_st_boundary_wd_map.emplace(*protected_st_boundary_id,
                                          &st_boundary_wd);
  }

  bool lc_left = input.lane_change_state->lc_left();

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }
    MakeIgnoreAndPreBrakeDecisionForStBoundary(
        *input.params, *input.path, path_semantics, *input.st_traj_mgr,
        *input.drive_passage, *input.vehicle_geometry_params, *input.av_shapes,
        input.current_v, input.max_v, input.time_step, input.trajectory_steps,
        *input.psmm, /*emergency_stop_points,*/ on_left_turn_waiting_lane,
        av_frenet_box, av_in_drive_passage_lane, upcoming_merge_lane_ids,
        &st_boundary_wd, speed_limit, input.lc_stage, merge_direction,
        input.current_acc, input.driving_process_seq, split_direction,
        input.is_narrow_near_large_vehicle, cipv_st_boundary,
        input.follow_time_headway, *input.path_kd_tree, *cipv_object_info,
        lc_left);
    VLOG(2) << "[speed finder] id: " << st_boundary_wd.id()
            << " | decision info " << st_boundary_wd.decision_info()
            << " | decision_type " << st_boundary_wd.decision_type();
    if (const auto protective_st_boundary_wd_ptr =
            FindOrNull(protective_st_boundary_wd_map, st_boundary_wd.id());
        nullptr != protective_st_boundary_wd_ptr) {
      auto& protective_st_boundary_wd = **protective_st_boundary_wd_ptr;
      // The decision of protective st-boundary follows the decision of the
      // corresponding original st-boundary.
      if (protective_st_boundary_wd.decision_type() !=
          st_boundary_wd.decision_type()) {
        protective_st_boundary_wd.set_decision_type(
            st_boundary_wd.decision_type());
        protective_st_boundary_wd.set_decision_reason(
            StBoundaryProto::FOLLOW_PROTECTED);
        protective_st_boundary_wd.set_ignore_reason(
            st_boundary_wd.ignore_reason());
        protective_st_boundary_wd.set_decision_info(
            st_boundary_wd.decision_info());
      }
    }
  }

  // Whether to recalculate CIPV.
  for (const auto& st_boundary_wd : *st_boundaries_wd) {
    const auto& object_id = st_boundary_wd.object_id();
    if (st_boundary_wd.decision_type() != StBoundaryProto::IGNORE ||
        !object_id.has_value()) {
      continue;
    }
    const auto& cipv = cipv_object_info->nearest_object_id;
    const auto& stay_cipv = cipv_object_info->nearest_stay_object_id;
    if ((cipv.has_value() && *object_id == *cipv) ||
        (stay_cipv.has_value() && *object_id == *stay_cipv)) {
      *cipv_object_info = ComputeCipvObjectInfo(
          *input.st_traj_mgr, *input.path_kd_tree, st_boundaries_wd);
      break;
    }
  }

  // Only keep the nearest non-ignored stationary st-boundary.
  MakeIgnoreDecisionForNonNearestStationaryStBoundaries(st_boundaries_wd);

  MakeIgnoreDecisionForFarAwaySignificantLateralStBoundaries(
      st_boundaries_wd, *input.path, *input.st_traj_mgr,
      *input.vehicle_geometry_params, *input.av_shapes, cipv_st_boundary,
      *input.path_kd_tree, *cipv_object_info);

  if (input.params->ignore_occluded_on_path_objects()) {
    MakeIgnoreForOccludedStBoundaries(
        input.plan_id, *input.st_traj_mgr, input.path->front(),
        *input.drive_passage, *input.path, *input.vehicle_geometry_params,
        cipv_object_info->nearest_object_id,
        cipv_object_info->nearest_stay_object_id,
        cipv_object_info->object_distance_map, st_boundaries_wd);
  }
}
}  // namespace planning
}  // namespace st
