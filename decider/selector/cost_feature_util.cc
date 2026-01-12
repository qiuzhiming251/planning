#include "decider/selector/cost_feature_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <optional>
#include <string_view>
#include <tuple>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/types/span.h"

// #include "global/logging.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/geometry/util.h"
#include "object_manager/partial_spacetime_object_trajectory.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/path_sl_boundary.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_object_state.h"
// #include "semantic_map.pb.h"
#include "plan_common/maps/semantic_map_util.h"
#include "decider/selector/selector_defs.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/spatial_search_util.h"
#include "plan_common/util/status_macros.h"

namespace st {
namespace planning {
namespace {
constexpr double kOccupiedCrossBoundaryFactor = 1.1;
constexpr double kFullCrossBoundaryFactor = 1.0;
constexpr double kDefaultCrossBoundaryFactor = 0.8;
constexpr double kCheckIsLeaderObjectTime = 3.0;  // s
constexpr double kObjectConfidenceThreshold = 0.7;
constexpr double kNearObjectConfidenceThreshold = 0.9;
constexpr double kCheckIsLeaderObjectBeginTimeHighWay = 1.0;  // s
constexpr double kLcCheckIsLeaderObjectEndTimeHighWay = 5.5;  // s
constexpr double kLkCheckIsLeaderObjectEndTimeHighWay = 6.0;  // s
constexpr double kCheckIsLeaderObjectBeginTime = 1.5;         // s
constexpr double kLcCheckIsLeaderObjectEndTime = 4.5;         // s
constexpr double kLkCheckIsLeaderObjectEndTime = 5.0;         // s
constexpr double kCheckIsLeaderObjectStep = 1.0;              // s

bool IsValidLaneAttrType(const LaneAttrType& lane_attr_type) {
  return lane_attr_type == LaneAttrType::LANEATTR_SELF ||
         lane_attr_type == LaneAttrType::LANEATTR_ON_LINE;
}

bool IsFollowLaneType(
    const PartialSpacetimeObjectTrajectory* const* st_object_ptr,
    const double start_time, const double end_time, const double step) {
  if (st_object_ptr == nullptr || start_time < 0 || start_time > 8.0 ||
      end_time < 0 || end_time > 8.0 || step <= 0 || step > 8.0)
    return false;

  int count = 0;
  for (double t = start_time; t <= end_time; t += step) {
    const auto decision_type = (*st_object_ptr)->GetDecisionTypeAtTime(t);
    if (decision_type.has_value() &&
        *decision_type ==
            PartialSpacetimeObjectTrajectory::DecisionType::FOLLOW) {
      if (++count >= 2) return true;
    }
  }
  return false;
}

}  // namespace

// absl::StatusOr<PointOnRouteSections> FindLastTrajPointOnRouteSections(
//     const PlannerSemanticMapManager& psmm,
//     const RouteSectionsInfo& sections_info, const DrivePassage& dp,
//     const PlannerTrajectory& traj_pts) {
//   const double lp_end_s = dp.lane_path().length() + dp.lane_path_start_s();
//   for (int idx = traj_pts.size() - 1; idx >= 0; --idx) {
//     const Vec2d pt_pos = Vec2dFromApolloTrajectoryPointProto(traj_pts[idx]);
//     const double heading = traj_pts[idx].path_point().theta();
//     const auto proj_or = dp.QueryFrenetLonOffsetAt(pt_pos);
//     if (!proj_or.ok() || proj_or->accum_s > lp_end_s) continue;

//     ASSIGN_OR_CONTINUE(
//         const auto lane_pt,
//         FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
//             psmm.GetLevel(), psmm, pt_pos, dp.lane_path(), heading));

//     const auto& sections = sections_info.section_segments();
//     double accum_s = 0.0;
//     int sec_idx = 0;
//     for (; sec_idx < sections_info.size(); ++sec_idx) {
//       if (sections[sec_idx].id_idx_map.contains(lane_pt.lane_id())) {
//         break;
//       }
//       accum_s += sections[sec_idx].length();
//     }
//     accum_s += sections[sec_idx].average_length *
//                (lane_pt.fraction() - sections[sec_idx].start_fraction);
//     return PointOnRouteSections{.accum_s = accum_s,
//                                 .section_idx = sec_idx,
//                                 .fraction = lane_pt.fraction(),
//                                 .lane_id = lane_pt.lane_id()};
//   }

//   return absl::NotFoundError("The whole trajectory is not on route
//   sections.");
// }

// bool IsInTlControlledIntersection(const PlannerSemanticMapManager& psmm,
//                                   const DrivePassage& drive_passage, double
//                                   s) {
//   const auto& station = drive_passage.FindNearestStationAtS(s);
//   if (!station.is_in_intersection()) return false;

//   const auto lane_pt = station.GetLanePoint();
//   SMM_ASSIGN_LANE_OR_RETURN(lane_info, psmm, lane_pt.lane_id(), false);
//   for (const auto& [id, frac] : lane_info.Intersections()) {
//     const auto* intersection_ptr = psmm.FindIntersectionByIdOrNull(id);
//     if (intersection_ptr != nullptr &&
//         intersection_ptr->proto->traffic_light_controlled() &&
//         frac[0] <= lane_pt.fraction() && lane_pt.fraction() <= frac[1]) {
//       return true;
//     }
//   }
//   return false;
// }
double LinearInterpolate(double x0, double x1, double t0, double t1, double t) {
  if (std::fabs(t1 - t0) < 1e-6) {
    return x0;
  }
  return x0 + (x1 - x0) * (t - t0) / (t1 - t0);
}

double CalculateCrossingBoundary(
    const DrivePassage& drive_passage,
    const std::vector<BoundaryInterval>& solid_boundaries,
    const std::vector<Box2d>& ego_boxes, LaneChangeStage stage,
    const absl::flat_hash_set<StationBoundaryType>& type_set,
    const absl::StatusOr<double>& start_l_or,
    const absl::StatusOr<double>& end_l_or, const double ego_half_width) {
  double crossing_factor = 0;

  for (const auto& boundary : solid_boundaries) {
    if (!type_set.contains(boundary.type)) {
      continue;
    }

    const auto& boundary_pts = boundary.points;
    for (const auto& ego_box : ego_boxes) {
      bool has_overlap = false;
      absl::StatusOr<double> cross_l_or;
      for (int j = 1; j < boundary_pts.size(); ++j) {
        const Segment2d boundary_seg(boundary_pts[j - 1], boundary_pts[j]);
        if (ego_box.HasOverlap(boundary_seg)) {
          cross_l_or = drive_passage.QueryFrenetLatOffsetAt(boundary_pts.at(j));
          has_overlap = true;
          break;
        }
      }
      if (has_overlap) {
        double factor = kDefaultCrossBoundaryFactor;
        if (cross_l_or.ok() && start_l_or.ok() && end_l_or.ok()) {
          const double cross_l = cross_l_or.value();
          const double start_l = start_l_or.value();
          const double end_l = end_l_or.value();
          if (stage == LaneChangeStage::LCS_PAUSE) {
            if (std::fabs(end_l - cross_l) < ego_half_width) {
              // Occupy solid line.
              factor = kOccupiedCrossBoundaryFactor;
            }
          } else {
            if ((start_l - cross_l) * (end_l - cross_l) < 0.0) {
              // Crossed the solid line.
              factor = kFullCrossBoundaryFactor;
            }
          }
        }
        crossing_factor = std::max(factor, crossing_factor);
        break;
      }
    }
  }
  return crossing_factor;
}

// double CalculateCrossingBoundary(
//     const DrivePassage& drive_passage,
//     const std::vector<BoundaryInterval>& solid_boundaries,
//     const std::vector<Box2d>& ego_boxes, LaneChangeStage stage,
//     const absl::flat_hash_set<StationBoundaryType>& type_set,
//     const absl::StatusOr<double>& start_l_or,
//     const absl::StatusOr<double>& end_l_or, const double ego_half_width,
//     const PlannerSemanticMapManager& psmm) {
//   double crossing_factor = 0;

//   for (const auto& boundary : solid_boundaries) {
//     if (!type_set.contains(boundary.type)) continue;

//     const auto& boundary_pts = boundary.points;
//     for (const auto& ego_box : ego_boxes) {
//       bool has_overlap = false;
//       bool has_split = false;
//       absl::StatusOr<double> cross_l_or;
//       for (int j = 1; j < boundary_pts.size(); ++j) {
//         const Segment2d boundary_seg(boundary_pts[j - 1], boundary_pts[j]);
//         if (ego_box.HasOverlap(boundary_seg)) {
//           cross_l_or =
//           drive_passage.QueryFrenetLatOffsetAt(boundary_pts.at(j));
//           has_overlap = true;

//           /// ignore overlapping around split points: in split points, the
//           lane
//           /// type is set to VITRUAL
//           // get drive passage lane ids
//           const auto& pass_lane_ids = drive_passage.lane_path().lane_ids();
//           // get overlapped lane id and its info
//           const auto overlap_lane_id =
//               drive_passage.FindNearestStation(boundary_pts.at(j)).lane_id();
//           const auto& overlap_lane_info =
//               psmm.FindCurveLaneByIdOrNull(overlap_lane_id);
//           if (overlap_lane_info == nullptr) {  // exit if not found lane info
//             DLOG(INFO) << "Not found overlapped lane id info in psmm.";
//             break;
//           }
//           if (overlap_lane_info->Proto().type() ==
//                   mapping::LaneProto::VIRTUAL &&
//               overlap_lane_info->is_in_intersection == false) {
//             has_split = true;
//             break;
//           }
//           // get the next lane id and its info
//           const auto it = std::find(pass_lane_ids.begin(),
//           pass_lane_ids.end(),
//                                     overlap_lane_id);
//           if (it == pass_lane_ids.end()) {
//             DLOG(INFO) << "Not find the overlapped lane id in the input drive
//             "
//                           "passage.";
//             break;
//           }
//           int overlapped_next_index =
//               std::distance(pass_lane_ids.begin(), it) + 1;
//           if (overlapped_next_index < pass_lane_ids.size()) {
//             const auto& next_lane_info =
//                 psmm.FindCurveLaneByIdOrNull(pass_lane_ids[overlapped_next_index]);
//             if (next_lane_info != nullptr &&
//                 next_lane_info->Proto().type() == mapping::LaneProto::VIRTUAL
//                 && next_lane_info->is_in_intersection == false) {
//               has_split = true;
//             }
//           }

//           break;
//         }
//       }
//       // ignore overlapping solid lane boundaries when it happens nearby
//       split
//       // points
//       if (has_split) {
//         break;
//       }

//       if (has_overlap) {
//         double factor = kDefaultCrossBoundaryFactor;
//         if (cross_l_or.ok() && start_l_or.ok() && end_l_or.ok()) {
//           const double cross_l = cross_l_or.value();
//           const double start_l = start_l_or.value();
//           const double end_l = end_l_or.value();
//           if (stage == LaneChangeStage::LCS_PAUSE) {
//             if (std::fabs(end_l - cross_l) < ego_half_width) {
//               // Occupy solid line.
//               factor = kOccupiedCrossBoundaryFactor;
//             }
//           } else {
//             if ((start_l - cross_l) * (end_l - cross_l) < 0.0) {
//               // Crossed the solid line.
//               factor = kFullCrossBoundaryFactor;
//             }
//           }
//         }
//         crossing_factor = std::max(factor, crossing_factor);
//         break;
//       }
//     }
//   }
//   return crossing_factor;
// }

absl::flat_hash_set<std::string> FindBlockObjectIds(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const EstPlannerOutput& planner_output,
    const VehicleGeometryParamsProto& vehicle_geom, const bool is_highway,
    const bool is_lane_keep, const bool is_lc_safety_check_falied,
    const double ego_v) {
  absl::flat_hash_set<std::string> block_obj_ids;
  constexpr double kEgoPassingLatBuffer = 0.4;               // m.
  constexpr double kObsLatInLaneBuffer = 0.4;                // m.
  constexpr double kLcSafetyCheckFailedConsiderDis = 100.0;  // m.
  constexpr double kLcSafetyCheckFailedConsiderTime = 6.0;   // s.
  const auto allow_width = vehicle_geom.width() + kEgoPassingLatBuffer;
  const auto& passage = planner_output.scheduler_output.drive_passage;
  double speed_max_consider_distance = 0.0;

  if (is_lc_safety_check_falied) {
    speed_max_consider_distance =
        std::max(kLcSafetyCheckFailedConsiderDis,
                 ego_v * kLcSafetyCheckFailedConsiderTime);
  } else {
    if (planner_output.traj_points.empty()) {
      return {};
    }
    const auto last_point_sl_or = passage.QueryFrenetCoordinateAt(
        Vec2dFromApolloTrajectoryPointProto(planner_output.traj_points.back()));
    if (last_point_sl_or.ok()) {
      speed_max_consider_distance =
          std::max(last_point_sl_or->s, speed_max_consider_distance);
    }
  }

  absl::flat_hash_map<std::string, const PartialSpacetimeObjectTrajectory*>
      speed_consider_objects_map;
  for (const auto& traj : planner_output.considered_st_objects) {
    speed_consider_objects_map[std::string(traj.st_traj().object_id())] = &traj;
    const auto init_decision_type = traj.GetDecisionTypeAtTime(0.0);
    if (init_decision_type.has_value()) {
      ASSIGN_OR_CONTINUE(const auto aabbox, passage.QueryFrenetBoxAtContour(
                                                traj.st_traj().contour()));
      speed_max_consider_distance =
          std::max(speed_max_consider_distance, aabbox.s_min);
    }
  }

  absl::flat_hash_set<std::string> has_checked_set;
  for (const auto& traj : st_traj_mgr.frame_dropped_trajectories()) {
    // Remove duplicated object id
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());

    // Ignore objects just passing through the current lane path.
    const auto& obj_pose = *traj.states().front().traj_point;
    const auto* st_object_ptr =
        FindOrNull(speed_consider_objects_map, std::string(traj.object_id()));
    ASSIGN_OR_CONTINUE(const auto dp_tan,
                       passage.QueryTangentAt(obj_pose.pos()));
    if (!traj.is_stationary() &&
        std::abs(NormalizeAngle(dp_tan.FastAngle() - obj_pose.theta())) >
            M_PI_4) {
      continue;
    }
    // Ignore objects not in front.
    ASSIGN_OR_CONTINUE(const auto aabbox,
                       passage.QueryFrenetBoxAtContour(traj.contour()));
    if (aabbox.s_min < 0.0) continue;

    // Ignore stationary object ignored by speed
    if (traj.is_stationary() && aabbox.s_min < speed_max_consider_distance &&
        st_object_ptr == nullptr) {
      continue;
    }
    double boundary_right_l, boundary_left_l;
    if (planner_output.scheduler_output.borrow_lane) {
      // Use target boundary for lane borrow
      std::tie(boundary_right_l, boundary_left_l) =
          planner_output.scheduler_output.sl_boundary.QueryTargetBoundaryL(
              aabbox.center_s());
    } else {
      const auto [right_boundary, left_boundary] =
          passage.QueryEnclosingLaneBoundariesAtS(aabbox.center_s());
      boundary_right_l =
          right_boundary.has_value()
              ? std::max(right_boundary->lat_offset, -kMaxHalfLaneWidth)
              : -kMaxHalfLaneWidth;
      boundary_left_l =
          left_boundary.has_value()
              ? std::min(left_boundary->lat_offset, kMaxHalfLaneWidth)
              : kMaxHalfLaneWidth;
    }
    const double l_max =
        std::clamp(aabbox.l_max, boundary_right_l, boundary_left_l);
    const double l_min =
        std::clamp(aabbox.l_min, boundary_right_l, boundary_left_l);

    if (std::min(boundary_left_l - l_min, l_max - boundary_right_l) <
        kObsLatInLaneBuffer) {
      continue;
    }
    // //// 针对绕行task忽略前方慢车
    // if (planner_output.scheduler_output.borrow_lane &&
    //     traj.planner_object().pose().v() < 8.33 &&
    //     std::max(boundary_left_l - l_max, l_min - boundary_right_l) >
    //     allow_width) {
    //   continue;
    // }

    if (st_object_ptr != nullptr) {
      double begin_time = is_highway ? kCheckIsLeaderObjectBeginTimeHighWay
                                     : kCheckIsLeaderObjectBeginTime;
      double end_time =
          is_highway ? (is_lane_keep ? kLkCheckIsLeaderObjectEndTimeHighWay
                                     : kLcCheckIsLeaderObjectEndTimeHighWay)
                     : (is_lane_keep ? kLkCheckIsLeaderObjectEndTime
                                     : kLcCheckIsLeaderObjectEndTime);
      if (!IsFollowLaneType(st_object_ptr, begin_time, end_time,
                            kCheckIsLeaderObjectStep)) {
        continue;
      }
    } else {
      if (std::max(boundary_left_l - l_max, l_min - boundary_right_l) >
          allow_width) {
        // Out of target lane path or too small to block the ego vehicle.
        continue;
      }
    }
    block_obj_ids.emplace(traj.object_id());
  }
  return block_obj_ids;
}

// not block but invade
std::map<std::string, InvadeStaticObjInfo> FindInvadeStaticObjects(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const EstPlannerOutput& planner_output,
    const absl::flat_hash_set<std::string>& block_obj_ids,
    const absl::flat_hash_set<std::string>& stalled_objects) {
  const auto& passage = planner_output.scheduler_output.drive_passage;
  absl::flat_hash_set<std::string> has_checked_set;
  std::map<std::string, InvadeStaticObjInfo> invade_static_obj_map;
  for (const auto& traj : st_traj_mgr.trajectories()) {
    // Remove duplicated object id
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());
    if (block_obj_ids.contains(traj.object_id())) continue;
    if (!traj.is_stationary()) continue;
    // Ignore objects not in front.
    ASSIGN_OR_CONTINUE(const auto aabbox,
                       passage.QueryFrenetBoxAtContour(traj.contour()));
    if (aabbox.s_min < 0.0) continue;

    double boundary_right_l, boundary_left_l;
    const auto [right_boundary, left_boundary] =
        passage.QueryEnclosingLaneBoundariesAtS(aabbox.center_s());
    boundary_right_l =
        right_boundary.has_value()
            ? std::max(right_boundary->lat_offset, -kMaxHalfLaneWidth)
            : -kMaxHalfLaneWidth;
    boundary_left_l =
        left_boundary.has_value()
            ? std::min(left_boundary->lat_offset, kMaxHalfLaneWidth)
            : kMaxHalfLaneWidth;
    if (aabbox.l_max < boundary_right_l || aabbox.l_min > boundary_left_l) {
      continue;
    }
    const double l_max =
        std::clamp(aabbox.l_max, boundary_right_l, boundary_left_l);
    const double l_min =
        std::clamp(aabbox.l_min, boundary_right_l, boundary_left_l);

    invade_static_obj_map.insert(
        {std::string(traj.object_id()),
         {
             .obj_s_min = aabbox.s_min,
             .obj_s_max = aabbox.s_max,
             .obj_l_min = aabbox.l_min,
             .obj_l_max = aabbox.l_max,
             .obj_type = traj.object_type(),
             .boundary_left_l = boundary_left_l,
             .boundary_right_l = boundary_right_l,
             .left_remain_width = std::fmax(0.0, boundary_left_l - l_max),
             .right_remain_width = std::fmax(0.0, l_min - boundary_right_l),
             .is_stalled = stalled_objects.contains(traj.object_id()),
         }});
  }
  return invade_static_obj_map;
}

TargetLaneStateProto GenerateTargetLaneState(
    const PlannerSemanticMapManager& psmm, const SchedulerOutput& output,
    const double preview_distance) {
  TargetLaneStateProto target_lane_state;
  target_lane_state.set_is_borrow(output.borrow_lane);
  target_lane_state.set_is_fallback(output.is_fallback);
  if (output.drive_passage.lane_seq_info() != nullptr &&
      output.drive_passage.lane_seq_info()->lane_seq != nullptr) {
    const auto& lanes = output.drive_passage.lane_seq_info()->lane_seq->lanes();
    for (const auto& lane : lanes) {
      if (lane != nullptr) {
        target_lane_state.add_lane_ids(lane->id());
      }
    }
  } else {
    target_lane_state.mutable_lane_ids()->Reserve(
        output.drive_passage.extend_lane_path().lane_ids().size());
    for (const auto& lane_id :
         output.drive_passage.extend_lane_path().lane_ids()) {
      target_lane_state.add_lane_ids(lane_id);
    }
  }
  double distance = 0.0;
  const auto sample_center_points = SampleLanePathPoints(
      *psmm.map_ptr(), output.drive_passage.extend_lane_path());
  target_lane_state.mutable_center_points()->Reserve(
      sample_center_points.size());
  for (int i = 0; i < sample_center_points.size(); ++i) {
    Vec2dToProto(sample_center_points[i],
                 target_lane_state.add_center_points());
    if (i > 0) {
      distance +=
          sample_center_points[i].DistanceTo(sample_center_points[i - 1]);
    }
    if (distance > preview_distance) {
      break;
    }
  }
  target_lane_state.set_successive_count(1);
  return target_lane_state;
}

std::optional<LeaderObjectInfo> CalcNearestLeaderFromBlockObjects(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage,
    const absl::flat_hash_set<std::string>& block_obj_ids) {
  absl::flat_hash_set<absl::string_view> has_checked_set;
  std::optional<LeaderObjectInfo> leader_obj_info;
  for (const auto& traj : st_traj_mgr.trajectories()) {
    if (!block_obj_ids.contains(traj.object_id())) {
      continue;
    }
    auto [_, inserted] = has_checked_set.insert(traj.object_id());
    if (!inserted) {
      continue;
    }
    const auto& contour = traj.contour();
    auto obj_aabox_or = drive_passage.QueryFrenetBoxAtContour(contour);
    if (!obj_aabox_or.ok()) {
      continue;
    }
    if (!leader_obj_info.has_value() ||
        leader_obj_info->obj_s > obj_aabox_or->s_min) {
      leader_obj_info = LeaderObjectInfo{
          .obj_id = std::string(traj.object_id()),
          .obj_s = obj_aabox_or->s_min,
          .obj_v = traj.pose().v(),
          .obj_type = traj.object_type(),
          .is_stationary = traj.is_stationary(),
      };
    }
  }
  return leader_obj_info;
}

bool IsObjectBlockingCenterLine(double obj_l_min, double obj_l_max,
                                double center_l, double lat_buffer) {
  return obj_l_max < center_l + lat_buffer && obj_l_min > center_l - lat_buffer;
}

bool IsObjectOverlapLine(double obj_l_min, double obj_l_max,
                         double boundary_line_offset, double left_buffer,
                         double right_buffer) {
  return obj_l_max > boundary_line_offset - right_buffer &&
         obj_l_min < boundary_line_offset + left_buffer;
}

LaneConesInfo CalcNeighborConesOnLane(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage, const FrenetBox& av_frenet_box,
    const absl::flat_hash_set<std::string>& block_obj_ids) {
  absl::flat_hash_set<absl::string_view> has_checked_set;
  int left_negibor_cone_num = 0;
  int right_negibor_cone_num = 0;
  int left_neighbor_cone_num_strigent = 0;
  int right_neighbor_cone_num_strigent = 0;
  int blocking_lane_num = 0;
  int lane_attr_left_negibor_cone_num = 0;
  int lane_attr_right_negibor_cone_num = 0;
  std::string left_invade_cone_id = "Invalid";
  std::string right_invade_cone_id = "Invalid";
  int left_attribute_satisfied_count = 0;
  int right_attribute_satisfied_count = 0;
  int lc_left_direction_cones_count = 0;
  int lc_right_direction_cones_count = 0;

  for (const auto& [object_id, object] : st_traj_mgr.stationary_objects()) {
    if (block_obj_ids.contains(object_id)) {
      continue;
    }
    auto [_, inserted] = has_checked_set.insert(object_id);
    if (!inserted) {
      continue;
    }
    if (!IsConstructionObject(object.type())) {
      continue;
    }

    const auto& contour = object.contour();
    auto obj_aabox_or = drive_passage.QueryFrenetBoxAtContour(contour);
    if (!obj_aabox_or.ok()) {
      continue;
    }
    constexpr double kOutofCredibleLength = 150.0;
    constexpr double kTooCloseLength = 10.0;
    constexpr double kNearConesLength = 80;  // m.
    if (obj_aabox_or->s_min > kOutofCredibleLength) {
      continue;
    }
    if (obj_aabox_or->s_max < av_frenet_box.s_max + kTooCloseLength) {
      continue;
    }

    constexpr double kBlockingBuffer = kDefaultHalfLaneWidth - 0.7;
    if (IsObjectBlockingCenterLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                   /*center_l*/ 0.0, kBlockingBuffer)) {
      ++blocking_lane_num;
      continue;
    }
    const auto [right_boundary, left_boundary] =
        drive_passage.QueryEnclosingLaneBoundariesAtS(obj_aabox_or->s_min);
    // Exclude widen lane
    if (std::fabs(left_boundary->lat_offset - right_boundary->lat_offset) >
        2 * kMaxHalfLaneWidth) {
      continue;
    }

    constexpr double kOutInvasionBufferlcside = 0.6;    // m.
    constexpr double kInInvasionBuffer = 0.4;           // m.
    constexpr double kOutInvasionBuffer = 0.5;          // m.
    constexpr double kOutInvasionBufferstrigent = 0.3;  // m.
    if (obj_aabox_or->s_min > kNearConesLength) {
      if (right_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
          IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                              right_boundary->lat_offset, kInInvasionBuffer,
                              kOutInvasionBufferlcside)) {
        ++lc_right_direction_cones_count;
        if (right_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
            IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                right_boundary->lat_offset, kInInvasionBuffer,
                                kOutInvasionBuffer)) {
          ++right_negibor_cone_num;
          if (IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                  right_boundary->lat_offset, kInInvasionBuffer,
                                  kOutInvasionBufferstrigent)) {
            ++right_neighbor_cone_num_strigent;

            if (object.object_proto().vision_attribute().lane_attr_conf() >=
                    kObjectConfidenceThreshold &&
                IsValidLaneAttrType(object.object_proto()
                                        .vision_attribute()
                                        .lane_attr_type())) {
              if (lane_attr_right_negibor_cone_num < 1) {
                right_invade_cone_id = object_id;
              }

              lane_attr_right_negibor_cone_num++;
            }
          }
        }
      }
      if (left_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
          IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                              left_boundary->lat_offset, kOutInvasionBuffer,
                              kOutInvasionBufferlcside)) {
        ++lc_left_direction_cones_count;
        if (left_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
            IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                left_boundary->lat_offset, kOutInvasionBuffer,
                                kInInvasionBuffer)) {
          ++left_negibor_cone_num;
          if (IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                  right_boundary->lat_offset, kInInvasionBuffer,
                                  kOutInvasionBufferstrigent)) {
            ++left_neighbor_cone_num_strigent;

            if (object.object_proto().vision_attribute().lane_attr_conf() >=
                    kObjectConfidenceThreshold &&
                IsValidLaneAttrType(object.object_proto()
                                        .vision_attribute()
                                        .lane_attr_type())) {
              if (lane_attr_left_negibor_cone_num < 1) {
                left_invade_cone_id = object_id;
              }

              lane_attr_left_negibor_cone_num++;
            }
          }
        }
      }
    } else {
      if (right_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
          IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                              right_boundary->lat_offset, kInInvasionBuffer,
                              kOutInvasionBufferlcside)) {
        ++lc_right_direction_cones_count;
        if (right_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
            IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                right_boundary->lat_offset, kInInvasionBuffer,
                                kOutInvasionBuffer)) {
          ++right_negibor_cone_num;
          if (IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                  right_boundary->lat_offset, kInInvasionBuffer,
                                  kOutInvasionBufferstrigent)) {
            ++right_neighbor_cone_num_strigent;
            if (object.object_proto().vision_attribute().lane_attr_conf() >=
                kNearObjectConfidenceThreshold) {
              if (lane_attr_right_negibor_cone_num < 1) {
                right_invade_cone_id = object_id;
              }

              lane_attr_right_negibor_cone_num++;
            }
          }
        }
      }
      if (left_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
          IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                              left_boundary->lat_offset, kOutInvasionBuffer,
                              kOutInvasionBufferlcside)) {
        ++lc_left_direction_cones_count;
        if (left_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
            IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                left_boundary->lat_offset, kOutInvasionBuffer,
                                kInInvasionBuffer)) {
          ++left_negibor_cone_num;
          if (IsObjectOverlapLine(obj_aabox_or->l_min, obj_aabox_or->l_max,
                                  right_boundary->lat_offset, kInInvasionBuffer,
                                  kOutInvasionBufferstrigent)) {
            ++left_neighbor_cone_num_strigent;
            if (object.object_proto().vision_attribute().lane_attr_conf() >=
                kNearObjectConfidenceThreshold) {
              if (lane_attr_left_negibor_cone_num < 1) {
                left_invade_cone_id = object_id;
              }

              lane_attr_left_negibor_cone_num++;
            }
          }
        }
      }
    }
  }
  return {
      .left_cones_count = lane_attr_left_negibor_cone_num > 0
                              ? left_neighbor_cone_num_strigent
                              : 0,
      .right_cones_count = lane_attr_right_negibor_cone_num > 0
                               ? right_neighbor_cone_num_strigent
                               : 0,
      .blocking_cones_count = blocking_lane_num,
      .left_invade_cone_id = left_invade_cone_id,
      .right_invade_cone_id = right_invade_cone_id,
      .left_cones_invade_count = left_negibor_cone_num,
      .right_cones_invade_count = right_negibor_cone_num,
      .left_attribute_satisfied_count = lane_attr_left_negibor_cone_num,
      .right_attribute_satisfied_count = lane_attr_right_negibor_cone_num,
      .lc_left_direction_cones_count =
          lc_left_direction_cones_count,
      .lc_right_direction_cones_count =
          lc_right_direction_cones_count,
  };
}

bool CanIgnoreCrossSolidBoundary(LaneChangeType lane_change_type, int lc_num,
                                 double length_along_route, bool /*lc_left*/,
                                 bool is_highway, bool in_tunnel,
                                 double lane_keep_dist_to_merge) {
  if (in_tunnel) {
    switch (lane_change_type) {
      case TYPE_EMERGENCY_CHANGE:
      case TYPE_PADDLE_CHANGE:
      case TYPE_OBSTACLE_CHANGE:
      case TYPE_CONSTRUCTION_ZONE_CHANGE:
      case TYPE_STALLED_VEHICLE_CHANGE:
        return true;
      case TYPE_DEFAULT_ROUTE_CHANGE:
      case TYPE_DEFAULT_CHANGE:
      case TYPE_NO_CHANGE:
      case TYPE_OVERTAKE_CHANGE:
      case TYPE_ROAD_SPEED_LIMIT_CHANGE:
      case TYPE_MAINROAD_EXIT_CHANGE:
      case TYPE_ENTER_MAINROAD_CHANGE:
      case TYPE_MERGE_CHANGE:
      case TYPE_CENTER_CHANGE:
      case TYPE_INTERSECTION_OBS:
      case TYPE_AVOID_CONES:
      case TYPE_CURB_CUTOFF_CHANGE:
      case TYPE_AVOID_BUS_LANE:
      case TYPE_AVOID_MERGE_AREA:
        return false;
    }
    return false;
  }

  constexpr double kIgnoreSolidDrivingDistHighway = 350.0;       // m.
  constexpr double kIgnoreSolidContinuousLcDistHighway = 500.0;  // m.
  constexpr double kIgnoreSolidDrivingDistCity = 120.0;          // m.
  constexpr double kIgnoreSolidMergeDistHighWay = 80.0;          // m.
  constexpr double kIgnoreSolidMergeDistCity = 60.0;             // m.
  switch (lane_change_type) {
    case TYPE_EMERGENCY_CHANGE:
    case TYPE_PADDLE_CHANGE:
    case TYPE_OBSTACLE_CHANGE:
    case TYPE_CONSTRUCTION_ZONE_CHANGE:
    case TYPE_STALLED_VEHICLE_CHANGE:
    case TYPE_CURB_CUTOFF_CHANGE:
    case TYPE_AVOID_BUS_LANE:
      return true;
    case TYPE_DEFAULT_ROUTE_CHANGE:
      if (is_highway) {
        return length_along_route < (lc_num < 2
                                         ? kIgnoreSolidDrivingDistHighway
                                         : kIgnoreSolidContinuousLcDistHighway);
      } else {
        return length_along_route < kIgnoreSolidDrivingDistCity;
      }
      break;
    case TYPE_DEFAULT_CHANGE:
    case TYPE_NO_CHANGE:
    case TYPE_OVERTAKE_CHANGE:
    case TYPE_ROAD_SPEED_LIMIT_CHANGE:
    case TYPE_MAINROAD_EXIT_CHANGE:
    case TYPE_ENTER_MAINROAD_CHANGE:
      return false;
    case TYPE_MERGE_CHANGE:
      if (is_highway) {
        return lane_keep_dist_to_merge < kIgnoreSolidMergeDistHighWay;
      } else {
        return lane_keep_dist_to_merge < kIgnoreSolidMergeDistCity;
      }
    case TYPE_CENTER_CHANGE:
    case TYPE_INTERSECTION_OBS:
    case TYPE_AVOID_CONES:
    case TYPE_AVOID_MERGE_AREA:
      return false;
  }
  return false;
}

std::optional<int> FindMinValidLaneNumAtMost(const MppSectionInfo& mpp_section,
                                             double preview_length) {
  double accum_s = -mpp_section.start_s;
  if (mpp_section.section_lanes.size() != mpp_section.section_lengths.size()) {
    return std::nullopt;
  }
  int min_lc_num = INT_MAX;
  for (int i = 0; i < static_cast<int>(mpp_section.section_ids.size()); i++) {
    const auto& lanes = mpp_section.section_lanes[i];
    int valid_lc_num = 0;
    for (const auto& lane : lanes) {
      if (lane == nullptr || IsIgnoreLaneType(lane->type())) {
        continue;
      }

      ++valid_lc_num;
    }
    if (valid_lc_num == 0) {
      break;
    }
    min_lc_num = std::min(min_lc_num, valid_lc_num);
    accum_s += mpp_section.section_lengths[i];
    if (accum_s > preview_length) {
      break;
    }
  }
  if (min_lc_num == INT_MAX) {
    return std::nullopt;
  }
  return min_lc_num;
}

}  // namespace planning
}  // namespace st
