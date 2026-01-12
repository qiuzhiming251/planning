

#include "planner/speed_optimizer/st_overlap_analyzer.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "plan_common/log.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/circle2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_object_state.h"
// #include "semantic_map.pb.h"
#include "plan_common/maps/overlap_info.h"
#include "plan_common/maps/st_point.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/lane_point_util.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/perception_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st {
namespace planning {

namespace {

constexpr double kEps = 1e-6;
constexpr double kCurrPosMatchLaneInteractionDistThres = 4.0;  // m
constexpr double kLowSpeedThreshold = 2.0;                     // mps
constexpr double kOnComingAngleThreshold = M_PI * 7 / 8;
constexpr double kMaxBacktrackLength = 150.0;  // m
constexpr double kLaneCrossCutinAngleThres = M_PI / 8;
constexpr double kLaneCrossCutoutAngleThres = M_PI / 8;

using ad_byd::planning::Lane;
using ad_byd::planning::LaneInfo;
using ad_byd::planning::LaneMergeInfo;
using LaneGeometricConfig =
    ad_byd::planning::LaneInteraction::GeometricConfiguration;

struct LaneInteraction {
  double fraction;
  mapping::LanePoint other_lane_point;
  StOverlapMetaProto::OverlapPriority priority;
  LaneGeometricConfig geo_config;
  LaneType other_lane_type;

  LaneInteraction(double frac, mapping::LanePoint other_lane_pt,
                  StOverlapMetaProto::OverlapPriority prio,
                  LaneGeometricConfig geo_conf,
                  ad_byd::planning::LaneType other_lane_tp)
      : fraction(frac),
        other_lane_point(other_lane_pt),
        priority(prio),
        geo_config(geo_conf),
        other_lane_type(other_lane_tp) {}

  // static StOverlapMetaProto::OverlapPriority ReactionRuleToPriority(
  //     mapping::LaneInteractionProto::ReactionRule reaction_rule) {
  //   switch (reaction_rule) {
  //     case mapping::LaneInteractionProto::YIELD:
  //     case mapping::LaneInteractionProto::YIELD_ON_RED:
  //     case mapping::LaneInteractionProto::YIELD_ON_GREEN_CIRCLE:
  //     case mapping::LaneInteractionProto::YIELD_MERGE:
  //     case mapping::LaneInteractionProto::STOP: {
  //       return StOverlapMetaProto::LOW;
  //     }
  //     case mapping::LaneInteractionProto::PROCEED_MERGE:
  //     case mapping::LaneInteractionProto::PROCEED: {
  //       return StOverlapMetaProto::HIGH;
  //     }
  //     case mapping::LaneInteractionProto::FFA:
  //     case mapping::LaneInteractionProto::BOTH_STOP: {
  //       return StOverlapMetaProto::EQUAL;
  //     }
  //     default:
  //       throw std::runtime_error("switch case on enum unexpected");
  //   }
  // }
};

using LaneInteractionMap =
    absl::flat_hash_map<mapping::ElementId, std::vector<LaneInteraction>>;

// Only run overlap analyzer for this object type of st boundaries.
bool RunStOverlapAnalzyerByStBoundaryObjectType(
    StBoundaryProto::ObjectType object_type) {
  switch (object_type) {
    case StBoundaryProto::VEHICLE:
    case StBoundaryProto::CYCLIST:
    case StBoundaryProto::PEDESTRIAN:
      return true;
    case StBoundaryProto::STATIC:
    case StBoundaryProto::UNKNOWN_OBJECT:
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::VIRTUAL:
    case StBoundaryProto::IMPASSABLE_BOUNDARY:
    case StBoundaryProto::PATH_BOUNDARY:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

// Only run overlap analyzer for this source type of st boundaries.
bool RunStOverlapnalyzerByStBoundarySourceType(
    StBoundarySourceTypeProto::Type source) {
  switch (source) {
    case StBoundarySourceTypeProto::ST_OBJECT:
      return true;
    case StBoundarySourceTypeProto::UNKNOWN:
    case StBoundarySourceTypeProto::VIRTUAL:
    case StBoundarySourceTypeProto::IMPASSABLE_BOUNDARY:
    case StBoundarySourceTypeProto::PATH_BOUNDARY:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

StOverlapMetaProto::OverlapPriority ComputeMergingPriority(
    const Lane& ego_lane, const Lane& other_lane) {
  const auto is_merging = [](const LaneMergeInfo& lane_merge_info) {
    constexpr double kMergeForwardDist = 150.0;  // m.
    return lane_merge_info.valid &&
           lane_merge_info.dist_to_merge < kMergeForwardDist;
  };
  const LaneInfo& ego_lane_info = ego_lane.lane_info();
  const LaneInfo& other_lane_info = other_lane.lane_info();
  bool is_other_lane_ramp = other_lane_info.type == LaneType::LANE_RAMP;
  if (!is_other_lane_ramp && is_merging(ego_lane_info.lane_merge_info) &&
      !is_merging(other_lane_info.lane_merge_info)) {
    return StOverlapMetaProto::LOW;
  } else if (!is_merging(ego_lane_info.lane_merge_info) &&
             is_merging(other_lane_info.lane_merge_info)) {
    return StOverlapMetaProto::HIGH;
  }
  return StOverlapMetaProto::EQUAL;
}

LaneInteractionMap GetLaneInteractionMap(
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm) {
  LaneInteractionMap path_lane_interaction_map;
  for (const PathPointSemantic& path_semantic : path_semantics) {
    const auto& closest_lane_point = path_semantic.closest_lane_point;
    if (!closest_lane_point.Valid()) {
      continue;
    }
    if (path_lane_interaction_map.contains(closest_lane_point.lane_id())) {
      continue;
    }
    const auto lane_ptr = psmm.FindLaneByIdOrNull(closest_lane_point.lane_id());
    if (nullptr == lane_ptr) {
      continue;
    }
    const Lane& lane = *lane_ptr;
    path_lane_interaction_map[closest_lane_point.lane_id()] = {};
    // Tips: Do not consider lane merge in junction.
    if (!(lane.lane_info().junction_id == 0)) {
      continue;
    }
    for (const auto& next_lane_id : lane.next_lane_ids()) {
      const auto next_lane_ptr = psmm.FindLaneByIdOrNull(next_lane_id);
      if (nullptr == next_lane_ptr || !next_lane_ptr->IsValid()) {
        continue;
      }
      for (const auto& pre_lane_id : next_lane_ptr->pre_lane_ids()) {
        if (closest_lane_point.lane_id() == pre_lane_id) {
          continue;
        }
        const auto pre_lane_ptr = psmm.FindLaneByIdOrNull(pre_lane_id);
        if (nullptr == pre_lane_ptr || !pre_lane_ptr->IsValid()) {
          continue;
        }
        const Lane& pre_lane = *pre_lane_ptr;
        path_lane_interaction_map[closest_lane_point.lane_id()].emplace_back(
            /*fraction*/ 1.0, mapping::LanePoint(pre_lane_id, 1.0),
            ComputeMergingPriority(lane, pre_lane), LaneGeometricConfig::MERGE,
            pre_lane.type());
      }
    }
  }
  return path_lane_interaction_map;
}

void BacktrackPreLanes(const PlannerSemanticMapManager& psmm,
                       const ad_byd::planning::LaneConstPtr& current_lane,
                       double accumulated_length,
                       std::set<mapping::ElementId>* pre_lanes_set) {
  if (current_lane == nullptr) return;

  // check befor insert：
  // 1. exist in map
  // 2. locate beyond max backtrack dist
  if (!pre_lanes_set->insert(current_lane->id()).second ||
      accumulated_length + current_lane->curve_length() > kMaxBacktrackLength) {
    return;
  }

  accumulated_length += current_lane->curve_length();

  // backtrack pre lanes
  for (const auto& pre_lane_id : current_lane->pre_lane_ids()) {
    const auto prev_lane = psmm.FindLaneByIdOrNull(pre_lane_id);

    BacktrackPreLanes(psmm, prev_lane, accumulated_length, pre_lanes_set);
  }
}

// Ensure that the subsequent lane of the target lane does not pass by the
// self driving vehicle
bool VerifyObjLaneWithAVLane(
    const PlannerSemanticMapManager& psmm,
    const mapping::ElementId& current_av_lane_id,
    const ad_byd::planning::LaneConstPtr& obj_lane,
    const std::set<mapping::ElementId>& pre_lanes_set) {
  if (obj_lane == nullptr) return false;

  if (pre_lanes_set.find(obj_lane->id()) != pre_lanes_set.end()) {
    return false;
  }

  if (obj_lane->id() == current_av_lane_id) {
    return true;
  }

  for (const auto& next_lane_id : obj_lane->next_lane_ids()) {
    const auto next_lane = psmm.FindLaneByIdOrNull(next_lane_id);
    if (VerifyObjLaneWithAVLane(psmm, current_av_lane_id, next_lane,
                                pre_lanes_set)) {
      return true;
    }
  }

  return false;
}

bool MatchOverlapWithLaneInteraction(
    const StBoundaryProto::ObjectType object_type, const Vec2d& current_obj_pos,
    const Vec2d& first_overlap_obj_pos, double first_overlap_obj_heading,
    const mapping::ElementId& current_av_lane_id,
    const LaneInteraction& lane_interaction,
    const PlannerSemanticMapManager& psmm, const StBoundaryRef& st_boundary) {
  constexpr double kBacktrackObjTimeThres = 7.0;          // s
  constexpr double kBacktrackEgoDistThres = 150.0;        // m
  constexpr double kMatchLaneInteractionDistThres = 2.0;  // m.
  constexpr double kMatchLaneInteractionHeadingThres = M_PI / 4.0;

  CHECK(object_type == StBoundaryProto::VEHICLE ||
        object_type == StBoundaryProto::CYCLIST);

  const auto& bottom_left_point = st_boundary->bottom_left_point();
  // shorten backtrack distance
  if (bottom_left_point.t() > kBacktrackObjTimeThres ||
      bottom_left_point.s() > kBacktrackEgoDistThres) {
    return false;
  }
  std::set<mapping::ElementId> pre_lanes_set;
  ad_byd::planning::LaneConstPtr obj_lane;

  const auto other_lane_id = lane_interaction.other_lane_point.lane_id();
  const auto other_lane = psmm.FindLaneByIdOrNull(other_lane_id);
  if (other_lane == nullptr) {
    return false;
  }

  BacktrackPreLanes(psmm, other_lane, 0.0, &pre_lanes_set);

  for (const auto& lane_id : pre_lanes_set) {
    const auto lane = psmm.FindLaneByIdOrNull(lane_id);
    if (lane == nullptr) continue;

    double fraction = 0.0;
    double min_dist = 0.0;
    if (psmm.GetLaneProjection(current_obj_pos, lane->id(), &fraction, nullptr,
                               &min_dist)) {
      if (fraction > 0.0 && fraction < 1.0 &&
          min_dist < kCurrPosMatchLaneInteractionDistThres) {
        obj_lane = lane;
        break;
      }
    }
  }

  if (obj_lane == nullptr ||
      pre_lanes_set.find(obj_lane->id()) == pre_lanes_set.end()) {
    return false;
  }

  // Check heading diff.
  double fraction_fo = 0.0;
  if (!psmm.GetLaneProjection(first_overlap_obj_pos,
                              lane_interaction.other_lane_point.lane_id(),
                              &fraction_fo,
                              /*point=*/nullptr, /*min_dist=*/nullptr)) {
    return false;
  }

  const mapping::LanePoint closest_other_lane_point(
      lane_interaction.other_lane_point.lane_id(), fraction_fo);
  const double closest_other_lane_point_heading =
      ComputeLanePointLerpTheta(psmm, closest_other_lane_point);

  const double heading_diff = std::abs(NormalizeAngle(
      first_overlap_obj_heading - closest_other_lane_point_heading));

  if (heading_diff >= kMatchLaneInteractionHeadingThres) {
    // failed to matched with current lane interaction.
    return false;
  }

  if (obj_lane->id() == other_lane_id) {
    return true;
  }

  return !VerifyObjLaneWithAVLane(psmm, current_av_lane_id, obj_lane,
                                  pre_lanes_set);
}

//  If drive passage lane is the current lane the AV is driving on,
//    use std::greater<double>() to determine if AV is out of the current
//    lane; If drive passage lane is the target lane for lane change, use
// std::less<double>() to determine if AV invades the target lane.
bool IsOverlapAreaOutOfCurrentLane(
    const SpacetimeObjectTrajectory& st_traj, const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, int av_idx, int obj_idx,
    bool is_current_lane) {
  std::function<bool(double, double)> comp;
  if (is_current_lane) {
    comp = std::greater<double>();
  } else {
    comp = std::less<double>();
  }
  const auto& obj_contour = st_traj.states()[obj_idx].contour;
  // NOTE: Buffer may be time-varying in the future.
  const double buffer = st_traj.required_lateral_gap();

  const PiecewiseLinearFunction<double> time_boundary_buffer_plf(
      {0.0, 5.0, 7.0, 10.0}, {0.0, 0.0, 0.2, 0.5});
  const double boundary_buffer =
      time_boundary_buffer_plf(st_traj.states()[obj_idx].traj_point->t());

  const auto& av_shape_ptr = av_shapes[av_idx];

  const auto is_polygon_in_lane = [&drive_passage, &comp](
                                      const Polygon2d& polygon,
                                      double boundary_buffer) -> bool {
    const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(polygon);
    if (frenet_box_or.ok()) {
      const auto s_min_boundary =
          drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_box_or->s_min);
      const auto s_max_boundary =
          drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_box_or->s_max);
      // Check if polygon is entirely in drive passage lane.
      return (s_min_boundary.left.has_value() &&
              s_max_boundary.left.has_value() &&
              s_min_boundary.right.has_value() &&
              s_max_boundary.right.has_value() &&
              comp(s_min_boundary.left->lat_offset + boundary_buffer,
                   frenet_box_or->l_max) &&
              comp(s_max_boundary.left->lat_offset + boundary_buffer,
                   frenet_box_or->l_max) &&
              comp(frenet_box_or->l_min,
                   s_min_boundary.right->lat_offset - boundary_buffer) &&
              comp(frenet_box_or->l_min,
                   s_max_boundary.right->lat_offset - boundary_buffer));
    }
    return true;
  };

  const auto [min_mirror_height, max_mirror_height] =
      ComputeMinMaxMirrorAverageHeight(vehicle_params);
  const bool consider_mirrors =
      IsConsiderMirrorObject(st_traj.planner_object().object_proto(),
                             min_mirror_height, max_mirror_height);
  if (consider_mirrors) {
    const auto is_circle_in_lane =
        [&drive_passage, &comp](const Circle2d& circle, double buffer) -> bool {
      const auto frenet_pt_or =
          drive_passage.QueryFrenetCoordinateAt(circle.center());
      if (frenet_pt_or.ok()) {
        const auto s_boundary =
            drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_pt_or->s);
        const double buffered_radius = circle.radius() + buffer;
        // Check if circle is entirely in drive passage lane.
        return (s_boundary.right.has_value() && s_boundary.left.has_value() &&
                comp(frenet_pt_or->l - buffered_radius,
                     s_boundary.right->lat_offset) &&
                comp(s_boundary.left->lat_offset,
                     frenet_pt_or->l + buffered_radius));
      }
      return true;
    };
    const double mirror_buffer = buffer - boundary_buffer;
    if ((av_shape_ptr->LeftMirrorHasOverlapWithBuffer(obj_contour, buffer) &&
         !is_circle_in_lane(*av_shape_ptr->left_mirror(), mirror_buffer)) ||
        (av_shape_ptr->RightMirrorHasOverlapWithBuffer(obj_contour, buffer) &&
         !is_circle_in_lane(*av_shape_ptr->right_mirror(), mirror_buffer))) {
      return true;
    }
  }

  Polygon2d overlap_polygon;
  const Polygon2d av_polygon =
      Polygon2d(av_shape_ptr->GetCornersWithBufferCounterClockwise(
                    /*lat_buffer=*/buffer, /*lon_buffer=*/buffer),
                /*is_convex=*/true);
  if (obj_contour.ComputeOverlap(av_polygon, &overlap_polygon) &&
      !is_polygon_in_lane(overlap_polygon, boundary_buffer)) {
    return true;
  }

  return false;
}

bool ProcessIntersection(
    const StBoundaryRef& st_boundary,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& path,
    const std::vector<mapping::ElementId>& fo_av_lane_id_list,
    const OverlapInfo& fo_info, int av_mean_idx, const LaneChangeStage lc_stage,
    OverlapSourcePriority* res) {
  CHECK_NOTNULL(res);
  // Check if AV is going straight
  bool fo_av_going_straight = false;
  for (const auto lane_id : fo_av_lane_id_list) {
    const auto lane_info_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
    if (nullptr == lane_info_ptr) continue;
    const auto& lane_info = *lane_info_ptr;
    if (lane_info.turn_type() == ad_byd::planning::TurnType::NO_TURN) {
      fo_av_going_straight = true;
      break;
    }
  }

  // Check if AV is going straight and cut in by a u-turn like object
  // trajectory.
  const double init_obj_heading = st_traj.states().front().traj_point->theta();
  const double end_obj_heading = st_traj.states().back().traj_point->theta();
  constexpr double kUTurnHeadingDiff = 0.75 * M_PI;  // 135 deg.
  res->is_making_u_turn =
      (fo_av_going_straight &&
       std::abs(NormalizeAngle(end_obj_heading - init_obj_heading)) >=
           kUTurnHeadingDiff);
  // If AV is crossed by other objects while going straight in lane, AV has a
  // higher priority to pass.
  const auto get_angle_diff = [&path, &st_traj](int av_path_idx,
                                                int obj_traj_idx) -> double {
    const auto& obj_traj_point = *st_traj.states()[obj_traj_idx].traj_point;
    const auto& av_path_point = path[av_path_idx];
    return NormalizeAngle(obj_traj_point.theta() - av_path_point.theta());
  };
  if (av_mean_idx >= path.size() || av_mean_idx >= path_semantics.size()) {
    DLOG(ERROR) << "[speed finder] av_mean_idx error " << av_mean_idx << " | "
                << path.size() << " | " << path_semantics.size();
    return false;
  }

  const double theta_diff = get_angle_diff(av_mean_idx, fo_info.obj_idx);
  constexpr double kCrossingAngleDiffThre = M_PI_4;  // 45 deg.
  res->is_crossing_straight_lane =
      (fo_av_going_straight &&
       st_boundary->object_type() == StBoundaryProto::VEHICLE &&
       std::abs(theta_diff) > kCrossingAngleDiffThre);
  VLOG(2) << "[speed finder] st id: " << st_boundary->id()
          << " |is_making_u_turn "
          << static_cast<int>(res->is_making_u_turn.has_value() &&
                              res->is_making_u_turn.value())
          << " |fo_av_going_straight " << fo_av_going_straight
          << " |is_crossing_straight_lane "
          << static_cast<int>(res->is_crossing_straight_lane.has_value() &&
                              res->is_crossing_straight_lane.value());

  const ad_byd::planning::LaneConstPtr lane_info =
      path_semantics[av_mean_idx].lane_info;
  if (lane_info == nullptr) {
    return false;
  }
  if (!lane_info->IsVirtual()) {
    VLOG(2) << "[speed finder] st id: " << st_boundary->id()
            << " not in intersection, lane_id" << lane_info->id();
    return false;
  }

  const auto& overlap_infos = st_boundary->overlap_infos();
  const auto& last_overlap_info = overlap_infos.back();
  const auto& first_overlap_info = overlap_infos.front();

  const double last_overlap_obj_heading =
      st_traj.states()[last_overlap_info.obj_idx].box.heading();
  const double last_overlap_ego_heading =
      path[(last_overlap_info.av_start_idx + last_overlap_info.av_end_idx) / 2]
          .theta();
  const double first_overlap_obj_heading =
      st_traj.states()[first_overlap_info.obj_idx].box.heading();
  const double first_overlap_ego_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();

  const double cut_in_angle = std::abs(
      NormalizeAngle(first_overlap_obj_heading - first_overlap_ego_heading));
  const double cut_out_angle = std::abs(
      NormalizeAngle(last_overlap_obj_heading - last_overlap_ego_heading));

  bool is_protected = false;
  if (lane_info->light_status() != ad_byd::planning::LightStatus::NONE_LIGHT) {
    is_protected = true;
  }
  switch (path_semantics[av_mean_idx].lane_semantic) {
    case LaneSemantic::INTERSECTION_LEFT_TURN: {
      if (cut_in_angle > kLaneCrossCutinAngleThres &&
          cut_in_angle < kOnComingAngleThreshold &&
          cut_out_angle > kLaneCrossCutoutAngleThres &&
          cut_out_angle < kOnComingAngleThreshold) {
        if (is_protected &&
            st_boundary->object_type() == StBoundaryProto::VEHICLE) {
          res->source = StOverlapMetaProto::LANE_CROSS;
          res->priority = StOverlapMetaProto::EQUAL;
          res->priority_reason = "EQUAL priority for AV protected left turn";
          res->is_unprotected_left_turn = false;
        } else {
          res->source = StOverlapMetaProto::LANE_CROSS;
          res->priority = StOverlapMetaProto::LOW;
          res->priority_reason = "LOW priority for AV unprotected left turn";
          res->is_unprotected_left_turn = true;
        }
        return true;
      } else {
        return false;
      }
    }
    case LaneSemantic::INTERSECTION_RIGHT_TURN: {
      res->priority = StOverlapMetaProto::LOW;
      if (st_boundary->object_type() == StBoundaryProto::VEHICLE) {
        res->source = StOverlapMetaProto::LANE_MERGE;
      }
      res->priority_reason = "LOW priority for AV right turn";
      return true;
    }
    case LaneSemantic::INTERSECTION_STRAIGHT: {
      if (cut_in_angle > kLaneCrossCutinAngleThres &&
          cut_in_angle < kOnComingAngleThreshold &&
          cut_out_angle > kLaneCrossCutoutAngleThres &&
          cut_out_angle < kOnComingAngleThreshold) {
        if (st_boundary->object_type() == StBoundaryProto::VEHICLE) {
          res->priority = StOverlapMetaProto::HIGH;
          res->priority_reason = "HIGH priority for AV intersection straight";
          res->source = StOverlapMetaProto::LANE_CROSS;
        } else {
          res->priority = StOverlapMetaProto::LOW;
          res->priority_reason = "LOW priority for for AV to cyclist";
          res->source = StOverlapMetaProto::LANE_CROSS;
        }
        return true;
      } else {
        return false;
      }
    }
    case LaneSemantic::INTERSECTION_UTURN:
    case LaneSemantic::ROAD:
    case LaneSemantic::NONE:
      VLOG(2) << "[speed finder] id: " << st_boundary->id()
              << " ego lane_semantic:"
              << static_cast<int>(path_semantics[av_mean_idx].lane_semantic);
      break;
  }
  return false;
}

absl::StatusOr<OverlapSourcePriority> AnalyzeOverlapSourceAndPriority(
    const StBoundaryRef& st_boundary,
    StOverlapMetaProto::OverlapPattern overlap_pattern,
    const SpacetimeObjectTrajectory& st_traj,
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const DrivePassage& drive_passage,
    const LaneInteractionMap& lane_interaction_map,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const LaneChangeStage& lc_stage,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v) {
  // VLOG(4) << "Analyze overlap source and priority for st-boundary "
  //         << st_boundary->id();

  OverlapSourcePriority res;
  // Only analyze source and priority for overlap of pattern ENTER, CROSS
  // and INTERFERE.
  if (overlap_pattern != StOverlapMetaProto::ENTER &&
      overlap_pattern != StOverlapMetaProto::CROSS &&
      overlap_pattern != StOverlapMetaProto::INTERFERE) {
    return res;
  }

  // Only analyze source and priority for overlap happening at a future
  // path point.
  if (st_boundary->bottom_left_point().s() <= 0.0) {
    return res;
  }

  // Low priority for all pedestrian overlaps.
  if (st_boundary->object_type() == StBoundaryProto::PEDESTRIAN) {
    res.source = StOverlapMetaProto::OTHER;
    res.priority = StOverlapMetaProto::LOW;
    res.priority_reason = absl::StrCat(
        "LOW priority for object type ",
        StBoundaryProto::ObjectType_Name(st_boundary->object_type()));
    return res;
  }

  const auto& overlap_infos = st_boundary->overlap_infos();
  // In this function, 'fo' denotes 'first_overlap'.
  const auto& fo_info = overlap_infos.front();

  // NOLINTNEXTLINE
  enum class LaneChangeSemantic { NONE = 0, LEFT = 1, RIGHT = 2 };
  LaneChangeSemantic fo_av_lane_change_semantic = LaneChangeSemantic::NONE;
  std::vector<mapping::ElementId> fo_av_lane_id_list;
  constexpr double kLaneChangeCheckLookAheadDist = 20.0;  // m.
  // Check whether AV is changing lane during the first overlap up to a
  // lookahead distance. This is to compensate for the fact the AV front
  // wheels would enter target lane before its rac point.
  std::optional<std::vector<int>> lc_lane_path_id_history = std::nullopt;
  const double check_lc_end_path_s =
      path[fo_info.av_end_idx].s() + kLaneChangeCheckLookAheadDist;
  for (int i = fo_info.av_start_idx;
       i < path_semantics.size() && path[i].s() <= check_lc_end_path_s; ++i) {
    const auto& lane_path_id_history = path_semantics[i].lane_path_id_history;
    if (fo_av_lane_change_semantic == LaneChangeSemantic::NONE &&
        lane_path_id_history.back() != 0) {
      CHECK_GT(lane_path_id_history.size(), 1);
      CHECK_NE(lane_path_id_history.back(),
               lane_path_id_history[lane_path_id_history.size() - 2]);
      if (lane_path_id_history.back() >
          lane_path_id_history[lane_path_id_history.size() - 2]) {
        fo_av_lane_change_semantic = LaneChangeSemantic::LEFT;
      } else {
        fo_av_lane_change_semantic = LaneChangeSemantic::RIGHT;
      }
      lc_lane_path_id_history = lane_path_id_history;
    }
    const auto& closest_lane_point = path_semantics[i].closest_lane_point;
    if (i <= fo_info.av_end_idx && closest_lane_point.Valid() &&
        std::find(fo_av_lane_id_list.begin(), fo_av_lane_id_list.end(),
                  closest_lane_point.lane_id()) == fo_av_lane_id_list.end()) {
      fo_av_lane_id_list.push_back(closest_lane_point.lane_id());
    }
  }
  // Compute the overlap area of AV & object when overlap for 1st time.
  // Then check whether the overlap area is out of current lane. If so, we
  // think AV is changing lane and cut-in this object.
  if (fo_info.av_end_idx >= path_semantics.size()) {
    return absl::NotFoundError("Overlap index exceeds path semantics range!");
  }

  struct DeviationInfo {
    int av_idx = 0;
    int obj_idx = 0;
    double deviation_distance = 0.0;
    DeviationInfo(int av_idx, int obj_idx, double deviation_distance)
        : av_idx(av_idx),
          obj_idx(obj_idx),
          deviation_distance(deviation_distance) {}
  };
  std::optional<DeviationInfo> max_deviation_info = std::nullopt;
  for (const auto& overlap_info : overlap_infos) {
    const int av_idx =
        (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;
    if (av_idx >= path_semantics.size() ||
        !path_semantics[av_idx].closest_lane_point.Valid() ||
        path_semantics[av_idx].lane_path_id_history.size() > 1) {
      continue;
    }
    constexpr double kFractionFilterThres = 0.99;
    if (path_semantics[av_idx].closest_lane_point.fraction() >
        kFractionFilterThres) {
      continue;
    }
    CHECK_NOTNULL(path_semantics[av_idx].lane_info);
    if (path_semantics[av_idx].lane_info->IsVirtual()) {
      const auto& nearest_station =
          drive_passage.FindNearestStation(ToVec2d(path[av_idx]));
      if (nearest_station.lane_id() == path_semantics[av_idx].lane_info->id() ||
          nearest_station.is_virtual()) {
        // Both current lane & target lane are virtual.
        continue;
      }
    }
    if (!max_deviation_info.has_value() ||
        path_semantics[av_idx].deviation_distance >
            max_deviation_info->deviation_distance) {
      max_deviation_info =
          DeviationInfo(av_idx, overlap_info.obj_idx,
                        path_semantics[av_idx].deviation_distance);
    }
  }

  if (max_deviation_info.has_value()) {
    const auto av_idx = max_deviation_info->av_idx;
    const auto obj_idx = max_deviation_info->obj_idx;
    const auto current_lane_id = path_semantics[av_idx].lane_info->id();
    const auto& dp_lane_ids = drive_passage.extend_lane_path().lane_ids();
    const bool is_current_lane =
        std::find(dp_lane_ids.begin(), dp_lane_ids.end(), current_lane_id) !=
        dp_lane_ids.end();
    if (IsOverlapAreaOutOfCurrentLane(st_traj, drive_passage,
                                      vehicle_geometry_params, av_shapes,
                                      av_idx, obj_idx, is_current_lane)) {
      const auto& av_pos = path[0];
      auto av_lane_id = drive_passage.lane_path().lane_ids().front();
      for (const auto& lane_id : drive_passage.lane_path().lane_ids()) {
        double start_fraction = 0.0;
        double end_fraction = 1.0;
        const auto& lane = psmm.FindLaneByIdOrNull(lane_id);
        if (!lane) continue;

        // find first av position in lane path
        const auto ff = BuildBruteForceFrenetFrame(
            lane->points(), /*down_sample_raw_points=*/false);
        if (!ff.ok()) continue;
        FrenetCoordinate sl;
        Vec2d normal;
        std::pair<int, int> index_pair;
        double alpha;
        ff.value().XYToSL({av_pos.x(), av_pos.y()}, &sl, &normal, &index_pair,
                          &alpha);
        start_fraction = sl.s / std::fmax(lane->curve_length(), 1e-2);
        if (start_fraction < 0.0 - kEps || start_fraction > 1.0 - kEps) {
          continue;
        } else {
          av_lane_id = lane_id;
        }
      }
      const auto& nearest_lane = psmm.FindLaneByIdOrNull(av_lane_id);

      if (nearest_lane && nearest_lane->center_line().IsValid() &&
          nearest_lane->type() == ad_byd::planning::LANE_VIRTUAL_JUNCTION) {
        bool find_start = false;
        constexpr double kSpeedDiffThreshold = 10.0;
        Vec2d ref_point = nearest_lane->points().back();
        for (const auto& seg : drive_passage.lane_path()) {
          if (seg.lane_id == av_lane_id || find_start) {
            find_start = true;
            const auto& lane = psmm.FindLaneByIdOrNull(seg.lane_id);
            if (!lane) break;
            if (lane->type() != ad_byd::planning::LANE_VIRTUAL_JUNCTION) {
              ref_point = lane->points().front();
              break;
            } else {
              ref_point = lane->points().back();
            }
          } else {
            continue;
          }
        }
        const auto& lane_theta = drive_passage.QueryTangentAt(ref_point);
        bool smaller_theta_diff = false;
        if (lane_theta.ok()) {
          const double av_theta_diff =
              NormalizeAngle(path[av_idx].theta() - lane_theta->Angle());
          const double obj_theta_diff =
              NormalizeAngle(st_traj.states()[obj_idx].traj_point->theta() -
                             lane_theta->Angle());
          const bool av_valid = std::abs(av_theta_diff) > kEps;
          const bool obj_valid = std::abs(obj_theta_diff) > kEps;
          if (nearest_lane->turn_type() == ad_byd::planning::LEFT_TURN) {
            smaller_theta_diff =
                av_valid && obj_valid && av_theta_diff < obj_theta_diff;
          } else if (nearest_lane->turn_type() ==
                     ad_byd::planning::RIGHT_TURN) {
            smaller_theta_diff =
                av_valid && obj_valid && av_theta_diff > obj_theta_diff;
          }
        }
        if (st_traj.states().front().traj_point->v() - init_v >
                Kph2Mps(kSpeedDiffThreshold) ||
            !smaller_theta_diff) {
          res.source = StOverlapMetaProto::AV_CUTIN;
          res.priority = StOverlapMetaProto::LOW;
          res.priority_reason =
              "LOW priority for AV cutting in object (overlap method)";
          return res;
        }
      } else {
        res.source = StOverlapMetaProto::AV_CUTIN;
        res.priority = StOverlapMetaProto::LOW;
        res.priority_reason =
            "LOW priority for AV cutting in object (overlap method)";
        return res;
      }
    }
  }

  const auto* fo_obj_point = st_traj.states()[fo_info.obj_idx].traj_point;
  const auto& fo_obj_pos = fo_obj_point->pos();
  const int av_mean_idx = (fo_info.av_start_idx + fo_info.av_end_idx) / 2;
  if (fo_av_lane_change_semantic != LaneChangeSemantic::NONE) {
    // The overlap happens during AV lane change. Check if the object is
    // on the corresponding side of path to see if it is being cut in by
    // AV.
    const auto& fo_av_path_point = path[av_mean_idx];
    const bool is_fo_obj_on_path_left =
        Vec2d::FastUnitFromAngle(fo_av_path_point.theta())
            .CrossProd(fo_obj_pos - ToVec2d(fo_av_path_point)) >= 0.0;
    if ((fo_av_lane_change_semantic == LaneChangeSemantic::LEFT &&
         is_fo_obj_on_path_left) ||
        (fo_av_lane_change_semantic == LaneChangeSemantic::RIGHT &&
         !is_fo_obj_on_path_left)) {
      res.source = StOverlapMetaProto::AV_CUTIN;
      res.priority = StOverlapMetaProto::LOW;
      res.priority_reason = "LOW priority for AV cutting in object";
      CHECK(lc_lane_path_id_history.has_value());
      for (int i = 0; i < path_semantics.size(); ++i) {
        if (path_semantics[i].lane_path_id_history ==
            *lc_lane_path_id_history) {
          res.time_to_lc_complete = path[i].s() / (init_v + 1e-6);
          break;
        }
      }
      CHECK(res.time_to_lc_complete.has_value());
      return res;
    }
  } else {
    // The overlap happens during AV lane keeping.
    // const auto fo_obj_heading = fo_obj_point->theta();
    const auto fo_obj_heading = fo_obj_point->theta();
    const auto current_obj_pos = st_traj.pose().pos();
    const auto current_av_lane_id =
        path_semantics[0].closest_lane_point.lane_id();

    // Check if first overlap can be matched to a lane interaction.
    for (const auto lane_id : fo_av_lane_id_list) {
      const auto* lane_interactions = FindOrNull(lane_interaction_map, lane_id);
      if (lane_interactions == nullptr) continue;
      for (const auto& lane_interaction : *lane_interactions) {
        if (MatchOverlapWithLaneInteraction(
                st_boundary->object_type(), current_obj_pos, fo_obj_pos,
                fo_obj_heading, current_av_lane_id, lane_interaction, psmm,
                st_boundary)) {
          const auto lane_info_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
          const auto other_lane_info_ptr = psmm.FindCurveLaneByIdOrNull(
              lane_interaction.other_lane_point.lane_id());
          if (nullptr == lane_info_ptr || nullptr == other_lane_info_ptr) {
            continue;
          }
          const auto& lane_info = *lane_info_ptr;
          const auto& other_lane_info = *other_lane_info_ptr;
          if (lane_interaction.geo_config == LaneGeometricConfig::MERGE) {
            res.source = StOverlapMetaProto::LANE_MERGE;
            res.priority = lane_interaction.priority;
            res.priority_reason = absl::StrCat(
                StOverlapMetaProto::OverlapPriority_Name(
                    lane_interaction.priority),
                " priority for AV lane ", lane_id, " merging object lane ",
                lane_interaction.other_lane_point.lane_id());
            res.is_merging_straight_lane =
                (lane_info.turn_type() == ad_byd::planning::TurnType::NO_TURN &&
                 lane_interaction.priority == StOverlapMetaProto::HIGH);
          } else {
            res.source = StOverlapMetaProto::LANE_CROSS;
            res.priority = lane_interaction.priority;
            res.priority_reason = absl::StrCat(
                StOverlapMetaProto::OverlapPriority_Name(
                    lane_interaction.priority),
                " priority for AV lane ", lane_id, " crossing object lane ",
                lane_interaction.other_lane_point.lane_id());
            // If AV is going straight, we can safely assume that any
            // left-turn prediction cutting in AV path is unprotected.
            res.is_crossing_straight_lane =
                (lane_info.turn_type() == ad_byd::planning::TurnType::NO_TURN &&
                 lane_interaction.priority == StOverlapMetaProto::HIGH);
          }
          res.is_making_u_turn =
              (lane_info.turn_type() == ad_byd::planning::TurnType::NO_TURN &&
               other_lane_info.turn_type() ==
                   ad_byd::planning::TurnType::U_TURN);
          res.obj_lane_direction = other_lane_info.turn_type();
          // TODO: Consider to match the most likely interaction
          // instead of early exit.
          return res;
        }
      }
    }
  }

  // handle intersection
  if (ProcessIntersection(st_boundary, path_semantics, psmm, st_traj, path,
                          fo_av_lane_id_list, fo_info, av_mean_idx, lc_stage,
                          &res)) {
    return res;
  }

  // Other cases (AV is changing lane but object is not cut in by AV, or
  // AV is keeping lane but the overlap is not matched to a lane
  // interaction), we consider the object is cutting AV.
  res.source = StOverlapMetaProto::OBJECT_CUTIN;
  res.priority = StOverlapMetaProto::HIGH;
  res.priority_reason = "HIGH priority for AV being cut in by object";
  return res;
}

bool IsDrivingParallel(const std::optional<double>& theta_diff) {
  if (!theta_diff.has_value()) return false;
  constexpr double kDrivingParallelHeadingThres = M_PI / 6.0;
  return std::abs(*theta_diff) < kDrivingParallelHeadingThres;
}

std::optional<double> CalculateThetaDiff(
    const SpacetimeObjectTrajectory& st_traj, const DrivePassage& drive_passage,
    const std::optional<KdTreeFrenetFrame>& target_frenet_frame) {
  // If object is too far away from frenet frame, theta calculation is
  // inaccurate.
  constexpr double kMaxLLimit = 10.0;  // m.
  const Vec2d& obj_pos = st_traj.pose().pos();
  if (target_frenet_frame.has_value()) {
    const auto obj_frenet_coord = target_frenet_frame->XYToSL(obj_pos);
    if (std::abs(obj_frenet_coord.l) > kMaxLLimit) {
      return std::nullopt;
    }
    const auto obj_frenet_tangent =
        target_frenet_frame->InterpolateTangentByS(obj_frenet_coord.s);
    return NormalizeAngle(st_traj.pose().theta() -
                          obj_frenet_tangent.FastAngle());
  } else {
    const auto obj_frenet_coord =
        drive_passage.QueryFrenetCoordinateAt(obj_pos);
    if (obj_frenet_coord.ok()) {
      if (std::abs(obj_frenet_coord->l) > kMaxLLimit) {
        return std::nullopt;
      }
      const auto obj_frenet_theta =
          drive_passage.QueryTangentAngleAtS(obj_frenet_coord->s);
      if (obj_frenet_theta.ok()) {
        return NormalizeAngle(st_traj.pose().theta() - *obj_frenet_theta);
      }
    }
  }
  return std::nullopt;
}

std::pair<double, double> CalculateProjectionDistance(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& path) {
  const auto& obj_contour = st_traj.contour();

  const Vec2d av_heading_dir = Vec2d::FastUnitFromAngle(path.front().theta());
  const Vec2d av_pos = Vec2d(path.front().x(), path.front().y());
  Vec2d front_most, back_most;
  obj_contour.ExtremePoints(av_heading_dir, &back_most, &front_most);

  const double front_dis = (front_most - av_pos).Dot(av_heading_dir);
  const double back_dis = (back_most - av_pos).Dot(av_heading_dir);
  return std::make_pair(front_dis, back_dis);
}

StOverlapMetaProto::ModificationType AnalyzeOverlapModificationType(
    const StBoundaryRef& st_boundary, const std::optional<double>& theta_diff,
    StOverlapMetaProto::OverlapSource overlap_source,
    StOverlapMetaProto::OverlapPriority overlap_priority) {
  if (overlap_priority == StOverlapMetaProto::UNKNOWN_PRIORITY) {
    // An overlap without priority must be non-interactive.
    return StOverlapMetaProto::NON_MODIFIABLE;
  }
  CHECK(st_boundary->object_type() == StBoundaryProto::VEHICLE ||
        st_boundary->object_type() == StBoundaryProto::CYCLIST ||
        st_boundary->object_type() == StBoundaryProto::PEDESTRIAN);

  // For vehicles/cyclists.
  if (st_boundary->object_type() == StBoundaryProto::VEHICLE ||
      st_boundary->object_type() == StBoundaryProto::CYCLIST) {
    switch (overlap_source) {
      case StOverlapMetaProto::LANE_MERGE:
      case StOverlapMetaProto::LANE_CROSS:
      case StOverlapMetaProto::AV_CUTIN: {
        return StOverlapMetaProto::LON_MODIFIABLE;
      }
      case StOverlapMetaProto::OBJECT_CUTIN: {
        if (IsDrivingParallel(theta_diff)) {
          return StOverlapMetaProto::LON_LAT_MODIFIABLE;
        } else {
          return StOverlapMetaProto::LON_MODIFIABLE;
        }
      }
      case StOverlapMetaProto::UNKNOWN_SOURCE:
      case StOverlapMetaProto::OTHER: {
        return StOverlapMetaProto::NON_MODIFIABLE;
      }
    }
  }

  // For pedestrians.
  return StOverlapMetaProto::NON_MODIFIABLE;
}

bool AnalyzeOncomingRelationship(const StBoundaryRef& st_boundary,
                                 const DiscretizedPath& path,
                                 const SpacetimeObjectTrajectory& st_traj) {
  const auto& overlap_infos = st_boundary->overlap_infos();
  if (overlap_infos.empty()) {
    return false;
  }

  if ((st_boundary->bottom_left_point().s() <=
       st_boundary->bottom_right_point().s()) ||
      (st_traj.pose().v() < kLowSpeedThreshold)) {
    return false;
  }

  const auto& first_overlap_info = overlap_infos.front();

  const auto object_heading =
      st_traj.states()[first_overlap_info.obj_idx].traj_point->theta();
  const auto av_path_idx =
      (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2;
  const auto av_path_dir = path[av_path_idx].theta();

  const auto heading_diff = NormalizeAngle(object_heading - av_path_dir);

  return std::fabs(heading_diff) >= kOnComingAngleThreshold;
}

absl::StatusOr<StOverlapMetaProto> AnalyzeStOverlap(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const LaneInteractionMap& lane_interaction_map,
    const DrivePassage& drive_passage,
    const std::optional<KdTreeFrenetFrame>& target_frenet_frame,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const LaneChangeStage& lc_stage,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v) {
  StOverlapMetaProto overlap_meta;

  overlap_meta.set_is_oncoming(
      AnalyzeOncomingRelationship(st_boundary, path, st_traj));

  // Step 1: Analyze overlap pattern.
  overlap_meta.set_pattern(AnalyzeOverlapPattern(st_boundary, st_traj, path,
                                                 vehicle_geometry_params));

  // Step 2: Analyze overlap source and priority.
  ASSIGN_OR_RETURN(auto source_priority,
                   AnalyzeOverlapSourceAndPriority(
                       st_boundary, overlap_meta.pattern(), st_traj, psmm, path,
                       path_semantics, drive_passage, lane_interaction_map,
                       vehicle_geometry_params, lc_stage, av_shapes, init_v));
  VLOG(2) << "[speed finder] id" << st_boundary->id() << " source "
          << source_priority.source
          << " | priority: " << source_priority.priority
          << " | priority_reason: " << source_priority.priority_reason;
  overlap_meta.set_source(source_priority.source);
  overlap_meta.set_priority(source_priority.priority);
  overlap_meta.set_priority_reason(std::move(source_priority.priority_reason));
  if (source_priority.time_to_lc_complete.has_value()) {
    overlap_meta.set_time_to_lc_complete(*source_priority.time_to_lc_complete);
  }
  if (source_priority.is_making_u_turn.has_value()) {
    overlap_meta.set_is_making_u_turn(*source_priority.is_making_u_turn);
  }
  if (source_priority.is_merging_straight_lane.has_value()) {
    overlap_meta.set_is_merging_straight_lane(
        *source_priority.is_merging_straight_lane);
  }
  if (source_priority.is_crossing_straight_lane.has_value()) {
    overlap_meta.set_is_crossing_straight_lane(
        *source_priority.is_crossing_straight_lane);
  }
  if (source_priority.obj_lane_direction.has_value()) {
    overlap_meta.set_obj_lane_direction(
        static_cast<StOverlapMetaProto_LaneDirection>(
            *source_priority.obj_lane_direction));
  }
  if (source_priority.is_unprotected_left_turn.has_value()) {
    overlap_meta.set_is_unprotected_left_turn(
        *source_priority.is_unprotected_left_turn);
  }
  const auto theta_diff =
      CalculateThetaDiff(st_traj, drive_passage, target_frenet_frame);
  if (theta_diff.has_value()) {
    overlap_meta.set_theta_diff(*theta_diff);
  }
  const auto projection_dis_pair = CalculateProjectionDistance(st_traj, path);
  overlap_meta.set_front_most_projection_distance(projection_dis_pair.first);
  overlap_meta.set_rear_most_projection_distance(projection_dis_pair.second);
  // Step 3: Analyze modification type.
  overlap_meta.set_modification_type(AnalyzeOverlapModificationType(
      st_boundary, theta_diff, overlap_meta.source(), overlap_meta.priority()));

  return overlap_meta;
}

}  // namespace

StOverlapMetaProto::OverlapPattern AnalyzeOverlapPattern(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto& overlap_infos = st_boundary->overlap_infos();
  CHECK(!overlap_infos.empty());

  const auto& first_overlap_info = overlap_infos.front();
  const auto& last_overlap_info = overlap_infos.back();

  const int state_num = st_traj.states().size();
  const int path_num = path.size();

  // If first object contour before overlap is behind the first AV box on
  // path, we consider it to be on the path at the beginning; if first object
  // contour after overlap is ahead of the last AV box on path, we consider it
  // to be on the path at the end.
  enum class RelativeLonPosition {
    BEHIND = 0,     // NOLINT
    AHEAD = 1,      // NOLINT
    INTERSECT = 2,  // NOLINT
  };
  const auto compute_obj_relative_position_with_av_box =
      [&path, &st_traj, &vehicle_geometry_params](
          int av_path_idx, int obj_traj_idx) -> RelativeLonPosition {
    const auto& obj_contour = st_traj.states()[obj_traj_idx].contour;
    const auto& av_path_point = path[av_path_idx];
    const auto av_path_dir = Vec2d::FastUnitFromAngle(av_path_point.theta());
    Vec2d front, back;
    obj_contour.ExtremePoints(av_path_dir, &back, &front);
    const Vec2d av_path_pos = ToVec2d(av_path_point);
    if ((back - av_path_pos).Dot(av_path_dir) >
        vehicle_geometry_params.front_edge_to_center() +
            st_traj.required_lateral_gap()) {
      return RelativeLonPosition::AHEAD;
    } else if ((front - av_path_pos).Dot(av_path_dir) <
               -vehicle_geometry_params.back_edge_to_center() -
                   st_traj.required_lateral_gap()) {
      return RelativeLonPosition::BEHIND;
    } else {
      return RelativeLonPosition::INTERSECT;
    }
  };

  if (first_overlap_info.obj_idx == 0 ||
      compute_obj_relative_position_with_av_box(
          0, first_overlap_info.obj_idx - 1) == RelativeLonPosition::BEHIND) {
    // If first object contour out of path is in front of the last AV box, we
    // also consider it to be of type STAY.
    if (last_overlap_info.obj_idx == state_num - 1 ||
        compute_obj_relative_position_with_av_box(
            path_num - 1, last_overlap_info.obj_idx + 1) ==
            RelativeLonPosition::AHEAD) {
      return StOverlapMetaProto::STAY;
    } else {
      return StOverlapMetaProto::LEAVE;
    }
  }

  // If first object contour out of path is in front of the last AV box, we
  // also consider it to be of type ENTER.
  if (last_overlap_info.obj_idx == state_num - 1 ||
      compute_obj_relative_position_with_av_box(
          path_num - 1, last_overlap_info.obj_idx + 1) ==
          RelativeLonPosition::AHEAD) {
    return StOverlapMetaProto::ENTER;
  } else {
    // True for left side, false for right side.
    const auto get_side_on_path = [&path, &st_traj](int av_path_idx,
                                                    int obj_traj_idx) -> bool {
      const auto& av_path_point = path[av_path_idx];
      const auto& obj_traj_point = *st_traj.states()[obj_traj_idx].traj_point;
      const Vec2d ref = obj_traj_point.pos() - ToVec2d(av_path_point);
      const bool is_on_path_left =
          Vec2d::FastUnitFromAngle(av_path_point.theta()).CrossProd(ref) >= 0.0;
      return is_on_path_left;
    };

    const bool first_overlap_side = get_side_on_path(
        (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2,
        first_overlap_info.obj_idx);
    const bool last_overlap_side = get_side_on_path(
        (last_overlap_info.av_start_idx + last_overlap_info.av_end_idx) / 2,
        last_overlap_info.obj_idx);
    if (first_overlap_side == last_overlap_side) {
      return StOverlapMetaProto::INTERFERE;
    } else {
      return StOverlapMetaProto::CROSS;
    }
  }
}

bool IsAnalyzableStBoundary(const StBoundaryRef& st_boundary) {
  if (!RunStOverlapnalyzerByStBoundarySourceType(st_boundary->source_type())) {
    return false;
  }
  if (!RunStOverlapAnalzyerByStBoundaryObjectType(st_boundary->object_type())) {
    return false;
  }
  if (st_boundary->is_stationary()) return false;
  if (st_boundary->protection_type() == StBoundaryProto::LANE_CHANGE_GAP) {
    return false;
  }
  return true;
}

void AnalyzeStOverlaps(
    const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const LaneChangeStage& lc_stage,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v,
    std::vector<StBoundaryRef>* st_boundaries) {
  constexpr double kBackwardExtendLength = 30.0;  // m.
  std::optional<KdTreeFrenetFrame> target_frenet_frame = std::nullopt;
  const auto target_lane_path_extend = BackwardExtendLanePath(
      psmm, drive_passage.extend_lane_path(), kBackwardExtendLength);
  auto target_frenet_frame_or = BuildKdTreeFrenetFrame(
      SampleLanePathPoints(psmm, target_lane_path_extend),
      /*down_sample_raw_points=*/true);
  if (target_frenet_frame_or.ok()) {
    target_frenet_frame = std::move(*target_frenet_frame_or);
  }

  // Generate lane interaction map.
  const auto interaction_map = GetLaneInteractionMap(path_semantics, psmm);

  // TODO: See if need to parallelize this loop.
  for (auto& st_boundary : *st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;
    // All st-object-generated st-boundaries must have a traj_id.
    CHECK(st_boundary->traj_id().has_value());
    const auto& traj_id = *st_boundary->traj_id();
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

    ASSIGN_OR_CONTINUE(
        StOverlapMetaProto overlap_meta,
        AnalyzeStOverlap(st_boundary, *traj, psmm, path, path_semantics,
                         interaction_map, drive_passage, target_frenet_frame,
                         vehicle_geometry_params, lc_stage, av_shapes, init_v));

    st_boundary->set_overlap_meta(std::move(overlap_meta));
  }
}
}  // namespace planning
}  // namespace st
