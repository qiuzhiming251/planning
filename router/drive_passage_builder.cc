

#include "router/drive_passage_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "plan_common/maps/map_def.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/circle_fitter.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/plan_common_defs.h"
// #include "semantic_map.pb.h"
#include "plan_common/util/gradient_points_smoother.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/lane_point_util.h"

namespace st::planning {

namespace {

constexpr double kVehicleWidth = 2.02;
constexpr double kDrivePassageCutOffAngleDiff = 1.5 * M_PI;
constexpr double kDrivePassageContinuesCutOffAngleDiff = 0.6 * M_PI;
constexpr double kDrivePassageMaxForwardExtendLength = 10.0;  // m.
constexpr double kFarStationHorizonFirstRatio = 0.2;
constexpr double kFarStationHorizonSecondRatio = 0.5;
constexpr double kDrivePassageSmallEps = 1e-9;
constexpr double emergency_lat_offset = 0.1;
// constexpr double kFarStationHorizonRatio = 1.0 - 0.3;
// constexpr double kFarRouteStationStep = 2.0 * kRouteStationUnitStep;

// StationBoundaryType MapBoundaryTypeToStationBoundaryType(
//     mapping::LaneBoundaryProto::Type map_type) {
//   switch (map_type) {
//     case mapping::LaneBoundaryProto::UNKNOWN_TYPE:
//       return StationBoundaryType::UNKNOWN_TYPE;
//     case mapping::LaneBoundaryProto::BROKEN_WHITE:
//       return StationBoundaryType::BROKEN_WHITE;
//     case mapping::LaneBoundaryProto::SOLID_WHITE:
//       return StationBoundaryType::SOLID_WHITE;
//     case mapping::LaneBoundaryProto::BROKEN_YELLOW:
//       return StationBoundaryType::BROKEN_YELLOW;
//     case mapping::LaneBoundaryProto::SOLID_YELLOW:
//       return StationBoundaryType::SOLID_YELLOW;
//     case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
//       return StationBoundaryType::SOLID_DOUBLE_YELLOW;
//     case mapping::LaneBoundaryProto::CURB:
//       return StationBoundaryType::CURB;
//     case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
//       return StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE;
//     case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
//       return StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE;
//     default:
//       throw std::runtime_error("switch case on enum unexpected");
//   }
// }

StationBoundaryType MapLaneBoundaryTypeToStationBoundaryType(
    ad_byd::planning::LaneBoundaryType lane_type) {
  switch (lane_type.line_type) {
    case ad_byd::planning::LineType::SOLID:
      switch (lane_type.line_color) {
        case ad_byd::planning::LineColor::COLOR_YELLOW:
          return StationBoundaryType::SOLID_YELLOW;
        case ad_byd::planning::LineColor::COLOR_WHITE:
        default:
          return StationBoundaryType::SOLID_WHITE;
      }
    case ad_byd::planning::LineType::SOLID_SOLID:
      switch (lane_type.line_color) {
        case ad_byd::planning::LineColor::COLOR_YELLOW:
          return StationBoundaryType::SOLID_DOUBLE_YELLOW;
        case ad_byd::planning::LineColor::COLOR_WHITE:
        default:
          return StationBoundaryType::SOLID_WHITE;
      }
    case ad_byd::planning::LineType::DASHED:
    case ad_byd::planning::LineType::DASHED_DASHED:
      switch (lane_type.line_color) {
        case ad_byd::planning::LineColor::COLOR_YELLOW:
          return StationBoundaryType::BROKEN_YELLOW;
        case ad_byd::planning::LineColor::COLOR_WHITE:
        default:
          return StationBoundaryType::BROKEN_WHITE;
      }
    case ad_byd::planning::LineType::DASHED_SOLID:
      return StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE;
    case ad_byd::planning::LineType::SOLID_DASHED:
      return StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE;
    case ad_byd::planning::LineType::SHADED_AREA:
      return StationBoundaryType::SOLID_WHITE;
    case ad_byd::planning::LineType::VIRTUAL_LANE:
    case ad_byd::planning::LineType::VIRTUAL_JUNCTION:
      return StationBoundaryType::VIRTUAL_LANE;
    case ad_byd::planning::LineType::UNKNOWN:
      return StationBoundaryType::UNKNOWN_TYPE;
    default:
      LOG_ERROR << "Um-matched lane type: " << (int)lane_type.line_type;
      return StationBoundaryType::UNKNOWN_TYPE;
      // throw std::runtime_error("switch case on enum unexpected");
  }
}

// Returns true if type1 is strictly lower than type2.
bool LowerType(StationBoundaryType type1, StationBoundaryType type2) {
  switch (type1) {
    case StationBoundaryType::UNKNOWN_TYPE:
      return true;
    case StationBoundaryType::VIRTUAL_LANE:
      return StationBoundaryType::UNKNOWN_TYPE != type2 &&
             StationBoundaryType::VIRTUAL_LANE != type2;
    case StationBoundaryType::BROKEN_WHITE:
      return type2 == StationBoundaryType::SOLID_WHITE ||
             type2 == StationBoundaryType::BROKEN_YELLOW ||
             type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_WHITE:
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
      return type2 == StationBoundaryType::BROKEN_YELLOW ||
             type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::BROKEN_YELLOW:
      return type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_YELLOW:
      return type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      return type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::CURB:
    case StationBoundaryType::VIRTUAL_CURB:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

std::pair<double, double> ComputeLateralLimits(
    const std::vector<StationCenter>& centers,
    const StationCenter& current_center) {
  if (centers.empty()) return {-kMaxLateralOffset, kMaxLateralOffset};

  const double d_theta = NormalizeAngle(current_center.tangent.FastAngle() -
                                        centers.back().tangent.FastAngle());
  const double ds = current_center.accum_s - centers.back().accum_s;
  constexpr double kVehicleMinTurnRadius = 6.15;  // m.
  const double turn_radius =
      std::max(kVehicleMinTurnRadius, ds / std::abs(d_theta));

  if (std::abs(d_theta) * kMaxLateralOffset < ds)
    return {-kMaxLateralOffset, kMaxLateralOffset};
  else if (current_center.turn_type !=
           ad_byd::planning::NO_TURN)  // if turn_type is
                                       // U_turn/Right_TURN/Left_TURN, then
                                       // trust the large angle
  {
    if (d_theta < 0.0) {
      // Turn right
      return {-turn_radius, kMaxLateralOffset};
    } else {
      // Turn left
      return {-kMaxLateralOffset, turn_radius};
    }

  } else if (centers.size() >=
             3)  // if the turn_type is NO_TURN, then large_angle should
                 // continue 3 times, to avoid a broken center_point
  {
    bool is_continue_large_angle{true};
    auto it = centers.rbegin();
    for (size_t i = 0; i < 2; i++) {
      const double d_theta_temp = NormalizeAngle(
          (*it).tangent.FastAngle() - (*(it + 1)).tangent.FastAngle());
      const double ds_temp = (*it).accum_s - (*(it + 1)).accum_s;
      if ((std::abs(d_theta_temp) * kMaxLateralOffset < ds_temp) ||
          (std::signbit(d_theta_temp) !=
           std::signbit(d_theta))) {  // not large angle or opposite direction
        is_continue_large_angle = false;
        break;
      }
      ++it;
    }

    if (is_continue_large_angle) {
      if (d_theta < 0.0) {
        // Turn right
        return {-turn_radius, kMaxLateralOffset};
      } else {
        // Turn left
        return {-kMaxLateralOffset, turn_radius};
      }
    } else  // not continue large angle
    {
      return {-kMaxLateralOffset, kMaxLateralOffset};
    }
  } else  // centers size less than size 3 but large angle
  {
    return {-kMaxLateralOffset, kMaxLateralOffset};
  }
}

// mapping::LaneBoundaryProto::Type GetBoundaryType(
//     const mapping::LaneBoundaryInfo& boundary, int index) {
//   if (boundary.proto->points_type().empty()) {
//     return boundary.type;
//   }

//   return boundary.proto->points_type(index);
// }

void UpdateStationCenter(
    const PlannerSemanticMapManager& psmm, StationCenter& center,
    const double& max_right_offset, const double& max_left_offset,
    const absl::flat_hash_set<uint64_t>& lane_boundary_set, 
    bool* is_update_success) {
  if (psmm.map_ptr()->type() == ad_byd::planning::MapType::BEV_MAP ||
      !psmm.IsOnHighway() || center.is_in_intersection) {
        if (is_update_success != nullptr) {
          *is_update_success = true;
        }
        return;
      }

  const Segment2d normal_line(center.lat_point(max_right_offset),
                              center.lat_point(max_left_offset));

  // consider the lane boundary
  bool has_left_lane_boundary = false;
  bool has_right_lane_boundary = false;
  Vec2d left_closest_intersection, right_closest_intersection;
  Vec2d left_tangent, right_tangent;
  const auto lane_boundaries =
      psmm.GetLaneBoundaries(center.xy, kMaxLateralOffset);
  const bool is_merge_or_split = center.is_merging || center.is_splitting;
  for (const auto& boundary : lane_boundaries) {
    // Compute cross point of boundary and normal line
    if (lane_boundary_set.empty() ||
        !lane_boundary_set.contains(boundary->id())) {
      continue;
    }
    Vec2d closest_intersection;
    Vec2d closest_tangent;
    double min_sqr_dis = std::numeric_limits<double>::max();
    for (int k = 0; k + 1 < boundary->points().size(); ++k) {
      const Vec2d& p0 = boundary->points()[k];
      const Vec2d& p1 = boundary->points()[k + 1];
      const Segment2d boundary_segment(p0, p1);
      Vec2d intersection;
      if (!normal_line.GetIntersect(boundary_segment, &intersection)) continue;
      const double sqr_dis = center.xy.DistanceSquareTo(intersection);
      if (sqr_dis < min_sqr_dis) {
        min_sqr_dis = sqr_dis;
        closest_intersection = intersection;
        closest_tangent = boundary_segment.unit_direction();
      }
    }
    if (min_sqr_dis < std::numeric_limits<double>::max()) {
      auto station_boundary =
          MapLaneBoundaryTypeToStationBoundaryType(boundary->type());
      const auto lat_offset = center.lat_offset(closest_intersection);
      if (StationBoundaryType::UNKNOWN_TYPE != station_boundary) {
        lat_offset > 0 ? has_left_lane_boundary = true
                       : has_right_lane_boundary = true;
        if (lat_offset > 0) {
          left_closest_intersection = closest_intersection;
          left_tangent = closest_tangent;
        } else {
          right_closest_intersection = closest_intersection;
          right_tangent = closest_tangent;
        }
      }
    }
  }
  if (has_left_lane_boundary && has_right_lane_boundary) {
    Vec2d new_center =
        (left_closest_intersection + right_closest_intersection) * 0.5;
    double delta_lat_offset = center.lat_offset(new_center);
    if (delta_lat_offset > kDefaultHalfLaneWidth) {
      delta_lat_offset = kDefaultHalfLaneWidth;
    } else if (delta_lat_offset < -kDefaultHalfLaneWidth) {
      delta_lat_offset = -kDefaultHalfLaneWidth;
    }
    center.xy = center.lat_point(delta_lat_offset);
    if (is_update_success != nullptr)
      *is_update_success = true;
    // LOG_ERROR << " , center lat offset1: " << delta_lat_offset
    //            << "center lat offset2: " << center.lat_offset(new_center);

    // center.tangent = (left_tangent + right_tangent).normalized();
  }
}

std::vector<StationBoundary> CollectStationBoundaries(
    const PlannerSemanticMapManager& psmm, const StationCenter& center,
    const double& max_right_offset, const double& max_left_offset,
    const absl::flat_hash_set<uint64_t>& lane_boundary_set,
    const bool ignore_curb, bool* has_cross_curb_ptr, uint64_t* cross_curb_id) {
  const Segment2d normal_line(center.lat_point(max_right_offset),
                              center.lat_point(max_left_offset));

  std::vector<StationBoundary> station_boundaries;

  // 1. consider the road boundary
  const auto road_boundaries =
      psmm.GetRoadBoundaries(center.xy, kMaxLateralOffset);
  bool has_left_curb = false;
  bool has_right_curb = false;
  const double ignore_center_range_th = ad_byd::planning::Constants::ZERO;
  for (const auto& boundary : road_boundaries) {
    if (ignore_curb) break;
    if (!boundary || boundary->type().boundary_type ==
                         ad_byd::planning::BoundaryType::OCC_VEGETATION) {
      continue;
    }
    double most_left_offset = -1.0;
    double least_left_offset = 100.0;
    double most_right_offset = 1.0;
    double least_right_offset = -100.0;
    double closest_offset = std::numeric_limits<double>::max();
    for (int k = 0; k < boundary->points().size(); ++k) {
      const Vec2d& cp = boundary->points()[k];
      const Segment2d boundary_segment(center.xy, cp);
      if (normal_line.DistanceTo(cp) > 3.0 * kRouteStationUnitStep) continue;
      double proj_value = normal_line.ProjectOntoUnit(cp);
      if (proj_value < 0.0 || proj_value > max_left_offset - max_right_offset)
        continue;
      double lat_offset = proj_value + max_right_offset;
      if (lat_offset > 0) {
        most_left_offset = std::fmax(most_left_offset, lat_offset);
        least_left_offset = std::fmin(least_left_offset, lat_offset);
      } else {
        most_right_offset = std::fmin(most_right_offset, lat_offset);
        least_right_offset = std::fmax(least_right_offset, lat_offset);
      }
    }
    if (most_left_offset < -0.5) {
      if (least_right_offset > -50.0 &&
          least_right_offset < -ignore_center_range_th) {
        closest_offset = least_right_offset;
      }
    } else if (most_right_offset > 0.5) {
      if (least_left_offset < 50.0 &&
          least_left_offset > ignore_center_range_th) {
        closest_offset = least_left_offset;
      }
    } else {
      // closest_offset = most_left_offset > -most_right_offset
      //                      ? least_left_offset
      //                      : least_right_offset;
      if (std::fmin(most_left_offset, -most_right_offset) >
              ad_byd::planning::Constants::ZERO &&
          nullptr != has_cross_curb_ptr && nullptr != cross_curb_id) {
        *has_cross_curb_ptr = true;
        *cross_curb_id = boundary->id();
      }
    }
    if (closest_offset < std::numeric_limits<double>::max()) {
      station_boundaries.push_back({StationBoundaryType::CURB, closest_offset});
      closest_offset > 0 ? has_left_curb = true : has_right_curb = true;
    }
  }

  // 2. consider the clear areas
  const auto clear_areas = psmm.GetClearAreas(center.xy, kMaxLateralOffset);
  for (const auto& clear_area : clear_areas) {
    if (ignore_curb) break;
    if (ad_byd::planning::ImpassableAeraType::LINEAR_SAFE_ISLAND ==
        clear_area->type())
      continue;
    double min_sqr_dis = std::numeric_limits<double>::max();
    double closest_offset = 10.0;
    const double center_offset =
        normal_line.ProjectOntoUnit(clear_area->polygon().CircleCenter()) +
        max_right_offset;
    for (int k = 0; k < clear_area->points().size(); ++k) {
      const Vec2d& cp = clear_area->points()[k];
      const Segment2d boundary_segment(center.xy, cp);
      if (normal_line.DistanceTo(cp) > 1.1 * kRouteStationUnitStep) continue;
      double proj_value = normal_line.ProjectOntoUnit(cp);
      if (proj_value < 0.0 || proj_value > max_left_offset - max_right_offset)
        continue;
      double lat_offset = proj_value + max_right_offset;
      if (lat_offset * center_offset < 0.0) continue;
      double sqr_dis = std::fabs(lat_offset);
      if (sqr_dis < min_sqr_dis) {
        min_sqr_dis = sqr_dis;
        closest_offset = lat_offset;
      }
    }
    if (min_sqr_dis < std::numeric_limits<double>::max()) {
      station_boundaries.push_back({StationBoundaryType::CURB, closest_offset});
      closest_offset > 0 ? has_left_curb = true : has_right_curb = true;
    }
  }

  if (!has_left_curb)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_CURB, max_left_offset});
  if (!has_right_curb)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_CURB, max_right_offset});

  // 3. consider the lane boundary
  bool has_left_lane_boundary = false;
  bool has_right_lane_boundary = false;
  const auto lane_boundaries =
      psmm.GetLaneBoundaries(center.xy, kMaxLateralOffset);
  const bool is_merge_or_split = center.is_merging || center.is_splitting;
  for (const auto& boundary : lane_boundaries) {
    // Compute cross point of boundary and normal line
    if (lane_boundary_set.empty() ||
        !lane_boundary_set.contains(boundary->id())) {
      continue;
    }
    Vec2d closest_intersection;
    double min_sqr_dis = std::numeric_limits<double>::max();
    for (int k = 0; k + 1 < boundary->points().size(); ++k) {
      const Vec2d& p0 = boundary->points()[k];
      const Vec2d& p1 = boundary->points()[k + 1];
      const Segment2d boundary_segment(p0, p1);
      Vec2d intersection;
      if (!normal_line.GetIntersect(boundary_segment, &intersection)) continue;
      const double sqr_dis = center.xy.DistanceSquareTo(intersection);
      if (sqr_dis < min_sqr_dis) {
        min_sqr_dis = sqr_dis;
        closest_intersection = intersection;
      }
    }
    if (min_sqr_dis < std::numeric_limits<double>::max()) {
      auto station_boundary =
          MapLaneBoundaryTypeToStationBoundaryType(boundary->type());
      if (!is_merge_or_split ||
          (StationBoundaryType::SOLID_YELLOW == station_boundary ||
           StationBoundaryType::SOLID_WHITE == station_boundary)) {
        const auto lat_offset = center.lat_offset(closest_intersection);
        if (StationBoundaryType::UNKNOWN_TYPE != station_boundary) {
          station_boundaries.push_back({station_boundary, lat_offset});
          lat_offset > 0 ? has_left_lane_boundary = true
                         : has_right_lane_boundary = true;
        } else {
          const auto abs_lat_offset = std::fabs(lat_offset);
          if (abs_lat_offset > kMinHalfLaneWidth - 0.3 &&
              abs_lat_offset < kMaxHalfLaneWidth) {
            station_boundaries.push_back(
                {StationBoundaryType::VIRTUAL_LANE, lat_offset});
            lat_offset > 0 ? has_left_lane_boundary = true
                           : has_right_lane_boundary = true;
          }
        }
      }
    }
  }

  if (!has_left_lane_boundary)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_LANE, kDefaultHalfLaneWidth});
  if (!has_right_lane_boundary)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_LANE, -kDefaultHalfLaneWidth});

  return station_boundaries;
}

void PostProcessStationBoundaries(
    const StationCenter& center,
    std::vector<StationBoundary>* mutable_boundaries) {
  std::stable_sort(mutable_boundaries->begin(), mutable_boundaries->end(),
                   [](const StationBoundary& lhs, const StationBoundary& rhs) {
                     return lhs.lat_offset < rhs.lat_offset;
                   });
  std::vector<StationBoundary> remaining_boundaries;
  const double min_half_virtual_lane_width = 0.9;

  auto current_top_type = StationBoundaryType::UNKNOWN_TYPE;
  for (auto it = mutable_boundaries->rbegin(); it != mutable_boundaries->rend();
       ++it) {
    // From center to right.
    if (it->lat_offset > 0.0 || LowerType(it->type, current_top_type)) {
      continue;
    }
    if (StationBoundaryType::VIRTUAL_LANE == it->type &&
        it->lat_offset > -min_half_virtual_lane_width) {
      continue;
    }

    remaining_boundaries.push_back(*it);
    if (it->type == StationBoundaryType::CURB ||
        it->type == StationBoundaryType::VIRTUAL_CURB) {
      break;
    }
    current_top_type = it->type;
  }
  std::reverse(remaining_boundaries.begin(), remaining_boundaries.end());

  current_top_type = StationBoundaryType::UNKNOWN_TYPE;
  for (const auto& bound : *mutable_boundaries) {
    // From center to left.
    if (bound.lat_offset < 0.0 || LowerType(bound.type, current_top_type)) {
      continue;
    }
    if (StationBoundaryType::VIRTUAL_LANE == bound.type &&
        bound.lat_offset < min_half_virtual_lane_width) {
      continue;
    }

    remaining_boundaries.push_back(bound);
    if (bound.type == StationBoundaryType::CURB ||
        bound.type == StationBoundaryType::VIRTUAL_CURB) {
      break;
    }
    current_top_type = bound.type;
  }

  // We can only borrow one lane at most if crossed yellow lane boundary.
  const double borrow_road_width = kDefaultLaneWidth + kVehicleWidth * 0.5;
  double left_solid_yellow_line_offset = std::numeric_limits<double>::max();
  double right_solid_yellow_line_offset = std::numeric_limits<double>::lowest();
  for (const auto& bound : remaining_boundaries) {
    if (bound.type == StationBoundaryType::SOLID_YELLOW ||
        bound.type == StationBoundaryType::SOLID_DOUBLE_YELLOW) {
      bound.lat_offset < 0.0 ? right_solid_yellow_line_offset = bound.lat_offset
                             : left_solid_yellow_line_offset = bound.lat_offset;
    }
  }
  if (!remaining_boundaries.empty() &&
      left_solid_yellow_line_offset + borrow_road_width <
          remaining_boundaries.back().lat_offset) {
    remaining_boundaries.back().type = StationBoundaryType::VIRTUAL_CURB;
    remaining_boundaries.back().lat_offset =
        left_solid_yellow_line_offset + borrow_road_width;
  }
  if (!remaining_boundaries.empty() &&
      right_solid_yellow_line_offset - borrow_road_width >
          remaining_boundaries.front().lat_offset) {
    remaining_boundaries.front().type = StationBoundaryType::VIRTUAL_CURB;
    remaining_boundaries.front().lat_offset =
        right_solid_yellow_line_offset - borrow_road_width;
  }

  *mutable_boundaries = std::move(remaining_boundaries);
}

bool IsChangeStation(const StationInfo& last, const StationInfo& cur) {
  if ((last.turn_type != cur.turn_type) ||
      (last.split_topo != cur.split_topo) ||
      (last.merge_topo != cur.merge_topo) ||
      (last.lane_type != LaneType::LANE_ROUND_ABOUT &&
       cur.lane_type == LaneType::LANE_ROUND_ABOUT) ||
      (last.lane_type == LaneType::LANE_ROUND_ABOUT &&
       cur.lane_type != LaneType::LANE_ROUND_ABOUT)) {
    return true;
  }
  return false;
}

struct DrivePassageData {
  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
};

DrivePassageData SampleLanePathWithPlannerSemanticMapMgr(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double end_s, bool avoid_loop, bool avoid_notcontinuous,
    std::optional<double> override_speed_limit_mps, double ref_ego_s,
    double required_planning_horizon, std::vector<int>* change_index) {
  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
  const int n = CeilToInt((end_s - start_s) / kRouteStationUnitStepForBev);
  centers.reserve(n);
  stations_boundaries.reserve(n);
  StationInfo last_station_info{};

  if (change_index != nullptr) {
    change_index->clear();
  }
  int station_idx = 0;
  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;
  double angle_diff = 0.0;
  const double far_station_firstthres_psmm =
      kFarStationHorizonFirstRatio * required_planning_horizon;
  const double far_station_secondthres_psmm =
      kFarStationHorizonSecondRatio * required_planning_horizon;
  constexpr double kFarRouteStationFirstStepPsmm =
      2.0 * kRouteStationUnitStepForBev;
  constexpr double kFarRouteStationSecondStepPsmm =
      5.0 * kRouteStationUnitStepForBev;
  double sample_s = start_s;
  double ego_front_s = start_s - ref_ego_s;
  // emergency lane judgement
  bool is_emergency_lane = false;
  for (const auto& seg : lane_path) {
    const auto& lane_info_ptr = psmm.map_ptr()->GetLaneById(seg.lane_id);
    if ((lane_info_ptr != nullptr) &&
        (lane_info_ptr->type() == LaneType::LANE_EMERGENCY)) {
      is_emergency_lane = true;
      break;
    }
  }
  while (sample_s <= end_s) {
    const auto sample_lane_point = lane_path.ArclengthToLanePoint(sample_s);

    // Ensure we do not create a loop in drive passage. Planner does not support
    // looped passage.
    if (station_idx == 0) {
      prev_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
    } else {
      const auto cur_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
      angle_diff = NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if ((angle_diff >= kDrivePassageContinuesCutOffAngleDiff ||
         angle_diff <= -kDrivePassageContinuesCutOffAngleDiff) &&
        avoid_notcontinuous) {
      break;
    }
    if ((max_accumulated_angle_diff >= kDrivePassageCutOffAngleDiff ||
         min_accumulated_angle_diff <= -kDrivePassageCutOffAngleDiff) &&
        avoid_loop) {
      break;
    }
    station_idx++;

    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm,
                                      sample_lane_point.lane_id());

    double speed_limit =
        override_speed_limit_mps.has_value()
            ? *override_speed_limit_mps
            : psmm.QueryLaneSpeedLimitById(sample_lane_point.lane_id());

    StationInfo station_info{
        .is_in_intersection =
            (lane_info.junction_id() != 0 &&
             lane_info.type() == LaneType::LANE_VIRTUAL_JUNCTION),
        .is_exclusive_right_turn =
            (lane_info.turn_type() == ad_byd::planning::RIGHT_TURN),
        .is_in_roundabout =
            (lane_info.type() == ad_byd::planning::LANE_ROUND_ABOUT),
        .speed_limit = speed_limit,
        .lane_type = lane_info.type(),
        .turn_type = lane_info.turn_type(),
        .split_topo = lane_info.split_topology(),
        .merge_topo = lane_info.merge_topology()};

    if ((change_index != nullptr) && !centers.empty() &&
        IsChangeStation(last_station_info, station_info)) {
      change_index->emplace_back(centers.size());
    }
    last_station_info = station_info;
    // Station center:
    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = ComputeLanePointPos(psmm, sample_lane_point),
        .tangent = ComputeLanePointTangent(psmm, sample_lane_point),
        .accum_s = sample_s,
        .speed_limit =
            override_speed_limit_mps.has_value()
                ? *override_speed_limit_mps
                : psmm.QueryLaneSpeedLimitById(sample_lane_point.lane_id()),
        // .point_mse =
        //     psmm.GetpointsmseById(sample_lane_point.lane_id(), sample_s),
        .is_virtual = lane_info.IsVirtual(),
        // .is_merging = (lane_info.merge_topology() ==
        //                    ad_byd::planning::TOPOLOGY_MERGE_LEFT ||
        //                lane_info.merge_topology() ==
        //                    ad_byd::planning::TOPOLOGY_MERGE_RIGHT),
        .is_merging =
            (lane_info.lane_info().lane_merge_info.valid &&
             lane_info.lane_info().lane_merge_info.dist_to_merge < 150.0),
        .is_splitting = (lane_info.split_topology() ==
                             ad_byd::planning::TOPOLOGY_SPLIT_LEFT ||
                         lane_info.split_topology() ==
                             ad_byd::planning::TOPOLOGY_SPLIT_RIGHT),
        .is_in_intersection =
            (lane_info.junction_id() != 0 &&
             lane_info.type() == LaneType::LANE_VIRTUAL_JUNCTION),
        .has_cross_curb = false,
        .cross_curb_id = 0,
        .turn_type = lane_info.turn_type(),
        .station_info = std::move(station_info)};

    // lane_boundaries
    absl::flat_hash_set<uint64_t> lane_boundaries;
    lane_boundaries.reserve(
        lane_info.left_boundary()->lane_boundaries().size() +
        lane_info.right_boundary()->lane_boundaries().size());
    lane_boundaries.clear();
    for (const auto& bound : lane_info.left_boundary()->lane_boundaries()) {
      lane_boundaries.insert(bound->id());
    }
    for (const auto& bound : lane_info.right_boundary()->lane_boundaries()) {
      lane_boundaries.insert(bound->id());
    }

    // Station boundaries:
    const auto [right_lat_offset, left_lat_offset] =
        ComputeLateralLimits(centers, center);
    // UpdateStationCenter(psmm, center, right_lat_offset, left_lat_offset,
    //                     lane_boundaries);
    if (is_emergency_lane) center.xy = center.lat_point(emergency_lat_offset);
    constexpr double kBehindEgoBuffer = 2.5;
    auto current_station_boundaries = CollectStationBoundaries(
        psmm, center, right_lat_offset, left_lat_offset, lane_boundaries,
        sample_s < ref_ego_s - kBehindEgoBuffer, &center.has_cross_curb,
        &center.cross_curb_id);
    PostProcessStationBoundaries(center, &current_station_boundaries);

    centers.emplace_back(std::move(center));
    stations_boundaries.emplace_back(std::move(current_station_boundaries));
    ego_front_s = sample_s - ref_ego_s;
    if (ego_front_s > far_station_secondthres_psmm) {
      sample_s += kFarRouteStationSecondStepPsmm;
    } else if ((ego_front_s > far_station_firstthres_psmm) ||
               (ego_front_s < -kDrivePassageSmallEps)) {
      sample_s += kFarRouteStationFirstStepPsmm;
    } else {
      sample_s += kRouteStationUnitStepForBev;
    }
  }
  return DrivePassageData{
      .centers = std::move(centers),
      .stations_boundaries = std::move(stations_boundaries)};
}

void ExtendDrivePassageDataByCircleFit(double fit_length,
                                       DrivePassageData* dp_data_ptr) {
  constexpr int kFitDataMaxSize = 50;
  constexpr int kFitEveryNPt = 1;

  auto& centers = dp_data_ptr->centers;
  auto& stations_boundaries = dp_data_ptr->stations_boundaries;

  std::vector<Vec2d> fit_data;
  std::vector<double> weights;
  fit_data.reserve(kFitDataMaxSize);
  weights.reserve(kFitDataMaxSize);
  for (int i = centers.size() - 1; i >= 0; i -= kFitEveryNPt) {
    fit_data.push_back(centers[i].xy);
    weights.push_back(Sqr(kFitDataMaxSize - fit_data.size()));
    if (fit_data.size() >= kFitDataMaxSize) break;
  }
  double mse = 0.0;
  const auto circle_or =
      FitCircleToData(fit_data, weights, /*solver=*/SVD, &mse);
  if (!circle_or.ok()) {
    LOG_WARN << "Drive passage forward fitting failed: "
             << circle_or.status().message();
    return;
  }
  constexpr double kMaxFittingMSError = 0.003;
  if (mse > kMaxFittingMSError) {
    // LOG_WARN << "Drive passage forward fitting failed: fitting error "
    //              << mse << " is too large.";
    return;
  }
  const auto& circle = *circle_or;
  constexpr double kMinFittedRadius = 60.0;
  constexpr double kMaxFittedRadius = 2000.0;
  if (circle.radius() < kMinFittedRadius ||
      circle.radius() > kMaxFittedRadius) {
    // LOG_WARN << "Drive passage forward fitting failed: fitted radius "
    //              << circle.radius() << " out of reasonable range ("
    //              << kMinFittedRadius << ", " << kMaxFittedRadius << ").";
    return;
  }
  double last_accum_s = centers.back().accum_s;
  const double far_station_firstthres_cirfit =
      kFarStationHorizonFirstRatio * fit_length;
  constexpr double kFittedRadiusThres = 500.0;
  const double kFarRouteStationStepCirfit =
      (circle.radius() > kFittedRadiusThres)
          ? 10.0 * kRouteStationUnitStepForBev
          : 5.0 * kRouteStationUnitStepForBev;

  double cirfit_step_s = kRouteStationUnitStepForBev;
  while (last_accum_s < fit_length) {
    if (last_accum_s > far_station_firstthres_cirfit) {
      cirfit_step_s = kFarRouteStationStepCirfit;
    }
    const double d_theta = std::copysign(
        cirfit_step_s / circle.radius(),
        AngleDifference(circle.EvaluateTheta(centers[centers.size() - 2].xy),
                        circle.EvaluateTheta(centers.back().xy)));
    // while (last_accum_s < fit_length) {
    auto center = centers.back();
    center.lane_id = mapping::kInvalidElementId;
    center.fraction = 1.0;
    center.is_virtual = true;
    center.is_splitting = false;

    const Vec2d new_pos =
        circle.EvaluateXY(circle.EvaluateTheta(center.xy) + d_theta);
    center.tangent = (new_pos - center.xy).normalized();
    last_accum_s += center.xy.DistanceTo(new_pos);
    center.xy = new_pos;
    center.accum_s = last_accum_s;

    centers.emplace_back(std::move(center));
    stations_boundaries.push_back(
        {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
         {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    //}
  }
}

}  // namespace

void SmoothDrivePassageCenterPoints(DrivePassageData& dp_data) {
  int dp_centers_size = dp_data.centers.size();
  if (dp_centers_size <= 2) {
    LOG_ERROR << "drive passage points size not enough";
    return;
  }
  std::vector<Vec2d> dp_data_center_points;
  dp_data_center_points.reserve(dp_centers_size + 2);
  dp_data_center_points.emplace_back(dp_data.centers.front().xy);
  for (const auto& center_it : dp_data.centers) {
    dp_data_center_points.emplace_back(center_it.xy);
  }
  dp_data_center_points.emplace_back(dp_data.centers.back().xy);
  const auto smooth_result = SmoothPointsByGradientDescent(
      absl::MakeSpan(dp_data_center_points), 100, 0.01, 0.15);
  if (!smooth_result.success) {
    LOG_WARN << "Smooth drive passage points failed.";
    return;
  }

  if (dp_data_center_points.size() == dp_centers_size + 2) {
    for (int i = 0; i < dp_centers_size; i++) {
      dp_data.centers[i].xy = dp_data_center_points[i + 1];
      if (i == 0) {
        dp_data.centers[i].tangent =
            (dp_data_center_points[i + 2] - dp_data_center_points[i + 1])
                .normalized();
        dp_data.centers[i].accum_s = 0.0;
      } else {
        dp_data.centers[i].tangent =
            (dp_data_center_points[i + 1] - dp_data_center_points[i])
                .normalized();
        const double dx =
            dp_data.centers[i].xy[0] - dp_data.centers[i - 1].xy[0];
        const double dy =
            dp_data.centers[i].xy[1] - dp_data.centers[i - 1].xy[1];
        dp_data.centers[i].accum_s =
            dp_data.centers[i - 1].accum_s + sqrt(dx * dx + dy * dy);
      }
    }
  }
}

absl::StatusOr<DrivePassage> BuildDrivePassageForBevMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose, bool avoid_loop,
    bool avoid_notcontinuous, double backward_extend_len,
    double required_planning_horizon,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type,
    ad_byd::planning::LaneSeqInfoPtr lane_seq_info) {
  // ("BuildDrivePassageFromLanePath");
  double start_s;
  mapping::LanePath extended_lane_path;
  if (backward_extend_len > 0.0) {
    extended_lane_path =
        BackwardExtendLanePath(psmm, lane_path_from_pose, backward_extend_len);
    start_s = extended_lane_path.FirstOccurrenceOfLanePointToArclength(
        lane_path_from_pose.front());
  } else {
    start_s = 0.0;
    extended_lane_path = lane_path_from_pose;
  }
  std::vector<int> change_index;
  auto dp_data = SampleLanePathWithPlannerSemanticMapMgr(
      psmm, extended_lane_path, 0.0, extended_lane_path.length(), avoid_loop,
      avoid_notcontinuous, override_speed_limit_mps, start_s,
      required_planning_horizon, &change_index);

  if (dp_data.centers.size() <= 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Too few stations in BuildDrivePassageFromLanePath: Lane path: ",
        lane_path_from_pose.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }

  if (dp_data.centers.back().accum_s - start_s < 3.0) {
    return absl::FailedPreconditionError(absl::StrCat(
        "No enough legth in front of ego car: Lane path: ",
        lane_path_from_pose.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }

  // const int kFilterLastPointsNum = static_cast<int>(10 / step_s);
  // const int kMinFrontPointsNum = static_cast<int>(50 / step_s);
  // constexpr const double PointMseThresholdEndPoints = 0.25;
  // int dp_front_size =
  //     static_cast<int>((dp_data.centers.back().accum_s - start_s) / step_s);
  // if (dp_front_size > kMinFrontPointsNum) {
  //   dp_front_size =
  //       std::max(kMinFrontPointsNum, dp_front_size - kFilterLastPointsNum);
  // }
  // int dp_total_size = dp_front_size + static_cast<int>(start_s / step_s);
  // if (dp_total_size < dp_data.centers.size()) {
  //   while ((dp_total_size < dp_data.centers.size()) &&
  //          (dp_data.centers[dp_total_size].point_mse <
  //           PointMseThresholdEndPoints)) {
  //     dp_total_size++;
  //   }
  //   dp_data.centers.resize(dp_total_size);
  //   dp_data.stations_boundaries.resize(dp_total_size);
  // }
  // const double kMinFrontPointsAccumS = 50.0;
  // const double kFilterLastPointsAccumS = 9.0;
  // if (dp_data.centers.back().accum_s > kMinFrontPointsAccumS) {
  //   int dp_total_size = dp_data.centers.size();
  //   double FiterAccumS = 0;
  //   double LastAccumS = dp_data.centers.back().accum_s;
  //   while (((LastAccumS - FiterAccumS) > kMinFrontPointsAccumS) &&
  //          (FiterAccumS < kFilterLastPointsAccumS) && dp_total_size > 4 &&
  //          (dp_data.centers[dp_total_size - 1].point_mse >
  //           PointMseThresholdEndPoints)) {
  //     FiterAccumS += dp_data.centers[dp_total_size - 1].accum_s -
  //                    dp_data.centers[dp_total_size - 2].accum_s;
  //     dp_total_size--;
  //   }
  //   dp_data.centers.resize(dp_total_size);
  //   dp_data.stations_boundaries.resize(dp_total_size);
  // }
  SmoothDrivePassageCenterPoints(dp_data);

  const double forward_extend_len =
      required_planning_horizon - lane_path_from_pose.length();
  // const auto forward_extend_lane_path = ForwardExtendLanePath(
  //     psmm, lane_path_from_pose, forward_extend_len, /*report_issue=*/false);
  if (forward_extend_len > 0.0
      // && forward_extend_lane_path.back() == lane_path_from_pose.back()
  ) {
    // If no forward extension is made (meaning the lane path goes beyond loaded
    // map), fake the rest part of drive passage to the required length.
    // const int n = CeilToInt(required_planning_horizon / step_s) + 1;
    const int n = dp_data.centers.size() + 1;
    dp_data.centers.reserve(n);
    dp_data.stations_boundaries.reserve(n);

    double last_accum_s = dp_data.centers.back().accum_s;
    double kMaxFittedLength = last_accum_s + 80;  // m.
    double kMinLaneLengthForFitting = 40.0;       // m.
    const double fit_length =
        std::min(required_planning_horizon, kMaxFittedLength);
    if (last_accum_s < fit_length && last_accum_s >= kMinLaneLengthForFitting) {
      ExtendDrivePassageDataByCircleFit(fit_length, &dp_data);
    }
    const double far_station_firstthres_bev =
        kFarStationHorizonFirstRatio * fit_length;
    constexpr double kFarRouteStationStepBev =
        10.0 * kRouteStationUnitStepForBev;
    last_accum_s = dp_data.centers.back().accum_s;
    double bev_step_s = kRouteStationUnitStepForBev;
    while (last_accum_s <= required_planning_horizon) {
      if (last_accum_s > far_station_firstthres_bev) {
        bev_step_s = kFarRouteStationStepBev;
      }
      last_accum_s += bev_step_s;
      auto center = dp_data.centers.back();
      center.lane_id = mapping::kInvalidElementId;
      center.fraction = 1.0;
      center.xy += center.tangent * bev_step_s;
      center.accum_s = last_accum_s;
      center.is_virtual = true;
      center.is_splitting = false;

      dp_data.centers.emplace_back(std::move(center));
      dp_data.stations_boundaries.push_back(
          {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
           {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    }
  }
  if (dp_data.centers.size() < 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Too few stations in BuildDrivePassageFromLanePath: Lane path: ",
        lane_path_from_pose.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }

  // Create station vector.
  StationVector<Station> stations;
  stations.reserve(dp_data.centers.size());
  for (int i = 0; i < dp_data.centers.size(); ++i) {
    if (backward_extend_len > 0.0) {
      dp_data.centers[i].accum_s -= start_s;
    }
    stations.emplace_back(std::move(dp_data.centers[i]),
                          std::move(dp_data.stations_boundaries[i]));
  }
  return DrivePassage(std::move(stations), lane_path_from_pose,
                      std::move(extended_lane_path),
                      /*lane_path_start_s=*/0.0,
                      /*reach_destination=*/false, type,
                      /*change_index=*/change_index, lane_seq_info);
}

absl::StatusOr<DrivePassage> BuildDrivePassageFromLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step_s, bool avoid_loop, bool avoid_notcontinuous,
    double backward_extend_len, double required_planning_horizon,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type) {
  double start_s;
  mapping::LanePath extended_lane_path;
  if (backward_extend_len > 0.0) {
    extended_lane_path =
        BackwardExtendLanePath(psmm, lane_path, backward_extend_len);
    start_s = extended_lane_path.FirstOccurrenceOfLanePointToArclength(
        lane_path.front());
  } else {
    start_s = 0.0;
    extended_lane_path = lane_path;
  }
  std::vector<int> change_index;
  auto dp_data = SampleLanePathWithPlannerSemanticMapMgr(
      psmm, extended_lane_path, 0.0, extended_lane_path.length(), avoid_loop,
      avoid_notcontinuous, override_speed_limit_mps, start_s,
      required_planning_horizon, &change_index);

  if (dp_data.centers.size() <= 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Too few stations in BuildDrivePassageFromLanePath: Lane path: ",
        lane_path.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }

  const double forward_extend_len =
      required_planning_horizon - lane_path.length();
  const auto forward_extend_lane_path = ForwardExtendLanePath(
      psmm, lane_path, forward_extend_len, /*report_issue=*/false);
  if (forward_extend_len > 0.0 &&
      forward_extend_lane_path.back() == lane_path.back()) {
    // If no forward extension is made (meaning the lane path goes beyond loaded
    // map), fake the rest part of drive passage to the required length.
    const int n = CeilToInt(required_planning_horizon / step_s) + 1;
    dp_data.centers.reserve(n);
    dp_data.stations_boundaries.reserve(n);

    double last_accum_s = dp_data.centers.back().accum_s;
    double kMaxFittedLength = 100.0;         // m.
    double kMinLaneLengthForFitting = 40.0;  // m.
    const double fit_length =
        std::min(required_planning_horizon, kMaxFittedLength);
    if (last_accum_s < fit_length && last_accum_s >= kMinLaneLengthForFitting) {
      ExtendDrivePassageDataByCircleFit(fit_length, &dp_data);
    }

    last_accum_s = dp_data.centers.back().accum_s;
    while ((last_accum_s += step_s) <= required_planning_horizon) {
      auto center = dp_data.centers.back();
      center.lane_id = mapping::kInvalidElementId;
      center.fraction = 1.0;
      center.xy += center.tangent * step_s;
      center.accum_s = last_accum_s;
      center.is_virtual = true;
      center.is_splitting = false;

      dp_data.centers.emplace_back(std::move(center));
      dp_data.stations_boundaries.push_back(
          {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
           {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    }
  }
  if (dp_data.centers.size() < 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Too few stations in BuildDrivePassageFromLanePath: Lane path: ",
        lane_path.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }

  // Create station vector.
  StationVector<Station> stations;
  stations.reserve(dp_data.centers.size());
  for (int i = 0; i < dp_data.centers.size(); ++i) {
    if (backward_extend_len > 0.0) {
      dp_data.centers[i].accum_s -= start_s;
    }
    stations.emplace_back(std::move(dp_data.centers[i]),
                          std::move(dp_data.stations_boundaries[i]));
  }

  return DrivePassage(std::move(stations), lane_path,
                      std::move(extended_lane_path),
                      /*lane_path_start_s=*/start_s,
                      /*reach_destination=*/false, type, /*change_index=*/{});
}

absl::StatusOr<DrivePassage> BuildDrivePassage(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePath& backward_extended_lane_path,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    const mapping::LanePoint& destination, bool all_lanes_virtual,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type,
    ad_byd::planning::LaneSeqInfoPtr lane_seq_info) {
  // ("BuildDrivePassage");

  auto lane_path_in_horizon =
      lane_path_from_pose.BeforeArclength(planning_horizon);

  // Forward and backward extend lane path. Forward for projection of objects
  // that are slightly beyond the current lane path's end, and backward for plan
  // start point projection of the next frame.
  const double forward_extend_len =
      std::min(planning_horizon - lane_path_from_pose.length(),
               kDrivePassageMaxForwardExtendLength);
  auto ref_lane_path = ForwardExtendLanePath(
      psmm,
      backward_extended_lane_path.BeforeFirstOccurrenceOfLanePoint(
          lane_path_in_horizon.back()),
      forward_extend_len, /*report_issue=*/false);

  const double ref_ego_s = ref_lane_path.FirstOccurrenceOfLanePointToArclength(
      lane_path_from_pose.front());
  const double ref_anchor_s =
      ref_lane_path.ContainsLanePoint(anchor_point)
          ? ref_lane_path.FirstOccurrenceOfLanePointToArclength(anchor_point)
          : ref_ego_s;
  const double ref_neutral_s =
      ref_anchor_s +
      RoundToInt((ref_ego_s - ref_anchor_s) / kRouteStationUnitStep) *
          kRouteStationUnitStep;
  const double start_station_accum_s =
      -FloorToInt(ref_neutral_s / kRouteStationUnitStep) *
      kRouteStationUnitStep;
  const double start_station_ref_s = ref_neutral_s + start_station_accum_s;

  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
  const int n =
      CeilToInt((planning_horizon + ref_neutral_s) / kRouteStationUnitStep) + 1;
  centers.reserve(n);
  stations_boundaries.reserve(n);

  int station_idx = 0;
  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;
  const double far_station_firstthres =
      kFarStationHorizonFirstRatio * planning_horizon + ref_ego_s;
  const double far_station_secondthres =
      kFarStationHorizonSecondRatio * planning_horizon + ref_ego_s;
  constexpr double kFarRouteStationFirstStep = 2.0 * kRouteStationUnitStep;
  constexpr double kFarRouteStationSecondStep = 5.0 * kRouteStationUnitStep;
  // const double far_station_thres =
  //     kFarStationHorizonRatio * planning_horizon + ref_ego_s;
  double sample_s = start_station_ref_s;
  const double loaded_length = ref_lane_path.length();
  bool angle_diff_cutoff = false;
  st::mapping::LanePoint last_point;
  StationInfo last_station_info{};
  std::vector<int> change_index;
  // emergency lane judgement
  bool is_emergency_lane = false;
  for (const auto& seg : ref_lane_path) {
    const auto& lane_info_ptr = psmm.map_ptr()->GetLaneById(seg.lane_id);
    if ((lane_info_ptr != nullptr) &&
        (lane_info_ptr->type() == LaneType::LANE_EMERGENCY)) {
      is_emergency_lane = true;
      break;
    }
  }
  while (sample_s <= loaded_length) {
    const auto sample_lane_point = ref_lane_path.ArclengthToLanePoint(sample_s);

    // Ensure we do not create a loop in drive passage. Planner does not support
    // looped passage.
    if (station_idx == 0) {
      prev_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
    } else {
      const auto cur_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
      const double angle_diff =
          NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if (max_accumulated_angle_diff >= kDrivePassageCutOffAngleDiff ||
        min_accumulated_angle_diff <= -kDrivePassageCutOffAngleDiff) {
      angle_diff_cutoff = true;
      break;
    }
    if (station_idx > 0) {
      auto now_pt = ComputeLanePointPos(psmm, sample_lane_point);
      auto last_pt = ComputeLanePointPos(psmm, last_point);
      auto two_pt_dis =
          std::hypot((now_pt.x() - last_pt.x()), (now_pt.y() - last_pt.y()));
      if (two_pt_dis > 7.0) break;
    }
    last_point = sample_lane_point;

    station_idx++;

    SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm,
                                   sample_lane_point.lane_id());

    double speed_limit =
        override_speed_limit_mps.has_value()
            ? *override_speed_limit_mps
            : psmm.QueryLaneSpeedLimitById(sample_lane_point.lane_id());

    StationInfo station_info{
        .is_in_intersection =
            (lane_info.junction_id() != 0 &&
             lane_info.type() == LaneType::LANE_VIRTUAL_JUNCTION),
        .is_exclusive_right_turn =
            (lane_info.turn_type() == ad_byd::planning::RIGHT_TURN),
        .is_in_roundabout =
            (lane_info.type() == ad_byd::planning::LANE_ROUND_ABOUT),
        .speed_limit = speed_limit,
        .lane_type = lane_info.type(),
        .turn_type = lane_info.turn_type(),
        .split_topo = lane_info.split_topology(),
        .merge_topo = lane_info.merge_topology()};

    if (!centers.empty() && IsChangeStation(last_station_info, station_info)) {
      change_index.emplace_back(centers.size());
    }
    last_station_info = station_info;

    // Station center:
    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = ComputeLanePointPos(psmm, sample_lane_point),
        .tangent = ComputeLanePointTangent(psmm, sample_lane_point),
        .accum_s = sample_s - ref_neutral_s,
        .speed_limit = speed_limit,
        .is_virtual = (all_lanes_virtual || lane_info.IsVirtual()),
        .is_merging = (lane_info.merge_topology() ==
                           ad_byd::planning::TOPOLOGY_MERGE_LEFT ||
                       lane_info.merge_topology() ==
                           ad_byd::planning::TOPOLOGY_MERGE_RIGHT),
        .is_splitting = (lane_info.split_topology() ==
                             ad_byd::planning::TOPOLOGY_SPLIT_LEFT ||
                         lane_info.split_topology() ==
                             ad_byd::planning::TOPOLOGY_SPLIT_RIGHT),
        .is_in_intersection =
            (lane_info.junction_id() != 0 &&
             lane_info.type() == LaneType::LANE_VIRTUAL_JUNCTION),
        .has_cross_curb = false,
        .cross_curb_id = 0,
        .turn_type = lane_info.turn_type(),
        .station_info = std::move(station_info)};

    // lane_boundaries
    absl::flat_hash_set<uint64_t> lane_boundaries;
    lane_boundaries.reserve(
        lane_info.left_boundary()->lane_boundaries().size() +
        lane_info.right_boundary()->lane_boundaries().size());
    lane_boundaries.clear();
    bool is_virtual_lane = lane_info.IsBoundaryVirtual();
    bool is_update_success = false;
    for (const auto& bound : lane_info.left_boundary()->lane_boundaries()) {
      lane_boundaries.insert(bound->id());
    }
    for (const auto& bound : lane_info.right_boundary()->lane_boundaries()) {
      lane_boundaries.insert(bound->id());
    }

    // Station boundaries:
    const auto [right_lat_offset, left_lat_offset] =
        ComputeLateralLimits(centers, center);
    std::vector<StationBoundary> current_station_boundaries;
    // ignore CURB behind ego 2.5m.
    if (is_virtual_lane)
      UpdateStationCenter(psmm, center, right_lat_offset, left_lat_offset,
                          lane_boundaries, &is_update_success);
    if (is_emergency_lane) center.xy = center.lat_point(emergency_lat_offset);
    current_station_boundaries = CollectStationBoundaries(
        psmm, center, right_lat_offset, left_lat_offset, lane_boundaries,
        sample_s < ref_ego_s - 2.5, &center.has_cross_curb,
        &center.cross_curb_id);
    PostProcessStationBoundaries(center, &current_station_boundaries);
    if (!is_virtual_lane || is_update_success) {
      centers.push_back(std::move(center));
      stations_boundaries.push_back(std::move(current_station_boundaries));
    }
    if (center.accum_s > far_station_secondthres) {
      sample_s += kFarRouteStationSecondStep;
    } else if ((center.accum_s > far_station_firstthres) ||
               (center.accum_s < -kDrivePassageSmallEps)) {
      sample_s += kFarRouteStationFirstStep;
    } else {
      sample_s += kRouteStationUnitStep;
    }
    // sample_s += sample_s >= far_station_thres ? kFarRouteStationStep
    //                                           : kRouteStationUnitStep;
  }

  if (!angle_diff_cutoff &&
      lane_path_in_horizon.back() == ref_lane_path.back() && !centers.empty()) {
    // If no forward extension is made (meaning the lane path goes beyond loaded
    // map), fake the rest part of drive passage to the required length.
    const double kFarRouteStationExtendStep = 10.0 * kRouteStationUnitStep;
    double last_accum_s = centers.back().accum_s;
    double kFarRouteStationStep = kRouteStationUnitStep;
    while (last_accum_s <= planning_horizon) {
      if (last_accum_s > far_station_firstthres) {
        kFarRouteStationStep = kFarRouteStationExtendStep;
      }
      last_accum_s += kFarRouteStationStep;
      auto center = centers.back();
      center.lane_id = mapping::kInvalidElementId;
      center.fraction = 1.0;
      center.xy += center.tangent * kFarRouteStationStep;
      center.accum_s = last_accum_s;
      center.is_virtual = true;
      center.is_splitting = false;
      center.turn_type = ad_byd::planning::NO_TURN;

      centers.push_back(std::move(center));
      stations_boundaries.push_back(
          {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
           {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    }
  }
  if (centers.size() < 2) {
    std::cout << "Too few stations in BuildDrivePassage: Lane path: "
              << lane_path_from_pose.DebugString()
              << "\nExtended lane path: " << ref_lane_path.DebugString()
              << std::endl;
    return absl::FailedPreconditionError(
        absl::StrCat("Too few stations in BuildDrivePassage: Lane path: ",
                     lane_path_from_pose.DebugString(),
                     "\nExtended lane path: ", ref_lane_path.DebugString()));
  }

  // Create station vector.
  StationVector<Station> stations;
  stations.reserve(centers.size());
  for (int i = 0; i < centers.size(); ++i) {
    stations.emplace_back(std::move(centers[i]),
                          std::move(stations_boundaries[i]));
  }

  const bool reach_destination =
      lane_path_in_horizon.ContainsLanePoint(destination);
  return DrivePassage(std::move(stations), std::move(lane_path_in_horizon),
                      std::move(ref_lane_path),
                      /*lane_path_start_s=*/ref_ego_s - ref_neutral_s,
                      reach_destination, type, std::move(change_index),
                      lane_seq_info);
}

absl::StatusOr<DrivePassage> BuildDrivePassageV2(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePath& backward_extended_lane_path,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    const mapping::LanePoint& destination, bool all_lanes_virtual,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type,
    ad_byd::planning::LaneSeqInfoPtr lane_seq_info) {
  if (psmm.map_ptr() &&
      psmm.map_ptr()->type() == ad_byd::planning::MapType::BEV_MAP) {
    return BuildDrivePassageForBevMap(
        psmm, lane_path_from_pose, /*avoid_loop=*/true,
        /*avoid_notcontinuous=*/true,
        /*backward_extend_len=*/30.0, planning_horizon,
        override_speed_limit_mps, FrenetFrameType::kQtfmKdTree, lane_seq_info);
  }
  return BuildDrivePassage(
      psmm, lane_path_from_pose, backward_extended_lane_path, anchor_point,
      planning_horizon, destination, all_lanes_virtual,
      override_speed_limit_mps, FrenetFrameType::kQtfmKdTree, lane_seq_info);
}

}  // namespace st::planning
