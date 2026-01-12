

#include <algorithm>
#include <limits>
#include <memory>
#include <ostream>
#include <utility>

#include "plan_common/async/parallel_for.h"
#include "plan_common/base/macros.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/map.h"
#include "plan_common/maps/maps_helper.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/piecewise_const_function.h"
#include "plan_common/math/util.h"
#include "plan_common/gflags.h"
#include "plan_common/util/map_util.h"
namespace st::planning {

namespace {

const PiecewiseConstFunction<int, double> kModSpeedLimitFraction(
    {0, 40, 60, 70, 80, 320}, {0.3, 0.2, 0.1, 0.05, 0.0});

std::optional<double> GetDistanceBetweenLinesWithTangents(
    const Vec2d& point, PlannerSemanticMapManager::Side side,
    const Segment2d& boundary_seg, const Segment2d& norm_seg,
    double lane_heading) {
  Vec2d intersection;
  if (!norm_seg.GetIntersect(boundary_seg, &intersection)) {
    return std::nullopt;
  }
  // Align the direction of boundary with lane.
  Segment2d aligned_boundary_seg = boundary_seg;
  if (std::fabs(NormalizeAngle(aligned_boundary_seg.heading() - lane_heading)) >
      M_PI_2) {
    aligned_boundary_seg.Reverse();
  }
  double prod = aligned_boundary_seg.ProductOntoUnit(point);
  if (side == PlannerSemanticMapManager::Side::kLEFT) prod *= -1.0;
  return std::copysign(aligned_boundary_seg.DistanceTo(point), prod);
}

}  // namespace

PlannerSemanticMapManager::PlannerSemanticMapManager(
    ad_byd::planning::MapPtr map_ptr,
    ad_byd::planning::LdLiteMapPtr lite_map_ptr)
    : map_ptr_(std::move(map_ptr)), lite_map_ptr_(std::move(lite_map_ptr)) {}

PlannerSemanticMapManager::PlannerSemanticMapManager(
    ad_byd::planning::MapPtr map_ptr, PlannerSemanticMapModification modifier)
    : map_ptr_(std::move(map_ptr)), modifier_(std::move(modifier)) {}

double PlannerSemanticMapManager::QueryLaneSpeedLimitById(
    mapping::ElementId id) const {
  const auto& lane_ptr = FindLaneByIdOrNull(id);
  if (lane_ptr == nullptr) {
    return modifier_.max_speed_limit;
  }

  const auto& lane = *lane_ptr;

  const int lane_speed_limit_kph = RoundToInt(Mps2Kph(lane.speed_limit()));

  const double speed_limit_crease_factor =
      FLAGS_planner_override_lane_speed_limit_proportion == 0.0
          ? (FLAGS_planner_enable_dynamic_lane_speed_limit
                 ? kModSpeedLimitFraction(lane_speed_limit_kph)
                 : 0.0)
          : FLAGS_planner_override_lane_speed_limit_proportion;

  const double increased_lane_speed_limit =
      lane.speed_limit() * (1.0 + speed_limit_crease_factor);

  double speed_limit =
      std::min(modifier_.max_speed_limit, increased_lane_speed_limit);

  if (const auto it = modifier_.lane_speed_limit_map.find(id);
      it != modifier_.lane_speed_limit_map.end()) {
    speed_limit = std::min(it->second, speed_limit);
  }

  return speed_limit;
}

std::vector<ad_byd::planning::LaneConstPtr>
PlannerSemanticMapManager::GetLanesInRadius(const Vec2d& smooth_coord,
                                            double radius) const {
  const auto lane_vec =
      map_ptr_->FindLanesInRadius(smooth_coord.x(), smooth_coord.y(), radius);
  return lane_vec;
}

ad_byd::planning::LaneConstPtr
PlannerSemanticMapManager::GetNearestLaneWithHeading(
    const Vec2d& smooth_coord, double theta, double radius,
    double max_heading_diff) const {
  const auto found_segments = map_ptr_->FindLaneSegmentsInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);
  if (UNLIKELY(found_segments.empty())) return nullptr;

  double min_dist = std::numeric_limits<double>::infinity();
  mapping::ElementId lane_id = mapping::kInvalidElementId;
  for (const auto& segment : found_segments) {
    const auto& lane_ptr = map_ptr_->GetLaneById(segment.element_id);
    if (LIKELY(lane_ptr != nullptr)) {
      DCHECK_GE(segment.segment_id.value(), 0);
      DCHECK_LT(segment.segment_id.value(), lane_ptr->points().size() - 1);
      const Segment2d smooth_seg(
          lane_ptr->points()[segment.segment_id.value()],
          lane_ptr->points()[segment.segment_id.value() + 1]);
      const double heading_diff = NormalizeAngle(theta - smooth_seg.heading());
      if (std::fabs(heading_diff) > max_heading_diff) {
        continue;
      }
      const double dist = smooth_seg.DistanceTo(smooth_coord);
      if (dist < min_dist) {
        min_dist = dist;
        lane_id = segment.element_id;
      }
    }
  }
  return map_ptr_->GetLaneById(lane_id);
}

// TODO: Delete level_id.
bool PlannerSemanticMapManager::GetLaneProjection(
    const Vec2d& smooth_coord, mapping::ElementId lane_id,
    double* const fraction, Vec2d* const point, double* const min_dist,
    Segment2d* const segment) const {
  const auto& lane_info_ptr = FindLaneByIdOrNull(lane_id);
  if (UNLIKELY(lane_info_ptr == nullptr)) return false;

  const auto& points = lane_info_ptr->points();
  if (UNLIKELY(points.size() < 2)) return false;

  std::vector<Segment2d> segments = mapping::Vec2dToSegments(points);

  double min_distance = std::numeric_limits<double>::infinity();
  Vec2d proj_point;
  Vec2d* const point_ptr = point == nullptr ? nullptr : &proj_point;
  int min_idx = -1;
  for (int i = 0; i < segments.size(); ++i) {
    const double cur_dist = segments[i].DistanceTo(smooth_coord, point_ptr);
    if (cur_dist < min_distance) {
      min_idx = i;
      min_distance = cur_dist;
      if (point != nullptr) {
        *point = proj_point;
      }
    }
  }
  if (segment != nullptr) {
    *segment = segments[min_idx];
  }
  if (min_dist != nullptr) {
    *min_dist = min_distance;
  }

  if (fraction != nullptr) {
    double accum_s = 0.0;
    for (int i = 0; i < min_idx; ++i) {
      accum_s += segments[i].length();
    }
    accum_s += segments[min_idx].ProjectOntoUnit(smooth_coord);
    *fraction = std::clamp(accum_s / lane_info_ptr->curve_length(), 0.0, 1.0);
  }
  return true;
}

ad_byd::planning::LaneConstPtr PlannerSemanticMapManager::GetNearestLane(
    const Vec2d& smooth_coord) const {
  const auto lane =
      map_ptr_->FindNearestLane(smooth_coord.x(), smooth_coord.y());
  return lane;
}

// TODO: delete level_id.
std::optional<double> PlannerSemanticMapManager::ComputeLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id, Side side) const {
  ad_byd::planning::LaneConstPtr lane = FindLaneByIdOrNull(lane_id);
  if (lane == nullptr) return std::nullopt;

  Segment2d nearest_lane_segment;
  if (!GetLaneProjection(smooth_coord, lane_id,
                         /*fraction=*/nullptr,
                         /*point=*/nullptr,
                         /*min_dist=*/nullptr, &nearest_lane_segment)) {
    return std::nullopt;
  }

  constexpr double kMaxLaneWidth = 5.0;  // m.
  const Vec2d normal =
      Vec2d::FastUnitFromAngle(nearest_lane_segment.heading()).Perp();
  const Segment2d normal_seg(smooth_coord + normal * kMaxLaneWidth,
                             smooth_coord - normal * kMaxLaneWidth);

  const auto& boundaries =
      side == Side::kLEFT ? lane->left_boundary() : lane->right_boundary();
  std::vector<Segment2d> boundary_segments;
  for (const auto& boundary : boundaries->lane_boundaries()) {
    if (boundary == nullptr || boundary->points().size() < 2) continue;
    const auto segs = mapping::Vec2dToSegments(boundary->points());
    boundary_segments.insert(boundary_segments.end(), segs.begin(), segs.end());
  }

  std::optional<double> width;
  for (const auto& boundary_seg : boundary_segments) {
    if (boundary_seg.DistanceTo(smooth_coord) > kMaxLaneWidth) {
      continue;
    }
    const auto dist = GetDistanceBetweenLinesWithTangents(
        smooth_coord, side, boundary_seg, normal_seg,
        nearest_lane_segment.heading());
    if (!dist.has_value()) continue;
    if (!width.has_value() || std::fabs(*dist) < std::fabs(*width)) {
      width = *dist;
    }
  }
  return width;
}

// TODO: delete level id.
std::optional<double> PlannerSemanticMapManager::GetLeftLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id) const {
  return ComputeLaneWidth(smooth_coord, lane_id, Side::kLEFT);
}

// TODO: delete level id.
std::optional<double> PlannerSemanticMapManager::GetRightLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id) const {
  return ComputeLaneWidth(smooth_coord, lane_id, Side::kRIGHT);
}

std::vector<ad_byd::planning::LaneBoundaryConstPtr>
PlannerSemanticMapManager::GetLaneBoundaries(const Vec2d& smooth_coord,
                                             double radius) const {
  const auto boundary_vec = map_ptr_->FindLaneBoundariesInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_byd::planning::LaneBoundaryConstPtr> lane_boundaries;
  lane_boundaries.reserve(boundary_vec.size());
  for (const auto& lane_boundary : boundary_vec) {
    const auto& lane_boundary_ptr =
        FindLaneBoundaryByIdOrNull(lane_boundary->id());
    if (lane_boundary_ptr == nullptr) continue;
    lane_boundaries.push_back(lane_boundary_ptr);
  }
  return lane_boundaries;
}

std::vector<ad_byd::planning::RoadBoundaryConstPtr>
PlannerSemanticMapManager::GetRoadBoundaries(const Vec2d& smooth_coord,
                                             double radius) const {
  const auto boundary_vec = map_ptr_->FindRoadBoundariesInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_byd::planning::RoadBoundaryConstPtr> road_boundaries;
  road_boundaries.reserve(boundary_vec.size());
  for (const auto& road_boundary : boundary_vec) {
    const auto& road_boundary_ptr =
        FindRoadBoundaryByIdOrNull(road_boundary->id());
    if (road_boundary_ptr == nullptr) continue;
    road_boundaries.push_back(road_boundary_ptr);
  }
  return road_boundaries;
}

std::vector<ad_byd::planning::ClearAreaConstPtr>
PlannerSemanticMapManager::GetClearAreas(const Vec2d& smooth_coord,
                                         double radius) const {
  const auto clear_area_vec = map_ptr_->FindClearAreasInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_byd::planning::ClearAreaConstPtr> clear_areas;
  clear_areas.reserve(clear_area_vec.size());
  for (const auto& clear_area : clear_area_vec) {
    const auto& clear_area_ptr = FindClearAreaByIdOrNull(clear_area->id());
    if (nullptr == clear_area_ptr) continue;
    clear_areas.push_back(clear_area_ptr);
  }
  return clear_areas;
}

std::vector<ad_byd::planning::CrosswalkConstPtr>
PlannerSemanticMapManager::GetCrosswalks(const Vec2d& smooth_coord,
                                         double radius) const {
  const auto crosswalk_vec = map_ptr_->FindCrosswalksInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_byd::planning::CrosswalkConstPtr> crosswalks;
  crosswalks.reserve(crosswalk_vec.size());
  for (const auto& crosswalk : crosswalk_vec) {
    const auto& crosswalk_ptr = FindCrosswalkByIdOrNull(crosswalk->id());
    if (crosswalk_ptr == nullptr) continue;
    crosswalks.push_back(crosswalk_ptr);
  }
  return crosswalks;
}

ad_byd::planning::JunctionConstPtr
PlannerSemanticMapManager::GetNearestJunction(const Vec2d& smooth_coord) const {
  const auto& intersection =
      map_ptr_->FindNearestJunction(smooth_coord.x(), smooth_coord.y());
  return intersection;
}
std::vector<Segment2d> PlannerSemanticMapManager::GetImpassableBoundaries(
    const Vec2d& smooth_coord, double radius) const {
  const auto curb_vec = map_ptr_->FindRoadBoundarySegmentsInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);
  std::vector<Segment2d> segments;
  segments.reserve(curb_vec.size());
  for (const auto& curb_seg : curb_vec) {
    const auto& lane_boundary = FindRoadBoundaryByIdOrNull(curb_seg.element_id);
    if (lane_boundary != nullptr) {
      segments.emplace_back(
          lane_boundary->points()[curb_seg.segment_id.value()],
          lane_boundary->points()[curb_seg.segment_id.value() + 1]);
    }
  }
  return segments;
}
std::vector<ImpassableBoundaryInfo>
PlannerSemanticMapManager::GetImpassableBoundariesInfo(
    const Vec2d& smooth_coord, double radius) const {
  const auto curb_vec = map_ptr_->FindRoadBoundarySegmentsInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ImpassableBoundaryInfo> boundary_infos;
  boundary_infos.reserve(curb_vec.size());
  for (const auto& curb_seg : curb_vec) {
    const auto& lane_boundary = FindRoadBoundaryByIdOrNull(curb_seg.element_id);
    if (lane_boundary != nullptr) {
      auto& boundary_info = boundary_infos.emplace_back();
      boundary_info.segment =
          Segment2d(lane_boundary->points()[curb_seg.segment_id.value()],
                    lane_boundary->points()[curb_seg.segment_id.value() + 1]);
      boundary_info.height = lane_boundary->has_height()
                                 ? std::make_optional(lane_boundary->height())
                                 : std::nullopt;
      boundary_info.id = absl::StrFormat("CURB|%lld|%lld", curb_seg.element_id,
                                         curb_seg.segment_id);
      boundary_info.type = lane_boundary->type();
    }
  }
  return boundary_infos;
}

ad_byd::planning::LaneConstPtr PlannerSemanticMapManager::FindLaneByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetLaneById(id);
}
ad_byd::planning::LaneConstPtr
PlannerSemanticMapManager::FindCurveLaneByIdOrNull(
    mapping::ElementId id) const {
  const auto& lane = FindLaneByIdOrNull(id);
  if (!lane || lane->points().empty()) {
    return nullptr;
  }
  return lane;
}
ad_byd::planning::LaneBoundaryConstPtr
PlannerSemanticMapManager::FindLaneBoundaryByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetLaneBoundaryById(id);
}
ad_byd::planning::RoadBoundaryConstPtr
PlannerSemanticMapManager::FindRoadBoundaryByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetRoadBoundaryById(id);
}
ad_byd::planning::JunctionConstPtr
PlannerSemanticMapManager::FindJunctionByIdOrNull(mapping::ElementId id) const {
  return map_ptr_->GetJunctionById(id);
}
ad_byd::planning::StopLineConstPtr
PlannerSemanticMapManager::FindStopLineByIdOrNull(mapping::ElementId id) const {
  return map_ptr_->GetStopLineById(id);
}
ad_byd::planning::CrosswalkConstPtr
PlannerSemanticMapManager::FindCrosswalkByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetCrosswalkById(id);
}
ad_byd::planning::SpeedBumpConstPtr
PlannerSemanticMapManager::FindSpeedBumpByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetSpeedBumpById(id);
}
ad_byd::planning::ClearAreaConstPtr
PlannerSemanticMapManager::FindClearAreaByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetClearAreaById(id);
}
ad_byd::planning::SectionConstPtr
PlannerSemanticMapManager::FindSectionByIdOrNull(mapping::ElementId id) const {
  return map_ptr_->GetSectionById(id);
};
std::vector<ad_byd::planning::CrosswalkConstPtr>
PlannerSemanticMapManager::GetCrosswalksInRadius(const Vec2d& smooth_coord,
                                                 double radius) const {
  const auto crosswalk_vec = map_ptr_->FindCrosswalksInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_byd::planning::CrosswalkConstPtr> crosswalk_infos;
  crosswalk_infos.reserve(crosswalk_vec.size());
  for (const auto& crosswalk : crosswalk_vec) {
    const auto& crosswalk_ptr = FindCrosswalkByIdOrNull(crosswalk->id());
    if (crosswalk_ptr == nullptr) continue;
    crosswalk_infos.push_back(crosswalk_ptr);
  }
  return crosswalk_infos;
};
// double PlannerSemanticMapManager::GetpointsmseById(mapping::ElementId id,
//                                                    double sample_s) const {
//   const auto& lane_info_ptr = FindLaneByIdOrNull(id);
//   if (UNLIKELY(lane_info_ptr == nullptr)) return 1.0;
//   const auto& points_mse = lane_info_ptr->points_mse();
//   if (UNLIKELY(points_mse.size() < 2)) return 1.0;
//   const auto& cumulative_lengths =
//   lane_info_ptr->center_line().GetAccuLength(); if
//   (cumulative_lengths.empty()) {
//     return 1.0;
//   }
//   const int num_vertices = std::min(cumulative_lengths.size(),
//   points_mse.size()); if (num_vertices < 2) return 1.0; int i =
//   std::upper_bound(cumulative_lengths.begin(), cumulative_lengths.end(),
//                            sample_s) -
//           cumulative_lengths.begin();
//   if (i >= num_vertices) i = num_vertices - 1;
//   if (i == 0) ++i;
//   return std::max(points_mse[i - 1], points_mse[i]);
// };

void PlannerSemanticMapManager::SectionLanesFilter(
    const Vec2d& ego_pos, const double ego_theta,
    std::vector<std::pair<uint64_t, double>>& section_lanes) const {
  if (section_lanes.empty()) return;

  constexpr double kMinRoadBoundaryLength = 5.0;
  constexpr double kCurbSearchExtendOffset = 5.0;
  const double max_curb_search_radius =
      kCurbSearchExtendOffset + std::max(std::abs(section_lanes.front().second),
                                         std::abs(section_lanes.back().second));
  double left_curb_offset = std::numeric_limits<double>::lowest(),
         right_curb_offset = std::numeric_limits<double>::max();
  const auto road_boundaries =
      GetRoadBoundaries(ego_pos, max_curb_search_radius);
  for (const auto& boundary : road_boundaries) {
    if (!boundary || boundary->curve_length() < kMinRoadBoundaryLength)
      continue;
    double curb_s, curb_l, curb_heading;
    if (!boundary->line_curve().GetProjection(ego_pos, &curb_s, &curb_l))
      continue;
    if (curb_s < -2.0 || curb_s > boundary->curve_length() + 2.0) continue;
    if (!boundary->line_curve().GetHeadingFromS(curb_s, &curb_heading))
      continue;
    const double heading_diff = NormalizeAngle(ego_theta - curb_heading);
    if (std::abs(heading_diff) > M_PI / 3.0 &&
        std::abs(heading_diff) < 2.0 * M_PI / 3.0)
      continue;

    curb_l = std::abs(heading_diff) < M_PI / 2.0 ? curb_l : -curb_l;
    if (curb_l < 0.0 && left_curb_offset < curb_l) left_curb_offset = curb_l;
    if (curb_l > 0.0 && right_curb_offset > curb_l) right_curb_offset = curb_l;
    // Log2DDS::LogDataV2(
    //     "ld_lite_debug",
    //     absl::StrCat("boundary_id: ", boundary->id(), ", curve_length: ",
    //                  boundary->curve_length(), ", curb_s: ", curb_s,
    //                  ", curb_l: ", curb_l, ", heading_diff: ",
    //                  heading_diff));
  }
  // Log2DDS::LogDataV2("ld_lite_debug",
  //                    absl::StrCat("left_curb_offset: ", left_curb_offset,
  //                                 ", right_curb_offset: ",
  //                                 right_curb_offset));
  int min_idx = section_lanes.size();
  double min_lat_offset = std::numeric_limits<double>::max();
  for (int i = 0; i < section_lanes.size(); i++) {
    if (std::abs(section_lanes[i].second) < min_lat_offset) {
      min_lat_offset = std::abs(section_lanes[i].second);
      min_idx = i;
    }
  }
  if (min_idx < 0 || min_idx >= section_lanes.size()) return;

  constexpr double kMaxLaneLateralOffset = 8.0;
  std::vector<std::pair<uint64_t, double>> lanes_filter;
  lanes_filter.push_back(section_lanes[min_idx]);
  for (int i = min_idx - 1; i >= 0; i--) {
    if (std::abs(section_lanes[i].second - section_lanes[i + 1].second) >
            kMaxLaneLateralOffset ||
        section_lanes[i].second < left_curb_offset)
      break;
    lanes_filter.insert(lanes_filter.begin(), section_lanes[i]);
  }
  for (int i = min_idx + 1; i < section_lanes.size(); i++) {
    if (std::abs(section_lanes[i].second - section_lanes[i - 1].second) >
            kMaxLaneLateralOffset ||
        section_lanes[i].second > right_curb_offset)
      break;
    lanes_filter.push_back(section_lanes[i]);
  }
  section_lanes = lanes_filter;
}
void PlannerSemanticMapManager::SectionLanesFilterByGap(
    const double s,
    std::vector<std::pair<ad_byd::planning::LaneConstPtr, double>>&
        valid_lanes_with_l) const {
  if (valid_lanes_with_l.size() < 2) return;
  // check all first is valid.
  for (auto lane_cell : valid_lanes_with_l) {
    if (!lane_cell.first) return;
  }
  // find the lat nearest lane
  int nearest_idx = -1;
  double nearest_lat = std::numeric_limits<double>::max();
  for (size_t idx = 0; idx < valid_lanes_with_l.size(); idx++) {
    if (nearest_lat > abs(valid_lanes_with_l.at(idx).second)) {
      nearest_lat = abs(valid_lanes_with_l.at(idx).second);
      nearest_idx = idx;
    }
  }
  if (nearest_idx < 0 || nearest_idx >= valid_lanes_with_l.size()) {
    return;
  }
  // key: lane_id, value:<left_width, right_width>
  std::unordered_map<uint64_t, std::pair<double, double>> lane_lw_rw_map;
  auto get_lane_left_right_width =
      [&lane_lw_rw_map, s](ad_byd::planning::LaneConstPtr lane,
                           double& left_width, double& right_width) {
        if (!lane) return;
        auto llrm = lane_lw_rw_map.find(lane->id());
        if (llrm != lane_lw_rw_map.end()) {
          left_width = llrm->second.first;
          right_width = llrm->second.second;
        } else {
          lane->GetWidthAtAccumS(s, &left_width, &right_width);
          lane_lw_rw_map[lane->id()] =
              std::pair<double, double>(left_width, right_width);
        }
      };
  std::vector<std::pair<ad_byd::planning::LaneConstPtr, double>> result;
  result.reserve(valid_lanes_with_l.size());
  // the nearest lane be added.
  result.push_back(valid_lanes_with_l.at(nearest_idx));

  // add the left lanes until large gap.
  for (int idx = nearest_idx - 1; idx >= 0; idx--) {
    const auto& L_lane_data = valid_lanes_with_l.at(idx);
    const auto& R_lane_data = valid_lanes_with_l.at(idx + 1);
    ad_byd::planning::LaneConstPtr L_lane = L_lane_data.first;
    double L_lane_lat = L_lane_data.second;
    ad_byd::planning::LaneConstPtr R_lane = R_lane_data.first;
    double R_lane_lat = R_lane_data.second;
    // if the neighber lane is too close, ignore it.
    if (abs(L_lane_lat - result.front().second) < 1.0) continue;
    // if the gap is too large, it's a road split or merge.
    double L_right_lw, L_right_rw, R_right_lw, R_right_rw;
    get_lane_left_right_width(L_lane, L_right_lw, L_right_rw);
    get_lane_left_right_width(R_lane, R_right_lw, R_right_rw);
    double gap = abs(L_lane_lat - R_lane_lat) - L_right_rw - R_right_lw;
    if (gap > 1.0) {
      break;
    }
    // left lane, insert before begin.
    result.insert(result.begin(), valid_lanes_with_l.at(idx));
  }

  // add the right lanes until large gap.
  for (int idx = nearest_idx + 1; idx < valid_lanes_with_l.size(); idx++) {
    auto L_lane_data = valid_lanes_with_l.at(idx - 1);
    auto R_lane_data = valid_lanes_with_l.at(idx);
    ad_byd::planning::LaneConstPtr L_lane = L_lane_data.first;
    double L_lane_lat = L_lane_data.second;
    ad_byd::planning::LaneConstPtr R_lane = R_lane_data.first;
    double R_lane_lat = R_lane_data.second;
    // if the neighber lane is too close, ignore it.
    if (abs(R_lane_lat - result.back().second) < 1.0) continue;
    // if the gap is too large, it's a road split or merge.
    double L_right_lw, L_right_rw, R_right_lw, R_right_rw;
    get_lane_left_right_width(L_lane, L_right_lw, L_right_rw);
    get_lane_left_right_width(R_lane, R_right_lw, R_right_rw);
    double gap = abs(L_lane_lat - R_lane_lat) - L_right_rw - R_right_lw;
    if (gap > 1.0) {
      break;
    }
    // right lane, push back bebind.
    result.push_back(valid_lanes_with_l.at(idx));
  }
  valid_lanes_with_l = result;
}

std::vector<uint64_t> PlannerSemanticMapManager::GetValidLanesInRangeHdMap(
    const Vec2d& ego_pos, double look_forward_s,
    std::vector<std::string>& debug) const {
  std::stringstream ss;
  auto update_debug = [&debug, &ss]() {
    debug.push_back(ss.str());
    ss.str("");
  };

  std::vector<uint64_t> result{};
  auto map = map_ptr();
  if (!map || !map->route()) return result;
  ss << "Hd map. look_forward_s:" << look_forward_s;
  update_debug();

  const auto& navi_start = map->route()->navi_start();
  bool find_start = false;
  double s_in_lane = look_forward_s;
  // 1. find the section in look forward s (visit_section);
  ad_byd::planning::SectionConstPtr visit_section = nullptr;
  for (const auto& sec_id : map->GetRouteSectionSeq()) {
    if (!find_start) {
      if (sec_id == navi_start.section_id) {
        find_start = true;
        auto section = map->GetSectionById(sec_id);
        if (!section) return result;
        visit_section = section;
        s_in_lane -= section->topo_length() - navi_start.s_offset;
        if (s_in_lane < 0.0) {
          break;
        }
      }
      continue;
    }
    auto section = map->GetSectionById(sec_id);
    if (!section || section->lanes().empty()) {
      s_in_lane = 0.0;
      break;
    }
    visit_section = section;
    s_in_lane -= section->topo_length();
    if (s_in_lane < 0.0) break;
  }

  if (!visit_section || visit_section->lanes().empty()) {
    return result;
  }
  ss << "visit_section:" << visit_section->id();
  update_debug();
  std::vector<ad_byd::planning::LaneConstPtr> valid_lanes;
  valid_lanes.reserve(visit_section->lanes().size());
  for (const auto& lane_id : visit_section->lanes()) {
    auto lane = map->GetLaneById(lane_id);
    if (!lane) continue;
    valid_lanes.push_back(lane);
  }
  if (valid_lanes.empty()) return result;

  std::sort(
      valid_lanes.begin(), valid_lanes.end(),
      [&](ad_byd::planning::LaneConstPtr a, ad_byd::planning::LaneConstPtr b) {
        return a->lane_ind_in_section() < b->lane_ind_in_section();
      });
  auto in_black_list = [](ad_byd::planning::LaneConstPtr lane) -> bool {
    if (lane->type() == ad_byd::planning::LaneType::LANE_EMERGENCY ||
        lane->type() == ad_byd::planning::LaneType::LANE_NON_MOTOR ||
        lane->type() == ad_byd::planning::LaneType::LANE_UNKNOWN) {
      return true;
    }
    return false;
  };

  ss << "result lanes:";
  result.reserve(valid_lanes.size());
  for (auto lane : valid_lanes) {
    if (in_black_list(lane)) continue;
    ss << lane->id() << ",";
    result.push_back(lane->id());
  }
  update_debug();
  return result;
}
std::vector<uint64_t> PlannerSemanticMapManager::GetValidLanesInRangeBevMap(
    const Vec2d& ego_pos, const double look_forward_s,
    std::vector<std::string>& debug) const {
  std::stringstream ss;
  auto update_debug = [&debug, &ss]() {
    debug.push_back(ss.str());
    ss.str("");
  };
  // funciton: Get the lanes' number in s along the route.
  // if return 0, means error.
  // if s beyonds the farest valid range, return the farest valid range.
  std::vector<uint64_t> result{};
  auto map = map_ptr();
  if (!map || !map->route()) return result;
  const auto& navi_start = map->route()->navi_start();
  bool find_start = false;
  double s_in_lane = look_forward_s;
  ss << "BEV_map. look_forward_s:" << look_forward_s;
  update_debug();
  // 1. find the section in look forward s (visit_section);
  ad_byd::planning::SectionConstPtr visit_section = nullptr;
  for (const auto& sec_id : map->GetRouteSectionSeq()) {
    if (!find_start) {
      if (sec_id == navi_start.section_id) {
        auto section = map->GetSectionById(sec_id);
        if (!section) return result;
        auto lane = map->GetLaneById(section->get_navi_priority_lane_id());
        if (!lane || !lane->center_line().IsValid()) return result;
        double s, l;
        if (!lane->center_line().GetProjection(ego_pos, &s, &l)) {
          return result;
        }
        find_start = true;
        visit_section = section;
        s_in_lane -= lane->curve_length() - s;
        ss << "navi_start: lane_id=" << lane->id() << ",proj_s=" << s
           << ",curve_length=" << lane->curve_length()
           << ",s_in_lane=" << s_in_lane;
        update_debug();
        if (s_in_lane < 0.0) {
          break;
        }
      }
      continue;
    }
    auto section = map->GetSectionById(sec_id);
    bool stop = false;
    if (!section || section->lanes().empty()) {
      stop = true;
    }
    auto prio_lane = map->GetLaneById(section->get_navi_priority_lane_id());
    if (!prio_lane || !prio_lane->center_line().IsValid()) {
      stop = true;
    }
    if (stop) {
      s_in_lane = 0.0;
      break;
    }
    visit_section = section;
    s_in_lane -= prio_lane->curve_length();
    if (s_in_lane < 0.0) break;
  }

  if (!visit_section) return result;
  ad_byd::planning::LaneConstPtr visit_lane =
      map->GetLaneById(visit_section->get_navi_priority_lane_id());
  if (!visit_lane || !visit_lane->center_line().IsValid()) return result;
  ss << "visit section:" << visit_section->id()
     << ", visit lane:" << visit_lane->id() << ", s_in_lane:" << s_in_lane;
  update_debug();

  // 2. find the valid_lanes in visit_section.
  if (visit_section->lanes().empty()) return result;
  std::vector<ad_byd::planning::LaneConstPtr> valid_lanes;
  valid_lanes.reserve(visit_section->lanes().size());
  for (const auto& lane_id : visit_section->lanes()) {
    auto lane = map->GetLaneById(lane_id);
    if (!lane || !lane->center_line().IsValid()) continue;
    valid_lanes.push_back(lane);
  }
  if (valid_lanes.empty()) return result;
  std::sort(
      valid_lanes.begin(), valid_lanes.end(),
      [&](ad_byd::planning::LaneConstPtr a, ad_byd::planning::LaneConstPtr b) {
        return a->lane_ind_in_section() < b->lane_ind_in_section();
      });
  ss << "valid lanes:";
  for (auto lane : valid_lanes) {
    ss << lane->id() << ",";
  }
  update_debug();

  // 4. make a virtual ego positon and theta, the get the s, l values.
  // if s_in_lane == 0.0, visit_point is in the end of visit_lane.
  double visited_s = visit_lane->center_line().length();
  if (s_in_lane < 0.0) {
    visited_s = std::max(0.0, visit_lane->center_line().length() + s_in_lane);
  }
  double visited_s2;
  if (visited_s > 0.0) {
    visited_s2 = std::max(0.0, visited_s - 5.0);
    std::swap(visited_s, visited_s2);
  } else {
    visited_s2 = std::min(visit_lane->center_line().length(), visited_s + 5.0);
  }

  auto virtual_ego = visit_lane->center_line().GetPointAtS(visited_s);
  auto p2 = visit_lane->center_line().GetPointAtS(visited_s2);
  double virtual_theta =
      atan2(p2.y() - virtual_ego.y(), p2.x() - virtual_ego.x());
  ss << "virtual_ego:(x:" << virtual_ego.x() << ",y:" << virtual_ego.y()
     << ",theta:" << virtual_theta << ")";
  update_debug();
  // 5. get the lanes in visit section, and sort them by l.
  std::vector<std::pair<ad_byd::planning::LaneConstPtr, double>>
      valid_lanes_with_l;
  valid_lanes_with_l.reserve(valid_lanes.size());
  const double drop_s_limit = std::min(2.0 + abs(look_forward_s) * 0.05, 15.0);
  for (const auto lane : valid_lanes) {
    double s, l;
    if (!lane->center_line().GetProjection(virtual_ego, &s, &l)) continue;
    if (s < -drop_s_limit || s > lane->curve_length() + drop_s_limit) continue;
    ss << "SL lane:" << lane->id() << ", s:" << s << ", l:" << l
       << ", curve_length:" << lane->curve_length();
    update_debug();
    valid_lanes_with_l.emplace_back(lane, l);
  }
  // sort: from left to right.
  std::sort(valid_lanes_with_l.begin(), valid_lanes_with_l.end(),
            [&](const std::pair<ad_byd::planning::LaneConstPtr, double>& a,
                const std::pair<ad_byd::planning::LaneConstPtr, double>& b) {
              return a.second < b.second;
            });

  SectionLanesFilterByGap(visited_s, valid_lanes_with_l);

  // remove lane on the right of black list lane.
  auto in_black_list = [](ad_byd::planning::LaneConstPtr lane) -> bool {
    if (lane->type() == ad_byd::planning::LaneType::LANE_EMERGENCY ||
        lane->type() == ad_byd::planning::LaneType::LANE_NON_MOTOR ||
        lane->type() == ad_byd::planning::LaneType::LANE_UNKNOWN) {
      return true;
    }
    return false;
  };
  std::vector<std::pair<uint64_t, double>> remain_section_lanes;
  remain_section_lanes.reserve(valid_lanes_with_l.size());
  for (const auto lane : valid_lanes_with_l) {
    if (in_black_list(lane.first)) {
      break;
    }
    remain_section_lanes.emplace_back(lane.first->id(), lane.second);
  }
  ss << "remain lanes:";
  for (auto lane : remain_section_lanes) {
    ss << lane.first << ",";
  }
  update_debug();
  // 6. filting the lanes based on the virtual_ego.
  SectionLanesFilter(virtual_ego, virtual_theta, remain_section_lanes);
  result.reserve(remain_section_lanes.size());
  ss << "result lanes:";
  for (auto lane : remain_section_lanes) {
    ss << lane.first << ",";
    result.push_back(lane.first);
  }
  update_debug();
  return result;
}
std::vector<uint64_t> PlannerSemanticMapManager::GetValidLanesInRange(
    const Vec2d& ego_pos, double look_forward_s) const {
  std::vector<uint64_t> result{};
  auto map = map_ptr();
  if (!map || !map->route()) return result;
  std::vector<std::string> debug;
  if (map->type() == ad_byd::planning::MapType::HD_MAP) {
    result = GetValidLanesInRangeHdMap(ego_pos, look_forward_s, debug);
  } else {
    result = GetValidLanesInRangeBevMap(ego_pos, look_forward_s, debug);
  }
  Log2DDS::LogDataV2("lane_numbers", debug);
  return result;
}
}  // namespace st::planning
