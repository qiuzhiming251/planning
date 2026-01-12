

#include "plan_common/maps/semantic_map_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include "onboard/container/strong_int.h"
// IWYU pragma: no_include <memory>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "plan_common/log.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "google/protobuf/reflection.h"
//#include "map_geometry.pb.h"

//#include "lite/check.h"
//#include "lite/logging.h"
//#include "lite/qissue_trans.h"

#include "plan_common/maps/maps_helper.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/util.h"
//#include "q_issue.pb.h"
#include "plan_common/util/file_util.h"
//#include "plan_common/util/source_location.h"
#include "plan_common/maps/map.h"
#include "plan_common/util/status_macros.h"
//#include "offboard/mapping/quill/polyline_smoother.h"

namespace st::mapping {

double GetLaneDiscomfort(double curveture, double speed, double arch_length) {
  speed = std::abs(speed);
  if (speed < kEpsilon) return 0.0;
  return speed * speed * curveture * log(arch_length / speed + 1.0);
}

void AssignSemanticMapNewId(
    const std::map<std::pair<int, ElementId>, ElementId>& feature_id_table,
    ::google::protobuf::Message* message, int map_index) {
  CHECK(message);
  const auto* message_descriptor = message->GetDescriptor();
  const auto* message_reflection = message->GetReflection();
  for (int i = 0; i < message_descriptor->field_count(); ++i) {
    const auto* field_descriptor = message_descriptor->field(i);
    const auto cpp_type = field_descriptor->cpp_type();
    if (::google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE == cpp_type) {
      if (field_descriptor->is_repeated()) {
        for (int j = 0;
             j < message_reflection->FieldSize(*message, field_descriptor);
             ++j) {
          AssignSemanticMapNewId(feature_id_table,
                                 message_reflection->MutableRepeatedMessage(
                                     message, field_descriptor, j),
                                 map_index);
        }
      } else {
        if (message_reflection->HasField(*message, field_descriptor)) {
          AssignSemanticMapNewId(
              feature_id_table,
              message_reflection->MutableMessage(message, field_descriptor),
              map_index);
        }
      }
    } else {
      if (::google::protobuf::FieldDescriptor::TYPE_SFIXED64 ==
          field_descriptor->type()) {
        if (field_descriptor->is_repeated()) {
          for (int j = 0;
               j < message_reflection->FieldSize(*message, field_descriptor);
               ++j) {
            const ElementId old_id(message_reflection->GetRepeatedInt64(
                *message, field_descriptor, j));
            if (kInvalidElementId == old_id) continue;
            message_reflection->SetRepeatedInt64(
                message, field_descriptor, j,
                feature_id_table.at({map_index, old_id}));
          }
        } else {
          if (message_reflection->HasField(*message, field_descriptor)) {
            const ElementId old_id(
                message_reflection->GetInt64(*message, field_descriptor));
            if (kInvalidElementId == old_id) continue;
            message_reflection->SetInt64(
                message, field_descriptor,
                feature_id_table.at({map_index, old_id}));
          }
        }
      }
    }
  }
}

bool CheckResampleLanePoints(const ad_byd::planning::Lane& lane,
                             double start_fraction, double end_fraction) {
  if (end_fraction < start_fraction) return false;
  if (start_fraction < 0) return false;
  if (start_fraction > 1) return false;
  if (end_fraction < 0) return false;
  if (end_fraction > 1) return false;

  const double start_cum_len =
      start_fraction * lane.center_line().GetAccuLength().back();
  auto start_it =
      std::upper_bound(lane.center_line().GetAccuLength().begin(),
                       lane.center_line().GetAccuLength().end(), start_cum_len);
  int start_idx = start_it - lane.center_line().GetAccuLength().begin();
  if (start_idx <= 0) return false;

  const double end_cum_len =
      end_fraction * lane.center_line().GetAccuLength().back();
  auto end_it =
      std::lower_bound(lane.center_line().GetAccuLength().begin(),
                       lane.center_line().GetAccuLength().end(), end_cum_len);
  int end_idx = end_it - lane.center_line().GetAccuLength().begin();
  if (end_idx <= 0) return false;
  if (end_idx >= lane.center_line().GetAccuLength().size()) return false;
  return true;
}

std::vector<Vec2d> ResampleLanePoints(const ad_byd::planning::Lane& lane,
                                      double start_fraction,
                                      double end_fraction,
                                      std::vector<double>* cumulative_lengths) {
  CHECK_GT(end_fraction, start_fraction);
  CHECK_GE(start_fraction, 0.0);
  CHECK_LE(start_fraction, 1.0);
  CHECK_GE(end_fraction, 0.0);
  CHECK_LE(end_fraction, 1.0);

  const double start_cum_len =
      start_fraction * lane.center_line().GetAccuLength().back();
  auto start_it =
      std::upper_bound(lane.center_line().GetAccuLength().begin(),
                       lane.center_line().GetAccuLength().end(), start_cum_len);
  int start_idx = start_it - lane.center_line().GetAccuLength().begin();
  CHECK_GT(start_idx, 0);
  const auto start_point =
      Lerp(lane.points()[start_idx - 1], lane.points()[start_idx],
           (start_cum_len - *(start_it - 1)) / (*start_it - *(start_it - 1)));

  const double end_cum_len =
      end_fraction * lane.center_line().GetAccuLength().back();
  auto end_it =
      std::lower_bound(lane.center_line().GetAccuLength().begin(),
                       lane.center_line().GetAccuLength().end(), end_cum_len);
  int end_idx = end_it - lane.center_line().GetAccuLength().begin();
  std::vector<Vec2d> resampled_points;
  if (end_idx == 0) {
    resampled_points.push_back(lane.points()[0]);
    return resampled_points;
  }
  CHECK_LT(end_idx, lane.center_line().GetAccuLength().size());
  const auto end_point =
      Lerp(lane.points()[end_idx - 1], lane.points()[end_idx],
           (end_cum_len - *(end_it - 1)) / (*end_it - *(end_it - 1)));

  const double start_s = lane.curve_length() * start_fraction;
  const double end_s = lane.curve_length() * end_fraction;

  if (end_idx > start_idx) {
    resampled_points.assign(lane.points().begin() + start_idx,
                            lane.points().begin() + end_idx);
    if (cumulative_lengths != nullptr) {
      cumulative_lengths->assign(
          lane.center_line().GetAccuLength().begin() + start_idx,
          lane.center_line().GetAccuLength().begin() + end_idx);
    }
  }

  constexpr double kMinDistance = 1.0;  // m.
  if (resampled_points.empty() || resampled_points.front().DistanceSquareTo(
                                      start_point) > Sqr(kMinDistance)) {
    resampled_points.insert(resampled_points.begin(), start_point);
    if (cumulative_lengths != nullptr) {
      cumulative_lengths->insert(cumulative_lengths->begin(), start_s);
    }
  } else {
    resampled_points.front() = start_point;
    if (cumulative_lengths != nullptr) {
      cumulative_lengths->front() = start_s;
    }
  }

  if (resampled_points.size() < 2 ||
      resampled_points.back().DistanceSquareTo(end_point) > Sqr(kMinDistance)) {
    resampled_points.push_back(end_point);
    if (cumulative_lengths != nullptr) {
      cumulative_lengths->push_back(end_s);
    }
  } else {
    resampled_points.back() = end_point;
    if (cumulative_lengths != nullptr) {
      cumulative_lengths->back() = end_s;
    }
  }

  return resampled_points;
}

bool IsNeighborLane(bool left, const ad_byd::planning::Lane& source_lane,
                    ElementId target_id) {
  const auto& neighbors =
      left ? source_lane.left_lane_id() : source_lane.right_lane_id();
  if (neighbors == 0) return false;
  if (neighbors == target_id) {
    return true;
  } else {
    return false;
  }
}

bool ContainsNeighborLane(bool left, const ad_byd::planning::Lane& source_lane,
                          const std::deque<ElementId>& target_ids) {
  for (const auto id : target_ids) {
    if (IsNeighborLane(left, source_lane, id)) {
      return true;
    }
  }
  return false;
}

std::pair<double, double> GetNeighborRange(
    bool left, const LanePath& target,
    const ad_byd::planning::Lane& source_lane) {
  for (int i = 0; i < target.size(); ++i) {
    if (IsNeighborLane(left, source_lane, target.lane_id(i))) {
      const auto segment = target.lane_segment(i);
      return {segment.start_fraction, segment.end_fraction};
    }
  }
  return {.0, .0};
}

bool HasNeighborPointOnPath(const ad_byd::planning::Map& map, bool left,
                            const LanePoint& source_point,
                            const LanePath& target, LanePoint* neighbor_point) {
  const auto& source_lane = map.GetLaneById(source_point.lane_id());
  for (const auto& lane_seg : target) {
    if (mapping::IsNeighborLane(left, *source_lane, lane_seg.lane_id)) {
      if (neighbor_point) {
        const auto& target_lane_ptr = map.GetLaneById(lane_seg.lane_id);
        if (target_lane_ptr == nullptr) {
          continue;
        }
        if (std::fabs(source_lane->curve_length() -
                      target_lane_ptr->curve_length()) <
            source_lane->curve_length() * 1E-2) {
          *neighbor_point =
              LanePoint(lane_seg.lane_id, source_point.fraction());
        } else {
          *neighbor_point = LanePoint(
              lane_seg.lane_id, std::max(lane_seg.start_fraction,
                                         std::min(source_point.fraction(),
                                                  lane_seg.end_fraction)));
        }
      }
      return true;
    }
  }
  return false;
}

bool IsOutgoingLane(const ad_byd::planning::Lane& source_lane,
                    ElementId out_lane_id) {
  for (const auto idx : source_lane.next_lane_ids()) {
    if (out_lane_id == idx) {
      return true;
    }
  }
  return false;
}

bool IsRightMostLane(const ad_byd::planning::Map& map,
                     const LanePath& lane_path, double s) {
  double accum_len = 0.0;
  for (const auto& seg : lane_path) {
    accum_len += seg.length();
    if (accum_len >= s) {
      return map.GetLaneById(seg.lane_id)->right_lane_id() == 0;
    }
  }
  return map.GetLaneById(lane_path.back().lane_id())->right_lane_id() == 0;
}

bool IsRightMostLane(const ad_byd::planning::Map& map,
                     const LanePath& lane_path) {
  return map.GetLaneById(lane_path.back().lane_id())->right_lane_id() == 0;
}

bool IsRightMostLane(const ad_byd::planning::Map& map,
                     const ElementId lane_id) {
  return map.GetLaneById(lane_id)->right_lane_id() == 0;
}

bool IsLeftMostLane(const ad_byd::planning::Map& map, const LanePath& lane_path,
                    double s) {
  double accum_len = 0.0;
  for (const auto& seg : lane_path) {
    accum_len += seg.length();
    if (accum_len >= s) {
      return map.GetLaneById(seg.lane_id)->left_lane_id() == 0;
    }
  }
  return map.GetLaneById(lane_path.back().lane_id())->left_lane_id() == 0;
}

bool IsLeftMostLane(const ad_byd::planning::Map& map, const ElementId lane_id) {
  return map.GetLaneById(lane_id)->left_lane_id() == 0;
}

bool IsLanePathBlockedByBox2d(const ad_byd::planning::Map& map,
                              const Box2d& box, const LanePath& lane_path,
                              double lat_thres) {
  ASSIGN_OR_DIE(const auto ff,
                BuildBruteForceFrenetFrame(SampleLanePathPoints(map, lane_path),
                                           /*down_sample_raw_points=*/true));
  bool reached_left = false, reached_right = false;
  for (const auto& pt : box.GetCornersCounterClockwise()) {
    const double lat_offset = ff.XYToSL(pt).l;
    if (lat_offset >= lat_thres) reached_left = true;
    if (lat_offset <= -lat_thres) reached_right = true;

    if (reached_left && reached_right) return true;
  }
  return false;
}

std::vector<Vec2d> SampleLanePathPoints(const ad_byd::planning::Map& map,
                                        const LanePath& lane_path) {
  std::vector<Vec2d> sample_points;
  for (const auto& seg : lane_path) {
    if (seg.end_fraction <= seg.start_fraction + kEpsilon) continue;

    const auto& lane_info_ptr = map.GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) {
      break;
    }

    auto tmp_points =
        (seg.start_fraction == 0.0 && seg.end_fraction == 1.0)
            ? lane_info_ptr->points()
            : mapping::ResampleLanePoints(*lane_info_ptr, seg.start_fraction,
                                          seg.end_fraction,
                                          /*cumulative_lengths=*/nullptr);

    if (!sample_points.empty()) sample_points.pop_back();
    sample_points.insert(sample_points.end(), tmp_points.begin(),
                         tmp_points.end());
  }
  return sample_points;
}

std::pair<int, double> ComputePosition(const std::vector<Vec2d>& points,
                                       double fraction) {
  std::vector<double> dist;
  dist.reserve(points.size());
  dist.push_back(0.0);
  for (auto it = points.cbegin() + 1; it != points.cend(); it++) {
    dist.push_back(it->DistanceTo(*(it - 1)) + dist.back());
  }
  const double pos_dist = dist.back() * fraction;
  const auto pos_it = std::lower_bound(dist.begin(), dist.end(), pos_dist);

  const auto pos = std::distance(dist.begin(), pos_it);
  if (pos == 0) {
    return {0, 0.0};
  }
  const auto t = (pos_dist - *(pos_it - 1)) / (*pos_it - *(pos_it - 1));
  return {pos - 1, t};
}

std::vector<Vec2d> TruncatePoints(const std::vector<Vec2d>& points,
                                  double start_fraction, double end_fraction,
                                  double epsilon) {
  auto points_tmp = points;
  if (end_fraction <= start_fraction + epsilon) {
    return {};
  }
  if (start_fraction == 0.0 && end_fraction == 1.0) {
    return points_tmp;
  }
  // interpolation
  int start_pos = 0;
  int end_pos = 0;
  Vec2d start_lerp;
  Vec2d end_lerp;
  {
    const auto [pos, fraction] = ComputePosition(points_tmp, start_fraction);
    if (pos >= points_tmp.size() - 1) {
      return {};
    }
    // Special case: fraction == 0 or 1. Only happened when precision lost.
    if (std::fabs(1 - fraction) < epsilon) {
      start_lerp = points_tmp[pos + 1];
      start_pos = pos + 1;
    } else {
      start_lerp = Lerp(points_tmp[pos], points_tmp[pos + 1], fraction);
      start_pos = pos;
    }
  }
  {
    const auto [pos, fraction] = ComputePosition(points_tmp, end_fraction);
    if (pos >= points_tmp.size() - 1) {
      end_pos = points_tmp.size() - 1;
      end_lerp = points_tmp.back();
    } else {
      if (fraction < epsilon) {
        end_lerp = points_tmp[pos];
        end_pos = pos;
      } else {
        end_lerp = Lerp(points_tmp[pos], points_tmp[pos + 1], fraction);
        end_pos = pos + 1;
      }
    }
  }
  points_tmp[start_pos] = start_lerp;
  points_tmp[end_pos] = end_lerp;
  VLOG(4) << "start_pos:" << start_pos << ", end_pos:" << end_pos
          << ", point[end_pos]:" << points_tmp[end_pos];
  return std::vector<Vec2d>(points_tmp.begin() + start_pos,
                            points_tmp.begin() + end_pos + 1);
}

absl::StatusOr<SamplePathPointsResult> SampleLanePathProtoPoints(
    const ad_byd::planning::Map& map, const LanePathProto& lane_path) {
  std::vector<Vec2d> sample_points;
  bool is_partial = false;
  std::string message = "";
  constexpr auto kFractionEpsilon = 1E-6;
  for (int i = 0; i < lane_path.lane_ids().size(); i++) {
    const auto lane_id = lane_path.lane_ids()[i];
    const auto& lane_info_ptr = map.GetLaneById(ElementId(lane_id));
    if (lane_info_ptr == nullptr) {
      is_partial = true;
      message = absl::StrCat("Cannot find lane_id:", lane_id);
      break;
    }
    const double start_fraction = (i == 0) ? lane_path.start_fraction() : 0.0;
    const double end_fraction =
        (i == lane_path.lane_ids().size() - 1) ? lane_path.end_fraction() : 1.0;
    if (end_fraction <= start_fraction + kFractionEpsilon) continue;
    auto tmp_points = TruncatePoints(lane_info_ptr->points(), start_fraction,
                                     end_fraction, kFractionEpsilon);
    if (!sample_points.empty()) sample_points.pop_back();
    sample_points.insert(sample_points.end(), tmp_points.begin(),
                         tmp_points.end());
  }

  return SamplePathPointsResult{.points = std::move(sample_points),
                                .is_partial = is_partial,
                                .message = std::move(message)};
}

std::vector<LanePoint> LanePathToLanePoints(const LanePath& lane_path) {
  std::vector<LanePoint> lane_points;
  for (const auto& seg : lane_path) {
    if (seg.end_fraction <= seg.start_fraction) continue;
    lane_points.push_back(LanePoint(seg.lane_id, seg.start_fraction));
    lane_points.push_back(LanePoint(seg.lane_id, seg.end_fraction));
  }
  return lane_points;
}

std::vector<Polygon2d> SampleLaneBoundaryPolyLine(
    const ad_byd::planning::Lane& left_lane_info,
    const ad_byd::planning::Lane& right_lane_info, double start_fraction,
    double end_fraction) {
  constexpr double kStepS = 10.0;  // m.

  const double left_len =
      left_lane_info.curve_length() * (end_fraction - start_fraction);
  const double right_len =
      right_lane_info.curve_length() * (end_fraction - start_fraction);

  const int n =
      Max(1, RoundToInt(left_len / kStepS), RoundToInt(right_len / kStepS));

  const double left_step_fraction =
      (left_len / n) / left_lane_info.curve_length();
  const double right_step_fraction =
      (right_len / n) / right_lane_info.curve_length();

  std::vector<Polygon2d> polygons;
  polygons.reserve(n);

  const auto lerp_point = [](const ad_byd::planning::Lane& lane_info,
                             double fraction, int sign) {
    Vec2d tan;
    lane_info.GetTangent(fraction, &tan);
    const Vec2d nor = tan.Perp();
    return lane_info.LerpPointFromFraction(fraction) +
           nor * (sign * kDefaultMapLaneWidth * 0.5);
  };

  double left_fraction = start_fraction;
  double right_fraction = start_fraction;
  for (int i = 0; i < n; ++i) {
    const auto p0 = lerp_point(left_lane_info, Min(left_fraction, 1.0), 1);
    const auto p1 = lerp_point(left_lane_info,
                               Min(left_fraction + left_step_fraction, 1.0), 1);
    const auto p2 = lerp_point(
        right_lane_info, Min(right_fraction + right_step_fraction, 1.0), -1);
    const auto p3 = lerp_point(right_lane_info, Min(right_fraction, 1.0), -1);

    polygons.emplace_back(Polygon2d({p0, p1, p2, p3}));

    left_fraction += left_step_fraction;
    right_fraction += right_step_fraction;
  }

  return polygons;
}

absl::StatusOr<LanePath> ClampLanePathFromPos(const ad_byd::planning::Map& map,
                                              const LanePath& lane_path,
                                              const Vec2d& pos) {
  if (lane_path.IsEmpty()) {
    return absl::InvalidArgumentError("Empty lane path.");
  }

  const auto points = mapping::SampleLanePathPoints(map, lane_path);
  ASSIGN_OR_RETURN(const auto ff, BuildBruteForceFrenetFrame(
                                      points, /*down_sample_raw_points=*/true));
  const auto sl = ff.XYToSL(pos);

  if (sl.s >= lane_path.length()) {
    return absl::NotFoundError(
        absl::StrFormat("Queried pos %s beyond lane path %s.",
                        pos.DebugString(), lane_path.DebugString()));
  }

  return lane_path.AfterArclength(sl.s);
}

}  // namespace st::mapping
