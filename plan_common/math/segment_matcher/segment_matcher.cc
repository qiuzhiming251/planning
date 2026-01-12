

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ostream>

#include "absl/meta/type_traits.h"
#include "plan_common/math/segment_matcher/segment_matcher.h"
#include "plan_common/math/util.h"
//#include "plan_common/util/map_util.h"

namespace st {
namespace {

bool NearestSegsIndex(absl::Span<const AABoxInfo> seg_info, double x, double y,
                      int* index) {
  CHECK_NOTNULL(index);
  if (seg_info.empty()) return false;
  double nearest_dist = std::numeric_limits<double>::infinity();
  int nearest_index = 0;
  for (size_t i = 0; i < seg_info.size(); ++i) {
    const double dist_square = seg_info[i].DistanceSquareTo({x, y});
    if (dist_square < nearest_dist) {
      nearest_dist = dist_square;
      nearest_index = i;
    }
  }
  *index = nearest_index;
  return true;
}

}  // namespace

SegmentMatcher::SegmentMatcher(absl::Span<const Vec2d> points) {
  aa_boxes_info_.clear();
  segments_.clear();
  aa_boxes_info_.reserve(points.size() - 1);
  segments_.reserve(points.size() - 1);
  for (int i = 0; i + 1 < points.size(); ++i) {
    const Segment2d seg(points[i], points[i + 1]);
    aa_boxes_info_.emplace_back(seg, i);
    segments_.emplace_back(&aa_boxes_info_.back().segment());
  }
  index_flag_ = true;
}

SegmentMatcher::SegmentMatcher(const std::vector<Segment2d>& segments) {
  aa_boxes_info_.clear();
  aa_boxes_info_.reserve(segments.size());
  segments_.reserve(segments.size());
  for (int i = 0; i < segments.size(); ++i) {
    aa_boxes_info_.emplace_back(segments[i], i);
    segments_.emplace_back(&aa_boxes_info_.back().segment());
  }
  index_flag_ = true;
}

SegmentMatcher::SegmentMatcher(
    absl::Span<const std::pair<std::string, Segment2d>> named_segments) {
  segments_.clear();
  segment_names_.clear();
  aa_boxes_info_.clear();
  segment_map_.clear();
  segments_.reserve(named_segments.size());
  segment_names_.reserve(named_segments.size());
  aa_boxes_info_.reserve(named_segments.size());
  segment_map_.reserve(named_segments.size());
  for (const auto& seg : named_segments) {
    aa_boxes_info_.emplace_back(seg.second, segment_names_.size());
    segments_.emplace_back(&aa_boxes_info_.back().segment());
    const auto [it, _] =
        segment_map_.insert(std::make_pair(seg.first, segments_.back()));
    segment_names_.emplace_back(&(it->first));
  }
  CHECK_EQ(segment_map_.size(), named_segments.size())
      << " segments id duplicate.";
  named_flag_ = true;
}

const Segment2d* SegmentMatcher::GetSegmentByIndex(int index) const {
  CHECK(index_flag_);
  return index + 1 > segments_.size() ? nullptr : segments_[index];
}

const Segment2d* SegmentMatcher::GetSegmentById(const std::string& id) const {
  CHECK(named_flag_);
  // TODO
  return nullptr;
  // const auto* it = FindOrNull(segment_map_, id);
  // return it == nullptr ? nullptr : *it;
}

bool SegmentMatcher::GetProjection(double x, double y, bool is_clamp,
                                   double* accumulated_s,
                                   double* lateral) const {
  CHECK(index_flag_);
  CHECK_NOTNULL(accumulated_s);
  CHECK_NOTNULL(lateral);
  int idx = 0;
  if (!GetNearestSegmentIndex(x, y, &idx)) return false;
  const auto& nearest_seg = *segments_[idx];
  double length = 0.0;
  for (int i = 0; i < idx; ++i) length += segments_[i]->length();
  double proj = nearest_seg.ProjectOntoUnit({x, y});
  const double prod = nearest_seg.ProductOntoUnit({x, y});
  proj = is_clamp ? std::clamp<double>(proj, 0, nearest_seg.length()) : proj;
  *accumulated_s = length + proj;
  *lateral = prod;
  return true;
}

bool SegmentMatcher::GetProjection(double x, double y, st::Vec2d* nearest_point,
                                   double* accumulated_s, double* min_dist,
                                   Segment2d* segment) const {
  CHECK(index_flag_);
  int idx = 0;
  if (!GetNearestSegmentIndex(x, y, &idx)) return false;
  const auto& nearest_seg = *segments_[idx];
  const double dist = nearest_seg.DistanceTo({x, y}, nearest_point);
  if (min_dist != nullptr) *min_dist = dist;
  if (accumulated_s != nullptr) {
    double length = 0.0;
    for (int i = 0; i < idx; ++i) length += segments_[i]->length();
    *accumulated_s = length + nearest_seg.ProjectOntoUnit({x, y});
  }
  if (segment != nullptr) *segment = *segments_[idx];
  return true;
}

bool SegmentMatcher::GetNearestSegmentIndexWithHeading(
    double x, double y, double theta, double max_radius,
    double max_heading_diff, int* nearest_index) const {
  CHECK(index_flag_);
  CHECK_NOTNULL(nearest_index);
  std::vector<AABoxInfo> partial_aaboxes;
  for (const int index : GetSegmentIndexInRadius(x, y, max_radius)) {
    const double heading_diff =
        NormalizeAngle(theta - segments_[index]->heading());
    if (std::abs(heading_diff) > max_heading_diff) continue;
    partial_aaboxes.emplace_back(*segments_[index], index);
  }
  int index_in_partial = 0;
  if (!NearestSegsIndex(partial_aaboxes, x, y, &index_in_partial)) {
    return false;
  }
  *nearest_index = partial_aaboxes[index_in_partial].index();
  return true;
}

// bool SegmentMatcher::GetNearestSegmentIdWithHeading(
//     double x, double y, double theta, double max_radius,
//     double max_heading_diff, std::string* nearest_id) const {
//   CHECK(named_flag_);
//   CHECK_NOTNULL(nearest_id);
//   std::vector<AABoxInfo> partial_aaboxes;
//   std::vector<std::string> partial_names;
//   for (const auto& id : GetSegmentIdInRadius(x, y, max_radius)) {
//     const double heading_diff =
//         NormalizeAngle(theta - GetSegmentById(id)->heading());
//     if (std::abs(heading_diff) > max_heading_diff) continue;
//     partial_aaboxes.emplace_back(*GetSegmentById(id));
//     partial_names.emplace_back(id);
//   }
//   CHECK_EQ(partial_aaboxes.size(), partial_names.size());
//   int index_in_partial = 0;
//   if (!NearestSegsIndex(partial_aaboxes, x, y, &index_in_partial)) {
//     return false;
//   }
//   *nearest_id = partial_names[index_in_partial];
//   return true;
// }

// std::vector<std::string> SegmentMatcher::GetSegmentIdInRadiusWithHeading(
//     double x, double y, double theta, double max_radius,
//     double max_heading_diff) const {
//   CHECK(named_flag_);
//   std::vector<std::string> ids_with_in_radius;
//   for (const auto& id : GetSegmentIdInRadius(x, y, max_radius)) {
//     const double heading_diff =
//         NormalizeAngle(theta - GetSegmentById(id)->heading());
//     if (std::fabs(heading_diff) > max_heading_diff) continue;
//     ids_with_in_radius.emplace_back(id);
//   }
//   return ids_with_in_radius;
// }

std::vector<int> SegmentMatcher::GetSegmentIndexInRadiusWithHeading(
    double x, double y, double theta, double max_radius,
    double max_heading_diff) const {
  CHECK(index_flag_);
  std::vector<int> index_in_radius;
  for (const int index : GetSegmentIndexInRadius(x, y, max_radius)) {
    const double heading_diff =
        NormalizeAngle(theta - GetSegmentByIndex(index)->heading());
    if (std::fabs(heading_diff) > max_heading_diff) continue;
    index_in_radius.push_back(index);
  }
  return index_in_radius;
}

}  // namespace st
