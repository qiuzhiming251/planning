

#include "plan_common/math/frenet_frame.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "plan_common/timer.h"
#include "plan_common/log.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
//#include "plan_common/util/status_macros.h"

namespace st {

namespace {
constexpr double kMinSegmentLength = 0.1;                         // unit, m
constexpr double kMinSampleDistanceSqr = Sqr(kMinSegmentLength);  // unit, m^2.
constexpr double kMinKdTreeSampleDistanceSqr = 1e-12;             // unit, m^2;

absl::Status CheckAndBuildFrenetInput(
    const absl::Span<const Vec2d>& raw_points, bool down_sample_raw_points,
    FrenetFrameType type, std::vector<Vec2d>* points,
    std::vector<double>* s_knots, std::vector<double>* segment_len_inv,
    std::vector<Vec2d>* tangents, std::vector<int>* raw_indices,
    std::vector<Segment2d>* segments) {
  if (raw_points.size() < 2) {
    return absl::InvalidArgumentError(
        absl::StrCat("The number of input points is less than 2 at "));
  }

  CHECK_NOTNULL(points);
  CHECK_NOTNULL(s_knots);
  CHECK_NOTNULL(segment_len_inv);
  CHECK_NOTNULL(raw_indices);
  CHECK_NOTNULL(segments);

  // Check if there is any invalid data.
  DCHECK([raw_points]() {
    for (const auto& p : raw_points) {
      if (std::isnan(p.x()) || std::isnan(p.y())) return false;
    }
    return true;
  }()) << ::st::DebugString(raw_points);

  points->reserve(raw_points.size());
  points->push_back(raw_points.front());
  raw_indices->reserve(raw_points.size());
  raw_indices->push_back(0);

  for (int i = 1; i < raw_points.size(); ++i) {
    const double distance_sqr = raw_points[i].DistanceSquareTo(points->back());
    if ((down_sample_raw_points && distance_sqr < kMinSampleDistanceSqr) ||
        ((type == FrenetFrameType::kKdTree ||
          type == FrenetFrameType::kQtfmKdTree) &&
         distance_sqr < kMinKdTreeSampleDistanceSqr)) {
      continue;  // Skip if too close.
    }
    points->push_back(raw_points[i]);
    raw_indices->push_back(i);
  }
  if (points->size() < 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        " raw points size ", raw_points.size(), " raw point front (",
        raw_points.front().x(), ", ", raw_points.front().y(),
        ") raw point back (", raw_points.back().x(), ", ",
        raw_points.back().y(), ") at "));
  }

  s_knots->reserve(points->size());
  s_knots->push_back(0.0);
  segment_len_inv->reserve(points->size() - 1);
  segments->reserve(points->size() - 1);
  tangents->reserve(points->size());
  for (int i = 1; i < points->size(); ++i) {
    const Vec2d segment = (*points)[i] - (*points)[i - 1];
    segments->emplace_back((*points)[i - 1], (*points)[i]);
    const double segment_len = segment.norm();
    const double seg_len_inv = 1.0 / segment_len;
    s_knots->push_back(s_knots->back() + segment_len);
    segment_len_inv->push_back(seg_len_inv);
    tangents->push_back(segment * seg_len_inv);
  }
  tangents->push_back(tangents->back());
  return absl::OkStatus();
}

FrenetBox FindBoundingFrenetBox(const std::vector<FrenetCoordinate>& coords) {
  CHECK_GT(coords.size(), 0);

  FrenetBox frenet_box{
      .s_max = -std::numeric_limits<double>::infinity(),
      .s_min = std::numeric_limits<double>::infinity(),
      .l_max = -std::numeric_limits<double>::infinity(),
      .l_min = std::numeric_limits<double>::infinity(),
  };
  for (const auto& coord : coords) {
    frenet_box.s_max = std::max(frenet_box.s_max, coord.s);
    frenet_box.s_min = std::min(frenet_box.s_min, coord.s);
    frenet_box.l_max = std::max(frenet_box.l_max, coord.l);
    frenet_box.l_min = std::min(frenet_box.l_min, coord.l);
  }

  constexpr double kMinFrenetBoxLength = 1e-3;  // unit, m
  frenet_box.s_max =
      std::max(frenet_box.s_max, frenet_box.s_min + kMinFrenetBoxLength);
  frenet_box.l_max =
      std::max(frenet_box.l_max, frenet_box.l_min + kMinFrenetBoxLength);

  return frenet_box;
}

}  // namespace

Vec2d FrenetFrame::InterpolateTangentByS(double s) const {
  const auto it = std::lower_bound(s_knots_.begin(), s_knots_.end(), s);
  if (it == s_knots_.begin()) {
    return tangents_.front();
  }
  if (it == s_knots_.end() || it == s_knots_.end() - 1) {
    return tangents_.back();
  }
  if (std::fabs(s - *it) < 1e-3) {
    double t = (s - *(it - 1)) / (*(it + 1) - *(it - 1));
    return Vec2d::FastUnitFromAngle(
        LerpAngle(tangents_[it - 1 - s_knots_.begin()].Angle(),
                  tangents_[it - s_knots_.begin()].Angle(), t));
  }
  return tangents_[it - 1 - s_knots_.begin()];
}

Vec2d FrenetFrame::InterpolateTangentByXY(const Vec2d& xy) const {
  FrenetCoordinate sl;
  Vec2d normal;
  int index;
  double alpha = 0.0;
  XYToSL(xy, &sl, &normal, &index, &alpha);
  alpha = std::clamp(alpha - 1, 0.0, 1.0);
  return Vec2d::FastUnitFromAngle(
      LerpAngle(tangents_[index].Angle(), tangents_[index + 1].Angle(), alpha));
}

Vec2d FrenetFrame::SLToXY(const FrenetCoordinate& sl) const {
  Vec2d prev_pt, succ_pt;
  double interp_t;
  std::tie(prev_pt, succ_pt, interp_t) = GetInterpolationRange(sl.s);
  const Vec2d heading_vec = (succ_pt - prev_pt).normalized();
  const Vec2d normal_vec = heading_vec.Perp();

  return Lerp(prev_pt, succ_pt, interp_t) + normal_vec * sl.l;
}

FrenetCoordinate FrenetFrame::XYToSL(const Vec2d& xy) const {
  FrenetCoordinate sl;
  Vec2d normal;
  XYToSL(xy, &sl, &normal);
  return sl;
}

void FrenetFrame::XYToSL(const Vec2d& xy, FrenetCoordinate* sl,
                         Vec2d* normal) const {
  int index;
  double alpha = 0.0;
  XYToSL(xy, sl, normal, &index, &alpha);
}

absl::StatusOr<FrenetCoordinate> FrenetFrame::XYToSLWithHeadingDiffLimit(
    const Vec2d& xy, double heading, double max_heading_diff) const {
  return XYToSLWithHeadingDiffLimitImplement(xy, heading, max_heading_diff);
}

void FrenetFrame::XYToSL(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
                         int* index, double* alpha) const {
  CHECK(std::isfinite(xy.x()) && std::isfinite(xy.y())) << xy.DebugString();
  return XYToSLImplement(xy, sl, normal, index, alpha);
}

void FrenetFrame::XYToSL(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
                         std::pair<int, int>* raw_index_pair,
                         double* alpha) const {
  CHECK_NOTNULL(sl);
  CHECK_NOTNULL(normal);
  CHECK_NOTNULL(raw_index_pair);
  CHECK_NOTNULL(alpha);

  int index = 0;
  XYToSL(xy, sl, normal, &index, alpha);
  raw_index_pair->first = raw_indices_[index];
  raw_index_pair->second = raw_indices_[index + 1];
}

std::tuple<Vec2d, Vec2d, double> FrenetFrame::GetInterpolationRange(
    double s) const {
  // Allow extra-polation
  const auto it = std::lower_bound(s_knots_.begin(), s_knots_.end(), s);

  if (it == s_knots_.begin()) {
    return {points_[0], points_[1], (s - *it) / (*(it + 1) - *it)};
  }

  if (it == s_knots_.end()) {
    return {points_[points_.size() - 2], points_.back(),
            (s - *(it - 2)) / (*(it - 1) - *(it - 2))};
  }

  return {points_[it - 1 - s_knots_.begin()], points_[it - s_knots_.begin()],
          (s - *(it - 1)) / (*it - *(it - 1))};
}

Vec2d FrenetFrame::FindAABBNearestPoint(const Polygon2d& polygon,
                                        bool use_max_s) const {
  double smax = -std::numeric_limits<double>::infinity();
  double smin = std::numeric_limits<double>::infinity();
  double lmax = -std::numeric_limits<double>::infinity();
  double lmin = std::numeric_limits<double>::infinity();

  for (const auto& pt : polygon.points()) {
    FrenetCoordinate sl = XYToSL(pt);
    smax = std::max(smax, sl.s);
    smin = std::min(smin, sl.s);
    lmax = std::max(lmax, sl.l);
    lmin = std::min(lmin, sl.l);
  }

  double near_l;
  if (lmax * lmin < 0) {
    near_l = 0.0;
  } else {
    near_l = std::abs(lmax) > std::abs(lmin) ? lmin : lmax;
  }

  return Vec2d(use_max_s ? smax : smin, near_l);
}

absl::StatusOr<FrenetBox> FrenetFrame::QueryFrenetBoxWithHeading(
    const Box2d& box, double max_heading_diff) const {
  return QueryFrenetBoxAtPointsWithHeading(box.GetCornersCounterClockwise(),
                                           box.heading(), max_heading_diff);
}

absl::StatusOr<FrenetBox> FrenetFrame::QueryFrenetBoxAtPointsWithHeading(
    absl::Span<const Vec2d> points, double heading,
    double max_heading_diff) const {
  std::vector<FrenetCoordinate> coords;
  coords.reserve(points.size());
  for (const auto& pt : points) {
    auto p = XYToSLWithHeadingDiffLimit(pt, heading, max_heading_diff);
    if (p.ok()) {
      coords.emplace_back() = p.value();
    }
  }
  if (coords.empty()) {
    return absl::NotFoundError("No frenet conversion is found.");
  }
  return FindBoundingFrenetBox(coords);
}

absl::StatusOr<FrenetBox> FrenetFrame::QueryFrenetBoxAt(
    const Box2d& box) const {
  return QueryFrenetBoxAtPoints(box.GetCornersCounterClockwise());
}

absl::StatusOr<FrenetBox> FrenetFrame::QueryFrenetBoxAtContour(
    const Polygon2d& contour) const {
  return QueryFrenetBoxAtPoints(contour.points());
}

absl::StatusOr<FrenetBox> FrenetFrame::QueryFrenetBoxAtPoints(
    absl::Span<const Vec2d> points) const {
  std::vector<FrenetCoordinate> coords;
  coords.reserve(points.size());
  for (const auto& pt : points) {
    coords.push_back(XYToSL(pt));
  }
  if (coords.empty()) {
    return absl::NotFoundError("No frenet conversion is found.");
  }
  return FindBoundingFrenetBox(coords);
}

absl::StatusOr<FrenetCoordinate>
KdTreeFrenetFrame::XYToSLWithHeadingDiffLimitImplement(
    const Vec2d& xy, double heading, double max_heading_diff) const {
  constexpr double kMinSearchRadiusWithHeadingLimit = 3.0;  // m.

  int nearest_index;
  if (!segment_matcher_->GetNearestSegmentIndex(xy.x(), xy.y(),
                                                &nearest_index)) {
    return absl::NotFoundError("Cannot find nearest segment index. (KdTree)");
  }

  const auto& nearest_seg_by_kdtree = segments_[nearest_index];
  const double d = nearest_seg_by_kdtree.DistanceTo(xy);
  const auto max_radius = std::max(2.0 * d, kMinSearchRadiusWithHeadingLimit);
  int nearest_index_h_diff;
  if (!segment_matcher_->GetNearestSegmentIndexWithHeading(
          xy.x(), xy.y(), heading, max_radius, max_heading_diff,
          &nearest_index_h_diff)) {
    return absl::NotFoundError("Cannot find segment with correct heading.");
  }

  const auto& seg = segments_[nearest_index_h_diff];
  const double prod = seg.ProductOntoUnit(xy);
  const double proj = seg.ProjectOntoUnit(xy);
  const double alpha = proj * segment_len_inv_[nearest_index_h_diff];
  const auto& seg_p0 = points_[nearest_index_h_diff];
  double s;
  double l;
  if (alpha < 0.0 && nearest_index_h_diff > 0) {
    s = s_knots_[nearest_index_h_diff];
    l = std::copysign(seg_p0.DistanceTo(xy), prod);
  } else if (alpha > 1.0 && nearest_index_h_diff + 2 < points_.size()) {
    s = s_knots_[nearest_index_h_diff + 1];
    const auto& seg_p1 = points_[nearest_index_h_diff + 1];
    l = std::copysign(seg_p1.DistanceTo(xy), prod);
  } else {
    s = s_knots_[nearest_index_h_diff] + proj;
    l = prod;
  }
  return FrenetCoordinate{s, l};
}

absl::StatusOr<FrenetCoordinate>
FrenetFrame::XYToSLWithHeadingDiffLimitImplement(
    const Vec2d& xy, double heading, double max_heading_diff) const {
  FrenetCoordinate sl;
  Vec2d normal;

  double min_d = std::numeric_limits<double>::infinity();
  for (int i = 1; i < points_.size(); ++i) {
    const Vec2d p0 = points_[i - 1];
    const Vec2d p1 = points_[i];
    const Vec2d heading_vec = tangents_[i - 1];
    const Vec2d query_segment = xy - p0;
    const double projection =
        query_segment.dot(heading_vec) * segment_len_inv_[i - 1];
    const double production = heading_vec.CrossProd(query_segment);
    double l = std::numeric_limits<double>::infinity();
    double s = 0.0;
    if (projection < 0.0 && i > 1) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p0.DistanceTo(xy) * sign;
      s = s_knots_[i - 1];
    } else if (projection > 1.0 && i + 1 < points_.size()) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p1.DistanceTo(xy) * sign;
      s = s_knots_[i];
    } else {
      l = production;
      s = Lerp(s_knots_[i - 1], s_knots_[i], projection);
    }

    if (std::fabs(NormalizeAngle(heading - heading_vec.FastAngle())) >
        max_heading_diff) {
      continue;
    }

    if (std::fabs(l) < min_d) {
      min_d = std::fabs(l);
      sl = {s, l};
      normal = heading_vec.Perp();
    }
  }
  if (min_d < std::numeric_limits<double>::infinity()) {
    return sl;
  }
  return absl::NotFoundError("No viable frenet projection! (Brute force)");
}

void KdTreeFrenetFrame::XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl,
                                        Vec2d* normal, int* index,
                                        double* alpha) const {
  DCHECK_NOTNULL(sl);
  DCHECK_NOTNULL(normal);
  DCHECK_NOTNULL(index);
  DCHECK_NOTNULL(alpha);

  CHECK(segment_matcher_->GetNearestSegmentIndex(xy.x(), xy.y(), index));

  const auto& nearest_seg = segments_[*index];
  const double prod = nearest_seg.ProductOntoUnit(xy);
  const double proj = nearest_seg.ProjectOntoUnit(xy);
  *alpha = proj * segment_len_inv_[*index];
  *normal = tangents_[*index].Perp();
  const auto& p0 = points_[*index];

  if (*alpha < 0.0 && *index > 0) {
    sl->s = s_knots_[*index];
    sl->l = std::copysign(p0.DistanceTo(xy), prod);
  } else if (*alpha > 1.0 && *index + 2 < points_.size()) {
    sl->s = s_knots_[*index + 1];
    const auto& p1 = points_[*index + 1];
    sl->l = std::copysign(p1.DistanceTo(xy), prod);
  } else {
    sl->s = s_knots_[*index] + proj;
    sl->l = prod;
  }
}

void BruteForceFrenetFrame::XYToSLImplement(const Vec2d& xy,
                                            FrenetCoordinate* sl, Vec2d* normal,
                                            int* index, double* alpha) const {
  double min_d = std::numeric_limits<double>::infinity();
  for (int i = 1; i < points_.size(); ++i) {
    const Vec2d p0 = points_[i - 1];
    const Vec2d p1 = points_[i];
    const Vec2d heading_vec = tangents_[i - 1];
    const Vec2d query_segment = xy - p0;
    const double projection =
        query_segment.dot(heading_vec) * segment_len_inv_[i - 1];
    const double production = heading_vec.CrossProd(query_segment);
    double l = 0.0;
    double s = 0.0;
    if (projection < 0.0 && i > 1) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p0.DistanceTo(xy) * sign;
      s = s_knots_[i - 1];
    } else if (projection > 1.0 && i + 1 < points_.size()) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p1.DistanceTo(xy) * sign;
      s = s_knots_[i];
    } else {
      l = production;
      s = Lerp(s_knots_[i - 1], s_knots_[i], projection);
    }

    if (std::fabs(l) < min_d) {
      min_d = std::fabs(l);
      *sl = {s, l};
      *normal = heading_vec.Perp();
      *index = i - 1;
      *alpha = projection;
    }
  }
}

void QtfmEnhancedKdTreeFrenetFrame::XYToSLImplement(const Vec2d& xy,
                                                    FrenetCoordinate* sl,
                                                    Vec2d* normal, int* index,
                                                    double* alpha) const {
  DCHECK_NOTNULL(sl);
  DCHECK_NOTNULL(normal);
  DCHECK_NOTNULL(index);
  DCHECK_NOTNULL(alpha);
  DCHECK_NOTNULL(qtfm_segment_matcher_);
  const bool found_by_qtfm =
      qtfm_segment_matcher_->GetNearestSegmentIndex(xy.x(), xy.y(), index);
  if (!found_by_qtfm ||
      segments_[*index].DistanceSquareTo(xy) >
          Sqr(qtfm_segment_matcher_->config().cutoff_distance)) {
    CHECK(segment_matcher_->GetNearestSegmentIndex(xy.x(), xy.y(), index));
  }

  const auto& nearest_seg = segments_[*index];
  const double prod = nearest_seg.ProductOntoUnit(xy);
  const double proj = nearest_seg.ProjectOntoUnit(xy);
  *alpha = proj * segment_len_inv_[*index];
  *normal = tangents_[*index].Perp();
  const auto& p0 = points_[*index];

  if (*alpha < 0.0 && *index > 0) {
    sl->s = s_knots_[*index];
    sl->l = std::copysign(p0.DistanceTo(xy), prod);
  } else if (*alpha > 1.0 && *index + 2 < points_.size()) {
    sl->s = s_knots_[*index + 1];
    const auto& p1 = points_[*index + 1];
    sl->l = std::copysign(p1.DistanceTo(xy), prod);
  } else {
    sl->s = s_knots_[*index] + proj;
    sl->l = prod;
  }
}

absl::StatusOr<BruteForceFrenetFrame> BuildBruteForceFrenetFrame(
    absl::Span<const Vec2d> raw_points, bool down_sample_raw_points) {
  std::vector<Vec2d> points;
  std::vector<double> s_knots;
  std::vector<double> segment_len_inv;
  std::vector<Vec2d> tangents;
  std::vector<int> raw_indices;
  std::vector<Segment2d> segments;

  auto status = CheckAndBuildFrenetInput(
      raw_points, down_sample_raw_points, FrenetFrameType::kBruteFroce, &points,
      &s_knots, &segment_len_inv, &tangents, &raw_indices, &segments);
  if (!status.ok()) {
    return status;
  }

  return BruteForceFrenetFrame(std::move(points), std::move(s_knots),
                               std::move(segment_len_inv), std::move(tangents),
                               std::move(raw_indices));
}

absl::StatusOr<KdTreeFrenetFrame> BuildKdTreeFrenetFrame(
    absl::Span<const Vec2d> raw_points, bool down_sample_raw_points) {
  std::vector<Vec2d> points;
  std::vector<double> s_knots;
  std::vector<double> segment_len_inv;
  std::vector<Vec2d> tangents;
  std::vector<int> raw_indices;
  std::vector<Segment2d> segments;
  std::shared_ptr<SegmentMatcherKdtree> segment_matcher;

  auto status = CheckAndBuildFrenetInput(
      raw_points, down_sample_raw_points, FrenetFrameType::kKdTree, &points,
      &s_knots, &segment_len_inv, &tangents, &raw_indices, &segments);
  if (!status.ok()) {
    return status;
  }

  segment_matcher = std::make_unique<SegmentMatcherKdtree>(segments);
  return KdTreeFrenetFrame(std::move(points), std::move(s_knots),
                           std::move(segment_len_inv), std::move(tangents),
                           std::move(raw_indices), std::move(segments),
                           std::move(segment_matcher));
}

absl::StatusOr<QtfmEnhancedKdTreeFrenetFrame>
BuildQtfmEnhancedKdTreeFrenetFrame(absl::Span<const Vec2d> raw_points,
                                   bool down_sample_raw_points) {
  using planning::QtfmSegmentMatcherV2;
  std::vector<Vec2d> points;
  std::vector<double> s_knots;
  std::vector<double> segment_len_inv;
  std::vector<Vec2d> tangents;
  std::vector<int> raw_indices;
  std::vector<Segment2d> segments;
  std::shared_ptr<SegmentMatcherKdtree> segment_matcher;
  std::shared_ptr<QtfmSegmentMatcherV2> qtfm_segment_matcher;

  auto status = CheckAndBuildFrenetInput(
      raw_points, down_sample_raw_points, FrenetFrameType::kQtfmKdTree, &points,
      &s_knots, &segment_len_inv, &tangents, &raw_indices, &segments);
  if (!status.ok()) {
    return status;
  }

  CHECK_GE(kMinSegmentLength, QtfmSegmentMatcherV2::kMinSegmentLength);
  // Plan B is kdtree.
  segment_matcher = std::make_shared<SegmentMatcherKdtree>(segments);

  // Plan A: Quad Tree Field Map.
  constexpr double kCutoffDistance = 8.0;
  constexpr double kResolution = 0.5;
  constexpr int kDepth = 5;
  constexpr int kCutoffNumRange = 1;
  constexpr int kCutoffNumCandidate = 32;
  double x_min = +std::numeric_limits<double>::infinity();
  double y_min = +std::numeric_limits<double>::infinity();
  double x_max = -std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();

  for (const auto& segment : segments) {
    x_max = std::max(segment.max_x(), x_max);
    x_min = std::min(segment.min_x(), x_min);
    y_max = std::max(segment.max_y(), y_max);
    y_min = std::min(segment.min_y(), y_min);
  }
  CHECK_GE(x_max, x_min);
  CHECK_GE(y_max, y_min);

  QtfmSegmentMatcherV2::Config config{
      .x_min = x_min - kCutoffDistance,
      .x_max = x_max + kCutoffDistance,
      .y_min = y_min - kCutoffDistance,
      .y_max = y_max + kCutoffDistance,
      .resolution = kResolution,
      .qtfm_depth = kDepth,
      .cutoff_num_range = kCutoffNumRange,
      .cutoff_max_num_candidate = kCutoffNumCandidate,
      .cutoff_distance = kCutoffDistance};
  std::vector<std::pair<int, int>> segment_connection_status;
  segment_connection_status.reserve(segments.size());

  for (int i = 0; i < segments.size(); ++i) {
    segment_connection_status.emplace_back(
        i - 1, (i + 1 < segments.size()) ? (i + 1) : (-1));
  }

  qtfm_segment_matcher = std::make_shared<QtfmSegmentMatcherV2>(
      config, segments, std::vector<int>{0});

  return QtfmEnhancedKdTreeFrenetFrame(
      std::move(points), std::move(s_knots), std::move(segment_len_inv),
      std::move(tangents), std::move(raw_indices), std::move(segments),
      std::move(segment_matcher), std::move(qtfm_segment_matcher));
}

}  // namespace st
