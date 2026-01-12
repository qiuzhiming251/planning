

// NOTE: This file is copied from Apollo project and modified by BYD.ai for
// its own use.

#include "plan_common/math/geometry/polygon2d.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <utility>

#include <stddef.h>

#include "absl/strings/str_join.h"
#include "plan_common/log.h"
#include "plan_common/math/geometry/gjk2d.h"
#include "plan_common/math/util.h"

namespace st {
namespace {

constexpr double kEpsilon = 1e-10;

inline double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                        const Vec2d& end_point_2) {
  return Vec2d(end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

inline AABox2d SegmentToAABox(const Segment2d& seg) {
  return AABox2d(seg.center(), seg.max_x() - seg.min_x(),
                 seg.max_y() - seg.min_y());
}
}  // namespace

Polygon2d::Polygon2d(const Box2d& box) : is_convex_(true) {
  points_ = box.GetCornersCounterClockwise();
  BuildFromPoints();
}

Polygon2d::Polygon2d(std::vector<Vec2d> points) : points_(std::move(points)) {
  BuildFromPoints();
  is_convex_ = AreConvexHullPoints(points_);
}

Polygon2d::Polygon2d(std::vector<Vec2d> points, int force_convex)
    : points_(std::move(points)) {
  if (force_convex == 1) {
    is_convex_ = AreConvexHullPoints(points_);
    if (!is_convex_) {
      auto result_points = ComputeConvexHullPoints(points_);
      if (result_points.has_value()) {
        points_ = std::move(result_points.value());
      }
    }
  }
  BuildFromPoints();
  is_convex_ = AreConvexHullPoints(points_);
}

Polygon2d::Polygon2d(std::vector<Vec2d> points, bool is_convex)
    : points_(std::move(points)), is_convex_(is_convex) {
  auto result_points = ComputeConvexHullPoints(points_);
  if (result_points.has_value()) {
    points_ = std::move(result_points.value());
  }
  BuildFromPoints();
  // Check convexity.
  if (is_convex_) {
    DCHECK(AreConvexHullPoints(points_));
  }
}

void Polygon2d::SetPoints(const std::vector<Vec2d>& points) {
  points_ = points;
  BuildFromPoints();
}

bool Polygon2d::IsSelfIntersecting() const {
  const int n = line_segments_.size();
  for (int i = 0; i < n; ++i) {
    const auto& seg_i = line_segments_[i];
    for (int j = i + 2; j < n - 1; ++j) {
      if (seg_i.HasIntersect(line_segments_[j])) {
        return true;
      }
    }
  }
  return false;
}

double Polygon2d::DistanceTo(const Vec2d& point) const {
  CHECK_GE(points_.size(), 3);
  if (IsPointIn(point)) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segments_[i].DistanceTo(point));
  }
  return distance;
}

double Polygon2d::DistanceSquareTo(const Vec2d& point) const {
  CHECK_GE(points_.size(), 3);
  if (IsPointIn(point)) {
    return 0.0;
  }
  double distance_sqr = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance_sqr =
        std::min(distance_sqr, line_segments_[i].DistanceSquareTo(point));
  }
  return distance_sqr;
}

double Polygon2d::DistanceTo(const Segment2d& line_segment) const {
  if (line_segment.length() <= kEpsilon) {
    return DistanceTo(line_segment.start());
  }
  CHECK_GE(points_.size(), 3);
  if (IsPointIn(line_segment.center())) {
    return 0.0;
  }
  if (std::any_of(line_segments_.begin(), line_segments_.end(),
                  [&](const Segment2d& poly_seg) {
                    return poly_seg.HasIntersect(line_segment);
                  })) {
    return 0.0;
  }

  double distance = std::min(DistanceTo(line_segment.start()),
                             DistanceTo(line_segment.end()));
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segment.DistanceTo(points_[i]));
  }
  return distance;
}

double Polygon2d::DistanceTo(const Box2d& box) const {
  CHECK_GE(points_.size(), 3);
  if (IsPointIn(box.center())) {
    return 0.0;
  }
  if (box.IsPointIn(points_[0])) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, box.DistanceTo(line_segments_[i]));
  }
  return distance;
}

double Polygon2d::ConvexDistanceToConvex(const Polygon2d& polygon) const {
  // c = this_polygon - polygon.
  const auto random_point_inside_c = centroid() - polygon.centroid();
  const auto calculate_extreme_point_of_c =
      [this, &polygon](const Vec2d& support_vec) {
        return points_[ConvexExtremePoint(support_vec)] -
               polygon.points()[polygon.ConvexExtremePoint(-support_vec)];
      };
  constexpr double kMinDistThresholdSqr = 1e-4 * 1e-4;  // Less than 0.1mm.
  const int max_iterations = num_points() + polygon.num_points();

  // this_polygon's distance to polygon <=>
  // origin point's distance to c.
  return Gjk2d::DistanceTo(random_point_inside_c, calculate_extreme_point_of_c,
                           kMinDistThresholdSqr, max_iterations);
}

double Polygon2d::DistanceTo(const Polygon2d& polygon) const {
  CHECK_GE(points_.size(), 3);
  CHECK_GE(polygon.num_points(), 3);

  if (is_convex_ && polygon.is_convex()) {
    return ConvexDistanceToConvex(polygon);
  }

  if (IsPointIn(polygon.points()[0])) {
    return 0.0;
  }
  if (polygon.IsPointIn(points_[0])) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, polygon.DistanceTo(line_segments_[i]));
  }
  return distance;
}

double Polygon2d::DistanceToBoundary(const Vec2d& point) const {
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segments_[i].DistanceTo(point));
  }
  return distance;
}

bool Polygon2d::HasOverlapWithBuffer(const Box2d& box, double lat_buffer,
                                     double lon_buffer) const {
  CHECK_GE(points_.size(), 3);
  const auto corners =
      box.GetCornersWithBufferCounterClockwise(lat_buffer, lon_buffer);
  double min_x = corners[0].x();
  double max_x = corners[0].x();
  double min_y = corners[0].y();
  double max_y = corners[0].y();
  for (const auto& corner : corners) {
    min_x = std::min(min_x, corner.x());
    max_x = std::max(max_x, corner.x());
    min_y = std::min(min_y, corner.y());
    max_y = std::max(max_y, corner.y());
  }
  if (min_x > this->max_x() || max_x < this->min_x() || min_y > this->max_y() ||
      max_y < this->min_y()) {
    return false;
  }
  if (IsPointIn(corners[0])) {
    return true;
  }
  for (const auto& segment : line_segments()) {
    if (box.HasOverlapWithBuffer(segment, lat_buffer, lon_buffer)) {
      return true;
    }
  }
  return false;
}

bool Polygon2d::ConvexHasOverlapConvex(const Polygon2d& polygon) const {
  // c = this_polygon - polygon.
  const auto random_point_inside_c = centroid() - polygon.centroid();
  const auto calculate_extreme_point_of_c = [&](const Vec2d& support_vec) {
    return points_[ConvexExtremePoint(support_vec)] -
           polygon.points()[polygon.ConvexExtremePoint(-support_vec)];
  };
  constexpr double kMinDistThresholdSqr = 1e-4 * 1e-4;  // Less than 0.1mm.
  const int max_iterations = num_points() + polygon.num_points();

  // this_polygon and polygon overlaps <=>
  // origin point is inside c.
  return Gjk2d::HasOverlap(random_point_inside_c, calculate_extreme_point_of_c,
                           kMinDistThresholdSqr, max_iterations);
}

bool Polygon2d::GetMinPenetrationDistance(const Polygon2d& polygon,
                                          double* min_penetration,
                                          Vec2d* dir_vec) const {
  const auto random_point_inside_c = centroid() - polygon.centroid();
  const auto calculate_extreme_point_of_c = [&](const Vec2d& support_vec) {
    return points_[ConvexExtremePoint(support_vec)] -
           polygon.points()[polygon.ConvexExtremePoint(-support_vec)];
  };
  constexpr double kMinDistThreshold = 1e-4;  // Less than 0.1mm.
  const int max_iterations = num_points() + polygon.num_points();
  return Gjk2d::GetMinPenetrationDistance(
      random_point_inside_c, calculate_extreme_point_of_c, kMinDistThreshold,
      max_iterations, min_penetration, dir_vec);
}

bool Polygon2d::GetMinPenetrationDistance(const Polygon2d& polygon,
                                          double* min_penetration) const {
  Vec2d unused;
  return GetMinPenetrationDistance(polygon, min_penetration, &unused);
}

bool Polygon2d::GetPenetrationDistanceAlongDir(const Polygon2d& polygon,
                                               const Vec2d& dir_vec,
                                               double* penetration) const {
  const auto random_point_inside_c = centroid() - polygon.centroid();
  const auto calculate_extreme_point_of_c = [&](const Vec2d& support_vec) {
    return points_[ConvexExtremePoint(support_vec)] -
           polygon.points()[polygon.ConvexExtremePoint(-support_vec)];
  };
  constexpr double kMinDistThreshold = 1e-4;  // Less than 0.1mm.
  const int max_iterations = num_points() + polygon.num_points();
  return Gjk2d::GetPenetrationDistanceAlongDir(
      random_point_inside_c, calculate_extreme_point_of_c, dir_vec,
      kMinDistThreshold, max_iterations, penetration);
}

bool Polygon2d::HasOverlap(const Polygon2d& polygon) const {
  CHECK_GE(points_.size(), 3);
  if (polygon.max_x() < min_x() || polygon.min_x() > max_x() ||
      polygon.max_y() < min_y() || polygon.min_y() > max_y()) {
    return false;
  }

  if (is_convex_ && polygon.is_convex()) {
    return ConvexHasOverlapConvex(polygon);
  }

  if (polygon.IsPointIn(GetAllVertices()[0])) {
    return true;
  }

  for (const auto& segment : polygon.line_segments()) {
    if (HasOverlap(segment)) {
      return true;
    }
  }
  return false;
}

bool Polygon2d::Contains(const Segment2d& line_segment) const {
  if (line_segment.length() <= kEpsilon) {
    return IsPointIn(line_segment.start());
  }
  CHECK_GE(points_.size(), 3);
  if (!IsPointIn(line_segment.start())) {
    return false;
  }
  if (!IsPointIn(line_segment.end())) {
    return false;
  }
  if (!is_convex_) {
    std::vector<Segment2d> overlaps = GetAllOverlaps(line_segment);
    double total_length = 0;
    for (const auto& overlap_seg : overlaps) {
      total_length += overlap_seg.length();
    }
    return total_length >= line_segment.length() - kEpsilon;
  }
  return true;
}

bool Polygon2d::Contains(const Polygon2d& polygon) const {
  CHECK_GE(points_.size(), 3);
  if (area_ < polygon.area() - kEpsilon) {
    return false;
  }
  if (!IsPointIn(polygon.points()[0])) {
    return false;
  }
  const auto& line_segments = polygon.line_segments();
  return std::all_of(
      line_segments.begin(), line_segments.end(),
      [&](const Segment2d& line_segment) { return Contains(line_segment); });
}

int Polygon2d::Next(int at) const { return at >= num_points_ - 1 ? 0 : at + 1; }

int Polygon2d::Prev(int at) const { return at == 0 ? num_points_ - 1 : at - 1; }

void Polygon2d::BuildFromPoints() {
  num_points_ = static_cast<int>(points_.size());
  CHECK_GE(num_points_, 3);

  // Make sure the points are in ccw order.
  area_ = 0.0;
  for (int i = 1; i < num_points_; ++i) {
    area_ += CrossProd(points_[0], points_[i - 1], points_[i]);
  }
  if (area_ < 0) {
    area_ = -area_;
    std::reverse(points_.begin(), points_.end());
  }
  area_ /= 2.0;

  // Construct line_segments.
  line_segments_.reserve(num_points_);
  for (int i = 0; i < num_points_; ++i) {
    line_segments_.emplace_back(points_[i], points_[Next(i)]);
  }

  aabox_ = AABox2d(points_);
}

bool Polygon2d::AreConvexHullPoints(absl::Span<const Vec2d> points) {
  const int n = points.size();
  if (n < 3) return false;
  const auto next = [n](int x) { return (x + 1) >= n ? (x + 1 - n) : (x + 1); };
  const auto prev = [n](int x) { return x == 0 ? n - 1 : x - 1; };
  for (int i = 0; i < n; ++i) {
    const auto dd = CrossProd(points[prev(i)], points[i], points[next(i)]);
    if (dd <= kMathEpsilon) {
      LOG_WARN << absl::StrFormat(
          "polygon is not convex %f, %f, %f, %f, %f, %f dd %f",
          points[prev(i)].x(), points[prev(i)].y(), points[i].x(),
          points[i].y(), points[next(i)].x(), points[next(i)].y(), dd);
      return false;
    }
  }
  return true;
}

bool Polygon2d::AreNonDegeneratedConvexHullPoints(
    absl::Span<const Vec2d> points) {
  const int n = static_cast<int>(points.size());
  CHECK_GE(n, 3);
  Vec2d prev_seg = points[0] - points[n - 1];
  for (int i = 0; i < n; ++i) {
    const int next_id = (i == n - 1) ? 0 : (i + 1);
    const Vec2d current_seg = points[next_id] - points[i];

    if (prev_seg.CrossProd(current_seg) <
        std::numeric_limits<double>::epsilon()) {
      return false;
    }
    prev_seg = current_seg;
  }
  return true;
}

std::optional<std::vector<Vec2d>> Polygon2d::ComputeConvexHullPoints(
    absl::Span<const Vec2d> points) {
  const int origin_n = static_cast<int>(points.size());

  std::vector<int> sorted_by_x_indices(origin_n, 0);
  std::iota(sorted_by_x_indices.begin(), sorted_by_x_indices.end(), 0);
  std::sort(sorted_by_x_indices.begin(), sorted_by_x_indices.end(),
            [&](const int idx1, const int idx2) {
              return points[idx1].x() < points[idx2].x();
            });

  // Stores indices of non-duplicate points.
  std::vector<int> sorted_indices;
  sorted_indices.reserve(origin_n);

  // NOTE(zheng): If the polygon edge length is too small, it will return false
  // when we use AreNonDegeneratedConvexHullPoints() function, and also the
  // segment of the polygon may be zero, so we should remove the duplicated
  // polygon points.
  constexpr double kMaxDuplicatedDist = 1e-6;
  constexpr double kMaxDuplicatedDistSqr =
      kMaxDuplicatedDist * kMaxDuplicatedDist;
  for (int i = 0; i < origin_n; ++i) {
    bool has_duplicated_points = false;
    const Vec2d& point_i = points[sorted_by_x_indices[i]];
    for (int j = i + 1; j < origin_n; ++j) {
      const Vec2d& point_j = points[sorted_by_x_indices[j]];
      if (point_j.x() - point_i.x() >= kMaxDuplicatedDist) {
        break;
      }

      if (point_i.DistanceSquareTo(point_j) < kMaxDuplicatedDistSqr) {
        has_duplicated_points = true;
        break;
      }
    }
    if (!has_duplicated_points)
      sorted_indices.push_back(sorted_by_x_indices[i]);
  }
  if (sorted_indices.size() < 3) return std::nullopt;

  const int n = sorted_indices.size();
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&](const int idx1, const int idx2) {
              const Vec2d& pt1 = points[idx1];
              const Vec2d& pt2 = points[idx2];
              const double dx = pt1.x() - pt2.x();
              if (std::abs(dx) > kEpsilon) {
                return dx < 0.0;
              }
              return pt1.y() < pt2.y();
            });

  int count = 0;
  std::vector<int> results;
  results.reserve(n);
  int last_count = 1;
  for (int i = 0; i < n + n; ++i) {
    if (i == n) {
      last_count = count;
    }
    const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
    const Vec2d& pt = points[idx];
    while (count > last_count &&
           CrossProd(points[results[count - 2]], points[results[count - 1]],
                     pt) <= kEpsilon) {
      results.pop_back();
      --count;
    }
    results.push_back(idx);
    ++count;
  }
  results.pop_back();
  --count;
  if (count < 3) {
    return std::nullopt;
  }
  std::vector<Vec2d> convex_hull_points;
  convex_hull_points.reserve(count);
  for (const auto idx : results) {
    convex_hull_points.push_back(points[idx]);
  }
  return convex_hull_points;
}

bool Polygon2d::ComputeConvexHull(absl::Span<const Vec2d> points,
                                  Polygon2d* const polygon) {
  CHECK_NOTNULL(polygon);
  auto result_points = ComputeConvexHullPoints(points);
  if (result_points.has_value()) {
    *polygon = Polygon2d(std::move(*result_points), /*is_convex=*/true);
    return true;
  }
  return false;
}

Polygon2d Polygon2d::MergeTwoBoxes(const Box2d& first_box,
                                   const Box2d& second_box) {
  Polygon2d merged_polygon;
  std::vector<Vec2d> corners = first_box.GetCornersCounterClockwise();
  const std::vector<Vec2d>& second_corners =
      second_box.GetCornersCounterClockwise();
  corners.insert(corners.end(), second_corners.begin(), second_corners.end());
  CHECK(Polygon2d::ComputeConvexHull(corners, &merged_polygon))
      << "First box: " << first_box.DebugStringFullPrecision()
      << ",  second box: " << second_box.DebugStringFullPrecision();
  return merged_polygon;
}

Polygon2d Polygon2d::MergeBoxes(const std::vector<Box2d>& boxes) {
  Polygon2d merged_polygon;
  std::vector<Vec2d> corners;
  for (const auto& box : boxes) {
    const std::vector<Vec2d>& current_corners =
        box.GetCornersCounterClockwise();
    corners.insert(corners.end(), current_corners.begin(),
                   current_corners.end());
  }
  CHECK(Polygon2d::ComputeConvexHull(corners, &merged_polygon)) << [&boxes]() {
    return absl::StrJoin(boxes, ", ", [](std::string* out, const Box2d& box) {
      absl::StrAppend(out, box.DebugStringFullPrecision());
    });
  }();
  return merged_polygon;
}

Polygon2d Polygon2d::MergeTwoPolygons(const Polygon2d& first_polygon,
                                      const Polygon2d& second_polygon) {
  Polygon2d merged_polygon;
  std::vector<Vec2d> corners = first_polygon.points();
  const std::vector<Vec2d>& second_corners = second_polygon.points();
  corners.insert(corners.end(), second_corners.begin(), second_corners.end());
  CHECK(Polygon2d::ComputeConvexHull(corners, &merged_polygon))
      << st::DebugStringFullPrecision(corners);
  return merged_polygon;
}

bool Polygon2d::ClipConvexHull(const Segment2d& line_segment,
                               std::vector<Vec2d>* const points) {
  if (line_segment.length() <= kEpsilon) {
    return true;
  }
  CHECK_NOTNULL(points);
  const size_t n = points->size();
  if (n < 3) {
    return false;
  }
  std::vector<double> prod(n);
  std::vector<int> side(n);
  for (size_t i = 0; i < n; ++i) {
    prod[i] = CrossProd(line_segment.start(), line_segment.end(), (*points)[i]);
    if (std::abs(prod[i]) <= kEpsilon) {
      side[i] = 0;
    } else {
      side[i] = ((prod[i] < 0) ? -1 : 1);
    }
  }

  std::vector<Vec2d> new_points;
  for (size_t i = 0; i < n; ++i) {
    if (side[i] >= 0) {
      new_points.push_back((*points)[i]);
    }
    const size_t j = ((i == n - 1) ? 0 : (i + 1));
    if (side[i] * side[j] < 0) {
      const double ratio = prod[j] / (prod[j] - prod[i]);
      new_points.emplace_back(
          (*points)[i].x() * ratio + (*points)[j].x() * (1.0 - ratio),
          (*points)[i].y() * ratio + (*points)[j].y() * (1.0 - ratio));
    }
  }

  points->swap(new_points);
  return points->size() >= 3;
}

bool Polygon2d::ComputeOverlap(const Polygon2d& other_polygon,
                               Polygon2d* const overlap_polygon) const {
  CHECK_GE(points_.size(), 3);
  CHECK_NOTNULL(overlap_polygon);
  CHECK(is_convex_ && other_polygon.is_convex())
      << "Polygon2d this_polygon(" << DebugStringFullPrecision()
      << ");\n Polygon2d other_polygon(" << DebugStringFullPrecision() << ");";
  std::vector<Vec2d> points = other_polygon.points();
  for (int i = 0; i < num_points_; ++i) {
    if (!ClipConvexHull(line_segments_[i], &points)) {
      return false;
    }
  }
  return ComputeConvexHull(points, overlap_polygon) &&
         overlap_polygon->area_ > kEpsilon;
}

bool Polygon2d::HasOverlap(const Segment2d& line_segment) const {
  CHECK_GE(points_.size(), 3);
  if (const auto aabox = SegmentToAABox(line_segment);
      !aabox.HasOverlap(aabox_)) {
    return false;
  }

  Vec2d first;
  Vec2d last;
  return GetOverlap(line_segment, &first, &last);
}

bool Polygon2d::GetOverlap(const Segment2d& line_segment, Vec2d* const first,
                           Vec2d* const last) const {
  CHECK_GE(points_.size(), 3);
  CHECK_NOTNULL(first);
  CHECK_NOTNULL(last);

  if (line_segment.length() <= kEpsilon) {
    if (!IsPointIn(line_segment.start())) {
      return false;
    }
    *first = line_segment.start();
    *last = line_segment.start();
    return true;
  }

  double min_proj = line_segment.length();
  double max_proj = 0;
  if (IsPointIn(line_segment.start())) {
    *first = line_segment.start();
    min_proj = 0.0;
  }
  if (IsPointIn(line_segment.end())) {
    *last = line_segment.end();
    max_proj = line_segment.length();
  }
  for (const auto& poly_seg : line_segments_) {
    Vec2d pt;
    if (poly_seg.GetIntersect(line_segment, &pt)) {
      double proj = line_segment.ProjectOntoUnit(pt);
      proj = std::clamp(proj, 0.0, line_segment.length());
      if (proj <= min_proj) {
        min_proj = proj;
        *first = pt;
      }
      if (proj >= max_proj) {
        max_proj = proj;
        *last = pt;
      }
    }
  }
  return min_proj <= max_proj + kEpsilon;
}

std::vector<Segment2d> Polygon2d::GetAllOverlaps(
    const Segment2d& line_segment) const {
  CHECK_GE(points_.size(), 3);

  if (line_segment.length() <= kEpsilon) {
    std::vector<Segment2d> overlaps;
    if (IsPointIn(line_segment.start())) {
      overlaps.push_back(line_segment);
    }
    return overlaps;
  }
  std::vector<double> projections;
  if (IsPointIn(line_segment.start())) {
    projections.push_back(0.0);
  }
  if (IsPointIn(line_segment.end())) {
    projections.push_back(line_segment.length());
  }
  for (const auto& poly_seg : line_segments_) {
    Vec2d pt;
    if (poly_seg.GetIntersect(line_segment, &pt)) {
      projections.push_back(line_segment.ProjectOntoUnit(pt));
    }
  }
  std::sort(projections.begin(), projections.end());
  std::vector<std::pair<double, double>> overlaps;
  for (size_t i = 0; i + 1 < projections.size(); ++i) {
    const double start_proj = projections[i];
    const double end_proj = projections[i + 1];
    if (end_proj - start_proj <= kEpsilon) {
      continue;
    }
    const Vec2d reference_point =
        line_segment.start() +
        (start_proj + end_proj) * 0.5 * line_segment.unit_direction();
    if (!IsPointIn(reference_point)) {
      continue;
    }
    if (overlaps.empty() || start_proj > overlaps.back().second + kEpsilon) {
      overlaps.emplace_back(start_proj, end_proj);
    } else {
      overlaps.back().second = end_proj;
    }
  }
  std::vector<Segment2d> overlap_line_segments;
  overlap_line_segments.reserve(overlaps.size());
  for (const auto& overlap : overlaps) {
    overlap_line_segments.emplace_back(
        line_segment.start() + overlap.first * line_segment.unit_direction(),
        line_segment.start() + overlap.second * line_segment.unit_direction());
  }
  return overlap_line_segments;
}

void Polygon2d::ExtremePoints(const double heading, Vec2d* const first,
                              Vec2d* const last) const {
  ExtremePoints(Vec2d::UnitFromAngle(heading), first, last);
}

void Polygon2d::ExtremePoints(const Vec2d& direction_vec, Vec2d* first,
                              Vec2d* last) const {
  CHECK(first != nullptr);
  CHECK(last != nullptr);
  int first_index, last_index;
  ExtremePoints(direction_vec, &first_index, &last_index);
  *first = points()[first_index];
  *last = points()[last_index];
}

void Polygon2d::ExtremePoints(const Vec2d& direction_vec, int* first_idx,
                              int* last_idx, Vec2d* first_pt,
                              Vec2d* last_pt) const {
  DCHECK(first_idx != nullptr);
  DCHECK(last_idx != nullptr);
  DCHECK(first_pt != nullptr);
  DCHECK(first_pt != nullptr);
  ExtremePoints(direction_vec, first_idx, last_idx);
  *first_pt = points()[*first_idx];
  *last_pt = points()[*last_idx];
}

int Polygon2d::ConvexExtremePoint(const Vec2d& direction_vec) const {
  double max_proj = direction_vec.dot(points_[0]);
  int index = 0, ret = 0;
  while (++index < points_.size()) {
    double product = direction_vec.dot(points_[index]);
    if (product > max_proj) {
      max_proj = product;
      ret = index;
    } else {
      break;
    }
  }
  if (ret != 0) {
    return ret;
  }
  for (int i = points_.size() - 1; i > index; --i) {
    double product = direction_vec.dot(points_[i]);
    if (product > max_proj) {
      max_proj = product;
      ret = i;
    } else {
      break;
    }
  }
  return ret;
}

int Polygon2d::ExtremePoint(const Vec2d& direction_vec) const {
  if (is_convex_) {
    return ConvexExtremePoint(direction_vec);
  } else {
    return ExtremePointBruteForce(direction_vec);
  }
}

int Polygon2d::ExtremePointBruteForce(const Vec2d& direction_vec) const {
  CHECK_GE(points_.size(), 3);
  int ret = 0;
  double max_proj = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    const double proj = points_[i].dot(direction_vec);
    if (proj > max_proj) {
      max_proj = proj;
      ret = i;
    }
  }
  return ret;
}

int Polygon2d::ConvexExtremePointBinarySearch(
    const Vec2d& direction_vec) const {
  CHECK(is_convex());

  // Judge whether the point at given index is on the
  // right half boundary. The direction of right is
  // along -direction_vec.Perp().
  auto is_right_point = [&](int index) {
    DCHECK_GE(index, 0);
    DCHECK_LT(index, num_points_);
    const Vec2d to_next = points_[Next(index)] - points_[index];
    const double y_proj = to_next.Dot(direction_vec);

    return y_proj > 0.0 ||
           (y_proj == 0.0 && to_next.CrossProd(direction_vec) > 0.0);
  };

  // Find a vertex such that its prev edge is not horizontal.
  // For non-degenerated polygon it returns in constant time.
  int start = -1;
  for (int i = 0; i < num_points_; ++i) {
    if (abs(direction_vec.Dot(points_[i] - points_[Prev(i)])) >= 1e-9) {
      start = i;
      break;
    }
  }

  if (start < 0) {
    // All points are in the same horizontal level.
    return 0;
  }

  const bool start_at_right = is_right_point(start);
  if (!start_at_right && is_right_point(Prev(start))) {
    // start being the extreme point.
    return start;
  }

  // Otherwise, enumerating index from start to start+num_points-1.
  // there must be one id being extreme point.
  // And it is certain that
  // start is before extreme point.
  // start+num_points-1 is not before extreme point.
  auto before_extreme_point = [&](int raw_index) {
    const int index =
        raw_index >= num_points_ ? (raw_index - num_points_) : raw_index;
    DCHECK_GE(index, 0);
    DCHECK_LT(index, num_points_);
    if (start_at_right) {
      if (is_right_point(index)) {
        return (points_[index] - points_[start]).Dot(direction_vec) >= -1e-10;
      } else {
        return false;
      }
    } else {
      if (is_right_point(index)) {
        return true;
      } else {
        return (points_[index] - points_[start]).Dot(direction_vec) <= 1e-10;
      }
    }
  };

  int l = start, r = start + num_points_ - 1;
  CHECK(before_extreme_point(l));
  CHECK(!before_extreme_point(r));
  while (l + 1 < r) {
    int mid = (l + r) / 2;
    if (before_extreme_point(mid)) {
      l = mid;
    } else {
      r = mid;
    }
  }
  return r >= num_points_ ? (r - num_points_) : r;
}

void Polygon2d::ExtremePoints(const Vec2d& direction_vec, int* first,
                              int* last) const {
  CHECK_GE(points_.size(), 3);
  CHECK(first != nullptr);
  CHECK(last != nullptr);

  double min_proj = std::numeric_limits<double>::infinity();
  double max_proj = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    const double proj = points_[i].dot(direction_vec);
    if (proj < min_proj) {
      min_proj = proj;
      *first = i;
    }
    if (proj > max_proj) {
      max_proj = proj;
      *last = i;
    }
  }
}

Box2d Polygon2d::BoundingBoxWithHeading(const double heading) const {
  CHECK_GE(points_.size(), 3);
  const Vec2d direction_vec = Vec2d::UnitFromAngle(heading);
  Vec2d px1;
  Vec2d px2;
  Vec2d py1;
  Vec2d py2;
  ExtremePoints(heading, &px1, &px2);
  ExtremePoints(heading - M_PI_2, &py1, &py2);
  const double x1 = px1.dot(direction_vec);
  const double x2 = px2.dot(direction_vec);
  const double y1 = py1.CrossProd(direction_vec);
  const double y2 = py2.CrossProd(direction_vec);
  return Box2d(
      (x1 + x2) * 0.5 * direction_vec +
          (y1 + y2) * 0.5 * Vec2d(direction_vec.y(), -direction_vec.x()),
      heading, x2 - x1, y2 - y1);
}

Box2d Polygon2d::MinAreaBoundingBox() const {
  CHECK_GE(points_.size(), 3);
  if (!is_convex_) {
    Polygon2d convex_polygon;
    ComputeConvexHull(points_, &convex_polygon);
    CHECK(convex_polygon.is_convex());
    return convex_polygon.MinAreaBoundingBox();
  }
  double min_area = std::numeric_limits<double>::infinity();
  double min_area_at_heading = 0.0;
  int left_most = 0;
  int right_most = 0;
  int top_most = 0;
  for (int i = 0; i < num_points_; ++i) {
    const auto& line_segment = line_segments_[i];
    double proj = 0.0;
    double min_proj = line_segment.ProjectOntoUnit(points_[left_most]);
    while ((proj = line_segment.ProjectOntoUnit(points_[Prev(left_most)])) <
           min_proj) {
      min_proj = proj;
      left_most = Prev(left_most);
    }
    while ((proj = line_segment.ProjectOntoUnit(points_[Next(left_most)])) <
           min_proj) {
      min_proj = proj;
      left_most = Next(left_most);
    }
    double max_proj = line_segment.ProjectOntoUnit(points_[right_most]);
    while ((proj = line_segment.ProjectOntoUnit(points_[Prev(right_most)])) >
           max_proj) {
      max_proj = proj;
      right_most = Prev(right_most);
    }
    while ((proj = line_segment.ProjectOntoUnit(points_[Next(right_most)])) >
           max_proj) {
      max_proj = proj;
      right_most = Next(right_most);
    }
    double prod = 0.0;
    double max_prod = line_segment.ProductOntoUnit(points_[top_most]);
    while ((prod = line_segment.ProductOntoUnit(points_[Prev(top_most)])) >
           max_prod) {
      max_prod = prod;
      top_most = Prev(top_most);
    }
    while ((prod = line_segment.ProductOntoUnit(points_[Next(top_most)])) >
           max_prod) {
      max_prod = prod;
      top_most = Next(top_most);
    }
    const double area = max_prod * (max_proj - min_proj);
    if (area < min_area) {
      min_area = area;
      min_area_at_heading = line_segment.heading();
    }
  }
  return BoundingBoxWithHeading(min_area_at_heading);
}

Polygon2d Polygon2d::ExpandByDistance(const double distance) const {
  if (!is_convex_) {
    Polygon2d convex_polygon;
    CHECK(ComputeConvexHull(points_, &convex_polygon))
        << st::DebugStringFullPrecision(points_);
    CHECK(convex_polygon.is_convex());
    return convex_polygon.ExpandByDistance(distance);
  }
  constexpr double kMinAngle = 0.1;
  std::vector<Vec2d> points;
  for (int i = 0; i < num_points_; ++i) {
    const double start_angle = line_segments_[Prev(i)].heading() - M_PI_2;
    const double end_angle = line_segments_[i].heading() - M_PI_2;
    const double diff = NormalizeAngle(end_angle - start_angle);
    if (diff <= kEpsilon) {
      points.push_back(points_[i] +
                       Vec2d::UnitFromAngle(start_angle) * distance);
    } else {
      const int count = static_cast<int>(diff / kMinAngle) + 1;
      for (int k = 0; k <= count; ++k) {
        const double angle = start_angle + diff * static_cast<double>(k) /
                                               static_cast<double>(count);
        points.push_back(points_[i] + Vec2d::UnitFromAngle(angle) * distance);
      }
    }
  }
  Polygon2d new_polygon;
  CHECK(ComputeConvexHull(points, &new_polygon))
      << st::DebugStringFullPrecision(points);
  return new_polygon;
}

Polygon2d Polygon2d::ExtrudeAlongVector(const Vec2d& vec) const {
  std::vector<Vec2d> points;
  points.reserve(num_points_ + 2);
  const double vec_angle = vec.Angle();
  for (int i = 0; i < num_points_; ++i) {
    const double start_angle =
        NormalizeAngle(line_segments_[Prev(i)].heading() - vec_angle);
    const double end_angle =
        NormalizeAngle(line_segments_[i].heading() - vec_angle);
    if (start_angle < 0.0 && end_angle >= 0.0) {
      points.push_back(points_[i]);
    } else if (start_angle >= 0.0 && end_angle < 0.0) {
      points.push_back(points_[i] + vec);
    }
    points.push_back(points_[i] + (end_angle > 0.0 ? vec : Vec2d::Zero()));
  }
  return Polygon2d(std::move(points));
}

Polygon2d Polygon2d::Transform(const Vec2d& center, double cos_angle,
                               double sin_angle,
                               const Vec2d& translation) const {
  // Use large error tolerance to accept approximately computed cos/sin values.
  DCHECK_GE(Sqr(cos_angle) + Sqr(sin_angle), 1.0 - 1e-4);
  DCHECK_LE(Sqr(cos_angle) + Sqr(sin_angle), 1.0 + 1e-4);
  std::vector<Vec2d> points;
  points.reserve(num_points());
  const Vec2d new_center = center + translation;
  for (const auto& pt : points_) {
    points.emplace_back(Vec2d(pt - center).Rotate(cos_angle, sin_angle) +
                        new_center);
  }
  return Polygon2d(std::move(points), is_convex());
}

Polygon2d Polygon2d::AffineTransform(const Vec2d& center, double cos_angle,
                                     double sin_angle,
                                     const Vec2d& translation) const {
  // Use large error tolerance to accept approximately computed cos/sin values.
  DCHECK_GE(Sqr(cos_angle) + Sqr(sin_angle), 1.0 - 1e-4);
  DCHECK_LE(Sqr(cos_angle) + Sqr(sin_angle), 1.0 + 1e-4);
  std::vector<Vec2d> points;
  points.reserve(num_points());
  for (const auto& pt : points_) {
    points.emplace_back(pt.Rotate(cos_angle, sin_angle) + translation);
  }
  return Polygon2d(std::move(points), is_convex());
}

Polygon2d Polygon2d::Shift(const Vec2d& shift_vec) const {
  std::vector<Vec2d> points;
  points.reserve(num_points());
  for (const auto& pt : points_) {
    points.emplace_back(pt + shift_vec);
  }
  return Polygon2d(std::move(points), is_convex());
}

Polygon2d Polygon2d::Rotate(double yaw) const {
  std::vector<Vec2d> points;
  points.reserve(num_points());
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  for (const auto& pt : points_) {
    points.push_back(pt.Rotate(cos_yaw, sin_yaw));
  }
  return Polygon2d(std::move(points), is_convex());
}

std::optional<Polygon2d> Polygon2d::MinkowskiSumWithVec(
    const Vec2d& vec) const {
  CHECK(is_convex());
  const int left_most_index = ExtremePoint(vec.Perp());
  const int right_most_index = ExtremePoint(-vec.Perp());
  constexpr double kParrallelEpsilon = 1e-9;
  const bool left_next_parrallel_with_vec =
      abs((points_[Next(left_most_index)] - points_[left_most_index])
              .CrossProd(vec)) <= kParrallelEpsilon;
  const bool left_prev_parrallel_with_vec =
      abs((points_[Prev(left_most_index)] - points_[left_most_index])
              .CrossProd(vec)) <= kParrallelEpsilon;
  const bool right_next_parrallel_with_vec =
      abs((points_[Next(right_most_index)] - points_[right_most_index])
              .CrossProd(vec)) <= kParrallelEpsilon;
  const bool right_prev_parrallel_with_vec =
      abs((points_[Prev(right_most_index)] - points_[right_most_index])
              .CrossProd(vec)) <= kParrallelEpsilon;

  std::vector<Vec2d> new_points;
  new_points.reserve(num_points_ + 2);

  bool reached_right = false;
  for (int index = left_most_index; !reached_right; index = Next(index)) {
    if (index == right_most_index) {
      reached_right = true;
    }
    if (index == left_most_index && left_next_parrallel_with_vec) {
      continue;
    }
    if (index == right_most_index && right_prev_parrallel_with_vec) {
      continue;
    }
    new_points.push_back(points_[index]);
  }

  bool reached_left = false;
  for (int index = right_most_index; !reached_left; index = Next(index)) {
    if (index == left_most_index) {
      reached_left = true;
    }
    if (index == left_most_index && left_prev_parrallel_with_vec) {
      continue;
    }
    if (index == right_most_index && right_next_parrallel_with_vec) {
      continue;
    }
    new_points.push_back(points_[index] + vec);
  }

  if (new_points.size() >= 3) {
    return Polygon2d(std::move(new_points), /*is_convex=*/true);
  } else {
    return {};
  }
}

std::string Polygon2d::DebugString() const {
  std::string points_str;
  for (const auto& point : points_) {
    points_str += point.DebugString();
  }
  return absl::StrCat("polygon2d (  num_points = ", num_points_, "  points = (",
                      points_str, " )  ", is_convex_ ? "convex" : "non-convex",
                      "  area = ", area_, " )");
}

Segment2d Polygon2d::GetPrincipalAxis() const {
  const auto box = MinAreaBoundingBox();
  double half_len = box.half_length();
  Vec2d heading(box.cos_heading(), box.sin_heading());
  if (box.length() < box.width()) {
    heading = heading.Perp();
    half_len = box.half_width();
  }
  return Segment2d(box.center() + heading * half_len,
                   box.center() - heading * half_len);
}

bool Polygon2d::IsPointInSlow(const Vec2d& point) const {
  CHECK_GE(points_.size(), 3);
  if (IsPointOnBoundary(point)) {
    return true;
  }
  int j = num_points_ - 1;
  int c = 0;
  for (int i = 0; i < num_points_; ++i) {
    if ((points_[i].y() > point.y()) != (points_[j].y() > point.y())) {
      const double side =
          Vec2d(points_[i] - point).CrossProd(points_[j] - point);
      if (points_[i].y() < points_[j].y() ? side > 0.0 : side < 0.0) {
        ++c;
      }
    }
    j = i;
  }
  return c & 1;
}

}  // namespace st
