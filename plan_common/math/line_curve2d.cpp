
#include "plan_common/log.h"
#include "plan_common/math/double.h"
#include "plan_common/math/line_curve2d.h"
#include "plan_common/math/linear_interpolation.h"

namespace ad_byd {
namespace planning {
namespace math {
using CompareType = Double::CompareType;
LineCurve2d::LineCurve2d(const std::vector<Vec2d> &points) {
  InitializePoints(points);
}

void LineCurve2d::InitializePoints(const std::vector<Vec2d> &points) {
  if (points.empty()) return;
  Clear();
  double s = 0.0;
  points_.emplace_back(points.front());
  accumulated_s_.emplace_back(s);
  headings_.emplace_back(0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    auto delta_pt = points[i] - points_.back();
    auto ds = delta_pt.Length();
    s += ds;
    points_.emplace_back(points[i]);
    accumulated_s_.emplace_back(s);
    headings_.emplace_back(delta_pt.Angle());
  }
}

Vec2d LineCurve2d::begin_point() const {
  if (points_.empty()) return {};
  return points_.front();
}

Vec2d LineCurve2d::end_point() const {
  if (points_.empty()) return {};
  return points_.back();
}

void LineCurve2d::Clear() {
  accumulated_s_.clear();
  points_.clear();
  headings_.clear();
}

Vec2d LineCurve2d::GetPointAtS(const double s) const {
  if (!IsValid()) return {};
  auto it = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (it == accumulated_s_.begin()) {
    return points_.front();
  } else if (it == accumulated_s_.end()) {
    return points_.back();
  } else {
    size_t idx = it - accumulated_s_.begin();
    auto w = (s - *(it - 1)) / (*it - *(it - 1));
    return math::InterpolateUsingLinearApproximation(points_[idx - 1],
                                                     points_[idx], w);
  }
}

int LineCurve2d::GetIndexByS(const double s) const {
  if (!IsValid()) return -1;
  if (s > accumulated_s_.back()) return accumulated_s_.size() - 1;
  if (s < accumulated_s_.front()) return 0;
  auto it = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  return (int)std::distance(accumulated_s_.begin(), it);
}

bool LineCurve2d::SamplePoints(const double start_s, const double length,
                               std::vector<Vec2d> *const points,
                               const double interval) const {
  if (!IsValid()) return false;
  if (Double::Compare(start_s, 0.0) == CompareType::LESS) {
    LINFO("can not sample points from negative start_s = %.2f", start_s);
    return false;
  }
  std::size_t idx = 0;
  Vec2d sample_pt;
  auto cur_s = start_s;
  double sample_end_s = std::min(start_s + length, accumulated_s_.back());
  while (Double::Compare(cur_s, sample_end_s) != CompareType::GREATER) {
    while (idx < accumulated_s_.size() &&
           Double::Compare(cur_s, accumulated_s_[idx]) ==
               CompareType::GREATER) {
      idx++;
    }
    if (idx == 0) {
      sample_pt = points_[idx];
    } else {
      auto w = (cur_s - accumulated_s_[idx - 1]) /
               (accumulated_s_[idx] - accumulated_s_[idx - 1]);
      sample_pt = math::InterpolateUsingLinearApproximation(points_[idx - 1],
                                                            points_[idx], w);
    }
    points->push_back(sample_pt);
    cur_s += interval;
  }
  return true;
}

double LineCurve2d::GetDistance(const double x, const double y) const {
  return GetDistance({x, y}, nullptr, nullptr);
}

double LineCurve2d::GetDistance(const Vec2d &point) const {
  return GetDistance(point, nullptr, nullptr);
}

double LineCurve2d::GetDistance(const Vec2d &point, Vec2d *nearest_point,
                                double *s_offset) const {
  double min_dist = std::numeric_limits<double>::max();
  Vec2d nearest_pt;
  if (!IsValid()) return min_dist;
  for (int32_t i = 0; i < static_cast<int32_t>(points_.size()) - 1; ++i) {
    const auto &start = points_[i];
    const auto &end = points_[i + 1];
    const double length = accumulated_s_[i + 1] - accumulated_s_[i];
    const math::Vec2d unit_direction = (end - start) / length;
    double dist_t = DBL_MAX;
    const double proj = unit_direction.InnerProd(point - start);
    if (length <= math::kMathEpsilon || proj < 0.0) {
      nearest_pt = start;
      dist_t = point.DistanceTo(start);
    } else if (proj > length) {
      nearest_pt = end;
      dist_t = point.DistanceTo(end);
    } else {
      nearest_pt = start + unit_direction * proj;
      dist_t = std::abs(unit_direction.CrossProd(point - start));
    }
    // get min_dist
    if (dist_t < min_dist) {
      min_dist = dist_t;
      if (nearest_point) {
        *nearest_point = nearest_pt;
      }
      if (s_offset) {
        *s_offset = accumulated_s_[i] + points_[i].DistanceTo(nearest_pt);
      }
    }
  }
  return min_dist;
}

bool LineCurve2d::GetProjection(const Vec2d &point, double *accumulate_s,
                                double *lateral) const {
  if (!IsValid() || accumulate_s == nullptr || lateral == nullptr) return false;
  double min_dist = std::numeric_limits<double>::infinity();
  int seg_num = static_cast<int>(points_.size() - 1);
  int min_index = 0;
  for (int i = 0; i < seg_num; ++i) {
    const auto &start = points_[i];
    const auto &end = points_[i + 1];
    const double length = accumulated_s_[i + 1] - accumulated_s_[i];
    if (math::Double::Compare(length, 0) == math::Double::CompareType::EQUAL) {
      continue;
    }
    const math::Vec2d unit_direction = (end - start) / length;
    double distance = DBL_MAX;
    const double proj = unit_direction.InnerProd(point - start);
    if (proj < 0.0) {
      distance = point.DistanceSquareTo(start);
    } else if (proj > length) {
      distance = point.DistanceSquareTo(end);
    } else {
      double prod = unit_direction.CrossProd(point - start);
      distance = prod * prod;
    }
    // get min_dist
    if (distance < min_dist) {
      min_index = i;
      min_dist = distance;
    }
  }
  min_dist = std::sqrt(min_dist);
  const auto &nearest_seg =
      math::LineSegment2d(points_[min_index], points_[min_index + 1]);
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else if (min_index == seg_num - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
  }
  return true;
}

bool LineCurve2d::GetPoint(const double s, const double l,
                           Vec2d *const point) const {
  if (!IsValid() || point == nullptr) return false;
  int index = 0;
  if (s < 0.0) {
    index = 0;
  } else if (s > accumulated_s_.back()) {
    index = static_cast<int>(points_.size() - 1) - 1;
  } else {
    int seg_num = static_cast<int>(points_.size() - 1);
    index = seg_num - 1;
    // TODO: using binary search to speed up computation
    for (int i = 0; i < seg_num; ++i) {
      if (accumulated_s_.at(i + 1) > s) {
        index = i;
        break;
      }
    }
  }

  const auto &nearest_seg =
      math::LineSegment2d(points_[index], points_[index + 1]);
  double proj = s - accumulated_s_.at(index);
  const auto &unit_direction = nearest_seg.unit_direction();
  const auto &proj_pt = nearest_seg.start() + unit_direction * proj;
  point->set_x(proj_pt.x() - unit_direction.y() * l);
  point->set_y(proj_pt.y() + unit_direction.x() * l);
  return true;
}

bool LineCurve2d::GetHeadingFromS(const double s, double *const heading) const {
  if (!IsValid() || headings_.size() < 2u) return false;
  if (s < 0.0) {
    *heading = headings_.at(1);
  } else if (s > accumulated_s_.back()) {
    *heading = headings_.back();
  } else {
    int seg_num = static_cast<int>(points_.size() - 1);
    int index = seg_num - 1;
    // TODO: using binary search to speed up computation
    for (int i = 0; i < seg_num; ++i) {
      if (accumulated_s_.at(i + 1) > s) {
        index = i + 1;
        break;
      }
    }
    *heading = headings_.at(index);
  }
  return true;
}

}  // namespace math
}  // namespace planning
}  // namespace ad_byd