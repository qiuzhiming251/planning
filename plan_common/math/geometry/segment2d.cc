

// NOTE: This file is copied from Apollo project and modified by BYD.ai for
// its own use.

#include <algorithm>
#include <utility>

#include "absl/strings/str_cat.h"
#include "plan_common/log.h"
#include "plan_common/math/double.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/util.h"
namespace st {

Vec2d Segment2d::rotate_expand(const double angle, const double length) const {
  Vec2d diff_vec = end_ - start_;
  diff_vec.SelfRotate(angle);
  diff_vec.Normalize();
  return start_ + diff_vec * length;
}

void Segment2d::Reset(const Vec2d& start, const Vec2d& end) {
  start_ = start;
  end_ = end;
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (ad_byd::planning::math::Double::Compare(length_, 0.0) !=
               ad_byd::planning::math::Double::CompareType::GREATER
           ? Vec2d(0, 0)
           : Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.Angle();
}

double Segment2d::DistanceTo(const Vec2d& point,
                             Vec2d* const nearest_pt) const {
  if (length_ <= kEpsilon) {
    if (nearest_pt != nullptr) *nearest_pt = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    if (nearest_pt != nullptr) *nearest_pt = start_;
    return Hypot(x0, y0);
  }
  if (proj > length_) {
    if (nearest_pt != nullptr) *nearest_pt = end_;
    return point.DistanceTo(end_);
  }
  if (nearest_pt != nullptr) *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double Segment2d::DistanceTo(const Segment2d& segment) const {
  Vec2d nearest_pt, other_nearest_pt;
  return DistanceTo(segment, &nearest_pt, &other_nearest_pt);
}

double Segment2d::DistanceTo(const Segment2d& segment, Vec2d* nearest_pt,
                             Vec2d* other_nearest_pt) const {
  Vec2d intersection_point;
  if (GetIntersect(segment, &intersection_point)) {
    if (nearest_pt != nullptr) *nearest_pt = intersection_point;
    if (other_nearest_pt != nullptr) *other_nearest_pt = intersection_point;
    return 0.0;
  }

  Vec2d nearest_pt_start, nearest_pt_end;
  const double dist_start = DistanceTo(segment.start(), &nearest_pt_start);
  const double dist_end = DistanceTo(segment.end(), &nearest_pt_end);

  Vec2d other_nearest_pt_start, other_nearest_pt_end;
  const double other_dist_start =
      segment.DistanceTo(start(), &other_nearest_pt_start);
  const double other_dist_end =
      segment.DistanceTo(end(), &other_nearest_pt_end);

  const double min_dist = std::min(std::min(dist_start, dist_end),
                                   std::min(other_dist_start, other_dist_end));
  if (min_dist == dist_start) {
    if (nearest_pt != nullptr) *nearest_pt = nearest_pt_start;
    if (other_nearest_pt != nullptr) *other_nearest_pt = segment.start();
  } else if (min_dist == dist_end) {
    if (nearest_pt != nullptr) *nearest_pt = nearest_pt_end;
    if (other_nearest_pt != nullptr) *other_nearest_pt = segment.end();
  } else if (min_dist == other_dist_start) {
    if (nearest_pt != nullptr) *nearest_pt = start();
    if (other_nearest_pt != nullptr) *other_nearest_pt = other_nearest_pt_start;
  } else if (min_dist == other_dist_end) {
    if (nearest_pt != nullptr) *nearest_pt = end();
    if (other_nearest_pt != nullptr) *other_nearest_pt = other_nearest_pt_end;
  }
  return min_dist;
}

bool Segment2d::HasIntersect(const Segment2d& other_segment) const {
  if (IsPointIn(other_segment.start()) || IsPointIn(other_segment.end()) ||
      other_segment.IsPointIn(start_) || other_segment.IsPointIn(end_)) {
    return true;
  }
  if (length_ <= kEpsilon || other_segment.length() <= kEpsilon) {
    return false;
  }
  const double cc1 = CrossProd(start_, end_, other_segment.start());
  const double cc2 = CrossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kEpsilon) {
    return false;
  }
  const double cc3 =
      CrossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      CrossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kEpsilon) {
    return false;
  }
  return true;
}

bool Segment2d::GetIntersect(const Segment2d& other_segment,
                             Vec2d* const point) const {
  CHECK_NOTNULL(point);
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= kEpsilon || other_segment.length() <= kEpsilon) {
    return false;
  }
  const double cc1 = CrossProd(start_, end_, other_segment.start());
  const double cc2 = CrossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kEpsilon) {
    return false;
  }
  const double cc3 =
      CrossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      CrossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kEpsilon) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double Segment2d::GetPerpendicularFoot(const Vec2d& point,
                                       Vec2d* const foot_point) const {
  CHECK_NOTNULL(foot_point);
  if (length_ <= kEpsilon) {
    *foot_point = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

void Segment2d::Reverse() {
  std::swap(start_, end_);
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  unit_direction_ =
      (length_ <= kEpsilon ? Vec2d(0, 0) : Vec2d(dx / length_, dy / length_));
}

void Segment2d::ClampByYMax(double y_max) {
  const double seg_y_min = min_y();
  const double seg_y_max = max_y();
  CHECK_LE(seg_y_min, y_max);

  if (seg_y_max > y_max + kEpsilon) {
    const double ratio_from_start =
        (y_max - start_.y()) / (end_.y() - start_.y());
    const double x_at_y_max = Lerp(start_.x(), end_.x(), ratio_from_start);

    if (start_.y() < end_.y()) {
      end_ = Vec2d{x_at_y_max, y_max};
      length_ *= ratio_from_start;
    } else {
      start_ = Vec2d{x_at_y_max, y_max};
      length_ *= (1.0 - ratio_from_start);
    }
  }  // else do nothing.
}

void Segment2d::ClampByYMin(double y_min) {
  const double seg_y_min = min_y();
  const double seg_y_max = max_y();
  CHECK_GE(seg_y_max, y_min);

  if (seg_y_min < y_min - kEpsilon) {
    const double ratio_from_start =
        (y_min - start_.y()) / (end_.y() - start_.y());
    const double x_at_y_min = Lerp(start_.x(), end_.x(), ratio_from_start);

    if (start_.y() > end_.y()) {
      end_ = Vec2d{x_at_y_min, y_min};
      length_ *= ratio_from_start;
    } else {
      start_ = Vec2d{x_at_y_min, y_min};
      length_ *= (1.0 - ratio_from_start);
    }
  }  // else do nothing.
}

void Segment2d::Shift(const Vec2d& offset) {
  start_ += offset;
  end_ += offset;
}

void Segment2d::Scale(double gain) {
  DCHECK_GE(gain, 0.0);
  DCHECK(std::isfinite(gain));
  end_ = Lerp(start_, end_, gain);
  length_ *= gain;
}

std::string Segment2d::DebugString() const {
  return absl::StrCat("Segment2d(/*start=*/", start_.DebugString(),
                      ", /*end=*/", end_.DebugString(), ")");
}

std::string Segment2d::DebugStringFullPrecision() const {
  return absl::StrCat("Segment2d(/*start=*/", start_.DebugStringFullPrecision(),
                      ", /*end=*/", end_.DebugStringFullPrecision(), ")");
}

}  // namespace st
