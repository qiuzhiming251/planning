

// NOTE: This file is copied from Apollo project and modified by BYD.ai for
// its own use.

#ifndef ST_PLANNING_MATH_GEOMETRY_AABOX2D
#define ST_PLANNING_MATH_GEOMETRY_AABOX2D

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "plan_common/log.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace st {

/**
 * @class AABox2d
 * @brief Implements a class of (undirected) axes-aligned bounding boxes in 2-D.
 * This class is referential-agnostic.
 */
class AABox2d {
 public:
  /**
   * @brief Default constructor.
   * Creates an axes-aligned box with zero length and width at the origin.
   */
  AABox2d() = default;
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box with given center, length, and width.
   * @param center The center of the box
   * @param length The size of the box along the x-axis
   * @param width The size of the box along the y-axis
   */
  AABox2d(const Vec2d& center, const double length, const double width);

  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box with half_length, half_width, and a given
   * center.
   * @param half_length Half of the size of the box along the x-axis.
   * @param half_width Half of the size of the box along the y-axis.
   * @param center The center of the box.
   */
  AABox2d(double half_length, double half_width, const Vec2d& center);

  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box from two opposite corners.
   * @param one_corner One corner of the box
   * @param opposite_corner The opposite corner to the first one
   */
  AABox2d(const Vec2d& one_corner, const Vec2d& opposite_corner);
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box containing all points in a given vector.
   * @param points Vector of points to be included inside the box.
   */
  explicit AABox2d(absl::Span<const Vec2d> points);

  bool operator==(const AABox2d& o) const;

  /**
   * @brief Getter of center_
   * @return Center of the box
   */
  const Vec2d& center() const { return center_; }

  /**
   * @brief Getter of x-component of center_
   * @return x-component of the center of the box
   */
  double center_x() const { return center_.x(); }

  /**
   * @brief Getter of y-component of center_
   * @return y-component of the center of the box
   */
  double center_y() const { return center_.y(); }

  /**
   * @brief Getter of length_
   * @return The length of the box
   */
  double length() const { return half_length_ * 2.0; }

  /**
   * @brief Getter of width_
   * @return The width of the box
   */
  double width() const { return half_width_ * 2.0; }

  /**
   * @brief Getter of half_length_
   * @return Half of the length of the box
   */
  double half_length() const { return half_length_; }

  /**
   * @brief Getter of half_width_
   * @return Half of the width of the box
   */
  double half_width() const { return half_width_; }

  /**
   * @brief Getter of length_*width_
   * @return The area of the box
   */
  double area() const { return half_length_ * half_width_ * 4.0; }

  /**
   * @brief Returns the minimum x-coordinate of the box
   *
   * @return x-coordinate
   */
  double min_x() const { return center_.x() - half_length_; }

  /**
   * @brief Returns the maximum x-coordinate of the box
   *
   * @return x-coordinate
   */
  double max_x() const { return center_.x() + half_length_; }

  /**
   * @brief Returns the minimum y-coordinate of the box
   *
   * @return y-coordinate
   */
  double min_y() const { return center_.y() - half_width_; }

  /**
   * @brief Returns the maximum y-coordinate of the box
   *
   * @return y-coordinate
   */
  double max_y() const { return center_.y() + half_width_; }

  /**
   * @brief Returns the half size as a Vec2d.
   *
   * @return half size vector.
   */
  Vec2d half_size() const { return {half_length_, half_width_}; }

  /**
   * @brief Returns the half diagonal, which can be used as a conservative
   * radius in overlap pruning.
   *
   * @return half size vector.
   */
  double half_diagonal() const { return half_size().norm(); }

  /**
   * @brief Gets all corners in counter clockwise order.
   *
   * @param corners Output where the corners are written
   */
  void GetAllCorners(std::vector<Vec2d>* const corners) const;

  /**
   * @brief Gets all corners in counter clockwise order.
   *
   * @param corners Output where the corners are written
   */
  std::array<Vec2d, 4> GetAllCorners() const;

  /**
   * @brief Determines whether a given point is in the box.
   *
   * @param point The point we wish to test for containment in the box
   */
  bool IsPointIn(const Vec2d& point) const;

  /**
   * @brief Determines whether a given point is on the boundary of the box.
   *
   * @param point The point we wish to test for boundary membership
   */
  bool IsPointOnBoundary(const Vec2d& point) const;

  /**
   * @brief Determines whether a given aabox is in the aabox.
   *
   * @param other_aabox The aabox we wish to test for containment in the aabox
   */
  bool Contains(const AABox2d& other_aabox) const;

  /**
   * @brief Determines the distance between a point and the box.
   *
   * @param point The point whose distance to the box we wish to determine.
   */
  double DistanceTo(const Vec2d& point) const;

  /**
   * @brief Determines the squared distance between a point and the box.
   *
   * @param point The point whose squared distance to the box.
   */
  double SquaredDistanceTo(const Vec2d& point) const;

  /**
   * @brief Determines the distance between two boxes.
   *
   * @param box Another box.
   */
  double DistanceTo(const AABox2d& box) const;

  /**
   * @brief Determines whether two boxes overlap.
   *
   * @param box Another box
   */
  bool HasOverlap(const AABox2d& box) const {
    return std::abs(box.center_x() - center_.x()) <=
               box.half_length() + half_length_ &&
           std::abs(box.center_y() - center_.y()) <=
               box.half_width() + half_width_;
  }

  /**
   * @brief Determines whether an aabox overlaps a segment.
   *
   * @param line_segment Segment.
   */
  bool HasOverlap(const Segment2d& line_segmnet) const;

  /**
   * @brief Compute overlapping area between two boxes.
   *
   * @param box Another box
   */
  double ComputeOverlapArea(const AABox2d& box) const;

  /**
   * @brief Shift the center of AABox by the input vector.
   *
   * @param shift_vec The vector by which we wish to shift the box
   */
  void Shift(const Vec2d& shift_vec);

  /**
   * @brief Changes box to include another given box, as well as the current
   * one.
   *
   * @param other_box Another box
   */
  void MergeFrom(const AABox2d& other_box);

  /**
   * @brief Changes box to include a given point, as well as the current box.
   *
   * @param other_point Another point
   */
  void MergeFrom(const Vec2d& other_point);

  /**
   * @brief Gets a human-readable debug string
   *
   * @return A string
   */

  std::string DebugString() const;

 private:
  static constexpr double kEpsilon = 1e-10;

  Vec2d center_;
  double half_length_ = 0.0;
  double half_width_ = 0.0;

  template <typename Archive>
  friend void serialize(Archive& ar, AABox2d& aabox2d);
};

inline AABox2d::AABox2d(const Vec2d& center, double length, double width)
    : center_(center), half_length_(length * 0.5), half_width_(width * 0.5) {
  DCHECK_GT(half_length_, -kEpsilon);
  DCHECK_GT(half_width_, -kEpsilon);
}

inline AABox2d::AABox2d(double half_length, double half_width,
                        const Vec2d& center)
    : center_(center), half_length_(half_length), half_width_(half_width) {
  DCHECK_GT(half_length_, -kEpsilon);
  DCHECK_GT(half_width_, -kEpsilon);
}

inline AABox2d::AABox2d(const Vec2d& one_corner, const Vec2d& opposite_corner)
    : AABox2d((one_corner + opposite_corner) * 0.5,
              std::abs(one_corner.x() - opposite_corner.x()),
              std::abs(one_corner.y() - opposite_corner.y())) {}

inline AABox2d::AABox2d(absl::Span<const Vec2d> points) {
  CHECK(!points.empty());
  double min_x = points[0].x();
  double max_x = points[0].x();
  double min_y = points[0].y();
  double max_y = points[0].y();
  for (const auto& point : points) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }
  center_ = {(min_x + max_x) * 0.5, (min_y + max_y) * 0.5};
  half_length_ = (max_x - min_x) * 0.5;
  half_width_ = (max_y - min_y) * 0.5;
}

inline void AABox2d::GetAllCorners(std::vector<Vec2d>* const corners) const {
  CHECK_NOTNULL(corners)->clear();
  corners->reserve(4);
  corners->emplace_back(center_.x() + half_length_, center_.y() - half_width_);
  corners->emplace_back(center_.x() + half_length_, center_.y() + half_width_);
  corners->emplace_back(center_.x() - half_length_, center_.y() + half_width_);
  corners->emplace_back(center_.x() - half_length_, center_.y() - half_width_);
}

inline std::array<Vec2d, 4> AABox2d::GetAllCorners() const {
  return {Vec2d{center_.x() + half_length_, center_.y() - half_width_},
          Vec2d{center_.x() + half_length_, center_.y() + half_width_},
          Vec2d{center_.x() - half_length_, center_.y() + half_width_},
          Vec2d{center_.x() - half_length_, center_.y() - half_width_}};
}

inline bool AABox2d::operator==(const AABox2d& o) const {
  return center_ == o.center() && half_length_ == o.half_length() &&
         half_width_ == o.half_width_;
}

inline bool AABox2d::IsPointIn(const Vec2d& point) const {
  return std::abs(point.x() - center_.x()) <= half_length_ + kEpsilon &&
         std::abs(point.y() - center_.y()) <= half_width_ + kEpsilon;
}

inline bool AABox2d::IsPointOnBoundary(const Vec2d& point) const {
  const double dx = std::abs(point.x() - center_.x());
  const double dy = std::abs(point.y() - center_.y());
  return (std::abs(dx - half_length_) <= kEpsilon &&
          dy <= half_width_ + kEpsilon) ||
         (std::abs(dy - half_width_) <= kEpsilon &&
          dx <= half_length_ + kEpsilon);
}

inline bool AABox2d::Contains(const AABox2d& other_aabox) const {
  return !(min_x() > other_aabox.min_x() || min_y() > other_aabox.min_y() ||
           max_x() < other_aabox.max_x() || max_y() < other_aabox.max_y());
}

inline double AABox2d::DistanceTo(const Vec2d& point) const {
  const double dx = std::abs(point.x() - center_.x()) - half_length_;
  const double dy = std::abs(point.y() - center_.y()) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

inline double AABox2d::SquaredDistanceTo(const Vec2d& point) const {
  const double dx = std::abs(point.x() - center_.x()) - half_length_;
  const double dy = std::abs(point.y() - center_.y()) - half_width_;
  return Sqr(std::max(0.0, dx)) + Sqr(std::max(0.0, dy));
}

inline double AABox2d::DistanceTo(const AABox2d& box) const {
  const double dx =
      std::abs(box.center_x() - center_.x()) - box.half_length() - half_length_;
  const double dy =
      std::abs(box.center_y() - center_.y()) - box.half_width() - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

inline double AABox2d::ComputeOverlapArea(const AABox2d& box) const {
  if (!HasOverlap(box)) return 0;
  double overlap_area =
      (std::min(max_x(), box.max_x()) - std::max(min_x(), box.min_x())) *
      (std::min(max_y(), box.max_y()) - std::max(min_y(), box.min_y()));
  return overlap_area;
}

inline void AABox2d::Shift(const Vec2d& shift_vec) { center_ += shift_vec; }

inline void AABox2d::MergeFrom(const AABox2d& other_box) {
  const double x1 = std::min(min_x(), other_box.min_x());
  const double x2 = std::max(max_x(), other_box.max_x());
  const double y1 = std::min(min_y(), other_box.min_y());
  const double y2 = std::max(max_y(), other_box.max_y());
  center_ = Vec2d((x1 + x2) * 0.5, (y1 + y2) * 0.5);
  half_length_ = x2 - center_.x();
  half_width_ = y2 - center_.y();
}

inline void AABox2d::MergeFrom(const Vec2d& other_point) {
  const double x1 = std::min(min_x(), other_point.x());
  const double x2 = std::max(max_x(), other_point.x());
  const double y1 = std::min(min_y(), other_point.y());
  const double y2 = std::max(max_y(), other_point.y());
  center_ = Vec2d((x1 + x2) * 0.5, (y1 + y2) * 0.5);
  half_length_ = x2 - center_.x();
  half_width_ = y2 - center_.y();
}

inline std::string AABox2d::DebugString() const {
  return absl::StrCat("aabox2d(center = ", center_.DebugString(),
                      "  length = ", length(), "  width = ", width(), ")");
}

}  // namespace st

#endif  // ST_PLANNING_MATH_GEOMETRY_AABOX2D
