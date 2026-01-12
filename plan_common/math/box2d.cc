//#include "box2d.h"

//#include <algorithm>
//#include <cmath>
//#include <iostream>

//#include "plan_common/math/angle.h"
//#include "plan_common/math/math_utils.h"

// namespace ad_byd {
// namespace planning {
// namespace math {

// Box2d::Box2d(const Vec2d &center, const double heading, const double length,
// const double width)
//     : center_(center),
//       length_(length),
//       width_(width),
//       half_length_(length / 2.0),
//       half_width_(width / 2.0),
//       heading_(heading),
//       cos_heading_(cos(Angle16::FromRad(heading))),
//       sin_heading_(sin(Angle16::FromRad(heading))) {
//   if (std::abs(length_) < -kMathEpsilon) {
//     std::cout << "length_ is negative in Box2d" << std::endl;
//   }
//   if (std::abs(width_) < -kMathEpsilon) {
//     std::cout << "width_ is negative in Box2d" << std::endl;
//   }
//   InitCorners();
// }

// void Box2d::InitCorners() {
//   const double dx1 = cos_heading_ * half_length_;
//   const double dy1 = sin_heading_ * half_length_;
//   const double dx2 = sin_heading_ * half_width_;
//   const double dy2 = -cos_heading_ * half_width_;
//   corners_.clear();
//   corners_.emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
//   corners_.emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
//   corners_.emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
//   corners_.emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);

//   for (auto &corner : corners_) {
//     max_x_ = std::fmax(corner.x(), max_x_);
//     min_x_ = std::fmin(corner.x(), min_x_);
//     max_y_ = std::fmax(corner.y(), max_y_);
//     min_y_ = std::fmin(corner.y(), min_y_);
//   }
// }

// Box2d Box2d::CreateAABox(const Vec2d &one_corner, const Vec2d
// &opposite_corner) {
//   const double x1 = std::min(one_corner.x(), opposite_corner.x());
//   const double x2 = std::max(one_corner.x(), opposite_corner.x());
//   const double y1 = std::min(one_corner.y(), opposite_corner.y());
//   const double y2 = std::max(one_corner.y(), opposite_corner.y());
//   return {{(x1 + x2) / 2.0, (y1 + y2) / 2.0}, 0.0, x2 - x1, y2 - y1};
// }

// void Box2d::GetAllCorners(std::vector<Vec2d> *const corners) const {
//   if (corners == nullptr) {
//     return;
//   }
//   *corners = corners_;
// }

// const std::vector<Vec2d> &Box2d::GetAllCorners() const { return corners_; }

// bool Box2d::IsPointIn(const Vec2d &point) const {
//   const double x0 = point.x() - center_.x();
//   const double y0 = point.y() - center_.y();
//   const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
//   const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
//   return dx <= half_length_ + kMathEpsilon && dy <= half_width_ +
//   kMathEpsilon;
// }

// bool Box2d::IsPointOnBoundary(const Vec2d &point) const {
//   const double x0 = point.x() - center_.x();
//   const double y0 = point.y() - center_.y();
//   const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
//   const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
//   return (std::abs(dx - half_length_) <= kMathEpsilon && dy <= half_width_ +
//   kMathEpsilon) ||
//          (std::abs(dy - half_width_) <= kMathEpsilon && dx <= half_length_ +
//          kMathEpsilon);
// }

// double Box2d::DistanceTo(const Vec2d &point) const {
//   const double x0 = point.x() - center_.x();
//   const double y0 = point.y() - center_.y();
//   const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_) -
//   half_length_; const double dy = std::abs(x0 * sin_heading_ - y0 *
//   cos_heading_) - half_width_; if (dx <= 0.0) {
//     return std::max(0.0, dy);
//   }
//   if (dy <= 0.0) {
//     return dx;
//   }
//   return hypot(dx, dy);
// }

// double Box2d::DistanceTo(const Box2d &box) const {
//   // has overlap or not
//   if (HasOverlap(box)) {
//     return 0.0;
//   }
//   // constexpr double kMathEpsilon = 1e-10;
//   double angel_0 = (box.center_ - center_).Angle();
//   Vec2d point_0 = box.GetSupportPoint(angel_0 - M_PI);
//   Vec2d point_1 = this->GetSupportPoint(angel_0);
//   double angel_1 = (math::AngleDiff(point_1.Angle(), angel_0) < 0) ? angel_0
//   + 0.5 * M_PI : angel_0 - 0.5 * M_PI; double angel_2 =
//   (math::AngleDiff(point_0.Angle() + M_PI, angel_0) < 0) ? angel_0 - 0.5 *
//   M_PI : angel_0 - 1.5 * M_PI;
//   // get distance
//   Vec2d point_2 = box.GetSupportPoint(angel_2);
//   Vec2d point_3 = this->GetSupportPoint(angel_1);
//   double distance = std::fmin(this->DistanceTo(point_0),
//   box.DistanceTo(point_1)); distance = std::fmin(this->DistanceTo(point_2),
//   distance); distance = std::fmin(box.DistanceTo(point_3), distance); return
//   distance;
// }

// Vec2d Box2d::GetSupportPoint(const double angle) const {
//   const double angle_ = math::AngleDiff(angle, this->heading_);
//   if (angle_ < -0.5 * M_PI) {
//     return corners_[3];
//   } else if (angle_ < kMathEpsilon) {
//     return corners_[0];
//   } else if (angle_ < 0.5 * M_PI) {
//     return corners_[1];
//   } else {
//     return corners_[2];
//   }
// }

// double Box2d::LazyDistanceTo(const Box2d &box) const {
//   if (HasOverlap(box)) {
//     return 0.0;
//   }
//   double distance = -1.0;
//   for (auto point : corners_) {
//     distance = (distance < -0.5) ? box.DistanceTo(point) : std::min(distance,
//     box.DistanceTo(point));
//   }
//   for (auto point : box.corners_) {
//     distance = std::min(this->DistanceTo(point), distance);
//   }
//   return distance;
// }

// bool Box2d::HasOverlap(const Box2d &box) const {
//   if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y()
//   || box.min_y() > max_y()) {
//     return false;
//   }

//   const double shift_x = box.center_x() - center_.x();
//   const double shift_y = box.center_y() - center_.y();

//   const double dx1 = cos_heading_ * half_length_;
//   const double dy1 = sin_heading_ * half_length_;
//   const double dx2 = sin_heading_ * half_width_;
//   const double dy2 = -cos_heading_ * half_width_;
//   const double dx3 = box.cos_heading() * box.half_length();
//   const double dy3 = box.sin_heading() * box.half_length();
//   const double dx4 = box.sin_heading() * box.half_width();
//   const double dy4 = -box.cos_heading() * box.half_width();

//   return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
//              std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) + std::abs(dx4
//              * cos_heading_ + dy4 * sin_heading_) +
//                  half_length_ &&
//          std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
//              std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) + std::abs(dx4
//              * sin_heading_ - dy4 * cos_heading_) +
//                  half_width_ &&
//          std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading())
//          <=
//              std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
//                  std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading())
//                  + box.half_length() &&
//          std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading())
//          <=
//              std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
//                  std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading())
//                  + box.half_width();
// }

// void Box2d::RotateFromCenter(const double rotate_angle) {
//   heading_ = NormalizeAngle(heading_ + rotate_angle);
//   const auto angle = math::Angle16::FromRad(heading_);
//   cos_heading_ = sin(angle);
//   sin_heading_ = cos(angle);
//   InitCorners();
// }

// void Box2d::Shift(const Vec2d &shift_vec) {
//   center_ += shift_vec;
//   InitCorners();
// }

// void Box2d::LongitudinalExtend(const double extension_length) {
//   length_ += extension_length;
//   half_length_ += extension_length / 2.0;
//   InitCorners();
// }

// void Box2d::LateralExtend(const double extension_length) {
//   width_ += extension_length;
//   half_width_ += extension_length / 2.0;
//   InitCorners();
// }

// }  // namespace math
// }  // namespace planning
// }  // namespace ad_byd
