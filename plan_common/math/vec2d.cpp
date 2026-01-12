//
//#include "plan_common/math/vec2d.h"
// #include "plan_common/math/fast_math.h"

//#include <cmath>
//#include <iostream>

// namespace ad_byd {
// namespace planning {
// namespace math {

// Vec2d Vec2d::CreateUnitVec2d(const double angle) { return {cos(angle),
// sin(angle)}; }

// double Vec2d::Length() const { return std::hypot(x_, y_); }

// double Vec2d::LengthSquare() const { return x_ * x_ + y_ * y_; }

// double Vec2d::Angle() const { return st::fast_math::Atan2(y_, x_); }

// void Vec2d::Normalize() {
//   const double l = Length();
//   if (l > kMathEpsilon) {
//     x_ /= l;
//     y_ /= l;
//   }
// }

// double Vec2d::DistanceTo(const Vec2d &other) const { return std::hypot(x_ -
// other.x_, y_ - other.y_); }

// double Vec2d::DistanceToLine(const Vec2d &start, const Vec2d &end) const {
//   const double dx = end.x() - start.x();
//   const double dy = end.y() - start.y();
//   double length = hypot(dx, dy);
//   if (length <= kMathEpsilon) {
//     return DistanceTo(start);
//   }
//   auto unit_direction = Vec2d(dx / length, dy / length);
//   const double x0 = x() - start.x();
//   const double y0 = y() - start.y();
//   return std::abs(x0 * unit_direction.y() - y0 * unit_direction.x());
// }

// double Vec2d::DistanceSquareTo(const Vec2d &other) const {
//   const double dx = x_ - other.x_;
//   const double dy = y_ - other.y_;
//   return dx * dx + dy * dy;
// }

// double Vec2d::CrossProd(const Vec2d &other) const { return x_ * other.y() -
// y_ * other.x(); }

// double Vec2d::InnerProd(const Vec2d &other) const { return x_ * other.x() +
// y_ * other.y(); }

// Vec2d Vec2d::rotate(const double angle) const {
//   return {x_ * cos(angle) - y_ * sin(angle), x_ * sin(angle) + y_ *
//   cos(angle)};
// }

// void Vec2d::SelfRotate(const double angle) {
//   double tmp_x = x_;
//   x_ = x_ * cos(angle) - y_ * sin(angle);
//   y_ = tmp_x * sin(angle) + y_ * cos(angle);
// }

// Vec2d Vec2d::operator+(const Vec2d &other) const { return {x_ + other.x(), y_
// + other.y()}; }

// Vec2d Vec2d::operator-(const Vec2d &other) const { return {x_ - other.x(), y_
// - other.y()}; }

// Vec2d Vec2d::operator*(const double ratio) const { return {x_ * ratio, y_ *
// ratio}; }

// Vec2d Vec2d::operator/(const double ratio) const {
//   if (std::abs(ratio) < kMathEpsilon) {
//     std::cout << "Divide by zero, Vec2d::operator/ fail !" << std::endl;
//   }
//   return {x_ / ratio, y_ / ratio};
// }
// Vec2d &Vec2d::operator+=(const Vec2d &other) {
//   x_ += other.x();
//   y_ += other.y();
//   return *this;
// }

// Vec2d &Vec2d::operator-=(const Vec2d &other) {
//   x_ -= other.x();
//   y_ -= other.y();
//   return *this;
// }

// Vec2d &Vec2d::operator*=(const double ratio) {
//   x_ *= ratio;
//   y_ *= ratio;
//   return *this;
// }

// Vec2d &Vec2d::operator/=(const double ratio) {
//   // CHECK_GT(std::abs(ratio), kMathEpsilon);
//   x_ /= ratio;
//   y_ /= ratio;
//   return *this;
// }

// bool Vec2d::operator==(const Vec2d &other) const {
//   return (std::abs(x_ - other.x()) < kMathEpsilon && std::abs(y_ - other.y())
//   < kMathEpsilon);
// }

// Vec2d operator*(const double ratio, const Vec2d &vec) { return vec * ratio; }

// std::string Vec2d::DebugString() const { return "Vec2d::DebugString"; }

// }  // namespace math
// }  // namespace planning
// }  // namespace ad_byd
