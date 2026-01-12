//
//#include "plan_common/math/vec3d.h"

//#include <cmath>
//#include <iostream>
//#include "plan_common/math/fast_math.h"

// namespace ad_byd {
// namespace planning {
// namespace math {

// constexpr double kMathEpsilon = 1e-10;

// double Vec3d::Length() const { return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
// }

// double Vec3d::LengthSquare() const { return x_ * x_ + y_ * y_ + z_ * z_; }

// double Vec3d::Yaw() const { return st::fast_math::Atan2(y_, x_); }

// void Vec3d::Normalize() {
//   const double l = Length();
//   if (l > kMathEpsilon) {
//     x_ /= l;
//     y_ /= l;
//     z_ /= l;
//   }
// }

// double Vec3d::DistanceTo(const Vec3d &other) const {
//   const double dx = x_ - other.x_;
//   const double dy = y_ - other.y_;
//   const double dz = z_ - other.z_;
//   return std::sqrt(dx * dx + dy * dy + dz + dz);
// }

// double Vec3d::DistanceSquareTo(const Vec3d &other) const {
//   const double dx = x_ - other.x_;
//   const double dy = y_ - other.y_;
//   const double dz = z_ - other.z_;
//   return dx * dx + dy * dy + dz + dz;
// }

// Vec3d Vec3d::CrossProd(const Vec3d &other) const {
//   const double cx = y_ * other.z() - other.y() * z_;
//   const double cy = z_ * other.x() - other.z() * x_;
//   const double cz = x_ * other.y() - other.x() * y_;
//   return {cx, cy, cz};
// }

// double Vec3d::InnerProd(const Vec3d &other) const { return x_ * other.x() +
// y_ * other.y() + z_ * other.z(); }

// Vec3d Vec3d::operator+(const Vec3d &other) const { return {x_ + other.x(), y_
// + other.y(), z_ + other.z()}; }

// Vec3d Vec3d::operator-(const Vec3d &other) const { return {x_ - other.x(), y_
// - other.y(), z_ - other.z()}; }

// Vec3d Vec3d::operator*(const double ratio) const { return {x_ * ratio, y_ *
// ratio, z_ * ratio}; }

// Vec3d Vec3d::operator/(const double ratio) const {
//   if (std::abs(ratio) < kMathEpsilon) {
//     std::cout << "Divide by zero, Vec3d::operator/ fail !" << std::endl;
//   }
//   return {x_ / ratio, y_ / ratio, z_ / ratio};
// }

// Vec3d &Vec3d::operator+=(const Vec3d &other) {
//   x_ += other.x();
//   y_ += other.y();
//   z_ += other.z();
//   return *this;
// }

// Vec3d &Vec3d::operator-=(const Vec3d &other) {
//   x_ -= other.x();
//   y_ -= other.y();
//   z_ -= other.z();
//   return *this;
// }

// Vec3d &Vec3d::operator*=(const double ratio) {
//   x_ *= ratio;
//   y_ *= ratio;
//   z_ *= ratio;
//   return *this;
// }

// Vec3d &Vec3d::operator/=(const double ratio) {
//   // CHECK_GT(std::abs(ratio), kMathEpsilon);
//   x_ /= ratio;
//   y_ /= ratio;
//   z_ /= ratio;
//   return *this;
// }

// bool Vec3d::operator==(const Vec3d &other) const {
//   return (std::abs(x_ - other.x()) < kMathEpsilon && std::abs(y_ - other.y())
//   < kMathEpsilon &&
//           std::abs(z_ - other.z()) < kMathEpsilon);
// }

// Vec3d operator*(const double ratio, const Vec3d &vec) { return vec * ratio; }

// std::string Vec3d::DebugString() const { return "Vec3d::DebugString"; }

// }  // namespace math
// }  // namespace planning
// }  // namespace ad_byd
