

#ifndef AD_BYD_PLANNING_MATH_VEC3D_H
#define AD_BYD_PLANNING_MATH_VEC3D_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>

#include "plan_common/math/vec.h"

namespace ad_byd {
namespace planning {
namespace math {

using Vec3d = st::Vec3<double>;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Quaterniond Quaterniond;
typedef Eigen::AngleAxisd AngleAxisd;
typedef Eigen::Isometry3d Isometry3d;
// /**
//  * @class Vec3d
//  *
//  * @brief Implements a class of 3-dimensional vectors.
//  */
// class Vec3d {
//  public:
//   //! Constructor which takes x- y- and z- coordinates.
//   constexpr Vec3d(const double x, const double y, const double z) noexcept :
//   x_(x), y_(y), z_(z) {}

//   //! Constructor returning the zero vector.
//   constexpr Vec3d() noexcept : Vec3d(0, 0, 0) {}

//   //! Getter for x component
//   double x() const { return x_; }

//   //! Getter for y component
//   double y() const { return y_; }

//   //! Getter for z component
//   double z() const { return z_; }

//   //! Setter for x component
//   void set_x(const double x) { x_ = x; }

//   //! Setter for y component
//   void set_y(const double y) { y_ = y; }

//   //! Setter for y component
//   void set_z(const double z) { z_ = z; }

//   //! Gets the length of the vector
//   double Length() const;

//   //! Gets the squared length of the vector
//   double LengthSquare() const;

//   //! Gets the x-y projection between the vector and the positive x semi-axis
//   double Yaw() const;

//   //! Returns the unit vector that is co-linear with this vector
//   void Normalize();

//   //! Returns the distance to the given vector
//   double DistanceTo(const Vec3d &other) const;

//   //! Returns the squared distance to the given vector
//   double DistanceSquareTo(const Vec3d &other) const;

//   //! Returns the "cross" product between these three Vec3d (non-standard).
//   Vec3d CrossProd(const Vec3d &other) const;

//   //! Returns the inner product between these three Vec3d.
//   double InnerProd(const Vec3d &other) const;

//   //! Sums three Vec3d
//   Vec3d operator+(const Vec3d &other) const;

//   //! Subtracts three Vec3d
//   Vec3d operator-(const Vec3d &other) const;

//   //! Multiplies Vec3d by a scalar
//   Vec3d operator*(const double ratio) const;

//   //! Divides Vec3d by a scalar
//   Vec3d operator/(const double ratio) const;

//   //! Sums another Vec3d to the current one
//   Vec3d &operator+=(const Vec3d &other);

//   //! Subtracts another Vec3d to the current one
//   Vec3d &operator-=(const Vec3d &other);

//   //! Multiplies this Vec3d by a scalar
//   Vec3d &operator*=(const double ratio);

//   //! Divides this Vec3d by a scalar
//   Vec3d &operator/=(const double ratio);

//   //! Compares three Vec3d
//   bool operator==(const Vec3d &other) const;

//   //! Returns a human-readable string representing this object
//   std::string DebugString() const;

//  protected:
//   double x_ = 0.0;
//   double y_ = 0.0;
//   double z_ = 0.0;
// };

// //! Multiplies the given Vec3d by a given scalar
// Vec3d operator*(const double ratio, const Vec3d &vec);

}  // namespace math
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MATH_VEC3D_H