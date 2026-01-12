

#ifndef AD_BYD_PLANNING_MATH_ANGLE_H
#define AD_BYD_PLANNING_MATH_ANGLE_H

#include <cmath>
#include <cstdint>
#include <limits>

namespace ad_byd {
namespace planning {
namespace math {

template <typename T>
class Angle {
 public:
  static_assert(std::numeric_limits<T>::is_integer &&
                    std::numeric_limits<T>::is_signed,
                "T must be a signed integer type");

  static constexpr T RAW_PI = std::numeric_limits<T>::min();
  static constexpr T RAW_PI_2 = -(std::numeric_limits<T>::min() >> 1);
  static constexpr double DEG_TO_RAW = RAW_PI / -180.0;
  static constexpr double RAD_TO_RAW = RAW_PI * -M_1_PI;
  static constexpr double RAW_TO_DEG = -180.0 / RAW_PI;
  static constexpr double RAW_TO_RAD = -M_PI / RAW_PI;

  // Construct an object from an angle in degrees.
  static Angle FromDeg(double value) {
    Angle a(lround(value * DEG_TO_RAW));
    return a;
  }

  // Construct an object from an angle in radians.
  static Angle FromRad(double value) {
    Angle a(lround(value * RAD_TO_RAW));
    return a;
  }

  explicit Angle(T value = 0) : _value(value) {}

  double ToDeg() const { return _value * RAW_TO_DEG; }

  double ToRad() const { return _value * RAW_TO_RAD; }

  T raw() const { return _value; }

  Angle operator+=(Angle other) {
    _value += other._value;
    return *this;
  }

  Angle operator-=(Angle other) {
    _value -= other._value;
    return *this;
  }

  template <typename Scalar>
  Angle operator*=(Scalar s) {
    _value = lround(_value * s);
    return *this;
  }

  template <typename Scalar>
  Angle operator/=(Scalar s) {
    _value = lround(_value / s);
    return *this;
  }

 private:
  T _value;
};

using Angle8 = Angle<int8_t>;
using Angle16 = Angle<int16_t>;
using Angle32 = Angle<int32_t>;
using Angle64 = Angle<int64_t>;

template <typename T>
Angle<T> operator+(Angle<T> lhs, Angle<T> rhs) {
  lhs += rhs;
  return lhs;
}

template <typename T>
Angle<T> operator-(Angle<T> lhs, Angle<T> rhs) {
  lhs -= rhs;
  return lhs;
}

template <typename T, typename Scalar>
Angle<T> operator*(Angle<T> lhs, Scalar rhs) {
  lhs *= rhs;
  return lhs;
}

template <typename T, typename Scalar>
Angle<T> operator*(Scalar lhs, Angle<T> rhs) {
  rhs *= lhs;
  return rhs;
}

template <typename T, typename Scalar>
Angle<T> operator/(Angle<T> lhs, Scalar rhs) {
  lhs /= rhs;
  return lhs;
}

template <typename T>
double operator/(Angle<T> lhs, Angle<T> rhs) {
  return double(lhs.raw()) / rhs.raw();
}

template <typename T>
bool operator==(Angle<T> lhs, Angle<T> rhs) {
  return lhs.raw() == rhs.raw();
}

template <typename T>
bool operator!=(Angle<T> lhs, Angle<T> rhs) {
  return !(lhs == rhs);
}

// Fast trigonometric functions. Single precision is sufficient for Angle16 and
// Angle8.
float sin(Angle16 a);
float cos(Angle16 a);
float tan(Angle16 a);
float sin(Angle8 a);
float cos(Angle8 a);
float tan(Angle8 a);

}  // namespace math
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MATH_ANGLE_H