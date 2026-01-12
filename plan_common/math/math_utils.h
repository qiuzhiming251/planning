

#ifndef AD_BYD_PLANNING_MATH_MATH_UTIL_H
#define AD_BYD_PLANNING_MATH_MATH_UTIL_H

#include <cstdint>
#include <limits>
#include <list>
#include <utility>
#include <vector>

#include "plan_common/type_def.h"
#include "plan_common/math/vec2d.h"

namespace ad_byd {
namespace planning {
namespace math {
double Sqr(const double x);

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

/**
 * @brief Inner product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

/**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Inner product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Wrap angle to [0, 2 * PI).
 * @param angle the original value of the angle.
 * @return The wrapped value of the angle.
 */
double WrapAngle(const double angle);

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle);

/**
 * @brief Calculate the difference between angle from and to
 * @param a1 the angle1
 * @param a2 the angle2
 * @return The a1- a2. The range is between [-PI, PI).
 */
double AngleDiff(const double a1, const double a2);

/**
 * @brief Check if target angle is inside the angle range formed by start
 *        angle, middle angle and end angle.
 *        And input angles should within [-PI, PI).
 * @param target_angle Target angle to check.
 * @param start_angle Start angle of the angle range.
 * @param middle_angle Middle angle of the angle range. To confirm the
 *        actual range of angles from start angle to end angle.
 * @param end_angle End anlge of the angle range.
 * @param buffer Buffer to extend the angle range.
 * @return Check result that target angle is in angle ranges or not.
 */
bool IsAngleBetween(double target_angle, double start_angle,
                    double middle_angle, double end_angle, double buffer);

/**
 * @brief Get a random integer between two integer values by a random seed.
 * @param s The lower bound of the random integer.
 * @param t The upper bound of the random integer.
 * @param random_seed The random seed.
 * @return A random integer between s and t based on the input random_seed.
 */
int RandomInt(const int s, const int t, unsigned int rand_seed = 1);

/**
 * @brief Get a random double between two integer values by a random seed.
 * @param s The lower bound of the random double.
 * @param t The upper bound of the random double.
 * @param random_seed The random seed.
 * @return A random double between s and t based on the input random_seed.
 */
double RandomDouble(const double s, const double t, unsigned int rand_seed = 1);

TurnType GetTurnTypeByHeading(const double &entry_heading,
                              const double &exit_heading);

/**
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
template <typename T>
inline T Square(const T value) {
  return value * value;
}

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

// Gaussian
double Gaussian(const double u, const double std, const double x);

inline double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

// Rotate a 2d point counter-clockwise by theta
std::pair<double, double> RotateVector2d(const double x, const double y,
                                         const double theta);

Vec2d RotateVector2d(const Vec2d &vec, const double &theta);

inline std::pair<double, double> RFUToFLU(const double x, const double y) {
  return std::make_pair(y, -x);
}

inline std::pair<double, double> FLUToRFU(const double x, const double y) {
  return std::make_pair(-y, x);
}

inline void L2Norm(int feat_dim, float *feat_data) {
  if (feat_dim == 0) {
    return;
  }
  // feature normalization
  float l2norm = 0.0f;
  for (int i = 0; i < feat_dim; ++i) {
    l2norm += feat_data[i] * feat_data[i];
  }
  if (l2norm == 0) {
    float val = 1.f / std::sqrt(static_cast<float>(feat_dim));
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] = val;
    }
  } else {
    l2norm = std::sqrt(l2norm);
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] /= l2norm;
    }
  }
}

// Cartesian coordinates to Polar coordinates
std::pair<double, double> Cartesian2Polar(double x, double y);

// linear interpolation - increase series
template <typename T>
T interp1_inc(const std::vector<T> &X, const std::vector<T> &Y, T xp) {
  T temp1 = 0;
  uint16_t i;
  //
  if (X.size() != Y.size()) {
    // std::cout<< "ERROR input for interp1_inc()!" << std::endl;
    return 0;
  }
  //
  if (xp <= X.front()) {
    temp1 = Y.front();
  } else if (xp >= X.back()) {
    temp1 = Y.back();
  } else {
    for (i = 0; i < X.size() - 1; ++i) {
      if (xp >= X.at(i) && xp < X.at(i + 1)) {
        break;
      }
    }
    if (i == X.size() - 1) {
      i = i - 1;
    }
    temp1 = Y.at(i) +
            (Y.at(i + 1) - Y.at(i)) / (X.at(i + 1) - X.at(i)) * (xp - X.at(i));
  }
  return temp1;
}

// linear interpolation - decrease series
template <typename T>
T interp1_dec(std::vector<T> &X, std::vector<T> &Y, T xp) {
  T temp1 = 0;
  uint16_t i;
  //
  if (X.size() != Y.size()) {
    // std::cout<< "ERROR input for interp1_inc()!" << std::endl;
    return 0;
  }
  //
  if (xp >= X.front()) {
    temp1 = Y.front();
  } else if (xp <= X.back()) {
    temp1 = Y.back();
  } else {
    for (i = 0; i < X.size() - 1; ++i) {
      if (xp < X.at(i) && xp >= X.at(i + 1)) {
        break;
      }
    }
    if (i == X.size() - 1) {
      i = i - 1;
    }
    temp1 = Y.at(i) +
            (Y.at(i + 1) - Y.at(i)) / (X.at(i + 1) - X.at(i)) * (xp - X.at(i));
  }
  return temp1;
}

// RateLmt - rate limitation
template <typename T>
inline T RateLmt(T CurrentValue, const T &LastValue, T &MinLmt, T &MaxLmt) {
  if (MinLmt > MaxLmt) {
    std::swap(MinLmt, MaxLmt);
  }

  if (CurrentValue - LastValue > MaxLmt) {
    CurrentValue = LastValue + MaxLmt;
  } else if (CurrentValue - LastValue < MinLmt) {
    CurrentValue = LastValue + MinLmt;
  }
  return CurrentValue;
}

// Cubic - x^3
template <typename T>
inline T Cubic(const T value) {
  return value * value * value;
}

// interp2 xp,yp increase
template <typename T>
T interp2_inc(std::vector<T> &X, std::vector<T> &Y,
              std::vector<std::vector<T>> &Z, T xp, T yp) {
  std::vector<T> temp_z_line;
  std::vector<T> temp_y_line = Z.front();
  uint16_t size_x = X.size();
  uint16_t size_y = Y.size();
  if (size_y != Z.size()) {
    return 0;
  }
  //
  for (uint16_t iy = 0; iy < size_y; iy++) {
    temp_y_line = Z[iy];
    if (size_x != temp_y_line.size()) {
      return 0;
    }
    temp_z_line.push_back(interp1_inc(X, temp_y_line, xp));
  }
  return interp1_inc(Y, temp_z_line, yp);
}

template <typename T>
inline T SafeDivide(const T &divisor, T dividend) {
  const T eps = std::numeric_limits<T>::epsilon();
  if (std::abs(dividend) < eps) {
    dividend = eps;
  }
  return dividend == static_cast<T>(0) ? std::numeric_limits<T>::max()
                                       : divisor / dividend;
}

template <typename T>
inline T QuaternionToYaw(T w, T x, T y, T z) {
  return atan2(static_cast<T>(2.0) * (x * y + w * z),
               static_cast<T>(1.0) - static_cast<T>(2.0) * (y * y + z * z));
}
}  // namespace math
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MATH_MATH_UTIL_H
