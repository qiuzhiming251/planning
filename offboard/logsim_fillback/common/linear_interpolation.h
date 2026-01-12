

#ifndef AD_BYD_PLANNING_MATH_LINEAR_INTERPOLATION_H
#define AD_BYD_PLANNING_MATH_LINEAR_INTERPOLATION_H
#include <cmath>
// #include "common/vec2d.h"
#include "modules/msg/st_msgs/planning_result.pb.h"

namespace worldview {
namespace util {
namespace math {

inline constexpr double kMathEpsilon = 1e-10;

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t, bool clamp = false) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return x0;
  }
  if (clamp) {
    if (t0 <= t1) {
      if (t <= t0) {
        return x0;
      }
      if (t >= t1) {
        return x1;
      }
    } else {
      if (t >= t0) {
        return x0;
      }
      if (t <= t1) {
        return x1;
      }
    }
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param x1 The coordinate of the second point.
 * @param w The weight from t0 to t, normally: w = (t - t0) / (t1 - t0)
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T &x0, const T &x1, const double w) {
  const T x = x0 + w * (x1 - x0);
  return x;
}

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param a1 The value of the second angle.
 * @param w The interpolation parameter for interpolation.
 * @return The value of the spherically interpolated angle.
 */
double lerp_angle(const double a0, const double a1, const double w);

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @return The value of the spherically interpolated angle.
 */
double lerp_angle(const double a0, const double t0, const double a1,
                  const double t1, const double t);

byd::msg::planning::TrajectoryPoint InterpolateUsingLinearApproximation(const byd::msg::planning::TrajectoryPoint &p0,
                                                    const byd::msg::planning::TrajectoryPoint &p1,
                                                    const double w);

/**
 * @brief InterpolateUsingLinearApproximation two points of type T.
 * @param p0 The first point.
 * @param t0 The interpolation parameter of the first point.
 * @param p1 The second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param p The interpolated point.
 * @return Interpolated point.
 */
// template <typename T>
// T InterpolateUsingLinearApproximation(const T &p0, const double t0, const T &p1,
//                                       const double t1, const double t) {
//   if (std::abs(t1 - t0) <= kMathEpsilon) {
//     return p0;
//   }
//   const double r = (t - t0) / (t1 - t0);
//   return InterpolateUsingLinearApproximation(p0, p1, r);
// }
}  // namespace math
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MATH_LINEAR_INTERPOLATION_H