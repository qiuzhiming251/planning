

#ifndef AD_BYD_PLANNING_MATH_LINEAR_INTERPOLATION_H
#define AD_BYD_PLANNING_MATH_LINEAR_INTERPOLATION_H
#include <cmath>

#include "plan_common/type_def.h"
#include "plan_common/math/math_utils.h"

namespace ad_byd {
namespace planning {
namespace math {

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

/**
 * @brief InterpolateUsingLinearApproximation between two points.
 * @param p0 The first point.
 * @param p1 The second point.
 * @param w The weight from t0 to t, normally: w = (t - t0) / (t1 - t0)
 * @return The interpolated point.
 */
Vec2d InterpolateUsingLinearApproximation(const Vec2d &p0, const Vec2d &p1,
                                          const double w);

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w);

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double w);

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &p0,
                                                    const TrajectoryPoint &p1,
                                                    const double w);

SpeedPoint InterpolateUsingLinearApproximation(const SpeedPoint &p0,
                                               const SpeedPoint &p1,
                                               const double w);
FrenetPoint InterpolateUsingLinearApproximation(const FrenetPoint &p0,
                                                const FrenetPoint &p1,
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
template <typename T>
T InterpolateUsingLinearApproximation(const T &p0, const double t0, const T &p1,
                                      const double t1, const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return p0;
  }
  const double r = (t - t0) / (t1 - t0);
  return InterpolateUsingLinearApproximation(p0, p1, r);
}
}  // namespace math
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MATH_LINEAR_INTERPOLATION_H