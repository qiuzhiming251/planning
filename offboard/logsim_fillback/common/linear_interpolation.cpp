
#include <cmath>
#include <utility>

#include "common/linear_interpolation.h"

namespace worldview {
namespace util {
namespace math {

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double lerp_angle(const double a0, const double a1, const double w) {
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }
  const double a = a0_n + d * w;
  return NormalizeAngle(a);
}

double lerp_angle(const double a0, const double t0, const double a1,
                  const double t1, const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return NormalizeAngle(a0);
  }
  const double r = (t - t0) / (t1 - t0);
  return lerp_angle(a0, a1, r);
}

byd::msg::planning::TrajectoryPoint InterpolateUsingLinearApproximation(const byd::msg::planning::TrajectoryPoint &p0,
                                                    const byd::msg::planning::TrajectoryPoint &p1,
                                                    const double w) {
  byd::msg::planning::TrajectoryPoint tp;
  tp.set_x(lerp(p0.x(), p1.x(), w));
  tp.set_y(lerp(p0.y(), p1.y(), w));
  tp.set_z(lerp(p0.z(), p1.z(), w));
  tp.set_theta(lerp_angle(p0.theta(), p1.theta(), w));
  tp.set_kappa(lerp(p0.kappa(), p1.kappa(), w));
  tp.set_dkappa(lerp(p0.dkappa(), p1.dkappa(), w));
  tp.set_t(lerp(p0.t(), p1.t(), w));
  tp.set_v(lerp(p0.v(), p1.v(), w));
  tp.set_a(lerp(p0.a(), p1.a(), w));
  tp.set_s(lerp(p0.s(), p1.s(), w));
  tp.set_yaw_rate(lerp(p0.yaw_rate(), p1.yaw_rate(), w));
  tp.set_steering_angle(lerp(p0.steering_angle(), p1.steering_angle(), w));
  return tp;
}

}  // namespace math
}  // namespace planning
}  // namespace ad_byd
