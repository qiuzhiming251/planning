
#include <cmath>

#include "plan_common/math/linear_interpolation.h"

namespace ad_byd {
namespace planning {
namespace math {

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

Vec2d InterpolateUsingLinearApproximation(const Vec2d &p0, const Vec2d &p1,
                                          const double w) {
  Vec2d p;
  p.set_x((1 - w) * p0.x() + w * p1.x());
  p.set_y((1 - w) * p0.y() + w * p1.y());
  return p;
}

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w) {
  SLPoint p;
  p.s = (1 - w) * p0.s + w * p1.s;
  p.l = (1 - w) * p0.l + w * p1.l;
  return p;
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double w) {
  PathPoint path_point;
  path_point.set_x(lerp(p0.x(), p1.x(), w));
  path_point.set_y(lerp(p0.y(), p1.y(), w));
  path_point.theta = lerp_angle(p0.theta, p1.theta, w);
  path_point.kappa = lerp(p0.kappa, p1.kappa, w);
  path_point.dkappa = lerp(p0.dkappa, p1.dkappa, w);
  path_point.l = lerp(p0.l, p1.l, w);
  path_point.s = lerp(p0.s, p1.s, w);
  path_point.accum_s = lerp(p0.accum_s, p1.accum_s, w);
  return path_point;
}

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &p0,
                                                    const TrajectoryPoint &p1,
                                                    const double w) {
  TrajectoryPoint tp;
  tp.set_x(lerp(p0.x(), p1.x(), w));
  tp.set_y(lerp(p0.y(), p1.y(), w));
  tp.theta = lerp_angle(p0.theta, p1.theta, w);
  tp.kappa = lerp(p0.kappa, p1.kappa, w);
  tp.dkappa = lerp(p0.dkappa, p1.dkappa, w);
  tp.l = lerp(p0.l, p1.l, w);
  tp.s = lerp(p0.s, p1.s, w);
  tp.accum_s = lerp(p0.accum_s, p1.accum_s, w);
  tp.t = lerp(p0.t, p1.t, w);
  tp.v = lerp(p0.v, p1.v, w);
  tp.a = lerp(p0.a, p1.a, w);
  tp.j = lerp(p0.j, p1.j, w);
  return tp;
}

SpeedPoint InterpolateUsingLinearApproximation(const SpeedPoint &p0,
                                               const SpeedPoint &p1,
                                               const double w) {
  SpeedPoint p;
  p.t = lerp(p0.t, p1.t, w);
  p.s = lerp(p0.s, p1.s, w);
  p.v = lerp(p0.v, p1.v, w);
  p.a = lerp(p0.a, p1.a, w);
  return p;
}
FrenetPoint InterpolateUsingLinearApproximation(const FrenetPoint &p0,
                                                const FrenetPoint &p1,
                                                const double w) {
  FrenetPoint p;
  p.s = lerp(p0.s, p1.s, w);
  p.ds = lerp(p0.ds, p1.ds, w);
  p.dds = lerp(p0.dds, p1.dds, w);
  p.l = lerp(p0.l, p1.l, w);
  p.dl = lerp(p0.dl, p1.dl, w);
  p.ddl = lerp(p0.ddl, p1.ddl, w);
  return p;
}

}  // namespace math
}  // namespace planning
}  // namespace ad_byd
