
#include <cmath>
#include <utility>

#include "plan_common/math/angle.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/fast_math.h"
namespace ad_byd {
namespace planning {
namespace math {
double Sqr(const double x) { return x * x; }

double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * x1 + y0 * y1;
}

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double AngleDiff(const double a1, const double a2) {
  return NormalizeAngle(a1 - a2);
}

bool IsAngleBetween(double target_angle, double start_angle,
                    double middle_angle, double end_angle, double buffer) {
  double d1 = NormalizeAngle(end_angle - middle_angle);
  double d2 = NormalizeAngle(start_angle - middle_angle);
  double d3 = NormalizeAngle(end_angle - start_angle);
  double d4 = NormalizeAngle(target_angle - start_angle);
  bool is_large_angle = d1 * d2 > 0.0 || std::abs(d1) + std::abs(d2) > M_PI;

  if (is_large_angle) {
    return !(d3 < 0.0 ? (d4 < buffer && d4 > d3 + buffer)
                      : (d4 > -buffer && d4 < d3 - buffer));
  }

  return d3 < 0.0 ? (d4 < buffer && d4 > d3 - buffer)
                  : (d4 > -buffer && d4 < d3 + buffer);
}

int RandomInt(const int s, const int t, unsigned int rand_seed) {
  if (s >= t) {
    return s;
  }
  return s + rand_r(&rand_seed) % (t - s + 1);
}

double RandomDouble(const double s, const double t, unsigned int rand_seed) {
  return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
}

// Gaussian
double Gaussian(const double u, const double std, const double x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2 * std * std));
}

std::pair<double, double> RotateVector2d(const double x_in, const double y_in,
                                         const double theta) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  double x = cos_theta * x_in - sin_theta * y_in;
  double y = sin_theta * x_in + cos_theta * y_in;

  return std::make_pair(x, y);
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = st::fast_math::Atan2(y, x);
  return std::make_pair(r, theta);
}

Vec2d RotateVector2d(const Vec2d &vec, const double &theta) {
  const double cos_angle = std::cos(theta);
  const double sin_angle = std::sin(theta);
  return Vec2d(cos_angle * vec.x() - sin_angle * vec.y(),
               sin_angle * vec.x() + cos_angle * vec.y());
}

TurnType GetTurnTypeByHeading(const double &entry_heading,
                              const double &exit_heading) {
  double diff = math::AngleDiff(exit_heading, entry_heading);
  if (std::fabs(diff) < M_PI / 6) {
    return TurnType::NO_TURN;
  } else if (std::fabs(math::AngleDiff(diff, M_PI)) < M_PI / 6) {
    return TurnType::U_TURN;
  } else if (diff > 0) {
    return TurnType::LEFT_TURN;
  } else {
    return TurnType::RIGHT_TURN;
  }
}

}  // namespace math
}  // namespace planning
}  // namespace ad_byd
