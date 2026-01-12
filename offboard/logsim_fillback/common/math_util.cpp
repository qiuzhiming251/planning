#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <set>

#include "common/math_util.h"
namespace worldview {
namespace util {
double sgn(double v) { return v > 0 ? 1 : -1; }

bool doubleEqual(double x, double y, double delta) {
  return std::abs(x - y) < delta;
}

bool doubleLessEqual(double x, double y, double delta) {
  return x < y || doubleEqual(x, y, delta);
}

bool doubleGreaterEqual(double x, double y, double delta) {
  return doubleLessEqual(y, x, delta);
}

double linearInterp(double x1, double x2, double y1, double y2, double x) {
  return (y2 - y1) / (x2 - x1) * (x - x1) + y1;
}
double linearInterp(double y1, double y2, double proportion) {
  return (y2 - y1) * proportion + y1;
}

Quaternion::Quaternion(const Euler& euler) {
  double cr = std::cos(euler.roll / 2);
  double sr = std::sin(euler.roll / 2);
  double cp = std::cos(euler.pitch / 2);
  double sp = std::sin(euler.pitch / 2);
  double cy = std::cos(euler.yaw / 2);
  double sy = std::sin(euler.yaw / 2);

  w = (cr * cp * cy + sr * sp * sy);
  x = (sr * cp * cy - cr * sp * sy);
  y = (cr * sp * cy + sr * cp * sy);
  z = (cr * cp * sy - sr * sp * cy);
}

Quaternion Euler::toQuaternion() const { return Quaternion(*this); }
Euler Quaternion::toEuler() const { return Euler(*this); }

Euler::Euler(const Quaternion& qte) {
  roll = std::atan2(2 * (qte.w * qte.x + qte.y * qte.z),
                    1 - 2 * (qte.x * qte.x + qte.y * qte.y));
  double sinp = 2 * (qte.w * qte.y - qte.z * qte.x);
  pitch = 0;
  if (sinp > 1) {
    pitch = M_PI / 2.0;
  } else if (sinp < -1) {
    pitch = -M_PI / 2;
  } else {
    pitch = std::asin(sinp);
  }
  yaw = std::atan2(2 * (qte.w * qte.z + qte.x * qte.y),
                   1 - 2 * (qte.y * qte.y + qte.z * qte.z));
}

double polynomial(const std::vector<double>& coeffs, double x, bool constAt0) {
  double res = 0;
  for (size_t i = 0; i < coeffs.size(); i++) {
    if (constAt0) {
      res = res * x + coeffs[coeffs.size() - 1 - i];
    } else {
      res = res * x + coeffs[i];
    }
  }
  return res;
}

double polydiff(const std::vector<double>& coeffs, double x, bool constAt0) {
  std::vector<double> diff_coeffs;
  for (size_t i = 1; i < coeffs.size(); i++) {
    if (constAt0) {
      diff_coeffs.push_back(coeffs[i] * i);
    } else {
      diff_coeffs.push_back(coeffs[coeffs.size() - 1 - i] * i);
    }
  }
  return polynomial(diff_coeffs, x, true);
}

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

std::vector<Point2D> pointsTranslation(const std::vector<Point2D>& points,
                                       double l) {
  if (doubleEqual(l, 0)) {
    return points;
  }
  std::vector<Point2D> points_l;
  for (int i = 0; i < static_cast<int>(points.size()); i++) {
    int key = i == 0 ? 1 : i;
    const auto& p0 = points[key - 1];
    const auto& p1 = points[key];
    auto dir = (p1 - p0).unit();
    Point2D point = points[i] + Point2D{-dir.y, dir.x} * l;
    points_l.push_back(point);
  }
  return points_l;
}
std::vector<Point2D> pointsTranslation(const std::vector<Point2D>& points,
                                       double l_s, double l_e) {
  if (doubleEqual(l_s, 0) && doubleEqual(l_e, 0)) {
    return points;
  }
  double total_s = util::splineLength(points);
  double current_s = 0;
  std::vector<Point2D> points_l;
  int length = static_cast<int>(points.size());
  for (int i = 0; i < length; i++) {
    double l = l_s + (l_e - l_s) * (current_s / total_s);
    if (i < length - 1) {
      current_s += (points[i + 1] - points[i]).norm();
    }
    const auto& p0 = points[std::max(i - 1, 0)];
    const auto& p1 = points[std::min(i + 1, length - 1)];
    auto dir = (p1 - p0).unit();
    Point2D point = points[i] + Point2D{-dir.y, dir.x} * l;
    points_l.push_back(point);
  }
  return points_l;
}
std::vector<Point2D> straightSpline(const Point2D& p0, double arg,
                                    double length, double step, bool reverse) {
  return straightSpline(p0, Point2D::unit(arg), length, step, reverse);
}

std::vector<Point2D> straightSplineWithEnds(const Point2D& p0,
                                            const Point2D& p1, double step,
                                            bool reverse) {
  auto link = p1 - p0;
  return straightSpline(p0, link.unit(), link.norm(), step, reverse);
}

std::vector<Point2D> straightSpline(const Point2D& p0, const Point2D& dir,
                                    double length, double step, bool reverse) {
  double cos_arg = dir.x;
  double sin_arg = dir.y;

  std::vector<Point2D> res;

  DOUBLE_LOOP(s, 0, length, step, 1e-6) {
    res.push_back(p0 + Point2D(cos_arg, sin_arg) * s);
  }

  if (reverse) {
    std::reverse(res.begin(), res.end());
  }

  return res;
}

/*
  s = a + b * t + c * t^2 + d * t^3 + e * t^4 + f * t^5

  s0 = a + b * t0 +  c * t0^2 +  d * t0^3 +   e * t0^4 +   f * t0^5
  s1 = a + b * t1 +  c * t1^2 +  d * t1^3 +   e * t1^4 +   f * t1^5
  v0 =     b      + 2c * t0   + 3d * t0^2 +  4e * t0^3 +  5f * t0^4
  v1 =     b      + 2c * t1   + 3d * t1^2 +  4e * t1^3 +  5f * t1^4
  a0 =              2c        + 6d * t0   + 12e * t0^2 + 20f * t0^3
  a1 =              2c        + 6d * t1   + 12e * t1^2 + 20f * t1^3
*/

std::vector<double> quinticPolynomial(double t0, double s0, double v0,
                                      double a0, double t1, double s1,
                                      double v1, double a1) {
  Eigen::MatrixXd A(6, 6);
  Eigen::VectorXd b(6), parameters(6);
  double t0_2 = t0 * t0;
  double t0_3 = t0 * t0 * t0;
  double t0_4 = t0 * t0 * t0 * t0;
  double t0_5 = t0 * t0 * t0 * t0 * t0;
  double t1_2 = t1 * t1;
  double t1_3 = t1 * t1 * t1;
  double t1_4 = t1 * t1 * t1 * t1;
  double t1_5 = t1 * t1 * t1 * t1 * t1;

  A << 1, t0, t0_2, t0_3, t0_4, t0_5, 1, t1, t1_2, t1_3, t1_4, t1_5, 0, 1,
      2 * t0, 3 * t0_2, 4 * t0_3, 5 * t0_4, 0, 1, 2 * t1, 3 * t1_2, 4 * t1_3,
      5 * t1_4, 0, 0, 2, 6 * t0, 12 * t0_2, 20 * t0_3, 0, 0, 2, 6 * t1,
      12 * t1_2, 20 * t1_3;
  b << s0, s1, v0, v1, a0, a1;
  parameters = A.colPivHouseholderQr().solve(b);

  std::vector<double> coeff{parameters[0], parameters[1], parameters[2],
                            parameters[3], parameters[4], parameters[5]};

  return coeff;
}

/*
  x = a_x + b_x * t + c_x * t^2
  y = a_y + b_y * t + c_y * t^2

  x_0 = a_x
  y_0 = a_y
  x_1 = a_x + b_x + c_x
  y_1 = a_y + b_y + c_y

  dy/dx_0 = b_y/b_x = tg(arg0)
  dy/dx_1 = (b_y + 2 * c_y) / (b_x + 2 * c_x) = tg(arg1)
     (b_x  c_x    b_y   c_y)
      1    1      0     0    x_1 - x_0
      0    0      1     1    y_1 - y_0
   sin0    0  -cos0    0     0
  sin1 2sin1 -cos1 -2cos1    0

  determinant = -sin(a0-a1)
*/

std::vector<Point2D> parabolicSpline(const Point2D& p0, const Point2D& p1,
                                     double arg0, double arg1, double step,
                                     bool reverse) {
  return parabolicSpline(p0, p1, Point2D::unit(arg0), Point2D::unit(arg1), step,
                         reverse);
}

std::vector<Point2D> parabolicSpline(const Point2D& p0, const Point2D& p1,
                                     const Point2D& dir0, const Point2D& dir1,
                                     double step, bool reverse) {
  double sin0 = dir0.y;
  double cos0 = dir0.x;
  double sin1 = dir1.y;
  double cos1 = dir1.x;

  double num_steps = std::ceil((p1 - p0).norm() / step);
  double unified_step = std::min(1 / num_steps, 0.1);

  Eigen::Matrix4d A;
  Eigen::Vector4d b, parameters;
  A << 1, 1, 0, 0, 0, 0, 1, 1, sin0, 0, -cos0, 0, sin1, 2 * sin1, -cos1,
      -2 * cos1;
  b << p1.x - p0.x, p1.y - p0.y, 0, 0;
  parameters = A.colPivHouseholderQr().solve(b);

  std::vector<double> coeff_x{p0.x, parameters[0], parameters[1]};
  std::vector<double> coeff_y{p0.y, parameters[2], parameters[3]};

  std::vector<Point2D> res;
  DOUBLE_LOOP(p, 0, 1, unified_step, 1e-6) {
    res.emplace_back(util::polynomial(coeff_x, p),
                     util::polynomial(coeff_y, p));
  }
  if (reverse) {
    std::reverse(res.begin(), res.end());
  }
  return res;
}

/**
 * x = a_x + b_x * t
 * y = a_y + b_y * t + c_y * t^2 + d_y * t^3
 *
 * x(0) = a_x = x_0
 * y(0) = a_y = y_0
 * x(1) = a_x + b_x = x_1
 * y(1) = a_y + b_y + c_y + d_y = y_1
 * dy/dx(0) = b_y / b_x = tan0
 * dy/dx(1) = (b_y + 2 * c_y + 3 * d_y) / b_x = tan1
 *
 *  b_x    b_y     c_y     d_y
 *    1      0       0       0                x_1 - x_0
 *    0      1       1       1                y_1 - y_0
 * sin0  -cos0       0       0                        0
 * sin1  -cos1 -2*cos1 -3*cos1                        0
 *
 *  determinant = -cos0*cos1
 *
 * TODO: not alway a good result.
 */

std::vector<Point2D> cubicSpline(const Point2D& p0, const Point2D& p1,
                                 double arg0, double arg1, double step,
                                 bool reverse) {
  return cubicSpline(p0, p1, Point2D::unit(arg0), Point2D::unit(arg1), step,
                     reverse);
}

std::vector<Point2D> cubicSpline(const Point2D& p0, const Point2D& p1,
                                 const Point2D& dir0, const Point2D& dir1,
                                 double step, bool reverse) {
  double num_steps = std::ceil((p1 - p0).norm() / step);
  double unified_step = std::min(1 / num_steps, 0.1);

  double sin0 = dir0.y;
  double cos0 = dir0.x;
  double sin1 = dir1.y;
  double cos1 = dir1.x;

  Eigen::Matrix4d A;
  Eigen::Vector4d b, parameters;

  A << 1, 0, 0, 0, 0, 1, 1, 1, sin0, -cos0, 0, 0, sin1, -cos1, -2 * cos1,
      -3 * cos1;
  b << p1.x - p0.x, p1.y - p0.y, 0, 0;

  parameters = A.colPivHouseholderQr().solve(b);

  std::vector<double> coeff_x{p0.x, parameters[0]};
  std::vector<double> coeff_y{p0.y, parameters[1], parameters[2],
                              parameters[3]};
  std::vector<Point2D> res;
  DOUBLE_LOOP(p, 0, 1, unified_step, 1e-6) {
    res.emplace_back(util::polynomial(coeff_x, p),
                     util::polynomial(coeff_y, p));
  }

  if (reverse) {
    std::reverse(res.begin(), res.end());
  }
  return res;
}
DoublePairs<Point2D> pointAtS(const std::vector<Point2D>& line) {
  DoublePairs<Point2D> points_at_s;
  double res = 0;
  points_at_s.push_back(res, line[0]);
  for (size_t i = 0; i < line.size() - 1; i++) {
    res += (line[i + 1] - line[i]).norm();
    points_at_s.push_back(res, line[i + 1]);
  }
  return points_at_s;
}
std::vector<Point2D> interceptPointByS(const std::vector<Point2D>& line,
                                       double begin_s, double end_s) {
  std::vector<Point2D> points;
  double res = 0;
  for (size_t i = 0; i < line.size() - 1; i++) {
    double ds = (line[i + 1] - line[i]).norm();
    res += ds;
    if (res > end_s) {
      points.push_back(
          Point2D::interpolate(line[i], line[i + 1], res - ds, res, end_s));
      break;
    }
    if (res > begin_s && doubleLessEqual(res - ds, begin_s)) {
      points.push_back(
          Point2D::interpolate(line[i], line[i + 1], res - ds, res, begin_s));
    }
    if (res > begin_s) {
      points.push_back(line[i + 1]);
    }
  }
  return points;
}

double splineLength(const std::vector<Point2D>& line) {
  double res = 0;
  for (size_t i = 0; i < line.size() - 1; i++) {
    res += (line[i + 1] - line[i]).norm();
  }
  return res;
}

std::vector<Point2D> arcSpline(const Point2D& p0, double arg, double delta,
                               double radius, double step, bool reverse) {
  return arcSpline(p0, util::Point2D::unit(arg), delta, radius, step, reverse);
}

std::vector<Point2D> arcSpline(const Point2D& p0, const util::Point2D& dir,
                               double delta, double radius, double step,
                               bool reverse) {
  std::vector<Point2D> res;

  auto rel_j = dir;
  auto rel_i = util::Point2D(dir.y, -dir.x) * util::sgn(delta);

  auto center = p0 - rel_i * radius;

  DOUBLE_LOOP(angle, 0, std::abs(delta), step / radius, 1e-6) {
    auto rel = util::Point2D::unit(angle) * radius;
    auto p = center + rel_i * rel.x + rel_j * rel.y;
    res.push_back(p);
  }

  if (reverse) {
    std::reverse(res.begin(), res.end());
  }

  return res;
}

/***
 *https://zhuanlan.zhihu.com/p/530701678?utm_campaign=shareopn&utm_content=group3_article&utm_medium=social&utm_oi=36122520453120&utm_psn=1564304781128175617&utm_source=wechat_session&utm_id=0
 *https://en.wikipedia.org/wiki/B%C3%A9zier_curve
 */
std::vector<Point2D> bezierSpline(const Point2D& p0, const Point2D& p1,
                                  double arg0, double arg1, double step,
                                  bool reverse, const double kappa0,
                                  const double kappa1) {
  return bezierSpline(p0, p1, Point2D::unit(arg0), Point2D::unit(arg1), step,
                      reverse, kappa0, kappa1);
}

std::vector<Point2D> bezierSpline(const Point2D& p0, const Point2D& p1,
                                  const Point2D& dir0, const Point2D& dir1,
                                  double step, bool reverse,
                                  const double kappa0, const double kappa1) {
  // 1.build coordinate base on start point,(0,0,0)
  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  double sin0 = dir0.y;
  double cos0 = dir0.x;
  double sin1 = dir1.y;
  double cos1 = dir1.x;
  double e_x = dx * cos0 + dy * sin0;
  double e_y = dx * -sin0 + dy * cos0;
  double e_th = NormalizeAngle(dir1.heading() - dir0.heading());
  // 2.build equation based on start and end point limit with
  // (x,y,theta,kappa)
  double len1 = 0.0, len2 = 0.0;
  double a = 27.0 * kappa0 * kappa0 * kappa1;
  double b = 0.0;
  double c = -36.0 * kappa0 * kappa1 * e_y;
  double sin_eth = std::sin(e_th);
  double cos_eth = std::cos(e_th);
  double d = 8.0 * pow(sin_eth, 3.0);
  double e = 12.0 * kappa1 * e_y * e_y -
             8.0 * sin_eth * sin_eth * (e_x * sin_eth - e_y * cos_eth);
  // 3. solve the equation
  if (std::abs(a) < 1e-5) {
    if (std::abs(sin_eth) < 1e-5 && cos_eth > 0) {
      len1 = std::max(e_x, e_y) / 3;
      len2 = len1;
    } else if (std::abs(sin_eth) < 1e-5 && cos_eth < 0) {  // uturn
      len1 = std::sqrt(e_x * e_x + e_y * e_y) * 2;
      len2 = len1;
    } else {  // normal
      len1 = -e / d / 3 * 2;
      len2 = 0.5 * (2.0 * e_y - 3.0 * len1 * len1 * kappa0) / sin_eth / 3 * 2;
    }
  } else {
    // quartic equation solver
    // ax^4+bx^3+cx^2+dx+e=0;
    Eigen::Matrix4d A;
    double a_recip = -1.0 / a;
    A << b * a_recip, c * a_recip, d * a_recip, e * a_recip, 1, 0, 0, 0, 0, 1,
        0, 0, 0, 0, 1, 0;

    Eigen::EigenSolver<Eigen::Matrix4d> solver(A);
    auto solutions = solver.eigenvalues();

    for (auto i = 0; i < solutions.size(); i++) {
      if (std::abs(solutions[i].imag()) > 1e-5 || solutions[i].real() < 1e-5) {
        continue;
      }
      len1 = solutions[i].real();
      if (std::abs(sin_eth) < 1e-5) {
        len2 = len1;
        break;
      }
      len2 = 0.5 * (2.0 * e_y - 3.0 * len1 * len1 * kappa0) / sin_eth;
      if (len2 > 1e-5) {
        break;
      }
    }
  }
  if (len1 < 0 || len2 < 0) {
    len1 = std::sqrt(e_x * e_x + e_y * e_y) / 3;
    len2 = len1;
  }
  // 4. get control points
  std::vector<Point2D> control_points;
  Point2D pos_tmp0, pos_tmp1;
  double x_temp = p0.x + len1 * cos0;
  double y_temp = p0.y + len1 * sin0;
  pos_tmp0.x = x_temp;
  pos_tmp0.y = y_temp;

  x_temp = p1.x - len2 * cos1;
  y_temp = p1.y - len2 * sin1;
  pos_tmp1.x = x_temp;
  pos_tmp1.y = y_temp;

  double len_min = 0.0001;
  if (len1 < len_min) {
    control_points.push_back(p0);
    control_points.push_back(pos_tmp1);
    control_points.push_back(p1);
  } else if (len2 < len_min) {
    control_points.push_back(p0);
    control_points.push_back(pos_tmp0);
    control_points.push_back(p1);
  } else {
    control_points.push_back(p0);
    control_points.push_back(pos_tmp0);
    control_points.push_back(pos_tmp1);
    control_points.push_back(p1);
  }

  double num_steps = std::ceil((p1 - p0).norm() / step);
  double unified_step = std::min(1 / num_steps, 0.1);

  std::vector<Point2D> res;
  DOUBLE_LOOP(p, 0, 1, unified_step, 1e-6) {
    if (control_points.size() == 3) {
      res.emplace_back((1 - p) * (1 - p) * control_points[0].x +
                           2 * p * (1 - p) * control_points[1].x +
                           p * p * control_points[2].x,
                       (1 - p) * (1 - p) * control_points[0].y +
                           2 * p * (1 - p) * control_points[1].y +
                           p * p * control_points[2].y);
    } else {
      res.emplace_back((1 - p) * (1 - p) * (1 - p) * control_points[0].x +
                           3 * p * (1 - p) * (1 - p) * control_points[1].x +
                           3 * p * p * (1 - p) * control_points[2].x +
                           p * p * p * control_points[3].x,
                       (1 - p) * (1 - p) * (1 - p) * control_points[0].y +
                           3 * p * (1 - p) * (1 - p) * control_points[1].y +
                           3 * p * p * (1 - p) * control_points[2].y +
                           p * p * p * control_points[3].y);
    };
  }
  if (reverse) {
    std::reverse(res.begin(), res.end());
  }

  return res;
}

// true 为C型曲线（不含Uturn）， false为其他
bool judgeLineType(const util::Point2D& p0, double arg0,
                   const util::Point2D& p1, double arg1,
                   util::Point2D& intersection) {
  double sin1 = std::sin(arg1);
  double sin0 = std::sin(arg0);
  double cos1 = std::cos(arg1);
  double cos0 = std::cos(arg0);
  if (util::doubleEqual(fmod(arg0 - arg1, M_PI), 0, 1e-5) ||
      util::doubleEqual(fmod(arg0 - arg1, M_PI), M_PI, 1e-5)) {
    return false;
  } else {
    double t0 =
        ((p1.x - p0.x) * sin1 - (p1.y - p0.y) * cos1) / (std::sin(arg1 - arg0));
    double t1 =
        ((p1.x - p0.x) * sin0 - (p1.y - p0.y) * cos0) / (std::sin(arg1 - arg0));
    intersection.x = p0.x + cos0 * t0;
    intersection.y = p0.y + sin0 * t0;
    if (t0 * t1 < 0) {
      return true;
    }
  }
  return false;
}

std::vector<Point2D> bezierSpline5thorder(const Point2D& p0, const Point2D& p1,
                                          double arg0, double arg1, double step,
                                          bool reverse, const double kappa0,
                                          const double kappa1) {
  return bezierSpline5thorder(p0, p1, Point2D::unit(arg0), Point2D::unit(arg1),
                              step, reverse, kappa0, kappa1);
}
std::vector<Point2D> bezierSpline5thorder(const Point2D& p0, const Point2D& p1,
                                          const Point2D& dir0,
                                          const Point2D& dir1, double step,
                                          bool reverse, const double kappa0,
                                          const double kappa1) {
  util::Point2D intersection;
  bool line_type =
      judgeLineType(p0, dir0.heading(), p1, dir1.heading(), intersection);
  // 1.build coordinate base on start point,(0,0,0)
  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  double sin0 = dir0.y;
  double cos0 = dir0.x;
  double e_s = dx * cos0 + dy * sin0;
  double e_l = dx * -sin0 + dy * cos0;

  Point2D p_e{e_s, e_l};
  double length = p_e.norm();
  double len_max_1 = line_type ? (intersection - p0).norm() : length * 1.5;
  double len_max_2 = line_type ? (intersection - p1).norm() : length * 1.5;
  double len_min = length * 0.1;
  double len1 = length;
  double len2 = length;

  double c1 = 0.5;
  double c2 = 0.5;
  int update_cnt = 0;
  PSOInfo particle_global;  // global optimal particle

  size_t particle_num = 9;
  PSOInfo particle_single1[particle_num];
  PSOInfo particle_single2[particle_num];

  double vel_min_1 = -0.05 * (len_max_1 - len_min);
  double vel_max_1 = -vel_min_1;
  double step_len_1 = (len_max_1 - len_min) / (particle_num - 1);

  double vel_min_2 = -0.05 * (len_max_2 - len_min);
  double vel_max_2 = -vel_min_2;
  double step_len_2 = (len_max_2 - len_min) / (particle_num - 1);

  for (size_t i = 0; i < particle_num; i++) {
    particle_single1[i].x = len_min + i * step_len_1;
    particle_single1[i].x_best = particle_single1[i].x;
    particle_single2[i].x = len_min + i * step_len_2;
    particle_single2[i].x_best = particle_single2[i].x;
  }
  std::vector<Point2D> control_points;
  //粒子群优化
  double kappa;
  size_t iter_max = 5;
  for (size_t i = 0; i < iter_max; i++) {
    bool update_flag = false;
    for (size_t j = 0; j < particle_num; j++) {
      for (size_t k = 0; k < particle_num; k++) {
        double len1_t = particle_single1[j].x;
        double len2_t = particle_single2[k].x;
        control_points = GetCtrlPoints(p0, p1, dir0, dir1, len1_t, len2_t);
        kappa = std::abs(GetMaxKappa(control_points, 0.0, 1.0));

        if (kappa < particle_single1[j].val_best_up) {
          particle_single1[j].x_best = particle_single1[j].x;
          particle_single1[j].val_best_up = kappa;
        }
        if (kappa < particle_single2[k].val_best_up) {
          particle_single2[k].x_best = particle_single2[k].x;
          particle_single2[k].val_best_up = kappa;
        }
        if (kappa < particle_global.val_best_up) {
          update_flag = true;  // tag
          len1 = particle_single1[j].x;
          len2 = particle_single2[k].x;
          particle_global.val_best_up = kappa;
        }

        particle_single1[j].vel_x +=
            c1 * (particle_single1[j].x_best - particle_single1[j].x) +
            c2 * (len1 - particle_single1[j].x);

        particle_single1[j].vel_x =
            Clamp(particle_single1[j].vel_x, vel_min_1, vel_max_1);
        particle_single1[j].x += particle_single1[j].vel_x;

        particle_single2[k].vel_x +=
            c1 * (particle_single2[k].x_best - particle_single2[k].x) +
            c2 * (len1 - particle_single2[k].x);

        particle_single2[k].vel_x =
            Clamp(particle_single2[k].vel_x, vel_min_2, vel_max_2);
        particle_single2[k].x += particle_single2[k].vel_x;
      }
    }
    if (update_flag) {
      update_cnt++;
      if (update_cnt > static_cast<int>(iter_max / 2)) {
        break;
      }
    }
  }

  len1 = Clamp(len1, len_min, len_max_1);
  len2 = Clamp(len2, len_min, len_max_2);
  control_points = GetCtrlPoints(p0, p1, dir0, dir1, len1, len2);

  double num_steps = std::ceil((p1 - p0).norm() / step);
  double unified_step = std::min(1 / num_steps, 0.1);

  std::vector<Point2D> res;
  DOUBLE_LOOP(p, 0, 1, unified_step, 1e-6) {
    res.emplace_back(GetPoint(control_points, p));
  };
  if (reverse) {
    std::reverse(res.begin(), res.end());
  }

  return res;
}

double effectiveAngle(double angle) {
  double remainder = std::fmod(angle, M_PI * 2);
  if (remainder < 0) {
    remainder += M_PI * 2;
  }
  if (remainder < M_PI_2) {
    return remainder;
  }
  if (remainder < M_PI) {
    return M_PI - remainder;
  }
  if (remainder < M_PI_2 * 3) {
    return remainder - M_PI;
  }
  return M_PI * 2 - remainder;
}

double point2Line(const std::vector<Point2D>& line, const Point2D& target,
                  bool is_closed) {
  double dist = std::numeric_limits<double>::max();
  if (line.empty()) {
    return dist;
  }
  for (size_t i = 0; i < line.size() - (is_closed ? 0 : 1); i++) {
    auto& start = line[i];
    auto& end = line[(i + 1) % line.size()];
    auto diff = end - start;
    auto len = diff.norm();
    auto unit = diff.unit();
    auto proj = (target - start) * unit;
    double one_dist = std::numeric_limits<double>::max();
    if (proj < 0) {
      one_dist = (start - target).norm();
    } else if (proj > len) {
      one_dist = (end - target).norm();
    } else {
      auto nearest = start + unit * proj;
      one_dist = (nearest - target).norm();
    }
    dist = std::min(one_dist, dist);
  }
  return dist;
}

std::vector<Point2D> resample(const std::vector<Point2D>& line, double step) {
  if (line.size() <= 1 || util::doubleLessEqual(step, 0)) {
    return line;
  }
  std::vector<Point2D> res{line[0]};
  for (size_t i = 1; i < line.size(); i++) {
    auto prev = res.back();
    auto diff = line[i] - prev;
    auto step_vec = diff.unit() * step;
    for (int j = 1; j <= diff.norm() / step; j++) {
      res.push_back(prev + step_vec * j);
    }
  }
  if ((line.back() - res.back()).norm() > step * 0.01) {
    res.push_back(line.back());
  }

  return res;
}

bool isPointInScope(const std::vector<util::Point2D>& scope,
                    const util::Point2D& p) {
  std::set<bool> v_product0;
  std::set<bool> v_product1;
  for (size_t i = 0; i < scope.size(); i++) {
    v_product0.emplace(
        ((scope[(i + 1) % scope.size()] - scope[i]) ^ (p - scope[i])) >= 0);
    v_product1.emplace(((scope[i] - scope[(i + 1) % scope.size()]) ^
                        (p - scope[(i + 1) % scope.size()])) >= 0);
  }

  if (v_product0.count(false) == 0 || v_product1.count(false) == 0) {
    return true;
  }
  return false;
}

std::pair<bool, util::Point2D> intersection2D(
    const std::vector<util::Point2D>& line_segment_1,
    const std::vector<util::Point2D>& line_segment_2) {
  const auto& a = line_segment_1.at(0);
  const auto& b = line_segment_1.at(1);
  const auto& c = line_segment_2.at(0);
  const auto& d = line_segment_2.at(1);

  if (((a - d) ^ (c - d)) * ((b - d) ^ (c - d)) <= 0 &&
      ((a - b) ^ (d - b)) * ((a - b) ^ (c - b)) <= 0) {
    // 求取两条线段交点
    double t = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) /
               ((b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x));
    util::Point2D intersection_point;
    intersection_point.x = a.x + t * (b.x - a.x);
    intersection_point.y = a.y + t * (b.y - a.y);
    return std::make_pair(true, intersection_point);
  }
  return std::make_pair(false, util::Point2D{});
}

bool isBoxIntersect(const std::vector<util::Point2D>& ego_points,
                    const std::vector<util::Point2D>& obs_points,
                    bool is_closed) {
  std::vector<std::vector<util::Point2D>> ego_segments;
  std::vector<std::vector<util::Point2D>> obs_segments;

  for (size_t i = 0; i < obs_points.size() - (is_closed ? 0 : 1); i++) {
    auto po = obs_points[i];
    std::vector<util::Point2D> obs_segment_tmp;
    obs_segment_tmp.push_back(po);
    obs_segment_tmp.push_back(obs_points[(i + 1) % obs_points.size()]);
    obs_segments.push_back(obs_segment_tmp);
  }
  for (size_t i = 0; i < ego_points.size(); i++) {
    auto pe = ego_points[i];
    std::vector<util::Point2D> ego_segment_tmp;
    ego_segment_tmp.push_back(pe);
    ego_segment_tmp.push_back(ego_points[(i + 1) % ego_points.size()]);
    ego_segments.push_back(ego_segment_tmp);
  }
  for (size_t i = 0; i < obs_segments.size(); i++) {
    if (util::isPointInScope(ego_points, obs_points[i])) {
      return true;
    };
    for (size_t j = 0; j < ego_segments.size(); j++) {
      if (util::intersection2D(ego_segments[j], obs_segments[i]).first) {
        return true;
      }
    }
  }
  return false;
}
std::vector<util::Point2D> getRect(const util::Point2D& center, double heading,
                                   double length2front, double length2rear,
                                   double width) {
  auto dir1 = util::Point2D::unit(heading);
  auto dir2 = util::Point2D(-dir1.y, dir1.x);
  return std::vector<util::Point2D>{
      center + dir1 * length2front + dir2 * width * 0.5,
      center - dir1 * length2rear + dir2 * width * 0.5,
      center - dir1 * length2rear - dir2 * width * 0.5,
      center + dir1 * length2front - dir2 * width * 0.5};
}
double getPointToLineDistance(const std::vector<util::Point2D>& line_segment,
                              const util::Point2D& p) {
  double sin_theta =
      ((line_segment.at(1) - line_segment.at(0)) ^ (p - line_segment.at(0))) /
      ((p - line_segment.at(0)).norm() *
       (line_segment.at(1) - line_segment.at(0)).norm());
  return (p - line_segment.at(0)).norm() * std::abs(sin_theta);
}

int getHighestBit(int num) {
  int highest_bit = -1;
  while (num != 0) {
    num >>= 1;
    highest_bit++;
  }
  return highest_bit;
}

}  // namespace util
}  // namespace worldview