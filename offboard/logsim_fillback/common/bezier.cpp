#include "common/bezier.h"

namespace worldview {
namespace util {
std::vector<Point2D> GetCtrlPoints(const Point2D& pos_start,
                                   const Point2D& pos_end, const Point2D& dir0,
                                   const Point2D& dir1, const double& len1,
                                   const double& len2) {
  Point2D pos_tmp0, pos_tmp1, pos_tmp2, pos_tmp3;
  double k = 0.5;  //可调参数
  double x_temp = pos_start.x + k * len1 * std::cos(dir0.heading());
  double y_temp = pos_start.y + k * len1 * std::sin(dir0.heading());
  pos_tmp0.x = x_temp;
  pos_tmp0.y = y_temp;

  x_temp = pos_start.x + len1 * std::cos(dir0.heading());
  y_temp = pos_start.y + len1 * std::sin(dir0.heading());
  pos_tmp1.x = x_temp;
  pos_tmp1.y = y_temp;

  x_temp = pos_end.x - len2 * std::cos(dir1.heading());
  y_temp = pos_end.y - len2 * std::sin(dir1.heading());
  pos_tmp2.x = x_temp;
  pos_tmp2.y = y_temp;

  x_temp = pos_end.x - k * len2 * std::cos(dir1.heading());
  y_temp = pos_end.y - k * len2 * std::sin(dir1.heading());
  pos_tmp3.x = x_temp;
  pos_tmp3.y = y_temp;

  std::vector<Point2D> control_points;

  control_points.push_back(pos_start);
  control_points.push_back(pos_tmp0);
  control_points.push_back(pos_tmp1);
  control_points.push_back(pos_tmp2);
  control_points.push_back(pos_tmp3);
  control_points.push_back(pos_end);
  return control_points;
}
/***********************************************************************************************
 *  @brief: nchoosek
 *
 *  @return:C_n^ k
 ***********************************************************************************************/
double Nchoosek(const int& n, const int& k) {
  double out = 1.0;
  for (int i = 0; i < k; i++) {
    out = out * (n - i) / (k - i);
  }
  return out;
}
/***********************************************************************************************
 *  @brief: bernstein
 *
 *  @return:C_n^ k* (1 - t) ^ (n - k)* t^ k
 ***********************************************************************************************/
double Bernstein(const int& n, const int& k, const double& t) {
  if (k < 0 || n - k < 0) {
    return 0;
  }
  double out = Nchoosek(n, k) * pow(1.0 - t, n - k) * pow(t, k);
  return out;
}
Point2D GetPoint(const std::vector<Point2D>& control_points, const double& t) {
  auto n = control_points.size();

  double x = 0;
  double y = 0;
  for (size_t i = 0; i < n; i++) {
    double bernstein = Bernstein(n - 1, i, t);
    x += control_points[i].x * bernstein;
    y += control_points[i].y * bernstein;
  }
  return Point2D{x, y};
}
Point2D GetDerivative(const std::vector<Point2D>& control_points,
                      const double& t, const int& order) {
  Point2D grad;
  if (order < 1 || control_points.size() < 3) {
    return grad;
  }
  int n = control_points.size() - 1;
  std::vector<std::vector<double>> dx(n + 1, std::vector<double>(order + 1, 0));
  std::vector<std::vector<double>> dy(n + 1, std::vector<double>(order + 1, 0));
  for (int i = 0; i <= n; i++) {
    dx[i][0] = control_points[i].x;
    dy[i][0] = control_points[i].y;
  }
  for (int j = 1; j <= order; j++) {
    for (int i = n - 1; i >= 0; i--) {
      dx[i][j] = dx[i + 1][j - 1] - dx[i][j - 1];
      dy[i][j] = dy[i + 1][j - 1] - dy[i][j - 1];
    }
  }
  double x = 0;
  double y = 0;
  for (int i = 0; i <= n - order; i++) {
    double bernstein = Bernstein(n - order, i, t);
    x += dx[i][order] * bernstein;
    y += dy[i][order] * bernstein;
  }
  double coef = 1.0 * n;
  for (int i = 1; i < order; i++) {
    coef = coef * (n - i);
  }

  grad.x = coef * x;
  grad.y = coef * y;

  return grad;
}

double GetKappa(const std::vector<Point2D>& control_points, const double& t) {
  double t_in = Clamp(t, 0.0, 1.0);
  Point2D grad1ord = GetDerivative(control_points, t_in, 1);
  Point2D grad2ord = GetDerivative(control_points, t_in, 2);

  double tmp1 = grad1ord.x * grad2ord.y - grad2ord.x * grad1ord.y;
  double tmp2 = grad1ord.x * grad1ord.x + grad1ord.y * grad1ord.y;
  double kappa = tmp1 / pow(tmp2, 1.5);

  return kappa;
}

double GetMaxKappa(const std::vector<Point2D>& control_points,
                   const double& t_a, const double& t_b) {
  if (control_points.size() < 3) {
    return DBL_MAX;
  }
  double c1 = 0.75;
  double c2 = 0.75;
  int update_cnt = 0;
  PSOInfo particle_global;  // global optimal particle
  size_t particle_num = 5;
  std::vector<PSOInfo> particle_single(particle_num);

  double vel_min = -0.1;
  double vel_max = -vel_min;

  double t_per = 1.0 / (particle_num - 1);
  for (size_t i = 0; i < particle_num; i++) {
    particle_single[i].x = t_per * i;
    particle_single[i].x_best = particle_single[i].x;
  }

  double kappa;
  size_t iter_max = 10;
  for (size_t i = 0; i < iter_max; i++) {
    bool update_flag = false;
    for (size_t j = 0; j < particle_num; j++) {
      particle_single[j].x = Clamp(particle_single[j].x, t_a, t_b);
      kappa = std::abs(GetKappa(control_points, particle_single[j].x));

      if (kappa > particle_single[j].val_best_down) {
        particle_single[j].x_best = particle_single[j].x;
        particle_single[j].val_best_down = kappa;
      }
      if (kappa > particle_global.val_best_down) {
        update_flag = true;  // tag
        particle_global.x_best = particle_single[j].x;
        particle_global.val_best_down = kappa;
      }
      particle_single[j].vel_x +=
          c1 * (particle_single[j].x_best - particle_single[j].x) +
          c2 * (particle_global.x_best - particle_single[j].x);

      particle_single[j].vel_x =
          Clamp(particle_single[j].vel_x, vel_min, vel_max);
      particle_single[j].x += particle_single[j].vel_x;
    }
    if (update_flag) {
      update_cnt++;
      if (update_cnt > static_cast<int>(iter_max / 2)) {
        break;
      }
    }
  }
  particle_global.x_best = Clamp(particle_global.x_best, t_a, t_b);
  return GetKappa(control_points, particle_global.x_best);
}
}  // namespace util
}  // namespace worldview