#pragma once

#include <float.h>
#include <vector>

#include "common/point_2d.h"

namespace worldview {
namespace util {
struct PSOInfo {
  PSOInfo() {
    x = 0;
    y = 0;
    x_best = 0;
    y_best = 0;
    val_best_up = DBL_MAX;
    val_best_down = -DBL_MAX;
    vel_x = 0;
    vel_y = 0;
  };
  double x;              // particle
  double y;              // particle
  double x_best;         // optimal particle
  double y_best;         // optimal particle
  double val_best_up;    // optimal particle value
  double val_best_down;  // optimal particle value
  double vel_x;          // move speed
  double vel_y;          // move speed
};

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

std::vector<Point2D> GetCtrlPoints(const Point2D& pos_start,
                                   const Point2D& pos_end, const Point2D& dir0,
                                   const Point2D& dir1, const double& len1,
                                   const double& len2);
Point2D GetPoint(const std::vector<Point2D>& control_points, const double& t);
double Nchoosek(const int& n, const int& k);
double Bernstein(const int& n, const int& k, const double& t);
Point2D GetDerivative(const std::vector<Point2D>& control_points,
                      const double& t, const int& order);
double GetKappa(const std::vector<Point2D>& control_points, const double& t);
double GetMaxKappa(const std::vector<Point2D>& control_points,
                   const double& t_a, const double& t_b);
}  // namespace util
}  // namespace worldview