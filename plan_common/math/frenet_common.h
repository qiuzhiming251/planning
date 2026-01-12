

#ifndef ONBOARD_MATH_FRENET_COMMON_H_
#define ONBOARD_MATH_FRENET_COMMON_H_

namespace st {

struct FrenetCoordinate {
  double s;
  double l;
};

struct SecondOrderFrenetCoordinate {
  double s;
  double l;
  double dl;
  double d2l;
};

struct SecondOrderCartesianCoordinate {
  double x;
  double y;
  double theta;
  double kappa;
};

struct FrenetBox {
  double s_max;
  double s_min;
  double l_max;
  double l_min;

  inline double center_s() const { return 0.5 * (s_min + s_max); }
  inline double center_l() const { return 0.5 * (l_min + l_max); }
  inline FrenetCoordinate center() const { return {center_s(), center_l()}; }
  inline double length() const { return s_max - s_min; }
  inline double width() const { return l_max - l_min; }
};

struct FrenetPolygon {
  double s_max;
  double s_min;
  double l_max;
  double l_min;
  FrenetCoordinate center;

  inline double center_s() const { return center.s; }
  inline double center_l() const { return center.l; }
};

}  // namespace st

#endif  // ONBOARD_MATH_FRENET_COMMON_H_
