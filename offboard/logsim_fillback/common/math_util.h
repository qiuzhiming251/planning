#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "common/bezier.h"
#include "common/point_2d.h"
#include "common/type_def.h"

#define DOUBLE_LOOP(VAR, BEGIN, END, STEP, DELTA)                         \
  for (double VAR = BEGIN;                                                \
       (STEP > 0 ? VAR < END : VAR > END) || std::abs(VAR - END) < DELTA; \
       VAR += (STEP > 0 ? std::max(2 * DELTA, std::min(STEP, END - VAR))  \
                        : std::min(-2 * DELTA, std::max(STEP, END - VAR))))
namespace worldview {
namespace util {
template <typename T>
T lerp(const T& x0, const double t0, const T& x1, const double t1,
       const double t, bool clamp = false) {
  if (std::abs(t1 - t0) <= 1e-10) {
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

double sgn(double v);

bool doubleEqual(double x, double y, double delta = 1e-6);
bool doubleLessEqual(double x, double y, double delta = 1e-6);
bool doubleGreaterEqual(double x, double y, double delta = 1e-6);
double linearInterp(double x1, double x2, double y1, double y2, double x);
double linearInterp(double y1, double y2, double proportion);
double polynomial(const std::vector<double>& coeffs, double x,
                  bool constAt0 = true);
double polydiff(const std::vector<double>& coeffs, double x,
                bool constAt0 = true);
class Poly3 {
 public:
  Poly3(double a = 0, double b = 0, double c = 0, double d = 0)
      : coeffs_({a, b, c, d}) {}
  void reset(double a, double b, double c, double d) { coeffs_ = {a, b, c, d}; }
  double calc(double x) const { return polynomial(coeffs_, x); }
  double calc_d(double x) const { return polydiff(coeffs_, x); }

 private:
  std::vector<double> coeffs_;
};

struct Quaternion;

struct Euler {
  Euler(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
  Euler(double heading = 0) : Euler(0, 0, heading) {}
  Euler(const Quaternion& qte);
  Quaternion toQuaternion() const;

  std::string print() const {
    std::stringstream ssm;
    ssm << roll << ", " << pitch << ", " << yaw;
    return ssm.str();
  }

  double roll, pitch, yaw;
};

// ref: https://github.com/infusion/Quaternion.js/blob/master/quaternion.js
struct Quaternion {
  Quaternion(double x_ = 0, double y_ = 0, double z_ = 0, double w_ = 1)
      : x(x_), y(y_), z(z_), w(w_) {}
  Quaternion(const Json::Value& v)
      : Quaternion(v["x"].asDouble(), v["y"].asDouble(), v["z"].asDouble(),
                   v["w"].asDouble()) {}
  Quaternion(const Euler& euler);
  Euler toEuler() const;

  std::string print() const {
    std::stringstream ssm;
    ssm << x << ", " << y << ", " << z << ", w=" << w;
    return ssm.str();
  }

  std::tuple<double, double, double> rotate(double vx, double vy,
                                            double vz) const {
    auto tx = 2 * (y * vz - z * vy);
    auto ty = 2 * (z * vx - x * vz);
    auto tz = 2 * (x * vy - y * vx);

    return {vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx};
  }

  Point2D rotate(const Point2D& p) const {
    auto [px, py, pz] = rotate(p.x, p.y, 0);
    return Point2D{px, py};
  }

  Quaternion multiply(const Quaternion& q) const {
    return Quaternion(w * q.x + x * q.w + y * q.z - z * q.y,
                      w * q.y + y * q.w + z * q.x - x * q.z,
                      w * q.z + z * q.w + x * q.y - y * q.x,
                      w * q.w - x * q.x - y * q.y - z * q.z);
  }
  Quaternion inverse() const { return Quaternion(-x, -y, -z, w); }

  double x, y, z, w;
};
template <typename T>
T lerp(const T& x0, const T& x1, const double w) {
  const T x = x0 + w * (x1 - x0);
  return x;
}

template <typename T>
class LinearInterpolator {
 public:
  T operator()(const T& v1, const T& v2, double proportion) const {
    return (v2 - v1) * proportion + v1;
  }
};

template <typename T>
class DoublePairs {
 public:
  using pair_type = std::pair<double, T>;
  using interpolator_t =
      std::function<T(const T& v1, const T& v2, double proportion)>;

  DoublePairs() = default;
  DoublePairs(const std::vector<pair_type>& vec) : storage_(vec) {}

  void push_back(double key, const T& val) {
    storage_.push_back(std::make_pair(key, val));
  };

  std::pair<size_t, size_t> getRange(double key) const {
    if (storage_.size() < 2) {
      throw std::runtime_error("storage empty when searching for " +
                               std::to_string(key));
    }

    static auto comp = [](double d, const pair_type& p) -> bool {
      return d < p.first;
    };

    auto it = std::upper_bound(storage_.begin(), storage_.end(), key, comp);
    std::pair<size_t, size_t> res;
    if (it == storage_.end()) {
      res.second = storage_.size() - 1;
    } else if (it == storage_.begin()) {
      res.second = 1;
    } else {
      res.second = it - storage_.begin();
    }

    res.first = res.second - 1;

    return res;
  }

  T get(double key, const interpolator_t& interp = interpolator_t()) const {
    auto range = getRange(key);
    double proportion = (key - getKey(range.first)) /
                        (getKey(range.second) - getKey(range.first));
    if (!interp) {
      return getValue(range.first);
    }
    return interp(getValue(range.first), getValue(range.second), proportion);
  }

  const pair_type& front() const { return storage_.front(); }
  const pair_type& back() const { return storage_.back(); }
  const pair_type& operator[](int n) const { return storage_[n]; }
  double getSize() const { return storage_.size(); }
  const T& getValue(size_t idx) const { return storage_[idx].second; }
  double getKey(size_t idx) const { return storage_[idx].first; }
  double minKey() const { return storage_.front().first; }
  double maxKey() const { return storage_.back().first; }

 private:
  std::vector<pair_type> storage_;
};
double NormalizeAngle(const double angle);
std::vector<Point2D> pointsTranslation(const std::vector<Point2D>& points,
                                       double l);
std::vector<Point2D> pointsTranslation(const std::vector<Point2D>& points,
                                       double l_s, double l_e);
std::vector<Point2D> straightSpline(const Point2D& p0, const Point2D& dir,
                                    double length, double step = 5,
                                    bool reverse = false);
std::vector<Point2D> straightSpline(const Point2D& p0, double arg,
                                    double length, double step = 5,
                                    bool reverse = false);
std::vector<Point2D> straightSplineWithEnds(const Point2D& p0,
                                            const Point2D& p1, double step = 5,
                                            bool reverse = false);
std::vector<double> quinticPolynomial(double t0, double s0, double v0,
                                      double a0, double t1, double s1,
                                      double v1, double a1);
std::vector<Point2D> parabolicSpline(const Point2D& p0, const Point2D& p1,
                                     const Point2D& dir0, const Point2D& dir1,
                                     double step = 5, bool reverse = false);
std::vector<Point2D> parabolicSpline(const Point2D& p0, const Point2D& p1,
                                     double arg0, double arg1, double step = 5,
                                     bool reverse = false);
std::vector<Point2D> cubicSpline(const Point2D& p0, const Point2D& p1,
                                 const Point2D& dir0, const Point2D& dir1,
                                 double step = 5, bool reverse = false);
std::vector<Point2D> cubicSpline(const Point2D& p0, const Point2D& p1,
                                 double arg0, double arg1, double step = 5,
                                 bool reverse = false);
DoublePairs<Point2D> pointAtS(const std::vector<Point2D>& line);
std::vector<Point2D> interceptPointByS(const std::vector<Point2D>& line,
                                       double begin_s, double end_s);
std::vector<Point2D> arcSpline(const Point2D& p0, double arg, double delta,
                               double radius, double step = 5,
                               bool reverse = false);
std::vector<Point2D> arcSpline(const Point2D& p0, const Point2D& dir,
                               double delta, double radius, double step = 5,
                               bool reverse = false);
std::vector<Point2D> bezierSpline(const Point2D& p0, const Point2D& p1,
                                  const Point2D& dir0, const Point2D& dir1,
                                  double step = 5, bool reverse = false,
                                  const double kappa0 = 0,
                                  const double kappa1 = 0);
std::vector<Point2D> bezierSpline(const Point2D& p0, const Point2D& p1,
                                  double arg0, double arg1, double step = 5,
                                  bool reverse = false, const double kappa0 = 0,
                                  const double kappa1 = 0);
bool judgeLineType(util::Point2D& p0, double arg0, util::Point2D& p1,
                   double arg1, util::Point2D& intersection);
std::vector<Point2D> bezierSpline5thorder(const Point2D& p0, const Point2D& p1,
                                          const Point2D& dir0,
                                          const Point2D& dir1, double step = 5,
                                          bool reverse = false,
                                          const double kappa0 = 0,
                                          const double kappa1 = 0);
std::vector<Point2D> bezierSpline5thorder(const Point2D& p0, const Point2D& p1,
                                          double arg0, double arg1,
                                          double step = 5, bool reverse = false,
                                          const double kappa0 = 0,
                                          const double kappa1 = 0);
double splineLength(const std::vector<Point2D>& line);
double effectiveAngle(double angle);
double point2Line(const std::vector<Point2D>& line, const Point2D& p,
                  bool is_closed = false);
std::vector<Point2D> resample(const std::vector<Point2D>& line, double step);
std::vector<util::Point2D> getRect(const util::Point2D& center, double heading,
                                   double length2front, double length2rear,
                                   double width);
bool isPointInScope(const std::vector<util::Point2D>& scope,
                    const util::Point2D& p);
std::pair<bool, util::Point2D> intersection2D(
    const std::vector<util::Point2D>& line_segment_1,
    const std::vector<util::Point2D>& line_segment_2);
bool isBoxIntersect(const std::vector<util::Point2D>& ego_points,
                    const std::vector<util::Point2D>& obs_points,
                    bool is_closed = false);
double getPointToLineDistance(const std::vector<util::Point2D>& line_segment,
                              const util::Point2D& p);
int getHighestBit(int num);

}  // namespace util
}  // namespace worldview