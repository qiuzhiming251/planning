#pragma once
#include <cmath>
#include <memory>
#include <string>

#include "common/json_util.h"

namespace worldview {
namespace util {
struct Point2D {
  using Ptr = std::shared_ptr<Point2D>;
  Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
  Point2D(const Json::Value& v)
      : Point2D(v["x"].asDouble(), v["y"].asDouble()) {}

  Point2D& operator+=(const Point2D& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }
  Point2D& operator-=(const Point2D& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }
  Point2D operator+(const Point2D& rhs) const {
    Point2D p(*this);
    return p += rhs;
  }
  Point2D operator-(const Point2D& rhs) const {
    Point2D p(*this);
    return p -= rhs;
  }
  Point2D& operator*=(double m) {
    x *= m;
    y *= m;
    return *this;
  }
  Point2D& operator/=(double m) {
    x /= m;
    y /= m;
    return *this;
  }
  Point2D operator*(double m) const { return Point2D(x * m, y * m); }
  Point2D operator/(double m) const { return Point2D(x / m, y / m); }
  Point2D operator-() const { return *this * -1; }

  double operator*(const Point2D& rhs) const { return x * rhs.x + y * rhs.y; }
  double operator^(const Point2D& rhs) const { return x * rhs.y - y * rhs.x; }
  double norm() const { return std::sqrt(x * x + y * y); }
  Point2D unit() const { return *this / norm(); }
  double heading() const { return std::atan2(y, x); }

  Point2D normLeft() const { return Point2D{-y, x}; }

  static Point2D unit(double rad) {
    return Point2D(std::cos(rad), std::sin(rad));
  }

  static util::Point2D interpolate(const util::Point2D& v1,
                                   const util::Point2D& v2, double proportion) {
    return (v2 - v1) * proportion + v1;
  };
  static util::Point2D interpolate(const util::Point2D& v1,
                                   const util::Point2D& v2, double x1,
                                   double x2, double x) {
    double p = (x - x1) / (x2 - x1);
    return interpolate(v1, v2, p);
  };

  std::string print() const {
    return std::to_string(x) + ", " + std::to_string(y);
  }

  Json::Value toJson() const {
    Json::Value res;
    res["x"] = x;
    res["y"] = y;
    return res;
  }

  double x{0}, y{0};
};

struct Pose2D {
  using Ptr = std::shared_ptr<Pose2D>;
  Pose2D(double x_ = 0, double y_ = 0, double theta_ = 0)
      : point(x_, y_), theta(theta_) {}
  Pose2D(const Json::Value& v)
      : Pose2D(v["x"].asDouble(), v["y"].asDouble(), v["theta"].asDouble()) {}
  Point2D point;
  double theta;
};
Point2D transformVectorToOdometryFrame(const Point2D& vcs_vector,
                                       const Pose2D& car_pose);
Point2D transformPointToOdometryFrame(const Point2D& vcs_point,
                                      const Pose2D& car_pose);
Pose2D transformPoseToOdometryFrame(const Pose2D& vcs_pose,
                                    const Pose2D& car_pose);
}  // namespace util
}  // namespace worldview