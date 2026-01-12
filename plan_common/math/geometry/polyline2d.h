

#ifndef ONBOARD_MATH_GEOMETRY_POLYLINE2D_H_
#define ONBOARD_MATH_GEOMETRY_POLYLINE2D_H_

#include <vector>

#include "plan_common/math/geometry/aabox2d.h"
#include "plan_common/math/vec.h"

namespace st {

/**
 * @class Polyline2d
 * @brief A connected line sequence of 2-D line segments.
 */
class Polyline2d {
 public:
  Polyline2d() = delete;
  explicit Polyline2d(std::vector<Vec2d> points);
  Polyline2d(std::vector<Vec2d> points, std::vector<double> point_s);

  double length() const { return point_s_.back(); }

  const std::vector<Vec2d>& points() const { return points_; }
  const std::vector<double>& point_s() const { return point_s_; }

  const AABox2d& aabb() const { return aabb_; }

  Vec2d Sample(double s) const;
  // s must be monotonically increasing
  std::vector<Vec2d> Sample(const std::vector<double>& s) const;

  Vec2d SampleTangent(double s) const;
  // s must be monotonically increasing
  std::vector<Vec2d> SampleTangent(const std::vector<double>& s) const;

  // Returns the index i that s locates in [point_s_[i], point_s_[i + 1])
  // If s == length, returns points_.size() - 1
  int GetSegmentIndexFromS(double s) const;

 protected:
  std::vector<Vec2d> points_;
  std::vector<double> point_s_;  // Same size with points_, with the
                                 // first element being always zero

  AABox2d aabb_;
};

/**
 * @class SampledPolyline2d
 * @brief Uniformly sampled Polyline2d.
 */
class SampledPolyline2d : public Polyline2d {
 public:
  SampledPolyline2d() = delete;
  SampledPolyline2d(std::vector<Vec2d> points, double interval);
  SampledPolyline2d(std::vector<Vec2d> points, std::vector<double> point_s,
                    double interval);

  const std::vector<Vec2d>& samples() const { return samples_; }
  const std::vector<double>& sample_s() const { return sample_s_; }
  const std::vector<Vec2d>& tangents() const { return tangents_; }

  // Returns the index i that s locates in [sample_s_[i], sample_s[i + 1])
  // If s == length, returns samples_.size() - 1
  int GetSampleSegmentIndexFromS(double s) const;

 protected:
  void BuildSamples();

  std::vector<Vec2d> samples_;
  std::vector<double> sample_s_;
  std::vector<Vec2d> tangents_;

  double interval_;
};

std::vector<Vec2d> ResampleWithTolerance(const std::vector<Vec2d>& points,
                                         double tolerance);

}  // namespace st

#endif  // ONBOARD_MATH_GEOMETRY_POLYLINE2D_H_
