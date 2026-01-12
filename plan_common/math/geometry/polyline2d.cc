

#include "plan_common/math/geometry/polyline2d.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <memory>
#include <utility>

#include "plan_common/math/util.h"

namespace st {

// Polyline2d
Polyline2d::Polyline2d(std::vector<Vec2d> points)
    : points_(std::move(points)), aabb_(points_.front(), points_.back()) {
  CHECK(!points_.empty());
  CHECK_GT(points_.size(), 1);

  point_s_.reserve(points_.size() + 1);
  point_s_.push_back(0);
  for (int i = 0; i + 1 < points_.size(); ++i) {
    point_s_.push_back(points_[i].DistanceTo(points_[i + 1]) + point_s_.back());
  }
}

Polyline2d::Polyline2d(std::vector<Vec2d> points, std::vector<double> point_s)
    : points_(std::move(points)),
      point_s_(std::move(point_s)),
      aabb_(points_.front(), points_.back()) {
  CHECK(!points_.empty());
  CHECK_GT(points_.size(), 1);
  CHECK_EQ(point_s_.size(), points_.size());
  CHECK_EQ(point_s_[0], 0.0);
}

Vec2d Polyline2d::Sample(double s) const {
  const int i = GetSegmentIndexFromS(s);
  const double alpha = (s - point_s_[i]) / (point_s_[i + 1] - point_s_[i]);
  CHECK_GE(alpha, 0.0);
  CHECK_LE(alpha, 1.0);

  return Lerp(points_[i], points_[i + 1], alpha);
}

std::vector<Vec2d> Polyline2d::Sample(const std::vector<double>& s) const {
  // Sanity checks.
  CHECK(!s.empty());
  for (int i = 0; i < s.size(); ++i) {
    CHECK_GE(s[i], 0.0);
    CHECK_LE(s[i], point_s_.back());
    if (i > 0) CHECK_GT(s[i], s[i - 1]);
  }

  std::vector<Vec2d> res;
  res.reserve(s.size());
  int index = 0;
  constexpr double kEpsilon = 1E-6;
  for (int i = 0; i < s.size(); ++i) {
    if (i == 0) {
      index = GetSegmentIndexFromS(s[i]);
    } else {
      while (s[i] >= point_s_[index + 1] && index + 1 < point_s_.size()) {
        index++;
      }
    }
    if (index + 1 >= point_s_.size()) {
      break;
    }
    const double alpha =
        std::fabs(point_s_[index + 1] - point_s_[index] < kEpsilon)
            ? 0.0
            : (s[i] - point_s_[index]) /
                  (point_s_[index + 1] - point_s_[index]);
    res.push_back(Lerp(points_[index], points_[index + 1], alpha));
  }

  return res;
}

Vec2d Polyline2d::SampleTangent(double s) const {
  const int i = GetSegmentIndexFromS(s);

  return (points_[i + 1] - points_[i]).normalized();
}
std::vector<Vec2d> Polyline2d::SampleTangent(
    const std::vector<double>& s) const {
  // Sanity checks.
  CHECK(!s.empty());
  for (int i = 0; i < s.size(); ++i) {
    CHECK_GE(s[i], 0.0);
    CHECK_LE(s[i], point_s_.back());
    if (i > 0) CHECK_GT(s[i], s[i - 1]);
  }

  std::vector<Vec2d> res;
  res.reserve(s.size());
  int index = 0;
  for (int i = 0; i < s.size(); ++i) {
    if (i == 0) {
      index = GetSegmentIndexFromS(s[i]);
    } else {
      while (s[i] >= point_s_[index + 1] && index + 1 < point_s_.size()) {
        index++;
      }
    }
    res.push_back((points_[i + 1] - points_[i]).normalized());
  }

  return res;
}

int Polyline2d::GetSegmentIndexFromS(double s) const {
  CHECK_GE(s, 0.0);
  CHECK_LE(s, point_s_.back());

  int index = std::upper_bound(point_s_.begin(), point_s_.end(), s) -
              point_s_.begin() - 1;
  if (index == point_s_.size() - 1) {
    CHECK_EQ(s, point_s_.back());
    index -= 1;
  }

  return index;
}

// SampledPolyline2d
SampledPolyline2d::SampledPolyline2d(std::vector<Vec2d> points, double interval)
    : Polyline2d(std::move(points)), interval_(interval) {
  CHECK_GT(interval_, 0);

  BuildSamples();
}

SampledPolyline2d::SampledPolyline2d(std::vector<Vec2d> points,
                                     std::vector<double> point_s,
                                     double interval)
    : Polyline2d(std::move(points), std::move(point_s)), interval_(interval) {
  CHECK_GT(interval_, 0);

  BuildSamples();
}

int SampledPolyline2d::GetSampleSegmentIndexFromS(double s) const {
  CHECK_GE(s, 0.0);
  CHECK_LE(s, sample_s_.back());

  int index = std::upper_bound(sample_s_.begin(), sample_s_.end(), s) -
              sample_s_.begin() - 1;
  if (index == sample_s_.size() - 1) {
    CHECK_EQ(s, sample_s_.back());
    index -= 1;
  }

  return index;
}

void SampledPolyline2d::BuildSamples() {
  const int estimated_num_samples =
      static_cast<int>(point_s_.back() / interval_);
  samples_.reserve(estimated_num_samples);
  sample_s_.reserve(estimated_num_samples);
  tangents_.reserve(estimated_num_samples);

  int index = 0;
  double s = 0.0;
  while (true) {
    while (index + 1 < point_s_.size() && s > point_s_[index + 1]) {
      ++index;
    }
    if (index + 1 >= point_s_.size()) break;
    const double alpha =
        (s - point_s_[index]) / (point_s_[index + 1] - point_s_[index]);
    samples_.push_back(Lerp(points_[index], points_[index + 1], alpha));
    sample_s_.push_back(s);
    tangents_.push_back(
        Vec2d(points_[index + 1] - points_[index]).normalized());
    constexpr double kEpsilon = 0.01;
    if (s < point_s_.back() - kEpsilon && s + interval_ > point_s_.back()) {
      s = point_s_.back();
    } else {
      s += interval_;
    }
  }
  samples_.back() = points_.back();
  sample_s_.back() = point_s_.back();
}

std::vector<Vec2d> ResampleWithTolerance(const std::vector<Vec2d>& points,
                                         double tolerance) {
  CHECK_GT(points.size(), 1);
  CHECK_GT(tolerance, 0.0);

  std::vector<double> s_knots(points.size(), 0.0);
  for (int i = 1; i < points.size(); ++i) {
    s_knots[i] = s_knots[i - 1] + points[i].DistanceTo(points[i - 1]);
  }

  std::vector<Vec2d> resampled;
  resampled.reserve(points.size() + FloorToInt(s_knots.back() / tolerance));
  for (int i = 0; i + 1 < points.size(); ++i) {
    resampled.push_back(points[i]);
    const double rel_s = s_knots[i + 1] - s_knots[i];
    if (rel_s <= tolerance) continue;

    const int n_segs = CeilToInt(rel_s / tolerance);
    const double n_seg_inv = 1.0 / n_segs;
    for (int j = 1; j < n_segs; ++j) {
      resampled.push_back(Lerp(points[i], points[i + 1], j * n_seg_inv));
    }
  }
  resampled.push_back(points.back());

  return resampled;
}

}  // namespace st
