#include "plan_common/util/gradient_points_smoother.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "absl/status/status.h"
#include "plan_common/log.h"

#include "plan_common/math/geometry/polyline2d.h"

namespace st::planning {
namespace {
constexpr int kMinPointsNum = 5;
}

GradientSmootherResult SmoothPointsByGradientDescent(absl::Span<double> x,
                                                     absl::Span<double> y,
                                                     int max_iter, double eps,
                                                     double alpha) {
  DCHECK_EQ(x.size(), y.size());
  std::vector<Vec2d> points;
  points.reserve(x.size());
  for (int i = 0; i < x.size(); ++i) {
    points.push_back({x[i], y[i]});
  }
  auto result = SmoothPointsByGradientDescent(absl::MakeSpan(points), max_iter,
                                              eps, alpha);
  if (!result.success) return result;
  for (int i = 0; i < x.size(); ++i) {
    x[i] = points[i].x();
    y[i] = points[i].y();
  }
  return result;
}

GradientSmootherResult SmoothPointsByGradientDescent(absl::Span<Vec2d> points,
                                                     int max_iter, double eps,
                                                     double alpha) {
  GradientSmootherResult result;
  if (points.size() < kMinPointsNum) {
    result.success = false;
    result.state = GradientSmootherState::INVALID_INPUT;
    return result;
  }

  int iter = 0;
  while (iter <= max_iter) {
    double max_abs_g = 0.0;
    for (int i = 2; i < static_cast<int>(points.size()) - 2; ++i) {
      const Vec2d& p1 = points[i - 2];
      const Vec2d& p2 = points[i - 1];
      const Vec2d& p3 = points[i];
      const Vec2d& p4 = points[i + 1];
      const Vec2d& p5 = points[i + 2];
      const Vec2d gradient = -(p1 - 4.0 * p2 + 6.0 * p3 - 4.0 * p4 + p5);
      points[i] += alpha * gradient;
      max_abs_g =
          std::max({std::abs(gradient.x()), std::abs(gradient.y()), max_abs_g});
    }
    result.max_abs_gradient = max_abs_g;
    if (max_abs_g < eps) {
      result.iter = iter;
      result.success = true;
      result.state = GradientSmootherState::EPS_THRESHOLD;
      return result;
    }
    ++iter;
  }
  result.iter = iter;
  result.success = true;
  result.state = GradientSmootherState::MAX_ITER_REACHED;

  return result;
}

absl::StatusOr<std::vector<Vec2d>> ResampleAndSmoothPoints(
    const std::vector<Vec2d>& points, double tolerance, int max_iter,
    double eps, double alpha) {
  if (points.size() < 2) return absl::CancelledError("Not enough points.");
  if (points.size() == 2) return points;

  auto resampled = ResampleWithTolerance(points, tolerance);
  std::vector<Vec2d> smoother_points;
  smoother_points.reserve(resampled.size() + 2);
  smoother_points.push_back(resampled.front());
  smoother_points.insert(smoother_points.end(), resampled.begin(),
                         resampled.end());
  smoother_points.push_back(resampled.back());
  const auto smooth_result = SmoothPointsByGradientDescent(
      absl::MakeSpan(smoother_points), max_iter, eps, alpha);
  if (!smooth_result.success) return absl::InternalError("Smoothing failed.");
  std::copy(smoother_points.begin() + 1, smoother_points.end() - 1,
            resampled.begin());
  const Polyline2d polyline(std::move(resampled));
  std::vector<double> s_samples(points.size());
  const double ds = polyline.length() / (points.size() - 1);
  for (int i = 0; i + 1 < points.size(); ++i) {
    s_samples[i] = ds * i;
  }
  s_samples.back() = polyline.length();
  return polyline.Sample(s_samples);
}

}  // namespace st::planning
