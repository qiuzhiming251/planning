#pragma once

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "plan_common/math/vec.h"

namespace st::planning {

enum class GradientSmootherState {
  NONE,
  INVALID_INPUT,
  MAX_ITER_REACHED,
  EPS_THRESHOLD
};

struct GradientSmootherResult {
  int iter = 0;
  double max_abs_gradient = 0.0;
  bool success = false;
  GradientSmootherState state = GradientSmootherState::NONE;
  std::string DebugString() const {
    return absl::StrCat(
        "***** Gradient smoother result *****", "\niter: ", iter,
        "\nmax_abs_gradient: ", max_abs_gradient, "\nsuccess: ", success,
        "\nstate: ", static_cast<int>(state));
  }
};

GradientSmootherResult SmoothPointsByGradientDescent(absl::Span<double> x,
                                                     absl::Span<double> y,
                                                     int max_iter, double eps,
                                                     double alpha);

GradientSmootherResult SmoothPointsByGradientDescent(absl::Span<Vec2d> points,
                                                     int max_iter = 100,
                                                     double eps = 0.05,
                                                     double alpha = 0.1);

absl::StatusOr<std::vector<Vec2d>> ResampleAndSmoothPoints(
    const std::vector<Vec2d>& points, double tolerance, int max_iter = 75,
    double eps = 0.01, double alpha = 0.15);

}  // namespace st::planning
