#include "acc/acc_util.h"

#include <cmath>
#include <vector>

#include "plan_common/math/cubic_spline.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/acc.pb.h"
#include "planner/planner_manager/planner_flags.h"

namespace st::planning {
// Circle if fixed kappa
UniCycleModelState GenUniCycleModelState(const UniCycleModelState& state,
                                         double s, double kappa_decay,
                                         double init_heading,
                                         double abs_max_heading) {
  auto cos = std::cos(state.heading);
  auto sin = std::sin(state.heading);
  double new_heading = state.heading + s * state.kappa;
  if (std::fabs(NormalizeAngle(new_heading - init_heading)) >=
      abs_max_heading) {
    new_heading = state.heading;
    kappa_decay = 1.0;
  }
  return {
      .x = state.x + s * cos,
      .y = state.y + s * sin,
      .heading = new_heading,
      .kappa = state.kappa * kappa_decay,
  };
}

void FillFitPathPoint(const CubicSpline& x_s, const CubicSpline& y_s,
                      double cur_s, double abs_max_curvature, PathPoint* pt) {
  pt->set_x(x_s.Evaluate(cur_s));
  pt->set_y(y_s.Evaluate(cur_s));
  const double dx = x_s.EvaluateDerivative<1>(cur_s);
  const double dy = y_s.EvaluateDerivative<1>(cur_s);
  const double ddx = x_s.EvaluateDerivative<2>(cur_s);
  const double ddy = y_s.EvaluateDerivative<2>(cur_s);
  pt->set_s(cur_s);
  pt->set_theta(fast_math::Atan2(dy, dx));
  double curvature = ComputeSignedCurvature(dx, dy, ddx, ddy);
  curvature = std::clamp(curvature, -abs_max_curvature, abs_max_curvature);
  pt->set_kappa(curvature);
}

inline double ComputeSignedCurvature(double dx, double dy, double ddx,
                                     double ddy) {
  const auto dr = Sqr(dx) + Sqr(dy);
  constexpr double kMinDenominator = 0.00046;
  return dr < kMinDenominator ? 0.0 : (dx * ddy - dy * ddx) / std::pow(dr, 1.5);
}

DiscretizedPath ExtendedCubicSplinePath(const std::vector<Vec2d>& ref_center_xy,
                                        const std::vector<double>& ref_s_vec,
                                        const Vec2d& heading, double step_s,
                                        double resample_step_s,
                                        double extended_length,
                                        double abs_max_curvature) {
  constexpr int kMinCubicSplinePointNum = 5;
  constexpr double kEpsilonS = 1e-3;
  const int spline_n = std::max(kMinCubicSplinePointNum,
                                static_cast<int>(extended_length / step_s) + 1);
  std::vector<double> vec_x;
  std::vector<double> vec_y;
  std::vector<double> vec_s;
  vec_x.reserve(spline_n);
  vec_y.reserve(spline_n);
  vec_s.reserve(spline_n);
  const int n = ref_center_xy.size();
  for (int i = 0; i < n; ++i) {
    if (!vec_s.empty() && std::fabs(ref_s_vec[i] - vec_s.back()) < kEpsilonS) {
      continue;
    }
    vec_x.push_back(ref_center_xy[i].x());
    vec_y.push_back(ref_center_xy[i].y());
    vec_s.push_back(ref_s_vec[i]);
  }
  double length = step_s;
  const auto& xy_init = ref_center_xy.back();
  const auto& s_init = ref_s_vec.back();
  for (int i = 0; i < spline_n - n; ++i) {
    const auto xy = xy_init + length * heading;
    vec_x.push_back(xy.x());
    vec_y.push_back(xy.y());
    vec_s.push_back(s_init + length);
    length += step_s;
  }
  const CubicSpline x_s(vec_s, vec_x);
  const CubicSpline y_s(vec_s, vec_y);

  DiscretizedPath path;
  const int speed_path_n =
      static_cast<int>(extended_length / resample_step_s) + 1;
  path.reserve(speed_path_n);
  for (int i = 0; i < speed_path_n && i * resample_step_s < vec_s.back(); ++i) {
    const double cur_s = i * resample_step_s;
    path.emplace_back();
    FillFitPathPoint(x_s, y_s, cur_s, abs_max_curvature, &path.back());
  }
  return path;
}

}  // namespace st::planning
