
#ifndef AD_BYD_PLANNING_MATH_SPLINE_1D_SMOOTHER_H_
#define AD_BYD_PLANNING_MATH_SPLINE_1D_SMOOTHER_H_
#include <vector>

#include "plan_common/type_def.h"
#include "plan_common/math/vec2d.h"

namespace ad_byd {
namespace planning {
class Spline1dSmoother {
 public:
  explicit Spline1dSmoother(const std::vector<double> &x_points,
                            const std::vector<double> &y_points)
      : x_points_(x_points), y_points_(y_points) {}
  ~Spline1dSmoother() = default;

  /// @brief Smooth xy points with s_knots, using spline2d
  /// @param s_revolution discrete spline by s_revolution
  /// @param result output the smoothed vec2d points
  /// @return False if invalid input points or generate spline fail
  bool Smooth(const double &s_revolution, std::vector<math::Vec2d> *result);

 private:
  std::vector<double> x_points_;
  std::vector<double> y_points_;
};
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MATH_SPLINE_1D_SMOOTHER_H_
