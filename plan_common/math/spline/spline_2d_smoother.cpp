
#include <iostream>

#include "plan_common/math/spline/spline_1d.h"
#include "plan_common/math/spline/spline_2d_smoother.h"
namespace ad_byd {
namespace planning {
bool Spline2dSmoother::Smooth(const double &s_revolution,
                              std::vector<math::Vec2d> *result) {
  result->clear();
  // 1. Check if points valid
  if (s_knots_.empty()) {
    std::cout << "no knots to smooth !" << std::endl;
    return false;
  }
  if (s_knots_.size() != x_points_.size() ||
      s_knots_.size() != y_points_.size()) {
    std::cout << "Err size of knots or points!" << std::endl;
    return false;
  }
  if (s_knots_.size() == 1) {
    result->push_back(math::Vec2d(x_points_.back(), y_points_.back()));
    return true;
  }

  if (s_knots_.size() == 2) {
    s_knots_.insert(s_knots_.begin() + 1,
                    (s_knots_.front() + s_knots_.back()) * 0.5);
    x_points_.insert(x_points_.begin() + 1,
                     (x_points_.front() + x_points_.back()) * 0.5);
    y_points_.insert(y_points_.begin() + 1,
                     (y_points_.front() + y_points_.back()) * 0.5);
  }

  // 2. Generate spline x = f(s) and y = f(s)
  tk::spline x_spline;
  try {
    x_spline.set_points(s_knots_, x_points_);
  } catch (const std::exception &e) {
    std::cout << "Generate spline x = f(s) Fail!" << std::endl;
    std::cerr << e.what() << '\n';
    return false;
  } catch (...) {
    std::cout << "Generate spline x = f(s) Fail!" << std::endl;
    std::cerr << "e.what()" << '\n';
    return false;
  }

  // 3. Generate spline y = f(s)
  tk::spline y_spline;
  try {
    y_spline.set_points(s_knots_, y_points_);
  } catch (const std::exception &e) {
    std::cout << "Generate spline y = f(s) Fail!" << std::endl;
    std::cerr << e.what() << '\n';
    return false;
  } catch (...) {
    std::cout << "Generate spline y = f(s) Fail!" << std::endl;
    std::cerr << "e.what()" << '\n';
    return false;
  }

  // 4. Discrete spline by s_revolution
  for (double s = 0.0; s < s_knots_.back(); s += s_revolution) {
    result->push_back(math::Vec2d(x_spline(s), y_spline(s)));
  }
  return true;
}
}  // namespace planning
}  // namespace ad_byd
