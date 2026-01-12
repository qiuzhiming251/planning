

#include <cmath>

#include "plan_common/log.h"
#include "plan_common/math/spline/spline_2d_smoother.h"
#include "plan_common/ref_line/ts_reference_line.h"
namespace ad_byd {
namespace planning {

void TsReferenceLine::Reset() {
  center_pts_.clear();
  path_.Reset();
}

PlanResult TsReferenceLine::UpdateReferenceLine(
    const std::vector<Point2d> &path_point) {
  center_pts_.clear();
  const double control_interval = 0.5;
  for (auto point : path_point) {
    if (!center_pts_.empty()) {
      double dx = point.x() - center_pts_.back().x();
      double dy = point.y() - center_pts_.back().y();
      double d = std::sqrt(dx * dx + dy * dy);
      if (d > 0.5) {
        center_pts_.push_back(point);
      }
    } else {
      center_pts_.push_back(point);
    }
  }

  if (center_pts_.size() < 5) {
    return PLAN_FAIL;
  }

  std::vector<double> s_knots, x_vec, y_vec;
  double s_knot = 0.0;

  x_vec.push_back(center_pts_[0].x());
  y_vec.push_back(center_pts_[0].y());
  s_knots.push_back(s_knot);
  for (int index = 1; index < center_pts_.size(); index++) {
    auto pp = center_pts_[index];
    double dx = pp.x() - x_vec.back();
    double dy = pp.y() - y_vec.back();
    double ds = std::sqrt(dx * dx + dy * dy);
    if (ds < control_interval) {
      continue;
    }
    s_knot += ds;
    s_knots.emplace_back(s_knot);
    x_vec.emplace_back(pp.x());
    y_vec.emplace_back(pp.y());
  }
  std::vector<math::Vec2d> result;
  Spline2dSmoother smoother(s_knots, x_vec, y_vec);
  if (!smoother.Smooth(0.5, &result)) {
    return PLAN_FAIL;
  }
  path_.Reset();
  path_.set_points(result);
  return PLAN_OK;
}

bool TsReferenceLine::SLToXY(const FrenetPoint &frenet_point,
                             TrajectoryPoint *trajectory_point) const {
  if (!trajectory_point) {
    return false;
  }
  auto ref_point = GetReferencePoint(frenet_point.s);
  std::array<double, 3> s_conditions = {ref_point.s, frenet_point.ds,
                                        frenet_point.dds};
  std::array<double, 3> d_conditions = {frenet_point.l, frenet_point.dl,
                                        frenet_point.ddl};
  double x, y;
  math::CartesianFrenetConverter::frenet_to_cartesian(
      ref_point.s, ref_point.x(), ref_point.y(), ref_point.theta,
      ref_point.kappa, ref_point.dkappa, s_conditions, d_conditions, &x, &y,
      &trajectory_point->theta, &trajectory_point->kappa, &trajectory_point->v,
      &trajectory_point->a);
  trajectory_point->set_x(x);
  trajectory_point->set_y(y);
  return true;
}

}  // namespace planning
}  // namespace ad_byd
