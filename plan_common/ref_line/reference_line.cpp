
#include <cmath>

#include "plan_common/log.h"
#include "plan_common/ref_line/reference_line.h"
//#include "plan_common/ref_line/linear_ref_line_optimizer.h"
//#include "plan_common/ref_line/ref_line_problem_builder.h"
#include "plan_common/maps/map.h"
#include "plan_common/math/spline/spline_2d_smoother.h"

namespace ad_byd {
namespace planning {
void ReferenceLine::Reset() {
  lane_sequence_ = nullptr;
  center_pts_.clear();
  path_.Reset();
  drive_boundary_.Clear();
}

PlanResult ReferenceLine::UpdateReferenceLine(
    const TrajectoryPoint &start_point, const LaneSequencePtr &lane_sequence,
    const MapPtr &map) {
  if (!map || !lane_sequence || lane_sequence->lanes().empty()) {
    Reset();
    LOG_ERROR << "<ReferenceLine> lane_sequence = nullptr or empty!";
    return PLAN_FAIL;
  }
  lane_sequence_ = lane_sequence;
  // return ReferenceLineOptimize(start_point, lane_sequence, map);
  return ReferenceLineFitting(start_point, lane_sequence, map);
}

PlanResult ReferenceLine::ReferenceLineFitting(
    const TrajectoryPoint &start_point, const LaneSequencePtr &lane_sequence,
    const MapPtr &map) {
  const double kRefLineBehindLength = 20.0;
  const double kRefLineFrontLength = 110.0;
  const double kControlInterval = 10.0;
  const double kRefLineRevolution = 0.5;

  // 1. lane_sequence SamplePoints
  double s_offset = 0.0;
  lane_sequence->GetProjectionDistance(start_point, &s_offset);
  s_offset = std::max(s_offset - kRefLineBehindLength, Constants::ZERO);
  std::vector<Point2d> sample_points;
  lane_sequence->SamplePoints(s_offset, &sample_points, 3.0);

  // 2. Resampling
  std::vector<double> s_knots, x_vec, y_vec;
  double s_knot = 0.0;
  double s_sum = 0.0;
  for (const auto &point : sample_points) {
    if (s_sum > kRefLineBehindLength + kRefLineFrontLength + Constants::ZERO) {
      break;
    }
    if (!x_vec.empty()) {
      double dx = point.x() - x_vec.back();
      double dy = point.y() - y_vec.back();
      double ds = sqrt(dx * dx + dy * dy);
      s_sum += ds;
      if (ds < kControlInterval) {
        continue;
      }
      s_knot += ds;
    }
    s_knots.emplace_back(s_knot);
    x_vec.emplace_back(point.x());
    y_vec.emplace_back(point.y());
  }
  if (s_sum < kControlInterval) {
    if (s_sum < kRefLineRevolution) {
      LOG_ERROR << "line is shorter than revolution, too short to smooth";
      return PLAN_FAIL;
    }
    LOG_WARN << "center point is shorter than control interval !";
    x_vec.emplace_back(sample_points.back().x());
    y_vec.emplace_back(sample_points.back().y());
  }

  // 3. Smoothing
  std::vector<math::Vec2d> result;
  Spline2dSmoother smoother(s_knots, x_vec, y_vec);
  if (!smoother.Smooth(kRefLineRevolution, &result)) {
    LOG_ERROR << "Spline2dSmoother fail !";
    return PLAN_FAIL;
  }
  path_.Reset();
  path_.set_points(result);
  center_pts_ = std::move(sample_points);
  return PLAN_OK;
}

bool ReferenceLine::SLToXY(const FrenetPoint &frenet_point,
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