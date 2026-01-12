

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <utility>

#include "plan_common/log.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/math/util.h"
#include "plan_common/util/path_util.h"

namespace st {
namespace planning {

DiscretizedPath::DiscretizedPath(std::vector<PathPoint> path_points)
    : std::vector<PathPoint>(std::move(path_points)) {
  for (auto it = begin(); it + 1 < end(); ++it) {
    DCHECK_LE(it->s(), (it + 1)->s())
        << "at path point[" << it - begin() << "]";
  }
  CHECK_EQ(begin()->s(), 0.0);
}

FrenetCoordinate DiscretizedPath::XYToSL(const Vec2d& pos) const {
  constexpr double kEpsilon = 1e-10;
  CHECK_GT(size(), 1);
  std::vector<Vec2d> points;
  std::vector<double> s_knots;
  points.reserve(size());
  s_knots.reserve(size());
  for (auto iter = begin(); iter < end(); ++iter) {
    points.emplace_back(iter->x(), iter->y());
    s_knots.emplace_back(iter->s());
  }

  FrenetCoordinate res{0.0, 0.0};
  double min_dist_sqr = std::numeric_limits<double>::infinity();

  for (size_t i = 1; i < points.size(); ++i) {
    const Vec2d& start = points[i - 1];
    const Vec2d& end = points[i];
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    const double length = Hypot(dx, dy);
    const Vec2d unit_direction =
        (length <= kEpsilon ? Vec2d(0, 0) : Vec2d(dx / length, dy / length));
    double s = 0.0;
    double l_sqr = 0.0;
    double l_sign = 1.0;
    if (length < kEpsilon) {
      s = s_knots[i - 1];
      l_sqr = Sqr(pos.x() - start.x()) + Sqr(pos.y() - start.y());
    } else {
      const double x0 = pos.x() - start.x();
      const double y0 = pos.y() - start.y();
      const double projection =
          x0 * unit_direction.x() + y0 * unit_direction.y();
      const double production =
          x0 * unit_direction.y() - y0 * unit_direction.x();
      l_sign = production < 0.0 ? 1.0 : -1.0;
      if (projection < 0.0 && i > 1) {
        s = s_knots[i - 1];
        l_sqr = Sqr(x0) + Sqr(y0);
      } else if (projection > length && i + 1 < size()) {
        s = s_knots[i];
        l_sqr = Sqr(pos.x() - end.x()) + Sqr(pos.y() - end.y());
      } else {
        s = s_knots[i - 1] + projection;
        l_sqr = Sqr(production);
      }
    }

    if (l_sqr < min_dist_sqr) {
      min_dist_sqr = l_sqr;
      res = {s, std::sqrt(l_sqr) * l_sign};
    }
  }
  return res;
}

FrenetPolygon DiscretizedPath::XYToSL(const Polygon2d& contour,
                                      const Vec2d& pos) const {
  CHECK_GT(size(), 1);
  const auto& points = contour.points();
  std::vector<FrenetCoordinate> coords;
  coords.reserve(points.size());
  for (const auto& pt : points) {
    coords.push_back(XYToSL(pt));
  }

  FrenetCoordinate center_coord;
  center_coord = XYToSL(pos);

  FrenetPolygon frenet_polygon{
      .s_max = -std::numeric_limits<double>::infinity(),
      .s_min = std::numeric_limits<double>::infinity(),
      .l_max = -std::numeric_limits<double>::infinity(),
      .l_min = std::numeric_limits<double>::infinity(),
      .center = center_coord};
  for (const auto& coord : coords) {
    frenet_polygon.s_max = std::max(frenet_polygon.s_max, coord.s);
    frenet_polygon.s_min = std::min(frenet_polygon.s_min, coord.s);
    frenet_polygon.l_max = std::max(frenet_polygon.l_max, coord.l);
    frenet_polygon.l_min = std::min(frenet_polygon.l_min, coord.l);
  }

  constexpr double kMinFrenetBoxLength = 1e-3;  // unit, m
  frenet_polygon.s_max = std::max(frenet_polygon.s_max,
                                  frenet_polygon.s_min + kMinFrenetBoxLength);
  frenet_polygon.l_max = std::max(frenet_polygon.l_max,
                                  frenet_polygon.l_min + kMinFrenetBoxLength);

  return frenet_polygon;
}

PathPoint DiscretizedPath::Evaluate(double path_s) const {
  CHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }

  PathPoint path_point;
  if ((it_lower - 1)->s() == it_lower->s()) {
    path_point = *(it_lower - 1);
  } else {
    const double alpha =
        (path_s - (it_lower - 1)->s()) / (it_lower->s() - (it_lower - 1)->s());
    LerpPathPoint(*(it_lower - 1), *it_lower, alpha, &path_point);
  }

  return path_point;
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    double path_s) const {
  auto func = [](const PathPoint& tp, double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

PathPoint DiscretizedPath::EvaluateReverse(double path_s) const {
  CHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }

  PathPoint path_point;
  if ((it_upper - 1)->s() == it_upper->s()) {
    path_point = *(it_upper - 1);
  } else {
    const double alpha =
        (path_s - (it_upper - 1)->s()) / (it_upper->s() - (it_upper - 1)->s());
    LerpPathPoint(*(it_upper - 1), *it_upper, alpha, &path_point);
  }

  return path_point;
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(
    double path_s) const {
  auto func = [](const double path_s, const PathPoint& tp) {
    return tp.s() < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}

}  // namespace planning
}  // namespace st
