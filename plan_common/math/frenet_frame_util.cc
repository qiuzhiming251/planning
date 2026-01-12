

#include <algorithm>
#include <cmath>
#include <limits>

#include "plan_common/math/frenet_frame_util.h"
#include "plan_common/math/util.h"

namespace st::frenet_frame_util {

Vec2d SLToXY(const FrenetCoordinate& sl, absl::Span<const Vec2d> anchor_points,
             absl::Span<const double> anchor_s) {
  CHECK_EQ(anchor_points.size(), anchor_s.size());
  CHECK_GT(anchor_s.size(), 1);

  const auto it = std::lower_bound(anchor_s.begin(), anchor_s.end(), sl.s);

  Vec2d prev_pt, succ_pt;
  double interp;
  if (it == anchor_s.begin()) {
    prev_pt = anchor_points[0];
    succ_pt = anchor_points[1];
    interp = (sl.s - *it) / (*(it + 1) - *it);
  } else if (it == anchor_s.end()) {
    prev_pt = anchor_points[anchor_points.size() - 2];
    succ_pt = anchor_points.back();
    interp = (sl.s - *(it - 2)) / (*(it - 1) - *(it - 2));
  } else {
    prev_pt = anchor_points[it - 1 - anchor_s.begin()];
    succ_pt = anchor_points[it - anchor_s.begin()];
    interp = (sl.s - *(it - 1)) / (*it - *(it - 1));
  }
  const Vec2d heading_vec = (succ_pt - prev_pt).normalized();
  const Vec2d normal_vec = heading_vec.Perp();
  return Lerp(prev_pt, succ_pt, interp) + normal_vec * sl.l;
}

FrenetCoordinate XYToSLBruteForce(const Vec2d& xy,
                                  absl::Span<const Vec2d> anchor_points,
                                  absl::Span<const double> anchor_s) {
  CHECK_EQ(anchor_points.size(), anchor_s.size());
  CHECK_GT(anchor_s.size(), 1);
  double min_d = std::numeric_limits<double>::infinity();
  FrenetCoordinate sl;
  for (size_t i = 1; i < anchor_points.size(); ++i) {
    const Vec2d p0 = anchor_points[i - 1];
    const Vec2d p1 = anchor_points[i];
    const Vec2d segment = p1 - p0;
    const double seg_len_inv = 1.0 / segment.norm();
    const Vec2d heading_vec = segment * seg_len_inv;
    const Vec2d query_segment = xy - p0;
    const double projection = query_segment.dot(heading_vec) * seg_len_inv;
    const double production = heading_vec.CrossProd(query_segment);
    double l = 0.0;
    double s = 0.0;
    if (projection < 0.0 && i > 1) {
      l = std::copysign(p0.DistanceTo(xy), production);
      s = anchor_s[i - 1];
    } else if (projection > 1.0 && i + 1 < anchor_points.size()) {
      l = std::copysign(p1.DistanceTo(xy), production);
      s = anchor_s[i];
    } else {
      l = production;
      s = Lerp(anchor_s[i - 1], anchor_s[i], projection);
    }

    if (std::fabs(l) < min_d) {
      min_d = std::fabs(l);
      sl = {s, l};
    }
  }
  return sl;
}

}  // namespace st::frenet_frame_util
