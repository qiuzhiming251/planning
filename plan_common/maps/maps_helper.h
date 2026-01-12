

#ifndef ONBOARD_MAPS_MAPS_HELPER_H_
#define ONBOARD_MAPS_MAPS_HELPER_H_

#include <algorithm>
#include <iterator>
#include <vector>

#include <stdint.h>

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/log.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/vec.h"

namespace st::mapping {

template <class T>
bool IsOnSameLevel(const T& a, const T& b) {
  for (const auto& val : b) {
    if (std::find(a.begin(), a.end(), val) != a.end()) return true;
  }
  return false;
}

// The unit of the return value is meters.

// shape.
double ScaleToValue(int scale, int min_scale, int max_scale,
                    double scale_precision);
int ValueToScale(double value, int min_scale, int max_scale,
                 double scale_precision);
double ScaleToCurvature(int scale);
int CurvatureToScale(double curvature);
double ScaleToHeading(int scale);  // unit: deg.
int HeadingToScale(double heading);
double ScaleToSlope(int scale);  // unit: deg.
int SlopeToScale(double slope);
double ScaleToBanking(int scale);  // unit: deg.
int BankingToScale(double banking);

inline std::vector<Segment2d> Vec2dToSegments(
    const std::vector<Vec2d>& points) {
  CHECK_GE(points.size(), 2);
  std::vector<Segment2d> segments;
  segments.reserve(points.size() - 1);
  for (auto first = points.begin(), second = std::next(first);
       second != points.end(); ++first, ++second) {
    segments.emplace_back(*first, *second);
  }
  return segments;
}

}  // namespace st::mapping

#endif  // ONBOARD_MAPS_MAPS_HELPER_H_
