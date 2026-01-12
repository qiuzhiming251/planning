
#include <float.h>

#include "plan_common/math/line_curve1d.h"
#include "plan_common/math/linear_interpolation.h"

namespace ad_byd {
namespace planning {
void LineCurve1d::SetValue(const double x, const double value) {
  curve_map_[x] = value;
}

double LineCurve1d::GetValue(const double x) const {
  if (curve_map_.empty()) {
    return DBL_MAX;
  }
  if (curve_map_.size() == 1) {
    return curve_map_.begin()->second;
  }
  auto it0 = curve_map_.lower_bound(x);
  if (it0 == curve_map_.begin()) {
    return curve_map_.begin()->second;
  } else if (it0 == curve_map_.end()) {
    return curve_map_.rbegin()->second;
  }
  auto it1 = it0;
  it0--;
  return math::lerp(it0->second, it0->first, it1->second, it1->first, x, true);
}

bool LineCurve1d::GetRange(double *begin, double *end) const {
  if (curve_map_.empty()) {
    return false;
  }
  *begin = curve_map_.begin()->first;
  *end = curve_map_.rbegin()->first;
  return true;
}
}  // namespace planning
}  // namespace ad_byd