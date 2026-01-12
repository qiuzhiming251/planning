//
// Created by xxx on 11/02/21.
//
#ifndef PILOT_PLANNING_COMMON_CURVE_BOUNDARY_H_
#define PILOT_PLANNING_COMMON_CURVE_BOUNDARY_H_
#include <map>
#include <vector>

#include "plan_common/type_def.h"
#include "plan_common/math/line_curve1d.h"

namespace ad_byd {
namespace planning {
class CurveBoundary1d {
 public:
  CurveBoundary1d() = default;
  ~CurveBoundary1d() = default;

  bool Create(const LineCurve1d &lower_boundary,
              const LineCurve1d &upper_boundary, double s_interval = 0.5);

  // Clear curve map.
  void Clear() { boundary_map_.clear(); };
  bool IsValid() const { return !boundary_map_.empty(); }

  // Set boundaries at position x
  void SetBoundary(const double x, const double lower, const double upper);

  /// @brief Get boundary by key, output terminal value if key is out of range.
  /// @param key get boundary by key.
  /// @param lower curve lower boundary in position x.
  /// @param upper  curve upper boundary in position x.
  /// @return FALSE if too less key or value.
  bool GetBoundary(const double x, double *lower, double *upper) const;
  const std::map<double, std::pair<double, double>> &GetBoundary() const {
    return boundary_map_;
  }
  bool GetBoundary(std::vector<Point2d> &lower,
                   std::vector<Point2d> &upper) const;
  /// @brief Get the range of key.
  /// @param begin begin key.
  /// @param end end key.
  bool GetRange(double *begin, double *end) const;

 private:
  /// map<s, pair<l_lower, l_upper>>
  std::map<double, std::pair<double, double>> boundary_map_;
};
using CurveBoundary1dPtr = std::shared_ptr<CurveBoundary1d>;
}  // namespace planning
}  // namespace ad_byd
#endif  // PILOT_PLANNING_COMMON_CURVE_BOUNDARY_H_
