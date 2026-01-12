
#ifndef AD_BYD_PLANNING_MATH_CURVE_BOUNDARY_1D_H_
#define AD_BYD_PLANNING_MATH_CURVE_BOUNDARY_1D_H_
#include <map>
#include <vector>

#include "plan_common/type_def.h"
#include "plan_common/math/line_curve1d.h"

namespace ad_byd {
namespace planning {
class CurveLimit {
 public:
  CurveLimit() = default;
  ~CurveLimit() = default;

  bool Create(const LineCurve1d &lower_boundary,
              const LineCurve1d &upper_boundary, double s_interval = 0.5);

  // Clear curve map.
  void Clear() { boundary_map_.clear(); };
  bool IsValid() const { return !boundary_map_.empty(); }

  // Set boundaries at position x
  void SetBoundary(const double &x, const double &lower, const double &upper);

  /// @brief Get boundary by key, output terminal value if key is out of range.
  /// @param key get boundary by key.
  /// @param lower curve lower boundary in position x.
  /// @param upper  curve upper boundary in position x.
  /// @return FALSE if too less key or value.
  bool GetBoundary(const double &x, double *lower, double *upper) const;
  const std::map<double, std::pair<double, double>> &GetBoundary() const {
    return boundary_map_;
  }
  bool GetBoundary(std::vector<double> &x, std::vector<double> &lower,
                   std::vector<double> &upper) const;
  bool GetBoundary(std::vector<ad_byd::planning::Point2d> &lower,
                   std::vector<ad_byd::planning::Point2d> &upper) const;
  bool GetBoundary(std::map<double, double> &left_limit,
                   std::map<double, double> &right_limit) const;

  /// @brief Get the range of key.
  /// @param begin begin key.
  /// @param end end key.
  bool GetRange(double *begin, double *end) const;

 private:
  /// map<s, pair<l_lower, l_upper>>
  std::map<double, std::pair<double, double>> boundary_map_;
};
using CurveLimitPtr = std::shared_ptr<CurveLimit>;
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MATH_CURVE_BOUNDARY_1D_H_
