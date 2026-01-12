
#ifndef AD_BYD_PLANNING_MATH_LINE_CURVE1D_H_
#define AD_BYD_PLANNING_MATH_LINE_CURVE1D_H_
#include <map>

namespace ad_byd {
namespace planning {
class LineCurve1d {
 public:
  LineCurve1d() = default;
  ~LineCurve1d() = default;

  // Clear curve map.
  void Clear() { curve_map_.clear(); };

  bool IsValid() const { return !curve_map_.empty(); }

  // Set position x and value.
  void SetValue(const double x, const double value);

  /// @brief Get value by key, output terminal value if the key is out of range.
  /// @param x key.
  /// @return return value.
  double GetValue(const double x) const;

  /// @brief Get the range of x.
  /// @param begin begin key.
  /// @param end end key.
  /// @return FALSE if too less key or value.
  bool GetRange(double *begin, double *end) const;

  const std::map<double, double> &GetCurveMap() const { return curve_map_; }

 private:
  std::map<double, double> curve_map_;
};
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MATH_LINE_CURVE1D_H_