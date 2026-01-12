

#ifndef AD_BYD_PLANNING_MATH_LINE_CURVE_H
#define AD_BYD_PLANNING_MATH_LINE_CURVE_H

#include <iostream>

#include <cereal/access.hpp>
#include "plan_common/math/line_segment2d.h"
#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {
namespace math {

class LineCurve2d {
 public:
  LineCurve2d() = default;
  explicit LineCurve2d(const std::vector<Vec2d> &points);

  void InitializePoints(const std::vector<Vec2d> &points);
  Vec2d begin_point() const;
  Vec2d end_point() const;
  const std::vector<Vec2d> &points() const { return points_; }
  double length() const {
    return accumulated_s_.empty() ? 0.0 : accumulated_s_.back();
  }

  const std::vector<double> &GetAccuLength() const { return accumulated_s_; }
  /// @brief check if input has valid geometry info
  /// @return true-false
  bool IsValid() const { return points_.size() >= 2; }

  /// @brief clear private variable
  /// @return NULL
  void Clear();

  /// @brief get point on line at s
  /// @param s the length from start
  /// @return x-y
  Vec2d GetPointAtS(const double s) const;

  /// @brief get index on line by s
  /// @param s the length from start
  /// @return index
  int GetIndexByS(const double s) const;

  /// @brief sample points from start_s to end with interval
  /// @param start_s the begin position from line start
  /// @param interval
  /// @return points
  bool SamplePoints(const double start_s, const double length,
                    std::vector<Vec2d> *const points,
                    const double interval = 3.0) const;

  /// @brief get the min distance from the center line
  /// @param point
  /// @param nearest_point the nearest point on center line
  /// @param s_offset the length from start of lane
  /// @return min distance
  double GetDistance(const Vec2d &point, Vec2d *nearest_point,
                     double *s_offset) const;

  /// @brief get the min distance from the center line
  /// @param x coordinate
  /// @param y coordinate
  /// @return min distance
  double GetDistance(const double x, const double y) const;

  /// @brief get the min distance from the center line
  /// @param point
  /// @return min distance
  double GetDistance(const Vec2d &point) const;

  /// @brief get the projection to center line
  /// @param point
  /// @param accumulate_s the length from start of lane
  /// @param lateral positive if on left of lane
  /// @return true if can project
  bool GetProjection(const Vec2d &point, double *accumulate_s,
                     double *lateral) const;

  bool GetPoint(const double s, const double l, Vec2d *const point) const;

  bool GetHeadingFromS(const double s, double *const heading) const;

 private:
  std::vector<double> accumulated_s_;
  std::vector<double> headings_;
  std::vector<Vec2d> points_;

  template <class Archive>
  friend void serialize(Archive &ar, LineCurve2d &line_curve);
};

}  // namespace math
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MATH_LINE_CURVE_H