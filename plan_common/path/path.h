
#ifndef AD_BYD_PLANNING_COMMON_PATH_PATH_H
#define AD_BYD_PLANNING_COMMON_PATH_PATH_H

#include "plan_common/log.h"
#include "plan_common/type_def.h"
#include "plan_common/math/box2d.h"
#include "plan_common/math/cartesian_frenet_conversion.h"
#include "plan_common/math/line_segment2d.h"
#include "plan_common/math/linear_interpolation.h"

namespace ad_byd {
namespace planning {

class Path {
 public:
  explicit Path(const std::vector<Point2d> &xy_points) {
    set_points(xy_points);
  }

  /// @brief None copy constructor, if the input is an lvalue, use move please
  /// @param compute_path_profile Only compute accum_s if False.
  explicit Path(std::vector<PathPoint> &&path_points,
                bool compute_path_profile = true);

  void set_points(const std::vector<Point2d> &xy_points);
  void set_points(std::vector<PathPoint> &&path_points,
                  bool compute_path_profile = true);
  void Reset() { points_.clear(); }
  const std::vector<PathPoint> &points() const { return points_; }

  /// @brief Get trajectory point by accum_s, interpolate by accum_s
  /// @param accum_s input accum_s, unit m
  /// @return PathPoint point
  PathPoint GetPointAtS(const double &accum_s) const;
  PathPoint GetPrecisionPointAtS(const double &accum_s) const;

  /// @brief Get nearest point by accum_s, lower_bound
  /// @param accum_s input accum_s, unit m
  /// @return PathPoint point index
  int GetNearestIndexAtS(const double &accum_s) const;

  /// @brief Get the vertical distance to the path
  /// @param point input point2d
  /// @param nearest_point PathPoint point
  /// @return output distance to points_
  double GetDistance(const Point2d &point, PathPoint *nearest_point) const;

  /// @brief Convert Frenet SL to XY point
  bool SLToXY(const SLPoint &sl_point, Point2d *xy_point,
              PathPoint *ref_point) const;

  bool SLToXY(std::vector<PathPoint> &path_points) const;

  /// @brief Convert XY to Frenet SL point
  bool XYToSL(const Point2d &xy_point, SLPoint *sl_point,
              PathPoint *ref_point) const;

  bool XYToSL(const TrajectoryPoint &traj_point,
              FrenetPoint *frenet_point) const;

  bool XYToSL(std::vector<PathPoint> &path_points) const;

  bool XYToSL(const math::Box2d &box_2d, SLBoundary *sl_boundary) const;

  bool XYToSL(const std::vector<Point2d> &corners,
              SLBoundary *sl_boundary) const;

  /// @brief Total path length
  double length() const {
    return points_.size() < 2 ? 0.0 : points_.back().accum_s;
  }

  bool IsValid() const {
    return points_.size() >= 2 && (!std::isnan(points_.front().x())) &&
           (!std::isnan(points_.front().y()));
  }

  /// @brief path resample by accumulated s
  /// @param start_accu_s the start accu_s is accum_s
  /// @param length The total length of the sample
  /// @param interval Sampling interval
  /// @param path_points The rvalue of output path of the sample
  bool SamplePoints(const double &start_accu_s, const double &length,
                    const double &interval,
                    std::vector<PathPoint> *const path_points) const;
  /// @brief Sample the points from the accumulated s
  /// @param accum_sequence input accumulated s vector
  /// @param path_points path points
  bool SamplePoints(const std::vector<double> &accum_sequence,
                    std::vector<PathPoint> *const path_points) const;

  bool HasIntersect(const math::LineSegment2d &input_segment,
                    math::Vec2d *const point) const;

  Path() = default;
  ~Path() = default;

 private:
  /// @brief Computing path profile, rewrite its accum_s theta kappa
  /// dkappa
  static bool ComputePathProfile(std::vector<PathPoint> &path_points);

  /// @brief Computing accumulated s, rewrite its accum_s
  static bool ComputeAccumulatedS(std::vector<PathPoint> &path_points);

  /// @brief Find a match PathPoint in the reference by x and y
  static PathPoint MatchToPath(const std::vector<PathPoint> &reference_line,
                               const double &x, const double &y);

  /// @brief Finds the first index that exceeds the input accum_s
  inline size_t GetIndexOfLowerBound(const double &accum_s) const;

 private:
  std::vector<PathPoint> points_;
};
using PathPtr = std::shared_ptr<Path>;
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_PATH_PATH_H
