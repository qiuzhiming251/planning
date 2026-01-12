
#ifndef AD_BYD_PLANNING_MATH_POSE_TRANSFROM_H_
#define AD_BYD_PLANNING_MATH_POSE_TRANSFROM_H_
#include "plan_common/type_def.h"
#include "plan_common/math/vec3d.h"

namespace ad_byd {
namespace planning {
class PoseTransform {
 public:
  PoseTransform() = delete;
  explicit PoseTransform(const TransformInfo &info);  // left multiplication tf
  explicit PoseTransform(const math::Vec2d &delta_pos, const double &delta_yaw);
  ~PoseTransform() = default;

  /// @brief Transform using TransformInfo (Transform * point)
  /// @param pt Origin Point2d/PathPoint
  /// @return return transformed Point2d/PathPoint
  Point2d Trans(const Point2d &pt) const;
  std::vector<Point2d> Trans(const std::vector<Point2d> &pts) const;
  PathPoint Trans(const PathPoint &pt) const;
  std::vector<PathPoint> Trans(const std::vector<PathPoint> &pts) const;
  TrajectoryPoint Trans(const TrajectoryPoint &pt) const;

  /// @brief Transform_inverse using TransformInfo (point * Transform)
  /// @param pt Origin Point2d/PathPoint
  /// @return return transformed Point2d/PathPoint
  Point2d TransInverse(const Point2d &pt) const;
  std::vector<Point2d> TransInverse(const std::vector<Point2d> &pts) const;
  PathPoint TransInverse(const PathPoint &pt) const;
  std::vector<PathPoint> TransInverse(const std::vector<PathPoint> &pts) const;
  TrajectoryPoint TransInverse(const TrajectoryPoint &pt) const;

  /// @brief Trans Quaterniond to EulerAngles
  /// reference:
  /// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  /// @param q Quaterniond
  /// @return return Vector3d EulerAngles, rotation about X, Y and Z axis.
  static math::Vector3d ToEulerAngles(const math::Quaterniond &q);

 private:
  math::Isometry3d tf_ = math::Isometry3d::Identity();
};
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MATH_POSE_TRANSFROM_H_
