//
// Created by xxx on 20/5/22.
//

#ifndef COMMON_TRANSFORMER
#define COMMON_TRANSFORMER
#include "plan_common/type_def.h"
#include "plan_common/math/vec3d.h"

namespace ad_byd::planning {
class Transformer {
 public:
  Transformer() = delete;
  explicit Transformer(const TransformInfo &info);  // left multiplication tf
  explicit Transformer(const math::Vec2d &delta_pos, const double &delta_yaw);
  ~Transformer() = default;

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

 private:
  math::Isometry3d tf_ = math::Isometry3d::Identity();
};
}  // namespace ad_byd::planning

#endif  // COMMON_TRANSFORMER
