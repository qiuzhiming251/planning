

#include <algorithm>
#include <cmath>
#include <ostream>
#include <vector>

#include "plan_common/log.h"
#include "plan_common/math/geometry/util.h"

namespace st {

// TODO: maybe useful
// Polygon2d SmoothPolygon2dFromGeoPolygonProto(const mapping::GeoPolygonProto&
// geo_polygon_proto,
//                                              const CoordinateConverter&
//                                              coordinate_converter) {
//   std::vector<Vec2d> smooth_points;
//   for (const auto& point : geo_polygon_proto.points()) {
//     const Vec3d smooth_point =
//         coordinate_converter.GlobalToSmooth({point.longitude(),
//         point.latitude(), point.altitude()});
//     smooth_points.emplace_back(smooth_point.x(), smooth_point.y());
//   }
//   if (smooth_points.size() < 3) return Polygon2d();
//   return Polygon2d(smooth_points);
// }

// AffineTransformation Mat4fToAffineTransformation(const Mat4f& mat) {
//   return AffineTransformation::FromTranslation(
//              mat.col(3).segment<3>(0).cast<double>())
//       .ApplyRotation(Eigen::Quaternionf(mat.block<3, 3>(0, 0))
//                          .cast<double>()
//                          .normalized());
// }

std::pair<double, double> ProjectBoxToRay(const Vec2d& ray_center,
                                          const Vec2d& ray_dir,
                                          const Box2d& box) {
  const double center = ray_dir.Dot(box.center() - ray_center);
  const Vec2d box_tangent = box.tangent();
  const double offset =
      0.5 * (std::abs(ray_dir.Dot(box_tangent * box.length())) +
             std::abs(ray_dir.Dot(box_tangent.Perp() * box.width())));
  return {center - offset, center + offset};
}

std::vector<Circle2d> GenerateCirclesInsideBox(const Box2d& box,
                                               double circle_center_dist) {
  // Return one circle if the box's length and width very similar.
  if (std::abs(box.half_length() - box.half_width()) < 0.01) {
    return {
        Circle2d(box.center(), std::min(box.half_length(), box.half_width()))};
  }

  // Swap the box's length and width if width is larger than length.
  if (box.length() < box.width()) {
    return GenerateCirclesInsideBox(
        Box2d(box.half_width(), box.half_length(), box.center(),
              NormalizeAngle(box.heading() + M_PI_2), box.tangent().Perp()),
        circle_center_dist);
  }

  const double circle_radius = box.half_width();
  const int num_circles =
      CeilToInt((box.length() - 2.0 * circle_radius) / circle_center_dist) + 1;

  std::vector<Circle2d> circles;
  circles.reserve(num_circles);

  const Vec2d rear_center = box.center() - box.tangent() * box.half_length();
  const double inner_circle_step =
      (box.length() - 2.0 * circle_radius) / (num_circles - 1);
  for (int i = 0; i < num_circles; ++i) {
    circles.push_back(Circle2d(
        rear_center + box.tangent() * (circle_radius + i * inner_circle_step),
        circle_radius));
  }

  return circles;
}

Mat3d ComputeMeanValueForRoatationMatrices(
    const std::vector<Mat3d>& rotation_matrices) {
  CHECK_GT(rotation_matrices.size(), 1);
  Mat3d R = rotation_matrices[0];  // NOLINT(readability-identifier-naming)
  const double epsilon = 1e-13;
  for (int loop = 0; loop < 20; ++loop) {
    Vec3d r = Vec3d::Zero();
    for (const auto& mat : rotation_matrices) {
      Eigen::AngleAxisd angle_axis(R.transpose() * mat);
      r += angle_axis.angle() * angle_axis.axis();
    }
    r /= rotation_matrices.size();

    if (r.norm() < epsilon) return R;

    Eigen::AngleAxisd so3_r(r.norm(), r.normalized());
    R = R * so3_r;
  }
  LOG_WARN << "Compute Mean Value For Roatation Matrices : bad converge!!!";
  return R;
}

}  // namespace st
