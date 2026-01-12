

#ifndef ST_PLANNING_MATH_GEOMETRY_UTIL
#define ST_PLANNING_MATH_GEOMETRY_UTIL

#include <iterator>
#include <utility>
#include <vector>

#ifndef BOOST_ALLOW_DEPRECATED_HEADERS
#define BOOST_ALLOW_DEPRECATED_HEADERS
#include "boost/geometry/algorithms/simplify.hpp"
#undef BOOST_ALLOW_DEPRECATED_HEADERS
#else
#include "boost/geometry/algorithms/simplify.hpp"
#endif

#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
//#include "lidar/vehicle_pose.h"
//#include "map_geometry.pb.h"
//#include "plan_common/math/coordinate_converter.h"
#include "plan_common/math/eigen.h"
//#include "plan_common/math/geometry/affine_transformation.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/circle2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st {

class VehiclePoseLerper {
 public:
  // VehiclePose operator()(const VehiclePose& pose1, const VehiclePose& pose2,
  // double alpha) const {
  //   const Quaternion q1 =
  //   AffineTransformation::YawPitchRollToQuaternion({pose1.yaw, pose1.pitch,
  //   pose1.roll}); const Quaternion q2 =
  //   AffineTransformation::YawPitchRollToQuaternion({pose2.yaw, pose2.pitch,
  //   pose2.roll}); const Quaternion q = q1.slerp(alpha, q2); const Vec3d ypr =
  //   AffineTransformation::QuaternionToYawPitchRoll(q); const double x =
  //   Lerp(pose1.x, pose2.x, alpha); const double y = Lerp(pose1.y, pose2.y,
  //   alpha); const double z = Lerp(pose1.z, pose2.z, alpha); return
  //   VehiclePose(x, y, z, ypr[0], ypr[1], ypr[2]);
  // }
};

template <typename T>
Eigen::Matrix<T, 3, 3> SkewSymmetricMatrix(const Eigen::Matrix<T, 3, 1>& v) {
  Eigen::Matrix<T, 3, 3> m = Eigen::Matrix<T, 3, 3>::Zero();
  m(0, 1) = -v[2];
  m(1, 0) = v[2];
  m(0, 2) = v[1];
  m(2, 0) = -v[1];
  m(1, 2) = -v[0];
  m(2, 1) = v[0];
  return m;
}

inline Quaternion VectorToVectorQuaternion(const Vec3d& vec) {
  return Quaternion(0.0, vec.x(), vec.y(), vec.z());
}

inline Quaternion EulerVectorToQuaternion(Vec3d vec) {
  const double angle = vec.norm();
  if (angle < 1e-8) {
    Quaternion q(1.0, 0.5 * vec.x(), 0.5 * vec.y(), 0.5 * vec.z());
    q.normalize();
    return q;
  }
  vec /= angle;
  return Quaternion(AngleAxis(angle, vec));
}

inline void Vec2dToProto(const Vec2d& v, Vec2dProto* proto) {
  proto->set_x(v.x());
  proto->set_y(v.y());
}

inline Vec2d Vec2dFromProto(const Vec2dProto& proto) {
  return Vec2d(proto.x(), proto.y());
}

inline Vec2d Vec2dFromProto(const Vec3dProto& proto) {
  return Vec2d(proto.x(), proto.y());
}

inline Vec2d Vec2dFromPoseProto(const PoseProto& proto) {
  return Vec2d(proto.pos_smooth().x(), proto.pos_smooth().y());
}

inline Vec2d Vec2dFromTrajectoryPointProto(const TrajectoryPointProto& proto) {
  return Vec2d(proto.pos().x(), proto.pos().y());
}

inline Vec2d Vec2dFromApolloTrajectoryPointProto(
    const ApolloTrajectoryPointProto& proto) {
  return Vec2d(proto.path_point().x(), proto.path_point().y());
}

inline Vec2d Vec2dFromPathPoint(const PathPoint& proto) {
  return Vec2d(proto.x(), proto.y());
}

inline void Vec3dToProto(const Vec3d& v, Vec3dProto* proto) {
  proto->set_x(v.x());
  proto->set_y(v.y());
  proto->set_z(v.z());
}

inline Vec3d Vec3dFromProto(const Vec3dProto& proto) {
  return Vec3d(proto.x(), proto.y(), proto.z());
}

inline void Vec3iToProto(const Vec3i& v, Vec3iProto* proto) {
  proto->set_x(v.x());
  proto->set_y(v.y());
  proto->set_z(v.z());
}

inline Vec3i Vec3iFromProto(const Vec3iProto& proto) {
  return Vec3i(proto.x(), proto.y(), proto.z());
}

inline void Vec4dToProto(const Vec4d& v, Vec4dProto* proto) {
  proto->set_x(v.x());
  proto->set_y(v.y());
  proto->set_z(v.z());
  proto->set_w(v.w());
}

inline Vec4d Vec4dFromProto(const Vec4dProto& proto) {
  return Vec4d(proto.x(), proto.y(), proto.z(), proto.w());
}

inline void Mat2dToProto(const Mat2d& m, Mat2dProto* proto) {
  for (int i = 0; i < 4; ++i) proto->add_m(m.data()[i]);
}

inline void Mat2dFromProto(const Mat2dProto& proto, Mat2d* m) {
  CHECK_EQ(proto.m_size(), 4);
  for (int i = 0; i < 4; ++i) m->data()[i] = proto.m(i);
}

inline void Mat3dToProto(const Mat3d& m, Mat3dProto* proto) {
  for (int i = 0; i < 9; ++i) proto->add_m(m.data()[i]);
}

inline void Mat3dFromProto(const Mat3dProto& proto, Mat3d* m) {
  CHECK_EQ(proto.m_size(), 9);
  for (int i = 0; i < 9; ++i) m->data()[i] = proto.m(i);
}

inline void Mat4dToProto(const Mat4d& m, Mat4dProto* proto) {
  for (int i = 0; i < 16; ++i) proto->add_m(m.data()[i]);
}

inline void Mat4dFromProto(const Mat4dProto& proto, Mat4d* m) {
  CHECK_EQ(proto.m_size(), 16);
  for (int i = 0; i < 16; ++i) m->data()[i] = proto.m(i);
}

// Project a box to a ray, and return the minimum and maximum projection
// distance on the ray. The ray is defined by a point `ray_center` and a unit
// direction vector `ray_dir`.
std::pair<double, double> ProjectBoxToRay(const Vec2d& ray_center,
                                          const Vec2d& ray_dir,
                                          const Box2d& box);

// Converts a GeoPolygonProto into a smooth Polygon2d object.
// Polygon2d CoordinateConverter(const mapping::GeoPolygonProto&
// geo_polygon_proto,
//                               const CoordinateConverter&
//                               coordinate_converter);

// convert Eigen's Mat4f to AffineTransformation to avoid precision conversion
// error
// AffineTransformation Mat4fToAffineTransformation(const Mat4f& mat);

/// @brief out always std::vector,shoule allocate enough memory before.
template <typename InputIt, typename OutIt>
int FillNoSelfIntersectSimplifyPoints(InputIt first, InputIt last, OutIt out,
                                      double epision) {
  typedef boost::geometry::model::d2::point_xy<double> xy;
  typedef boost::geometry::model::linestring<xy> PLine;
  PLine pline;
  pline.reserve(std::distance(first, last));
  for (InputIt it = first; it != last; it++) {
    pline.emplace_back(it->x(), it->y());
  }
  PLine simplified;
  boost::geometry::simplify(pline, simplified, epision);
  for (const auto& point : simplified) {
    *out++ = {point.x(), point.y()};
  }
  return simplified.size();
}

// Create circles equally distributed along the longer edge of the box. The
// circles are inside the box. The circles centers are separated by maximum
// distance `dist`. See
std::vector<Circle2d> GenerateCirclesInsideBox(const Box2d& box,
                                               double circle_center_dist);

Mat3d ComputeMeanValueForRoatationMatrices(
    const std::vector<Mat3d>& rotation_matrices);

}  // namespace st

#endif  // ST_PLANNING_MATH_GEOMETRY_UTIL
