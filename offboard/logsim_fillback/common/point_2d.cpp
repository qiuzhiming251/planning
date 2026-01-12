#include "common/point_2d.h"
namespace worldview {
namespace util {
Point2D transformVectorToOdometryFrame(const Point2D& vcs_vector,
                                       const Pose2D& car_pose) {
  Point2D odometry_vector;

  double cos_theta = std::cos(car_pose.theta);
  double sin_theta = std::sin(car_pose.theta);
  odometry_vector.x = vcs_vector.x * cos_theta - vcs_vector.y * sin_theta;
  odometry_vector.y = vcs_vector.x * sin_theta + vcs_vector.y * cos_theta;

  return odometry_vector;
}
Point2D transformPointToOdometryFrame(const Point2D& vcs_point,
                                      const Pose2D& car_pose) {
  Point2D odometry_point;

  double cos_theta = std::cos(car_pose.theta);
  double sin_theta = std::sin(car_pose.theta);
  odometry_point.x =
      vcs_point.x * cos_theta - vcs_point.y * sin_theta + car_pose.point.x;
  odometry_point.y =
      vcs_point.x * sin_theta + vcs_point.y * cos_theta + car_pose.point.y;

  return odometry_point;
}
Pose2D transformPoseToOdometryFrame(const Pose2D& vcs_pose,
                                    const Pose2D& car_pose) {
  Pose2D odometry_pose_of_car;

  double cos_theta = std::cos(car_pose.theta);
  double sin_theta = std::sin(car_pose.theta);
  odometry_pose_of_car.point.x = vcs_pose.point.x * cos_theta -
                                 vcs_pose.point.y * sin_theta +
                                 car_pose.point.x;
  odometry_pose_of_car.point.y = vcs_pose.point.x * sin_theta +
                                 vcs_pose.point.y * cos_theta +
                                 car_pose.point.y;

  odometry_pose_of_car.theta = vcs_pose.theta + car_pose.theta;

  return odometry_pose_of_car;
}
}  // namespace util
}  // namespace worldview