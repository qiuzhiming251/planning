
#include "plan_common/log.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/pose_transform.h"
namespace ad_byd {
namespace planning {
PoseTransform::PoseTransform(const TransformInfo &info) {
  if (std::abs(info.delta_x) < math::kMathEpsilon &&
      std::abs(info.delta_y) < math::kMathEpsilon &&
      std::abs(info.delta_yaw) < math::kMathEpsilon) {
    tf_ = math::Isometry3d::Identity();
    return;
  }
  tf_.prerotate(math::AngleAxisd(0.0, math::Vector3d::UnitX()) *
                math::AngleAxisd(0.0, math::Vector3d::UnitY()) *
                math::AngleAxisd(info.delta_yaw, math::Vector3d::UnitZ()));
  tf_.pretranslate(math::Vector3d(info.delta_x, info.delta_y, 0.0));
}
PoseTransform::PoseTransform(const math::Vec2d &delta_pos,
                             const double &delta_yaw) {
  if (std::abs(delta_pos.x()) < math::kMathEpsilon &&
      std::abs(delta_pos.y()) < math::kMathEpsilon &&
      std::abs(delta_yaw) < math::kMathEpsilon) {
    tf_ = math::Isometry3d::Identity();
    return;
  }
  tf_.prerotate(math::AngleAxisd(0.0, math::Vector3d::UnitX()) *
                math::AngleAxisd(0.0, math::Vector3d::UnitY()) *
                math::AngleAxisd(delta_yaw, math::Vector3d::UnitZ()));
  tf_.pretranslate(math::Vector3d(delta_pos.x(), delta_pos.y(), 0.0));
}

Point2d PoseTransform::Trans(const Point2d &pt) const {
  //  left multiplication
  const math::Vector3d p = tf_.inverse() * math::Vector3d(pt.x(), pt.y(), 0.0);
  return {p.x(), p.y()};
}
std::vector<Point2d> PoseTransform::Trans(
    const std::vector<Point2d> &pts) const {
  //  left multiplication
  const math::Isometry3d tf_inverse = tf_.inverse();
  std::vector<Point2d> trans_pts;
  for (const auto &pt : pts) {
    const math::Vector3d p1 = tf_inverse * math::Vector3d(pt.x(), pt.y(), 0.0);
    trans_pts.emplace_back(p1.x(), p1.y());
  }
  return trans_pts;
}
PathPoint PoseTransform::Trans(const PathPoint &pt) const {
  // left multiplication
  const math::Isometry3d tf_inverse = tf_.inverse();
  const math::Vector3d pos = tf_inverse * math::Vector3d(pt.x(), pt.y(), 0.0);
  const auto delta_yaw =
      ToEulerAngles(math::Quaterniond(tf_inverse.rotation())).z();
  PathPoint trans_pt = pt;
  trans_pt.set_x(pos.x());
  trans_pt.set_y(pos.y());
  trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
  return trans_pt;
}
std::vector<PathPoint> PoseTransform::Trans(
    const std::vector<PathPoint> &pts) const {
  const math::Isometry3d tf_inverse = tf_.inverse();
  const auto delta_yaw =
      ToEulerAngles(math::Quaterniond(tf_inverse.rotation())).z();
  std::vector<PathPoint> trans_pts;
  for (const auto &pt : pts) {
    // left multiplication
    const math::Vector3d pos = tf_inverse * math::Vector3d(pt.x(), pt.y(), 0.0);
    PathPoint trans_pt = pt;
    trans_pt.set_x(pos.x());
    trans_pt.set_y(pos.y());
    trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
    trans_pts.emplace_back(trans_pt);
  }
  return trans_pts;
}
TrajectoryPoint PoseTransform::Trans(const TrajectoryPoint &pt) const {
  // left multiplication
  const math::Isometry3d tf_inverse = tf_.inverse();
  const math::Vector3d pos = tf_inverse * math::Vector3d(pt.x(), pt.y(), 0.0);
  const auto delta_yaw =
      ToEulerAngles(math::Quaterniond(tf_inverse.rotation())).z();
  TrajectoryPoint trans_pt = pt;
  trans_pt.set_x(pos.x());
  trans_pt.set_y(pos.y());
  trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
  return trans_pt;
}

Point2d PoseTransform::TransInverse(const Point2d &pt) const {
  const math::Vector3d p = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
  return {p.x(), p.y()};
}
std::vector<Point2d> PoseTransform::TransInverse(
    const std::vector<Point2d> &pts) const {
  std::vector<Point2d> trans_pts;
  for (const auto &pt : pts) {
    const math::Vector3d p1 = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
    trans_pts.emplace_back(p1.x(), p1.y());
  }
  return trans_pts;
}
PathPoint PoseTransform::TransInverse(const PathPoint &pt) const {
  const math::Vector3d pos = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
  const auto delta_yaw = ToEulerAngles(math::Quaterniond(tf_.rotation())).z();
  PathPoint trans_pt = pt;
  trans_pt.set_x(pos.x());
  trans_pt.set_y(pos.y());
  trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
  return trans_pt;
}
std::vector<PathPoint> PoseTransform::TransInverse(
    const std::vector<PathPoint> &pts) const {
  // left multiplication
  const auto delta_yaw = ToEulerAngles(math::Quaterniond(tf_.rotation())).z();
  std::vector<PathPoint> trans_pts;
  for (const auto &pt : pts) {
    PathPoint trans_pt = pt;
    const math::Vector3d pos = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
    trans_pt.set_x(pos.x());
    trans_pt.set_y(pos.y());
    trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
    trans_pts.emplace_back(trans_pt);
  }
  return trans_pts;
}
TrajectoryPoint PoseTransform::TransInverse(const TrajectoryPoint &pt) const {
  const math::Vector3d pos = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
  const auto delta_yaw = ToEulerAngles(math::Quaterniond(tf_.rotation())).z();
  TrajectoryPoint trans_pt = pt;
  trans_pt.set_x(pos.x());
  trans_pt.set_y(pos.y());
  trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
  return trans_pt;
}
math::Vector3d PoseTransform::ToEulerAngles(const math::Quaterniond &q) {
  math::Vector3d angles;
  // roll (x-axis rotation)
  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  angles.x() = st::fast_math::Atan2(sinr_cosp, cosr_cosp);
  // pitch (y-axis rotation)
  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1.0) {
    angles.y() = std::copysign(M_PI_2, sinp);  // use 90 degrees if out of range
  } else {
    angles.y() = std::asin(sinp);
  }
  // yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  angles.z() = st::fast_math::Atan2(siny_cosp, cosy_cosp);
  return angles;
}
}  // namespace planning
}  // namespace ad_byd
