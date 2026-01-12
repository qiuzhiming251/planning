//
// Created by xxx on 20/5/22.
//

#include "localization.h"
#include "plan_common/log.h"
#include "transformer.h"
#include "plan_common/math/math_utils.h"
namespace ad_byd::planning {
Transformer::Transformer(const TransformInfo &info) {
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
Transformer::Transformer(const math::Vec2d &delta_pos,
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

Point2d Transformer::Trans(const Point2d &pt) const {
  //  left multiplication
  const math::Vector3d p = tf_.inverse() * math::Vector3d(pt.x(), pt.y(), 0.0);
  return {p.x(), p.y()};
}
std::vector<Point2d> Transformer::Trans(const std::vector<Point2d> &pts) const {
  //  left multiplication
  const math::Isometry3d tf_inverse = tf_.inverse();
  std::vector<Point2d> trans_pts;
  for (const auto &pt : pts) {
    const math::Vector3d p1 = tf_inverse * math::Vector3d(pt.x(), pt.y(), 0.0);
    trans_pts.emplace_back(p1.x(), p1.y());
  }
  return trans_pts;
}
PathPoint Transformer::Trans(const PathPoint &pt) const {
  // left multiplication
  const math::Isometry3d tf_inverse = tf_.inverse();
  const math::Vector3d pos = tf_inverse * math::Vector3d(pt.x(), pt.y(), 0.0);
  const auto delta_yaw =
      Localization::ToEulerAngles(math::Quaterniond(tf_inverse.rotation())).z();
  PathPoint trans_pt = pt;
  trans_pt.set_x(pos.x());
  trans_pt.set_y(pos.y());
  trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
  return trans_pt;
}
std::vector<PathPoint> Transformer::Trans(
    const std::vector<PathPoint> &pts) const {
  const math::Isometry3d tf_inverse = tf_.inverse();
  const auto delta_yaw =
      Localization::ToEulerAngles(math::Quaterniond(tf_inverse.rotation())).z();
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
TrajectoryPoint Transformer::Trans(const TrajectoryPoint &pt) const {
  // left multiplication
  const math::Isometry3d tf_inverse = tf_.inverse();
  const math::Vector3d pos = tf_inverse * math::Vector3d(pt.x(), pt.y(), 0.0);
  const auto delta_yaw =
      Localization::ToEulerAngles(math::Quaterniond(tf_inverse.rotation())).z();
  TrajectoryPoint trans_pt = pt;
  trans_pt.set_x(pos.x());
  trans_pt.set_y(pos.y());
  trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
  return trans_pt;
}

Point2d Transformer::TransInverse(const Point2d &pt) const {
  const math::Vector3d p = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
  return {p.x(), p.y()};
}
std::vector<Point2d> Transformer::TransInverse(
    const std::vector<Point2d> &pts) const {
  std::vector<Point2d> trans_pts;
  for (const auto &pt : pts) {
    const math::Vector3d p1 = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
    trans_pts.emplace_back(p1.x(), p1.y());
  }
  return trans_pts;
}
PathPoint Transformer::TransInverse(const PathPoint &pt) const {
  const math::Vector3d pos = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
  const auto delta_yaw =
      Localization::ToEulerAngles(math::Quaterniond(tf_.rotation())).z();
  PathPoint trans_pt = pt;
  trans_pt.set_x(pos.x());
  trans_pt.set_y(pos.y());
  trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
  return trans_pt;
}
std::vector<PathPoint> Transformer::TransInverse(
    const std::vector<PathPoint> &pts) const {
  // left multiplication
  const auto delta_yaw =
      Localization::ToEulerAngles(math::Quaterniond(tf_.rotation())).z();
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
TrajectoryPoint Transformer::TransInverse(const TrajectoryPoint &pt) const {
  const math::Vector3d pos = tf_ * math::Vector3d(pt.x(), pt.y(), 0.0);
  const auto delta_yaw =
      Localization::ToEulerAngles(math::Quaterniond(tf_.rotation())).z();
  TrajectoryPoint trans_pt = pt;
  trans_pt.set_x(pos.x());
  trans_pt.set_y(pos.y());
  trans_pt.theta = math::NormalizeAngle(pt.theta + delta_yaw);
  return trans_pt;
}
}  // namespace ad_byd::planning
