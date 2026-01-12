//
// Created by xxx on 8/6/21.
//

#include "localization.h"
#include "plan_common/log.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/fast_math.h"
namespace ad_byd::planning {
Localization::Localization(const LocalizationInfo &info) {
  timestamp_ = info.timestamp;
  frame_id_ = info.frame_id;
  type_ = info.coordinate_type;
  position_ =
      math::Vector3d(info.position.x(), info.position.y(), info.position.z());
  q_ = math::Quaterniond(info.q.w, info.q.x, info.q.y, info.q.z).normalized();
  v_ = math::Vector3d(info.v.x(), info.v.y(), info.v.z());
  a_ = math::Vector3d(info.a.x(), info.a.y(), info.a.z());
  angular_v_ = math::Vector3d(info.angular_v.x(), info.angular_v.y(),
                              info.angular_v.z());
  position_cov_ = info.position_cov;
  quaternion_cov_ = info.quaternion_cov;
}

Localization::Localization(const math::Vec2d &position, const double &yaw) {
  position_ = math::Vector3d(position.x(), position.y(), 0.0);
  math::AngleAxisd rollAngle(math::AngleAxisd(0.0, math::Vector3d::UnitX()));
  math::AngleAxisd pitchAngle(math::AngleAxisd(0.0, math::Vector3d::UnitY()));
  math::AngleAxisd yawAngle(math::AngleAxisd(yaw, math::Vector3d::UnitZ()));
  q_ = rollAngle * pitchAngle * yawAngle;
}

TrajectoryPoint Localization::WorldToVehicle(
    const TrajectoryPoint &world_p) const {
  math::Vector3d p0(world_p.x(), world_p.y(), position_.z());
  math::Vector3d p1 = q_.inverse() * (p0 - position_);
  double angles_diff = ToEulerAngles(q_).z();
  TrajectoryPoint trj_point = world_p;
  trj_point.set_x(p1.x());
  trj_point.set_y(p1.y());
  trj_point.theta = math::NormalizeAngle(world_p.theta - angles_diff);
  return trj_point;
}

std::shared_ptr<Localization> Localization::WorldToVehicle(
    const std::shared_ptr<Localization> &world_p) const {
  if (!world_p) {
    LOG_ERROR << "LocalizationPtr is null, WorldToVehicle fail!";
    return nullptr;
  }
  LocalizationPtr vehicle_p = std::make_shared<Localization>(*world_p);
  vehicle_p->set_position(q_.inverse() * (world_p->position() - position_));
  vehicle_p->set_quaternion(q_.inverse() * world_p->quaternion());
  vehicle_p->set_v(q_.inverse() * world_p->v());
  vehicle_p->set_a(q_.inverse() * world_p->a());
  return vehicle_p;
}

math::Vec2d Localization::WorldToVehicle(const math::Vec2d &world_p) const {
  const math::Vector3d p0(world_p.x(), world_p.y(), position_.z());
  const math::Vector3d p1 = q_.inverse() * (p0 - position_);
  return {p1.x(), p1.y()};
}

TrajectoryPoint Localization::VehicleToWorld(
    const TrajectoryPoint &vehicle_p) const {
  const math::Vector3d p0(vehicle_p.x(), vehicle_p.y(), position_.z());
  const math::Vector3d p1 = q_ * p0 + position_;
  const double angles_diff = ToEulerAngles(q_).z();
  TrajectoryPoint trj_point = vehicle_p;
  trj_point.set_x(p1.x());
  trj_point.set_y(p1.y());
  trj_point.theta = math::NormalizeAngle(vehicle_p.theta + angles_diff);
  return trj_point;
}

std::shared_ptr<Localization> Localization::VehicleToWorld(
    const std::shared_ptr<Localization> &vehicle_p) const {
  if (!vehicle_p) {
    LOG_ERROR << "LocalizationPtr is null, VehicleToWorld fail!";
    return nullptr;
  }
  LocalizationPtr world_p = std::make_shared<Localization>(*vehicle_p);
  world_p->set_position(q_ * vehicle_p->position() + position_);
  world_p->set_quaternion(q_ * vehicle_p->quaternion());
  world_p->set_v(q_ * vehicle_p->v());
  world_p->set_a(q_ * vehicle_p->a());
  return world_p;
}

math::Vec2d Localization::VehicleToWorld(const math::Vec2d &vehicle_p) const {
  const math::Vector3d p0(vehicle_p.x(), vehicle_p.y(), position_.z());
  const math::Vector3d p1 = q_ * p0 + position_;
  return {p1.x(), p1.y()};
}

math::Vector3d Localization::ToEulerAngles(const math::Quaterniond &q) {
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
}  // namespace ad_byd::planning
