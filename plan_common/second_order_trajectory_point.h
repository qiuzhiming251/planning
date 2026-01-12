

#ifndef ONBOARD_PLANNER_SECOND_ORDER_TRAJECTORY_POINT_H_
#define ONBOARD_PLANNER_SECOND_ORDER_TRAJECTORY_POINT_H_

#include <string>
#include <cereal/cereal.hpp>

#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st {
namespace planning {

// A class for points on a trajectory (both temporal and spatial), with up to
// second order derivatives. For higher derivatives, see class TrajectoryPoint.
class SecondOrderTrajectoryPoint {
 public:
  SecondOrderTrajectoryPoint() = default;
  explicit SecondOrderTrajectoryPoint(const TrajectoryPointProto& proto) {
    FromProto(proto);
  }

  // Field access.
  // Spatial quantities.
  const Vec2d& pos() const { return pos_; }

  double s() const { return s_; }
  double theta() const { return theta_; }
  double kappa() const { return kappa_; }
  double steer_angle() const { return steer_angle_; }
  void set_pos(const Vec2d& pos) { pos_ = pos; }
  void set_s(double s) { s_ = s; }
  void set_theta(double theta) { theta_ = theta; }
  void set_kappa(double kappa) { kappa_ = kappa; }
  void set_steer_angle(double steer_angle) { steer_angle_ = steer_angle; }

  // Temporal quantities.
  double t() const { return t_; }
  double v() const { return v_; }
  double a() const { return a_; }
  void set_t(double t) { t_ = t; }
  void set_v(double v) { v_ = v; }
  void set_a(double a) { a_ = a; }

  void FromProto(const SecondOrderTrajectoryPointProto& proto);
  void ToProto(SecondOrderTrajectoryPointProto* proto) const;

  // Serialization to proto. Note this class does not use all fields in
  // TrajectoryPointProto. See TrajectoryPoint class also.
  // Also note these methods are intentionally not virtual. Prefer using this
  // class and its subclasses explicitly without polymorphism.
  void FromProto(const TrajectoryPointProto& proto);
  void ToProto(TrajectoryPointProto* proto) const;

  void ToProto(PoseTrajectoryPointProto* proto) const;

  std::string DebugString() const;

 protected:
  // Vehicle kinematics reference:
  // Spatial quantities.
  Vec2d pos_ = Vec2d::Zero();  // Position (local coordinates).
  double s_ = 0.0;             // Arclength. Relative to time 0.
  double theta_ = 0.0;         // Heading (radians).
  double kappa_ = 0.0;         // Curvature.
  double steer_angle_ = 0.0;   // steer angle, rad.

  // Temporal quantities.
  double t_ = 0.0;  // Time. Relative to now (now is 0).
  double v_ = 0.0;  // Speed (m/s).
  double a_ = 0.0;  // Scalar acceleration (m/s^2).

  template <typename Archive>
  friend void serialize(
      Archive& ar, SecondOrderTrajectoryPoint& secondorder_trajectory_point);
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SECOND_ORDER_TRAJECTORY_POINT_H_
