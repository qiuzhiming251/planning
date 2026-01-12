

#ifndef ONBOARD_PLANNER_TRAJECTORY_POINT_H_
#define ONBOARD_PLANNER_TRAJECTORY_POINT_H_

#include <algorithm>
#include <string>

#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "second_order_trajectory_point.h"

namespace st {
namespace planning {

class TrajectoryPoint : public SecondOrderTrajectoryPoint {
 public:
  TrajectoryPoint() = default;
  explicit TrajectoryPoint(const TrajectoryPointProto& proto) {
    FromProto(proto);
  }
  explicit TrajectoryPoint(const ApolloTrajectoryPointProto& proto) {
    FromProto(proto);
  }

  // Spatial quantities (in addition to those in SecondOrderTrajectoryPoint).
  // Arclength-parameterized curvature rate, dkappa/ds.
  static double ComputeLambda(double v, double psi) {
    return psi / (v < 0.0 ? std::min(v, -kSpeedEps) : std::max(v, kSpeedEps));
  }
  // Arclength-parameterized lambda rate, dlambda/ds.
  static double ComputeMu(double v, double a, double lambda, double chi) {
    return (chi - a * lambda) / std::max(Sqr(v), Sqr(kSpeedEps));
  }
  double lambda() const { return ComputeLambda(v_, psi_); }
  double mu() const { return ComputeMu(v_, a_, lambda(), chi_); }

  // Temporal quantities (in addition to those in SecondOrderTrajectoryPoint).
  double j() const { return j_; }
  void set_j(double j) { j_ = j; }

  // Spatial-temporal quantities.
  double psi() const { return psi_; }
  double chi() const { return chi_; }
  void set_psi(double psi) { psi_ = psi; }
  void set_chi(double chi) { chi_ = chi; }

  // Serialization to proto.
  void FromProto(const TrajectoryPointProto& proto);
  void ToProto(TrajectoryPointProto* proto) const;

  void FromProto(const ApolloTrajectoryPointProto& proto);
  void ToProto(ApolloTrajectoryPointProto* proto) const;

  void ToProto(PoseTrajectoryPointProto* proto) const;

  std::string DebugString() const;

 private:
  static constexpr double kSpeedEps = 0.001;
  // Vehicle kinematics reference:

  // Temporal quantities (in addition to those in SecondOrderTrajectoryPoint).
  double j_ = 0.0;  // Jerk (m/s^3).

  // Spatial-temporal quantites.
  double psi_ = 0.0;  // Curvature rate, dkappa/dt.
  double chi_ = 0.0;  // Curvature second rate, dpsi/dt.
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_TRAJECTORY_POINT_H_
