#pragma once

#include <cmath>

#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/math/discretized_path.h"
#include "planner/planner_manager/planner_defs.h"
#include "planner/planner_manager/planner_flags.h"

#include "modules/cnoa_pnc/planning/proto/acc.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"

namespace st::planning {
inline constexpr double kMaxAllowedAccKappa = 1 / 5.2;  // R=5.2.
inline constexpr double kKappaEpsilon = 1E-6;

inline bool IsAccEnable(AccState state) { return state == AccState::ACC_ON; }
inline double GetLonSpeed(const PoseProto& pose) {
  return std::fabs(pose.vel_body().x()) < 0.5 ? 0.0 : pose.vel_body().x();
}
inline double GetLatSpeed(const PoseProto& pose) {
  return std::fabs(pose.vel_body().y()) < 0.1 ? 0.0 : pose.vel_body().y();
}
inline Vec2d LatPoint(Vec2d pos, Vec2d tangent, double width) {
  return pos + tangent.Perp() * width;
}

// Only used for low speed.
inline double CalcVehicleCurvature(double front_wheel_angle,
                                   double wheel_base) {
  return std::tan(front_wheel_angle) / wheel_base;
}

inline double MaybeKappaDeadZone(double mps) {
  static const PiecewiseLinearFunction<double, double> kMpsToKappaDeadZonePlf =
      {{30.0 / 3.6, 50.0 / 3.6}, {0.0005, 0.0002}};
  return FLAGS_planner_acc_use_dead_zone ? kMpsToKappaDeadZonePlf(mps) : 0.0;
}

struct UniCycleModelState {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double kappa = 0.0;
};

double ComputeSignedCurvature(double dx, double dy, double ddx, double ddy);
// Circle if fixed kappa
UniCycleModelState GenUniCycleModelState(const UniCycleModelState& state,
                                         double s, double kappa_decay,
                                         double init_heading,
                                         double abs_max_heading);
DiscretizedPath ExtendedCubicSplinePath(const std::vector<Vec2d>& ref_center_xy,
                                        const std::vector<double>& ref_s_vec,
                                        const Vec2d& heading, double step_s,
                                        double resample_step_s,
                                        double extended_length,
                                        double abs_max_curvature);
}  // namespace st::planning
