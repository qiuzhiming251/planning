#include "go_point_pursuit.h"
#include "plan_common/math/util.h"
#include "plan_common/util/path_util.h"
namespace st::planning {
constexpr double kMinVirtualTimeForPP = 0.7;
constexpr double kMaxVirtualTimeForPP = 3.0;
constexpr double kFastRtcVirtualTime = 0.2;
constexpr double kMaxLateralJerk = 1.2;
constexpr double kMaxRtcLateralJerk = 4.0;
constexpr double kMinLatConstrainVel = 3.0;
constexpr double kMaxFrictionCircleAcc = 2.0;
constexpr double kMaxFrictionCircleAcc2 = 4.0;
constexpr double kMaxVel = 8.0;

void RfPurePursuit::AddFrictionCircleConstraint(double cur_v,
                                                double target_kappa,
                                                double* target_acc) {
  if (target_acc == nullptr) {
    return;
  }
  if (*target_acc > 0.0) {
    double lat_acc = (cur_v * cur_v) * std::fabs(target_kappa);
    double max_target_acc =
        std::sqrt(std::max(kMaxFrictionCircleAcc2 - lat_acc * lat_acc, 1.0e-6));
    *target_acc = std::min(*target_acc, max_target_acc);
  }
  return;
}

double RfPurePursuit::CalcTargetDkappa(const VehicleState& av_state,
                                       double target_kappa,
                                       double virtual_time) {
  double cur_kappa = av_state.kappa;
  bool need_fast_rtc = ((cur_kappa < -1.0e-3 && target_kappa > 1.0e-3) ||
                        (cur_kappa > 1.0e-3 && target_kappa < -1.0e-3));
  double target_dkappa = virtual_time > std::numeric_limits<double>::epsilon()
                             ? (target_kappa - cur_kappa) / virtual_time
                             : 0.0f;
  if (need_fast_rtc) {
    target_dkappa = -cur_kappa / kFastRtcVirtualTime;
  }

  return target_dkappa;
}

std::pair<PathPoint, bool> RfPurePursuit::CalcProjPoint(
    const DiscretizedPath& reference, const PathPoint& p, int init_index) {
  auto sl = reference.XYToSL(Vec2d(p.x(), p.y()));
  PathPoint path_point = reference.Evaluate(sl.s);
  bool proj_succeed = sl.s < reference.length();
  return {path_point, proj_succeed};
}

PurePursuitInfo RfPurePursuit::CalcPathTrackingAction(
    const DiscretizedPath& reference, const VehicleState& ego_state,
    double target_point_s, int init_index, double lat_acc_max,
    std::string* debug) {
  PurePursuitInfo pp_info;
  const double& x_ego = ego_state.x;
  const double& y_ego = ego_state.y;
  const double& theta_ego = ego_state.theta;
  const double& v_ego = ego_state.v;
  const double& kappa_ego = ego_state.kappa;
  const double& a_ego = ego_state.a;
  const double dot_theta_ego = v_ego * kappa_ego;
  PathPoint rear_point;
  rear_point.set_x(x_ego);
  rear_point.set_y(y_ego);
  auto [proj_point, succ_projected] =
      CalcProjPoint(reference, rear_point, init_index);
  // *debug +=
  // absl::StrCat("proj_point.x",proj_point.x(),"proj_point.y",proj_point.y());
  constexpr double min_l_ah_s = 5.0;
  constexpr double alpha = 0.4;
  constexpr double T_0 = 0.1;
  constexpr double target_a_val = 2.0;

  constexpr double theta_T = M_PI / 2.0;
  constexpr double w1 = 1.0, w2 = 1.0, w3 = 1.0;

  PathPoint target_point = reference.Evaluate(target_point_s);
  if (succ_projected) {
    double theta_r = proj_point.theta();
    double sin_theta_r = sin(theta_r);
    double cos_theta_r = cos(theta_r);
    double kappa_r = proj_point.kappa();
    double dTheta = theta_ego - theta_r;
    double sin_theta_ego = sin(theta_ego);
    double cos_theta_ego = cos(theta_ego);
    double sin_dTheta = sin(dTheta);
    double cos_dTheta = cos(dTheta);
    double T = T_0 + alpha * std::max(cos_dTheta, 0.0);
    double x_r = proj_point.x();
    double y_r = proj_point.y();
    double s_r = proj_point.s();
    double dx = x_ego - x_r;
    double dy = y_ego - y_r;
    double l = -dx * sin_theta_r + dy * cos_theta_r;
    double T_prod_N =
        -sin_theta_r * cos_theta_ego + cos_theta_r * sin_theta_ego;
    double look_ahead_ds = std::max(T * v_ego + min_l_ah_s, std::abs(l) * 0.0);
    double look_ahead_s = look_ahead_ds + s_r;

    // Evaluation of pp_info:

    // If we are behind the target point, our look-ahead-point should not
    // further than the target point
    if (s_r <= target_point_s) {
      look_ahead_ds =
          std::min(0.5 * (target_point_s - s_r), look_ahead_ds) + 4.0;
      look_ahead_s = s_r + look_ahead_ds;
    }

    pp_info.succ_projected = true;
    pp_info.pp_look_ahead_ds = look_ahead_ds;
    pp_info.pp_look_ahead_s = look_ahead_s;
    pp_info.av_proj_s_on_path = s_r;

    double target_point_x = 0.0;
    double target_point_y = 0.0;

    if (T <= 100.0) {
      PathPoint pt = reference.Evaluate(look_ahead_s);
      target_point_x = pt.x();
      target_point_y = pt.y();

      // debug_info.look_ahead_point = TwoDPoint{target_point_x,
      // target_point_y};
      double dx_target_ego = target_point_x - x_ego;
      double dy_target_ego = target_point_y - y_ego;
      double L =
          sqrt(dx_target_ego * dx_target_ego + dy_target_ego * dy_target_ego);
      double H = -dx_target_ego * sin_theta_ego + dy_target_ego * cos_theta_ego;
      // debug_info.look_ahead_distance = L;
      double target_kappa = 2.0 * H / (L * L);
      double target_R = 1.0 / std::max(std::abs(target_kappa), 1e-04);
      double arg_length = 2.0 * asin(std::min(std::abs(H) / L, 1.0)) * target_R;
      arg_length = std::max(L, arg_length);
      double pp_virtual_time = 0.5 * arg_length / std::max(v_ego, 1.0);
      pp_virtual_time = std::max(kMinVirtualTimeForPP, pp_virtual_time);
      pp_virtual_time = std::min(pp_virtual_time, kMaxVirtualTimeForPP);
      double curvature_speed_limit = std::min(
          sqrt(lat_acc_max / std::max(1e-4, std::abs(target_kappa))), kMaxVel);
      double target_a = (curvature_speed_limit - v_ego) / pp_virtual_time;
      target_a = std::max(-3.0, target_a);
      target_a = std::min(1.0, target_a);
      target_kappa = std::max(-0.16, target_kappa);
      target_kappa = std::min(0.16, target_kappa);
      pp_info.pp_target_kappa = target_kappa;
      pp_info.pp_target_arg_length = arg_length;
      pp_info.pp_target_curv_speed_limit = curvature_speed_limit;
      pp_info.pp_target_acc = target_a;
      AddFrictionCircleConstraint(v_ego, target_kappa, &pp_info.pp_target_acc);
      pp_info.pp_dkappa =
          CalcTargetDkappa(ego_state, target_kappa, pp_virtual_time);
      pp_info.pp_jerk = 2.0 * (target_a - a_ego) / pp_virtual_time;
      pp_info.pp_virtual_time = pp_virtual_time;
    }
  }

  return pp_info;
}

}  // namespace st::planning