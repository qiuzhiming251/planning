//
// Created by xxx on 10/26/21.
//
#include "plan_common/speed/ad_byd_speed/constant_acc_builder.h"
#include "plan_common/speed/ad_byd_speed/speed_profile_utils.h"
#include "plan_common/math/linear_interpolation.h"
namespace ad_byd {
namespace planning {

using speed_profile_utils::CalConstAccSpeedPoint;
using speed_profile_utils::CalConstJerkSpeedPoint;

double ConstantAccBuilder::ComputeJerk(const SpeedPoint &pt,
                                       const double &target_a,
                                       const double &t_interval) const {
  const double delta_a = target_a - pt.a;
  double jerk = 6.0;
  if (delta_a >= 0.0) {
    if (pt.a < -0.5) {
      jerk = math::Clamp(-pt.a, 2.0, 6.0);
    } else if (pt.a < 0.0) {
      jerk = math::Clamp(-pt.a / 0.25, 1.0, 2.0);
    } else {
      jerk = std::abs(pt.a) + 0.5;
    }
  } else {
    if (pt.a > 0.0) {
      jerk = math::Clamp(-pt.a / 0.5, -2.0, -6.0);
    } else {
      double a_delta = math::lerp(1.0, 30.0 / 3.6, 0.5, 50.0 / 3.6, pt.v, true);
      jerk = std::min(-1.0, -(std::abs(pt.a) + a_delta));
    }
  }
  return jerk;
}

SpeedProfilePtr ConstantAccBuilder::Build(const SpeedPoint &start,
                                          const SpeedPoint &end,
                                          const double &time_length) {
  //
  if (end.t <= start.t || end.v < 0 || time_length <= Constants::ZERO) {
    return nullptr;
  }
  //
  const double delta_v = end.v - start.v;
  const double delta_a = end.a - start.a;
  const double target_v = end.v;
  const double target_a = end.a;
  // input not right
  if (delta_v * target_a < 0) {
    return nullptr;
  }
  //
  std::vector<SpeedPoint> speed_points;
  const size_t points_size = std::ceil(time_length / interval_ + 1);
  if (points_size < 2) {
    return nullptr;
  }
  speed_points.reserve(points_size);
  speed_points.emplace_back(start);
  double t = start.t + interval_;
  auto jerk = ComputeJerk(start, target_a, interval_);
  auto speed_point = CalConstJerkSpeedPoint(start, jerk, t);
  bool reach_target_v =
      (std::abs(delta_v) <= Constants::ZERO) ||
      (delta_v > 0 && end.v - speed_point.v <= Constants::ZERO) ||
      (delta_v < 0 && end.v - speed_point.v >= -Constants::ZERO);
  bool reach_target_a =
      (std::abs(delta_a) <= Constants::ZERO) ||
      (delta_a > 0 && end.a - speed_point.a < Constants::ZERO) ||
      (delta_a < 0 && end.a - speed_point.a >= -Constants::ZERO);
  bool get_target_v = false;
  bool get_target_a = false;
  while (t <= start.t + time_length + Constants::ZERO) {
    // if not reach target v and target v
    if (!reach_target_v && !reach_target_a) {
      speed_points.emplace_back(speed_point);
      //
      if (std::fabs(t - start.t - time_length) < Constants::ZERO) {
        break;
      }
      //
      jerk = ComputeJerk(speed_points.back(), target_a, interval_);
      t = std::min(t + interval_, start.t + time_length);
      speed_point = CalConstJerkSpeedPoint(speed_points.back(), jerk, t);
      reach_target_v =
          (delta_v > 0 && end.v - speed_point.v <= Constants::ZERO) ||
          (delta_v < 0 && end.v - speed_point.v >= -Constants::ZERO);
      reach_target_a =
          ((delta_a > 0 && end.a - speed_point.a < Constants::ZERO) ||
           (delta_a < 0 && end.a - speed_point.a >= -Constants::ZERO));
      continue;
    }
    // if reach target a
    if (!reach_target_v && reach_target_a) {
      if (!get_target_a) {
        const double delta_t_a_turn =
            std::abs((target_a - speed_points.back().a) / jerk);
        // a turning point
        SpeedPoint a_turning_point = CalConstJerkSpeedPoint(
            speed_points.back(), jerk, speed_points.back().t + delta_t_a_turn);
        a_turning_point.a = target_a;
        if (delta_t_a_turn > interval_) {
          a_turning_point.t = speed_points.back().t + interval_;
        }
        t = a_turning_point.t;
        speed_points.emplace_back(a_turning_point);
        get_target_a = true;
      } else {
        speed_points.emplace_back(speed_point);
      }
      //
      if (std::fabs(t - start.t - time_length) < Constants::ZERO) {
        break;
      }
      // predict next point
      t = std::min(t + interval_, start.t + time_length);
      speed_point = CalConstAccSpeedPoint(speed_points.back(), target_a, t);
      reach_target_v =
          (delta_v > 0 && end.v - speed_point.v <= Constants::ZERO) ||
          (delta_v < 0 && end.v - speed_point.v >= -Constants::ZERO);
      continue;
    }
    //
    if (reach_target_v) {
      if (!get_target_v) {
        if (std::abs(speed_points.back().v - target_v) <= Constants::ZERO) {
          speed_point = CalConstAccSpeedPoint(speed_points.back(), target_a, t);
          speed_points.emplace_back(speed_point);
        } else {
          if (std::abs(target_a) <= Constants::ZERO) {
            speed_point = CalConstAccSpeedPoint(speed_points.back(), 0.0, t);
            speed_points.emplace_back(speed_point);
          } else {
            const double delta_t_v_turn =
                std::abs((target_v - speed_points.back().v) / target_a);
            SpeedPoint v_turning_point =
                CalConstAccSpeedPoint(speed_points.back(), target_a,
                                      speed_points.back().t + delta_t_v_turn);
            v_turning_point.v = target_v;
            if (delta_t_v_turn > interval_) {
              v_turning_point.t = speed_points.back().t + interval_;
            }
            t = v_turning_point.t;
            speed_points.emplace_back(v_turning_point);
          }
        }
        get_target_v = true;
      } else {
        speed_points.emplace_back(speed_point);
      }
      //
      if (std::fabs(t - start.t - time_length) < Constants::ZERO) {
        break;
      }
      // predict next point
      t = std::min(t + interval_, start.t + time_length);
      speed_point = CalConstAccSpeedPoint(speed_points.back(), 0.0, t);
      if (speed_point.v < Constants::ZERO) {
        speed_point.a = speed_points.back().a;
      }
      continue;
    }
  }
  return std::make_shared<SpeedProfile>(std::move(speed_points));
}

}  // namespace planning
}  // namespace ad_byd
