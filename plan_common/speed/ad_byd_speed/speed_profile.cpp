
#include "plan_common/math/linear_interpolation.h"
#include "plan_common/speed/ad_byd_speed/speed_profile.h"

namespace ad_byd {
namespace planning {
SpeedProfile::SpeedProfile(std::vector<SpeedPoint> &&points, bool re_evaluate)
    : speed_points_(std::move(points)) {
  if (re_evaluate) {
    ComputeSpeedProfile();
  }
}

void SpeedProfile::SetPoints(const std::vector<SpeedPoint> &points,
                             bool re_evaluate) {
  speed_points_ = points;
  if (re_evaluate) {
    ComputeSpeedProfile();
  }
}

double SpeedProfile::GetTimeLength() const {
  if (speed_points_.size() <= 1) {
    return 0.0;
  } else {
    return speed_points_.back().t - speed_points_.front().t;
  }
}

bool SpeedProfile::ComputeSpeedProfile() {
  const size_t points_size = speed_points_.size();
  if (points_size <= 1) {
    return false;
  }
  // recompute all points a,s except first point
  for (size_t i = 1; i < points_size; i++) {
    const auto &pre_point = speed_points_.at(i - 1);
    auto &point = speed_points_.at(i);
    if (point.v <= Constants::ZERO) {
      point.v = 0.0;
    }
    // compute a
    if (points_size - 1 == i) {
      point.a = pre_point.a;
    } else {
      const auto &next_point = speed_points_.at(i + 1);
      if (std::abs(next_point.t - point.t) <= Constants::ZERO ||
          (point.v < Constants::ZERO && point.a < -Constants::ZERO)) {
        point.a = pre_point.a;
      } else {
        point.a = (next_point.v - point.v) / (next_point.t - point.t);
      }
    }
    if (point.v <= Constants::ZERO && point.a <= Constants::ZERO) {
      point.a = std::min(Constants::SOFT_BRAKE, point.a);
    }
    point.a = math::Clamp(point.a, -8.0, 4.0);
    // compute s
    point.s =
        pre_point.s + 0.5 * (point.v + pre_point.v) * (point.t - pre_point.t);
  }
  //
  // recompute all points j except first point
  for (size_t i = 1; i < points_size; i++) {
    const auto &pre_point = speed_points_.at(i - 1);
    auto &point = speed_points_.at(i);
    // compute j
    if (points_size - 1 == i) {
      point.j = 0.0;
    } else {
      const auto &next_point = speed_points_.at(i + 1);
      if (std::abs(next_point.t - point.t) <= Constants::ZERO) {
        point.j = pre_point.j;
      } else {
        point.j = (next_point.a - point.a) / (next_point.t - point.t);
      }
    }
    point.j = math::Clamp(point.j, -5.0, 5.0);
  }
  // recompute first point a and j
  auto &first_point = speed_points_.at(0);
  const auto &second_point = speed_points_.at(1);
  if (second_point.t - first_point.t > Constants::ZERO) {
    first_point.a =
        (second_point.v - first_point.v) / (second_point.t - first_point.t);
    first_point.j =
        (second_point.a - first_point.a) / (second_point.t - first_point.t);
  }
  return true;
}

SpeedPoint SpeedProfile::GetSpeedPointAtTime(const double &t) const {
  size_t found_idx = 0;
  return GetSpeedPointAtTime(t, 0, false, found_idx);
}

SpeedPoint SpeedProfile::GetSpeedPointAtTime(const double &t,
                                             const size_t &start_idx,
                                             const bool sequential_search,
                                             size_t &found_idx) const {
  if (speed_points_.size() <= 1) {
    return {};
  }
  if (t < speed_points_.front().t - Constants::ZERO) {
    return {};
  }
  SpeedPoint result_point;
  auto comp = [](const SpeedPoint &point, const double &t) {
    return point.t < t;
  };
  auto iter = speed_points_.begin();
  if (!sequential_search) {
    iter = std::lower_bound(speed_points_.begin() + start_idx,
                            speed_points_.end(), t, comp);
    found_idx = std::distance(speed_points_.begin(), iter);
  } else {
    for (iter = speed_points_.begin() + start_idx; iter != speed_points_.end();
         iter++) {
      if (iter->t >= t) {
        found_idx = std::distance(speed_points_.begin(), iter);
        break;
      }
    }
  }
  if (iter == speed_points_.begin()) {
    result_point = speed_points_.front();
  } else if (iter != speed_points_.end()) {
    result_point.t = t;
    result_point.v =
        math::lerp((iter - 1)->v, (iter - 1)->t, iter->v, iter->t, t);
    result_point.a =
        math::lerp((iter - 1)->a, (iter - 1)->t, iter->a, iter->t, t);
    result_point.j =
        math::lerp((iter - 1)->j, (iter - 1)->t, iter->j, iter->t, t);
    result_point.s = (iter - 1)->s + 0.5 * ((iter - 1)->v + result_point.v) *
                                         (t - (iter - 1)->t);
  } else {
    const auto p_end = speed_points_.end() - 1;
    result_point = CalConstAccSpeedPoint(*p_end, p_end->a, t);
  }
  return result_point;
}

SpeedPoint SpeedProfile::GetSpeedPointAtS(const double &s,
                                          const size_t &start_idx,
                                          const bool sequential_search,
                                          size_t &found_idx) const {
  if (speed_points_.size() <= 1) {
    return {};
  }
  if (s < speed_points_.front().s - Constants::ZERO) {
    return {};
  }
  //
  SpeedPoint result_point;
  auto comp = [](const SpeedPoint &point, const double &s) {
    return point.s < s;
  };
  auto iter = speed_points_.begin();
  if (!sequential_search) {
    iter = std::lower_bound(speed_points_.begin() + start_idx,
                            speed_points_.end(), s, comp);
    found_idx = std::distance(speed_points_.begin(), iter);
  } else {
    for (iter = speed_points_.begin() + start_idx; iter != speed_points_.end();
         iter++) {
      if (iter->s >= s) {
        found_idx = std::distance(speed_points_.begin(), iter);
        break;
      }
    }
  }

  if (iter == speed_points_.begin()) {
    return speed_points_.front();
  }

  if (iter == speed_points_.end()) {
    const auto &last_point_t = speed_points_.back().t;
    const auto &last_point_s = speed_points_.back().s;
    const auto &last_point_v = speed_points_.back().v;
    if (last_point_v <= 0.01) {
      return {};
    }
    double delta_t = (s - last_point_s) / last_point_v;
    result_point.t = last_point_t + delta_t;
    result_point.s = s;
    result_point.v = last_point_v;
    result_point.a = 0.0;
    result_point.j = 0.0;
    return result_point;
  }

  const double &pre_s = (iter - 1)->s;
  const double &pre_v = (iter - 1)->v;
  const double &pre_a = (iter - 1)->a;
  double delta_t = 0;
  if (s < pre_s + Constants::ZERO) {
    delta_t = 0.0;
  } else if (std::abs(pre_a) <= Constants::ZERO) {
    delta_t = (s - pre_s) / std::max(Constants::ZERO, pre_v);
  } else {
    delta_t =
        (-pre_v + std::sqrt(pre_v * pre_v + 2 * pre_a * (s - pre_s))) / pre_a;
  }
  result_point.t = (iter - 1)->t + delta_t;
  result_point.s = s;
  result_point.v = pre_v + pre_a * delta_t;
  result_point.a = math::lerp((iter - 1)->a, (iter - 1)->t, iter->a, iter->t,
                              result_point.t, true);
  result_point.j = math::lerp((iter - 1)->j, (iter - 1)->t, iter->j, iter->t,
                              result_point.t, true);
  return result_point;
}

std::vector<SpeedPoint> SpeedProfile::SampleSpeedPointsByS(
    const double &start, const double &length, const double &interval) const {
  std::vector<SpeedPoint> speed_points;
  if (speed_points_.size() <= 1) {
    return speed_points;
  }
  if (length <= Constants::ZERO || interval <= Constants::ZERO ||
      start + length < speed_points_.front().s) {
    return speed_points;
  }
  //
  SpeedPoint stop_point;
  const auto will_stop = ComputeStopPoint(stop_point);
  const size_t points_size = std::ceil(length / interval + 1);
  speed_points.reserve(points_size);
  double current_s = start;
  size_t start_index = 0;
  bool sequential_seach = false;
  while (current_s <= start + length) {
    size_t found_idx = 0;
    SpeedPoint speed_point =
        GetSpeedPointAtS(current_s, start_index, sequential_seach, found_idx);
    if (speed_point.t > -Constants::ZERO) {
      speed_points.emplace_back(speed_point);
    }
    start_index = found_idx;
    current_s += interval;
    sequential_seach = true;
    //
    if (will_stop && speed_points_.front().v > Constants::ZERO &&
        (current_s > speed_points_.back().s || speed_point.t > stop_point.t)) {
      if (!speed_points.empty()) {
        stop_point.t = std::fmax(speed_points.back().t + 0.1, stop_point.t);
        stop_point.s =
            std::fmax(speed_points.back().s + Constants::ZERO, stop_point.s);
      }
      speed_points.emplace_back(stop_point);
      break;
    }
    if (!speed_points.empty() &&
        speed_points.back().t - speed_points.front().t >=
            GetTimeLength() + 1.0) {
      break;
    }
  }
  return speed_points;
}

bool SpeedProfile::ComputeStopPoint(SpeedPoint &stop_point) const {
  if (speed_points_.size() < 2) {
    return false;
  }
  //
  for (const auto &point : speed_points_) {
    if (point.v <= Constants::ZERO) {
      stop_point = point;
      return true;
    }
  }
  //
  const auto &end_point = speed_points_.back();
  const auto &end_v = end_point.v;
  const auto &end_a = end_point.a;
  if (end_a > -Constants::ZERO) {
    return false;
  }
  const double stop_delta_t = std::abs(end_v / end_a);
  stop_point =
      CalConstAccSpeedPoint(end_point, end_point.a, end_point.t + stop_delta_t);
  stop_point.v = 0.0;
  return true;
}

SpeedPoint SpeedProfile::CalConstAccSpeedPoint(const SpeedPoint &start,
                                               const double a,
                                               const double t) const {
  if (t < start.t) {
    return {};
  }
  SpeedPoint result_point;
  const double delta_t = t - start.t;
  if (a >= -Constants::ZERO) {
    result_point.t = t;
    result_point.a = a;
    result_point.v = start.v + a * delta_t;
    result_point.s = start.s + 0.5 * (start.v + result_point.v) * delta_t;
    return result_point;
  }
  //
  const double stop_t = start.v / std::abs(a) + start.t;
  if (t <= stop_t - Constants::ZERO) {
    result_point.t = t;
    result_point.a = a;
    result_point.v = start.v + a * delta_t;
    result_point.s = start.s + 0.5 * (start.v + result_point.v) * delta_t;
  } else {
    result_point.t = t;
    result_point.a = a;
    result_point.v = 0.0;
    result_point.s = start.s + 0.5 * start.v * (stop_t - start.t);
  }
  return result_point;
}

SpeedPoint SpeedProfile::CalConstJerkSpeedPoint(const SpeedPoint &start,
                                                const double jerk,
                                                const double t) const {
  //
  if (t < start.t) {
    return {};
  }
  if (std::abs(jerk) <= Constants::ZERO) {
    return CalConstAccSpeedPoint(start, start.a, t);
  }
  //
  SpeedPoint result_point;
  const double discriminant = start.a * start.a - 2 * jerk * start.v;

  auto cal_const_jerk = [](const SpeedPoint &start, const double &jerk,
                           const double &t) {
    SpeedPoint result_point;
    const double delta_t = t - start.t;
    result_point.t = t;
    result_point.j = jerk;
    result_point.a = start.a + jerk * delta_t;
    result_point.v =
        start.v + start.a * delta_t + 0.5 * jerk * delta_t * delta_t;
    result_point.s = start.s + start.v * delta_t +
                     0.5 * start.a * delta_t * delta_t +
                     jerk * delta_t * delta_t * delta_t / 6.0;
    return result_point;
  };

  if (jerk > 0 && (start.a >= 0 || (start.a < 0 && discriminant <= 0))) {
    result_point = cal_const_jerk(start, jerk, t);
  } else {
    const double stop_delta_t = -(start.a + std::sqrt(discriminant)) / jerk;
    if (t < start.t + stop_delta_t - Constants::ZERO) {
      result_point = cal_const_jerk(start, jerk, t);
    } else {
      result_point = cal_const_jerk(start, jerk, start.t + stop_delta_t);
      result_point.t = t;
      result_point.j = 0.0;
      result_point.v = 0.0;
    }
  }
  return result_point;
}
}  // namespace planning
}  // namespace ad_byd
