

#include "decider/initializer/ref_speed_table.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <ostream>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/log.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/util.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "predictor/predicted_trajectory.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/gradient_points_smoother.h"
#include "plan_common/log_data.h"

namespace st::planning {

namespace {
constexpr int kSampleTimeSteps = 9;
constexpr double kPredictionTimeSampleStep = 0.1;  // s
constexpr std::array<double, kSampleTimeSteps> kTimeSamples{
    0.0, 0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9, 7.9};

constexpr double kSpanDiscretizationStep = 0.5;  // m

constexpr double kSafeLeadingDist = 4.5;              // m
constexpr double kSafeLeadingTime = 1.0;              // s
constexpr double kMaxComfortableDeceleration = -1.0;  // m/s2

}  // namespace

// #################### RefSpeedVec ####################
void RefSpeedLimitByCurvature(const std::vector<StationCenter>& center_points,
                              const std::string& task_id,
                              std::vector<double>* speed_limit) {
  std::vector<double> station_points_s;
  std::vector<double> station_thetas;
  station_points_s.reserve(center_points.size());
  station_thetas.reserve(center_points.size());
  for (const auto& station : center_points) {
    station_points_s.push_back(station.accum_s);
    station_thetas.push_back(station.tangent.FastAngle());
  }

  constexpr double curvature_power = 0.74;
  constexpr double curvature_numerator = 0.47;
  constexpr double curvature_bias1 = 0.008;
  constexpr double curvature_bias2 = 0.1;

  constexpr double kSmallKappaThres = 0.003;             // m^-1.
  constexpr double kIgnoreKappaSpeedLimitThres = 0.002;  // m^-1.
  constexpr double kSmallEps = 1e-9;

  const double k_power = std::pow(kSmallKappaThres, curvature_power);
  const double k_power_plus_bias1 = k_power + curvature_bias1;
  const double small_kappa_speed_limit_sqr =
      Sqr(curvature_numerator / k_power_plus_bias1 + curvature_bias2);
  const double small_kappa_lat_acc_derivative = std::max(
      0.0, -(2.0 * curvature_numerator * curvature_power * k_power *
             (curvature_numerator + curvature_bias2 * k_power_plus_bias1)) /
                   Cube(k_power_plus_bias1) +
               small_kappa_speed_limit_sqr);
  const double intercept =
      std::max(0.0, -kSmallKappaThres * small_kappa_lat_acc_derivative +
                        small_kappa_speed_limit_sqr * kSmallKappaThres);
  constexpr int kHalfAverageScope = 5;
  if (center_points.size() <= 2 * kHalfAverageScope) return;
  std::vector<double> smoothed_kappa(center_points.size());
  for (int i = kHalfAverageScope;
       i < center_points.size() - 1 - kHalfAverageScope; ++i) {
    const double kappa_fd =
        std::abs(NormalizeAngle(station_thetas[i + kHalfAverageScope] -
                                station_thetas[i - kHalfAverageScope]) /
                 (station_points_s[i + kHalfAverageScope] -
                  station_points_s[i - kHalfAverageScope]));
    smoothed_kappa[i] = kappa_fd;
    double v_kappa_fd = 0.0;
    if (kappa_fd < kIgnoreKappaSpeedLimitThres) {
      continue;
    } else if (kappa_fd >= kSmallKappaThres) {
      v_kappa_fd = curvature_numerator /
                       (std::pow(kappa_fd, curvature_power) + curvature_bias1) +
                   curvature_bias2;
    } else {
      const double lat_acc =
          kappa_fd * small_kappa_lat_acc_derivative + intercept;
      v_kappa_fd = std::sqrt(lat_acc / (kappa_fd + kSmallEps));
    }
    speed_limit->at(i) = std::min(speed_limit->at(i), v_kappa_fd);
  }
  for (int i = 0; i < kHalfAverageScope; ++i) {
    speed_limit->at(i) = speed_limit->at(kHalfAverageScope);
    smoothed_kappa[i] = smoothed_kappa[kHalfAverageScope];
  }
  int end_idx = center_points.size() - 1 - kHalfAverageScope;
  for (int i = end_idx; i < center_points.size(); ++i) {
    speed_limit->at(i) = speed_limit->at(end_idx);
    smoothed_kappa[i] = smoothed_kappa[end_idx];
  }
  Log2DDS::LogChartV1(task_id + "s-kappa", "smoothed", Log2DDS::kRed, false,
                      station_points_s, smoothed_kappa);
}

bool SmoothDrivePassageCenterPoints(std::vector<StationCenter>* center_points) {
  std::vector<Vec2d> points;
  points.reserve(center_points->size());
  for (const auto& point : *center_points) {
    points.emplace_back(point.xy);
  }
  const auto smooth_result =
      SmoothPointsByGradientDescent(absl::MakeSpan(points), 500, 0.01, 0.15);
  if (!smooth_result.success) {
    LOG_ERROR << "Failed to Smooth drive passage!";
    return false;
  }
  DCHECK_EQ(points.size(), center_points->size());
  for (int i = 1; i < center_points->size() - 1; ++i) {
    center_points->at(i).xy = points[i];
    center_points->at(i).tangent = (points[i] - points[i - 1]).normalized();
  }
  // for i = 0;
  center_points->at(0).xy = points[0];
  center_points->at(0).tangent = center_points->at(1).tangent;
  // for i = center_points->size
  int size = center_points->size();
  center_points->at(size - 1).xy = points[size - 1];
  center_points->at(size - 1).tangent = center_points->at(size - 2).tangent;
  return true;
}

std::vector<double> CalSpeedLimitByCurvature(const DrivePassage& drive_passage,
                                             const std::string& task_id) {
  // Get Curvature speed limit
  std::vector<StationCenter> center_points;
  std::vector<double> speed_limit_by_curvature(
      drive_passage.size(), std::numeric_limits<double>::max());
  center_points.reserve(drive_passage.size());
  for (const auto& station : drive_passage.stations()) {
    StationCenter point{};
    point.xy = station.xy();
    point.tangent = station.tangent();
    point.accum_s = station.accumulated_s();
    point.speed_limit = DBL_MAX;
    point.is_virtual = false;
    point.is_merging = false;
    point.is_splitting = false;
    point.is_in_intersection = false;
    point.has_cross_curb = false;
    point.turn_type = ad_byd::planning::TurnType::NO_TURN;
    center_points.emplace_back(point);
  }
  // before
  std::vector<double> orin_kappa(center_points.size());
  std::vector<double> s_vec(center_points.size());
  for (int i = 1; i < center_points.size() - 1; ++i) {
    double s = center_points[i].accum_s;
    s_vec[i] = s;
    double k =
        std::fabs(NormalizeAngle(center_points[i + 1].tangent.FastAngle() -
                                 center_points[i - 1].tangent.FastAngle())) /
        (center_points[i + 1].accum_s - center_points[i - 1].accum_s);
    orin_kappa[i] = k;
  }
  s_vec[0] = center_points[0].accum_s;
  s_vec[center_points.size() - 1] =
      center_points[center_points.size() - 1].accum_s;
  orin_kappa[0] = orin_kappa[1];
  orin_kappa[center_points.size() - 1] = orin_kappa[center_points.size() - 2];
  Log2DDS::LogChartV1(task_id + "s-kappa", "orin", Log2DDS::kBlue, false, s_vec,
                      orin_kappa);

  bool res = SmoothDrivePassageCenterPoints(&center_points);
  if (res) {
    RefSpeedLimitByCurvature(center_points, task_id, &speed_limit_by_curvature);
  }
  return speed_limit_by_curvature;
}

RefSpeedVec::RefSpeedVec(const DrivePassage& drive_passage,
                         const std::vector<std::pair<double, double>>& obj_info,
                         double stop_s) {
  std::vector<double> station_accum_s, ref_speed_vec;
  station_accum_s.reserve(drive_passage.size());
  ref_speed_vec.reserve(drive_passage.size());

  for (const auto& station : drive_passage.stations()) {
    double current_s = station.accumulated_s();
    if (current_s > stop_s) break;

    const double limit_from_stop =
        std::sqrt(2 * (stop_s - current_s) * (-kMaxComfortableDeceleration));

    double limit_from_obj = std::numeric_limits<double>::max();
    for (const auto& [obj_s, obj_v] : obj_info) {
      const double obj_ref_v =
          current_s >= obj_s
              ? obj_v
              : std::sqrt(Sqr(obj_v) - 2 * (obj_s - current_s) *
                                           kMaxComfortableDeceleration);
      limit_from_obj = std::min(limit_from_obj, obj_ref_v);
    }

    station_accum_s.push_back(current_s);
    ref_speed_vec.push_back(std::min(limit_from_stop, limit_from_obj));
  }

  start_s_ = station_accum_s.front();
  end_s_ = station_accum_s.back();
  const int discretized_num =
      CeilToInt((end_s_ - start_s_) / kSpanDiscretizationStep);
  discretized_ref_speed_by_s_.resize(discretized_num + 1, 0.0);
  for (int i = 0, s_idx = 0; i <= discretized_num; ++i) {
    const double sample_s = i * kSpanDiscretizationStep + start_s_;
    if (sample_s <= start_s_ || sample_s >= end_s_) continue;

    if (sample_s > station_accum_s[s_idx]) ++s_idx;
    discretized_ref_speed_by_s_[i] =
        Lerp(ref_speed_vec[s_idx - 1], ref_speed_vec[s_idx],
             (sample_s - station_accum_s[s_idx - 1]) /
                 (station_accum_s[s_idx] - station_accum_s[s_idx - 1]));
  }
}

double RefSpeedVec::FastComputeRefSpeed(double s) const {
  if (s <= start_s_ || s >= end_s_) return 0.0;

  return discretized_ref_speed_by_s_[RoundToInt((s - start_s_) /
                                                kSpanDiscretizationStep)];
}

// #################### RefSpeedTable ####################
RefSpeedTable::RefSpeedTable(const SpacetimeTrajectoryManager& st_traj_mgr,
                             const std::vector<std::string>& leading_trajs,
                             const DrivePassage& drive_passage,
                             const std::vector<double>& stop_s,
                             const std::string& task_id) {
  // limits from stop constraints
  const double nearest_stop_s =
      stop_s.empty() ? std::numeric_limits<double>::max()
                     : *std::min_element(stop_s.begin(), stop_s.end());
  // speed limit by curvature
  auto curvature_speed_limits =
      CalSpeedLimitByCurvature(drive_passage, task_id);
  // limits from station speed limits
  station_accum_s_.reserve(drive_passage.size());
  station_speed_limits_.reserve(drive_passage.size());
  int cnt = 0;
  for (const auto& station : drive_passage.stations()) {
    station_accum_s_.push_back(station.accumulated_s());
    station_speed_limits_.push_back(
        std::min(station.speed_limit(), curvature_speed_limits[cnt]));
    ++cnt;
  }
  // log
  std::vector<double> speed_limit_without_curvature;
  std::vector<double> speed_limit_with_curvature;
  int counter = 0;
  for (const auto& station : drive_passage.stations()) {
    speed_limit_without_curvature.emplace_back(station.speed_limit());
    speed_limit_with_curvature.emplace_back(
        std::min(station.speed_limit(), curvature_speed_limits[counter]));
    ++counter;
  }
  Log2DDS::LogChartV1(task_id + "speed_limit", "without_curvature_s_speed",
                      Log2DDS::kBlue, false, station_accum_s_,
                      speed_limit_without_curvature);
  Log2DDS::LogChartV1(task_id + "speed_limit", "with_curvature_s_speed",
                      Log2DDS::kRed, false, station_accum_s_,
                      speed_limit_with_curvature);
  // limits from leading object predictions
  ref_speed_table_.reserve(kSampleTimeSteps);
  for (const double time_sample : kTimeSamples) {
    const int obj_state_idx =
        RoundToInt(time_sample / kPredictionTimeSampleStep);

    std::vector<std::pair<double, double>> obj_info;
    obj_info.reserve(leading_trajs.size());
    for (const auto& traj_id : leading_trajs) {
      const auto* traj_ptr = st_traj_mgr.FindTrajectoryById(traj_id);
      if (traj_ptr == nullptr) {
        LOG_WARN << "Leading trajectory " << traj_id << " cannot be found.";
        continue;
      }
      const auto& states = traj_ptr->states();
      if (states.size() <= obj_state_idx) continue;
      if (states[0].traj_point->v() > 4.0 &&
          states[obj_state_idx].traj_point->v() < 1e-5)
        continue;

      ASSIGN_OR_CONTINUE(const auto obj_aabbox, drive_passage.QueryFrenetBoxAt(
                                                    states[obj_state_idx].box));
      const double obj_v = states[obj_state_idx].traj_point->v();
      const double obj_s = std::max(
          0.0, obj_aabbox.s_min - kSafeLeadingDist - obj_v * kSafeLeadingTime);
      obj_info.push_back({obj_s, obj_v});
    }
    ref_speed_table_.emplace_back(drive_passage, obj_info, nearest_stop_s);
  }
}

std::pair<double, double> RefSpeedTable::LookUpRefSpeed(double time,
                                                        double span) const {
  CHECK_GE(time, 0.0);  // No look-up into the past.

  const int station_idx =
      std::upper_bound(station_accum_s_.begin(), station_accum_s_.end(), span) -
      station_accum_s_.begin();
  const double speed_limit = station_idx == station_accum_s_.size()
                                 ? station_speed_limits_.back()
                                 : station_speed_limits_[station_idx];

  // double passage_speed_limit = 0.0;
  // for (const auto& ssl : station_speed_limits_) {
  //   passage_speed_limit += ssl;
  // }
  // passage_speed_limit /= std::max(1.0, 1.0 * station_speed_limits_.size());
  // const double speed_limit = std::fmax(passage_speed_limit, 3.0);

  double ref_speed;
  const int next_index =
      std::upper_bound(kTimeSamples.begin(), kTimeSamples.end(), time) -
      kTimeSamples.begin();
  if (next_index == kTimeSamples.size()) {
    ref_speed = ref_speed_table_.back().FastComputeRefSpeed(span);
  } else {
    const double prev_ref_speed =
        ref_speed_table_[next_index - 1].FastComputeRefSpeed(span);
    const double succ_ref_speed =
        ref_speed_table_[next_index].FastComputeRefSpeed(span);
    const double interp_t =
        (time - kTimeSamples[next_index - 1]) /
        (kTimeSamples[next_index] - kTimeSamples[next_index - 1]);

    ref_speed = Lerp(prev_ref_speed, succ_ref_speed, interp_t);
  }

  return {speed_limit, std::min(speed_limit, ref_speed)};
}

}  // namespace st::planning
