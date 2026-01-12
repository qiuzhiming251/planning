

#include "planner/speed_optimizer/speed_optimizer.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <map>
#include <ostream>
#include <string>

#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "plan_common/log.h"
#include "plan_common/math/util.h"
#include "plan_common/log_data.h"
#include "plan_common/speed/st_speed/speed_point.h"
#include "plan_common/util/status_macros.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"

namespace st::planning {
namespace {

using DecisionType = StBoundaryProto::DecisionType;
using ConstraintMgr = SpeedOptimizerConstraintManager;

const double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kVCheckThreshold = -1e-2;
constexpr double kSDiffCheckThreshold = -1e-2;
constexpr double kStartSCheckThreshold = 1e-2;
constexpr double kLowSpeedThreshold = 1.0;
constexpr double kPathLengthThreshold = 0.1;  // m.

constexpr double kMaxProbOfAllowableCollision = 0.1;
constexpr double kMinProbOfNotShrunk = 0.8;

const PiecewiseLinearFunction<double, double> kTtcStopWeightGainPlf = {
    {1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, {0.68, 0.16, 0.048, 0.026, 0.018, 0.013}};

const PiecewiseLinearFunction<double, double> KTtcMovingWeakGainPlf = {
    {0.0, 3.0, 5.0}, {1.0, 0.4, 0.1}};

// Soft jerk constraint param.
constexpr double kAccelJerkTimeGain = 0.5;  ///
constexpr double kDecelJerkTimeGain = 0.4;
constexpr double kTimeToAddJerkConstraint = 4.0;  // s.
const PiecewiseLinearFunction<double, double> kAvSpeedJerkLowerWeightPlf = {
    {5.0, 10.0, 20.0}, {0.01, 6.0, 10.0}};
const PiecewiseLinearFunction<double, double> kAvSpeedJerkUpperWeightAccelPlf =
    {{2.0, 5.0, 10.0, 20.0, 30.0}, {0.1, 1.0, 6.0, 10.0, 15.0}};
const PiecewiseLinearFunction<double, double> kAvSpeedJerkUpperWeightDecelPlf =
    {{2.0, 5.0, 10.0, 20.0, 30.0}, {0.1, 6.0, 50.0, 70.0, 80.0}};
const PiecewiseLinearFunction<double, double> kAvSpeedJerkSoftUpperBoundPlf = {
    {2.0, 10.0, 20.0}, {1.0, 0.4, 0.2}};

const PiecewiseLinearFunction<double, double>
    kAvSpeedFollowDistanceLowestGainPlf = {{0.0, 6.0, 10.0}, {0.9, 0.6, 0.1}};
const PiecewiseLinearFunction<double, double> KMinFollowDistSpeedPartPlf = {
    {0.0, 10.0, 20.0, 30.0}, {0.0, 1.0, 2.0, 3.0}};

absl::Status PostProcessForValidity(std::string_view base_name,
                                    SpeedVector* speed_data) {
  CHECK_NOTNULL(speed_data);
  if (speed_data->empty()) {
    return absl::InternalError("Optimized speed is empty.");
  }

  for (int i = 0; i < speed_data->size(); ++i) {
    const double s = (*speed_data)[i].s();
    if (i == 0) {
      if (std::fabs(s) < kStartSCheckThreshold) {
        (*speed_data)[i].set_s(0.0);
      } else {
        return absl::InternalError(
            absl::StrCat("Start s is not close to 0.0, fabs(", s,
                         ") >= ", kStartSCheckThreshold, "."));
      }
    } else {
      // Monotomic requirement.
      (*speed_data)[i].set_s(std::max(s, (*speed_data)[i - 1].s()));
    }
    const double v = (*speed_data)[i].v();
    (*speed_data)[i].set_v(std::max(v, 0.0));
  }

  return absl::OkStatus();
}

double MakeUpperBoundByProbability(double prob, double reaction_bound,
                                   double collision_bound,
                                   double max_path_length) {
  CHECK_GE(max_path_length, collision_bound);
  CHECK_GE(collision_bound, reaction_bound);
  const PiecewiseLinearFunction<double> upper_bound_plf = {
      {0.0, kMaxProbOfAllowableCollision, kMinProbOfNotShrunk, 1.0},
      {max_path_length, collision_bound, reaction_bound, reaction_bound}};
  return upper_bound_plf(prob);
}

double MakeLowerBoundByProbability(double prob, double reaction_bound,
                                   double collision_bound) {
  CHECK_GE(reaction_bound, collision_bound);
  CHECK_GE(collision_bound, 0.0);
  const PiecewiseLinearFunction<double> lower_bound_plf = {
      {0.0, kMaxProbOfAllowableCollision, kMinProbOfNotShrunk, 1.0},
      {0.0, collision_bound, reaction_bound, reaction_bound}};
  return lower_bound_plf(prob);
}

double GetObjectTimeGain(double t, const std::vector<double>& time_ranges,
                         const std::vector<double>& time_gains) {
  CHECK_GT(time_ranges.size(), 1);
  CHECK_EQ(time_ranges.size() - 1, time_gains.size());
  const int dist = std::distance(
      time_ranges.begin(),
      std::lower_bound(time_ranges.begin(), time_ranges.end(), t));
  const int idx =
      std::clamp(dist - 1, 0, static_cast<int>(time_ranges.size() - 2));
  return time_gains[idx];
}

SpeedVector MakeComfortableBrakeSpeed(double init_v, double init_a,
                                      double total_time, double max_decel_t,
                                      double max_decel, double const_jerk) {
  constexpr double kDeltaT = 0.2;  // s.
  SpeedVector speed_points;
  speed_points.reserve(static_cast<int>(total_time / kDeltaT) + 2);
  SpeedPoint prev_speed_point(/*t=*/0.0, /*s=*/0.0, std::max(0.0, init_v),
                              init_a,
                              /*j=*/0.0);
  speed_points.push_back(prev_speed_point);

  for (double t = kDeltaT; t <= total_time + kDeltaT; t += kDeltaT) {
    const double prev_a = prev_speed_point.a();
    const double prev_v = prev_speed_point.v();
    const double prev_s = prev_speed_point.s();
    if (prev_v < 1e-6) {
      prev_speed_point.set_t(t);
      prev_speed_point.set_s(prev_s);
      prev_speed_point.set_v(0.0);
      prev_speed_point.set_a(0.0);
      prev_speed_point.set_j(0.0);
      speed_points.push_back(prev_speed_point);
      continue;
    }

    // Flip jerk.
    constexpr double kMinDecelDurationTime = 4.0;
    constexpr double kMaxDecelDurationTime = 6.0;
    const double flit_time =
        max_decel > 0.0 ? kMinDecelDurationTime
                        : std::clamp(max_decel_t, kMinDecelDurationTime,
                                     kMaxDecelDurationTime);
    double a, jerk;
    if (t > flit_time && prev_a < 0.0) {
      jerk = -const_jerk;
      a = std::min(prev_a + jerk * kDeltaT, 0.0);
    } else {
      jerk = const_jerk;
      if (prev_a > 0.0 && const_jerk < 0.0) {
        jerk *= 4.0;
      }
      a = std::max(prev_a + jerk * kDeltaT, max_decel);
    }
    const double v = std::max(0.0, prev_v + (prev_a + a) * kDeltaT * 0.5);
    a = 2.0 * (v - prev_v) / kDeltaT - prev_a;
    const double prev_j = (a - prev_a) / kDeltaT;
    constexpr double kOneSixth = 1.0 / 6.0;  // one-sixth.
    const double s =
        prev_s + std::max(prev_v * kDeltaT + 0.5 * prev_a * Sqr(kDeltaT) +
                              kOneSixth * prev_j * Cube(kDeltaT),
                          0.0);
    speed_points.back().set_j(prev_j);
    prev_speed_point.set_t(t);
    prev_speed_point.set_s(s);
    prev_speed_point.set_v(v);
    prev_speed_point.set_a(a);
    prev_speed_point.set_j(0.0);
    speed_points.push_back(prev_speed_point);
  }
  return speed_points;
}

SpeedVector GenerateMaxBrakingSpeed(double init_v, double init_a,
                                    double max_decel, double total_time,
                                    double delta_t) {
  constexpr double kJerk = -10.0;  // m/(s^3)
  SpeedVector speed_points;
  speed_points.reserve(static_cast<int>(total_time / delta_t) + 2);
  SpeedPoint prev_speed_point(/*t=*/0.0, /*s=*/0.0, std::max(0.0, init_v),
                              init_a,
                              /*j=*/0.0);
  speed_points.push_back(prev_speed_point);
  const double delta_t_sqr = Sqr(delta_t);
  const double delta_t_cube = delta_t_sqr * delta_t;
  for (double t = delta_t; t <= total_time + delta_t; t += delta_t) {
    const double prev_a = prev_speed_point.a();
    const double prev_v = prev_speed_point.v();
    const double prev_s = prev_speed_point.s();
    if (prev_v < 1e-6) {
      prev_speed_point.set_t(t);
      prev_speed_point.set_s(prev_s);
      prev_speed_point.set_v(0.0);
      prev_speed_point.set_a(0.0);
      prev_speed_point.set_j(0.0);
      speed_points.push_back(prev_speed_point);
      continue;
    }
    double a = std::max(prev_a + kJerk * delta_t, max_decel);
    const double v = std::max(0.0, prev_v + (prev_a + a) * delta_t * 0.5);
    a = 2.0 * (v - prev_v) / delta_t - prev_a;
    const double prev_j = (a - prev_a) / delta_t;
    constexpr double kOneSixth = 1.0 / 6.0;  // one-sixth.
    const double s =
        prev_s + std::max(prev_v * delta_t + 0.5 * prev_a * delta_t_sqr +
                              kOneSixth * prev_j * delta_t_cube,
                          0.0);
    speed_points.back().set_j(prev_j);
    prev_speed_point.set_t(t);
    prev_speed_point.set_s(s);
    prev_speed_point.set_v(v);
    prev_speed_point.set_a(a);
    prev_speed_point.set_j(0.0);
    speed_points.push_back(prev_speed_point);
  }
  return speed_points;
}

double ComputeMaxDesiredDecel(const SpeedOptimizerObjectManager& opt_obj_mgr,
                              double av_speed, double delta_t,
                              double* const max_decel_t) {
  if (max_decel_t == nullptr) {
    return 0.0;
  }
  double max_desired_decel = 0.0;
  constexpr double kStartTime = 1.0;        // s.
  constexpr double kLookForwardTime = 4.0;  // s.
  const int start_time_index = static_cast<int>(kStartTime / delta_t);
  const int look_forward_time_index =
      static_cast<int>(kLookForwardTime / delta_t);
  std::map<std::string, std::pair<double, double>> first_ds_t_map;
  for (int time_idx = start_time_index; time_idx < look_forward_time_index;
       ++time_idx) {
    const double time = time_idx * delta_t;
    for (const SpeedOptimizerObject& opt_obj :
         opt_obj_mgr.MovingFollowObjects()) {
      const auto& overlap_state = opt_obj.GetOverlapStateByIndex(time_idx);
      if (!overlap_state.has_value()) continue;
      if (opt_obj.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
        continue;
      }
      if (first_ds_t_map.find(std::string(opt_obj.id())) ==
          first_ds_t_map.end()) {
        const auto first_ds = std::min(overlap_state->bound - av_speed * time,
                                       overlap_state->lon_buffer);
        first_ds_t_map[std::string(opt_obj.id())] =
            std::make_pair(first_ds, time);
      }
      const auto& first_ds_t = first_ds_t_map[std::string(opt_obj.id())];
      // double delt_speed = overlap_state->speed > av_speed
      //                         ? std::max(overlap_state->speed - av_speed,
      //                         0.0) : std::max(av_speed -
      //                         overlap_state->speed, 0.0);
      const double desired_decel =
          2.0 *
          (overlap_state->bound -
           0.6 * Lerp(first_ds_t.first, first_ds_t.second,
                      overlap_state->lon_buffer, kLookForwardTime, time, true) -
           av_speed * time) /
          Sqr(time);
      if (desired_decel < max_desired_decel) {
        max_desired_decel = desired_decel;
        *max_decel_t = time;
      }
    }
  }
  return max_desired_decel;
}

SpeedVector GenerateComfortBrakeSpeedByMaxDesiredDecel(
    double init_v, double init_a, double total_time, double max_decel_t,
    double max_decel, double max_desired_decel,
    const PiecewiseLinearFunction<double>& comfort_jerk_rel_desired_decel_plf) {
  const double jerk = comfort_jerk_rel_desired_decel_plf(max_desired_decel);
  VLOG(2) << "Comfort brake speed jerk : " << jerk;

  return MakeComfortableBrakeSpeed(init_v, init_a, total_time, max_decel_t,
                                   max_decel, jerk);
}

SpeedLimitLevelProto::Level SpeedLimitTypeToLevel(
    SpeedLimitTypeProto::Type speed_limit_type) {
  switch (speed_limit_type) {
    case SpeedLimitTypeProto_Type_CURVATURE:
    case SpeedLimitTypeProto_Type_UNCERTAIN_PEDESTRAIN:
    case SpeedLimitTypeProto_Type_STEER_RATE:
    case SpeedLimitTypeProto_Type_LANE:
    case SpeedLimitTypeProto_Type_V2_TURN_TYPE:
    case SpeedLimitTypeProto_Type_BIG_JUNCTION:
    case SpeedLimitTypeProto_Type_NEAREST_CLOSE:
    case SpeedLimitTypeProto_Type_IN_JUNCTION_T_MAP:
      return SpeedLimitLevelProto::STRONG;
    case SpeedLimitTypeProto_Type_UNCERTAIN_VEHICLE:
    case SpeedLimitTypeProto_Type_NEAR_PARALLEL_VEHICLE:
    case SpeedLimitTypeProto_Type_IGNORE_OBJECT:
    case SpeedLimitTypeProto_Type_CLOSE_CURB:
    case SpeedLimitTypeProto_Type_CROSS_CURB:
    case SpeedLimitTypeProto_Type_APPROACH_CURB:
    case SpeedLimitTypeProto_Type_FAST_SPEED_LIMIT:
    case SpeedLimitTypeProto_Type_DEFAULT:
    case SpeedLimitTypeProto_Type_RIGHT_TURN_CLOSE:
    case SpeedLimitTypeProto_Type_GAP_REF_SPEED:
    case SpeedLimitTypeProto_Type_DEFENSIVE:
    case SpeedLimitTypeProto_Type_EXTERNAL:
      return SpeedLimitLevelProto::MEDIUM;
    case SpeedLimitTypeProto_Type_MOVING_CLOSE_TRAJ:
    case SpeedLimitTypeProto_Type_COMBINATION:
    case SpeedLimitTypeProto_Type_SOFT_ACC:
    case SpeedLimitTypeProto_Type_TOLL_SPEED_LIMIT:
    case SpeedLimitTypeProto_Type_CREEP_INTERACTION:
    case SpeedLimitTypeProto_Type_TRAFFIC_ACC_GAP:
    case SpeedLimitTypeProto_Type_PERCEPTION_LOSS:
      return SpeedLimitLevelProto::WEAK;
  }
  return SpeedLimitLevelProto::STRONG;
}

}  // namespace

SpeedOptimizer::SpeedOptimizer(
    std::string_view base_name, double init_v, double init_a,
    const MotionConstraintParamsProto* motion_constraint_params,
    const SpeedFinderParamsProto* speed_finder_params, double path_length,
    double default_speed_limit, double delta_t)
    : base_name_(std::string(base_name)),
      init_v_(init_v),
      init_a_(init_a),
      motion_constraint_params_(CHECK_NOTNULL(motion_constraint_params)),
      speed_finder_params_(CHECK_NOTNULL(speed_finder_params)),
      speed_optimizer_params_(&speed_finder_params->speed_optimizer_params()),
      delta_t_(delta_t),
      knot_num_(speed_optimizer_params_->knot_num()),
      total_time_(delta_t_ * (knot_num_ - 1)),
      allowed_max_speed_(Mph2Mps(default_speed_limit)),
      max_path_length_(path_length - kPathLengthThreshold),
      probability_gain_plf_(PiecewiseLinearFunctionFromProto(
          speed_finder_params_->probability_gain_plf())),
      accel_weight_gain_plf_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->accel_weight_gain_plf())),
      piecewise_time_range_(
          speed_optimizer_params_->piecewise_time_range().begin(),
          speed_optimizer_params_->piecewise_time_range().end()),
      moving_obj_time_gain_(
          speed_optimizer_params_->time_gain_for_moving_object().begin(),
          speed_optimizer_params_->time_gain_for_moving_object().end()),
      static_obj_time_gain_(
          speed_optimizer_params_->time_gain_for_static_object().begin(),
          speed_optimizer_params_->time_gain_for_static_object().end()),
      accel_lower_bound_plf_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->accel_lower_bound_plf())),
      ref_speed_time_gain_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->ref_speed_time_gain())),
      ttc_stop_weight_gain_plf_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->ttc_stop_weight_gain_plf())) {
  const int range_size = piecewise_time_range_.size() - 1;
  CHECK_EQ(range_size, moving_obj_time_gain_.size());
  CHECK_EQ(range_size, static_obj_time_gain_.size());
  slack_coeff_plf_.reserve(range_size);
  for (int i = 0; i < range_size; ++i) {
    CHECK_GT(moving_obj_time_gain_[i], 0.0);
    const double factor =
        i > 0 ? moving_obj_time_gain_[i - 1] / moving_obj_time_gain_[i] : 1.0;
    slack_coeff_plf_.emplace_back(
        std::vector<double>{piecewise_time_range_[i],
                            piecewise_time_range_[i + 1]},
        std::vector<double>{factor, 1.0});
  }
}

absl::Status SpeedOptimizer::Optimize(
    const SpeedOptimizerObjectManager& opt_obj_mgr,
    const SpeedBoundMapType& speed_bound_map,
    const SpeedVector& reference_speed,
    const std::vector<AccelBounds>& accel_bounds,
    const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
    SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  CHECK_NOTNULL(optimized_speed);
  CHECK_NOTNULL(speed_finder_debug_proto);

  auto* speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();

  double max_decel_t = 0.0;
  const double max_desired_decel =
      ComputeMaxDesiredDecel(opt_obj_mgr, init_v_, delta_t_, &max_decel_t);
  // Log2DDS::LogDataV0("max_desired_decel", max_desired_decel);
  VLOG(2) << "Max desired decel: " << max_desired_decel;

  auto comfortable_brake_speed = GenerateComfortBrakeSpeedByMaxDesiredDecel(
      init_v_, init_a_, total_time_, max_decel_t,
      motion_constraint_params_->max_deceleration(), max_desired_decel,
      PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->comfort_jerk_rel_desired_decel_plf()));

  for (const auto& speed_pt : comfortable_brake_speed) {
    speed_pt.ToProto(speed_optimizer_debug->add_comfortable_brake_speed());
  }
  comfortable_brake_speed_ = std::move(comfortable_brake_speed);

  auto max_brake_speed = GenerateMaxBrakingSpeed(
      init_v_, init_a_, motion_constraint_params_->max_deceleration(),
      total_time_, delta_t_);
  for (const auto& speed_pt : max_brake_speed) {
    speed_pt.ToProto(speed_optimizer_debug->add_max_brake_speed());
  }
  max_brake_speed_ = max_brake_speed;

  const auto min_stationary_upper_bound =
      ComputeMinStationaryUpperBound(opt_obj_mgr);

  ConstraintMgr constraint_mgr(piecewise_time_range_, delta_t_, knot_num_);
  for (int i = 1; i < knot_num_; ++i) {
    // ("SpeedOptimizer::MakeConstraint");
    MakeSConstraint(i, opt_obj_mgr, min_stationary_upper_bound, &constraint_mgr,
                    speed_finder_debug_proto);
    MakeSpeedConstraint(i, speed_bound_map, &constraint_mgr,
                        speed_finder_debug_proto);
    MakeAccelConstraint(i, accel_bounds, &constraint_mgr);
  }
  // Add hard s constraint for last knot.
  constraint_mgr.AddHardSConstraint(knot_num_ - 1, 0.0, max_path_length_);
  // Make jerk constraint after MakeSConstraint.
  MakeJerkConstraint(time_aligned_prev_traj, &constraint_mgr);

  const int soft_s_lower_num = constraint_mgr.GetSlackNumOfLowerS();
  const int soft_s_upper_num = constraint_mgr.GetSlackNumOfUpperS();
  const int soft_v_lower_num = constraint_mgr.GetSlackNumOfLowerSpeed();
  const int soft_v_upper_num = constraint_mgr.GetSlackNumOfUpperSpeed();
  const int soft_a_lower_num = constraint_mgr.GetSlackNumOfLowerAccel();
  const int soft_a_upper_num = constraint_mgr.GetSlackNumOfUpperAccel();
  const int soft_j_lower_num = constraint_mgr.GetSlackNumOfLowerJerk();
  const int soft_j_upper_num = constraint_mgr.GetSlackNumOfUpperJerk();

  // Set debug proto.
  speed_optimizer_debug->set_init_v(init_v_);
  speed_optimizer_debug->set_init_a(init_a_);
  speed_optimizer_debug->set_delta_t(delta_t_);
  speed_optimizer_debug->set_lower_s_slack_term_num(soft_s_lower_num);
  speed_optimizer_debug->set_upper_s_slack_term_num(soft_s_upper_num);
  speed_optimizer_debug->set_lower_v_slack_term_num(soft_v_lower_num);
  speed_optimizer_debug->set_upper_v_slack_term_num(soft_v_upper_num);
  speed_optimizer_debug->set_lower_a_slack_term_num(soft_a_lower_num);
  speed_optimizer_debug->set_upper_a_slack_term_num(soft_a_upper_num);

  VLOG(3) << "lower_s_num = " << soft_s_lower_num;
  VLOG(3) << "upper_s_num = " << soft_s_upper_num;

  solver_ = std::make_unique<PiecewiseJerkQpSolver>(
      speed_optimizer_params_->knot_num(), delta_t_, soft_s_lower_num,
      soft_s_upper_num, soft_v_lower_num, soft_v_upper_num, soft_a_lower_num,
      soft_a_upper_num, soft_j_lower_num, soft_j_upper_num);

  if (!AddConstarints(init_v_, init_a_, constraint_mgr)) {
    const std::string err_msg = "Failed to add constraint.";
    LOG_ERROR << err_msg;
    return absl::InternalError(err_msg);
  }
  for (const auto& speed_pt : reference_speed) {
    speed_pt.ToProto(speed_optimizer_debug->add_ref_speed());
  }
  if (!AddKernel(constraint_mgr, reference_speed)) {
    const std::string err_msg = "Failed to add kernel.";
    LOG_ERROR << err_msg;
    return absl::InternalError(err_msg);
  }

  if (const absl::Status status = Solve(); !status.ok()) {
    const auto err_msg = status.ToString();
    LOG_ERROR << err_msg;
    return absl::InternalError(err_msg);
  }

  // extract output
  optimized_speed->clear();
  const double output_time_resolution =
      speed_optimizer_params_->output_time_resolution();
  optimized_speed->reserve(
      static_cast<int>(total_time_ / output_time_resolution) + 1);
  for (double t = 0.0; t < total_time_; t += output_time_resolution) {
    std::array<double, 4> res = solver_->Evaluate(t);
    const SpeedPoint& speed_point =
        optimized_speed->emplace_back(t, res[0], res[1], res[2], res[3]);
    speed_point.ToProto(speed_optimizer_debug->add_optimized_speed());
  }

  RETURN_IF_ERROR(PostProcessForValidity(base_name_, optimized_speed));

  return absl::OkStatus();
}

void SpeedOptimizer::FilterQpSpeedPoints(
    std::vector<SpeedPoint>& qp_speed_points) {
  const double kFilterLowSpeedThreshold =
      qp_speed_points.front().v() < 1e-2 ? 0.4 : 0.2;
  // constexpr double kFilterLowAccThreshold = 0.3;
  constexpr double kFilterSLength = 1.0;
  constexpr double kMaxAcc = -0.5;

  bool is_low_speed_for_stop = true;
  for (const auto& sp : qp_speed_points) {
    if (sp.v() > kFilterLowSpeedThreshold ||
        sp.s() - qp_speed_points.front().s() > kFilterSLength) {
      is_low_speed_for_stop = false;
      break;
    }
  }
  if (is_low_speed_for_stop && qp_speed_points.size() > 2) {
    double current_s = qp_speed_points.front().s();
    double current_v = qp_speed_points.front().v();
    const double acc = std::min(qp_speed_points.front().a(), kMaxAcc);
    double tmp_s = current_s;
    qp_speed_points.clear();
    for (double t = 0.0; t < total_time_;
         t += speed_optimizer_params_->output_time_resolution()) {
      const double v = std::max(current_v + t * acc, 0.0);
      if (v > 1e-3) {
        tmp_s = std::max(tmp_s, current_s + current_v * t + 0.5 * acc * Sqr(t));
      }
      qp_speed_points.emplace_back(t, tmp_s, v, acc, 0.0);
    }
  }
  Log2DDS::LogDataV2("filter_qp_speed", is_low_speed_for_stop);

  return;
}

void SpeedOptimizer::MakeStationaryObjectFollowConstraint(
    int knot_idx, const ObjectOverlapState& overlap_state, std::string id,
    double time_gain, ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  CHECK_NOTNULL(constraint_mgr);
  CHECK_NOTNULL(speed_finder_debug_proto);

  const double upper_s = overlap_state.bound;
  const double follow_dist = overlap_state.lon_buffer;

  const double time = knot_idx * delta_t_;
  const auto max_braking_pt = max_brake_speed_.EvaluateByTime(time);
  const double s_upper = max_braking_pt.has_value()
                             ? std::max(upper_s, max_braking_pt->s())
                             : upper_s;

  double upper_bound = std::max(0.0, s_upper - follow_dist);
  constexpr double kFullStopSpeedThres = 0.5;  // m/s.
  double weight = speed_optimizer_params_->s_stop_weight();
  if (init_v_ >= kFullStopSpeedThres) {
    // Calculate stop weight gain based on ttc if AV not fully stopped.
    const double ttc = upper_bound / init_v_;
    weight *= ttc_stop_weight_gain_plf_(ttc);
    if (5 == knot_idx) {  //  temp print, to check package version.
      Log2DDS::LogDataV2("speed_testversion_kttc3",
                         ttc_stop_weight_gain_plf_(3.0));
    }

    const double ref_v = std::fmax(
        0.0, init_v_ + std::fmax(std::fmin(init_a_, 0.0), -1.0) * time);
    const double delta_v = std::fmax(0.0, ref_v);
    const auto& ref_speed_static_decrease_plf =
        PiecewiseLinearFunctionFromProto(
            speed_optimizer_params_->ref_speed_static_decrease());
    double static_bound_decrease_threshold =
        ref_speed_static_decrease_plf(Mps2Kph(init_v_));
    upper_bound = std::max(0.0, upper_bound);
  }
  weight *= time_gain;
  constraint_mgr->AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::MEDIUM,
      {.knot_idx = knot_idx, .weight = weight, .bound = upper_bound});

  const std::string chart_tips =
      absl::StrFormat("following_distance: %.2fm\nweight: %.2f(stop_weight)",
                      follow_dist, weight);

  auto* speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();
  auto& plot_data = (*speed_optimizer_debug->mutable_soft_s_upper_bound())[id];
  plot_data.add_value(upper_bound);
  plot_data.add_time(time);
  plot_data.add_soft_bound_distance(follow_dist);
  plot_data.add_info(chart_tips);
}

void SpeedOptimizer::MakeMovingObjectFollowConstraint(
    int knot_idx, const ObjectOverlapState& overlap_state, std::string id,
    double follow_standstill,
    const std::optional<double>& min_stationary_upper_bound,
    double time_att_gain,
    const PiecewiseLinearFunction<double>&
        comfortable_brake_bound_violation_rel_speed_plf,
    ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  CHECK_NOTNULL(constraint_mgr);
  CHECK_NOTNULL(speed_finder_debug_proto);

  const double time = knot_idx * delta_t_;
  const double obj_speed = std::max(0.0, overlap_state.speed);
  const double upper_s = overlap_state.bound;
  const double obj_average_prob = overlap_state.prob;
  const double min_follow_dist =
      follow_standstill + KMinFollowDistSpeedPartPlf(init_v_);

  // Step 1: Calculate following distance.
  constexpr double kNegDistanceBuffer = 0.5;
  const double follow_dist =
      overlap_state.no_weak_buffer
          ? std::min(0.0, upper_s + kNegDistanceBuffer)
          : std::min(std::max(overlap_state.lon_buffer, min_follow_dist),
                     upper_s + kNegDistanceBuffer);

  // Step 2: Clamp weak upper bound by comfortable brake speed.
  const auto comfortable_brake_pt =
      comfortable_brake_speed_.EvaluateByTime(time);
  double weak_upper_bound = upper_s - follow_dist;
  if (speed_optimizer_params_->enable_comfort_brake_speed() &&
      comfortable_brake_pt.has_value()) {
    const double comfortable_brake_upper_bound =
        comfortable_brake_pt->s() -
        comfortable_brake_bound_violation_rel_speed_plf(obj_speed - init_v_);
    if (comfortable_brake_upper_bound > weak_upper_bound) {
      weak_upper_bound = std::max(
          -kNegDistanceBuffer,
          std::min(comfortable_brake_upper_bound, upper_s - min_follow_dist));
    }
  }
  weak_upper_bound = std::fmax(-kNegDistanceBuffer, weak_upper_bound);
  // const double ref_v =
  //   std::fmax(0.0, init_v_ + std::fmax(std::fmin(init_a_, 0.0), -1.0) *
  //   time);
  // const double delta_v = std::fmax(0.0, ref_v - std::fmax(obj_speed, 0.0));
  // DLOG(INFO) << "speed para: "
  //            << speed_optimizer_params_->upper_bound_decrease_threshold();

  // Step 3: Add weak and strong soft constraints to constraint manager.
  // Tips: Now all probablilty set to 1.0, so no processing is required.
  double weak_upper_bound_by_prob = weak_upper_bound;
  const double strong_weight =
      time_att_gain * speed_optimizer_params_->s_follow_strong_weight();
  // if (obj_speed < kLowSpeedThreshold) {
  //   // When the target object speed is low, only add a strong soft constraint
  //   // and use the weak upper_s as the bound.
  //   // Make strong constraints larger than 0 to accelerate computation.
  //   weak_upper_bound_by_prob = std::max(0.0, weak_upper_bound_by_prob);
  //   constraint_mgr->AddSoftSUpperConstraint(
  //       ConstraintMgr::BoundStrType::STRONG,
  //       {.knot_idx = knot_idx,
  //        .weight = strong_weight,
  //        .bound = weak_upper_bound_by_prob});
  // } else {
  const double follow_safety_distance = std::min(
      speed_finder_params_->follow_safety_distance(), follow_standstill);
  const double strong_upper_bound =
      std::max(upper_s - follow_safety_distance, weak_upper_bound);
  // Tips: Now all probablilty set to 1.0, so no processing is required.
  const double strong_upper_bound_by_prob = strong_upper_bound;
  constraint_mgr->AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = knot_idx,
       .weight = strong_weight,
       .bound = std::max(0.0, strong_upper_bound_by_prob)});

  if (!overlap_state.no_weak_buffer) {
    double weak_weight =
        time_att_gain * speed_optimizer_params_->s_follow_weak_weight();
    double ttc =
        weak_upper_bound_by_prob / std::max(1.0, init_v_ - obj_speed) - time;
    if (time < 2.5) {
      weak_weight =
          std::max(strong_weight * KTtcMovingWeakGainPlf(ttc), weak_weight);
    }
    constraint_mgr->AddSoftSUpperConstraint(
        ConstraintMgr::BoundStrType::WEAK, {.knot_idx = knot_idx,
                                            .weight = weak_weight,
                                            .bound = weak_upper_bound_by_prob});
  }
  // }

  if (!min_stationary_upper_bound.has_value() ||
      (time < kTimeToAddJerkConstraint &&
       weak_upper_bound_by_prob < *min_stationary_upper_bound)) {
    use_soft_jerk_constraint_ = true;
  }

  // Step 4: Debug and plot.
  const double actual_follow_dist =
      upper_s - weak_upper_bound_by_prob;  // Could be negative.
  const std::string chart_tips = absl::StrFormat(
      "final prob: %.2f\nraw_follow_dist: "
      "%.2fm\nactual_follow_dist: "
      "%.2fm\nobj_speed: %.2fm/s\ntime_gain: %.2f",
      obj_average_prob, follow_dist, actual_follow_dist, obj_speed,
      time_att_gain);
  auto* speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();
  auto& plot_data = (*speed_optimizer_debug->mutable_soft_s_upper_bound())[id];
  plot_data.add_value(weak_upper_bound_by_prob);
  plot_data.add_time(time);
  plot_data.add_soft_bound_distance(actual_follow_dist);
  plot_data.add_info(chart_tips);
}

void SpeedOptimizer::MakeMovingObjectLeadConstraint(
    int knot_idx, const ObjectOverlapState& overlap_state, std::string id,
    double time_att_gain, ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  CHECK_NOTNULL(constraint_mgr);
  CHECK_NOTNULL(speed_finder_debug_proto);

  const double lower_s = overlap_state.bound;
  const double obj_average_prob = overlap_state.prob;
  const double lead_dist = overlap_state.lon_buffer;
  const double obj_speed = std::max(0.0, overlap_state.speed);

  // Step 1: Calculate leading distance.
  const double weak_lower_bound = lower_s + lead_dist;
  const double strong_lower_bound = lower_s;

  // Step 2: Make lower bound by the probability.
  const double weak_lower_bound_by_prob = weak_lower_bound;
  const double strong_lower_bound_by_prob = strong_lower_bound;

  // Step 3: Make slack variable's weight.
  const double strong_weight =
      time_att_gain * speed_optimizer_params_->s_lead_strong_weight();
  const double weak_weight =
      time_att_gain * speed_optimizer_params_->s_lead_weak_weight();

  // Step 4: Add constraint data to constraint manager.
  constraint_mgr->AddSoftSLowerConstraint(ConstraintMgr::BoundStrType::WEAK,
                                          {.knot_idx = knot_idx,
                                           .weight = weak_weight,
                                           .bound = weak_lower_bound_by_prob});
  constraint_mgr->AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = knot_idx,
       .weight = strong_weight,
       .bound = strong_lower_bound_by_prob});

  // Step 5: Debug and plot.
  const double actual_lead_dist =
      weak_lower_bound_by_prob - lower_s;  // Could be negative.
  auto* speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();

  auto& plot_data = (*speed_optimizer_debug->mutable_soft_s_lower_bound())[id];
  plot_data.add_value(weak_lower_bound_by_prob);
  plot_data.add_time(knot_idx * delta_t_);
  plot_data.add_soft_bound_distance(actual_lead_dist);
  plot_data.add_info(absl::StrFormat(
      "final prob: %.2f\nraw_lead_dist: "
      "%.2f\nactual_lead_dist: "
      "%.2f\nweight: %.2f\nobj_speed : %.2fm/s\ntime_gain: %.2f",
      obj_average_prob, lead_dist, actual_lead_dist, weak_weight, obj_speed,
      time_att_gain));
}

void SpeedOptimizer::MakeSConstraint(
    int knot_idx, const SpeedOptimizerObjectManager& opt_obj_mgr,
    const std::optional<double>& min_stationary_upper_bound,
    ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  CHECK_NOTNULL(constraint_mgr);
  CHECK_NOTNULL(speed_finder_debug_proto);

  const double time_att_gain = GetObjectTimeGain(
      knot_idx * delta_t_, piecewise_time_range_, moving_obj_time_gain_);

  // Add moving follow objects constraints.
  for (const SpeedOptimizerObject& obj : opt_obj_mgr.MovingFollowObjects()) {
    const auto& overlap_state = obj.GetOverlapStateByIndex(knot_idx);
    if (!overlap_state.has_value()) continue;
    MakeMovingObjectFollowConstraint(
        knot_idx, *overlap_state, obj.id(), obj.standstill(),
        min_stationary_upper_bound, time_att_gain,
        PiecewiseLinearFunctionFromProto(
            speed_optimizer_params_
                ->comfortable_brake_bound_violation_rel_speed_plf()),
        constraint_mgr, speed_finder_debug_proto);
  }

  const double static_time_att_gain = GetObjectTimeGain(
      knot_idx * delta_t_, piecewise_time_range_, static_obj_time_gain_);
  // Add stationary follow objects constraints.
  for (const SpeedOptimizerObject& obj : opt_obj_mgr.StationaryObjects()) {
    const auto& overlap_state = obj.GetOverlapStateByIndex(knot_idx);
    if (!overlap_state.has_value()) continue;
    MakeStationaryObjectFollowConstraint(knot_idx, *overlap_state, obj.id(),
                                         static_time_att_gain, constraint_mgr,
                                         speed_finder_debug_proto);
  }

  // Add moving lead objects constraints.
  for (const SpeedOptimizerObject& obj : opt_obj_mgr.MovingLeadObjects()) {
    const auto& overlap_state = obj.GetOverlapStateByIndex(knot_idx);
    if (!overlap_state.has_value()) continue;
    MakeMovingObjectLeadConstraint(knot_idx, *overlap_state, obj.id(),
                                   time_att_gain, constraint_mgr,
                                   speed_finder_debug_proto);
  }

  constraint_mgr->FilterObjectSConstraint(knot_idx);
}

void SpeedOptimizer::MakeSpeedConstraint(
    int knot_idx,
    const std::map<SpeedLimitTypeProto::Type, std::vector<SpeedBoundWithInfo>>&
        speed_limit_map,
    ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug) const {
  CHECK_NOTNULL(constraint_mgr);
  CHECK_NOTNULL(speed_finder_debug);

  auto* speed_optimizer_debug = speed_finder_debug->mutable_speed_optimizer();

  constexpr double kDecel = -1.0;  // m/s^2
  constexpr double kAllowedIgnoreTime = 2.0;
  const double time = knot_idx * delta_t_;
  const double v_at_time = std::max(init_v_ + kDecel * time, 0.0);

  const auto get_slack_weight = [](auto level, const auto& params) {
    switch (level) {
      case SpeedLimitLevelProto::STRONG:
        return params.speed_limit_strong_weight();
      case SpeedLimitLevelProto::MEDIUM:
        return params.speed_limit_medium_weight();
      case SpeedLimitLevelProto::WEAK:
        return params.speed_limit_weak_weight();
      default:
        throw std::runtime_error("switch case on enum unexpected");
    }
  };

  for (const auto& [type, speed_limit] : speed_limit_map) {
    if (type == SpeedLimitTypeProto_Type_COMBINATION) continue;
    const double speed_upper_bound = speed_limit[knot_idx].bound;

    // Avoid the close range speed limits lower than the init speed, it may
    // increase the time cost and failure probability.
    // BANDAID(ping): Ignore curvature speed limit may cause steering
    // rate exceed the limitation.
    if (speed_upper_bound < v_at_time && time < kAllowedIgnoreTime &&
        type != SpeedLimitTypeProto_Type_CURVATURE &&
        type != SpeedLimitTypeProto_Type_UNCERTAIN_PEDESTRAIN &&
        type != SpeedLimitTypeProto_Type_UNCERTAIN_VEHICLE &&
        type != SpeedLimitTypeProto_Type_NEAR_PARALLEL_VEHICLE &&
        type != SpeedLimitTypeProto_Type_IGNORE_OBJECT) {
      continue;
    }

    // Add constraint.
    const auto level = SpeedLimitTypeToLevel(type);
    constraint_mgr->AddSoftSpeedUpperConstraints(
        level, {.knot_idx = knot_idx,
                .weight = get_slack_weight(level, *speed_optimizer_params_),
                .bound = speed_upper_bound});

    // Add debug info for plot speed limit in chart.
    auto& plot_data =
        (*speed_optimizer_debug
              ->mutable_speed_limit())[SpeedLimitTypeProto::Type_Name(type)];
    plot_data.add_value(std::min(speed_upper_bound, allowed_max_speed_));
    plot_data.add_time(knot_idx * delta_t_);
    plot_data.add_info(speed_limit[knot_idx].info);
  }

  // Filter the constraints of different levels at same knot_idx.
  constraint_mgr->FilterObjectSpeedConstraint(knot_idx);

  constraint_mgr->AddHardSpeedConstraint(knot_idx, 0.0, kInfinity);
}

bool SpeedOptimizer::AddConstarints(double init_v, double init_a,
                                    const ConstraintMgr& constraint_mgr) {
  // ("SpeedOptimizer::AddConstarints");

  /******************* add init constraint **********************/
  solver_->AddNthOrderEqualityConstraint(/*order=*/0, std::vector<int>{0},
                                         std::vector<double>{1.0}, 0.0);
  solver_->AddNthOrderEqualityConstraint(/*order=*/1, std::vector<int>{0},
                                         std::vector<double>{1.0},
                                         std::max(0.0, init_v));
  solver_->AddNthOrderEqualityConstraint(
      /*order=*/2, std::vector<int>{0}, std::vector<double>{1.0}, init_a);

  // set last jerk to 0.0
  solver_->AddNthOrderEqualityConstraint(/*order=*/3,
                                         std::vector<int>{knot_num_ - 1},
                                         std::vector<double>{1.0}, 0.0);

  /******************** add s constraint ********************/
  for (const auto& data : constraint_mgr.hard_s_constraint()) {
    solver_->AddNthOrderInequalityConstraint(
        /*order=*/0, {data.knot_idx},
        /*coeff=*/{1.0}, data.lower_bound, data.upper_bound);
  }

  for (const auto& data : constraint_mgr.GetLowerConstraintOfSoftS()) {
    const int range_idx =
        GetTimeRangeIndex(piecewise_time_range_, data.knot_idx * delta_t_);
    const double slack_coeff =
        1.0 / std::sqrt(slack_coeff_plf_[range_idx](data.knot_idx * delta_t_));
    solver_->AddZeroOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound,
        slack_coeff);
  }

  for (const auto& data : constraint_mgr.GetUpperConstraintOfSoftS()) {
    const int range_idx =
        GetTimeRangeIndex(piecewise_time_range_, data.knot_idx * delta_t_);
    const double slack_coeff =
        1.0 / std::sqrt(slack_coeff_plf_[range_idx](data.knot_idx * delta_t_));
    solver_->AddZeroOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, data.bound,
        slack_coeff);
  }

  /******************** add v constraint ********************/
  for (const auto& data : constraint_mgr.hard_v_constraint()) {
    solver_->AddNthOrderInequalityConstraint(
        /*order=*/1, {data.knot_idx},
        /*coeff=*/{1.0}, data.lower_bound, data.upper_bound);
  }

  for (const auto& data : constraint_mgr.GetLowerConstraintOfSpeed()) {
    solver_->AddFirstOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound);
  }

  for (const auto& data : constraint_mgr.GetUpperConstraintOfSpeed()) {
    solver_->AddFirstOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, data.bound);
  }

  /******************** add accel constraint ******************/
  for (const auto& data : constraint_mgr.hard_a_constraint()) {
    solver_->AddNthOrderInequalityConstraint(
        /*order=*/2, {data.knot_idx},
        /*coeff=*/{1.0}, data.lower_bound, data.upper_bound);
  }

  for (const auto& data : constraint_mgr.GetLowerConstraintOfSoftAccel()) {
    solver_->AddSecondOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound);
  }

  for (const auto& data : constraint_mgr.GetUpperConstraintOfSoftAccel()) {
    solver_->AddSecondOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, data.bound);
  }

  /******************** add jerk constraint ******************/
  for (const auto& data : constraint_mgr.GetLowerConstraintOfSoftJerk()) {
    solver_->AddThirdOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound);
  }

  for (const auto& data : constraint_mgr.GetUpperConstraintOfSoftJerk()) {
    solver_->AddThirdOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, data.bound);
  }

  return true;
}

bool SpeedOptimizer::AddKernel(const ConstraintMgr& constraint_mgr,
                               const SpeedVector& reference_speed) {
  // ("SpeedOptimizer::AddKernel");
  CHECK_EQ(reference_speed.size(), knot_num_);

  // Add s and v weight.
  for (int i = 0; i < knot_num_; ++i) {
    const auto time = static_cast<double>(i) * delta_t_;
    solver_->AddNthOrderReferencePointKernel</*order=*/0>(
        i, max_path_length_, speed_optimizer_params_->s_kernel_weight());

    const double ref_v = speed_optimizer_params_->enable_const_speed_ref_v()
                             ? init_v_
                             : reference_speed[i].v();

    const auto ref_speed_time_gain = ref_speed_time_gain_(time);
    const auto ref_speed_weight =
        ref_speed_time_gain * speed_optimizer_params_->speed_kernel_weight();
    solver_->AddNthOrderReferencePointKernel</*order=*/1>(i, ref_v,
                                                          ref_speed_weight);
  }

  const double accel_weight_gain = accel_weight_gain_plf_(init_v_);
  solver_->AddSecondOrderDerivativeKernel(
      accel_weight_gain * speed_optimizer_params_->accel_kernel_weight());

  solver_->AddThirdOrderDerivativeKernel(
      speed_optimizer_params_->jerk_kernel_weight());

  // Add s slack weight.
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfSoftS()) {
    solver_->AddZeroOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                        {slack_weight});
  }

  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfSoftS()) {
    solver_->AddZeroOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                        {slack_weight});
  }

  // Add v slack weight.
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfSpeed()) {
    solver_->AddFirstOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                         {slack_weight});
  }
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfSpeed()) {
    solver_->AddFirstOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                         {slack_weight});
  }

  // Add a slack weight.
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfAccel()) {
    solver_->AddSecondOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                          {slack_weight});
  }

  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfAccel()) {
    solver_->AddSecondOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                          {slack_weight});
  }

  // Add jerk slack weight
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfJerk()) {
    solver_->AddThirdOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                         {slack_weight});
  }

  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfJerk()) {
    solver_->AddThirdOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                         {slack_weight});
  }

  // Add regularization.
  solver_->AddRegularization(speed_optimizer_params_->regularization());

  return true;
}

absl::Status SpeedOptimizer::Solve() {
  // ("SpeedOptimizer::Solve");
  auto status = solver_->Optimize();
  const auto iters = solver_->osqp_iter();
  if (constexpr int kCheckNum = 1000; iters >= kCheckNum) {
  }

  if (solver_->status_val() == OSQP_MAX_ITER_REACHED) {
  }
  return status;
}

void SpeedOptimizer::MakeAccelConstraint(
    int knot_idx, const std::vector<AccelBounds>& accel_bounds,
    ConstraintMgr* constraint_mgr) const {
  CHECK_NOTNULL(constraint_mgr);

  constraint_mgr->AddAccelConstraint(knot_idx,
                                     accel_bounds[knot_idx].lower_bound,
                                     accel_bounds[knot_idx].upper_bound);

  const auto& accel_soft_lower_bound = accel_bounds[knot_idx].soft_lower_bound;
  if (accel_soft_lower_bound.has_value()) {
    constraint_mgr->AddAccelSoftLowerConstraint(
        {.knot_idx = knot_idx,
         .weight = speed_optimizer_params_->accel_lower_slack_weight(),
         .bound = *accel_soft_lower_bound});
  }

  const auto& accel_soft_upper_bound = accel_bounds[knot_idx].soft_upper_bound;
  if (accel_soft_upper_bound.has_value()) {
    constraint_mgr->AddAccelSoftUpperConstraint(
        {.knot_idx = knot_idx,
         .weight = speed_optimizer_params_->accel_upper_slack_weight(),
         .bound = *accel_soft_upper_bound});
  }
}

void SpeedOptimizer::MakeJerkConstraint(
    const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
    ConstraintMgr* constraint_mgr) const {
  CHECK_NOTNULL(constraint_mgr);
  int knot_idx = 0;
  double time = knot_idx * delta_t_;
  const double lower_weight = kAvSpeedJerkLowerWeightPlf(init_v_);
  constexpr double kThrottleAccel = -0.2;  // m/s^2.
  const double upper_weight = init_a_ > kThrottleAccel
                                  ? kAvSpeedJerkUpperWeightAccelPlf(init_v_)
                                  : kAvSpeedJerkUpperWeightDecelPlf(init_v_);

  double min_jerk = 0.0;
  double max_jerk = 0.0;
  if (time_aligned_prev_traj != nullptr && !time_aligned_prev_traj->empty()) {
    constexpr double kPrevTrajTimeThres = 1.0;  // s.
    for (const auto& traj_pt : *time_aligned_prev_traj) {
      if (traj_pt.relative_time() > kPrevTrajTimeThres) break;
      min_jerk = std::min(min_jerk, traj_pt.j());
      max_jerk = std::max(max_jerk, traj_pt.j());
    }
  } else {
    min_jerk = speed_finder_params_->max_decel_jerk();
    max_jerk = kAvSpeedJerkSoftUpperBoundPlf(init_v_);
  }
  while (time <= kTimeToAddJerkConstraint) {
    constraint_mgr->AddJerkSoftLowerConstraint(
        {.knot_idx = knot_idx, .weight = lower_weight, .bound = min_jerk});
    constraint_mgr->AddJerkSoftUpperConstraint(
        {.knot_idx = knot_idx, .weight = upper_weight, .bound = max_jerk});
    time = ++knot_idx * delta_t_;
  }
}

std::optional<double> SpeedOptimizer::ComputeMinStationaryUpperBound(
    const SpeedOptimizerObjectManager& opt_obj_mgr) {
  std::optional<double> min_stationary_upper_bound;
  for (const SpeedOptimizerObject& opt_obj : opt_obj_mgr.StationaryObjects()) {
    const auto& overlap_state = opt_obj.GetOverlapStateByIndex(0);
    CHECK(overlap_state.has_value());
    const double upper_bound = overlap_state->bound - overlap_state->lon_buffer;
    if (!min_stationary_upper_bound.has_value() ||
        upper_bound < *min_stationary_upper_bound) {
      min_stationary_upper_bound = upper_bound;
    }
  }
  return min_stationary_upper_bound;
}

}  // namespace st::planning
