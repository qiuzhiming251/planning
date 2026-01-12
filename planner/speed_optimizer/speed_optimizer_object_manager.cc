

#include "planner/speed_optimizer/speed_optimizer_object_manager.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "plan_common/log_data.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "object_manager/planner_object.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/maps/st_boundary.h"
namespace st::planning {
namespace {
using ClassifiedObjects = std::vector<std::optional<SpeedOptimizerObject>>;

using ClassifiedOverlapState = std::vector<std::optional<ObjectOverlapState>>;

constexpr int kObjectTypeNum = 3;

const PiecewiseLinearFunction<double, double>
    kAvSpeedFollowDistanceLowestGainPlf = {{0.0, 6.0, 10.0}, {0.9, 0.6, 0.1}};
const PiecewiseLinearFunction<double, double> KFollowTimeHeadwayGainPlf = {
    {3.0, 6.0, 10.0, 15.0, 20.0}, {0.7, 0.85, 1.0, 1.0, 1.0}};
const double KMinHeadwayTime = 1.1;

const PiecewiseLinearFunction<double, double> KOvertakeRelSpeedGainPlf = {
    {0.5, 3.0}, {1.0, 0.4}};

double ComputeFollowTimeHeadway(double av_speed, double raw_follow_time_headway,
                                const std::optional<double>& cutin_duration) {
  double follow_time_headway =
      raw_follow_time_headway * KFollowTimeHeadwayGainPlf(av_speed);
  follow_time_headway =
      std::clamp(follow_time_headway, KMinHeadwayTime, raw_follow_time_headway);
  if (!cutin_duration.has_value()) {
    return follow_time_headway;
  }
  constexpr double kMinCutinTimeHeadway = 0.6;  // s.
  const PiecewiseLinearFunction<double, double> follow_time_rel_cutin_time_plf =
      {{0.0, 5.0},
       {std::min(kMinCutinTimeHeadway, follow_time_headway),
        follow_time_headway}};
  return follow_time_rel_cutin_time_plf(*cutin_duration);
}

void FillOverlapStateLonBuffer(
    const std::string& id,
    const std::vector<const StBoundaryWithDecision*>& st_boundaries_wd,
    double time, const std::vector<std::optional<double>>& standstills,
    double follow_time_headway,
    const PiecewiseLinearFunction<double>& follow_rel_speed_gain_plf,
    double lead_time_headway, double av_speed,
    StBoundaryProto::ProtectionType protection_type,
    const SpeedVector& preliminary_speed,
    const std::optional<double>& cutin_duration, double max_weak_t,
    const std::optional<double>& stationary_min_s, LaneChangeStage lc_stage,
    double lonbuffer_coeffi_match_lc_style, double current_time,
    const LcFinishedTimeProto* lc_finished_time,
    double follow_dist_shrink_ratio, ClassifiedOverlapState* overlap_states) {
  CHECK_NOTNULL(overlap_states);
  constexpr double kEgoFullStopSpeed = 0.1;       // m/s
  constexpr double kLcFinishedHoldingtime = 3.0;  // s
  constexpr double kGapStandstill = 3.0;          // m.
  for (int type = 0; type < overlap_states->size(); ++type) {
    auto& state = (*overlap_states)[type];
    if (!state.has_value()) {
      continue;
    }
    if (protection_type == StBoundaryProto::LANE_CHANGE_GAP) {
      state->lon_buffer = kGapStandstill;
      continue;
    }
    const double obj_speed = std::max(0.0, state->speed);
    const bool is_large_vehicle_blind_spot =
        (protection_type == StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT);
    double dp_speed = av_speed;
    if (!preliminary_speed.empty()) {
      dp_speed = preliminary_speed.EvaluateByTime(time)
                     .value_or(preliminary_speed.back())
                     .v();
    }
    double final_follow_time_headway =
        ComputeFollowTimeHeadway(dp_speed, follow_time_headway, cutin_duration);
    switch (type) {
      case MOVING_FOLLOW: {
        // filter the lon_buffer for the yield_time
        //        if (time > max_weak_t) {
        //          state->lon_buffer = 0;
        //          state->no_weak_buffer = true;
        //          break;
        //        }
        const auto& standstill_dist = standstills[MOVING_FOLLOW];
        CHECK(standstill_dist.has_value());
        if (is_large_vehicle_blind_spot) {
          state->lon_buffer = *standstill_dist;
        } else {
          state->lon_buffer =  // obj_speed -> dp_speed 20250612
              (dp_speed * final_follow_time_headway + *standstill_dist) *
              follow_rel_speed_gain_plf(obj_speed - av_speed);
          if (av_speed < kEgoFullStopSpeed) {
            constexpr double kMovingFollowDeadzoneFactor = 0.3;
            state->lon_buffer += time * kMovingFollowDeadzoneFactor;
          }
        }
        if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
          state->lon_buffer *= lonbuffer_coeffi_match_lc_style;
        }
        if (lc_finished_time != nullptr &&
            lc_finished_time->has_prev_coeffi_match_lc_style() &&
            lc_finished_time->has_prev_lc_executing_time()) {
          const double curr_lc_finished_holdingtime =
              current_time - lc_finished_time->prev_lc_executing_time();
          if (curr_lc_finished_holdingtime <= kLcFinishedHoldingtime) {
            const double LcFinishedTimeGain =
                curr_lc_finished_holdingtime / kLcFinishedHoldingtime;
            state->lon_buffer *=
                Lerp(lc_finished_time->prev_coeffi_match_lc_style(), 1.0,
                     LcFinishedTimeGain);
          }
        }
        state->lon_buffer = std::max(
            state->lon_buffer * follow_dist_shrink_ratio, *standstill_dist);
        break;
      }
      case MOVING_LEAD: {
        const auto standstill_dist = standstills[MOVING_LEAD];
        CHECK(standstill_dist.has_value());
        if (is_large_vehicle_blind_spot) {
          state->lon_buffer = *standstill_dist;
        } else {
          state->lon_buffer =
              std::max((obj_speed * lead_time_headway + *standstill_dist) *
                           KOvertakeRelSpeedGainPlf(av_speed - obj_speed),
                       *standstill_dist);
        }
        if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
          constexpr double kComfortableAcc = 0.5;  // m/s^2.
          const double const_accel_dist =
              (av_speed + 0.5 * kComfortableAcc * time) * time;
          const double comfortable_upper_bound =
              std::min(state->bound + state->lon_buffer, const_accel_dist);
          constexpr double kMaxOvertakeDistForLc = 10.0;  // Meter.
          state->lon_buffer =
              std::clamp(comfortable_upper_bound - state->bound,
                         *standstill_dist, kMaxOvertakeDistForLc);
        }
        break;
      }
      case SpeedOptimizerObjectType::STATIONARY: {
        CHECK(standstills[SpeedOptimizerObjectType::STATIONARY].has_value());
        constexpr double kStationaryDeadzoneSpeed = 3.0;  // m/s
        constexpr double kStationaryDeadzone = 1.5;       // m
        constexpr double kMinSGapBuffer = 3.0;            // m.
        constexpr double kStationaryHeadway = 1.0;        // s
        state->lon_buffer =
            *standstills[SpeedOptimizerObjectType::STATIONARY] +
            std::max(0.0, (dp_speed - kStationaryDeadzoneSpeed)) *
                kStationaryHeadway;
        if (av_speed < kEgoFullStopSpeed) {
          double min_s_gap = 0.0;
          if (stationary_min_s.has_value()) {
            min_s_gap = std::clamp(*stationary_min_s - state->lon_buffer, 0.0,
                                   kMinSGapBuffer);
          }
          state->lon_buffer += kStationaryDeadzone + min_s_gap;
        }
        break;
      }
    }
  }
}

ClassifiedOverlapState IntegrateAndClassifyOverlapState(
    const std::vector<const StBoundaryWithDecision*>& st_boundaries,
    double time, double prediction_impact_factor,
    const std::optional<std::pair<std::string, double>>& maybe_hard_brake_obj,
    const SpeedFinderParamsProto& speed_finder_params) {
  // The lambda function to integrate overlap state.
  const auto integrate_overlap_state =
      [](const ObjectOverlapState& new_state,
         std::optional<ObjectOverlapState>* raw_state) {
        CHECK_NOTNULL(raw_state);
        if (!raw_state->has_value()) *raw_state = ObjectOverlapState{};
        (*raw_state)->bound += new_state.bound * new_state.prob;
        (*raw_state)->speed += new_state.speed * new_state.prob;
        (*raw_state)->delta_speed_factor +=
            new_state.delta_speed_factor * new_state.prob;
        (*raw_state)->prob += new_state.prob;
      };

  ClassifiedOverlapState classified_overlap_states(kObjectTypeNum);
  double min_lower_bound = std::numeric_limits<double>::max();
  double delta_speed_factor = 0.0;
  const auto& speed_opt_params = speed_finder_params.speed_optimizer_params();
  for (const StBoundaryWithDecision* stb_wd : st_boundaries) {
    const StBoundary* stb = stb_wd->st_boundary();
    const auto& raw_st = stb_wd->raw_st_boundary();
    double acc = 0.0;
    if (raw_st) {
      acc = raw_st->obj_pose_info().a();
    }
    const auto s_range = stb->GetBoundarySRange(time);
    if (!s_range.has_value()) continue;
    double bound = 0.0;
    const auto decision = stb_wd->decision_type();
    if (decision == StBoundaryProto::YIELD ||
        decision == StBoundaryProto::FOLLOW) {
      bound = s_range->second;
      min_lower_bound = std::min(min_lower_bound, s_range->second);
      if (decision == StBoundaryProto::FOLLOW) {
        const double nomal_delta_speed_factor =
            speed_opt_params.delta_speed_factor();
        const double middle_delta_speed_factor =
            speed_opt_params.middle_follow_delta_speed_factor();
        const double hard_delta_speed_factor =
            speed_opt_params.hard_follow_delta_speed_factor();
        double max_speed_factor = middle_delta_speed_factor;
        if ((!speed_finder_params.enable_hard_brake_for_every_obj()) &&
            maybe_hard_brake_obj.has_value() && stb->object_id().has_value() &&
            maybe_hard_brake_obj.value().first == stb->object_id().value()) {
          max_speed_factor =
              Lerp(middle_delta_speed_factor, 0.0, hard_delta_speed_factor,
                   -0.8, maybe_hard_brake_obj.value().second, true);
          delta_speed_factor =
              Lerp(nomal_delta_speed_factor,
                   speed_opt_params.hard_brake_start_acc(), max_speed_factor,
                   speed_opt_params.hard_brake_end_acc(), acc, true);
        } else {
          if (speed_finder_params.enable_hard_brake_for_every_obj()) {
            delta_speed_factor =
                Lerp(nomal_delta_speed_factor,
                     speed_opt_params.hard_brake_start_acc(),
                     hard_delta_speed_factor,
                     speed_opt_params.hard_brake_end_acc(), acc, true);
          } else {
            delta_speed_factor = Lerp(nomal_delta_speed_factor, -2.0,
                                      max_speed_factor, -4.0, acc, true);
          }
        }
      } else {
        delta_speed_factor = speed_opt_params.delta_speed_factor();
      }

      if (raw_st) {
        if (raw_st->object_type() == StBoundaryProto::VIRTUAL &&
            raw_st->is_traffic_light()) {
          delta_speed_factor = speed_opt_params.stop_line_delta_speed_factor();
          double factor = Lerp(
              1.0, speed_opt_params.stop_line_delta_speed_enable_distance(),
              speed_opt_params.min_stop_line_decay(),
              speed_opt_params.stop_line_delta_speed_disable_distance(),
              raw_st->bottom_left_point().s(), true);
          delta_speed_factor = delta_speed_factor * factor;
        }
      }
    } else {
      bound = s_range->first;
    }
    const auto obj_speed = stb->GetStBoundarySpeedAtT(time);
    CHECK(obj_speed.has_value());

    const ObjectOverlapState tmp_overlap_state = {
        .bound = bound,
        .speed = *obj_speed,
        .prob = stb->probability(),
        .delta_speed_factor = delta_speed_factor};
    if (stb->is_stationary()) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::STATIONARY]);
    } else if (decision == StBoundaryProto::YIELD ||
               decision == StBoundaryProto::FOLLOW) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::MOVING_FOLLOW]);
    } else if (decision == StBoundaryProto::OVERTAKE) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::MOVING_LEAD]);
    }
  }

  // Normalize and adjust by `prediction_impact_factor`.
  for (int type = 0; type < classified_overlap_states.size(); ++type) {
    auto& state = classified_overlap_states[type];
    if (!state.has_value()) continue;
    const double prob = state->prob;
    CHECK_GT(prob, 0.0);
    state->bound /= prob;
    state->speed /= prob;
    if (type == SpeedOptimizerObjectType::MOVING_FOLLOW ||
        type == SpeedOptimizerObjectType::STATIONARY) {
      state->bound =
          Lerp(state->bound, min_lower_bound, prediction_impact_factor);
    }
  }
  return classified_overlap_states;
}

std::optional<std::pair<std::string, double>> MaybeHardBrakeObjInfo(
    const std::map<std::string, std::vector<const StBoundaryWithDecision*>>&
        st_boundaries_wd_map) {
  constexpr double kFrontLatThres = 0.2;
  std::vector<const StBoundaryWithDecision*> front_vehicles;
  std::string nearest_follow_obj = "";
  double min_dis = DBL_MAX;
  double nearest_follow_speed = 0.0;
  double min_stop_line_dis = DBL_MAX;

  // 1.0 find nearest follow obj, all front obj, all stop line
  for (const auto& [id, st_boundaries] : st_boundaries_wd_map) {
    if (st_boundaries.empty()) continue;
    const auto& stb_wd = st_boundaries.front();
    if (!stb_wd) continue;
    const auto& raw_st_boundary = stb_wd->raw_st_boundary();
    const auto& obj_sl_info = raw_st_boundary->obj_sl_info();
    if (!obj_sl_info.has_value()) {
      continue;
    }
    if (!raw_st_boundary) continue;
    if ((raw_st_boundary->object_type() == StBoundaryProto::VEHICLE ||
         raw_st_boundary->object_type() == StBoundaryProto::CYCLIST ||
         raw_st_boundary->object_type() == StBoundaryProto::PEDESTRIAN) &&
        (stb_wd->decision_type() == StBoundaryProto::YIELD ||
         stb_wd->decision_type() == StBoundaryProto::FOLLOW) &&
        std::abs(obj_sl_info->dl) < kFrontLatThres && obj_sl_info->ds > 1e-2) {
      front_vehicles.emplace_back(stb_wd);
      if (stb_wd->decision_type() == StBoundaryProto::FOLLOW &&
          obj_sl_info->ds > 0.0 && stb_wd->object_id().has_value()) {
        if (obj_sl_info->ds < min_dis) {
          nearest_follow_obj = stb_wd->object_id().value();
          min_dis = obj_sl_info->ds;
          nearest_follow_speed = stb_wd->obj_pose_info().v();
        }
      }
    }
    if (raw_st_boundary->is_traffic_light() &&
        raw_st_boundary->bottom_left_point().s() < min_stop_line_dis) {
      min_stop_line_dis = raw_st_boundary->bottom_left_point().s();
    }
  }

  // 2.0 choose obj which is front of main target
  if (nearest_follow_obj.empty()) {
    return std::nullopt;
  } else if (min_dis > min_stop_line_dis) {
    return std::nullopt;
  }
  front_vehicles.erase(
      std::remove_if(
          front_vehicles.begin(), front_vehicles.end(),
          [&nearest_follow_obj,
           &min_dis](const StBoundaryWithDecision* stb_wd) {
            const auto& obj_sl_info = stb_wd->raw_st_boundary()->obj_sl_info();
            CHECK(obj_sl_info.has_value());
            return stb_wd->object_id().has_value() &&
                   (stb_wd->object_id().value() == nearest_follow_obj ||
                    obj_sl_info->ds < min_dis);
          }),
      front_vehicles.end());

  // 3.0 sort obj
  std::sort(front_vehicles.begin(), front_vehicles.end(),
            [](const StBoundaryWithDecision* stb_wd1,
               const StBoundaryWithDecision* stb_wd2) {
              const auto& sl_info1 = stb_wd1->raw_st_boundary()->obj_sl_info();
              const auto& sl_info2 = stb_wd2->raw_st_boundary()->obj_sl_info();
              return sl_info1->ds < sl_info2->ds;
            });

  // 4.0 calculate acc if main target will stop before low speed obj or stop
  // line
  double follow_brake_a_with_front = 0.0;
  double follow_brake_a_with_stopline = 0.0;
  if (!front_vehicles.empty() &&
      front_vehicles.front()->obj_pose_info().v() < Kph2Mps(2.0)) {
    // follow_brake_a_with_front =
    //     (Sqr(front_vehicles.front()->obj_pose_info().v()) -
    //      Sqr(nearest_follow_speed)) /
    //     (std::max(1e-2, 2.0 * (front_vehicles.front()->ds()-6.0 - min_dis)));

    const auto& obj_sl_info =
        front_vehicles.front()->raw_st_boundary()->obj_sl_info();
    CHECK(obj_sl_info.has_value());
    follow_brake_a_with_front =
        -1.0 * Sqr(nearest_follow_speed) /
        (std::max(1e-2, 2.0 * (obj_sl_info->ds - 6.0 - min_dis)));
  }
  follow_brake_a_with_stopline =
      -1.0 * Sqr(nearest_follow_speed) /
      (std::max(1e-2, 2.0 * (min_stop_line_dis - 6.0 - min_dis)));

  // 5.0 output
  std::pair<std::string, double> obj_info = std::make_pair(
      nearest_follow_obj,
      std::min(follow_brake_a_with_front, follow_brake_a_with_stopline));
  return std::optional<std::pair<std::string, double>>(obj_info);
}

bool IsCrossObject(int plan_id, const StBoundary& raw_st_boundary,
                   const DiscretizedPath& path) {
  const auto& overlap_meta = raw_st_boundary.overlap_meta();
  const auto& overlap_infos = raw_st_boundary.overlap_infos();
  if (!overlap_meta.has_value() || overlap_infos.empty() || path.empty()) {
    return false;
  }
  if (overlap_meta->pattern() == StOverlapMetaProto::CROSS) {
    return true;
  }
  constexpr double kLeaveTimeThres = 1.5;  // s.
  if (overlap_meta->pattern() == StOverlapMetaProto::LEAVE &&
      raw_st_boundary.max_t() < kLeaveTimeThres) {
    const int av_start_idx = std::clamp(overlap_infos.front().av_start_idx, 0,
                                        static_cast<int>(path.size() - 1));
    if (std::abs(NormalizeAngle(raw_st_boundary.obj_pose_info().theta() -
                                path[av_start_idx].theta())) > M_PI_4) {
      return true;
    }
  }
  return false;
}

ClassifiedObjects GenerateIntegratedClassifiedObjects(
    int plan_id,
    const std::vector<const StBoundaryWithDecision*>& st_boundaries_wd,
    const std::optional<std::pair<std::string, double>>& maybe_hard_brake_obj,
    const SpacetimeTrajectoryManager& traj_mgr, const DiscretizedPath& path,
    const std::string& id, double delta_t, double plan_total_time,
    double av_speed, const SpeedFinderParamsProto& speed_finder_params,
    const SpeedVector& preliminary_speed,
    const CutinHistoryProto* cutin_history, double current_time,
    LaneChangeStage lc_stage, double lonbuffer_coeffi_match_lc_style,
    const LcFinishedTimeProto* lc_finished_time, bool need_accel_for_gap) {
  CHECK(!st_boundaries_wd.empty());

  const auto protection_type =
      st_boundaries_wd[0]->st_boundary()->protection_type();
  const auto object_id = st_boundaries_wd[0]->st_boundary()->object_id();

  double min_t = std::numeric_limits<double>::max();
  double max_t = -std::numeric_limits<double>::max();
  double max_weak_t = -std::numeric_limits<double>::max();
  std::vector<std::optional<double>> standstills(kObjectTypeNum);
  for (const StBoundaryWithDecision* stb_wd : st_boundaries_wd) {
    const StBoundary* st_boundary = stb_wd->st_boundary();
    // Each protection type of st-boundary is handled individually.
    CHECK_EQ(st_boundary->protection_type(), protection_type);
    min_t = std::min(min_t, st_boundary->min_t());
    max_t = std::max(max_t, st_boundary->max_t());
    max_weak_t =
        std::max(max_weak_t, st_boundary->max_t() - stb_wd->yield_time());
    if (st_boundary->is_stationary()) {
      standstills[SpeedOptimizerObjectType::STATIONARY] =
          stb_wd->follow_standstill_distance();
    } else if (stb_wd->decision_type() == StBoundaryProto::YIELD ||
               stb_wd->decision_type() == StBoundaryProto::FOLLOW) {
      standstills[SpeedOptimizerObjectType::MOVING_FOLLOW] =
          stb_wd->follow_standstill_distance();
    } else if (stb_wd->decision_type() == StBoundaryProto::OVERTAKE) {
      standstills[SpeedOptimizerObjectType::MOVING_LEAD] =
          stb_wd->lead_standstill_distance();
    }
  }

  // Generate rel_speed_gain_plf.
  PiecewiseLinearFunction<double> follow_rel_speed_gain_plf = {
      {-1.0, 1.0}, {1.0, 1.0}};  // Default value.
  for (const StBoundaryWithDecision* stb_wd : st_boundaries_wd) {
    const StBoundary* raw_st_boundary = stb_wd->raw_st_boundary();
    if (stb_wd->decision_type() == StBoundaryProto::OVERTAKE ||
        raw_st_boundary->is_stationary() || !object_id.has_value()) {
      continue;
    }
    if (raw_st_boundary->object_type() == StBoundaryProto::PEDESTRIAN ||
        IsCrossObject(plan_id, *raw_st_boundary, path)) {
      follow_rel_speed_gain_plf =
          PiecewiseLinearFunction<double>{/*x=*/{-8.0, 8.0},
                                          /*y=*/{0.8, 0.2}};
    } else {
      const double obj_v = raw_st_boundary->obj_pose_info().v();
      follow_rel_speed_gain_plf = PiecewiseLinearFunction<double>{
          /*x=*/{1.5, 4.0},
          /*y=*/{1.0, kAvSpeedFollowDistanceLowestGainPlf(obj_v)}};
    }
    break;
  }

  bool is_cipv = false;
  for (const StBoundaryWithDecision* stb_wd : st_boundaries_wd) {
    if (stb_wd->is_cipv() || stb_wd->is_stay_cipv()) {
      is_cipv = true;
      break;
    }
  }

  const auto& opt_params = speed_finder_params.speed_optimizer_params();
  const double lead_time_headway = speed_finder_params.lead_time_headway();
  const int knot_num = opt_params.knot_num();
  const double prediction_impact_factor = opt_params.prediction_impact_factor();

  std::vector<std::vector<std::optional<ObjectOverlapState>>>
      classified_entire_time_overlap_states(kObjectTypeNum);
  std::vector<std::optional<int>> classified_first_overlap_idx(kObjectTypeNum);
  max_t = std::min(max_t, plan_total_time);
  const int start_idx = FloorToInt(min_t / delta_t);
  const int end_idx = FloorToInt(max_t / delta_t);
  const double follow_time_headway =
      st_boundaries_wd[0]->st_boundary()->is_large_vehicle()
          ? speed_finder_params.large_vehicle_follow_time_headway()
          : speed_finder_params.follow_time_headway();

  Log2DDS::LogDataV0("follow_time_headway",
                     absl::StrCat("follow_time_headway: ",
                                  speed_finder_params.follow_time_headway()));

  std::optional<double> cutin_duration;
  if (cutin_history != nullptr && cutin_history->has_object_id() &&
      object_id.has_value() && cutin_history->object_id() == *object_id) {
    cutin_duration = current_time - cutin_history->cutin_start_time();
  }

  std::optional<double> stationary_min_s;
  for (const auto& stb : st_boundaries_wd) {
    if (stb->raw_st_boundary()->is_stationary()) {
      stationary_min_s = stb->raw_st_boundary()->min_s();
      break;
    }
  }

  double follow_dist_shrink_ratio = 1.0;
  if (is_cipv && need_accel_for_gap > 0.0) {
    follow_dist_shrink_ratio = 0.5;
  }

  for (int idx = start_idx; idx <= end_idx; ++idx) {
    auto overlap_states_at_t = IntegrateAndClassifyOverlapState(
        st_boundaries_wd, idx * delta_t, prediction_impact_factor,
        maybe_hard_brake_obj, speed_finder_params);
    // Fill lon buffer.
    FillOverlapStateLonBuffer(
        id, st_boundaries_wd, idx * delta_t, standstills, follow_time_headway,
        follow_rel_speed_gain_plf, lead_time_headway, av_speed, protection_type,
        preliminary_speed, cutin_duration, max_weak_t, stationary_min_s,
        lc_stage, lonbuffer_coeffi_match_lc_style, current_time,
        lc_finished_time, follow_dist_shrink_ratio, &overlap_states_at_t);
    CHECK_LT(idx, knot_num);
    for (int type = 0; type < overlap_states_at_t.size(); ++type) {
      auto state = overlap_states_at_t[type];
      if (state.has_value()) {
        auto& entire_time_states = classified_entire_time_overlap_states[type];
        if (entire_time_states.empty()) entire_time_states.resize(knot_num);
        entire_time_states[idx] = state;
        auto& first_overlap_idx = classified_first_overlap_idx[type];
        if (!first_overlap_idx.has_value()) {
          first_overlap_idx = idx;
        }
      }
    }
  }

  // Construct speed optimizer objects.
  const auto* stb = st_boundaries_wd.front()->st_boundary();
  const auto object_type = stb->object_type();
  const auto source_type = stb->source_type();
  ClassifiedObjects classified_objects(kObjectTypeNum);
  for (int type = 0; type < classified_objects.size(); ++type) {
    auto& states = classified_entire_time_overlap_states[type];
    if (!states.empty()) {
      const auto standstill = standstills[type];
      CHECK(standstill.has_value());
      const auto first_overlap_idx = classified_first_overlap_idx[type];
      CHECK(first_overlap_idx.has_value());
      classified_objects[type] = SpeedOptimizerObject(
          std::move(states), delta_t, id, *standstill, object_type, source_type,
          protection_type, *first_overlap_idx);
    }
  }
  return classified_objects;
}

}  // namespace

SpeedOptimizerObjectManager::SpeedOptimizerObjectManager(
    int plan_id,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr, const DiscretizedPath& path,
    double av_speed, double plan_total_time, double plan_time_interval,
    const SpeedFinderParamsProto& speed_finder_params,
    const SpeedVector& preliminary_speed,
    const CutinHistoryProto* cutin_history, double current_time,
    LaneChangeStage lc_stage, double coeffi_match_lc_style,
    const LcFinishedTimeProto* lc_finished_time, bool need_accel_for_gap) {
  std::map<std::string, std::vector<const StBoundaryWithDecision*>>
      st_boundary_map;
  for (const StBoundaryWithDecision& stb_wd : st_boundaries_with_decision) {
    if (stb_wd.decision_type() == StBoundaryProto::UNKNOWN ||
        stb_wd.decision_type() == StBoundaryProto::IGNORE) {
      continue;
    }
    const auto id = GetStBoundaryIntegrationId(*stb_wd.st_boundary());
    st_boundary_map[id].push_back(&stb_wd);
  }

  objects_.resize(kObjectTypeNum);
  const auto& maybe_hard_brake_obj_info =
      MaybeHardBrakeObjInfo(st_boundary_map);
  if (maybe_hard_brake_obj_info.has_value()) {
    Log2DDS::LogDataV2(
        "follow_factor",
        absl::StrCat("object_id: ", maybe_hard_brake_obj_info.value().first,
                     " ref_acc: ", maybe_hard_brake_obj_info.value().second));
    if (maybe_hard_brake_obj_info.value().second < -0.5) {
      attention_obj_id_ = maybe_hard_brake_obj_info.value().first;
    }
  }

  for (const auto& [id, st_boundaries] : st_boundary_map) {
    auto classified_objects = GenerateIntegratedClassifiedObjects(
        plan_id, st_boundaries, maybe_hard_brake_obj_info, traj_mgr, path, id,
        plan_time_interval, plan_total_time, av_speed, speed_finder_params,
        preliminary_speed, cutin_history, current_time, lc_stage,
        coeffi_match_lc_style, lc_finished_time, need_accel_for_gap);
    for (int type = 0; type < classified_objects.size(); ++type) {
      auto object = classified_objects[type];
      if (object.has_value()) {
        objects_[type].push_back(std::move(*object));
      }
    }
  }
}

}  // namespace st::planning
