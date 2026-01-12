

#include "planner/speed_optimizer/dp_svt_cost.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <optional>
#include <ostream>
#include <utility>

#include "gflags/gflags.h"

//#include "lite/check.h"
//#include "lite/logging.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "plan_common/maps/st_boundary.h"
#include "planner/speed_optimizer/svt_point.h"
#include "plan_common/util/map_util.h"

DECLARE_bool(enable_sampling_dp_reference_speed);

namespace st::planning {
namespace {
constexpr double kMaxSpeedLimitDiff = 10.0;  // m/s
constexpr double kEps = 0.0001;
constexpr double kBillion = 1e9;
const PiecewiseLinearFunction<double> kObjectWeightDecayPlf({0, 3, 7},
                                                            {1.0, 0.7, 0.4});
const PiecewiseLinearFunction<double> kOvertakeRelativeSpeedGainPlf(
    {-3.0, 0.0, 2.0, 3.0}, {1.0, 0.6, 0.4, 0.3});

using DecisionType = StBoundaryProto::DecisionType;

DecisionType MakeStBoundaryDecision(double s, double s_lower, double s_upper,
                                    double follow_lead_ratio = 0.5) {
  if (s < s_lower + follow_lead_ratio * (s_upper - s_lower)) {
    return StBoundaryProto::YIELD;
  } else {
    return StBoundaryProto::OVERTAKE;
  }
}

#define DEBUG_DP_DECISION_COST (0)

double GetExternalPedestrianCostForLowerSpeed(
    double agent_dist_to_collision, double agent_v,
    double av_dist_to_leave_collision, double av_speed,
    StBoundaryProto_ObjectType object_type) {
  constexpr double kPedestrianFollowDist = 2.0;
  constexpr double kAvMaxVelForPedestrianScene = Kph2Mps(10.0);
  double pedestrian_risk_cost = 0;
  const PiecewiseLinearFunction<double> kPedestrianCollisionRiskPlf(
      {-10.0 - kEps, -10.0, -7.0, -5.5, 0.0},
      {kBillion, 100.0, 90.0, 80.0, 10.0});

  if (StBoundaryProto::PEDESTRIAN != object_type) return pedestrian_risk_cost;
  if (av_speed >= kAvMaxVelForPedestrianScene) return pedestrian_risk_cost;

  double agent_t =
      std::max(agent_dist_to_collision - kPedestrianFollowDist, 0.0) /
      (agent_v + kEps);
  double av_through_dist = av_speed * agent_t;

  double delta_dist = av_through_dist - av_dist_to_leave_collision;

  pedestrian_risk_cost = kPedestrianCollisionRiskPlf(delta_dist);

  return pedestrian_risk_cost;
}

double CalcYieldResult(double ego_vel, double obj_t,
                       double ego_dist_to_collision) {
  if (obj_t <= kEps || ego_vel <= kEps || ego_dist_to_collision <= kEps) {
    return 0.0;
  }
  const double ego_t = ego_dist_to_collision / (ego_vel + kEps);
  constexpr double kTimeThres = 2.0;  // s.
  if (ego_t - obj_t >= kTimeThres) return 0.0;
  return (ego_dist_to_collision - ego_vel * obj_t) / (0.5 * Sqr(obj_t));
}

double GetStBoundaryCost(
    DecisionType decision, double av_speed, const StBoundary& st_boundary,
    double s, double s_lower, double s_upper, double first_s_upper,
    const double first_object_v, double obj_acc, double t,
    double follow_standstill_distance,
    const PiecewiseLinearFunction<double>& follow_distance_rel_speed_plf,
    const SpeedFinderParamsProto& speed_finder_params,
    const SpeedFinderParamsProto::SamplingDpSpeedParamsProto& params,
    bool enable_add_collision_risk_cost_for_dp, double delta_t,
    double agent_vel, double agent_dist_to_collision, double ego_yield_acc,
    double obj_yield_acc, double av_dist_to_collision,
    double curr_comfort_brake_s) {
  double cost = 0.0;
  constexpr double kOvertakeObjYieldDecel = -0.5;  // m/ss.
  const PiecewiseLinearFunction<double> kCollisionRiskPlf(
      {-2.5, -1.0, 0.0, 5.0, 5.0 - kEps}, {0.0, 40.0, 80, 100.0, kBillion});
  const PiecewiseLinearFunction<double> kRationalityPlf(
      {-15.0 - kEps, -15.0, -5.0, -2.0, 0.0}, {kBillion, 100.0, 50, 20, 0.0});

  if (decision == StBoundaryProto::YIELD ||
      decision == StBoundaryProto::FOLLOW) {
    double follow_distance_s = 0.0;
    double comfort_follow_distance_s = 0.0;
    if (st_boundary.source_type() == StBoundarySourceTypeProto::VIRTUAL ||
        st_boundary.source_type() ==
            StBoundarySourceTypeProto::IMPASSABLE_BOUNDARY ||
        st_boundary.source_type() == StBoundarySourceTypeProto::PATH_BOUNDARY ||
        st_boundary.protection_type() ==
            StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT) {
      follow_distance_s = follow_standstill_distance;
    } else {
      const auto object_v = st_boundary.GetStBoundarySpeedAtT(t);
      CHECK(object_v.has_value());
      const double follow_time_headway =
          st_boundary.is_large_vehicle()
              ? speed_finder_params.large_vehicle_follow_time_headway()
              : speed_finder_params.follow_time_headway();
      follow_distance_s = follow_time_headway * std::max(0.0, *object_v) +
                          follow_standstill_distance;
      // TODO： Consider to use speed of st graph point to compute
      // relative speed.
      const double rel_speed_gain =
          follow_distance_rel_speed_plf(*object_v - av_speed);
      follow_distance_s = std::max(follow_distance_s * rel_speed_gain,
                                   follow_standstill_distance);
      // Follow distance considering comfort braking
      comfort_follow_distance_s =
          std::max(s_lower - curr_comfort_brake_s, follow_standstill_distance);
      follow_distance_s =
          std::min(follow_distance_s, comfort_follow_distance_s);
    }

    if (enable_add_collision_risk_cost_for_dp) {
      if (av_speed > Kph2Mps(15.0) && ego_yield_acc < 0 &&
          av_dist_to_collision <= 10.0 && agent_dist_to_collision <= 10.0) {
        cost = kRationalityPlf(ego_yield_acc - obj_yield_acc);
      }
    }

    if (s + follow_distance_s > s_lower) {
      const double s_diff = follow_distance_s - s_lower + s;
      cost +=
          st_boundary.probability() * params.object_weight() * s_diff * s_diff;
    }
  } else if (decision == StBoundaryProto::OVERTAKE) {
    double object_v_cal = 0.0;
    double s_upper_cal = 0.0;
    const auto object_v = st_boundary.GetStBoundarySpeedAtT(t);
    CHECK(object_v.has_value());

    if (obj_acc > kOvertakeObjYieldDecel) {
      // Assumed obj slows down (recalculate s_upper and object_v)
      // TODO(shi.ping5): Need refactor.
      double delta_t = t - st_boundary.min_t();
      double object_v_dec =
          std::clamp(first_object_v + kOvertakeObjYieldDecel * delta_t, 0.0,
                     first_object_v);
      double delta_t_obj_dece_to_zero =
          (0.0 - first_object_v) / (kOvertakeObjYieldDecel + kEps);
      double delta_s_upper_dec =
          delta_t_obj_dece_to_zero < delta_t
              ? first_object_v * delta_t_obj_dece_to_zero +
                    0.5 * kOvertakeObjYieldDecel * delta_t_obj_dece_to_zero *
                        delta_t_obj_dece_to_zero
              : first_object_v * delta_t +
                    0.5 * kOvertakeObjYieldDecel * delta_t * delta_t;
      double s_upper_dec = first_s_upper + delta_s_upper_dec;
      object_v_cal = object_v_dec;
      s_upper_cal = s_upper_dec;
    } else {
      object_v_cal = *object_v;
      s_upper_cal = s_upper;
    }

    // calculate overtake_distance_s
    double overtake_distance_s =
        st_boundary.protection_type() ==
                StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT
            ? 0.0
            : speed_finder_params.lead_time_headway() *
                      std::max(object_v_cal, 0.0) +
                  speed_finder_params.lead_standstill_distance();

    // add relative speed gain
    const double overtake_relative_speed_gain =
        kOvertakeRelativeSpeedGainPlf(av_speed - object_v_cal);
    overtake_distance_s =
        std::max(overtake_distance_s * overtake_relative_speed_gain,
                 speed_finder_params.lead_standstill_distance());

    if (enable_add_collision_risk_cost_for_dp) {
      cost = kCollisionRiskPlf(delta_t);
    }

    constexpr double kDefaultPedestrianLength = 0.5;  // Meter.
    cost += GetExternalPedestrianCostForLowerSpeed(
        agent_dist_to_collision, agent_vel,
        st_boundary.min_s() + kDefaultPedestrianLength, av_speed,
        st_boundary.object_type());

    // Object Weight Decay Because of the accuracy of the prediction
    const double object_weight_decay = kObjectWeightDecayPlf(t);

    if (s < s_upper_cal + overtake_distance_s) {
      const double s_diff = overtake_distance_s + s_upper_cal - s;
      cost += st_boundary.probability() * object_weight_decay *
              params.object_weight() * s_diff * s_diff;
    }
  } else {
    // We must have lead or follow decision here.
    LOG_FATAL << "Bad unknown decision for st-boundary " << st_boundary.id()
              << " at s = " << s << " t = " << t;
  }
  return cost;
}

}  // namespace

DpSvtCost::DpSvtCost(const SpeedFinderParamsProto* speed_finder_params,
                     double total_t, double total_s, double init_v,
                     const std::vector<StBoundaryWithDecision*>*
                         sorted_st_boundaries_with_decision,
                     const SpacetimeTrajectoryManager& st_traj_mgr)
    : speed_finder_params_(CHECK_NOTNULL(speed_finder_params)),
      params_(&speed_finder_params->sampling_dp_speed_params()),
      sorted_st_boundaries_with_decision_(
          CHECK_NOTNULL(sorted_st_boundaries_with_decision)),
      unit_t_(params_->unit_t()),
      total_s_(total_s),
      st_traj_mgr_(st_traj_mgr) {
  int index = 0;
  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    boundary_map_[st_boundary_with_decision->id()] = index++;
  }
  const auto dimension_t = CeilToInt(total_t / unit_t_) + 1;
  boundary_cost_.resize(sorted_st_boundaries_with_decision_->size());
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  // Get the map of original st-boundaries.
  for (const auto* st_boundary_wd : *sorted_st_boundaries_with_decision_) {
    if (st_boundary_wd->raw_st_boundary()->is_protective()) {
      continue;
    }
    original_st_boundary_wd_map_[st_boundary_wd->id()] = st_boundary_wd;
  }

  exceed_speed_unit_cost_ =
      params_->exceed_speed_penalty() * params_->speed_weight() * unit_t_;
  low_speed_unit_cost_ =
      params_->low_speed_penalty() * params_->speed_weight() * unit_t_;

  // Compute comfortable brake s.
  comfortable_brake_s_.clear();
  comfortable_brake_s_.reserve(dimension_t);
  comfortable_brake_s_.push_back(0.0);
  constexpr double kComfortableBrakeAcc = -3.0;  // m/ss.
  const double max_brake_time = -init_v / kComfortableBrakeAcc;
  const double max_brake_s = init_v * max_brake_time +
                             0.5 * kComfortableBrakeAcc * Sqr(max_brake_time);
  for (int i = 1; i < dimension_t; ++i) {
    const double t = static_cast<double>(i) * unit_t_;
    const double comfortable_brake_s =
        t > max_brake_time ? max_brake_s
                           : init_v * t + 0.5 * kComfortableBrakeAcc * Sqr(t);
    comfortable_brake_s_.push_back(comfortable_brake_s);
  }
}

std::vector<SvtGraphPoint::StBoundaryDecision>
DpSvtCost::GetStBoundaryDecisionsForInitPoint(
    const SvtGraphPoint& svt_graph_point) {
  std::vector<SvtGraphPoint::StBoundaryDecision> st_boundary_decisions;
  const double s = svt_graph_point.point().s();  // Should be zero.
  const double t = svt_graph_point.point().t();  // Should be zero.

  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    const auto& st_boundary = *st_boundary_with_decision->st_boundary();
    if (t < st_boundary.min_t() || t > st_boundary.max_t()) {
      continue;
    }
    if (st_boundary_with_decision->decision_type() == StBoundaryProto::IGNORE) {
      // Skip computing cost for st_boundary that has a prior IGNORE decision.
      continue;
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    const int boundary_index = boundary_map_[st_boundary.id()];
    {
      absl::MutexLock lock(&boundary_cost_mutex_);
      if (boundary_cost_[boundary_index][svt_graph_point.index_t()].first <
          0.0) {
        const auto s_range = st_boundary.GetBoundarySRange(t);
        CHECK(s_range.has_value());
        boundary_cost_[boundary_index][svt_graph_point.index_t()] = *s_range;
        s_upper = s_range->first;
        s_lower = s_range->second;
      } else {
        s_upper =
            boundary_cost_[boundary_index][svt_graph_point.index_t()].first;
        s_lower =
            boundary_cost_[boundary_index][svt_graph_point.index_t()].second;
      }
      // Make decision based on relative position of s, s_lower and s_upper.
      const auto decision = MakeStBoundaryDecision(s, s_lower, s_upper);
      // Record decision on this st_boundary without prior decision.
      st_boundary_decisions.emplace_back(st_boundary.id(), decision);
      // Synchronize the decisions of protective st-boundary & corresponding
      // original st-boundary.
      if (st_boundary_with_decision->raw_st_boundary()->is_protective()) {
        const auto& protected_st_boundary_id =
            st_boundary_with_decision->raw_st_boundary()
                ->protected_st_boundary_id();
        if (protected_st_boundary_id.has_value()) {
          if (const auto original_st_boundary_wd = FindOrNull(
                  original_st_boundary_wd_map_, *protected_st_boundary_id);
              nullptr != original_st_boundary_wd &&
              (*original_st_boundary_wd)->decision_type() ==
                  StBoundaryProto_DecisionType_UNKNOWN) {
            st_boundary_decisions.emplace_back(*protected_st_boundary_id,
                                               decision);
          }
        }
      }
    }
  }
  return st_boundary_decisions;
}

void DpSvtCost::GetStBoundaryCostAndDecisions(
    const SvtGraphPoint& prev_svt_graph_point,
    const SvtGraphPoint& svt_graph_point, double av_speed,
    double* st_boundary_cost,
    std::vector<SvtGraphPoint::StBoundaryDecision>* st_boundary_decisions) {
  CHECK_NOTNULL(st_boundary_cost);
  CHECK_NOTNULL(st_boundary_decisions);
  CHECK(st_boundary_decisions->empty());
  constexpr double kDeltaVel = Kph2Mps(10.0);

  const double prev_s = prev_svt_graph_point.point().s();
  const double prev_t = prev_svt_graph_point.point().t();

  const int curr_t_index = svt_graph_point.index_t();
  const int pre_t_index = prev_svt_graph_point.index_t();
  CHECK_GE(curr_t_index, 0);
  CHECK_LE(curr_t_index, comfortable_brake_s_.size());
  double curr_comfort_brake_s = comfortable_brake_s_[curr_t_index];

  double cost = 0.0;
  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    if (st_boundary_with_decision->decision_type() == StBoundaryProto::IGNORE) {
      // Skip computing cost for st_boundary that has a prior IGNORE decision.
      continue;
    }

    double s = svt_graph_point.point().s();
    double t = svt_graph_point.point().t();
    const auto& st_boundary = *st_boundary_with_decision->st_boundary();
    if (t < st_boundary.min_t() ||
        (prev_t >= st_boundary.min_t() && t > st_boundary.max_t())) {
      continue;
    }

    // If overlap period is too small that located between current and previous
    // point, recalculate decision making s and t.
    if (t >= st_boundary.max_t()) {
      s = Lerp(prev_s, s, LerpFactor(prev_t, t, st_boundary.max_t()));
      t = st_boundary.max_t();
      const double pre_t = static_cast<double>(pre_t_index) * unit_t_;
      const double cur_t = static_cast<double>(curr_t_index) * unit_t_;
      curr_comfort_brake_s = Lerp(
          comfortable_brake_s_[pre_t_index], comfortable_brake_s_[curr_t_index],
          LerpFactor(pre_t, cur_t, st_boundary.max_t()));
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    double first_s_upper = 0.0;
    int boundary_index = boundary_map_[st_boundary.id()];
    {
      absl::MutexLock lock(&boundary_cost_mutex_);
      auto& boundary_bounds =
          boundary_cost_[boundary_index][svt_graph_point.index_t()];
      if (boundary_bounds.first < 0.0) {
        const auto s_range = st_boundary.GetBoundarySRange(t);
        CHECK(s_range.has_value());
        boundary_bounds = *s_range;
        s_upper = s_range->first;
        s_lower = s_range->second;
      } else {
        s_upper = boundary_bounds.first;
        s_lower = boundary_bounds.second;
      }

      int index_min_t = FloorToInt(st_boundary.min_t() / unit_t_);
      auto& first_boundary_range = boundary_cost_[boundary_index][index_min_t];
      if (first_boundary_range.first < 0.0) {
        const auto first_s_range =
            st_boundary.GetBoundarySRange(st_boundary.min_t());
        CHECK(first_s_range.has_value());
        first_boundary_range = *first_s_range;
        first_s_upper = first_s_range->first;
      } else {
        first_s_upper = first_boundary_range.first;
      }
    }

    const auto first_object_v =
        st_boundary.GetStBoundarySpeedAtT(st_boundary.min_t());
    CHECK(first_object_v.has_value());

    DecisionType decision = StBoundaryProto_DecisionType_UNKNOWN;
    bool decided_by_protective = false;
    if (st_boundary_with_decision->decision_type() !=
        StBoundaryProto_DecisionType_UNKNOWN) {
      // If st_boundary has a prior decision, use it.
      decision = st_boundary_with_decision->decision_type();
    } else {
      // st_boundary doesn't have a prior decision.
      const auto prev_point_decision =
          prev_svt_graph_point.GetStBoundaryDecision(st_boundary.id());
      if (prev_point_decision) {
        CHECK_NE(*prev_point_decision, StBoundaryProto_DecisionType_UNKNOWN);
        // If prev point has decision on the st_boundary, keep it.
        decision = *prev_point_decision;
      } else {
        // Compute decision decisive s, we can make decision via decisive st
        // point (decisive_s, min_t) no matter whether speed profile
        // cross st boundary between lower points or first lower point and upper
        // point.
        for (const auto& st_boundary_decision : *st_boundary_decisions) {
          if (st_boundary_decision.first == st_boundary.id()) {
            // If the decision of this st-boundary is already made by its
            // protective st-boundary, use this decison directly.
            decision = st_boundary_decision.second;
            decided_by_protective = true;
            CHECK_NE(decision, StBoundaryProto_DecisionType_UNKNOWN);
            CHECK(ContainsKey(original_st_boundary_wd_map_, st_boundary.id()));
            break;
          }
        }
        if (!decided_by_protective) {
          const double decisive_s =
              Lerp(prev_s, s, LerpFactor(prev_t, t, st_boundary.min_t()));
          const double first_s_upper = st_boundary.upper_points().front().s();
          const double first_s_lower = st_boundary.lower_points().front().s();
          // Make decision based on relative position of s, s_lower and s_upper.
          // if(st_boundary_with_decision->object_id().value() == "50698"){
          //   std::cout<<prev_s<<"  "<<s<<"  "<<prev_t<<"  "<<t<<std::endl;
          //   std::cout<<first_s_lower<<"  "<<first_s_upper<<std::endl;
          //   std::cout<<decisive_s<<std::endl;
          //   std::cout<<"-----------------"<<std::endl;
          // }
          decision = MakeStBoundaryDecision(
              decisive_s, first_s_lower, first_s_upper,
              st_boundary_with_decision->decision_param().dp_follow_lead_ratio);
        }
      }
      if (!decided_by_protective) {
        // Record decision on this st_boundary without prior decision.
        st_boundary_decisions->emplace_back(st_boundary.id(), decision);
        // Synchronize the decisions of protective st-boundary & corresponding
        // original st-boundary.
        if (st_boundary_with_decision->raw_st_boundary()->is_protective()) {
          const auto& protected_st_boundary_id =
              st_boundary_with_decision->raw_st_boundary()
                  ->protected_st_boundary_id();
          if (protected_st_boundary_id.has_value()) {
            if (const auto original_st_boundary_wd = FindOrNull(
                    original_st_boundary_wd_map_, *protected_st_boundary_id);
                nullptr != original_st_boundary_wd &&
                (*original_st_boundary_wd)->decision_type() ==
                    StBoundaryProto_DecisionType_UNKNOWN) {
              st_boundary_decisions->emplace_back(*protected_st_boundary_id,
                                                  decision);
            }
          }
        }
      }
    }

    double delta_t = -std::numeric_limits<double>::max();
    double obj_v = st_boundary_with_decision->obj_pose_info().v();

    const auto& obj_sl_info =
        st_boundary_with_decision->raw_st_boundary()->obj_sl_info();
    if (st_boundary_with_decision->decision_param()
            .enable_add_collision_risk_cost_for_dp &&
        ((av_speed - obj_v) > kDeltaVel) && obj_sl_info.has_value()) {
      double av_t = st_boundary.min_s() / (av_speed + kEps);
      double obj_t = st_boundary.min_t();

      const auto& frenet_polygon = obj_sl_info->frenet_polygon;
      double obj_dist = std::max(std::min(std::fabs(frenet_polygon.l_max),
                                          std::fabs(frenet_polygon.l_min)) -
                                     1.0,
                                 0.0);
      double tmp_t = std::numeric_limits<double>::max();
      if (obj_v > 0.0) {
        tmp_t = std::fabs(obj_dist / obj_v);
      }
      obj_t = std::min(obj_t, tmp_t);
      delta_t = av_t - obj_t;
    }

    double agent_dist_to_collision = 0.0;
    double obj_acc = 0.0;
    if (st_boundary.obj_sl_info().has_value()) {
      agent_dist_to_collision = st_boundary.obj_sl_info()->dl;
    }
    if (st_boundary.traj_id().has_value()) {
      const SpacetimeObjectTrajectory* st_traj =
          st_traj_mgr_.FindTrajectoryById(st_boundary.traj_id().value());
      CHECK_NOTNULL(st_traj);
      obj_acc = st_traj->planner_object().pose().a();
      if (nullptr != st_traj && 0 != st_boundary.overlap_infos().size()) {
        const auto& first_overlap_info = st_boundary.overlap_infos().front();
        if (0 <= first_overlap_info.obj_idx &&
            first_overlap_info.obj_idx < st_traj->states().size()) {
          const auto* overlap_agent_traj_point =
              st_traj->states()[first_overlap_info.obj_idx].traj_point;
          agent_dist_to_collision = overlap_agent_traj_point->s();
        }
      }
    }

    double ego_yield_acc = 0.0;
    double obj_yield_acc = 0.0;
    if (st_boundary_with_decision->decision_param()
            .enable_add_collision_risk_cost_for_dp) {
      ego_yield_acc =
          CalcYieldResult(av_speed, st_boundary.min_t(), st_boundary.min_s());
      double ego_t = st_boundary.min_s() / (av_speed + kEps);
      obj_yield_acc = CalcYieldResult(obj_v, ego_t, agent_dist_to_collision);
    }

    cost += GetStBoundaryCost(
        decision, av_speed, st_boundary, s, s_lower, s_upper, first_s_upper,
        *first_object_v, obj_acc, t,
        st_boundary_with_decision->follow_standstill_distance(),
        follow_distance_rel_speed_plf_, *speed_finder_params_, *params_,
        st_boundary_with_decision->decision_param()
            .enable_add_collision_risk_cost_for_dp,
        delta_t, obj_v, agent_dist_to_collision, ego_yield_acc, obj_yield_acc,
        st_boundary.min_s(), curr_comfort_brake_s);
  }
  *st_boundary_cost = cost * unit_t_;
}

double DpSvtCost::GetSpatialPotentialCost(double s) const {
  return (total_s_ - s) * params_->spatial_potential_weight();
}

double DpSvtCost::GetSpeedLimitCost(
    double speed, double speed_limit,
    std::optional<double> lane_merge_speed_limit) const {
  double cost = 0.0;
  const double det_speed =
      std::clamp(speed - speed_limit, -kMaxSpeedLimitDiff, kMaxSpeedLimitDiff);
  const double det_speed_sqr = det_speed * det_speed;
  if (det_speed > 0) {
    cost += exceed_speed_unit_cost_ * det_speed_sqr;
  } else if (det_speed < 0) {
    cost += low_speed_unit_cost_ * det_speed_sqr;
  }

  // Increase the cost for  lane merge.
  const double kMinLaneMergeCostOffset = 1.0;  // m/s
  if (lane_merge_speed_limit.has_value()) {
    const double det_speed =
        std::clamp(speed - (*lane_merge_speed_limit - kMinLaneMergeCostOffset),
                   -kMaxSpeedLimitDiff, kMaxSpeedLimitDiff);
    const double det_speed_sqr = det_speed * det_speed;
    if (det_speed > 0) {
      cost += exceed_speed_unit_cost_ * det_speed_sqr;
    } else if (det_speed < 0) {
      cost += low_speed_unit_cost_ * det_speed_sqr;
    }
  }
  return cost;
}

double DpSvtCost::GetReferenceSpeedCost(double speed,
                                        double cruise_speed) const {
  double cost = 0.0;

  if (FLAGS_enable_sampling_dp_reference_speed) {
    const double diff_speed = speed - cruise_speed;
    cost += params_->reference_speed_penalty() * params_->speed_weight() *
            diff_speed * diff_speed * unit_t_;
  }

  return cost;
}

double DpSvtCost::GetAccelCost(double accel) const {
  double cost = 0.0;

  const double accel_sq = accel * accel;
  const double accel_penalty = params_->accel_penalty();
  const double decel_penalty = params_->decel_penalty();

  if (accel > 0.0) {
    cost = accel_penalty * accel_sq;
  } else {
    cost = decel_penalty * accel_sq;
  }

  return cost * unit_t_;
}

}  // namespace st::planning
