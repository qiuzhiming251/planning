

#include "decider/decision_manager/traffic_gap_finder.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iterator>
#include <limits>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "plan_common/log.h"
#include "object_manager/planner_object.h"
#include "plan_common/log_data.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_object_state.h"
#include "plan_common/util/status_macros.h"

namespace st::planning {

namespace {

constexpr double kMinCoastingSpeed = 2.222;  // m/s
constexpr double kMaxSpeedLimitFactor = 1.0;
constexpr double kEpsilon = 1e-7;
constexpr double kTimeForNaviDist = 9.0;
constexpr double kEachLaneChangeLength = 80.0;
constexpr double kMaxGapLength = 50.0;
constexpr double kMaxGapLength_HNOA = 100.0;
constexpr double kGapTimeDist = 0.5;
constexpr double kGapConsistencyCost = 1.0;
constexpr double kGapConsistencyCost_HNOA = 10.0;
constexpr double kFullyEnterTargetLateralThreshold = 1.2;
constexpr int kGapCountNum = 2;
constexpr double kAccWight = 10.0;
constexpr double kTWeight = 1.5;
constexpr double kTTCWeight = 12.0;
constexpr double kConsistWeight = 10.0;
constexpr double kLengthWeight = 0.5;
constexpr double kSpeedLimitWeight = 40.0;
constexpr double kAccVariationWeight = 5.0;
constexpr double kDecelerationWeight = 1.0;
constexpr double kLcFailWeight = 0.05;

int kmaxReliablePerceptiontime = 30;  // unit : 0.1s
struct SVState {
  double s = 0.0;
  double v = 0.0;
};

bool HasFullyEnteredTargetLane(const double center_l, const double half_width) {
  return std::abs(center_l) < kFullyEnterTargetLateralThreshold + half_width;
}

bool ObjIsLeavingTargetLane(const FrenetBox& obj_cur_box,
                            const st::FrenetCoordinate& obj_preview_pnt) {
  const double obj_width = obj_cur_box.width();
  const double obj_cur_l = obj_cur_box.center_l();
  const auto obj_preview_l = obj_preview_pnt.l;
  bool obj_preview_not_in_target =
      !HasFullyEnteredTargetLane(obj_preview_l, 0.5 * obj_width);

  const bool obj_lc_left = (obj_preview_l > obj_cur_l);
  const bool obj_cut_out_clearly =
      obj_preview_not_in_target &&
      (std::fabs(obj_preview_l) > std::fabs(obj_cur_l)) &&
      (std::fabs(obj_preview_l - obj_cur_l) > 0.3);

  return obj_cut_out_clearly;
}

// search obstacles on target lane
std::vector<PlannerObjectProjectionInfo> SearchObstaclesOnTargetLane(
    const FrenetFrame& target_frenet_frame,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const FrenetBox& ego_frenet_box) {
  std::vector<PlannerObjectProjectionInfo> objects_on_lane;
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    const PlannerObject& obj = trajectories.front()->planner_object();

    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

    const double kLateralThreshold = 1.0;
    const double kLonDistThreshold = 100.0;  // m.
    if (frenet_box.l_min > kLateralThreshold ||
        frenet_box.l_max < -kLateralThreshold ||
        frenet_box.s_min - ego_frenet_box.s_max > kLonDistThreshold) {
      continue;
    }

    // is cut out
    const auto obj_preview_pnt = target_frenet_frame.XYToSL(
        trajectories.front()->states().size() < kmaxReliablePerceptiontime
            ? trajectories.front()->states().back().traj_point->pos()
            : (trajectories.front()->states().begin() +
               kmaxReliablePerceptiontime - 1)
                  ->traj_point->pos());
    bool is_obj_cutout = ObjIsLeavingTargetLane(frenet_box, obj_preview_pnt);
    if (is_obj_cutout) {
      Log2DDS::LogDataV2("gap_debug",
                         absl::StrCat("obj id: ", obj.id(), " cut out"));
      continue;
    }

    objects_on_lane.push_back(PlannerObjectProjectionInfo{
        .st_trajectories = trajectories, .frenet_box = frenet_box});
  }

  return objects_on_lane;
}

bool IsOncomingObject(double obj_heading, double ego_heading) {
  if (std::abs(NormalizeAngle(ego_heading - obj_heading)) > M_PI_2) {
    return true;
  }
  return false;
}

std::vector<TrafficGap> SearchCandidateGaps(
    std::vector<PlannerObjectProjectionInfo>& objects_on_lane,
    const st::planning::DeciderStateProto& pre_decider_state,
    const FrenetBox& ego_frenet_box, double ego_heading,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const st::LaneChangeStateProto& lc_state, double ego_v,
    double dist_to_merge) {
  constexpr double kGapMinDis = 6.0;  // m
  std::vector<TrafficGap> traffic_gaps;

  if (objects_on_lane.empty()) {
    return traffic_gaps;
  }
  double avg_speed = 0.0;
  for (const auto& obj : objects_on_lane) {
    avg_speed += obj.st_trajectories.front()->pose().v();
  }
  // simplyfied EMA(exponential moving average) method: keep last smooth result,
  // smoothness factor = 0.5 to decrease variation of avg speed
  avg_speed = objects_on_lane.empty() ? std::numeric_limits<double>::max()
              : pre_decider_state.traffic_gap().follow_traffic_flow()
                  ? (avg_speed / objects_on_lane.size() +
                     pre_decider_state.traffic_gap().traffic_flow_v()) /
                        2
                  : avg_speed / objects_on_lane.size();
  double speed_buffer = 0.0;
  if (pre_decider_state.traffic_gap().follow_traffic_flow()) {
    speed_buffer = 2.0;
  }
  // speed diff larger than 7 m/s and close to merge point
  // TODO any case with large speed diff is supposed to be considered
  if (ego_v - avg_speed > 7.0 - speed_buffer && dist_to_merge < 500.0) {
    TrafficGap gap;
    gap.avg_speed = avg_speed;
    traffic_gaps.push_back(std::move(gap));
    return traffic_gaps;
  }
  std::sort(objects_on_lane.begin(), objects_on_lane.end(),
            [](const auto& a, const auto& b) {
              return a.frenet_box.s_min < b.frenet_box.s_min;
            });
  //

  const double ego_center_s = ego_frenet_box.center_s();
  const auto closest_object_it = std::min_element(
      objects_on_lane.begin(), objects_on_lane.end(),
      [ego_center_s](const auto& lhs, const auto& rhs) {
        return std::abs(lhs.frenet_box.center_s() - ego_center_s) <
               std::abs(rhs.frenet_box.center_s() - ego_center_s);
      });

  Log2DDS::LogDataV2(
      "gap_debug",
      absl::StrCat(
          "closest object id: ",
          absl::StrFormat(
              closest_object_it->st_trajectories.front()->object_id())));

  int closest_object_idx = closest_object_it - objects_on_lane.begin();
  int gap_count = 0;
  for (int index = closest_object_idx; index >= 0; --index) {
    if (gap_count >= kGapCountNum) {
      break;
    }
    std::string obj_id =
        objects_on_lane[index].st_trajectories.front()->planner_object().id();
    // oncoming
    double obj_heading = objects_on_lane[index]
                             .st_trajectories.front()
                             ->states()
                             .front()
                             .traj_point->theta();
    if (IsOncomingObject(obj_heading, ego_heading)) {
      Log2DDS::LogDataV2("gap_debug",
                         absl::StrCat("obj id: ", obj_id, " is oncoming"));
      continue;
    }
    // stalled
    if (stalled_objects.contains(obj_id)) {
      Log2DDS::LogDataV2("gap_debug",
                         absl::StrCat("obj id: ", obj_id, " is stalled"));
      continue;
    }
    TrafficGap gap;
    if (index >= 1) {
      // short
      double dis_buffer = 0.0;
      std::string follower_obj_id = objects_on_lane[index - 1]
                                        .st_trajectories.front()
                                        ->planner_object()
                                        .id();
      for (const auto& gap : pre_decider_state.candidate_traffic_gap()) {
        if (gap.has_leader_id() && gap.has_follower_id() &&
            obj_id == gap.leader_id() && follower_obj_id == gap.follower_id()) {
          dis_buffer = -2.0;
          // if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
          //   dis_buffer = -1.0;
          // }
        }
      }
      if (objects_on_lane[index].frenet_box.s_min -
              objects_on_lane[index - 1].frenet_box.s_max <
          kGapMinDis + dis_buffer) {
        Log2DDS::LogDataV2(
            "gap_debug",
            absl::StrCat("obj id: ", obj_id, " as leader's gap is small"));
        continue;
      } else {
        gap.leader_trajectories = objects_on_lane[index].st_trajectories;
        gap.s_end = objects_on_lane[index].frenet_box.s_min;
        gap.follower_trajectories = objects_on_lane[index - 1].st_trajectories;
        gap.s_start = objects_on_lane[index - 1].frenet_box.s_max;
        gap.avg_speed = avg_speed;
        gap_count++;
        gap.ego_in_gap =
            objects_on_lane[index].frenet_box.s_max > ego_frenet_box.s_max &&
            objects_on_lane[index - 1].frenet_box.s_min < ego_frenet_box.s_min;
        traffic_gaps.push_back(std::move(gap));
      }
    } else {
      gap.leader_trajectories = objects_on_lane[index].st_trajectories;
      gap.s_end = objects_on_lane[index].frenet_box.s_min;
      gap.s_start = std::numeric_limits<double>::lowest();
      gap.avg_speed = avg_speed;
      gap.ego_in_gap =
          objects_on_lane[index].frenet_box.s_max > ego_frenet_box.s_max;
      traffic_gaps.push_back(std::move(gap));
    }
  }
  std::reverse(traffic_gaps.begin(), traffic_gaps.end());

  gap_count = 0;
  int index = closest_object_idx + 1;
  for (; index < objects_on_lane.size(); ++index) {
    if (gap_count >= kGapCountNum) {
      break;
    }
    std::string obj_id =
        objects_on_lane[index].st_trajectories.front()->planner_object().id();
    // oncoming
    double obj_heading = objects_on_lane[index]
                             .st_trajectories.front()
                             ->states()
                             .front()
                             .traj_point->theta();
    if (IsOncomingObject(obj_heading, ego_heading)) {
      Log2DDS::LogDataV2("gap_debug",
                         absl::StrCat("obj id: ", obj_id, " is oncomming"));
      continue;
    }
    // stalled
    if (stalled_objects.contains(obj_id)) {
      Log2DDS::LogDataV2("gap_debug",
                         absl::StrCat("obj id: ", obj_id, " is stalled"));
      continue;
    }
    // short
    double dis_buffer = 0.0;
    std::string follower_obj_id = objects_on_lane[index - 1]
                                      .st_trajectories.front()
                                      ->planner_object()
                                      .id();
    for (const auto& gap : pre_decider_state.candidate_traffic_gap()) {
      if (gap.has_leader_id() && gap.has_follower_id() &&
          obj_id == gap.leader_id() && follower_obj_id == gap.follower_id()) {
        dis_buffer = -2.0;
        // if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
        //   dis_buffer = -1.0;
        // }
      }
    }
    if (objects_on_lane[index].frenet_box.s_min -
            objects_on_lane[index - 1].frenet_box.s_max <
        kGapMinDis + dis_buffer) {
      Log2DDS::LogDataV2("gap_debug", absl::StrCat("obj id: ", obj_id,
                                                   "as leader's gap is small"));
      continue;
    }
    TrafficGap gap;
    gap.leader_trajectories = objects_on_lane[index].st_trajectories;
    gap.s_end = objects_on_lane[index].frenet_box.s_min;
    gap.follower_trajectories = objects_on_lane[index - 1].st_trajectories;
    gap.s_start = objects_on_lane[index - 1].frenet_box.s_max;
    gap.avg_speed = avg_speed;
    gap.ego_in_gap =
        objects_on_lane[index].frenet_box.s_max > ego_frenet_box.s_max &&
        objects_on_lane[index - 1].frenet_box.s_min < ego_frenet_box.s_min;
    gap_count++;
    traffic_gaps.push_back(std::move(gap));
  }

  if (gap_count < 2 && index == objects_on_lane.size()) {
    TrafficGap gap;
    gap.s_end = std::numeric_limits<double>::max();
    gap.follower_trajectories = objects_on_lane[index - 1].st_trajectories;
    gap.s_start = objects_on_lane[index - 1].frenet_box.s_max;
    gap.avg_speed = avg_speed;
    gap.ego_in_gap =
        objects_on_lane[index - 1].frenet_box.s_min < ego_frenet_box.s_min;
    traffic_gaps.push_back(std::move(gap));
  }

  return traffic_gaps;
}

}  // namespace

std::vector<TrafficGap> FindCandidateTrafficGapsOnLanePath(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr, double ego_v,
    double dist_to_merge,
    const st::planning::DeciderStateProto& pre_decider_state,
    const std::vector<std::string>& lc_lead_obj_ids, double ego_heading,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const st::LaneChangeStateProto& lc_state) {
  std::vector<PlannerObjectProjectionInfo> objects_on_lane =
      SearchObstaclesOnTargetLane(target_frenet_frame, st_traj_mgr,
                                  ego_frenet_box);

  std::vector<TrafficGap> traffic_gaps = SearchCandidateGaps(
      objects_on_lane, pre_decider_state, ego_frenet_box, ego_heading,
      stalled_objects, lc_state, ego_v, dist_to_merge);
  return traffic_gaps;
}

struct GapPointInfo {
  double gap_len = DBL_MAX;
  double time_dist_lead = 0.0;
  double time_dist_follow = 0.0;
  double ttc = 0.0;
  double v = 0.0;
  double a = 0.0;
  double t = 0.0;
  double dec_factor = 0.0;
  // sub cost
  double sub_acc_cost = 0.0;
  double sub_t_cost = 0.0;
  double sub_ttc_cost = 0.0;
  double sub_length_cost = 0.0;
  double sub_consist_cost = 0.0;
  double sub_speed_limit_cost = 0.0;
  double sub_acc_variation_cost = 0.0;
  double sub_follow_gap_cost = 0.0;
  double sub_dec_cost = 0.0;
  double sub_lc_fail_cost = 0.0;
  // sum cost
  double acc_cost = 0.0;
  double t_cost = 0.0;
  double ttc_cost = 0.0;
  double length_cost = 0.0;
  double consist_cost = 0.0;
  double speed_limit_cost = 0.0;
  double acc_variation_cost = 0.0;
  double follow_gap_cost = 0.0;
  double dec_cost = 0.0;
  double lc_fail_cost = 0.0;
  // total cost
  double total_cost = DBL_MAX;
  std::string DebugString() const {
    return absl::StrCat(
        " total_cost: ", total_cost, " sub_acc_cost: ", sub_acc_cost,
        ", weight: ", kAccWight, ", acc_cost: ", acc_cost,
        " sub_t_cost: ", sub_t_cost, ", weight: ", kTWeight,
        ", t_cost: ", t_cost, " sub_ttc_cost: ", sub_ttc_cost,
        ", weight: ", kTTCWeight, ", ttc_cost: ", ttc_cost,
        " sub_length_cost: ", sub_length_cost, ", weight: ", kLengthWeight,
        ", length_cost: ", length_cost, " sub_consist_cost: ", sub_consist_cost,
        ", weight: ", kConsistWeight, ", consist_cost: ", consist_cost,
        ", sub_acc_variation_cost: ", sub_acc_variation_cost,
        ", acc_variation_cost: ", acc_variation_cost,
        ", sub_speed_limit_cost: ", sub_speed_limit_cost,
        ", speed_limit_cost: ", speed_limit_cost,
        ", deceleration_cost: ", dec_cost,
        ", sub_follow_gap_cost: ", sub_follow_gap_cost,
        ", follow_gap_cost: ", follow_gap_cost,
        ", lc_fail_cost: ", lc_fail_cost);
  }
};

const std::vector<double> kAccelerationList = {-2.0, -1.8, -1.5, -1.3, -1.0,
                                               -0.8, -0.5, 0.0,  0.5,  0.8,
                                               1.0,  1.3,  1.5,  1.8,  2.0};
const std::vector<double> kTimeList = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0,
                                       4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0};

SVState GetObjSVState(const SpacetimeObjectTrajectory* obj_traj, double start_s,
                      double t) {
  if (!obj_traj) return SVState{};

  if (obj_traj->is_stationary()) {
    return SVState{.s = start_s, .v = 0.0};
  }

  auto nearest_it =
      std::min_element(obj_traj->states().begin(), obj_traj->states().end(),
                       [t](const auto& lhs, const auto& rhs) {
                         return std::abs(t - lhs.traj_point->t()) <
                                std::abs(t - rhs.traj_point->t());
                       });

  return SVState{.s = start_s + nearest_it->traj_point->s(),
                 .v = nearest_it->traj_point->v()};
}

std::optional<GapPointInfo> CalculateGapPoint(
    double accel, double t, double ego_init_v, double speed_limit,
    const FrenetBox& ego_frenet_box, const TrafficGap& gap,
    const std::optional<PlannerObjectProjectionInfo>& leader_obj_current_lane) {
  double max_speed = speed_limit * kMaxSpeedLimitFactor;
  const double accel_v = ego_init_v + accel * t;
  // const double ego_v = std::max(kMinCoastingSpeed, accel_v);
  double ego_v = accel_v;
  double ego_delta_s = 0.0;
  double gap_s_max = DBL_MAX;
  double gap_s_min = DBL_MAX;
  // in case only one obj with high speed, avg speed will be unreasonable
  double reference_speed = std::min(gap.avg_speed, speed_limit) * 0.8;
  double dec_factor = std::max(reference_speed - ego_v, 0.0);
  if (accel_v < kMinCoastingSpeed) {
    if (ego_init_v < kMinCoastingSpeed) {
      ego_delta_s = ego_init_v * t + accel * t * t * 0.5;
      ego_v = ego_init_v + accel * t;
    } else {
      double break_t = 0.0;
      break_t = std::abs(accel) < kEpsilon
                    ? 0.0
                    : std::fabs((kMinCoastingSpeed - ego_init_v) / accel);
      ego_delta_s = (kMinCoastingSpeed + ego_init_v) * break_t * 0.5 +
                    kMinCoastingSpeed * (t - break_t);
      ego_v = kMinCoastingSpeed;
    }
  } else {
    ego_delta_s = (accel_v + ego_init_v) * t * 0.5;
    ego_v = ego_init_v + accel * t;
  }
  double ego_front_s = ego_delta_s + ego_frenet_box.s_max;
  double ego_rear_s = ego_delta_s + ego_frenet_box.s_min;

  std::string lead_id, follow_id;
  double ttc = DBL_MAX;
  double kEpsilon = 1e-3;
  double time_dist_lead = 0.0;
  double time_dist_follow = 0.0;
  if (!gap.leader_trajectories.empty()) {
    lead_id = gap.leader_trajectories.front()->planner_object().id();

    SVState sv_state_front =
        GetObjSVState(gap.leader_trajectories.front(), gap.s_end, t);
    gap_s_max = sv_state_front.s;
    if (ego_front_s >= sv_state_front.s) {
      return std::nullopt;
    } else {
      double decay_factor =
          Lerp(2.0, -0.3, 0.0, 0.2,
               sv_state_front.v / std::max(ego_v, kEpsilon) - 1, true);
      time_dist_lead = std::max(ego_v * kGapTimeDist * decay_factor -
                                    (sv_state_front.s - ego_front_s),
                                0.0) /
                       kGapTimeDist;
      ttc =
          sv_state_front.v > ego_v
              ? ttc
              : std::min(ttc, (sv_state_front.s - ego_front_s) /
                                  std::max(ego_v - sv_state_front.v, kEpsilon));
    }
  }
  if (!gap.follower_trajectories.empty()) {
    follow_id = gap.follower_trajectories.front()->planner_object().id();

    SVState sv_state_rear =
        GetObjSVState(gap.follower_trajectories.front(), gap.s_start, t);
    gap_s_min = sv_state_rear.s;
    if (ego_rear_s <= sv_state_rear.s) {
      return std::nullopt;
    } else {
      double decay_factor =
          Lerp(2.0, -0.2, 0.0, 0.3,
               ego_v / std::max(sv_state_rear.v, kEpsilon) - 1, true);
      time_dist_follow =
          std::max(sv_state_rear.v * kGapTimeDist * decay_factor -
                       (ego_rear_s - sv_state_rear.s),
                   0.0) /
          kGapTimeDist;
      ttc =
          sv_state_rear.v < ego_v
              ? ttc
              : std::min(ttc,
                         std::fabs((ego_rear_s - sv_state_rear.s) /
                                   std::max(std::fabs(ego_v - sv_state_rear.v),
                                            kEpsilon)));
    }
  }

  // leader safety
  constexpr double kSafeDistance = 2.0;
  if (leader_obj_current_lane.has_value()) {
    auto leader = leader_obj_current_lane.value();
    SVState sv_state_leader = GetObjSVState(leader.st_trajectories.front(),
                                            leader.frenet_box.s_min, t);
    if (sv_state_leader.s <= ego_front_s + kSafeDistance) {
      return std::nullopt;
    }
    // ttc
    double v_diff = ego_v - sv_state_leader.v;
    double s_diff = sv_state_leader.s - ego_front_s;
    const double kTTCThreshold = 2.0;  // s
    if (v_diff > 0 && s_diff / v_diff <= kTTCThreshold) {
      return std::nullopt;
    }
  }
  return GapPointInfo{
      .gap_len = std::min(kMaxGapLength, std::fabs(gap_s_max - gap_s_min)),
      .time_dist_lead = time_dist_lead,
      .time_dist_follow = time_dist_follow,
      .ttc = ttc,
      .v = ego_v,
      .a = accel,
      .t = t,
      .dec_factor = dec_factor};
}

bool IsSameWithPreGap(const TrafficGap& cur_gap,
                      const ConstraintProto::TrafficGapProto& pre_gap) {
  if (!cur_gap.leader_trajectories.empty() &&
      !cur_gap.follower_trajectories.empty()) {
    if (cur_gap.leader_trajectories.front()->planner_object().id() ==
            pre_gap.leader_id() ||
        cur_gap.follower_trajectories.front()->planner_object().id() ==
            pre_gap.follower_id()) {
      return true;
    }
  } else if (!cur_gap.leader_trajectories.empty()) {
    if (cur_gap.leader_trajectories.front()->planner_object().id() ==
        pre_gap.leader_id()) {
      return true;
    }
  } else if (!cur_gap.follower_trajectories.empty()) {
    if (cur_gap.follower_trajectories.front()->planner_object().id() ==
        pre_gap.follower_id()) {
      return true;
    }
  }
  return false;
}

void CalculateGapPointCost(const TrafficGap& gap,
                           const ConstraintProto::TrafficGapProto& pre_gap,
                           GapPointInfo* gap_point, double dist_to_navi_end,
                           double lc_num, double speed_limit, double ego_v) {
  constexpr double kAMaxLimit = 4.0;
  constexpr double kTMaxLimit = 8.0;

  // speed limit cost
  double overspeed_factor =
      Lerp(1.18, 30.0 / 3.6, 1.03, 120.0 / 3.6, speed_limit, true);
  double weight_factor =
      Lerp(0.5, 30.0 / 3.6, 2.0, 120.0 / 3.6, speed_limit, true);
  double speed_limit_cost =
      std::max(gap_point->v - speed_limit * overspeed_factor, 0.0) /
      speed_limit * kSpeedLimitWeight * weight_factor;
  gap_point->sub_speed_limit_cost =
      std::max(gap_point->v - speed_limit * overspeed_factor, 0.0) /
      speed_limit;
  gap_point->speed_limit_cost = speed_limit_cost;

  // deceleration cost, punish far slower than target traffic flow speed case
  double dec_cost = gap_point->dec_factor * kDecelerationWeight;
  gap_point->dec_cost = dec_cost;
  gap_point->sub_dec_cost = gap_point->dec_factor;

  // acc variation between last frame cost
  double acc_variation_cost = 0.0;
  gap_point->sub_acc_variation_cost = 0.0;
  gap_point->acc_variation_cost = 0.0;
  if (pre_gap.has_gap_point()) {
    acc_variation_cost =
        std::fabs(gap_point->a - pre_gap.gap_point().a()) * kAccVariationWeight;
    gap_point->sub_acc_variation_cost =
        std::fabs(gap_point->a - pre_gap.gap_point().a());
    gap_point->acc_variation_cost = acc_variation_cost;
  };

  // acc weight
  double acc_weight = kAccWight;
  if (lc_num > 0) {
    if (dist_to_navi_end / lc_num >
        kEachLaneChangeLength) {  // 距离导航终点还有一段距离
      acc_weight = gap_point->a > 0.0 ? 0.5 * kAccWight : 1.3 * kAccWight;
    } else {
      acc_weight = gap_point->a > 0.0 ? 1.3 * kAccWight : 0.5 * kAccWight;
    }
  }
  double acc_cost = std::abs(gap_point->a) / kAMaxLimit * acc_weight;
  gap_point->sub_acc_cost = std::abs(gap_point->a) / kAMaxLimit;
  gap_point->acc_cost = acc_cost;
  // follow gap cost
  double follow_gap_weight = 1.0;
  const double kPreviewTime = 3.0;
  double gap_leader_v = 0.0;
  double gap_follower_v = 0.0;
  double gap_speed = gap_point->v;
  if (!gap.leader_trajectories.empty() && !gap.follower_trajectories.empty()) {
    gap_leader_v =
        gap.leader_trajectories.front()->states().size() <
                kmaxReliablePerceptiontime
            ? gap.leader_trajectories.front()->states().back().traj_point->v()
            : (gap.leader_trajectories.front()->states().begin() +
               kmaxReliablePerceptiontime - 1)
                  ->traj_point->v();
    gap_follower_v =
        gap.follower_trajectories.front()->states().size() <
                kmaxReliablePerceptiontime
            ? gap.follower_trajectories.front()->states().back().traj_point->v()
            : (gap.follower_trajectories.front()->states().begin() +
               kmaxReliablePerceptiontime - 1)
                  ->traj_point->v();
    gap_speed = gap_point->ttc < 3.0 ? (gap_leader_v + gap_follower_v) * 0.5
                                     : gap_point->v;
  } else if (gap.leader_trajectories.empty() &&
             !gap.follower_trajectories.empty()) {
    gap_follower_v =
        gap.follower_trajectories.front()->states().size() <
                kmaxReliablePerceptiontime
            ? gap.follower_trajectories.front()->states().back().traj_point->v()
            : (gap.follower_trajectories.front()->states().begin() +
               kmaxReliablePerceptiontime - 1)
                  ->traj_point->v();
    gap_speed = gap_follower_v > gap_point->v ? gap_follower_v : gap_point->v;
  } else {
    gap_leader_v =
        gap.leader_trajectories.front()->states().size() <
                kmaxReliablePerceptiontime
            ? gap.leader_trajectories.front()->states().back().traj_point->v()
            : (gap.leader_trajectories.front()->states().begin() +
               kmaxReliablePerceptiontime - 1)
                  ->traj_point->v();
    gap_speed = gap_leader_v < gap_point->v ? gap_leader_v : gap_point->v;
  }
  double follow_gap_cost = std::abs(gap_speed - gap_point->v) / kPreviewTime /
                           kAMaxLimit * follow_gap_weight;
  gap_point->sub_follow_gap_cost =
      std::abs(gap_speed - gap_point->v) / kPreviewTime / kAMaxLimit;
  gap_point->follow_gap_cost = follow_gap_cost;
  // t cost
  double t_cost = gap_point->t / kTMaxLimit * kTWeight;
  gap_point->sub_t_cost = gap_point->t / kTMaxLimit;
  gap_point->t_cost = t_cost;
  // ttc cost
  constexpr double kTTCTthr = 3.0;
  gap_point->sub_ttc_cost =
      gap_point->ttc > kTTCTthr
          ? 0.0
          : std::pow((kTTCTthr - gap_point->ttc) / kTTCTthr, 2);

  double ttc_cost = gap_point->sub_ttc_cost * kTTCWeight;
  gap_point->ttc_cost = ttc_cost;
  // gap length cost
  double length_cost =
      kLengthWeight * (gap_point->time_dist_lead + gap_point->time_dist_follow);
  gap_point->sub_length_cost =
      gap_point->time_dist_lead + gap_point->time_dist_follow;
  gap_point->length_cost = length_cost;
  // consist cost
  double consist_cost = 0.0;
  if (!IsSameWithPreGap(gap, pre_gap)) {
    consist_cost = kConsistWeight * kGapConsistencyCost;
    gap_point->sub_consist_cost = kGapConsistencyCost;
    gap_point->consist_cost = consist_cost;
  }
  // gap change case: in case continuous LC fail, add continuous accumulated
  // cost in cur best gap till cur best switches to second-best, which is
  // ralated to speed: under low speed, cost accumulation is more conservative,
  // under high speed, long time stalemate should be avioded
  double sub_lc_fail_cost = 0.0;
  for (const auto& best_gap : pre_gap.best_gap_count()) {
    if (!gap.leader_trajectories.empty() &&
        !gap.follower_trajectories.empty() &&
        (gap.leader_trajectories.front()->object_id() == best_gap.lead_id() ||
         gap.follower_trajectories.front()->object_id() ==
             best_gap.follow_id())) {
      sub_lc_fail_cost = std::max(0.0, best_gap.gap_count() - 40.0) * ego_v;
    } else if (!gap.leader_trajectories.empty() &&
               gap.leader_trajectories.front()->object_id() ==
                   best_gap.lead_id()) {
      sub_lc_fail_cost = std::max(0.0, best_gap.gap_count() - 40.0) * ego_v;
    } else if (!gap.follower_trajectories.empty() &&
               gap.follower_trajectories.front()->object_id() ==
                   best_gap.follow_id()) {
      sub_lc_fail_cost = std::max(0.0, best_gap.gap_count() - 40.0) * ego_v;
    }
  }
  double lc_fail_cost = sub_lc_fail_cost * kLcFailWeight;
  gap_point->sub_lc_fail_cost = sub_lc_fail_cost;
  gap_point->lc_fail_cost = lc_fail_cost;

  double total_cost = speed_limit_cost + acc_variation_cost + follow_gap_cost +
                      acc_cost + t_cost + ttc_cost + length_cost +
                      consist_cost + dec_cost + lc_fail_cost;
  gap_point->total_cost = total_cost;
}

absl::StatusOr<TrafficGapResult> EvaluateAndTakeBestTrafficGap(
    std::vector<TrafficGap>& candidate_gaps, const FrenetBox& ego_frenet_box,
    const FrenetFrame& target_frenet_frame, double ego_init_v,
    double speed_limit, double navi_dist, int lc_num, double dist_to_merge,
    const ConstraintProto::TrafficGapProto& pre_gap,
    const st::Behavior_FunctionId& function_id, double ego_heading,
    const std::optional<PlannerObjectProjectionInfo>& leader_obj_current_lane) {
  if (candidate_gaps.empty()) {
    Log2DDS::LogDataV2("gap_debug", "Input candidate gaps empty!");
    return absl::NotFoundError("Input candidate gaps empty!");
  } else if (candidate_gaps.size() == 1 &&
             candidate_gaps.front().follower_trajectories.empty() &&
             candidate_gaps.front().leader_trajectories.empty()) {
    TrafficGapResult result;
    result.gap_ref_v = candidate_gaps.front().avg_speed;
    return result;
  }
  Log2DDS::LogDataV2("gap_debug", absl::StrCat("navi_dist: ", navi_dist,
                                               ", lc_num: ", lc_num));

  // for merge gap, consider traffic flow speed
  if (dist_to_merge < 500.0 && !candidate_gaps.empty()) {
    speed_limit = std::min(speed_limit, candidate_gaps[0].avg_speed);
  }
  int best_gap_idx = -1;
  GapPointInfo best_gap_info;
  for (int gap_idx = 0; gap_idx < candidate_gaps.size(); ++gap_idx) {
    std::vector<GapPointInfo> gap_points_info;
    for (double a : kAccelerationList) {
      for (double t : kTimeList) {
        auto gap_point_info =
            CalculateGapPoint(a, t, ego_init_v, speed_limit, ego_frenet_box,
                              candidate_gaps[gap_idx], leader_obj_current_lane);
        if (gap_point_info.has_value()) {
          gap_points_info.push_back(gap_point_info.value());
        }
      }
    }
    if (gap_points_info.empty()) {
      Log2DDS::LogDataV2("gap_debug",
                         absl::StrCat("gap index: ", gap_idx,
                                      ", don't have satisfied gap points"));
      continue;
    }
    double cur_min_cost = std::numeric_limits<double>::max();
    GapPointInfo cur_gap_best_point;
    for (GapPointInfo gap_point : gap_points_info) {
      CalculateGapPointCost(candidate_gaps[gap_idx], pre_gap, &gap_point,
                            navi_dist, lc_num, speed_limit, ego_init_v);
      if (gap_point.total_cost < cur_gap_best_point.total_cost) {
        cur_gap_best_point = gap_point;
      }
    }
    Log2DDS::LogDataV2("gap_debug",
                       absl::StrCat("gap index: ", gap_idx, ": ",
                                    cur_gap_best_point.DebugString(),
                                    ", speed limit: ", speed_limit));

    if (cur_gap_best_point.total_cost < best_gap_info.total_cost) {
      best_gap_idx = gap_idx;
      best_gap_info = cur_gap_best_point;
    }
  }

  if (best_gap_idx == -1) {
    return absl::NotFoundError("No viable candidate after evaluation!");
  }

  TrafficGapResult result;
  if (!candidate_gaps[best_gap_idx].leader_trajectories.empty()) {
    result.leader_id = candidate_gaps[best_gap_idx]
                           .leader_trajectories.front()
                           ->planner_object()
                           .id();
  }
  if (!candidate_gaps[best_gap_idx].follower_trajectories.empty()) {
    result.follower_id = candidate_gaps[best_gap_idx]
                             .follower_trajectories.front()
                             ->planner_object()
                             .id();
  }
  result.gap_length = best_gap_info.gap_len;
  result.gap_ttc = best_gap_info.ttc;
  result.gap_v = best_gap_info.v;
  result.gap_a = best_gap_info.a;
  result.gap_t = best_gap_info.t;
  result.gap_total_cost = best_gap_info.total_cost;
  result.ego_in_gap = candidate_gaps[best_gap_idx].ego_in_gap;
  std::string lead_id =
      result.leader_id.has_value() ? result.leader_id.value() : "";
  std::string follow_id =
      result.follower_id.has_value() ? result.follower_id.value() : "";
  Log2DDS::LogDataV2(
      "gap_debug",
      absl::StrCat(
          "best gap index: ", best_gap_idx, ", {", lead_id, ", ", follow_id,
          "}", " length ", best_gap_info.length_cost, " ttc ",
          best_gap_info.ttc_cost, " v ", best_gap_info.speed_limit_cost,
          " dec ", best_gap_info.dec_cost, " a variation ",
          best_gap_info.acc_variation_cost, " acc ", best_gap_info.acc_cost,
          " t ", best_gap_info.t_cost, " consist ", best_gap_info.consist_cost,
          ", LC fail ", best_gap_info.lc_fail_cost,
          ", total cost: ", best_gap_info.total_cost, ", a: ", best_gap_info.a,
          ", t: ", best_gap_info.t));
  return result;
}

}  // namespace st::planning
