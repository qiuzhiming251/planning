#include "decider/selector/candidate_stats.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <cfloat>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/log_data.h"
// #include "global/trace.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/util.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/drive_passage.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction.h"
#include "predictor/prediction_object_state.h"
#include "decider/selector/selector_util.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/status_macros.h"

#include "modules/cnoa_pnc/planning/proto/perception.pb.h"

namespace st::planning {
namespace {

constexpr double kFollowAccelDecel = 0.6;         // m/s^2.
constexpr double kMinFollowTime = 3.0;            // s.
constexpr double kPreviewTimeForDecel = 5.0;      // s.
constexpr double kPreviewTimeForLeaderVel = 4.0;  // s.
constexpr double kSlowWorkingObjProb = 0.7;
constexpr int kCenterLangeChangeMinLaneNumHighway = 3;
constexpr int kCenterLangeChangeMinLaneNum = 3;  // In forward direction.
constexpr double kMinDrivingDistDiscourgeRightMost = 2000.0;  // m.
constexpr double kMinDrivingDistEncourgeRightMost = 500.0;    // m.
constexpr double kPreviewLengthForRightMostLane = 150.0;      // m.
constexpr double kSingleLaneStalledFactor = 5.0;
constexpr double kLengthEpsilon = 1.0;      // m.
constexpr double kInvalidLength = 10000.0;  // m.

double CalculateProgressSpeed(const double ego_v, const double leader_v,
                              const double leader_s,
                              const double min_front_dist) {
  const double target_follow_distance =
      std::max(min_front_dist, leader_v * kMinFollowTime);
  const double init_obj_v_lim =
      leader_v +
      std::sqrt(kFollowAccelDecel *
                    std::max(0.0, leader_s - target_follow_distance) +
                0.5 * (Sqr(ego_v) + Sqr(leader_v)) - ego_v * leader_v);
  // fix when leader v is far smaller than ego v
  const double deceleration_time = std::max(
      kPreviewTimeForDecel - (init_obj_v_lim - ego_v) / kFollowAccelDecel,
      kPreviewTimeForDecel);
  const double deceleration = std::clamp(
      (ego_v - leader_v) / kTrajectoryTimeHorizon, 0.0, kFollowAccelDecel);
  const double obj_v_lim =
      std::max(leader_v, init_obj_v_lim - deceleration_time * deceleration);
  return obj_v_lim;
}

double IsSlowWorkingLargeObjectProb(const double lane_speed_limit,
                                    const double time_since_last_red_light,
                                    const double ego_v, const double ego_s,
                                    const double obj_s,
                                    const SpacetimeObjectTrajectory& traj,
                                    const bool is_leftmost_lane,
                                    const bool is_rightmost_lane) {
  constexpr double kSlowSpeedRatioThreshold = 0.8;
  constexpr double kSlowTimeSinceLastRedLight = 40.0;  // s.
  constexpr double kInMiddleLanePenalty = 0.7;

  DLOG(INFO) << "slow prob calculation leader vehicle id: " << traj.object_id();
  DLOG(INFO) << "slow prob calculation leader vehicle perception type: "
             << traj.object_type() << "; " << ObjectType::OT_VEHICLE << "; "
             << ObjectType::OT_LARGE_VEHICLE << "; " << traj.is_stationary();

  if ((traj.object_type() != ObjectType::OT_VEHICLE &&
       traj.object_type() != ObjectType::OT_LARGE_VEHICLE) ||
      traj.is_stationary()) {
    // 1. Ignore non-vehicle and non-large vehicle directly from perception
    // result
    // 2. Ignore stationary vehicles.
    return 0.0;
  }

  // consider only when the large vehicle is in certain distance range
  if (obj_s - ego_s > std::min(80.0, std::max(15.0, 4 * ego_v))) {
    return 0.0;
  }

  double slow_object_prob = 1.0;
  // calculate the max smoothed prediction velocity
  double max_obj_prediction_v = traj.planner_object().pose().v();
  for (const auto& traj_state : traj.states()) {
    if (traj_state.traj_point->t() > kPreviewTimeForLeaderVel) {
      break;
    }
    max_obj_prediction_v =
        std::max(max_obj_prediction_v, traj_state.traj_point->v());
  }
  DLOG(INFO) << "slow leader v for slow working obj prob: "
             << max_obj_prediction_v;

  slow_object_prob *=
      std::max(0.0, 1.0 - Sqr(max_obj_prediction_v /
                              (lane_speed_limit * kSlowSpeedRatioThreshold)));

  if (time_since_last_red_light < kSlowTimeSinceLastRedLight) {
    slow_object_prob *=
        Sqr(time_since_last_red_light / kSlowTimeSinceLastRedLight);
  }

  if (!is_leftmost_lane && !is_rightmost_lane) {
    slow_object_prob *= kInMiddleLanePenalty;
  }

  // high slow vehicle probability.
  return slow_object_prob;
}

int CountLeaderObjects(
    const std::vector<std::pair<FrenetBox, const SpacetimeObjectTrajectory*>>&
        block_objs) {
  constexpr double kFollowDistance = 8.0;  // m.
  constexpr double kFollowTime = 2.0;      // s.
  for (size_t i = 1; i < block_objs.size(); ++i) {
    const auto& last_traj = *block_objs.at(i - 1).second;
    const double distance =
        block_objs.at(i).first.s_min - block_objs.at(i - 1).first.s_max;
    if (distance >
        kFollowDistance + last_traj.planner_object().pose().v() * kFollowTime) {
      return i - 1;
    }
  }
  return block_objs.size() - 1;
}

}  // namespace

ProgressStats::ProgressStats(
    const SelectorCommonFeature& common_feature,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results)
    : ego_v(plan_start_point.v()), ego_width(vehicle_geom.width()) {
  constexpr double kEgoFrontBuffer = 5.0;  // m.
  const double min_front_dist =
      vehicle_geom.front_edge_to_center() + kEgoFrontBuffer;

  for (size_t idx = 0; idx < est_status.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) continue;

    const auto& result = results[idx];
    const auto& passage = result.scheduler_output.drive_passage;

    const auto& lane_feature_infos = common_feature.lane_feature_infos;
    if (!lane_feature_infos.contains(idx)) continue;
    const auto& lane_feature_info = FindOrDieNoPrint(lane_feature_infos, idx);
    const double lane_speed_limit = lane_feature_info.speed_limit;
    const auto& block_obj_ids = lane_feature_info.block_obj_ids;

    std::optional<std::string> block_obj = std::nullopt;
    double lowest_v = lane_speed_limit;
    double leader_v = lowest_v;
    double leader_init_v = leader_v;
    double nearest_obj_s = 0.0;

    // Set merge lane as stop line
    std::optional<double> merge_lane_s = std::nullopt;
    for (const auto& station : passage.stations()) {
      if (station.accumulated_s() < 0.0) continue;
      if (station.is_in_intersection() ||
          station.accumulated_s() >
              result.scheduler_output.length_along_route) {
        break;
      }
      if (station.is_merging()) {
        const double dist_to_merge =
            (passage.lane_seq_info() != nullptr)
                ? passage.lane_seq_info()->dist_to_merge
                : DBL_MAX;
        merge_lane_s = std::max(station.accumulated_s(), dist_to_merge);
        break;
      }
    }

    double nearest_stop_s = std::min(result.first_stop_s.value_or(DBL_MAX),
                                     merge_lane_s.value_or(DBL_MAX));
    const double stop_line_ref_v = CalculateProgressSpeed(
        ego_v, 0.0, nearest_stop_s, vehicle_geom.front_edge_to_center());
    if (stop_line_ref_v < lowest_v) {
      leader_v = 0.0;
      leader_init_v = 0.0;
      block_obj = "stopline";
      lowest_v = std::min(lowest_v, stop_line_ref_v);
      nearest_obj_s = nearest_stop_s;
    }

    absl::flat_hash_set<std::string> checked_set;
    std::vector<std::pair<FrenetBox, const SpacetimeObjectTrajectory*>>
        block_objs;
    Log2DDS::LogDataV2(
        "frame_dropped_trajectories.size()",
        st_traj_mgr_list[idx].frame_dropped_trajectories().size());
    for (const auto& traj :
         st_traj_mgr_list[idx].frame_dropped_trajectories()) {
      if (checked_set.contains(traj.object_id())) continue;
      checked_set.emplace(traj.object_id());
      if (!block_obj_ids.contains(traj.object_id())) continue;

      ASSIGN_OR_CONTINUE(const auto aabbox,
                         passage.QueryFrenetBoxAtContour(traj.contour()));
      block_objs.push_back(std::make_pair(aabbox, &traj));
      // consider when leader is accelerating when start up
      const double obs_perception_v = traj.planner_object().pose().v();
      double max_obj_prediction_v = obs_perception_v;
      // for (const auto& traj_state : traj.states()) {
      //   if (traj_state.traj_point->t() > kPreviewTimeForLeaderVel) break;
      //   max_obj_prediction_v =
      //       std::max(max_obj_prediction_v, traj_state.traj_point->v());
      // }
      const double obj_ref_v = max_obj_prediction_v;
      double obj_v_lim = CalculateProgressSpeed(
          ego_v, obj_ref_v, aabbox.s_min, min_front_dist);

      bool is_half_right_lane = false;
      if (lane_feature_info.valid_lane_num >= 0 &&
          lane_feature_info.right_index >= 0) {
        is_half_right_lane = lane_feature_info.right_index == 1 &&
                                 lane_feature_info.valid_lane_num >= 4 ||
                             lane_feature_info.right_index == 0 &&
                                 lane_feature_info.valid_lane_num >= 3;
      }
      Log2DDS::LogDataV2("is_half_right_lane", is_half_right_lane);
      Log2DDS::LogDataV2("lane_feature_info.right_index",
                         lane_feature_info.right_index);
      Log2DDS::LogDataV2("lane_feature_info.valid_lane_num",
                         lane_feature_info.valid_lane_num);

      // Highway right lc
      constexpr double kLeaderV = Kph2Mps(60.0);
      const bool is_highway_right_change_lane =
          ego_v > kLeaderV && is_half_right_lane &&
          result.scheduler_output.lane_change_state.stage() ==
              LaneChangeStage::LCS_EXECUTING &&
          !result.scheduler_output.lane_change_state.lc_left();
      if (is_highway_right_change_lane) obj_v_lim = obj_ref_v;
      Log2DDS::LogDataV2("is_highway_right_change_lane",
                         is_highway_right_change_lane);

      if (obj_v_lim < lowest_v) {
        leader_init_v = obs_perception_v;
        leader_v = obj_ref_v;
        lowest_v = obj_v_lim;
        block_obj = std::string(traj.object_id());
        nearest_obj_s = aabbox.s_min;
      }
    }

    std::optional<SlowWorkingObject> slow_working_object = std::nullopt;
    if (!block_objs.empty()) {
      const auto& lane_info_ptr =
          psmm.FindCurveLaneByIdOrNull(passage.lane_path().front().lane_id());
      bool is_leftmost_lane = false, is_rightmost_lane = false;
      if (lane_info_ptr != nullptr) {
        is_leftmost_lane = lane_info_ptr->left_lane_id() == 0;
        is_rightmost_lane = lane_info_ptr->right_lane_id() == 0;
      }
      std::stable_sort(block_objs.begin(), block_objs.end(),
                       [](const auto& lhs, const auto& rhs) {
                         return lhs.first.s_min < rhs.first.s_min;
                       });
      const auto& block_obj = block_objs.front();

      const double ego_s_max_position =
          result.scheduler_output.av_frenet_box_on_drive_passage.s_max;
      double slow_working_obj_prob = IsSlowWorkingLargeObjectProb(
          lane_speed_limit, common_feature.time_since_last_red_light, ego_v,
          ego_s_max_position, block_obj.first.s_min, *block_obj.second,
          is_leftmost_lane, is_rightmost_lane);

      const int leader_object_count = CountLeaderObjects(block_objs);
      slow_working_obj_prob = slow_working_obj_prob / (leader_object_count + 1);
      const bool is_slow_working_object =
          slow_working_obj_prob >= kSlowWorkingObjProb;
      const bool is_large_vehicle =
          block_obj.second->planner_object().is_large_vehicle();
      slow_working_object = SlowWorkingObject{
          .object_id = std::string(block_obj.second->object_id()),
          .object_s = block_obj.first.s_min,
          .object_l = block_obj.first.center_l(),
          .object_len = block_obj.first.length(),
          .probability = slow_working_obj_prob,
          .leader_count = leader_object_count,
          .is_large_vehicle = is_large_vehicle,
          .is_slow_working_object = is_slow_working_object};
      if (is_slow_working_object) {
        min_slow_working_object_s =
            std::min(min_slow_working_object_s, slow_working_object->object_s);
      }
    }
    slow_working_objs_map[idx] = std::move(slow_working_object);

    if (!lane_speed_map.contains(idx) ||
        lane_speed_map[idx].lane_speed_limit_by_leader > lowest_v) {
      lane_speed_map[idx] =
          LaneSpeedInfo{.init_leader_speed = leader_init_v,
                        .max_leader_speed = leader_v,
                        .lane_speed_limit_by_leader = lowest_v,
                        .lane_speed_limit = lane_speed_limit,
                        .block_obj = block_obj,
                        .nearest_obj_s = nearest_obj_s};
      max_lane_speed = std::max(max_lane_speed, lowest_v);
      min_lane_speed = std::min(min_lane_speed, lowest_v);
      max_init_leader_speed = std::max(leader_init_v, max_init_leader_speed);
    }
    if (idx == 0) lane_keep_init_leader_speed = leader_init_v;
  }
}

RouteLookAheadStats::RouteLookAheadStats(
    const SelectorCommonFeature& common_feature,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results) {
  // 0. get navi start
  const auto& map = psmm.map_ptr();
  if (map && map->route()) {
    const auto& temp_navi_sart = map->route()->navi_start();
    // copy value to private variable
    navi_sart.section_id = temp_navi_sart.section_id;
    navi_sart.s_offset = temp_navi_sart.s_offset;
  }

  // 1. get parameters from inputs
  int front_lane_size = 3;
  bool ego_lane_is_on_ramp_dec = false;
  Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const auto& ego_lane = psmm.GetNearestLane(ego_pos);
  if (ego_lane) {
    ego_lane_is_on_ramp_dec =
        ego_lane->type() == ad_byd::planning::LaneType::LANE_RAMP ||
        ego_lane->type() == ad_byd::planning::LaneType::LANE_DEC;
  }

  const double length_before_intersection =
      std::min(kInvalidLength, common_feature.road_horizon_info.dist_to_cross);
  const bool in_high_way =
      common_feature.in_high_way || common_feature.preview_in_high_way;
  is_left_turn = common_feature.is_left_turn;
  is_right_turn = common_feature.is_right_turn;

  // 2. generate is right most lane for each policy
  double min_driving_dist = kInvalidLength;
  double dist_to_ramp = psmm.map_ptr()->v2_info().dist_to_ramp;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) {
      continue;
    }
    const auto& passage = results[idx].scheduler_output.drive_passage;
    const auto start_lane_id = passage.lane_path().front().lane_id();

    const int lc_num_to_targets = (passage.lane_seq_info() != nullptr)
                                      ? passage.lane_seq_info()->lc_num
                                      : 0;
    const double dist_to_merge = (passage.lane_seq_info() != nullptr)
                                     ? passage.lane_seq_info()->dist_to_merge
                                     : 0.0;
    min_lc_num = std::min(min_lc_num, lc_num_to_targets);
    max_lc_num = std::max(max_lc_num, lc_num_to_targets);
    if (lc_num_to_targets == min_lc_num) {
      // When congestion_factor is greater than zero, it means the road has
      // heavy traffic.
      traffic_congestion_factor =
          results[idx].scheduler_output.traffic_congestion_factor -
          results[idx].scheduler_output.standard_congestion_factor;
    }
    if (results[idx].scheduler_output.drive_passage.lane_seq_info() !=
        nullptr) {
      min_driving_dist = std::min(
          min_driving_dist, results[idx]
                                .scheduler_output.drive_passage.lane_seq_info()
                                ->dist_to_navi_end);
    }
    const double dist_to_merge_vaild =
        std::min(min_driving_dist, dist_to_merge);
    max_length_before_merge_lane =
        std::max(max_length_before_merge_lane, dist_to_merge_vaild);

    bool is_right_most_lane = false;
    const auto& lane_feature_infos = common_feature.lane_feature_infos;
    if (lane_feature_infos.contains(idx)) {
      const auto& lane_feature_info = FindOrDieNoPrint(lane_feature_infos, idx);
      is_right_most_lane =
          (in_high_way && lane_feature_info.right_index == 0) ||
          lane_feature_info.preivew_right_index == 0;
      front_lane_size = lane_feature_info.valid_lane_num;
    }

    bool is_continuous_right_most_lane = true;
    for (int i = 0; i < passage.lane_path().size(); ++i) {
      const auto start_length = passage.lane_path().start_s(i);
      if (start_length > kPreviewLengthForRightMostLane) {
        break;
      }
      const auto lane_id = passage.lane_path().lane_id(i);
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, lane_id);
      is_continuous_right_most_lane =
          is_continuous_right_most_lane && lane_info.right_lane_id() == 0;
    }
    is_right_most_lane_map[idx] =
        is_right_most_lane || is_continuous_right_most_lane;
  }

  if (max_lc_num == min_lc_num) {
    traffic_congestion_factor = 0.0;
  }

  if (in_high_way) {
    enable_discourage_right_most_cost =
        (front_lane_size >= kCenterLangeChangeMinLaneNumHighway &&
         dist_to_ramp > kMinDrivingDistDiscourgeRightMost &&
         !ego_lane_is_on_ramp_dec);
  } else {
    enable_discourage_right_most_cost =
        (front_lane_size >= kCenterLangeChangeMinLaneNum &&
         min_driving_dist > kMinDrivingDistDiscourgeRightMost * 0.5);
  }

  enable_encourage_right_most_cost =
      (!in_high_way &&
       (min_driving_dist > kMinDrivingDistEncourgeRightMost || is_right_turn));

  // 4. adjust the length along route according to merge lane
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!IsValidBranch(est_status[idx])) {
      continue;
    }

    const auto& passage = results[idx].scheduler_output.drive_passage;
    const auto start_lane_id = passage.lane_path().front().lane_id();

    double length_along_route = kInvalidLength;
    double dist_to_merge = kInvalidLength;
    if (passage.lane_seq_info() != nullptr) {
      Log2DDS::LogDataV2("selectot_data",
                         "Candidate: length_along_route is seted");
      length_along_route =
          std::min(length_along_route,
                   results[idx]
                       .scheduler_output.drive_passage.lane_seq_info()
                       ->dist_to_navi_end);
      Log2DDS::LogDataV2(
          "selectot_data",
          absl::StrCat("Candidate: length_along_route: ", length_along_route));
      dist_to_merge = std::min(
          dist_to_merge, results[idx]
                             .scheduler_output.drive_passage.lane_seq_info()
                             ->dist_to_merge);
    }
    std::optional<double> curb_cut_off_s = std::nullopt;
    has_cross_curb_map[idx] = false;
    for (const auto& station : passage.stations()) {
      if (station.has_cross_curb()) {
        has_cross_curb_map[idx] = true;
        curb_cut_off_s = station.accumulated_s();
        Log2DDS::LogDataV2("selectot_data",
                           absl::StrCat("Candidate: curb_cut_off_s: ",
                                        station.accumulated_s()));
        break;
      }
    }
    const double raw_leng_along_route = length_along_route;
    raw_len_along_route_map[idx] = raw_leng_along_route;
    length_along_route =
        std::fmin(length_along_route, curb_cut_off_s.value_or(DBL_MAX));

    bool has_merge = false;
    for (int i = 0; i < passage.lane_path().lane_ids_size(); ++i) {
      const auto& lane_info =
          psmm.FindCurveLaneByIdOrNull(passage.lane_path().lane_id(i));
      if (!lane_info) break;

      // if (i == 0) {
      //   dist_to_merge +=
      //       lane_info->curve_length() * (cur_end_frac - cur_start_frac);
      // } else {
      //   dist_to_merge += lane_info->curve_length();
      // }

      // check if is_merging
      if (lane_info->merge_topology() ==
              ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_LEFT ||
          lane_info->merge_topology() ==
              ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_RIGHT) {
        has_merge = true;
        break;
      }
    }
    if (!has_merge) {
      dist_to_merge = kInvalidLength;
    }

    if (dist_to_merge > length_before_intersection - kLengthEpsilon ||
        dist_to_merge == std::numeric_limits<double>::infinity()) {
      // Ignore merge lane after intersection or length along route
      dist_to_merge = kInvalidLength;
    }

    const auto& lane_feature_infos = common_feature.lane_feature_infos;
    if (!lane_feature_infos.contains(idx)) continue;
    const auto& lane_feature_info = FindOrDieNoPrint(lane_feature_infos, idx);
    const auto& block_obj_ids = lane_feature_info.block_obj_ids;

    std::optional<StalledObjInfo> stalled_obj_info = std::nullopt;
    constexpr double kDoubleEpsilon = 1e-6;
    for (const auto& traj : st_traj_mgr_list[idx].trajectories()) {
      if (!block_obj_ids.contains(traj.object_id())) continue;
      if (!stalled_objects.contains(traj.object_id())) continue;
      if (IsConstructionObject(traj.object_type())) continue;
      ASSIGN_OR_CONTINUE(const auto aabbox,
                         passage.QueryFrenetBoxAtContour(traj.contour()));
      const auto obj_station = passage.FindNearestStationAtS(aabbox.center_s());
      const auto obj_lane_pt = obj_station.GetLanePoint();

      const auto obj_lane = psmm.FindLaneByIdOrNull(obj_lane_pt.lane_id());
      if (obj_lane == nullptr) continue;
      const auto& obj_sec = psmm.FindSectionByIdOrNull(obj_lane->section_id());
      if (obj_sec == nullptr) continue;
      const double punish_factor =
          (obj_sec->lanes().size() > 1 || obj_station.is_in_intersection())
              ? 1.0
              : kSingleLaneStalledFactor;
      if (!stalled_obj_info.has_value() ||
          stalled_obj_info->punish_factor < punish_factor ||
          (std::abs(stalled_obj_info->punish_factor - punish_factor) <
               kDoubleEpsilon &&
           stalled_obj_info->stalled_obj_s > aabbox.s_min)) {
        // Choose the biggest punish factor, then choose nearest obj
        stalled_obj_info =
            StalledObjInfo{.stalled_obj_id = std::string(traj.object_id()),
                           .stalled_obj_s = aabbox.s_min,
                           .punish_factor = punish_factor};
      }

      length_along_route = std::min(
          length_along_route, std::max(0.0, aabbox.s_min - kMinLcLaneLength));
    }

    const double curr_len_before_intersection =
        std::min(length_along_route, length_before_intersection);
    len_along_route_map[idx] = length_along_route;
    len_before_intersection_map[idx] = curr_len_before_intersection;
    front_stalled_obj_map[idx] = std::move(stalled_obj_info);
    len_before_merge_lane_map[idx] = dist_to_merge;
    if (raw_leng_along_route > max_length_along_route) {
      max_length_along_route = raw_leng_along_route;
    }
    if (raw_leng_along_route < min_length_along_route) {
      min_length_along_route = raw_leng_along_route;
    }
  }
}

MppSectionInfo BuildMppSectionInfo(const PlannerSemanticMapManager* psmm) {
  MppSectionInfo mpp_section_info;
  int valid_lane_num = 0, navigation_lane_num = 0;
  const auto& route = psmm->map_ptr()->route();
  if (route == nullptr) {
    return mpp_section_info;
  }
  const auto& sections = route->sections();
  const auto& navi_start = route->navi_start();
  const auto& navi_end = route->navi_end();
  bool enter_navi_section = false;
  mpp_section_info.start_s = navi_start.s_offset;
  for (const auto& section_info : sections) {
    if (!enter_navi_section && section_info.id != navi_start.section_id) {
      continue;
    }
    enter_navi_section = true;
    if (section_info.id == navi_end.section_id) {
      break;
    }
    if (section_info.id == mapping::kInvalidSectionId) {
      continue;
    }
    auto* section_lanes = &mpp_section_info.section_lanes;
    auto* section_lengths = &mpp_section_info.section_lengths;
    auto* section_ids = &mpp_section_info.section_ids;

    section_lanes->push_back({});
    section_ids->push_back(section_info.id);
    section_lengths->push_back(section_info.length);
    section_lanes->reserve(section_info.lane_ids.size());
    for (int lane_index = 0; lane_index < section_info.lane_ids.size();
         lane_index++) {
      const auto& lane_id = section_info.lane_ids[lane_index];
      const auto& lane_ptr = psmm->FindCurveLaneByIdOrNull(lane_id);
      section_lanes->back().push_back(lane_ptr);
    }
  }
  return mpp_section_info;
}

}  // namespace st::planning
