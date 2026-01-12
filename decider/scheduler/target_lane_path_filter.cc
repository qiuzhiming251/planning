
#include "plan_common/log_data.h"
#include "decider/scheduler/target_lane_path_filter.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <utility>
#include <valarray>

#include <float.h>
#include <limits.h>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "plan_common/constants.h"
#include "plan_common/log_data.h"
#include "plan_common/container/strong_int.h"
//#include "global/logging.h"
//#include "global/trace.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/plan_common_defs.h"
//#include "planner/planner_manager/planner_flags.h"
//#include "semantic_map.pb.h"
#include "plan_common/util/lane_point_util.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/planner_semantic_map_util.h"

namespace st::planning {

namespace {

// Should both be much smaller than -LaneGraph::kDeadEndToTargetCost.
constexpr double kLastTargetLanePathReward = -1e10;
constexpr double kPreferredLanePathReward = -1e12;

double ComputeLanePathCost(const PlannerSemanticMapManager& psmm,
                           const LanePathInfo& lp_info,
                           const RouteSectionsInfo& route_sections_info,
                           const RouteNaviInfo& route_navi_info) {
  std::vector<std::string> output_debug;
  std::string str = "";
  for (int i = 0; i < lp_info.lane_path().lane_ids_size(); ++i) {
    str += (lp_info.lane_path().lane_id(i) + ",");
  }
  output_debug.emplace_back(str);

  double cost = lp_info.path_cost();
  double raw_cost = cost;
  output_debug.emplace_back(absl::StrCat("raw_cost:", raw_cost));

  constexpr double kLcNumToTargetsWeight = 0.1;
  constexpr double kMergeLaneCostBase = 100.0;
  constexpr double kConsiderMaxDrivingDistance = 4000.0;  // m.
  constexpr double kLengthEpsilon = 0.1;                  // m.
  constexpr double kIntersectionAngleWeight = 40.0;
  constexpr double kIntersectionDistBase = 80.0;  // m.
  constexpr double kIntersectionDistInvalidBase = 1000.0;
  constexpr double kSectionIndexBase = 5.0;
  constexpr double kSectionIndexWeight = 5.0;
  constexpr double kSplitWeight = 1.0;
  constexpr double kLaneTypeWeight = 1000.0;
  // Minor difference based on global route.
  const auto* lane_navi_info_ptr =
      FindOrNull(route_navi_info.route_lane_info_map, lp_info.start_lane_id());
  if (lane_navi_info_ptr == nullptr) return DBL_MAX;
  cost += route_sections_info.planning_horizon() /
          (std::min(lane_navi_info_ptr->max_driving_distance,
                    kConsiderMaxDrivingDistance) +
           kLengthEpsilon);
  output_debug.emplace_back(absl::StrCat("cost with route:", cost));

  int lc_num = lane_navi_info_ptr->min_lc_num_to_target;

  const double cur_start_frac = route_sections_info.front().start_fraction;
  const double cur_end_frac = route_sections_info.front().end_fraction;
  double dist_to_merge = 0.0;
  bool has_merge = false;
  for (int i = 0; i < lp_info.lane_path().lane_ids_size(); ++i) {
    const auto& lane_info =
        psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i));
    if (!lane_info) break;
    if (i == 0) {
      dist_to_merge +=
          lane_info->curve_length() * (cur_end_frac - cur_start_frac);
    } else {
      dist_to_merge += lane_info->curve_length();
    }
    // check if is_merging
    if (lane_info->merge_topology() == ad_byd::planning::TOPOLOGY_MERGE_LEFT ||
        lane_info->merge_topology() == ad_byd::planning::TOPOLOGY_MERGE_RIGHT) {
      has_merge = true;
      break;
    }
  }
  if (!has_merge) {
    dist_to_merge = DBL_MAX;
  }
  output_debug.emplace_back(absl::StrCat("dist_to_merge:", dist_to_merge));

  double len_before_merge = DBL_MAX;
  for (const auto& lane_seg :
       lp_info.lane_path().BeforeArclength(lp_info.max_reach_length())) {
    const auto* lane_navi_info_ptr =
        FindOrNull(route_navi_info.route_lane_info_map, lane_seg.lane_id);
    lc_num = std::max(lc_num, lane_navi_info_ptr == nullptr
                                  ? INT_MAX
                                  : lane_navi_info_ptr->min_lc_num_to_target);
    len_before_merge = std::min(
        len_before_merge,
        lane_seg.start_s + (lane_navi_info_ptr == nullptr
                                ? 0.0
                                : lane_navi_info_ptr->len_before_merge_lane));
  }
  // Intersection STRAIGHT angle cost.
  bool has_dist_cost = false;
  int bend_dir = 0;  // none:0, left:1, right:2
  for (int i = 0; i < lp_info.lane_path().lane_ids_size(); ++i) {
    const auto& lane_info =
        psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i));
    ad_byd::planning::LaneConstPtr pre_lane_info = nullptr;
    ad_byd::planning::LaneConstPtr next_lane_info = nullptr;
    if (i >= 1) {
      pre_lane_info =
          psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i - 1));
    } else if (!lane_info->next_lane_ids().empty()) {
      pre_lane_info =
          psmm.FindCurveLaneByIdOrNull(lane_info->next_lane_ids().front());
    }
    if (!lane_info->next_lane_ids().empty() &&
        i < lp_info.lane_path().lane_ids_size() - 1) {
      next_lane_info =
          psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i + 1));
    }
    // check bend straight
    if (pre_lane_info && next_lane_info && lane_info &&
        !(lane_info->junction_id() == 0) &&
        (next_lane_info->junction_id() == 0) &&
        (pre_lane_info->junction_id() == 0)) {
      mapping::LanePoint pre_lane_pt(pre_lane_info->id(), 1.0);
      const auto prev_lane_vec = ComputeLanePointLerpTheta(psmm, pre_lane_pt);
      mapping::LanePoint next_lane_pt(next_lane_info->id(), 1.0);
      const auto next_lane_vec = ComputeLanePointLerpTheta(psmm, next_lane_pt);
      const double angle = NormalizeAngle(next_lane_vec - prev_lane_vec);
      if (angle > std::sin(8.0 * ad_byd::planning::Constants::DEG2RAD)) {
        bend_dir = 1;
      } else if (angle <
                 -std::sin(8.0 * ad_byd::planning::Constants::DEG2RAD)) {
        bend_dir = 2;
      }
      output_debug.emplace_back(
          absl::StrCat("check bend straight angle:", angle));
    }
    if (bend_dir != 0) break;
    if (pre_lane_info && lane_info && !(lane_info->junction_id() == 0)) {
      if (lane_info->turn_type() != ad_byd::planning::NO_TURN) break;
      mapping::LanePoint pre_lane_pt(pre_lane_info->id(), 1.0);
      const auto prev_lane_vec = ComputeLanePointLerpTheta(psmm, pre_lane_pt);
      mapping::LanePoint lane_pt(lp_info.lane_path().lane_id(i), 0.5);
      const auto lane_vec = ComputeLanePointLerpTheta(psmm, lane_pt);
      double angle_cost = kIntersectionAngleWeight *
                          std::fabs(NormalizeAngle(lane_vec - prev_lane_vec));
      // LOG_INFO << "inter_lane_info.id " << lp_info.lane_path().lane_id(i)
      //           << " prev_lane_angle " << prev_lane_vec << " lane_angle "
      //           << lane_vec << "angle cost = " << angle_cost;
      // cost += angle_cost;
      double dis_cost = kIntersectionAngleWeight * lane_info->curve_length() /
                        kIntersectionDistBase;
      LOG_INFO << "inter_lane_info.id " << lp_info.lane_path().lane_id(i)
               << " length:" << lane_info->curve_length()
               << " dis_cost:" << dis_cost;
      cost += dis_cost;
      has_dist_cost = true;
      break;
    }
  }
  if (!has_dist_cost) {
    cost += kIntersectionDistInvalidBase;
  }
  output_debug.emplace_back(absl::StrCat("cost with dis cost:", cost));

  // Intersection TURN index cost.
  for (int i = 0; i + 1 < lp_info.lane_path().lane_ids_size(); ++i) {
    const auto& lane_info =
        psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i));
    ad_byd::planning::LaneConstPtr pre_lane_info = nullptr;
    if (i >= 1) {
      pre_lane_info =
          psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i - 1));
    } else if (!lane_info->next_lane_ids().empty()) {
      pre_lane_info =
          psmm.FindCurveLaneByIdOrNull(lane_info->next_lane_ids().front());
    }
    const auto& next_lane_info =
        psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i + 1));
    if (pre_lane_info && next_lane_info && lane_info &&
        !(lane_info->junction_id() == 0) &&
        (lane_info->turn_type() == ad_byd::planning::LEFT_TURN ||
         lane_info->turn_type() == ad_byd::planning::RIGHT_TURN ||
         bend_dir != 0)) {
      const auto& pre_section_info =
          psmm.FindSectionByIdOrNull(pre_lane_info->section_id());
      const auto& next_section_info =
          psmm.FindSectionByIdOrNull(next_lane_info->section_id());
      if (pre_section_info != nullptr && next_section_info != nullptr) {
        int pre_index = -1, pre_section_size = 0;
        int next_index = -1, next_section_size = 0;
        for (int idx = 0; idx < pre_section_info->lanes().size(); idx++) {
          const auto& info =
              psmm.FindCurveLaneByIdOrNull(pre_section_info->lanes()[idx]);
          if (!info ||
              (info->type() != ad_byd::planning::LANE_UNKNOWN &&
               info->type() != ad_byd::planning::LANE_BUS_NORMAL &&
               info->type() != ad_byd::planning::LANE_VIRTUAL_JUNCTION &&
               info->type() != ad_byd::planning::LANE_VIRTUAL_COMMON &&
               info->type() != ad_byd::planning::LANE_ROUND_ABOUT)) {
            continue;
          }
          if (pre_section_info->lanes()[idx] == pre_lane_info->id()) {
            pre_index = idx;
          }
          pre_section_size++;
        }
        for (int idx = 0; idx < next_section_info->lanes().size(); idx++) {
          const auto& info =
              psmm.FindCurveLaneByIdOrNull(next_section_info->lanes()[idx]);
          if (!info ||
              (info->type() != ad_byd::planning::LANE_UNKNOWN &&
               info->type() != ad_byd::planning::LANE_BUS_NORMAL &&
               info->type() != ad_byd::planning::LANE_VIRTUAL_JUNCTION &&
               info->type() != ad_byd::planning::LANE_VIRTUAL_COMMON &&
               info->type() != ad_byd::planning::LANE_ROUND_ABOUT)) {
            continue;
          }
          if (next_section_info->lanes()[idx] == next_lane_info->id()) {
            next_index = idx;
          }
          next_section_size++;
        }
        double index_cost = kSectionIndexWeight;
        if (pre_index != -1 && next_index != -1) {
          // lane index from left 1 to right 1.
          double index_diff =
              (lane_info->turn_type() == ad_byd::planning::LEFT_TURN ||
               bend_dir == 1)
                  ? next_index - pre_index
                  : (next_section_size - next_index) -
                        (pre_section_size - pre_index);
          // enter round-about and not right 1st
          if (lane_info->type() != ad_byd::planning::LANE_ROUND_ABOUT &&
              next_lane_info->type() == ad_byd::planning::LANE_ROUND_ABOUT &&
              next_section_size > 2 && (pre_section_size != pre_index + 1)) {
            index_diff = next_section_size - 2 - next_index;
          }
          index_cost = kSectionIndexWeight *
                       static_cast<double>(std::abs(index_diff)) /
                       kSectionIndexBase;
          cost += index_cost;
        }
        cost += index_cost;
        LOG_INFO << "inter_lane_id " << lane_info->id() << " prev_lane_id "
                 << pre_lane_info->id() << " next_lane_id "
                 << next_lane_info->id() << " turn index_cost = " << index_cost
                 << " cost = " << cost << " raw_cost = " << raw_cost;
      }
      break;
    }
  }
  output_debug.emplace_back(absl::StrCat("cost with turn cost:", cost));

  // Split cost.
  for (int i = 0; i < lp_info.lane_path().lane_ids_size(); ++i) {
    const auto& lane_info =
        psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i));
    if (lane_info && lane_info->junction_id() == 0 && lane_info->IsVirtual()) {
      double split_cost = kSplitWeight;
      LOG_INFO << "inter_lane_info.id " << lp_info.lane_path().lane_id(i)
               << "split_cost = " << split_cost;
      cost += split_cost;
      break;
    }
  }
  output_debug.emplace_back(absl::StrCat("cost with split cost:", cost));

  // Lane type cost
  for (int i = 0; i < lp_info.lane_path().lane_ids_size(); ++i) {
    const auto& lane_info =
        psmm.FindCurveLaneByIdOrNull(lp_info.lane_path().lane_id(i));
    if (lane_info && lane_info->type() == ad_byd::planning::LANE_NON_MOTOR) {
      LOG_INFO << "non motor lane cost";
      return DBL_MAX;
    }
  }

  cost += kLcNumToTargetsWeight * lc_num;
  output_debug.emplace_back(absl::StrCat("cost with lc num cost:", cost));

  cost += kMergeLaneCostBase / (dist_to_merge + kMergeLaneCostBase);
  output_debug.emplace_back(absl::StrCat("cost with merge cost:", cost));

  Log2DDS::LogDataV2("filter-cost", output_debug);
  return cost;
}

absl::flat_hash_map<mapping::ElementId, int> CollectLaneIndexDiff(
    const PlannerSemanticMapManager& psmm,
    const std::vector<LanePathInfo>& lp_infos,
    const RouteSectionsInfo& route_sections_info,
    const ApolloTrajectoryPointProto& plan_start_point) {
  // NOLINTBEGIN(readability-function-cognitive-complexity)
  const auto& cur_lane_ids = route_sections_info.front().lane_ids;
  const int lane_id_size = cur_lane_ids.size();
  const double cur_start_frac = route_sections_info.front().start_fraction;
  const double cur_end_frac = route_sections_info.front().end_fraction;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);

  // Left -> right: negative -> positive.
  std::vector<double> l_offsets(lane_id_size);
  int neutral_next_idx = -1;
  mapping::ElementId closest_lane_id;
  for (int i = 0; i < lane_id_size; ++i) {
    const mapping::LanePath lane_path(psmm.map_ptr(), {cur_lane_ids[i]},
                                      cur_start_frac, cur_end_frac);
    const auto frenet_frame_or =
        BuildBruteForceFrenetFrame(SampleLanePathPoints(psmm, lane_path),
                                   /*down_sample_raw_points=*/true);
    if (frenet_frame_or.ok()) {
      l_offsets[i] = frenet_frame_or->XYToSL(ego_pos).l;
    } else {
      const mapping::LanePoint lane_pt(cur_lane_ids[i], cur_start_frac);
      l_offsets[i] =
          ComputeLanePointTangent(psmm, lane_pt)
              .CrossProd(ego_pos - ComputeLanePointPos(psmm, lane_pt));
    }
    if (i > 0 && l_offsets[i - 1] * l_offsets[i] < 0.0) {
      neutral_next_idx = i;
      closest_lane_id = std::abs(l_offsets[i - 1]) < std::abs(l_offsets[i])
                            ? cur_lane_ids[i - 1]
                            : cur_lane_ids[i];
    }
  }
  if (neutral_next_idx == -1) {
    neutral_next_idx = l_offsets[0] > 0.0 ? 0 : lane_id_size;
    closest_lane_id =
        l_offsets[0] > 0.0 ? cur_lane_ids[0] : cur_lane_ids[lane_id_size - 1];
  }

  absl::flat_hash_map<mapping::ElementId, double> reach_length_map;
  for (const auto& lp_info : lp_infos) {
    const auto start_id = lp_info.start_lane_id();
    const auto start_id_iter = reach_length_map.find(start_id);
    if (start_id_iter != reach_length_map.end()) {
      start_id_iter->second =
          std::max(start_id_iter->second, lp_info.max_reach_length());
    } else {
      reach_length_map.insert({start_id, lp_info.max_reach_length()});
    }
  }
  // Allow continuous lane change if the rest distance is not enough.
  const bool allow_cont_lc = FindWithDefault(reach_length_map, closest_lane_id,
                                             0.0) < 0.5 * kMinLcLaneLength;
  const double lat_dist_thres =
      allow_cont_lc ? 1.5 * kDefaultHalfLaneWidth : kMaxLaneKeepLateralOffset;

  constexpr double kMinReachLengthContLc = 2.0 * kMinLcLaneLength;
  absl::flat_hash_map<mapping::ElementId, int> results;
  if (neutral_next_idx > 0) {  // From ego to the left side.
    int index_diff =
        std::abs(l_offsets[neutral_next_idx - 1]) < lat_dist_thres ? 0 : -1;
    for (int i = neutral_next_idx - 1; i > 0; --i) {
      results[cur_lane_ids[i]] = index_diff;
      if (l_offsets[i] - l_offsets[i - 1] < kMaxLaneKeepLateralOffset) continue;

      if (allow_cont_lc && index_diff == 0 &&
          FindWithDefault(reach_length_map, cur_lane_ids[i], 0.0) >
              kMinReachLengthContLc) {
        --index_diff;
      }
      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, cur_lane_ids[i - 1], {});
      if ((lane_info.merge_topology() !=
               ad_byd::planning::TOPOLOGY_MERGE_LEFT &&
           lane_info.merge_topology() !=
               ad_byd::planning::TOPOLOGY_MERGE_RIGHT) ||
          l_offsets[i] - l_offsets[i - 1] > kDefaultHalfLaneWidth) {
        --index_diff;
      }
    }
    results[cur_lane_ids[0]] = index_diff;
  }
  if (neutral_next_idx < lane_id_size) {  // From ego to the right side.
    int index_diff =
        std::abs(l_offsets[neutral_next_idx]) < lat_dist_thres ? 0 : 1;
    for (int i = neutral_next_idx; i + 1 < lane_id_size; ++i) {
      results[cur_lane_ids[i]] = index_diff;
      if (l_offsets[i + 1] - l_offsets[i] < kMaxLaneKeepLateralOffset) continue;

      if (allow_cont_lc && index_diff == 0 &&
          FindWithDefault(reach_length_map, cur_lane_ids[i], 0.0) >
              kMinReachLengthContLc) {
        ++index_diff;
      }
      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, cur_lane_ids[i + 1], {});
      if ((lane_info.merge_topology() !=
               ad_byd::planning::TOPOLOGY_MERGE_LEFT &&
           lane_info.merge_topology() !=
               ad_byd::planning::TOPOLOGY_MERGE_RIGHT) ||
          l_offsets[i + 1] - l_offsets[i] > kDefaultHalfLaneWidth) {
        ++index_diff;
      }
    }
    results[cur_lane_ids[lane_id_size - 1]] = index_diff;
  }

  return results;
}  // NOLINTEND(readability-function-cognitive-complexity)

int FindMostSimilarLanePathIndexToLastTargetLanePath(
    const std::vector<LanePathInfo>& lp_infos,
    const std::vector<std::pair<int, double>>& lane_path_costs,
    const mapping::LanePath& last_target_lane_path, double ego_v) {
  if (last_target_lane_path.IsEmpty()) return -1;

  constexpr double kCostEpsilon = 1e-10;
  constexpr double kMinSharedTimeOnLanePath = 2.0;  // s.
  const double shared_len_thres = kMinSharedTimeOnLanePath * ego_v;

  int best_index = -1;
  double best_cost = DBL_MAX;
  for (int i = 0; i < lp_infos.size(); ++i) {
    if (lp_infos[i].start_lane_id() !=
        last_target_lane_path.front().lane_id()) {
      // Assume already aligned.
      continue;
    }

    const auto lane_path =
        lp_infos[i].lane_path().BeforeArclength(lp_infos[i].max_reach_length());
    const int lp_size =
        std::min(lane_path.size(), last_target_lane_path.size());
    int last_shared_lane_idx = 0;
    while (last_shared_lane_idx + 1 < lp_size &&
           lane_path.lane_id(last_shared_lane_idx + 1) ==
               last_target_lane_path.lane_id(last_shared_lane_idx + 1)) {
      ++last_shared_lane_idx;
    }
    const double shared_len = lane_path.LaneIndexPointToArclength(
        last_shared_lane_idx,
        std::min(lane_path.lane_segment(last_shared_lane_idx).end_fraction,
                 last_target_lane_path.lane_segment(last_shared_lane_idx)
                     .end_fraction));

    if (shared_len >= shared_len_thres &&
        lane_path_costs[i].second < best_cost - kCostEpsilon) {
      best_index = i;
      best_cost = lane_path_costs[i].second;
    }
  }

  return best_index;
}

std::pair<mapping::LanePoint, mapping::LanePoint> FindNeighborLanePoints(
    const RouteSectionsInfo& route_sections_info,
    const mapping::LanePoint& lane_pt) {
  const auto& cur_sec =
      *route_sections_info.FindSegmentContainingLanePointOrNull(lane_pt);
  const int cur_idx = FindOrDie(cur_sec.id_idx_map, lane_pt.lane_id());

  auto left_lane_pt = cur_idx > 0
                          ? mapping::LanePoint(cur_sec.lane_ids[cur_idx - 1],
                                               lane_pt.fraction())
                          : mapping::LanePoint();
  auto right_lane_pt = cur_idx + 1 < cur_sec.lane_ids.size()
                           ? mapping::LanePoint(cur_sec.lane_ids[cur_idx + 1],
                                                lane_pt.fraction())
                           : mapping::LanePoint();
  return {left_lane_pt, right_lane_pt};
}

}  // namespace

bool GetMatchLanePathByPnpInfos(
    const absl::flat_hash_set<mapping::ElementId>& pnp_lanes,
    const std::vector<LanePathInfo>& lp_infos, LanePathInfo* match_lp) {
  // std::vector<mapping::ElementId> pnp_lane_ids;
  // for (const auto& id : pnp_info.target_lane_sequence_ids()) {
  //   Log2DDS::LogDataV0("lc_debug", absl::StrCat("target id: ", id));
  //   pnp_lane_ids.push_back(static_cast<mapping::ElementId>(id));
  // }
  // absl::flat_hash_set<mapping::ElementId> pnp_lanes(pnp_lane_ids.begin(),
  //                                                   pnp_lane_ids.end());
  const int n_lps = lp_infos.size();
  std::vector<int> repeat_num(n_lps, 0);
  for (int i = 0; i < n_lps; ++i) {
    for (const auto& id : lp_infos[i].lane_path().lane_ids()) {
      if (pnp_lanes.contains(id)) {
        Log2DDS::LogDataV2("lc_debug", absl::StrCat("match id: ", id));
        repeat_num[i] += 1;
      }
    }
  }
  std::vector<int>::iterator biggest =
      std::max_element(std::begin(repeat_num), std::end(repeat_num));
  if (*biggest == 0) return false;
  int index = std::distance(std::begin(repeat_num), biggest);
  *match_lp = lp_infos[index];
  return true;
}

bool GetMatchLanePathByPnpInfos(const PNPInfo& pnp_info,
                                const std::vector<LanePathInfo>& lp_infos,
                                LanePathInfo* match_lp) {
  std::vector<mapping::ElementId> pnp_lane_ids;
  for (const auto& id : pnp_info.target_lane_sequence_ids()) {
    Log2DDS::LogDataV0("lc_debug", absl::StrCat("target id: ", id));
    pnp_lane_ids.push_back(static_cast<mapping::ElementId>(id));
  }
  absl::flat_hash_set<mapping::ElementId> pnp_lanes(pnp_lane_ids.begin(),
                                                    pnp_lane_ids.end());
  const int n_lps = lp_infos.size();
  std::vector<int> repeat_num(n_lps, 0);
  for (int i = 0; i < n_lps; ++i) {
    for (const auto& id : lp_infos[i].lane_path().lane_ids()) {
      if (pnp_lanes.contains(id)) {
        Log2DDS::LogDataV2("lc_debug", absl::StrCat("match id: ", id));
        repeat_num[i] += 1;
      }
    }
  }
  std::vector<int>::iterator biggest =
      std::max_element(std::begin(repeat_num), std::end(repeat_num));
  if (*biggest == 0) return false;
  int index = std::distance(std::begin(repeat_num), biggest);
  *match_lp = lp_infos[index];
  return true;
}

bool SelectSmoothCandidateLanePath(const PlannerSemanticMapManager& psmm,
                                   const PNPInfo& pnp_info,
                                   const std::vector<LanePathInfo>& lp_infos,
                                   LanePathInfo* smooth_lp) {
  // // select best candidate laneseq
  // Log2DDS::LogDataV2("lc_debug",
  //                    absl::StrCat("find_cand_seqs(size=",
  //                                 pnp_info.candidate_lane_seq_size()));
  // double min_cost = std::numeric_limits<double>::max();
  // int min_cost_index = -1;
  // for (int seq_index = 0; seq_index < pnp_info.candidate_lane_seq_size();
  //      seq_index++) {
  //   const auto seq = pnp_info.candidate_lane_seq(seq_index);
  //   double seq_cost = 0.0;
  //   for (int i = 0; i < seq.candidate_lane_id_size(); i++) {
  //     const auto& lane_info =
  //     psmm.FindLaneInfoOrNull(seq.candidate_lane_id(i)); if (!lane_info)
  //     continue; if (lane_info->is_in_intersection && i > 0 &&
  //         lane_info->direction ==
  //             mapping::LaneProto::STRAIGHT) {  // split geo cost
  //       const auto& pre_lane_info =
  //           psmm.FindLaneInfoOrNull(seq.candidate_lane_id(i - 1));
  //       if (!pre_lane_info) continue;
  //       mapping::LanePoint pre_lane_pt(pre_lane_info->id, 1.0);
  //       const auto prev_lane_vec = ComputeLanePointLerpTheta(psmm,
  //       pre_lane_pt); mapping::LanePoint lane_pt(lane_info->id, 0.5); const
  //       auto lane_vec = ComputeLanePointLerpTheta(psmm, lane_pt); seq_cost
  //       += 10.0 * std::fabs(NormalizeAngle(lane_vec - prev_lane_vec));
  //     } else if (lane_info->IsVirtual()) {  // split topo cost
  //       seq_cost += 10.0;
  //     }
  //     if (true) {  // navi cost
  //       seq_cost -= 20.0;
  //     }
  //   }
  //   if (min_cost > seq_cost) {
  //     min_cost = seq_cost;
  //     min_cost_index = seq_index;
  //   }
  // }
  // // match lane path
  // if (min_cost_index != -1) {
  //   const auto best_seq = pnp_info.candidate_lane_seq(min_cost_index);
  //   absl::flat_hash_set<std::string> lane_ids(
  //       best_seq.candidate_lane_id().begin(),
  //       best_seq.candidate_lane_id().end());
  //   Log2DDS::LogDataV2("lc_debug",
  //                      absl::StrCat("find_best_seq(size=", lane_ids.size()));
  //   return GetMatchLanePathByPnpInfos(lane_ids, lp_infos, smooth_lp);
  // }
  // Log2DDS::LogDataV2("lc_debug", "without_find_best_seq");
  return false;
}

std::vector<LanePathInfo> FilterMultipleTargetLanePath(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& route_sections_info,
    const RouteNaviInfo& route_navi_info,
    const mapping::LanePath& last_target_lane_path,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& preferred_lane_path,
    const std::shared_ptr<PNPInfos>& pnp_infos,
    std::vector<LanePathInfo>* mutable_lp_infos,
    const LaneChangeStage& lane_change_stage, bool pre_lc_direction_left,
    PnpTop1History* top1_history) {
  // NOLINTBEGIN(readability-function-cognitive-complexity)
  // ("FilterMultipleTargetLanePath");

  const auto lp_infos = std::move(*mutable_lp_infos);
  const int n_lps = lp_infos.size();
  bool pnp_fail_cur = true;
  LanePathInfo pnp_top1;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  if (pnp_infos && preferred_lane_path.IsEmpty()) {
    std::vector<LanePathInfo> results;
    Log2DDS::LogDataV2("lc_debug", absl::StrCat("pnp_infos size: ",
                                                pnp_infos->infos().size()));
    for (const auto& pnp_info : pnp_infos->infos()) {
      LanePathInfo smooth_lane_path;
      if (SelectSmoothCandidateLanePath(psmm, pnp_info, lp_infos,
                                        &smooth_lane_path)) {
        std::string str = "";
        for (int i = 0; i < smooth_lane_path.lane_path().lane_ids_size(); ++i) {
          str += (smooth_lane_path.lane_path().lane_id(i) + ",");
        }
        Log2DDS::LogDataV2("lc_debug", "smooth lp: " + str);
        mutable_lp_infos->emplace_back(smooth_lane_path);
        results.emplace_back(smooth_lane_path);
      }
    }

    if (lane_change_stage == LaneChangeStage::LCS_EXECUTING ||
        lane_change_stage == LaneChangeStage::LCS_RETURN) {
      // 1. find last target lane path from input candidate lp_infos
      std::vector<mapping::ElementId> prev_target_lane_ids;
      for (const auto& last_id : last_target_lane_path.lane_ids()) {
        prev_target_lane_ids.push_back(
            static_cast<mapping::ElementId>(last_id));
      }
      absl::flat_hash_set<mapping::ElementId> pre_target_lane_ids_set(
          prev_target_lane_ids.begin(), prev_target_lane_ids.end());

      LanePathInfo target_lane_path;
      if (GetMatchLanePathByPnpInfos(pre_target_lane_ids_set, lp_infos,
                                     &target_lane_path)) {
        // 2. find last target lc path from results
        bool flag_find_prev_target = false;
        for (const auto& id : target_lane_path.lane_path().lane_ids()) {
          Log2DDS::LogDataV0("lc_pnp_prevent_jump",
                             absl::StrCat("matched prev target id: ", id));
        }
        for (int i = 0; i < results.size(); i++) {
          flag_find_prev_target =
              (target_lane_path.lane_path() == results[i].lane_path());
          Log2DDS::LogDataV2("lc_pnp_prevent_jump",
                             absl::StrCat("result index: ", i));
          Log2DDS::LogDataV2("lc_pnp_prevent_jump",
                             absl::StrCat("result matched true or false: ",
                                          flag_find_prev_target));
          // prev_target_lc_path exists in results:
          // case 1: target is the top 1, no action
          // case 2: target is the top 2, exchange top 1 and top 2
          if (flag_find_prev_target && i == 0) {
            break;
          } else if (flag_find_prev_target && i != 0) {
            std::swap(results[0], results[1]);
          }
        }
        // prev_target_lc_path does not exist in results, add to the beginning
        // of results
        if (!flag_find_prev_target) {
          results.insert(results.begin(), target_lane_path);
          mutable_lp_infos->emplace_back(target_lane_path);
        }
        // resize the size of results (max 2)
        if (results.size() > 2) {
          results.pop_back();
        }
      } else {  // TODO: if it exists a better solution when prev target is lost
        Log2DDS::LogDataV0("lc_pnp_prevent_jump",
                           "Not find prev target lc path");
      }
    }

    for (const auto& lp_info : results) {
      const auto ego_sl = lp_info.ProjectionSL(ego_pos);
      if (0.5 * kDefaultLaneWidth > std::fabs(ego_sl.l)) {
        pnp_fail_cur = false;
        break;
      }
    }
    if (pnp_fail_cur && !results.empty()) {
      pnp_top1 = results.front();
    } else if (results.empty()) {
      pnp_fail_cur = false;
    } else {
      return results;
    }
  } else {
    pnp_fail_cur = false;
  }

  const auto lane_idx_diff_map = CollectLaneIndexDiff(
      psmm, lp_infos, route_sections_info, plan_start_point);
  // Vector indices correspond to start lanes' indices in the current section.
  std::vector<std::pair<int, double>> lane_path_costs(n_lps);
  for (int i = 0; i < n_lps; ++i) {
    lane_path_costs[i] = {
        FindOrDie(lane_idx_diff_map, lp_infos[i].start_lane_id()),
        ComputeLanePathCost(psmm, lp_infos[i], route_sections_info,
                            route_navi_info)};
    LOG_INFO << " lane_path_costs " << i << " : " << lane_path_costs[i].first
             << " : " << lane_path_costs[i].second;
  }
  // If the last target lane path is still viable, choose it as one candidate.
  const int last_target_index =
      FindMostSimilarLanePathIndexToLastTargetLanePath(
          lp_infos, lane_path_costs, last_target_lane_path,
          plan_start_point.v());
  if (last_target_index != -1) {
    if (std::abs(lane_path_costs[last_target_index].first) > 1) {
      lane_path_costs[last_target_index].first =
          std::copysign(1, lane_path_costs[last_target_index].first);
    }
    lane_path_costs[last_target_index].second += kLastTargetLanePathReward;
    std::vector<std::string> output_debug;
    output_debug.emplace_back(
        absl::StrCat("kLastTargetLanePathReward:", last_target_index));
    Log2DDS::LogDataV2("filter-cost", output_debug);

    const auto& front_sec_idx_map = route_sections_info.front().id_idx_map;
    const int last_start_id_idx = FindOrDie(
        front_sec_idx_map, lp_infos[last_target_index].start_lane_id());

    constexpr double kLcPreviewTime = 3.0;     // s.
    constexpr double kMinLcPreviewLen = 20.0;  // m.
    const auto& last_lane_path = lp_infos[last_target_index].lane_path();
    const double preview_length = std::min(
        last_lane_path.length(),
        std::max(kMinLcPreviewLen, kLcPreviewTime * plan_start_point.v()));
    const auto preview_lane_pt =
        last_lane_path.ArclengthToLanePoint(preview_length);
    const auto [left_lane_pt, right_lane_pt] =
        FindNeighborLanePoints(route_sections_info, preview_lane_pt);
    for (int i = 0; i < n_lps; ++i) {
      if (i == last_target_index) continue;

      if (std::abs(FindOrDie(front_sec_idx_map, lp_infos[i].start_lane_id()) -
                   last_start_id_idx) > 1 ||
          (!lp_infos[i].lane_path().ContainsLanePoint(preview_lane_pt) &&
           ((i < last_target_index &&
             !lp_infos[i].lane_path().ContainsLanePoint(left_lane_pt)) ||
            (i > last_target_index &&
             !lp_infos[i].lane_path().ContainsLanePoint(right_lane_pt))))) {
        // For lane paths that split later, do not consider them now.
        lane_path_costs[i].first = INT_MAX;
      }
    }
  }  // NOLINTEND(readability-function-cognitive-complexity)

  absl::flat_hash_set<mapping::ElementId> preferred_lanes(
      preferred_lane_path.lane_ids().begin(),
      preferred_lane_path.lane_ids().end());
  for (int i = 0; i < n_lps; ++i) {
    if (preferred_lanes.contains(lp_infos[i].start_lane_id())) {
      // Guarantee the closest preferred lane is selected.
      lane_path_costs[i].first = std::copysign(1, lane_path_costs[i].first);
      lane_path_costs[i].second += kPreferredLanePathReward;
      std::vector<std::string> output_debug;
      output_debug.emplace_back(absl::StrCat("kPreferredLanePathReward:", i));
      Log2DDS::LogDataV2("filter-cost", output_debug);
    }
  }

  // discourage lc when closing or overlap with poi
  // const Vec2d ego_pos =
  // Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const auto crosswalks =
      psmm.GetCrosswalksInRadius(ego_pos, 5.0 * plan_start_point.v());
  std::vector<ad_byd::planning::CrosswalkConstPtr> front_cw;
  std::vector<std::string> output_debug;
  for (const auto& cw : crosswalks) {
    if (!cw) continue;
    output_debug.emplace_back(absl::StrCat("all cw:", cw->id()));

    // in poi
    if (cw->polygon().DistanceTo(ego_pos) < 1.0) {
      front_cw.emplace_back(cw);
      output_debug.emplace_back(absl::StrCat("in cw:", cw->id()));
      continue;
    }

    // before poi
    const auto& polygon_center = cw->polygon().CircleCenter();
    const auto ego_to_cw_dir = polygon_center - ego_pos;
    const auto heading_ego_cw = ego_to_cw_dir.Angle();
    if (std::fabs(plan_start_point.path_point().theta() - heading_ego_cw) >
        M_PI * 0.5) {
      continue;
    }
    output_debug.emplace_back(absl::StrCat("before cw:", cw->id()));
    front_cw.emplace_back(cw);
  }
  Log2DDS::LogDataV2("filter-cost", output_debug);

  std::vector<int> idx_vec(n_lps);
  std::iota(idx_vec.begin(), idx_vec.end(), 0);
  std::stable_sort(
      idx_vec.begin(), idx_vec.end(), [&lane_path_costs](int i1, int i2) {
        return std::abs(lane_path_costs[i1].first) <
                   std::abs(lane_path_costs[i2].first) ||
               (std::abs(lane_path_costs[i1].first) ==
                    std::abs(lane_path_costs[i2].first) &&
                lane_path_costs[i1].second < lane_path_costs[i2].second);
      });

  // Keep only the best one for each start lane id.
  absl::flat_hash_set<mapping::ElementId> start_lane_set;
  for (int i = 0; i < n_lps; ++i) {
    const int lane_idx = idx_vec[i];
    if (!start_lane_set.contains(lp_infos[lane_idx].start_lane_id())) {
      start_lane_set.insert(lp_infos[lane_idx].start_lane_id());
      mutable_lp_infos->emplace_back(lp_infos[lane_idx]);
    }
  }

  const int res_size = std::min(n_lps, FLAGS_planner_est_parallel_branch_num);
  std::vector<LanePathInfo> results;
  results.reserve(res_size);
  bool can_ignore_lp_by_cw = false;
  for (int i = 0; i < n_lps; ++i) {
    const int lane_idx = idx_vec[i];
    if (std::abs(lane_path_costs[lane_idx].first) == 0) {
      can_ignore_lp_by_cw = true;
    }
  }
  for (int i = 0, prev_lane_diff = INT_MAX; i < n_lps; ++i) {
    const int lane_idx = idx_vec[i];
    // Discourage one single lane change over multiple lanes.
    if (std::abs(lane_path_costs[lane_idx].first) > 1) break;
    // Same index diff, choose only the better one.
    if (lane_path_costs[lane_idx].first == prev_lane_diff) continue;
    // discourage lc when closing or overlap with poi
    if (can_ignore_lp_by_cw && std::abs(lane_path_costs[lane_idx].first) == 1 &&
        !front_cw.empty())
      continue;
    // Delete non motor lane
    if (lane_path_costs[lane_idx].second == DBL_MAX) continue;

    results.push_back(lp_infos[lane_idx]);
    prev_lane_diff = lane_path_costs[lane_idx].first;
    if (results.size() == res_size) break;
  }

  //
  if (pnp_fail_cur) {
    std::vector<LanePathInfo> new_results;
    new_results.emplace_back(pnp_top1);
    for (const auto& lp_info : results) {
      const auto ego_sl = lp_info.ProjectionSL(ego_pos);
      if (0.5 * kDefaultLaneWidth > std::fabs(ego_sl.l)) {
        new_results.emplace_back(lp_info);
        break;
      }
    }
    return new_results;
  }
  return results;
}

}  // namespace st::planning
