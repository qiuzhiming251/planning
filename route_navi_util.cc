

#include "router/navi/route_navi_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <vector>

#include <limits.h>

#include "absl/hash/hash.h"
#include "absl/types/span.h"
#include "plan_common/container/strong_int.h"
//#include "global/logging.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_util.h"
#include "plan_common/util/map_util.h"

namespace st {
namespace planning {

absl::flat_hash_map<mapping::ElementId, double> CalculateMaxDrivingDistance(
    const ad_byd::planning::Map& smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    bool from_lane_beginning) {
  const auto& sections = sections_info.section_segments();
  const auto& back_sec = sections.back();
  absl::flat_hash_map<mapping::ElementId, double> driving_dist_map;

  constexpr double kBonusForDestination = 10.0;
  for (int i = 0; i < back_sec.lane_ids.size(); ++i) {
    driving_dist_map[back_sec.lane_ids[i]] = back_sec.length();
    if (back_sec.lane_ids[i] == sections_info.destination().lane_id()) {
      driving_dist_map[back_sec.lane_ids[i]] += kBonusForDestination;
    }
  }

  for (auto iter = sections.rbegin() + 1; iter != sections.rend(); ++iter) {
    const auto& this_section = *iter;
    const auto& next_section = *(iter - 1);
    for (const auto& this_lane_id : this_section.lane_ids) {
      SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, smm, this_lane_id);
      if (avoid_lanes.contains(this_lane_id) ||
          lane_info.IsPassengerVehicleAvoidLaneType()) {
        driving_dist_map[this_lane_id] = 0.0;
        continue;
      }

      auto& this_driving_dist = driving_dist_map[this_lane_id];
      const double section_length = from_lane_beginning
                                        ? this_section.average_length
                                        : this_section.length();
      this_driving_dist = section_length;
      for (const auto& next_lane_id : next_section.lane_ids) {
        if (mapping::IsOutgoingLane(smm, lane_info, next_lane_id)) {
          this_driving_dist =
              std::max(this_driving_dist,
                       driving_dist_map[next_lane_id] + section_length);
        }
      }
    }
  }
  return driving_dist_map;
}

absl::flat_hash_map<mapping::ElementId, int> FindLcNumToTargets(
    const ad_byd::planning::Map& smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_length, int start_section_idx) {
  int preview_idx = start_section_idx;
  for (; preview_idx + 1 < sections_info.size(); ++preview_idx) {
    if (sections_info.section_segment(preview_idx).lane_ids.empty()) {
      preview_idx = std::max(preview_idx - 1, 0);
      break;
    }
    preview_length -= sections_info.section_segment(preview_idx).length();
    if (preview_length <= 0.0) break;
  }
  const auto& target_ids = sections_info.section_segment(preview_idx).lane_ids;
  std::optional<int> destination_id_index = std::nullopt;
  if (preview_idx == sections_info.size() - 1) {
    destination_id_index =
        FindWithDefault(sections_info.section_segment(preview_idx).id_idx_map,
                        sections_info.destination().lane_id(), INT_MAX);
  }

  absl::flat_hash_map<mapping::ElementId, int> lc_num_map;
  for (const auto target_id : target_ids) {
    // Consider destination target when preview reach destination.
    if (destination_id_index.has_value() && *destination_id_index != INT_MAX) {
      int target_id_index =
          FindWithDefault(sections_info.section_segment(preview_idx).id_idx_map,
                          target_id, INT_MAX);
      lc_num_map[target_id] = std::abs(target_id_index - *destination_id_index);
    } else {
      lc_num_map[target_id] = 0;
    }
  }
  if (preview_idx == start_section_idx) return lc_num_map;

  int cur_sec_idx = preview_idx, prev_sec_idx = preview_idx - 1;
  while (cur_sec_idx != start_section_idx) {
    const auto& prev_lane_ids =
        sections_info.section_segment(prev_sec_idx).lane_ids;

    for (const auto prev_id : prev_lane_ids) {
      SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, smm, prev_id);
      int min_lc_num = INT_MAX;
      if (avoid_lanes.contains(prev_id)) {
        lc_num_map[prev_id] = min_lc_num;
        continue;
      }
      for (const auto lane_idx : lane_info.outgoing_lane_indices) {
        const auto next_lane_id = smm.lane_info()[lane_idx].id;
        if (avoid_lanes.contains(next_lane_id)) continue;
        if (lc_num_map.contains(next_lane_id)) {
          min_lc_num =
              std::min(min_lc_num, FindOrDie(lc_num_map, next_lane_id));
        }
      }
      lc_num_map[prev_id] = min_lc_num;
    }

    for (int i = 0; i < prev_lane_ids.size(); ++i) {
      auto& min_lc_num = lc_num_map[prev_lane_ids[i]];
      for (int j = 0; j < prev_lane_ids.size(); ++j) {
        const int other_lc_num = lc_num_map.at(prev_lane_ids[j]);
        if (other_lc_num == INT_MAX) continue;

        min_lc_num = std::min(min_lc_num, other_lc_num + std::abs(i - j));
      }
    }
    cur_sec_idx = prev_sec_idx--;
  }

  return lc_num_map;
}

namespace smm2 {

absl::flat_hash_map<mapping::ElementId, int> FindLcNumToTargets(
    const ad_byd::planning::Map& v2smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_length, int start_section_idx) {
  int preview_idx = start_section_idx;
  for (; preview_idx + 1 < sections_info.size(); ++preview_idx) {
    if (sections_info.section_segment(preview_idx).lane_ids.empty()) {
      preview_idx = std::max(preview_idx - 1, 0);
      break;
    }
    preview_length -= sections_info.section_segment(preview_idx).length();
    if (preview_length <= 0.0) break;
  }
  const auto& target_ids = sections_info.section_segment(preview_idx).lane_ids;
  std::optional<int> destination_id_index = std::nullopt;
  if (preview_idx == sections_info.size() - 1) {
    destination_id_index =
        FindWithDefault(sections_info.section_segment(preview_idx).id_idx_map,
                        sections_info.destination().lane_id(), INT_MAX);
  }

  absl::flat_hash_map<mapping::ElementId, int> lc_num_map;
  for (const auto target_id : target_ids) {
    // Consider destination target when preview reach destination.
    if (destination_id_index.has_value() && *destination_id_index != INT_MAX) {
      int target_id_index =
          FindWithDefault(sections_info.section_segment(preview_idx).id_idx_map,
                          target_id, INT_MAX);
      lc_num_map[target_id] = std::abs(target_id_index - *destination_id_index);
    } else {
      lc_num_map[target_id] = 0;
    }
  }
  if (preview_idx == start_section_idx) return lc_num_map;

  int cur_sec_idx = preview_idx, prev_sec_idx = preview_idx - 1;
  while (cur_sec_idx != start_section_idx) {
    const auto& prev_lane_ids =
        sections_info.section_segment(prev_sec_idx).lane_ids;
    for (const auto prev_id : prev_lane_ids) {
      SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, v2smm,
                                   mapping::ElementId(prev_id));
      int min_lc_num = INT_MAX;
      if (avoid_lanes.contains(prev_id)) {
        lc_num_map[prev_id] = min_lc_num;
        continue;
      }
      for (const auto out_lane_id : lane_info.proto().outgoing_lanes()) {
        auto next_lane_id = mapping::ElementId(out_lane_id);
        if (avoid_lanes.contains(next_lane_id)) continue;
        if (lc_num_map.contains(next_lane_id)) {
          min_lc_num =
              std::min(min_lc_num, FindOrDie(lc_num_map, next_lane_id));
        }
      }
      lc_num_map[prev_id] = min_lc_num;
    }

    for (int i = 0; i < static_cast<int>(prev_lane_ids.size()); ++i) {
      auto& min_lc_num = lc_num_map[prev_lane_ids[i]];
      for (int j = 0; j < static_cast<int>(prev_lane_ids.size()); ++j) {
        // LOG_WARN << "prev_lane_ids.size:"<<prev_lane_ids.size();
        const int other_lc_num = lc_num_map.at(prev_lane_ids[j]);
        if (other_lc_num == INT_MAX) continue;

        min_lc_num = std::min(min_lc_num, other_lc_num + std::abs(i - j));
      }
    }
    cur_sec_idx = prev_sec_idx--;
  }

  return lc_num_map;
}
absl::flat_hash_map<mapping::ElementId, double> CalculateMaxDrivingDistance(
    const ad_byd::planning::Map& v2smm, const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    bool from_lane_beginning) {
  const auto& sections = sections_info.section_segments();
  const auto& back_sec = sections.back();
  absl::flat_hash_map<mapping::ElementId, double> driving_dist_map;

  constexpr double kBonusForDestination = 10.0;
  for (size_t i = 0; i < back_sec.lane_ids.size(); ++i) {
    driving_dist_map[back_sec.lane_ids[i]] = back_sec.length();
    if (back_sec.lane_ids[i] == sections_info.destination().lane_id()) {
      driving_dist_map[back_sec.lane_ids[i]] += kBonusForDestination;
    }
  }

  for (auto iter = sections.rbegin() + 1; iter != sections.rend(); ++iter) {
    const auto& this_section = *iter;
    const auto& next_section = *(iter - 1);
    for (const auto& this_lane_id : this_section.lane_ids) {
      SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, v2smm, this_lane_id);
      if (avoid_lanes.contains(this_lane_id) ||
          mapping::IsPassengerVehicleAvoidLaneType(lane_info.proto().type())) {
        driving_dist_map[this_lane_id] = 0.0;
        continue;
      }

      auto& this_driving_dist = driving_dist_map[this_lane_id];
      const double section_length = from_lane_beginning
                                        ? this_section.average_length
                                        : this_section.length();
      this_driving_dist = section_length;
      for (const auto& next_lane_id : next_section.lane_ids) {
        if (mapping::IsOutgoingLane(v2smm, lane_info.Proto(), next_lane_id)) {
          this_driving_dist =
              std::max(this_driving_dist,
                       driving_dist_map[next_lane_id] + section_length);
        }
      }
    }
  }
  return driving_dist_map;
}
}  // namespace smm2
}  // namespace planning
}  // namespace st
