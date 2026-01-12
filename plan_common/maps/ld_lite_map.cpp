

#include "plan_common/maps/ld_lite_map.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"

namespace ad_byd {
namespace planning {

constexpr double kMinJunctionLength = 5.0;

LdLiteMap::LdLiteMap(const LdLiteMapInfo &sd_map_info) {
  ConvertMapInfo(sd_map_info);
  UpdateLdLiteLaneMap(sd_map_info);
  CalLaneNaviDance();
  dist_to_junction_ = CalDistToJunction();

  // LOG_ERROR << "lite_navi_start_: " << lite_navi_start_.lane_group_id << " "
  //           << lite_navi_start_.lane_index << " " <<
  //           lite_navi_start_.s_offset;
  // LOG_ERROR << "navi_lane_group_list size: " << navi_lane_group_list_.size();
  // LOG_ERROR << "mpp_lane_group_map_size: " << mpp_lane_group_map_.size();
  // LOG_ERROR << "lite_lane_map_size: " << lite_lane_map_.size();
  // auto lane_group_iter =
  //     mpp_lane_group_map_.find(lite_navi_start_.lane_group_id);
  // if (lane_group_iter == mpp_lane_group_map_.end()) {
  //   LOG_ERROR << " *************cant find lane group: "
  //             << lite_navi_start_.lane_group_id;
  // } else {
  //   for (auto lane_id : lane_group_iter->second->lane_ids) {
  //     auto lane_iter = lite_lane_map_.find(lane_id);
  //     if (lane_iter == lite_lane_map_.end() || !lane_iter->second) continue;
  //     LOG_ERROR << "lane_id: " << lane_id
  //               << ", index: " << lane_iter->second->lane_seq
  //               << ", length: " << lane_iter->second->length
  //               << ", navi_distance: " << lane_iter->second->navi_distance;
  //   }
  // }
}

LdliteLaneConstPtr LdLiteMap::GetLiteLaneById(uint64_t id) const {
  auto it = lite_lane_map_.find(id);
  return it != lite_lane_map_.end() ? it->second : nullptr;
}

void LdLiteMap::ConvertMapInfo(const LdLiteMapInfo &sd_map_info) {
  seq_ = sd_map_info.seq;
  int mpp_section_size = sd_map_info.mpp_info.size();
  navi_lane_group_list_.reserve(mpp_section_size);
  // LdliteNaviStart lite_navi_start;
  LdliteLaneGroup cur_lane_group;
  double accu_s = 0.0;
  bool find_start_section = false;
  // uint32_t lane_group_idx_on_navi = 1;
  std::unordered_set<uint64_t> mpp_link_ids;
  for (int mpp_section_idx = 0; mpp_section_idx < mpp_section_size;
       mpp_section_idx++) {
    const auto &mpp_section = sd_map_info.mpp_info[mpp_section_idx];
    if (mpp_link_ids.find(mpp_section.id) != mpp_link_ids.end()) break;
    if (!find_start_section &&
        sd_map_info.navi_start.section_id == mpp_section.id) {
      find_start_section = true;
      int lane_group_size =
          sd_map_info.mpp_info[mpp_section_idx].lane_group_idx.size();
      int l_group_idx = 0;
      for (; l_group_idx < lane_group_size; l_group_idx++) {
        const auto &lane_group =
            sd_map_info.mpp_info[mpp_section_idx].lane_group_idx[l_group_idx];
        if (sd_map_info.navi_start.s_offset <= lane_group.end_range ||
            l_group_idx == lane_group_size - 1) {
          lite_navi_start_.lane_group_id = lane_group.id;
          lite_navi_start_.s_offset =
              sd_map_info.navi_start.s_offset - lane_group.start_range;
          break;
        }
      }
      int navi_start_idx = mpp_section_idx;
      if (lite_navi_start_.lane_group_id == 0) {
        lite_navi_start_.s_offset = sd_map_info.navi_start.s_offset;
        while ((--navi_start_idx) >= 0) {
          const auto &l_group =
              sd_map_info.mpp_info[navi_start_idx].lane_group_idx;
          if (l_group.empty()) {
            lite_navi_start_.s_offset +=
                sd_map_info.mpp_info[navi_start_idx].length;
          } else {
            lite_navi_start_.s_offset +=
                (sd_map_info.mpp_info[navi_start_idx].length -
                 l_group.back().start_range);
            lite_navi_start_.lane_group_id = l_group.back().id;
            l_group_idx = l_group.size() - 1;
            break;
          }
        }
      }
      if (l_group_idx == 0 && lite_navi_start_.lane_group_id != 0) {
        while ((--navi_start_idx) >= 0) {
          const auto &l_group =
              sd_map_info.mpp_info[navi_start_idx].lane_group_idx;
          if (l_group.empty() ||
              l_group.back().id != lite_navi_start_.lane_group_id)
            break;
          lite_navi_start_.s_offset +=
              (sd_map_info.mpp_info[navi_start_idx].length -
               l_group.back().start_range);
          if (l_group.size() > 1) break;
        }
      }
      // lite_navi_start_ = lite_navi_start;
      navi_lane_group_list_.push_back(lite_navi_start_.lane_group_id);

      cur_lane_group.id = lite_navi_start_.lane_group_id;
      cur_lane_group.dis = -lite_navi_start_.s_offset;
      // cur_lane_group.navi_group_idx = lane_group_idx_on_navi++;
      mpp_lane_group_map_[cur_lane_group.id] =
          std::make_shared<LdliteLaneGroup>(std::move(cur_lane_group));
    }
    if (!find_start_section) continue;

    mpp_link_ids.insert(mpp_section.id);

    for (const auto &mpp_lane_group : mpp_section.lane_group_idx) {
      if (accu_s < 0.1 &&
          mpp_lane_group.end_range < sd_map_info.navi_start.s_offset)
        continue;
      if (!navi_lane_group_list_.empty() &&
          mpp_lane_group.id == navi_lane_group_list_.back())
        continue;
      navi_lane_group_list_.push_back(mpp_lane_group.id);
      cur_lane_group.id = mpp_lane_group.id;
      // cur_lane_group.navi_group_idx = lane_group_idx_on_navi++;
      if (accu_s < 0.1)
        cur_lane_group.dis =
            mpp_lane_group.start_range - sd_map_info.navi_start.s_offset;
      else
        cur_lane_group.dis = accu_s + mpp_lane_group.start_range;
      mpp_lane_group_map_[cur_lane_group.id] =
          std::make_shared<LdliteLaneGroup>(std::move(cur_lane_group));
    }
    if (accu_s < 0.1)
      accu_s = mpp_section.length - sd_map_info.navi_start.s_offset;
    else
      accu_s += mpp_section.length;
  }
}

void LdLiteMap::UpdateLdLiteLaneMap(const LdLiteMapInfo &sd_map_info) {
  for (int i = 0; i < navi_lane_group_list_.size(); i++) {
    auto lane_group_iter = mpp_lane_group_map_.find(navi_lane_group_list_[i]);
    if (lane_group_iter == mpp_lane_group_map_.end()) continue;
    auto &lane_group_ptr = lane_group_iter->second;
    lane_group_ptr->navi_group_idx = i + 1;
    if (i < navi_lane_group_list_.size() - 1)
      lane_group_ptr->next_navi_group = navi_lane_group_list_[i + 1];
    if (i > 0) lane_group_ptr->pre_navi_group = navi_lane_group_list_[i - 1];
  }
  for (const auto &lane_group : sd_map_info.lane_group_info) {
    auto lane_group_iter = mpp_lane_group_map_.find(lane_group.id);
    if (lane_group_iter == mpp_lane_group_map_.end()) continue;
    auto &lane_group_ptr = lane_group_iter->second;
    lane_group_ptr->lane_nums = lane_group.lane_num;
    lane_group_ptr->length = lane_group.length;
    if (!lane_group.next_lane_group.empty() &&
        !lane_group.previous_lane_group.empty() &&
        lane_group.next_lane_group.front() ==
            lane_group.previous_lane_group.front() &&
        mpp_lane_group_map_.find(lane_group.next_lane_group.front()) !=
            mpp_lane_group_map_.end()) {
      // is_lite_map_valid_ = false;
      Log2DDS::LogDataV2("ld_lite_debug",
                         absl::StrCat("wrong_lane_group: ",
                                      lane_group.next_lane_group.front()));
    }

    for (const auto &lite_lane_info : lane_group.lane_info) {
      lane_group_ptr->lane_ids.push_back(lite_lane_info.id);
      if (lite_lane_info.type == LANE_RIGHT_TURN_LANE) {
        lane_group_ptr->is_in_junction = true;
      }
      lite_lane_map_[lite_lane_info.id] =
          std::make_shared<LdLiteLaneInfo>(lite_lane_info);
      lite_lane_map_[lite_lane_info.id]->length = lane_group.length;
      lite_lane_map_[lite_lane_info.id]->navi_distance = lane_group.length;
      lite_lane_map_[lite_lane_info.id]->min_navi_distance = lane_group.length;
    }
  }
}

void LdLiteMap::CalLaneNaviDance() {
  int navi_group_size = navi_lane_group_list_.size();
  for (int i = navi_group_size - 2; i >= 0; i--) {
    auto lane_group_iter = mpp_lane_group_map_.find(navi_lane_group_list_[i]);
    if (lane_group_iter == mpp_lane_group_map_.end() ||
        !lane_group_iter->second)
      continue;
    std::vector<LdliteLanePtr> has_next_lanes, no_next_lanes;
    int lane_ids_nums = lane_group_iter->second->lane_ids.size();
    for (const auto lane_id : lane_group_iter->second->lane_ids) {
      auto lane_iter = lite_lane_map_.find(lane_id);
      if (lane_iter == lite_lane_map_.end() || !lane_iter->second) continue;
      auto &lane_ptr = lane_iter->second;
      // if (i == navi_group_size - 1) {
      //   lane_ptr->navi_distance = lane_ptr->length;
      //   continue;
      // }
      bool is_next_lane_exist = false, first_next_lane_found = false;
      for (const auto next_lane_id : lane_ptr->next_lanes) {
        auto next_lane_iter = lite_lane_map_.find(next_lane_id);
        if (next_lane_iter == lite_lane_map_.end() || !next_lane_iter->second)
          continue;
        // if (next_lane_iter->second->lane_group_id !=
        //     lane_group_iter->second->next_navi_group)
        //   continue;
        auto next_group_iter =
            mpp_lane_group_map_.find(next_lane_iter->second->lane_group_id);
        if (next_group_iter == mpp_lane_group_map_.end() ||
            !next_group_iter->second ||
            next_group_iter->second->navi_group_idx -
                    lane_group_iter->second->navi_group_idx >
                2)
          continue;

        lane_ptr->navi_distance =
            std::max(lane_ptr->navi_distance,
                     next_lane_iter->second->navi_distance + lane_ptr->length);
        if (!first_next_lane_found) {
          lane_ptr->min_navi_distance =
              next_lane_iter->second->min_navi_distance + lane_ptr->length;
          first_next_lane_found = true;
        } else {
          lane_ptr->min_navi_distance = std::min(
              lane_ptr->min_navi_distance,
              next_lane_iter->second->min_navi_distance + lane_ptr->length);
        }
        is_next_lane_exist = true;
      }
      if (is_next_lane_exist) {
        has_next_lanes.push_back(lane_ptr);
      } else {
        no_next_lanes.push_back(lane_ptr);
      }
    }
    if (!has_next_lanes.empty() && !no_next_lanes.empty()) {
      for (auto &no_next_lane : no_next_lanes) {
        int min_idx_gap =
            std::abs(static_cast<int>(no_next_lane->lane_seq) -
                     static_cast<int>(has_next_lanes.front()->lane_seq));
        for (auto &has_next_lane : has_next_lanes) {
          min_idx_gap = std::min(
              min_idx_gap, std::abs(static_cast<int>(no_next_lane->lane_seq) -
                                    static_cast<int>(has_next_lane->lane_seq)));
        }
        no_next_lane->navi_distance =
            no_next_lane->length + 5.0 * (lane_ids_nums - min_idx_gap);
      }
    }
  }
}

double LdLiteMap::CalDistToJunction() {
  int navi_lane_group_size = navi_lane_group_list_.size();
  for (int i = 0; i < navi_lane_group_size; i++) {
    auto lane_group_iter = mpp_lane_group_map_.find(navi_lane_group_list_[i]);
    if (lane_group_iter == mpp_lane_group_map_.end()) continue;
    auto &lane_group_front = lane_group_iter->second;
    if (lane_group_front->is_in_junction) {
      return std::max(0.0, lane_group_front->dis);
    }
    if (i + 1 < navi_lane_group_size) {
      auto lane_group_back_iter =
          mpp_lane_group_map_.find(navi_lane_group_list_[i + 1]);
      if (lane_group_back_iter != mpp_lane_group_map_.end()) {
        auto &lane_group_back = lane_group_back_iter->second;
        if (lane_group_back->dis - lane_group_front->dis >
            lane_group_front->length + kMinJunctionLength) {
          return std::max(0.0,
                          lane_group_front->dis + lane_group_front->length);
        }
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

void LdLiteMap::UpdateNaviStartPriority(const int target_lane_idx,
                                        const int lane_nums) {
  Log2DDS::LogDataV2("ld_lite_debug",
                     absl::StrCat("dist_to_junction: ", dist_to_junction_));
  auto lane_group_start =
      mpp_lane_group_map_.find(lite_navi_start_.lane_group_id);
  if (lane_group_start == mpp_lane_group_map_.end()) return;
  // if (lane_group_start->second->lane_nums != lane_nums) return;
  std::vector<LdliteLanePtr> navi_start_lanes;
  for (const auto lane_id : lane_group_start->second->lane_ids) {
    auto lane_iter = lite_lane_map_.find(lane_id);
    if (lane_iter == lite_lane_map_.end() || !lane_iter->second) continue;
    navi_start_lanes.push_back(lane_iter->second);
    Log2DDS::LogDataV2(
        "ld_lite_debug",
        absl::StrCat(
            "navi_lane: ", lane_id, ", max_navi_dis: ",
            lane_iter->second->navi_distance - lite_navi_start_.s_offset,
            ", min_navi_dis: ",
            lane_iter->second->min_navi_distance - lite_navi_start_.s_offset));
  }
  std::sort(navi_start_lanes.begin(), navi_start_lanes.end(),
            [&](LdliteLanePtr &l1, LdliteLanePtr &l2) {
              return (l1->navi_distance > l2->navi_distance) ||
                     (std::abs(l1->navi_distance - l2->navi_distance) < 1.0 &&
                      l1->min_navi_distance > l2->min_navi_distance);
            });
  if (lane_group_start->second->lane_nums != lane_nums) return;
  if (!navi_start_lanes.empty()) {
    lite_prio_lane_.is_valid = true;
    lite_prio_lane_.priority_lane_idx = navi_start_lanes.front()->lane_seq;
    lite_prio_lane_.navi_dis = navi_start_lanes.front()->navi_distance;
    lite_prio_lane_.min_navi_dis = navi_start_lanes.front()->min_navi_distance;
    for (int i = 1; i < navi_start_lanes.size(); i++) {
      if (navi_start_lanes[i]->navi_distance < lite_prio_lane_.navi_dis - 1.0)
        break;
      if (navi_start_lanes[i]->min_navi_distance <
              lite_prio_lane_.min_navi_dis - 1.0 &&
          navi_start_lanes[i]->min_navi_distance - lite_navi_start_.s_offset <
              dist_to_junction_ + 50.0)
        break;
      if (std::abs(target_lane_idx -
                   static_cast<int>(navi_start_lanes[i]->lane_seq)) <
          std::abs(target_lane_idx -
                   static_cast<int>(lite_prio_lane_.priority_lane_idx)))
        lite_prio_lane_.priority_lane_idx = navi_start_lanes[i]->lane_seq;
      lite_prio_lane_.min_navi_dis = navi_start_lanes[i]->min_navi_distance;
    }
    Log2DDS::LogDataV2(
        "ld_lite_debug",
        absl::StrCat("lite_prio_lane_idx: ", lite_prio_lane_.priority_lane_idx,
                     ", max_navi_dis: ",
                     lite_prio_lane_.navi_dis - lite_navi_start_.s_offset,
                     ", min_navi_dis: ",
                     lite_prio_lane_.min_navi_dis - lite_navi_start_.s_offset));
  }
}

bool LdLiteMap::GetNaviDistanceByIndex(const int idx, double *navi_dis) {
  auto lane_group_start =
      mpp_lane_group_map_.find(lite_navi_start_.lane_group_id);
  if (lane_group_start == mpp_lane_group_map_.end()) return false;
  if (idx <= 0 || idx > lane_group_start->second->lane_ids.size()) return false;
  auto lane_iter =
      lite_lane_map_.find(lane_group_start->second->lane_ids[idx - 1]);
  if (lane_iter == lite_lane_map_.end() || !lane_iter->second) return false;
  double cur_lane_navi_dis =
      lane_iter->second->navi_distance - lite_navi_start_.s_offset;
  if (cur_lane_navi_dis < 0.0) return false;
  if (navi_dis) *navi_dis = cur_lane_navi_dis;
  return true;
}

bool LdLiteMap::GetNaviDistanceBySplitDir(const int idx, const bool split_left,
                                          int *lc_num, double *navi_dis) {
  auto lane_group_start =
      mpp_lane_group_map_.find(lite_navi_start_.lane_group_id);
  if (lane_group_start == mpp_lane_group_map_.end()) return false;
  if (idx <= 0 || idx > lane_group_start->second->lane_ids.size()) return false;
  auto lane_iter =
      lite_lane_map_.find(lane_group_start->second->lane_ids[idx - 1]);
  if (lane_iter == lite_lane_map_.end() || !lane_iter->second) return false;
  auto lane_ptr = lane_iter->second;
  double start_navi_dis = lane_ptr->navi_distance - lite_navi_start_.s_offset;

  double accum_search_dis = 0.0;
  while (lane_ptr) {
    if (lane_ptr->next_lanes.size() < 1) return false;
    if (lane_ptr->next_lanes.size() == 1) {
      if (accum_search_dis > 150.0) return false;
      auto next_lane_iter = lite_lane_map_.find(lane_ptr->next_lanes.front());
      if (next_lane_iter == lite_lane_map_.end() || !next_lane_iter->second)
        return false;
      lane_ptr = next_lane_iter->second;
      accum_search_dis += lane_ptr->length;
      continue;
    } else {
      std::vector<LdliteLaneConstPtr> split_section_lanes;
      double max_navi_dis = 0.0;
      split_section_lanes.reserve(lane_ptr->next_lanes.size());
      for (const auto next_lane_id : lane_ptr->next_lanes) {
        auto next_lane_iter = lite_lane_map_.find(next_lane_id);
        if (next_lane_iter == lite_lane_map_.end() || !next_lane_iter->second)
          continue;
        split_section_lanes.emplace_back(next_lane_iter->second);
        max_navi_dis =
            std::max(max_navi_dis, next_lane_iter->second->navi_distance);
      }
      if (split_section_lanes.size() < 2) return false;
      std::sort(split_section_lanes.begin(), split_section_lanes.end(),
                [&](LdliteLaneConstPtr &a, LdliteLaneConstPtr &b) {
                  return a->lane_seq < b->lane_seq;
                });

      LdliteLaneConstPtr left_split_lane = split_section_lanes.front(),
                         right_split_lane = split_section_lanes.back();
      double max_navi_dis_delta =
          left_split_lane->navi_distance - right_split_lane->navi_distance;
      double min_navi_dis_delta = left_split_lane->min_navi_distance -
                                  right_split_lane->min_navi_distance;
      if (split_left) {
        bool is_left_special_lane =
            left_split_lane->type == LaneType::LANE_BUS_NORMAL ||
            left_split_lane->type == LaneType::LANE_HARBOR_STOP ||
            left_split_lane->type == LaneType::LANE_BRT ||
            left_split_lane->type == LaneType::LANE_EMERGENCY;
        if (navi_dis) {
          if (!is_left_special_lane)
            *navi_dis =
                start_navi_dis - max_navi_dis + left_split_lane->navi_distance;
          else
            *navi_dis = start_navi_dis - max_navi_dis;
        }
        if (lc_num) {
          if (std::abs(max_navi_dis_delta) < 1.0 && !is_left_special_lane) {
            if (min_navi_dis_delta > -1.0)
              *lc_num = std::abs(lite_prio_lane_.priority_lane_idx - idx);
            else
              *lc_num = std::abs(lite_prio_lane_.priority_lane_idx - idx) + 1;
          } else if (max_navi_dis_delta > -1.0 && !is_left_special_lane) {
            *lc_num = std::abs(lite_prio_lane_.priority_lane_idx - idx);
          } else {
            *lc_num = std::abs(lite_prio_lane_.priority_lane_idx - idx) + 1;
          }
        }
      } else {
        bool is_right_special_lane =
            right_split_lane->type == LaneType::LANE_BUS_NORMAL ||
            right_split_lane->type == LaneType::LANE_HARBOR_STOP ||
            right_split_lane->type == LaneType::LANE_BRT ||
            right_split_lane->type == LaneType::LANE_EMERGENCY;
        if (navi_dis) {
          if (!is_right_special_lane)
            *navi_dis =
                start_navi_dis - max_navi_dis + right_split_lane->navi_distance;
          else
            *navi_dis = start_navi_dis - max_navi_dis;
        }
        if (lc_num) {
          if (std::abs(max_navi_dis_delta) < 1.0 && !is_right_special_lane) {
            if (min_navi_dis_delta < 1.0)
              *lc_num = std::abs(lite_prio_lane_.priority_lane_idx - idx);
            else
              *lc_num = std::abs(lite_prio_lane_.priority_lane_idx - idx) + 1;
          } else if (max_navi_dis_delta < 1.0 && !is_right_special_lane) {
            *lc_num = std::abs(lite_prio_lane_.priority_lane_idx - idx);
          } else {
            *lc_num = std::abs(lite_prio_lane_.priority_lane_idx - idx) + 1;
          }
        }
      }
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace ad_byd
