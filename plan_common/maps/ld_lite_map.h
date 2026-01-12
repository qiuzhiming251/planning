
#pragma once

#include <map>
#include <memory>
#include <unordered_map>

#include "plan_common/maps/map_def.h"

namespace ad_byd {
namespace planning {

// struct LiteLaneKey {
//   uint64_t lane_group_id;
//   uint32_t lane_index;

//   LiteLaneKey(uint64_t l_group_id, uint32_t idx)
//       : lane_group_id(l_group_id), lane_index(idx) {}

//   bool operator<(const LiteLaneKey &key) const {
//     return (lane_group_id < key.lane_group_id) ||
//            (lane_group_id == key.lane_group_id && lane_index <
//            key.lane_index);
//   }

//   bool operator==(const LiteLaneKey &key) const {
//     return (lane_group_id == key.lane_group_id && lane_index ==
//     key.lane_index);
//   }

//   bool operator!=(const LiteLaneKey &key) const { return !(*this == key); }
// };

// struct LdliteLane{
//   LdLiteLaneInfo lite_lane_info;
//   double navi_distance = 0.0;
//   bool is_navigation = false;
// };
using LdliteLanePtr = std::shared_ptr<LdLiteLaneInfo>;
using LdliteLaneConstPtr = std::shared_ptr<const LdLiteLaneInfo>;

struct LdliteNaviStart {
  uint64_t lane_group_id = 0;
  uint32_t lane_index = 0;
  double s_offset;
};

struct LdlitePriorityLane {
  bool is_valid = false;
  int priority_lane_idx = 0;
  double navi_dis = 0.0;
  double min_navi_dis = 0.0;
};

struct LdliteLaneGroup {
  uint64_t id;
  uint32_t lane_nums;
  std::vector<uint64_t> lane_ids;
  uint64_t next_navi_group = 0;
  uint64_t pre_navi_group = 0;
  double dis;
  double length;
  bool is_in_junction = false;
  uint32_t navi_group_idx = 0;
};
using LdliteLaneGroupPtr = std::shared_ptr<LdliteLaneGroup>;
using LdliteLaneGroupConstPtr = std::shared_ptr<LdliteLaneGroup>;

class LdLiteMap {
 public:
  LdLiteMap(const LdLiteMapInfo &sd_map_info);
  ~LdLiteMap() = default;

  const bool is_lite_map_valid() const { return is_lite_map_valid_; }
  const LdliteNaviStart &lite_navi_start() const { return lite_navi_start_; }
  double dist_to_junction() const { return dist_to_junction_; }
  LdliteLaneConstPtr GetLiteLaneById(uint64_t id) const;
  void UpdateNaviStartPriority(const int target_lane_idx, const int lane_nums);
  const LdlitePriorityLane &lite_prio_lane() const { return lite_prio_lane_; }
  bool GetNaviDistanceByIndex(const int idx, double *navi_dis);
  bool GetNaviDistanceBySplitDir(const int idx, const bool split_left,
                                 int *lc_num, double *navi_dis);
  int64_t seq() const { return seq_; };

 private:
  void ConvertMapInfo(const LdLiteMapInfo &sd_map_info);
  void UpdateLdLiteLaneMap(const LdLiteMapInfo &sd_map_info);
  void CalLaneNaviDance();
  double CalDistToJunction();

 private:
  bool is_lite_map_valid_ = true;
  std::unordered_map<uint64_t, LdliteLanePtr> lite_lane_map_;
  std::unordered_map<uint64_t, LdliteLaneGroupPtr> mpp_lane_group_map_;
  std::vector<uint64_t> navi_lane_group_list_;
  LdliteNaviStart lite_navi_start_;
  double dist_to_junction_ = std::numeric_limits<double>::infinity();
  LdlitePriorityLane lite_prio_lane_;
  int64_t seq_ = 0;
};

using LdLiteMapPtr = std::shared_ptr<LdLiteMap>;
using LdLiteMapConstPtr = std::shared_ptr<const LdLiteMap>;

}  // namespace planning
}  // namespace ad_byd
