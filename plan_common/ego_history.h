
#ifndef AD_BYD_PLANNING_EGO_HISTORY_H
#define AD_BYD_PLANNING_EGO_HISTORY_H

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "plan_common/util/time_util.h"
#include <cereal/cereal.hpp>

namespace st {
namespace planning {

#define MAX_EGO_FRAME_NUM (20)

struct V2SpeedLimitInfo {
  bool is_generate_small_speed_limit = false;
  bool is_close_curb = false;
};

struct EgoFrame {
  V2SpeedLimitInfo v2_speed_limit_info;
};

class EgoHistory {
 public:
  EgoHistory();

  ~EgoHistory();

  std::deque<EgoFrame>& GetFrames() { return frames_; }
  const EgoFrame* GetOldestFrame() const {
    if (Empty()) {
      return nullptr;
    }
    return &(frames_.front());
  }
  const EgoFrame* GetLatestFrame() const {
    if (Empty()) {
      return nullptr;
    }
    return &(frames_.back());
  }

  const EgoFrame* GetSecondLatestFrame() const {
    if (Size() < 2) {
      return nullptr;
    }
    return &(frames_[Size() - 2]);
  }

  const int Size() const { return frames_.size(); }
  bool Empty() const { return frames_.empty(); }

  void CleanExceeded();

  void AddNewFrame(EgoFrame ego_new_frame);

  void UpdateEgoHistory(EgoFrame ego_curr_frame);

  std::deque<EgoFrame> frames_;  // here is selected task history result

  template <typename Archive>
  friend void serialize(Archive& ar, EgoHistory& ego_history);
};

}  // namespace planning
}  // namespace st

#endif  // AD_BYD_PLANNING_EGO_HISTORY_H