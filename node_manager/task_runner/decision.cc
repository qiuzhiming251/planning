//
// Created by xxx on 7/28/21.
//

#include "node_manager/task_runner/decision.h"

namespace st::planning {

void Decision::Reset() {
  lane_follow_keep_time_ = 0.0;
  ResetByScenario();
}
void Decision::ResetByScenario() {
  for (auto &supplement : supplements_map_) {
    if (supplement.second) {
      supplement.second->Reset();
    }
  }
}
void Decision::set_lane_follow_keep_time_(const double &lane_follow_keep_time) {
  lane_follow_keep_time_ = lane_follow_keep_time;
}

}  // namespace st::planning
