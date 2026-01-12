#ifndef AD_BYD_PLANNING_NODES_BEHAVIOR_CONTAINER_H
#define AD_BYD_PLANNING_NODES_BEHAVIOR_CONTAINER_H

#include <mutex>

#include "modules/msg/st_msgs/sm_behavior.pb.h"

namespace ad_byd {
namespace planning {

struct Behavior {
  byd::msg::planning::FunctionId function_id =
      byd::msg::planning::FunctionId::FUNCTION_NONE;
};

class BehaviorContainer {
 public:
  BehaviorContainer() = default;

  void OnMsg(const byd::msg::planning::SMBehavior& msg,
             const bool on_high_way) {
    std::scoped_lock<std::mutex> lock(mutex_);
    behavior_.function_id = msg.function_id();
    if (on_high_way && behavior_.function_id ==
                           byd::msg::planning::FunctionId::FUNCTION_CITY_NOA) {
      behavior_.function_id = byd::msg::planning::FunctionId::FUNCTION_HW_NOA;
    }
    // behavior_.function_id = byd::msg::planning::FunctionId::FUNCTION_LKA;
  }

  Behavior Get() const {
    std::scoped_lock<std::mutex> lock(mutex_);
    return behavior_;
  }

 private:
  Behavior behavior_;
  mutable std::mutex mutex_;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_BEHAVIOR_CONTAINER_H
