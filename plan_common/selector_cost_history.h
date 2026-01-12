#pragma once
#include <cereal/access.hpp>

#include "absl/container/flat_hash_map.h"
#include "absl/time/time.h"

#include "boost/circular_buffer.hpp"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"

// #include "object_manager/st_inference/scheduler_output.h"

namespace st::planning {

using ObjectIdType = std::string;

struct FollowCostInfo {
  ObjectIdType obj_id = "";
  ObjectType obj_type = ObjectType::OT_UNKNOWN_MOVABLE;
  double speed_follow_cost = 0.0;
  double final_follow_cost = 0.0;
  double integration = 0.0;
  int count = 0;
  absl::Time ts;
  std::string DebugString() const;
};

using object_cost_map_type = std::unordered_map<ObjectIdType, FollowCostInfo>;
FollowCostInfo ComputeFollowCost(ObjectType obj_type,
                                 double origin_speed_follow_cost,
                                 double last_integration_cost,
                                 bool is_valid_ego_v, bool is_valid_speed_diff,
                                 bool is_slow_driving);
class SelectorCostHistory {
 public:
  SelectorCostHistory() = default;

  // @return the count of removed keys.
  int UpdateAndRemoveDecay(absl::Time plan_time);
  // Getter && Setters
  double GetDecayFactor() const { return decay_factor_; }
  auto *mutable_path_object_costs() { return &path_object_costs_; }
  const auto &path_object_costs() const { return path_object_costs_; }
  object_cost_map_type &operator[](int idx) { return path_object_costs_[idx]; }

 private:
  absl::Time cur_time_;  // secs;
  std::map<int, object_cost_map_type> path_object_costs_;
  double decay_factor_ = 0.5;
  double expire_cost_threshold_ = 0.01;

  template <typename Archive>
  friend void serialize(Archive &ar,
                        SelectorCostHistory &selector_cost_history);
};

}  // namespace st::planning