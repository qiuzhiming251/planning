#include "plan_common/selector_cost_history.h"

#include <tuple>
#include <unordered_map>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"
// #include "decider/selector/common_feature.h"

namespace st::planning {
namespace {
inline constexpr double GetFollowKiByObjectType(ObjectType type) {
  // OT_UNKNOWN_STATIC = 0,
  // OT_VEHICLE = 1,
  // OT_MOTORCYCLIST = 2,
  // OT_PEDESTRIAN = 3,
  // OT_CYCLIST = 4,
  // OT_FOD = 5,
  // OT_UNKNOWN_MOVABLE = 6,
  // OT_VEGETATION = 7,
  // OT_BARRIER = 8,
  // OT_CONE = 10,
  // OT_WARNING_TRIANGLE = 13,
  // OT_TRICYCLIST = 14,
  // OT_LARGE_VEHICLE = 15
  switch (type) {
    case OT_UNKNOWN_STATIC:
    case OT_UNKNOWN_MOVABLE:
      return 0.02;
    case OT_VEHICLE:
      return 0.06;
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
      return 0.2;
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.08;
    case OT_TRICYCLIST:
      return 0.2;
    case OT_LARGE_VEHICLE:
      return 0.1;
    default:
      return 0.0;
  }
}

}  // namespace

std::string FollowCostInfo::DebugString() const {
  std::stringstream ss;
  ss << "obj_id: " << obj_id << ", obj_type: " << ObjectType_Name(obj_type)
     << ", speed_follow_cost: " << speed_follow_cost
     << ", final_follow_cost: " << final_follow_cost
     << ", integration: " << integration << ", count: " << count
     << ", ts: " << ts;
  return ss.str();
}

FollowCostInfo ComputeFollowCost(ObjectType obj_type,
                                 double origin_speed_follow_cost,
                                 double last_integration_cost,
                                 bool is_valid_ego_v, bool is_valid_speed_diff,
                                 bool is_slow_driving) {
  // leader_object.obj_type
  FollowCostInfo follow_cost_info;
  // 10HZ as base.
  follow_cost_info.obj_type = obj_type;
  follow_cost_info.speed_follow_cost = origin_speed_follow_cost;
  auto follow_ki = GetFollowKiByObjectType(obj_type);
  follow_cost_info.integration =
      last_integration_cost + origin_speed_follow_cost * follow_ki;
  if (origin_speed_follow_cost == 0.0 || !is_slow_driving) {
    follow_cost_info.integration *= 0.5;
  }
  constexpr double kEpsilon = 1E-5;
  if (follow_cost_info.integration < kEpsilon) {
    follow_cost_info.integration = 0.0;
  }
  constexpr double kMinCostwithSlowDriving = 0.01;
  if (is_slow_driving)
    follow_cost_info.integration =
        last_integration_cost +
        std::max(kMinCostwithSlowDriving, origin_speed_follow_cost * follow_ki);
  if (!is_valid_ego_v || !is_valid_speed_diff)
    follow_cost_info.integration = 0.0;
  follow_cost_info.integration =
      std::clamp(follow_cost_info.integration, 0.0, 0.6);
  if (follow_cost_info.integration == 0.0) {
    follow_cost_info.count = 0;
  }
  follow_cost_info.final_follow_cost =
      follow_cost_info.integration + follow_cost_info.speed_follow_cost;
  return follow_cost_info;
}

int SelectorCostHistory::UpdateAndRemoveDecay(absl::Time plan_time) {
  cur_time_ = plan_time;
  int removed_count = 0;
  for (auto &[_, object_cost_map] : path_object_costs_) {
    std::vector<ObjectIdType> expired_keys;
    for (auto &[key, follow_cost_info] : object_cost_map) {
      if (follow_cost_info.ts >= plan_time) {
        continue;
      }
      follow_cost_info.final_follow_cost *= GetDecayFactor();
      follow_cost_info.integration *= GetDecayFactor();
      follow_cost_info.ts = plan_time;
      if (follow_cost_info.final_follow_cost < expire_cost_threshold_) {
        expired_keys.push_back(key);
      }
    }
    removed_count += static_cast<int>(expired_keys.size());
    for (const auto &key : expired_keys) {
      std::ignore = object_cost_map.erase(key);
    }
  }
  return removed_count;
}

}  // namespace st::planning
