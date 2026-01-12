#ifndef PLAN_COMMON_UTIL_TRAFFIC_UTIL
#define PLAN_COMMON_UTIL_TRAFFIC_UTIL

#include <string>
#include <vector>
#include <optional>

#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"

namespace st::planning {

using LeadingGroup = std::map<std::string, ConstraintProto::LeadingObjectProto>;

struct TrafficGapResult {
  std::optional<std::string> leader_id = std::nullopt;
  std::optional<std::string> follower_id = std::nullopt;
  std::optional<double> dec_gap_target_speed = std::nullopt;
  std::optional<double> dec_gap_target_a = std::nullopt;
  std::optional<double> acc_gap_target_speed = std::nullopt;
  std::optional<double> acc_gap_target_a = std::nullopt;
  std::optional<double> gap_length = std::nullopt;
  std::optional<double> gap_ttc = std::nullopt;
  std::optional<double> gap_v = std::nullopt;
  std::optional<double> gap_a = std::nullopt;
  std::optional<double> gap_t = std::nullopt;
  std::optional<double> gap_total_cost = std::nullopt;
  std::optional<double> gap_ref_v = std::nullopt;
  bool ego_in_gap = false;
};

struct NudgeObjectInfo {
  enum Type {
    NORMAL = 0,
    LARGE_VEHICLE = 1,
  };
  enum NudgeState {
    NUDGE = 0,
    BORROW = 1,
  };
  std::string id;
  int direction;
  double arc_dist_to_object;
  ObjectType type;
  NudgeState nudge_state;
};

struct NudgeInfos {
  std::vector<NudgeObjectInfo> nudgeInfos;
  void addNudgeInfo(const NudgeObjectInfo& info) { nudgeInfos.push_back(info); }
  std::optional<NudgeObjectInfo> findNudgeInfoById(
      const std::string& id) const {
    for (const auto& info : nudgeInfos) {
      if (info.id == id) {
        return info;
      }
    }
    return std::nullopt;  // Return an empty optional if not found
  }
};

struct TrajEvalInfo {
  double eval_cost;
  absl::flat_hash_set<std::string> follower_set{};
  absl::flat_hash_set<std::string> leader_set{};
  double follower_max_decel = 0.0;
  double leader_max_decel = 0.0;

  std::string follower_max_id = "";
  std::string leader_max_id = "";

  double follower_risk_factor = 0.0;
  double leader_risk_factor = 0.0;

  std::ostringstream follower_debug_info;
  std::ostringstream leader_debug_info;

  std::string unsafe_object_id = "";
  PlannerStatusProto::PlannerStatusCode status_code = PlannerStatusProto::OK;
};
}  // namespace st::planning

#endif  // PLAN_COMMON_UTIL_TRAFFIC_UTIL