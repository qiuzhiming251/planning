//
// Created by xxx on 7/28/21.
//

#ifndef ST_PLANNING_DECISION
#define ST_PLANNING_DECISION
#include <stdint.h>

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <cereal/cereal.hpp>

#include "absl/time/time.h"
#include "decision_supplement.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/lite_common.pb.h"
#include "planner/planner_manager/planner_defs.h"

namespace st::planning {

class Decision {
 public:
  Decision() = default;

  // reset all decision to none
  void Reset();

  /// reset when scenario reset
  void ResetByScenario();
  // the offset we should keep when lane keeping, unit m
  double lane_follow_keep_time() const { return lane_follow_keep_time_; }
  void set_lane_follow_keep_time_(const double &lane_follow_keep_time);

  DecisionSupplementPtr GetSupplement(const std::string &name) const;
  void SetSupplement(const std::string &name, DecisionSupplementPtr supplement);
  const std::map<std::string, DecisionSupplementPtr> &supplements_map() const {
    return supplements_map_;
  }

 private:
  double lane_follow_keep_time_ = 0.0;
  // supplemenlast_ts
  std::map<std::string, DecisionSupplementPtr> supplements_map_;

  // template <typename Archive>
  // friend void serialize(Archive& ar, st::planning::Decision& decision);
};

}  // namespace st::planning

#endif  // ST_PLANNING_DECISION
