

#ifndef ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_H_
#define ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_H_

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "plan_common/util/map_util.h"

namespace st::planning {

struct ObjectOverlapState {
  double bound = 0.0;
  double speed = 0.0;
  double prob = 0.0;
  double delta_speed_factor = 0.0;
  double lon_buffer = 0.0;
  bool no_weak_buffer = false;
};

// Speed optimizer object created by st_boundary.
class SpeedOptimizerObject {
 public:
  SpeedOptimizerObject(
      std::vector<std::optional<ObjectOverlapState>> overlap_states,
      double state_time_interval, std::string id, double standstill,
      StBoundaryProto::ObjectType object_type,
      StBoundarySourceTypeProto::Type source_type,
      StBoundaryProto::ProtectionType protection_type, int first_overlap_idx)
      : overlap_states_(std::move(overlap_states)),
        state_time_interval_(state_time_interval),
        id_(std::move(id)),
        standstill_(standstill),
        object_type_(object_type),
        source_type_(source_type),
        protection_type_(protection_type),
        first_overlap_idx_(first_overlap_idx) {}

  const std::optional<ObjectOverlapState>& GetOverlapStateByIndex(
      int idx) const {
    CHECK_LT(idx, overlap_states_.size());
    return overlap_states_[idx];
  }

  const std::optional<ObjectOverlapState>& GetOverlapStateByTime(
      double time) const {
    const int idx = FloorToInt(time / state_time_interval_);
    CHECK_LT(idx, overlap_states_.size());
    return overlap_states_[idx];
  }

  const std::string& id() const { return id_; }

  bool is_protective() const {
    return protection_type_ != StBoundaryProto::NON_PROTECTIVE;
  }

  StBoundaryProto::ProtectionType protection_type() const {
    return protection_type_;
  }

  StBoundaryProto::ObjectType object_type() const { return object_type_; }

  StBoundarySourceTypeProto::Type source_type() const { return source_type_; }

  double standstill() const { return standstill_; }

  double first_overlap_time() const {
    return first_overlap_idx_ * state_time_interval_;
  }

  const ObjectOverlapState& first_overlap_state() const {
    const auto& fo_state = overlap_states_[first_overlap_idx_];
    CHECK(fo_state.has_value());
    return *fo_state;
  }

 private:
  // Vector index corresponds to the knot index in speed optimizer.
  std::vector<std::optional<ObjectOverlapState>> overlap_states_;
  double state_time_interval_ = 0.0;
  // If object source_type is st_obj, the id is object_id, otherwise the id is
  // st_boundary_id.
  std::string id_;
  double standstill_ = 0.0;
  StBoundaryProto::ObjectType object_type_;
  StBoundarySourceTypeProto::Type source_type_;
  StBoundaryProto::ProtectionType protection_type_;
  int first_overlap_idx_ = 0;
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_H_
