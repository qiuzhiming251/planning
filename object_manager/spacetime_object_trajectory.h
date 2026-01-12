

#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_TRAJECTORY_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_TRAJECTORY_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "planner_object.h"
#include "predictor/prediction_object_state.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction.h"

namespace st::planning {

// A trajectory of a spacetime object.
class SpacetimeObjectTrajectory {
 public:
  SpacetimeObjectTrajectory(const PlannerObject& planner_object, int traj_index,
                            double required_lateral_gap);
  SpacetimeObjectTrajectory(
      const PlannerObject& planner_object,
      std::vector<prediction::PredictionObjectState> states, int traj_index,
      double required_lateral_gap);
  SpacetimeObjectTrajectory(const PlannerObject& planner_object,
                            prediction::PredictedTrajectory traj,
                            int traj_index, double required_lateral_gap);

  // Deep copy.
  SpacetimeObjectTrajectory(const SpacetimeObjectTrajectory& other);
  SpacetimeObjectTrajectory& operator=(const SpacetimeObjectTrajectory& other);

  SpacetimeObjectTrajectory(SpacetimeObjectTrajectory&& other) = default;
  SpacetimeObjectTrajectory& operator=(SpacetimeObjectTrajectory&& other) =
      default;

  absl::StatusOr<SpacetimeObjectTrajectory> CreateTruncatedCopy(
      double start_offset, double end_t) const;

  SpacetimeObjectTrajectory CreateTrajectoryMutatedInstance(
      prediction::PredictedTrajectory traj) const {
    return SpacetimeObjectTrajectory(planner_object_, std::move(traj),
                                     traj_index_, required_lateral_gap_);
  }

  SpacetimeObjectTrajectory CreateLateralGapMutatedInstance(
      double lateral_gap) const {
    return SpacetimeObjectTrajectory(planner_object_, traj_index_, lateral_gap);
  }

  absl::Span<const prediction::PredictionObjectState> states() const {
    return states_;
  }

  // absl::Span<prediction::PredictionObjectState> mutable_states() {
  //   return absl::MakeSpan(states_);
  // }

  std::vector<prediction::PredictionObjectState>* mutable_states() {
    return &states_;
  }

  const std::string& traj_id() const { return traj_id_; }

  int traj_index() const { return traj_index_; }

  double required_lateral_gap() const { return required_lateral_gap_; }

  void set_required_lateral_gap(double lateral_gap) {
    required_lateral_gap_ = lateral_gap;
  }

  bool is_stationary() const { return is_stationary_; }

  absl::string_view object_id() const { return planner_object_.id(); }
  // std::string object_id() const { return planner_object_.id(); }

  ObjectType object_type() const { return planner_object_.type(); }

  const prediction::ObjectLongTermBehavior& long_term_behavior() const {
    return planner_object_.long_term_behavior();
  }

  const SecondOrderTrajectoryPoint& pose() const {
    return *states_[0].traj_point;
  }

  const Polygon2d& contour() const { return states_[0].contour; }

  const Box2d& bounding_box() const { return states_[0].box; }

  // TODO： Remove it.
  const PlannerObject& planner_object() const { return planner_object_; }

  const prediction::PredictedTrajectory& trajectory() const {
    return trajectory_;
  }

  prediction::PredictedTrajectory* mutable_trajectory() { return &trajectory_; }

  // A utility function to generate trajectory's id.
  static std::string MakeTrajectoryId(std::string obj_id, int traj_index) {
    return absl::StrFormat("%s-idx%d", obj_id, traj_index);
  }

  // A utility function to extract object's id from trajectory id.
  static std::string GetObjectIdFromTrajectoryId(std::string traj_id) {
    const std::vector<std::string> tokens = absl::StrSplit(traj_id, "-");
    CHECK(!tokens.empty());
    return tokens.front();
  }

 private:
  int traj_index_;
  // The id of the spacetime trajectory.
  std::string traj_id_;
  bool is_stationary_;
  PlannerObject planner_object_;
  double required_lateral_gap_;
  std::vector<prediction::PredictionObjectState> states_;
  prediction::PredictedTrajectory trajectory_;

  template <typename Archive>
  friend void serialize(Archive& ar,
                        SpacetimeObjectTrajectory& spacetime_object_trajectory);
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_TRAJECTORY_H_
