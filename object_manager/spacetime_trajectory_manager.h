

#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_

#include <string>
#include <utility>
#include <vector>
#include <cereal/cereal.hpp>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "plan_common/async/thread_pool.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "planner_object.h"
#include "spacetime_object_trajectory.h"
#include "predictor/predicted_trajectory.h"
#include "plan_common/util/map_util.h"

namespace st {

namespace planning {

class TrajectoryFilter;

// The manager of all the spacetime object trajectories. Each prediction or
// stationary object is modeled as a `SpacetimeObjectTrajectory`.
class SpacetimeTrajectoryManager {
 public:
  SpacetimeTrajectoryManager() {}

  SpacetimeTrajectoryManager(
      absl::Span<const TrajectoryFilter* const> filters,
      absl::Span<const PlannerObject> planner_objects,
      absl::Span<const PlannerObject> frame_dropped_objects,
      ThreadPool* thread_pool, int plan_id = 99);
  explicit SpacetimeTrajectoryManager(
      absl::Span<const PlannerObject> planner_objects,
      absl::Span<const PlannerObject> frame_dropped_objects,
      ThreadPool* thread_pool = nullptr)
      : SpacetimeTrajectoryManager(
            /*filter=*/{}, planner_objects, frame_dropped_objects,
            thread_pool) {}

  explicit SpacetimeTrajectoryManager(
      absl::Span<SpacetimeObjectTrajectory> spacetime_trajectories);

  // Deep copy.
  SpacetimeTrajectoryManager(const SpacetimeTrajectoryManager& other);
  SpacetimeTrajectoryManager& operator=(
      const SpacetimeTrajectoryManager& other);

  SpacetimeTrajectoryManager(SpacetimeTrajectoryManager&& other) = default;
  SpacetimeTrajectoryManager& operator=(SpacetimeTrajectoryManager&& other) =
      default;

  // All the considered stationary object 'trajectories'.
  absl::Span<const SpacetimeObjectTrajectory* const> stationary_object_trajs()
      const {
    return considered_stationary_trajs_;
  }

  // All the considered moving trajectories.
  absl::Span<const SpacetimeObjectTrajectory* const> moving_object_trajs()
      const {
    return considered_moving_trajs_;
  }

  // All the considered trajectories.
  absl::Span<const SpacetimeObjectTrajectory> trajectories() const {
    return considered_trajs_;
  }

  std::vector<SpacetimeObjectTrajectory>* mutable_trajectories() {
    return &considered_trajs_;
  }

  absl::Span<const SpacetimeObjectTrajectory> frame_dropped_trajectories()
      const {
    return frame_dropped_trajs_;
  }

  const absl::flat_hash_map<std::string,
                            std::vector<const SpacetimeObjectTrajectory*>>&
  object_trajectories_map() const {
    return objects_id_map_;
  }

  // Returns all the considered trajectories of an object id.
  absl::Span<const SpacetimeObjectTrajectory* const> FindTrajectoriesByObjectId(
      std::string id) const {
    const auto iter = objects_id_map_.find(id);
    if (iter == objects_id_map_.end()) return {};
    return iter->second;
  }

  // Returns the planner object of an object id.
  const PlannerObject* FindObjectByObjectId(std::string id) const {
    const auto iter = objects_id_map_.find(id);
    if (iter == objects_id_map_.end()) return nullptr;
    return &(iter->second.front()->planner_object());
  }

  // Returns the considered trajectory of traj id.
  const SpacetimeObjectTrajectory* FindTrajectoryById(
      std::string_view traj_id) const {
    auto str = std::string(traj_id);
    // LOG_ERROR << "input trajectory id: " << traj_id;
    // for (auto iter = trajectories_id_map_.begin(); iter !=
    // trajectories_id_map_.end(); ++iter) {
    //   LOG_ERROR << "trajectory idq: " << iter->first;
    // }
    return FindPtrOrNull(trajectories_id_map_, str);
  }

  // An Ignored trajectory and the ignore reason.
  struct IgnoredTrajectory {
    const prediction::PredictedTrajectory* traj;
    std::string object_id;
    FilterReason::Type reason;
  };
  struct StationaryObject {
    std::string object_id;
    PlannerObject planner_object;
  };

  // All the ignored trajectories.
  absl::Span<const IgnoredTrajectory> ignored_trajectories() const {
    return ignored_trajs_;
  }

  absl::Span<const StationaryObject> stationary_objects() const {
    return stationary_objs_;
  }

  void UpdatePointers(int stationary_size);

 protected:
  absl::flat_hash_map<std::string,
                      std::vector<const SpacetimeObjectTrajectory*>>
      objects_id_map_;
  absl::flat_hash_map<std::string, const SpacetimeObjectTrajectory*>
      trajectories_id_map_;

  std::vector<SpacetimeObjectTrajectory> considered_trajs_;
  std::vector<IgnoredTrajectory> ignored_trajs_;
  std::vector<StationaryObject> stationary_objs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_stationary_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_moving_trajs_;
  std::vector<SpacetimeObjectTrajectory> frame_dropped_trajs_;

  template <typename Archive>
  friend void serialize(
      Archive& ar, SpacetimeTrajectoryManager& spacetime_trajectory_manager);
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_
