

#include "spacetime_trajectory_manager.h"

#include <algorithm>
#include <iterator>
#include <ostream>

#include "plan_common/async/parallel_for.h"
//#include "global/buffered_logger.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/log_data.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "trajectory_filter.h"
#include "predictor/prediction.h"

namespace st {
namespace planning {
namespace {
template <typename T>
using NestedVector = std::vector<std::vector<T>>;

// This is the object buffer that AV should never enter.
double ComputeRequiredLateralGap(const PlannerObject& object) {
  switch (object.type()) {
    case OT_FOD:
      return 0.0;
    case OT_UNKNOWN_STATIC:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.15;
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
    case OT_ROW_OBSTACLES:
      return 0.2;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}
}  // namespace

SpacetimeTrajectoryManager::SpacetimeTrajectoryManager(
    absl::Span<const TrajectoryFilter* const> filters,
    absl::Span<const PlannerObject> planner_objects,
    absl::Span<const PlannerObject> frame_dropped_objects,
    ThreadPool* thread_pool, int plan_id) {
  // ("SpacetimeTrajectoryManager");
  TIMELINE("SpacetimeTrajectoryManager");

  const int num_objects_dropped = frame_dropped_objects.size();
  NestedVector<SpacetimeObjectTrajectory> dropped_trajs_per_object(
      num_objects_dropped);
  const int num_objects = planner_objects.size();
  NestedVector<SpacetimeObjectTrajectory> considered_trajs_per_object(
      num_objects);
  NestedVector<IgnoredTrajectory> ignored_trajs_per_object(num_objects);
  NestedVector<StationaryObject> stationary_per_object(num_objects);
  const int num_objects_total = num_objects_dropped + num_objects;

  // do not use the thread pool
  ParallelFor(0, num_objects_total, thread_pool, [&](int i) {
    if (i < num_objects) {
      const auto& planner_object = planner_objects[i];
      const auto& trajectories = planner_object.prediction().trajectories();
      CHECK(!trajectories.empty())
          << planner_object.id() << " has no trajectory.";
      if (planner_object.is_stationary()) {
        stationary_per_object[i].push_back({.object_id = planner_object.id(),
                                            .planner_object = planner_object});
      }
      for (int traj_index = 0, s = trajectories.size(); traj_index < s;
           ++traj_index) {
        const auto& pred_traj = trajectories[traj_index];
        bool filtered = false;
        for (const auto* filter : filters) {
          const auto filter_reason = filter->Filter(planner_object, pred_traj);
          if (filter_reason != FilterReason::NONE) {
            ignored_trajs_per_object[i].push_back(
                {.traj = &pred_traj,
                 .object_id = planner_object.id(),
                 .reason = filter_reason});
            filtered = true;
            Log2DDS::LogDataV2(
                "object_filters",
                absl::StrCat(
                    Log2DDS::TaskPrefix(plan_id),
                    "[st_traj_mgr] filtered obj_id:", planner_object.id(),
                    " reason:", FilterReason_Type_Name(filter_reason)));
            break;
          }
        }
        if (filtered) continue;

        const double required_lateral_gap =
            ComputeRequiredLateralGap(planner_object);
        considered_trajs_per_object[i].emplace_back(planner_object, traj_index,
                                                    required_lateral_gap);
      }
    } else {
      const auto& planner_object_dropped =
          frame_dropped_objects[i - num_objects];
      const auto& trajectories =
          planner_object_dropped.prediction().trajectories();
      CHECK(!trajectories.empty())
          << planner_object_dropped.id() << " has no trajectory.";
      for (int traj_index = 0, s = trajectories.size(); traj_index < s;
           ++traj_index) {
        const auto& pred_traj = trajectories[traj_index];
        bool filtered = false;
        for (const auto* filter : filters) {
          const auto filter_reason =
              filter->Filter(planner_object_dropped, pred_traj);
          if (filter_reason != FilterReason::NONE) {
            filtered = true;
            Log2DDS::LogDataV2(
                "object_filters",
                absl::StrCat(Log2DDS::TaskPrefix(plan_id),
                             "[st_traj_mgr] filtered obj_id:",
                             planner_object_dropped.id(), " reason:",
                             FilterReason_Type_Name(filter_reason)));
            break;
          }
        }
        if (filtered) continue;

        const double required_lateral_gap =
            ComputeRequiredLateralGap(planner_object_dropped);
        dropped_trajs_per_object[i - num_objects].emplace_back(
            planner_object_dropped, traj_index, required_lateral_gap);
      }
    }
  });

  int trajectories_size = 0;
  for (const auto& planner_object : planner_objects) {
    trajectories_size += planner_object.prediction().trajectories().size();
  }
  considered_trajs_.reserve(trajectories_size);
  ignored_trajs_.reserve(trajectories_size);

  // Collect results from parallel for.
  int stationary_count = 0;
  for (auto& trajs : considered_trajs_per_object) {
    for (const auto& traj : trajs) {
      // Count stationary trajs in all considered trajs.
      if (traj.is_stationary()) stationary_count++;
    }
    auto temp_trajs = trajs;
    std::move(trajs.begin(), trajs.end(),
              std::back_inserter(considered_trajs_));
    std::move(temp_trajs.begin(), temp_trajs.end(),
              std::back_inserter(frame_dropped_trajs_));
  }
  for (auto& trajs : ignored_trajs_per_object) {
    std::move(trajs.begin(), trajs.end(), std::back_inserter(ignored_trajs_));
  }
  for (auto& obj : stationary_per_object) {
    std::move(obj.begin(), obj.end(), std::back_inserter(stationary_objs_));
  }

  for (auto& trajs : dropped_trajs_per_object) {
    std::move(trajs.begin(), trajs.end(),
              std::back_inserter(frame_dropped_trajs_));
  }

  // Classify trajectories.
  UpdatePointers(stationary_count);
  // for (auto iter = trajectories_id_map_.begin(); iter !=
  // trajectories_id_map_.end(); ++iter) {
  //   LOG_ERROR << "trajectories_id_map_wwwwwwwww: " << iter->first << ", " <<
  //   iter->second->states().size();
  // }
}

SpacetimeTrajectoryManager::SpacetimeTrajectoryManager(
    absl::Span<SpacetimeObjectTrajectory> spacetime_trajectories) {
  const int traj_size = spacetime_trajectories.size();
  considered_trajs_.reserve(traj_size);
  int stationary_count = 0;
  for (auto& traj : spacetime_trajectories) {
    if (traj.is_stationary()) {
      stationary_count++;
    }
    considered_trajs_.push_back(std::move(traj));
  }
  UpdatePointers(stationary_count);
}

SpacetimeTrajectoryManager::SpacetimeTrajectoryManager(
    const SpacetimeTrajectoryManager& other) {
  *this = other;
}

SpacetimeTrajectoryManager& SpacetimeTrajectoryManager::operator=(
    const SpacetimeTrajectoryManager& other) {
  // ("SpacetimeTrajectoryManager/CopyAssignment");
  if (this != &other) {
    ignored_trajs_ = other.ignored_trajs_;
    considered_trajs_ = other.considered_trajs_;
    stationary_objs_ = other.stationary_objs_;
    frame_dropped_trajs_ = other.frame_dropped_trajs_;
    UpdatePointers(other.considered_stationary_trajs_.size());
  }
  return *this;
}

void SpacetimeTrajectoryManager::UpdatePointers(int stationary_size) {
  // ("SpacetimeTrajectoryManager/UpdatePointers");
  considered_stationary_trajs_.clear();
  considered_moving_trajs_.clear();
  objects_id_map_.clear();
  trajectories_id_map_.clear();
  const int traj_size = considered_trajs_.size();
  considered_stationary_trajs_.reserve(stationary_size);
  CHECK_GE(traj_size, stationary_size);
  considered_moving_trajs_.reserve(traj_size - stationary_size);
  objects_id_map_.reserve(traj_size);
  trajectories_id_map_.reserve(traj_size);
  for (const auto& traj : considered_trajs_) {
    if (traj.is_stationary()) {
      considered_stationary_trajs_.push_back(&traj);
    } else {
      considered_moving_trajs_.push_back(&traj);
    }
    objects_id_map_[traj.planner_object().id()].push_back(&traj);
    trajectories_id_map_[traj.traj_id()] = &traj;
  }
}

}  // namespace planning
}  // namespace st
