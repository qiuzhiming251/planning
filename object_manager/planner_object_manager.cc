

#include <algorithm>
#include <utility>

#include "planner_object_manager.h"

namespace st {
namespace planning {

PlannerObjectManager::PlannerObjectManager(
    ObjectVector<PlannerObject> planner_objects,
    ObjectVector<PlannerObject> frame_dropped_objects)
    : planner_objects_(std::move(planner_objects)),
      frame_dropped_objects_(std::move(frame_dropped_objects)) {
  stationary_objects_.reserve(planner_objects_.size());
  moving_objects_.reserve(planner_objects_.size());
  object_map_.reserve(planner_objects_.size());
  for (auto& object : planner_objects_) {
    if (object.is_stationary()) {
      stationary_objects_.push_back(&object);
    } else {
      moving_objects_.push_back(&object);
    }
    object_map_[object.id()] = &object;
  }
}

}  // namespace planning
}  // namespace st
