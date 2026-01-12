#ifndef AD_BYD_PLANNING_MAPS_MAP_ELEMENT_H
#define AD_BYD_PLANNING_MAPS_MAP_ELEMENT_H
#include "plan_common/maps/map.h"
#include "plan_common/maps/map_scene.h"
#include "plan_common/maps/vector_map/vector_map.h"

namespace ad_byd {
namespace planning {

struct MapElement {
  MapPtr map_ptr = nullptr;
  MapScenePtr map_scene_ptr = nullptr;
  VectorMapPtr vector_map_ptr = nullptr;

  bool IsValid() const {
    // return map_ptr != nullptr && map_scene_ptr != nullptr &&
    //        vector_map_ptr != nullptr && map_ptr->IsValid();
    return map_ptr != nullptr && map_ptr->IsValid();
  }
  void reset() {
    map_ptr = nullptr;
    map_scene_ptr = nullptr;
    vector_map_ptr = nullptr;
  }
};
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAPS_MAP_ELEMENT_H
