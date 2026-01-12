

#ifndef AD_BYD_PLANNING_MAP_MAP_SCENE_H
#define AD_BYD_PLANNING_MAP_MAP_SCENE_H

#include <list>
#include <unordered_map>

#include "plan_common/maps/map_graph.h"

namespace ad_byd {
namespace planning {

class MapScene {
 public:
  void Insert(const std::string& lane_id, const MapGraphConstPtr& map_graph);

  const MapGraphConstPtr GetMapGraphByLane(const std::string& lane_id) const;
  const std::unordered_map<std::string, MapGraphConstPtr>& MapGraph() const;

 private:
  std::unordered_map<std::string, MapGraphConstPtr> lane_map_graph_;
};

using MapScenePtr = std::shared_ptr<MapScene>;
using MapSceneConstPtr = std::shared_ptr<const MapScene>;
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_MAP_SCENE_H
