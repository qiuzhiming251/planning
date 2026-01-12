
#include "plan_common/maps/map_scene.h"

namespace ad_byd {
namespace planning {
void MapScene::Insert(const std::string& lane_id,
                      const MapGraphConstPtr& map_graph) {
  if (lane_map_graph_.find(lane_id) == lane_map_graph_.end() &&
      map_graph->lane_ptr() != nullptr) {
    lane_map_graph_[lane_id] = map_graph;
  }
}

const MapGraphConstPtr MapScene::GetMapGraphByLane(
    const std::string& lane_id) const {
  if (lane_map_graph_.find(lane_id) == lane_map_graph_.end()) {
    return nullptr;
  }
  return lane_map_graph_.at(lane_id);
}

const std::unordered_map<std::string, MapGraphConstPtr>& MapScene::MapGraph()
    const {
  return lane_map_graph_;
}

}  // namespace planning
}  // namespace ad_byd