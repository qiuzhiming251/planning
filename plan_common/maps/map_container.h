

#ifndef AD_BYD_PLANNING_MAPS_MAP_CONTAINER_H
#define AD_BYD_PLANNING_MAPS_MAP_CONTAINER_H

#include <atomic>
#include <memory>
#include <vector>

#include "plan_common/maps/ld_lite_map.h"
#include "plan_common/maps/map_element.h"
#include "plan_common/planning_error.h"

namespace ad_byd {
namespace planning {

class MapContainer {
 public:
  MapContainer();
  ~MapContainer() = default;

  void Init();
  void InsertMap(const MapInfo& map_info);
  void InsertLdLiteMap(const LdLiteMapInfo& sd_map_info);
  ErrorCode GetMap(MapElement* const map_element);
  ErrorCode GetLdLiteMap(LdLiteMapPtr& ld_lite_map);

 private:
  void GenerateMapScene();
  void GenerateVectorMap();

 private:
  std::mutex map_mutex_;
  MapElement ready_element_;
  MapElement latest_element_;
  // std::vector<PolylineExtractor*> polyline_extractors_;
  std::mutex lite_map_mutex_;
  LdLiteMapPtr ready_lite_map_{nullptr};
  LdLiteMapPtr latest_lite_map_{nullptr};
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAPS_MAP_CONTAINER_H
