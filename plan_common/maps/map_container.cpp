

#include "plan_common/async/parallel_for.h"
#include "plan_common/maps/map_container.h"
//#include "module/extractor/polyline/polyline_extractor_manager.h"

namespace ad_byd {
namespace planning {

MapContainer::MapContainer() {}

void MapContainer::Init() {
  LOG_WARN << "MapContainer Init";

  // std::vector<PolylineExtractorType> polyline_extractor_types = {
  //     PolylineExtractorType::LANEPOLYLINE,
  //     PolylineExtractorType::POLYGONPOLYLINE,
  //     PolylineExtractorType::ROADBOUNDARYPOLYLINE,
  //     PolylineExtractorType::STOPLINEPOLYLINE};
  // for (const auto& type : polyline_extractor_types) {
  //   PolylineExtractor* extractor = nullptr;
  //   if (PolylineExtractorManager::instance()->GetExtractor(type, &extractor)
  //   !=
  //           ErrorCode::PLANNING_OK ||
  //       extractor == nullptr) {
  //     continue;
  //   }
  //   polyline_extractors_.emplace_back(extractor);
  // }
}

void MapContainer::InsertMap(const MapInfo& map_info) {
  latest_element_.map_ptr = std::make_shared<Map>(map_info);
  // GenerateMapScene();
  // GenerateVectorMap();
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    ready_element_ = latest_element_;
  }
}

ErrorCode MapContainer::GetMap(MapElement* const map_element) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!ready_element_.IsValid()) {
    return ErrorCode::PLANNING_MAP_FAILED;
  }
  *map_element = ready_element_;
  return ErrorCode::PLANNING_OK;
}

void MapContainer::InsertLdLiteMap(const LdLiteMapInfo& sd_map_info) {
  latest_lite_map_ = std::make_shared<LdLiteMap>(sd_map_info);
  {
    std::lock_guard<std::mutex> lock(lite_map_mutex_);
    ready_lite_map_ = latest_lite_map_;
  }
}

ErrorCode MapContainer::GetLdLiteMap(LdLiteMapPtr& ld_lite_map) {
  std::lock_guard<std::mutex> lock(lite_map_mutex_);
  ld_lite_map = ready_lite_map_;
  return ErrorCode::PLANNING_OK;
}

// void MapContainer::GenerateMapScene() {
//   auto map_scene_ptr = std::make_shared<MapScene>();
//   const auto& map_ptr = latest_element_.map_ptr;
//   std::mutex mutex;
//   st::MapParallelFor(
//       map_ptr->lanes().begin(), map_ptr->lanes().end(),
//       [&](LaneConstPtr lane_ptr) {
//         if (!lane_ptr->IsValid()) return;
//         MapGraphConstPtr map_graph = std::make_shared<MapGraph>(
//             lane_ptr, FLAGS_ad_byd_planning_forward_lane_sequence_length,
//             FLAGS_ad_byd_planning_backward_lane_sequence_length, map_ptr);
//         {
//           std::lock_guard<std::mutex> lock(mutex);
//           map_scene_ptr->Insert(lane_ptr->id(), map_graph);
//         }
//       });
//   latest_element_.map_scene_ptr = map_scene_ptr;
// }

// void MapContainer::GenerateVectorMap() {
//   VectorMapPtr vector_map = std::make_shared<VectorMap>();
//   const auto& map_ptr = latest_element_.map_ptr;
//   vector_map->set_timestamp(map_ptr->timestamp());
//   st::MapParallelFor(polyline_extractors_.begin(),
//   polyline_extractors_.end(),
//                      [&](PolylineExtractor* extractor) {
//                        extractor->ExtractPolyline(map_ptr, vector_map);
//                      });
//   latest_element_.vector_map_ptr = vector_map;
// }

}  // namespace planning
}  // namespace ad_byd
