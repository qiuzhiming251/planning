

#include "plan_common/maps/vector_map/vector_map.h"

namespace ad_byd {
namespace planning {

void VectorMap::Insert(LanePolyline&& lane_polyline) {
  auto id = lane_polyline.ToString();
  lane_polyline_map_[id] = std::move(lane_polyline);
  for (const auto& segment_id :
       lane_polyline_map_.at(id).polyline_id().segment_ids()) {
    lane_id_polyline_map_[segment_id.instance_id()].emplace_back(id);
  }
}

void VectorMap::Insert(LaneBoundaryPolyline&& lane_boundary_polyline) {
  auto id = lane_boundary_polyline.ToString();
  lane_boundary_polyline_map_[id] = std::move(lane_boundary_polyline);
  for (const auto& segment_id :
       lane_boundary_polyline_map_[id].polyline_id().segment_ids()) {
    lane_bound_id_polyline_map_[segment_id.instance_id()].emplace_back(id);
  }
}

void VectorMap::Insert(PolygonPolyline&& polygon_polyline) {
  if (polygon_polyline.polyline_id().segment_ids().empty()) return;
  auto id = polygon_polyline.ToString();
  const auto& instance_id =
      polygon_polyline.polyline_id().segment_ids().front().instance_id();
  if (polygon_polyline.polygon_type() == PolygonType::CROSSWALK) {
    crosswalk_id_polyline_map_[instance_id] = id;
  } else if (polygon_polyline.polygon_type() == PolygonType::JUNCTION) {
    junction_id_polyline_map_[instance_id] = id;
  } else if (polygon_polyline.polygon_type() == PolygonType::SPEEDBUMP) {
    speed_bump_id_polyline_map_[instance_id] = id;
  }
  polygon_polyline_map_[id] = std::move(polygon_polyline);
}

void VectorMap::Insert(RoadBoundaryPolyline&& road_boundary_polyline) {
  auto id = road_boundary_polyline.ToString();
  road_boundary_polyline_map_[id] = std::move(road_boundary_polyline);
  for (const auto& segment_id :
       road_boundary_polyline_map_.at(id).polyline_id().segment_ids()) {
    road_bound_id_polyline_map_[segment_id.instance_id()].emplace_back(id);
  }
}

void VectorMap::Insert(StopLinePolyline&& stop_line_polyline) {
  auto id = stop_line_polyline.ToString();
  stop_line_id_polyline_map_
      [stop_line_polyline.polyline_id().segment_ids().front().instance_id()] =
          id;
  stop_line_polyline_map_[id] = std::move(stop_line_polyline);
}

}  // namespace planning
}  // namespace ad_byd
