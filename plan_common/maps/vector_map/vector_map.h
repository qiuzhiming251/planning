

#ifndef AD_BYD_PLANNING_MAP_VECTOR_MAP_H
#define AD_BYD_PLANNING_MAP_VECTOR_MAP_H

#include <list>
#include <memory>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "plan_common/planning_error.h"
#include "plan_common/maps/polyline.h"
#include "plan_common/maps/map.h"

namespace ad_byd {
namespace planning {

class VectorMap {
 public:
  struct VectorizeContext {
    std::unordered_map<std::string, Point2d> lane_split_point;
    std::unordered_map<std::string, Point2d> lane_boundary_split_point;
  };
  VectorMap() = default;
  ~VectorMap() = default;

  void Insert(LanePolyline&& lane_polyline);
  void Insert(LaneBoundaryPolyline&& lane_boundary_polyline);
  void Insert(PolygonPolyline&& polygon_polyline);
  void Insert(RoadBoundaryPolyline&& road_boundary_polyline);
  void Insert(StopLinePolyline&& stop_line_polyline);

  const std::unordered_map<std::string, LanePolyline>& lane_polyline_map()
      const {
    return lane_polyline_map_;
  }
  const std::unordered_map<std::string, LaneBoundaryPolyline>&
  lane_boundary_polyline_map() const {
    return lane_boundary_polyline_map_;
  }
  const std::unordered_map<std::string, std::vector<std::string>>&
  lane_id_polyline_map() const {
    return lane_id_polyline_map_;
  }
  const std::unordered_map<std::string, std::vector<std::string>>&
  lane_bound_id_polyline_map() const {
    return lane_bound_id_polyline_map_;
  }
  const std::unordered_map<std::string, PolygonPolyline>& polygon_polyline_map()
      const {
    return polygon_polyline_map_;
  }
  const std::unordered_map<std::string, std::string>&
  crosswalk_id_polyline_map() const {
    return crosswalk_id_polyline_map_;
  }
  const std::unordered_map<std::string, std::string>& junction_id_polyline_map()
      const {
    return junction_id_polyline_map_;
  }

  const std::unordered_map<std::string, std::string>&
  speed_bump_id_polyline_map() const {
    return speed_bump_id_polyline_map_;
  }

  const std::unordered_map<std::string, RoadBoundaryPolyline>&
  road_boundary_polyline_map() const {
    return road_boundary_polyline_map_;
  }

  const std::unordered_map<std::string, std::vector<std::string>>&
  road_bound_id_polyline_map() const {
    return road_bound_id_polyline_map_;
  }

  const std::unordered_map<std::string, StopLinePolyline>&
  stop_line_polyline_map() const {
    return stop_line_polyline_map_;
  }

  const std::unordered_map<std::string, std::string>&
  stop_line_id_polyline_map() const {
    return stop_line_id_polyline_map_;
  }

  void set_timestamp(double timestamp) { timestamp_ = timestamp; }
  double timestamp() const { return timestamp_; }

 private:
  double timestamp_;
  std::unordered_map<std::string, LanePolyline> lane_polyline_map_;
  std::unordered_map<std::string, LaneBoundaryPolyline>
      lane_boundary_polyline_map_;
  std::unordered_map<std::string, PolygonPolyline> polygon_polyline_map_;
  std::unordered_map<std::string, RoadBoundaryPolyline>
      road_boundary_polyline_map_;
  std::unordered_map<std::string, std::vector<std::string>>
      lane_id_polyline_map_;
  std::unordered_map<std::string, std::vector<std::string>>
      lane_bound_id_polyline_map_;
  std::unordered_map<std::string, std::string> crosswalk_id_polyline_map_;
  std::unordered_map<std::string, std::string> junction_id_polyline_map_;
  std::unordered_map<std::string, std::string> speed_bump_id_polyline_map_;
  std::unordered_map<std::string, std::vector<std::string>>
      road_bound_id_polyline_map_;
  std::unordered_map<std::string, StopLinePolyline> stop_line_polyline_map_;
  std::unordered_map<std::string, std::string> stop_line_id_polyline_map_;
};

using VectorMapPtr = std::shared_ptr<VectorMap>;
using VectorMapConstPtr = std::shared_ptr<const VectorMap>;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_VECTOR_MAP_H
