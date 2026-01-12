
#include "plan_common/maps/junction.h"

namespace ad_byd {
namespace planning {

Junction::Junction(const JunctionInfo& junction_info) {
  id_ = junction_info.id;
  points_.assign(junction_info.points.begin(), junction_info.points.end());
  is_polygon_convex_ = math::Polygon2d::ComputeConvexHull(points_, &polygon_);
}

void Junction::SetEntryLanes(const std::vector<LaneConstPtr>& entry_lanes) {
  entry_lanes_.assign(entry_lanes.begin(), entry_lanes.end());
  has_entry_lanes_ = !entry_lanes_.empty();
}

void Junction::SetExitLanes(const std::vector<LaneConstPtr>& exit_lanes) {
  exit_lanes_.assign(exit_lanes.begin(), exit_lanes.end());
  has_exit_lanes_ = !exit_lanes_.empty();
}

void Junction::PushOverlapLane(const LaneConstPtr& lane_ptr) {
  overlap_lanes_.emplace_back(lane_ptr);
  if (!is_roundabout_ &&
      (lane_ptr != nullptr && lane_ptr->type() == LaneType::LANE_ROUND_ABOUT)) {
    is_roundabout_ = true;
  }
}

void Junction::SetCrosswalks(
    std::unordered_set<CrosswalkConstPtr>&& crosswalks) {
  crosswalks_ = std::move(crosswalks);
}

double Junction::DistanceTo(const Point2d& point) const {
  if (is_polygon_convex_) {
    return polygon_.DistanceTo(point);
  } else {
    return std::numeric_limits<double>::max();
  }
}

}  // namespace planning
}  // namespace ad_byd