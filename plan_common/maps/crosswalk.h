

#ifndef AD_BYD_PLANNING_MAP_CROSSWALK_H
#define AD_BYD_PLANNING_MAP_CROSSWALK_H
#include <memory>

#include "plan_common/type_def.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/math/polygon2d.h"

namespace ad_byd {
namespace planning {
class Crosswalk {
 public:
  Crosswalk() = default;
  Crosswalk(const CrossWalkInfo& cross_walk_info);

  const uint64_t id() const { return id_; }
  const std::vector<Point2d>& points() const { return points_; }
  const bool IsValid() const { return is_polygon_convex_; }

  double DistanceTo(const Point2d& point) const;
  const math::Polygon2d& polygon() const { return polygon_; };

  void add_intersected_lanes(const std::string& lane_id) {
    intersected_lanes_.insert(lane_id);
  }

  st::Segment2d bone_axis_smooth() const { return bone_axis_smooth_; }

 protected:
  std::vector<Point2d> points_;
  math::Polygon2d polygon_;  // convex polygon
  uint64_t id_;
  bool is_polygon_convex_ = false;
  std::set<std::string> intersected_lanes_;
  st::Segment2d bone_axis_smooth_;

  template <typename Archive>
  friend void serialize(Archive& ar, Crosswalk& junction);
};

using CrosswalkPtr = std::shared_ptr<Crosswalk>;
using CrosswalkConstPtr = std::shared_ptr<const Crosswalk>;

}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MAP_CROSSWALK_H