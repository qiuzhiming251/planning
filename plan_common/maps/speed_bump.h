
#ifndef AD_BYD_PLANNING_MAP_SPEED_BUMP_H
#define AD_BYD_PLANNING_MAP_SPEED_BUMP_H
#include "plan_common/maps/map_def.h"
#include "plan_common/math/polygon2d.h"

namespace ad_byd {
namespace planning {
class SpeedBump {
 public:
  explicit SpeedBump(const SpeedBumpInfo &stop_line_info);

  const uint64_t id() const { return id_; }
  const std::vector<Point2d> &points() const { return points_; }

  double DistanceTo(const Point2d &point) const;
  const math::Polygon2d &polygon() const { return polygon_; };

  void add_intersected_lanes(const std::string &lane_id) {
    intersected_lanes_.insert(lane_id);
  }

 private:
  uint64_t id_;
  std::vector<Point2d> points_;
  math::Polygon2d polygon_;
  std::set<std::string> intersected_lanes_;

  template <typename Archive>
  friend void serialize(Archive &ar, SpeedBump &speedbump);
};
using SpeedBumpPtr = std::shared_ptr<SpeedBump>;
using SpeedBumpConstPtr = std::shared_ptr<const SpeedBump>;

}  // namespace planning
}  // namespace ad_byd
#endif  // PILOT_PLANNING_MAP_CLEAR_AREA_H_
