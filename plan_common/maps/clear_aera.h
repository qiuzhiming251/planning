
#ifndef AD_BYD_PLANNING_MAP_CLEAR_AERA_H
#define AD_BYD_PLANNING_MAP_CLEAR_AERA_H
#include "plan_common/maps/map_def.h"
#include "plan_common/math/polygon2d.h"

namespace ad_byd {
namespace planning {
class ClearArea {
 public:
  explicit ClearArea(const ClearAreaInfo &clear_area_info) {
    clear_area_info_ = clear_area_info;
    points_ = clear_area_info.points;
    polygon_.SetPoints(clear_area_info.points);
  }

  const std::vector<math::Vec2d> &points() const { return points_; }
  const uint64_t id() const { return clear_area_info_.id; }
  const ImpassableAeraType type() const { return clear_area_info_.type; }
  const math::Polygon2d &polygon() const { return polygon_; }

 private:
  math::Polygon2d polygon_;
  std::vector<math::Vec2d> points_;
  ClearAreaInfo clear_area_info_;

  template <typename Archive>
  friend void serialize(Archive &ar, ClearArea &clear_area);
};
typedef std::shared_ptr<ClearArea> ClearAreaPtr;
typedef std::shared_ptr<const ClearArea> ClearAreaConstPtr;
}  // namespace planning
}  // namespace ad_byd
#endif  // PILOT_PLANNING_MAP_CLEAR_AREA_H_
