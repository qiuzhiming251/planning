
#ifndef AD_BYD_PLANNING_MAP_STOP_LINE_H
#define AD_BYD_PLANNING_MAP_STOP_LINE_H
#include "plan_common/maps/map_def.h"

namespace ad_byd {
namespace planning {
class StopLine {
 public:
  explicit StopLine(const StopLineInfo& stop_line_info) {
    id_ = stop_line_info.id;
    points_.assign(stop_line_info.points.begin(), stop_line_info.points.end());
    type_ = stop_line_info.type;
    light_type_ = stop_line_info.light_type;
    sub_type_ = stop_line_info.sub_type;
    virtual_type_ = stop_line_info.virtual_type;
  }

  const uint64_t id() const { return id_; }
  const StopLineType type() const { return type_; }
  const std::vector<Point2d>& points() const { return points_; }
  const LightStatus light_type() const { return light_type_; }
  const int8_t sub_type() const { return sub_type_; }
  const int8_t virtual_type() const { return virtual_type_; }

 private:
  uint64_t id_;
  std::vector<Point2d> points_;
  StopLineType type_;
  LightStatus light_type_;
  int8_t sub_type_;
  int8_t virtual_type_;

  template <typename Archive>
  friend void serialize(Archive& ar, StopLine& stopline);
};
using StopLinePtr = std::shared_ptr<StopLine>;
using StopLineConstPtr = std::shared_ptr<const StopLine>;
}  // namespace planning
}  // namespace ad_byd
#endif  // PILOT_PLANNING_MAP_CLEAR_AREA_H_
