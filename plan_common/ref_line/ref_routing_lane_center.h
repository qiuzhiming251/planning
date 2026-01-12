
#ifndef AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REF_ROUTING_LANE_CENTER_H_
#define AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REF_ROUTING_LANE_CENTER_H_

#include "plan_common/type_def.h"
#include "plan_common/maps/map.h"
#include "plan_common/maps/route.h"
#include "plan_common/math/box2d.h"
#include "plan_common/math/curve_limits.h"
#include "plan_common/math/vec2d.h"

namespace ad_byd {
namespace planning {

using LineSegment2d = math::LineSegment2d;
using Vec2d = math::Vec2d;
using MapConstPtr = std::shared_ptr<const Map>;

class RefRoutingLaneCenter {
 public:
  RefRoutingLaneCenter() = default;
  RefRoutingLaneCenter(const Vec2d& pos, MapConstPtr map, RoutePtr route);
  ~RefRoutingLaneCenter() = default;

  double adc_l() const { return adc_l_; }
  double adc_s() const { return adc_s_; }
  bool GetProjection(const Vec2d& point, double* const s,
                     double* const l) const;
  double GetHeadingFromS(const double& s) const;
  bool XYFromS(const double& s, Vec2d* const pos) const;
  void Init(const Vec2d& pos, MapConstPtr map, RoutePtr route);
  void Clear();
  bool GetLanePtrFromS(const double& s, LaneConstPtr& lane_ptr) const;
  bool IsStraight() const;
  const std::vector<LaneConstPtr>& route_lane() const;

 protected:
  math::LineCurve2d center_line_;
  double adc_s_ = 0.0;
  double adc_l_ = 0.0;
  Vec2d ego_position_;
  std::vector<LaneConstPtr> route_lanes_;
  std::vector<double> acc_length_;
  bool is_straight_flag_ = true;
  const double m_pi_3_ = M_PI / 3.0;
};

using RefRoutingLaneCenterPtr = std::shared_ptr<RefRoutingLaneCenter>;

}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REF_ROUTING_LANE_CENTER_H_