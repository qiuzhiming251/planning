

#ifndef AD_BYD_PLANNING_MAP_ROUTE_H
#define AD_BYD_PLANNING_MAP_ROUTE_H
#include <map>
#include <unordered_map>
#include <utility>

#include "plan_common/maps/lane.h"
#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/map_def.h"

namespace ad_byd {
namespace planning {
class Map;
class Route {
 public:
  Route(const RouteInfo &route_info, const Map *map);
  ~Route() {
    if (map_) map_ = nullptr;
  }

  const RouteInfo &GetRouteInfo() const { return route_info_; }
  RouteInfo *mutable_route_info() { return &route_info_; }

  /// @brief check if a lane sequence can drive to navi end section
  /// @param lane_seq lane sequence to be checked
  /// @return true if lane sequence connect to navi end
  bool CanDriveToRouteEnd(const LaneSequencePtr &lane_seq,
                          const LaneConstPtr &start_lane) const;

  /// @brief return distance from input point to the front nearest special
  /// road type event
  /// @param x_road_type the input x_road_type
  /// @param x the input x
  /// @param y the input y
  /// @return distance to nearest none_odd event
  double GetDistanceToX(const XRoadType &x_road_type, const double &x,
                        const double &y) const;
  /// @brief return if get the distance from input point to navi end
  /// @return distance to navi end
  double GetDistanceToNaviEnd() const;

  double GetDistToJunctionEndOnNavi() const;

  const NaviPosition &navi_start() const { return route_info_.navi_start; }
  const NaviPosition &navi_end() const { return route_info_.navi_end; }
  const std::vector<SectionInfo> &sections() const {
    return route_info_.sections;
  }

  const std::vector<PathExtend> &extend_sections_vec() const {
    return route_info_.extend_sections_vec;
  }
  SerialJunctionType NextTwoJunctionOnNavi(double &dis_between_two_junction,
                                           TurnType &first_junction_turn_type,
                                           uint64_t &section_id);

 private:
  RouteInfo route_info_;
  const Map *map_ = nullptr;
  // std::map<uint64_t, std::vector<LaneConstPtr>> pre_nextvr_lanes_;

  template <typename Archive>
  friend void serialize(Archive &ar, Route &route);
};
typedef std::shared_ptr<Route> RoutePtr;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_ROUTE_H
