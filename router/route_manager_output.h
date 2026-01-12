

#ifndef ST_PLANNING_ROUTER_ROUTE_MANAGER_OUTPUT
#define ST_PLANNING_ROUTER_ROUTE_MANAGER_OUTPUT

#include <optional>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"

//#include "drive_mission.pb.h"

#include "plan_common/maps/semantic_map_defs.h"
#include "modules/cnoa_pnc/planning/proto/route_manager_output.pb.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections.h"

namespace st {
namespace planning {
constexpr int kInvalidRouteUpdateId = -1;

struct RouteManagerOutput {
  void ToProto(RouteManagerOutputProto* proto) const;
  void FromProto(const RouteManagerOutputProto& proto);

  struct AlternateRouteMsg {
    RouteSections route_sections_from_current;
    RouteNaviInfo route_navi_info;
    double roundabout_distance;
  };

  std::optional<AlternateRouteMsg> alter_route_msg = std::nullopt;

  RouteSections route_sections_from_current;
  absl::flat_hash_set<mapping::ElementId> avoid_lanes;

  RouteNaviInfo route_navi_info;

  // RoutingRequestProto routing_request_proto;

  TurnSignal signal = TURN_SIGNAL_NONE;
  // TODO: add signal reason.
  bool rerouted = false;

  std::optional<double> recommend_max_speed_limit = std::nullopt;  // Unit: m/s.

  // MultipleStopsRequestProto::StopProto destination_stop;
  int update_id = kInvalidRouteUpdateId;
  // NOTE: External cmd from driver.
  bool reset = false;
  bool is_valid = true;
};

RouteManagerOutputProto CompatibilityProcessForRouteManagerOutput(
    const RouteManagerOutputProto& rm_out);

}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_ROUTER_ROUTE_MANAGER_OUTPUT
