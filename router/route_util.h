

#ifndef ST_PLANNING_ROUTER_ROUTE_UTIL
#define ST_PLANNING_ROUTER_ROUTE_UTIL

#include <algorithm>  // IWYU pragma: keep
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/log.h"

//#include "drive_mission.pb.h"

#include "plan_common/maps/composite_lane_path.h"
//#include "global/logging.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_path_util.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/hmi_content.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/route_manager_output.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
//#include "route_params.pb.h"
#include "plan_common/maps/route_navi_info.h"
#include "router/route_manager_output.h"
#include "plan_common/maps/route_sections.h"
#include "plan_common/maps/route_sections_info.h"
//#include "router/util/map_index.h"
//#include "semantic_map.pb.h"

namespace st {
namespace planning {
// namespace internal {
// Convert different types of routing destination to global point
// representation.
// absl::StatusOr<mapping::GeoPointProto> ConvertToGlobal(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const RoutingDestinationProto& dest);
// }  // namespace internal

// TODO: Should move to semantic map util.
// TODO: Eliminate later, only used in
// offboard/simulation/metric_evaluator/online_lane_offset_evaluator.cc.
// absl::StatusOr<mapping::LanePoint>
// FindOutgoingLanePointWithMinimumHeadingDiff(
//     const ad_byd::planning::Map& smm, mapping::ElementId id);

// std::pair<mapping::LevelId, std::vector<mapping::PointToLane>>
// PointToNearLanesWithInferLevel(const ad_byd::planning::Map&
// semantic_map_manager,
//                                const route::MapIndex& index,
//                                const Vec2d& global, double z,
//                                double min_distance);

// std::vector<mapping::PointToLane> PointToNearLanesAtLevel(
//     const route::MapIndex& index, mapping::LevelId level, const Vec2d&
//     global, double min_distance);

// std::vector<mapping::PointToLane> PointToNearLanesWithHeadingAtLevel(
//     const route::MapIndex& index, mapping::LevelId level, const Vec2d&
//     global, double max_distance, double heading, double max_heading_diff);

// absl::StatusOr<mapping::LanePoint> ParseDestinationProtoToLanePoint(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const route::MapIndex* map_index, const RoutingDestinationProto& proto,
//     double min_distance = 10.0,
//     RouteMatchType match_type = RouteMatchType::STATIONARY);

// absl::StatusOr<std::vector<mapping::LanePoint>>
// ParseDestinationProtoToLanePoints(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const route::MapIndex* map_index,
//     const RoutingRequestProto& routing_request);

// absl::StatusOr<RoutingRequestProto> ConvertRoutingRequestToGlobalPoint(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const RoutingRequestProto& raw_routing_request);

// TODO: Eliminate later, Only used in
// offboard/mapping/post_processing/lane_boundary_processor.cc.
// mapping::LanePath BackwardExtendLanePath(
// const ad_byd::planning::Map& smm, const mapping::LanePath& raw_lane_path,
// double extend_len,
// const std::function<bool(const ad_byd::planning::Lane&)>*
//     nullable_should_stop_and_avoid_extend = nullptr);

// absl::StatusOr<RoutingRequestProto>
// ConvertAllRouteFlagsToRoutingRequestProto();

// absl::flat_hash_set<mapping::ElementId> FindAvoidLanesFromAvoidRegions(
//     const RoutingRequestProto& routing_request,
//     const ad_byd::planning::Map&  semantic_map_manager);

// absl::StatusOr<std::vector<Vec2d>> SimplifyPathPoints(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const CompositeLanePathProto& composite_lane_path,
//     double max_distance_meters);

// bool HasOnlyOneOffRoadRequest(const RoutingRequestProto& routing_request);

// inline bool IsOffRoadDestination(
//     const MultipleStopsRequestProto::StopProto& next_stop) {
//   return next_stop.stop_point().has_off_road();
// }

// inline bool IsValidRouteOutputProto(
//     const RouteManagerOutputProto& route_out_proto) {
//   if (route_out_proto.has_is_valid()) {
//     return route_out_proto.is_valid();
//   }
//   if (route_out_proto.has_destination_stop() &&
//       IsOffRoadDestination(route_out_proto.destination_stop()))
//     return true;
//   return route_out_proto.has_route_sections_from_current() ||
//          route_out_proto.has_route_from_current();
// }

inline bool IsValidRouteOutput(const RouteManagerOutput& route_mgr_output) {
  return !route_mgr_output.route_sections_from_current.empty() &&
         !route_mgr_output.route_navi_info.route_lane_info_map.empty() &&
         route_mgr_output.is_valid;
}

bool IsTravelMatchedValid(const CompositeLanePath::CompositeIndex& first,
                          double first_fraction,
                          const CompositeLanePath::CompositeIndex& second,
                          double second_fraction, int64_t last_time_micros,
                          int64_t cur_time_micros, double distance,
                          double speed, double reroute_threshold_meters,
                          double estimate_speed);

// bool CrossSolidBoundary(mapping::LaneBoundaryProto::Type type, bool lc_left);

// template <typename CompositeLanePathType,
//           std::enable_if_t<
//               std::is_same_v<CompositeLanePathType, CompositeLanePath> ||
//                   std::is_same_v<CompositeLanePathType,
//                   CompositeLanePathProto>,
//               bool> = true>
// double TravelDistanceBetween(const ad_byd::planning::Map&
// semantic_map_manager,
//                              const CompositeLanePathType&
//                              composite_lane_path, const
//                              CompositeLanePath::CompositeIndex& first, double
//                              first_fraction, const
//                              CompositeLanePath::CompositeIndex& second,
//                              double second_fraction) {
//   VLOG(3) << composite_lane_path.DebugString()
//           << " first:" << first.DebugString()
//           << "first fraction:" << first_fraction
//           << " second fraction:" << second_fraction
//           << ", second:" << second.DebugString();

//   const auto length_fn = [&semantic_map_manager, &composite_lane_path](
//                              int lane_path_idx, int first_lane_idx,
//                              int last_lane_idx, double first_frac,
//                              double last_frac) {
//     double accum_len = 0.0;
//     const auto& cur_lane_path =
//     composite_lane_path.lane_paths()[lane_path_idx]; for (int i =
//     first_lane_idx; i <= last_lane_idx; ++i) {
//       const double start_frac = i == first_lane_idx ? first_frac : 0.0;
//       const double end_frac = i == last_lane_idx ? last_frac : 1.0;
//       SMM_ASSIGN_LANE_OR_CONTINUE(
//           cur_lane_info, semantic_map_manager,
//           mapping::ElementId(cur_lane_path.lane_ids()[i]));
//       accum_len += cur_lane_info.length() * (end_frac - start_frac);
//     }
//     return accum_len;
//   };

//   // Reversing is allowed here, therefore the distance could be negative.
//   const auto& first_lane_path =
//       composite_lane_path.lane_paths()[first.lane_path_index];
//   const auto& last_lane_path =
//       composite_lane_path.lane_paths()[second.lane_path_index];

//   double travel_distance = 0.0;
//   if (first.lane_path_index == second.lane_path_index) {
//     travel_distance +=
//         length_fn(first.lane_path_index, first.lane_index, second.lane_index,
//                   first_fraction, second_fraction);
//   } else {
//     // first lane path
//     travel_distance +=
//         length_fn(first.lane_path_index, first.lane_index,
//                   std::max(0, first_lane_path.lane_ids_size() - 1),
//                   first_fraction, first_lane_path.end_fraction());
//     // last lane path
//     travel_distance +=
//         length_fn(second.lane_path_index, 0, second.lane_index,
//                   last_lane_path.start_fraction(), second_fraction);
//     // all mid lane path
//     for (int i = first.lane_path_index + 1; i < second.lane_path_index; i++)
//     {
//       const auto& lane_path = composite_lane_path.lane_paths()[i];
//       travel_distance +=
//           mapping::GetLanePathLength(semantic_map_manager, lane_path);
//     }
//   }
//   return travel_distance;
// }

// std::vector<mapping::LanePathProto> FindAvailableSimplePaths(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     absl::Span<const mapping::PointToLane> origins,
//     absl::Span<const mapping::PointToLane> dests);

Vec3d ComputePointOnLane(const std::vector<Vec2d>& geo_points, double fraction);

// RouteParamProto CreateDefaultRouteParam();

// double AccumRouteSectionsLengthBetween(
//     const ad_byd::planning::Map& smm, const RouteSectionSequenceProto&
//     sections, int start_idx, int end_idx);

// template <typename CompositeLanePathType,
//           std::enable_if_t<
//               std::is_same_v<CompositeLanePathType, CompositeLanePath> ||
//                   std::is_same_v<CompositeLanePathType,
//                   CompositeLanePathProto>,
//               bool> = true>
// double ComputeEtaToRouteEnd(const ad_byd::planning::Map&
// semantic_map_manager,
//                             const CompositeLanePathType& composite_lane_path)
//                             {
//   double cum_time_secs = 0.0;
//   for (const auto& lane_path : composite_lane_path.lane_paths()) {
//     for (int i = 0; i < lane_path.lane_ids_size(); ++i) {
//       SMM_ASSIGN_LANE_OR_BREAK(lane_info, semantic_map_manager,
//                                mapping::ElementId(lane_path.lane_ids()[i]));
//       const double start_frac = i == 0 ? lane_path.start_fraction() : 0.0;
//       const double end_frac =
//           i + 1 == lane_path.lane_ids_size() ? lane_path.end_fraction()
//           : 1.0;
//       cum_time_secs +=
//           lane_info.length() * (end_frac - start_frac) /
//           lane_info.speed_limit;
//     }
//   }
//   return cum_time_secs;
// }

// TODO: Adapt to smm v2.
// absl::StatusOr<RouteSectionsInfo> BuildRouteSectionsInfoFromData(
//     const ad_byd::planning::Map& smm, const RouteSections* route_sections,
//     double planning_horizon);

// TODO: Delete when router replaces lane info with lane proto totally.
inline std::vector<mapping::ElementId> ConvertToElementIdFrom(
    const ::google::protobuf::RepeatedField<uint64_t>& lane_ids) {
  std::vector<mapping::ElementId> res;
  res.reserve(lane_ids.size());
  std::transform(
      lane_ids.begin(), lane_ids.end(), std::back_inserter(res),
      [](const uint64_t lane_id) { return mapping::ElementId(lane_id); });
  return res;
}

}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_ROUTER_ROUTE_UTIL
