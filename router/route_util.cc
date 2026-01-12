

#include "router/route_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <ostream>
#include <random>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "absl/types/span.h"
#ifndef BOOST_ALLOW_DEPRECATED_HEADERS
#define BOOST_ALLOW_DEPRECATED_HEADERS
#include "boost/geometry/algorithms/simplify.hpp"
#undef BOOST_ALLOW_DEPRECATED_HEADERS
#else
#include "boost/geometry/algorithms/simplify.hpp"
#endif
#include "boost/geometry/core/cs.hpp"
#include "boost/geometry/geometries/register/linestring.hpp"
#include "boost/geometry/geometries/register/point.hpp"
#include "plan_common/container/strong_int.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"
//#include "map_geometry.pb.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_util.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
//#include "q_issue.pb.h"
//#include "router/geometry/gfc.h"
//#include "router/router_flags.h"
//#include "router/util/route_extend_path.h"
#include "plan_common/util/file_util.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/proto_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/time_util.h"

BOOST_GEOMETRY_REGISTER_POINT_2D(
    st::Vec2d, double,
    ::boost::geometry::cs::cartesian, operator[](0), operator[](1));
BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<st::Vec2d>);

namespace st {
namespace planning {

namespace {

constexpr double kMaxRetrogradeMeters = -1.0;
constexpr double kMinForwardSpeed = 0.5;
constexpr double kLcOutermostDist = 800;    // m.
constexpr double kMinIgnoreRampDist = 350;  // m.

}  // namespace

namespace internal {
// Convert different types of routing destination to global point
// representation.
// absl::StatusOr<mapping::GeoPointProto> ConvertToGlobal(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const RoutingDestinationProto& dest) {
//   mapping::GeoPointProto global_point;
//   if (dest.has_global_point()) {
//     global_point = dest.global_point();
//   } else if (dest.has_lane_point()) {
//     const auto lane_pt = mapping::LanePoint(dest.lane_point());
//     SMM_LANE_PROTO_OR_RETURN(lane_proto, semantic_map_manager,
//                              lane_pt.lane_id(),
//                              absl::NotFoundError(absl::StrCat(
//                                  "Cannot find lane:", lane_pt.lane_id())));
//     const auto pt3d =
//         ComputePointOnLane(lane_proto->polyline().points(),
//         lane_pt.fraction());
//     global_point.set_longitude(pt3d.x());
//     global_point.set_latitude(pt3d.y());
//     global_point.set_altitude(pt3d.z());
//   } else {
//     const mapping::NamedSpotProto* named_spot = nullptr;
//     for (const auto& spot :
//     semantic_map_manager.semantic_map().named_spots()) {
//       if (spot.name() == dest.named_spot()) {
//         named_spot = &spot;
//         break;
//       }
//     }
//     if (named_spot == nullptr) {
//       return absl::InvalidArgumentError(
//           absl::StrCat("Could not find the named spot ", dest.named_spot(),
//                        ", please check your semantic map version."));
//     }
//     global_point = named_spot->point();
//   }
//   return global_point;
// }
}  // namespace internal

// absl::StatusOr<mapping::LanePoint>
// FindOutgoingLanePointWithMinimumHeadingDiff(
//     const ad_byd::planning::Map& smm, mapping::ElementId id) {
//   SMM_ASSIGN_LANE_OR_ERROR(lane_info, smm, id);
//   if (lane_info.next_lane_ids().empty()) {
//     return absl::NotFoundError(absl::StrCat(
//         "Can not find outgoing lane. Lane id:", lane_info.id(), "."));
//   }

//   if (lane_info.next_lane_ids().size() == 1) {
//     return mapping::LanePoint(smm.lanes().front()->id(), 0.0);
//   }

//   constexpr double kSampleLen = 4.0;  // m.
//   const Vec2d origin_pt = lane_info.points().back();
//   const Vec2d prev_pt = lane_info.LerpPointFromFraction(
//       std::max(0.0, 1.0 - kSampleLen / lane_info.curve_length()));
//   const Vec2d heading = (origin_pt - prev_pt).normalized();

//   struct OutgoingLaneData {
//     double project_value;
//     bool is_virtual;
//   };

//   std::vector<OutgoingLaneData> outgoing_lane_data;
//   outgoing_lane_data.reserve(lane_info.next_lane_ids().size());

//   // Calculate proj value for each outgoing lane.
//   for (int i = 0; i < lane_info.next_lane_ids().size(); ++i) {
//     const auto temp_id = lane_info.next_lane_ids()[i];
//     const auto& temp_lane_info = smm.GetLaneById(temp_id);
//     const Vec2d next_pt = temp_lane_info.LerpPointFromFraction(
//         std::min(1.0, kSampleLen / temp_lane_info->curve_length()));
//     const Vec2d tmp_heading = (next_pt - origin_pt).normalized();
//     const double proj = heading.Dot(tmp_heading);
//     outgoing_lane_data.push_back(
//         {temp_index, proj, temp_lane_info.IsVirtual()});
//   }

//   // Sort according proj value from large to small.
//   std::stable_sort(outgoing_lane_data.begin(), outgoing_lane_data.end(),
//                    [](const auto& lhs, const auto& rls) {
//                      return lhs.project_value > rls.project_value;
//                    });

//   // Select first non virtual lane as extend lane. If all lanes are
//   // virtual, select first lane from lane_index_set (after sort).
//   auto opt_outgoing_index = outgoing_lane_data.front().lane_index;
//   for (const auto& mgr : outgoing_lane_data) {
//     if (mgr.is_virtual == false) {
//       opt_outgoing_index = mgr.lane_index;
//       break;
//     }
//   }

//   return mapping::LanePoint(smm.lane_info()[opt_outgoing_index].id, 0.0);
// }

// absl::StatusOr<mapping::LanePoint> ParseDestinationProtoToLanePoint(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const route::MapIndex* map_index,
//     const RoutingDestinationProto& destination_proto, double min_distance,
//     RouteMatchType match_type) {
//   CHECK_NOTNULL(map_index);
//   if (destination_proto.has_off_road()) {
//     return mapping::LanePoint{mapping::kInvalidElementId, 0.0};
//   }
//   if (destination_proto.has_global_point() ||
//       destination_proto.has_named_spot()) {
//     Vec3d global;
//     if (!destination_proto.has_global_point() &&
//         destination_proto.has_named_spot()) {
//       const auto& map = semantic_map_manager.semantic_map();
//       const mapping::NamedSpotProto* named_spot = nullptr;
//       for (const auto& spot : map.named_spots()) {
//         if (spot.name() == destination_proto.named_spot()) {
//           named_spot = &spot;
//         }
//       }
//       if (named_spot == nullptr) {
//         return absl::InvalidArgumentError(absl::StrCat(
//             "Could not find the named spot ",
//             destination_proto.named_spot()));
//       }
//       global = {named_spot->point().longitude(),
//       named_spot->point().latitude(),
//                 named_spot->point().altitude()};
//     } else {
//       global = {destination_proto.global_point().longitude(),
//                 destination_proto.global_point().latitude(),
//                 destination_proto.global_point().altitude()};
//     }
//     std::vector<mapping::PointToLane> point_to_lanes;
//     mapping::LevelId level;
//     auto level_lanes_pair = PointToNearLanesWithInferLevel(
//         semantic_map_manager, *map_index, {global.x(), global.y()},
//         global.z(), min_distance);
//     level = level_lanes_pair.first;
//     point_to_lanes = std::move(level_lanes_pair.second);
//     VLOG(3) << "Infer level:" << level
//             << ", lanes count:" << point_to_lanes.size();
//     if (FLAGS_route_match_filter_enable) {
//       for (const mapping::PointToLane& ptlane : point_to_lanes) {
//         if (ptlane.lane_proto == nullptr) {
//           LOG_ERROR << "Invalid PointToNearLanesAtLevel return nullptr.";
//           continue;
//         } else if (match_type == RouteMatchType::STATIONARY &&
//                    mapping::IsPassengerVehicleAvoidLaneType(
//                        ptlane.lane_proto->type())) {
//           LOG_ERROR << "Invalid destination_proto, too close to
//           non-motorway. "
//                         "destination:"
//                      << destination_proto.ShortDebugString()
//                      << ", match lane:" << ptlane.lane_proto->id();
//           continue;
//         } else {
//           return
//           mapping::LanePoint{mapping::ElementId(ptlane.lane_proto->id()),
//                                     ptlane.fraction};
//         }
//       }
//     } else if (point_to_lanes.size() > 0) {
//       return mapping::LanePoint{
//           mapping::ElementId(point_to_lanes.front().lane_proto->id()),
//           point_to_lanes.front().fraction};
//     }
//     return absl::NotFoundError(absl::StrCat(
//         "Could not found lane id from destination proto: ",
//         destination_proto.ShortDebugString(),
//         absl::StrFormat("lonlat:{ %.8f %.8f} ", global.x(), global.y()),
//         ", Level:", level));
//   } else if (destination_proto.has_lane_point()) {
//     const auto lp = mapping::LanePoint(destination_proto.lane_point());
//     if (!lp.Valid()) {
//       return absl::InvalidArgumentError(
//           absl::StrCat("The provided lane point is invalid: ",
//                        destination_proto.ShortDebugString()));
//     } else {
//       return lp;
//     }
//   } else {
//     return absl::InvalidArgumentError(absl::StrCat(
//         "Invalid routing request,", destination_proto.ShortDebugString()));
//   }
// }

// absl::StatusOr<std::vector<mapping::LanePoint>>
// ParseDestinationProtoToLanePoints(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const route::MapIndex* map_index,
//     const RoutingRequestProto& routing_request) {
//   std::vector<mapping::LanePoint> destinations;
//   destinations.reserve(routing_request.destinations_size());
//   for (const RoutingDestinationProto& destination_proto :
//        routing_request.destinations()) {
//     const auto lane_point_or = ParseDestinationProtoToLanePoint(
//         semantic_map_manager, map_index, destination_proto);
//     if (!lane_point_or.ok()) return lane_point_or.status();
//     destinations.push_back(lane_point_or.value());
//   }
//   if (destinations.empty()) {
//     return absl::NotFoundError(
//         absl::StrCat("No route destination found from request: ",
//                      routing_request.ShortDebugString()));
//   }
//   return destinations;
// }

// absl::StatusOr<RoutingRequestProto> ConvertRoutingRequestToGlobalPoint(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const RoutingRequestProto& raw_routing_request) {
//   RoutingRequestProto proto;
//   *proto.mutable_avoid_lanes() = raw_routing_request.avoid_lanes();
//   *proto.mutable_lane_cost_modifiers() =
//       raw_routing_request.lane_cost_modifiers();

//   if (raw_routing_request.has_multi_stops()) {
//     proto.mutable_multi_stops()->set_infinite_loop(
//         raw_routing_request.multi_stops().infinite_loop());

//     for (const auto& raw_stop : raw_routing_request.multi_stops().stops()) {
//       MultipleStopsRequestProto::StopProto stop_global;
//       stop_global.set_stop_name(raw_stop.stop_name());
//       if (raw_stop.has_stop_point()) {
//         ASSIGN_OR_RETURN(auto global_point,
//                          internal::ConvertToGlobal(semantic_map_manager,
//                                                    raw_stop.stop_point()));
//         *stop_global.mutable_stop_point()->mutable_global_point() =
//             std::move(global_point);

//       } else {
//         RoutingDestinationProto tmp_dest;
//         *tmp_dest.mutable_named_spot() = raw_stop.stop_name();
//         ASSIGN_OR_RETURN(
//             auto global_point,
//             internal::ConvertToGlobal(semantic_map_manager, tmp_dest));
//         *stop_global.mutable_stop_point()->mutable_global_point() =
//             std::move(global_point);
//       }

//       for (const auto& via_p : raw_stop.via_points()) {
//         ASSIGN_OR_RETURN(auto global_point, internal::ConvertToGlobal(
//                                                 semantic_map_manager,
//                                                 via_p));
//         *stop_global.add_via_points()->mutable_global_point() =
//             std::move(global_point);
//       }

//       *proto.mutable_multi_stops()->add_stops() = std::move(stop_global);
//     }
//   }

//   for (const auto& dest : raw_routing_request.destinations()) {
//     ASSIGN_OR_RETURN(auto global_point,
//                      internal::ConvertToGlobal(semantic_map_manager, dest));
//     *proto.add_destinations()->mutable_global_point() =
//     std::move(global_point);
//   }

//   return proto;
// }

// mapping::LanePath BackwardExtendLanePath(
//     const ad_byd::planning::Map& smm, const mapping::LanePath& raw_lane_path,
//     double extend_len,
//     const std::function<bool(const ad_byd::planning::Lane&)>*
//         nullable_should_stop_and_avoid_extend) {
//   if (extend_len <= 0.0) {
//     return raw_lane_path;
//   }

//   mapping::LanePoint start_lp = raw_lane_path.front();
//   mapping::LanePath backward_path(&smm, start_lp);
//   while (extend_len > 0.0) {
//     SMM_ASSIGN_LANE_OR_BREAK(lane_info, smm, start_lp.lane_id());
//     if (nullable_should_stop_and_avoid_extend != nullptr &&
//         (*nullable_should_stop_and_avoid_extend)(lane_info)) {
//       break;
//     }
//     if (start_lp.fraction() == 0.0) {
//       if (lane_info.incoming_lane_indices.empty()) break;
//       if (lane_info.incoming_lane_indices.size() == 1) {
//         const auto incoming_lane_idx =
//         lane_info.incoming_lane_indices.front(); start_lp =
//             mapping::LanePoint(smm.lane_info()[incoming_lane_idx].id, 1.0);
//       } else {
//         constexpr double kSampleLen = 4.0;  // m.
//         const Vec2d origin_pt = lane_info.points_smooth.front();
//         const Vec2d next_pt = lane_info.LerpPointFromFraction(
//             std::min(1.0, kSampleLen / lane_info.length()));
//         const Vec2d heading = (next_pt - origin_pt).normalized();

//         double max_projection = std::numeric_limits<double>::lowest();
//         auto opt_incoming_index = lane_info.incoming_lane_indices.front();

//         for (int i = 0; i < lane_info.incoming_lane_indices.size(); ++i) {
//           const auto& tmp_lane_info =
//               smm.lane_info()[lane_info.incoming_lane_indices[i]];
//           const Vec2d prev_pt = tmp_lane_info.LerpPointFromFraction(
//               std::max(0.0, 1.0 - kSampleLen / tmp_lane_info.length()));

//           const Vec2d tmp_heading = (origin_pt - prev_pt).normalized();

//           const double proj = heading.Dot(tmp_heading);

//           if (proj > max_projection) {
//             max_projection = proj;
//             opt_incoming_index = lane_info.incoming_lane_indices[i];
//           }
//         }
//         start_lp =
//             mapping::LanePoint(smm.lane_info()[opt_incoming_index].id, 1.0);
//       }

//     } else {
//       const double len = lane_info.length();
//       if (len * start_lp.fraction() > extend_len) {
//         const double fraction = start_lp.fraction() - extend_len / len;
//         backward_path = mapping::LanePath(&smm, {lane_info.id}, fraction,
//                                           start_lp.fraction())
//                             .Connect(&smm, backward_path);
//         break;
//       } else {
//         extend_len -= len * start_lp.fraction();
//         backward_path =
//             mapping::LanePath(&smm, {lane_info.id}, 0.0, start_lp.fraction())
//                 .Connect(&smm, backward_path);
//         start_lp = mapping::LanePoint(start_lp.lane_id(), 0.0);
//       }
//     }
//   }

//   return backward_path.Connect(&smm, raw_lane_path);
// }

// absl::StatusOr<RoutingRequestProto>
// ConvertAllRouteFlagsToRoutingRequestProto() {
//   RoutingRequestProto routing_request;
//   StringToProtoLogCollector log_collector;

//   if (!FLAGS_route.empty()) {
//     bool from_route_flags =
//         StringToProtoLogDetail(FLAGS_route, &routing_request,
//         &log_collector);
//     if (!from_route_flags) {
//       LOG_ERROR << "Failed to parse routing request, "
//                  << log_collector.GetLastParseError().DebugString();
//       QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
//               QIssueSubType::QIST_PLANNER_ROUTING_ROUTE_RECOVER_FAILED,
//               "Router initialization from FLAGS_route.");
//       return absl::InvalidArgumentError(FLAGS_route);
//     }
//     LOG_INFO << "Create default routing request by " << FLAGS_route;
//     return routing_request;
//   }

//   if (!FLAGS_multi_stops_route.empty()) {
//     MultipleStopsRequestProto multiple_stops_request;
//     bool from_stops_flag = StringToProtoLogDetail(
//         FLAGS_multi_stops_route, &multiple_stops_request, &log_collector);
//     if (!from_stops_flag) {
//       LOG_ERROR << "Failed to parse routing request, "
//                  << log_collector.GetLastParseError().DebugString();
//       QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
//               QIssueSubType::QIST_PLANNER_ROUTING_ROUTE_RECOVER_FAILED,
//               "Router initialization from FLAGS_multi_stops_route.");

//       return absl::InvalidArgumentError(FLAGS_multi_stops_route);
//     }
//     LOG_INFO << "Create default routing request by "
//               << FLAGS_multi_stops_route;
//     *routing_request.mutable_multi_stops() = multiple_stops_request;
//     return routing_request;
//   }

//   if (!FLAGS_route_str.empty()) {
//     LOG_INFO << "Create default routing request by " << FLAGS_route_str;
//     const auto tokens = absl::StrSplit(FLAGS_route_str, ',');
//     std::vector<std::string> named_spots(tokens.begin(), tokens.end());
//     if (FLAGS_random_route) {
//       // Read random seed from /dev/random.
//       std::random_device rd("/dev/random");
//       std::mt19937 g(rd());
//       std::shuffle(named_spots.begin(), named_spots.end(), g);
//     }
//     for (const auto& named_spot : named_spots) {
//       if (named_spot.empty()) continue;
//       routing_request.add_destinations()->set_named_spot(
//           std::string(named_spot));
//     }
//     return routing_request;
//   }

//   return absl::NotFoundError(
//       "Routing request is empty or invalid, please check it again.");
// }

// absl::flat_hash_set<mapping::ElementId> FindAvoidLanesFromAvoidRegions(
//     const RoutingRequestProto& routing_request,
//     const ad_byd::planning::Map&  semantic_map_manager) {
//   if (routing_request.avoid_regions().empty()) return {};

//   absl::flat_hash_set<mapping::ElementId> avoid_lanes_in_regions;

//   std::vector<Polygon2d> polygons;
//   polygons.reserve(routing_request.avoid_regions_size());
//   for (const auto& avoid_region : routing_request.avoid_regions()) {
//     auto points = mapping::ConverterGeoPoints(avoid_region.points());
//     polygons.push_back(Polygon2d(std::move(points)));
//   }
//   for (const auto& lane_info : semantic_map_manager.lane_info()) {
//     for (const auto& polygon : polygons) {
//       bool lane_in_polygon = true;
//       for (const auto& pt : lane_info.proto->polyline().points()) {
//         if (!polygon.IsPointIn({pt.longitude(), pt.latitude()})) {
//           lane_in_polygon = false;
//           break;
//         }
//       }
//       if (lane_in_polygon) {
//         avoid_lanes_in_regions.insert(lane_info.id);
//       }
//     }
//   }
//   return avoid_lanes_in_regions;
// }

// /// @brief Cannot keep self-intersect,max_distance(epsilon,unit=meter), use
// /// boost:geometry doglas-peuker,
// /// @return Global coordinate  points seq (rad)
// /// @see mapping::SampleLanePathPoints
// absl::StatusOr<std::vector<Vec2d>> SimplifyPathPoints(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     const CompositeLanePathProto& composite_lane_path,
//     double max_distance_meters) {
//   if (max_distance_meters < 0.0) {
//     return absl::InvalidArgumentError("distance must GE 0");
//   }
//   std::vector<Vec2d> points;
//   constexpr double kApproximateMeterToRadRatio = 1.11E-5 * M_PI / 180;
//   constexpr double kMinFractionDistance = 0.001;
//   for (const auto& lane_path : composite_lane_path.lane_paths()) {
//     if (lane_path.lane_ids().size() == 1 &&
//         (lane_path.end_fraction() - lane_path.start_fraction()) <
//             kMinFractionDistance) {
//       continue;
//     }
//     ASSIGN_OR_RETURN(
//         const auto sample_result,
//         SampleLanePathProtoPoints(semantic_map_manager, lane_path));
//     if (sample_result.is_partial) {
//       LOG_INFO << "The partial result is not allowed, "
//                 << sample_result.message;
//       return absl::NotFoundError(sample_result.message);
//     }
//     const auto& global_points = sample_result.points;
//     if (global_points.empty()) {
//       continue;
//     }
//     std::vector<Vec2d> simplified;
//     boost::geometry::simplify(
//         global_points, simplified,
//         max_distance_meters * kApproximateMeterToRadRatio);
//     const int simplified_size = simplified.size();
//     points.insert(points.end(), simplified.begin(), simplified.end());
//     VLOG(2) << " Simplify, source points num:" << global_points.size()
//             << ", after:" << simplified_size
//             << ", total points:" << points.size();
//   }
//   return points;
// }

// bool HasOnlyOneOffRoadRequest(const RoutingRequestProto& routing_request) {
//   return (routing_request.destinations_size() == 1 &&
//           routing_request.destinations(0).has_off_road()) ||
//          (routing_request.has_multi_stops() &&
//           routing_request.multi_stops().stops_size() == 1 &&
//           IsOffRoadDestination(routing_request.multi_stops().stops(0)));
// }

bool IsTravelMatchedValid(const CompositeLanePath::CompositeIndex& first,
                          double first_fraction,
                          const CompositeLanePath::CompositeIndex& second,
                          double second_fraction, int64_t last_time_micros,
                          int64_t cur_time_micros, double distance,
                          double speed, double reroute_threshold_meters,
                          double estimate_speed) {
  VLOG(2) << "first" << first.DebugString() << ", second"
          << second.DebugString() << ", first_fraction:" << first_fraction
          << ", second_fraction:" << second_fraction
          << ", last_time_micros:" << last_time_micros
          << ", cur_time_micros:" << cur_time_micros
          << ", current speed:" << speed
          << ", estimate_speed:" << estimate_speed;
  if (last_time_micros == -1) {
    return true;
  }
  // Retrograde is not allowed, forward direction
  if (speed >= kMinForwardSpeed && distance <= kMaxRetrogradeMeters) {
    VLOG(2) << "Retrograde is not allowed.distance:" << distance
            << ", max:" << kMaxRetrogradeMeters << ", speed:" << speed;
    return false;
  }
  constexpr double kAllowedDriftThres = 35.0 * 0.5;  // 35m/s*0.5s
  const double reroute_threshold =
      std::min(reroute_threshold_meters,
               MicroSecondsToSeconds(cur_time_micros - last_time_micros) *
                   estimate_speed);
  VLOG(2) << "RerouteThresholdMeters:" << reroute_threshold_meters
          << ", reroute_threshold:" << reroute_threshold
          << ", drift_allowed: " << kAllowedDriftThres
          << ", distance:" << distance;
  return distance <= std::max(reroute_threshold, kAllowedDriftThres);
}

// bool CrossSolidBoundary(mapping::LaneBoundaryProto::Type type, bool lc_left)
// {
//   switch (type) {
//     case mapping::LaneBoundaryProto::UNKNOWN_TYPE:
//     case mapping::LaneBoundaryProto::BROKEN_WHITE:
//     case mapping::LaneBoundaryProto::BROKEN_YELLOW:
//       return false;
//     case mapping::LaneBoundaryProto::SOLID_WHITE:
//     case mapping::LaneBoundaryProto::SOLID_YELLOW:
//     case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
//     case mapping::LaneBoundaryProto::CURB:
//       return true;
//     case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
//       return lc_left;
//     case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
//       return !lc_left;
//     default:
//       throw std::runtime_error("switch case on enum unexpected");
//   }
// }

// @see onboard/maps/semantic_map_manager.cc:InferLevelIdFromNearbyLanes
// std::pair<mapping::LevelId, std::vector<mapping::PointToLane>>
// PointToNearLanesWithInferLevel(const ad_byd::planning::Map&
// semantic_map_manager,
//                                const route::MapIndex& index,
//                                const Vec2d& global, double z,
//                                double min_distance) {

//   constexpr double kMaxDistance = 15.0;  // ignore data whose dist > 15m

//   // To determine if a point is on the level, the ground altitude of the
//   level
//   // must be lower than altitude + kLevelAllowance
//   constexpr double kLevelZAllowance = 2.0;
//   const double max_altitude = z + kLevelZAllowance;
//   double min_delta_z = std::numeric_limits<double>::max();
//   std::vector<mapping::PointToLane> all_lanes = index.PointToNearLanes(
//       global, kMaxDistance, [](const auto*) { return true; });

//   std::sort(all_lanes.begin(), all_lanes.end(),
//             [](const auto& lhs, const auto& rhs) {
//               return lhs.dist < rhs.dist ||
//                      (lhs.dist == rhs.dist && lhs.fraction < rhs.fraction);
//             });
//   VLOG(3) << "begin inferring level by near lanes.";

//   mapping::LevelId inferred_level = mapping::kMapLevel0;
//   if (all_lanes.size() > 0 &&
//       !all_lanes[0].lane_proto->belonging_levels().empty()) {
//     inferred_level =
//         mapping::LevelId(all_lanes[0].lane_proto->belonging_levels()[0]);
//   }
//   for (const auto& pt_lane : all_lanes) {
//     const auto* lane = pt_lane.lane_proto;
//     if (lane->belonging_levels().empty()) {
//       break;
//     }
//     double nearest_point_alt;
//     double lane_min_dist = std::numeric_limits<double>::max();
//     bool valid_nearest_point_found = false;
//     for (const auto& geo_pt : lane->polyline().points()) {
//       const auto altitude = geo_pt.altitude();
//       if (altitude > max_altitude) {
//         continue;
//       }
//       const auto dist_pt = route::HaversineDistance(
//           {geo_pt.longitude(), geo_pt.latitude()}, global);
//       if (dist_pt > kMaxDistance) {
//         continue;
//       } else if (dist_pt < lane_min_dist) {
//         lane_min_dist = dist_pt;
//         nearest_point_alt = altitude;
//         valid_nearest_point_found = true;
//       }
//     }
//     if (valid_nearest_point_found &&
//         std::fabs(nearest_point_alt - z) < min_delta_z) {
//       min_delta_z = std::fabs(nearest_point_alt - z);
//       inferred_level = mapping::LevelId(lane->belonging_levels()[0]);
//     }
//   }

//   std::vector<mapping::PointToLane> filter_lanes;
//   filter_lanes.reserve(all_lanes.size());
//   for (const auto& pt_lane : all_lanes) {
//     const auto* lane = pt_lane.lane_proto;
//     const auto& levels = lane->belonging_levels();
//     VLOG(3) << "lane_id: " << lane->id() << ",dist:" << pt_lane.dist
//             << ", levels:" << absl::StrJoin(levels, ",");
//     if ((levels.empty() || std::find(levels.begin(), levels.end(),
//                                      inferred_level.value()) != levels.end())
//                                      &&
//         pt_lane.dist <= min_distance) {
//       filter_lanes.push_back(pt_lane);
//     }
//   }
//   VLOG(3) << "point:" << global.DebugStringFullPrecision() << " smooth:"
//           << "z: " << z << ",inferred_level:" << inferred_level
//           << ",lanes:" << filter_lanes.size();
//   return std::make_pair(inferred_level, std::move(filter_lanes));
// }

// std::vector<mapping::PointToLane> PointToNearLanesAtLevel(
//     const route::MapIndex& map_index, mapping::LevelId level,
//     const Vec2d& global, double min_distance) {
//   auto match_lanes = map_index.PointToNearLanes(
//       global, min_distance, [level](const auto* lane_proto) {
//         const auto& levels = lane_proto->belonging_levels();
//         return levels.empty() || std::find(levels.begin(), levels.end(),
//                                            level.value()) != levels.end();
//       });
//   std::sort(match_lanes.begin(), match_lanes.end(),
//             [](const auto& lhs, const auto& rhs) {
//               return lhs.dist < rhs.dist ||
//                      (lhs.dist == rhs.dist && lhs.fraction < rhs.fraction);
//             });
//   return match_lanes;
// }

// Can be optimized into MapIndex match class.
// std::vector<mapping::PointToLane> PointToNearLanesWithHeadingAtLevel(
//     const route::MapIndex& map_index, mapping::LevelId level,
//     const Vec2d& global, double max_distance, double heading,
//     double max_heading_diff) {
//   auto match_lanes = map_index.PointToNearLanes(
//       global, max_distance,
//       [level](const auto* lane_proto) {
//         const auto& levels = lane_proto->belonging_levels();
//         return levels.empty() || std::find(levels.begin(), levels.end(),
//                                            level.value()) != levels.end();
//       },
//       [heading, max_heading_diff](const Vec2d& p1, const Vec2d& p2) {
//         const Segment2d seg(p1, p2);
//         return std::fabs(NormalizeAngle(seg.heading() - heading)) <
//                max_heading_diff;
//       });
//   std::sort(match_lanes.begin(), match_lanes.end(),
//             [](const auto& lhs, const auto& rhs) {
//               return lhs.dist < rhs.dist ||
//                      (lhs.dist == rhs.dist && lhs.fraction < rhs.fraction);
//             });
//   return match_lanes;
// }

// std::vector<mapping::LanePathProto> FindAvailableSimplePaths(
//     const ad_byd::planning::Map&  semantic_map_manager,
//     absl::Span<const mapping::PointToLane> origins,
//     absl::Span<const mapping::PointToLane> dests) {
//   std::vector<mapping::LanePathProto> available_paths;
//   constexpr int kDefaultMaxPathCount = 4;
//   available_paths.reserve(kDefaultMaxPathCount);
//   for (const auto& origin : origins) {
//     for (const auto& dest : dests) {
//       auto simple_path_or = route::FindConnectedLanePathWithin(
//           semantic_map_manager,
//           {mapping::ElementId(origin.lane_proto->id()), origin.fraction},
//           {mapping::ElementId(dest.lane_proto->id()), dest.fraction},
//           /*allow_distance=*/100.0);
//       if (simple_path_or.ok()) {
//         available_paths.push_back(std::move(simple_path_or).value());
//       }
//     }
//   }
//   return available_paths;
// }

Vec3d ComputePointOnLane(const std::vector<Vec2d>& geo_points,
                         double fraction) {
  const auto [pos, t] = mapping::ComputePosition(geo_points, fraction);
  const auto z = Lerp(geo_points[pos].x(), geo_points[pos + 1].y(), t);
  return Vec3d((pos >= geo_points.size() - 1)
                   ? geo_points[geo_points.size() - 1]
                   : Lerp(geo_points[pos], geo_points[pos + 1], t),
               z);
}

// RouteParamProto CreateDefaultRouteParam() {
//   RouteParamProto route_param_proto;
//   file_util::TextFileToProto(FLAGS_route_default_params_file,
//                              &route_param_proto);
//   return route_param_proto;
// }

// double AccumRouteSectionsLengthBetween(
//     const ad_byd::planning::Map& smm, const RouteSectionSequenceProto&
//     sections, int start_idx, int end_idx) {
//   CHECK_GE(start_idx, 0);
//   CHECK_LE(end_idx, sections.section_id_size());
//   double length = 0.0;
//   for (int i = start_idx; i < end_idx; ++i) {
//     const double start_frac = i == 0 ? sections.start_fraction() : 0.0;
//     const double end_frac =
//         i + 1 == sections.section_id_size() ? sections.end_fraction() : 1.0;
//     SMM_SECTION_PROTO_OR_BREAK(sec_proto, smm, sections.section_id()[i]);
//     length += sec_proto->average_length() * (end_frac - start_frac);
//   }
//   return length;
// }

// RoutingResultProto::SuggestNaviAction GenerateSuggestNaviAction(
//     const ad_byd::planning::Map&  smm, const PoseProto& pose,
//     const RoutingResultProto& routing_result_proto,
//     const google::protobuf::RepeatedField<int64_t>& lateral_lane_ids,
//     bool in_ramp) {
//   RoutingResultProto::SuggestNaviAction suggest_action;
//   suggest_action.set_action(RoutingResultProto::SuggestNaviAction::LC_NONE);
//   // Hack.
//   suggest_action.set_confidence(1.0);
//   if (!routing_result_proto.has_current_navi_instruction()) {
//     return suggest_action;
//   }

//   const auto dist_lane_points =
//       FindCloseLanePointsAndDistanceToSmoothPointWithHeadingBoundAmongLanesAtLevel(
//       // NOLINT
//           smm.GetLevel(), smm,
//           Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()), pose.yaw(),
//           /*heading_penalty_weight=*/0.0, /*spatial_distance_threshold=*/2.5,
//           /*angle_error_threshold=*/M_PI_2);
//   if (dist_lane_points.empty()) {
//     return suggest_action;
//   }
//   std::vector<mapping::LanePoint> close_points;
//   close_points.reserve(dist_lane_points.size());
//   for (const auto& dist_lanept : dist_lane_points) {
//     VLOG(3) << "dist:" << dist_lanept.first
//             << ",lane:" << dist_lanept.second.DebugString();
//     if (dist_lanept.first > /*spatial_distance_threshold=*/2.5) {
//       continue;
//     }
//     close_points.push_back(dist_lanept.second);
//   }
//   if (close_points.empty()) {
//     LOG_INFO << "Cannot project to nearest lane.";
//     return suggest_action;
//   }

//   for (const auto& lp : close_points) {
//     VLOG(3) << "Find lane point:" << lp.DebugString();
//   }

//   const auto ego_lane_iter =
//       std::find(lateral_lane_ids.begin(), lateral_lane_ids.end(),
//                 close_points.front().lane_id().value());
//   if (ego_lane_iter == lateral_lane_ids.end()) {
//     LOG_ERROR << "Can not match ego car with lane_ids_at_ego_pos";
//     return suggest_action;
//   }
//   const int ego_idx = ego_lane_iter - lateral_lane_ids.begin();

//   const auto& cur_navi_action =
//   routing_result_proto.current_navi_instruction()
//                                     .navi_action()
//                                     .navi_action_type();
//   const double cur_length =
//       routing_result_proto.current_navi_instruction().distance();

//   const auto* lane_info_ptr =
//       smm.FindLaneByIdOrNull(close_points.front().lane_id());
//   if (lane_info_ptr == nullptr) {
//     LOG_ERROR << "Can not find ego car lane id: "
//                << close_points.front().lane_id();
//     return suggest_action;
//   }

//   if (in_ramp && cur_length > kMinIgnoreRampDist) {
//     return suggest_action;
//   }

//   switch (cur_navi_action) {
//     case NaviActionProto::TURN_LEFT:
//     case NaviActionProto::LEFT_SIDE:
//       if (ego_idx == 0 || (ego_idx == 1 && cur_length > kLcOutermostDist)) {
//         break;
//       }
//       if (lane_info_ptr->lane_neighbors_on_left.empty() ||
//           CrossSolidBoundary(
//               lane_info_ptr->lane_neighbors_on_left.front().lane_boundary_type,
//               /*lc_left=*/true)) {
//         break;
//       }
//       suggest_action.set_action(RoutingResultProto::SuggestNaviAction::LC_LEFT);
//       return suggest_action;
//     case NaviActionProto::TURN_RIGHT:
//     case NaviActionProto::RIGHT_SIDE:
//       if (ego_idx + 1 == lateral_lane_ids.size() ||
//           (ego_idx + 2 == lateral_lane_ids.size() &&
//            cur_length > kLcOutermostDist)) {
//         break;
//       }
//       if (lane_info_ptr->lane_neighbors_on_right.empty() ||
//           CrossSolidBoundary(
//               lane_info_ptr->lane_neighbors_on_right.front().lane_boundary_type,
//               /*lc_left=*/false)) {
//         break;
//       }
//       suggest_action.set_action(
//           RoutingResultProto::SuggestNaviAction::LC_RIGHT);
//       return suggest_action;
//     case NaviActionProto::STRAIGHT:
//     case NaviActionProto::U_TURN:
//     case NaviActionProto::DESTINATION:
//     case NaviActionProto::ROUNDABOUT:
//     case NaviActionProto::UNKNOWN:
//       return suggest_action;
//   }
//   return suggest_action;
// }

// absl::StatusOr<RouteSectionsInfo> BuildRouteSectionsInfoFromData(
//     const ad_byd::planning::Map& smm, const RouteSections* route_sections,
//     double planning_horizon) {
//   if (route_sections == nullptr || route_sections->empty()) {
//     return absl::InvalidArgumentError("Input route sections is empty!");
//   }

//   std::vector<RouteSectionsInfo::RouteSectionSegmentInfo> section_segs;
//   section_segs.reserve(route_sections->size());
//   int i = 0;
//   for (; i < route_sections->size(); ++i) {
//     const auto* section_info =
//         smm.FindSectionInfoOrNull(route_sections->section_ids()[i]);
//     if (section_info == nullptr) {
//       section_segs.emplace_back(RouteSectionsInfo::RouteSectionSegmentInfo{
//           .id = route_sections->section_ids()[i],
//           .start_fraction = 0.0,
//           .end_fraction = 1.0,
//           .average_length = 0.0,
//           .lane_ids = std::vector<mapping::ElementId>()});
//       continue;
//     }
//     section_segs.emplace_back(RouteSectionsInfo::RouteSectionSegmentInfo{
//         .id = route_sections->section_ids()[i],
//         .start_fraction = 0.0,
//         .end_fraction = 1.0,
//         .average_length = section_info->average_length,
//         .lane_ids = section_info->lane_ids});
//   }

//   // Refine first and last section fraction.
//   section_segs.front().start_fraction = route_sections->start_fraction();
//   section_segs.back().end_fraction = route_sections->end_fraction();

//   // Build id_idx_map.
//   for (auto& seg : section_segs) {
//     absl::flat_hash_map<mapping::ElementId, int> id_idx_map;
//     for (int i = 0; i < seg.lane_ids.size(); ++i) {
//       id_idx_map.insert({seg.lane_ids[i], i});
//     }
//     seg.id_idx_map = std::move(id_idx_map);
//   }
//   return RouteSectionsInfo(route_sections, std::move(section_segs),
//                            planning_horizon);
// }

}  // namespace planning
}  // namespace st
