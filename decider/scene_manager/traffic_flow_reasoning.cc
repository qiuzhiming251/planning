

#include "decider/scene_manager/traffic_flow_reasoning.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "plan_common/async/parallel_for.h"
//#include "global/logging.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/box2d.pb.h"
//#include "perception/lidar_pipeline/sensor_fov/sensor_fov.h"
//#include "planner/planner_manager/planner_flags.h"
//#include "predictor/prediction_util.h"
#include "plan_common/log_data.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
//#include "semantic_map.pb.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/perception_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/util/spatial_search_util.h"

namespace st::planning {
namespace {
// Threshold value to ignore OT_UNKNOWN_STATIC object, which is far from AV.
constexpr double kIgnoreUnknownStaticDist = 100.0;  // m.
// Threshold value to check traffic chain is continuous.
constexpr double kMaxLonDistBetweenObjects = 10.0;  // m.
// Min threshold value to check object is waiting traffic light.
constexpr double kTrafficLightDistMinThreshold = 10.0;  // m.
// Max threshold value to check object is waiting traffic light.
double kTrafficLightDistMaxThreshold = 80.0;  // m
// Default stop time threshold for checking stall objects.
constexpr double kDefaultStopTimeThreshold = 10.0;  // s.
// Threshold value to check special zone effective.
constexpr double kSpecialZoneDistThreshold = 30.0;  // m
// Default stall object probability threshold.
constexpr double kStalledProbThreshold = 0.6;  // ratio.
// If the tl state is TL_RED , front object less likely to be stall.
constexpr double kDefaultRedTrafficLightMultiplier = 1.5;  // ratio.
constexpr double kEpsilon = 0.1;
constexpr double kMaxTrafficWaitingDistance = 300.0;  // m

// MIN traffic light control_zone
constexpr double kMinTrafficWaitingDistance = 20.0;  // m

constexpr double kMinMovingSpeedThes = 0.6;

// The more heading diff is, the less factor is.
const PiecewiseLinearFunction kCalculateFactorAccordHeadingDiffPlf(
    std::vector<double>{M_PI / 6.0, M_PI / 2.0}, std::vector<double>{1.0, 0.3});

using LanePath = mapping::LanePath;
using ObjectsOnLane =
    std::vector<std::pair<double, const ObjectPredictionProto*>>;
// using SensorFov = sensor_fov::SensorFov;
// using SensorFovRef = sensor_fov::SensorFovRef;

struct AnalyzeOnLaneOutput {
  TrafficWaitingQueueProto traffic_waiting_queue;
  std::vector<ObjectAnnotationProto> object_annotations;
  std::optional<double> distance_to_roadblock = std::nullopt;
};

double HalfLength(const Box2dProto& box_proto) {
  return box_proto.length() * 0.5;
}

double GetObjectFrontS(double object_s, const ObjectProto& object) {
  return object_s + HalfLength(object.bounding_box());
}

double GetObjectBackS(double object_s, const ObjectProto& object) {
  return object_s - HalfLength(object.bounding_box());
}

bool IsStationary(const ObjectPredictionProto& object_pred) {
  // return object_pred.trajectories_size() == 0;
  //  ||
  // return object_pred.trajectories().size() >= 1 &&
  //        object_pred.trajectories()[0].type() ==
  //        PredictionType::PT_STATIONARY;
  const auto object = object_pred.perception_object();
  double speed = std::sqrt(object.vel().x() * object.vel().x() +
                           object.vel().y() * object.vel().y());
  return speed < kMinMovingSpeedThes ||
         (object_pred.trajectories().size() >= 1 &&
          object_pred.trajectories()[0].type() ==
              PredictionType::PT_STATIONARY);
}

// TODO: Lane path maybe too short to get traffic light info, extend lane
// path to sufficient length.
// Return all traffic light stop line distance which in front of AV.
std::vector<double> CollectTrafficLightStopLine(
    const PlannerSemanticMapManager& psmm, const LanePath& lane_path) {
  std::vector<double> tl_stop_lines;
  std::string debug_tls_lane;
  for (auto it = lane_path.begin(); it != lane_path.end(); ++it) {
    const auto& lane_info_ptr = psmm.FindCurveLaneByIdOrNull((*it).lane_id);
    if (lane_info_ptr == nullptr) continue;
    // const auto& lane_proto = *CHECK_NOTNULL(lane_info_ptr->proto);
    // if (!lane_proto.startpoint_associated_traffic_lights().empty()) {
    //   const double stop_line_s =
    //       lane_path.LaneIndexPointToArclength({it.lane_index(), 0.0});
    //   tl_stop_lines.push_back(stop_line_s);
    // }

    // for (const auto& multi_tl_control_point :
    //      lane_proto.multi_traffic_light_control_points()) {
    //   const double stop_line_s = lane_path.LaneIndexPointToArclength(
    //       {it.lane_index(), multi_tl_control_point.lane_fraction()});
    //   tl_stop_lines.push_back(stop_line_s);
    // }

    // Extract next lane stop line info of end lane in lane path.
    if (lane_info_ptr == nullptr) continue;
    if (!lane_info_ptr->next_lane_ids().empty()) {
      for (const auto out_lane_id : lane_info_ptr->next_lane_ids()) {
        SMM_ASSIGN_LANE_OR_CONTINUE(out_lane_info, psmm, out_lane_id);
        // has light or pre has light
        if (out_lane_info.IsValid() && out_lane_info.junction_id() != 0 &&
            (out_lane_info.light_status() !=
                 ad_byd::planning::LightStatus::NONE_LIGHT ||
             out_lane_info.pre_has_light())) {
          if (out_lane_info.turn_type() ==
                  ad_byd::planning::TurnType::RIGHT_TURN &&
              out_lane_info.light_status() ==
                  ad_byd::planning::LightStatus::GREEN_LIGHT &&
              out_lane_info.traffic_set_reason() ==
                  ad_byd::planning::TrafficSetReason::SET_DEFAULT_OBJ) {
            continue;
          }
          debug_tls_lane +=
              absl::StrCat(" junction id: ", out_lane_info.junction_id());
          debug_tls_lane +=
              absl::StrCat(" light status: ",
                           static_cast<int>(out_lane_info.light_status()));
          debug_tls_lane +=
              absl::StrCat(" junction lane id: ", lane_info_ptr->id());
          tl_stop_lines.push_back((*it).end_s);
          break;
        }
      }
    }
  }
  Log2DDS::LogDataV0("debug_tls_lane", debug_tls_lane);
  // Delete duplicate traffic light stop lines.
  std::stable_sort(tl_stop_lines.begin(), tl_stop_lines.end());
  auto iter = std::unique(
      tl_stop_lines.begin(), tl_stop_lines.end(),
      [](double lsh, double rsh) { return std::fabs(lsh - rsh) < kEpsilon; });
  tl_stop_lines.erase(iter, tl_stop_lines.end());
  return tl_stop_lines;
}

std::vector<std::pair<double, double>> CollectSpecialZonesInSegment(
    const std::vector<std::pair<mapping::ElementId, Vec2d>>& zones_info,
    const LanePath::LaneSegment& seg, double lane_length) {
  std::vector<std::pair<double, double>> zones;
  zones.reserve(zones_info.size());
  for (const auto& [_, frac] : zones_info) {
    if (seg.start_fraction <= frac[1] && frac[1] <= seg.end_fraction) {
      const double start_s =
          seg.start_s + lane_length * (frac[0] - seg.start_fraction);
      const double end_s =
          seg.start_s + lane_length * (frac[1] - seg.start_fraction);
      zones.push_back({start_s, end_s});
    }
  }
  return zones;
}

std::vector<double> CollectSpecialZonesStartSOnLanePath(
    const PlannerSemanticMapManager& psmm, const LanePath& lane_path) {
  std::vector<double> arc_lens;
  auto collect_zone_start_s =
      [&arc_lens](const std::vector<std::pair<double, double>>& zones,
                  double* prev_end_s) {
        for (const auto& [start_s, end_s] : zones) {
          if (std::fabs(start_s - *prev_end_s) > kEpsilon) {
            arc_lens.push_back(start_s);
          }
          *prev_end_s = end_s;
        }
      };

  double prev_crosswalk_zone_end_s = -std::numeric_limits<double>::infinity();
  double prev_intersection_zone_end_s =
      -std::numeric_limits<double>::infinity();

  for (const auto& seg : lane_path) {
    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, seg.lane_id);
    // Collect crosswalk zones and collect start s.
    // collect_zone_start_s(CollectSpecialZonesInSegment(lane_info.Crosswalks(),
    //                                                   seg,
    //                                                   lane_info.topo_length()),
    //                      &prev_crosswalk_zone_end_s);

    // Collect intersection zones and collect start s.
    std::vector<std::pair<mapping::ElementId, Vec2d>> filtered_intersections;
    std::vector<uint64_t> junction_ids = {lane_info.junction_id()};
    for (const auto& intersection : junction_ids) {
      const auto intersection_info_ptr =
          psmm.FindJunctionByIdOrNull(intersection);
      if (intersection_info_ptr == nullptr) continue;

      const bool traffic_light_controlled_intersection =
          intersection_info_ptr->traffic_light_controlled();
      if (!FLAGS_planner_enable_un_tl_controlled_intersection_reasoning &&
          !traffic_light_controlled_intersection) {
        continue;
      }
      filtered_intersections.push_back(
          std::make_pair(intersection, Vec2d(0.0, 1.0)));
    }
    collect_zone_start_s(
        CollectSpecialZonesInSegment(filtered_intersections, seg,
                                     lane_info.curve_length()),
        &prev_intersection_zone_end_s);
  }
  std::stable_sort(arc_lens.begin(), arc_lens.end());
  return arc_lens;
}

std::vector<std::pair<double, double>> CollectRightTurnZone(
    const PlannerSemanticMapManager& psmm, const LanePath& lane_path) {
  std::vector<std::pair<double, double>> right_turn_zones;
  for (auto it = lane_path.begin(); it != lane_path.end(); ++it) {
    const auto& lane_info_ptr = psmm.FindCurveLaneByIdOrNull((*it).lane_id);
    if (lane_info_ptr == nullptr) continue;
    if (lane_info_ptr->turn_type() == ad_byd::planning::TurnType::RIGHT_TURN) {
      right_turn_zones.push_back({(*it).start_s, (*it).end_s});
    }
  }
  return right_turn_zones;
}

double CalculateFactorForRightTurn(
    const std::vector<std::pair<double, double>>& right_turn_s,
    double object_back_s) {
  const double kRightTurnFactor = 0.5;
  const double kRightTurnZoneS = 50.0;
  double right_turn_factor = 1.0;
  for (const auto& right_turn : right_turn_s) {
    if (object_back_s > right_turn.first && object_back_s < right_turn.second) {
      return kRightTurnFactor;
    } else if (object_back_s > right_turn.second &&
               object_back_s < right_turn.second + kRightTurnZoneS) {
      double decay_factor = Lerp(kRightTurnFactor, 0.0, 1.0, kRightTurnZoneS,
                                 object_back_s - right_turn.second, true);
      right_turn_factor = std::min(right_turn_factor, decay_factor);
    }
  }
  return right_turn_factor;
}

std::optional<double> CalcSOnLanePathByPose(
    const PlannerSemanticMapManager& psmm, const LanePath& lane_path,
    const Vec2d& ego_pos, double lateral_error_buffer = 2.0) {
  double arc_len;
  if (IsPointOnLanePathAtLevel(psmm, ego_pos, lane_path, &arc_len,
                               lateral_error_buffer)) {
    return arc_len;
  }

  return std::nullopt;
}

bool IsObjectOnLanePathByPose(const PlannerSemanticMapManager& psmm,
                              const LanePath& lane_path,
                              const ObjectProto& object, double* arc_len) {
  if (lane_path.IsEmpty()) return false;

  // Check whether object pose on the lane path.
  const bool is_on_lane = IsPointOnLanePathAtLevel(
      psmm, Vec2dFromProto(object.pos()), lane_path, arc_len);

  // // Ignore far unknown static object.
  // if (object.type() == ObjectType::OT_UNKNOWN_STATIC &&
  //     *arc_len >= kIgnoreUnknownStaticDist) {
  //   return false;
  // }

  return is_on_lane;
}

// “arc_len” represents the object pose projection distance on lane path.
// Check whether object on lane path by bounding box.
bool IsObjectOnLanePathByBoundingBox(const PlannerSemanticMapManager& psmm,
                                     const LanePath& lane_path,
                                     const ObjectProto& object,
                                     double* arc_len) {
  if (lane_path.IsEmpty()) return false;

  const auto closest_lane_point_or =
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm, Vec2dFromProto(object.pos()), lane_path, object.yaw());
  if (!closest_lane_point_or.ok()) return false;

  // *arc_len =
  //     lane_path.FirstOccurrenceOfLanePointToArclength(*closest_lane_point_or);

  // // Ignore far unknown static object.
  // if (object.type() == ObjectType::OT_UNKNOWN_STATIC &&
  //     *arc_len >= kIgnoreUnknownStaticDist) {
  //   return false;
  // }

  const auto bounding_box =
      ComputeObjectContour(object).BoundingBoxWithHeading(object.yaw());
  for (const auto& pt : bounding_box.GetCornersCounterClockwise()) {
    double s;
    if (IsPointOnLanePathAtLevel(psmm, pt, lane_path, &s)) {
      return true;
    }
  }

  return false;
}

/*
Two scenarios are considered:
1. If the dist from stop line to front edge belongs to [0.0, 10.0], regard as
traffic waiting.
    stop line     ------------
                       __
                      |  |
    object box        |  |
                      |__|

2. If the stop line crosses an object, regard as traffic waiting.
                       __
                      |  |
                      |  |
                  ------------
                      |__|
*/
bool IsTrafficWaiting(absl::Span<const double> tls_s,
                      double traffic_chain_front_object_front_s,
                      double traffic_chain_front_object_back_s,
                      double object_back_s, const std::string& obj_id) {
  const auto it_tl_front =
      std::upper_bound(tls_s.begin(), tls_s.end(), object_back_s);
  // No traffic light ahead, return false;
  Log2DDS::LogDataV2("tlw_s", "cal_tls");
  if (it_tl_front == tls_s.end()) return false;

  const double dist_to_front_edge =
      *it_tl_front - traffic_chain_front_object_front_s;
  const double dist_to_back_edge =
      *it_tl_front - traffic_chain_front_object_back_s;
  const std::string dist_to_tls = absl::StrCat(
      "it_tl_front: ", *it_tl_front,
      " dist_to_front_edge: ", dist_to_front_edge,
      " dist_to_back_edge: ", dist_to_back_edge, " obj_id: ", obj_id);
  Log2DDS::LogDataV2("dist_to_tls", dist_to_tls);
  return (0.0 < dist_to_front_edge &&
          dist_to_front_edge < kTrafficLightDistMinThreshold) ||
         (dist_to_front_edge * dist_to_back_edge < 0.0);
}

// Calculate multiplying factor between 1.0 and <max_factor>, when special zone
// ahead. Consider crosswalk/intersection/traffic light control  area as special
// zone. Param <s_vec> record the start_s of all special zones. Param
// <front_object_s> represent the  first front object's back_edge_s of current
// object chain. Param <current_object_s> represent current object's
// back_edge_s.
double CalculateFactorForSpecialZone(absl::Span<const double> zone_s_vec,
                                     absl::Span<const double> object_s_vec,
                                     double distance_threshold,
                                     double max_factor,
                                     const std::string& obj_id) {
  double factor = 1.0;
  for (const double s : object_s_vec) {
    const auto it_prev_special_zone_s =
        std::upper_bound(zone_s_vec.begin(), zone_s_vec.end(), s);
    if (it_prev_special_zone_s == zone_s_vec.end()) continue;
    const double distance = *it_prev_special_zone_s - s;
    const std::string st_l_distance =
        absl::StrCat("obj_id: ", obj_id, " distance: ", distance, " s: ", s,
                     " it_prev_special_zone_s: ", *it_prev_special_zone_s);
    Log2DDS::LogDataV2("distance", st_l_distance);
    if (distance > distance_threshold) continue;
    // The more close to special zone, the more greater factor is.
    factor = std::max(factor, 1.0 + (max_factor - 1.0) *
                                        (1.0 - distance / distance_threshold));
  }
  return factor;
}

// The more moving time of neighbor object is, the greater stall probability is.
double CalculateFactorForMovingNeighbor(
    const ObjectStopTimeProto& stop_time_info) {
  const double factor =
      std::max(0.0, stop_time_info.previous_stop_time_duration() -
                        stop_time_info.last_move_time_duration()) /
      stop_time_info.last_move_time_duration();
  return std::min(1.0, factor);
}

std::vector<const ObjectPredictionProto*> CollectContinuousObjectsAheadOfS(
    const ObjectsOnLane& objects, double s, double* end_back_s) {
  std::vector<const ObjectPredictionProto*> objects_ahead;
  if (objects.empty() ||
      GetObjectFrontS(objects.back().first,
                      objects.back().second->perception_object()) < s) {
    return objects_ahead;
  }

  auto start_it = objects.rbegin(), end_it = objects.rbegin() + 1;
  for (; end_it != objects.rend(); ++end_it) {
    const double object_front_s =
        GetObjectFrontS(end_it->first, end_it->second->perception_object());
    if (object_front_s < s) break;

    const double prev_object_back_s = GetObjectBackS(
        (end_it - 1)->first, (end_it - 1)->second->perception_object());

    if (prev_object_back_s - object_front_s > kMaxLonDistBetweenObjects) {
      start_it = end_it;
    }
  }

  *end_back_s = GetObjectBackS((end_it - 1)->first,
                               (end_it - 1)->second->perception_object());
  // if (*end_back_s - s > kMaxLonDistBetweenObjects) {
  //   return objects_ahead;
  // }

  objects_ahead.reserve(end_it - start_it);
  for (auto it = start_it; it != end_it; ++it) {
    objects_ahead.push_back(it->second);
  }
  return objects_ahead;
}

bool CheckWhetherLanePathControlledByRedTrafficLight(
    const ad_byd::planning::TrafficLightStatusMap& tl_info_map,
    const mapping::LanePath& lane_path) {
  for (const auto id : lane_path.lane_ids()) {
    const auto iter = tl_info_map.find(id);
    if (iter == tl_info_map.end()) continue;
    const auto& tl_info = iter->second;
    // switch (tl_info.junction_id.has_value()) {
    //   case TrafficLightControlType::SINGLE_DIRECTION:
    //     return tl_info.tls().at(TrafficLightDirection::UNMARKED).tl_state ==
    //            TrafficLightState::TL_STATE_RED;
    //   case TrafficLightControlType::LEFT_WAITING_AREA:
    //     return tl_info.tls().at(TrafficLightDirection::LEFT).tl_state ==
    //            TrafficLightState::TL_STATE_RED;
    // }
    if (tl_info.junction_id.has_value() &&
        tl_info.light_status == ad_byd::planning::LightStatus::RED_LIGHT) {
      return true;
    }
  }
  return false;
}

double CalculateFactorAccordHeadingDiff(double object_theta,
                                        double lane_theta) {
  return kCalculateFactorAccordHeadingDiffPlf(
      std::fabs(NormalizeAngle(object_theta - lane_theta)));
}

// DEMO: Don't make stalled decision for object, which on traffic light
// controlled leftmost lane.
double CalculateFactorForTrafficLightControlledLeftmostLane(
    absl::Span<const double> tls_s) {
  return tls_s.empty() || tls_s.front() > kMaxTrafficWaitingDistance ? 1.0
                                                                     : 100.0;
}

bool IsIsolatedLaneAtS(const PlannerSemanticMapManager& psmm,
                       const LanePath& lane_path, double s) {
  const auto& lane_ids = lane_path.lane_ids();
  const auto lane_idx = lane_path.ArclengthToLaneIndex(s);
  const auto lane_info = psmm.FindCurveLaneByIdOrNull(lane_ids[lane_idx]);
  if (lane_info == nullptr) return false;
  return lane_info->left_lane_id() == 0 && lane_info->right_lane_id() == 0;
}

bool IsFarAwayFromTlAtS(absl::Span<const double> tls_s, double s,
                        double max_dist_threshold) {
  if (tls_s.empty()) return true;
  const auto iter_tl_front = std::upper_bound(tls_s.begin(), tls_s.end(), s);
  if (iter_tl_front == tls_s.end()) return true;
  return *iter_tl_front - s > max_dist_threshold;
}

AnalyzeOnLaneOutput AnalyzeOnLane(
    const PlannerSemanticMapManager& psmm, const ObjectsOnLane& objects_on_lane,
    const LanePath& lane_path, bool surrounded_by_obj,
    const ad_byd::planning::TrafficLightStatusMap& tl_info_map,
    const Vec2d& ego_pos, absl::Span<const ObjectsOnLane* const> neighbors,
    ObjectHistoryManager& obj_manager, const BruteForceFrenetFrame& ff) {
  // Extend lane path to kMaxTrafficWaitingDistance to collect stop lines.
  const auto extend_lane_path = ForwardExtendLanePathWithoutFork(
      psmm, lane_path, kMaxTrafficWaitingDistance - lane_path.length());
  auto tls_s = CollectTrafficLightStopLine(psmm, extend_lane_path);
  const auto zones_s = CollectSpecialZonesStartSOnLanePath(psmm, lane_path);
  const auto right_turn_s = CollectRightTurnZone(psmm, lane_path);
  double lateral_buffer = 25.0;
  const auto ego_s_on_lane_path =
      CalcSOnLanePathByPose(psmm, lane_path, ego_pos, lateral_buffer);
  double dis_tls = 0.0;
  const auto& map = psmm.map_ptr();
  const auto& ehp_v2_info = map->v2_info();
  double dist_to_junction_v2 = DBL_MAX;
  // if (ehp_v2_info.has_navigation) {
  //   for (const auto& turn_info : ehp_v2_info.turn_info) {
  //     if (!turn_info.is_valid) break;
  //     if ((turn_info.detail_turn_type ==
  //              ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT ||
  //          turn_info.detail_turn_type ==
  //              ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT ||
  //          turn_info.detail_turn_type ==
  //              ad_byd::planning::V2TurnInfo::V2DetailTurnType::UTURN ||
  //          turn_info.detail_turn_type ==
  //              ad_byd::planning::V2TurnInfo::V2DetailTurnType::CONTINUE ||
  //          turn_info.detail_turn_type ==
  //              ad_byd::planning::V2TurnInfo::V2DetailTurnType::
  //                  TURN_RIGHT_ONLY)) {
  //       dist_to_junction_v2 = turn_info.dist;
  //       break;
  //     }
  //   }
  // }
  double ego_s = 0.0;
  std::string debug_stop_s = "ego_s: ";
  if (ego_s_on_lane_path.has_value()) {
    ego_s = ego_s_on_lane_path.value();
    debug_stop_s =
        debug_stop_s + absl::StrCat(ego_s_on_lane_path.value()).substr(0, 4);
  }
  debug_stop_s = debug_stop_s + " stop_s: ";

  if (!tls_s.empty()) {
    dis_tls = tls_s.front();
  } else if (dist_to_junction_v2 != DBL_MAX) {
    dis_tls = dist_to_junction_v2;
    tls_s.emplace_back(dist_to_junction_v2 + ego_s);
  }
  const std::string tls_s_size =
      absl::StrCat("size: ", tls_s.size(), " s_value: ", dis_tls);

  Log2DDS::LogDataV2("tls_s_size", tls_s_size);

  // for debug
  // double ego_s = 0.0;
  // std::string debug_stop_s = "ego_s: ";
  // if (ego_s_on_lane_path.has_value()) {
  //   ego_s = ego_s_on_lane_path.value();
  //   debug_stop_s =
  //       debug_stop_s + absl::StrCat(ego_s_on_lane_path.value()).substr(0, 4);
  // }
  // debug_stop_s = debug_stop_s + " stop_s: ";

  for (auto& tls : tls_s) {
    debug_stop_s = debug_stop_s + " " + absl::StrCat(tls).substr(0, 4);
  }
  for (auto& zone : zones_s) {
    debug_stop_s = debug_stop_s + " " + absl::StrCat(zone).substr(0, 4);
  }
  Log2DDS::LogDataV0("debug_stop_s", debug_stop_s);

  std::string debug_right_turn_s;
  for (const auto& right_turn : right_turn_s) {
    debug_right_turn_s =
        debug_right_turn_s +
        " start: " + absl::StrCat(right_turn.first).substr(0, 4) +
        " end: " + absl::StrCat(right_turn.second).substr(0, 4);
  }
  Log2DDS::LogDataV0("debug_right_turn_s", debug_right_turn_s);
  // Record all stall analysis temporary result, key object_id; value stall
  // probability.
  absl::flat_hash_map<std::string, double> stall_analysis;

  // Record all traffic waiting queue temporary result, first: object_s ;
  // second: object_id.
  std::vector<std::pair<double, std::string>> traffic_waiting_analysis;

  // Record distance to nearest accident zone.
  std::optional<double> distance_to_roadblock = std::nullopt;
  // First front vehicle in a continuous traffic chain.
  auto prev_cont_it = objects_on_lane.rbegin();
  bool has_stalled_ahead = false;
  // constexpr double kMinMovingSpeedThes = 0.6;
  // Analyze objects from far to near
  for (auto it = objects_on_lane.rbegin(); it != objects_on_lane.rend(); ++it) {
    std::string stop_time_threshold_debug;
    const auto [object_s, object_pred_ptr] = *it;
    const auto& object = object_pred_ptr->perception_object();
    const double object_half_length = HalfLength(object.bounding_box());
    const double object_front_s = object_s + object_half_length;
    const double object_back_s = object_s - object_half_length;
    const auto& stop_time_info = object_pred_ptr->stop_time();
    // NOTE: whether traffic chain is continuous, only depends on the
    // distance between two adjacent objects, with no relevance to object motion
    // state or type.
    if (it != objects_on_lane.rbegin()) {
      const auto it_prev = it - 1;
      const double prev_back_s =
          GetObjectBackS(it_prev->first, it_prev->second->perception_object());
      if (prev_back_s - object_front_s > kMaxLonDistBetweenObjects) {
        prev_cont_it = it;
        has_stalled_ahead = false;
      }
    }
    const auto& object_id = object.id();

    double speed = std::sqrt(object.vel().x() * object.vel().x() +
                             object.vel().y() * object.vel().y());
    std::string stall_speed_debug =
        absl::StrCat("id: ", object_id, " speed: ", speed);
    Log2DDS::LogDataV2("stall_speed_debug", stall_speed_debug);
    // Filter moving object.
    if (!IsStationary(*object_pred_ptr)) continue;
    // if(speed >= kMinMovingSpeedThes) continue;
    auto obs_histroy_info = obj_manager.GetObjLatestFrame(object_id);

    // Ignore all unknown static due to heavy mis-detection from perception.
    // if (object.type() == ObjectType::OT_UNKNOWN_STATIC) {
    //   stall_analysis[object_id] = 0.0;
    //   continue;
    // }
    // Cones and barriers must be stalled.
    if (object.type() == ObjectType::OT_BARRIER ||
        object.type() == ObjectType::OT_CONE ||
        object.type() == ObjectType::OT_WARNING_TRIANGLE ||
        object.type() == ObjectType::OT_UNKNOWN_STATIC ||
        (obs_histroy_info && obs_histroy_info->is_stalled)) {
      stall_analysis[object_id] = 1.0;
      // has_stalled_ahead = true;

      // Objects have been sorted from far to near.
      if (ego_s_on_lane_path.has_value() && object_s > *ego_s_on_lane_path) {
        distance_to_roadblock = object_s - *ego_s_on_lane_path;
      }
      continue;
    }

    // Check object is waiting traffic light turn green.
    // 1. There is no stall object in current traffic chain.
    // 2. There is traffic light ahead.
    // 3. The vehicle in the front of traffic chain is waiting traffic or just
    // getting started.

    // Set default stop_time_threshold. For left most lane, init with 70.0s;
    // for right most lane, init with 10.0s; for other lane, init with 30.0s.
    const bool is_leftmost_lane = IsLeftMostLane(psmm, lane_path, object_s);
    const bool is_rightmost_lane = IsRightMostLane(psmm, lane_path, object_s);
    if (is_rightmost_lane && !is_leftmost_lane)
      kTrafficLightDistMaxThreshold = 60.0;
    double stop_time_threshold =
        (is_leftmost_lane && is_rightmost_lane)
            ? 1.5 * kDefaultStopTimeThreshold
        : is_leftmost_lane  ? 7.0 * kDefaultStopTimeThreshold
        : is_rightmost_lane ? 0.6 * kDefaultStopTimeThreshold
                            : 3.0 * kDefaultStopTimeThreshold;
    stop_time_threshold_debug = absl::StrCat(
        stop_time_threshold_debug, " rel_lane: ", stop_time_threshold);

    // right turn threshold: due to perception performance during turnning,
    // reduce stop_time_threshold
    stop_time_threshold *=
        CalculateFactorForRightTurn(right_turn_s, object_back_s);
    stop_time_threshold_debug = absl::StrCat(
        stop_time_threshold_debug, " right_turn: ", stop_time_threshold);

    const double traffic_chain_front_object_front_s = GetObjectFrontS(
        prev_cont_it->first, prev_cont_it->second->perception_object());
    const double traffic_chain_front_object_back_s = GetObjectBackS(
        prev_cont_it->first, prev_cont_it->second->perception_object());
    bool traffic_wait = IsTrafficWaiting(
        tls_s, traffic_chain_front_object_front_s,
        traffic_chain_front_object_back_s, object_back_s, object_id);

    std::string traffic_wait_debug =
        absl::StrCat(object_id, " ", traffic_wait, " ", has_stalled_ahead);
    Log2DDS::LogDataV2("traffic_wait_debug", traffic_wait_debug);

    const auto it_tl_front =
        std::upper_bound(tls_s.begin(), tls_s.end(), object_front_s);
    const double dist_to_tl = it_tl_front != tls_s.end()
                                  ? *it_tl_front - object_front_s
                                  : std::numeric_limits<double>::infinity();
    bool within_stl_ = (dist_to_tl > kMinTrafficWaitingDistance &&
                        dist_to_tl < kTrafficLightDistMaxThreshold);

    double distance_ = it->first - ego_s;
    double isolated_w = object.type() == ObjectType::OT_LARGE_VEHICLE ? 0.5
                        : distance_ > 0.0 && distance_ < 50.0         ? 0.5
                                                                      : 0.3;
    double traffic_w = isolated_w;
    if ((it + 1) != objects_on_lane.rend()) {
      const auto it_back = it + 1;
      if (IsStationary(*(it_back->second)) && (it_back)->first > ego_s) {
        bool is_back_brakelight =
            it_back->second->perception_object()
                .obstacle_light()
                .brake_lights() == ObstacleLightType::LIGHT_ON;
        if (is_back_brakelight) {
          traffic_w *= 1.5;
          isolated_w *= 2.0;
        }
      }
    }

    if (!has_stalled_ahead && !tls_s.empty() &&
        IsTrafficWaiting(tls_s, traffic_chain_front_object_front_s,
                         traffic_chain_front_object_back_s, object_back_s,
                         object_id)) {
      Log2DDS::LogDataV2("traffic_wait_debug_1", traffic_wait_debug);
      traffic_waiting_analysis.push_back({object_s, object_id});
    } else {
      // const auto it_tl_front =
      //     std::upper_bound(tls_s.begin(), tls_s.end(), object_front_s);
      // const double dist_to_tl = it_tl_front != tls_s.end()
      //                               ? *it_tl_front - object_front_s
      //                               :
      //                               std::numeric_limits<double>::infinity();
      // if ((dist_to_tl > kMinTrafficWaitingDistance &&
      //      dist_to_tl < kTrafficLightDistMaxThreshold)){
      if (within_stl_) {
        // TODO: brake_light
        if ((it == objects_on_lane.rbegin() ||
             (it != objects_on_lane.rbegin() &&
              ((it - 1)->first - it->first) > kMinTrafficWaitingDistance)) &&
            stop_time_info.time_duration_since_stop() > 5.0 &&
            object.obstacle_light().brake_lights() ==
                ObstacleLightType::LIGHT_OFF) {
          stop_time_threshold *=
              std::max(traffic_w,
                       (1.0 - stop_time_info.time_duration_since_stop() / 8.0));
        }
      }
    }
    stop_time_threshold_debug = absl::StrCat(
        stop_time_threshold_debug, " traffic_light: ", stop_time_threshold,
        " traffic_w: ", traffic_w);
    // if ((obs_histroy_info && obs_histroy_info->is_stalled))
    //   stop_time_threshold *= 0.2;
    // stop_time_threshold_debug =
    //     absl::StrCat(stop_time_threshold_debug,
    //                  " is_stalled: ", stop_time_threshold);

    // if (object.obstacle_light().brake_lights() ==
    //     ObstacleLightType::LIGHT_ON) {
    //   stop_time_threshold *= 5.0;
    // }
    // stop_time_threshold_debug = stop_time_threshold_debug + " brake_light: "
    // +
    //                             absl::StrCat(stop_time_threshold);

    // if (object.obstacle_light().hazard_lights() ==
    //     ObstacleLightType::LIGHT_ON) {
    //   stop_time_threshold *= 0.2;
    // }
    // stop_time_threshold_debug =
    //     stop_time_threshold_debug +
    //     " hazard_lights: " + absl::StrCat(stop_time_threshold);

    // const bool is_leftmost_lane = IsLeftMostLane(psmm, lane_path,
    // object_s); const bool is_rightmost_lane = IsRightMostLane(psmm,
    // lane_path, object_s); Set default stop_time_threshold. For left most
    // lane, init with 70.0s; for right most lane, init with 10.0s; for other
    // lane, init with 30.0s.

    // double stop_time_threshold =
    //     (is_leftmost_lane && is_rightmost_lane) ? kDefaultStopTimeThreshold
    //     : is_leftmost_lane  ? 7.0 * kDefaultStopTimeThreshold
    //     : is_rightmost_lane ? kDefaultStopTimeThreshold
    //                         : 3.0 * kDefaultStopTimeThreshold;
    // stop_time_threshold_debug = stop_time_threshold_debug + " rel_lane: " +
    //                             absl::StrCat(stop_time_threshold);
    // Intersection or crosswalk ahead, less likely to be stalled.
    stop_time_threshold *= CalculateFactorForSpecialZone(
        zones_s, {traffic_chain_front_object_back_s, object_back_s},
        kSpecialZoneDistThreshold,
        /*max_factor= */ 2.0, object_id);
    stop_time_threshold_debug = stop_time_threshold_debug + " zones_s: " +
                                absl::StrCat(stop_time_threshold);
    // Traffic light ahead, less likely to be stalled.
    const double max_factor_for_tl =
        (!is_leftmost_lane && !is_rightmost_lane) ? 13.0 : 10.0;

    // in case extreme congestion beyond tl threshold
    if (tls_s.size() != 0 && surrounded_by_obj) {
      kTrafficLightDistMaxThreshold =
          std::max(kTrafficLightDistMaxThreshold, tls_s.front());
    }
    stop_time_threshold *= CalculateFactorForSpecialZone(
        tls_s, {traffic_chain_front_object_back_s, object_back_s},
        kTrafficLightDistMaxThreshold, /*max_factor= */ max_factor_for_tl,
        object_id);
    // in case extreme congestion without map
    if (tls_s.size() == 0 && surrounded_by_obj) {
      stop_time_threshold *= max_factor_for_tl;
    }
    stop_time_threshold_debug = stop_time_threshold_debug +
                                " tls_s: " + absl::StrCat(stop_time_threshold);
    // Laterally neighboring vehicles, less likely to be stalled.
    for (const auto& neighbor_lane : neighbors) {
      double end_back_s = DBL_MAX;
      const auto neighbor_objects = CollectContinuousObjectsAheadOfS(
          *neighbor_lane, object_s - object_half_length, &end_back_s);
      for (const auto& neighbor_ptr : neighbor_objects) {
        double factor = 0.0;
        const auto& neighbor_stop_time_info = neighbor_ptr->stop_time();
        if (IsStationary(*neighbor_ptr)) {
          factor = neighbor_stop_time_info.time_duration_since_stop() /
                   kDefaultStopTimeThreshold;
        } else {
          factor = CalculateFactorForMovingNeighbor(neighbor_stop_time_info);
        }
        factor *= Lerp(1.0, kMaxLonDistBetweenObjects, 0.0,
                       1.5 * kMaxLonDistBetweenObjects,
                       end_back_s - object_s + object_half_length, true);
        stop_time_threshold *= 1.0 + std::min(1.0, factor);
      }
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " neighbors: " +
                                absl::StrCat(stop_time_threshold);
    // The traffic light state is TL_RED, less likely to be stalled.
    // Strategy may conflict with line:308.
    if (CheckWhetherLanePathControlledByRedTrafficLight(tl_info_map,
                                                        lane_path)) {
      stop_time_threshold *= kDefaultRedTrafficLightMultiplier;
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " red_light: " +
                                absl::StrCat(stop_time_threshold);
    // Has moved in the recent past, less likely to be stalled, consider high
    // speed percentage
    if (stop_time_info.last_move_time_duration() > 0.0) {
      stop_time_threshold *=
          std::clamp(15.0 * stop_time_info.accumulated_high_speed_duration() /
                         stop_time_info.time_duration_since_stop(),
                     1.0, 10.0);
    }
    stop_time_threshold_debug = stop_time_threshold_debug +
                                " moved: " + absl::StrCat(stop_time_threshold);
    // Has something stalled ahead, more likely to be stalled.
    if (has_stalled_ahead) {
      stop_time_threshold *= 0.3;
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " stalled: " +
                                absl::StrCat(stop_time_threshold);
    // Stops in parking area, more likely to be stalled.
    if (object.has_parked() && object.parked()) {
      stop_time_threshold *= 0.3;
    }
    stop_time_threshold_debug = stop_time_threshold_debug +
                                " parked: " + absl::StrCat(stop_time_threshold);
    if (FLAGS_planner_ignore_stalled_objects_on_tl_controlled_leftmost_lane &&
        is_leftmost_lane) {
      stop_time_threshold *=
          CalculateFactorForTrafficLightControlledLeftmostLane(tls_s);
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " tl_control: " +
                                absl::StrCat(stop_time_threshold);
    // Front object on isolated lane path without traffic light controlled,
    // more likely to be stalled.
    if (it == prev_cont_it && IsIsolatedLaneAtS(psmm, lane_path, object_s) &&
        IsFarAwayFromTlAtS(tls_s, object_s, kTrafficLightDistMaxThreshold)) {
      // constexpr double kDecayFactorForIsolatedLaneWithoutTLControlled = 0.3;
      stop_time_threshold *= isolated_w;
    }
    stop_time_threshold_debug =
        stop_time_threshold_debug +
        absl::StrCat(" isolated: ", stop_time_threshold,
                     " isolated_w: ", isolated_w, " distance_: ", distance_,
                     " surrounded_by_obj: ", surrounded_by_obj);
    // Calculate factor accord angle diff between object heading and nearest
    // lane point heading. The more heading diff is, the more likely to be
    // stalled object.
    bool is_object_in_right_turn = false;
    for (const auto p : right_turn_s) {
      if (p.first <= object_s && object_s <= p.second) {
        is_object_in_right_turn = true;
        break;
      }
    }
    double theta_ = ArclengthToLerpTheta(psmm, lane_path, object_s);
    double theta_diff_ = std::fabs(NormalizeAngle(object.yaw() - theta_));
    if (!is_object_in_right_turn) {
      stop_time_threshold *= CalculateFactorAccordHeadingDiff(
          object.yaw(), ArclengthToLerpTheta(psmm, lane_path, object_s));
    }
    stop_time_threshold_debug =
        stop_time_threshold_debug +
        absl::StrCat(" theta_: ", theta_, " theta_diff_: ", theta_diff_,
                     " heading: ", stop_time_threshold,
                     " object.yaw:", object.yaw(),
                     " obj_in_right_turn: ", is_object_in_right_turn);
    // Multiple objects ahead, not on rightmost lane, less likely to be
    // stalled.
    if (!is_rightmost_lane) {
      for (auto it_front = prev_cont_it; it_front != it; ++it_front) {
        const auto front_ptr = it_front->second;
        if (IsStationary(*front_ptr)) {
          stop_time_threshold *=
              1.0 + (it - it_front) * std::min(1.0, FindOrDie(stall_analysis,
                                                              front_ptr->id()));
        } else {
          stop_time_threshold *=
              1.0 + (it - it_front) * CalculateFactorForMovingNeighbor(
                                          front_ptr->stop_time());
        }
      }
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " multiple: " +
                                absl::StrCat(stop_time_threshold);

    // judge object is on the line side
    Box2d obstacle_box(object.bounding_box());
    double left_l = -std::numeric_limits<double>::infinity();
    double right_l = std::numeric_limits<double>::infinity();
    for (const auto& ps : obstacle_box.GetCornersCounterClockwise()) {
      const double corner_l = ff.XYToSL(ps).l;
      left_l = std::fmax(corner_l, left_l);
      right_l = std::fmin(corner_l, right_l);
    }
    if (is_rightmost_lane) {
      if (left_l < 0.0) {
        stop_time_threshold *= 0.3;
      } else if (left_l < 0.3 && object.obstacle_light().brake_lights() !=
                                     ObstacleLightType::LIGHT_ON) {
        stop_time_threshold *= 0.3;
      } else {
        stop_time_threshold *=
            (1.4 * std::fmin(0.5, std::fabs(left_l) / (std::fabs(left_l) +
                                                       std::fabs(right_l))) +
             0.3);
      }
    }

    stop_time_threshold_debug = stop_time_threshold_debug +
                                absl::StrCat(" offset: ", left_l, " ", right_l,
                                             " ", stop_time_threshold);

    // if (NormalizeAngle(object.yaw() - theta_) > M_PI / 6.0)
    //   stop_time_threshold *= 3.0;
    // stop_time_threshold_debug = stop_time_threshold_debug + " heading-inner"
    // +
    //                             absl::StrCat(stop_time_threshold);
    if (stop_time_info.last_hazard_light_time() >= 20.0) {
      if (object.obstacle_light().brake_lights() ==
              ObstacleLightType::LIGHT_ON &&
          object.obstacle_light().hazard_lights() ==
              ObstacleLightType::LIGHT_OFF) {
        stop_time_threshold *= 5.0;
      } else if (stop_time_info.previous_stop_time_duration() > 0.0) {
        stop_time_threshold *= std::clamp(
            5.0 * stop_time_info.previous_time_duration_brake_light() /
                stop_time_info.time_duration_none_brake_light(),
            1.0, 5.0);
      }
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " brake_light: " +
                                absl::StrCat(stop_time_threshold);

    double hazard_lights_w_ = is_rightmost_lane && !is_leftmost_lane
                                  ? (within_stl_ ? 0.3 : 0.2)
                              : (!is_leftmost_lane) ? (within_stl_ ? 0.4 : 0.3)
                              : (is_rightmost_lane && is_leftmost_lane) ? 0.2
                                                                        : 0.3;
    // && std::fabs(NormalizeAngle(object.yaw() - theta_)) ?
    // hazard light lag 20s
    if (object.obstacle_light().hazard_lights() ==
            ObstacleLightType::LIGHT_ON ||
        stop_time_info.last_hazard_light_time() < 20.0) {
      stop_time_threshold *= hazard_lights_w_;
    }
    stop_time_threshold_debug =
        stop_time_threshold_debug +
        absl::StrCat(" ****hazard_lights: ", stop_time_threshold,
                     " hazard_lights_w_: ", hazard_lights_w_, " within_stl_ ",
                     within_stl_);

    // check if subtype is special vehicle, which is more likely to be stalled
    // vehicle
    if (object.subtype() == ObjectSubType::OST_SPECIAL_VEHICLE) {
      stop_time_threshold *= 0.2;
    }

    stop_time_threshold_debug =
        stop_time_threshold_debug +
        " ****special_vehicle: " + absl::StrCat(stop_time_threshold);

    // Check stopped time at last.
    const double stalled_prob =
        stop_time_info.time_duration_since_stop() / stop_time_threshold;
    if (stalled_prob >= kStalledProbThreshold) {
      has_stalled_ahead = true;
    }
    stop_time_threshold_debug =
        stop_time_threshold_debug +
        absl::StrCat(" time_duration_since_stop: ",
                     stop_time_info.time_duration_since_stop(),
                     " stalled_prob: ", stalled_prob);
    stall_analysis[object_id] = std::min(1.0, stalled_prob);
    Log2DDS::LogDataV0("stop_time_threshold", object_pred_ptr->id() + ": " +
                                                  stop_time_threshold_debug);
  }

  // Generate traffic waiting queue.
  TrafficWaitingQueueProto traffic_waiting_queue;
  std::stable_sort(traffic_waiting_analysis.begin(),
                   traffic_waiting_analysis.end());
  for (const auto& tw_info : traffic_waiting_analysis) {
    // Note: The id which in the traffic_waiting_queue was contained by
    // stall_analysis either .
    if (stall_analysis[tw_info.second] > kStalledProbThreshold) {
      continue;
    }
    traffic_waiting_queue.add_object_id(tw_info.second);
  }
  lane_path.ToProto(traffic_waiting_queue.mutable_lane_path());

  std::vector<ObjectAnnotationProto> object_annotations;
  object_annotations.reserve(stall_analysis.size());
  // Generate object annotation.
  for (const auto& [object_id, stall_probability] : stall_analysis) {
    if (stall_probability >= kStalledProbThreshold) {
      ObjectAnnotationProto object_annotation;
      object_annotation.set_object_id(object_id);
      object_annotation.set_stalled_vehicle_likelyhood(stall_probability);
      object_annotation.set_depart_soon_likelyhood(1.0 - stall_probability);
      object_annotations.push_back(std::move(object_annotation));
    }
  }

  return AnalyzeOnLaneOutput{
      .traffic_waiting_queue = std::move(traffic_waiting_queue),
      .object_annotations = std::move(object_annotations),
      .distance_to_roadblock = distance_to_roadblock};
}

std::optional<double> MergeDistanceToAccidentZone(
    std::optional<double> dist_1, std::optional<double> dist_2) {
  if (dist_1.has_value() && dist_2.has_value()) {
    return std::min<double>(*dist_1, *dist_2);
  } else if (dist_1.has_value()) {
    return dist_1;
  } else if (dist_2.has_value()) {
    return dist_2;
  } else {
    return std::nullopt;
  }
}

TrafficFlowReasoningOutput MergeMultiAnalyzeOutput(
    std::vector<AnalyzeOnLaneOutput> analyze_outputs) {
  TrafficFlowReasoningOutput output;
  std::map<std::string, ObjectAnnotationProto> objects_analysis_map;

  output.traffic_waiting_queues.reserve(analyze_outputs.size());
  // Collect all traffic waiting queues and build objects analysis map.
  for (auto& analysis : analyze_outputs) {
    output.traffic_waiting_queues.push_back(
        std::move(analysis.traffic_waiting_queue));

    for (auto& object_annotation : analysis.object_annotations) {
      // NOTE: id will be empty after object_annotation is moved.
      const auto& id = object_annotation.object_id();
      // Choose object annotation which stalled likelihood greater.
      if (objects_analysis_map.find(id) == objects_analysis_map.end()) {
        objects_analysis_map.emplace(id, std::move(object_annotation));
      } else {
        const auto& pre_object_annotation = objects_analysis_map[id];

        if (pre_object_annotation.stalled_vehicle_likelyhood() <
            object_annotation.stalled_vehicle_likelyhood()) {
          objects_analysis_map[id] = std::move(object_annotation);
        }
      }
    }

    output.distance_to_roadblock = MergeDistanceToAccidentZone(
        output.distance_to_roadblock, analysis.distance_to_roadblock);
  }

  // Collect all objects annotation.
  for (auto& pair : objects_analysis_map) {
    output.object_annotations.push_back(std::move(pair.second));
  }

  return output;
}
}  // namespace

absl::StatusOr<TrafficFlowReasoningOutput> RunTrafficFlowReasoning(
    const TrafficFlowReasoningInput& input, ThreadPool* thread_pool,
    ObjectHistoryManager& obj_manager) {
  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.prediction);
  CHECK_NOTNULL(input.lane_paths);
  CHECK_NOTNULL(input.tl_info_map);
  CHECK_NOTNULL(input.plan_start_point);

  const auto& psmm = *input.psmm;
  const auto& prediction = *input.prediction;
  // The lane path must in order, from left to right.
  const auto& lane_paths = *input.lane_paths;
  const auto& tl_info_map = *input.tl_info_map;
  // SensorFovRef sensor_fov;
  // if (input.sensor_fovs != nullptr) {
  //   sensor_fov = std::make_shared<SensorFov>(
  //       sensor_fov::BuildLidarViewSensorFov(*input.sensor_fovs));
  // }
  // Check lane path set not empty.
  if (lane_paths.empty()) {
    return absl::InternalError("No Lane Path to make scene understanding");
  }
  // Classify object according to pos, and associate with lane path.
  std::vector<ObjectsOnLane> objects_on_lane_vec;
  const int lane_paths_size = lane_paths.size();
  objects_on_lane_vec.resize(lane_paths_size);
  for (auto& objects_on_lane : objects_on_lane_vec) {
    objects_on_lane.reserve(prediction.objects_size());
  }

  // TODO: associate one object with multi lane path.
  // Classify objects from right to left.
  for (const auto& object_pred : prediction.objects()) {
    const auto& object = object_pred.perception_object();
    // Ignore vegetation, pedestrian, cyclist in traffic flow reasoning.
    if (object.type() == ObjectType::OT_VEGETATION ||
        object.type() == ObjectType::OT_PEDESTRIAN ||
        object.type() == ObjectType::OT_CYCLIST) {
      continue;
    }

    // Associate object with lane path by pose.
    bool has_associated = false;
    for (int i = lane_paths_size - 1; i >= 0; --i) {
      double s;
      const auto& lane_path = lane_paths[i];
      if (IsObjectOnLanePathByPose(psmm, lane_path, object, &s)) {
        objects_on_lane_vec[i].push_back({s, &object_pred});
        has_associated = true;
        break;
      }
    }

    // Associate object with lane path by bounding box. If associate failed by
    // object pose.
    if (has_associated == false) {
      for (int i = lane_paths_size - 1; i >= 0; --i) {
        double s;
        const auto& lane_path = lane_paths[i];
        if (IsObjectOnLanePathByBoundingBox(psmm, lane_path, object, &s)) {
          objects_on_lane_vec[i].push_back({s, &object_pred});
          break;
        }
      }
    }
  }

  // Sort object according s, from near to far.
  for (auto& objects_on_lane : objects_on_lane_vec) {
    std::stable_sort(objects_on_lane.begin(), objects_on_lane.end());
  }

  const auto ego_pos =
      Vec2dFromApolloTrajectoryPointProto(*input.plan_start_point);

  std::vector<AnalyzeOnLaneOutput> analyze_outputs(lane_paths_size);
  bool front_obj = false;
  bool back_obj = false;
  bool neighbor_obj = false;
  const double kObjDist = 15.0;
  const double kTrafficJamSpeed = 5.0;
  // find ego lane path index with default lateral buffer
  int ego_lp_index = 0;
  bool find_ego_lp = false;
  std::string ego_lp_end = "";
  for (int i = 0; i < lane_paths_size; ++i) {
    const auto ego_s = CalcSOnLanePathByPose(psmm, lane_paths[i], ego_pos);
    if (ego_s.has_value()) {
      find_ego_lp = true;
      ego_lp_index = i;
      if (!lane_paths[i].lane_seq()->lanes().empty()) {
        ego_lp_end =
            std::to_string(lane_paths[i].lane_seq()->lanes().back()->id());
      }
    }
  }
  // find ego s in every lane path with large lateral buffer
  for (int i = 0; i < lane_paths_size; ++i) {
    const auto ego_s =
        CalcSOnLanePathByPose(psmm, lane_paths[i], ego_pos, 50.0);
    if (!ego_s.has_value() || !find_ego_lp) {
      continue;
    }
    // in merge case, 2 lps have same end lane
    bool has_same_end = false;
    if (i != ego_lp_index && !lane_paths[i].lane_seq()->lanes().empty() &&
        std::to_string(lane_paths[i].lane_seq()->lanes().back()->id()) ==
            ego_lp_end) {
      has_same_end = true;
    }
    for (const auto& obj : objects_on_lane_vec[i]) {
      if (!obj.second->has_perception_object() ||
          !obj.second->perception_object().has_type() ||
          (obj.second->perception_object().type() !=
               st::ObjectType::OT_VEHICLE &&
           obj.second->perception_object().type() !=
               st::ObjectType::OT_LARGE_VEHICLE)) {
        continue;
      }
      // skip surrounding stalled vehicle, only none-stalled is considered
      auto obs_histroy_info = obj_manager.GetObjLatestFrame(obj.second->id());
      if (obs_histroy_info && obs_histroy_info->is_stalled) continue;
      double obj_v = 0.0;
      if (obj.second->perception_object().has_vel()) {
        obj_v = std::hypot(obj.second->perception_object().vel().x(),
                           obj.second->perception_object().vel().y());
      }
      double obj_front_s =
          GetObjectFrontS(obj.first, obj.second->perception_object());
      double obj_back_s =
          GetObjectBackS(obj.first, obj.second->perception_object());
      double dist =
          std::hypot(obj.second->perception_object().pos().x() - ego_pos.x(),
                     obj.second->perception_object().pos().y() - ego_pos.y());
      if (i == ego_lp_index && obj.first > ego_s.value() &&
          obj_back_s - ego_s.value() < kObjDist && obj_v < kTrafficJamSpeed) {
        front_obj = true;
      } else if (i == ego_lp_index && obj.first < ego_s.value() &&
                 ego_s.value() - obj_front_s < kObjDist &&
                 obj_v < kTrafficJamSpeed) {
        back_obj = true;
      } else if ((i == ego_lp_index - 1 || i == ego_lp_index + 1 ||
                  (has_same_end && ego_lp_index == lane_paths_size - 1 &&
                   i == ego_lp_index - 2)) &&
                 obj.first < ego_s.value() + kObjDist &&
                 obj.first > ego_s.value() - kObjDist &&
                 obj_v < kTrafficJamSpeed) {
        neighbor_obj = true;
      }
    }
  }
  if (lane_paths_size == 1) {
    neighbor_obj = true;
  }
  bool surrounded_by_obj = neighbor_obj && front_obj && back_obj;
  ParallelFor(0, lane_paths_size, thread_pool, [&](int i) {
    std::vector<const ObjectsOnLane*> neighbors;
    // Not leftmost lane, has left neighbor.
    if (i != 0) {
      neighbors.push_back(&objects_on_lane_vec[i - 1]);
      // for rightmost merge lane path, consider leftmost lane path after merge
      // point
      if (i == lane_paths_size - 1 && lane_paths_size > 2) {
        for (auto it = lane_paths[i].lane_seq()->lanes().begin();
             it != lane_paths[i].lane_seq()->lanes().end(); ++it) {
          if ((*it)->pre_lane_ids().size() >= 2) {
            neighbors.push_back(&objects_on_lane_vec[i - 2]);
          }
        }
      }
    }
    // Not rightmost lane and not second rightmost lane, consider right
    // neighbor.
    if (i != lane_paths_size - 1 && i != lane_paths_size - 2) {
      neighbors.push_back(&objects_on_lane_vec[i + 1]);
    }
    const auto ff =
        BuildBruteForceFrenetFrame(SampleLanePathPoints(psmm, lane_paths[i]),
                                   /*down_sample_raw_points=*/true);

    if (ff.ok()) {
      analyze_outputs[i] = AnalyzeOnLane(
          psmm, objects_on_lane_vec[i], lane_paths[i], surrounded_by_obj,
          tl_info_map, ego_pos, neighbors, obj_manager, ff.value());
    }
  });

  return MergeMultiAnalyzeOutput(std::move(analyze_outputs));
}
}  // namespace st::planning
