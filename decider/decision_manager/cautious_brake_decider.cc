

#include <algorithm>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "plan_common/container/strong_int.h"
#include "decider/decision_manager/cautious_brake_decider.h"
#include "decider/decision_manager/decision_util.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
//#include "planner/planner_manager/planner_flags.h"
//#include "planner/planner_manager/planner_util.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/util/status_macros.h"

namespace st {
namespace planning {
namespace {
constexpr double kCrosswalkAssociationDistance = 2.0;
constexpr double kEpsilon = 0.1;
constexpr double kDrivePassageEndPointEpsilon = 0.1;
constexpr double kIntersectionSpeedReductionRatio = 0.75;
constexpr double kIntersectionLowerSpeedLimit = 60.0;  // kph
constexpr double kIntersectionUpperSpeedLimit = 80.0;  // kph
constexpr double kEmptyCrossWalkSpeedReductionRatio = 0.9;
constexpr double kVRUNearCrossWalkSpeedReductionRatio = 0.8;
constexpr double kVRUOnCrossWalkSpeedReductionRatio = 0.7;
constexpr double kBrakeDisInAdvance = 10.0;
constexpr double kEhpV2TurnSpeedLimit = Kph2Mps(40.0);
constexpr double kEhpV2TurnRightSpeedLimit = Kph2Mps(20.0);
constexpr double kEhpV2SplitSpeedLimit = Kph2Mps(15.0);

constexpr double kMinCautiousSpeed = 3.0;  // m/s

std::optional<double> GetDistanceToSplit(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path_from_start, double* const lane_length,
    Vec2d* const start_pt, Vec2d* const end_pt) {
  if ((!lane_length) || (!start_pt) || (!end_pt)) return std::nullopt;
  double distance_to_split = 0.0;
  for (const auto& seg : lane_path_from_start) {
    const auto& lane_info_ptr =
        planner_semantic_map_manager.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) {
      return std::nullopt;
    }
    if (lane_info_ptr->split_topology() ==
        ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
      *lane_length = lane_info_ptr->curve_length() * (1.0 - seg.start_fraction);
      *start_pt = lane_info_ptr->LerpPointFromFraction(seg.start_fraction);
      *end_pt = lane_info_ptr->LerpPointFromFraction(1.0);
      return distance_to_split;
    }
    if (lane_info_ptr->turn_type() != ad_byd::planning::TurnType::NO_TURN) {
      return std::nullopt;
    }
    distance_to_split += (seg.end_s - seg.start_s);
  }
  return std::nullopt;
}

std::optional<double> GetDistanceToUturn(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path_from_start, double* const lane_length,
    Vec2d* const start_pt, Vec2d* const end_pt) {
  if ((!lane_length) || (!start_pt) || (!end_pt)) return std::nullopt;
  double distance_to_split = 0.0;
  for (const auto& seg : lane_path_from_start) {
    const auto& lane_info_ptr =
        planner_semantic_map_manager.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) {
      return std::nullopt;
    }
    if (lane_info_ptr->turn_type() == ad_byd::planning::TurnType::U_TURN) {
      *lane_length = lane_info_ptr->curve_length() * (1.0 - seg.start_fraction);
      *start_pt = lane_info_ptr->LerpPointFromFraction(seg.start_fraction);
      *end_pt = lane_info_ptr->LerpPointFromFraction(1.0);
      return distance_to_split;
    }
    if (lane_info_ptr->turn_type() != ad_byd::planning::TurnType::NO_TURN) {
      return std::nullopt;
    }
    distance_to_split += (seg.end_s - seg.start_s);
  }
  return std::nullopt;
}

absl::StatusOr<ConstraintProto::SpeedRegionProto> ProcessMapElement(
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    const mapping::LanePath::LaneSegment& seg,
    const ad_byd::planning::Lane& lane_info,
    const std::pair<mapping::ElementId, Vec2d>& element, double s_offset,
    double lane_speed_limit, double speed_reduction_ratio) {
  if (seg.start_fraction > element.second.y()) {
    return absl::NotFoundError("Map Element not on current seg.");
  }

  const double passage_min_s = passage.front_s();
  const double passage_max_s = passage.end_s();
  const auto start_point = lane_info.LerpPointFromFraction(element.second.x());
  // End point.
  const auto end_point = lane_info.LerpPointFromFraction(element.second.y());
  ConstraintProto::SpeedRegionProto cautious_brake_constraint;
  start_point.ToProto(cautious_brake_constraint.mutable_start_point());
  end_point.ToProto(cautious_brake_constraint.mutable_end_point());

  const double start_point_s =
      lane_path_from_start.LaneIndexPointToArclength(
          /*lane_index_point=*/{seg.lane_index, element.second.x()}) +
      s_offset + passage.lane_path_start_s();
  const double end_point_s =
      lane_path_from_start.LaneIndexPointToArclength(
          /*lane_index_point=*/{seg.lane_index, element.second.y()}) +
      s_offset + passage.lane_path_start_s();

  if (start_point_s >= passage_max_s - kDrivePassageEndPointEpsilon) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element beyond current drive passage. "
                        "start_point_s: %f, passage_max_s:%f",
                        start_point_s, passage_max_s));
  } else if (end_point_s <= passage_min_s + kDrivePassageEndPointEpsilon) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element behind current drive passage. "
                        "end_point_s:%f, drive passage_min_s: %f",
                        end_point_s, passage_min_s));
  } else if (start_point_s + kDrivePassageEndPointEpsilon >= end_point_s) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element range on drive passage is too short. "
                        "start_pont_s: %f, end_point_s: %f",
                        start_point_s, end_point_s));
  }
  cautious_brake_constraint.set_start_s(std::max(start_point_s, passage_min_s));
  cautious_brake_constraint.set_end_s(std::min(end_point_s, passage_max_s));
  const auto speed_limit =
      std::max(kMinCautiousSpeed, lane_speed_limit * speed_reduction_ratio);
  cautious_brake_constraint.set_max_speed(speed_limit);

  return cautious_brake_constraint;
}
}  // namespace

std::vector<ConstraintProto::SpeedRegionProto> BuildCautiousBrakeConstraints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const SpacetimeTrajectoryManager& st_mgr) {
  absl::flat_hash_map<std::string,
                      std::vector<ConstraintProto::SpeedRegionProto>>
      id_element_map;
  for (const auto& seg : lane_path_from_start) {
    const auto lane_info_ptr =
        planner_semantic_map_manager.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) continue;
    const double speed_limit = std::fmax(
        Kph2Mps(kIntersectionLowerSpeedLimit),
        planner_semantic_map_manager.QueryLaneSpeedLimitById(seg.lane_id));
    // Intersection.
    // TODO: reopen by user
    // for (const auto& intersection : lane_info_ptr->Intersections()) {
    //   double speed_reduce_ratio =
    //       Lerp(kIntersectionSpeedReductionRatio,
    //            Kph2Mps(kIntersectionUpperSpeedLimit), 1.0,
    //            Kph2Mps(kIntersectionLowerSpeedLimit), speed_limit, true);
    //   ASSIGN_OR_CONTINUE(
    //       auto cautious_brake_constraint,
    //       ProcessMapElement(passage, lane_path_from_start, seg,
    //       *lane_info_ptr,
    //                         intersection, s_offset, speed_limit,
    //                         speed_reduce_ratio));
    //   const auto id = intersection.first;
    //   cautious_brake_constraint.mutable_source()
    //       ->mutable_intersection()
    //       ->set_id(id);
    //   const std::string str_id =
    //       absl::StrFormat("cautious_brake_intersection_%s", id);
    //   cautious_brake_constraint.set_id(str_id);
    //   id_element_map[str_id].push_back(std::move(cautious_brake_constraint));
    // }
    // Crosswalk.
    // for (const auto& cw_idx : lane_info_ptr->Crosswalks()) {
    //   const auto* cw_ptr =
    //       planner_semantic_map_manager.FindCrosswalkByIdOrNull(cw_idx.first);
    //   if (cw_ptr == nullptr) continue;
    //   double reduction_ratio = kEmptyCrossWalkSpeedReductionRatio;
    //   if (IsAnyVruOnCrosswalk(*cw_ptr, st_mgr)) {
    //     reduction_ratio = kVRUOnCrossWalkSpeedReductionRatio;
    //   } else if (IsAnyVruNearCrosswalk(*cw_ptr, st_mgr)) {
    //     reduction_ratio = kVRUNearCrossWalkSpeedReductionRatio;
    //   }
    //   ASSIGN_OR_CONTINUE(
    //       auto cautious_brake_constraint,
    //       ProcessMapElement(passage, lane_path_from_start, seg,
    //       *lane_info_ptr,
    //                         cw_idx, s_offset, speed_limit, reduction_ratio));
    //   const auto id = cw_idx.first;
    //   cautious_brake_constraint.mutable_source()
    //       ->mutable_intersection()
    //       ->set_id(id);
    //   const std::string str_id =
    //       absl::StrFormat("cautious_brake_crosswalk_%d", id);
    //   cautious_brake_constraint.set_id(str_id);
    //   id_element_map[str_id].push_back(std::move(cautious_brake_constraint));
    // }
  }

  // ehp v2 turn type
  const auto& ehp_v2_info = planner_semantic_map_manager.map_ptr()->v2_info();

  // v2 split brake
  if (ehp_v2_info.has_navigation && !FLAGS_planner_mapless_status) {
    double split_length = 0.0;
    Vec2d start_pt;
    Vec2d end_pt;
    std::optional<double> distance_to_split =
        GetDistanceToSplit(planner_semantic_map_manager, lane_path_from_start,
                           &split_length, &start_pt, &end_pt);
    if (distance_to_split.has_value() && distance_to_split.value() < 100.0) {
      for (const auto& turn_info : ehp_v2_info.turn_info) {
        if (((turn_info.turn_type ==
              ad_byd::planning::V2TurnInfo::V2TurnType::RIGHT) ||
             (turn_info.turn_type ==
              ad_byd::planning::V2TurnInfo::V2TurnType::RAMP_RIGHT)) &&
            ((distance_to_split.value() < turn_info.dist + 15.0) &&
             (distance_to_split.value() > turn_info.dist - 15.0))) {
          ConstraintProto::SpeedRegionProto
              ehp_v2_split_cuatious_speed_constraint;
          std::string str_id = "v2_split_brake";
          ehp_v2_split_cuatious_speed_constraint.set_id(str_id);
          ehp_v2_split_cuatious_speed_constraint.set_start_s(
              distance_to_split.value());
          ehp_v2_split_cuatious_speed_constraint.set_end_s(
              distance_to_split.value() + split_length);
          ehp_v2_split_cuatious_speed_constraint.set_max_speed(
              kEhpV2SplitSpeedLimit);
          start_pt.ToProto(
              ehp_v2_split_cuatious_speed_constraint.mutable_start_point());
          end_pt.ToProto(
              ehp_v2_split_cuatious_speed_constraint.mutable_end_point());
          ehp_v2_split_cuatious_speed_constraint.mutable_source()
              ->mutable_intersection()
              ->set_id("v2_split_brake");
          id_element_map[str_id].push_back(
              std::move(ehp_v2_split_cuatious_speed_constraint));
          break;
        }
      }
    }
  }

  // uturn brake

  if (ehp_v2_info.has_navigation && FLAGS_planner_enable_u_turn_speed_limit) {
    double uturn_length = 0.0;
    Vec2d start_pt;
    Vec2d end_pt;
    std::optional<double> distance_to_uturn =
        GetDistanceToUturn(planner_semantic_map_manager, lane_path_from_start,
                           &uturn_length, &start_pt, &end_pt);
    // std::cout << "[wxyDebug] distance_to_turn " << distance_to_uturn.value()
    // << std::endl;
    if (distance_to_uturn.has_value() && distance_to_uturn.value() < 120.0) {
      ConstraintProto::SpeedRegionProto uturn_cautious_speed_constraint;
      std::string str_id = "uturn_brake";
      uturn_cautious_speed_constraint.set_id(str_id);
      const double modified_start_distance =
          std::max(0.0, distance_to_uturn.value() - 25.0);
      // std::cout << "[wxyDebug] modified_start_distance " <<
      // modified_start_distance << "distance_to_uturn.value() " <<
      // distance_to_uturn.value() << std::endl;
      uturn_cautious_speed_constraint.set_start_s(modified_start_distance);
      const double modifed_end_distance =
          distance_to_uturn.value() + uturn_length + 10.0;
      // std::cout << "[wxyDebug] modifed_end_distance " << modifed_end_distance
      // << "distance_to_uturn.value() " << distance_to_uturn.value() << "
      // uturn_length " << uturn_length << std::endl;
      uturn_cautious_speed_constraint.set_end_s(modifed_end_distance);
      uturn_cautious_speed_constraint.set_max_speed(2.0);
      start_pt.ToProto(uturn_cautious_speed_constraint.mutable_start_point());
      end_pt.ToProto(uturn_cautious_speed_constraint.mutable_end_point());
      uturn_cautious_speed_constraint.mutable_source()
          ->mutable_intersection()
          ->set_id("uturn_brake");
      id_element_map[str_id].push_back(
          std::move(uturn_cautious_speed_constraint));
    }
  }

  std::vector<ConstraintProto::SpeedRegionProto> cautious_brake_constraints;
  cautious_brake_constraints.reserve(id_element_map.size());
  for (const auto& pair : id_element_map) {
    cautious_brake_constraints.push_back(MergeSameElement(pair.second));
  }
  return cautious_brake_constraints;
}

}  // namespace planning
}  // namespace st
