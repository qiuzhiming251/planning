

#include "planner/speed_optimizer/path_semantic_analyzer.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_set>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "plan_common/async/parallel_for.h"
#include "plan_common/base/macros.h"
#include "plan_common/container/strong_int.h"
//#include "global/trace.h"
//#include "lite/check.h"
//#include "lite/logging.h"
//#include "lite/qissue_trans.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/util.h"
#include "plan_common/plan_common_defs.h"
//#include "q_issue.pb.h"
//#include "semantic_map.pb.h"
#include "plan_common/math/frenet_frame.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "plan_common/util/lane_point_util.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/spatial_search_util.h"

namespace st {
namespace planning {

namespace {

constexpr double kMinDistSqrThreshold = Sqr(0.5 * kPathSampleInterval);  // m^2.
constexpr double kHighPriorDistDiffThreshold = 2.0 * kPathSampleInterval;  // m.
constexpr double kHighPriorDistDiffSqrThreshold =
    Sqr(kHighPriorDistDiffThreshold);  // m^2.
constexpr double kEps = 1e-3;

inline LaneSemantic QueryLaneSemantic(const ad_byd::planning::Lane& lane_info) {
  if (!(lane_info.junction_id() == 0)) {
    switch (lane_info.turn_type()) {
      case ad_byd::planning::TurnType::NO_TURN:
        return LaneSemantic::INTERSECTION_STRAIGHT;
      case ad_byd::planning::TurnType::LEFT_TURN:
        return LaneSemantic::INTERSECTION_LEFT_TURN;
      case ad_byd::planning::TurnType::RIGHT_TURN:
        return LaneSemantic::INTERSECTION_RIGHT_TURN;
      case ad_byd::planning::TurnType::U_TURN:
        return LaneSemantic::INTERSECTION_UTURN;
      default:
        throw std::runtime_error("switch case on enum unexpected");
    }
  } else {
    switch (lane_info.turn_type()) {
      case ad_byd::planning::TurnType::NO_TURN:
        return LaneSemantic::ROAD;
      case ad_byd::planning::TurnType::LEFT_TURN:
      case ad_byd::planning::TurnType::RIGHT_TURN:
      case ad_byd::planning::TurnType::U_TURN:
        // LOG_EVERY_N_SEC(ERROR, 2.0)
        //     << "Lane " << lane_info.id << " direction is "
        //     << mapping::LaneProto::Direction_Name(lane_info.direction)
        //     << " but not in intersection.";
        return LaneSemantic::ROAD;
      default:
        throw std::runtime_error("switch case on enum unexpected");
    }
  }
}

inline void FillExtendedPartPathSemantics(const PathPoint& path_point,
                                          PathPointSemantic* path_semantic) {
  CHECK_NOTNULL(path_semantic);
  path_semantic->closest_lane_point_pos = ToVec2d(path_point);
  path_semantic->deviation_distance = 0.0;
  path_semantic->lane_semantic = LaneSemantic::ROAD;
}

void AnalyzePrimaryPathSemantic(
    const PathPoint& path_point,
    const std::vector<RouteLaneInfo>& high_priority_lanes,
    const std::vector<RouteLaneInfo>& other_lanes,
    const PlannerSemanticMapManager& psmm, PathPointSemantic* path_semantic) {
  const Vec2d path_point_xy = ToVec2d(path_point);
  double min_dist_sqr = std::numeric_limits<double>::infinity();
  mapping::LanePoint closest_lane_point(
      /*lane_id=*/mapping::kInvalidElementId,
      /*fraction=*/0.0);
  Vec2d closest_lane_point_pos;
  for (const auto& route_lane_info : high_priority_lanes) {
    if (route_lane_info.lane == nullptr) {
      continue;
    }
    Vec2d closest_point;
    const auto lane_point_opt =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm, path_point_xy, route_lane_info.lane->id(), path_point.theta(),
            /*heading_penalty_weight=*/0.0, &closest_point,
            route_lane_info.start_fraction, route_lane_info.end_fraction);
    if (!lane_point_opt.has_value()) continue;
    const double dist_sqr = path_point_xy.DistanceSquareTo(closest_point);
    if (dist_sqr < min_dist_sqr) {
      min_dist_sqr = dist_sqr;
      closest_lane_point = *lane_point_opt;
      closest_lane_point_pos = closest_point;
    }
    // Since path point interval is about 0.2m, if minimum distance is less
    // than 0.1m, we don't need to continue find closest point on other lanes.
    if (min_dist_sqr < kMinDistSqrThreshold) {
      break;
    }
  }
  if (!closest_lane_point.Valid() ||
      min_dist_sqr > kHighPriorDistDiffSqrThreshold) {
    double other_lane_max_dist_sqr =
        closest_lane_point.Valid()
            ? Sqr(std::sqrt(min_dist_sqr) - kHighPriorDistDiffThreshold)
            : min_dist_sqr;
    for (const auto& lane : other_lanes) {
      Vec2d closest_point;
      const auto lane_point_opt =
          FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
              psmm, path_point_xy, lane.lane->id(), path_point.theta(),
              /*heading_penalty_weight=*/0.0, &closest_point,
              lane.start_fraction, lane.end_fraction);
      if (!lane_point_opt.has_value()) {
        continue;
      }
      const double dist_sqr = path_point_xy.DistanceSquareTo(closest_point);
      if (dist_sqr < other_lane_max_dist_sqr) {
        other_lane_max_dist_sqr = dist_sqr;
        min_dist_sqr = dist_sqr;
        closest_lane_point = *lane_point_opt;
        closest_lane_point_pos = closest_point;
      }
      if (min_dist_sqr < kMinDistSqrThreshold) {
        break;
      }
    }
  }
  if (!closest_lane_point.Valid()) {
    return;
  }
  path_semantic->closest_lane_point = closest_lane_point;
  path_semantic->closest_lane_point_pos = closest_lane_point_pos;
  path_semantic->deviation_distance = std::sqrt(min_dist_sqr);
  const auto& lane_info_ptr =
      psmm.FindCurveLaneByIdOrNull(closest_lane_point.lane_id());
  if (UNLIKELY(lane_info_ptr == nullptr)) {
    LOG_ERROR << "Cannot find semantic id: " << closest_lane_point.lane_id();
    // QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
    //                   QIssueSubType::QIST_SEMANTIC_MAP,
    //                   absl::StrCat("It is a map issue, not a planner issue.
    //                   ",
    //                                closest_lane_point.lane_id()));
    return;
  }
  path_semantic->lane_semantic = QueryLaneSemantic(*lane_info_ptr);
  path_semantic->lane_info = lane_info_ptr;
  return;
}

void AnalyzeSecondaryPathSemantic(
    const PathPoint& path_point,
    const std::vector<RouteLaneInfo>& high_priority_lanes,
    const std::vector<RouteLaneInfo>& other_lanes,
    const PlannerSemanticMapManager& psmm,
    const PathPointSemantic& prev_primary_path_semantic,
    const PathPointSemantic& next_primary_path_semantic, double ratio,
    PathPointSemantic* path_semantic) {
  if (!prev_primary_path_semantic.closest_lane_point.Valid() ||
      !next_primary_path_semantic.closest_lane_point.Valid()) {
    AnalyzePrimaryPathSemantic(path_point, high_priority_lanes, other_lanes,
                               psmm, path_semantic);
    return;
  }
  if (prev_primary_path_semantic.closest_lane_point.lane_id() !=
      next_primary_path_semantic.closest_lane_point.lane_id()) {
    AnalyzePrimaryPathSemantic(path_point, high_priority_lanes, other_lanes,
                               psmm, path_semantic);
    return;
  }
  const mapping::LanePoint closest_lane_point(
      prev_primary_path_semantic.closest_lane_point.lane_id(),
      Lerp(prev_primary_path_semantic.closest_lane_point.fraction(),
           next_primary_path_semantic.closest_lane_point.fraction(), ratio));
  path_semantic->deviation_distance =
      Lerp(prev_primary_path_semantic.deviation_distance,
           next_primary_path_semantic.deviation_distance, ratio);
  const auto closest_lane_point_pos =
      ComputeLanePointPos(psmm, closest_lane_point);
  path_semantic->closest_lane_point = closest_lane_point;
  path_semantic->closest_lane_point_pos = closest_lane_point_pos;
  path_semantic->lane_semantic = prev_primary_path_semantic.lane_semantic;
  path_semantic->lane_info = prev_primary_path_semantic.lane_info;
  return;
}

}  // namespace

absl::StatusOr<std::vector<PathPointSemantic>> AnalyzePathSemantics(
    int plan_id, const DiscretizedPath& path, int max_analyze_path_index,
    const PlannerSemanticMapManager& psmm, const DrivePassage* drive_passage,
    const DrivingMapTopo* driving_map_topo, ThreadPool* thread_pool) {
  // ("AnalyzePathSemantics");
  TIMELINE("AnalyzePathSemantics");
  CHECK(!path.empty());
  CHECK_LT(max_analyze_path_index, path.size());

  int path_semantic_size = max_analyze_path_index + 1;
  std::vector<PathPointSemantic> path_semantics(path_semantic_size);
  if (!psmm.map_ptr()) {
    return absl::NotFoundError("map_ptr is nullptr!");
  } else if (!psmm.map_ptr()->route()) {
    return absl::NotFoundError("route is nullptr!");
  }

  absl::flat_hash_set<mapping::ElementId> high_priority_lanes_id;
  std::vector<RouteLaneInfo> high_priority_lanes;
  if (nullptr != drive_passage) {
    const auto& lane_path_data = drive_passage->lane_path().lane_path_data();
    const auto& lane_ids = lane_path_data.lane_ids();
    high_priority_lanes.reserve(lane_ids.size());
    high_priority_lanes_id.reserve(lane_ids.size());
    bool is_first = true;
    for (const auto lane_id : lane_ids) {
      const auto& lane = psmm.FindCurveLaneByIdOrNull(lane_id);
      RouteLaneInfo route_lane_info;
      route_lane_info.lane = lane;
      route_lane_info.start_fraction =
          is_first ? lane_path_data.start_fraction() : 0.0;
      is_first = false;
      route_lane_info.end_fraction = 1.0;
      high_priority_lanes.push_back(route_lane_info);
      high_priority_lanes_id.insert(lane_id);
    }
  }

  std::vector<RouteLaneInfo> other_lanes;
  const auto& route_info = psmm.map_ptr()->route()->GetRouteInfo();
  const auto& navi_start = route_info.navi_start;
  for (const auto& section : route_info.sections) {
    for (const auto& lane_id : section.lane_ids) {
      if (high_priority_lanes_id.contains(lane_id)) {
        continue;
      }
      const auto& lane = psmm.FindCurveLaneByIdOrNull(lane_id);
      if (lane != nullptr) {
        RouteLaneInfo route_lane_info;
        route_lane_info.lane = lane;
        double start_fraction = 0.0;
        if (section.id == navi_start.section_id) {
          if (const auto ff = BuildBruteForceFrenetFrame(
                  lane->points(), /*down_sample_raw_points=*/false);
              ff.ok()) {
            start_fraction =
                std::clamp(ff->XYToSL(ToVec2d(path.front())).s /
                               std::fmax(lane->curve_length(), kEps),
                           0.0, 1.0 - kEps);
          }
        }
        route_lane_info.start_fraction = start_fraction;
        route_lane_info.end_fraction = 1.0;
        other_lanes.push_back(route_lane_info);
      }
    }
  }

  // const auto& lanes = driving_map_topo->lanes();
  constexpr double kPrimaryPathSampleInterval = 1.0;  // m.
  std::vector<int> primary_indices;
  std::unordered_set<int> primary_index_set;
  primary_indices.reserve(
      CeilToInt(path[max_analyze_path_index].s() / kPrimaryPathSampleInterval) +
      1);
  primary_index_set.reserve(primary_indices.capacity());
  double last_primary_s = 0.0;
  for (int i = 0; i < path_semantic_size; ++i) {
    if (i == 0 || i + 1 == path_semantic_size ||
        path[i].s() - last_primary_s > kPrimaryPathSampleInterval - kEps) {
      last_primary_s = path[i].s();
      primary_indices.push_back(i);
      primary_index_set.insert(i);
    }
  }

  std::optional<int> first_beyond_real_map_path_idx;
  ParallelFor(0, primary_indices.size(), thread_pool, [&](int i) {
    const int idx = primary_indices[i];
    if (first_beyond_real_map_path_idx.has_value()) {
      FillExtendedPartPathSemantics(path[idx], &path_semantics[idx]);
    } else if (const auto nearest_station_index =
                   drive_passage->FindNearestStationIndex(ToVec2d(path[idx]));
               nearest_station_index >
               drive_passage->last_real_station_index()) {
      FillExtendedPartPathSemantics(path[idx], &path_semantics[idx]);
      first_beyond_real_map_path_idx = idx;
    } else {
      AnalyzePrimaryPathSemantic(path[idx], high_priority_lanes, other_lanes,
                                 psmm, &path_semantics[idx]);
    }
  });

  ParallelFor(0, path_semantic_size, thread_pool, [&](int i) {
    if (ContainsKey(primary_index_set, i)) {
      return;
    }
    int idx = FloorToInt(path[i].s() / kPrimaryPathSampleInterval);
    while (idx + 1 > primary_indices.size() || primary_indices[idx] > i) {
      --idx;
    }
    while (idx + 1 < primary_indices.size() && primary_indices[idx + 1] < i) {
      ++idx;
    }
    const int prev_primary_idx = primary_indices[idx];
    const int next_primary_idx = primary_indices[idx + 1];
    if (first_beyond_real_map_path_idx.has_value() &&
        (prev_primary_idx >= *first_beyond_real_map_path_idx ||
         next_primary_idx >= *first_beyond_real_map_path_idx)) {
      FillExtendedPartPathSemantics(path[idx], &path_semantics[i]);
    } else {
      const double ratio = LerpFactor(path[prev_primary_idx].s(),
                                      path[next_primary_idx].s(), path[i].s());
      AnalyzeSecondaryPathSemantic(path[i], high_priority_lanes, other_lanes,
                                   psmm, path_semantics[prev_primary_idx],
                                   path_semantics[next_primary_idx], ratio,
                                   &path_semantics[i]);
    }
  });

  if (path_semantic_size == 0) {
    return absl::NotFoundError("Failed to analyze path semantics!");
  }
  // Set lane path id. If two adjacent closest lane points have a larger squared
  // distance than this, we consider that a lane change happens.
  constexpr double kClosestLanePointSqrDistThres = Sqr(2.8);  // m^2.
  std::vector<int> curr_lane_path_id_history = {0};
  path_semantics[0].lane_path_id_history = curr_lane_path_id_history;
  for (int i = 1; i < path_semantic_size; ++i) {
    const auto& prev_lane_point = path_semantics[i - 1].closest_lane_point;
    const auto& curr_lane_point = path_semantics[i].closest_lane_point;
    const auto& prev_lane_pos = path_semantics[i - 1].closest_lane_point_pos;
    const auto& curr_lane_pos = path_semantics[i].closest_lane_point_pos;
    if (prev_lane_point.Valid() && curr_lane_point.Valid() &&
        curr_lane_point.lane_id() != prev_lane_point.lane_id() &&
        curr_lane_pos.DistanceSquareTo(prev_lane_pos) >
            kClosestLanePointSqrDistThres) {
      const auto closest_lane_point_tangent =
          ComputeLanePointTangent(psmm, curr_lane_point);
      const int back_history = curr_lane_path_id_history.back();
      if (closest_lane_point_tangent.CrossProd(curr_lane_pos - prev_lane_pos) >
          0.0) {
        // Lane change left.
        curr_lane_path_id_history.push_back(back_history + 1);
      } else {
        // Lane change right.
        curr_lane_path_id_history.push_back(back_history - 1);
      }
    }
    path_semantics[i].lane_path_id_history = curr_lane_path_id_history;
  }

  return path_semantics;
}

}  // namespace planning
}  // namespace st
