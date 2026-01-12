

#include "planner/trajectory_optimizer/ddp/static_boundary_cost_util.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_set>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "plan_common/container/strong_int.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"
//#include "global/buffered_logger.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/aabox3d.pb.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "planner/trajectory_optimizer/problem/mfob_path_boundary_cost.h"
#include "planner/trajectory_optimizer/problem/msd_static_boundary_cost.h"
#include "planner/trajectory_optimizer/problem/msd_static_boundary_cost_v2.h"
#include "plan_common/math/discretized_path.h"
#include "planner/planner_manager/planner_defs.h"
//#include "semantic_map.pb.h"
#include "plan_common/util/min_segment_distance_problem.h"
#include "plan_common/util/qtfm_segment_matcher_v2.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "plan_common/log_data.h"

DEFINE_bool(msd_static_boundary_cost_v2, true,
            "V2 version of msd_static_boundary_cost, the curb buffer is speed "
            "variant.");

DEFINE_bool(trajectory_optimizer_ignore_u_turn_right_boundary, true,
            "Whether to ignore right boundary of u turn.");

namespace st {
namespace planning {
namespace optimizer {
namespace {

// NOLINTNEXTLINE
void CollectCurbSegmentsAroundDrivePassage(
    bool consider_mirrors_by_default,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectoryPoint& plan_start_point, const double start_point_s,
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const std::function<std::tuple<double, double, double>(const Vec2d&)>&
        query_extra_buffer,
    const std::optional<double>& u_turn_middle_s,
    std::vector<MsdProblemWithBuffer::SegmentType>*
        below_mirror_curb_named_segments,
    std::vector<int>* below_mirror_curb_start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType>*
        above_mirror_curb_named_segments,
    std::vector<int>* above_mirror_curb_start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType>* uturn_right_named_segments,
    std::vector<int>* uturn_right_start_segment_ids,
    bool* uturn_curb_consider_mirrors, double* nearest_curb_distance) {
  CHECK_NOTNULL(below_mirror_curb_named_segments);
  CHECK_NOTNULL(below_mirror_curb_start_segment_ids);
  CHECK_NOTNULL(above_mirror_curb_named_segments);
  CHECK_NOTNULL(above_mirror_curb_start_segment_ids);
  CHECK_NOTNULL(uturn_right_named_segments);
  CHECK_NOTNULL(uturn_right_start_segment_ids);
  CHECK_NOTNULL(uturn_curb_consider_mirrors);
  CHECK_NOTNULL(nearest_curb_distance);
  ad_byd::planning::JunctionConstPtr nearest_junction_ptr = nullptr;
  constexpr double kMaxDsThreshold = 16.0;
  constexpr double kDsStep = 5.0;
  for (double s = start_point_s; s < start_point_s + kMaxDsThreshold;
       s += kDsStep) {
    const auto& cur_station = drive_passage.FindNearestStationAtS(s);
    if (cur_station.is_in_intersection() && cur_station.is_virtual()) {
      const auto junction_ptr = psmm.GetNearestJunction(cur_station.xy());
      if (junction_ptr != nullptr && junction_ptr->IsValid()) {
        nearest_junction_ptr = junction_ptr;
        break;
      }
    }
  }

  constexpr double kSearchRadiusBuffer = 0.1;
  constexpr double kSampleStep = 2.0;
  std::unordered_set<mapping::ElementId> ignored_curb;
  const double last_real_s =
      drive_passage
          .station(
              StationIndex(drive_passage.last_real_station_index().value()))
          .accumulated_s();
  for (double s = last_real_s; s < drive_passage.end_s(); s += kSampleStep) {
    const auto p0 = drive_passage.QueryPointXYAtS(s);
    const auto p1 = drive_passage.QueryPointXYAtS(
        std::min(s + kSampleStep, drive_passage.end_s()));
    if (!p0.ok() || !p1.ok()) {
      break;
    }
    const double search_radius = kSearchRadiusBuffer + kSampleStep * 0.5;
    const Vec2d search_center = 0.5 * (p0.value() + p1.value());
    const std::vector<ad_byd::planning::RoadBoundaryConstPtr>
        candidate_boundaries =
            psmm.GetRoadBoundaries(search_center, search_radius);
    for (const auto& candidate_boundary : candidate_boundaries) {
      ignored_curb.insert(candidate_boundary->id());
    }
  }
  // Collect curb boundaries around drive passage without repeated ones.
  std::unordered_set<mapping::ElementId> curb_boundaries;
  for (int i = 1; i < drive_passage.size(); ++i) {
    // Don't collect curb for extended drive passage.
    // TODO: Cut curb boundaries outside lane keeping drive passage.
    if (i > drive_passage.last_real_station_index().value()) {
      break;
    }
    const Vec2d p0 = drive_passage.station(StationIndex(i - 1)).xy();
    const Vec2d& p1 = drive_passage.station(StationIndex(i)).xy();
    const double search_radius = kMaxLateralOffset + (p1 - p0).Length() * 0.5;
    const Vec2d search_center = 0.5 * (p0 + p1);
    const std::vector<ad_byd::planning::RoadBoundaryConstPtr>
        candidate_boundaries =
            psmm.GetRoadBoundaries(search_center, search_radius);
    for (const auto& candidate_boundary : candidate_boundaries) {
      if (ignored_curb.find(candidate_boundary->id()) != ignored_curb.end()) {
        continue;
      }
      curb_boundaries.insert(candidate_boundary->id());
    }
  }

  const auto is_on_uturn_right = [&drive_passage,
                                  &u_turn_middle_s](const Segment2d& boundary) {
    if (!u_turn_middle_s.has_value()) return false;
    const auto start_frenet_or =
        drive_passage.QueryUnboundedFrenetCoordinateAt(boundary.start());
    if (!start_frenet_or.ok()) return false;
    const auto end_frenet_or =
        drive_passage.QueryUnboundedFrenetCoordinateAt(boundary.end());
    if (!end_frenet_or.ok()) return false;
    if ((start_frenet_or->s >= *u_turn_middle_s && start_frenet_or->l < 0.0) ||
        (end_frenet_or->s >= *u_turn_middle_s && end_frenet_or->l < 0.0)) {
      return true;
    }
    return false;
  };

  // Mirror info.
  const bool has_mirror =
      veh_geo_params.has_left_mirror() && veh_geo_params.has_right_mirror();
  const double mirror_height_avg =
      ((veh_geo_params.left_mirror().z() -
        veh_geo_params.left_mirror().height() * 0.5) +
       (veh_geo_params.right_mirror().z() -
        veh_geo_params.right_mirror().height() * 0.5)) *
      0.5;

  // Emplace to outputs.
  above_mirror_curb_named_segments->clear();
  above_mirror_curb_start_segment_ids->clear();
  below_mirror_curb_named_segments->clear();
  below_mirror_curb_start_segment_ids->clear();
  uturn_right_named_segments->clear();
  uturn_right_start_segment_ids->clear();
  for (const auto& id : curb_boundaries) {
    const auto& road_boundary = psmm.FindRoadBoundaryByIdOrNull(id);
    if (road_boundary == nullptr) continue;
    const bool consider_mirrors =
        has_mirror && (road_boundary->has_height()
                           ? (road_boundary->height() > mirror_height_avg)
                           : consider_mirrors_by_default);
    const std::vector<Vec2d>& points = road_boundary->points();
    if (points.empty()) {
      continue;
    }

    bool has_no_prev_segment = true;
    bool uturn_right_has_no_prev_segment = true;
    Vec2d prev = points.front();
    for (int i = 1; i < points.size(); ++i) {
      // Skip short segments.
      if (prev.DistanceTo(points[i]) <=
          QtfmSegmentMatcherV2::kMinSegmentLength) {
        continue;
      }

      const Segment2d cur_seg(prev, points[i]);
      double extra_buffer =
          std::min(std::get<0>(query_extra_buffer(cur_seg.start())),
                   std::get<0>(query_extra_buffer(cur_seg.end())));

      const double lane_width =
          std::min(std::get<1>(query_extra_buffer(cur_seg.start())),
                   std::get<1>(query_extra_buffer(cur_seg.end())));

      const double length =
          std::min(std::get<2>(query_extra_buffer(cur_seg.start())),
                   std::get<2>(query_extra_buffer(cur_seg.end())));

      if (is_on_uturn_right(cur_seg)) {
        has_no_prev_segment = true;
        // Append a segment.
        uturn_right_named_segments->push_back(
            {absl::StrCat("id:", id, ",seg:", i), cur_seg, extra_buffer,
             lane_width, length});
        if (uturn_right_has_no_prev_segment) {
          uturn_right_start_segment_ids->push_back(
              uturn_right_named_segments->size() - 1);
          uturn_right_has_no_prev_segment = false;
        }
        *uturn_curb_consider_mirrors |= consider_mirrors;
      } else {
        uturn_right_has_no_prev_segment = true;

        constexpr double kIntersectionExtraBufferBase = 0.2;
        constexpr double kHalfDsStep = kDsStep / 2.0;
        if (nearest_junction_ptr != nullptr &&
            (nearest_junction_ptr->DistanceTo(cur_seg.start()) < kHalfDsStep ||
             nearest_junction_ptr->DistanceTo(cur_seg.end()) < kHalfDsStep)) {
          extra_buffer += kIntersectionExtraBufferBase;
        }
        // Append a segment.
        consider_mirrors ? above_mirror_curb_named_segments->push_back(
                               {absl::StrCat("id:", id, ",seg:", i), cur_seg,
                                extra_buffer, lane_width, length})
                         : below_mirror_curb_named_segments->push_back(
                               {absl::StrCat("id:", id, ",seg:", i), cur_seg,
                                extra_buffer, lane_width, length});
        if (has_no_prev_segment) {
          consider_mirrors ? above_mirror_curb_start_segment_ids->push_back(
                                 above_mirror_curb_named_segments->size() - 1)
                           : below_mirror_curb_start_segment_ids->push_back(
                                 below_mirror_curb_named_segments->size() - 1);
          has_no_prev_segment = false;
        }
      }
      prev = points[i];
    }
  }
}

void CollectExtendSolidLinesWithinPathBoundary(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const ConstraintManager& constraint_manager,
    const std::vector<TrajectoryPoint>& smooth_init_traj,
    double solid_line_buffer, double extend_distance,
    std::vector<int>* solid_line_start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType>* solid_line_segments) {
  // Check if solid line has overlap with trajectory.
  constexpr double kVehicleBoxHalfWidth = 0.2;  // m.
  const double lateral_buffer =
      kVehicleBoxHalfWidth - 0.5 * vehicle_geometry_params.width();
  std::vector<Box2d> av_boxes;
  av_boxes.reserve(smooth_init_traj.size());
  for (const auto& pt : smooth_init_traj) {
    av_boxes.push_back(
        ComputeAvBoxWithBuffer(pt.pos(), pt.theta(), vehicle_geometry_params,
                               /*length_buffer=*/0.0, lateral_buffer));
  }
  const auto has_overlap_with_solid_line =
      [&av_boxes](const std::vector<Vec2d>& points) {
        for (int i = 0; i + 1 < points.size(); ++i) {
          const Segment2d segment(points[i], points[i + 1]);
          for (const auto& box : av_boxes) {
            if (box.HasOverlap(segment)) return true;
          }
        }
        return false;
      };

  const Vec2d start_pos = smooth_init_traj.front().pos();
  const double start_theta = smooth_init_traj.front().theta();

  solid_line_start_segment_ids->clear();
  solid_line_segments->clear();
  for (const auto& avoid_line : constraint_manager.AvoidLine()) {
    if (!avoid_line.source().has_solid_line_within_boundary()) continue;
    std::vector<Vec2d> solid_line;
    solid_line.reserve(avoid_line.xy_points().size() + 2);
    for (const auto& pt : avoid_line.xy_points()) {
      solid_line.emplace_back(pt.x(), pt.y());
    }
    if (has_overlap_with_solid_line(solid_line)) continue;

    // Extend at start and end.
    CHECK_GT(solid_line.size(), 1);
    const Segment2d seg1(solid_line[1], solid_line[0]);
    solid_line.insert(solid_line.begin(),
                      seg1.end() + seg1.unit_direction() * extend_distance);
    const Segment2d seg2(solid_line[solid_line.size() - 2], solid_line.back());
    solid_line.insert(solid_line.end(),
                      seg2.end() + seg2.unit_direction() * extend_distance);

    bool has_no_prev_segment = true;
    Vec2d prev = solid_line.front();
    for (int i = 1; i < solid_line.size(); ++i) {
      // Skip short segments.
      if (prev.DistanceTo(solid_line[i]) <=
          QtfmSegmentMatcherV2::kMinSegmentLength) {
        continue;
      }
      const Segment2d line(prev, solid_line[i]);
      double extra_buffer = 0.0;  // m.
      for (const auto& circle :
           trajectory_optimizer_vehicle_model_params.circles()) {
        const Vec2d center =
            start_pos +
            Vec2d::FastUnitFromAngleN12(start_theta + circle.angle_to_axis()) *
                circle.dist_to_rac();
        const double dist = line.DistanceTo(center) - circle.radius();
        extra_buffer = std::min(extra_buffer, dist - solid_line_buffer);
      }
      for (const auto& circle :
           trajectory_optimizer_vehicle_model_params.mirror_circles()) {
        const Vec2d center =
            start_pos +
            Vec2d::FastUnitFromAngleN12(start_theta + circle.angle_to_axis()) *
                circle.dist_to_rac();
        const double dist = line.DistanceTo(center) - circle.radius();
        extra_buffer = std::min(extra_buffer, dist - solid_line_buffer);
      }

      // Append a segment.
      solid_line_segments->push_back(
          {absl::StrCat("id:", avoid_line.id(), ",seg:", i), line,
           extra_buffer});
      prev = solid_line[i];

      if (has_no_prev_segment) {
        solid_line_start_segment_ids->push_back(solid_line_segments->size() -
                                                1);
        has_no_prev_segment = false;
      }
    }
  }
}

void GeneratePathBoundariesWithRightBoundaryAfterUTurnIgnored(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    std::vector<double>* left_dists_to_curb,
    std::vector<double>* right_dists_to_curb,
    std::vector<double>* left_dists_to_path_boundary,
    std::vector<double>* right_dists_to_path_boundary,
    std::vector<double>* left_dists_to_target_path_boundary,
    std::vector<double>* right_dists_to_target_path_boundary,
    std::vector<double>* reference_center_l_offsets,
    std::optional<double>* uturn_middle_s) {
  // Get first uturn middle s.
  bool u_turn_passed = false;
  double u_turn_start_s = 0.0;
  double u_turn_end_s = 0.0;
  for (const auto& station : drive_passage.stations()) {
    if (station.turn_type() == ad_byd::planning::U_TURN) {
      if (!u_turn_passed) {
        u_turn_start_s = station.accumulated_s();
        u_turn_end_s = station.accumulated_s();
        u_turn_passed = true;
      } else {
        u_turn_end_s = station.accumulated_s();
      }
    } else if (u_turn_passed) {
      break;
    }
  }
  if (u_turn_passed) {
    *uturn_middle_s = 0.5 * (u_turn_start_s + u_turn_end_s);
  }

  // Extend boundary during uturn.
  // Ignore right curb from uturn middle s.
  constexpr double kBoundaryExtendDistOnUTurn = 20.0;  // m.
  constexpr double kEps = 1e-6;
  const auto drive_passage_size = drive_passage.stations().size();
  left_dists_to_curb->reserve(drive_passage_size);
  right_dists_to_curb->reserve(drive_passage_size);
  for (int i = 0; i < drive_passage.stations().size(); ++i) {
    const auto& station = drive_passage.stations()[StationIndex(i)];
    const auto curb_pair_or = station.QueryCurbOffsetAt(/*signed_lat=*/0.0);
    // CHECK_OK(curb_pair_or.status());
    left_dists_to_curb->push_back(curb_pair_or->second);
    if (uturn_middle_s->has_value() &&
        station.accumulated_s() + kEps >= **uturn_middle_s) {
      right_dists_to_curb->push_back(-curb_pair_or->first +
                                     kBoundaryExtendDistOnUTurn);
    } else {
      right_dists_to_curb->push_back(-curb_pair_or->first);
    }
  }

  // Ignore right path boundary from uturn middle s.
  const auto path_boundary_size = path_sl_boundary.size();
  left_dists_to_path_boundary->reserve(path_boundary_size);
  left_dists_to_target_path_boundary->reserve(path_boundary_size);
  right_dists_to_path_boundary->reserve(path_boundary_size);
  right_dists_to_target_path_boundary->reserve(path_boundary_size);
  reference_center_l_offsets->reserve(path_boundary_size);
  CHECK_GE(drive_passage.stations().size(), path_boundary_size);
  for (int i = 0; i < path_boundary_size; ++i) {
    left_dists_to_path_boundary->push_back(path_sl_boundary.left_l_vector()[i]);
    left_dists_to_target_path_boundary->push_back(
        path_sl_boundary.target_left_l_vector()[i]);
    if (uturn_middle_s->has_value() &&
        drive_passage.stations()[StationIndex(i)].accumulated_s() + kEps >=
            **uturn_middle_s) {
      right_dists_to_path_boundary->push_back(
          -path_sl_boundary.right_l_vector()[i] + kBoundaryExtendDistOnUTurn);
      right_dists_to_target_path_boundary->push_back(
          -path_sl_boundary.target_right_l_vector()[i]);
    } else {
      right_dists_to_path_boundary->push_back(
          -path_sl_boundary.right_l_vector()[i]);
      right_dists_to_target_path_boundary->push_back(
          -path_sl_boundary.target_right_l_vector()[i]);
    }
    reference_center_l_offsets->push_back(
        path_sl_boundary.reference_center_l_vector()[i]);
  }
}

void GeneratePathBoundaries(
    const DrivePassage& drive_passage, const NudgeInfos& nudge_info,
    const PathSlBoundary& path_sl_boundary,
    std::vector<double>* left_dists_to_curb,
    std::vector<double>* right_dists_to_curb,
    std::vector<double>* left_dists_to_path_boundary,
    std::vector<double>* right_dists_to_path_boundary,
    std::vector<double>* left_dists_to_target_path_boundary,
    std::vector<double>* right_dists_to_target_path_boundary,
    std::vector<double>* reference_center_l_offsets) {
  const auto drive_passage_size = drive_passage.stations().size();
  left_dists_to_curb->reserve(drive_passage_size);
  right_dists_to_curb->reserve(drive_passage_size);
  for (int i = 0; i < drive_passage.stations().size(); ++i) {
    const auto& station = drive_passage.stations()[StationIndex(i)];
    const auto curb_pair_or = station.QueryCurbOffsetAt(/*signed_lat=*/0.0);
    // CHECK_OK(curb_pair_or.status());
    left_dists_to_curb->push_back(curb_pair_or->second);
    right_dists_to_curb->push_back(-curb_pair_or->first);
  }

  const auto path_boundary_size = path_sl_boundary.size();
  left_dists_to_path_boundary->reserve(path_boundary_size);
  left_dists_to_target_path_boundary->reserve(path_boundary_size);
  right_dists_to_path_boundary->reserve(path_boundary_size);
  right_dists_to_target_path_boundary->reserve(path_boundary_size);
  reference_center_l_offsets->reserve(path_boundary_size);
  CHECK_GE(drive_passage.stations().size(), path_boundary_size);

  bool nudge_left = false;
  bool nudge_right = false;

  // if (!nudge_info.empty()) {
  //   nudge_left = nudge_info.front().direction == 1 ? true : false;    // 左
  //   nudge_right = nudge_info.front().direction == -1 ? true : false;  // 左
  // }

  for (int i = 0; i < path_boundary_size; ++i) {
    left_dists_to_path_boundary->push_back(path_sl_boundary.left_l_vector()[i]);
    if (nudge_left) {
      left_dists_to_target_path_boundary->push_back(
          path_sl_boundary.left_l_vector()[i]);
    } else {
      left_dists_to_target_path_boundary->push_back(
          path_sl_boundary.target_left_l_vector()[i]);
    }

    right_dists_to_path_boundary->push_back(
        -path_sl_boundary.right_l_vector()[i]);
    if (nudge_right) {
      right_dists_to_target_path_boundary->push_back(
          -path_sl_boundary.right_l_vector()[i]);
    } else {
      right_dists_to_target_path_boundary->push_back(
          -path_sl_boundary.target_right_l_vector()[i]);
    }
    reference_center_l_offsets->push_back(
        path_sl_boundary.reference_center_l_vector()[i]);
  }
}

void AddMsdStaticBoundaryCost(
    int trajectory_steps, std::string_view base_name,
    std::string_view source_name,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const TrajectoryPoint& plan_start_point,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    bool consider_mirrors, const std::vector<int>& start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType> named_segments,
    std::vector<std::string> sub_names, std::vector<double> cascade_buffers,
    std::vector<double> cascade_gains,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  constexpr double kCutoffDistExtraBuffer = 0.1;
  const double cutoff_distance =
      *std::max_element(cascade_buffers.begin(), cascade_buffers.end()) +
      0.5 * veh_geo_params.width() + kCutoffDistExtraBuffer;
  MsdProblemWithBuffer msd(std::move(named_segments), cutoff_distance,
                           start_segment_ids);

  std::vector<Vec2d> circle_center_offsets;
  std::vector<double> circle_radiuses;
  int circle_size = trajectory_optimizer_vehicle_model_params.circles_size();
  if (consider_mirrors) {
    circle_size +=
        trajectory_optimizer_vehicle_model_params.mirror_circles_size();
  }
  circle_center_offsets.reserve(circle_size);
  circle_radiuses.reserve(circle_size);
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    circle_center_offsets.push_back(
        Vec2d(circle.dist_to_rac() * std::cos(circle.angle_to_axis()),
              circle.dist_to_rac() * std::sin(circle.angle_to_axis())));
    circle_radiuses.push_back(circle.radius());
  }
  if (consider_mirrors) {
    for (const auto& circle :
         trajectory_optimizer_vehicle_model_params.mirror_circles()) {
      circle_center_offsets.push_back(
          Vec2d(circle.dist_to_rac() * std::cos(circle.angle_to_axis()),
                circle.dist_to_rac() * std::sin(circle.angle_to_axis())));
      circle_radiuses.push_back(circle.radius());
    }
  }

  costs->emplace_back(std::make_unique<MsdStaticBoundaryCost<Mfob>>(
      trajectory_steps, veh_geo_params, std::move(msd),
      stations_query_helper.get(), std::move(sub_names),
      std::move(cascade_buffers), std::move(cascade_gains),
      circle_center_offsets, circle_radiuses,
      /*effect_index=*/trajectory_steps,
      /*ignore_invasion_second_order_derivative=*/true, "MsdStaticBoundaryCost",
      cost_weight_params.msd_static_boundary_cost_weight(),
      /*cost_type=*/source_name == "solid_line"
          ? Cost<Mfob>::CostType::SOLID_LINE_MSD_STATIC_BOUNDARY
          : Cost<Mfob>::CostType::MUST_HAVE));
}

// Don't use this function for solid line!
// It is not supposed to do so.
void AddMsdStaticBoundaryCostV2(
    int trajectory_steps, std::string_view base_name,
    std::string_view source_name, double source_scale,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const TrajectoryPoint& plan_start_point,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    bool consider_mirrors, const std::vector<int>& start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType> named_segments,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    const std::optional<double>& lane_width_curb_buffer_opt,
    const std::optional<double>& current_curb_interval,
    const std::optional<double>& speed_rel_hard_curb_interval,
    const double lane_width, std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  CHECK_GT(trajectory_steps, 0);
  double delta_dist =
      speed_rel_hard_curb_interval.value() - current_curb_interval.value();
  double decay_scale = delta_dist < 0 ? 1.0 : delta_dist * 0.1;
  Log2DDS::LogDataV0("curb_debug", absl::StrCat("delta_dist: ", delta_dist));
  Log2DDS::LogDataV0("curb_debug", absl::StrCat("decay_scale: ", decay_scale));
  decay_scale = 1;

  // auto speed_rel_hard_curb_clearance_plf =
  // PiecewiseLinearFunctionFromProto(
  //     cost_weight_params.speed_rel_hard_curb_clearance_plf());
  auto lane_width_curb_scale = PiecewiseLinearFunctionFromProto(
      cost_weight_params.lane_width_curb_scale());

  double lane_width_scale = lane_width_curb_scale(lane_width);

  Log2DDS::LogDataV2("plan_start_point_curb_offset",
                     absl::StrCat("lane_width_scale: ", lane_width_scale));

  auto speed_rel_hard_curb_clearance_plf =
      PiecewiseLinearFunctionFromProtoScale(
          cost_weight_params.speed_rel_hard_curb_clearance_plf(),
          lane_width_scale);

  Log2DDS::LogDataV2("plan_start_point_curb_offset",
                     absl::StrCat("speed_rel_hard_curb_clearance_plf: ",
                                  speed_rel_hard_curb_clearance_plf(
                                      std::abs(plan_start_point.v()))));

  // auto speed_rel_soft_curb_clearance_plf = PiecewiseLinearFunctionFromProto(
  //     cost_weight_params.speed_rel_soft_curb_clearance_plf());

  auto speed_rel_soft_curb_clearance_plf =
      PiecewiseLinearFunctionFromProtoScale(
          cost_weight_params.speed_rel_soft_curb_clearance_plf(),
          lane_width_scale);
  Log2DDS::LogDataV2("plan_start_point_curb_offset",
                     absl::StrCat("speed_rel_soft_curb_clearance_plf: ",
                                  speed_rel_soft_curb_clearance_plf(
                                      std::abs(plan_start_point.v()))));

  double target_curb_interval = current_curb_interval.value();

  // double max_target_curb_interval = std::max(
  //     lane_width_curb_buffer_opt.value(),
  //     speed_rel_hard_curb_interval.value());
  // if (lane_width_curb_buffer_opt.value() != 0)
  //   max_target_curb_interval = lane_width_curb_buffer_opt.value();

  // bool need_change_buffer = false;
  // if (max_target_curb_interval - current_curb_interval.value() > 0.005) {
  //   need_change_buffer = true;
  //   target_curb_interval += 0.005;
  // } else {
  //   target_curb_interval = max_target_curb_interval;
  // }

  Log2DDS::LogDataV2(
      "plan_start_point_curb_offset",
      absl::StrCat(
          "current_curb_interval: ", current_curb_interval.value(),
          ", lane_width_curb_buffer_opt: ", lane_width_curb_buffer_opt.value(),
          ", speed_rel_hard_curb_interval: ",
          speed_rel_hard_curb_interval.value()));

  // Hard layer.
  std::vector<double> x{2.0, 5.0, 10.0, 30.0};
  std::vector<double> y(x.size(), lane_width_curb_buffer_opt.value());
  PiecewiseLinearFunction<double, double, Lerper<double, double>>
      target_curb_buffer(std::move(x), std::move(y));
  MsdStaticBoundaryCostV2<Mfob>::Layer hard_layer{
      .gain_of_weight = cost_weight_params.static_boundary_hard_cost_weight(),
      .buffer_at_speed = speed_rel_hard_curb_clearance_plf/*lane_width_curb_buffer_opt.value() == 0.0
                             ? speed_rel_hard_curb_clearance_plf
                             : target_curb_buffer*/};
  // .buffer_at_speed = lane_width_curb_buffer_opt.value() != 0
  //                        ? target_curb_buffer
  //                        : speed_rel_hard_curb_clearance_plf};

  // Soft layer.
  MsdStaticBoundaryCostV2<Mfob>::Layer soft_layer{
      .gain_of_weight = cost_weight_params.static_boundary_soft_cost_weight(),
      .buffer_at_speed = speed_rel_soft_curb_clearance_plf/*lane_width_curb_buffer_opt.value() == 0.2
                             ? target_curb_buffer
                             : speed_rel_soft_curb_clearance_plf*/};

  // Vehicle shape.
  std::vector<Vec2d> circle_center_offsets;
  std::vector<double> circle_radiuses;
  int circle_size = trajectory_optimizer_vehicle_model_params.circles_size();
  if (consider_mirrors) {
    circle_size +=
        trajectory_optimizer_vehicle_model_params.mirror_circles_size();
  }
  circle_center_offsets.reserve(circle_size);
  circle_radiuses.reserve(circle_size);
  double max_circle_radius = 0.0;
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    circle_center_offsets.push_back(
        Vec2d(circle.dist_to_rac() * std::cos(circle.angle_to_axis()),
              circle.dist_to_rac() * std::sin(circle.angle_to_axis())));
    circle_radiuses.push_back(circle.radius());
    max_circle_radius = std::max(max_circle_radius, circle.radius());
  }
  if (consider_mirrors) {
    for (const auto& circle :
         trajectory_optimizer_vehicle_model_params.mirror_circles()) {
      circle_center_offsets.push_back(
          Vec2d(circle.dist_to_rac() * std::cos(circle.angle_to_axis()),
                circle.dist_to_rac() * std::sin(circle.angle_to_axis())));
      circle_radiuses.push_back(circle.radius());
      max_circle_radius = std::max(max_circle_radius, circle.radius());
    }
  }

  // Build msd.
  constexpr double kCutoffDistExtraBuffer = 0.1;
  double max_buffer = std::max(hard_layer.buffer_at_speed.y().back(),
                               soft_layer.buffer_at_speed.y().back());
  const double cutoff_distance =
      max_buffer + max_circle_radius + kCutoffDistExtraBuffer;
  MsdProblemWithBuffer msd(std::move(named_segments), cutoff_distance,
                           start_segment_ids);

  // chaneg decay time by current speed
  int decay_time = plan_start_point.v() < 5 ? 1 : 10;
  Log2DDS::LogDataV0("curb_debug", absl::StrCat("decay_time: ", decay_time));

  // double lane_width_scale = lane_width / 3.75;

  // if(decay_scale < lane_width_scale) {
  //   // decay_scale = 20;
  //   decay_time = 20;
  // }

  costs->emplace_back(std::make_unique<MsdStaticBoundaryCostV2<Mfob>>(
      trajectory_steps, decay_time, decay_scale, std::move(msd),
      stations_query_helper.get(),
      std::vector<std::string>{SoftNameString, HardNameString},
      std::vector<MsdStaticBoundaryCostV2<Mfob>::Layer>{std::move(soft_layer),
                                                        std::move(hard_layer)},
      std::move(circle_center_offsets), std::move(circle_radiuses),
      /*use_hessian_approximate=*/true,
      /*name=*/
      absl::StrCat("MsdStaticBoundaryCostV2_", std::string(source_name)),
      cost_weight_params.msd_static_boundary_cost_weight() * source_scale,
      /*cost_type=*/source_name == "uturn_right_curb"
          ? Cost<Mfob>::CostType::UTURN_RIGHT_CURB_MSD_STATIC_BOUNDARY_V2
          : Cost<Mfob>::CostType::CURB_MSD_STATIC_BOUNDARY_V2));
}

}  // namespace

// NOLINTNEXTLINE
void AddStaticBoundaryCosts(
    LaneChangeStage lc_stage, int trajectory_steps, std::string_view base_name,
    bool enable_three_point_turn, const TrajectoryPoint& plan_start_point,
    const DrivePassage& drive_passage, const PlannerSemanticMapManager& psmm,
    const NudgeInfos& nudge_info, const PathSlBoundary& path_sl_boundary,
    const std::vector<double>& inner_path_boundary_gains,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    const std::optional<double>& lane_width_curb_buffer_opt,
    const double lane_width, std::optional<double>* extra_curb_buffer_opt,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(drive_passage.stations().size(), 0);

  std::vector<double> mid_edges_to_center = {};
  if (cost_weight_params.vehicle_model_params().enable_middle_circles()) {
    const auto& mid_edge_to_centers =
        cost_weight_params.vehicle_model_params().mid_edge_to_center();
    mid_edges_to_center.assign(mid_edge_to_centers.begin(),
                               mid_edge_to_centers.end());
  }

  // Get curb and path boundary info.
  const std::vector<Vec2d>& station_points = stations_query_helper->points();
  std::vector<double> left_dists_to_curb;
  std::vector<double> right_dists_to_curb;
  std::vector<double> left_dists_to_path_boundary;
  std::vector<double> right_dists_to_path_boundary;
  std::vector<double> left_dists_to_target_path_boundary;
  std::vector<double> right_dists_to_target_path_boundary;
  std::vector<double> reference_center_l_offsets;
  std::optional<double> uturn_middle_s = std::nullopt;
  if (FLAGS_trajectory_optimizer_ignore_u_turn_right_boundary &&
      enable_three_point_turn) {
    GeneratePathBoundariesWithRightBoundaryAfterUTurnIgnored(
        drive_passage, path_sl_boundary, &left_dists_to_curb,
        &right_dists_to_curb, &left_dists_to_path_boundary,
        &right_dists_to_path_boundary, &left_dists_to_target_path_boundary,
        &right_dists_to_target_path_boundary, &reference_center_l_offsets,
        &uturn_middle_s);
  } else {
    GeneratePathBoundaries(
        drive_passage, nudge_info, path_sl_boundary, &left_dists_to_curb,
        &right_dists_to_curb, &left_dists_to_path_boundary,
        &right_dists_to_path_boundary, &left_dists_to_target_path_boundary,
        &right_dists_to_target_path_boundary, &reference_center_l_offsets);
  }

  // Build curb extra buffer querier.
  const auto& ref_center_vector = path_sl_boundary.reference_center_xy_vector();
  const int points_num = ref_center_vector.size();
  std::vector<PathPoint> raw_path_points;
  raw_path_points.reserve(points_num);
  for (const auto& pt : ref_center_vector) {
    auto& path_point = raw_path_points.emplace_back();
    path_point.set_x(pt.x());
    path_point.set_y(pt.y());
  }
  for (int i = 0; i + 2 < points_num; ++i) {
    // Evaluate kappa and s, please refer to
    const Vec2d dxy =
        -0.5 * ref_center_vector[i] + 0.5 * ref_center_vector[i + 2];
    const Vec2d d2xy = ref_center_vector[i] - 2.0 * ref_center_vector[i + 1] +
                       ref_center_vector[i + 2];
    const double tmp = 1.0 / (Sqr(dxy.x()) + Sqr(dxy.y()));
    const double kappa =
        (dxy.x() * d2xy.y() - d2xy.x() * dxy.y()) * tmp * std::sqrt(tmp);
    // Use absolute value to compute max value in a range.
    raw_path_points[i + 1].set_kappa(std::abs(kappa));
    raw_path_points[i + 1].set_s(
        raw_path_points[i].s() +
        ref_center_vector[i + 1].DistanceTo(ref_center_vector[i]));
  }
  CHECK_GT(points_num, 1);
  raw_path_points[0].set_kappa(raw_path_points[1].kappa());
  raw_path_points.back().set_s(raw_path_points[points_num - 2].s() +
                               ref_center_vector[points_num - 1].DistanceTo(
                                   ref_center_vector[points_num - 2]));
  // Set curvature to the max value in a range.
  constexpr double kMaxCurvatureLookBackDist = 5.0;       // m.
  constexpr double kLookBackDistExtensionInUTurn = 15.0;  // m.
  constexpr double kUTurnCheckMaxDist = 40.0;             // m.
  double look_back_dist = kMaxCurvatureLookBackDist;
  for (const auto& station : drive_passage.stations()) {
    if (station.turn_type() == ad_byd::planning::U_TURN) {
      look_back_dist += kLookBackDistExtensionInUTurn;
      break;
    }
    if (station.accumulated_s() > kUTurnCheckMaxDist) {
      break;
    }
  }
  for (int i = points_num - 1; i >= 0; --i) {
    double max_kappa = raw_path_points[i].kappa();
    for (int j = i; j >= 0; --j) {
      if (raw_path_points[i].s() - raw_path_points[j].s() > look_back_dist) {
        break;
      }
      max_kappa = std::max(max_kappa, raw_path_points[j].kappa());
    }
    raw_path_points[i].set_kappa(max_kappa);
  }
  // Build querier.
  const DiscretizedPath ref_center_path(std::move(raw_path_points));
  const auto ref_center_frenet_status = BuildKdTreeFrenetFrame(
      ref_center_vector, /*down_sample_raw_points = */ false);
  CHECK(ref_center_frenet_status.ok());
  const auto kappa_buffer_plf = PiecewiseLinearFunctionFromProto(
      cost_weight_params.kappa_rel_curb_clearance_buffer_plf());

  // mean value filter
  std::deque<double> input_kappa;
  for (double i = 0.1; i < 10; ++i) {
    input_kappa.push_back(std::abs(
        ref_center_path
            .Evaluate(
                ref_center_frenet_status->XYToSL(plan_start_point.pos()).s +
                0.5 * i)
            .kappa()));
  }

  // lambda func
  CHECK_GT(input_kappa.size(), 0);
  auto filteredKappa = [&](const std::deque<double>& input) {
    double sum = 0.0;
    for (const double& value : input) {
      sum += value;
      Log2DDS::LogDataV0("curb_debug", absl::StrCat("input kappa: ", value));
    }
    return sum / input.size();
  };

  double filter_kappa = filteredKappa(input_kappa);
  input_kappa.clear();

  Log2DDS::LogDataV0(
      "curb_debug",
      absl::StrCat("origin kappa: ",
                   std::abs(ref_center_path
                                .Evaluate(ref_center_frenet_status
                                              ->XYToSL(plan_start_point.pos())
                                              .s)
                                .kappa())));

  Log2DDS::LogDataV0("curb_debug",
                     absl::StrCat("filter kappa: ", filter_kappa));
  const auto query_curb_extra_buffer =
      [&ref_center_path, &ref_center_frenet_status, &kappa_buffer_plf,
       &drive_passage, &plan_start_point,
       &filter_kappa](const Vec2d& pos) -> std::tuple<double, double, double> {
    const auto sl = ref_center_frenet_status->XYToSL(pos);
    const auto lane_boundary_info =
        drive_passage.QueryEnclosingLaneBoundariesAtS(sl.s);
    const double lane_width = lane_boundary_info.left->lat_offset -
                              lane_boundary_info.right->lat_offset;
    const double length = sl.s - plan_start_point.s();
    return {kappa_buffer_plf(filter_kappa), lane_width, length};
  };

  // Set extra curb buffer at plan start point.
  *extra_curb_buffer_opt =
      std::get<0>(query_curb_extra_buffer(plan_start_point.pos()));

  // Curb boundary cost.
  const auto av_sl_pos_or =
      drive_passage.QueryFrenetCoordinateAt(plan_start_point.pos());
  if (!av_sl_pos_or.ok()) return;
  const auto av_curb_pair_or =
      drive_passage.QueryCurbOffsetAtS(av_sl_pos_or->s);
  if (!av_curb_pair_or.ok()) return;

  const PiecewiseLinearFunction<double> speed_rel_soft_curb_clearance_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.speed_rel_soft_curb_clearance_plf());

  const double vehicle_half_width = 0.5 * veh_geo_params.width();
  const auto get_hard_curb_clearance =
      [&vehicle_half_width, &av_sl_pos_or](double dist_to_curb_at_av_s) {
        constexpr double kHardCurbClearanceShift = 0.05;
        constexpr double kHardCurbClearanceMax = 0.5;
        constexpr double kHardCurbClearanceMin = 0.15;
        return std::clamp(std::abs(dist_to_curb_at_av_s - av_sl_pos_or->l) -
                              vehicle_half_width + kHardCurbClearanceShift,
                          kHardCurbClearanceMin, kHardCurbClearanceMax);
      };

  const std::vector<double> cascade_gains = {
      cost_weight_params.static_boundary_soft_cost_weight(),
      cost_weight_params.static_boundary_hard_cost_weight()};
  // Add curb.
  const double soft_curb_clearance =
      speed_rel_soft_curb_clearance_plf(plan_start_point.v());
  const double hard_curb_clearance = get_hard_curb_clearance(
      std::min(av_curb_pair_or->second, av_curb_pair_or->first));

  std::vector<int> below_mirror_curb_start_segment_ids,
      above_mirror_curb_start_segment_ids, uturn_right_curb_start_segment_ids;
  std::vector<MsdProblemWithBuffer::SegmentType> below_mirror_curb_segments,
      above_mirror_curb_segments, uturn_right_curb_segments;
  bool uturn_curb_consider_mirrors = false;
  double nearest_curb_distance = 10.0;
  CollectCurbSegmentsAroundDrivePassage(
      trajectory_optimizer_vehicle_model_params.consider_mirrors_by_default(),
      veh_geo_params, plan_start_point, av_sl_pos_or->s, psmm, drive_passage,
      query_curb_extra_buffer, uturn_middle_s, &below_mirror_curb_segments,
      &below_mirror_curb_start_segment_ids, &above_mirror_curb_segments,
      &above_mirror_curb_start_segment_ids, &uturn_right_curb_segments,
      &uturn_right_curb_start_segment_ids, &uturn_curb_consider_mirrors,
      &nearest_curb_distance);

  // absl::StatusOr<std::pair<double, double>> plan_start_point_curb_offset =
  //     drive_passage.QueryCurbOffsetAtS(plan_start_point.s());
  // Log2DDS::LogDataV2(
  //     "plan_start_point_curb_offset",
  //     absl::StrCat(plan_start_point_curb_offset.value().first, " ",
  //                  plan_start_point_curb_offset.value().second));

  // ASSIGN_OR_RETURN(const auto ego_sl,
  //                  drive_passage.QueryFrenetCoordinateAt(ego_pos_),
  //                  _ << "Ego pos not in drive passage.");
  // const double lat_offset = std::abs(ego_sl.l);

  // const Box2d ego_box = ComputeAvBox(plan_start_point.pos(),
  //                                    plan_start_point.theta(),
  //                                    veh_geo_params);

  // auto ego_frenet_box = drive_passage.QueryFrenetBoxAt(ego_box);

  // double right_interval =
  //     ego_frenet_box.value().l_min -
  //     plan_start_point_curb_offset.value().first;
  // double left_interval = plan_start_point_curb_offset.value().second -
  //                        ego_frenet_box.value().l_max;

  // Log2DDS::LogDataV2("plan_start_point_curb_offset",
  //                    absl::StrCat("ego_frenet_box.value().l_min: ",
  //                                 ego_frenet_box.value().l_min,
  //                                 ", ego_frenet_box.value().l_max: ",
  //                                 ego_frenet_box.value().l_max));
  // Log2DDS::LogDataV2("plan_start_point_curb_offset",
  //                    absl::StrCat("left_interval: ", left_interval,
  //                                 ", right_interval: ", right_interval));

  // const double current_curb_interval =
  //     std::max(std::min(left_interval, right_interval), 0.0);
  std::vector<std::pair<double, Vec2d>> plan_start_circles;
  std::vector<std::pair<double, Vec2d>> plan_start_mirror_circles;

  for (int id : below_mirror_curb_start_segment_ids) {
    Log2DDS::LogDataV0("curb_debug", absl::StrCat(id));
  }
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(plan_start_point.theta() +
                                                   circle.angle_to_axis());
    plan_start_circles.emplace_back(
        circle.radius(),
        plan_start_point.pos() + circle.dist_to_rac() * tangent);
  }
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.mirror_circles()) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(plan_start_point.theta() +
                                                   circle.angle_to_axis());
    plan_start_mirror_circles.emplace_back(
        circle.radius(),
        plan_start_point.pos() + circle.dist_to_rac() * tangent);
  }
  const double min_mirror_height_avg =
      ComputeMinMaxMirrorAverageHeight(veh_geo_params).first;
  constexpr double kNearbyDistance = 10.0;  // m.
  double nearest_dist = kNearbyDistance;
  const std::vector<ImpassableBoundaryInfo> boundaries_info =
      psmm.GetImpassableBoundariesInfo(plan_start_point.pos(), kNearbyDistance);
  for (const auto& boundary_info : boundaries_info) {
    const bool consider_mirrors =
        boundary_info.height.has_value()
            ? (boundary_info.height.value() > min_mirror_height_avg)
            : trajectory_optimizer_vehicle_model_params
                  .consider_mirrors_by_default();
    for (const auto& circle : plan_start_circles) {
      nearest_dist = std::min(
          nearest_dist,
          boundary_info.segment.DistanceTo(circle.second) - circle.first);
    }
    if (consider_mirrors) {
      for (const auto& circle : plan_start_mirror_circles) {
        nearest_dist = std::min(
            nearest_dist,
            boundary_info.segment.DistanceTo(circle.second) - circle.first);
      }
    }
  }

  Log2DDS::LogDataV2("plan_start_point_curb_offset",
                     absl::StrCat("nearest_dist: ", nearest_dist));

  auto speed_rel_hard_curb_clearance_plf = PiecewiseLinearFunctionFromProto(
      cost_weight_params.speed_rel_hard_curb_clearance_plf());

  const double speed_rel_hard_curb_interval =
      speed_rel_hard_curb_clearance_plf(plan_start_point.v());

  if (!below_mirror_curb_segments.empty()) {
    if (FLAGS_msd_static_boundary_cost_v2) {
      AddMsdStaticBoundaryCostV2(
          trajectory_steps, base_name, /*source_name=*/"below_mirror_curb",
          /*source_scale=*/kCurbGain, cost_weight_params, plan_start_point,
          trajectory_optimizer_vehicle_model_params,
          /*consider_mirrors=*/false, below_mirror_curb_start_segment_ids,
          std::move(below_mirror_curb_segments), stations_query_helper,
          lane_width_curb_buffer_opt, nearest_dist,
          speed_rel_hard_curb_interval, lane_width, costs);
    } else {
      AddMsdStaticBoundaryCost(
          trajectory_steps, base_name, /*source_name=*/"below_mirror_curb",
          cost_weight_params, plan_start_point, veh_geo_params,
          trajectory_optimizer_vehicle_model_params,
          /*consider_mirrors=*/false, below_mirror_curb_start_segment_ids,
          std::move(below_mirror_curb_segments),
          /*sub_names=*/{"CurbSoft", "CurbHard"},
          /*cascade_buffers=*/{soft_curb_clearance, hard_curb_clearance},
          cascade_gains, stations_query_helper, costs);
    }
  }
  if (!above_mirror_curb_segments.empty()) {
    if (FLAGS_msd_static_boundary_cost_v2) {
      AddMsdStaticBoundaryCostV2(
          trajectory_steps, base_name, /*source_name=*/"above_mirror_curb",
          /*source_scale=*/kCurbGain, cost_weight_params, plan_start_point,
          trajectory_optimizer_vehicle_model_params,
          /*consider_mirrors=*/true, above_mirror_curb_start_segment_ids,
          std::move(above_mirror_curb_segments), stations_query_helper,
          lane_width_curb_buffer_opt, nearest_dist,
          speed_rel_hard_curb_interval, lane_width, costs);
    } else {
      AddMsdStaticBoundaryCost(
          trajectory_steps, base_name, /*source_name=*/"above_mirror_curb",
          cost_weight_params, plan_start_point, veh_geo_params,
          trajectory_optimizer_vehicle_model_params,
          /*consider_mirrors=*/true, above_mirror_curb_start_segment_ids,
          std::move(above_mirror_curb_segments),
          /*sub_names=*/{"CurbSoft", "CurbHard"},
          /*cascade_buffers=*/{soft_curb_clearance, hard_curb_clearance},
          cascade_gains, stations_query_helper, costs);
    }
  }
  if (!uturn_right_curb_segments.empty()) {
    if (FLAGS_msd_static_boundary_cost_v2) {
      AddMsdStaticBoundaryCostV2(
          trajectory_steps, base_name, /*source_name=*/"uturn_right_curb",
          /*source_scale=*/kUTurnCurbGain, cost_weight_params, plan_start_point,
          trajectory_optimizer_vehicle_model_params,
          uturn_curb_consider_mirrors, uturn_right_curb_start_segment_ids,
          std::move(uturn_right_curb_segments), stations_query_helper,
          lane_width_curb_buffer_opt, nearest_dist,
          speed_rel_hard_curb_interval, lane_width, costs);
    } else {
      const std::vector<double> uturn_right_curb_cascade_gains = {
          cost_weight_params.static_boundary_soft_cost_weight() *
              kUTurnCurbGain,
          cost_weight_params.static_boundary_hard_cost_weight() *
              kUTurnCurbGain};
      AddMsdStaticBoundaryCost(
          trajectory_steps, base_name, /*source_name=*/"uturn_right_curb",
          cost_weight_params, plan_start_point, veh_geo_params,
          trajectory_optimizer_vehicle_model_params,
          uturn_curb_consider_mirrors, uturn_right_curb_start_segment_ids,
          std::move(uturn_right_curb_segments),
          /*sub_names=*/{"UTurnRightCurbSoft", "UTurnRightCurbHard"},
          /*cascade_buffers=*/{soft_curb_clearance, hard_curb_clearance},
          uturn_right_curb_cascade_gains, stations_query_helper, costs);
    }
  }

  if (lc_stage == LaneChangeStage::LCS_EXECUTING ||
      lc_stage == LaneChangeStage::LCS_PAUSE) {
    // Path boundary costs.
    const std::vector<double> rear_gain = {
        cost_weight_params.path_boundary_cost_params()
            .rear_path_boundary_cost_weight(),
        cost_weight_params.target_path_boundary_cost_params()
            .rear_path_boundary_cost_weight()};
    const std::vector<double> front_gain = {
        cost_weight_params.path_boundary_cost_params()
            .front_path_boundary_cost_weight(),
        cost_weight_params.target_path_boundary_cost_params()
            .front_path_boundary_cost_weight()};
    const std::vector<double> buffers_min = {
        cost_weight_params.path_boundary_cost_params().buffer_min(),
        cost_weight_params.target_path_boundary_cost_params().buffer_min()};
    const std::vector<double> rear_buffers_max = {
        cost_weight_params.path_boundary_cost_params().rear_buffer_max(),
        cost_weight_params.target_path_boundary_cost_params()
            .rear_buffer_max()};
    const std::vector<double> front_buffers_max = {
        cost_weight_params.path_boundary_cost_params().front_buffer_max(),
        cost_weight_params.target_path_boundary_cost_params()
            .front_buffer_max()};
    const std::vector<double> mid_buffers_max = {
        cost_weight_params.path_boundary_cost_params().mid_buffer_max(),
        cost_weight_params.target_path_boundary_cost_params().mid_buffer_max()};

    for (const bool left : {false, true}) {
      std::vector<std::vector<double>> dists_to_path_boundary;
      dists_to_path_boundary.reserve(2);
      std::vector<double> cascade_gains;
      cascade_gains.reserve(2);
      if (left) {
        dists_to_path_boundary.push_back(left_dists_to_path_boundary);
        dists_to_path_boundary.push_back(left_dists_to_target_path_boundary);
        cascade_gains.push_back(cost_weight_params.path_boundary_cost_params()
                                    .left_path_boundary_cost_weight());
        cascade_gains.push_back(
            cost_weight_params.target_path_boundary_cost_params()
                .left_path_boundary_cost_weight());
      } else {
        dists_to_path_boundary.push_back(right_dists_to_path_boundary);
        dists_to_path_boundary.push_back(right_dists_to_target_path_boundary);
        cascade_gains.push_back(cost_weight_params.path_boundary_cost_params()
                                    .right_path_boundary_cost_weight());
        cascade_gains.push_back(
            cost_weight_params.target_path_boundary_cost_params()
                .right_path_boundary_cost_weight());
      }
      std::vector<std::vector<double>> ref_gains;
      ref_gains.reserve(cascade_gains.size());
      std::vector<double> outer_path_boundary_gains(station_points.size(), 1.0);
      ref_gains.push_back(std::move(outer_path_boundary_gains));
      ref_gains.push_back(inner_path_boundary_gains);
      // TODO: Consider decay weight based on  av dist to boundary.
      auto& gains = ref_gains.back();
      constexpr double kDecayGainLookAheadTime = 2.5;  // s
      const double decay_gain_look_ahead_s =
          plan_start_point.v() * kDecayGainLookAheadTime +
          veh_geo_params.front_edge_to_center();
      const auto& stations = drive_passage.stations();
      const int s_index = std::distance(
          stations.begin(),
          std::upper_bound(stations.begin(), stations.end(),
                           decay_gain_look_ahead_s,
                           [](double val, const Station& station) {
                             return val < station.accumulated_s();
                           }));
      constexpr double kDecayGain = 0.01;
      for (int i = 0; i < gains.size(); ++i) {
        if (i > s_index) break;
        gains[i] = std::min(gains[i], kDecayGain);
      }
      std::vector<std::string> sub_names = {"Outer", "Inner"};
      costs->emplace_back(std::make_unique<MfobPathBoundaryCost<Mfob>>(
          trajectory_steps, veh_geo_params, station_points,
          stations_query_helper.get(), reference_center_l_offsets,
          dists_to_path_boundary, left,
          /*using_hessian_approximate=*/true, mid_edges_to_center,
          std::move(ref_gains), std::move(sub_names), /*use_qtfm=*/true,
          buffers_min, rear_buffers_max, front_buffers_max, mid_buffers_max,
          cascade_gains, rear_gain, front_gain,
          absl::StrCat("PathBoundaryCost: ",
                       left ? "left path boundary" : "right path boundary"),
          /*scale=*/1.0,
          /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
    }
  }
}

void AddSolidWhiteLineCost(
    int trajectory_steps, std::string_view base_name,
    const std::vector<TrajectoryPoint>& solver_init_traj,
    const ConstraintManager& constraint_manager,
    const TrajectoryPoint& plan_start_point,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_NOTNULL(costs);

  // Add solid line within path boundary.
  constexpr double kSolidLineExtendDistance = 0.5;  // m.
  constexpr double kSolidLineBuffer = 0.2;          // m.
  std::vector<int> solid_line_start_segment_ids;
  std::vector<MsdProblemWithBuffer::SegmentType> solid_line_segments;
  CollectExtendSolidLinesWithinPathBoundary(
      veh_geo_params, trajectory_optimizer_vehicle_model_params,
      constraint_manager, solver_init_traj, kSolidLineBuffer,
      kSolidLineExtendDistance, &solid_line_start_segment_ids,
      &solid_line_segments);
  if (!solid_line_segments.empty()) {
    AddMsdStaticBoundaryCost(
        trajectory_steps, base_name, /*source_name=*/"solid_line",
        cost_weight_params, plan_start_point, veh_geo_params,
        trajectory_optimizer_vehicle_model_params,
        /*consider_mirrors=*/false, solid_line_start_segment_ids,
        std::move(solid_line_segments),
        /*sub_names=*/{"SolidLine"},
        /*cascade_buffers=*/{kSolidLineBuffer},
        /*cascade_gains=*/
        {cost_weight_params.static_boundary_solid_line_cost_weight()},
        stations_query_helper, costs);
  }
}

void AddRoadBoundaryCost(
    int trajectory_steps, std::string_view base_name,
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* const costs) {
  CHECK_NOTNULL(costs);
  if (psmm.map_ptr() == nullptr ||
      psmm.map_ptr()->type() != ad_byd::planning::MapType::HD_MAP ||
      drive_passage.size() < 2) {
    return;
  }

  const auto av_sl_pos_or = drive_passage.QueryFrenetCoordinateAt(
      {plan_start_point.path_point().x(), plan_start_point.path_point().y()});
  if (!av_sl_pos_or.ok()) return;
  double turn_finish_lane_width = 0.0;
  std::unordered_set<mapping::ElementId> boundary_ids;
  Vec2d turn_finish_xy;
  bool left_turn_finish = false;
  bool right_turn_finish = false;
  bool no_turn_finish = false;
  const auto& map_ptr = psmm.map_ptr();
  for (int i = 1; i < drive_passage.size(); i++) {
    const Station& cur_station = drive_passage.station(StationIndex(i));
    const Station& pre_station = drive_passage.station(StationIndex(i - 1));
    left_turn_finish = (pre_station.station_info().turn_type ==
                            ad_byd::planning::TurnType::LEFT_TURN &&
                        cur_station.station_info().turn_type ==
                            ad_byd::planning::TurnType::NO_TURN);
    right_turn_finish = (pre_station.station_info().turn_type ==
                             ad_byd::planning::TurnType::RIGHT_TURN &&
                         cur_station.station_info().turn_type ==
                             ad_byd::planning::TurnType::NO_TURN);
    no_turn_finish = (pre_station.station_info().turn_type ==
                          ad_byd::planning::TurnType::NO_TURN &&
                      pre_station.station_info().is_in_intersection &&
                      (!cur_station.station_info().is_in_intersection));
    if (i > drive_passage.last_real_station_index().value()) {
      break;
    }
    const double turn_finish_s = cur_station.accumulated_s();
    const double dis_to_turn_finish = turn_finish_s - av_sl_pos_or->s;
    constexpr double kMaxDisToTurnFinish = 60.0;
    constexpr double kMinDisToTurnFinish = -2.5;
    if (dis_to_turn_finish > kMaxDisToTurnFinish) {
      break;
    }
    if (dis_to_turn_finish < kMinDisToTurnFinish) {
      continue;
    }
    if (pre_station.station_info().is_in_intersection &&
        (left_turn_finish || right_turn_finish || no_turn_finish)) {
      const auto turn_finish_lane_boundary_info =
          drive_passage.QueryEnclosingLaneBoundariesAtS(turn_finish_s);
      turn_finish_lane_width = turn_finish_lane_boundary_info.left->lat_offset -
                               turn_finish_lane_boundary_info.right->lat_offset;
      Log2DDS::LogDataV2(
          "road_boundary_debug",
          absl::StrCat("turn_finish_lane_width: ", turn_finish_lane_width,
                       ", left_turn_finish: ", left_turn_finish,
                       ", right_turn_finish: ", right_turn_finish,
                       ", no_turn_finish: ", no_turn_finish,
                       ", dis_to_turn_finish: ", dis_to_turn_finish));
      const auto& lane_ptr = map_ptr->GetLaneById(cur_station.lane_id());
      if (lane_ptr == nullptr) {
        continue;
      }
      const auto& left_lane_ptr = map_ptr->GetLeftLane(lane_ptr);
      const auto& right_lane_ptr = map_ptr->GetRightLane(lane_ptr);
      if ((left_turn_finish && right_lane_ptr != nullptr) ||
          (right_turn_finish && left_lane_ptr != nullptr) ||
          (no_turn_finish &&
           (left_lane_ptr != nullptr || right_lane_ptr == nullptr))) {
        continue;
      }
      if (left_turn_finish) {
        for (const auto& boundary :
             lane_ptr->right_boundary()->lane_boundaries()) {
          if (boundary == nullptr || !boundary->IsValid() ||
              boundary->type().line_type ==
                  ad_byd::planning::LineType::VIRTUAL_LANE) {
            continue;
          }
          boundary_ids.insert(boundary->id());
        }
      }
      if (right_turn_finish) {
        for (const auto& boundary :
             lane_ptr->left_boundary()->lane_boundaries()) {
          if (boundary == nullptr || !boundary->IsValid() ||
              boundary->type().line_type ==
                  ad_byd::planning::LineType::VIRTUAL_LANE) {
            continue;
          }
          boundary_ids.insert(boundary->id());
        }
      }
      if (no_turn_finish) {
        if (left_lane_ptr == nullptr) {
          for (const auto& boundary :
               lane_ptr->left_boundary()->lane_boundaries()) {
            if (boundary == nullptr || !boundary->IsValid() ||
                boundary->type().line_type ==
                    ad_byd::planning::LineType::VIRTUAL_LANE) {
              continue;
            }
            boundary_ids.insert(boundary->id());
          }
        }
      }
      if (!boundary_ids.empty()) {
        turn_finish_xy = cur_station.xy();
        break;
      }
    }
  }
  if (boundary_ids.empty()) {
    Log2DDS::LogDataV0("road_boundary_debug", "boundary_ids.size is zero");
    return;
  }

  const auto road_boundary_buffer_plf =
      no_turn_finish
          ? PiecewiseLinearFunctionFromProto(
                cost_weight_params.noturn_lane_width_road_boundary_buffer_plf())
          : PiecewiseLinearFunctionFromProto(
                cost_weight_params.turn_lane_width_road_boundary_buffer_plf());
  const double road_boundary_buffer =
      road_boundary_buffer_plf(turn_finish_lane_width);
  const Vec2d start_pos{plan_start_point.path_point().x(),
                        plan_start_point.path_point().y()};
  const double start_theta = plan_start_point.path_point().theta();
  std::vector<MsdProblemWithBuffer::SegmentType> road_boundary_named_segments;
  std::vector<int> road_boundary_start_segment_ids;
  std::string debug_str =
      absl::StrCat("road_boundary_buffer: ", road_boundary_buffer) +
      ", boundary_ids:";
  for (const auto& id : boundary_ids) {
    const auto& lane_boundary = psmm.FindLaneBoundaryByIdOrNull(id);
    if (lane_boundary == nullptr) {
      continue;
    }
    const std::vector<Vec2d>& points = lane_boundary->points();
    if (points.empty()) {
      continue;
    }
    debug_str += " " + std::to_string(id);
    constexpr double kMaxBoundaryDistance = 13.0;
    bool has_no_prev_segment = true;
    Vec2d prev = points.front();
    for (int i = 1; i < points.size(); ++i) {
      if (prev.DistanceTo(points[i]) <=
          QtfmSegmentMatcherV2::kMinSegmentLength) {
        continue;
      }
      const Segment2d cur_seg(prev, points[i]);
      if (cur_seg.DistanceTo(turn_finish_xy) > kMaxBoundaryDistance) {
        continue;
      }
      double extra_buffer = 0.0;
      for (const auto& circle :
           trajectory_optimizer_vehicle_model_params.circles()) {
        const Vec2d center =
            start_pos +
            Vec2d::FastUnitFromAngleN12(start_theta + circle.angle_to_axis()) *
                circle.dist_to_rac();
        const double dist = cur_seg.DistanceTo(center) - circle.radius();
        extra_buffer =
            std::min(road_boundary_buffer, dist - road_boundary_buffer);
      }
      road_boundary_named_segments.push_back(
          {absl::StrCat("id:", id, ",seg:", i), cur_seg, extra_buffer});
      if (has_no_prev_segment) {
        road_boundary_start_segment_ids.push_back(
            road_boundary_named_segments.size() - 1);
        has_no_prev_segment = false;
      }
      prev = points[i];
    }
  }
  if (road_boundary_named_segments.empty()) {
    return;
  }
  Log2DDS::LogDataV0("road_boundary_info", debug_str);
  const int circle_size =
      trajectory_optimizer_vehicle_model_params.circles_size();
  double max_circle_radius = 0.0;
  std::vector<Vec2d> circle_center_offsets;
  std::vector<double> circle_radiuses;
  circle_center_offsets.reserve(circle_size);
  circle_radiuses.reserve(circle_size);
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    circle_center_offsets.push_back(
        Vec2d(circle.dist_to_rac() * std::cos(circle.angle_to_axis()),
              circle.dist_to_rac() * std::sin(circle.angle_to_axis())));
    circle_radiuses.push_back(circle.radius());
    max_circle_radius = std::max(max_circle_radius, circle.radius());
  }
  constexpr double kCutoffDistExtraBuffer = 0.25;
  const double cutoff_distance =
      road_boundary_buffer + max_circle_radius + kCutoffDistExtraBuffer;
  MsdProblemWithBuffer msd(std::move(road_boundary_named_segments),
                           cutoff_distance, road_boundary_start_segment_ids);
  std::vector<std::string> sub_names{HardNameString};
  std::vector<double> cascade_buffers{0.0};
  std::vector<double> cascade_gains{
      cost_weight_params.road_boundary_line_cost_weight()};
  costs->emplace_back(std::make_unique<MsdStaticBoundaryCost<Mfob>>(
      trajectory_steps, veh_geo_params, std::move(msd),
      stations_query_helper.get(), std::move(sub_names),
      std::move(cascade_buffers), std::move(cascade_gains),
      circle_center_offsets, circle_radiuses, trajectory_steps, true,
      "MsdRoadBoundaryCost", cost_weight_params.msd_road_boundary_cost_weight(),
      Cost<Mfob>::CostType::MSD_ROAD_BOUNDARY));
}

}  // namespace optimizer
}  // namespace planning
}  // namespace st
