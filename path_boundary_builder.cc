

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "decider/scheduler/path_boundary_builder.h"
//#include "global/trace.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/plan_common_defs.h"
#include "decider/scheduler/path_boundary_builder_helper.h"
//#include "semantic_map.pb.h"
#include "plan_common/log_data.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st::planning {
namespace {

constexpr double kMaxInnerBoundLaneChangeLatAccel = 0.09;  // m/s^2.
constexpr double kMaxOuterBoundLaneChangeLatAccel = 0.08;  // m/s^2.
constexpr double kMaxLaneChangeCancelLatAccel = 0.5;       // m/s^2.
constexpr double kExtendOuterBoundWidth = kDefaultHalfLaneWidth;

PathBoundary ExtendBoundaryBy(const DrivePassage& drive_passage,
                              PathBoundary boundary, bool is_lane_changing,
                              bool lc_left, double extend_width) {
  const double left_extend_path_boundary_width =
      is_lane_changing && lc_left ? 0.0 : extend_width;
  const double right_extend_path_boundary_width =
      is_lane_changing && !lc_left ? 0.0 : extend_width;

  // Get first uturn middle s.
  std::optional<double> uturn_middle_s;
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
    uturn_middle_s = 0.5 * (u_turn_start_s + u_turn_end_s);
  }

  for (int i = 0; i < boundary.size(); ++i) {
    const auto& station = drive_passage.station(StationIndex(i));
    boundary.ShiftLeftByIndex(i, left_extend_path_boundary_width);
    if (station.turn_type() == ad_byd::planning::U_TURN &&
        uturn_middle_s.has_value() &&
        station.accumulated_s() < *uturn_middle_s) {
      continue;
    }
    boundary.ShiftRightByIndex(i, -right_extend_path_boundary_width);
  }

  return boundary;
}

std::vector<double> PostprocessOuterBoundary(absl::Span<const double> s_vec,
                                             absl::Span<const double> inner_vec,
                                             std::vector<double> outer_vec) {
  constexpr double kEpsilon = 0.1;
  constexpr double kMinContinuousExtendedBoundLength = 3.0;  // m.
  int first_extended_idx = -1;
  for (int i = 1; i < inner_vec.size(); ++i) {
    bool curr_is_extended = std::fabs(inner_vec[i] - outer_vec[i]) >
                            (kExtendOuterBoundWidth - kEpsilon);
    bool prev_is_extended = std::fabs(inner_vec[i - 1] - outer_vec[i - 1]) >
                            (kExtendOuterBoundWidth - kEpsilon);
    if (first_extended_idx == -1) {
      if (curr_is_extended && !prev_is_extended) {
        first_extended_idx = i;
      }
    } else if (!curr_is_extended) {
      if ((s_vec[i - 1] - s_vec[first_extended_idx]) <=
          kMinContinuousExtendedBoundLength) {
        for (int j = first_extended_idx; j < i; ++j) {
          outer_vec[j] = outer_vec[first_extended_idx - 1];
        }
      }
      first_extended_idx = -1;
    }
  }

  return outer_vec;
}

void SmoothBoundary(absl::Span<const double> s_vec,
                    const std::pair<int, int>& split_range, bool is_left,
                    std::vector<double>* vec_ptr) {
  if (nullptr == vec_ptr || vec_ptr->size() < 4) return;
  auto& vec = *vec_ptr;
  int front_idx = 0;
  int back_idx = vec.size() - 1;
  const double ramp_factor = 0.25;
  if (is_left) {
    while (front_idx < back_idx) {
      if (vec[front_idx] > vec[back_idx] + 0.01) {
        --back_idx;
        if (vec[back_idx] < vec[back_idx + 1])
          vec[back_idx] = vec[back_idx + 1];
      } else {
        ++front_idx;
        if (vec[front_idx] < vec[front_idx - 1])
          vec[front_idx] = vec[front_idx - 1];
      }
    }
    while (front_idx > 0) {
      --front_idx;
      double left_ramp_factor =
          (split_range.first > 0 && front_idx >= split_range.first &&
           front_idx <= split_range.second)
              ? 0.1
              : ramp_factor;
      double temp_l =
          vec[front_idx + 1] -
          (s_vec[front_idx + 1] - s_vec[front_idx]) * left_ramp_factor;
      if (temp_l > vec[front_idx]) vec[front_idx] = temp_l;
    }
    while (back_idx < vec.size() - 1) {
      ++back_idx;
      double left_ramp_factor =
          (split_range.first > 0 && back_idx >= split_range.first &&
           back_idx <= split_range.second)
              ? 0.1
              : ramp_factor;
      double temp_l =
          vec[back_idx - 1] -
          (s_vec[back_idx] - s_vec[back_idx - 1]) * left_ramp_factor;
      if (temp_l > vec[back_idx]) vec[back_idx] = temp_l;
    }
  } else {
    while (front_idx < back_idx) {
      if (vec[front_idx] < vec[back_idx] - 0.01) {
        --back_idx;
        if (vec[back_idx] > vec[back_idx + 1])
          vec[back_idx] = vec[back_idx + 1];
      } else {
        ++front_idx;
        if (vec[front_idx] > vec[front_idx - 1])
          vec[front_idx] = vec[front_idx - 1];
      }
    }
    while (front_idx > 0) {
      --front_idx;
      double right_ramp_factor =
          (split_range.first > 0 && front_idx >= split_range.first &&
           front_idx <= split_range.second)
              ? 0.1
              : ramp_factor;
      double temp_l =
          vec[front_idx + 1] +
          (s_vec[front_idx + 1] - s_vec[front_idx]) * right_ramp_factor;
      if (temp_l < vec[front_idx]) vec[front_idx] = temp_l;
    }
    while (back_idx < vec.size() - 1) {
      ++back_idx;
      double right_ramp_factor =
          (split_range.first > 0 && back_idx >= split_range.first &&
           back_idx <= split_range.second)
              ? 0.1
              : ramp_factor;
      double temp_l =
          vec[back_idx - 1] +
          (s_vec[back_idx] - s_vec[back_idx - 1]) * right_ramp_factor;
      if (temp_l < vec[back_idx]) vec[back_idx] = temp_l;
    }
  }
}

}  // namespace

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromDrivePassage(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage) {
  const int n = drive_passage.size();
  std::vector<double> s_vec, center_l(n, 0.0);
  s_vec.reserve(n);
  for (const auto& station : drive_passage.stations()) {
    s_vec.push_back(station.accumulated_s());
  }
  const std::pair<int, int> left_split_range = {-1, -1};
  const std::pair<int, int> right_split_range = {-1, -1};
  const double ego_v = 5.0;
  auto inner_boundary = BuildPathBoundaryFromTargetLane(
      psmm, drive_passage, ego_v, left_split_range, right_split_range,
      0.5 * kMinLaneWidth, /*borrow_lane_boundary=*/false);

  const auto curb_boundary = BuildCurbPathBoundary(drive_passage);
  inner_boundary.OuterClampBy(curb_boundary);
  PathBoundary outer_boundary = inner_boundary;
  PathBoundary opt_outer_boundary = outer_boundary;

  return BuildPathSlBoundary(drive_passage, std::move(s_vec),
                             std::move(center_l), std::move(inner_boundary),
                             std::move(outer_boundary),
                             std::move(opt_outer_boundary));
}

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromPose(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const LaneChangeStateProto& lc_state,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    bool borrow_lane_boundary, bool should_smooth,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    PausePushSavedOffsetProto* saved_offset,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    bool is_congestion_scene, const ObjectHistoryManager* obj_history_mgr) {
  const Box2d ego_box =
      ComputeAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
                   plan_start_point.path_point().theta(), vehicle_geom);

  ASSIGN_OR_RETURN(const auto sl_box, drive_passage.QueryFrenetBoxAt(ego_box),
                   _ << "BuildPathBoundaryFromPose: Fail to project ego box on "
                        "drive passage.");

  ASSIGN_OR_RETURN(const auto cur_sl,
                   drive_passage.QueryFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
                   _ << "BuildPathBoundaryFromPose: Fail to project ego pos on "
                        "drive passage.");

  borrow_lane_boundary = borrow_lane_boundary &&
                         (lc_state.stage() != LaneChangeStage::LCS_EXECUTING &&
                          lc_state.stage() != LaneChangeStage::LCS_RETURN);
  const int n = drive_passage.stations().size();

  const bool enable_stop =
      drive_passage.traffic_static_obstacles_info().enable_stop;
  const BlockReason block_reason =
      drive_passage.traffic_static_obstacles_info().block_reason;
  const bool have_avliable_zone =
      drive_passage.traffic_static_obstacles_info().have_avliable_zone;

  bool enable_cutoff_drive_passage =
      ((block_reason == BlockReason::CURB_CROSS) ||
       (block_reason == BlockReason::OBS_CENTER_CURB) ||
       (enable_stop && (lc_state.stage() == LaneChangeStage::LCS_NONE ||
                        lc_state.stage() == LaneChangeStage::LCS_RETURN)));

  double stop_s = drive_passage.traffic_static_obstacles_info().stop_s;
  if (block_reason == BlockReason::CURB_CROSS) {
    stop_s = stop_s - FLAGS_planner_curb_cutoff_buffer;
  } else if (block_reason != BlockReason::NONE) {
    stop_s = stop_s - FLAGS_planner_obstacle_cutoff_buffer;
  }

  std::vector<double> s_vec;
  s_vec.reserve(n);
  std::pair<int, int> split_range = {-1, -1};
  int split_type = 0;  // 0: NONE, 1: LEFT, 2: RIGHT
  int clamp_idx = n + 4, idx = 0;
  uint64_t cross_curb_id = 0;
  bool is_split_followed_by_intersection = false;
  for (const auto& station : drive_passage.stations()) {
    s_vec.push_back(station.accumulated_s());
    // if (clamp_idx > n && station.has_cross_curb()) {
    // clamp_idx = idx;
    // cross_curb_id = station.cross_curb_id();
    //}
    if (clamp_idx > n && enable_cutoff_drive_passage &&
        station.accumulated_s() > stop_s) {
      clamp_idx = idx;
    }
    if (idx > 0) {
      if (station.is_splitting()) {
        if (split_range.first < 0) {
          split_type = (SplitTopology::TOPOLOGY_SPLIT_LEFT ==
                        station.station_info().split_topo)
                           ? 1
                           : ((SplitTopology::TOPOLOGY_SPLIT_RIGHT ==
                               station.station_info().split_topo)
                                  ? 2
                                  : 0);
          if (split_type > 0) {
            split_range.first = idx;
            split_range.second = idx;
          }
        } else if (split_range.second + 1 == idx) {
          if (station.is_in_intersection()) {
            is_split_followed_by_intersection = true;
          } else {
            split_range.second = idx;
          }
        }
      } else if (split_range.second + 1 == idx) {
        is_split_followed_by_intersection =
            station.is_in_intersection() || station.is_virtual();
      }
    }
    ++idx;
  }
  if (is_split_followed_by_intersection) {
    const double virtual_buffer = 10.0;
    split_range.second -= virtual_buffer;
    if (split_range.second - split_range.first < virtual_buffer * 0.5) {
      split_type = 0;
    }
  }
  std::pair<int, int> left_split_range =
      (1 == split_type) ? split_range : std::make_pair(-1, -1);
  std::pair<int, int> right_split_range =
      (2 == split_type) ? split_range : std::make_pair(-1, -1);
  // split点位置提前10m
  if (1 == split_type) {
    left_split_range.first = left_split_range.first + 10;
    left_split_range.second = std::min(left_split_range.second + 10, n);
  } else if (2 == split_type) {
    right_split_range.first = right_split_range.first + 10;
    right_split_range.second = std::min(right_split_range.second + 10, n);
  }

  // auto draw_boundary = [&](const PathBoundary& path_boundary,
  //                          const Log2DDS::Color& color = Log2DDS::kBlack,
  //                          std::string draw_name = "") {
  //   const int n = drive_passage.stations().size();
  //   std::vector<Vec2d> left_bound, right_bound;
  //   left_bound.reserve(n);
  //   right_bound.reserve(n);
  //   for (size_t i = 0; i < n; i++) {
  //     const auto& station = drive_passage.station(StationIndex(i));
  //     left_bound.emplace_back(station.lat_point(path_boundary.left(i)));
  //     right_bound.emplace_back(station.lat_point(path_boundary.right(i)));
  //   }

  //   Log2DDS::LogLineV1("sl_left_" + draw_name, color, {}, left_bound);
  //   Log2DDS::LogLineV1("sl_right_" + draw_name, color, {}, right_bound);
  // };

  std::vector<double> center_l(n, 0.0);
  if (should_smooth) {
    center_l =
        ComputeSmoothedReferenceLine(psmm, drive_passage, smooth_result_map);
  }

  const auto target_lane_offset = ComputeTargetLaneOffset(
      drive_passage, cur_sl, lc_state, plan_start_point,
      st_traj_mgr.trajectories(), sl_box, vehicle_geom, is_congestion_scene,
      saved_offset, unsafe_object_ids, obj_history_mgr);
  CenterLineOffsetType offset_type =
      CenterLineOffsetType::CENTER_LINE_OFFSET_UNKNOWN;
  if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
    offset_type = CenterLineOffsetType::CENTER_LINE_OFFSET_PAUSE;
  }
  if (lc_state.push_state() == PushState::LEFT_PUSH ||
      lc_state.push_state() == PushState::RIGHT_PUSH ||
      lc_state.push_state() == PushState::CONGESTION_LEFT_PUSH ||
      lc_state.push_state() == PushState::CONGESTION_RIGHT_PUSH) {
    offset_type = CenterLineOffsetType::CENTER_LINE_OFFSET_PUSH;
  }

  bool lane_change_pause_or_push =
      (lc_state.stage() == LaneChangeStage::LCS_PAUSE) ||
      (lc_state.push_state() != PushState::NONE_PUSH);
  double half_target_lane_width = kDefaultHalfLaneWidth;
  if (lane_change_pause_or_push) {
    const auto cur_station_index =
        drive_passage
            .FindNearestStationIndex(
                Vec2dFromApolloTrajectoryPointProto(plan_start_point))
            .value();
    const auto smoothed_center_offset =
        target_lane_offset - center_l[cur_station_index];
    ad_byd::planning::LaneConstPtr tgt_lane = nullptr;
    ad_byd::planning::LaneSequencePtr pre_lane_seq_before_lc = nullptr;
    double s = 0.0, l = 0.0;
    if (!prev_lane_path_before_lc_from_start.IsEmpty())
      pre_lane_seq_before_lc = prev_lane_path_before_lc_from_start.lane_seq();
    for (int i = 0; i < n; ++i) {
      center_l[i] += smoothed_center_offset;
      if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
        const auto& station = drive_passage.station(StationIndex(i));
        if (station.lane_id() == 0) {
          if (i > 0) center_l[i] = center_l[i - 1];
          continue;
        }
        if (pre_lane_seq_before_lc == nullptr ||
            !pre_lane_seq_before_lc->IsValid()) {
          if (i > 0) center_l[i] = center_l[i - 1];
          continue;
        }
        tgt_lane = pre_lane_seq_before_lc->GetNearestLane(station.xy());
        if (!tgt_lane || !tgt_lane->center_line().IsValid()) {
          if (i > 0) center_l[i] = center_l[i - 1];
          continue;
        }
        if (!tgt_lane->center_line().GetProjection(station.xy(), &s, &l)) {
          if (i > 0) center_l[i] = center_l[i - 1];
          continue;
        }
        if (std::fabs(l) < std::fabs(center_l[i])) {
          center_l[i] = center_l[i] > 0.0 ? std::fabs(l) : -std::fabs(l);
        }
      }
    }
    const auto& cur_station =
        drive_passage.station(StationIndex(cur_station_index));
    double right_offset = std::numeric_limits<double>::lowest();
    double left_offset = std::numeric_limits<double>::max();
    for (const auto& cur_bound : cur_station.boundaries()) {
      if (cur_bound.lat_offset < 0.0) {
        right_offset = std::max(right_offset, cur_bound.lat_offset);
      }
      if (cur_bound.lat_offset > 0.0) {
        left_offset = std::min(left_offset, cur_bound.lat_offset);
      }
    }
    half_target_lane_width =
        lc_state.lc_left() ? std::fabs(right_offset) : std::fabs(left_offset);
  }

  auto boundary = BuildPathBoundaryFromTargetLane(
      psmm, drive_passage, plan_start_point.v(), left_split_range,
      right_split_range, 0.5 * kMinLaneWidth, borrow_lane_boundary, sl_box);

  const auto solid_boundary = BuildSolidPathBoundary(
      psmm, drive_passage, cur_sl, vehicle_geom, plan_start_point, lc_state,
      target_lane_offset, center_l);

  const auto curb_boundary = BuildCurbPathBoundary(drive_passage);

  const bool is_lane_changing =
      (lc_state.stage() == LaneChangeStage::LCS_EXECUTING ||
       lc_state.stage() == LaneChangeStage::LCS_RETURN) &&
      !lc_state.entered_target_lane();

  // Kinematic boundaries assuming constant lateral acceleration.
  const double inner_bound_lat_acc =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING
          ? kMaxInnerBoundLaneChangeLatAccel
          : kMaxLaneChangeCancelLatAccel;
  const double outer_bound_lat_acc =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING
          ? kMaxOuterBoundLaneChangeLatAccel
          : kMaxLaneChangeCancelLatAccel;
  const auto inner_kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, lc_state,
      s_vec, target_lane_offset, inner_bound_lat_acc,
      lane_change_pause_or_push);
  const auto outer_kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, lc_state,
      s_vec, target_lane_offset, outer_bound_lat_acc,
      lane_change_pause_or_push);
  PathBoundary outer_boundary = boundary;
  outer_boundary.InnerClampBy(outer_kinematic_boundary);

  // Extend path boundary by half lane width.
  if (!(borrow_lane_boundary || lane_change_pause_or_push)) {
    const bool lc_left = lc_state.lc_left();
    outer_boundary =
        ExtendBoundaryBy(drive_passage, std::move(outer_boundary),
                         is_lane_changing, lc_left, kExtendOuterBoundWidth);

    if (is_lane_changing) {
      boundary.InnerClampBy(outer_boundary);
    }
  }

  // Shrink when lane change pause or push.
  if (lane_change_pause_or_push) {
    outer_boundary = ShrinkPathBoundaryForLaneChangePause(
        vehicle_geom, sl_box, lc_state, std::move(outer_boundary),
        target_lane_offset, center_l);
    outer_boundary.InnerClampBy(inner_kinematic_boundary);
    boundary.OuterClampBy(outer_boundary);
  }
  if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
    PathBoundary pause_extend_boundary = boundary;
    pause_extend_boundary = ExtendPathBoundaryForLaneChangePause(
        lc_state, half_target_lane_width, std::move(pause_extend_boundary));
    boundary.InnerClampBy(pause_extend_boundary);
  }

  // Smooth
  SmoothBoundary(s_vec, right_split_range, true, boundary.mutable_left_vec());
  SmoothBoundary(s_vec, left_split_range, false, boundary.mutable_right_vec());
  SmoothBoundary(s_vec, right_split_range, true,
                 outer_boundary.mutable_left_vec());
  SmoothBoundary(s_vec, left_split_range, false,
                 outer_boundary.mutable_right_vec());

  // Clamp boundary by curb and solid line.
  if (!borrow_lane_boundary) {
    // boundary.OuterClampBy(solid_boundary);
    // outer_boundary.OuterClampBy(solid_boundary);
    boundary.SoftOuterClampBy(solid_boundary, left_split_range,
                              right_split_range);
    outer_boundary.SoftOuterClampBy(solid_boundary, left_split_range,
                                    right_split_range);
    outer_boundary.InnerClampBy(outer_kinematic_boundary);
    if (is_lane_changing || lane_change_pause_or_push) {
      boundary.InnerClampBy(inner_kinematic_boundary);
    }
  }
  boundary.OuterClampBy(curb_boundary);
  outer_boundary.OuterClampBy(curb_boundary);
  PathBoundary opt_outer_boundary = outer_boundary;

  // Post-process center_l to make sure it is within path boundary.
  const double clamp_width = vehicle_geom.width() * 0.5;
  for (int i = 0; i < n; ++i) {
    if (i > clamp_idx /*i > clamp_idx - 3 ||
        boundary.left(i) - boundary.right(i) <
            clamp_width + ad_byd::planning::Constants::ZERO*/) {
      if (i <= 1) {
        return absl::InternalError("Path boundary only has one point.");
      }
      s_vec.resize(i);
      center_l.resize(i);
      boundary.EraseFrom(i);
      outer_boundary.EraseFrom(i);
      opt_outer_boundary.EraseFrom(i);
      // const auto& station = drive_passage.station(StationIndex(i));
      // Log2DDS::LogDataV0(
      //     "schedule_debug",
      //     absl::StrCat(
      //         "lane_id: ", station.lane_id(), ", curb_id: ", cross_curb_id,
      //         ", accumu_s = ", station.accumulated_s(),
      //         ", boundary_width = ", boundary.left(i) - boundary.right(i),
      //         ", clamp_width = ", clamp_width));
      break;
    } else {
      center_l[i] =
          std::clamp(center_l[i], boundary.right(i) + clamp_width * 0.5,
                     boundary.left(i) - clamp_width * 0.5);
    }
  }

  // Shrink to fit objects.
  std::vector<Vec2d> center_xy;
  for (int i = 0; i < boundary.size(); ++i) {
    const auto& station = drive_passage.station(StationIndex(i));
    center_xy.push_back(station.lat_point(center_l[i]));
  }
  if (!is_lane_changing) {
    outer_boundary = ShrinkPathBoundaryForObject(
        drive_passage, st_traj_mgr, plan_start_point, s_vec, center_l,
        center_xy, boundary, curb_boundary, std::move(outer_boundary), psmm,
        sl_box);
    outer_boundary.InnerClampBy(outer_kinematic_boundary);
    outer_boundary.OuterClampBy(curb_boundary);
  }

  // Post-process outer boundary to avoid zigzags.
  outer_boundary =
      PathBoundary(PostprocessOuterBoundary(s_vec, boundary.right_vec(),
                                            outer_boundary.right_vec()),
                   PostprocessOuterBoundary(s_vec, boundary.left_vec(),
                                            outer_boundary.left_vec()));
  opt_outer_boundary =
      PathBoundary(PostprocessOuterBoundary(s_vec, boundary.right_vec(),
                                            opt_outer_boundary.right_vec()),
                   PostprocessOuterBoundary(s_vec, boundary.left_vec(),
                                            opt_outer_boundary.left_vec()));

  outer_boundary.InnerClampBy(boundary);
  opt_outer_boundary.InnerClampBy(boundary);
  return BuildPathSlBoundary(
      drive_passage, std::move(s_vec), std::move(center_l), std::move(boundary),
      std::move(outer_boundary), std::move(opt_outer_boundary), offset_type,
      target_lane_offset);
}

}  // namespace st::planning
