

#include "planner/trajectory_optimizer/ddp/path_time_corridor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "plan_common/log_data.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "planner/planner_manager/planner_defs.h"
#include "planner/planner_manager/planner_util.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/trajectory_point.h"
#include "predictor/predicted_trajectory.h"

DEFINE_bool(traj_opt_draw_nudge_buffer, false,
            "If send nudge buffer manager results to worldview.");

namespace st {
namespace planning {
namespace optimizer {

namespace {
using BoundaryInfo = PathTimeCorridor::BoundaryInfo;
using PositionType = PathTimeCorridor::PositionType;
using ObjectPositionInfo = PathTimeCorridor::ObjectPositionInfo;
using ObjectPositionInfos = PathTimeCorridor::ObjectPositionInfos;

void DrawPathTimeCorridorBoundary(
    const int plan_id, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const std::vector<std::vector<BoundaryInfo>>& left_boundary,
    const std::vector<std::vector<BoundaryInfo>>& right_boundary,
    const std::vector<int>& time_indices) {
  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  const auto& stations = drive_passage.stations();
  for (int i = 0; i < time_indices.size(); ++i) {
    if (i % 10 != 0) continue;
    const auto& left_boundary_i = left_boundary[i];
    const auto& right_boundary_i = right_boundary[i];
    std::vector<Vec2d> left_boundary_points;
    std::vector<Vec2d> right_boundary_points;
    std::vector<Vec2d> left_boundary_points_outer;
    std::vector<Vec2d> right_boundary_points_outer;
    for (int j = 0; j < path_sl_boundary.size(); j++) {
      const auto& station = stations[StationIndex(j)];
      left_boundary_points.push_back(
          station.lat_point(left_boundary_i[j].l_object));
      right_boundary_points.push_back(
          station.lat_point(right_boundary_i[j].l_object));
      left_boundary_points_outer.push_back(
          station.lat_point(left_boundary_i[j].l_outer_object));
      right_boundary_points_outer.push_back(
          station.lat_point(right_boundary_i[j].l_outer_object));
    }
    Log2DDS::LogLineV3(prefix + "object-boundary_" + "inner", Log2DDS::kCoral,
                       {}, left_boundary_points);
    Log2DDS::LogLineV3(prefix + "object-boundary_" + "inner", Log2DDS::kCoral,
                       {}, right_boundary_points);
    Log2DDS::LogLineV3(prefix + "object-boundary_" + "outer", Log2DDS::kCoral,
                       {}, left_boundary_points_outer);
    Log2DDS::LogLineV3(prefix + "object-boundary_" + "outer", Log2DDS::kCoral,
                       {}, right_boundary_points_outer);
  }
  std::vector<Vec2d> left_curb_points;
  std::vector<Vec2d> right_curb_points;
  for (int j = 0; j < path_sl_boundary.size(); ++j) {
    const auto& station = stations[StationIndex(j)];
    left_curb_points.push_back(
        station.lat_point(left_boundary.front()[j].l_curb));
    right_curb_points.push_back(
        station.lat_point(right_boundary.front()[j].l_curb));
  }
  Log2DDS::LogLineV3(prefix + "object-boundary_" + "curb", Log2DDS::kCoral, {},
                     left_curb_points);
  Log2DDS::LogLineV3(prefix + "object-boundary_" + "curb", Log2DDS::kCoral, {},
                     right_curb_points);
}

void SampleBoundaryForObjectSegment(bool left, double init_start_s,
                                    double init_end_s, double sample_step,
                                    const Segment2d& line,
                                    const FrenetFrame& dp_frenet,
                                    const SpacetimeObjectTrajectory& traj,
                                    std::vector<BoundaryInfo>* boundary) {
  const int indices = std::ceil(line.length() / sample_step);
  const double actual_step = line.length() / static_cast<double>(indices);
  const int max_idx = static_cast<int>(boundary->size() - 1);
  for (int i = 0; i <= indices; ++i) {
    const Vec2d pt = line.start() + actual_step * static_cast<double>(i) *
                                        line.unit_direction();
    FrenetCoordinate sl;
    Vec2d normal;
    std::pair<int, int> index_pair;
    double alpha;
    dp_frenet.XYToSL(pt, &sl, &normal, &index_pair, &alpha);
    if (sl.s > init_end_s) continue;
    if (sl.s < init_start_s) continue;
    const int cur_idx = std::clamp(index_pair.first, 0, max_idx);
    const int next_idx = std::clamp(index_pair.second, 0, max_idx);
    const double l = sl.l;
    if (left) {
      if (l <= (*boundary)[cur_idx].l_outer_object) {
        (*boundary)[cur_idx].type =
            PathTimeCorridor::BoundaryInfo::GetObjectType(traj.object_type());
        (*boundary)[cur_idx].l_outer_object = l;
        (*boundary)[cur_idx].object_ptr = &traj;
      }
      if (l <= (*boundary)[cur_idx].l_object) {
        (*boundary)[cur_idx].l_object = l;
      }
      if (l <= (*boundary)[next_idx].l_outer_object) {
        (*boundary)[next_idx].type =
            PathTimeCorridor::BoundaryInfo::GetObjectType(traj.object_type());
        (*boundary)[next_idx].l_outer_object = l;
        (*boundary)[next_idx].object_ptr = &traj;
      }
      if (l <= (*boundary)[next_idx].l_object) {
        (*boundary)[next_idx].l_object = l;
      }
    } else {
      if (l >= (*boundary)[cur_idx].l_outer_object) {
        (*boundary)[cur_idx].type =
            PathTimeCorridor::BoundaryInfo::GetObjectType(traj.object_type());
        (*boundary)[cur_idx].l_outer_object = l;
        (*boundary)[cur_idx].object_ptr = &traj;
      }
      if (l >= (*boundary)[cur_idx].l_object) {
        (*boundary)[cur_idx].l_object = l;
      }
      if (l >= (*boundary)[next_idx].l_outer_object) {
        (*boundary)[next_idx].type =
            PathTimeCorridor::BoundaryInfo::GetObjectType(traj.object_type());
        (*boundary)[next_idx].l_outer_object = l;
        (*boundary)[next_idx].object_ptr = &traj;
      }
      if (l >= (*boundary)[next_idx].l_object) {
        (*boundary)[next_idx].l_object = l;
      }
    }
  }
}

void SampleBoundaryForContour(
    const DrivePassage& drive_passage, double init_start_s, double init_end_s,
    const Vec2d& contour_pos, const Polygon2d& contour,
    const FrenetFrame& init_frenet, const FrenetFrame& dp_frenet,
    const SpacetimeObjectTrajectory& traj,
    std::vector<BoundaryInfo>* left_boundary,
    std::vector<BoundaryInfo>* right_boundary,
    ObjectPositionInfos* object_position_infos,
    const VehicleGeometryParamsProto& veh_geo_params,
    const bool is_static_object, const double time_point = 0.0) {
  constexpr double kSegmentSampleStep = 1.0;  // m
  // Get nudge direction for object. Get object position.
  FrenetCoordinate init_frenet_center;
  Vec2d init_center_normal;
  init_frenet.XYToSL(contour_pos, &init_frenet_center, &init_center_normal);
  const std::string object_id = std::string(traj.object_id());
  const double init_object_s = init_frenet_center.s;
  const double init_object_l = init_frenet_center.l;
  double object_s = init_object_s;
  double object_l = init_object_l;
  const auto& frenet_center_drive_passge =
      drive_passage.QueryFrenetBoxAtContour(contour);
  if (frenet_center_drive_passge.ok()) {
    object_s = frenet_center_drive_passge.value().center_s();
    object_l = frenet_center_drive_passge.value().center_l();
  }
  const double position_threshold = 0.0;  // 0.5 * veh_geo_params.width();
  PositionType position_type = PositionType::POSITION_UNKNOWN;
  if (init_object_l > position_threshold) {
    position_type = optimizer::PositionType::POSITION_LEFT;
  } else if (init_object_l < -position_threshold) {
    position_type = optimizer::PositionType::POSITION_RIGHT;
  }
  ObjectPositionInfo object_position_info{
      is_static_object, time_point,    object_s,     object_l,
      init_object_s,    init_object_l, position_type};
  (*object_position_infos)[object_id].emplace_back(object_position_info);
  // Find half contour based on nudge direction.
  Vec2d front, back;
  int front_index, back_index;
  contour.ExtremePoints(-init_center_normal.Perp(), &back_index, &front_index,
                        &back, &front);
  const bool left = init_frenet_center.l > 0.0;
  const auto& contour_lines = contour.line_segments();
  if (left) {
    for (int i = back_index; i != front_index; i = contour.Next(i)) {
      SampleBoundaryForObjectSegment(left, init_start_s, init_end_s,
                                     kSegmentSampleStep, contour_lines[i],
                                     dp_frenet, traj, left_boundary);
    }
  } else {
    for (int i = front_index; i != back_index; i = contour.Next(i)) {
      SampleBoundaryForObjectSegment(left, init_start_s, init_end_s,
                                     kSegmentSampleStep, contour_lines[i],
                                     dp_frenet, traj, right_boundary);
    }
  }
}

std::vector<prediction::PredictionObjectState> SampleObjectStates(
    const std::vector<int>& time_indices,
    absl::Span<const prediction::PredictionObjectState> states) {
  std::vector<prediction::PredictionObjectState> sampled_states;
  sampled_states.reserve(time_indices.size());
  for (const auto k : time_indices) {
    if (k == 0) {
      sampled_states.emplace_back(states.front());
    } else if (k < states.size()) {
      sampled_states.emplace_back(states[k]);
    }
  }
  return sampled_states;
}
}  // namespace

PathTimeCorridor::PathTimeCorridor(
    const DrivePassage* drive_passage, const PathSlBoundary* path_sl_boundary,
    std::vector<std::vector<PathTimeCorridor::BoundaryInfo>> left_boundary,
    std::vector<std::vector<PathTimeCorridor::BoundaryInfo>> right_boundary,
    std::vector<int> time_indices, ObjectPositionInfos object_position_infos)
    : drive_passage_(drive_passage),
      path_sl_boundary_(path_sl_boundary),
      left_boundary_(std::move(left_boundary)),
      right_boundary_(std::move(right_boundary)),
      time_indices_(std::move(time_indices)),
      object_position_infos_(std::move(object_position_infos)) {
  CHECK_NOTNULL(path_sl_boundary_);
  CHECK_EQ(time_indices_.size(), left_boundary_.size());
  CHECK_EQ(time_indices_.size(), right_boundary_.size());
  for (size_t i = 0; i < time_indices_.size(); ++i) {
    CHECK_EQ(path_sl_boundary->size(), left_boundary_[i].size());
    CHECK_EQ(path_sl_boundary->size(), right_boundary_[i].size());
  }
}

std::pair<PathTimeCorridor::BoundaryInfo, PathTimeCorridor::BoundaryInfo>
PathTimeCorridor::QueryBoundaryL(double s, double t) const {
  const StationIndex index = drive_passage_->FindNearestStationIndexAtS(s);
  const int idx = std::min(index.value(), path_sl_boundary_->size() - 1);
  int time_idx = time_indices_.size() - 1;
  for (int i = 0; i < time_indices_.size() - 1; ++i) {
    if (kTrajectoryTimeStep * time_indices_[i] <= t &&
        kTrajectoryTimeStep * time_indices_[i + 1] > t) {
      time_idx = i;
    }
  }
  return std::make_pair(right_boundary_[time_idx][idx],
                        left_boundary_[time_idx][idx]);
}

std::vector<ObjectPositionInfo> PathTimeCorridor::QueryObjectPositionInfo(
    std::string object_id) const {
  if (object_position_infos_.find(object_id) != object_position_infos_.end()) {
    return object_position_infos_.at(object_id);
  } else {
    return {};
  }
}

absl::StatusOr<PathTimeCorridor> BuildPathTimeCorridor(
    const int plan_id, std::string_view base_name,
    const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& veh_geo_params,
    const double trajectory_time_step) {
  std::vector<BoundaryInfo> left_boundary_static;
  std::vector<BoundaryInfo> right_boundary_static;
  ObjectPositionInfos object_position_infos;
  left_boundary_static.reserve(path_sl_boundary.size());
  right_boundary_static.reserve(path_sl_boundary.size());
  int idx = 0;
  const auto& right_l_vector = path_sl_boundary.opt_right_l_vector();
  const auto& left_l_vector = path_sl_boundary.opt_left_l_vector();
  const auto& target_right_l_vector = path_sl_boundary.target_right_l_vector();
  const auto& target_left_l_vector = path_sl_boundary.target_left_l_vector();
  for (const auto& station : drive_passage.stations()) {
    if (idx >= path_sl_boundary.size()) break;
    const auto [right_curb, left_curb] = station.QueryCurbOffsetAt(0.0).value();
    ;
    right_boundary_static.push_back(
        BoundaryInfo{.type = right_l_vector[idx] > right_curb
                                 ? BoundaryInfo::Type::LANE_BOUNDARY
                                 : BoundaryInfo::Type::CURB,
                     .l_outer_boundary = right_l_vector[idx],
                     .l_boundary = target_right_l_vector[idx],
                     .l_outer_object = right_l_vector[idx],
                     .l_object = target_right_l_vector[idx],
                     .l_curb = right_curb,
                     .object_ptr = nullptr});
    left_boundary_static.push_back(
        BoundaryInfo{.type = left_l_vector[idx] < left_curb
                                 ? BoundaryInfo::Type::LANE_BOUNDARY
                                 : BoundaryInfo::Type::CURB,
                     .l_outer_boundary = left_l_vector[idx],
                     .l_boundary = target_left_l_vector[idx],
                     .l_outer_object = left_l_vector[idx],
                     .l_object = target_left_l_vector[idx],
                     .l_curb = left_curb,
                     .object_ptr = nullptr});

    idx++;
  }

  std::vector<Vec2d> init_points;
  init_points.reserve(init_traj.size());
  for (const auto& pt : init_traj) {
    init_points.push_back(pt.pos());
  }
  const auto init_frenet_frame =
      BuildKdTreeFrenetFrame(init_points,
                             /*down_sample_raw_points=*/true);
  if (!init_frenet_frame.ok()) {
    return absl::OutOfRangeError("init_frenet_frame build failed.");
  }

  // Use frenet frame in drive passage to query point.
  const auto& dp_frenet_frame = *drive_passage.frenet_frame();

  const auto init_end_frenet_dp =
      dp_frenet_frame.XYToSL(init_traj.back().pos());
  const double init_end_s =
      init_end_frenet_dp.s + veh_geo_params.front_edge_to_center();

  const auto init_start_frenet_dp =
      dp_frenet_frame.XYToSL(init_traj.front().pos());
  const double init_start_s =
      init_start_frenet_dp.s - veh_geo_params.back_edge_to_center();

  // Firstly, Sample boundary for static or stationary object.
  const auto& spacetime_trajs = st_planner_object_traj.trajectories;
  const auto& trajectory_infos = st_planner_object_traj.trajectory_infos;
  const int num_trajs = spacetime_trajs.size();
  for (const auto& traj : spacetime_trajs) {
    if ((prediction::IsStaticObjectType(traj.object_type()) ||
         traj.is_stationary()) &&
        leading_trajs.find(std::string(traj.traj_id())) ==
            leading_trajs.end()) {
      SampleBoundaryForContour(drive_passage, init_start_s, init_end_s,
                               traj.pose().pos(), traj.contour(),
                               *init_frenet_frame, dp_frenet_frame, traj,
                               &left_boundary_static, &right_boundary_static,
                               &object_position_infos, veh_geo_params, true);
    }
  }

  // Secondly, Sample boundary for dynamic object.
  std::vector<std::vector<BoundaryInfo>> left_boundary;
  std::vector<std::vector<BoundaryInfo>> right_boundary;

  const int timestep_ratio =
      static_cast<int>(trajectory_time_step / kTrajectoryTimeStep);
  const int obj_state_size =
      static_cast<int>(kTrajectorySteps / timestep_ratio);
  std::vector<int> time_indices(obj_state_size);

  for (int i = 0; i < obj_state_size; ++i) {
    time_indices[i] = i * timestep_ratio;
  }
  left_boundary.resize(time_indices.size(), left_boundary_static);
  right_boundary.resize(time_indices.size(), right_boundary_static);

  for (int i = 0; i < num_trajs; ++i) {
    const auto& traj = spacetime_trajs[i];
    const auto& trajectory_info = trajectory_infos[i];
    if (!prediction::IsStaticObjectType(traj.object_type()) &&
        !traj.is_stationary() &&
        leading_trajs.find(std::string(traj.traj_id())) ==
            leading_trajs.end()) {
      const auto states = SampleObjectStates(time_indices, traj.states());
      for (int k = 0; k < states.size(); ++k) {
        SampleBoundaryForContour(drive_passage, init_start_s, init_end_s,
                                 states[k].traj_point->pos(), states[k].contour,
                                 *init_frenet_frame, dp_frenet_frame, traj,
                                 &left_boundary[k], &right_boundary[k],
                                 &object_position_infos, veh_geo_params, false,
                                 trajectory_time_step * k);
      }
    }
  }

  if (FLAGS_traj_opt_draw_nudge_buffer) {
    DrawPathTimeCorridorBoundary(plan_id, drive_passage, path_sl_boundary,
                                 left_boundary, right_boundary, time_indices);
  }

  return PathTimeCorridor(&drive_passage, &path_sl_boundary,
                          std::move(left_boundary), std::move(right_boundary),
                          std::move(time_indices),
                          std::move(object_position_infos));
}

}  // namespace optimizer
}  // namespace planning
}  // namespace st
