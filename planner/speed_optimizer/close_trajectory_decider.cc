

#include "planner/speed_optimizer/close_trajectory_decider.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_format.h"
#include "plan_common/log_data.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "plan_common/plan_common_defs.h"
#include "planner/speed_optimizer/speed_finder_util.h"
#include "plan_common/speed/st_speed/speed_limit.h"
#include "planner/speed_optimizer/st_close_trajectory.h"
#include "planner/speed_optimizer/st_graph.h"

namespace st::planning {
namespace {
using SpeedLimitRange = SpeedLimit::SpeedLimitRange;
using StNearestPoint = StCloseTrajectory::StNearestPoint;

constexpr double kMaxDecelByCloseTraj = -1.5;          // m/s^2.
constexpr double kMaxDecelByOncomingCloseTraj = -0.5;  // m/s^2.

constexpr double kSpeedLimitBeforeRangeDist = 10.0;  // m.
constexpr double kSpeedLimitAfterRangeDist = 3.0;    // m.

const std::vector<double> kCloseObjectLatDistRange = {0.0, 0.5, 1.0, 1.5,
                                                      2.0};  // m.

const PiecewiseLinearFunction<double, double> kMovingVehicleMaxRelVPlf = {
    kCloseObjectLatDistRange, {3.0, 4.0, 6.0, 8.0, 10.0}};  // m/s.
const PiecewiseLinearFunction<double, double> kMovingCyclistMaxRelVPlf = {
    kCloseObjectLatDistRange, {3.0, 4.0, 6.0, 8.0, 10.0}};  // m/s.
const PiecewiseLinearFunction<double, double> kMovingPedestrianMaxRelVPlf = {
    kCloseObjectLatDistRange, {3.0, 4.0, 6.0, 8.0, 10.0}};  // m/s.

const PiecewiseLinearFunction<double, double> kTimeAttenuationPlfOncoming = {
    {0.0, 10.0}, {1.0, 1.6}};
const PiecewiseLinearFunction<double, double> kTimeAttenuationPlf = {
    {0.0, 10.0}, {1.0, 1.2}};
const PiecewiseLinearFunction<double, double> kProbabilityGainPlf = {
    {0.0, 1.0}, {1.2, 1.0}};

const PiecewiseLinearFunction<double, double> kTimeAttenuationPlfForCutIn = {
    {3.0, 7.0, 10.0}, {1.0, 1.08, 1.2}};

double GetMaxSpeedByNearestPoint(const StNearestPoint& nearest_point,
                                 StBoundaryProto::ObjectType object_type) {
  double max_rel_speed = 0.0;
  const double lat_dist = std::fabs(nearest_point.lat_dist);
  switch (object_type) {
    case StBoundaryProto::VEHICLE:
      max_rel_speed = kMovingVehicleMaxRelVPlf(lat_dist);
      break;
    case StBoundaryProto::CYCLIST:
      max_rel_speed = kMovingCyclistMaxRelVPlf(lat_dist);
      break;
    case StBoundaryProto::PEDESTRIAN:
      max_rel_speed = kMovingPedestrianMaxRelVPlf(lat_dist);
      break;
    case StBoundaryProto::STATIC:
    case StBoundaryProto::IMPASSABLE_BOUNDARY:
    case StBoundaryProto::PATH_BOUNDARY:
    case StBoundaryProto::VIRTUAL:
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::UNKNOWN_OBJECT:
      max_rel_speed = kMovingVehicleMaxRelVPlf(lat_dist);
  }
  return max_rel_speed + std::max(nearest_point.v, 0.0);
}

SpeedLimitRange MakeSpeedLimitRange(
    double time, std::string id,
    std::vector<const StCloseTrajectory*> st_close_trajs, double path_length,
    double av_speed, bool is_oncoming) {
  double probability = 0.0;
  StNearestPoint nearest_pt;
  for (const auto* st_close_traj : st_close_trajs) {
    const auto temp_pt = st_close_traj->GetNearestPointByTime(time);
    const double temp_probability = st_close_traj->probability();
    CHECK(temp_pt.has_value());
    nearest_pt.s += temp_pt->s * temp_probability;
    nearest_pt.v += temp_pt->v * temp_probability;
    nearest_pt.lat_dist += temp_pt->lat_dist * temp_probability;
    probability += temp_probability;
  }
  CHECK_GT(probability, 0.0);
  nearest_pt.s /= probability;
  nearest_pt.v /= probability;
  nearest_pt.lat_dist /= probability;
  double speed_limit =
      GetMaxSpeedByNearestPoint(nearest_pt, st_close_trajs[0]->object_type());
  const double time_gain = is_oncoming ? kTimeAttenuationPlfOncoming(time)
                                       : kTimeAttenuationPlf(time);
  const double prob_gain = kProbabilityGainPlf(probability);
  speed_limit *= time_gain * prob_gain;
  const double start_s =
      std::max(0.0, nearest_pt.s - kSpeedLimitBeforeRangeDist);
  const double end_s =
      std::min(path_length, nearest_pt.s + kSpeedLimitAfterRangeDist);
  // Prevent hard braking.
  const double decel =
      (Sqr(speed_limit) - Sqr(av_speed)) / ((2.0 * start_s) + 1e-3);
  if (decel < kMaxDecelByCloseTraj) {
    speed_limit = std::sqrt(
        std::max(0.0, 2.0 * kMaxDecelByCloseTraj * start_s + Sqr(av_speed)));
  }
  if (is_oncoming && decel < kMaxDecelByOncomingCloseTraj) {
    speed_limit = std::sqrt(std::max(
        0.0, 2.0 * kMaxDecelByOncomingCloseTraj * start_s + Sqr(av_speed)));
  }
  std::string debug_info = absl::StrFormat(
      "CLOSE_OBJECT(moving) Id: %s, lat_dis: %.2f, lon_dis: %.2f,\nobj_lon_v: "
      "%.2f, time: "
      "%.2f, prob: %.2f, value: %.2f",
      id, nearest_pt.lat_dist, nearest_pt.s, nearest_pt.v, time, probability,
      speed_limit);
  return SpeedLimitRange{.start_s = start_s,
                         .end_s = end_s,
                         .speed_limit = speed_limit,
                         .info = std::move(debug_info)};
}

std::optional<SpeedLimit> MakeMovingSpeedLimit(
    double time, absl::Span<const StCloseTrajectory> st_close_trajs,
    double path_length, double av_speed) {
  std::vector<SpeedLimitRange> speed_limit_ranges;
  absl::flat_hash_map<std::string, std::vector<const StCloseTrajectory*>>
      obj_close_trajs_map;
  absl::flat_hash_map<std::string, std::vector<const StCloseTrajectory*>>
      oncoming_obj_close_trajs_map;
  for (const StCloseTrajectory& st_close_traj : st_close_trajs) {
    if (!InRange(time, st_close_traj.min_t(), st_close_traj.max_t())) {
      continue;
    }
    auto min_t_np = st_close_traj.GetNearestPointByTime(st_close_traj.min_t());
    auto max_t_np = st_close_traj.GetNearestPointByTime(st_close_traj.max_t());
    if (st_close_traj.object_type() == StBoundaryProto::VEHICLE &&
        min_t_np.has_value() && max_t_np.has_value() &&
        min_t_np.value().s > (std::fabs(av_speed - min_t_np.value().v)) * 1.0 &&
        min_t_np.value().s > max_t_np.value().s) {
      auto obs_t_np = st_close_traj.GetNearestPointByTime(time);
      constexpr double kSpeedLimitRangeDist = 10.0;  // m
      // continue;
      if (obs_t_np.has_value() &&
          std::fabs(obs_t_np.value().s - av_speed * time) >
              kSpeedLimitRangeDist) {
        continue;
      }
      std::string debug = "";
      debug = "!!!!!!!!!!!!!!!" +
              absl::StrCat(st_close_traj.object_id(), ", ", time);
      Log2DDS::LogDataV0("MakeMovingSpeedLimit", debug);
      oncoming_obj_close_trajs_map[st_close_traj.object_id()].emplace_back(
          &st_close_traj);
    }
    obj_close_trajs_map[st_close_traj.object_id()].emplace_back(&st_close_traj);
  }
  for (const auto& [id, st_close_trajs] : obj_close_trajs_map) {
    bool is_oncoming = oncoming_obj_close_trajs_map.find(id) !=
                       oncoming_obj_close_trajs_map.end();
    if (st_close_trajs.size() > 0 &&
        st_close_trajs[0]->st_nearest_points().size() > 0) {
      double obj_vel = st_close_trajs[0]->st_nearest_points()[0].v;
      if (std::fabs(obj_vel) < Kph2Mps(10.0)) continue;
    }
    auto speed_limit_range = MakeSpeedLimitRange(
        time, id, st_close_trajs, path_length, av_speed, is_oncoming);
    speed_limit_ranges.push_back(std::move(speed_limit_range));
  }
  if (speed_limit_ranges.empty()) return std::nullopt;
  return SpeedLimit(speed_limit_ranges);
}
}  // namespace

std::vector<std::optional<SpeedLimit>> GetMovingCloseTrajSpeedLimits(
    absl::Span<const StCloseTrajectory> st_close_trajs, double path_length,
    double av_speed, double time_step, double max_time) {
  if (st_close_trajs.empty()) return {};
  std::vector<std::optional<SpeedLimit>> dynamic_speed_limit;
  dynamic_speed_limit.resize(kTrajectorySteps + 1);
  constexpr double kTimeHorizon = kTrajectorySteps * kTrajectoryTimeStep;
  const double max_search_time = std::min(kTimeHorizon, max_time);
  for (double time = st_close_trajs.front().min_t(); time <= max_search_time;
       time += time_step) {
    auto speed_limit =
        MakeMovingSpeedLimit(time, st_close_trajs, path_length, av_speed);
    if (!speed_limit.has_value()) continue;
    const int time_idx = static_cast<int>(time / time_step);
    dynamic_speed_limit[time_idx] = std::move(speed_limit);
  }
  return dynamic_speed_limit;
}
std::optional<SpeedLimit> GetRightTurnCloseSpeedLimit(
    const RightTurnCloseSpeedLimitInput& input,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const std::vector<DrivingProcess>& driving_process_seq) {
  std::optional<SpeedLimit> speed_limit;

  // const auto& path_semantics = *input.path_semantics;
  // if (path_semantics.empty() || path_semantics.front().lane_semantic !=
  //                                   LaneSemantic::INTERSECTION_RIGHT_TURN) {
  //   Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit", "11111");
  //   // return speed_limit;
  // }
  if (driving_process_seq.empty()) {
    Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                       "rerturn {driving_process_seq empty }");
    return speed_limit;
  }
  const auto& path_points = *input.path;
  const auto driving_zone =
      std::lower_bound(driving_process_seq.begin(), driving_process_seq.end(),
                       path_points.front().s(),
                       [](const DrivingProcess& driving_process, double av_s) {
                         return driving_process.end_s < av_s;
                       });
  if (driving_zone == driving_process_seq.end()) {
    Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                       "rerturn { no driving_zone}");
    return speed_limit;
  }
  if (driving_zone->lane_semantic != LaneSemantic::INTERSECTION_RIGHT_TURN) {
    Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                       "rerturn { not in  turn right lane}");
    return speed_limit;
  }
  constexpr double kBufferOutRightLane = 3.0;
  if (driving_zone->end_s <
      input.vehicle_geom->front_edge_to_center() + kBufferOutRightLane) {
    Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                       "rerturn { out turn right lane}");
    return speed_limit;
  }
  // const auto& path_points = *input.path;
  const auto& ego_predict_path = *input.ego_predict_path;
  const auto av_shapes =
      BuildAvShapes(*input.vehicle_geom, *input.ego_predict_path);
  if (input.ego_predict_path->size() < 2) {
    return speed_limit;
  }
  // Check if there is a significant difference between the predicted path and
  // the planned path
  double s_check = std::min(12.0, std::max(4.0 * input.current_v, 8.0));
  const auto ego_predict_point = ego_predict_path.Evaluate(s_check);
  const auto ego_path_point = path_points.Evaluate(s_check);
  const double dist =
      path_points.XYToSL(Vec2d(ego_predict_point.x(), ego_predict_point.y())).l;
  // Vec2d(ego_predict_point.x(), ego_predict_point.y())
  //     .DistanceTo(Vec2d(ego_path_point.x(), ego_path_point.y()));
  constexpr double kSimilar_path_dist = 0.35;  // m
  if (std::fabs(dist) < kSimilar_path_dist) {
    Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                       absl::StrCat("rerturn { Similar paths: }", dist));
    return speed_limit;
  }
  Log2DDS::LogDataV0(
      "GetRightTurnCloseSpeedLimit",
      absl::StrCat("rerturn { Similar paths: }", dist, " ", s_check));

  const auto path_kd_tree = BuildPathKdTree(*input.ego_predict_path);
  if (!path_kd_tree) {
    Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                       "rerturn { path_kd_tree empty: }");
    return speed_limit;
  }
  Vec2d start_point(path_points.front().x(), path_points.front().y());
  const auto& drive_passage = *input.drive_passage;
  // const auto ego_fpos = drive_passage.QueryFrenetCoordinateAt(start_point);
  const auto& ego_fpos = path_points.XYToSL(start_point);
  double ego_radius = Hypot(std::max(input.vehicle_geom->front_edge_to_center(),
                                     input.vehicle_geom->back_edge_to_center()),
                            input.vehicle_geom->right_edge_to_center());

  //  const SpacetimeTrajectoryManager& st_traj_mgr,
  // get on st obs
  const auto& speed_traj_manager = *input.st_traj_mgr;
  const auto& moving_spacetime_objects =
      speed_traj_manager.moving_object_trajs();
  bool need_break = false;
  std::string need_break_object_id = "";
  std::vector<StBoundaryPoints> st_boundaries_points;
  for (size_t i = 0; i < moving_spacetime_objects.size(); i++) {
    const auto* st_traj = moving_spacetime_objects.at(i);
    const std::string traj_id(st_traj->traj_id());
    const std::string object_id(st_traj->object_id());
    if (st_traj->planner_object().type() != ObjectType::OT_VEHICLE &&
        st_traj->planner_object().type() != ObjectType::OT_LARGE_VEHICLE &&
        st_traj->planner_object().type() != ObjectType::OT_UNKNOWN_MOVABLE) {
      continue;
    }
    // 0
    if (st_traj->trajectory().intention() !=
        TrajectoryIntention::INTENTION_TURN_RIGHT) {
      continue;
    }
    // 1
    const auto& obs_fbox =
        path_points.XYToSL(st_traj->planner_object().contour(),
                           st_traj->states().front().box.center());

    // const auto obs_fbox = drive_passage.QueryFrenetBoxAtContour(
    //     st_traj->planner_object().contour());
    double ds = std::numeric_limits<double>::max();
    bool is_left_side = false;
    // if (obs_fbox.ok() && ego_fpos.ok()) {
    ds = obs_fbox.s_min - ego_fpos.s -
         input.vehicle_geom->front_edge_to_center();
    if (ds > 20.0 || ds < 1.0) {
      Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                         object_id + " : traj_id " + traj_id + "--333333333");
      continue;
    }
    if (obs_fbox.l_min <= 0 ||
        (std::abs(obs_fbox.l_min) > std::abs(obs_fbox.l_max))) {
      Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                         object_id + " : traj_id " + traj_id + "--44444444");
      continue;
    }
    Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                       object_id + " : traj_id " + traj_id + "--{{!!!!!!}}");

    ///////////////////////////////////////////////////////////////////////////////////////

    const auto& object_states = st_traj->states();
    // CHECK_GT(object_states.size(), 0);

    // const auto& path_points = *path_points_;
    // std::vector<AgentNearestPoint> agent_nearest_points;

    int low_idx, high_idx;
    auto& boundary_points = st_boundaries_points.emplace_back();
    for (size_t i = 0; i < object_states.size(); ++i) {
      const auto& cur_state = object_states[i];
      const auto* traj_point = cur_state.traj_point;

      const Box2d& obj_box = st_traj->bounding_box();
      const double obj_radius =
          obj_box.diagonal() * 0.5 + st_traj->required_lateral_gap();
      const double search_radius = obj_radius + ego_radius;

      const Vec2d& search_point = cur_state.box.center();
      auto indices = path_kd_tree->GetSegmentIndexInRadius(
          search_point.x(), search_point.y(), search_radius);
      if (indices.empty()) {
        continue;
      }
      std::stable_sort(indices.begin(), indices.end());
      const auto& obj_shape = cur_state.contour;
      for (auto it = indices.begin(); it != indices.end(); ++it) {
        const auto& raw_av_shape = (av_shapes)[*it];
        if (raw_av_shape &&
            raw_av_shape->HasOverlapWithBuffer(obj_shape, 0.2, 0.2, false)) {
          low_idx = *it;
          need_break_object_id = object_id;
          // Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
          //                    object_id + " : traj_id " + traj_id + "--11");
          need_break = true;
          break;
        }
      }

      for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
        const auto& raw_av_shape = (av_shapes)[*it];
        if (raw_av_shape &&
            raw_av_shape->HasOverlapWithBuffer(obj_shape, 0.2, 0.2, false)) {
          high_idx = *it;
          need_break_object_id = object_id;
          // Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
          //                    object_id + " : traj_id " + traj_id + "--11");
          need_break = true;
          break;
        }
      }
      // const double middle_s =
      //     (ego_predict_path[high_idx].s() + ego_predict_path[low_idx].s()) *
      //     0.5;
      // const auto middle_point = ego_predict_path.Evaluate(middle_s);
      // const double middle_speed =
      //     traj_point->v() *
      //     Vec2d::FastUnitFromAngle(traj_point->theta())
      //         .dot(Vec2d::FastUnitFromAngle(middle_point.theta()));
      // const double low_s = (ego_predict_path)[low_idx].s();
      // const double high_s = (ego_predict_path)[high_idx].s();
      // boundary_points.speed_points.emplace_back(middle_speed,
      // traj_point->t()); boundary_points.lower_points.emplace_back(low_s,
      // traj_point->t()); boundary_points.upper_points.emplace_back(high_s,
      // traj_point->t()); boundary_points.overlap_infos.push_back(
      //     OverlapInfo{.time = traj_point->t(),
      //                 .obj_idx = static_cast<int>(i),
      //                 .av_start_idx = low_idx,
      //                 .av_end_idx = high_idx});
    }
  }
  if (need_break) {
    Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
                       need_break_object_id + " : need break ");
    std::vector<SpeedLimitRange> speed_limit_ranges;
    const auto& path_points = *input.path;
    const int num_points = path_points.size();
    speed_limit_ranges.reserve(num_points);
    // first: s second: v
    std::pair<double, double> prev_speed_limit_point =
        std::make_pair(path_points[0].s(), input.current_v);
    double last_sample_s = 0.0;
    for (int i = 1; i < num_points; ++i) {
      // Only check the speed limit at every meter to save computation.
      constexpr double kSpeedLimitSampleRange = 1.0;  // Meters.
      if (path_points[i].s() - last_sample_s > kSpeedLimitSampleRange ||
          i == num_points - 1) {
        last_sample_s = path_points[i].s();
        speed_limit_ranges.push_back(
            {.start_s = prev_speed_limit_point.first,
             .end_s = path_points[i].s(),
             .speed_limit = input.current_v,
             .info = SpeedLimitTypeProto::Type_Name(
                 SpeedLimitTypeProto_Type_RIGHT_TURN_CLOSE)});
        prev_speed_limit_point =
            std::make_pair(path_points[i].s(), input.current_v);
      }
    }
    CHECK(!speed_limit_ranges.empty());
    speed_limit = SpeedLimit(speed_limit_ranges);
  }
  return speed_limit;
}
}  // namespace st::planning
