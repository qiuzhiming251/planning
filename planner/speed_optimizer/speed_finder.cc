

#include "planner/speed_optimizer/speed_finder.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "plan_common/timer.h"
// #include "global/timer.h"
// #include "global/trace.h"
// #include "lite/check.h"
// #include "lite/logging.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/math/geometry/offset_rect.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/path_approx.h"
#include "plan_common/second_order_trajectory_point.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
// #include "ml/act_net_speed/act_net_speed_decider.h"
#include "object_manager/partial_spacetime_object_trajectory.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/overlap_info.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/planning_macros.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction.h"
#include "planner/speed_optimizer/cipv_object_info.h"
#include "planner/speed_optimizer/close_trajectory_decider.h"
#include "planner/speed_optimizer/constraint_generator.h"
#include "planner/speed_optimizer/cross_blind_decider.h"
#include "planner/speed_optimizer/decider/gaming_st_boundary_modifier.h"
#include "planner/speed_optimizer/decider/pre_brake_decider.h"
#include "planner/speed_optimizer/decider/pre_brake_util.h"
#include "planner/speed_optimizer/decider/pre_st_boundary_modifier.h"
#include "planner/speed_optimizer/decider/st_boundary_pre_decider.h"
#include "planner/speed_optimizer/defensive_speed_decider.h"
#include "planner/speed_optimizer/ignore_decider.h"
#include "planner/speed_optimizer/interactive_speed_decision.h"
#include "planner/speed_optimizer/path_semantic_analyzer.h"
#include "planner/speed_optimizer/path_speed_combiner.h"
#include "planner/speed_optimizer/processed_spacetime_trajectory_manager.h"

#include "alternative_gaming/speed_gaming/speed_gaming_decider.h"
#include "plan_common/async/async_util.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/maps/st_point.h"
#include "plan_common/speed/st_speed/speed_limit.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/speed/st_speed/speed_point.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "plan_common/speed/st_speed/vt_speed_limit.h"
#include "planner/speed_optimizer/decider/speed_limit_decider.h"
#include "planner/speed_optimizer/object_scene_recognition.h"
#include "planner/speed_optimizer/speed_bound.h"
#include "planner/speed_optimizer/speed_finder_flags.h"
#include "planner/speed_optimizer/speed_finder_util.h"
#include "planner/speed_optimizer/speed_limit_generator.h"
#include "planner/speed_optimizer/speed_optimizer.h"
#include "planner/speed_optimizer/speed_optimizer_config_dispatcher.h"
#include "planner/speed_optimizer/speed_optimizer_object.h"
#include "planner/speed_optimizer/speed_optimizer_object_manager.h"
#include "planner/speed_optimizer/st_graph.h"
#include "planner/speed_optimizer/st_overlap_analyzer.h"
#include "planner/speed_optimizer/standstill_distance_decider.h"
#include "planner/speed_optimizer/time_buffer_decider.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/motion_util.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/time_util.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction_common.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
// #include "vis/common/color.h"

namespace st {
namespace planning {

namespace {
absl::Status OptimizeSpeed(
    std::string_view base_name, double init_v, double init_a, double delta_t,
    const SpeedOptimizerObjectManager& opt_obj_mgr,
    const SpeedBoundMapType& speed_bound_map, double path_length,
    const SpeedVector& reference_speed,
    const std::vector<AccelBounds>& accel_bounds,
    const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  // ("OptimizeSpeed");

  CHECK_NOTNULL(optimized_speed);
  CHECK_NOTNULL(speed_finder_debug_proto);
  CHECK_GT(path_length, 0.0);

  SpeedOptimizer speed_optimizer(
      base_name, init_v, init_a, &motion_constraint_params,
      &speed_finder_params, path_length,
      motion_constraint_params.default_speed_limit(), delta_t);
  const absl::Status status = speed_optimizer.Optimize(
      opt_obj_mgr, speed_bound_map, reference_speed, accel_bounds,
      time_aligned_prev_traj, optimized_speed, speed_finder_debug_proto);
  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Speed optimizer failed: ", status.message()));
  }

  return absl::OkStatus();
}

std::optional<TrajectoryEndInfoProto> SetTrajectoryEndInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    double speed_length) {
  std::optional<TrajectoryEndInfoProto> traj_end_info = std::nullopt;
  for (const StBoundaryWithDecision& st_boundary_with_decision :
       st_boundaries_with_decision) {
    const auto* st_boundary = st_boundary_with_decision.st_boundary();
    if (!st_boundary->is_stationary()) {
      continue;
    }

    const double min_s = st_boundary->min_s();
    const double object_upper_bound =
        min_s - st_boundary_with_decision.follow_standstill_distance();
    const double intrusion_value = speed_length - object_upper_bound;

    if (intrusion_value > 0.0) {
      if (!traj_end_info.has_value() || min_s < traj_end_info->end_s()) {
        TrajectoryEndInfoProto end_info_proto;
        end_info_proto.set_st_boundary_id(st_boundary->id());
        end_info_proto.set_end_s(min_s);
        end_info_proto.set_intrusion_value(intrusion_value);
        end_info_proto.set_type(st_boundary->source_type());
        traj_end_info = std::move(end_info_proto);
      }
    }
  }
  return traj_end_info;
}

ObjectsPredictionProto ConstructModifiedPrediction(
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const int plan_id) {
  // Use these elements to construct ObjectPrediction.
  struct PredElements {
    std::vector<prediction::PredictedTrajectory> trajs;
    const ObjectProto* object_proto = nullptr;
  };

  std::map<std::string, PredElements> processed_preds;
  for (const auto& [traj_id, st_obj] : processed_st_objects) {
    const auto object_id =
        SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(traj_id);
    if (processed_preds[object_id].object_proto == nullptr) {
      processed_preds[object_id].object_proto =
          &st_obj.planner_object().object_proto();
    }
    processed_preds[object_id].trajs.push_back(st_obj.trajectory());
  }

  ObjectsPredictionProto modified_prediction;
  const auto& prefix = Log2DDS::TaskPrefix(plan_id) + "modified-pre_";
  for (auto& [object_id, pred_elements] : processed_preds) {
    // TODO: Set the correct prediction road status and intersection
    // status.

    const prediction::ObjectPrediction obj_pred(std::move(pred_elements.trajs),
                                                *pred_elements.object_proto);
    Log2DDS::LogDataV2(absl::StrCat(prefix, object_id),
                       obj_pred.DebugStringList());
    Log2DDS::LogPolygonV2(absl::StrCat(prefix, object_id, "_contour"),
                          Log2DDS::kMiddleBlueGreen, {},
                          obj_pred.contour().points());
    for (size_t i = 0; i < obj_pred.trajectories().size(); ++i) {
      Log2DDS::LogLineV2(
          absl::StrCat(prefix, object_id, "_traj-", i), Log2DDS::kDarkRed, {},
          obj_pred.trajectories()[i].points(),
          [](const prediction::PredictedTrajectoryPoint& p) {
            return p.pos().x();
          },
          [](const prediction::PredictedTrajectoryPoint& p) {
            return p.pos().y();
          });
    }
    obj_pred.ToProto(modified_prediction.add_objects());
  }

  return modified_prediction;
}

StBoundaryWithDecision GenerateStBoundaryWithDecisionByStopLine(
    const StGraph& st_graph,
    const ConstraintProto::PathStopLineProto& path_stop_line) {
  StBoundaryWithDecision stb_wd(st_graph.MapPathStopLine(path_stop_line),
                                StBoundaryProto::YIELD,
                                StBoundaryProto::CONSTRAINT_GENERATOR);
  stb_wd.set_follow_standstill_distance(path_stop_line.standoff());
  return stb_wd;
}

inline bool IsObjectBeyondAvFrontEdge(
    const PlannerObject& planner_obj, const PathPoint& av_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto& obj_contour = planner_obj.contour();
  const Vec2d av_tan = Vec2d::FastUnitFromAngle(av_point.theta());
  const Vec2d& obj_center = obj_contour.CircleCenter();
  return av_tan.Dot(obj_center - ToVec2d(av_point)) +
             obj_contour.CircleRadius() >
         vehicle_geometry_params.front_edge_to_center();
}

std::optional<std::string> GetAlertedFrontVehicleForHMI(
    const SpeedOptimizerObjectManager& opt_obj_mgr,
    const SpacetimeTrajectoryManager& traj_mgr,
    const PathPoint& current_path_point, const SpeedVector& optimized_speed,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    std::string_view base_name) {
  const double max_decel =
      std::min_element(
          optimized_speed.begin(), optimized_speed.end(),
          [](const auto& lhs, const auto& rhs) { return lhs.a() < rhs.a(); })
          ->a();
  constexpr double kDecelThres = -1.0;  // m/s^2.
  if (max_decel > kDecelThres) return std::nullopt;

  constexpr double kMaxTime = 3.0;  // s.
  constexpr double kDeltaT = 0.2;   // s.
  std::map<std::string, double> object_id_to_slack;
  for (const SpeedOptimizerObject& opt_obj :
       opt_obj_mgr.MovingFollowObjects()) {
    if (opt_obj.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (opt_obj.is_protective()) continue;
    if (opt_obj.object_type() != StBoundaryProto::VEHICLE) continue;
    const PlannerObject* planner_obj =
        CHECK_NOTNULL(traj_mgr.FindObjectByObjectId(opt_obj.id()));
    if (!IsObjectBeyondAvFrontEdge(*planner_obj, current_path_point,
                                   vehicle_geometry_params)) {
      continue;
    }
    for (double t = 0.0; t <= kMaxTime; t += kDeltaT) {
      const auto& overlap_state = opt_obj.GetOverlapStateByTime(t);
      if (!overlap_state.has_value()) continue;
      const auto speed_pt = optimized_speed.EvaluateByTime(t);
      CHECK(speed_pt.has_value());
      const double slack =
          speed_pt->s() - overlap_state->bound + overlap_state->lon_buffer;
      if (slack > 0.0) {
        object_id_to_slack[std::string(opt_obj.id())] += slack;
      }
    }
  }
  std::optional<std::string> alerted_front_vehicle_id;
  double max_slack = 0.0;
  for (const auto& [id, slack] : object_id_to_slack) {
    if (slack > max_slack) {
      max_slack = slack;
      alerted_front_vehicle_id = id;
    }
  }
  if (alerted_front_vehicle_id.has_value()) {
  }
  return alerted_front_vehicle_id;
}

std::optional<VtSpeedLimit> GenerateSoftAccSpeedLimit(
    const MotionConstraintParamsProto& motion_constraint_params, double start_v,
    double start_a, double soft_acc_jerk) {
  const double kEps = 1e-3;                 // instead of 0.0
  const double kHardBrakeThreshold = -2.0;  // m/ss
  const auto max_v = Mph2Mps(motion_constraint_params.default_speed_limit());
  const auto max_a = motion_constraint_params.max_acceleration();
  const auto min_a = motion_constraint_params.max_deceleration();
  const auto step_num = GetSpeedFinderTrajectorySteps(start_v, max_v);
  const auto end_t = kTrajectorySteps * kSpeedLimitProviderTimeStep;
  // avoid "pure acc" and "hard brake to acceleration" process too slow
  if (start_a > kEps || start_a < kHardBrakeThreshold) {
    return std::nullopt;
  }
  // calculate soft_acc_vt_limit
  VtSpeedLimit soft_acc_vt_limit;
  soft_acc_vt_limit.reserve(step_num + 1);
  ConstJerkMotion const_jerk_motion(start_v, start_a, min_a, max_a,
                                    soft_acc_jerk);
  for (int i = 0; i < step_num + 1; i++) {
    const double time = static_cast<double>(i) * kSpeedLimitProviderTimeStep;
    double current_v = max_v;
    if (time > -kEps && time < end_t + kEps) {
      current_v = std::min(const_jerk_motion.GetV(time), max_v);
    }
    current_v = std::max(current_v, 0.0);
    soft_acc_vt_limit.emplace_back(current_v, "soft_acc");
  }
  return soft_acc_vt_limit;
}

DiscretizedPath EgoPredictTrajectory(const PathPoint& current_path_point,
                                     double duration, double dt,
                                     double current_speed) {
  DiscretizedPath ego_predict_path;
  if (current_speed < 0.5) return ego_predict_path;
  PathPoint state = current_path_point;
  std::vector<Vec2d> pos;
  const double yaw_rate = current_speed * current_path_point.kappa();
  if (yaw_rate > 0.0) return ego_predict_path;
  for (double t = 0; t <= duration; t += dt) {
    state.set_x(state.x() + current_speed * std::cos(state.theta()) * dt);
    state.set_y(state.y() + current_speed * std::sin(state.theta()) * dt);
    state.set_theta(yaw_rate * dt + state.theta());
    state.set_s(state.s() + current_speed * dt);
    // state_speed = current_speed;
    ego_predict_path.push_back(state);
    pos.emplace_back(state.x(), state.y());
  }
  Log2DDS::LogLineV2("ego_predict_trajectory", Log2DDS::kOrange, {}, pos);
  return ego_predict_path;
}

std::unordered_map<std::string, const SpacetimeObjectTrajectory*>
GetAllOverlappedStObjectTrajs(
    absl::Span<const StBoundaryWithDecision> st_boundaries_wd,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const SpacetimeTrajectoryManager& traj_mgr) {
  std::unordered_map<std::string, const SpacetimeObjectTrajectory*>
      st_trajs_map;
  for (const auto& stb_wd : st_boundaries_wd) {
    if (stb_wd.decision_type() != StBoundaryProto::OVERTAKE &&
        stb_wd.decision_type() != StBoundaryProto::FOLLOW &&
        stb_wd.decision_type() != StBoundaryProto::YIELD) {
      continue;
    }
    if (!stb_wd.traj_id().has_value()) {
      continue;
    }
    const std::string traj_id = *stb_wd.traj_id();
    if (processed_st_objects.find(traj_id) != processed_st_objects.end()) {
      st_trajs_map.emplace(absl::StrCat(traj_id, "|m"),
                           &FindOrDie(processed_st_objects, traj_id));
      st_trajs_map.emplace(absl::StrCat(traj_id, "|raw"),
                           CHECK_NOTNULL(traj_mgr.FindTrajectoryById(traj_id)));
    } else {
      st_trajs_map.emplace(traj_id,
                           CHECK_NOTNULL(traj_mgr.FindTrajectoryById(traj_id)));
    }
  }
  return st_trajs_map;
}

struct PathContourPoints {
  std::vector<Vec2d> left_bound_points;
  std::vector<Vec2d> right_bound_points;
  std::vector<Vec2d> center_points;
};

PathContourPoints GetStTrajPathContourPoints(
    const SpacetimeObjectTrajectory& st_traj) {
  PathContourPoints path_contour_points;
  const auto& states = st_traj.states();
  if (states.size() < 2) return path_contour_points;
  const int pt_size = (states.size() + 1) / 2;
  path_contour_points.left_bound_points.reserve(pt_size);
  path_contour_points.right_bound_points.reserve(pt_size);
  path_contour_points.center_points.reserve(pt_size);
  bool prev_state_stationary = false;
  constexpr double kStationarySpeedThres = 0.1;  // m/s.
  const double lateral_gap = st_traj.required_lateral_gap();
  for (int i = 0; i < states.size(); i += 2) {
    const auto& traj_point = states[i].traj_point;
    const auto& pos = traj_point->pos();
    bool is_stationary = traj_point->v() < kStationarySpeedThres;
    if (is_stationary && prev_state_stationary) {
      continue;
    }
    prev_state_stationary = is_stationary;
    path_contour_points.center_points.push_back(traj_point->pos());
    const Polygon2d& contour = states[i].contour;
    const double perp_heaing = NormalizeAngle(traj_point->theta() - M_PI_2);
    Vec2d left_pt;
    Vec2d right_pt;
    contour.ExtremePoints(perp_heaing, &left_pt, &right_pt);

    const Vec2d perp_vec = Vec2d::FastUnitFromAngle(perp_heaing);
    const double left_dis = (left_pt - pos).Dot(perp_vec);
    const double right_dis = (right_pt - pos).Dot(perp_vec);
    const Vec2d final_left_pt = pos + perp_vec * (left_dis - lateral_gap);
    const Vec2d final_right_pt = pos + perp_vec * (right_dis + lateral_gap);

    path_contour_points.left_bound_points.push_back(final_left_pt);
    path_contour_points.right_bound_points.push_back(final_right_pt);
  }
  return path_contour_points;
}

PathContourPoints GetAvPathContourPoints(const DiscretizedPath& path,
                                         double ego_half_width) {
  PathContourPoints path_contour_points;
  if (path.size() < 2) return path_contour_points;
  const int pt_size = (path.size() + 2) / 3;
  path_contour_points.left_bound_points.reserve(pt_size);
  path_contour_points.right_bound_points.reserve(pt_size);
  path_contour_points.center_points.reserve(pt_size);
  for (int i = 0; i < path.size(); i += 3) {
    const Vec2d ra_point = ToVec2d(path[i]);
    const Vec2d av_perp_tangent =
        Vec2d::FastUnitFromAngle(path[i].theta()).Perp();
    const Vec2d left_point = ra_point + av_perp_tangent * ego_half_width;
    const Vec2d right_point = ra_point - av_perp_tangent * ego_half_width;
    path_contour_points.left_bound_points.push_back(left_point);
    path_contour_points.right_bound_points.push_back(right_point);
    path_contour_points.center_points.push_back(ra_point);
  }
  return path_contour_points;
}

void DumpPathContourToDebugFrame(
    int plan_id, const DiscretizedPath& path, const Box2d& av_box,
    const VehicleShapeBasePtr& av_vehicle_shape,
    const std::unordered_map<std::string, const SpacetimeObjectTrajectory*>&
        st_trajs_map) {
  const std::string group_name = Log2DDS::TaskPrefix(plan_id) + "PathContour";

  const auto dump_points = [&group_name](const auto& points, const auto& name) {
    Log2DDS::LogChartV2(
        group_name, name, Log2DDS::kGray, false, points,
        [](const auto& p) -> double { return p.x(); },
        [](const auto& p) -> double { return p.y(); });
  };

  // Dump st-traj path contour.
  for (const auto& [traj_id, traj] : st_trajs_map) {
    const PathContourPoints path_contour_points =
        GetStTrajPathContourPoints(*traj);
    if (path_contour_points.center_points.size() > 1) {
      dump_points(path_contour_points.left_bound_points, traj_id + "_left");
      dump_points(path_contour_points.right_bound_points, traj_id + "_right");
      dump_points(path_contour_points.center_points, traj_id + "_center");
    }
    const auto expanded_contour =
        traj->contour().ExpandByDistance(traj->required_lateral_gap());
    dump_points(expanded_contour.points(), traj_id + "_contour");
  }

  double ego_half_width = av_box.width() * 0.5;
  const auto& left_mirror = av_vehicle_shape->left_mirror();
  const auto& right_mirror = av_vehicle_shape->right_mirror();
  if (left_mirror.has_value() && right_mirror.has_value()) {
    const double half_width_with_mirror =
        left_mirror->center().DistanceTo(right_mirror->center()) * 0.5 +
        std::max(left_mirror->radius(), right_mirror->radius());
    ego_half_width = std::max(ego_half_width, half_width_with_mirror);
  }

  // Dump av path contour.
  const auto av_path_contour_points =
      GetAvPathContourPoints(path, ego_half_width);
  if (av_path_contour_points.left_bound_points.size() > 1) {
    dump_points(av_path_contour_points.left_bound_points, "ego_left");
    dump_points(av_path_contour_points.right_bound_points, "ego_right");
  }
  dump_points(av_box.GetCornersWithBufferCounterClockwise(/*lat_buffer=*/0.0,
                                                          /* lon_buffer=*/0.0),
              "ego_contour");
}

void DumpStopLineToDebugFrame(
    int plan_id, absl::Span<const ConstraintProto::StopLineProto> stop_lines) {
  const std::string group_name = Log2DDS::TaskPrefix(plan_id) + "PathContour";
  for (const ConstraintProto::StopLineProto& stop_line : stop_lines) {
    std::vector<Vec2d> points(2);
    if (!stop_line.has_half_plane()) continue;
    const auto& half_plane = stop_line.half_plane();
    if (!half_plane.has_start() || !half_plane.has_end()) continue;
    points[0].FromProto(half_plane.start());
    points[1].FromProto(half_plane.end());
    Log2DDS::LogChartV2(
        group_name, stop_line.id() + "_stopline", Log2DDS::kGray, false, points,
        [](const auto& p) -> double { return p.x(); },
        [](const auto& p) -> double { return p.y(); });
  }
}

void DumpDrivePassageToDebugFrame(int plan_id,
                                  const DrivePassage& drive_passage) {
  const std::string group_name = Log2DDS::TaskPrefix(plan_id) + "DrivePassage";
  const auto& stations = drive_passage.stations();
  std::vector<Vec2d> center_points;
  center_points.reserve(stations.size());
  for (const auto& station : stations) {
    center_points.push_back(station.xy());
  }
  Log2DDS::LogChartV2(
      group_name, "center", Log2DDS::kGray, false, center_points,
      [](const auto& p) -> double { return p.x(); },
      [](const auto& p) -> double { return p.y(); });
}

void DumpCipvObjIdToDebugFrame(const CipvObjectInfo& cipv_object_info,
                               int plan_id) {
  const auto& cipv_obj_id = cipv_object_info.nearest_object_id;
  const auto& cipv_stay_obj_id = cipv_object_info.nearest_stay_object_id;

  const auto prefix = Log2DDS::TaskPrefix(plan_id);

  if (cipv_obj_id.has_value()) {
    Log2DDS::LogDataV0(absl::StrCat(prefix, "CIPV_id"), *cipv_obj_id);
  }
}

bool IsLimitAccelByDeviationDistance(
    const std::vector<PathPointSemantic>& path_semantics,
    const DiscretizedPath& path, double plan_start_v) {
  constexpr double kForwardDist = 60.0;  // m.
  constexpr double kAvSpeedThres = Kph2Mps(50.0);
  constexpr double kDeviationDistThres = 1.0;  // m.
  if (plan_start_v > kAvSpeedThres) {
    return false;
  }
  double max_deviation_dist = -std::numeric_limits<double>::max();
  for (int i = 0; i < path_semantics.size(); ++i) {
    const auto& path_point = path[i];
    if (path_point.s() > kForwardDist) {
      break;
    }
    max_deviation_dist =
        std::max(max_deviation_dist, path_semantics[i].deviation_distance);
  }
  return max_deviation_dist > kDeviationDistThres;
}

std::vector<AccelBounds> GenerateAccelerationBounds(
    const SpeedOptimizerObjectManager& opt_obj_mgr,
    const st::MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto::SpeedOptimizerParamsProto&
        speed_optimizer_params,
    const std::vector<PathPointSemantic>& path_semantics,
    const DiscretizedPath& path, double plan_start_v, int knot_num,
    const NudgeObjectInfo* nudge_object_info) {
  const PiecewiseLinearFunction<double, double> kAvSpeedAccelBoundPlf = {
      {5.0, 15.56, 20.0, 27.78, 32.78}, {2.0, 1.05, 1.0, 0.5, 0.2}};

  // Make hard accel upper bound.
  const double upper_bound =
      std::min(kAvSpeedAccelBoundPlf(plan_start_v),
               motion_constraint_params.max_acceleration());

  // Make hard accel lower bound.
  const double lower_bound = motion_constraint_params.max_deceleration();

  // Make soft accel upper bound.
  bool need_accel_overtake = false;
  for (const auto& obj : opt_obj_mgr.MovingLeadObjects()) {
    const auto& fo_state = obj.first_overlap_state();
    const double time = obj.first_overlap_time();
    if (plan_start_v * time < fo_state.bound + fo_state.lon_buffer) {
      need_accel_overtake = true;
      break;
    }
  }
  std::optional<double> soft_upper_bound;
  if (nudge_object_info != nullptr &&
      nudge_object_info->type == NudgeObjectInfo::NudgeState::BORROW) {
    const PiecewiseLinearFunction<double, double>
        kAvSpeedSoftAccelBoundofBorrowPlf = {{0, 1, 10}, {0.6, 0.45, 0.15}};
    soft_upper_bound = kAvSpeedSoftAccelBoundofBorrowPlf(plan_start_v);
  } else if (!need_accel_overtake && IsLimitAccelByDeviationDistance(
                                         path_semantics, path, plan_start_v)) {
    const PiecewiseLinearFunction<double, double>
        kAvSpeedSoftAccelBoundByDeviationPlf = {{5.0, 15.0}, {0.7, 0.4}};
    soft_upper_bound = kAvSpeedSoftAccelBoundByDeviationPlf(plan_start_v);
  }

  // Make soft accel lower bound.
  const auto soft_lower_bound_plf = PiecewiseLinearFunctionFromProto(
      speed_optimizer_params.accel_lower_bound_plf());
  const double soft_lower_bound =
      std::max(soft_lower_bound_plf(plan_start_v),
               motion_constraint_params.max_deceleration());

  std::vector<AccelBounds> accel_bounds;
  accel_bounds.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    accel_bounds.push_back(AccelBounds{.lower_bound = lower_bound,
                                       .upper_bound = upper_bound,
                                       .soft_lower_bound = soft_lower_bound,
                                       .soft_upper_bound = soft_upper_bound});
  }
  return accel_bounds;
}

// Tips: Do not used.
bool IsNarrowNearLargeVehicle(
    const DrivePassage* drive_passage, const PathSlBoundary* path_boundary,
    const SpacetimeTrajectoryManager* traj_mgr,
    const std::vector<StBoundaryRef>& st_boundaries,
    const std::vector<StCloseTrajectory>& moving_close_trajs,
    const PathPoint& curr_path_point) {
  if (drive_passage == nullptr || path_boundary == nullptr ||
      traj_mgr == nullptr) {
    return false;
  }
  const auto av_sl =
      drive_passage->QueryFrenetCoordinateAt(ToVec2d(curr_path_point));
  if (av_sl.ok()) return false;

  constexpr double kPassageEps = 0.1;   // m.
  constexpr double kQueryLength = 5.0;  // m.
  const double max_query_s = drive_passage->end_s() - kPassageEps;

  double query_start_s = av_sl->s;
  double query_end_s = std::fmin(query_start_s + kQueryLength, max_query_s);
  double query_s = query_start_s;
  double accum_width = 0.0;
  int accum_n = 0;

  constexpr double kQueryStep = 0.3;       // m.
  constexpr double kLatDistBuffer = 0.45;  // m.
  while (query_s < query_end_s) {
    const auto [l_min, l_max] = path_boundary->QueryBoundaryL(query_s);
    const double width = std::fabs(l_max - l_min);
    accum_width += width;
    accum_n++;
    query_s += kQueryStep;
  }
  for (const auto& moving_close_traj : moving_close_trajs) {
    const PlannerObject* obj =
        traj_mgr->FindObjectByObjectId(moving_close_traj.object_id());
    if (!obj->is_large_vehicle()) {
      continue;
    }
    double lat_dist = std::numeric_limits<double>::max();
    for (const auto& single_nearest_point :
         moving_close_traj.st_nearest_points()) {
      lat_dist = std::min(std::fabs(single_nearest_point.lat_dist), lat_dist);
    }
    if (lat_dist < kLatDistBuffer) {
      return true;
    }
  }

  for (const auto& st_boundary : st_boundaries) {
    const auto& object_id = st_boundary->object_id();
    if (!object_id.has_value()) continue;
    const PlannerObject* obj =
        CHECK_NOTNULL(traj_mgr->FindObjectByObjectId(*object_id));
    if (!obj->is_large_vehicle()) continue;

    const auto& obj_sl_info = st_boundary->obj_sl_info();
    if (obj_sl_info.has_value() &&
        std::fabs(obj_sl_info->dl) < kLatDistBuffer) {
      return true;
    }
  }

  return false;
}

const StBoundaryWithDecision* FindNearestCutinStBoundary(
    const std::vector<StBoundaryWithDecision>& st_boundaries_wd,
    const SpacetimeTrajectoryManager& traj_mgr, const DiscretizedPath& path) {
  const StBoundaryWithDecision* nearest_cutin_st_boundary = nullptr;
  double first_overlap_s = std::numeric_limits<double>::max();
  for (const auto& st_boundary_wd : st_boundaries_wd) {
    const auto raw_st_boundary = st_boundary_wd.raw_st_boundary();
    if (raw_st_boundary->source_type() !=
            StBoundarySourceTypeProto::ST_OBJECT ||
        !raw_st_boundary->object_id().has_value() ||
        (st_boundary_wd.decision_type() != StBoundaryProto::FOLLOW &&
         st_boundary_wd.decision_type() != StBoundaryProto::YIELD) ||
        raw_st_boundary->is_protective()) {
      continue;
    }
    const PlannerObject* planner_object = CHECK_NOTNULL(
        traj_mgr.FindObjectByObjectId(*st_boundary_wd.object_id()));
    const auto& overlap_meta = raw_st_boundary->overlap_meta();
    if ((!overlap_meta.has_value() ||
         overlap_meta->source() != StOverlapMetaProto::OBJECT_CUTIN)) {
      continue;
    }

    const auto& overlap_infos = raw_st_boundary->overlap_infos();
    if (overlap_infos.empty() || path.empty()) {
      continue;
    }
    const int av_start_idx = std::clamp(overlap_infos.front().av_start_idx, 0,
                                        static_cast<int>(path.size() - 1));
    constexpr double kMaxThetaDiff = d2r(10.0);  // 10 deg.
    if (std::abs(NormalizeAngle(planner_object->pose().theta() -
                                path[av_start_idx].theta())) > kMaxThetaDiff) {
      continue;
    }

    const auto min_t_s_range =
        raw_st_boundary->GetBoundarySRange(raw_st_boundary->min_t());
    if (min_t_s_range.has_value() && min_t_s_range->second < first_overlap_s) {
      first_overlap_s = min_t_s_range->second;
      nearest_cutin_st_boundary = &st_boundary_wd;
    }
  }
  return nearest_cutin_st_boundary;
}

void UpdateCutinHistoryState(
    const absl::Time current_time,
    const StBoundaryWithDecision* current_cutin_target,
    const std::vector<StBoundaryWithDecision>& st_boundaries_wd,
    const SpacetimeTrajectoryManager& traj_mgr,
    CutinHistoryProto* cutin_history) {
  CHECK_NOTNULL(cutin_history);

  constexpr double kHoldingTime = 1.0;    // s.
  constexpr double kCompletedTime = 5.0;  // s.

  const StBoundaryWithDecision* prev_cutin_st_boundary = nullptr;
  const double now = ToUnixDoubleSeconds(current_time);
  const PlannerObject* cutin_history_object = nullptr;

  if (cutin_history->has_object_id()) {
    const auto& cutin_history_id = cutin_history->object_id();
    cutin_history_object = traj_mgr.FindObjectByObjectId(cutin_history_id);
    if (cutin_history_object == nullptr) {
      cutin_history->Clear();
    } else {
      // Prev cutin object exist in current frame.
      for (const auto& stb : st_boundaries_wd) {
        if (stb.object_id().has_value() &&
            *stb.object_id() == cutin_history_id) {
          prev_cutin_st_boundary = &stb;
        }
      }
      if (prev_cutin_st_boundary == nullptr) {
        if (now - cutin_history->prev_cutin_time() > kHoldingTime) {
          cutin_history->Clear();
        }
      } else {
        // Prev cutin object has overlap with path.
        // prev_cutin_st_boundary is not nullptr.
        cutin_history->set_prev_cutin_time(now);
        bool is_cutin_now = false;
        const auto meta =
            prev_cutin_st_boundary->raw_st_boundary()->overlap_meta();
        const auto decision = prev_cutin_st_boundary->decision_type();
        if (meta.has_value() &&
            meta->source() == StOverlapMetaProto::OBJECT_CUTIN &&
            (decision == StBoundaryProto::FOLLOW ||
             decision == StBoundaryProto::YIELD)) {
          is_cutin_now = true;
        }
        if (!is_cutin_now &&
            now - cutin_history->cutin_start_time() > kCompletedTime) {
          cutin_history->Clear();
        }
      }
    }
  }

  if (current_cutin_target != nullptr) {
    const auto& now_cutin_object_id = *current_cutin_target->object_id();
    if (!cutin_history->has_object_id() || cutin_history_object == nullptr ||
        prev_cutin_st_boundary == nullptr) {
      cutin_history->set_object_id(now_cutin_object_id);
      cutin_history->set_cutin_start_time(now);
      cutin_history->set_prev_cutin_time(now);
    }
    if (prev_cutin_st_boundary == nullptr) return;
    const auto& prev_cutin_obj_id = prev_cutin_st_boundary->object_id();
    if (!prev_cutin_obj_id.has_value() ||
        *prev_cutin_obj_id == now_cutin_object_id) {
      return;
    }
    const StBoundary* new_cutin_raw_stb =
        current_cutin_target->raw_st_boundary();
    const StBoundary* prev_cutin_raw_stb =
        prev_cutin_st_boundary->raw_st_boundary();
    const auto prev_cutin_s_range =
        prev_cutin_raw_stb->GetBoundarySRange(new_cutin_raw_stb->min_t());
    if (prev_cutin_s_range.has_value() &&
        new_cutin_raw_stb->min_s() >
            std::max(prev_cutin_s_range->second, prev_cutin_raw_stb->min_s())) {
      return;
    }
    cutin_history->set_object_id(now_cutin_object_id);
    cutin_history->set_cutin_start_time(now);
    cutin_history->set_prev_cutin_time(now);
  }
}

void UpdateLcFinishedState(const absl::Time current_time,
                           LaneChangeStage lc_stage,
                           double prev_coeffi_match_lc_style,
                           LcFinishedTimeProto* lc_finished_time) {
  CHECK_NOTNULL(lc_finished_time);
  constexpr double kLcFinishedHoldingTime = 3.0;  // s.
  const double now = ToUnixDoubleSeconds(current_time);
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    lc_finished_time->set_prev_coeffi_match_lc_style(
        prev_coeffi_match_lc_style);
    lc_finished_time->set_prev_lc_executing_time(0.0);
  }
  if (lc_finished_time->has_prev_is_lc_excuting() &&
      lc_finished_time->prev_is_lc_excuting() &&
      lc_stage == LaneChangeStage::LCS_NONE) {
    lc_finished_time->set_prev_lc_executing_time(now);
  }
  lc_finished_time->set_prev_is_lc_excuting(lc_stage ==
                                            LaneChangeStage::LCS_EXECUTING);
}

}  // namespace

#define DEBUG_NARROW_LARGE_VEHICLE (0)
// NOLINTNEXTLINE(readability-function-size)
absl::StatusOr<SpeedFinderOutput> FindSpeed(
    const SpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params, ThreadPool* thread_pool,
    EgoFrame* curr_ego_frame, SpeedFinderStateProto* speed_finder_state) {
  // _ARG1("FindSpeed", "trajectory size",
  //                    input.traj_mgr->trajectories().size());

  CHECK_NOTNULL(speed_finder_state);
  TIMELINE("FindSpeed");

  const auto task_prefix = Log2DDS::TaskPrefix(input.plan_id);

  std::string name = task_prefix + std::string(__FUNCTION__);
  SCOPED_TRACE(name.c_str());

  DLOG(INFO) << "============================ " << task_prefix
             << " ============================ ";
  Log2DDS::LogDataV2("speed_plan_id", input.plan_id);
  if (nullptr != input.path_sl_boundary) {
    Log2DDS::LogDataV2("speed_slboundary_ends",
                       input.path_sl_boundary->end_s());
  }
  if (nullptr != input.drive_passage) {
    Log2DDS::LogDataV2("speed_drivepassage_ends", input.drive_passage->end_s());
  }

  CHECK_NOTNULL(input.path);
  CHECK_NOTNULL(input.st_path_points);
  CHECK_NOTNULL(input.behavior);
  CHECK_NOTNULL(input.traj_mgr);

  if (input.path->size() < 2) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "Path size is less than 2, which is %d.", input.path->size()));
  }
  if (input.st_path_points->empty()) {
    return absl::FailedPreconditionError("st_path_points is empty.");
  }
  double duration = std::clamp(10.0 / input.plan_start_v, 1.0, 15.0);
  DiscretizedPath ego_predict_trajectory = EgoPredictTrajectory(
      input.path->front(), duration, 0.2, input.plan_start_v);

  OpenLoopSpeedLimit open_loop_speed_limit;
  if (input.constraint_mgr) {
    open_loop_speed_limit = input.constraint_mgr->OpenLoopSpeedLimits();
  }

  constexpr double kEpsilon = 1.0e-6;
  CHECK_NEAR(input.path->front().x(), input.st_path_points->front().x(),
             kEpsilon);
  CHECK_NEAR(input.path->front().y(), input.st_path_points->front().y(),
             kEpsilon);

  SpeedFinderOutput output;
  const int trajectory_steps = GetSpeedFinderTrajectorySteps(
      input.plan_start_v,
      Mph2Mps(motion_constraint_params.default_speed_limit()));

  // Generate path_approx.
  constexpr double kPathApproxTolerance = 0.05;  // m.
  const auto vehicle_rect =
      CreateOffsetRectFromVehicleGeometry(vehicle_geometry_params);
  const auto path_kd_tree = BuildPathKdTree(*input.path);
  const auto path_approx = BuildPathApprox(
      *input.path, vehicle_rect, kPathApproxTolerance, path_kd_tree.get());
  std::optional<PathApprox> path_approx_for_mirrors;
  /*
   *  Step 1: Map st boundaries of objects onto st graph.
   *
   */
  DLOG(INFO)
      << "[speed finder] Step 1: Map st boundaries of objects onto st graph.";
  Timer start_time;
  const auto av_shapes = BuildAvShapes(vehicle_geometry_params, *input.path);
  PathApprox* path_approx_for_mirrors_ptr = nullptr;

  if (FLAGS_planner_use_path_approx_based_st_mapping) {
    path_approx_for_mirrors =
        BuildPathApproxForMirrors(path_approx, vehicle_geometry_params);
    path_approx_for_mirrors_ptr = path_approx_for_mirrors.has_value()
                                      ? &(*path_approx_for_mirrors)
                                      : nullptr;
  }
  auto st_graph = std::make_unique<StGraph>(
      input.plan_id, input.path, trajectory_steps, input.plan_start_v,
      input.plan_start_a, motion_constraint_params.max_deceleration(),
      &vehicle_geometry_params, &speed_finder_params.st_graph_params(),
      &av_shapes, path_kd_tree.get(), &path_approx,
      path_approx_for_mirrors_ptr);

  StGraph::StBoundaryOutput st_boundary_output = st_graph->GetStBoundaries(
      *input.traj_mgr, *input.leading_trajs, input.consider_lane_change_gap,
      *input.constraint_mgr, input.psmm, input.drive_passage,
      input.path_sl_boundary, input.nudge_object_info, thread_pool);
  auto& st_boundaries = st_boundary_output.st_boundaries;

  VLOG(2) << "Build st_graph cost time(ms): " << start_time.TimeNs() / 1E6;
  VLOG(3) << "st_boundary size = " << st_boundaries.size();

  const auto gap_desired_a = st_boundary_output.gap_desired_a;

  const auto road_type = input.psmm->GetRoadClass();
  const bool is_on_highway =
      input.psmm->IsOnHighway() ||
      road_type == ad_byd::planning::V2RoadClass::EXPRESS_WAY_ROAD ||
      road_type == ad_byd::planning::V2RoadClass::HIGH_WAY_ROAD;

  // timer.Mark("map_st_boundaries");
  // std::cout << "road type" << input.psmm->GetRoadClass() <<
  // std::endl;

  /*
   *  Step 2: Analyze overlap.
   *
   */
  DLOG(INFO) << "[speed finder] Step 2: Analyze overlap.";
  std::vector<PathPointSemantic> path_semantics;
  // Analyze path semantics.
  int max_analyze_path_index = -1;
  for (const auto& st_boundary : st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;
    for (const auto& overlap_info : st_boundary->overlap_infos()) {
      max_analyze_path_index =
          std::max(max_analyze_path_index, overlap_info.av_end_idx);
    }
  }
  constexpr double kMinAnalyzeLength = 40.0;
  const int min_analyze_index =
      CeilToInt(kMinAnalyzeLength / kPathSampleInterval);
  max_analyze_path_index += min_analyze_index;
  max_analyze_path_index = std::min(static_cast<int>(input.path->size()) - 1,
                                    max_analyze_path_index);
  if (max_analyze_path_index >= 0) {
    start_time.Reset();
    auto path_semantics_or = AnalyzePathSemantics(
        input.plan_id, *input.path, max_analyze_path_index, *input.psmm,
        input.drive_passage, nullptr /*TODO using const map */, thread_pool);
    VLOG(2) << "Analyze path semantics cost time(ms): "
            << start_time.TimeNs() / 1e6;
    // if (path_semantics_or.ok() && VLOG_IS_ON(4)) {
    //   for (int i = 0; i < path_semantics_or->size(); ++i) {
    //     VLOG(4) << "Path point[" << i << "]: (" << (*input.path)[i].x() <<
    //     ",
    //     "
    //             << (*input.path)[i].y() << "), closest lane point "
    //             << (*path_semantics_or)[i].closest_lane_point.DebugString()
    //             << ", lane path id history size "
    //             << (*path_semantics_or)[i].lane_path_id_history.size();
    //     for (int j = 0; j <
    //     (*path_semantics_or)[i].lane_path_id_history.size();
    //          ++j) {
    //       VLOG(4) << "Path point[" << i << "] lane path id [" << j
    //               << "]: " <<
    //               (*path_semantics_or)[i].lane_path_id_history[j];
    //     }
    //   }
    // }

    if (path_semantics_or.ok()) {
      TIMELINE("AnalyzeStOverlaps");
      AnalyzeStOverlaps(*input.path, *path_semantics_or, *input.psmm,
                        *input.traj_mgr, *input.drive_passage,
                        vehicle_geometry_params, input.lc_stage, av_shapes,
                        input.plan_start_v, &st_boundaries);
      path_semantics = std::move(*path_semantics_or);
      // if (VLOG_IS_ON(4)) {
      //   for (const auto& st_boundary : st_boundaries) {
      //     if (st_boundary->overlap_meta().has_value()) {
      //       const auto& overlap_meta = *st_boundary->overlap_meta();
      //       VLOG(4) << "St-boundary " << st_boundary->id()
      //               << " overlap pattern: "
      //               << StOverlapMetaProto::OverlapPattern_Name(
      //                      overlap_meta.pattern())
      //               << ", source: "
      //               << StOverlapMetaProto::OverlapSource_Name(
      //                      overlap_meta.source())
      //               << ", priority: "
      //               << StOverlapMetaProto::OverlapPriority_Name(
      //                      overlap_meta.priority())
      //               << ", priority reason: " <<
      //               overlap_meta.priority_reason()
      //               << ", modification type: "
      //               << StOverlapMetaProto::ModificationType_Name(
      //                      overlap_meta.modification_type());
      //     }
      //   }
      // }
    } else {
      LOG_WARN << "Path semantic analyzer fails, skip analyzing overlap meta: "
               << path_semantics_or.status().message();
    }
  }
  std::vector<DrivingProcess> driving_process_seq;
  {
    TIMELINE("MakeObjectSceneRecognition");
    MakeObjectSceneRecognition(*input.psmm, *input.drive_passage, *input.path,
                               *input.traj_mgr, vehicle_geometry_params,
                               &st_boundaries, input.plan_start_v,
                               &driving_process_seq);
  }
  bool is_narrow_near_large_vehicle = false;

  /*
   *  Step 3: Pre-decision.
   *
   */
  DLOG(INFO) << "[speed finder] Step 3: Pre-decision.";
  // Initialize st-boudaries with decision.
  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));

  // Generate stop line for dense traffic flow.
  if (FLAGS_planner_enable_dense_traffic_flow_stop_line) {
    auto dense_traffic_flow_constraint = GenerateDenseTrafficFlowConstraint(
        st_boundaries_with_decision, *input.traj_mgr, path_semantics,
        *input.path, input.plan_start_v, vehicle_geometry_params);
    // Only generate path stop line.
    for (auto& path_stop_line : dense_traffic_flow_constraint.path_stop_lines) {
      TIMELINE("GenerateStBoundaryWithDecisionByStopLine");
      st_boundaries_with_decision.push_back(
          GenerateStBoundaryWithDecisionByStopLine(*st_graph, path_stop_line));
      output.constraint_mgr.AddPathStopLine(std::move(path_stop_line));
    }
  }

  auto cipv_object_info = ComputeCipvObjectInfo(*input.traj_mgr, *path_kd_tree,
                                                &st_boundaries_with_decision);

  // Set follow/lead standstill distance for st-boundaries.
  const StandstillDistanceDeciderInput standstill_distance_decider_input{
      .speed_finder_params = &speed_finder_params,
      .stalled_object_ids = input.stalled_objects,
      .planner_semantic_map_manager = input.psmm,
      .lane_path = &input.drive_passage->lane_path(),
      .st_traj_mgr = input.traj_mgr,
      .constraint_mgr = input.constraint_mgr,
      .plan_start_v = input.plan_start_v};
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    TIMELINE("DecideStandstillDistanceForStBoundary");
    DecideStandstillDistanceForStBoundary(standstill_distance_decider_input,
                                          &st_boundary_with_decision,
                                          input.is_open_gap, &cipv_object_info);
  }
  // Only keep all zero-distance stationary st-boudnaries & the nearest
  // non-zero-distance stationary st-boundary.
  {
    TIMELINE("KeepNearestStationarySpacetimeTrajectoryStBoundary");
    KeepNearestStationarySpacetimeTrajectoryStBoundary(
        &st_boundaries_with_decision);
  }

  // Make ignore decisions for st-boundaries.
  std::optional<VtSpeedLimit> ignore_speed_limit = std::nullopt;
  const auto ignore_decider_input = IgnoreDeciderInput(
      {.params = &speed_finder_params.ignore_decider_params(),
       .path = input.path,
       .path_kd_tree = path_kd_tree.get(),
       .path_semantics = &path_semantics,
       .psmm = input.psmm,
       .st_traj_mgr = input.traj_mgr,
       .drive_passage = input.drive_passage,
       .vehicle_geometry_params = &vehicle_geometry_params,
       .av_shapes = &av_shapes,
       .current_v = input.plan_start_v,
       .max_v = Mph2Mps(motion_constraint_params.default_speed_limit()),
       .time_step = kSpeedLimitProviderTimeStep,
       .trajectory_steps = trajectory_steps,
       .lc_stage = input.lc_stage,
       .driving_process_seq = driving_process_seq,
       .current_acc = input.plan_start_a,
       .follow_time_headway = speed_finder_params.follow_time_headway(),
       .plan_id = input.plan_id,
       .lane_change_state = &input.lane_change_state});
  {
    TIMELINE("MakeIgnoreAndPreBrakeDecisionForStBoundaries");
    MakeIgnoreAndPreBrakeDecisionForStBoundaries(
        ignore_decider_input, &st_boundaries_with_decision, &ignore_speed_limit,
        &cipv_object_info);
  }

  // Make pre-decisions for st-boundaries.
  const auto pre_decider_input = PreDeciderInput({
      .params = &speed_finder_params.st_boundary_pre_decider_params(),
      .leading_trajs = input.leading_trajs,
      .follower_set = input.follower_set,
      .lane_change_gap = &input.constraint_mgr->TrafficGap(),
      .st_traj_mgr = input.traj_mgr,
      .path = input.path,
      .vehicle_params = &vehicle_geometry_params,
      .drive_passage = input.drive_passage,
      .current_v = input.plan_start_v,
      .max_v = Mph2Mps(motion_constraint_params.default_speed_limit()),
      .time_step = kSpeedLimitProviderTimeStep,
      .trajectory_steps = trajectory_steps,
      // .planner_model_pool = input.planner_model_pool,
      // .planner_av_context = input.planner_av_context,
      // .objects_proto = input.objects_proto,
      .plan_time = input.plan_time,
      .run_act_net_speed_decision = input.run_act_net_speed_decision,
  });

  std::optional<VtSpeedLimit> parallel_cut_in_speed_limit = std::nullopt;
  {
    TIMELINE("MakePreDecisionForStBoundaries");
    MakePreDecisionForStBoundaries(pre_decider_input,
                                   &st_boundaries_with_decision,
                                   &parallel_cut_in_speed_limit);
  }
  /// add cross blind area
  if (FLAGS_planner_enable_cross_blind_close_decider) {
    TIMELINE("CrossCloseDecider");
    const CrossBlindInput cross_close_input{
        .vehicle_geom = &vehicle_geometry_params,
        .st_graph = st_graph.get(),
        .st_traj_mgr = input.traj_mgr,
        .current_v = input.plan_start_v,
        .current_a = input.plan_start_a,
        .path = input.path,
        .ego_predict_path = &ego_predict_trajectory,
        .path_semantics = &path_semantics,
        .drive_passage = input.drive_passage};
    CrossCloseDecider(cross_close_input, &st_boundaries_with_decision,
                      driving_process_seq);
  }

  // Set pass/yield time buffers for st-boundaries.
  {
    // ("TimeBufferDecider");
    TIMELINE(absl::StrFormat("TimeBufferDecider_sz%d",
                             st_boundaries_with_decision.size()));
    absl::flat_hash_set<std::string> disable_pass_time_buffer_id_set;
    for (const auto& st_boundary_wd : st_boundaries_with_decision) {
      if (!st_boundary_wd.raw_st_boundary()->is_protective()) {
        continue;
      }
      const auto& protected_st_boundary_id =
          st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
      if (!protected_st_boundary_id.has_value()) continue;
      switch (st_boundary_wd.raw_st_boundary()->protection_type()) {
        case StBoundaryProto::SMALL_ANGLE_CUT_IN:
        case StBoundaryProto::LANE_CHANGE_GAP: {
          disable_pass_time_buffer_id_set.insert(*protected_st_boundary_id);
          break;
        }
        case StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT:
        case StBoundaryProto::NON_PROTECTIVE:
          break;
      }
    }

    for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
      const bool disable_pass_time_buffer = ContainsKey(
          disable_pass_time_buffer_id_set, st_boundary_with_decision.id());
      DecideTimeBuffersForStBoundary(
          &st_boundary_with_decision, input.plan_start_v, *input.path,
          vehicle_geometry_params, *input.traj_mgr, disable_pass_time_buffer);
    }
  }
  // if (VLOG_IS_ON(4)) {
  //   for (const auto& st_boundary_with_decision :
  //   st_boundaries_with_decision)
  //   {
  //     VLOG(4) << "Set st-boundary " << st_boundary_with_decision.id()
  //             << " pass_time: " << st_boundary_with_decision.pass_time()
  //             << " yield_time: " << st_boundary_with_decision.yield_time();
  //   }
  // }

  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;

  const PreStboundaryModifierInput pre_st_boundary_modifier_input{
      .vehicle_geom = &vehicle_geometry_params,
      .st_graph = st_graph.get(),
      .st_traj_mgr = input.traj_mgr,
      .current_v = input.plan_start_v,
      .current_a = input.plan_start_a,
      .path = input.path,
      .path_semantics = &path_semantics,
      .drive_passage = input.drive_passage,
      .cipv_object_info = &cipv_object_info,
      .path_sl_boundary = input.path_sl_boundary,
      .leading_objs = input.leading_trajs};
  {
    TIMELINE("PreModifyStBoundaries");
    PreModifyStBoundaries(pre_st_boundary_modifier_input,
                          &st_boundaries_with_decision, &processed_st_objects,
                          &open_loop_speed_limit);
  }

  // NOTE： We may need to append more path semantics after this point
  // because there may have been newly generated st-boundaries.

  // Add some additional constraints (speed region, stop line, etc) to
  // constraint manager.
  // which are based on the path and those generated by upstream modules which
  // are based on drive passage.
  output.constraint_mgr = *input.constraint_mgr;
  if (speed_finder_params.close_object_params()
          .enable_stationary_close_object_slowdown()) {
    TIMELINE("GenerateStationaryCloseObjectConstraints");
    auto stationary_close_object_constraint =
        GenerateStationaryCloseObjectConstraints(
            st_boundaries_with_decision, *st_graph, *input.traj_mgr,
            *input.path, *input.drive_passage, *input.path_sl_boundary,
            input.plan_start_v, vehicle_geometry_params, is_on_highway);
    for (auto& path_speed_region :
         stationary_close_object_constraint.path_speed_regions) {
      output.constraint_mgr.AddPathSpeedRegion(std::move(path_speed_region));
    }
    for (auto& path_stop_line :
         stationary_close_object_constraint.path_stop_lines) {
      output.constraint_mgr.AddPathStopLine(std::move(path_stop_line));
    }
    // timer.Mark("stationary_close_object_decision");
  }

  /*
   *  Step 4: Calculate speed limit.
   *
   */
  DLOG(INFO) << "[speed finder] Step 4: Calculate speed limit.";
  const double planner_speed_cap =
      Mph2Mps(motion_constraint_params.default_speed_limit());  // m/s.
  const double av_max_acc = motion_constraint_params.max_acceleration();
  double spdlimit_curvature_gain = 1.0;
  // Whether raise lane speed limit.
  const bool raise_lane_speed_limit =
      input.lc_stage == LaneChangeStage::LCS_EXECUTING ||
      (input.consider_lane_change_gap &&
       input.constraint_mgr->TrafficGap().follower_id.has_value());
  std::optional<double> v2_trigger_distance = std::nullopt;
  if (speed_finder_state->has_v2_trigger_distance()) {
    v2_trigger_distance = speed_finder_state->v2_trigger_distance();
  }
  auto speed_limit_map = GetSpeedLimitMap(
      *input.path, *input.st_path_points, *input.path_sl_boundary,
      planner_speed_cap, input.plan_start_v, vehicle_geometry_params,
      vehicle_drive_params, *input.drive_passage, output.constraint_mgr,
      speed_finder_params.speed_limit_params(),
      st_graph->distance_info_to_impassable_boundaries(), input.psmm,
      av_max_acc, *input.behavior, st_graph->moving_close_trajs(),
      *input.traj_mgr, input.lane_change_state, input.nudge_object_info,
      input.ego_history, curr_ego_frame, is_narrow_near_large_vehicle,
      raise_lane_speed_limit, input.spdlimit_curvature_gain_prev,
      &spdlimit_curvature_gain, &v2_trigger_distance);
  if (v2_trigger_distance.has_value()) {
    speed_finder_state->set_v2_trigger_distance(*v2_trigger_distance);
  } else {
    speed_finder_state->clear_v2_trigger_distance();
  }

  // timer.Mark("get_speed_limit");
  // Moving close object speed limit.
  std::unordered_map<std::string, double> overlap_trajs_info;
  for (const auto& stb_wd : st_boundaries_with_decision) {
    if (const auto traj_id = stb_wd.traj_id(); traj_id.has_value()) {
      const auto& st_boundary = stb_wd.raw_st_boundary();
      overlap_trajs_info[*traj_id] = st_boundary->bottom_left_point().s();
    }
  }

  std::vector<std::optional<SpeedLimit>> close_traj_speed_limit;
  if (FLAGS_planner_enable_moving_close_traj_speed_limit) {
    TIMELINE("GetMovingCloseTrajSpeedLimits");
    close_traj_speed_limit = GetMovingCloseTrajSpeedLimits(
        st_graph->moving_close_trajs(), input.path->length(),
        std::max(0.0, input.plan_start_v), kSpeedLimitProviderTimeStep,
        speed_finder_params.speed_limit_params().moving_close_traj_max_time());
  }
  std::optional<SpeedLimit> right_turn_close_speed_limit;
  if (FLAGS_planner_enable_right_turn_close_speed_limit) {
    TIMELINE("GetRightTurnCloseSpeedLimit");
    const RightTurnCloseSpeedLimitInput right_turn_close_speed_limit_input{
        .vehicle_geom = &vehicle_geometry_params,
        .st_graph = st_graph.get(),
        .st_traj_mgr = input.traj_mgr,
        .current_v = input.plan_start_v,
        .current_a = input.plan_start_a,
        .path = input.path,
        .ego_predict_path = &ego_predict_trajectory,
        .path_semantics = &path_semantics,
        .drive_passage = input.drive_passage};
    right_turn_close_speed_limit = GetRightTurnCloseSpeedLimit(
        right_turn_close_speed_limit_input, st_boundaries_with_decision,
        driving_process_seq);
    if (right_turn_close_speed_limit.has_value()) {
      speed_limit_map.emplace(SpeedLimitTypeProto_Type_RIGHT_TURN_CLOSE,
                              std::move(right_turn_close_speed_limit.value()));
    }
  }

  std::map<SpeedLimitTypeProto::Type, VtSpeedLimit> vt_speed_limit_map;
  {
    TIMELINE("GetExternalVtSpeedLimit");
    vt_speed_limit_map[SpeedLimitTypeProto_Type_EXTERNAL] =
        GetExternalVtSpeedLimit(*input.constraint_mgr, trajectory_steps,
                                kSpeedLimitProviderTimeStep);
  }

  if (FLAGS_planner_enable_near_parallel_vehicle_speed_limit &&
      parallel_cut_in_speed_limit.has_value()) {
    vt_speed_limit_map[SpeedLimitTypeProto_Type_NEAR_PARALLEL_VEHICLE] =
        *parallel_cut_in_speed_limit;
  }
  if (ignore_speed_limit.has_value()) {
    vt_speed_limit_map[SpeedLimitTypeProto_Type_IGNORE_OBJECT] =
        *ignore_speed_limit;
  }

  auto soft_acc_vt_limit = GenerateSoftAccSpeedLimit(
      motion_constraint_params, input.plan_start_v, input.plan_start_a,
      speed_finder_params.speed_limit_params().soft_acc_jerk());
  if (soft_acc_vt_limit.has_value()) {
    vt_speed_limit_map[SpeedLimitTypeProto_Type_SOFT_ACC] = *soft_acc_vt_limit;
  }

  if (input.lc_stage == LaneChangeStage::LCS_EXECUTING) {
    TIMELINE("LaneChangeSpeedDecider");
    LaneChangeSpeedDecider(input.plan_start_v, av_max_acc,
                           &open_loop_speed_limit);
  }
  const auto& traffic_gap = input.constraint_mgr->TrafficGap();
  const bool generate_lane_change_gap = input.consider_lane_change_gap &&
                                        traffic_gap.leader_id.has_value() &&
                                        traffic_gap.follower_id.has_value();

  // Generate speed limit for gap_ref_v.
  const auto& gap_ref_v = traffic_gap.gap_ref_v;
  if (gap_ref_v.has_value() && *gap_ref_v < input.plan_start_v) {
    constexpr double kExtraTime = 1.0;             // s.
    constexpr double kAccelForGapRefSpeed = -0.7;  // m/ss.
    vt_speed_limit_map[SpeedLimitTypeProto_Type_GAP_REF_SPEED] =
        GenerateConstAccSpeedLimit(
            /*start_t=*/0.0,
            trajectory_steps * kTrajectoryTimeStep + kExtraTime,
            /*start_v=*/input.plan_start_v, /*min_v=*/std::max(*gap_ref_v, 0.0),
            /*max_v=*/input.plan_start_v, kAccelForGapRefSpeed,
            kSpeedLimitProviderTimeStep, trajectory_steps, "gap_ref_v");
  }

  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    const auto* raw_st_boundary = st_boundary_with_decision.raw_st_boundary();
    if (!raw_st_boundary->is_protective()) {
      continue;
    }
    if (raw_st_boundary->protection_type() !=
        StBoundaryProto::LANE_CHANGE_GAP) {
      continue;
    }
    if (traffic_gap.leader_id.has_value() &&
        st_boundary_with_decision.object_id() == *traffic_gap.leader_id &&
        raw_st_boundary->obj_sl_info().has_value() &&
        raw_st_boundary->obj_sl_info()->ds < 0.5 &&
        st_boundary_with_decision.obj_pose_info().v() >
            input.plan_start_v + 0.5) {
      open_loop_speed_limit.AddALimit(0.0, std::nullopt,
                                      "waiting_follow_faster_obs_acc_limit");
      break;
    }
  }

  if (generate_lane_change_gap) {
    const double jerk_lower = 0.8;
    double a_limit_by_jerk =
        std::min(std::max(input.plan_start_a + jerk_lower, jerk_lower), 1.5);
    Log2DDS::LogDataV0("lane_change_gap_acc_limit",
                       absl::StrCat(a_limit_by_jerk));
    open_loop_speed_limit.AddALimit(std::min(1.0, a_limit_by_jerk),
                                    std::nullopt, "lane_change_gap_acc_limit");
  }

  if (is_narrow_near_large_vehicle) {
    open_loop_speed_limit.AddALimit(-0.6, std::nullopt,
                                    "narrow_near_large_vehicle");
  }

  std::optional<VtSpeedLimit> open_loop_vt_limit;
  open_loop_vt_limit = open_loop_speed_limit.GenerateOpenLoopSpeedLimit(
      input.plan_start_v, trajectory_steps, kSpeedLimitProviderTimeStep);

  if (open_loop_vt_limit.has_value()) {
    vt_speed_limit_map[SpeedLimitTypeProto_Type_FAST_SPEED_LIMIT] =
        *open_loop_vt_limit;
  }

  // Generate defensive speed limit.
  const DefensiveSpeedDeciderInput defensive_speed_decider_input{
      .plan_id = input.plan_id,
      .plan_start_v = input.plan_start_v,
      .plan_start_a = input.plan_start_a,
      .time_step = kSpeedLimitProviderTimeStep,
      .step_num = trajectory_steps,
      .max_ego_v = planner_speed_cap,
      .lc_stage = input.lc_stage,
      .path = input.path,
      .traj_mgr = input.traj_mgr,
      .drive_passage = input.drive_passage,
      .obj_history_mgr = input.obj_history,
      .obj_sl_map = &st_graph->obj_sl_map(),

      .cipv_info = &cipv_object_info,
      .leading_trajs = input.leading_trajs,
      .vehicle_geometry_params = &vehicle_geometry_params,
      .speed_finder_params = &speed_finder_params};

  const std::optional<VtSpeedLimit> defensive_speed_limit =
      DecideDefensiveSpeedLimit(defensive_speed_decider_input,
                                speed_finder_state->mutable_defensive_speed());
  if (defensive_speed_limit.has_value()) {
    vt_speed_limit_map[SpeedLimitTypeProto_Type_DEFENSIVE] =
        *defensive_speed_limit;
  }

  // Construct speed limit provider.
  SpeedLimitProvider speed_limit_provider(
      std::move(speed_limit_map), std::move(close_traj_speed_limit),
      std::move(vt_speed_limit_map), kSpeedLimitProviderTimeStep);

  // Run speed gaming
  SpeedGamingDecider speed_gaming_decider(&vehicle_geometry_params,
                                          &vehicle_drive_params);
  const SpeedGamingInput speed_gaming_input{
      .traj_mgr = input.traj_mgr,
      .ego_path = input.path,
      .plan_start_v = input.plan_start_v,
      .plan_start_a = input.plan_start_a,
      .plan_start_j = input.plan_start_j,
      .drive_passage = input.drive_passage,
      .speed_limit_provider = &speed_limit_provider,
      .st_boundaries_with_decision = &st_boundaries_with_decision,
      .processed_st_objects = &processed_st_objects,
      .plan_id = input.plan_id,
      .gaming_lc_obs_set = input.gaming_lc_obs_set,
      .last_speed_gaming_result = input.last_speed_gaming_result,
  };
  speed_gaming_decider.SetIsSingleAgentGaming(true);
  auto speed_gaming_output = speed_gaming_decider.Execute(speed_gaming_input);
  // update st_boundaries_with_decision and processed_st_objects，but don't
  // update decision tag
  if (speed_gaming_output.ok()) {
    const GamingStboundaryModifierInput modifier_input{
        .st_graph = st_graph.get(),
        .speed_gaming_output = &speed_gaming_output.value(),
        .path = input.path};
    GamingModifyStBoundaries(modifier_input, &st_boundaries_with_decision,
                             &processed_st_objects);
    output.speed_gaming_result.CopyFrom(
        speed_gaming_output->speed_gaming_result);
  }

  /*
   *  Step 5: Coarse speed planning given st-graph and speed limit, and make
   *          decisions on st-boundaries at the same time.
   */
  start_time.Reset();
  SpeedVector preliminary_speed;
  InteractiveSpeedDebugProto interactive_speed_debug;

  // dist_to_merge from drive_passage
  double dist_to_merge = std::numeric_limits<double>::max();
  if (input.drive_passage->lane_seq_info() != nullptr) {
    dist_to_merge = input.drive_passage->lane_seq_info()->dist_to_merge;
  }

  {
    TIMELINE("MakeInteractiveSpeedDecision");
    RETURN_IF_ERROR(MakeInteractiveSpeedDecision(
        input.base_name, vehicle_geometry_params, motion_constraint_params,
        *st_graph, *input.traj_mgr, *input.path, input.plan_start_v,
        input.plan_start_a, speed_finder_params, planner_speed_cap,
        trajectory_steps, *input.leading_trajs, input.nudge_object_info,
        &speed_limit_provider, driving_process_seq, is_on_highway,
        dist_to_merge, &preliminary_speed, &st_boundaries_with_decision,
        &processed_st_objects, &interactive_speed_debug, thread_pool,
        input.lc_stage, input.follower_set, input.active_speed_response_style));
  }
  // timer.Mark("speed_search");

  // conflict resolution
  if (speed_gaming_output.ok()) {
    auto speed_gaming_output_new =
        speed_gaming_decider.ConflictResolutionAfterDP(
            speed_gaming_input, speed_gaming_output->gaming_decision);
    // update st_boundaries_with_decision and processed_st_objects，but update
    // decision tag
    if (speed_gaming_output_new.ok()) {
      const GamingStboundaryModifierInput modifier_input{
          .st_graph = st_graph.get(),
          .speed_gaming_output = &speed_gaming_output_new.value(),
          .path = input.path};
      GamingModifyStBoundariesAfterDP(modifier_input,
                                      &st_boundaries_with_decision,
                                      &processed_st_objects, input.plan_id);
    }
  }

  // Set st_boundary debug info.
  SetStBoundaryDebugInfo(st_boundaries_with_decision,
                         &output.speed_finder_proto);
  // Record decision inconsistency.
  // EvaluateActNetSpeedDecision(st_boundaries_with_decision);

  const auto uncertain_vehicle_speed_limit =
      MakeUncertainVehiclePreBrakeDecision(
          *input.traj_mgr, *input.path, vehicle_geometry_params,
          input.plan_start_v,
          Mph2Mps(motion_constraint_params.default_speed_limit()),
          kSpeedLimitProviderTimeStep, trajectory_steps, preliminary_speed,
          &st_boundaries_with_decision);

  if (uncertain_vehicle_speed_limit.has_value()) {
    speed_limit_provider.AddVtSpeedLimit(
        SpeedLimitTypeProto_Type_UNCERTAIN_VEHICLE,
        *uncertain_vehicle_speed_limit);
  }

  const auto creep_interaction_speed_limit = MakeCreepInteractionDecision(
      *input.traj_mgr, *input.path, vehicle_geometry_params, input.plan_start_v,
      Mph2Mps(motion_constraint_params.default_speed_limit()),
      kSpeedLimitProviderTimeStep, trajectory_steps, preliminary_speed,
      &st_boundaries_with_decision);

  if (creep_interaction_speed_limit.has_value()) {
    speed_limit_provider.AddVtSpeedLimit(
        SpeedLimitTypeProto_Type_CREEP_INTERACTION,
        *creep_interaction_speed_limit);
  }
  //  for (const auto& st_bpundary_wd : st_boundaries_with_decision) {
  //    LOG_INFO << "candidate obj id " << st_bpundary_wd.id();
  //    const auto st_boundary = st_bpundary_wd.raw_st_boundary();
  //    std::cout << "raw_overlap_s = " << st_boundary->bottom_left_point().s()
  //              << std::endl
  //              << "overlap_pattern = " <<
  //              st_boundary->overlap_meta()->pattern()
  //              << " overlap_source = " <<
  //              st_boundary->overlap_meta()->source();
  //  }
  /*
   *  Step 6: Speed optimization.
   *
   */
  start_time.Reset();

  const double plan_total_time = kTrajectoryTimeStep * trajectory_steps;
  const int knot_num = speed_finder_params.speed_optimizer_params().knot_num();
  CHECK_GT(knot_num - 1, 0);
  const double plan_time_interval = plan_total_time / (knot_num - 1);

  double coeffi_match_lc_style =
      input.active_speed_response_style ==
              SpeedResponseStyle::SPEED_RESPONSE_RADICAL
          ? speed_finder_params.speed_optimizer_params()
                .lonbuffer_coeffi_match_lc_style_radical()
          : speed_finder_params.speed_optimizer_params()
                .lonbuffer_coeffi_match_lc_style_normal();

  const StBoundaryWithDecision* nearest_cutin_st_boundary =
      FindNearestCutinStBoundary(st_boundaries_with_decision, *input.traj_mgr,
                                 *input.path);
  const auto now_time = absl::Now();
  UpdateCutinHistoryState(now_time, nearest_cutin_st_boundary,
                          st_boundaries_with_decision, *input.traj_mgr,
                          speed_finder_state->mutable_cutin_history());
  UpdateLcFinishedState(now_time, input.lc_stage, coeffi_match_lc_style,
                        speed_finder_state->mutable_lc_finished_time());

  bool need_accel_for_gap = gap_desired_a.value_or(0.0) > 0.0;
  const SpeedOptimizerObjectManager opt_obj_mgr(
      input.plan_id, st_boundaries_with_decision, *input.traj_mgr, *input.path,
      input.plan_start_v, plan_total_time, plan_time_interval,
      speed_finder_params, preliminary_speed,
      &speed_finder_state->cutin_history(), ToUnixDoubleSeconds(now_time),
      input.lc_stage, coeffi_match_lc_style,
      &speed_finder_state->lc_finished_time(), need_accel_for_gap);

  // Assume the step length of the path is a fixed value.
  const double path_step_length = input.path->at(1).s() - input.path->at(0).s();
  CHECK_GT(path_step_length, 0.0);

  const SpeedFinderParamsProto* new_speed_finder_params_ptr =
      &speed_finder_params;
  std::unique_ptr<SpeedFinderParamsProto> dispatched_speed_finder_params_ptr;
  auto dispatched_speed_optimizer_params = DispatchSpeedOptimizerConfig(
      st_boundaries_with_decision, *input.traj_mgr, path_approx, *path_kd_tree,
      vehicle_rect.radius(), path_step_length,
      static_cast<int>(input.path->size() - 1), input.plan_start_v,
      input.path->front(), vehicle_geometry_params,
      speed_finder_params.speed_optimizer_params(),
      speed_finder_params.speed_optimizer_config_dispatcher_params());
  if (dispatched_speed_optimizer_params.has_value()) {
    dispatched_speed_finder_params_ptr =
        std::make_unique<SpeedFinderParamsProto>(speed_finder_params);
    *dispatched_speed_finder_params_ptr->mutable_speed_optimizer_params() =
        std::move(*dispatched_speed_optimizer_params);
    new_speed_finder_params_ptr = dispatched_speed_finder_params_ptr.get();
  }
  const auto& new_speed_finder_params = *new_speed_finder_params_ptr;

  SpeedVector optimized_speed;
  {
    TIMELINE("OptimizeSpeed");
    const auto speed_bound_map = EstimateSpeedBound(
        speed_limit_provider, preliminary_speed, input.plan_start_v,
        planner_speed_cap, knot_num, plan_time_interval);

    std::optional<SpeedLimit> raw_lane_speed_limit;
    if (raise_lane_speed_limit) {
      raw_lane_speed_limit = GenerateRawLaneSpeedLimit(
          input.drive_passage, *input.path, planner_speed_cap);
    }
    const auto min_speed_limit = GenerateMinSpeedLimitWithLaneAndCurvature(
        FindOrDie(speed_bound_map, SpeedLimitTypeProto_Type_LANE),
        FindOrDie(speed_bound_map, SpeedLimitTypeProto_Type_CURVATURE));
    const auto reference_speed = GenerateReferenceSpeed(
        min_speed_limit, raw_lane_speed_limit, input.plan_start_v,
        new_speed_finder_params.speed_optimizer_params().ref_speed_bias(),
        new_speed_finder_params.speed_optimizer_params()
            .ref_speed_static_limit_bias(),
        motion_constraint_params.max_acceleration(),
        motion_constraint_params.max_deceleration(), plan_total_time,
        plan_time_interval);

    const auto accel_bounds = GenerateAccelerationBounds(
        opt_obj_mgr, motion_constraint_params,
        new_speed_finder_params.speed_optimizer_params(), path_semantics,
        *input.path, input.plan_start_v, knot_num, input.nudge_object_info);

    RETURN_IF_ERROR(OptimizeSpeed(
        input.base_name, input.plan_start_v, input.plan_start_a,
        plan_time_interval, opt_obj_mgr, speed_bound_map, input.path->length(),
        reference_speed, accel_bounds, input.time_aligned_prev_traj,
        motion_constraint_params, new_speed_finder_params, &optimized_speed,
        &output.speed_finder_proto));
  }

  DumpPrimaryBrakingTarget(optimized_speed, output.speed_finder_proto,
                           input.plan_id);

  CHECK(!optimized_speed.empty()) << "Optimized speed points is empty!";
  // timer.Mark("speed_optimization");

  /*
   *  Post-process the results of speed optimization.
   */
  CutoffSpeedByTimeHorizon(&optimized_speed);

  // Set trajectory end info.
  output.trajectory_end_info = SetTrajectoryEndInfo(
      st_boundaries_with_decision, optimized_speed.TotalLength());

  if (new_speed_finder_params.enable_full_stop()) {
    PostProcessSpeedByFullStop(new_speed_finder_params, &optimized_speed);
  }

  if (new_speed_finder_params.enable_front_vehicle_alert()) {
    output.alerted_front_vehicle = GetAlertedFrontVehicleForHMI(
        opt_obj_mgr, *input.traj_mgr, input.path->front(), optimized_speed,
        vehicle_geometry_params, input.base_name);
  }

  /*
   *  Step 7: Integrate path and speed.
   *
   */
  std::vector<ApolloTrajectoryPointProto> output_trajectory_points;
  output_trajectory_points.reserve(optimized_speed.size());
  RETURN_IF_ERROR(CombinePathAndSpeed(*input.path, /*forward=*/true,
                                      optimized_speed,
                                      &output_trajectory_points));
  for (const auto& p : output_trajectory_points) {
    *output.speed_finder_proto.add_trajectory() = p;
  }

  // if (FLAGS_planner_print_speed_finder_time_stats) {
  //   PrintMultiTimerReportStat(timer);
  // }

  /*
   *  Step 8: Plot speed data to chart.
   *
   */
  const auto plan_id = input.plan_id;
  // st_chart
  DumpStGraphBoundary(plan_id, st_boundaries_with_decision, *input.traj_mgr,
                      output.speed_finder_proto);
  SpeedVector comfortable_brake_speed;
  const auto& comfortable_brake_speed_proto =
      output.speed_finder_proto.speed_optimizer().comfortable_brake_speed();
  if (new_speed_finder_params.speed_optimizer_params()
          .enable_comfort_brake_speed()) {
    comfortable_brake_speed.reserve(comfortable_brake_speed_proto.size());
    for (const auto& speed_pt : comfortable_brake_speed_proto) {
      comfortable_brake_speed.emplace_back().FromProto(speed_pt);
    }
  }
  const auto& max_brake_speed_proto =
      output.speed_finder_proto.speed_optimizer().max_brake_speed();
  SpeedVector max_brake_speed;
  max_brake_speed.reserve(max_brake_speed_proto.size());
  for (const auto& speed_pt : max_brake_speed_proto) {
    max_brake_speed.emplace_back().FromProto(speed_pt);
  }
  DumpStGraphSpeed(plan_id, input.path->length(), trajectory_steps,
                   preliminary_speed, optimized_speed, comfortable_brake_speed,
                   max_brake_speed);
  // vt_chart
  DumpVtGraphSpeedLimit(plan_id, output.speed_finder_proto);
  const auto& ref_speed_proto =
      output.speed_finder_proto.speed_optimizer().ref_speed();
  SpeedVector ref_speeds;
  ref_speeds.reserve(ref_speed_proto.size());
  for (const auto& speed_pt : ref_speed_proto) {
    ref_speeds.emplace_back().FromProto(speed_pt);
  }
  DumpVtGraphSpeed(plan_id, preliminary_speed, optimized_speed, ref_speeds);

  // pred-traj at/vt chart
  auto st_traj_maps = GetAllOverlappedStObjectTrajs(
      st_boundaries_with_decision, processed_st_objects, *input.traj_mgr);
  const auto& gap_follower = traffic_gap.follower_id;
  if (gap_follower.has_value() && !ContainsKey(st_traj_maps, *gap_follower)) {
    const auto follower_trajs =
        input.traj_mgr->FindTrajectoriesByObjectId(*gap_follower);
    if (!follower_trajs.empty()) {
      auto traj = follower_trajs.front();
      st_traj_maps.emplace(traj->traj_id(), CHECK_NOTNULL(traj));
    }
  }
  const auto& gap_leader = traffic_gap.leader_id;
  if (gap_leader.has_value() && !ContainsKey(st_traj_maps, *gap_leader)) {
    const auto leader_trajs =
        input.traj_mgr->FindTrajectoriesByObjectId(*gap_leader);
    if (!leader_trajs.empty()) {
      auto traj = leader_trajs.front();
      st_traj_maps.emplace(traj->traj_id(), CHECK_NOTNULL(traj));
    }
  }
  DumpPredictionTrajectories(plan_id, st_traj_maps);

  // Path contour chart.
  const auto init_path_point = input.path->front();
  const auto av_box =
      ComputeAvBox(ToVec2d(init_path_point), init_path_point.theta(),
                   vehicle_geometry_params);
  DumpPathContourToDebugFrame(plan_id, *input.path, av_box, av_shapes.front(),
                              st_traj_maps);

  DumpStopLineToDebugFrame(plan_id, input.constraint_mgr->StopLine());

  // Drive passage center line.
  DumpDrivePassageToDebugFrame(plan_id, *input.drive_passage);

  DumpCipvObjIdToDebugFrame(cipv_object_info, plan_id);

  DumpInteractiveSetDebugInfoToDebugFrame(interactive_speed_debug, plan_id,
                                          st_boundaries_with_decision);

  if (!FLAGS_planner_send_speed_optimizer_debug) {
    output.speed_finder_proto.clear_speed_optimizer();
  }

  output.trajectory_points = std::move(output_trajectory_points);
  *output.speed_finder_proto.mutable_modified_prediction() =
      ConstructModifiedPrediction(processed_st_objects, plan_id);

  output.considered_st_objects =
      GetConsideredStObjects(st_boundaries_with_decision, *input.traj_mgr,
                             std::move(processed_st_objects));

  output.obj_sl_map.clear();
  if (st_graph) {
    output.obj_sl_map = st_graph->obj_sl_map();
  }

  output.speed_finder_proto.set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(input.plan_time));
  DestroyContainerAsyncMarkSource(std::move(st_graph), std::string{});

  output.st_boundaries_with_decision = std::move(st_boundaries_with_decision);
  output.attention_obj_id = opt_obj_mgr.attention_obj_id();
  output.spdlimit_curvature_gain = spdlimit_curvature_gain;
  output.cipv_obj_id = cipv_object_info.nearest_object_id;

  return output;
}

}  // namespace planning
}  // namespace st
