

#include "decider/decision_manager/constraint_builder.h"

#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "plan_common/log_data.h"
#include "plan_common/timer.h"
#include "plan_common/type_def.h"
// #include "common.pb.h"
#include "plan_common/log.h"
#include "decider/decision_manager/cautious_brake_decider.h"
#include "decider/decision_manager/crosswalk_decider.h"
#include "decider/decision_manager/decision_util.h"
#include "decider/decision_manager/end_of_current_lane_path.h"
#include "decider/decision_manager/end_of_path_boundary.h"
#include "decider/decision_manager/traffic_static_obstacles.h"
#include "decider/decision_manager/inferred_object_decider.h"
#include "decider/decision_manager/lc_end_of_current_lane_constraint.h"
#include "decider/decision_manager/no_block.h"
#include "decider/decision_manager/parking_brake_release.h"
#include "decider/decision_manager/pedestrians_decider.h"
#include "decider/decision_manager/solid_line_within_boundary.h"
#include "decider/decision_manager/speed_bump.h"
#include "decider/decision_manager/standstill_decider.h"
#include "decider/decision_manager/toll_decider.h"
#include "decider/decision_manager/traffic_gap_finder.h"
#include "decider/decision_manager/traffic_light_decider.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/util.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/speed/st_speed/speed_profile.h"
#include "plan_common/util/format_numeric_string.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/time_util.h"
#include "planner/planner_manager/planner_defs.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/crosswalk_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/piecewise_linear_function.pb.h"

namespace st {
namespace planning {
namespace {
using StopLineInterface = ad_byd::planning::StopLineInterface;
using V2RoadType = ad_byd::planning::V2RoadClass::V2RoadClassType;

constexpr double kZero = 1e-6;
constexpr double kEIEBrakeDefaultDec = -1.0;          // mpss
constexpr double kEIEBrakeActionDec = -1.5;           // mpss
constexpr double kEIEBrakeTimeThres = 3.0;            // s
constexpr double kEIEJunctionSupassDist = 50.0;       // m
constexpr double kEIESubpathSupassDist = 200.0;       // m
constexpr double kEIEEnterTunnelSupassDist = 50.0;    // m
constexpr double kEIEThroughTunnelSupassDist = 50.0;  // m
constexpr double kEIEPreBrakeTargetSpd = 16.7;        // mps

std::vector<PlannerObjectProjectionInfo> GetObstacleInFrontOfEgo(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  std::vector<PlannerObjectProjectionInfo> objects_in_front_ego;
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    if (trajectories.empty()) continue;
    const PlannerObject& obj = trajectories.front()->planner_object();
    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

    if (ego_frenet_box.center_s() > frenet_box.center_s()) {
      continue;
    }

    if (std::min(frenet_box.l_max, ego_frenet_box.l_max) >=
        std::max(frenet_box.l_min, ego_frenet_box.l_min)) {
      objects_in_front_ego.emplace_back(PlannerObjectProjectionInfo{
          .st_trajectories = trajectories, .frenet_box = frenet_box});
    }
  }
  std::stable_sort(objects_in_front_ego.begin(), objects_in_front_ego.end(),
                   [](const auto& a, const auto& b) {
                     return a.frenet_box.s_min < b.frenet_box.s_min;
                   });
  return objects_in_front_ego;
}

std::vector<PlannerObjectProjectionInfo> GetObstaclesOnLane(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr, bool only_front_obj = true) {
  std::vector<PlannerObjectProjectionInfo> objects_on_lane;
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    if (trajectories.empty()) continue;
    const PlannerObject& obj = trajectories.front()->planner_object();
    // if (obj.is_stationary()) continue;
    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

    constexpr double kLateralThreshold = 1.0;  // m.
    if (frenet_box.l_min > kLateralThreshold ||
        frenet_box.l_max < -kLateralThreshold) {
      continue;
    }

    if (ego_frenet_box.center_s() > frenet_box.center_s() && only_front_obj) {
      continue;
    }
    // TODO obstacle type onlane not been considered
    objects_on_lane.emplace_back(PlannerObjectProjectionInfo{
        .st_trajectories = trajectories, .frenet_box = frenet_box});
  }

  std::stable_sort(objects_on_lane.begin(), objects_on_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.frenet_box.s_min < b.frenet_box.s_min;
                   });
  return objects_on_lane;
}

absl::StatusOr<bool> IsOncomingObjectJudgeByDrivePassage(
    const DrivePassage& passage, const SecondOrderTrajectoryPoint& obj_pose) {
  ASSIGN_OR_RETURN(const auto tangent, passage.QueryTangentAt(obj_pose.pos()));
  const double passage_angle = tangent.Angle();
  const double angle_diff =
      std::abs(NormalizeAngle(passage_angle - obj_pose.theta()));

  return angle_diff > M_PI_2;
}

absl::StatusOr<FrenetBox> FilterObjectViaDrivePassage(
    const PlannerObject& object, const DrivePassage& passage,
    const PathSlBoundary& sl_boundary, const FrenetBox& ego_frenet_box) {
  // Calculate object frenet coordinate.
  ASSIGN_OR_RETURN(const auto object_frenet_box,
                   passage.QueryFrenetBoxAtContour(object.contour()));

  // Filter objects behind ego front edge or beyond drive passage length.
  if (object_frenet_box.s_min < ego_frenet_box.s_max ||
      object_frenet_box.s_min > sl_boundary.end_s()) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of longitudinal boundary, s range: ( %.2f, %.2f)",
        object.id(), object_frenet_box.s_min, object_frenet_box.s_max));
  }

  constexpr double kLateralEnterThres = 0.5;  // m.
  const auto [boundary_l_max, boundary_l_min] =
      CalcSlBoundaries(sl_boundary, object_frenet_box);
  // Filter objects not on path boundary.
  if (object_frenet_box.l_min > boundary_l_max - kLateralEnterThres ||
      object_frenet_box.l_max < boundary_l_min + kLateralEnterThres) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of lateral boundary, l range: (%.2f, %.2f)", object.id(),
        object_frenet_box.l_min, object_frenet_box.l_max));
  }
  return object_frenet_box;
}

// std::vector<PlannerObjectProjectionInfo> FindFrontObjectsOnLane(
//     const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
//     const SpacetimeTrajectoryManager& st_traj_mgr) {
//   std::vector<PlannerObjectProjectionInfo> objects_on_lane;
//   for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map())
//   {
//     if (trajectories.empty()) continue;
//     const PlannerObject& obj = trajectories.front()->planner_object();
//     // if (obj.is_stationary()) continue;
//     ASSIGN_OR_CONTINUE(
//         const auto frenet_box,
//         target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

//     constexpr double kLateralThreshold = 1.0;  // m.
//     if (frenet_box.l_min > kLateralThreshold ||
//         frenet_box.l_max < -kLateralThreshold) {
//       continue;
//     }

//     if (ego_frenet_box.center_s() > frenet_box.center_s()) {
//       continue;
//     }

//     objects_on_lane.emplace_back(PlannerObjectProjectionInfo{
//         .st_trajectories = trajectories, .frenet_box = frenet_box});
//   }
//   if (objects_on_lane.empty()) {
//     return {};
//   }

//   std::stable_sort(objects_on_lane.begin(), objects_on_lane.end(),
//                    [](const auto& a, const auto& b) {
//                      return a.frenet_box.s_min < b.frenet_box.s_min;
//                    });
// }

bool YieldToVruScenario(
    const PlannerObjectManager& obj_mgr,
    const ObjectHistoryManager& obs_history,
    const st::VehicleGeometryParamsProto& vehicle_geo_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, ad_byd::planning::SpeedState* speed_state) {
  // 1. get VRU interact timer from StartAccSupplement
  auto& vru_interact_timer = speed_state->vru_interact_timer;
  auto pre_yield_ids = speed_state->infront_vru_ids;
  auto& infront_vru_ids = speed_state->infront_vru_ids;
  infront_vru_ids.clear();
  const double wheel_base = vehicle_geo_params.wheel_base();
  const double ego_front_to_center = vehicle_geo_params.front_edge_to_center();
  const double ego_back_to_center = vehicle_geo_params.back_edge_to_center();
  const double ego_half_width = vehicle_geo_params.width() * 0.5;
  // 1.1 reset timer when ego vehicle velocity is high
  Vec2d ego_pos(plan_start_point.path_point().x(),
                plan_start_point.path_point().y());
  const auto& ego_lon_offset = passage.QueryFrenetLonOffsetAt(ego_pos);
  const auto& ego_lat_offset = passage.QueryFrenetLatOffsetAt(ego_pos);
  if (!ego_lon_offset.ok() || !ego_lat_offset.ok()) {
    return false;
  }
  constexpr double kResetSpeed = 15.0;  // unit: kph
  if (plan_start_point.v() > Kph2Mps(kResetSpeed)) {
    vru_interact_timer = 0.0;
    return false;
  }
  // 2. filter target VRU obstacles
  bool set_yield_timer = false;
  // std::vector<std::string> infront_vru_ids;
  double ds_range = plan_start_point.v() * 3.0;
  ds_range = std::fmax(std::fmin(ds_range, 20.0), 5.0);
  for (const auto& object : obj_mgr.planner_objects()) {
    // for (const auto& object : obj_mgr.planner_objects()) {
    // 2.1 whether filter out this object
    // 2.1.1 Check object type
    if (object.type() != OT_PEDESTRIAN && object.type() != OT_CYCLIST &&
        object.type() != OT_TRICYCLIST) {
      continue;
    }
    const auto& obj_history = obs_history.GetObjHistory(object.id());
    if (!obj_history || obj_history->Empty()) {
      continue;
    }
    bool pre_lon_yield =
        obj_history->GetLatestFrame()->lon_decision == StBoundaryProto::YIELD ||
        obj_history->GetLatestFrame()->lon_decision == StBoundaryProto::FOLLOW;

    auto obj_frenet_box =
        passage.QueryFrenetBoxAtContour(object.contour(), false);
    auto obj_frenet_point =
        passage.QueryUnboundedFrenetCoordinateAt(object.pose().pos());
    if ((!obj_frenet_box.ok()) || (!obj_frenet_point.ok())) {
      continue;
    }
    FrenetPolygon obj_frenet_polygon;
    obj_frenet_polygon.center = obj_frenet_point.value();
    obj_frenet_polygon.s_max =
        obj_frenet_box.value().s_max - ego_lon_offset.value().accum_s;
    obj_frenet_polygon.s_min =
        obj_frenet_box.value().s_min - ego_lon_offset.value().accum_s;
    obj_frenet_polygon.l_max =
        obj_frenet_box.value().l_max - ego_lat_offset.value();
    obj_frenet_polygon.l_min =
        obj_frenet_box.value().l_min - ego_lat_offset.value();

    double ds = 0.0;
    double dl = 0.0;
    if (obj_frenet_polygon.s_min > ego_front_to_center) {
      ds = obj_frenet_polygon.s_min - ego_front_to_center;
    } else if (obj_frenet_polygon.s_max < (-1.0 * ego_front_to_center)) {
      ds = obj_frenet_polygon.s_max + ego_back_to_center;
    } else {
      ds = 0.0;
    }
    if (obj_frenet_polygon.l_min > ego_half_width) {
      dl = obj_frenet_polygon.l_min - ego_half_width;
    } else if (obj_frenet_polygon.l_max < (-1.0 * ego_half_width)) {
      dl = obj_frenet_polygon.l_max + ego_half_width;
    } else {
      dl = 0.0;
    }
    //
    if (pre_lon_yield && ds > -kZero && ds < ds_range && std::fabs(dl) < 5.0 &&
        object.velocity().norm() < 1.0) {
      infront_vru_ids.emplace_back(object.id());
      set_yield_timer = true;
      vru_interact_timer = 0.7;
      continue;
    }
    // check if there is still VRU in the vehicle front area
    double relative_s = obj_frenet_box.value().center_s() -
                        ego_lon_offset.value().accum_s - wheel_base;
    double relative_s_min = obj_frenet_box.value().s_min -
                            ego_lon_offset.value().accum_s - 0.5 * wheel_base;
    double relative_s_max = obj_frenet_box.value().s_max -
                            ego_lon_offset.value().accum_s -
                            ego_front_to_center;
    double relative_dl = std::fabs(dl);
    auto it = find(pre_yield_ids.begin(), pre_yield_ids.end(), object.id());
    double dl_range = it != pre_yield_ids.end() ? 5.0 : 2.0;
    PiecewiseLinearFunction<double, double> dl_area_limit_plf({kZero, 10.0},
                                                              {dl_range, 0.5});
    double dl_area_limit = dl_area_limit_plf(relative_s);
    if (relative_s_min > kZero && relative_s_max < 10.0 &&
        relative_dl < dl_area_limit) {
      Vec2d obj_prev_pt = object.traj(0).points().back().pos();
      auto obj_prev_frenet_point =
          passage.QueryUnboundedFrenetCoordinateAt(obj_prev_pt);
      const auto& obs_l1 = obj_frenet_point.value().l;
      const auto& obs_l2 =
          obj_prev_frenet_point.ok() ? obj_prev_frenet_point.value().l : obs_l1;
      const double angle_diff = NormalizeAngle(
          object.pose().theta() - plan_start_point.path_point().theta());
      if (relative_dl > 0.1 && obs_l1 * obs_l2 > -kZero &&
          fabs(obs_l2) > fabs(obs_l1)) {
        continue;
      } else if (dl < -kZero &&
                 (angle_diff < 30.0 * ad_byd::planning::Constants::DEG2RAD ||
                  angle_diff > 150.0 * ad_byd::planning::Constants::DEG2RAD)) {
        continue;
      } else if (dl > kZero &&
                 (angle_diff > -30.0 * ad_byd::planning::Constants::DEG2RAD ||
                  angle_diff < -150.0 * ad_byd::planning::Constants::DEG2RAD)) {
        continue;
      }
      infront_vru_ids.emplace_back(object.id());
    }
  }
  // 3. timer works for a while after no yield VRU is detected
  if (vru_interact_timer > kZero) {
    // don't wait when there is no more VRU in the front area
    if (infront_vru_ids.empty()) {
      vru_interact_timer = 0.0;
      return false;
    }
    if (plan_start_point.v() > Kph2Mps(3.0)) {
      return false;
    }
    vru_interact_timer -= set_yield_timer ? 0.0 : 0.1;
    return true;
  }
  vru_interact_timer = 0.0;
  return false;
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildStopLineConstraint(
    const std::string& type_str, const DrivePassage& passage,
    const DecisionConstraintConfigProto& config, double front_to_ra,
    double stop_dist) {
  const double stop_s = passage.lane_path_start_s() + stop_dist + front_to_ra -
                        config.normal_offset();

  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(stop_s));
  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(stop_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(type_str);
  stop_line.mutable_source()->mutable_brake_to_stop()->set_reason(
      absl::StrCat("Teleop triggered ", type_str));

  return stop_line;
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildBrakeToStopConstraint(
    const std::string& type_str, const DrivePassage& passage,
    double front_to_ra, double ego_v, double brake) {
  constexpr double kStandStillDist = 0.3;  // m.

  // NOTE: We dont want to add more states. Recalculate stop s each frame is
  // acceptable as we do not require stop at a certain point.
  const double stop_dist =
      ego_v < 1.0 ? kStandStillDist : Sqr(ego_v) * 0.5 / brake;
  const double stop_s = passage.lane_path_start_s() + stop_dist + front_to_ra;

  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(stop_s));
  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(stop_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(type_str);
  stop_line.mutable_source()->mutable_brake_to_stop()->set_reason(
      absl::StrCat("Teleop triggered ", type_str));

  return stop_line;
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildLCCStopConstraint(
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const DecisionConstraintConfigProto& config,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const bool lcc_keep_brake, double* stop_s) {
  // LCC has to stop before the stop line if current lane cannot go straight
  const double kStopBuffer = 1.0;
  bool find_target_arrow = false;
  bool lcc_stop_flag = false;
  bool current_lane_valid = false;
  for (int i = 0; i < lane_path_from_start.lane_ids_size(); i++) {
    const auto lane =
        psmm.FindLaneByIdOrNull(lane_path_from_start.lane_ids().at(i));
    if (!lane) {
      continue;
    }
    if (find_target_arrow && !(lane->junction_id() == 0)) {
      lcc_stop_flag = true;
      break;
    } else {
      find_target_arrow = false;
    }
    if (i == 0 && lane->center_line().IsValid()) {
      if (!(lane->junction_id() == 0) && lcc_keep_brake) {
        lcc_stop_flag = true;
        current_lane_valid = true;
        *stop_s = 0.0;
        break;
      }
      Vec2d start_xy(plan_start_point.path_point().x(),
                     plan_start_point.path_point().y());
      double s = 0.0, l = 0.0;
      constexpr double kLaneRemainLengthThreshold = 100.0;  // unit: m
      if (lane->center_line().GetProjection(start_xy, &s, &l) &&
          lane->center_line().length() - s < kLaneRemainLengthThreshold) {
        *stop_s += lane->center_line().length() - s;
        current_lane_valid = true;
      } else {
        *stop_s += lane->topo_length();
        current_lane_valid = false;
      }
    } else {
      *stop_s += lane->topo_length();
    }
    if (!(lane->arrow_type() & 1) &&
        ((lane->arrow_type() & 2) || (lane->arrow_type() & 4) ||
         (lane->arrow_type() & 8))) {
      if (i == lane_path_from_start.lane_ids_size() - 1) {
        lcc_stop_flag = true;
        break;
      } else {
        find_target_arrow = true;
      }
    }
  }
  double s_offset =
      current_lane_valid ? 0.0 : psmm.map_ptr()->route()->navi_start().s_offset;
  *stop_s -=
      s_offset + vehicle_geometry_params.front_edge_to_center() + kStopBuffer;
  if (*stop_s < kMathEpsilon) {
    *stop_s = kMathEpsilon;
  }
  if (!lcc_stop_flag) {
    return absl::NotFoundError("lcc stop flag false.");
  }
  // add stop line
  constexpr double kLCCStopBrake = 3.0;  // m/s^2
  double brake_a =
      std::fmin(kLCCStopBrake, plan_start_point.v() * plan_start_point.v() /
                                   (2.0 * (*stop_s)));
  if (brake_a < kLCCStopBrake - kMathEpsilon) {
    return BuildStopLineConstraint(
        /*type_str=*/"lcc_turn_stop_line", passage, config,
        vehicle_geometry_params.front_edge_to_center(), *stop_s);
  }
  return BuildBrakeToStopConstraint(
      /*type_str=*/"lcc_turn_stop_soft", passage,
      vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
      brake_a);
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildTJunctionStopConstraint(
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const DecisionConstraintConfigProto& config,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    double* stop_s) {
  // LCC has to stop before the stop line if current lane cannot go straight
  const double kTJunctionStopBuffer = 1.0;
  bool T_junction_stop_flag = false;
  // double stop_s = 0.0;
  bool current_lane_valid = false;
  for (int i = 0; i < lane_path_from_start.lane_ids_size(); i++) {
    const auto lane =
        psmm.FindLaneByIdOrNull(lane_path_from_start.lane_ids().at(i));
    if (!lane) {
      continue;
    }
    if (!(lane->junction_id() == 0) || !lane->is_navigation()) {
      T_junction_stop_flag = true;
      break;
    }
    if (i == 0 && lane->center_line().IsValid()) {
      Vec2d start_xy(plan_start_point.path_point().x(),
                     plan_start_point.path_point().y());
      double s = 0.0, l = 0.0;
      if (lane->center_line().GetProjection(start_xy, &s, &l)) {
        *stop_s += lane->center_line().length() - s;
        current_lane_valid = true;
      } else {
        *stop_s += lane->topo_length();
        current_lane_valid = false;
      }
    } else {
      *stop_s += lane->topo_length();
    }
    if (i == lane_path_from_start.lane_ids_size() - 1) {
      T_junction_stop_flag = true;
    }
  }
  double s_offset =
      current_lane_valid ? 0.0 : psmm.map_ptr()->route()->navi_start().s_offset;
  *stop_s -= s_offset + vehicle_geometry_params.front_edge_to_center() +
             kTJunctionStopBuffer;
  if (std::fabs(*stop_s) < kMathEpsilon) {
    *stop_s = kMathEpsilon;
  }
  if (!T_junction_stop_flag) {
    return absl::NotFoundError("T Junction stop flag false.");
  }
  // add stop line
  constexpr double kLCCStopBrake = 3.0;  // m/s^2
  double brake_a =
      std::fmin(kLCCStopBrake, plan_start_point.v() * plan_start_point.v() /
                                   (2.0 * (*stop_s)));
  if (brake_a < kLCCStopBrake - kMathEpsilon) {
    return BuildStopLineConstraint(
        /*type_str=*/"T_Junction_stop_line", passage, config,
        vehicle_geometry_params.front_edge_to_center(), *stop_s);
  }
  return BuildBrakeToStopConstraint(
      /*type_str=*/"T_Junction_stop_line", passage,
      vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
      brake_a);
}

bool RightLaneIsRmergenctLane(
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& planner_semantic_map_manager) {
  const auto& map_ptr = planner_semantic_map_manager.map_ptr();
  if (map_ptr == nullptr) {
    return false;
  }

  for (const auto& lane_seg : lane_path_from_start) {
    const auto& lane_ptr = map_ptr->GetLaneById(lane_seg.lane_id);
    if (lane_ptr == nullptr) {
      continue;
    }
    if (lane_ptr->type() == LaneType::LANE_EMERGENCY) {
      return true;
    }
  }

  return false;
}

bool SupassEIEByJunction(double cur_dist_to_junction,
                         double cur_dist_to_prev_junction, double break_dist) {
  return ((cur_dist_to_junction - break_dist) <= kEIEJunctionSupassDist) ||
         ((break_dist - cur_dist_to_prev_junction) <= kEIEJunctionSupassDist);
}

bool SupassEIEBySubPath(
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    double break_dist, int plan_id) {
  const auto& map_ptr = planner_semantic_map_manager.map_ptr();
  if (map_ptr == nullptr) {
    return false;
  }

  const auto& ego_lane_seg = lane_path_from_start.lane_segment(0);
  const auto& ego_lane_ptr = map_ptr->GetLaneById(ego_lane_seg.lane_id);

  if (ego_lane_ptr->type() == LaneType::LANE_RAMP) {
    return false;
  }

  const auto& ehp_v2_info = map_ptr->v2_info();

  Log2DDS::LogDataV0(
      "EIE break function debug",
      absl::StrCat(Log2DDS::TaskPrefix(plan_id),
                   " dist_to_subpath: ", ehp_v2_info.dist_to_subpath));

  return (ehp_v2_info.dist_to_subpath - break_dist) <= kEIESubpathSupassDist;
}

bool SupassEIEByTunnel(double dist_to_tunnel_entrance,
                       double dist_to_tunnel_exitance, double break_dist) {
  if (((dist_to_tunnel_entrance - break_dist) <= kEIEEnterTunnelSupassDist &&
       (dist_to_tunnel_entrance - break_dist) >= -kEIEEnterTunnelSupassDist) ||
      ((dist_to_tunnel_exitance - break_dist) <= kEIEThroughTunnelSupassDist &&
       (dist_to_tunnel_exitance - break_dist) >=
           -kEIEThroughTunnelSupassDist)) {
    return true;
  }

  return false;
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildYieldToVruConstraint(
    const PlannerObjectManager& obj_mgr,
    const ObjectHistoryManager& obs_history,
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    std::optional<double> traffic_stopline_dist,
    ad_byd::planning::SpeedState* speed_state) {
  // First version, use junction_id and projection distance to check
  constexpr double kPreviewDist = 10.0;
  bool vru_interact_zone = false;
  double accum_s = -vehicle_geometry_params.front_edge_to_center();
  for (int i = 0; i < lane_path_from_start.lane_ids_size(); i++) {
    if (accum_s > kPreviewDist) break;
    const auto lane =
        psmm.FindLaneByIdOrNull(lane_path_from_start.lane_ids().at(i));
    if (!lane) continue;
    if (!(lane->junction_id() == 0)) {
      vru_interact_zone = true;
      break;
    }
    if (i == 0 && lane->center_line().IsValid()) {
      Vec2d start_xy(plan_start_point.path_point().x(),
                     plan_start_point.path_point().y());
      double s = 0.0, l = 0.0;
      if (lane->center_line().GetProjection(start_xy, &s, &l)) {
        accum_s += lane->center_line().length() - s;
      } else {
        accum_s += lane->topo_length() -
                   psmm.map_ptr()->route()->navi_start().s_offset;
      }
    } else {
      accum_s += lane->topo_length();
    }
  }
  /* // TODO: introduce crosswalk_distance as enable condition;
  const double check_dist = 100.0;
  const auto dis_crosswalk =
      planning_context.cur_lane_sequence()->GetFrontCrossWalkDist(
          navi_start, start_point, check_dist) -
      GetConfigManager()->vehicle_config().front_to_rear_axle();
  Log2DDS::LogDataV2("StartVruDebug",
                     absl::StrCat("crosswalk distance: ", dis_crosswalk));
  */
  if (!vru_interact_zone) {
    return absl::NotFoundError("not vru interact zone.");
  }
  if (traffic_stopline_dist.has_value() &&
      traffic_stopline_dist.value() > kZero &&
      traffic_stopline_dist.value() - passage.lane_path_start_s() <
          kPreviewDist) {
    return absl::NotFoundError(
        "has traffic stop line, no need to yield to vru.");
  }
  // 1. check yield to vru scenarios
  if (!YieldToVruScenario(obj_mgr, obs_history, vehicle_geometry_params,
                          plan_start_point, passage, speed_state)) {
    return absl::NotFoundError("no need to yield to vru.");
  }
  if (plan_start_point.v() > Kph2Mps(3.0)) {
    return absl::NotFoundError("av speed high, no need to yield to vru.");
  }
  // 2. build stop_line infront of av
  const double kYieldToVRUBrake =
      plan_start_point.v() < kZero ? 0.5 : 0.1;  // m/s^2
  return BuildBrakeToStopConstraint(
      "Yield_to_VRU", passage, vehicle_geometry_params.front_edge_to_center(),
      plan_start_point.v(), kYieldToVRUBrake);
}

// Build Construction Scene Speed Constraint
absl::StatusOr<ConstraintProto::SpeedProfileProto>
BuildConstructionSpeedConstraint(const DrivePassage& passage,
                                 const PlannerSemanticMapManager& psmm,
                                 double curent_set_speed, bool is_on_highway,
                                 double ego_v, const absl::Time plan_start_time,
                                 ad_byd::planning::SpeedState* speed_state) {
  bool is_construction_scene = false;
  bool is_depart_count = false;
  bool is_construction_speed_constrain_working = false;
  bool is_construction_speed_constrain_working_pre = false;
  constexpr double kConstructionSpeedTransFactor = 1.05;
  constexpr double kConstructionSpeedLimitMinSpeed =
      Kph2Mps(80.0) / kConstructionSpeedTransFactor;
  constexpr double kConstructionSpeedLimitMinTargetSpeed =
      Kph2Mps(90.0) / kConstructionSpeedTransFactor;
  constexpr double kConstructionSpeedLimitMidSpeed = Kph2Mps(100.0);
  constexpr double kConstructionSpeedLimitMaxSpeed = Kph2Mps(130.0);
  constexpr double kConstructionSpeedLimitNull = Kph2Mps(255.0);
  constexpr double kConstructionPreBrakeAcc = -1.5;
  constexpr double kEgoDecSlowGain = 0.8;
  constexpr double kEgoDecFastGain = 0.7;
  constexpr double kDistConstructionDecRng = 200.0;
  constexpr double kTimeConstructionHoldRngMax = 100.0;
  constexpr double kTimeConstructionExitRngMax = 300.0;
  constexpr double kTimeConstructionHoldRng = 5.0;
  constexpr double kTimeConstructionexitRng = 180.0;
  constexpr int kSampleStepNum = 10;
  double start_time = absl::ToUnixMillis(plan_start_time) / 1000.0;
  double dist_to_subpath = 0.0;   // Distance to the merge/spilt point
  double depart_hold_time = 0.0;  // Departure time from merge/spilt point
  double depart_exit_time = 0.0;  // Departure time from exit construction limit

  dist_to_subpath = (psmm.map_ptr() != nullptr)
                        ? psmm.map_ptr()->v2_info().dist_to_subpath
                        : DBL_MAX;
  // check
  if (passage.lane_seq_info() != nullptr) {
    is_construction_scene =
        passage.traffic_static_obstacles_info().is_construction_scene;
  } else {
    return absl::FailedPreconditionError("lane sequence info is empty!");
  }
  // precess function vaild signal
  is_construction_speed_constrain_working =
      is_construction_scene && speed_state->is_construction_scene_speed_plan;
  speed_state->is_construction_speed_constrain_working =
      is_construction_speed_constrain_working;
  is_construction_speed_constrain_working_pre =
      speed_state->is_construction_speed_constrain_working_pre;

  // exit debounce time count flag
  if (is_construction_speed_constrain_working == false &&
      is_construction_speed_constrain_working_pre == true) {
    is_depart_count = true;
  } else if (is_construction_speed_constrain_working == true &&
             is_construction_speed_constrain_working_pre == false) {
    is_depart_count = false;
  }
  // debounce time counter
  if (!is_construction_speed_constrain_working && is_depart_count == true) {
    int exit_count_temp = speed_state->function_exit_count;
    speed_state->function_exit_count = exit_count_temp++;
    if (speed_state->function_exit_count > 3) {
      speed_state->set_ExitConstructionSpeedLimitTime = start_time;
      speed_state->function_exit_count = 0;
    }
  }
  // calculate exit time
  depart_exit_time = std::clamp(
      0.0, abs(start_time - speed_state->set_ExitConstructionSpeedLimitTime),
      kTimeConstructionExitRngMax);
  if (depart_exit_time > 0.0 && depart_exit_time <= kTimeConstructionexitRng) {
    return absl::FailedPreconditionError(absl::StrCat(
        "depart_exit_time: ", depart_exit_time, " Exit time insufficient!"));
  }
  if (abs(dist_to_subpath) <= kDistConstructionDecRng) {
    speed_state->set_construction_hold_time = start_time;
    return absl::FailedPreconditionError(
        absl::StrCat("dist_to_subpath: ", dist_to_subpath,
                     " Near the Merge/Spilt Junction Area!"));
  }
  depart_hold_time =
      std::clamp(0.0, abs(start_time - speed_state->set_construction_hold_time),
                 kTimeConstructionHoldRngMax);

  if (depart_hold_time > 0.0 && depart_hold_time <= kTimeConstructionHoldRng) {
    return absl::FailedPreconditionError(
        absl::StrCat("depart_hold_time: ", depart_hold_time,
                     " depart Merge/Spilt Junction Area!"));
  }
  if (!is_construction_scene) {
    return absl::FailedPreconditionError(
        absl::StrCat("is_construction_scene: ", is_construction_scene,
                     " av is not in construction scene."));
  }
  if (!is_on_highway) {
    return absl::FailedPreconditionError(absl::StrCat(
        "is on highway: ", is_on_highway, " av is not on highway."));
  }
  if (curent_set_speed < kConstructionSpeedLimitMinSpeed ||
      curent_set_speed == kConstructionSpeedLimitNull ||
      ego_v < kConstructionSpeedLimitMinSpeed) {
    return absl::FailedPreconditionError(
        absl::StrCat("curent_set_speed: ", curent_set_speed, "ego_v: ", ego_v,
                     " Set speed or Ego speed below the effective range."));
  }
  // calculate brake to target speed;
  const double speed_profile_time_range =
      kTrajectoryTimeStep * kTrajectorySteps;
  double min_acc_csl = 0.0;
  double target_speed_csl = 0.0;

  if (curent_set_speed >= kConstructionSpeedLimitMinSpeed &&
      curent_set_speed <= kConstructionSpeedLimitMidSpeed) {
    target_speed_csl =
        std::max(kConstructionSpeedLimitMinSpeed, ego_v * kEgoDecSlowGain);
  }

  if (curent_set_speed > kConstructionSpeedLimitMidSpeed &&
      curent_set_speed <= kConstructionSpeedLimitMaxSpeed) {
    target_speed_csl = std::max(kConstructionSpeedLimitMinTargetSpeed,
                                ego_v * kEgoDecFastGain);
  }
  min_acc_csl = kConstructionPreBrakeAcc;
  const int m = static_cast<int>(speed_profile_time_range) + 1;
  PiecewiseLinearFunctionDoubleProto construction_scene_speed_profile;
  construction_scene_speed_profile.mutable_x()->Reserve(m);
  construction_scene_speed_profile.mutable_y()->Reserve(m);
  for (double t = 0.0; t <= speed_profile_time_range;
       t += kSampleStepNum * kTrajectoryTimeStep) {
    construction_scene_speed_profile.add_x(t);
    construction_scene_speed_profile.add_y(
        std::max(ego_v + min_acc_csl * t, target_speed_csl));
  }
  ConstraintProto::SpeedProfileProto
      construction_scene_speed_profile_constraint;
  *construction_scene_speed_profile_constraint.mutable_vt_upper_constraint() =
      std::move(construction_scene_speed_profile);
  construction_scene_speed_profile_constraint.mutable_source()
      ->mutable_construction_scene()
      ->set_reason("add construction scene speed limit constrain.");
  return construction_scene_speed_profile_constraint;
}

absl::StatusOr<ConstraintProto::SpeedProfileProto> BuildLaneMergeConstraint(
    const DrivePassage& passage, double ego_v, bool is_on_highway,
    double behavior_set_speed, const st::LaneChangeStateProto& lc_state) {
  constexpr double kLaneMergeMinSpeed = Kph2Mps(30.0);
  constexpr double kLaneMergeSpeedLimitCertainTriggerDistance = 180.0;
  constexpr double kLaneMergeSpeedLimitPreBrakeDistance = 350.0;
  constexpr double kLaneMergeSpeedLimitMinLeftDis = 5.0;
  constexpr double kLaneMergePreBrakeAcc = -0.8;
  constexpr double kLaneMergeCertainTriggerAcc = -1.0;  // m/ss
  constexpr double kEps = 1e-3;
  constexpr int kSampleStepNum = 10;
  constexpr double kLaneMergePreBrakeSpeedLimit =
      21.79;  // m/s (80km/h + buffer)
  constexpr double kSpeedLimitBrakeStep = Kph2Mps(10.0);  // kph->m/s
  double dist_to_merge = 0.0;
  if (passage.lane_seq_info() != nullptr) {
    dist_to_merge = passage.lane_seq_info()->dist_to_merge;
  } else {
    return absl::FailedPreconditionError("lane sequence info is empty!");
  }

  if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
    if (!passage.lane_seq_info()->dist_to_merge_vec.empty()) {
      dist_to_merge = passage.lane_seq_info()->dist_to_merge_vec.front();
    } else {
      return absl::FailedPreconditionError("merge dist vector is empty!");
    }
  }

  if (dist_to_merge >= kLaneMergeSpeedLimitPreBrakeDistance) {
    return absl::FailedPreconditionError(absl::StrCat(
        "dist to merge: ", dist_to_merge,
        " exceed range limit: ", kLaneMergeSpeedLimitPreBrakeDistance));
  }
  if (ego_v < kLaneMergeMinSpeed) {
    return absl::FailedPreconditionError(
        absl::StrCat("ego velocity: ", ego_v,
                     " lower than merge limit: ", kLaneMergeMinSpeed));
  }
  if (dist_to_merge < kLaneMergeSpeedLimitMinLeftDis) {
    return absl::FailedPreconditionError(absl::StrCat(
        "dist to merge: ", dist_to_merge,
        " exceed merge end limit: ", kLaneMergeSpeedLimitMinLeftDis));
  }
  if (!is_on_highway) {
    return absl::FailedPreconditionError(absl::StrCat(
        "is on highway: ", is_on_highway, " av is not on highway."));
  }
  if (passage.end_s() < kLaneMergeSpeedLimitMinLeftDis) {
    return absl::FailedPreconditionError(absl::StrCat(
        "passage end_s: ", passage.end_s(),
        " minimum passage limit: ", kLaneMergeSpeedLimitMinLeftDis));
  }

  // calculate brake to target speed;
  const double speed_profile_time_range =
      kTrajectoryTimeStep * kTrajectorySteps;
  double speed_limit_range =
      std::max(std::min(dist_to_merge, passage.end_s()), kEps);
  double min_acc = 0.0;
  double target_speed = 0.0;
  if (dist_to_merge < kLaneMergeSpeedLimitPreBrakeDistance &&
      dist_to_merge >= kLaneMergeSpeedLimitCertainTriggerDistance &&
      behavior_set_speed > kLaneMergePreBrakeSpeedLimit &&
      ego_v >= kLaneMergePreBrakeSpeedLimit) {
    min_acc = kLaneMergePreBrakeAcc;
    target_speed = kLaneMergePreBrakeSpeedLimit;
  } else if (dist_to_merge < kLaneMergeSpeedLimitCertainTriggerDistance) {
    if (Sqr(ego_v) + 2.0 * kLaneMergeCertainTriggerAcc * speed_limit_range >
        Sqr(kLaneMergeMinSpeed)) {
      min_acc = kLaneMergeCertainTriggerAcc;
      target_speed = sqrt(Sqr(ego_v) + 2.0 * min_acc * speed_limit_range);
    } else {
      target_speed = kLaneMergeMinSpeed;
      min_acc = std::min(
          ((Sqr(target_speed) - Sqr(ego_v)) / 2.0 / speed_limit_range), 0.0);
    }
  }

  const int n = static_cast<int>(speed_profile_time_range) + 1;
  PiecewiseLinearFunctionDoubleProto lane_merge_speed_profile;
  lane_merge_speed_profile.mutable_x()->Reserve(n);
  lane_merge_speed_profile.mutable_y()->Reserve(n);
  for (double t = 0.0; t <= speed_profile_time_range;
       t += kSampleStepNum * kTrajectoryTimeStep) {
    lane_merge_speed_profile.add_x(t);
    lane_merge_speed_profile.add_y(std::max(ego_v + min_acc * t, target_speed));
  }
  ConstraintProto::SpeedProfileProto lane_merge_speed_profile_constraint;
  *lane_merge_speed_profile_constraint.mutable_vt_upper_constraint() =
      std::move(lane_merge_speed_profile);
  lane_merge_speed_profile_constraint.mutable_source()
      ->mutable_lane_merge()
      ->set_reason("add lane merge speed limit constrain.");
  return lane_merge_speed_profile_constraint;
}

}  // namespace

absl::StatusOr<DeciderOutput>
BuildConstraints(  // NOLINT(readability-function-size)
    const DeciderInput& decider_input) {
  TIMELINE("BuildConstraints");

  CHECK_NOTNULL(decider_input.vehicle_geometry_params);
  CHECK_NOTNULL(decider_input.motion_constraint_params);
  CHECK_NOTNULL(decider_input.config);
  CHECK_NOTNULL(decider_input.planner_semantic_map_manager);
  CHECK_NOTNULL(decider_input.lc_state);
  CHECK_NOTNULL(decider_input.plan_start_point);
  CHECK_NOTNULL(decider_input.passage);
  CHECK_NOTNULL(decider_input.obj_mgr);
  CHECK_NOTNULL(decider_input.st_traj_mgr);
  CHECK_NOTNULL(decider_input.pre_decider_state);
  CHECK_NOTNULL(decider_input.behavior);
  CHECK_NOTNULL(decider_input.speed_state);
  CHECK_NOTNULL(decider_input.stalled_objects);
  // CHECK_NOTNULL(decider_input.map_event);

  const auto& vehicle_geometry_params = *decider_input.vehicle_geometry_params;
  const auto& motion_constraint_params =
      *decider_input.motion_constraint_params;
  const auto& config = *decider_input.config;
  const auto& planner_semantic_map_manager =
      *decider_input.planner_semantic_map_manager;
  const auto& lc_state = *decider_input.lc_state;
  const auto& plan_start_point = *decider_input.plan_start_point;
  const auto& passage = *decider_input.passage;
  const auto& obj_mgr = *decider_input.obj_mgr;
  const auto& sl_boundary = *decider_input.sl_boundary;
  const auto& st_traj_mgr = *decider_input.st_traj_mgr;
  // const auto& tl_info_map = *decider_input.tl_info_map;
  const auto traffic_light_status_map = decider_input.traffic_light_status_map;
  const auto& pre_decider_state = *decider_input.pre_decider_state;
  const auto& lc_num = decider_input.lc_num;
  const auto& left_navi_dist = decider_input.max_reach_length;
  const auto& map_func_id = decider_input.behavior->function_id();
  const auto& speed_state = *decider_input.speed_state;
  const auto& cur_dist_to_junction = decider_input.cur_dist_to_junction;
  const auto& traffic_light_fun_enable =
      decider_input.behavior->traffic_light_func_enable();
  const auto& plan_start_time = decider_input.plan_time;
  const absl::flat_hash_set<std::string>& stalled_objects =
      *decider_input.stalled_objects;
  const auto leading_id = decider_input.leading_id;
  const auto& cruising_speed_limit = *decider_input.cruising_speed_limit;
  const auto& behavior_set_speed =
      decider_input.behavior->cruising_speed_limit();

  ConstraintManager constraint_manager;
  DeciderStateProto new_decider_state;
  ad_byd::planning::SpeedState speed_state_output = speed_state;

  const PiecewiseLinearFunction<double> kLeftRouteDistanceRelativeSpeedGainPlf(
      {80.0, 120.0, 200.0, 280.0, 400.0, 500.0},
      {25.0, 40.0, 60.0, 80.0, 100.0, 120.0});
  const PiecewiseLinearFunction<double> kSpeedRelativeLaneChangeNeedMinDisPlf(
      {20.0, 30.0, 40.0, 50.0, 60.0, 80.0, 100.0, 120.0},
      {60.0, 80.0, 105.0, 130.0, 160.0, 220.0, 280.0, 350.0});

  bool no_acc_gap = false;
  if (decider_input.passage->empty()) {
    return absl::UnavailableError(
        "Drive passage on target lane path not available.");
  }
  // pre dec for miss navi
  double min_distance_to_end_or_junction = left_navi_dist;
  // special acc gap secnario
  bool special_acc_gap_secnario = false;

  // record lc_num & speed_limit to calculate lc_num change states
  double pre_speed_limit = speed_state_output.pre_fast_speed_limit;
  double pre_dynamic_acc = speed_state_output.pre_dynamic_acc_limit;
  bool pre_secnario_flag = speed_state_output.pre_special_acc_gap_secnario;
  int pre_lc_num = speed_state_output.pre_lc_num;
  bool lc_num_has_up = speed_state_output.lc_num_has_up;
  bool first_has_speed_limit = speed_state_output.first_has_speed_limit;

  // record is_construction_scene_speed_plan to calculate flag change state
  bool is_construction_speed_constrain_working_pre =
      speed_state_output.is_construction_speed_constrain_working_pre;

  // calculate speed_limit process
  if (lc_num == 1 && min_distance_to_end_or_junction > 30.0 &&
      decider_input.route_target_info != nullptr &&
      lc_state.stage() != LaneChangeStage::LCS_PAUSE) {
    std::vector<PlannerObjectProjectionInfo> front_objects_on_target_lane;
    const auto target_frenet_frame =
        decider_input.route_target_info->frenet_frame;
    const auto target_ego_frenet_box =
        decider_input.route_target_info->ego_frenet_box;
    for (const auto& [_, trajectories] :
         st_traj_mgr.object_trajectories_map()) {
      if (trajectories.empty()) continue;
      const PlannerObject& obj = trajectories.front()->planner_object();
      ASSIGN_OR_CONTINUE(
          const auto frenet_box,
          target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

      constexpr double kLateralThreshold = 1.0;  // m.
      if (frenet_box.l_min > kLateralThreshold ||
          frenet_box.l_max < -kLateralThreshold) {
        continue;
      }

      if (target_ego_frenet_box.center_s() > frenet_box.center_s()) {
        continue;
      }

      front_objects_on_target_lane.emplace_back(PlannerObjectProjectionInfo{
          .st_trajectories = trajectories, .frenet_box = frenet_box});
    }
    if (front_objects_on_target_lane.empty() ||
        front_objects_on_target_lane.size() == 1) {
      special_acc_gap_secnario = true;
    } else {
      std::stable_sort(front_objects_on_target_lane.begin(),
                       front_objects_on_target_lane.end(),
                       [](const auto& a, const auto& b) {
                         return a.frenet_box.s_min < b.frenet_box.s_min;
                       });
      double obstacle_density =
          min_distance_to_end_or_junction / front_objects_on_target_lane.size();
      double nearest_gap_length =
          front_objects_on_target_lane[1].frenet_box.s_min -
          front_objects_on_target_lane[0].frenet_box.s_max;
      Log2DDS::LogDataV2(
          "special_acc_gap_debug",
          absl::StrCat("obstacle_density: ", obstacle_density,
                       " nearest_gap_length: ", nearest_gap_length));
      if (obstacle_density > 40.0 && nearest_gap_length > 25.0) {
        special_acc_gap_secnario = true;
      }
    }
  }
  Log2DDS::LogDataV2(
      "special_acc_gap_debug",
      absl::StrCat("obstacle_density: ", special_acc_gap_secnario));

  std::vector<double> KSpeedDownDists(5);
  std::vector<double> KNaviDistStepThrs(4);
  std::vector<double> KPreLcDistThrs(3);
  const double kEverageDistOfLcNums =
      kSpeedRelativeLaneChangeNeedMinDisPlf(Mps2Kph(plan_start_point.v()));

  if (map_func_id ==
      Behavior_FunctionId_HW_NOA) {  // 待状态机有对城区和高速输出，此处再做区分
    KSpeedDownDists = {std::numeric_limits<double>::lowest(), 300.0, 500.0,
                       700.0, 1000.0};
    KNaviDistStepThrs = {300.0, 500.0, 700.0, 1000.0};
    KPreLcDistThrs = {200.0, 150.0, 100.0};
  } else {
    KSpeedDownDists = {std::numeric_limits<double>::lowest(), 150.0, 300.0,
                       450.0, 600.0};
    KNaviDistStepThrs = {150.0, 300.0, 450.0, 600.0};
    KPreLcDistThrs = {150.0, 100.0, 75.0};
  }

  int speed_level = -1;
  if (min_distance_to_end_or_junction < KNaviDistStepThrs[0]) {
    speed_level = 0;
  } else if (min_distance_to_end_or_junction < KNaviDistStepThrs[1]) {
    speed_level = 1;
  } else if (min_distance_to_end_or_junction < KNaviDistStepThrs[2]) {
    speed_level = 2;
  } else if (min_distance_to_end_or_junction < KNaviDistStepThrs[3]) {
    speed_level = 3;
  }

  double speed_down_dist =
      KSpeedDownDists[ad_byd::planning::math::Clamp(lc_num, 0, 4)];
  Log2DDS::LogDataV2(
      "pre_dec",
      absl::StrCat(Log2DDS::TaskPrefix(decider_input.plan_id),
                   "lc num: ", lc_num, "left navi dist: ", left_navi_dist));

  // auto_navi_lc_enable_status: 1 represent ai drive mode
  bool lcc_pre_dec_on = true;
  double target_speed = 0.0;
  constexpr double kLaneChangeSpeedOffset = 2.0;  // m/s
  constexpr double pre_speed_weight = 0.7;
  constexpr double target_speed_weight = 0.3;
  constexpr double cur_speed_to_target_speed_gap = 0.5;  // m/s
  constexpr double cur_acc_to_target_acc_gap = 0.2;      // m/s^2
  bool has_set_speed_limit = false;
  if (map_func_id == Behavior_FunctionId_LKA &&
      !decider_input.behavior->auto_navi_lc_enable_status()) {
    lcc_pre_dec_on = false;
  }

  if (min_distance_to_end_or_junction < speed_down_dist && speed_level >= 0 &&
      lc_num > 0 && lcc_pre_dec_on) {
    const double lc_speed_limit = kLeftRouteDistanceRelativeSpeedGainPlf(
        min_distance_to_end_or_junction / lc_num);
    target_speed = std::max(plan_start_point.v() - kLaneChangeSpeedOffset,
                            Kph2Mps(lc_speed_limit));
    double start_s = 0.0;
    double end_s = std::max(8 * plan_start_point.v(), 80.0);
    ASSIGN_OR_RETURN(const auto start_point, passage.QueryPointXYAtS(start_s));
    ASSIGN_OR_RETURN(const auto end_point, passage.QueryPointXYAtS(end_s));
    if (pre_lc_num < lc_num) {
      lc_num_has_up = true;
    }
    if ((pre_lc_num >= lc_num && (abs(pre_speed_limit - plan_start_point.v()) <
                                      cur_speed_to_target_speed_gap ||
                                  abs(pre_dynamic_acc - plan_start_point.a()) <
                                      cur_acc_to_target_acc_gap))) {
      lc_num_has_up = false;
    }
    if (pre_lc_num == 0 || pre_lc_num < lc_num) {
      pre_speed_limit = plan_start_point.v();
    }
    if ((lc_num >= pre_lc_num || pre_secnario_flag) &&
        !special_acc_gap_secnario) {
      first_has_speed_limit = true;
    }
    if ((lc_num == pre_lc_num) && (abs(pre_speed_limit - plan_start_point.v()) <
                                   cur_speed_to_target_speed_gap)) {
      first_has_speed_limit = false;
    }

    Log2DDS::LogDataV2("pre_dec",
                       absl::StrCat(Log2DDS::TaskPrefix(decider_input.plan_id),
                                    "dec to speed: ", target_speed));
    if (!special_acc_gap_secnario) {
      if (lc_num_has_up && first_has_speed_limit) {
        target_speed = Lerp(pre_speed_limit, target_speed, target_speed_weight);
      }
      constraint_manager.AddVLimit(target_speed, 2.0, "navi lc pre dec");
      no_acc_gap = true;
      pre_speed_limit = target_speed;
      has_set_speed_limit = true;
    }
    pre_secnario_flag = special_acc_gap_secnario;
    // ConstraintProto::SpeedRegionProto pre_dec_speed_region;
    // start_point.ToProto(pre_dec_speed_region.mutable_start_point());
    // end_point.ToProto(pre_dec_speed_region.mutable_end_point());
    // pre_dec_speed_region.set_start_s(start_s);
    // pre_dec_speed_region.set_end_s(end_s);
    // pre_dec_speed_region.set_max_speed(target_speed);
    // pre_dec_speed_region.set_min_speed(0.0);
    // pre_dec_speed_region.mutable_source()->mutable_pedestrian_object()->set_id(
    //     absl::StrCat("miss navi", "1"));
    // pre_dec_speed_region.set_id(absl::StrCat("miss navi", "1"));

    // constraint_manager.AddSpeedRegion(std::move(pre_dec_speed_region));
  }

  if (!special_acc_gap_secnario && lc_num != 0 &&
      (min_distance_to_end_or_junction / lc_num < kEverageDistOfLcNums) &&
      lcc_pre_dec_on) {
    double pre_lc_dist = min_distance_to_end_or_junction / lc_num;
    double remaining_dist =
        (min_distance_to_end_or_junction / lc_num) - kEverageDistOfLcNums;
    const std::vector<double> KTimes = {2.6, 2.4, 2.0, 1.8};
    int time_level = 0;
    if (pre_lc_dist < KPreLcDistThrs[0]) {
      time_level = 1;
    } else if (pre_lc_dist < KPreLcDistThrs[1]) {
      time_level = 2;
    } else if (pre_lc_dist < KPreLcDistThrs[2]) {
      time_level = 3;
    }

    double t = KTimes[time_level];
    constexpr double kEps = 1e-3;
    constexpr double kPreBrakeMaxDecel = -0.5;               // m/s^2
    constexpr double kPreBrakeLcSpdMinDec = 1.5;             // m/s
    constexpr double kPreBrakeLcSpdMaxDec = 2.5;             // m/s
    constexpr double kPreBrakeLcMinSpd = 4.5;                // m/s
    constexpr double kPreBrakeLcSpeedThres = Kph2Mps(20.0);  // km/h
    double dynamic_pre_dec_a =
        std::max((2 * remaining_dist) / (t * t), kPreBrakeMaxDecel);
    double target_speed_limit = kLeftRouteDistanceRelativeSpeedGainPlf(
        min_distance_to_end_or_junction / lc_num);
    target_speed_limit = std::max(Kph2Mps(target_speed_limit),
                                  plan_start_point.v() - kPreBrakeLcSpdMinDec);
    double temp_pre_dec_a = std::min(
        (target_speed_limit - plan_start_point.v()) / std::max(t, kEps), -kEps);
    dynamic_pre_dec_a = std::max(temp_pre_dec_a, dynamic_pre_dec_a);
    double dynamic_pre_dec_v =
        std::max(plan_start_point.v() + dynamic_pre_dec_a * t,
                 std::max(plan_start_point.v() - kPreBrakeLcSpdMaxDec,
                          kPreBrakeLcMinSpd));
    dynamic_pre_dec_v = std::max(dynamic_pre_dec_v,
                                 plan_start_point.v() - kPreBrakeLcSpdMinDec);
    Log2DDS::LogDataV2("Fast_speed_limit",
                       absl::StrCat(Log2DDS::TaskPrefix(decider_input.plan_id),
                                    "dynamic_pre_dec_a: ", dynamic_pre_dec_a,
                                    " dynamic_pre_dec_v: ", dynamic_pre_dec_v,
                                    "Strat_v: ", plan_start_point.v()));

    if (pre_lc_num == 0 || pre_lc_num < lc_num) {
      pre_dynamic_acc = plan_start_point.a();
    }
    if ((lc_num == pre_lc_num) && (abs(pre_dynamic_acc - plan_start_point.a()) <
                                   cur_acc_to_target_acc_gap)) {
      first_has_speed_limit = false;
    }

    if (plan_start_point.v() > kPreBrakeLcSpeedThres) {
      if (first_has_speed_limit || lc_num_has_up) {
        dynamic_pre_dec_v = std::min(
            Lerp(pre_speed_limit, dynamic_pre_dec_v, target_speed_weight),
            target_speed_limit);
        dynamic_pre_dec_a = std::max(
            Lerp(pre_dynamic_acc, dynamic_pre_dec_a, target_speed_weight),
            kPreBrakeMaxDecel);
      }

      constraint_manager.AddALimit(dynamic_pre_dec_a, dynamic_pre_dec_v,
                                   "dynamic pre dec");
      pre_speed_limit = dynamic_pre_dec_v;
      pre_dynamic_acc = dynamic_pre_dec_a;
      has_set_speed_limit = true;
    }
  }
  pre_lc_num = lc_num;
  is_construction_speed_constrain_working_pre =
      speed_state_output.is_construction_speed_constrain_working;
  speed_state_output.pre_fast_speed_limit = pre_speed_limit;
  speed_state_output.pre_lc_num = pre_lc_num;
  speed_state_output.lc_num_has_up = lc_num_has_up;
  speed_state_output.pre_special_acc_gap_secnario = pre_secnario_flag;
  speed_state_output.first_has_speed_limit = first_has_speed_limit;
  speed_state_output.pre_dynamic_acc_limit = pre_dynamic_acc;
  speed_state_output.is_construction_speed_constrain_working_pre =
      is_construction_speed_constrain_working_pre;

  const double start_s_offset = decider_input.target_offset_from_start;
  const auto lane_path_from_start =
      start_s_offset == 0.0
          ? decider_input.passage->lane_path()
          : decider_input.passage->lane_path().AfterArclength(start_s_offset);

  // params.
  // parking_brake_release_time = Unix 0, always return error
  // const bool requires_parking_brake_release = true;
  // if (config.enable_parking_brake_release() &&
  // requires_parking_brake_release) {
  //   auto parking_brake_release_constraint =
  //   BuildParkingBrakeReleaseConstraint(
  //       vehicle_geometry_params, passage,
  //       decider_input.parking_brake_release_time, decider_input.plan_time);
  //   if (parking_brake_release_constraint.ok()) {
  //     constraint_manager.AddStopLine(
  //         std::move(parking_brake_release_constraint).value());
  //   } else {
  //     VLOG(2) << "Build parking brake release constraint failed: "
  //             << parking_brake_release_constraint.status().ToString();
  //   }
  // }

  if (config.enable_lc_end_of_current_lane() &&
      lc_state.stage() == LaneChangeStage::LCS_PAUSE &&
      decider_input.lane_path_before_lc != nullptr) {
    const auto& lane_path_before_lc = *decider_input.lane_path_before_lc;
    if (!lane_path_before_lc.IsEmpty()) {
      auto lcp_speed = BuildLcEndOfCurrentLaneConstraints(
          passage, lane_path_before_lc, plan_start_point.v());
      if (lcp_speed.ok()) {
        constraint_manager.AddSpeedRegion(std::move(lcp_speed).value());
      }
    }
  }

  double current_v = plan_start_point.v();
  double break_dist = 1000.0;

  if (decider_input.eie_choice_type !=
          ad_byd::planning::EIEChoiceType::CHOICE_NONE &&
      !decider_input.eie_braking_down_flag &&
      current_v > kEIEPreBrakeTargetSpd &&
      RightLaneIsRmergenctLane(lane_path_from_start,
                               planner_semantic_map_manager)) {
    constraint_manager.AddALimit(kEIEBrakeDefaultDec, 0.0,
                                 "EIE pre break action decelration");
  }

  if (!decider_input.eie_braking_down_flag) {
    break_dist = (current_v * current_v) / (-2 * kEIEBrakeActionDec);
    speed_state_output.last_break_dist = break_dist;
  } else {
    break_dist = speed_state_output.last_break_dist;
  }

  bool supass_eie_by_junction =
      SupassEIEByJunction(decider_input.cur_dist_to_junction,
                          decider_input.cur_dist_to_prev_junction, break_dist);

  bool supass_eie_by_subpath =
      SupassEIEBySubPath(lane_path_from_start, planner_semantic_map_manager,
                         break_dist, decider_input.plan_id);

  bool supass_eie_by_tunnel =
      SupassEIEByTunnel(decider_input.dist_to_tunnel_entrance,
                        decider_input.dist_to_tunnel_exitance, break_dist);

  bool supass_eie_break_function = false;

  bool active_eie_break = false;

  const bool test = true;

  if (map_func_id == Behavior_FunctionId_CITY_NOA) {
    supass_eie_break_function = supass_eie_by_junction || supass_eie_by_tunnel;
    active_eie_break =
        decider_input.eie_braking_down_flag && !supass_eie_break_function;
  } else if (map_func_id == Behavior_FunctionId_HW_NOA) {
    supass_eie_break_function = supass_eie_by_subpath || supass_eie_by_tunnel;
    active_eie_break =
        decider_input.eie_braking_down_flag && !supass_eie_break_function;
  } else if (map_func_id == Behavior_FunctionId_LKA) {
    active_eie_break = decider_input.eie_braking_down_flag;
  }

  if (active_eie_break) {
    if (speed_state_output.is_eie_first_action) {
      speed_state_output.eie_atcion_start_time =
          absl::ToUnixMillis(plan_start_time) / 1000.0;
      speed_state_output.is_eie_first_action = false;
    }

    double eie_action_time = absl::ToUnixMillis(plan_start_time) / 1000.0;
    double brake_dec = kEIEBrakeDefaultDec;

    double eie_action_duration_time =
        eie_action_time - speed_state_output.eie_atcion_start_time;
    if (eie_action_duration_time > kEIEBrakeTimeThres) {
      brake_dec = kEIEBrakeActionDec;
    }

    constraint_manager.AddALimit(brake_dec, 0.0, "EIE action decelration");
    auto stop_line_or = BuildBrakeToStopConstraint(
        /*type_str=*/"BEHAVIOR_CHOICE_EIE_BRAKE", passage,
        vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
        -brake_dec);

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
      speed_state_output.is_eie_cnoa_action = true;
    }
  }

  if (!decider_input.eie_braking_down_flag &&
      !speed_state_output.is_eie_first_action) {
    speed_state_output.is_eie_first_action = true;
    speed_state_output.is_eie_cnoa_action = false;
  }

  if (supass_eie_break_function) {
    speed_state_output.is_eie_cnoa_action = false;
  }

  Log2DDS::LogDataV0(
      "EIE break function",
      absl::StrCat(
          Log2DDS::TaskPrefix(decider_input.plan_id),
          " supass_eie_by_subpath: ", supass_eie_by_subpath,
          " supass_eie_by_junction: ", supass_eie_by_junction,
          " supass_eie_by_tunnel: ", supass_eie_by_tunnel,
          " eie_braking_down_flag: ", decider_input.eie_braking_down_flag,
          " break_dist : ", break_dist, " cur_dist_to_junction: ",
          decider_input.cur_dist_to_junction, " cur_dist_to_prev_junction: ",
          decider_input.cur_dist_to_prev_junction,
          " dist_to_tunnel_entrance: ", decider_input.dist_to_tunnel_entrance,
          " dist_to_tunnel_exitance: ", decider_input.dist_to_tunnel_exitance));

  // const auto* lane_info_ptr =
  // planner_semantic_map_manager.FindCurveLaneByIdOrNull(
  //     lane_path_from_start.front().lane_id());
  // if (config.enable_beyond_length_along_route() && lane_info_ptr != nullptr)
  // {
  //   const auto* section_info_ptr =
  //       planner_semantic_map_manager.FindSectionInfoOrNull(
  //           lane_info_ptr->section_id);
  //   if (section_info_ptr != nullptr && section_info_ptr->proto != nullptr &&
  //       section_info_ptr->proto->road_class() ==
  //           mapping::SectionProto::NORMAL) {
  //     auto beyond_len_along_route_speed =
  //     BuildBeyondLengthAlongRouteConstraint(
  //         passage, motion_constraint_params, decider_input.max_reach_length,
  //         decider_input.borrow_lane_boundary, plan_start_point.v());
  //     if (beyond_len_along_route_speed.ok()) {
  //       constraint_manager.AddSpeedProfile(
  //           std::move(beyond_len_along_route_speed).value());
  //     }
  //   }
  // }

  if (config.enable_crosswalk()) {
    // Crosswalk.
    ASSIGN_OR_RETURN(
        auto cw_decider_output,
        BuildCrosswalkConstraints(CrosswalkDeciderInput{
            .vehicle_geometry_params = &vehicle_geometry_params,
            .psmm = &planner_semantic_map_manager,
            .plan_start_point = &plan_start_point,
            .passage = &passage,
            .lane_path_from_start = &lane_path_from_start,
            .obj_mgr = &obj_mgr,
            .last_crosswalk_states = &pre_decider_state.crosswalk_state(),
            .now_in_seconds = ToUnixDoubleSeconds(decider_input.plan_time),
            .s_offset = start_s_offset,
        }));

    for (auto& cw_stop_line : cw_decider_output.stop_lines) {
      constraint_manager.AddStopLine(std::move(cw_stop_line));
    }
    for (auto& cw_speed_region : cw_decider_output.speed_regions) {
      constraint_manager.AddSpeedRegion(std::move(cw_speed_region));
    }
    for (auto& crosswalk_state : cw_decider_output.crosswalk_states) {
      *new_decider_state.add_crosswalk_state() = std::move(crosswalk_state);
    }
  }

  // Pedestrians.
  if (config.enable_pedestrians()) {
    // lane keep
    if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
      ASSIGN_OR_RETURN(
          auto ped_speed_regions,
          BuildPedestriansConstraints(
              vehicle_geometry_params, planner_semantic_map_manager,
              plan_start_point, passage, lane_path_from_start, start_s_offset,
              sl_boundary, st_traj_mgr));

      for (auto& ped_speed_region : ped_speed_regions) {
        constraint_manager.AddSpeedRegion(std::move(ped_speed_region));
      }
    }
  }

  // No block constraints.
  if (config.enable_no_block()) {
    auto no_block_regions =
        BuildNoBlockConstraints(planner_semantic_map_manager, passage,
                                lane_path_from_start, start_s_offset);
    for (auto& no_block_region : no_block_regions) {
      CHECK_LE(no_block_region.start_s(), no_block_region.end_s())
          << no_block_region.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(no_block_region));
    }
  }

  // If we have end of current lane path constraint, add it.
  auto end_of_cur_lp_constraint = BuildEndOfCurrentLanePathConstraint(passage);
  if (end_of_cur_lp_constraint.ok()) {
    constraint_manager.AddStopLine(std::move(end_of_cur_lp_constraint).value());
  }

  // Add the end of path boundary constraint.
  // auto end_of_path_boundary_constraint =
  //     BuildEndOfPathBoundaryConstraint(passage, sl_boundary);
  // if (end_of_path_boundary_constraint.ok()) {
  //   constraint_manager.AddStopLine(
  //       std::move(end_of_path_boundary_constraint).value());
  // } else {
  //   LOG_WARN << "Build end of path boundary constraint failed: "
  //            << end_of_path_boundary_constraint.status().ToString();
  // }

  if (config.enable_traffic_obstacles() &&
      passage.traffic_static_obstacles_info().enable_stop) {
    // Traffic_obstacles
    auto end_of_traffic_static_obstacles_constraint =
        BuildTrafficStaticObstaclesConstraint(passage);
    if (end_of_traffic_static_obstacles_constraint.ok()) {
      constraint_manager.AddStopLine(
          std::move(end_of_traffic_static_obstacles_constraint).value());
      Log2DDS::LogDataV2(
          "Traffic_static_obs_speed_limit",
          absl::StrCat(Log2DDS::TaskPrefix(decider_input.plan_id),
                       "Traffic_static_obs add stopline"));

    } else {
      LOG_WARN
          << "Build end_of_traffic_static_obstacles_constraint failed: "
          << end_of_traffic_static_obstacles_constraint.status().ToString();
    }

  } else if (config.enable_traffic_obstacles() &&
             passage.traffic_static_obstacles_info().enable_slow_down) {
    double desire_a = -1.0;
    double desire_v = Kph2Mps(30.0);
    // double dynamic_pre_dec_v = std::sqrt(plan_start_point.v() *
    // plan_start_point.v() - 2 * a *
    // passage.traffic_static_obstacles_info().stop_s);
    constraint_manager.AddALimit(desire_a, desire_v,
                                 "dynamic pre dec static_obs");
    Log2DDS::LogDataV2("Traffic_static_obs_speed_limit",
                       absl::StrCat(Log2DDS::TaskPrefix(decider_input.plan_id),
                                    "dynamic_pre_dec_a: ", desire_a,
                                    " dynamic_pre_dec_v: ", desire_v,
                                    "Strat_v: ", plan_start_point.v()));
  }

  if (config.enable_speed_bump()) {
    // Speed bump.
    auto speed_bumps =
        BuildSpeedBumpConstraints(planner_semantic_map_manager, passage);
    for (auto& speed_bump : speed_bumps) {
      CHECK_LE(speed_bump.start_s(), speed_bump.end_s())
          << speed_bump.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(speed_bump));
    }
  }

  if (config.enable_cautious_brake()) {
    // Intersection
    auto cautious_brake_regions = BuildCautiousBrakeConstraints(
        planner_semantic_map_manager, passage, lane_path_from_start,
        start_s_offset, st_traj_mgr);
    for (auto& cautious_brake : cautious_brake_regions) {
      CHECK_LE(cautious_brake.start_s(), cautious_brake.end_s())
          << cautious_brake.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(cautious_brake));
    }
  }

  if (config.enable_toll()) {
    ASSIGN_OR_RETURN(
        auto toll_speed_regions,
        BuildTollConstraints(planner_semantic_map_manager, passage,
                             lane_path_from_start, start_s_offset));

    for (auto& toll_speed_region : toll_speed_regions) {
      constraint_manager.AddSpeedRegion(std::move(toll_speed_region));
    }
  }

  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  int tl_stop_interface = 0;
  TrafficLightIndicationInfoProto tl_ind_info;
  if (traffic_light_status_map != nullptr && config.enable_traffic_light()) {
    // teleop_enable_traffic_light_stop always true
    // decider_input.teleop_enable_traffic_light_stop) {
    // SpeedProfile need speed regions and stop line information from other
    // constraints, build this constraint at last.
    // SpeedProfile preliminary_speed_profile = CreateSpeedProfile(
    //     plan_start_point.v(), passage, planner_semantic_map_manager,
    //     constraint_manager.SpeedRegion(), constraint_manager.StopLine());
    std::vector<std::string> tl_input_debug;
    tl_input_debug.reserve(1);
    tl_input_debug.emplace_back(absl::StrCat(decider_input.plan_id, "*ok-",
                                             decider_input.enable_tl_ok_btn));
    Log2DDS::LogDataV2("fsd-traffic", tl_input_debug);
    bool need_prebrake_for_lka = false;
    auto tl_decider_output = BuildTrafficLightConstraints(
        vehicle_geometry_params, plan_start_point, passage,
        lane_path_from_start, *traffic_light_status_map,
        planner_semantic_map_manager,
        pre_decider_state.traffic_light_decider_state(),
        decider_input.enable_tl_ok_btn, decider_input.override_passable, config,
        map_func_id, st_traj_mgr, traffic_light_fun_enable,
        need_prebrake_for_lka, plan_start_time);

    if (!tl_decider_output.ok()) {
      LOG_WARN << "Build tl stop lines failed with message: "
               << tl_decider_output.status().ToString();
    } else {
      for (auto& tl_stop_line : tl_decider_output.value().stop_lines) {
        if (!distance_to_traffic_light_stop_line.has_value()) {
          distance_to_traffic_light_stop_line = tl_stop_line.s();
        }
        constraint_manager.AddStopLine(tl_stop_line);
      }
      // for (auto& tl_speed_profile : tl_decider_output.value().speed_profiles)
      // {
      //   constraint_manager.AddSpeedProfile(std::move(tl_speed_profile));
      // }
      tl_stop_interface = tl_decider_output.value()
                              .traffic_light_decider_state.fsd_tld_state()
                              .tl_stop_interface();
      tl_ind_info = tl_decider_output.value().tl_ind_info;
      *new_decider_state.mutable_traffic_light_decider_state() =
          tl_decider_output.value().traffic_light_decider_state;
      // Add traffic light stop line on GUI.
      const int stop_line_size = tl_decider_output.value().stop_lines.size();
      for (int i = 0; i < stop_line_size; ++i) {
        const double traffic_stop_line_s =
            tl_decider_output.value().stop_lines[i].s();
        std::vector<Vec2d> stop_line_points;
        auto left_point = passage.QueryPointXYAtSL(traffic_stop_line_s, 1.5);
        auto right_point = passage.QueryPointXYAtSL(traffic_stop_line_s, -1.5);
        if (left_point.ok() && right_point.ok()) {
          stop_line_points.push_back(left_point.value());
          stop_line_points.push_back(right_point.value());
          const auto& prefix = Log2DDS::TaskPrefix(decider_input.plan_id);
          Log2DDS::LogLineV2(absl::StrCat(prefix, "traffic-stop-line"),
                             Log2DDS::kYellow, {}, stop_line_points);
        }
      }
    }
    if (need_prebrake_for_lka) {
      Log2DDS::LogDataV2("fsd-traffic", "lka prebrake!!!");
      constraint_manager.AddALimit(-1.0, Kph2Mps(70.0), "lka prebrake");
    }
  }

  if (map_func_id == Behavior_FunctionId_LKA &&
      !decider_input.behavior->auto_navi_lc_enable_status()) {
    double stop_s = 0.0;
    auto stop_line_or = BuildLCCStopConstraint(
        vehicle_geometry_params, config, plan_start_point, lane_path_from_start,
        planner_semantic_map_manager, passage, speed_state.lcc_keep_brake,
        &stop_s);

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
      speed_state_output.lcc_keep_brake = true;
      // set ui interface
      const double kUIOutputDist = 100.0;  // m
      if (stop_s < kUIOutputDist) {
        tl_stop_interface = StopLineInterface::STOP_LINE_LCC_TURN;
      }
    } else {
      speed_state_output.lcc_keep_brake = false;
    }

    // junction T map
    const auto& sub_type = planner_semantic_map_manager.map_ptr()->sub_type();
    if (sub_type == ad_byd::planning::JUNCTION_T_MAP) {
      double stop_s = 0.0;
      auto stop_line_or = BuildTJunctionStopConstraint(
          vehicle_geometry_params, config, plan_start_point,
          lane_path_from_start, planner_semantic_map_manager, passage, &stop_s);

      if (stop_line_or.ok()) {
        constraint_manager.AddStopLine(std::move(stop_line_or).value());
        // set ui interface
        const double kTJunctionOutputDist = 80.0;  // m
        if (stop_s < kTJunctionOutputDist) {
          tl_stop_interface = StopLineInterface::STOP_LINE_T_JUNCTION;
        }
        std::vector<Vec2d> stop_line_points;
        auto left_point = passage.QueryPointXYAtSL(stop_s, 1.5);
        auto right_point = passage.QueryPointXYAtSL(stop_s, -1.5);
        if (left_point.ok() && right_point.ok()) {
          stop_line_points.push_back(left_point.value());
          stop_line_points.push_back(right_point.value());
          const auto& prefix = Log2DDS::TaskPrefix(decider_input.plan_id);
          Log2DDS::LogLineV2(absl::StrCat(prefix, "T-junction-stop-line"),
                             Log2DDS::kYellow, {}, stop_line_points);
        }
      }
    }
  }

  // if (decider_input.enable_pull_over) {
  //   constexpr double kPullOverBrake = 1.0;  // m/s^2
  //   auto stop_line_or = BuildBrakeToStopConstraint(
  //       /*type_str=*/"pull_over", passage,
  //       vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
  //       kPullOverBrake);

  //   if (stop_line_or.ok()) {
  //     constraint_manager.AddStopLine(std::move(stop_line_or).value());
  //   }
  // }

  // if (decider_input.brake_to_stop.has_value()) {
  //   auto stop_line_or = BuildBrakeToStopConstraint(
  //       /*type_str=*/"brake_to_stop", passage,
  //       vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
  //       *decider_input.brake_to_stop);

  //   if (stop_line_or.ok()) {
  //     constraint_manager.AddStopLine(std::move(stop_line_or).value());
  //   }
  // }

  // for (const auto& lead_obj : decider_input.lc_lead_obj_ids) {
  //   Log2DDS::LogDataV2("gap_debug_test", "dp lead obj: " + lead_obj);
  // }
  std::optional<double> merge_speed_limit = std::nullopt;
  if (config.enable_lane_merge()) {
    const auto road_type = planner_semantic_map_manager.GetRoadClass();
    const bool is_on_highway = planner_semantic_map_manager.IsOnHighway() ||
                               road_type == V2RoadType::HIGH_WAY_ROAD ||
                               road_type == V2RoadType::EXPRESS_WAY_ROAD;
    const auto prefix = Log2DDS::TaskPrefix(decider_input.plan_id);
    auto lane_merge_speed_limit =
        BuildLaneMergeConstraint(passage, plan_start_point.v(), is_on_highway,
                                 behavior_set_speed, lc_state);
    if (lane_merge_speed_limit.ok()) {
      if (auto& value = lane_merge_speed_limit.value();
          value.has_vt_upper_constraint()) {
        if (!value.vt_upper_constraint().y().empty()) {
          merge_speed_limit = *value.vt_upper_constraint().y().rbegin();
        }
      }
      constraint_manager.AddSpeedProfile(
          std::move(lane_merge_speed_limit).value());
    } else {
      Log2DDS::LogDataV2(
          absl::StrCat(prefix, "merge_speed_failed_reason"),
          std::string(lane_merge_speed_limit.status().message()));
    }
  }

  if (config.enable_prepare_lc() &&
      decider_input.route_target_info != nullptr &&
      !decider_input.borrow_lane_boundary) {
    constexpr double kMinGapCalSpeed = 2.777;          // m/s 10km/h
    constexpr double kMinGapCalSpeed_lb = 2.222;       // m/s 8km/h
    constexpr double kMaxGapCalSpeedPause = 8.333;     // m/s 30km/h
    constexpr double kMaxGapCalSpeedPause_ub = 9.444;  // m/s 34km/h

    // 1. speed debounce
    bool gap_cal_flag = true;
    if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
      if (!pre_decider_state.traffic_gap().is_gap_cal()) {
        if (plan_start_point.v() <= kMinGapCalSpeed) {
          gap_cal_flag = false;
        }
      } else {
        if (plan_start_point.v() < kMinGapCalSpeed_lb) {
          gap_cal_flag = false;
        }
      }
    } else if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
      // if (!pre_decider_state.traffic_gap().is_gap_cal()) {
      //   if (plan_start_point.v() > kMaxGapCalSpeedPause) {
      //     gap_cal_flag = false;
      //   }
      // } else {
      //   if (plan_start_point.v() >= kMaxGapCalSpeedPause_ub) {
      //     gap_cal_flag = false;
      //   }
      // }
    }
    // for merge
    double dist_to_merge = std::numeric_limits<double>::max();
    if (decider_input.passage->lane_seq_info() != nullptr) {
      dist_to_merge = decider_input.passage->lane_seq_info()->dist_to_merge;
      if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
        if (!decider_input.passage->lane_seq_info()
                 ->dist_to_merge_vec.empty()) {
          dist_to_merge =
              decider_input.passage->lane_seq_info()->dist_to_merge_vec.front();
        }
      }
    }

    Log2DDS::LogDataV2(
        "gap_debug",
        absl::StrCat("v: ", plan_start_point.v(), ", pre_gap_cal_flag: ",
                     pre_decider_state.traffic_gap().is_gap_cal(),
                     ", cur_gap_cal_flag: ", gap_cal_flag,
                     ", dist_to_merge: ", dist_to_merge));

    // 2. gap calculation
    if (gap_cal_flag) {
      auto traffic_gaps = FindCandidateTrafficGapsOnLanePath(
          decider_input.route_target_info->frenet_frame,
          decider_input.route_target_info->ego_frenet_box,
          decider_input.route_target_info->st_traj_mgr, plan_start_point.v(),
          dist_to_merge, pre_decider_state, decider_input.lc_lead_obj_ids,
          plan_start_point.path_point().theta(), stalled_objects, lc_state);
      // log
      for (int i = 0; i < traffic_gaps.size(); ++i) {
        auto candidate_gap =
            new_decider_state.mutable_candidate_traffic_gap()->Add();
        std::string leader_id = "";
        std::string follower_id = "";
        if (!traffic_gaps[i].leader_trajectories.empty()) {
          leader_id = std::string(
              traffic_gaps[i].leader_trajectories.front()->object_id());
          candidate_gap->set_leader_id(leader_id);
        }
        if (!traffic_gaps[i].follower_trajectories.empty()) {
          follower_id = std::string(
              traffic_gaps[i].follower_trajectories.front()->object_id());
          candidate_gap->set_follower_id(follower_id);
        }
        Log2DDS::LogDataV2(
            "gap_debug",
            absl::StrCat("gap list: ", i, ", leader_id: ", leader_id,
                         ", follower_id: ", follower_id));
      }

      std::optional<PlannerObjectProjectionInfo> leader_obj_on_lane =
          std::nullopt;
      double speed_limit = decider_input.behavior->cruising_speed_limit();
      if (has_set_speed_limit) {
        speed_limit =
            std::min(speed_limit, speed_state_output.pre_fast_speed_limit);
      }
      double speed_limit_history_v = 0.0;
      if (merge_speed_limit.has_value() && dist_to_merge < 500.0) {
        speed_limit_history_v = merge_speed_limit.value();
        speed_limit = std::min(speed_limit, merge_speed_limit.value());
      }
      Log2DDS::LogDataV2(
          "gap_debug",
          absl::StrCat(
              "speed limit: ", speed_limit,
              ", has_set_speed_limit: ", has_set_speed_limit,
              ", has_merge_speed_limit: ", merge_speed_limit.has_value(),
              ", merge_speed_limit: ", speed_limit_history_v));

      std::optional<double> leader_obj_s_min = std::nullopt;
      std::optional<double> leader_obj_v = std::nullopt;
      if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
        const auto ego_pos =
            Vec2dFromApolloTrajectoryPointProto(plan_start_point);
        const Box2d ego_box =
            ComputeAvBox(ego_pos, plan_start_point.path_point().theta(),
                         vehicle_geometry_params);
        const auto& ego_frenet_frame = *passage.frenet_frame();
        const auto ego_frenet_box = *ego_frenet_frame.QueryFrenetBoxAt(ego_box);
        std::vector<PlannerObjectProjectionInfo> objects_on_current_lane =
            GetObstaclesOnLane(ego_frenet_frame, ego_frenet_box, st_traj_mgr);
        if (!objects_on_current_lane.empty()) {
          Log2DDS::LogDataV2(
              "gap_debug",
              absl::StrCat("front obstacle: ", objects_on_current_lane.front()
                                                   .st_trajectories.front()
                                                   ->object_id()));
          leader_obj_s_min = objects_on_current_lane.front().frenet_box.s_min -
                             ego_frenet_box.s_max;
          leader_obj_v = objects_on_current_lane.front()
                             .st_trajectories.front()
                             ->pose()
                             .v();

          // project leader on lane into target frenet frame // TODO:
          auto leader_obj_on_lane_temp = objects_on_current_lane.front();
          leader_obj_on_lane_temp.frenet_box.s_min =
              decider_input.route_target_info->ego_frenet_box.s_max +
              leader_obj_s_min.value();
          leader_obj_on_lane = leader_obj_on_lane_temp;
        }

      } else if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
        // pause calculate leader object in front of ego
        const auto objects_obj_in_front_ego = GetObstacleInFrontOfEgo(
            decider_input.route_target_info->frenet_frame,
            decider_input.route_target_info->ego_frenet_box,
            decider_input.route_target_info->st_traj_mgr);
        if (!objects_obj_in_front_ego.empty()) {
          leader_obj_on_lane = objects_obj_in_front_ego.front();
          Log2DDS::LogDataV2(
              "gap_debug",
              absl::StrCat("2.front obstacle: ", leader_obj_on_lane.value()
                                                     .st_trajectories.front()
                                                     ->object_id()));
        }
      }

      auto gap_or = EvaluateAndTakeBestTrafficGap(
          traffic_gaps, decider_input.route_target_info->ego_frenet_box,
          decider_input.route_target_info->frenet_frame, plan_start_point.v(),
          speed_limit, left_navi_dist, lc_num, dist_to_merge,
          pre_decider_state.traffic_gap(), map_func_id,
          plan_start_point.path_point().theta(), leader_obj_on_lane);
      if (gap_or.ok()) {
        constraint_manager.SetTrafficGap(std::move(gap_or).value());
      }
      new_decider_state.mutable_traffic_gap()->set_is_gap_cal(true);
      // if find cur best in pre gap counter map, assign counter in
      // new_decider_state; if ego is within cur best gap, counter + 1; if
      // switch a new best gap, push back a new gap in map
      bool find_current = false;
      if (pre_decider_state.has_traffic_gap()) {
        for (const auto& best_gap :
             pre_decider_state.traffic_gap().best_gap_count()) {
          auto best_gap_count =
              new_decider_state.mutable_traffic_gap()->add_best_gap_count();
          best_gap_count->set_lead_id(best_gap.lead_id());
          best_gap_count->set_follow_id(best_gap.follow_id());
          best_gap_count->set_gap_count(best_gap.gap_count());
          if (constraint_manager.TrafficGap().leader_id.has_value() &&
              constraint_manager.TrafficGap().follower_id.has_value() &&
              (best_gap.lead_id() ==
                   constraint_manager.TrafficGap().leader_id.value() ||
               best_gap.follow_id() ==
                   constraint_manager.TrafficGap().follower_id.value())) {
            if (constraint_manager.TrafficGap().ego_in_gap) {
              best_gap_count->set_gap_count(best_gap.gap_count() + 1);
            }
            find_current = true;
          } else if (constraint_manager.TrafficGap().leader_id.has_value() &&
                     !constraint_manager.TrafficGap().follower_id.has_value() &&
                     best_gap.lead_id() ==
                         constraint_manager.TrafficGap().leader_id.value()) {
            if (constraint_manager.TrafficGap().ego_in_gap) {
              best_gap_count->set_gap_count(best_gap.gap_count() + 1);
            }
            find_current = true;
          } else if (!constraint_manager.TrafficGap().leader_id.has_value() &&
                     constraint_manager.TrafficGap().follower_id.has_value() &&
                     best_gap.follow_id() ==
                         constraint_manager.TrafficGap().follower_id.value()) {
            if (constraint_manager.TrafficGap().ego_in_gap) {
              best_gap_count->set_gap_count(best_gap.gap_count() + 1);
            }
            find_current = true;
          }
        }
        if (!find_current) {
          auto best_gap_count =
              new_decider_state.mutable_traffic_gap()->add_best_gap_count();
          if (constraint_manager.TrafficGap().leader_id.has_value()) {
            best_gap_count->set_lead_id(
                constraint_manager.TrafficGap().leader_id.value());
          } else {
            best_gap_count->set_lead_id("");
          }
          if (constraint_manager.TrafficGap().follower_id.has_value()) {
            best_gap_count->set_follow_id(
                constraint_manager.TrafficGap().follower_id.value());
          } else {
            best_gap_count->set_follow_id("");
          }
          best_gap_count->set_gap_count(0);
        }
      }
      if (constraint_manager.TrafficGap().gap_ref_v.has_value()) {
        new_decider_state.mutable_traffic_gap()->set_follow_traffic_flow(true);
        new_decider_state.mutable_traffic_gap()->set_traffic_flow_v(
            constraint_manager.TrafficGap().gap_ref_v.value());
      } else {
        new_decider_state.mutable_traffic_gap()->set_follow_traffic_flow(false);
      }
      if (constraint_manager.TrafficGap().leader_id.has_value()) {
        new_decider_state.mutable_traffic_gap()->set_leader_id(
            constraint_manager.TrafficGap().leader_id.value());
      }
      if (constraint_manager.TrafficGap().follower_id.has_value()) {
        new_decider_state.mutable_traffic_gap()->set_follower_id(
            constraint_manager.TrafficGap().follower_id.value());
      }
      if (constraint_manager.TrafficGap().gap_length.has_value()) {
        new_decider_state.mutable_traffic_gap()
            ->mutable_gap_point()
            ->set_gap_len(constraint_manager.TrafficGap().gap_length.value());
      }
      if (constraint_manager.TrafficGap().gap_ttc.has_value()) {
        new_decider_state.mutable_traffic_gap()->mutable_gap_point()->set_ttc(
            constraint_manager.TrafficGap().gap_ttc.value());
      }
      if (constraint_manager.TrafficGap().gap_v.has_value()) {
        new_decider_state.mutable_traffic_gap()->mutable_gap_point()->set_v(
            constraint_manager.TrafficGap().gap_v.value());
      }
      if (constraint_manager.TrafficGap().gap_a.has_value()) {
        new_decider_state.mutable_traffic_gap()->mutable_gap_point()->set_a(
            constraint_manager.TrafficGap().gap_a.value());
      }
      if (constraint_manager.TrafficGap().gap_t.has_value()) {
        new_decider_state.mutable_traffic_gap()->mutable_gap_point()->set_t(
            constraint_manager.TrafficGap().gap_t.value());
      }
      if (constraint_manager.TrafficGap().gap_total_cost.has_value()) {
        new_decider_state.mutable_traffic_gap()
            ->mutable_gap_point()
            ->set_total_cost(
                constraint_manager.TrafficGap().gap_total_cost.value());
      }
    }
  }

  if (config.enable_standstill()) {
    ASSIGN_OR_RETURN(
        auto ss_stop_lines,
        BuildStandstillConstraints(vehicle_geometry_params, plan_start_point,
                                   passage, constraint_manager.StopLine()));
    for (auto& ss_stop_line : ss_stop_lines) {
      constraint_manager.AddStopLine(std::move(ss_stop_line));
    }
  }

  if (config.enable_solid_line_within_boundary()) {
    const auto solid_lines = BuildSolidLineWithinBoundaryConstraint(
        passage, sl_boundary, plan_start_point);
    if (solid_lines.ok()) {
      for (auto& solid_line : *solid_lines) {
        constraint_manager.AddAvoidLine(solid_line);
      }
    }
  }

  if (config.enable_inferred_object() &&
      decider_input.scene_reasoning != nullptr) {
    auto inferred_object_constraint_or = BuildInferredObjectConstraint(
        planner_semantic_map_manager, *decider_input.scene_reasoning,
        lane_path_from_start, plan_start_point.v());
    if (inferred_object_constraint_or.ok()) {
      constraint_manager.AddSpeedProfile(
          std::move(inferred_object_constraint_or).value());
    }
  }

  /*if (decider_input.behavior->system_break_stop() &&) {
    constexpr double kSystemBrake = 1.5;  // m/s^2
    auto stop_line_or = BuildBrakeToStopConstraint(
        /*type_str= "BEHAVIOR_CHOICE_MRM_BRAKE", passage,
        vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
        kSystemBrake);

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
    }
  }*/

  // yield to vru constraint must be the last constraint to avoid misrecall
  if (config.enable_yield_to_vru() && decider_input.obs_history) {
    auto stop_line_or = BuildYieldToVruConstraint(
        obj_mgr, *decider_input.obs_history, vehicle_geometry_params,
        plan_start_point, lane_path_from_start, planner_semantic_map_manager,
        passage, distance_to_traffic_light_stop_line, &speed_state_output);

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
      speed_state_output.yield_to_vru = true;
    } else {
      speed_state_output.yield_to_vru = false;
    }
  }
  if (config.enable_construction_scene_speed_limit()) {
    const auto road_type = planner_semantic_map_manager.GetRoadClass();
    const bool is_on_highway = planner_semantic_map_manager.IsOnHighway() ||
                               road_type == V2RoadType::HIGH_WAY_ROAD ||
                               road_type == V2RoadType::EXPRESS_WAY_ROAD;
    const auto prefix = Log2DDS::TaskPrefix(decider_input.plan_id);
    auto construction_scene_speed_limit = BuildConstructionSpeedConstraint(
        passage, planner_semantic_map_manager, cruising_speed_limit,
        is_on_highway, plan_start_point.v(), plan_start_time,
        &speed_state_output);
    if (construction_scene_speed_limit.ok()) {
      constraint_manager.AddSpeedProfile(
          std::move(construction_scene_speed_limit).value());
      speed_state_output.is_construction_scene_speed_plan = true;
    } else {
      Log2DDS::LogDataV2(
          absl::StrCat(prefix, "construction_scene_failed_reason"),
          std::string(construction_scene_speed_limit.status().message()));
    }
  }

  // *ATTENTION* If constraints need to be added,
  // please evaluate if it has to be built after YieldToVruConstraint
  return DeciderOutput{
      .constraint_manager = std::move(constraint_manager),
      .decider_state = std::move(new_decider_state),
      .distance_to_traffic_light_stop_line =
          distance_to_traffic_light_stop_line,
      .tl_stop_interface = tl_stop_interface,
      .speed_state = speed_state_output,
      .tl_ind_info = std::move(tl_ind_info),
  };
}

}  // namespace planning
}  // namespace st
