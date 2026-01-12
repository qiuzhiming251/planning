#include "defensive_speed_decider.h"

#include "plan_common/drive_passage.h"
#include "plan_common/math/line_fitter.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "object_manager/object_history_util.h"

namespace st::planning {
namespace {

constexpr double kEps = 1e-3;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

constexpr int kMinFitNum = 6;
constexpr double kDefensiveDecelTmp = -0.7;  // m/ss.

const PiecewiseLinearFunction<double, double> kLonDistLateralTtcPlf(
    {30.0, 40.0, 50.0}, {8.0, 7.0, 6.0});
const PiecewiseLinearFunction<double, double> kRelSpeedPassFactorPlf(
    {0.0, 30.0, 60.0, 90.0, 120.0}, {1.0, 0.9, 0.6, 0.2, 0.01});
const PiecewiseLinearFunction<double, double> kLonDistLonSpeedBufferPlf(
    {0.0, 10.0, 20.0, 30.0}, {6.0, 4.5, 3.0, 1.5});
const PiecewiseLinearFunction<double, double> kLatSpeedLatDistPlf(
    {0.2, 0.4, 0.6, 0.8, 1.0}, {1.2, 1.5, 1.8, 2.1, 2.4});

std::vector<std::string> GetPotentialObjectsId(
    double plan_start_v,
    absl::Span<const SpacetimeObjectTrajectory* const> moving_object_trajs,
    const std::map<std::string, ObjectSlInfo>& obj_sl_map,
    const CipvObjectInfo& cipv_info,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs) {
  bool ignore_cipv = true;
  std::vector<std::string> target_object_ids;
  double cipv_s = std::numeric_limits<double>::max();
  const std::string cipv_id = cipv_info.nearest_object_id.value_or(" ");
  if (cipv_info.nearest_object_id.has_value()) {
    cipv_s = FindOrDie(cipv_info.object_distance_map, cipv_id);
  }
  for (const SpacetimeObjectTrajectory* traj : moving_object_trajs) {
    const auto object_id = std::string(traj->object_id());
    if (ContainsKey(leading_trajs, object_id)) {
      continue;
    }
    if (ignore_cipv && cipv_id == object_id) {
      continue;
    }
    constexpr double kMinCipvLonDist = 2.0;  // m.
    constexpr double kMaxBackLonDist = -2.0;  // m.
    const ObjectSlInfo* sl_info = FindOrNull(obj_sl_map, object_id);
    if (sl_info == nullptr) continue;
    const double speed_buffer = kLonDistLonSpeedBufferPlf(sl_info->ds);
    if (sl_info->vs > plan_start_v + speed_buffer ||
        sl_info->vs <= 0.0 || sl_info->ds > cipv_s - kMinCipvLonDist) {
      continue;
    }
    // TODO: Max considered distance related to av speed.
    constexpr double kMaxConsiderLonDist = 60.0;  // m.
    // TODO: Use current lane width.
    constexpr double kMaxConsiderLatDist = 3.0;  // m.
    if (sl_info->ds > kMaxConsiderLonDist || sl_info->ds < kMaxBackLonDist ||
        std::fabs(sl_info->dl) > kMaxConsiderLatDist) {
      continue;
    }
    target_object_ids.push_back(object_id);
  }
  return target_object_ids;
}

std::optional<std::pair<double, double>> ComputeNearestBoundaryDistWithLerp(
    const DrivePassage& drive_passage, const Vec2d& point) {
  const auto station_point = drive_passage.QueryFrenetLonOffsetAt(point);
  if (!station_point.ok()) return std::nullopt;
  const auto& station_idx = station_point->station_index;
  const auto station_size = drive_passage.stations().size();
  const auto find_next = station_point->lon_offset >= 0.0;
  std::vector<double> station_s;
  if (find_next) {
    const auto next_station_idx =
        std::min(station_idx.value() + 1, station_size - 1);
    const auto& next_station =
        drive_passage.station(StationIndex(next_station_idx));
    station_s = {drive_passage.station(station_idx).accumulated_s(),
                 next_station.accumulated_s()};
  } else {
    const auto prev_station_idx = std::max<int>(0, station_idx.value() - 1);
    const auto& prev_station =
        drive_passage.station(StationIndex(prev_station_idx));
    station_s = {prev_station.accumulated_s(),
                 drive_passage.station(station_idx).accumulated_s()};
  }
  const auto first_bound_l_or =
      drive_passage.QueryNearestBoundaryLateralOffset(station_s[0]);
  const auto second_bound_l_or =
      drive_passage.QueryNearestBoundaryLateralOffset(station_s[1]);
  if (!first_bound_l_or.ok() || !second_bound_l_or.ok()) {
    return std::nullopt;
  }
  const double s_diff = station_s[1] - station_s[0];
  if (s_diff < kEps) return *first_bound_l_or;
  const double fraction = find_next
                              ? station_point->lon_offset / s_diff
                              : (s_diff + station_point->lon_offset) / s_diff;
  return std::make_pair(
      Lerp(first_bound_l_or->first, second_bound_l_or->first, fraction),
      Lerp(first_bound_l_or->second, second_bound_l_or->second, fraction));
}

bool IsObjectOnEgoLaneBoundary(const DrivePassage& drive_passage,
                               const FrenetBox& obj_frenet_box,
                               const FrenetCoordinate& obj_sl, const Box2d& box,
                               const std::string& debug_id,
                               double* invasion_dist) {
  CHECK_NOTNULL(invasion_dist);
  const auto center_bound =
      drive_passage.QueryNearestBoundaryLateralOffset(obj_sl.s);
  if (!center_bound.ok()) return false;

  const bool from_right =
      std::fabs(obj_frenet_box.l_max) < std::fabs(obj_frenet_box.l_min);
  std::vector<Vec2d> side_points;
  if (from_right) {
    side_points = {box.GetCorner(Box2d::Corner::FRONT_LEFT),
                   box.GetCorner(Box2d::Corner::REAR_LEFT)};
  } else {
    side_points = {box.GetCorner(Box2d::Corner::FRONT_RIGHT),
                   box.GetCorner(Box2d::Corner::REAR_RIGHT)};
  }
  bool is_on_boundary = false;
  double max_lat_invasion = std::numeric_limits<double>::lowest();
  for (int i = 0; i < side_points.size(); ++i) {
    const auto& side_pt = side_points[i];
    const auto side_pt_sl =
        drive_passage.QueryUnboundedFrenetCoordinateAt(side_pt);
    if (!side_pt_sl.ok()) continue;
    const auto bound_l =
        ComputeNearestBoundaryDistWithLerp(drive_passage, side_pt);
    if (!bound_l.has_value()) continue;
    const auto reference_l = from_right ? bound_l->first : bound_l->second;
    const double relative_l =
        from_right ? side_pt_sl->l - reference_l : reference_l - side_pt_sl->l;
    constexpr double kLateralInvasionEpsilon = 0.01;  // m.
    const bool pt_invasion = from_right
                                 ? relative_l >= kLateralInvasionEpsilon &&
                                       obj_sl.l < center_bound->first
                                 : relative_l >= kLateralInvasionEpsilon &&
                                       obj_sl.l > center_bound->second;
    is_on_boundary = is_on_boundary || pt_invasion;
    max_lat_invasion = std::max<double>(max_lat_invasion, relative_l);
  }
  if (invasion_dist != nullptr) {
    *invasion_dist = max_lat_invasion;
  }
  return is_on_boundary;
}

absl::StatusOr<ObjectDerivedInfo> ComputeObjectDerivedInfo(
    const PathPoint& av_point, const PlannerObject& planner_object,
    const prediction::ObjectMotionHistory& motion_history,
    const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::string& debug_id) {
  ObjectDerivedInfo object_derived_info;
  const SecondOrderTrajectoryPoint& curr_pose = planner_object.pose();
  const auto& curr_bbox = planner_object.bounding_box();
  object_derived_info.object_length = curr_bbox.length();
  object_derived_info.object_width = curr_bbox.width();

  const auto sl =
      drive_passage.QueryUnboundedFrenetCoordinateAt(curr_pose.pos());
  if (!sl.ok()) {
    return absl::FailedPreconditionError(
        "Query object frenet coordinate failed.");
  }
  object_derived_info.sl_pos = *sl;

  constexpr double kHistoryTimeRange = 1.0;
  const auto states = motion_history.GetStatesInRange(kHistoryTimeRange);
  object_derived_info.lat_fit_speed =
      FitLateralSpeedByMotionHistory(states, drive_passage, kMinFitNum);
  object_derived_info.lon_fit_accel =
      FitLonAccelByMotionHistory(states, drive_passage, kMinFitNum);

  const auto dp_angle = drive_passage.QueryTangentAngleAtS(sl->s);
  if (!dp_angle.ok()) {
    return absl::FailedPreconditionError("Query object tangent angle failed.");
  }

  const auto dp_vec = Vec2d::FastUnitFromAngle(*dp_angle);
  object_derived_info.angle_diff_with_dp =
      NormalizeAngle(*dp_angle - curr_pose.theta());
  object_derived_info.lon_speed = dp_vec.Dot(planner_object.velocity());

  const auto frenet_box = drive_passage.QueryFrenetBoxAt(curr_bbox);
  if (!frenet_box.ok()) {
    return absl::FailedPreconditionError("Query object frenet_box failed.");
  }
  object_derived_info.frenet_box = *frenet_box;

  const auto bounds_or = drive_passage.QueryNearestBoundaryLateralOffset(sl->s);
  if (!bounds_or.ok()) {
    return absl::FailedPreconditionError(
        "Query nearest boundary lateral offset failed.");
  }
  object_derived_info.left_bound_l = bounds_or->second;
  object_derived_info.right_bound_l = bounds_or->first;

  const auto av_rac_cur_pos = ToVec2d(av_point);
  const auto av_heading = av_point.theta();
  const auto av_front_center =
      ComputeAvFrontCenter(av_rac_cur_pos, av_heading, vehicle_geometry_params);
  const auto av_front_sl =
      drive_passage.QueryUnboundedFrenetCoordinateAt(av_front_center);
  if (!av_front_sl.ok()) {
    return absl::FailedPreconditionError(
        "Query av_front_sl frenet coordinate failed.");
  }
  const double relative_to_av_s = frenet_box->s_max - (av_front_sl->s);
  object_derived_info.relative_s_to_av = relative_to_av_s;
  object_derived_info.av_sl_pos = *av_front_sl;

  const auto av_box = ComputeAvBox(ToVec2d(av_point), av_point.theta(),
                                   vehicle_geometry_params);
  const auto av_frenet_box = drive_passage.QueryFrenetBoxAt(av_box);
  if (!av_frenet_box.ok()) {
    return absl::FailedPreconditionError("Query av frenet box failed.");
  }
  const bool from_right =
      std::fabs(frenet_box->l_max) < std::fabs(frenet_box->l_min);
  object_derived_info.relative_l_to_av =
      from_right ? av_frenet_box->l_min - frenet_box->l_max
                 : frenet_box->l_min - av_frenet_box->l_max;

  double invasion_dist = 0.0;
  object_derived_info.is_on_av_boundary = IsObjectOnEgoLaneBoundary(
      drive_passage, object_derived_info.frenet_box, object_derived_info.sl_pos,
      curr_bbox, debug_id, &invasion_dist);
  object_derived_info.invasion_dist = invasion_dist;

  return object_derived_info;
}

inline bool IsAvCanPassQuickly(double ego_v, double obj_v,
                               double relative_to_av_s, double s_buffer) {
  constexpr double kAvPassMinSpeedThres = 0.0;  // m/s.
  if (ego_v < kAvPassMinSpeedThres) return false;
  // if (obj_v > ego_v) return false;
  return relative_to_av_s < s_buffer;
}

inline bool IsInFrontOfAv(double rel_l) {
  constexpr double kInvasionBuffer = 0.5;  // m.
  return rel_l < -kInvasionBuffer;
}

inline bool IsDepartingAvLane(double lat_speed, double obj_l) {
  constexpr double kLatMinSpeed = 0.1;  // m/s.
  return std::abs(lat_speed) > kLatMinSpeed && lat_speed * obj_l > 0.0;
}

inline double ComputePreviewRelativeDist(double rel_s, double obj_v,
                                         double obj_a, double ego_v,
                                         double ego_a, double preview_time) {
  obj_v = std::max(obj_v, 0.0);
  ego_v = std::max(ego_v, 0.0);
  const double obj_t = obj_a >= 0.0
                           ? preview_time
                           : std::min(preview_time, std::fabs(obj_v / obj_a));
  const double ego_t = ego_a >= 0.0
                           ? preview_time
                           : std::min(preview_time, std::fabs(ego_v / ego_a));
  return rel_s + obj_v * obj_t + 0.5 * obj_a * Sqr(obj_t) -
         (ego_v * ego_t + 0.5 * ego_a * Sqr(ego_t));
}

double ComputeLateralTtc(const ObjectDerivedInfo& obj_derived_info) {
  constexpr double kLateralSafetyBuffer = 0.2;  // m.
  const double lat_buffer =
      obj_derived_info.relative_s_to_av < 0.0
          ? std::min(kLateralSafetyBuffer, obj_derived_info.relative_l_to_av)
          : kLateralSafetyBuffer;
  const double rel_l_with_buffer =
      std::max(0.0, obj_derived_info.relative_l_to_av - lat_buffer);

  const double lat_vel = std::abs(obj_derived_info.lat_fit_speed);
  if (lat_vel <= 0.01) {
    return std::numeric_limits<double>::infinity();
  }
  return rel_l_with_buffer / lat_vel;
}

bool MaybePassDuringLatTtc(const ObjectDerivedInfo& obj_derived_info,
                           double av_speed, double lat_ttc,
                           bool is_large_vehicle, double ego_length) {
  constexpr double kExceedLengthBuffer = 10.0;
  // Object to av distance.
  double rel_dist_with_lat_ttc = ComputePreviewRelativeDist(
      obj_derived_info.relative_s_to_av, obj_derived_info.lon_speed,
      /*obj_a=*/0.0, av_speed, /*ego_a=*/0.0, lat_ttc);
  double min_overlap_length =
      is_large_vehicle ? obj_derived_info.object_length
                       : std::min(obj_derived_info.object_length, ego_length);

  const auto rel_speed = Mps2Kph(av_speed - obj_derived_info.lon_speed);
  min_overlap_length *= kRelSpeedPassFactorPlf(rel_speed);
  return rel_dist_with_lat_ttc < -min_overlap_length - kExceedLengthBuffer;
}

double ComputeTtcWithAccleration(double ds, double obj_v, double obj_a,
                                 double ego_v, double ego_a,
                                 double safety_lon_buffer) {
  CHECK_GE(ego_v, 0.0);
  CHECK_GE(obj_v, 0.0);
  CHECK_GE(ds, 0.0);
  CHECK_GE(safety_lon_buffer, 0.0);

  if (ego_v < kEps && ego_a <= 0.0) return kInfinity;
  if (obj_v < kEps && obj_a < 0.0) {
    obj_a = 0.0;
  }
  const double rel_v = ego_v - obj_v;
  const double rel_d = ds - safety_lon_buffer;
  const double rel_a = ego_a - obj_a;
  if (rel_a <= kEps && rel_v <= 0.0) return kInfinity;
  if (rel_d < 0.0) return 0.0;

  if (obj_a < 0.0) {
    const double obj_stop_t = -obj_v / obj_a;
    const double obj_stop_s = -obj_v * obj_v / (2.0 * obj_a);
    if (obj_v > kEps && ego_a > ego_v * obj_a / obj_v) {
      if (ego_v * obj_stop_t + 0.5 * ego_a * obj_stop_t * obj_stop_t <
          obj_stop_s + safety_lon_buffer) {
        if (ego_a < kEps) {
          return ego_v > kEps ? (rel_d + obj_stop_s) / ego_v : kInfinity;
        }
        const double safe_rel_d =
            ego_v * ego_v + 2.0 * ego_a * (rel_d + obj_stop_s);
        if (safe_rel_d <= 0.0) return kInfinity;
        return (-ego_v + std::sqrt(safe_rel_d)) / ego_a;
      }
    } else if (rel_v < 0.0) {
      const double ego_stop_t = -ego_v / ego_a;
      const double ego_stop_s = -ego_v * ego_v / (2.0 * ego_a);
      if (ego_stop_s <
          obj_v * ego_stop_t + 0.5 * obj_a * ego_stop_t * ego_stop_t + rel_d) {
        return kInfinity;
      }
    }
  }
  if (rel_a < kEps) return rel_d / rel_v;
  const double d_safe_rel = rel_v * rel_v + 2.0 * rel_a * rel_d;
  if (d_safe_rel <= 0.0) return kInfinity;
  return (-rel_v + std::sqrt(d_safe_rel)) / rel_a;
}

std::optional<std::string> ComputeDefensiveSpeedLimitDecel(
    int plan_id, double av_speed, double av_accel,
    const std::vector<std::string>& potential_braking_objects,
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& traj_mgr,
    const DrivePassage& drive_passage,
    const ObjectHistoryManager& object_history_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto debug_key = absl::StrCat(Log2DDS::TaskPrefix(plan_id), "shiping");
  for (const auto& obj_id : potential_braking_objects) {
    const ObjectHistory* object_history =
        object_history_mgr.GetObjHistory(obj_id);
    if (object_history == nullptr) continue;
    const auto obj_motion_history = ConvertToMotionHistory(*object_history);
    if (!obj_motion_history.has_value()) continue;
    const PlannerObject* planner_object = traj_mgr.FindObjectByObjectId(obj_id);
    if (planner_object == nullptr) continue;
    const auto object_derived_info = ComputeObjectDerivedInfo(
        path.front(), *planner_object, *obj_motion_history, drive_passage,
        vehicle_geometry_params, obj_id);
    if (!object_derived_info.ok()) {
      Log2DDS::LogDataV2(
          debug_key, absl::StrFormat("id %s %s", obj_id,
                                     object_derived_info.status().message()));
      continue;
    }

    constexpr double kAvHeadExceedBuffer = -1.5;  // m.
    if (IsAvCanPassQuickly(av_speed, object_derived_info->lon_speed,
                           object_derived_info->relative_s_to_av,
                           kAvHeadExceedBuffer)) {
      Log2DDS::LogDataV2(
          debug_key,
          absl::StrFormat("id %s can be quickly passed by av.", obj_id));
      continue;
    }
    if (IsInFrontOfAv(object_derived_info->relative_l_to_av)) {
      Log2DDS::LogDataV2(
          debug_key,
          absl::StrFormat("id %s is already in front of the vehicle.", obj_id));
      continue;
    }
    if (IsDepartingAvLane(object_derived_info->lat_fit_speed,
                          object_derived_info->sl_pos.l)) {
      Log2DDS::LogDataV2(
          debug_key, absl::StrFormat("id %s is departing av lane.", obj_id));
      continue;
    }

    const double lat_ttc = ComputeLateralTtc(*object_derived_info);
    const double lon_ttc = ComputeTtcWithAccleration(
        std::max(object_derived_info->relative_s_to_av, 0.0),
        std::max(object_derived_info->lon_speed, 0.0),
        /*obj_a=*/0.0, std::max(av_speed, 0.0), av_accel,
        /*safety_lon_buffer=*/0.0);
    const double lat_ttc_thres =
        kLonDistLateralTtcPlf(object_derived_info->relative_s_to_av);
    if (lat_ttc > lat_ttc_thres) {
      Log2DDS::LogDataV2(
          debug_key,
          absl::StrFormat("id %s lat_ttc %.3f is exceed threshold %.3f.",
                          obj_id, lat_ttc, lat_ttc_thres));
      continue;
    }

    if (MaybePassDuringLatTtc(*object_derived_info, av_speed, lat_ttc,
                              planner_object->is_large_vehicle(),
                              vehicle_geometry_params.length())) {
      Log2DDS::LogDataV2(
          debug_key,
          absl::StrFormat("id %s can be passed during lat_ttc.", obj_id));
      continue;
    }

    // TODO: Use the lateral and longitudinal ttc to avoid unnecessary
    // pre-braking.
    // TODO: Consider definite cut-in and possible cut-in and apply different
    // braking decel.
    constexpr double kDefensiveEnterLatSpeed = 0.2;  // m/s.
    constexpr double kDefensiveLatCheckTime = 0.7;   // s.
    constexpr double kDefensiveEnterMinL = 0.5;      // m.
    constexpr double kLonTtcTriggerThres = 6.0;      // s.
    constexpr double kLonTtcConsiderDist = 10.0;     // s.
    if (object_derived_info->relative_s_to_av >
            object_derived_info->object_length + kLonTtcConsiderDist &&
        lon_ttc > kLonTtcTriggerThres) {
      Log2DDS::LogDataV2(
          debug_key,
          absl::StrFormat("id %s lon_ttc %.3f is too large", obj_id, lon_ttc));
      continue;
    }
    const bool is_lat_closing =
        std::abs(object_derived_info->lat_fit_speed) >
            kDefensiveEnterLatSpeed &&
        object_derived_info->relative_l_to_av <
            kLatSpeedLatDistPlf(
                std::abs(object_derived_info->lat_fit_speed));
    const bool is_lat_too_close =
        std::max(object_derived_info->relative_l_to_av -
                     std::abs(object_derived_info->lat_fit_speed) *
                         kDefensiveLatCheckTime,
                 0.0) < kDefensiveEnterMinL;

    if (!is_lat_closing && !is_lat_too_close) {
      Log2DDS::LogDataV2(
          debug_key, absl::StrFormat("id %s is not satisfying trigger "
                                     "conditions: lat_speed: %.3f rel_l: %.3f",
                                     obj_id, object_derived_info->lat_fit_speed,
                                     object_derived_info->relative_l_to_av));
      continue;
    }
    Log2DDS::LogDataV2(
        debug_key,
        absl::StrFormat("id %s success to trigger, lat_ttc: %3f, lon_ttc: "
                        "%.3f, lat_speed: %.3f, rel_s: %.3f, rel_l: %.3f",
                        obj_id, lat_ttc, lon_ttc,
                        object_derived_info->lat_fit_speed,
                        object_derived_info->relative_s_to_av,
                        object_derived_info->relative_l_to_av));
    return obj_id;
  }
  return std::nullopt;
}

}  // namespace

std::optional<VtSpeedLimit> DecideDefensiveSpeedLimit(
    const DefensiveSpeedDeciderInput& input,
    DefensiveSpeedProto* defensive_speed_state) {
  CHECK_NOTNULL(input.obj_sl_map);
  CHECK_NOTNULL(input.path);
  CHECK_NOTNULL(input.traj_mgr);
  CHECK_NOTNULL(input.drive_passage);
  CHECK_NOTNULL(input.vehicle_geometry_params);
  CHECK_NOTNULL(input.cipv_info);
  CHECK_NOTNULL(defensive_speed_state);
  if (input.plan_id != 0 || input.lc_stage == LaneChangeStage::LCS_EXECUTING ||
      input.obj_history_mgr == nullptr) {
    defensive_speed_state->clear_prev_decel();
    return std::nullopt;
  }
  const auto target_obj_ids = GetPotentialObjectsId(
      input.plan_start_v, input.traj_mgr->moving_object_trajs(),
      *input.obj_sl_map, *input.cipv_info, *input.leading_trajs);
  const auto debug_key =
      absl::StrCat(Log2DDS::TaskPrefix(input.plan_id), "shiping");
  if (!target_obj_ids.empty()) {
    const std::string target_ids = absl::StrJoin(target_obj_ids, ", ");
    Log2DDS::LogDataV2(
        debug_key,
        absl::StrFormat("defensive_candidate_objects: %s", target_ids));
    //    LOG(INFO) << "[shiping] target_ids: " << target_ids;
  }
  const auto defensive_target_id = ComputeDefensiveSpeedLimitDecel(
      input.plan_id, input.plan_start_v, input.plan_start_a, target_obj_ids,
      *input.path, *input.traj_mgr, *input.drive_passage,
      *input.obj_history_mgr, *input.vehicle_geometry_params);

  constexpr double kDefensiveDecelDuration = 5.0;         // s.
  constexpr double kDefensiveMaxVelDiff = Kph2Mps(20.0);  // m/s.
  const double min_ego_v = input.speed_finder_params->defensive_min_ego_v();
  const double max_decel = input.speed_finder_params->defensive_max_decel();
  const double decel_incr = input.speed_finder_params->defensive_decel_incr();
  const double defensive_dec_before_acc =
      input.speed_finder_params->defensive_dec_before_acc();

  std::optional<double> defensive_decel;
  if (defensive_target_id.has_value() &&
      input.plan_start_v > min_ego_v) {
    if (defensive_speed_state->has_prev_decel()) {
      defensive_decel = defensive_speed_state->prev_decel() > 0.0
          ? defensive_speed_state->prev_decel() + defensive_dec_before_acc
          : defensive_speed_state->prev_decel() + decel_incr;
    } else {
      defensive_decel = input.plan_start_a > 0.0
          ? input.plan_start_a + defensive_dec_before_acc
          : input.plan_start_a + decel_incr;
    }
  } else if (defensive_speed_state->has_prev_decel() &&
             defensive_speed_state->prev_decel() - decel_incr < 0.0) {
    defensive_decel = defensive_speed_state->prev_decel() - decel_incr;
  }
  if (!defensive_decel.has_value()) {
    defensive_speed_state->clear_prev_decel();
    return std::nullopt;
  } else {
    defensive_decel = std::max(*defensive_decel, max_decel);
  }
  defensive_speed_state->set_prev_decel(*defensive_decel);
  const auto info =
      absl::StrCat("Defensive brake for object ", *defensive_target_id);
  Log2DDS::LogDataV2(debug_key,
                     absl::StrFormat("defensive_trigger_id: %s decel: %.3f",
                                     defensive_target_id.value_or("No object"),
                                     *defensive_decel));
  Log2DDS::LogDataV2(
      absl::StrCat(Log2DDS::TaskPrefix(0), "defensive_speed_limit"),
      absl::StrFormat("Defensive brake for object id is: %s, cur dec is: %.3f",
                      defensive_target_id.value_or("No object"),
                      *defensive_decel));
  return GenerateConstAccSpeedLimit(
      /*start_t=*/0.0, kDefensiveDecelDuration, input.plan_start_v,
      std::max(input.plan_start_v - kDefensiveMaxVelDiff, 0.0),
      input.max_ego_v, *defensive_decel, input.time_step, input.step_num,
      info);
}

}  // namespace st::planning
