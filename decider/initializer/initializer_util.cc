

#include "decider/initializer/initializer_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "plan_common/log.h"

#include "decider/initializer/cost_provider.h"
#include "decider/initializer/geometry/geometry_form.h"
#include "decider/initializer/geometry/geometry_state.h"
#include "decider/initializer/motion_graph.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/trajectory_util.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/log_data.h"
#include "plan_common/math/quintic_polynomial_curve1d.h"

namespace st::planning {

namespace {

double FindRequiredLonDecel(
    double ego_front_to_ra, double ego_s, double ego_v,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    int traj_steps) {
  ego_s += ego_front_to_ra;
  const double traj_time = traj_steps * kTrajectoryTimeStep;
  double decel = 0.0;
  if (!stop_lines.empty()) {
    const double min_stop_s = stop_lines.front().s();
    decel = 2.0 * (min_stop_s - ego_s - ego_v * traj_time) / Sqr(traj_time);
  }
  constexpr double kLeadingLonBuffer = 0.3;  // m.
  ego_s += kLeadingLonBuffer;
  for (const auto& [traj_id, leading_traj] : leading_trajs) {
    for (const auto& st_constraint : leading_traj.st_constraints()) {
      decel = std::min(
          decel, 2.0 * (st_constraint.s() - ego_s - ego_v * st_constraint.t()) /
                     Sqr(st_constraint.t() - kTrajectoryTimeStep));
    }
  }
  return decel;
}

ConstraintProto::LeadingObjectProto CreateLeadingObject(
    const SpacetimeObjectTrajectory& traj, const DrivePassage& passage) {
  constexpr double kEpsilon = 1e-3;
  ConstraintProto::LeadingObjectProto leading_object;
  leading_object.set_traj_id(std::string(traj.traj_id()));

  constexpr double kSampleTimeInterval = 1.0;  // Seconds.
  // Generate ST-constraints based on object current bounding box, for
  // stationary object.
  if (traj.is_stationary()) {
    ASSIGN_OR_RETURN(const auto obj_frenet_box,
                     passage.QueryFrenetBoxAt(traj.bounding_box()),
                     leading_object);
    for (double sample_time = 0.0; sample_time <= kTrajectoryTimeHorizon;
         sample_time += kSampleTimeInterval) {
      auto* constraint = leading_object.add_st_constraints();
      constraint->set_s(obj_frenet_box.s_min);
      constraint->set_t(sample_time);
    }
    if (kTrajectoryTimeHorizon - leading_object.st_constraints().rbegin()->t() >
        kEpsilon) {
      auto* constraint = leading_object.add_st_constraints();
      constraint->set_s(obj_frenet_box.s_min);
      constraint->set_t(kTrajectoryTimeHorizon);
    }

    return leading_object;
  }

  double sample_time = 0.0;
  const double traj_last_time = traj.states().back().traj_point->t();
  // Generate ST-constraints based on spacetime states, for moving object.
  for (const auto& state : traj.states()) {
    // Sample ST-constraints by 1.0s.
    const auto* traj_point = state.traj_point;
    const double t = traj_point->t();
    // Generate ST-constraints at sample time and trajectory last time.
    if (std::abs(t - sample_time) > kEpsilon &&
        std::abs(traj_last_time - t) > kEpsilon) {
      continue;
    }
    // Filter object state out of passage.
    ASSIGN_OR_CONTINUE(const auto obj_frenet_box,
                       passage.QueryFrenetBoxAt(state.box));
    // Filter object state by drive passage direction.
    ASSIGN_OR_CONTINUE(const auto passage_tangent,
                       passage.QueryTangentAngleAtS(obj_frenet_box.s_min));
    const double angle_diff =
        std::abs(NormalizeAngle(passage_tangent - traj_point->theta()));
    if (angle_diff > M_PI_2) continue;

    auto* constraint = leading_object.add_st_constraints();
    constraint->set_s(obj_frenet_box.s_min);
    constraint->set_t(t);

    sample_time += kSampleTimeInterval;
  }

  return leading_object;
}

}  // namespace

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GenerateConstLateralAccelConstSpeedTraj(
    const DrivePassage& drive_passage, double ego_front_to_ra, double target_l,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines,
    const ApolloTrajectoryPointProto& plan_start_point, int traj_steps) {
  CHECK_GT(traj_steps, 0);
  constexpr double kStationarySpeedThres = 0.1;  // m/s.
  if (plan_start_point.v() < kStationarySpeedThres) {
    std::vector<ApolloTrajectoryPointProto> stationary_traj(traj_steps,
                                                            plan_start_point);
    for (int i = 1; i < traj_steps; ++i) {
      stationary_traj[i].set_relative_time(i * kTrajectoryTimeStep);
    }
    return stationary_traj;
  }

  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  ASSIGN_OR_RETURN(const auto ego_sl,
                   drive_passage.QueryFrenetCoordinateAt(ego_pos),
                   _ << "Unable to project ego position onto drive passage.");
  ASSIGN_OR_RETURN(const auto lane_tangent,
                   drive_passage.QueryTangentAtS(ego_sl.s),
                   _ << "Unable to find lane tangent at ego position.");

  const double required_lon_decel =
      FindRequiredLonDecel(ego_front_to_ra, ego_sl.s, plan_start_point.v(),
                           stop_lines, leading_trajs, traj_steps);

  constexpr double kMaxLaneChangeCancelLatAccel = 0.5;   // m/s^2.
  constexpr double kZeroLateralVelThreshold = 0.01;      // m/s.
  constexpr double kZeroLongitudinalVelThreshold = 1.0;  // m/s.
  constexpr double kDt = kTrajectoryTimeStep;

  const auto heading_tangent =
      Vec2d::FastUnitFromAngle(plan_start_point.path_point().theta());
  double lon_v = plan_start_point.v() * lane_tangent.Dot(heading_tangent);
  double lat_v = plan_start_point.v() * lane_tangent.CrossProd(heading_tangent);
  lat_v = std::fabs(lat_v) < kZeroLateralVelThreshold ? 0.0 : lat_v;
  const double lon_a =
      std::min(plan_start_point.a() * lane_tangent.Dot(heading_tangent),
               required_lon_decel);
  double lat_a =
      plan_start_point.a() * lane_tangent.CrossProd(heading_tangent) +
      Sqr(plan_start_point.v()) * lane_tangent.Dot(heading_tangent) *
          plan_start_point.path_point().kappa();

  std::vector<FrenetCoordinate> traj_sl;
  traj_sl.reserve(traj_steps);
  traj_sl.push_back(ego_sl);
  double s = ego_sl.s, l = ego_sl.l;
  // Assume const lateral jerk until lateral acceleration reaches expected const
  // accel value.
  constexpr double kMaxComfortLatJerk = 1.0;  // m/s^3.
  const double expect_lat_a =
      std::copysign(kMaxLaneChangeCancelLatAccel, lat_v);
  const int const_lat_jerk_steps =
      CeilToInt(std::abs(expect_lat_a - lat_a) / (kMaxComfortLatJerk * kDt));
  if (std::abs(expect_lat_a) < std::abs(lat_a)) {
    for (int i = 1; i < const_lat_jerk_steps + 1; ++i) {
      s += lon_v * kDt;
      l += lat_v * kDt;
      traj_sl.push_back(FrenetCoordinate{.s = s, .l = l});

      lon_v = std::max(0.0, lon_v + lon_a * kDt);
      lat_v += lat_a * kDt;
      lat_a += std::copysign(
          std::min(kMaxComfortLatJerk * kDt, std::abs(expect_lat_a - lat_a)),
          expect_lat_a - lat_a);
      if (std::abs(lon_v) < kZeroLongitudinalVelThreshold) break;
    }
  }
  // Assume const lateral acceleration for the rest of trajectory.
  constexpr double kReachTargetLaneThreshold = 0.1;  // m.
  constexpr double kMaxLaneChangeLatAccel = 0.09;    // m/s^2.
  const double l_diff = l - target_l;
  constexpr double kNearTargetLaneThreshold = 0.3;  // m.
  lat_a = std::abs(l_diff) < kNearTargetLaneThreshold
              ? kMaxLaneChangeCancelLatAccel
              : (lat_v * l_diff < 0.0 ? kMaxLaneChangeLatAccel
                                      : kMaxLaneChangeCancelLatAccel);
  lat_a = std::abs(l_diff) > 1e-5 ? std::copysign(lat_a, -l_diff)
                                  : std::copysign(lat_a, -lat_v);
  for (int i = traj_sl.size(); i < traj_steps; ++i) {
    if (std::abs(lon_v) < kZeroLongitudinalVelThreshold) break;
    const double new_l = l + lat_v * kDt;
    if ((std::fabs(new_l - target_l) < kReachTargetLaneThreshold &&
         std::fabs(lat_v) < kZeroLateralVelThreshold) ||
        (new_l - target_l) * (l - target_l) < 0.0) {
      break;
    }

    s += lon_v * kDt;
    l += lat_v * kDt;
    traj_sl.push_back(FrenetCoordinate{.s = s, .l = l});

    lon_v = std::max(0.0, lon_v + lon_a * kDt);
    lat_v += lat_a * kDt;
  }
  // Fill the rest trajectory points.
  for (int i = traj_sl.size(); i < traj_steps; ++i) {
    s += lon_v * kDt;
    lon_v = std::max(0.0, lon_v + lon_a * kDt);
    traj_sl.push_back(FrenetCoordinate{.s = s, .l = l});
  }
  const double max_accum_s = drive_passage.end_s();
  if (traj_sl.back().s > max_accum_s) {
    const double ratio = max_accum_s / traj_sl.back().s;
    for (auto& pt_sl : traj_sl) pt_sl.s *= ratio;
  }

  std::vector<Vec2d> traj_xy;
  traj_xy.reserve(traj_sl.size());
  for (const auto& pt_sl : traj_sl) {
    // Should not exceed drive passage since already post-processed.
    ASSIGN_OR_RETURN(const auto pt_xy,
                     drive_passage.QueryPointXYAtSL(pt_sl.s, pt_sl.l),
                     _ << "Lc pause trajectory out of drive passage.");
    traj_xy.push_back(pt_xy);
  }
  std::vector<double> traj_v(traj_steps), traj_a(traj_steps);
  for (int i = 1; i + 1 < traj_steps; ++i) {
    traj_v[i] = traj_xy[i + 1].DistanceTo(traj_xy[i]) / kDt;
  }
  traj_v[traj_steps - 1] = traj_v[traj_steps - 2];
  for (int i = 1; i + 1 < traj_steps; ++i) {
    traj_a[i] = (traj_v[i + 1] - traj_v[i]) / kDt;
  }
  traj_a[traj_steps - 1] = traj_a[traj_steps - 2];

  constexpr double kEpsilon = 1e-3;
  // Transform into trajectory proto.
  std::vector<ApolloTrajectoryPointProto> traj_pts;
  traj_pts.reserve(traj_steps);
  traj_pts.push_back(plan_start_point);
  for (int i = 1; i < traj_steps; ++i) {
    PathPoint path_pt;
    path_pt.set_x(traj_xy[i].x());
    path_pt.set_y(traj_xy[i].y());
    path_pt.set_z(0.0);
    const double dist2prev = traj_xy[i].DistanceTo(traj_xy[i - 1]);
    const double cur_theta =
        i + 1 == traj_steps || traj_xy[i + 1].DistanceTo(traj_xy[i]) < kEpsilon
            ? traj_pts[i - 1].path_point().theta()
            : (traj_xy[i + 1] - traj_xy[i]).FastAngle();
    path_pt.set_theta(cur_theta);
    path_pt.set_kappa(
        dist2prev < kEpsilon
            ? 0.0
            : NormalizeAngle(cur_theta - traj_pts[i - 1].path_point().theta()) /
                  dist2prev);
    path_pt.set_s(traj_pts[i - 1].path_point().s() + dist2prev);

    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = std::move(path_pt);
    traj_point.set_v(traj_v[i]);
    traj_point.set_a(traj_a[i]);
    traj_point.set_relative_time(i * kTrajectoryTimeStep);

    traj_pts.push_back(std::move(traj_point));
  }

  return traj_pts;
}

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GenerateLatQuinticLonConstAccTrajToRefL(
    const DrivePassage& drive_passage, const PathSlBoundary& sl_boundary,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::string>& leading_trajs,
    const ApolloTrajectoryPointProto& plan_start_point,
    const double ego_front_to_center,
    const std::optional<double> nearest_stop_s, const int plan_id,
    const std::string fake_reason) {
  std::vector<std::string> debug_str_vec;
  debug_str_vec.emplace_back(fake_reason);

  constexpr double dt = kTrajectoryTimeStep;
  constexpr int traj_steps = kInitializerTrajectorySteps;
  constexpr double traj_time = traj_steps * dt;

  CHECK_GT(dt, 0.0);
  CHECK_GT(traj_steps, 0);
  CHECK_GT(traj_time, 0.0);

  // Get ego current info
  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  ASSIGN_OR_RETURN(const auto ego_sl,
                   drive_passage.QueryFrenetCoordinateAt(ego_pos),
                   _ << "Unable to project ego position onto drive passage.");
  const double ego_v = plan_start_point.v();
  const double ego_a = plan_start_point.a();
  const auto ego_heading_tangent =
      Vec2d::FastUnitFromAngle(plan_start_point.path_point().theta());

  // Get ref_l
  const double ref_l = sl_boundary.QueryReferenceCenterL(ego_sl.s);

  // Calculate reach_l_time
  constexpr double kReachLTimeMin = 0.05;  // s
  constexpr double kLatSpeedMax = 1.0;     // m/s.
  constexpr double kLatAccMax = 0.6;       // m/s^2.
  const std::vector<double> EGO_V_VEC = {20.0, 50.0, 80.0, 120.0};
  const std::vector<double> TARGET_LAT_SPEED_VEC = {0.4, 1.0, 2.0, 3.0};
  double target_lat_speed = ad_byd::planning::math::interp1_inc(
      EGO_V_VEC, TARGET_LAT_SPEED_VEC, Mps2Kph(ego_v));
  double reach_l_time = std::clamp(
      std::abs(ego_sl.l - ref_l) / target_lat_speed, kReachLTimeMin, traj_time);

  // Calculate lat and lon v、a
  const double preview_s =
      std::min(ego_sl.s + ego_v * reach_l_time, drive_passage.end_s());
  ASSIGN_OR_RETURN(const auto preview_lane_tangent,
                   drive_passage.QueryTangentAtS(preview_s),
                   _ << "Unable to find lane tangent at preview_s.");
  ASSIGN_OR_RETURN(const auto ego_lane_tangent,
                   drive_passage.QueryTangentAtS(ego_sl.s),
                   _ << "Unable to find lane tangent at ego_sl.s.");

  auto lane_tangent = ego_lane_tangent;
  if (std::abs(preview_lane_tangent.CrossProd(ego_heading_tangent)) <
      std::abs(ego_lane_tangent.CrossProd(ego_heading_tangent))) {
    lane_tangent = preview_lane_tangent;
  }

  double ego_lon_v = ego_v * lane_tangent.Dot(ego_heading_tangent);
  double ego_lat_v = ego_v * lane_tangent.CrossProd(ego_heading_tangent);
  double ego_lon_a = ego_a * lane_tangent.Dot(ego_heading_tangent);
  double ego_lat_a = ego_a * lane_tangent.CrossProd(ego_heading_tangent) +
                     Sqr(ego_lon_v) * plan_start_point.path_point().kappa();

  debug_str_vec.emplace_back(
      absl::StrCat(" [ego] v:", ego_v, " a:", ego_a, " l:", ego_sl.l,
                   " lat_v:", ego_lat_v, " lat_a:", ego_lat_a, " s:", ego_sl.s,
                   " lon_v:", ego_lon_v, " lon_a:", ego_lon_a));
  debug_str_vec.emplace_back(absl::StrCat(
      " reach_l_time_ori:", reach_l_time, " ref_l:", ref_l,
      " target_lat_speed:", target_lat_speed, " preview_s:", preview_s));

  ego_lon_v = std::max(ego_lon_v, 0.0);
  ego_lat_v = std::clamp(ego_lat_v, -kLatSpeedMax, kLatSpeedMax);
  ego_lat_a = std::clamp(ego_lat_a, -kLatAccMax, kLatAccMax);

  // Get target lon acc
  constexpr double kLonAccMax = 1.5;          // m/s^2
  constexpr double kLonAccMin = -5.0;         // m/s^2
  constexpr double kRelLonSMin = 3.0;         // m
  constexpr double kLeadingLonBuffer = -2.0;  // m.
  double target_lon_acc = 0.0;
  const auto speed_limit_or = drive_passage.QuerySpeedLimitAtS(ego_sl.s);
  if (speed_limit_or.ok()) {
    const double acc = (speed_limit_or.value() - ego_lon_v) / traj_time;
    target_lon_acc = std::clamp(acc, kLonAccMin, kLonAccMax);
    debug_str_vec.emplace_back(
        absl::StrCat(" speed_limit:", speed_limit_or.value(), " acc:", acc));
  }
  if (nearest_stop_s.has_value()) {
    const double rel_ds = std::max(
        nearest_stop_s.value() - ego_sl.s - ego_front_to_center, kRelLonSMin);
    const double acc = 2.0 * (rel_ds - ego_lon_v * traj_time) / Sqr(traj_time);
    target_lon_acc =
        std::clamp(std::min(target_lon_acc, acc), kLonAccMin, kLonAccMax);
    debug_str_vec.emplace_back(
        absl::StrCat(" nearest_stop_s:", nearest_stop_s.value(), " acc:", acc,
                     " rel_ds:", rel_ds));
  }
  std::stringstream leading_ss;
  if (!leading_trajs.empty()) {
    double acc = kLonAccMax;
    for (const auto& traj_id : leading_trajs) {
      const auto obj_traj = st_traj_mgr.FindTrajectoryById(traj_id);
      if (!obj_traj) continue;
      leading_ss << traj_id << ",";
      const auto leading_traj = CreateLeadingObject(*obj_traj, drive_passage);
      for (const auto& st_constraint : leading_traj.st_constraints()) {
        const double rel_ds = std::max(st_constraint.s() - kLeadingLonBuffer -
                                           ego_sl.s - ego_front_to_center,
                                       kRelLonSMin);
        acc = std::min(acc, 2.0 * (rel_ds - ego_lon_v * st_constraint.t()) /
                                Sqr(st_constraint.t() + 1e-5));
      }
    }
    target_lon_acc =
        std::clamp(std::min(target_lon_acc, acc), kLonAccMin, kLonAccMax);
    debug_str_vec.emplace_back(
        absl::StrCat(" leading_trajs:", leading_ss.str(), " acc:", acc));
  }

  constexpr double kVelAccErrorThres = 0.01;
  constexpr double kLatSpeedMin = 0.05;  // m/s.
  constexpr double kLatSpeedExpandFactor = 1.5;
  constexpr double kLatLBuffer = 0.8;  // m
  double target_l = ref_l;
  double target_dl = 0.0;
  double target_ddl = 0.0;
  const double lon_stop_time = target_lon_acc < -kVelAccErrorThres
                                   ? std::abs(ego_lon_v / target_lon_acc)
                                   : DBL_MAX;
  if (lon_stop_time < reach_l_time) {
    reach_l_time = std::clamp(lon_stop_time - 0.2, kReachLTimeMin, traj_time);
    target_lat_speed *= kLatSpeedExpandFactor;
    target_l =
        ego_sl.l + std::copysign(std::min(target_lat_speed * reach_l_time,
                                          std::abs(ref_l - ego_sl.l)),
                                 ref_l - ego_sl.l);
    target_l = std::clamp(target_l, ref_l - kLatLBuffer, ref_l + kLatLBuffer);
    target_dl = std::copysign(kLatSpeedMin, ref_l - ego_sl.l);
  }

  debug_str_vec.emplace_back(absl::StrCat(
      " reach_l_time:", reach_l_time, " lon_stop_time:", lon_stop_time,
      " target_l:", target_l, " target_lon_acc:", target_lon_acc,
      " target_dl:", target_dl, " target_ddl:", target_ddl));

  Log2DDS::LogDataV2(absl::StrCat(Log2DDS::TaskPrefix(plan_id), "init-fake"),
                     debug_str_vec);

  // Generate quintic polynomial curve 1d for l(t)
  ad_byd::planning::QuinticPolynomialCurve1d quintic_l_t(
      ego_sl.l, ego_lat_v, ego_lat_a, target_l, target_dl, target_ddl,
      reach_l_time);

  // Generate sl points
  std::vector<FrenetCoordinate> traj_sl;
  traj_sl.reserve(traj_steps);
  traj_sl.emplace_back(ego_sl);
  double cur_t = 0.0;
  double s = ego_sl.s, l = ego_sl.l;
  double lon_v = std::max(ego_lon_v, 0.4);
  while (traj_sl.size() < traj_steps) {
    if (lon_v < 0.0 ||
        (lon_v < kVelAccErrorThres && target_lon_acc < kVelAccErrorThres)) {
      break;
    }
    cur_t += dt;
    if (cur_t <= reach_l_time) {
      l = quintic_l_t.Evaluate(0, cur_t);
    }
    s += lon_v * dt;
    lon_v += target_lon_acc * dt;
    traj_sl.push_back({s, l});
  }

  // Fill the rest trajectory points.
  const auto end_sl = traj_sl.back();
  while (traj_sl.size() < traj_steps) {
    traj_sl.emplace_back(end_sl);
  }

  // Check drive_passage length
  if (traj_sl.back().s > drive_passage.end_s()) {
    const double ratio = drive_passage.end_s() / traj_sl.back().s;
    for (auto& pt_sl : traj_sl) pt_sl.s *= ratio;
  }

  // Get xy trajectories
  std::vector<Vec2d> traj_xy;
  traj_xy.reserve(traj_sl.size());
  for (const auto& pt_sl : traj_sl) {
    ASSIGN_OR_RETURN(const auto pt_xy,
                     drive_passage.QueryPointXYAtSL(pt_sl.s, pt_sl.l),
                     _ << "Trajectory out of drive passage.");
    traj_xy.emplace_back(pt_xy);
  }

  std::vector<double> traj_v(traj_steps), traj_a(traj_steps);
  for (int i = 1; i + 1 < traj_steps; ++i) {
    traj_v[i] = traj_xy[i + 1].DistanceTo(traj_xy[i]) / dt;
  }
  traj_v[traj_steps - 1] = traj_v[traj_steps - 2];
  for (int i = 1; i + 1 < traj_steps; ++i) {
    traj_a[i] = (traj_v[i + 1] - traj_v[i]) / dt;
  }
  traj_a[traj_steps - 1] = traj_a[traj_steps - 2];

  // Transform into trajectory proto.
  constexpr double kEpsilon = 1e-3;
  std::vector<ApolloTrajectoryPointProto> traj_pts;
  traj_pts.reserve(traj_steps);
  traj_pts.emplace_back(plan_start_point);
  for (int i = 1; i < traj_steps; ++i) {
    PathPoint path_pt;
    path_pt.set_x(traj_xy[i].x());
    path_pt.set_y(traj_xy[i].y());
    path_pt.set_z(0.0);
    const double dist2prev = traj_xy[i].DistanceTo(traj_xy[i - 1]);
    const double cur_theta =
        i + 1 == traj_steps || traj_xy[i + 1].DistanceTo(traj_xy[i]) < kEpsilon
            ? traj_pts[i - 1].path_point().theta()
            : (traj_xy[i + 1] - traj_xy[i]).FastAngle();
    path_pt.set_theta(cur_theta);
    path_pt.set_kappa(
        dist2prev < kEpsilon
            ? 0.0
            : NormalizeAngle(cur_theta - traj_pts[i - 1].path_point().theta()) /
                  dist2prev);
    path_pt.set_s(traj_pts[i - 1].path_point().s() + dist2prev);

    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = std::move(path_pt);
    traj_point.set_v(traj_v[i]);
    traj_point.set_a(traj_a[i]);
    traj_point.set_relative_time(i * dt);

    traj_pts.emplace_back(std::move(traj_point));
  }

  Log2DDS::LogLineV2(
      absl::StrCat(Log2DDS::TaskPrefix(plan_id), "init-fake"), Log2DDS::kPink,
      {}, traj_pts, [](const auto& p) -> double { return p.path_point().x(); },
      [](const auto& p) -> double { return p.path_point().y(); });
  return traj_pts;
}

void ParseMotionSearchOutputToMotionSearchDebugProto(
    const MotionSearchOutput& search_output, MotionSearchDebugProto* proto) {
  proto->Clear();

  // Set cost_names.
  for (const auto& name : search_output.cost_provider->cost_names()) {
    auto* cost_name = proto->add_cost_names();
    *cost_name = name;
  }

  search_output.motion_graph->ToProto(proto->mutable_motion_graph());

  // Set edge costs.
  proto->mutable_edge_costs()->Reserve(search_output.motion_graph->edge_size());
  for (const auto& edge_cost : search_output.search_costs) {
    auto* edge_cost_proto = proto->add_edge_costs();
    edge_cost_proto->mutable_costs()->Reserve(edge_cost.feature_cost.size());
    for (const double c : edge_cost.feature_cost) {
      edge_cost_proto->add_costs(c);
    }
    CHECK_EQ(edge_cost_proto->costs_size(), proto->cost_names_size());
    edge_cost_proto->set_cum_cost(edge_cost.cost_to_come);
  }

  proto->set_best_last_edge_index(search_output.best_last_edge_index.value());

  // Get considered trajectory costs (compare with Set edge costs)
  // feature costs, final costs, final progress costs
  proto->mutable_top_k_trajs()->Reserve(search_output.top_k_trajs.size());
  for (int i = 0; i < search_output.top_k_trajs.size(); ++i) {
    const auto last_edge = search_output.top_k_edges[i];
    auto* traj_info_proto = proto->add_top_k_trajs();
    // Feature costs
    traj_info_proto->mutable_costs()->Reserve(
        search_output.cost_provider->cost_names().size());
    for (const double c : search_output.search_costs[last_edge].feature_cost) {
      traj_info_proto->add_costs(c);
    }
    traj_info_proto->set_total_cost(search_output.top_k_total_costs[i]);
    traj_info_proto->set_last_edge_idx(last_edge.value());
    for (const auto& point : search_output.top_k_trajs[i]) {
      *traj_info_proto->add_traj_points() = ToPoseTrajectoryPointProto(point);
    }
  }
}

void ParseMotionSearchOutputToMultiTrajDebugProto(
    const MotionSearchOutput& search_output, MultiTrajDebugProto* proto) {
  const auto& multi_trajs = search_output.multi_traj_candidates;
  proto->Clear();

  proto->mutable_multi_traj_candidates()->Reserve(multi_trajs.size());
  for (const auto& traj : multi_trajs) {
    auto* traj_proto = proto->add_multi_traj_candidates();
    // Set traj points.
    traj_proto->mutable_traj_points()->Reserve(traj.trajectory.size());
    for (const auto& point : traj.trajectory) {
      *traj_proto->add_traj_points() = ToPoseTrajectoryPointProto(point);
    }
    // Set leading trajs.
    traj_proto->mutable_leading_traj_ids()->Reserve(
        traj.leading_traj_ids.size());
    for (const auto& lead_traj_id : traj.leading_traj_ids) {
      traj_proto->add_leading_traj_ids(lead_traj_id);
    }
    traj_proto->mutable_ignored_trajs()->Reserve(traj.ignored_trajs.size());
    for (const auto& [traj_id, info] : traj.ignored_trajs) {
      MultiTrajDebugProto::IgnoredObjectTrajectoryProto ignored_traj;
      ignored_traj.set_traj_id(traj_id);
      ignored_traj.set_time_idx(info.time_idx);
      ignored_traj.set_collision_config(
          static_cast<int>(info.collision_config));
      *traj_proto->add_ignored_trajs() = std::move(ignored_traj);
    }

    // Set cost.
    traj_proto->set_total_cost(traj.total_cost);
  }
}

void ParseMotionSearchOutputToInitializerResult(
    const MotionSearchOutput& search_output, InitializerDebugProto* proto) {
  proto->set_lc_pause(search_output.is_lc_pause);

  auto* follower_set = proto->mutable_follower_objects();
  follower_set->Clear();
  for (const auto& obj_id : search_output.follower_set) {
    *follower_set->Add() = obj_id;
  }

  auto* leader_set = proto->mutable_leader_objects();
  leader_set->Clear();
  for (const auto& obj_id : search_output.leader_set) {
    *leader_set->Add() = obj_id;
  }

  auto* resampled_traj = proto->mutable_resampled_trajectory();
  resampled_traj->Clear();
  resampled_traj->mutable_trajectory_points()->Reserve(
      search_output.traj_points.size());
  for (const auto& point : search_output.traj_points) {
    *resampled_traj->add_trajectory_points() = point;
  }
}

InitializerOutput MakeAebInitializerOutput(
    ApolloTrajectoryPointProto plan_start_point,
    InitializerStateProto new_state, std::string message,
    InitializerDebugProto* debug_proto) {
  LOG_WARN << message << "Initializer will output an AEB trajectory!";

  constexpr double kAebDecel = -10.0;  // m/s^2.
  std::vector<ApolloTrajectoryPointProto> aeb_traj;
  aeb_traj.reserve(kInitializerTrajectorySteps);
  ApolloTrajectoryPointProto cur_point = std::move(plan_start_point);
  for (int i = 0; i < kInitializerTrajectorySteps; ++i) {
    const double dist =
        std::max(0.0, (cur_point.v() + 0.5 * kAebDecel * kTrajectoryTimeStep) *
                          kTrajectoryTimeStep);
    ApolloTrajectoryPointProto next_point;
    *next_point.mutable_path_point() =
        GetPathPointAlongCircle(cur_point.path_point(), dist);
    next_point.set_relative_time((i + 1) * kTrajectoryTimeStep);
    next_point.set_v(
        std::max(0.0, cur_point.v() + kTrajectoryTimeStep * kAebDecel));
    next_point.set_a(
        std::max(kAebDecel, (0.0 - next_point.v()) / kTrajectoryTimeStep));
    cur_point.set_j((next_point.a() - cur_point.a()) / kTrajectoryTimeStep);
    aeb_traj.push_back(std::move(cur_point));
    cur_point = std::move(next_point);
  }

  debug_proto->mutable_multi_traj_debug()
      ->mutable_multi_traj_candidates()
      ->Clear();
  auto& debug_traj = *debug_proto->mutable_multi_traj_debug()
                          ->add_multi_traj_candidates()
                          ->mutable_traj_points();
  debug_traj.Clear();
  debug_traj.Reserve(aeb_traj.size());
  for (const auto& traj_pt : aeb_traj) {
    *debug_traj.Add() = ToPoseTrajectoryPointProto(traj_pt);
  }

  return InitializerOutput{.traj_points = std::move(aeb_traj),
                           .initializer_state = std::move(new_state)};
}

void ParseFeaturesDumpingProto(
    const MotionSearchOutput& search_output,
    ExpertEvaluationProto* expert_proto,
    SampledDpMotionEvaluationProto* candidates_proto) {
  // Sanity Check
  if (search_output.expert_evaluation.traj.empty() ||
      search_output.candidates_evaluation.empty()) {
    LOG_WARN << "No expert evaluation or candidates evaluation for "
                "ParseFeaturesDumpingProto.";
    return;
  }
  // Set expert_proto.
  expert_proto->Clear();

  for (const auto& name : search_output.cost_provider->cost_names()) {
    *expert_proto->add_cost_names() = name;
  }

  expert_proto->mutable_costs()->Reserve(expert_proto->cost_names_size());
  for (const auto& c : search_output.expert_evaluation.feature_costs) {
    expert_proto->add_costs(c);
  }

  expert_proto->set_total_cost(
      search_output.expert_evaluation.weighted_total_cost);

  expert_proto->mutable_trajectory()->mutable_trajectory_points()->Reserve(
      search_output.expert_evaluation.traj.size());
  for (const auto& point : search_output.expert_evaluation.traj) {
    auto* new_trajectory_point =
        expert_proto->mutable_trajectory()->add_trajectory_points();
    *new_trajectory_point = point;
  }

  *expert_proto->mutable_weights() = {
      search_output.cost_provider->weights().begin(),
      search_output.cost_provider->weights().end()};

  expert_proto->mutable_is_filtered_reasons()->set_is_out_of_bound(
      search_output.expert_evaluation.is_filtered_reasons.is_out_of_bound);
  expert_proto->mutable_is_filtered_reasons()->set_is_violating_stop_constraint(
      search_output.expert_evaluation.is_filtered_reasons
          .is_violating_stop_constraint);
  expert_proto->mutable_is_filtered_reasons()->set_is_dynamic_collision(
      search_output.expert_evaluation.is_filtered_reasons.is_dynamic_collision);
  expert_proto->mutable_is_filtered_reasons()->set_is_violating_leading_objects(
      search_output.expert_evaluation.is_filtered_reasons
          .is_violating_leading_objects);

  // Set candidates_proto.
  candidates_proto->Clear();

  auto candidates_cost_name = candidates_proto->mutable_cost_names();
  candidates_cost_name->CopyFrom(expert_proto->cost_names());

  // Get trajectories.
  candidates_proto->mutable_traj_costs()->Reserve(
      search_output.candidates_evaluation.size());
  candidates_proto->mutable_trajectory()->Reserve(
      search_output.candidates_evaluation.size());
  for (const auto& traj_eval : search_output.candidates_evaluation) {
    auto* traj_cost = candidates_proto->add_traj_costs();
    for (const auto& c : traj_eval.feature_costs) {
      traj_cost->add_costs(c);
    }
    traj_cost->set_total_cost(traj_eval.weighted_total_cost);

    auto* traj_proto = candidates_proto->add_trajectory();
    traj_proto->mutable_trajectory_points()->Reserve(traj_eval.traj.size());
    for (const auto& point : traj_eval.traj) {
      auto* new_trajectory_point = traj_proto->add_trajectory_points();
      *new_trajectory_point = point;
    }
  }

  candidates_proto->set_min_cost(search_output.min_cost);
  *candidates_proto->mutable_weights() = {
      search_output.cost_provider->weights().begin(),
      search_output.cost_provider->weights().end()};
}
}  // namespace st::planning
