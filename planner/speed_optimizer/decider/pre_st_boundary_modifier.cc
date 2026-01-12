

#include "planner/speed_optimizer/decider/pre_st_boundary_modifier.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"

//#include "global/buffered_logger.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/math/hermite_spline.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/math/math_utils.h"
#include "object_manager/planner_object.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "predictor/prediction_object_state.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_defs.h"
#include "planner/speed_optimizer/decider/st_boundary_modifier_util.h"
#include "plan_common/maps/overlap_info.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_point.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/path_util.h"
#include "plan_common/log_data.h"
#include "plan_common/util/status_macros.h"
#include "predictor/prediction_util.h"
#include "planner/speed_optimizer/path_speed_combiner.h"
#include "planner/speed_optimizer/speed_finder_util.h"

DEFINE_bool(enable_lateral_modifier_draw_st_traj, false,
            "Whether to enable lateral modifier draw st trajectory.");
DEFINE_bool(enable_modify_st_traj_laterally, false,
            "Whether to enable modify st traj laterally.");

namespace st {
namespace planning {
namespace {

constexpr double kEps = 1e-3;
const PiecewiseLinearFunction<double, double> kVelToMaxObjectAccelPlf(
    {Kph2Mps(30.0), Kph2Mps(60.0), Kph2Mps(90.0)}, {2.0, 1.5, 1.0});
const PiecewiseLinearFunction<double, double> kAccelToAccelDurationPlf(
    {-3.5, -2.0, -0.5, 0, 0.3, 1.0, 2.0}, {3.0, 1.5, 2.0, 2.5, 2.0, 1.5, 2.5});
const PiecewiseLinearFunction<double, double> kHeadwayToAccelDurationFactorPlf(
    {2.0, 5.0, 8.0}, {1.0, 0.9, 0.8});

void ExtendPathAlongDrivePassage(const DrivePassage& drive_passage,
                                 double length, DiscretizedPath* path) {
  CHECK_NOTNULL(path);
  constexpr double kPathInterval = 3.0;  // m.
  const auto end_point_sl =
      drive_passage.QueryFrenetCoordinateAt(ToVec2d(path->back()));
  if (!end_point_sl.ok()) return;
  const double end_s = end_point_sl->s;
  const double end_l = end_point_sl->l;
  double path_end_s = path->back().s();
  double ds = 0.0;
  while (path_end_s < length) {
    path_end_s += kPathInterval;
    ds += kPathInterval;
    const double curr_s = end_s + ds;
    ASSIGN_OR_BREAK(const Vec2d point,
                    drive_passage.QueryPointXYAtSL(curr_s, end_l));
    ASSIGN_OR_BREAK(const double theta,
                    drive_passage.QueryTangentAngleAtS(curr_s));
    auto extend_point = path->back();
    extend_point.set_x(point.x());
    extend_point.set_y(point.y());
    extend_point.set_theta(theta);
    extend_point.set_s(path_end_s);
    path->push_back(std::move(extend_point));
  }
  return;
}

void ExtendPathAlongCircle(double length, DiscretizedPath* path) {
  CHECK_NOTNULL(path);
  constexpr double kNonExtendKappaThres = 0.2;   // m^-1.
  constexpr double kNonExtendThetaThres = 0.25;  // rad, about 15 deg.
  const double s = length - path->back().s() + kEps;
  if (s < 0.0) return;
  if (std::abs(path->back().kappa()) > kNonExtendKappaThres ||
      std::abs(s * path->back().kappa()) > kNonExtendThetaThres)
    return;
  auto path_point = GetPathPointAlongCircle(path->back(), s);
  path->push_back(std::move(path_point));
  return;
}

bool RemodifySpeedProfileByLength(int time_horizon, double time_step,
                                  int const_a_idx, double pred_length,
                                  planning::SpeedVector* prediction_speed) {
  CHECK_NOTNULL(prediction_speed);
  if (const_a_idx > prediction_speed->size() || const_a_idx <= 0) {
    return false;
  }
  constexpr double kMinConstDeceleration = -1.5;  // m/s^2.
  constexpr double kMaxDecelerationTime = 4.0;    // s.
  const double a_duration = const_a_idx * time_step;
  const auto& const_acc_pt = (*prediction_speed)[const_a_idx - 1];
  const double s_const_acc = const_acc_pt.s();
  const double v_const_acc = const_acc_pt.v();
  double dec_time = sqrt(2.0 * (pred_length - prediction_speed->back().s()) /
                         kMinConstDeceleration);
  double const_deceleration = kMinConstDeceleration;
  bool is_cut_start_time = false;
  if (std::abs(dec_time * kMinConstDeceleration) > v_const_acc) {
    dec_time = std::min(v_const_acc / std::abs(kMinConstDeceleration),
                        kMaxDecelerationTime);
    is_cut_start_time = true;
  }
  if (dec_time > kMaxDecelerationTime) {
    dec_time = kMaxDecelerationTime;
    const_deceleration =
        2 * (pred_length - prediction_speed->back().s()) / Sqr(dec_time);
  }

  constexpr double kEps = 0.01;
  const double uniform_speed_time =
      (pred_length - v_const_acc * dec_time -
       0.5 * kMinConstDeceleration * Sqr(dec_time) - s_const_acc) /
      std::max(v_const_acc, kEps);
  int start_idx = is_cut_start_time
                      ? (a_duration + uniform_speed_time) / time_step - 1
                      : time_horizon - dec_time / time_step - 1;
  double curr_t = start_idx * time_step;
  if (start_idx < const_a_idx) {
    start_idx = const_a_idx;
    curr_t = start_idx * time_step;
    if (std::abs(pred_length - s_const_acc) < kEps) {
      LOG_WARN << "Remodify failed! Use origin prediction trajectory!";
      return false;
    }
    const_deceleration =
        -Sqr(v_const_acc) / (2 * std::abs(pred_length - s_const_acc));
    dec_time = std::abs(v_const_acc / const_deceleration);
  }
  prediction_speed->resize(start_idx + 1);
  for (int i = start_idx; i < time_horizon; ++i) {
    auto& prev_speed_pt = prediction_speed->back();
    curr_t += time_step;
    double curr_a = 0.0;
    if (curr_t <= start_idx * time_step + dec_time) curr_a = const_deceleration;
    const double curr_v = std::max(prev_speed_pt.v() + curr_a * time_step, 0.0);
    const double curr_s =
        prev_speed_pt.s() + 0.5 * (curr_v + prev_speed_pt.v()) * time_step;
    curr_a = (curr_v - prev_speed_pt.v()) / time_step;
    prediction_speed->emplace_back(curr_t, curr_s, curr_v, curr_a, /*j=*/0.0);
  }
  return true;
}

std::optional<StBoundaryModificationResult> ModifyCipvStBoundary(
    const StGraph& st_graph, const DrivePassage* drive_passage,
    const PathSlBoundary* path_sl_boundary,
    const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj, double current_v,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    double path_end_s) {
  const auto& cipv_traj_points = st_traj.trajectory().points();
  if (cipv_traj_points.size() < 2) return std::nullopt;

  const StBoundary& st_boundary =
      *CHECK_NOTNULL(st_boundary_wd.raw_st_boundary());

  double is_distance_vehicle = false;
  constexpr double kIgnoreAccelDistanceThres = 80.0;  // m.
  if (st_boundary.bottom_left_point().s() > kIgnoreAccelDistanceThres) {
    is_distance_vehicle = true;
  }
  const auto& object_pose = st_traj.planner_object().pose();
  const double object_vel = cipv_traj_points.front().v();
  constexpr double kMinPerceptionAcc = -5.0;  // m/s^2.
  double object_accel = 0.0;
  if (!is_distance_vehicle) {
    object_accel = std::clamp(object_pose.a(), kMinPerceptionAcc,
                              kVelToMaxObjectAccelPlf(object_vel));
  }

  constexpr double kDefaultTimeStep = 0.1;  // s.
  double prediction_time_step =
      cipv_traj_points[1].t() - cipv_traj_points[0].t();
  if (prediction_time_step < kDefaultTimeStep) {
    prediction_time_step = kDefaultTimeStep;
  }

  const double cipv_headway =
      st_boundary.bottom_left_point().s() / std::max(0.1, current_v);
  const double a_duration = kAccelToAccelDurationPlf(object_accel) *
                            kHeadwayToAccelDurationFactorPlf(cipv_headway);
  auto prediction_speed = prediction::GenerateSpeedProfileByConstAccel(
      object_vel, object_accel, cipv_traj_points.size(), prediction_time_step,
      a_duration);
  constexpr double kTrajectoryLengthBuffer = 3.0;  // m.
  if (prediction_speed.back().s() >
      cipv_traj_points.back().s() + kTrajectoryLengthBuffer) {
    if (!RemodifySpeedProfileByLength(
            cipv_traj_points.size(), prediction_time_step,
            a_duration / prediction_time_step, cipv_traj_points.back().s(),
            &prediction_speed)) {
      return std::nullopt;
    }
  }
  if (prediction_speed.size() < 2) return std::nullopt;

  auto prediction_path =
      prediction::PredictedTrajectoryPointsToPathPoints(cipv_traj_points);
  std::vector<prediction::PredictedTrajectoryPoint>
      prediction_trajectory_points;
  if (!prediction::CombinePathAndSpeed(prediction_path, prediction_speed,
                                       prediction_time_step,
                                       &prediction_trajectory_points)
           .ok()) {
    return std::nullopt;
  }

  auto new_pred_traj = st_traj.trajectory();
  *new_pred_traj.mutable_points() = std::move(prediction_trajectory_points);
  auto new_st_traj =
      st_traj.CreateTrajectoryMutatedInstance(std::move(new_pred_traj));

  StGraph::StBoundaryOutput st_boundary_output;
  if (ContainsKey(leading_objs, st_traj.traj_id()) &&
      drive_passage != nullptr && path_sl_boundary != nullptr) {
    auto leading_st_boundary = st_graph.MapLeadingObject(
        new_st_traj, *drive_passage, *path_sl_boundary);
    if (leading_st_boundary != nullptr) {
      st_boundary_output.st_boundaries.push_back(
          std::move(leading_st_boundary));
    }
  } else {
    auto mapping_output = st_graph.MapMovingSpacetimeObject(
        new_st_traj, /*generate_lane_change_gap=*/false,
        /*calc_moving_close_traj=*/false, /*nudge_object_info=*/nullptr);
    st_boundary_output.st_boundaries = std::move(mapping_output.st_boundaries);
  }

  auto& new_st_boundaries = st_boundary_output.st_boundaries;
  if (new_st_boundaries.empty()) return std::nullopt;
  std::vector<StBoundaryWithDecision> new_st_boundaries_wd;
  new_st_boundaries_wd.reserve(new_st_boundaries.size());
  const auto modifier_type = StBoundaryModifierProto::CIPV;
  for (auto& new_st_boundary : new_st_boundaries) {
    StBoundaryModifierProto modifier;
    modifier.set_modifier_type(modifier_type);
    new_st_boundary->set_id(absl::StrCat(new_st_boundary->id(), "|m"));
    if (st_boundary.overlap_meta().has_value()) {
      new_st_boundary->set_overlap_meta(*st_boundary.overlap_meta());
    }
    new_st_boundaries_wd.emplace_back(
        std::move(new_st_boundary), st_boundary_wd.decision_type(),
        st_boundary_wd.decision_reason(),
        absl::StrCat(st_boundary_wd.decision_info(),
                     " and keep it after modified by ",
                     StBoundaryModifierProto::ModifierType_Name(modifier_type)),
        st_boundary_wd.follow_standstill_distance(),
        st_boundary_wd.lead_standstill_distance(), st_boundary_wd.pass_time(),
        st_boundary_wd.yield_time(), path_end_s);
    new_st_boundaries_wd.back().set_modifier(std::move(modifier));
    new_st_boundaries_wd.back().set_is_cipv(st_boundary_wd.is_cipv());
    new_st_boundaries_wd.back().set_is_stay_cipv(st_boundary_wd.is_stay_cipv());
  }
  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(new_st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modifier_type});
}

std::optional<StBoundaryModificationResult> ModifyOncomingStBoundary(
    const PreStboundaryModifierInput& input, const StGraph& st_graph,
    const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj, double current_v,
    const DiscretizedPath& path) {
  // An st-boundary is considered to be ONCOMING if:
  // 1. Its first overlap time is less than 0.5s;
  // 2. Its first-overlap heading diff is beyond certain threshold;
  // 3. Its first-overlap s_lower is larger than last-overlap s_lower.
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  constexpr double kMaxTimeLimit = 1.5;  // s.

  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return std::nullopt;
  }

  if (st_boundary.bottom_left_point().t() > kMaxTimeLimit) {
    return std::nullopt;
  }

  // Do not modify prediction for objects making unprotected left-turn.
  if (st_boundary.overlap_meta()->source() == StOverlapMetaProto::LANE_CROSS) {
    return std::nullopt;
  }

  if (input.path_semantics == nullptr || input.drive_passage == nullptr) {
    return std::nullopt;
  }

  const auto& path_semantics = *input.path_semantics;
  const auto& drive_passage = *input.drive_passage;
  if (!path_semantics.empty() && path_semantics.front().lane_semantic ==
                                     LaneSemantic::INTERSECTION_STRAIGHT) {
    const double reac_time = 1.5;
    const double safet_buffer = 8.0;
    double safet_dist =
        safet_buffer +
        (input.current_v + std::fabs(st_traj.planner_object().pose().v())) *
            reac_time;
    safet_dist = std::clamp(safet_dist, 0.0, 30.0);
    const auto& path_points = *input.path;
    Vec2d start_point(path_points.front().x(), path_points.front().y());
    const auto ego_fpos = drive_passage.QueryFrenetCoordinateAt(start_point);
    const auto obs_fbox = drive_passage.QueryFrenetBoxAtContour(
        st_traj.planner_object().contour());
    double ds = std::numeric_limits<double>::max();
    if (obs_fbox.ok() && ego_fpos.ok()) {
      ds = obs_fbox->s_min - ego_fpos->s -
           input.vehicle_geom->front_edge_to_center();
    }
    const auto& current_lane_info = *path_semantics.front().lane_info;
    // debug
    const std::string debug =
        absl::StrCat(st_boundary_wd.object_id().value(), " ds = ", ds,
                     " safet_dist = ", safet_dist);
    Log2DDS::LogDataV0("ModifyOncomingStBoundary", debug);
    //
    if (!(current_lane_info.junction_id() == 0) &&
        (st_traj.trajectory().intention() ==
             TrajectoryIntention::INTENTION_TURN_LEFT ||
         st_traj.trajectory().intention() ==
             TrajectoryIntention::INTENTION_TURN_RIGHT)) {
      Log2DDS::LogDataV0("ModifyOncomingStBoundary",
                         st_boundary_wd.object_id().value() + " INTENTION");
      return std::nullopt;
    }

    if (!(current_lane_info.junction_id() == 0) && ds < safet_dist) {
      Log2DDS::LogDataV0("ModifyOncomingStBoundary",
                         st_boundary_wd.object_id().value() + " safet_dist");
      return std::nullopt;
    }
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      st_traj.states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kOnComingThreshold = 5.0 * M_PI / 6.0;
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) <
      kOnComingThreshold) {
    return std::nullopt;
  }

  VLOG(2) << "St-boundary " << st_boundary_wd.id()
          << " is considered to be ONCOMING.";

  // Only modify the oncoming prediction if it would cause uncomfortable brake.
  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (st_boundary.bottom_right_point().s() > const_speed_s) {
    // No brake is needed.
    return std::nullopt;
  }
  constexpr double kUncomfortableDecel = 0.8;  // m/s^2.
  constexpr double kAvMinVel = 3.0;            // m/s^2.
  const double max_decel_s =
      std::max(0.5 * Sqr(current_v) / kUncomfortableDecel,
               kAvMinVel * st_boundary.bottom_right_point().t());
  if (max_decel_s < st_boundary.bottom_right_point().s()) {
    const double estimated_av_decel =
        2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
        Sqr(st_boundary.bottom_right_point().t());
    if (estimated_av_decel < kUncomfortableDecel) {
      return std::nullopt;
    }
  }
  if (IsConsiderOncomingObs(*input.st_traj_mgr, st_boundary, drive_passage,
                            *input.vehicle_geom, current_v, path)) {
    return std::nullopt;
  }

  // Modify oncoming spacetime trajectory.
  VLOG(2) << "Modify ONCOMING st-boundary " << st_boundary.id();
  constexpr double kOncomingReactionTime = 0.5;      // s.
  constexpr double kOncomingObjectDecel = -1.8;      // m/s^2.
  constexpr double kOncomingObjectMildDecel = -1.5;  // m/s^2.

  // If AV is doing lane change or lane borrow to an opposite lane, we consider
  // oncoming object will decelerate more mildly to make AV more conservative.
  const double oncoming_obj_decel =
      st_boundary.overlap_meta()->source() == StOverlapMetaProto::AV_CUTIN
          ? std::min(st_traj.planner_object().pose().a(),
                     kOncomingObjectMildDecel)
          : std::min(st_traj.planner_object().pose().a(), kOncomingObjectDecel);

  // Make new spacetime trajectory.
  // FIXME： The current modification type for OBJECT_CUT_IN is
  // LON_LAT_MODIFIABLE but we modifies oncoming st-boundaries longitudinally
  // just to be in consistent with the old logic. The modification method should
  // be subject to the modification type given in overlap meta.
  auto new_st_traj = CreateSpacetimeTrajectoryByDecelAfterDelay(
      st_traj, kOncomingReactionTime, oncoming_obj_decel);

  // Generate new st_boundaries.
  auto st_boundary_output = st_graph.MapMovingSpacetimeObject(
      new_st_traj, /*generate_lane_change_gap=*/false,
      /*calc_moving_close_traj=*/false, nullptr);
  auto& new_st_boundaries = st_boundary_output.st_boundaries;
  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  st_boundaries_wd.reserve(new_st_boundaries.size());
  const auto modifier_type = StBoundaryModifierProto::ONCOMING;
  const auto decision_info =
      st_boundary_wd.decision_type() == StBoundaryProto::UNKNOWN
          ? st_boundary_wd.decision_info()
          : absl::StrCat(
                st_boundary_wd.decision_info(),
                " and keep it after modified by ",
                StBoundaryModifierProto::ModifierType_Name(modifier_type));
  for (auto& new_stb : new_st_boundaries) {
    StBoundaryModifierProto modifier;
    modifier.set_modifier_type(modifier_type);
    new_stb->set_id(absl::StrCat(new_stb->id(), "|m"));
    const auto& overlap_meta = st_boundary_wd.raw_st_boundary()->overlap_meta();
    if (overlap_meta.has_value()) {
      new_stb->set_overlap_meta(*overlap_meta);
    }
    // TODO(all): Compute pass_time and yield_time for new st_boundary.
    st_boundaries_wd.emplace_back(
        std::move(new_stb), st_boundary_wd.decision_type(),
        st_boundary_wd.decision_reason(), decision_info,
        st_boundary_wd.follow_standstill_distance(),
        st_boundary_wd.lead_standstill_distance(),
        /*pass_time=*/0.0, /*yield_time=*/0.0, input.path->length());
    st_boundaries_wd.back().set_modifier(std::move(modifier));
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modifier_type});
}

std::optional<StBoundaryModificationResult> ModifyRightTurnStBoundary(
    const PreStboundaryModifierInput& input, const StGraph& st_graph,
    const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj, double current_v,
    const DiscretizedPath& path) {
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  constexpr double kMinOverlapTime = 3.0;     // s.
  constexpr double kMinConsiderSpeed = 10.0;  // kph

  if (current_v > Kph2Mps(kMinConsiderSpeed)) {
    return std::nullopt;
  }

  if (st_boundary.bottom_left_point().t() < kMinOverlapTime) {
    return std::nullopt;
  }

  // Do not modify prediction for objects making unprotected left-turn.
  if (st_boundary.overlap_meta()->source() == StOverlapMetaProto::LANE_CROSS) {
    return std::nullopt;
  }

  // Do not modify prediction if a 1m/s^2 acceleration can not overtake.
  if (current_v * st_boundary.upper_left_point().t() +
          0.5 * Sqr(st_boundary.upper_left_point().t()) <
      st_boundary.upper_left_point().s()) {
    return std::nullopt;
  }

  if (input.path_semantics == nullptr || input.drive_passage == nullptr) {
    return std::nullopt;
  }

  const auto& path_semantics = *input.path_semantics;
  const auto& drive_passage = *input.drive_passage;
  if (!path_semantics.empty() && path_semantics.front().lane_semantic ==
                                     LaneSemantic::INTERSECTION_RIGHT_TURN) {
    const double reac_time = 2.0;
    const double safety_buffer = 1.0;
    double safet_dist = std::max(
        1.0, safety_buffer +
                 (st_traj.planner_object().pose().v() - input.current_v) *
                     reac_time);
    const auto& path_points = *input.path;
    Vec2d start_point(path_points.front().x(), path_points.front().y());
    const auto ego_fpos = drive_passage.QueryFrenetCoordinateAt(start_point);
    const auto obs_fbox = drive_passage.QueryFrenetBoxAtContour(
        st_traj.planner_object().contour());
    const auto obs_fpos = drive_passage.QueryFrenetCoordinateAt(
        st_traj.planner_object().pose().pos());
    const auto& lane_seq = drive_passage.lane_seq_info()->lane_seq;
    const auto lane_boundary_info =
        drive_passage.QueryEnclosingLaneBoundariesAtS(ego_fpos->s);
    const double half_lane_width =
        (lane_boundary_info.left.has_value() &&
         lane_boundary_info.right.has_value())
            ? (lane_boundary_info.left->lat_offset -
               lane_boundary_info.right->lat_offset) *
                  0.5
            : kDefaultHalfLaneWidth * 0.5;
    double ds = std::numeric_limits<double>::max();
    double l_pos_diff = std::numeric_limits<double>::max();
    if (obs_fbox.ok() && ego_fpos.ok() && obs_fpos.ok()) {
      ds = ego_fpos->s - input.vehicle_geom->back_edge_to_center() -
           obs_fbox->s_max;
      l_pos_diff = std::abs(ego_fpos->l - obs_fpos->l);
    }
    const auto& current_lane_info = *path_semantics.front().lane_info;
    // debug
    const std::string debug =
        absl::StrCat(st_boundary_wd.object_id().value(), " ds = ", ds,
                     " safet_dist = ", safet_dist);
    Log2DDS::LogDataV0("ModifyRightTurnStBoundary", debug);
    //

    if (ds < safet_dist || l_pos_diff > half_lane_width) {
      Log2DDS::LogDataV0("ModifyRightTurnStBoundary",
                         st_boundary_wd.object_id().value() + " safet_dist");
      return std::nullopt;
    }
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      st_traj.states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kRightTurnThreshold = M_PI / 6.0;
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) >
      kRightTurnThreshold) {
    return std::nullopt;
  }

  VLOG(2) << "St-boundary " << st_boundary_wd.id()
          << " is considered to be RightTurn Back VRU.";

  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (st_boundary.bottom_right_point().s() < const_speed_s) {
    // No brake is needed.
    return std::nullopt;
  }

  // Modify oncoming spacetime trajectory.
  VLOG(2) << "Modify RightTurn st-boundary " << st_boundary.id();
  constexpr double kRightTurnObjectMildDecel = -0.7;  // m/s^2.

  // If AV is doing lane change or lane borrow to an opposite lane, we consider
  // oncoming object will decelerate more mildly to make AV more conservative.
  const double RightTurn_obj_decel =
      st_traj.planner_object().pose().a() > kRightTurnObjectMildDecel
          ? kRightTurnObjectMildDecel
          : st_traj.planner_object().pose().a() - 0.3;

  // Make new spacetime trajectory.
  // FIXME： The current modification type for OBJECT_CUT_IN is
  // LON_LAT_MODIFIABLE but we modifies oncoming st-boundaries longitudinally
  // just to be in consistent with the old logic. The modification method should
  // be subject to the modification type given in overlap meta.
  auto new_st_traj = CreateSpacetimeTrajectoryByDecelAfterDelay(
      st_traj, st_boundary.bottom_left_point().t() - 1.0, RightTurn_obj_decel);

  // Generate new st_boundaries.
  auto st_boundary_output = st_graph.MapMovingSpacetimeObject(
      new_st_traj, /*generate_lane_change_gap=*/false,
      /*calc_moving_close_traj=*/false, nullptr);
  auto& new_st_boundaries = st_boundary_output.st_boundaries;
  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  st_boundaries_wd.reserve(new_st_boundaries.size());
  const auto modifier_type = StBoundaryModifierProto::RIGHT_TURN_BACK_CAR;
  const auto decision_info =
      st_boundary_wd.decision_type() == StBoundaryProto::UNKNOWN
          ? st_boundary_wd.decision_info()
          : absl::StrCat(
                st_boundary_wd.decision_info(),
                " and keep it after modified by ",
                StBoundaryModifierProto::ModifierType_Name(modifier_type));
  for (auto& new_stb : new_st_boundaries) {
    StBoundaryModifierProto modifier;
    modifier.set_modifier_type(modifier_type);
    new_stb->set_id(absl::StrCat(new_stb->id(), "|m"));
    const auto& overlap_meta = st_boundary_wd.raw_st_boundary()->overlap_meta();
    if (overlap_meta.has_value()) {
      new_stb->set_overlap_meta(*overlap_meta);
    }
    st_boundaries_wd.emplace_back(
        std::move(new_stb), st_boundary_wd.decision_type(),
        st_boundary_wd.decision_reason(), decision_info,
        st_boundary_wd.follow_standstill_distance(),
        st_boundary_wd.lead_standstill_distance(),
        /*pass_time=*/0.0, /*yield_time=*/0.0, input.path->length());
    st_boundaries_wd.back().set_modifier(std::move(modifier));
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modifier_type});
}

// Generate modified path with spline interpolation method. Note that instead of
// connecting start path point with nudge path point via spline directly,
// this function only fit every point's nudge offset with spline. By doing so,
// modified path has almost similar shape with origin predicted path. That
// consistent with our designed motivation: we just want to "push" predicted
// path some distance to comply av's intention, but not replan it entirely.
DiscretizedPath GeneratePredPathWithNudgeVector(
    const NudgeVector& nudge_vector,
    absl::Span<const prediction::PredictedTrajectoryPoint> pred_traj_points) {
  CHECK(!pred_traj_points.empty());
  CHECK_LT(nudge_vector.leverage_point_index, pred_traj_points.size());

  PathPoint prev_point =
      GetPathPointFromPredictedTrajectoryPoint(pred_traj_points.front());
  const double nudge_s =
      pred_traj_points[nudge_vector.leverage_point_index].s();
  const double end_s = pred_traj_points.back().s();
  constexpr double kMinPointDist = 0.01;  // m.
  std::vector<PathPoint> path_points;
  path_points.reserve(pred_traj_points.size());
  path_points.push_back(prev_point);
  for (int i = 1; i < pred_traj_points.size(); ++i) {
    const auto& traj_pt = pred_traj_points[i];
    double nudge_offset = 0.0;
    if (traj_pt.s() < nudge_s) {
      // Treat offset as l, set dl and ddl to 0.0 to ensure modified
      // path's heading and curvature continuous with origin predicted path
      // according to the formula below:
      /**
       * theta = dl / (1 - kappa_r*l) + theta_r
       * assume kappa_r*l << 1
       * theta = dl + theta_r
       * kappa = dtheta/ds = d(dl+theta_r)/ds = ddl + kappa_r
       **/
      nudge_offset =
          QuinticHermiteLerp(0.0, nudge_vector.offset, 0.0, 0.0, 0.0, 0.0,
                             LerpFactor(0.0, nudge_s, traj_pt.s()));
    } else {
      nudge_offset =
          QuinticHermiteLerp(nudge_vector.offset, 0.0, 0.0, 0.0, 0.0, 0.0,
                             LerpFactor(nudge_s, end_s, traj_pt.s()));
    }

    Vec2d nudge_direction;
    if (nudge_vector.direction == NudgeVector::Direction::RIGHT) {
      nudge_direction = Vec2d::FastUnitFromAngle(
          NormalizeAngle(traj_pt.theta() - M_PI * 0.5));
    } else {
      nudge_direction = Vec2d::FastUnitFromAngle(
          NormalizeAngle(traj_pt.theta() + M_PI * 0.5));
    }

    const Vec2d nudge_pos = traj_pt.pos() + nudge_direction * nudge_offset;
    const double dist =
        Vec2d(prev_point.x(), prev_point.y()).DistanceTo(nudge_pos);
    if (dist < kMinPointDist) continue;

    PathPoint curr_point;
    curr_point.set_x(nudge_pos.x());
    curr_point.set_y(nudge_pos.y());
    curr_point.set_s(prev_point.s() + dist);
    path_points.push_back(curr_point);
    prev_point = std::move(curr_point);
  }

  for (int i = 1; i < path_points.size() - 1; ++i) {
    const Vec2d curr_pos(path_points[i].x(), path_points[i].y());
    const Vec2d next_pos(path_points[i + 1].x(), path_points[i + 1].y());
    const double curr_theta = (next_pos - curr_pos).FastAngle();
    path_points[i].set_theta(curr_theta);
    const Vec2d prev_pos(path_points[i - 1].x(), path_points[i - 1].y());
    path_points[i].set_kappa(
        NormalizeAngle(curr_theta - path_points[i - 1].theta()) /
        (curr_pos - prev_pos).norm());
  }
  const auto last_path_point =
      GetPathPointFromPredictedTrajectoryPoint(pred_traj_points.back());
  path_points.back().set_theta(last_path_point.theta());
  path_points.back().set_kappa(last_path_point.kappa());

  // TODO: There is no need return to origin last point in many cases.

  return DiscretizedPath(path_points);
}

// Use origin prediction's accels to generate new traj points.
std::vector<prediction::PredictedTrajectoryPoint>
GenerateNewTrajPointsBasedOnPredictionAccels(
    const DiscretizedPath& path,
    absl::Span<const prediction::PredictedTrajectoryPoint> pred_traj_points) {
  const double t_step = pred_traj_points.size() > 1
                            ? pred_traj_points[1].t() - pred_traj_points[0].t()
                            : prediction::kPredictionTimeStep;

  std::vector<AccelPoint> accel_point_list;
  accel_point_list.reserve(pred_traj_points.size());
  for (const auto& traj_point : pred_traj_points) {
    accel_point_list.emplace_back(traj_point.t(), traj_point.a());
  }

  auto prev_traj_point = pred_traj_points.front();
  constexpr double kMinDist = 1e-6;
  const double path_length = path.length();
  std::vector<prediction::PredictedTrajectoryPoint> new_traj_points;
  new_traj_points.reserve(pred_traj_points.size());
  for (const auto& accel_point : accel_point_list) {
    const double curr_t = accel_point.t;
    const double dist =
        std::max(kMinDist, prev_traj_point.v() * t_step +
                               0.5 * prev_traj_point.a() * Sqr(t_step));
    const double curr_s = prev_traj_point.s() + dist;
    PathPoint curr_path_point;
    if (curr_s >= path_length) {
      const auto prev_path_point =
          GetPathPointFromPredictedTrajectoryPoint(prev_traj_point);
      curr_path_point = GetPathPointAlongCircle(prev_path_point, dist);
    } else {
      curr_path_point = path.Evaluate(curr_s);
    }
    prediction::PredictedTrajectoryPoint curr_traj_point;
    SetPredictedTrajectoryPointSpatialInfoFromPathPoint(curr_path_point,
                                                        &curr_traj_point);
    const double curr_v = prev_traj_point.v() + prev_traj_point.a() * t_step;
    curr_traj_point.set_t(curr_t);
    curr_traj_point.set_v(curr_v);
    curr_traj_point.set_a(accel_point.a);
    new_traj_points.push_back(curr_traj_point);
    prev_traj_point = std::move(curr_traj_point);
  }
  return new_traj_points;
}

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryWithNudgeVector(
    const NudgeVector& nudge_vector,
    const SpacetimeObjectTrajectory& st_object) {
  const auto new_pred_path = GeneratePredPathWithNudgeVector(
      nudge_vector, st_object.trajectory().points());
  auto generated_pred_traj = GenerateNewTrajPointsBasedOnPredictionAccels(
      new_pred_path, st_object.trajectory().points());
  auto new_pred_traj = st_object.trajectory();
  // Note: lane path, priority and other properties of the new prediction
  // trajectory would be inaccurate because we only replace the trajectory
  // points here.
  *new_pred_traj.mutable_points() = std::move(generated_pred_traj);
  return st_object.CreateTrajectoryMutatedInstance(std::move(new_pred_traj));
}

std::optional<StBoundaryModificationResult>
ModifyStBoundaryViaLatModificationInfo(
    const StGraph& st_graph, const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj,
    const StBoundaryLatModificationInfo& lat_modification_info,
    double path_end_s) {
  // Make new spacetime trajectory.
  auto new_st_traj = CreateSpacetimeTrajectoryWithNudgeVector(
      lat_modification_info.nudge_vector, st_traj);

  // Generate new st_boundaries.
  auto st_boundary_output = st_graph.MapMovingSpacetimeObject(
      new_st_traj, /*generate_lane_change_gap=*/false,
      /*calc_moving_close_traj=*/false, nullptr);
  auto& new_st_boundaries = st_boundary_output.st_boundaries;
  const auto modifier_type = StBoundaryModifierProto::ModifierType_Name(
      lat_modification_info.modifier_type);
  const std::string decision_info =
      absl::StrCat(st_boundary_wd.decision_info(), " and has been modified by ",
                   modifier_type);
  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  st_boundaries_wd.reserve(new_st_boundaries.size());
  for (auto& st_boundary : new_st_boundaries) {
    StBoundaryModifierProto modifier;
    modifier.set_modifier_type(lat_modification_info.modifier_type);
    st_boundary->set_id(absl::StrCat(st_boundary->id(), "|m"));

    // TODO: New st-boundary has no overlap meta after map st-traj,
    // for laterally modified boundaries, overlap meta data will be used later,
    // so set here, but this is a hack, should re-analysis overlap again.
    const auto& overlap_meta = st_boundary_wd.st_boundary()->overlap_meta();
    if (overlap_meta.has_value()) {
      st_boundary->set_overlap_meta(*overlap_meta);
    }

    st_boundaries_wd.emplace_back(
        std::move(st_boundary), st_boundary_wd.decision_type(),
        st_boundary_wd.decision_reason(), decision_info,
        st_boundary_wd.follow_standstill_distance(),
        st_boundary_wd.lead_standstill_distance(),
        /*pass_time=*/0.0, /*yield_time=*/0.0, path_end_s);
    st_boundaries_wd.back().set_modifier(std::move(modifier));
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = lat_modification_info.modifier_type});
}

std::optional<StBoundaryModificationResult> ModifyStBoundaryForOncoming(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd) {
  std::optional<StBoundaryModificationResult> res = std::nullopt;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  // Only pre-modify st-boundaries having overlap meta.
  if (!st_boundary.overlap_meta().has_value()) return res;

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) return res;

  CHECK(st_boundary.object_type() == StBoundaryProto::VEHICLE ||
        st_boundary.object_type() == StBoundaryProto::CYCLIST ||
        st_boundary.object_type() == StBoundaryProto::PEDESTRIAN)
      << StBoundaryProto::ObjectType_Name(st_boundary.object_type());

  const auto& traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());

  const auto* traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  // Modify oncoming predictions that would cause uncomfortable brake.
  res = ModifyOncomingStBoundary(input, *input.st_graph, st_boundary_wd, *traj,
                                 input.current_v, *input.path);
  if (res.has_value()) {
    return res;
  }

  return res;
}

std::optional<StBoundaryModificationResult> ModifyStBoundaryForRightTurn(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd,
    OpenLoopSpeedLimit* open_loop_speed_limit) {
  std::optional<StBoundaryModificationResult> res = std::nullopt;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  if (st_boundary.is_protective()) return res;
  // Only pre-modify st-boundaries having overlap meta.
  if (!st_boundary.overlap_meta().has_value()) return res;

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) {
    return res;
  }

  // CHECK(st_boundary.object_type() == StBoundaryProto::CYCLIST ||
  //       st_boundary.object_type() == StBoundaryProto::PEDESTRIAN)
  //     << StBoundaryProto::ObjectType_Name(st_boundary.object_type());
  if (st_boundary.object_type() != StBoundaryProto::CYCLIST &&
      st_boundary.object_type() != StBoundaryProto::PEDESTRIAN) {
    return res;
  }

  const auto& traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());

  const auto* traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));
  // Modify oncoming predictions that would cause uncomfortable brake.
  res = ModifyRightTurnStBoundary(input, *input.st_graph, st_boundary_wd, *traj,
                                  input.current_v, *input.path);
  if (res.has_value()) {
    open_loop_speed_limit->AddALimit(0.5, std::nullopt,
                                     "Rightturn cutin VRU ALimit");
  }

  return res;
}

std::optional<StBoundaryModificationResult> ModifyStBoundaryLaterally(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd) {
  std::optional<StBoundaryModificationResult> res = std::nullopt;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  // Only pre-modify st-boundaries having overlap meta.
  if (!st_boundary.overlap_meta().has_value()) return res;

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) return res;

  CHECK(st_boundary.object_type() == StBoundaryProto::VEHICLE ||
        st_boundary.object_type() == StBoundaryProto::CYCLIST ||
        st_boundary.object_type() == StBoundaryProto::PEDESTRIAN)
      << StBoundaryProto::ObjectType_Name(st_boundary.object_type());

  const auto& traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());

  const auto* traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  if (input.lat_modification_info_map != nullptr) {
    const auto* lat_modification_info =
        FindOrNull(*input.lat_modification_info_map, *traj_id);
    if (lat_modification_info == nullptr) return std::nullopt;
    res = ModifyStBoundaryViaLatModificationInfo(
        *input.st_graph, st_boundary_wd, *traj, *lat_modification_info,
        input.path->length());
    if (res.has_value()) {
      return res;
    }
  }

  // TODO: Implement other pre st-boundary modification functions.
  return res;
}

std::optional<StBoundaryModificationResult> ModifyStBoundaryForCipv(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd) {
  CHECK_NOTNULL(input.leading_objs);
  std::optional<StBoundaryModificationResult> res;
  const StBoundary& st_boundary =
      *CHECK_NOTNULL(st_boundary_wd.raw_st_boundary());
  if (!st_boundary.object_id().has_value() || st_boundary.is_protective() ||
      st_boundary.min_t() > 0.0 ||
      st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) {
    return res;
  }
  const auto& traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());
  const SpacetimeObjectTrajectory& st_traj =
      *CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));
  if (st_traj.is_stationary() || st_traj.planner_object().pose().v() < 0.0) {
    return res;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const double first_overlap_obj_heading =
      st_traj.states()[first_overlap_info.obj_idx].box.heading();
  const double first_overlap_av_start_heading =
      (*input.path)[first_overlap_info.av_start_idx].theta();
  const double first_overlap_av_end_heading =
      (*input.path)[first_overlap_info.av_end_idx].theta();
  const double first_overlap_av_middle_heading =
      (*input.path)[(first_overlap_info.av_start_idx +
                     first_overlap_info.av_end_idx) /
                    2]
          .theta();
  constexpr double kParallelThreshold = d2r(10.0);
  bool is_obj_heading_in_av_heading = ad_byd::planning::math::IsAngleBetween(
      first_overlap_obj_heading, first_overlap_av_start_heading,
      first_overlap_av_middle_heading, first_overlap_av_end_heading,
      kParallelThreshold);
  const auto& object_id = *st_boundary.object_id();
  const auto& cipv_obj_id = input.cipv_object_info->nearest_object_id;
  const auto& cipv_stay_obj_id = input.cipv_object_info->nearest_stay_object_id;
  const bool is_cipv =
      (cipv_stay_obj_id.has_value() && object_id == *cipv_stay_obj_id) ||
      (cipv_obj_id.has_value() && object_id == *cipv_obj_id &&
       is_obj_heading_in_av_heading);
  if (!is_cipv) return res;
  res = ModifyCipvStBoundary(*input.st_graph, input.drive_passage,
                             input.path_sl_boundary, st_boundary_wd, st_traj,
                             input.current_v, *input.leading_objs,
                             input.path->length());
  return res;
}

std::optional<StBoundaryModificationResult> PreModifyStboundarySequence(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd,
    OpenLoopSpeedLimit* open_loop_speed_limit) {
  std::optional<StBoundaryModificationResult> res;
  if (res = ModifyStBoundaryForOncoming(input, st_boundary_wd);
      res.has_value()) {
    return res;
  }
  if (res = ModifyStBoundaryForRightTurn(input, st_boundary_wd,
                                         open_loop_speed_limit);
      res.has_value()) {
    return res;
  }
  if (res = ModifyStBoundaryForCipv(input, st_boundary_wd); res.has_value()) {
    return res;
  }
  return res;
}

}  // namespace

void PreModifyStBoundaries(
    const PreStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    OpenLoopSpeedLimit* open_loop_speed_limit) {
  CHECK_NOTNULL(input.st_graph);
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.path);
  CHECK_NOTNULL(input.cipv_object_info);

  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      pre_processed_st_objects;

  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PreStboundaryModifierInput&, const StBoundaryWithDecision&,
          OpenLoopSpeedLimit*)>(PreModifyStboundarySequence),
      &pre_processed_st_objects, st_boundaries_wd, open_loop_speed_limit);

  // Merge newly processed trajectories with the original ones.
  for (auto& [traj_id, traj] : pre_processed_st_objects) {
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }
}

}  // namespace planning
}  // namespace st
