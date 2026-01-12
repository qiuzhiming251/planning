

#include "planner/speed_optimizer/interactive_laterally.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

// IWYU pragma: no_include "Eigen/Core"

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"
//#include "global/logging.h"
//#include "lite/check.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/planner_object.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "plan_common/maps/overlap_info.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "plan_common/maps/st_boundary.h"

DEFINE_bool(enable_planner_draw_nudge_path_point, false,
            "Whether to enable planner draw nudge path point.");

namespace st::planning {
namespace {
constexpr double kEps = 1e-6;
constexpr double kMaxNudgeOffset = 1.0;  // m.

void DrawNudgePoint(const SpacetimeObjectTrajectory& spacetime_obj,
                    const NudgeVector& nudge_vector) {
  CHECK_GT(spacetime_obj.states().size(), nudge_vector.leverage_point_index);

  const auto* traj_point =
      spacetime_obj.states()[nudge_vector.leverage_point_index].traj_point;
  Vec2d nudge_direction;
  if (nudge_vector.direction == NudgeVector::Direction::RIGHT) {
    nudge_direction =
        Vec2d::FastUnitFromAngle(NormalizeAngle(traj_point->theta() - M_PI_2));
  } else {
    nudge_direction =
        Vec2d::FastUnitFromAngle(NormalizeAngle(traj_point->theta() + M_PI_2));
  }
  const Vec2d nudge_pos =
      traj_point->pos() + nudge_direction * nudge_vector.offset;

  PathPoint nudge_point;
  nudge_point.set_x(nudge_pos.x());
  nudge_point.set_y(nudge_pos.y());
  nudge_point.set_theta(traj_point->theta());

  // speed::DrawPathPoint(nudge_point, spacetime_obj.traj_id(),
  //                      vis::Color::kOrange);
}

PiecewiseLinearFunction<double> GenerateKinematicNudgeOffsetUpperBound(
    const VehicleGeometryParamsProto& vehicle_gem, double current_v,
    double current_a, const PathPoint& current_path_point) {
  const auto av_pos = Vec2d(current_path_point.x(), current_path_point.y());
  const double av_theta = current_path_point.theta();
  const double av_v = current_v;
  const auto av_heading = Vec2d::FastUnitFromAngle(av_theta);
  const double av_accel = current_a;  // m/ss.

  constexpr double kDt = 0.2;                  // s.
  constexpr double kSteeringAngleRate = 0.15;  // r/s.
  constexpr double kMaxSteeringAngle = 0.57;   // r.
  constexpr int kMaxStatesNum = 15;

  auto pos = av_pos;
  double theta = av_theta;
  // Assume steering angle is zero before nudge.
  double steering_angle = 0.0;
  double v = av_v;

  std::vector<double> relative_s;
  relative_s.reserve(kMaxStatesNum);
  std::vector<double> nudge_offset;
  nudge_offset.reserve(kMaxStatesNum);
  for (int i = 0; i < kMaxStatesNum; ++i) {
    // Approximate to path s in a short time.
    const double s = (pos - av_pos).dot(av_heading);
    relative_s.push_back(s);
    const double offset = std::abs((pos - av_pos).CrossProd(av_heading));
    nudge_offset.push_back(std::min(offset, kMaxNudgeOffset));
    VLOG(3) << "s: " << s
            << ", kinematic offset: " << std::min(offset, kMaxNudgeOffset);
    if (offset > kMaxNudgeOffset) break;

    // Kinematic bicycle model.
    const Vec2d xy_dot = v * Vec2d::FastUnitFromAngle(theta);
    const double theta_dot =
        v * std::tan(steering_angle) / vehicle_gem.wheel_base();
    const double delta_dot = kSteeringAngleRate;
    pos += xy_dot * kDt;
    theta += theta_dot * kDt;
    steering_angle += delta_dot * kDt;
    if (steering_angle > kMaxSteeringAngle) {
      steering_angle = kMaxSteeringAngle;
    }
    v += av_accel * kDt;
    if (v <= 0.0) {
      break;
    }
  }

  return PiecewiseLinearFunction<double>(relative_s, nudge_offset);
}

absl::StatusOr<NudgeVector> GenerateNudgeVector(
    const SpacetimeObjectTrajectory& spacetime_obj, const DiscretizedPath& path,
    const OverlapInfo& overlap_info,
    const PiecewiseLinearFunction<double>& kinematic_upper_bound_plf) {
  // AV infos.
  // Use mid idx of av_start_idx and av_end_idx.
  const int overlap_av_idx =
      (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;
  CHECK_GT(path.size(), overlap_av_idx);
  const auto& overlap_av_path_point = path[overlap_av_idx];

  // Agent infos.
  const int overlap_agent_idx = overlap_info.obj_idx;
  CHECK_GT(spacetime_obj.states().size(), overlap_agent_idx);
  const auto* overlap_agent_traj_point =
      spacetime_obj.states()[overlap_agent_idx].traj_point;

  const bool is_right_cutin =
      NormalizeAngle(overlap_agent_traj_point->theta() -
                     overlap_av_path_point.theta()) > 0.0;
  const NudgeVector::Direction nudge_direction =
      is_right_cutin ? NudgeVector::Direction::RIGHT
                     : NudgeVector::Direction::LEFT;

  const PiecewiseLinearFunction<double> nudge_offset_plf(
      {1.0, 5.0}, {kMaxNudgeOffset, 0.4});

  const double agent_v = spacetime_obj.planner_object().pose().v();
  const double nudge_offset =
      std::min(nudge_offset_plf(agent_v),
               kinematic_upper_bound_plf(overlap_agent_traj_point->s()));

  VLOG(2) << "agent_v: " << agent_v << ", nudge_offset: " << nudge_offset
          << ", nudge point s: " << overlap_agent_traj_point->s();

  return NudgeVector{.leverage_point_index = overlap_agent_idx,
                     .direction = nudge_direction,
                     .offset = nudge_offset};
}

void PrintLatModifyObjInfos(
    const StBoundaryWithDecision& boundary_with_decision) {
  CHECK(boundary_with_decision.st_boundary()->overlap_meta().has_value());
  const auto& overlap_meta =
      *boundary_with_decision.st_boundary()->overlap_meta();
  VLOG(2) << "Lat modify st-boundary: " << boundary_with_decision.id()
          << ", object type: "
          << StBoundaryProto::ObjectType_Name(
                 boundary_with_decision.st_boundary()->object_type())
          << ", overlap pattern: "
          << StOverlapMetaProto::OverlapPattern_Name(overlap_meta.pattern())
          << ", overlap source: "
          << StOverlapMetaProto::OverlapSource_Name(overlap_meta.source())
          << ", modification type: "
          << StOverlapMetaProto::ModificationType_Name(
                 overlap_meta.modification_type())
          << ", priority: "
          << StOverlapMetaProto::OverlapPriority_Name(overlap_meta.priority());
}

// Assume that av keeps driving at a constant speed at the current speed. Check
// whether av need brake hardly to yield agent. Calculation method is almost the
// same like function `CalculateYieldingResultIfAgentYieldAV` in
// `interactive_speed_decision.cc`, but roles exchanged.
bool ShouldAvBrakeHardToYieldAgent(
    double current_v, const PathPoint& overlap_av_path_point,
    const prediction::PredictedTrajectoryPoint& overlap_agent_traj_point) {
  const double av_overlap_s = overlap_av_path_point.s();
  const double av_v = current_v;

  const double av_follow_v =
      overlap_agent_traj_point.v() * std::cos(overlap_agent_traj_point.theta() -
                                              overlap_av_path_point.theta());

  // We have two reasons to tune follow distance params to be aggressive:
  // 1. We have right of way.
  // 2. We should modify agent path cautiously.
  constexpr double kStandoffDistance = 3.0;  // m.
  constexpr double kStandoffTime = 0.5;      // s.
  const double av_decel_dist =
      av_overlap_s - kStandoffDistance - av_follow_v * kStandoffTime;
  if (av_decel_dist <= 0.0) {
    VLOG(2) << "Av has no enough distance to brake.";
    return true;
  }

  const double av_acc_decel =
      (Sqr(av_follow_v) - Sqr(av_v)) / av_decel_dist * 0.5;
  const double av_braking_to_stop_decel = -Sqr(av_v) / av_decel_dist * 0.5;
  const double t_stop = -av_v / av_braking_to_stop_decel;
  const double t_decel_to_acc = (av_follow_v - av_v) / av_acc_decel;
  const double t_yield = overlap_agent_traj_point.t();

  double av_yield_agent_decel = 0.0;
  double av_waiting_duration = 0.0;
  if (t_yield <= t_decel_to_acc) {
    av_yield_agent_decel = av_acc_decel;
  } else if (t_yield > t_decel_to_acc && t_yield < t_stop) {
    av_yield_agent_decel =
        2.0 * (av_decel_dist - av_v * t_yield) / Sqr(t_yield);
  } else {
    av_yield_agent_decel = av_braking_to_stop_decel;
    av_waiting_duration = t_yield - t_stop;
  }

  VLOG(2) << "av_overlap_s: " << av_overlap_s
          << ", av_follow_v: " << av_follow_v << ", av_v: " << av_v
          << ", av_decel_distance: " << av_decel_dist
          << ", av_yield_agent_decel: " << av_yield_agent_decel;

  constexpr double kHardBrakeDecelValue = -3.0;  // m/ss.
  return av_yield_agent_decel < kHardBrakeDecelValue ||
         av_waiting_duration > kEps;
}

bool CanAvSufficientlyLeadAgent(
    const PathPoint& overlap_av_path_point,
    const prediction::PredictedTrajectoryPoint& overlap_agent_traj_point,
    double current_v) {
  const double av_overlap_s = overlap_av_path_point.s();
  const double av_v = current_v;
  // Estimate av overlap time.
  const double av_overlap_time = av_overlap_s / av_v;
  const double agent_overlap_time = overlap_agent_traj_point.t();

  VLOG(2) << "agent_overlap_time: " << agent_overlap_time
          << ", av_overlap_time: " << av_overlap_time;

  constexpr double kSufficientlyLeadTime = 2.0;  // s.
  return agent_overlap_time - av_overlap_time > kSufficientlyLeadTime;
}

absl::flat_hash_set<std::string> FilterNonLatInteractiveObjs(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const DiscretizedPath& path, double current_v) {
  absl::flat_hash_set<std::string> non_lat_interactive_ids;
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    VLOG(3) << "Filter: " << boundary_with_decision.id();
    if (!boundary_with_decision.st_boundary()->overlap_meta().has_value()) {
      VLOG(3) << "No overlap meta data.";
      non_lat_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    const auto& overlap_meta =
        *boundary_with_decision.st_boundary()->overlap_meta();
    if (boundary_with_decision.decision_type() != StBoundaryProto::UNKNOWN) {
      VLOG(3) << "Pre-decision is: "
              << StBoundaryProto::DecisionType_Name(
                     boundary_with_decision.decision_type());
      non_lat_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    if (overlap_meta.modification_type() !=
        StOverlapMetaProto::LON_LAT_MODIFIABLE) {
      VLOG(3) << "Lateral modification not allowed.";
      non_lat_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    const auto* st_boundary = boundary_with_decision.st_boundary();
    const auto& first_overlap_info = st_boundary->overlap_infos().front();
    // Av infos.
    const int overlap_av_idx =
        (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2;
    CHECK_GT(path.size(), overlap_av_idx);
    const auto& overlap_av_path_point = path[overlap_av_idx];
    // Agent infos.
    const auto traj_id = boundary_with_decision.traj_id();
    CHECK(traj_id.has_value());
    const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
    const int overlap_agent_idx = first_overlap_info.obj_idx;
    CHECK_GT(spacetime_obj->states().size(), overlap_agent_idx);
    const auto* overlap_agent_traj_point =
        spacetime_obj->states()[overlap_agent_idx].traj_point;

    if (!ShouldAvBrakeHardToYieldAgent(current_v, overlap_av_path_point,
                                       *overlap_agent_traj_point)) {
      VLOG(3) << "Av can yield agent without hard brake, not modify laterally.";
      non_lat_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    if (CanAvSufficientlyLeadAgent(overlap_av_path_point,
                                   *overlap_agent_traj_point, current_v)) {
      VLOG(3) << "Av can sufficiently lead agent, not modify laterally.";
      non_lat_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }
  }

  return non_lat_interactive_ids;
}
}  // namespace

absl::flat_hash_map<std::string, StBoundaryLatModificationInfo>
GenerateLatModificationInfo(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, double current_a,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision) {
  // VLOG(2) << ansi::cyan << "-----------interact laterally-----------"
  //         << ansi::reset;

  absl::flat_hash_set<std::string> non_lat_interactive_ids =
      FilterNonLatInteractiveObjs(st_traj_mgr, st_boundaries_with_decision,
                                  path, current_v);

  const auto kinematic_upper_bound_plf = GenerateKinematicNudgeOffsetUpperBound(
      vehicle_geom, current_v, current_a, path.front());

  absl::flat_hash_map<std::string, StBoundaryLatModificationInfo>
      lat_modification_info;
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    if (non_lat_interactive_ids.find(boundary_with_decision.id()) !=
        non_lat_interactive_ids.end()) {
      continue;
    }

    PrintLatModifyObjInfos(boundary_with_decision);

    const auto* st_boundary = boundary_with_decision.st_boundary();
    const auto traj_id = boundary_with_decision.traj_id();
    CHECK(traj_id.has_value());
    const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
    const auto& first_overlap_info = st_boundary->overlap_infos().front();
    const auto nudge_vector = GenerateNudgeVector(
        *spacetime_obj, path, first_overlap_info, kinematic_upper_bound_plf);
    if (!nudge_vector.ok()) continue;

    if (FLAGS_enable_planner_draw_nudge_path_point) {
      DrawNudgePoint(*spacetime_obj, *nudge_vector);
    }

    lat_modification_info[*traj_id] = {
        .modifier_type = StBoundaryModifierProto::LAT_INTERACTIVE,
        .nudge_vector = *nudge_vector};
  }

  return lat_modification_info;
}

}  // namespace st::planning
