

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "planner/planner_manager/min_length_path_extension.h"
#include "planner/planner_manager/planner_defs.h"
#include "plan_common/util/path_util.h"

DEFINE_double(zigzag_start_end_point_dist, 1.0,
              "If we find a zigzag path in whole path, we need to remove the "
              "zigzag range. The start and end points of the path should not "
              "be too far, if so, path after disposing will be acceptable.");

namespace st {
namespace planning {

std::vector<TrajectoryPoint> GetExtendStateByPurePursuit(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    double trajectory_time_step, const TrajectoryPoint& extend_start_point,
    int k_extend_steps, double target_v) {
  CHECK_GT(trajectory_time_step, 0.0);
  constexpr double kLateralLookAhead = 1.0;
  TrajectoryPoint tj_tmp = extend_start_point;
  std::vector<TrajectoryPoint> res;
  const double time_base = extend_start_point.t();
  while (res.size() < k_extend_steps) {
    const double tmp_a =
        std::clamp<double>((target_v - tj_tmp.v()) / trajectory_time_step,
                           motion_constraint_params.max_deceleration(),
                           motion_constraint_params.max_acceleration());
    const double j =
        std::clamp<double>((tmp_a - tj_tmp.a()) / trajectory_time_step,
                           motion_constraint_params.max_decel_jerk(),
                           motion_constraint_params.max_accel_jerk());
    Vec2d lateral_target_pos = drive_passage.stations().back().xy();
    const double lateral_look_ahead_dist =
        kLateralLookAhead * tj_tmp.v() + veh_geo_params.wheel_base();
    const auto lateral_nearest_point_status =
        drive_passage.QueryFrenetCoordinateAt(tj_tmp.pos());
    if (lateral_nearest_point_status.ok()) {
      lateral_target_pos = path_sl_boundary.QueryReferenceCenterXY(
          lateral_look_ahead_dist + lateral_nearest_point_status->s);
    }
    const double alpha =
        Vec2d(lateral_target_pos - tj_tmp.pos()).Angle() - tj_tmp.theta();
    const double tmp_kappa = 2.0 * std::sin(alpha) / lateral_look_ahead_dist;
    const double tmp_psi = (tmp_kappa - tj_tmp.kappa()) / trajectory_time_step;

    const double chi = (tmp_psi - tj_tmp.psi()) / trajectory_time_step;
    const double theta = tj_tmp.theta();
    const double v = tj_tmp.v();
    const double kappa = tj_tmp.kappa();
    const double a = tj_tmp.a();
    const double s = tj_tmp.s();
    const double psi = tj_tmp.psi();

    const double half_da = 0.5 * j * trajectory_time_step;
    const double halfway_a = a + half_da;
    const double half_dv = 0.5 * halfway_a * trajectory_time_step;
    const double halfway_v = v + half_dv;
    const double half_dpsi = 0.5 * chi * trajectory_time_step;
    const double halfway_psi = psi + half_dpsi;
    const double half_dkappa = 0.5 * psi * trajectory_time_step +
                               0.25 * chi * Sqr(trajectory_time_step);
    const double halfway_kappa = kappa + half_dkappa;
    const double half_dtheta =
        0.5 * halfway_v * halfway_kappa * trajectory_time_step;
    const double halfway_theta = theta + half_dtheta;
    const double cos_halfway_theta = std::cos(halfway_theta);
    const double sin_halfway_theta = std::sin(halfway_theta);

    constexpr double kMaxValue = 1e50;
    const double next_pos_x = std::clamp(
        tj_tmp.pos().x() + halfway_v * cos_halfway_theta * trajectory_time_step,
        -kMaxValue, kMaxValue);
    const double next_pos_y = std::clamp(
        tj_tmp.pos().y() + halfway_v * sin_halfway_theta * trajectory_time_step,
        -kMaxValue, kMaxValue);
    const double next_theta =
        std::clamp(halfway_theta + half_dtheta, -kMaxValue, kMaxValue);
    const double next_v =
        std::clamp(halfway_v + half_dv, -kMaxValue, kMaxValue);
    const double next_kappa =
        std::clamp(halfway_kappa + half_dkappa, -kMaxValue, kMaxValue);
    const double next_psi =
        std::clamp(halfway_psi + half_dpsi, -kMaxValue, kMaxValue);
    const double next_a =
        std::clamp(halfway_a + half_da, -kMaxValue, kMaxValue);
    const double next_s = std::clamp(s + v * trajectory_time_step +
                                         0.5 * a * Sqr(trajectory_time_step) +
                                         j * Cube(trajectory_time_step) / 6.0,
                                     -kMaxValue, kMaxValue);
    TrajectoryPoint next_pt;
    next_pt.set_pos(Vec2d(next_pos_x, next_pos_y));
    next_pt.set_theta(next_theta);
    next_pt.set_kappa(next_kappa);
    next_pt.set_psi(next_psi);
    next_pt.set_v(next_v);
    next_pt.set_a(next_a);
    next_pt.set_s(next_s);
    next_pt.set_t((res.empty() ? time_base : res.back().t()) +
                  trajectory_time_step);
    tj_tmp = next_pt;
    res.push_back(std::move(next_pt));
  }
  return res;
}

absl::StatusOr<std::vector<PathPoint>> ExtendPathAndDeleteUnreasonablePart(
    const st::planning::DrivePassage& drive_passage,
    const st::planning::PathSlBoundary& path_sl_boundary,
    const st::MotionConstraintParamsProto& motion_constraint_params,
    const st::VehicleGeometryParamsProto& vehicle_geom_params,
    absl::Span<const ApolloTrajectoryPointProto> trajectory_points,
    double required_min_length, double max_curvature) {
  if (trajectory_points.empty()) {
    return absl::InternalError(absl::StrFormat(
        "Trajectory input is empty: num(%fm)", trajectory_points.size()));
  }
  std::vector<PathPoint> raw_path_points;
  raw_path_points.reserve(trajectory_points.size());
  for (const auto& pt : trajectory_points) {
    raw_path_points.push_back(pt.path_point());
  }
  // Delete Zigzag path.
  // Traversal path, if we find that next point's arc length is less than
  // current, we start to find the point pt_next whose s is larger than current
  // point and has a minimum positive projection along path, then remove points
  // between current point and pt_next. Repeat above steps.
  int index = 1;
  constexpr double kEpsilon = 1e-6;
  while (index < raw_path_points.size()) {
    if (raw_path_points[index].s() < raw_path_points[index - 1].s()) {
      // Find next s > current_s
      const PathPoint& current_path_point = raw_path_points[index - 1];
      // Add epsilon to avoid problem cause by numerical fault when integration.
      const double current_s = current_path_point.s() + kEpsilon;
      const Vec2d current_tangent =
          Vec2d::UnitFromAngle(current_path_point.theta());
      for (int i = index; i < raw_path_points.size(); ++i) {
        const Vec2d delta_vec =
            ToVec2d(raw_path_points[i]) - ToVec2d(current_path_point);
        const double projection = delta_vec.dot(current_tangent);
        if (raw_path_points[i].s() > current_s && projection > kEpsilon) {
          const double dist =
              DistanceTo(current_path_point, raw_path_points[i]);
          if (dist < FLAGS_zigzag_start_end_point_dist) {
            // Delete points between index and i;
            raw_path_points.erase(raw_path_points.begin() + index,
                                  raw_path_points.begin() + i);
            // Recompute s.
            const double delta_s =
                dist - (raw_path_points[index].s() - current_path_point.s());
            for (int k = index; k < raw_path_points.size(); ++k) {
              raw_path_points[k].set_s(raw_path_points[k].s() + delta_s);
            }
            break;
          } else {
            return absl::InternalError(absl::StrFormat(
                "Zigzag point dist too large: dist(%fm), "
                "start_index(%d), end_index(%d), projection(%f)",
                dist, index - 1, i, projection));
          }
        } else if (i == (raw_path_points.size() - 1)) {
          raw_path_points.erase(raw_path_points.begin() + index,
                                raw_path_points.end());
          break;
        }
      }
    }
    ++index;
  }
  // Recompute s.
  raw_path_points.begin()->set_s(0.0);
  for (int index = 1; index < raw_path_points.size(); ++index) {
    const double d =
        DistanceTo(raw_path_points[index - 1], raw_path_points[index]);
    raw_path_points[index].set_s(raw_path_points[index - 1].s() + d);
  }

  // Delete the part that exceeds curvature limit.
  std::optional<double> end_kappa = std::nullopt;
  auto init_point = raw_path_points.front();
  for (auto iter = raw_path_points.begin(); iter < raw_path_points.end();
       ++iter) {
    if (std::abs(iter->kappa()) > max_curvature) {
      raw_path_points.erase(iter, raw_path_points.end());
      // This is a hack to compute sign of end_kappa, because input path may
      // exceed -max_curvature firstly and exceed max_curvature later.
      // TODO: Solve such problem in optimization.
      double sum_kappa = 0.0;
      for (int i = std::distance(raw_path_points.begin(), iter);
           i < raw_path_points.size(); ++i) {
        if (std::abs(raw_path_points[i].kappa()) > max_curvature) {
          sum_kappa += raw_path_points[i].kappa();
        }
      }
      end_kappa = std::copysign(max_curvature, sum_kappa);
      break;
    }
  }
  if (raw_path_points.empty()) {
    raw_path_points.push_back(std::move(init_point));
  }
  if (end_kappa.has_value()) {
    // Add one more point with max kappa to the end of the path.
    auto last_pt = raw_path_points.back();
    last_pt.set_kappa((last_pt.kappa() + *end_kappa) * 0.5);
    raw_path_points.push_back(
        GetPathPointAlongCircle(last_pt, kPathSampleInterval));
    // Set end point kappa to the max.
    raw_path_points.back().set_kappa(*end_kappa);
  }

  // Extend raw path if it is too short.
  const double v_now = trajectory_points.front().v();
  constexpr double kDecel = 2.0;  // m/s^2.
  const double min_length =
      std::max(required_min_length, 0.5 * Sqr(v_now) / kDecel);
  constexpr double kDisEpsilon = 1e-2;
  const bool has_cutoff =
      Vec2d(raw_path_points.back().x(), raw_path_points.back().y())
          .DistanceTo(Vec2d(trajectory_points.back().path_point().x(),
                            trajectory_points.back().path_point().y())) >
      kDisEpsilon;
  const TrajectoryPoint traj_end(
      ToTrajectoryPointProto(trajectory_points.back()));
  const auto traj_end_sl_status =
      drive_passage.QueryFrenetCoordinateAt(traj_end.pos());
  if (has_cutoff || !traj_end_sl_status.ok() ||
      (traj_end_sl_status->s > path_sl_boundary.end_s())) {
    while (raw_path_points.back().s() < min_length) {
      PathPoint p =
          GetPathPointAlongCircle(raw_path_points.back(), kPathSampleInterval);
      raw_path_points.push_back(std::move(p));
    }
  } else {
    constexpr double kTargetTrajectoryTimeStep = 0.2;
    constexpr double kMinEndSpeed = 3.1;
    const double real_ds =
        std::min(min_length - raw_path_points.back().s(),
                 path_sl_boundary.end_s() - traj_end_sl_status->s);
    const double target_v = std::max(kMinEndSpeed, traj_end.v());
    const int real_dsteps = static_cast<int>(
        std::ceil(real_ds / target_v / kTargetTrajectoryTimeStep));
    const auto extend_state = GetExtendStateByPurePursuit(
        drive_passage, path_sl_boundary, vehicle_geom_params,
        motion_constraint_params, kTargetTrajectoryTimeStep, traj_end,
        real_dsteps, target_v);
    for (int k = 0; k < extend_state.size(); ++k) {
      PathPoint tmp;
      tmp.set_x(extend_state[k].pos().x());
      tmp.set_y(extend_state[k].pos().y());
      tmp.set_theta(extend_state[k].theta());
      tmp.set_s(extend_state[k].s());
      tmp.set_kappa(extend_state[k].kappa());
      tmp.set_lambda(0.0);
      raw_path_points.push_back(std::move(tmp));
    }
  }

  return raw_path_points;
}

}  // namespace planning
}  // namespace st
