

#include "decider/scheduler/path_boundary_builder_helper.h"
#include "plan_common/util/vehicle_geometry_util.h"

// IWYU pragma: no_include <memory>
// IWYU pragma: no_include <type_traits>

#include <cmath>
#include <limits>
#include <optional>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "plan_common/log_data.h"
#include "plan_common/vehicle_kinematic.h"
#include "gflags/gflags.h"
//#include "global/logging.h"
//#include "global/trace.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/object_history.h"
#include "plan_common/plan_common_defs.h"
//#include "semantic_map.pb.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/status_macros.h"

namespace st::planning {
namespace {

constexpr double kObjectBuffer = 0.55;            // meters.
constexpr double kVirtualStationHalfWidth = 3.0;  // meters.
constexpr double kBackWardTimeBuffer = 1.0;       // s.
constexpr double kFrontTimeBuffer = 1.5;          // s.
constexpr double kPrePursuitBuffer = 3.0;         // s.
constexpr double kPostPursuitBuffer = 4.0;        // s.

constexpr double kBorrowLaneOffset = 3.9;      // m.
constexpr double kEgoLatBuffer = 0.3;          // m.
constexpr double kLcPauseEgoLatBuffer = 0.75;  // m.

constexpr double kComfortLaneChangeCancelLatAccel = 0.25;  // m/s^2.
constexpr double kMaxComfortLatJerk = 1.0;                 // m/s^3.
constexpr int kAvTrajPointsMinSize = 5;
constexpr double kMinKinematicTrajLonSpeed = 3.0;            // m/s.
constexpr double kMinKinematicBoundaryProtectedZone = 15.0;  // m.

constexpr double kRiskMaxLaneChangePauseRefCenterStep = 1.4;  // m.
constexpr double kRiskMinLaneChangePauseRefCenterStep = 1.0;  // m.
constexpr double kMaxLaneChangePauseRefCenterStep = 0.7;      // m.

constexpr double kIntersectionTurningMaxHalfLaneWidth = 3.5;  // m.

std::optional<double> FindPursuitTime(absl::Span<const double> av_s_vec,
                                      absl::Span<const double> av_t_vec,
                                      absl::Span<const double> obj_s_vec,
                                      absl::Span<const double> obj_t_vec,
                                      const double& obj_length) {
  CHECK_EQ(av_s_vec.size(), av_t_vec.size());
  CHECK_EQ(obj_s_vec.size(), obj_t_vec.size());
  const double kRearToRearAxle = 1.15, kFrontToRearAxle = 4.10;
  const double half_obj_length = 0.5 * obj_length;

  for (int i = 0; i < av_t_vec.size() - 1; ++i) {
    for (int j = 0; j < obj_t_vec.size() - 1; ++j) {
      // No time overlap, ignore.
      if (av_t_vec[i + 1] < obj_t_vec[j] || av_t_vec[i] > obj_t_vec[j + 1]) {
        continue;
      }
      const Segment2d av_s_t(Vec2d(av_t_vec[i], av_s_vec[i]),
                             Vec2d(av_t_vec[i + 1], av_s_vec[i + 1]));
      Vec2d intersect_pt;
      const bool has_intersect = av_s_t.GetIntersect(
          Segment2d(Vec2d(obj_t_vec[j], obj_s_vec[j]),
                    Vec2d(obj_t_vec[j + 1], obj_s_vec[j + 1])),
          &intersect_pt);
      if (has_intersect) {
        return intersect_pt.x();
      } else {
        if (std::fmin(av_s_vec[i] + kFrontToRearAxle,
                      obj_s_vec[j] + half_obj_length) >
            std::fmax(av_s_vec[i] - kRearToRearAxle,
                      obj_s_vec[j] - half_obj_length)) {
          return av_t_vec[i];
        } else if (std::fmin(av_s_vec[i + 1] + kFrontToRearAxle,
                             obj_s_vec[j + 1] + half_obj_length) >
                   std::fmax(av_s_vec[i + 1] - kRearToRearAxle,
                             obj_s_vec[j + 1] - half_obj_length)) {
          return av_t_vec[i + 1];
        }
      }
    }
  }
  return std::nullopt;
}

absl::StatusOr<PiecewiseLinearFunction<double, double>>
GenerateAvTrajAlongRefCenterLine(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    absl::Span<const Vec2d> center_xy_vec) {
  ASSIGN_OR_RETURN(const auto center_frame,
                   BuildBruteForceFrenetFrame(center_xy_vec,
                                              /*down_sample_raw_points=*/true));
  const auto cur_sl = center_frame.XYToSL(
      Vec2dFromApolloTrajectoryPointProto(plan_start_point));

  std::vector<double> vec_t, vec_s;
  vec_t.reserve(kTrajectorySteps);
  vec_s.reserve(kTrajectorySteps);

  double center_s = cur_sl.s;
  const double speed = plan_start_point.v();
  for (double t = 0; t <= kTrajectoryTimeHorizon; t += kTrajectoryTimeStep) {
    const auto center_xy = center_frame.SLToXY({center_s, 0.0});
    const auto dp_sl = drive_passage.QueryFrenetCoordinateAt(center_xy);
    if (!dp_sl.ok()) break;

    vec_t.push_back(t);
    vec_s.push_back(dp_sl->s);
    center_s += kTrajectoryTimeStep * speed;
  }

  if (vec_s.size() > 1) {
    return PiecewiseLinearFunction(vec_s, vec_t);
  } else {
    return absl::InternalError("");
  }
}

absl::StatusOr<PiecewiseLinearFunction<double, double>>
GenerateConstLateralAccelConstSpeedTraj(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const FrenetCoordinate& cur_sl, double target_lane_offset,
    double max_lat_accel) {
  ASSIGN_OR_RETURN(const auto lane_tangent,
                   drive_passage.QueryTangentAtS(cur_sl.s),
                   _ << "Unable to find lane tangent at current s.");

  std::vector<double> vec_l, vec_s;
  vec_l.reserve(kTrajectorySteps);
  vec_s.reserve(kTrajectorySteps);

  constexpr double kReachTargetLaneThreshold = 0.1;  // m.
  constexpr double kZeroLateralVelThreshold = 0.01;  // m/s.
  constexpr double kDt = kTrajectoryTimeStep;
  const auto heading_tangent =
      Vec2d::FastUnitFromAngle(plan_start_point.path_point().theta());
  double s, l, lat_v, lat_a, speed;
  s = cur_sl.s;
  l = cur_sl.l;
  speed = plan_start_point.v();
  lat_v = speed * lane_tangent.CrossProd(heading_tangent);
  lat_a = plan_start_point.a() * lane_tangent.CrossProd(heading_tangent) +
          speed * lane_tangent.Dot(heading_tangent) * speed *
              plan_start_point.path_point().kappa();
  lat_v = std::fabs(lat_v) < kZeroLateralVelThreshold ? 0.0 : lat_v;
  speed = std::max(speed, kMinKinematicTrajLonSpeed);
  vec_l.push_back(l);
  vec_s.push_back(s);

  const double expect_lat_a =
      -std::copysign(max_lat_accel, l - target_lane_offset);

  // Assume const lateral jerk until lateral acceleration reaches expected const
  // accel value.
  const int const_lat_jerk_steps =
      FloorToInt(std::abs(expect_lat_a - lat_a) / kMaxComfortLatJerk / kDt);
  for (int i = 0; i < const_lat_jerk_steps; ++i) {
    l += lat_v * kDt;
    s += speed * kDt;
    lat_v += lat_a * kDt;
    lat_a += std::copysign(kMaxComfortLatJerk, expect_lat_a - lat_a) * kDt;
    vec_l.push_back(l);
    vec_s.push_back(s);
  }
  // Assume const lateral acceleration for the rest of trajectory.
  lat_a = -std::copysign(max_lat_accel, l - target_lane_offset);
  for (int i = 0; i < kTrajectorySteps - const_lat_jerk_steps - 1; ++i) {
    l += lat_v * kDt;
    s += speed * kDt;
    lat_v += lat_a * kDt;

    if ((std::fabs(l - target_lane_offset) < kReachTargetLaneThreshold &&
         std::fabs(lat_v) < kZeroLateralVelThreshold) ||
        (l - target_lane_offset) * (vec_l.back() - target_lane_offset) < 0.0) {
      break;
    }
    vec_l.push_back(l);
    vec_s.push_back(s);
  }

  if (vec_s.size() < kAvTrajPointsMinSize) {
    return absl::NotFoundError("");
  }

  const int extend_traj_size = CeilToInt(vec_s.size() * 0.1);
  for (int i = 0; i < extend_traj_size; ++i) {
    if (vec_s.size() >= kTrajectorySteps) break;
    vec_l.push_back(target_lane_offset);
    vec_s.push_back(s);
    s += speed * kDt;
  }
  return PiecewiseLinearFunction(vec_s, vec_l);
}

PathBoundary BuildBoundaryForStationaryObject(const DrivePassage& drive_passage,
                                              absl::Span<const double> center_l,
                                              const FrenetBox& obj_fbox,
                                              absl::Span<const double> s_vec,
                                              PathBoundary boundary) {
  const int n = boundary.size();
  std::vector<double> left_bound_vec, right_bound_vec;
  left_bound_vec.reserve(n);
  right_bound_vec.reserve(n);

  auto index = std::lower_bound(s_vec.begin(), s_vec.end(), obj_fbox.s_max) -
               s_vec.begin();
  if (index > n - 1) {
    index = n - 1;
  }
  auto index_near = index;
  auto index_far = index_near;
  while (index_near > 0 && s_vec[index_near] >= obj_fbox.s_min) {
    --index_near;
  }
  while (index_far < n - 1 && s_vec[index_far] <= obj_fbox.s_max) {
    ++index_far;
  }

  if (obj_fbox.l_max < center_l[index]) {  // Right side.
    const double lat_offset = obj_fbox.l_max;
    for (int i = index_near; i <= index_far; ++i) {
      boundary.OuterClampRightByIndex(i, lat_offset + kObjectBuffer);
    }
  } else if (obj_fbox.l_min > center_l[index]) {  // Left side.
    const double lat_offset = obj_fbox.l_min;
    for (int i = index_near; i <= index_far; ++i) {
      boundary.OuterClampLeftByIndex(i, lat_offset - kObjectBuffer);
    }
  }

  return boundary;
}

PathBoundary BuildBoundaryForDynamicObject(
    const DrivePassage& drive_passage, absl::Span<const double> center_l,
    const SpacetimeObjectTrajectory& traj,
    const PiecewiseLinearFunction<double, double>& av_t_s,
    absl::Span<const double> s_vec, const PathBoundary& inner_boundary,
    const PathBoundary& curb_boundary, PathBoundary boundary) {
  const auto& av_s_vec = av_t_s.x();
  const auto& av_t_vec = av_t_s.y();
  const int num_stations = s_vec.size();
  const int time_num_steps = traj.states().size();

  absl::flat_hash_map<int, FrenetBox> frenet_box_map;
  std::vector<double> obj_s_vec, obj_t_vec, obj_max_l_vec, obj_min_l_vec;
  obj_s_vec.reserve(time_num_steps);
  obj_t_vec.reserve(time_num_steps);
  obj_max_l_vec.reserve(time_num_steps);
  obj_min_l_vec.reserve(time_num_steps);

  std::vector<Box2d> box_vec;
  box_vec.reserve(time_num_steps);
  for (const auto& state : traj.states()) {
    box_vec.push_back(state.box);
  }
  ASSIGN_OR_RETURN(
      auto fbox_vec,
      drive_passage.BatchQueryFrenetBoxes(box_vec, /*laterally_bounded=*/false),
      boundary);

  for (int i = 0; i < time_num_steps; ++i) {
    if (!fbox_vec[i].has_value()) continue;

    auto fbox = fbox_vec[i].value();

    const auto index =
        std::lower_bound(s_vec.begin(), s_vec.end(), fbox.s_min) -
        s_vec.begin();
    if (index == s_vec.size()) continue;

    // If dynamic object on the target lane or cross the target lane, ignore.
    if (fbox.l_min < center_l[index] && fbox.l_max > center_l[index]) {
      continue;
    }
    obj_s_vec.push_back(0.5 * (fbox.s_max + fbox.s_min));
    obj_t_vec.push_back(i * kTrajectoryTimeStep);
    obj_max_l_vec.push_back(fbox.l_max);
    obj_min_l_vec.push_back(fbox.l_min);

    frenet_box_map[i] = fbox;
  }

  // 1. Calculate a point where the s of AV and object meets (the time which
  // the obj catches up with the AV), at this point AV should not interfer with
  // the object's path (with a time buffer).
  double obj_length = traj.states().front().box.length();
  if (av_t_vec.size() > 1 && obj_t_vec.size() > 1) {
    const auto pursuit_time =
        FindPursuitTime(av_s_vec, av_t_vec, obj_s_vec, obj_t_vec, obj_length);
    // std::string pursuit_info = pursuit_time.has_value()
    //                                ? "pursuit_" +
    //                                absl::StrCat(*pursuit_time) :
    //                                "pursuit_none";
    // Log2DDS::LogDataV2("obj_ignore", pursuit_info);
    if (pursuit_time.has_value()) {
      const PiecewiseLinearFunction obj_max_l_t(obj_t_vec, obj_max_l_vec);
      const PiecewiseLinearFunction obj_min_l_t(obj_t_vec, obj_min_l_vec);
      const int pursuit_idx =
          static_cast<int>(*pursuit_time / kTrajectoryTimeStep);
      const int pursuit_min_idx = std::max(
          0, pursuit_idx -
                 static_cast<int>(kPrePursuitBuffer / kTrajectoryTimeStep));
      const int pursuit_max_idx = std::min(
          time_num_steps - 1,
          pursuit_idx +
              static_cast<int>(kPostPursuitBuffer / kTrajectoryTimeStep));
      for (int i = 0; i < num_stations; ++i) {
        const auto s = s_vec[i];
        if (s < av_s_vec.front() || s > av_s_vec.back()) {
          continue;
        }
        const double av_arrive_t = av_t_s(s);
        const int av_arrive_t_index =
            static_cast<int>(av_arrive_t / kTrajectoryTimeStep);
        if (av_arrive_t_index < 0 || av_arrive_t_index >= time_num_steps ||
            av_arrive_t_index < pursuit_min_idx ||
            av_arrive_t_index > pursuit_max_idx) {
          continue;
        }
        const double pursuit_max_l = obj_max_l_t(av_arrive_t);
        const double pursuit_min_l = obj_min_l_t(av_arrive_t);
        const double pursuit_l_diff = pursuit_max_l - pursuit_min_l;
        if (pursuit_max_l < center_l[i] &&
            std::fabs(curb_boundary.right(i) - inner_boundary.right(i)) >
                pursuit_l_diff) {
          boundary.OuterClampRightByIndex(
              i,
              inner_boundary.right(i) - 0.35 /*pursuit_max_l + kObjectBuffer*/);
        }
        if (pursuit_min_l > center_l[i] &&
            std::fabs(curb_boundary.left(i) - inner_boundary.left(i)) >
                pursuit_l_diff) {
          boundary.OuterClampLeftByIndex(
              i,
              inner_boundary.left(i) + 0.35 /*pursuit_min_l - kObjectBuffer*/);
        }
      }
    }
  }

  // 2. Consider a time buffer zone around the object, AV should not interfer
  // with the object's path within the buffer zone.
  // std::vector<double> vec_min_l, vec_max_l, vec_min_s, vec_max_s;
  // vec_min_l.reserve(time_num_steps);
  // vec_max_l.reserve(time_num_steps);
  // vec_min_s.reserve(time_num_steps);
  // vec_max_s.reserve(time_num_steps);
  // const int backward_buffer =
  //     static_cast<int>(kBackWardTimeBuffer / kTrajectoryTimeStep);
  // const int front_buffer =
  //     static_cast<int>(kFrontTimeBuffer / kTrajectoryTimeStep);
  // for (int i = 0; i < time_num_steps; ++i) {
  //   double min_l = std::numeric_limits<double>::max();
  //   double max_l = std::numeric_limits<double>::lowest();
  //   double min_s = std::numeric_limits<double>::max();
  //   double max_s = std::numeric_limits<double>::lowest();
  //   bool has_value = false;
  //   for (int j = std::max(0, i - backward_buffer);
  //        j < std::min(time_num_steps, i + front_buffer + 1); ++j) {
  //     const auto fbox_ptr = FindOrNull(frenet_box_map, j);
  //     if (fbox_ptr) {
  //       has_value = true;
  //       min_l = std::min(min_l, fbox_ptr->l_min);
  //       max_l = std::max(max_l, fbox_ptr->l_max);
  //       min_s = std::min(min_s, fbox_ptr->s_min);
  //       max_s = std::max(max_s, fbox_ptr->s_max);
  //     }
  //   }
  //   if (has_value) {
  //     vec_min_l.push_back(min_l);
  //     vec_max_l.push_back(max_l);
  //     vec_min_s.push_back(min_s);
  //     vec_max_s.push_back(max_s);
  //   } else {
  //     vec_min_l.push_back(std::numeric_limits<double>::lowest());
  //     vec_max_l.push_back(std::numeric_limits<double>::lowest());
  //     vec_min_s.push_back(std::numeric_limits<double>::lowest());
  //     vec_max_s.push_back(std::numeric_limits<double>::lowest());
  //   }
  // }

  // for (int i = 0; i < num_stations; ++i) {
  //   const auto s = s_vec[i];
  //   if (s < av_s_vec.front() || s > av_s_vec.back()) {
  //     continue;
  //   }
  //   const int av_arrive_t = static_cast<int>(av_t_s(s) /
  //   kTrajectoryTimeStep); if (av_arrive_t < 0 || av_arrive_t >=
  //   time_num_steps) {
  //     continue;
  //   }
  //   // Region match.
  //   if (s > vec_min_s[av_arrive_t] && s <= vec_max_s[av_arrive_t]) {
  //     if (vec_max_l[av_arrive_t] < center_l[i]) {
  //       boundary.OuterClampRightByIndex(i,
  //                                       vec_max_l[av_arrive_t] +
  //                                       kObjectBuffer);
  //     }
  //     if (vec_min_l[av_arrive_t] > center_l[i]) {
  //       boundary.OuterClampLeftByIndex(i,
  //                                      vec_min_l[av_arrive_t] -
  //                                      kObjectBuffer);
  //     }
  //   }
  // }

  return boundary;
}

}  // namespace

bool IsTurningLanePath(const PlannerSemanticMapManager& psmm,
                       mapping::ElementId lane_id) {
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, lane_id, false);
  if (!lane_info.IsValid()) {
    return false;
  }

  // return (!lane_info.junction_id().empty()) &&
  //        (lane_info.turn_type() == ad_byd::planning::LEFT_TURN ||
  //         lane_info.turn_type() == ad_byd::planning::RIGHT_TURN);
  if (lane_info.turn_type() == ad_byd::planning::LEFT_TURN ||
      lane_info.turn_type() == ad_byd::planning::RIGHT_TURN) {
    return true;
  }

  if (lane_info.split_topology() == ad_byd::planning::TOPOLOGY_SPLIT_LEFT ||
      lane_info.split_topology() == ad_byd::planning::TOPOLOGY_SPLIT_RIGHT) {
    return true;
  }

  if (lane_info.merge_topology() == ad_byd::planning::TOPOLOGY_MERGE_LEFT ||
      lane_info.merge_topology() == ad_byd::planning::TOPOLOGY_MERGE_RIGHT) {
    return true;
  }

  int next_num = 0;
  for (auto next_lane_id : lane_info.valid_next_lane_ids()) {
    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(next_lane_info, psmm, next_lane_id);

    if (next_lane_info.turn_type() == ad_byd::planning::LEFT_TURN ||
        next_lane_info.turn_type() == ad_byd::planning::RIGHT_TURN) {
      return true;
    }

    if (next_lane_info.split_topology() ==
            ad_byd::planning::TOPOLOGY_SPLIT_LEFT ||
        next_lane_info.split_topology() ==
            ad_byd::planning::TOPOLOGY_SPLIT_RIGHT) {
      return true;
    }

    if (next_lane_info.merge_topology() ==
            ad_byd::planning::TOPOLOGY_MERGE_LEFT ||
        next_lane_info.merge_topology() ==
            ad_byd::planning::TOPOLOGY_MERGE_RIGHT) {
      return true;
    }

    if (!(lane_info.junction_id() == 0) ||
        !(next_lane_info.junction_id() == 0)) {
      continue;
    }

    if (++next_num > 1) {
      return true;
    }

    // 与当前同处一个section，无需判断是否为junction
    if (next_lane_info.valid_pre_lane_ids().size() > 1) {
      return true;
    }

    for (auto next_next_lane_id : next_lane_info.valid_next_lane_ids()) {
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(next_next_lane_info, psmm,
                                        next_next_lane_id);
      if (!(next_next_lane_info.junction_id() == 0)) {
        break;
      }

      // 与下个lane同处一个section，无需判断是否为junction
      if (next_next_lane_info.valid_pre_lane_ids().size() > 1) {
        return true;
      }
    }
  }

  int pre_num = 0;
  for (auto pre_lane_id : lane_info.valid_pre_lane_ids()) {
    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(pre_lane_info, psmm, pre_lane_id);

    if (pre_lane_info.turn_type() == ad_byd::planning::LEFT_TURN ||
        pre_lane_info.turn_type() == ad_byd::planning::RIGHT_TURN) {
      return true;
    }

    if (pre_lane_info.split_topology() ==
            ad_byd::planning::TOPOLOGY_SPLIT_LEFT ||
        pre_lane_info.split_topology() ==
            ad_byd::planning::TOPOLOGY_SPLIT_RIGHT) {
      return true;
    }

    if (pre_lane_info.merge_topology() ==
            ad_byd::planning::TOPOLOGY_MERGE_LEFT ||
        pre_lane_info.merge_topology() ==
            ad_byd::planning::TOPOLOGY_MERGE_RIGHT) {
      return true;
    }

    if (!(lane_info.junction_id() == 0) ||
        !(pre_lane_info.junction_id() == 0)) {
      continue;
    }

    if (++pre_num > 1) {
      return true;
    }

    // 与当前同处一个section，无需判断是否为junction
    if (pre_lane_info.valid_next_lane_ids().size() > 1) {
      return true;
    }

    for (auto pre_pre_lane_id : pre_lane_info.valid_pre_lane_ids()) {
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(pre_pre_lane_info, psmm,
                                        pre_pre_lane_id);
      if (!(pre_pre_lane_info.junction_id() == 0)) {
        break;
      }

      // 与上个lane同处一个section，无需判断是否为junction
      if (pre_pre_lane_info.valid_next_lane_ids().size() > 1) {
        return true;
      }
    }
  }

  return false;
}

PathBoundary BuildPathBoundaryFromTargetLane(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const double& ego_v, const std::pair<int, int>& left_split_range,
    const std::pair<int, int>& right_split_range, double min_half_lane_width,
    bool borrow_lane_boundary, const FrenetBox& sl_box) {
  min_half_lane_width -= 0.1;

  const int n = drive_passage.size();
  std::vector<double> left_bound_vec, right_bound_vec;
  left_bound_vec.reserve(n);
  right_bound_vec.reserve(n);
  const double v_factor = std::clamp(ego_v / 10.0, 0.3, 1.0);
  const double split_ramp_factor_by_v = 1.3 * v_factor;
  const double split_entrance_ramp_factor_by_v = 0.8 * v_factor;
  const int split_entrance_range =
      CeilToInt(kDefaultLaneWidth * (1.3 - 0.8) / 0.1);
  const int left_split_th_low =
      left_split_range.first > 0
          ? left_split_range.first +
                std::min(split_entrance_range,
                         (left_split_range.second - left_split_range.first) / 3)
          : -1;
  const int left_split_th_high =
      left_split_range.first > 0
          ? left_split_range.first +
                (left_split_range.second - left_split_range.first) * 2 / 3
          : -1;
  const int right_split_th_low =
      right_split_range.first > 0
          ? right_split_range.first +
                std::min(
                    split_entrance_range,
                    (right_split_range.second - right_split_range.first) / 3)
          : -1;
  const int right_split_th_high =
      right_split_range.first > 0
          ? right_split_range.first +
                (right_split_range.second - right_split_range.first) * 2 / 3
          : -1;
  int station_idx = -1;
  for (const auto& station : drive_passage.stations()) {
    ++station_idx;
    auto& left_bound = left_bound_vec.emplace_back();
    auto& right_bound = right_bound_vec.emplace_back();

    if (borrow_lane_boundary) {
      right_bound = -kBorrowLaneOffset;
      left_bound = kBorrowLaneOffset;
      continue;
    }

    if (station.turn_type() != ad_byd::planning::NO_TURN) {
      const auto& lane_info = psmm.FindCurveLaneByIdOrNull(station.lane_id());
      right_bound = -kVirtualStationHalfWidth;
      left_bound = kVirtualStationHalfWidth;
      if (station.turn_type() == ad_byd::planning::U_TURN) {
        // NOTE: Assume all uturns turn to left.
        right_bound = -kDefaultHalfLaneWidth;
        continue;
      }
      if (lane_info != nullptr) {
        if (station.turn_type() == ad_byd::planning::LEFT_TURN) {
          right_bound = -kDefaultHalfLaneWidth;
          left_bound = !(lane_info->left_lane_id() == 0)
                           ? left_bound
                           : kIntersectionTurningMaxHalfLaneWidth;
        } else if (station.turn_type() == ad_byd::planning::RIGHT_TURN) {
          right_bound = !(lane_info->right_lane_id() == 0)
                            ? right_bound
                            : -kIntersectionTurningMaxHalfLaneWidth;
          left_bound = kDefaultHalfLaneWidth;
        }
      }
      continue;
    } else if (station.is_in_intersection()) {
      left_bound = kVirtualStationHalfWidth;
      right_bound = -kVirtualStationHalfWidth;
      continue;
    }

    double right_l = -std::numeric_limits<double>::infinity();
    double left_l = std::numeric_limits<double>::infinity();
    for (const auto& bound : station.boundaries()) {
      if (-kMaxHalfLaneWidth < bound.lat_offset &&
          bound.lat_offset < -min_half_lane_width) {
        if (bound.lat_offset > right_l) {
          right_l = bound.lat_offset;
        }
      }

      if (min_half_lane_width < bound.lat_offset &&
          bound.lat_offset < kMaxHalfLaneWidth) {
        if (bound.lat_offset < left_l) {
          left_l = bound.lat_offset;
        }
      }
    }

    right_bound = right_l == -std::numeric_limits<double>::infinity()
                      ? std::min(-kDefaultHalfLaneWidth, sl_box.l_min - 0.8)
                      : right_l;

    left_bound = left_l == std::numeric_limits<double>::infinity()
                     ? std::max(kDefaultHalfLaneWidth, sl_box.l_max + 0.8)
                     : left_l;

    if (station_idx > left_split_range.first &&
        station_idx < left_split_range.second) {
      if (left_split_range.first != -1) {
        left_bound = std::fmax(left_bound, kDefaultLaneWidth * 0.5);
        right_bound = std::fmin(right_bound, -kDefaultLaneWidth * 1.0);
      }
    }

    if (station_idx > right_split_range.first &&
        station_idx < right_split_range.second) {
      if (right_split_range.first != -1) {
        right_bound = std::fmin(right_bound, -kDefaultLaneWidth * 0.5);
        left_bound = std::fmax(left_bound, kDefaultLaneWidth * 1.0);
      }
    }

    // if (station.is_splitting()) {
    //   if (left_split_range.first > 0) {
    //     if (station_idx < left_split_th_low) {
    //       right_bound =
    //           std::fmin(right_bound,
    //                     -kDefaultLaneWidth *
    //                     split_entrance_ramp_factor_by_v);
    //     } else if (station_idx < left_split_th_high) {
    //       right_bound = std::fmin(right_bound,
    //                               -kDefaultLaneWidth *
    //                               split_ramp_factor_by_v);
    //     }
    //   } else if (right_split_range.first > 0) {
    //     if (station_idx < right_split_th_low) {
    //       left_bound = std::fmax(
    //           left_bound, kDefaultLaneWidth *
    //           split_entrance_ramp_factor_by_v);
    //     } else if (station_idx < right_split_th_high) {
    //       left_bound =
    //           std::fmax(left_bound, kDefaultLaneWidth *
    //           split_ramp_factor_by_v);
    //     }
    //   }
    // }
  }

  return PathBoundary(std::move(right_bound_vec), std::move(left_bound_vec));
}

PathBoundary BuildCurbPathBoundary(const DrivePassage& drive_passage) {
  const int n = drive_passage.stations().size();
  std::vector<double> left_bound_vec, right_bound_vec;
  left_bound_vec.reserve(n);
  right_bound_vec.reserve(n);

  for (const auto& station : drive_passage.stations()) {
    const auto [right_curb, left_curb] = station.QueryCurbOffsetAt(0.0).value();
    right_bound_vec.push_back(right_curb);
    left_bound_vec.push_back(left_curb);
  }

  return PathBoundary(std::move(right_bound_vec), std::move(left_bound_vec));
}

PathBoundary BuildSolidPathBoundary(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const FrenetCoordinate& cur_sl,
    const VehicleGeometryParamsProto& vehicle_geom,
    const ApolloTrajectoryPointProto& plan_start_point,
    const LaneChangeStateProto& lc_state, double target_lane_offset,
    const std::vector<double>& center_l) {
  const int n = drive_passage.size();
  std::vector<double> left_bound_vec, right_bound_vec;
  left_bound_vec.reserve(n);
  right_bound_vec.reserve(n);

  // get the solid_range for lane_change_task
  const bool is_lane_change_task =
      (LaneChangeStage::LCS_EXECUTING == lc_state.stage() ||
       LaneChangeStage::LCS_RETURN == lc_state.stage());
  const bool is_congestion_scene =
      (PushState::CONGESTION_LEFT_PUSH == lc_state.push_state() ||
       PushState::CONGESTION_RIGHT_PUSH == lc_state.push_state());
  std::pair<int, int> allow_solid_range = {0, n - 1};
  std::pair<int, int> force_solid_range = {-1, -1};
  bool is_lane_change_consider_merge = false;
  bool is_left = lc_state.lc_left();
  if (is_lane_change_task) {
    int idx = 0, cur_idx = 0;
    for (const auto& station : drive_passage.stations()) {
      if (station.accumulated_s() < cur_sl.s) ++cur_idx;

      if (!is_lane_change_consider_merge) {
        auto curr_lane = psmm.FindLaneByIdOrNull(station.lane_id());
        if (curr_lane) {
          auto check_lane_id =
              is_left ? curr_lane->right_lane_id() : curr_lane->left_lane_id();
          auto check_lane = psmm.FindLaneByIdOrNull(check_lane_id);
          if (check_lane) {
            is_lane_change_consider_merge =
                check_lane->merge_topology() !=
                ad_byd::planning::TOPOLOGY_MERGE_NONE;
          }
        }
      }

      if (station.is_in_intersection() /*|| station.is_virtual()*/) {
        allow_solid_range.second = idx;
        cur_idx = std::min(cur_idx, std::max(0, idx - 12));
        auto start_idx =
            is_congestion_scene ? cur_idx : std::max(idx - 50, cur_idx);
        allow_solid_range.first = start_idx;
        force_solid_range.first = allow_solid_range.first;
        force_solid_range.second = allow_solid_range.second;
        break;
      }
      ++idx;
    }
  }

  double right_max_l = target_lane_offset - vehicle_geom.width() * 0.5;
  double left_min_l = target_lane_offset + vehicle_geom.width() * 0.5;
  const double line_buffer = vehicle_geom.width() * 0.5 + 0.3;
  for (int i = 0; i < n; ++i) {
    if (LaneChangeStage::LCS_PAUSE == lc_state.stage() ||
        lc_state.push_state() != PushState::NONE_PUSH) {
      if (i < center_l.size() - 1) {
        if (lc_state.lc_left()) {
          left_min_l =
              std::max(left_min_l, center_l.at(i) + vehicle_geom.width() * 0.5);
        } else {
          right_max_l = std::min(right_max_l,
                                 center_l.at(i) - vehicle_geom.width() * 0.5);
        }
      }
    }
    const auto& station = drive_passage.station(StationIndex(i));
    const auto [right_curb, left_curb] = station.QueryCurbOffsetAt(0.0).value();
    double right_l = right_curb;
    double left_l = left_curb;

    // ignore the solid_line out of allow_range
    if ((i < allow_solid_range.first || i > allow_solid_range.second) ||
        (is_lane_change_task && !is_lane_change_consider_merge)) {
      right_bound_vec.push_back(right_l);
      left_bound_vec.push_back(left_l);
      continue;
    }
    const bool force_solid_flag =
        (i >= force_solid_range.first && i <= force_solid_range.second);

    for (const auto& bound : station.boundaries()) {
      if (bound.lat_offset <= right_max_l && bound.lat_offset > right_l &&
          (bound.IsSolid(cur_sl.l) || force_solid_flag)) {
        right_l = std::fmin(bound.lat_offset, target_lane_offset - line_buffer);
      }

      if (bound.lat_offset >= left_min_l && bound.lat_offset < left_l &&
          (bound.IsSolid(cur_sl.l) || force_solid_flag)) {
        left_l = std::fmax(bound.lat_offset, target_lane_offset + line_buffer);
      }
    }
    right_bound_vec.push_back(right_l);
    left_bound_vec.push_back(left_l);
  }

  return PathBoundary(std::move(right_bound_vec), std::move(left_bound_vec));
}

PathBoundary BuildPathBoundaryFromAvKinematics(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const FrenetCoordinate& cur_sl, const FrenetBox& sl_box,
    const LaneChangeStateProto& lc_state, absl::Span<const double> s_vec,
    double target_lane_offset, double max_lane_change_lat_accel,
    bool lane_change_pause) {
  const auto traj_l_s = GenerateConstLateralAccelConstSpeedTraj(
      drive_passage, plan_start_point, cur_sl, target_lane_offset,
      max_lane_change_lat_accel);

  const int n = drive_passage.size();
  const double half_av_width = vehicle_geom.width() * 0.5;
  const double ego_lat_buffer = half_av_width + kEgoLatBuffer;
  std::vector<double> right_bound_vec, left_bound_vec;
  right_bound_vec.reserve(n);
  left_bound_vec.reserve(n);
  const bool is_lane_change_task =
      (LaneChangeStage::LCS_EXECUTING == lc_state.stage() ||
       LaneChangeStage::LCS_RETURN == lc_state.stage());
  const bool is_congestion_scene =
      (PushState::CONGESTION_LEFT_PUSH == lc_state.push_state() ||
       PushState::CONGESTION_RIGHT_PUSH == lc_state.push_state());
  const bool is_left_congestion =
      PushState::CONGESTION_LEFT_PUSH == lc_state.push_state();
  const double lc_congestion_dist = std::fmax(3.0, plan_start_point.v() * 4.0);
  const double lc_protect_dist = 0.5 * plan_start_point.v();
  std::vector<double> speed_vec = {40, 60, 80, 100, 120};          // km/h
  std::vector<double> lat_v_vec = {0.90, 0.81, 0.72, 0.65, 0.60};  // m/s
  const PiecewiseLinearFunction<double, double> plf(speed_vec, lat_v_vec);
  const double lat_v =
      plf.Evaluate(std::fmax(plan_start_point.v(), 1.0) * 3.6) * 1.05;

  for (int i = 0; i < drive_passage.size(); ++i) {
    const auto s = s_vec[i];
    auto& right_bound = right_bound_vec.emplace_back();
    auto& left_bound = left_bound_vec.emplace_back();

    right_bound = std::numeric_limits<double>::infinity();
    left_bound = -std::numeric_limits<double>::infinity();

    if (is_lane_change_task && is_congestion_scene) {
      if (s < sl_box.s_max + lc_congestion_dist) {
        double lat_dist = lat_v *
                          std::fmax(0.0, s - sl_box.s_max - lc_protect_dist) /
                          std::fmax(1.0, plan_start_point.v());
        if (is_left_congestion) {
          right_bound = sl_box.l_min - kEgoLatBuffer + lat_dist;
        } else {
          left_bound = sl_box.l_max + kEgoLatBuffer - lat_dist;
        }
      }
    } else {
      if (traj_l_s.ok() && s >= traj_l_s->x().front() &&
          s <= traj_l_s->x().back()) {
        right_bound = traj_l_s->Evaluate(s) - ego_lat_buffer;
        left_bound = traj_l_s->Evaluate(s) + ego_lat_buffer;
      }
      // TODO: maybe useless.
      if (s < sl_box.s_max + std::fmax(plan_start_point.v() * 2.0,
                                       kMinKinematicBoundaryProtectedZone)) {
        right_bound = std::min(right_bound, sl_box.l_min - kEgoLatBuffer);
        left_bound = std::max(left_bound, sl_box.l_max + kEgoLatBuffer);
      }
    }
    if (lane_change_pause) {
      right_bound = std::min(right_bound, target_lane_offset - ego_lat_buffer);
      left_bound = std::max(left_bound, target_lane_offset + ego_lat_buffer);
    }
  }

  return PathBoundary(std::move(right_bound_vec), std::move(left_bound_vec));
}

PathBoundary ShrinkPathBoundaryForLaneChangePause(
    const VehicleGeometryParamsProto& vehicle_geom, const FrenetBox& sl_box,
    const LaneChangeStateProto& lc_state, PathBoundary boundary,
    double target_lane_offset, const std::vector<double>& center_l) {
  const double half_av_width = vehicle_geom.width() * 0.5;
  if (lc_state.lc_left()) {
    for (int i = 0; i < boundary.size(); ++i) {
      double shrinked_offset = target_lane_offset + half_av_width;
      if (i < center_l.size() - 1) {
        shrinked_offset =
            std::max(shrinked_offset, center_l.at(i) + half_av_width);
      }
      double shrinked_l =
          std::max(shrinked_offset, sl_box.l_max) + kLcPauseEgoLatBuffer;
      boundary.OuterClampLeftByIndex(i, shrinked_l);
    }
  } else {
    for (int i = 0; i < boundary.size(); ++i) {
      double shrinked_offset = target_lane_offset - half_av_width;
      if (i < center_l.size() - 1) {
        shrinked_offset =
            std::min(shrinked_offset, center_l.at(i) - half_av_width);
      }
      double shrinked_l =
          std::min(shrinked_offset, sl_box.l_min) - kLcPauseEgoLatBuffer;
      boundary.OuterClampRightByIndex(i, shrinked_l);
    }
  }

  return boundary;
}

PathBoundary ExtendPathBoundaryForLaneChangePause(
    const LaneChangeStateProto& lc_state, const double half_lane_width,
    PathBoundary boundary) {
  if (lc_state.lc_left()) {
    for (int i = 0; i < boundary.size(); ++i) {
      boundary.InnerClampRightByIndex(i, -3.0 * half_lane_width);
    }
  } else {
    for (int i = 0; i < boundary.size(); ++i) {
      boundary.InnerClampLeftByIndex(i, 3.0 * half_lane_width);
    }
  }
  return boundary;
}

PathBoundary ShrinkPathBoundaryForObject(
    const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    absl::Span<const double> s_vec, absl::Span<const double> center_l,
    absl::Span<const Vec2d> center_xy, const PathBoundary& inner_boundary,
    const PathBoundary& curb_boundary, PathBoundary boundary,
    const PlannerSemanticMapManager& psmm, const FrenetBox& sl_box) {
  for (const auto& traj : st_traj_mgr.stationary_object_trajs()) {
    const auto obj_fbox_or =
        drive_passage.QueryFrenetBoxAtContour(traj->states().front().contour);
    if (!obj_fbox_or.ok() || obj_fbox_or->s_min > s_vec.back()) {
      continue;
    }
    const auto& obj_fbox = obj_fbox_or.value();
    if (std::fmin(std::fabs(obj_fbox.l_max), std::fabs(obj_fbox.l_min)) < 1.2) {
      continue;
    }
    boundary = BuildBoundaryForStationaryObject(
        drive_passage, center_l, obj_fbox, s_vec, std::move(boundary));
  }
  const auto av_t_s = GenerateAvTrajAlongRefCenterLine(
      drive_passage, plan_start_point, center_xy);

  if (av_t_s.ok()) {
    for (const auto& traj : st_traj_mgr.moving_object_trajs()) {
      // const auto obj_fbox_or =
      //   drive_passage.QueryFrenetBoxAtContour(traj->states().front().contour);
      // const auto obj_fbox_or =
      // drive_passage.QueryFrenetCoordinateAt(traj->pose().pos()); double
      // obs_speed = traj->states().front().traj_point->v();
      // // if ( obj_fbox_or->s_max < av_sl_point->s &&
      // //     std::fabs(obj_fbox_or->center_l() - av_sl_point->l) < 1.5)
      // //   continue;
      // if (av_sl_point.ok() && obj_fbox_or.ok()) {
      //   if (obj_fbox_or->s < av_sl_point->s || obj_fbox_or->s >
      //   av_sl_point->s + (plan_start_point.v() - obs_speed) * 3.0 /*&&
      //       std::fabs(obj_fbox_or->l - av_sl_point->l) < 1.5*/){
      //     Log2DDS::LogDataV0("obs_id", std::string(traj->object_id()));
      //     continue;
      //   }
      // }

      std::string obj_ignore = " ";
      obj_ignore += std::string(traj->object_id());
      // Log2DDS::LogDataV0("unsafe_obj_front", std::string(traj->object_id()));

      const auto obj_fbox_or =
          drive_passage.QueryFrenetBoxAtContour(traj->states().front().contour);
      if (!obj_fbox_or.ok() || obj_fbox_or->s_min > s_vec.back()) {
        obj_ignore += " ignore-0";
        Log2DDS::LogDataV2("obj_ignore", obj_ignore);
        continue;
      }

      if (obj_fbox_or.value().l_max < sl_box.l_min - 2.7 ||
          obj_fbox_or.value().l_min > sl_box.l_max + 2.7 ||
          (traj->states().front().traj_point->v() < plan_start_point.v() &&
           obj_fbox_or.value().s_max <= sl_box.s_min) ||
          (traj->states().front().traj_point->v() > plan_start_point.v() &&
           obj_fbox_or.value().s_min >= sl_box.s_max)) {
        obj_ignore += " ingore-1";
        Log2DDS::LogDataV2("obj_ignore", obj_ignore);
        continue;
      }

      double lat_overlap = 0.0;
      if (obj_fbox_or.value().l_max < sl_box.l_max &&
          obj_fbox_or.value().l_max >= sl_box.l_min) {
        lat_overlap = obj_fbox_or.value().l_max - sl_box.l_min;
      } else if (sl_box.l_max < obj_fbox_or.value().l_max &&
                 sl_box.l_max >= obj_fbox_or.value().l_min) {
        obj_ignore += " ignore-2";
        Log2DDS::LogDataV2("obj_ignore", obj_ignore);
        lat_overlap = sl_box.l_max - obj_fbox_or.value().l_min;
      }

      if (lat_overlap > 0.5) {
        obj_ignore += " ignore-3";
        Log2DDS::LogDataV2("obj_ignore", obj_ignore);
        continue;
      }

      double judge_length =
          std::max(10.0, std::fabs(plan_start_point.v() -
                                   traj->states().front().traj_point->v()) *
                             3.0);

      if (obj_fbox_or.value().s_max < sl_box.s_min) {
        double s_back = sl_box.s_min - obj_fbox_or.value().s_max;
        if (s_back > judge_length) {
          obj_ignore += " ignore-4";
          Log2DDS::LogDataV2("obj_ignore", obj_ignore);
          continue;
        }
      } else if (obj_fbox_or.value().s_max > sl_box.s_min) {
        double s_back = obj_fbox_or.value().s_min - sl_box.s_max;
        if (s_back > judge_length) {
          obj_ignore += " ignore-5";
          Log2DDS::LogDataV2("obj_ignore", obj_ignore);
          continue;
        }
      }

      Log2DDS::LogDataV2("unsafe_obj", std::string(traj->object_id()));

      // if(ObstsacleFilter(traj,drive_passage,plan_start_point,psmm)) continue;
      boundary = BuildBoundaryForDynamicObject(
          drive_passage, center_l, *traj, *av_t_s, s_vec, inner_boundary,
          curb_boundary, std::move(boundary));
    }
  }

  return boundary;
}

bool ObstsacleFilter(const SpacetimeObjectTrajectory* traj,
                     const DrivePassage& drive_passage,
                     const ApolloTrajectoryPointProto& plan_start_point,
                     const PlannerSemanticMapManager& psmm) {
  if (!traj) return true;
  const auto av_sl_point = drive_passage.QueryFrenetCoordinateAt(
      Vec2dFromApolloTrajectoryPointProto(plan_start_point));
  double obs_speed = traj->states().front().traj_point->v();
  const auto obj_fbox_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(traj->pose().pos());

  // const auto lane_id = drive_passage.lane_path().front().lane_id();
  // const auto lane_info = psmm.FindLaneInfoOrNull(lane_id);
  // bool is_stright = false;
  // if (lane_info && lane_info->direction == mapping::LaneProto::STRAIGHT){
  //   is_stright = true;
  // }

  if (av_sl_point.ok() && obj_fbox_or.ok()) {
    if (obj_fbox_or->s < av_sl_point->s ||
        obj_fbox_or->s >
            av_sl_point->s + (plan_start_point.v() - obs_speed) * 3.0) {
      Log2DDS::LogDataV0("obs_id", std::string(traj->object_id()));
      return true;
    }
  }
  return false;
}

double GetObstacleCurLatSpeed(const DrivePassage& drive_passage,
                              const double speed, const double theta,
                              const double s) {
  double lat_speed = 0.0;
  const auto obstacle_lane_tangent = drive_passage.QueryTangentAtS(s);
  if (obstacle_lane_tangent.ok()) {
    lat_speed = speed * obstacle_lane_tangent->CrossProd(
                            Vec2d::FastUnitFromAngle(theta));
  }
  return lat_speed;
}

bool GetObstacleMeanLOfHistory(const DrivePassage& drive_passage,
                               const ObjectHistoryManager* obj_history_mgr,
                               const SpacetimeObjectTrajectory& obs_traj,
                               double* center_l, double* box_l_min,
                               double* box_l_max, double* lat_speed) {
  if (obj_history_mgr == nullptr || center_l == nullptr ||
      box_l_min == nullptr || box_l_max == nullptr || lat_speed == nullptr) {
    Log2DDS::LogDataV2("GetObstacleMeanLOfHistory", "nullptr");
    return false;
  }
  int64_t kMaxHistoryLengthUs = 1e6;
  double sum_l = 0.0, sum_l_min = 0.0, sum_l_max = 0.0, sum_lat_speed = 0.0;
  int count_l = 0;
  std::string cur_obj_id = obs_traj.planner_object().id();
  const ObjectHistory* obs_history = obj_history_mgr->GetObjHistory(cur_obj_id);
  if (obs_history == nullptr || obs_history->Empty()) {
    return false;
  }
  const std::deque<ObjectFrame>& frames = obs_history->GetFrames();
  bool last_frame = true;
  int64_t first_timestamp = 0, last_timestamp = 0;
  double first_center_l = 0.0, last_center_l = 0.0;
  for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
    const auto& frame = *it;
    const auto& obs_box_proto = frame.object_proto.bounding_box();
    Vec2d obs_center{obs_box_proto.x(), obs_box_proto.y()};
    double speed =
        std::hypot(frame.object_proto.vel().x(), frame.object_proto.vel().y());
    Box2d obs_box{obs_center, obs_box_proto.heading(), obs_box_proto.length(),
                  obs_box_proto.width()};
    ASSIGN_OR_CONTINUE(const auto obs_frenet_box,
                       drive_passage.QueryFrenetBoxAt(obs_box, false));
    if (last_frame) {
      last_frame = false;
      last_timestamp = frame.timestamp;
      last_center_l = obs_frenet_box.center_l();
    }
    first_timestamp = frame.timestamp;
    first_center_l = obs_frenet_box.center_l();
    sum_l += obs_frenet_box.center_l();
    sum_l_min += obs_frenet_box.l_min;
    sum_l_max += obs_frenet_box.l_max;
    sum_lat_speed +=
        GetObstacleCurLatSpeed(drive_passage, speed, frame.object_proto.yaw(),
                               obs_frenet_box.center_s());
    ++count_l;
    if (last_timestamp - first_timestamp > kMaxHistoryLengthUs) {
      break;
    }
  }
  *box_l_min = sum_l_min / std::max(count_l, 1);
  *box_l_max = sum_l_max / std::max(count_l, 1);
  *center_l = sum_l / std::max(count_l, 1);
  *lat_speed = sum_lat_speed / std::max(count_l, 1);
  Log2DDS::LogDataV2("GetObstacleMeanLOfHistory",
                     absl::StrCat("id: ", cur_obj_id, ", count_l: ", count_l,
                                  ", box_l_min: ", *box_l_min, ", box_l_max: ",
                                  *box_l_max, ", lat_speed: ", *lat_speed));
  if (count_l > 0) {
    return true;
  } else {
    return false;
  }
}

// cal total time of lateral movement
bool CalculateTotalTime(const double& v0, const double& vf, const double& s,
                        const double& vmax, double acc, double dec,
                        double& total_time) {
  if (s < 0 || vmax <= 0 || acc <= 0 || dec <= 0) {
    return false;
  }
  acc = std::fmax(acc, std::numeric_limits<double>::epsilon());
  dec = std::fmax(dec, std::numeric_limits<double>::epsilon());
  double s_acc = 0.5 * (vmax * vmax - v0 * v0) / acc;
  double t_acc = (vmax - v0) / acc;
  double s_dec = 0.5 * (vf * vf - vmax * vmax) / (-dec);
  double t_dec = (vf - vmax) / (-dec);
  if (s >= (s_acc + s_dec)) {
    double s_uniform = s - (s_acc + s_dec);
    double t_uniform = (s_uniform > 0) ? s_uniform / vmax : 0;
    total_time = t_acc + t_uniform + t_dec;
    return true;
  } else {
    double a_quad = 0.5 * (1 / acc + 1 / dec);
    double b_quad = 0.0;
    double c_quad = -s - (v0 * v0 / (2 * acc) + vf * vf / (2 * dec));
    double discriminant = b_quad * b_quad - 4.0 * a_quad * c_quad;
    if (discriminant < 0) {
      return false;
    }
    double vpeak = (-b_quad + std::sqrt(discriminant)) /
                   (2 * a_quad + std::numeric_limits<double>::epsilon());
    vpeak = std::fmin(std::fmax(vpeak, v0), vmax);
    t_acc = (vpeak - v0) / acc;
    t_dec = (vpeak - vf) / dec;
    total_time = t_acc + t_dec;
    return true;
  }
  return false;
}

// check lateral safety in target_nudge_l
bool LateralSafeCheck(const double& adv_boundary_l, const double& adv_lat_speed,
                      const double& adv_target_acc,
                      const double& obstacle_boundary_l,
                      const double& obstacle_lat_speed,
                      const double& obstacle_target_acc,
                      const double& lateral_safe_distance,
                      const double& lateral_safe_ttc, const double& v_max,
                      const bool& is_left, const double& target_nudge_l) {
  auto cal_time_to_collide = [](double dv, double gap) -> double {
    double ttc = std::numeric_limits<double>::infinity();
    if ((dv > 0.0 && gap < std::numeric_limits<double>::epsilon()) ||
        (dv < 0.0 && gap > -std::numeric_limits<double>::epsilon())) {
      ttc = std::fmax(-gap / dv, 0.0);
    }
    return ttc;
  };
  const double v0 = adv_lat_speed;
  const double vt = 0;
  const double abs_acc_and_dec = std::fabs(adv_target_acc);
  const double s = std::fabs(target_nudge_l - adv_boundary_l);
  double t = 0;
  bool calc_success =
      CalculateTotalTime(v0, vt, s, v_max, abs_acc_and_dec, abs_acc_and_dec, t);
  if (!calc_success || t < 0) {
    return false;
  }
  const double obstacle_l_at_arrive_time = obstacle_boundary_l +
                                           obstacle_lat_speed * t +
                                           0.5 * obstacle_target_acc * t * t;
  const double obstacle_lateral_speed_at_arrive_time =
      obstacle_lat_speed + obstacle_target_acc * t;
  const bool no_overlap_before_end_point =
      is_left ? (obstacle_l_at_arrive_time > target_nudge_l)
              : (obstacle_l_at_arrive_time < target_nudge_l);
  const bool end_point_distance_safe =
      std::fabs(obstacle_l_at_arrive_time - target_nudge_l) >
      lateral_safe_distance;
  const double end_point_ttc =
      no_overlap_before_end_point
          ? cal_time_to_collide(obstacle_lateral_speed_at_arrive_time - vt,
                                obstacle_l_at_arrive_time - target_nudge_l)
          : 0;
  const bool end_point_ttc_safe = end_point_ttc > lateral_safe_ttc;
  if (end_point_distance_safe && end_point_ttc_safe &&
      no_overlap_before_end_point) {
    return true;
  } else {
    return false;
  }
}

bool SearchSuitableLateralDistance(
    const double adv_boundary_l, const double adv_speed,
    const double adv_target_acc, const double obstacle_boundary_l,
    const double obstacle_speed, const double obstacle_target_acc,
    const double lateral_safe_distance, const double lateral_safet_ttc,
    const FrenetBox& sl_box, double lateral_speed_max, bool is_left,
    const double ori_offset, double& lateral_l) {
  auto equals_with_epsilon = [](double a, double b,
                                double custom_epsilon =
                                    std::numeric_limits<double>::epsilon()) {
    return std::fabs(a - b) <= custom_epsilon;
  };
  const double eps = 1e-1;
  double half_ego_sl_width = sl_box.width() * 0.5;
  double left =
      std::min(sl_box.l_max, -ad_byd::planning::Constants::DEFAULT_LANE_WIDTH +
                                 half_ego_sl_width);
  double right = ori_offset;
  if (!is_left) {
    left = ori_offset;
    right =
        std::max(sl_box.l_min, ad_byd::planning::Constants::DEFAULT_LANE_WIDTH -
                                   half_ego_sl_width);
  }
  const double original_left = left;
  const double original_right = right;
  while (right - left > eps) {
    double mid = left + (right - left) * 0.5;
    if (LateralSafeCheck(adv_boundary_l, adv_speed, adv_target_acc,
                         obstacle_boundary_l, obstacle_speed,
                         obstacle_target_acc, lateral_safe_distance,
                         lateral_safet_ttc, lateral_speed_max, is_left, mid)) {
      if (is_left) {
        left = mid;
      } else {
        right = mid;
      }
    } else {
      if (is_left) {
        right = mid;
      } else {
        left = mid;
      }
    }
  }
  if (equals_with_epsilon(left, original_left, eps)) {
    lateral_l = original_left;
  } else if (equals_with_epsilon(left, original_right, eps)) {
    lateral_l = original_right;
  } else {
    lateral_l = left;
  }
  return true;
}

double ComputeCollisionFreeOffset(
    const DrivePassage& drive_passage, const FrenetCoordinate& cur_sl,
    const ApolloTrajectoryPointProto& plan_start_point, double half_av_width,
    absl::Span<const SpacetimeObjectTrajectory> obj_trajs, const bool is_left,
    const bool is_lc_pause, const FrenetBox& sl_box,
    const double ori_lc_pause_offset, const bool is_pause_congestion = false,
    absl::flat_hash_set<std::string>* unsafe_object_ids = nullptr,
    const ObjectHistoryManager* obj_history_mgr = nullptr) {
  const double kRearToRearAxle = 1.15, kFrontToRearAxle = 4.10,
               kDefaultLonDist = 2.5, kTimeToCollision = 2.0, kLonRange = 0.5,
               kSpeedDiffThreshold = 1.0, kRearExtraLonDist = 0.5;
  constexpr double kSlightAdvLateralAcc = 0.3;
  constexpr double kSlightAdvLateralAccDist = 0.7;
  constexpr double kMaxAdvLateralAcc = 0.7;
  constexpr double kMaxAdvLateralAccDist = 2.0;
  constexpr double kMaxPotentialLateralAcc = 0.08;
  constexpr double kPotentialCollisionBuffer = 0.4;
  constexpr double kTTCLateralTheshold = 1.5;
  constexpr double kCongestionTTCLateralTheshold = 1.0;
  constexpr double kMaxLateralSpeed = 0.8;
  constexpr double kLcPauseExtraOffsetBuffer = 1.0;
  constexpr double ObstacleSafeMinBuffer = 1.0;
  constexpr double ObstacleSafeMaxBuffer = 1.2;
  constexpr double PushObstacleSafeMinBuffer = 0.7;
  constexpr double CongestionObstacleSafeBuffer = 0.9;
  constexpr double CongestionIgnoreFrontDist = 0.2;
  std::vector<double> speed_safe_lat_dist_ramp = {0.8, 0.9, 1.1, 1.3, 1.4};
  std::vector<double> safe_speed_ramp = {
      0.0 * ad_byd::planning::Constants::KPH2MPS,
      10.0 * ad_byd::planning::Constants::KPH2MPS,
      30.0 * ad_byd::planning::Constants::KPH2MPS,
      60.0 * ad_byd::planning::Constants::KPH2MPS,
      135.0 * ad_byd::planning::Constants::KPH2MPS};
  PiecewiseLinearFunction<double, double> safe_lat_dist_plf(
      safe_speed_ramp, speed_safe_lat_dist_ramp);
  double speed_safe_lat_dist = safe_lat_dist_plf.Evaluate(plan_start_point.v());
  double obj_center_left_range =
      0.5 * ad_byd::planning::Constants::DEFAULT_LANE_WIDTH;
  double obj_center_right_range =
      -0.5 * ad_byd::planning::Constants::DEFAULT_LANE_WIDTH;
  double safe_offset = is_left ? std::numeric_limits<double>::max()
                               : std::numeric_limits<double>::lowest();
  double new_safe_offset = is_left ? std::numeric_limits<double>::max()
                                   : std::numeric_limits<double>::lowest();
  if (!is_lc_pause) {
    obj_center_left_range =
        is_left ? 1.5 * ad_byd::planning::Constants::DEFAULT_LANE_WIDTH
                : -0.5 * ad_byd::planning::Constants::DEFAULT_LANE_WIDTH;
    obj_center_right_range =
        is_left ? 0.5 * ad_byd::planning::Constants::DEFAULT_LANE_WIDTH
                : -1.5 * ad_byd::planning::Constants::DEFAULT_LANE_WIDTH;
    Log2DDS::LogDataV2(
        "safe_push_offset",
        absl::StrCat(" speed_safe_lat_dist: ", speed_safe_lat_dist,
                     " obj_center_left_range: ", obj_center_left_range,
                     " obj_center_right_range: ", obj_center_right_range));
  }
  for (const auto& traj : obj_trajs) {
    // obstacle type condition
    if (!(traj.object_type() == ObjectType::OT_VEHICLE ||
          traj.object_type() == ObjectType::OT_LARGE_VEHICLE ||
          traj.object_type() == ObjectType::OT_MOTORCYCLIST ||
          traj.object_type() == ObjectType::OT_CYCLIST ||
          traj.object_type() == ObjectType::OT_TRICYCLIST)) {
      continue;
    }
    bool is_unsafe_obs = false;
    if (unsafe_object_ids != nullptr && unsafe_object_ids->size() > 0) {
      if (unsafe_object_ids->find(traj.planner_object().id()) !=
          unsafe_object_ids->end()) {
        is_unsafe_obs = true;
      }
    }

    // coordinate transform
    auto object_box = traj.bounding_box();
    auto obj_frenet_box = drive_passage.QueryFrenetBoxAt(object_box);
    if (!obj_frenet_box.ok()) {
      continue;
    }

    // latitudinal condition
    double extra_lat_filter_buffer = is_lc_pause ? 0.2 : 0.0;
    if (obj_frenet_box->l_min >
            obj_center_left_range + extra_lat_filter_buffer ||
        obj_frenet_box->l_max <
            obj_center_right_range - extra_lat_filter_buffer) {
      continue;
    }

    // longitudinal condition
    double obj_speed = traj.pose().v();
    if (((obj_frenet_box->s_max <
              cur_sl.s - kRearToRearAxle - kDefaultLonDist &&
          kTimeToCollision * (obj_speed - plan_start_point.v()) <
              cur_sl.s - kRearToRearAxle - obj_frenet_box->s_max) ||
         (obj_frenet_box->s_min > cur_sl.s + kFrontToRearAxle +
                                      kDefaultLonDist + kRearExtraLonDist &&
          kTimeToCollision * (plan_start_point.v() - obj_speed) <
              obj_frenet_box->s_min - cur_sl.s - kFrontToRearAxle) ||
         (obj_frenet_box->s_min > sl_box.s_max + kRearExtraLonDist &&
          plan_start_point.v() < obj_speed)) &&
        !is_unsafe_obs) {
      continue;
    }
    if (obj_frenet_box->s_max < cur_sl.s - kRearToRearAxle - kLonRange &&
        plan_start_point.v() > obj_speed + kSpeedDiffThreshold &&
        !is_unsafe_obs) {
      continue;
    }

    // inhibit push - large vehicle
    if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE && !is_lc_pause) {
      safe_offset = 0.0;
      Log2DDS::LogDataV2("safe_push_offset",
                         " safe_offset = 0.0! large vehicle id: " +
                             std::string(traj.object_id()));
      break;
    }

    safe_offset =
        is_left
            ? std::fmin(safe_offset, obj_frenet_box->l_min -
                                         speed_safe_lat_dist - half_av_width)
            : std::fmax(safe_offset, obj_frenet_box->l_max +
                                         speed_safe_lat_dist + half_av_width);
    // filter obj in lane
    if (is_left) {
      if (obj_frenet_box->s_min > sl_box.s_max &&
          obj_frenet_box->l_min < sl_box.l_max)
        continue;
    } else {
      if (obj_frenet_box->s_min > sl_box.s_max &&
          obj_frenet_box->l_max > sl_box.l_min)
        continue;
    }

    double hist_avg_center_l = 0.0;
    double hist_avg_box_l_min = 0.0;
    double hist_avg_box_l_max = 0.0;
    double hist_avg_lat_speed = 0.0;
    bool hist_state_valid = GetObstacleMeanLOfHistory(
        drive_passage, obj_history_mgr, traj, &hist_avg_center_l,
        &hist_avg_box_l_min, &hist_avg_box_l_max, &hist_avg_lat_speed);
    double cal_obj_box_l_max =
        hist_state_valid ? hist_avg_box_l_max : obj_frenet_box->l_max;
    double cal_obj_box_l_min =
        hist_state_valid ? hist_avg_box_l_min : obj_frenet_box->l_min;

    // init vars
    // 搜索边界，最大设定为目标车道中心
    double pause_most_search_l =
        is_left ? std::min(ori_lc_pause_offset + half_av_width +
                               kLcPauseExtraOffsetBuffer,
                           half_av_width)
                : std::max(ori_lc_pause_offset - half_av_width -
                               kLcPauseExtraOffsetBuffer,
                           -half_av_width);

    const std::vector<double> speed_vec = {30, 80};
    const std::vector<double> safe_buffer_vec = {ObstacleSafeMinBuffer,
                                                 ObstacleSafeMaxBuffer};
    double safe_buffer = ad_byd::planning::math::interp1_inc(
        speed_vec, safe_buffer_vec, Mps2Kph(plan_start_point.v()));

    // 不同的pause场景下，需要考虑的障碍物不一样
    if (!is_pause_congestion) {
      if (is_left && cal_obj_box_l_min > sl_box.l_max) {
        pause_most_search_l =
            std::min(pause_most_search_l, cal_obj_box_l_min - safe_buffer);
      } else if (!is_left && cal_obj_box_l_max < sl_box.l_min) {
        pause_most_search_l =
            std::max(pause_most_search_l, cal_obj_box_l_max + safe_buffer);
      }
    } else {
      // 拥堵场景：忽略前方障碍物，同时减少后方障碍物的安全buffer
      if (obj_frenet_box->s_min > sl_box.s_max + CongestionIgnoreFrontDist) {
        continue;
      }
      if (is_left && cal_obj_box_l_min > sl_box.l_max) {
        pause_most_search_l =
            std::min(pause_most_search_l,
                     cal_obj_box_l_min - CongestionObstacleSafeBuffer);
      } else if (!is_left && cal_obj_box_l_max < sl_box.l_min) {
        pause_most_search_l =
            std::max(pause_most_search_l,
                     cal_obj_box_l_max + CongestionObstacleSafeBuffer);
      }
    }

    double adv_boundary_l = is_left ? sl_box.l_max : sl_box.l_min;
    double adv_lat_speed = 0.0;
    const auto adv_lane_tangent = drive_passage.QueryTangentAtS(cur_sl.s);
    if (adv_lane_tangent.ok()) {
      adv_lat_speed = plan_start_point.v() *
                      adv_lane_tangent->CrossProd(Vec2d::FastUnitFromAngle(
                          plan_start_point.path_point().theta()));
    }
    double obstacle_boundary_l =
        is_left ? cal_obj_box_l_min : cal_obj_box_l_max;
    double push_most_search_l =
        is_left ? cal_obj_box_l_min - PushObstacleSafeMinBuffer
                : cal_obj_box_l_max + PushObstacleSafeMinBuffer;
    const double adv_lateral_acc =
        Lerp(kSlightAdvLateralAcc, kSlightAdvLateralAccDist, kMaxAdvLateralAcc,
             kMaxAdvLateralAccDist,
             std::fabs(adv_boundary_l - obstacle_boundary_l), true);
    double obstacle_lat_speed = 0.0;
    const auto obstacle_lane_tangent =
        drive_passage.QueryTangentAtS(traj.pose().s());
    if (obstacle_lane_tangent.ok()) {
      obstacle_lat_speed =
          traj.pose().v() * obstacle_lane_tangent->CrossProd(
                                Vec2d::FastUnitFromAngle(traj.pose().theta()));
    }
    obstacle_lat_speed =
        hist_state_valid ? hist_avg_lat_speed : obstacle_lat_speed;
    double obstacle_target_acc = is_pause_congestion ? 0.0
                                 : is_left           ? -kMaxPotentialLateralAcc
                                                     : kMaxPotentialLateralAcc;
    double lateral_safe_distance = kPotentialCollisionBuffer;
    double lateral_safet_ttc = is_pause_congestion
                                   ? kCongestionTTCLateralTheshold
                                   : kTTCLateralTheshold;
    double adv_lat_v_max = kMaxLateralSpeed;
    double safe_offset_veh_boundary =
        is_left ? safe_offset + half_av_width : safe_offset - half_av_width;
    // check original offset safety
    bool lat_safe = LateralSafeCheck(
        adv_boundary_l, adv_lat_speed, adv_lateral_acc, obstacle_boundary_l,
        obstacle_lat_speed, obstacle_target_acc, lateral_safe_distance,
        lateral_safet_ttc, adv_lat_v_max, is_left, safe_offset_veh_boundary);
    double cur_safe_offset = 0.0;
    // search suitable offset
    bool search_success = SearchSuitableLateralDistance(
        adv_boundary_l, adv_lat_speed, adv_lateral_acc, obstacle_boundary_l,
        obstacle_lat_speed, obstacle_target_acc, lateral_safe_distance,
        lateral_safet_ttc, sl_box, adv_lat_v_max, is_left,
        is_lc_pause ? pause_most_search_l : push_most_search_l,
        cur_safe_offset);
    // output
    cur_safe_offset = is_left ? (cur_safe_offset - half_av_width)
                              : (cur_safe_offset + half_av_width);
    new_safe_offset = is_left ? std::fmin(new_safe_offset, cur_safe_offset)
                              : std::fmax(new_safe_offset, cur_safe_offset);
    Log2DDS::LogDataV2(
        "safe_push_offset",
        absl::StrCat("is_lc_pause: ", is_lc_pause,
                     " ori safe_offset = ", safe_offset,
                     " obj id: ", traj.object_id(), " ori lat_safe: ", lat_safe,
                     " new safe_offset: ", new_safe_offset,
                     " obstacle_lat_speed: ", obstacle_lat_speed,
                     " pause_most_search_l: ", pause_most_search_l,
                     " push_most_search_l: ", push_most_search_l));
  }
  return new_safe_offset;
}

// use kinematic model to simulate the lc pause kinematic offset
absl::StatusOr<bool> KinematicPauseOffsetSimulation(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    double& pause_kinematic_offset, bool is_left) {
  auto convert_to_basement_angle = [](double base, double phi) {
    double diff = phi - base;
    double two_pi = 2 * M_PI;
    double result = std::fmod(diff + M_PI, two_pi) - M_PI;
    return result;
  };
  constexpr double kMinSteerRateLimit = 0.1;
  constexpr double kSampleTime = 0.05;
  constexpr double kMaxSimulationTime = 1.0;
  constexpr double kMaxLatAccelLimit = 2.0;
  constexpr double kMaxSteerRateMultiplier = 1.5;
  constexpr double kMinSteerRateMultiplier = 1.0;
  constexpr double KActiveSpeedThreshold = 1.0;
  constexpr double kActiveSteeringAngleThreshold = 1.0;
  const double wheel_base = vehicle_geom.wheel_base();
  constexpr double kStopSimSteerRad = 0.15;
  constexpr double kStopSimHeadingDegree = 0.5;
  constexpr double steer_straighten_time = 0.4;
  double stop_sim_rad = kStopSimHeadingDegree * M_PI / 180.0;
  const int max_iterations = static_cast<int>(kMaxSimulationTime / kSampleTime);

  DynamicState current_state = {
      .x = plan_start_point.path_point().x(),
      .y = plan_start_point.path_point().y(),
      .heading = plan_start_point.path_point().theta(),
      .curvature = plan_start_point.path_point().kappa(),
      .steer = plan_start_point.path_point().steer_angle(),
      .v = plan_start_point.v(),
      .lon_acc = plan_start_point.a(),
      .lon_jerk = plan_start_point.j()};

  const std::vector<double> speed_vec = {30, 80};
  const std::vector<double> steer_multiplier_vec = {kMaxSteerRateMultiplier,
                                                    kMinSteerRateMultiplier};
  double steer_multiplier = ad_byd::planning::math::interp1_inc(
      speed_vec, steer_multiplier_vec, Mps2Kph(current_state.v));
  ASSIGN_OR_RETURN(const auto init_state_sl,
                   drive_passage.QueryFrenetCoordinateAt(
                       {current_state.x, current_state.y}));
  ASSIGN_OR_RETURN(const auto init_lane_angle,
                   drive_passage.QueryTangentAngleAtS(init_state_sl.s));
  // early stop
  double init_delta_angle_rad =
      convert_to_basement_angle(current_state.heading, init_lane_angle);
  if ((is_left && init_delta_angle_rad > -stop_sim_rad &&
       current_state.steer < kStopSimSteerRad) ||
      (!is_left && init_delta_angle_rad < stop_sim_rad &&
       current_state.steer > -kStopSimSteerRad) ||
      current_state.v < KActiveSpeedThreshold ||
      std::fabs(current_state.steer) > kActiveSteeringAngleThreshold) {
    pause_kinematic_offset = init_state_sl.l;
    return true;
  }
  // assume steer angle rate
  double steer_angle_rate =
      std::max(std::fabs(current_state.steer) / steer_straighten_time,
               kMinSteerRateLimit);
  // std::max(
  //     std::fabs(current_state.steer) * steer_multiplier, kMinSteerRateLimit);
  // start simulation
  for (int i = 0; i < max_iterations; ++i) {
    double v = current_state.v;
    Control control_inpt = {
        .steer_rate = is_left ? -steer_angle_rate : steer_angle_rate,
        .acc = 0.0};
    VehicleKinematicModel::CalNextState(wheel_base, control_inpt, kSampleTime,
                                        &current_state, v);

    ASSIGN_OR_CONTINUE(const auto current_state_sl,
                       drive_passage.QueryFrenetCoordinateAt(
                           {current_state.x, current_state.y}));
    ASSIGN_OR_CONTINUE(const auto lane_angle,
                       drive_passage.QueryTangentAngleAtS(current_state_sl.s));

    double delta_angle_rad =
        convert_to_basement_angle(current_state.heading, lane_angle);
    if ((is_left && delta_angle_rad > -stop_sim_rad) ||
        (!is_left && delta_angle_rad < stop_sim_rad) ||
        i == max_iterations - 1) {
      ASSIGN_OR_CONTINUE(const auto final_state_sl,
                         drive_passage.QueryFrenetCoordinateAt(
                             {current_state.x, current_state.y}));
      pause_kinematic_offset = final_state_sl.l;
      if (is_left) {
        pause_kinematic_offset =
            std::max(pause_kinematic_offset, init_state_sl.l);
      } else {
        pause_kinematic_offset =
            std::min(pause_kinematic_offset, init_state_sl.l);
      }
      return true;
    }
  }
  return false;
}

double ComputeTargetLaneOffset(
    const DrivePassage& drive_passage, const FrenetCoordinate& cur_sl,
    const LaneChangeStateProto& lc_state,
    const ApolloTrajectoryPointProto& plan_start_point,
    absl::Span<const SpacetimeObjectTrajectory> obj_trajs,
    const FrenetBox& sl_box, const VehicleGeometryParamsProto& vehicle_geom,
    bool is_congestion_scene, PausePushSavedOffsetProto* saved_offset,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    const ObjectHistoryManager* obj_history_mgr) {
  constexpr double kLcPauseSavedOffsetUpdateBuffer = 0.3;
  constexpr double kLcPauseSavedOffsetNoObjUpdateBuffer = 0.3;
  constexpr double kLcPauseSavedOffsetBackPauseUpdateBuffer = 1.2;
  constexpr double KPushSavedOffsetUpdateBuffer = 0.1;
  constexpr double kMinLateralBuffer = 0.2;  // m.
  constexpr double kBackPauseMinLateralBuffer = 0.1;
  constexpr double kBackPauseMaxLateralBuffer = 0.35;  // m
  constexpr double kMinKinematicBuffer = 0.1;
  constexpr double kLargeOffsetThres = 100.0;
  constexpr double kNormalLateralMoveOffset = 0.3;
  constexpr double kPauseNormalLateralMoveOffset = 0.8;
  constexpr double kPauseLargerLateralMoveOffset = 1.0;
  constexpr double kBackPauseHighSpeed = 30.0;        // km/h
  constexpr double kBackPauseCarDeltaVThrd = 15.0;    // km/h
  constexpr double kBackPauseTruckDeltaVThrd = 10.0;  // km/h
  double half_av_width = vehicle_geom.width() * 0.5;
  double target_lane_offset = 0.0;
  bool is_pause_offset_cal_without_obj = false;
  enum PauseScene : int {
    UnknownScene = 0,
    BackPause,
    RiskBackPause,
    NormalPause,
    CongestionPause
  };
  if (lc_state.stage() != LaneChangeStage::LCS_PAUSE &&
      lc_state.push_state() == PushState::NONE_PUSH) {
    if (saved_offset != nullptr) {
      if (lc_state.stage() != LaneChangeStage::LCS_PAUSE) {
        saved_offset->set_pre_pause_offset(0.0);
      }
      if (lc_state.push_state() == PushState::NONE_PUSH) {
        saved_offset->set_pre_push_offset(0.0);
      }
      saved_offset->set_pre_pause_scene(
          PausePushSavedOffsetProto_PauseScene_UnknownScene);
    }
    return target_lane_offset;
  }
  // pause场景分类
  PauseScene pause_scene = PauseScene::UnknownScene;
  double unsafe_delta_speed = 0.0;  // debug
  bool fast_back_obs = false;
  bool large_unsafe_obs = false;
  bool near_back_obs = false;
  if (saved_offset == nullptr) return target_lane_offset;
  // push状态，给pause场景清空
  if (lc_state.stage() != LaneChangeStage::LCS_PAUSE &&
      lc_state.push_state() != PushState::NONE_PUSH) {
    pause_scene = PauseScene::UnknownScene;
    saved_offset->set_pre_pause_scene(
        PausePushSavedOffsetProto_PauseScene_UnknownScene);
  } else if (saved_offset->pre_pause_scene() ==
             PausePushSavedOffsetProto_PauseScene_BackPause) {
    pause_scene = PauseScene::BackPause;
    saved_offset->set_pre_pause_scene(
        PausePushSavedOffsetProto_PauseScene_BackPause);
  } else if (saved_offset->pre_pause_scene() ==
             PausePushSavedOffsetProto_PauseScene_RiskBackPause) {
    pause_scene = PauseScene::RiskBackPause;
    saved_offset->set_pre_pause_scene(
        PausePushSavedOffsetProto_PauseScene_RiskBackPause);
  } else if (is_congestion_scene ||
             saved_offset->pre_pause_scene() ==
                 PausePushSavedOffsetProto_PauseScene_CongestionPause) {
    pause_scene = PauseScene::CongestionPause;
    saved_offset->set_pre_pause_scene(
        PausePushSavedOffsetProto_PauseScene_CongestionPause);
  } else if (unsafe_object_ids != nullptr && unsafe_object_ids->size() > 0) {
    bool found_unsafe_obs = false;
    for (const auto& traj : obj_trajs) {
      if (unsafe_object_ids->find(traj.planner_object().id()) ==
          unsafe_object_ids->end()) {
        continue;
      }
      large_unsafe_obs = traj.object_type() == OT_LARGE_VEHICLE;
      double delta_v_thrd = traj.object_type() == OT_LARGE_VEHICLE
                                ? kBackPauseTruckDeltaVThrd
                                : kBackPauseCarDeltaVThrd;
      ASSIGN_OR_CONTINUE(const auto obs_frenet_box,
                         drive_passage.QueryFrenetBoxAt(
                             traj.planner_object().bounding_box(), false));
      double delta_dist = obs_frenet_box.s_min > sl_box.s_max
                              ? obs_frenet_box.s_min - sl_box.s_max
                          : obs_frenet_box.s_max < sl_box.s_min
                              ? obs_frenet_box.s_max - sl_box.s_min
                              : 0.0;
      // 计算后方车辆速度差
      double delta_speed = traj.pose().v() - plan_start_point.v();
      unsafe_delta_speed = Mps2Kph(delta_speed);
      near_back_obs =
          delta_dist < 0.001 && delta_dist > -8.0 &&
          std::abs(obs_frenet_box.center_l() - cur_sl.l) <
              ((vehicle_geom.width() + obs_frenet_box.width()) * 0.5 + 0.3) &&
          delta_speed > 0.5;
      // 障碍物速度较快，不安全车辆为大车，
      fast_back_obs = Mps2Kph(delta_speed) > delta_v_thrd;
      if (fast_back_obs || large_unsafe_obs || near_back_obs) {
        found_unsafe_obs = true;
        pause_scene = PauseScene::RiskBackPause;
        saved_offset->set_pre_pause_scene(
            PausePushSavedOffsetProto_PauseScene_RiskBackPause);
        break;
      }
    }
    if (!found_unsafe_obs) {
      if (Mps2Kph(plan_start_point.v()) > kBackPauseHighSpeed) {
        pause_scene = PauseScene::BackPause;
        saved_offset->set_pre_pause_scene(
            PausePushSavedOffsetProto_PauseScene_BackPause);
      } else {
        pause_scene = PauseScene::NormalPause;
        saved_offset->set_pre_pause_scene(
            PausePushSavedOffsetProto_PauseScene_NormalPause);
      }
    }
  } else if (Mps2Kph(plan_start_point.v()) > kBackPauseHighSpeed) {
    pause_scene = PauseScene::BackPause;
    saved_offset->set_pre_pause_scene(
        PausePushSavedOffsetProto_PauseScene_BackPause);
  } else {
    pause_scene = PauseScene::NormalPause;
    saved_offset->set_pre_pause_scene(
        PausePushSavedOffsetProto_PauseScene_NormalPause);
  }
  // pause
  bool back_pause_close_to_line = false;
  if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
    double kinematic_lat_offset = cur_sl.l;
    const auto lane_tangent = drive_passage.QueryTangentAtS(cur_sl.s);
    if (lane_tangent.ok()) {
      const double lat_v = plan_start_point.v() *
                           lane_tangent->CrossProd(Vec2d::FastUnitFromAngle(
                               plan_start_point.path_point().theta()));
      kinematic_lat_offset += std::copysign(
          Sqr(lat_v) * 0.5 / kComfortLaneChangeCancelLatAccel, lat_v);
    }
    double pause_sim_kinematic_lat_offset = 0.0;
    auto cal_kinematic_lat_offset_flag = KinematicPauseOffsetSimulation(
        drive_passage, plan_start_point, vehicle_geom,
        pause_sim_kinematic_lat_offset, lc_state.lc_left());
    double final_kinematic_lat_offset =
        cal_kinematic_lat_offset_flag.ok() &&
                cal_kinematic_lat_offset_flag.value()
            ? pause_sim_kinematic_lat_offset
            : kinematic_lat_offset;
    if (lc_state.lc_left()) {
      final_kinematic_lat_offset =
          std::max(pause_sim_kinematic_lat_offset, kinematic_lat_offset);
    } else {
      final_kinematic_lat_offset =
          std::min(pause_sim_kinematic_lat_offset, kinematic_lat_offset);
    }
    const auto boundaries =
        drive_passage.QueryEnclosingLaneBoundariesAtS(cur_sl.s);
    // To deal with virtual lanes with no boundaries other than curbs.
    const double right_offset =
        std::max(boundaries.right->lat_offset, -kMaxHalfLaneWidth);
    const double left_offset =
        std::min(boundaries.left->lat_offset, kMaxHalfLaneWidth);

    const std::vector<double> speed_vec = {40, 60};
    const std::vector<double> back_pause_buffer_vec = {
        kBackPauseMinLateralBuffer, kBackPauseMaxLateralBuffer};
    double back_pause_buffer = ad_byd::planning::math::interp1_inc(
        speed_vec, back_pause_buffer_vec, Mps2Kph(plan_start_point.v()));
    if (fast_back_obs) {
      back_pause_buffer = kBackPauseMaxLateralBuffer;
    }
    // 左变道暂停
    if (lc_state.lc_left()) {
      // 返回原车道贴线
      if (pause_scene == PauseScene::BackPause ||
          pause_scene == PauseScene::RiskBackPause) {
        double close_line_offset =
            right_offset - half_av_width - back_pause_buffer;
        back_pause_close_to_line = true;
        target_lane_offset =
            std::max(std::min(close_line_offset, cur_sl.l),
                     -ad_byd::planning::Constants::DEFAULT_LANE_WIDTH);
      } else {
        target_lane_offset =
            std::max(right_offset - half_av_width - kMinLateralBuffer,
                     final_kinematic_lat_offset - kMinKinematicBuffer);
        bool is_pause_congestion = (pause_scene == PauseScene::CongestionPause);
        if (is_pause_congestion) {
          target_lane_offset += 0.5;
        }
        double collision_free_offset = ComputeCollisionFreeOffset(
            drive_passage, cur_sl, plan_start_point, half_av_width, obj_trajs,
            true, true, sl_box, target_lane_offset, is_pause_congestion,
            unsafe_object_ids, obj_history_mgr);
        is_pause_offset_cal_without_obj =
            collision_free_offset > kLargeOffsetThres ? true : false;
        target_lane_offset =
            std::fmin(target_lane_offset, collision_free_offset);
        if (collision_free_offset < kDefaultHalfLaneWidth) {
          target_lane_offset =
              std::fmin(target_lane_offset, std::fmax(cur_sl.l, 0.0));
        }
        Log2DDS::LogDataV2(
            "collision_free_offset",
            absl::StrCat(collision_free_offset,
                         ", kinematic_lat_offset: ", kinematic_lat_offset,
                         ", pause_sim_kinematic_lat_offset: ",
                         pause_sim_kinematic_lat_offset));
      }
      // 右变道暂停
    } else {
      if (pause_scene == PauseScene::BackPause ||
          pause_scene == PauseScene::RiskBackPause) {
        double close_line_offset =
            left_offset + half_av_width + back_pause_buffer;
        back_pause_close_to_line = true;
        target_lane_offset =
            std::min(std::max(close_line_offset, cur_sl.l),
                     ad_byd::planning::Constants::DEFAULT_LANE_WIDTH);
      } else {
        target_lane_offset =
            std::min(left_offset + half_av_width + kMinLateralBuffer,
                     final_kinematic_lat_offset + kMinKinematicBuffer);
        bool is_pause_congestion = (pause_scene == PauseScene::CongestionPause);
        if (is_pause_congestion) {
          target_lane_offset -= 0.5;
        }
        double collision_free_offset = ComputeCollisionFreeOffset(
            drive_passage, cur_sl, plan_start_point, half_av_width, obj_trajs,
            false, true, sl_box, target_lane_offset, is_pause_congestion,
            unsafe_object_ids, obj_history_mgr);
        is_pause_offset_cal_without_obj =
            collision_free_offset < -kLargeOffsetThres ? true : false;
        target_lane_offset =
            std::fmax(target_lane_offset, collision_free_offset);
        if (collision_free_offset > -kDefaultHalfLaneWidth) {
          target_lane_offset =
              std::fmax(target_lane_offset, std::fmin(cur_sl.l, 0.0));
        }
        Log2DDS::LogDataV2(
            "collision_free_offset",
            absl::StrCat(collision_free_offset,
                         ", kinematic_lat_offset: ", kinematic_lat_offset,
                         ", pause_sim_kinematic_lat_offset: ",
                         pause_sim_kinematic_lat_offset));
      }
    }

    // if (std::abs(cur_sl.l - target_lane_offset) >
    //     kMaxLaneChangePauseRefCenterStep) {
    //   target_lane_offset =
    //       cur_sl.l + std::copysign(kMaxLaneChangePauseRefCenterStep,
    //                                target_lane_offset - cur_sl.l);
    // }
  } else {
    const bool is_congestion =
        (lc_state.push_state() == PushState::CONGESTION_LEFT_PUSH ||
         lc_state.push_state() == PushState::CONGESTION_RIGHT_PUSH);
    const double kDefaultPushOffset = is_congestion ? 0.6 : 0.6;
    if (lc_state.push_state() == PushState::LEFT_PUSH ||
        lc_state.push_state() == PushState::CONGESTION_LEFT_PUSH) {
      double safe_push_offset = ComputeCollisionFreeOffset(
          drive_passage, cur_sl, plan_start_point, half_av_width, obj_trajs,
          true, false, sl_box, 0.0, false, nullptr, obj_history_mgr);
      target_lane_offset =
          std::fmin(std::fmax(safe_push_offset, 0.0), kDefaultPushOffset);
      Log2DDS::LogDataV2("safe_push_offset", absl::StrCat(safe_push_offset));
    } else if (lc_state.push_state() == PushState::RIGHT_PUSH ||
               lc_state.push_state() == PushState::CONGESTION_RIGHT_PUSH) {
      double safe_push_offset = ComputeCollisionFreeOffset(
          drive_passage, cur_sl, plan_start_point, half_av_width, obj_trajs,
          false, false, sl_box, 0.0, false, nullptr, obj_history_mgr);
      target_lane_offset =
          std::fmax(std::fmin(safe_push_offset, 0.0), -kDefaultPushOffset);
      Log2DDS::LogDataV2("safe_push_offset", absl::StrCat(safe_push_offset));
    }
    Log2DDS::LogDataV2("push_lane_offset", absl::StrCat(target_lane_offset));
  }
  double saved_offset_update_buffer =
      lc_state.stage() == LaneChangeStage::LCS_PAUSE
          ? is_pause_offset_cal_without_obj
                ? kLcPauseSavedOffsetNoObjUpdateBuffer
            : back_pause_close_to_line
                ? kLcPauseSavedOffsetBackPauseUpdateBuffer
                : kLcPauseSavedOffsetUpdateBuffer
          : KPushSavedOffsetUpdateBuffer;
  if (saved_offset != nullptr) {
    if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
      if (abs(target_lane_offset - saved_offset->pre_pause_offset()) <
          saved_offset_update_buffer) {
        target_lane_offset = saved_offset->pre_pause_offset();
      }
      saved_offset->set_pre_pause_offset(target_lane_offset);
    } else {
      if (abs(target_lane_offset - saved_offset->pre_push_offset()) <
          saved_offset_update_buffer) {
        target_lane_offset = saved_offset->pre_push_offset();
      }
      saved_offset->set_pre_push_offset(target_lane_offset);
    }
  }
  double base_l = cur_sl.l;
  // 仅在push状态下
  if (lc_state.stage() != LaneChangeStage::LCS_PAUSE &&
      lc_state.push_state() != PushState::NONE_PUSH &&
      ((target_lane_offset > std::numeric_limits<double>::epsilon() &&
        cur_sl.l < -std::numeric_limits<double>::epsilon()) ||
       (target_lane_offset < -std::numeric_limits<double>::epsilon() &&
        cur_sl.l > std::numeric_limits<double>::epsilon()))) {
    base_l = 0.0;
  }
  double delta_l_offset = target_lane_offset - base_l;
  double offset_limit = kNormalLateralMoveOffset;
  if (lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
    offset_limit = kMaxLaneChangePauseRefCenterStep;
    if (pause_scene == PauseScene::RiskBackPause) {
      const std::vector<double> speed_vec = {60, 130};
      const std::vector<double> offset_limit_vec = {
          kRiskMaxLaneChangePauseRefCenterStep,
          kRiskMinLaneChangePauseRefCenterStep};
      offset_limit = ad_byd::planning::math::interp1_inc(
          speed_vec, offset_limit_vec, Mps2Kph(plan_start_point.v()));
    }
  }
  delta_l_offset = std::clamp(delta_l_offset, -offset_limit, offset_limit);
  double target_lane_executing_offset =
      std::clamp(base_l + delta_l_offset, -std::abs(target_lane_offset),
                 std::abs(target_lane_offset));
  Log2DDS::LogDataV2(
      "target_lane_offset",
      absl::StrCat(target_lane_offset, ", cur_l: ", cur_sl.l,
                   ", executing_offset: ", target_lane_executing_offset,
                   ", push state: ", lc_state.push_state(), ", pause_scene: ",
                   pause_scene, ", unsafe_delta_speed: ", unsafe_delta_speed,
                   ", fast_back_obs: ", fast_back_obs, ", large_unsafe_obs: ",
                   large_unsafe_obs, ", near_back_obs: ", near_back_obs,
                   ", offset_limit: ", offset_limit));
  return target_lane_executing_offset;
}

PathSlBoundary BuildPathSlBoundary(
    const DrivePassage& drive_passage, std::vector<double> s_vec,
    std::vector<double> ref_center_l, PathBoundary inner_boundary,
    PathBoundary outer_boundary, PathBoundary opt_outer_boundary,
    CenterLineOffsetType offset_type, double offset_value) {
  const int n = s_vec.size();
  std::vector<Vec2d> inner_right_xy, inner_left_xy, outer_right_xy,
      outer_left_xy, opt_outer_right_xy, opt_outer_left_xy, ref_center_xy;
  inner_right_xy.reserve(n);
  inner_left_xy.reserve(n);
  outer_right_xy.reserve(n);
  outer_left_xy.reserve(n);
  opt_outer_right_xy.reserve(n);
  opt_outer_left_xy.reserve(n);
  ref_center_xy.reserve(n);
  for (int i = 0; i < n; ++i) {
    const auto& station = drive_passage.station(StationIndex(i));
    inner_left_xy.emplace_back(station.lat_point(inner_boundary.left(i)));
    inner_right_xy.emplace_back(station.lat_point(inner_boundary.right(i)));
    outer_right_xy.emplace_back(station.lat_point(outer_boundary.right(i)));
    outer_left_xy.emplace_back(station.lat_point(outer_boundary.left(i)));
    opt_outer_right_xy.emplace_back(
        station.lat_point(opt_outer_boundary.right(i)));
    opt_outer_left_xy.emplace_back(
        station.lat_point(opt_outer_boundary.left(i)));
    ref_center_xy.push_back(station.lat_point(ref_center_l[i]));
  }

  return PathSlBoundary(
      std::move(s_vec), std::move(ref_center_l),
      outer_boundary.moved_right_vec(), outer_boundary.moved_left_vec(),
      opt_outer_boundary.moved_right_vec(), opt_outer_boundary.moved_left_vec(),
      inner_boundary.moved_right_vec(), inner_boundary.moved_left_vec(),
      std::move(ref_center_xy), std::move(outer_right_xy),
      std::move(outer_left_xy), std::move(opt_outer_right_xy),
      std::move(opt_outer_left_xy), std::move(inner_right_xy),
      std::move(inner_left_xy), offset_type, offset_value);
}

std::vector<double> ComputeSmoothedReferenceLine(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const SmoothedReferenceLineResultMap& smooth_result_map) {
  const int n = drive_passage.size();
  std::vector<double> smoothed_reference_center(n, 0.0);
  absl::flat_hash_map<st::mapping::ElementId,
                      PiecewiseLinearFunction<double, double>>
      lane_id_to_smoothed_lateral_offset;

  int cur_begin = -1;
  const auto& lane_path = drive_passage.lane_path();
  for (int i = 0; i < lane_path.size(); ++i) {
    const bool should_smooth = IsTurningLanePath(psmm, lane_path.lane_id(i));

    if (cur_begin == -1 && should_smooth) {
      // rise.
      cur_begin = i;
    }
    if (cur_begin != -1 && !should_smooth) {
      // drop.
      const std::vector<mapping::ElementId> lane_ids(
          lane_path.lane_ids().begin() + cur_begin,
          lane_path.lane_ids().begin() + i);
      const auto smoothed_result =
          smooth_result_map.FindOverlapSmoothedResult(lane_ids);
      if (smoothed_result.ok()) {
        std::vector<std::string> str_lane_ids;
        for (const auto& i : lane_ids) {
          str_lane_ids.emplace_back(absl::StrCat(i));
        }
        Log2DDS::LogDataV2("smooth_debug", str_lane_ids);
        lane_id_to_smoothed_lateral_offset.insert(
            smoothed_result->lane_id_to_smoothed_lateral_offset.begin(),
            smoothed_result->lane_id_to_smoothed_lateral_offset.end());
      }
      cur_begin = -1;
    }
  }
  if (cur_begin != -1) {
    // drop.
    const std::vector<mapping::ElementId> lane_ids(
        lane_path.lane_ids().begin() + cur_begin, lane_path.lane_ids().end());
    if (lane_ids.size() > 1 || lane_path.lane_ids().size() < 2) {
      const auto smoothed_result =
          smooth_result_map.FindOverlapSmoothedResult(lane_ids);
      if (smoothed_result.ok()) {
        std::vector<std::string> str_lane_ids;
        for (const auto& i : lane_ids) {
          str_lane_ids.emplace_back(absl::StrCat(i));
        }
        Log2DDS::LogDataV2("smooth_debug", str_lane_ids);
        lane_id_to_smoothed_lateral_offset.insert(
            smoothed_result->lane_id_to_smoothed_lateral_offset.begin(),
            smoothed_result->lane_id_to_smoothed_lateral_offset.end());
      }
    }
  }

  std::vector<std::string> str_lane_ids;
  for (const auto& i : lane_path.lane_ids()) {
    str_lane_ids.emplace_back(absl::StrCat(i));
  }

  SmoothedReferenceCenterResult smooth_results = {
      .lane_id_to_smoothed_lateral_offset =
          std::move(lane_id_to_smoothed_lateral_offset)};
  for (int i = 0; i < n; ++i) {
    const mapping::LanePoint& lane_point =
        drive_passage.station(StationIndex(i)).GetLanePoint();
    const auto smoothed_l = smooth_results.GetSmoothedLateralOffset(lane_point);
    if (smoothed_l.ok()) {
      smoothed_reference_center[i] = *smoothed_l;
    }
  }

  return smoothed_reference_center;
}

}  // namespace st::planning
