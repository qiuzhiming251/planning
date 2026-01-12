

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/util.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "planner/trajectory_optimizer/ddp/hmi_util.h"
#include "planner/trajectory_optimizer/ddp/object_cost_util.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "plan_common/log_data.h"

namespace st {
namespace planning {
namespace optimizer {
absl::StatusOr<std::optional<NudgeObjectInfo>> ExtractNudgeObjectId(
    int trajectory_steps, double trajectory_time_step, LaneChangeStage lc_stage,
    PushState push_dir, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const std::vector<TrajectoryPoint>& result_points,
    absl::Span<const ApolloTrajectoryPointProto> previous_trajectory,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<std::string>& final_cost_debug,
    const NudgeObjectInfo* previous_nudge_object_info) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);

  std::optional<NudgeObjectInfo> nudge_object_info_optional;
  // Only extrack nudge object id when lane keeping.
  if (lc_stage != LaneChangeStage::LCS_NONE || push_dir != PushState::NONE_PUSH)
    return nudge_object_info_optional;
  // If trajectory in front of av have nudge behavior.
  const auto& plan_start_point = result_points.front();
  const auto& plan_start_frenet_point =
      drive_passage.QueryFrenetCoordinateAt(plan_start_point.pos());
  if (!plan_start_frenet_point.ok()) {
    return absl::OutOfRangeError(
        "Plan start point don't in drive passage range.");
  }
  const auto start_point_lane_theta =
      drive_passage.QueryTangentAngleAtS(plan_start_frenet_point->s);
  if (!start_point_lane_theta.ok()) {
    return absl::OutOfRangeError(
        "Plan start point s don't in drive passage range.");
  }
  const double plan_start_point_theta_diff_to_lane =
      NormalizeAngle(plan_start_point.theta() - *start_point_lane_theta);

  // Firstly, Obtain effective nudge intervals.
  constexpr double kBorrowLOffsetThreshold = 0.8;
  constexpr double kLOffsetThreshold = 0.1;
  const int kNudgeEffectiveStartIndex =
      previous_nudge_object_info && (previous_nudge_object_info->nudge_state ==
                                         NudgeObjectInfo::NudgeState::NUDGE ||
                                     previous_nudge_object_info->nudge_state ==
                                         NudgeObjectInfo::NudgeState::BORROW)
          ? 10
          : 5;
  constexpr double kKeepNudgeStateMinLat = 0.1;  // m

  const double start_point_center_l_at_s =
      path_sl_boundary.QueryReferenceCenterL(plan_start_frenet_point->s);
  const bool keep_nudge_state =
      previous_nudge_object_info &&
      std::fabs(plan_start_frenet_point->l - start_point_center_l_at_s) >
          kKeepNudgeStateMinLat;
  // Check if a nudge interval effective: if interval is close to start
  // point, duration is long enough and l offset is large enough.
  const auto is_interval_effective =
      [&plan_start_point_theta_diff_to_lane, keep_nudge_state,
       kNudgeEffectiveStartIndex](int start_nudge_index, int direction,
                                  int end_nudge_index,
                                  double l_nudge_max) -> bool {
    constexpr int kNudgeEffectiveIndexRange = 10;
    constexpr double kMaxNudgeBuffer = 0.2;
    constexpr double kThetaThreshold = 0.08;
    const bool in_loop_effective =
        start_nudge_index < kNudgeEffectiveStartIndex ||
        (Sign(plan_start_point_theta_diff_to_lane) == direction &&
         std::abs(plan_start_point_theta_diff_to_lane) > kThetaThreshold);
    return (in_loop_effective &&
            (end_nudge_index - start_nudge_index) > kNudgeEffectiveIndexRange &&
            std::abs(l_nudge_max) > kMaxNudgeBuffer) ||
           keep_nudge_state;
  };

  std::optional<int> direction;
  std::optional<double> l_nudge_max;
  std::optional<int> start_nudge_index;
  std::optional<int> end_nudge_index;

  const double ref_offset = path_sl_boundary.offset_type_value().first ==
                                    CENTER_LINE_OFFSET_LARGE_VEHICLE_AVOID
                                ? path_sl_boundary.offset_type_value().second
                                : 0.0;
  // Loop for nudge interval obtained.
  for (int k = 0; k < free_index; ++k) {
    const auto& traj_point = result_points[k];
    const auto& frenet_pt =
        drive_passage.QueryFrenetCoordinateAt(traj_point.pos());
    if (!frenet_pt.ok()) {
      return absl::OutOfRangeError(
          absl::StrFormat("%d traj point out of drive passage range.", k));
    }
    const double center_l_at_s =
        path_sl_boundary.QueryReferenceCenterL(frenet_pt->s);
    const double l_offset = frenet_pt->l - center_l_at_s + ref_offset;
    if (!start_nudge_index.has_value() &&
        std::abs(l_offset) > kLOffsetThreshold) {
      start_nudge_index = k;
    }
    if (start_nudge_index.has_value()) {
      if (!l_nudge_max.has_value()) {
        l_nudge_max = l_offset;
        direction = Sign(l_offset);
      } else {
        if (Sign(l_offset) == *direction) {
          if (*direction > 0) {
            l_nudge_max = std::max(*l_nudge_max, l_offset);
          } else {
            l_nudge_max = std::min(*l_nudge_max, l_offset);
          }
        }
      }
    }
    // Get end index of the nudge interval, and if current nudge interval is
    // noneffective, clear state and find next interval.
    const bool l_offset_noneffective =
        (*direction > 0 ? l_offset <= kLOffsetThreshold
                        : l_offset >= -kLOffsetThreshold) ||
        k == free_index - 1;
    if (l_nudge_max.has_value() && !end_nudge_index.has_value() &&
        l_offset_noneffective) {
      end_nudge_index = k - 1;
      if (!is_interval_effective(*start_nudge_index, *direction,
                                 *end_nudge_index, *l_nudge_max) &&
          *end_nudge_index < kNudgeEffectiveStartIndex) {
        direction.reset();
        l_nudge_max.reset();
        start_nudge_index.reset();
        end_nudge_index.reset();
      } else {
        break;
      }
    } else {
      continue;
    }
  }

  // Secondly, Extract nudge object id: In nudge the interval, find object whose
  // traj closest to av traj and on the side matching with nudge direction.
  double l_min_static = std::numeric_limits<double>::infinity();
  double l_min_dynamic = std::numeric_limits<double>::infinity();
  double s_min_static = std::numeric_limits<double>::infinity();
  double s_min_dynamic = std::numeric_limits<double>::infinity();

  const SpacetimeObjectTrajectory* min_dist_static_object_ptr = nullptr;
  const SpacetimeObjectTrajectory* min_dist_dynamic_object_ptr = nullptr;

  std::vector<FrenetBox> av_sl_boxes;
  if (l_nudge_max.has_value()) {
    Log2DDS::LogDataV0("nudge_debug",
                       absl::StrCat("l_nudge_max: ", l_nudge_max.value()));
    Log2DDS::LogDataV0("nudge_debug",
                       absl::StrCat("start_nudge_index: ", *start_nudge_index));
    Log2DDS::LogDataV0("nudge_debug",
                       absl::StrCat("end_nudge_index: ", *end_nudge_index));
    Log2DDS::LogDataV0("nudge_debug",
                       absl::StrCat("keep_nudge_state: ", keep_nudge_state));
  }
  if (l_nudge_max.has_value() &&
      is_interval_effective(*start_nudge_index, *direction, *end_nudge_index,
                            *l_nudge_max)) {
    std::set<int> avoid_obs;
    double cost_threshold = 1e-4;
    for (const auto& cost : final_cost_debug) {
      size_t start = cost.find("for ");
      size_t end = std::string::npos;
      if (start != std::string::npos) {
        start += 4;
        end = cost.find("-idx", start);
      }

      if (end != std::string::npos) {
        std::string id_str = cost.substr(start, end - start);

        int id = std::stoi(id_str);

        size_t cost_start = cost.find("cost: ");
        // Extract the cost from the second part
        if (cost_start != std::string::npos) {
          std::string cost_str = cost.substr(cost_start + 6);
          double cost_value = std::stod(cost_str);
          if (cost_value < cost_threshold) {
            continue;
          }

          avoid_obs.insert(id);
        }
      }
    }

    std::string s;
    for (const auto& it : avoid_obs) {
      s = s + " " + std::to_string(it);
    }
    if (!s.empty()) {
      Log2DDS::LogDataV0("nudge_debug", absl::StrCat("nudge obs idx:", s));
    }

    // Get av sl boxes first.
    av_sl_boxes.reserve(free_index);
    for (int k = 0; k < free_index; ++k) {
      const auto& traj_point = result_points[k];
      const auto box = ComputeAvBox(traj_point.pos(), traj_point.theta(),
                                    vehicle_geometry_params);
      auto frenet_box = drive_passage.QueryFrenetBoxAt(box);
      if (!frenet_box.ok()) {
        return absl::OutOfRangeError(absl::StrFormat(
            "%d traj point box out of drive passage range.", k));
      }
      av_sl_boxes.push_back(*frenet_box);
    }
    const double pre_trajectory_max_dis =
        previous_trajectory.empty() ? 0.0
        : previous_trajectory.rbegin()->has_path_point()
            ? (previous_trajectory.rbegin()->path_point().has_s()
                   ? previous_trajectory.rbegin()->path_point().s()
                   : 0.0)
            : 0.0;

    const auto get_nudge_object_id_for_step =
        [&s_min_dynamic, &s_min_static, &l_min_dynamic, &l_min_static,
         &min_dist_dynamic_object_ptr, &min_dist_static_object_ptr,
         previous_nudge_object_info, pre_trajectory_max_dis](
            int direction, const FrenetBox& object_frenet_box,
            const FrenetBox& av_frenet_box,
            const SpacetimeObjectTrajectory* object_ptr) {
          const double kSRangeExtend =
              2.0 + (previous_nudge_object_info ? 2.0 : 0.0);
          int object_side = 0;
          if (av_frenet_box.l_min > object_frenet_box.l_max) {
            object_side = 1;
          } else if (object_frenet_box.l_min > av_frenet_box.l_max) {
            object_side = -1;
          }
          if (object_side == direction) {
            const bool has_s_overlap = !(
                object_frenet_box.s_min > av_frenet_box.s_max + kSRangeExtend ||
                object_frenet_box.s_max < av_frenet_box.s_min - kSRangeExtend);
            if (has_s_overlap) {
              double l_diff = 0.0;
              if (direction > 0) {
                l_diff = av_frenet_box.l_min - object_frenet_box.l_max;
              } else {
                l_diff = object_frenet_box.l_min - av_frenet_box.l_max;
              }

              if (object_ptr->is_stationary()) {
                if (l_diff < l_min_static &&
                    ((object_frenet_box.center_s() < pre_trajectory_max_dis) ||
                     (previous_nudge_object_info &&
                      previous_nudge_object_info->id ==
                          object_ptr->object_id()))) {
                  s_min_static = object_frenet_box.center_s();
                  l_min_static = l_diff;
                  min_dist_static_object_ptr = object_ptr;
                }
              } else {
                if (object_frenet_box.center_s() < s_min_dynamic &&
                    ((object_frenet_box.center_s() < pre_trajectory_max_dis) ||
                     (previous_nudge_object_info &&
                      previous_nudge_object_info->id ==
                          object_ptr->object_id()))) {
                  s_min_dynamic = object_frenet_box.center_s();
                  l_min_dynamic = l_diff;
                  min_dist_dynamic_object_ptr = object_ptr;
                }
              }
            }
          }
        };

    const auto& spacetime_trajs = st_planner_object_traj.trajectories;
    const int num_trajs = spacetime_trajs.size();
    // Loop to find object closest to av traj.
    for (int i = 0; i < num_trajs; ++i) {
      const auto& traj = spacetime_trajs[i];
      const auto states = SampleObjectStates(
          trajectory_steps, trajectory_time_step, traj.states());
      std::optional<FrenetBox> stationary_object_frenet_box;
      if (traj.is_stationary()) {
        const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(
            traj.contour(), /*zone_checking */ false);
        if (!frenet_box_or.ok()) {
          break;
        }
        stationary_object_frenet_box = *frenet_box_or;
      }
      for (int k = *start_nudge_index;
           k < *end_nudge_index && k < states.size() && k < free_index; ++k) {
        if (!prediction::IsStaticObjectType(traj.object_type()) &&
            avoid_obs.find(std::stoi(std::string(traj.object_id()))) ==
                avoid_obs.end()) {
          break;
        }

        if (stationary_object_frenet_box.has_value()) {
          get_nudge_object_id_for_step(
              *direction, *stationary_object_frenet_box, av_sl_boxes[k], &traj);
        } else {
          const auto& state = states[k];
          const auto& contour = state.contour;
          const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(
              contour, /*zone_checking*/ false);
          if (!frenet_box_or.ok()) {
            break;
          }
          get_nudge_object_id_for_step(*direction, *frenet_box_or,
                                       av_sl_boxes[k], &traj);
        }
      }
    }
  }

  const SpacetimeObjectTrajectory* min_dist_object_ptr = nullptr;
  double l_min = std::numeric_limits<double>::infinity();

  if (min_dist_dynamic_object_ptr == nullptr &&
      min_dist_static_object_ptr != nullptr) {
    min_dist_object_ptr = min_dist_static_object_ptr;
    l_min = l_min_static;

  } else if (min_dist_dynamic_object_ptr != nullptr &&
             min_dist_static_object_ptr == nullptr) {
    min_dist_object_ptr = min_dist_dynamic_object_ptr;
    l_min = l_min_dynamic;

  } else {
    if (l_min_static < l_min_dynamic) {
      min_dist_object_ptr = min_dist_static_object_ptr;
      l_min = l_min_static;

    } else {
      min_dist_object_ptr = min_dist_dynamic_object_ptr;
      l_min = l_min_dynamic;
    }
  }

  Log2DDS::LogDataV0("nudge_debug", absl::StrCat("l_min: ", l_min));

  // TODO: Output nudge id and direction to hmi.
  const double kNudgeBuffer = previous_nudge_object_info ? 1.8 : 1.2;  // m
  if (l_min < kNudgeBuffer && l_nudge_max.has_value()) {
    NudgeObjectInfo nudge_object_info;
    nudge_object_info.id = std::string(min_dist_object_ptr->object_id());
    nudge_object_info.direction = *direction;

    const auto object_frenet_box =
        drive_passage.QueryFrenetBoxAt(min_dist_object_ptr->bounding_box());
    if (!object_frenet_box.ok()) {
      return absl::OutOfRangeError("Object is not on drive passage.");
    }
    nudge_object_info.arc_dist_to_object =
        std::max(0.0, object_frenet_box->s_min - av_sl_boxes.front().s_max);

    nudge_object_info.type = min_dist_object_ptr->object_type();

    nudge_object_info.nudge_state = NudgeObjectInfo::NudgeState::NUDGE;
    if (std::fabs(l_nudge_max.value()) > kBorrowLOffsetThreshold) {
      nudge_object_info.nudge_state = NudgeObjectInfo::NudgeState::BORROW;
    }
    nudge_object_info_optional = std::move(nudge_object_info);
  }

  return nudge_object_info_optional;
}

}  // namespace optimizer
}  // namespace planning
}  // namespace st
