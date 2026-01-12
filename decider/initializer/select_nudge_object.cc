

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "decider/initializer/select_nudge_object.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/util.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/log_data.h"
#include "plan_common/trajectory_point.h"
#include "predictor/prediction_object_state.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st {
namespace planning {
namespace initializer {

constexpr int kNudgeEffectiveIndexRange = 5;
constexpr double kMaxNudgeBuffer = 0.2;
constexpr double kThetaThreshold = 0.08;
constexpr double kSRangeExtend = 3.0;

struct NudgeState {
  double l_min = std::numeric_limits<double>::infinity();
  double l_min_lon_s = std::numeric_limits<double>::infinity();
  const SpacetimeObjectTrajectory* min_dist_object_ptr = nullptr;

  void reset() {
    l_min = std::numeric_limits<double>::infinity();
    l_min_lon_s = std::numeric_limits<double>::infinity();
    min_dist_object_ptr = nullptr;
  }
};

std::vector<prediction::PredictionObjectState> SampleObjectStates(
    int trajectory_steps, double trajectory_time_step,
    absl::Span<const prediction::PredictionObjectState> states) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  const int sample_step =
      static_cast<int>(trajectory_time_step / kTrajectoryTimeStep + 0.5);
  std::vector<prediction::PredictionObjectState> sampled_states;
  sampled_states.reserve(states.size() / sample_step);
  for (int i = 0; i < trajectory_steps; ++i) {
    if (i * sample_step >= states.size()) break;
    sampled_states.push_back(states[i * sample_step]);
  }
  return sampled_states;
}

// Check if a nudge interval effective: if interval is close to start point,
// duration is long enough and l offset is large enough.
bool IsIntervalEffective(double plan_start_point_theta_diff_to_lane,
                         int free_index, int start_nudge_index, int direction,
                         int end_nudge_index, double l_nudge_max) {
  const bool in_loop_effective =
      start_nudge_index < free_index ||
      (Sign(plan_start_point_theta_diff_to_lane) == direction &&
       std::abs(plan_start_point_theta_diff_to_lane) > kThetaThreshold);

  return in_loop_effective &&
         (end_nudge_index - start_nudge_index) > kNudgeEffectiveIndexRange &&
         std::abs(l_nudge_max) > kMaxNudgeBuffer;
}

void GetNudgeObjectIdForStep(
    double& l_min, const SpacetimeObjectTrajectory*& min_dist_object_ptr,
    double& l_min_lon_s, int direction, const FrenetBox& object_frenet_box,
    const FrenetBox& av_frenet_box,
    const SpacetimeObjectTrajectory* object_ptr) {
  int object_side = 0;
  if (av_frenet_box.l_min > object_frenet_box.l_max) {
    object_side = 1;
  } else if (object_frenet_box.l_min > av_frenet_box.l_max) {
    object_side = -1;
  }
  if (object_side == direction) {
    const bool has_s_overlap =
        av_frenet_box.s_max > object_frenet_box.s_min - kSRangeExtend &&
        av_frenet_box.s_min < object_frenet_box.s_max + kSRangeExtend;
    if (has_s_overlap) {
      double l_diff = 0.0;
      if (direction > 0) {
        l_diff = av_frenet_box.l_min - object_frenet_box.l_max;
      } else {
        l_diff = object_frenet_box.l_min - av_frenet_box.l_max;
      }
      if (l_diff < l_min) {
        l_min = l_diff;
        l_min_lon_s = object_frenet_box.center_s();
        min_dist_object_ptr = object_ptr;
      }
    }
  }
}

bool IsVru(const SpacetimeObjectTrajectory& traj) {
  return traj.object_type() == ObjectType::OT_MOTORCYCLIST ||
         traj.object_type() == ObjectType::OT_CYCLIST ||
         traj.object_type() == ObjectType::OT_TRICYCLIST ||
         traj.object_type() == ObjectType::OT_PEDESTRIAN;
}

absl::StatusOr<NudgeInfos> SelectNudgeObjectId(
    int trajectory_steps, double trajectory_time_step, bool is_lane_change,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const std::vector<TrajectoryPoint>& result_points,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const int plan_id) {
  CHECK_GT(trajectory_steps, 0);
  CHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);
  const std::string& prefix = Log2DDS::TaskPrefix(plan_id);
  Log2DDS::LogDataV2("init_nudge_object_info", prefix);
  NudgeInfos nudge_object_infos;
  // Only extrack nudge object id when lane keeping.
  if (is_lane_change) return nudge_object_infos;
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
  constexpr double kBorrowLOffsetThreshold = 0.6;
  constexpr double kLOffsetThreshold = 0.1;
  constexpr int kNudgeEffectiveStartIndex = 5;

  std::optional<int> direction;
  std::optional<double> l_nudge_max;
  std::optional<int> start_nudge_index;
  std::optional<int> end_nudge_index;

  // Loop for nudge interval obtained.
  for (int k = 0; k < free_index && k < result_points.size(); ++k) {
    const auto& traj_point = result_points[k];
    const auto& frenet_pt =
        drive_passage.QueryFrenetCoordinateAt(traj_point.pos());
    if (!frenet_pt.ok()) {
      return absl::OutOfRangeError(
          absl::StrFormat("%d traj point out of drive passage range.", k));
    }
    const double center_l_at_s =
        path_sl_boundary.QueryReferenceCenterL(frenet_pt->s);
    const double l_offset = frenet_pt->l - center_l_at_s;
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
    const bool l_offset_noneffective =
        (*direction > 0 ? l_offset <= kLOffsetThreshold
                        : l_offset >= -kLOffsetThreshold) ||
        k == free_index - 1;
    if (l_nudge_max.has_value() && !end_nudge_index.has_value() &&
        l_offset_noneffective) {
      end_nudge_index = k - 1;
      if (*end_nudge_index > kNudgeEffectiveStartIndex &&
          IsIntervalEffective(plan_start_point_theta_diff_to_lane, free_index,
                              *start_nudge_index, *direction, *end_nudge_index,
                              *l_nudge_max)) {
        // Secondly, Extract nudge object id: In nudge the interval, find object
        // whose traj closest to av traj and on the side matching with nudge
        // direction.
        NudgeState nudgeState;
        std::vector<FrenetBox> av_sl_boxes;
        // Get av sl boxes first.
        av_sl_boxes.reserve(free_index);
        for (int k = 0; k < free_index && k < result_points.size(); ++k) {
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
               k < *end_nudge_index && k < states.size() && k < free_index;
               ++k) {
            if (stationary_object_frenet_box.has_value()) {
              GetNudgeObjectIdForStep(
                  nudgeState.l_min, nudgeState.min_dist_object_ptr,
                  nudgeState.l_min_lon_s, *direction,
                  *stationary_object_frenet_box, av_sl_boxes[k], &traj);
            } else {
              const auto& state = states[k];
              const auto& contour = state.contour;
              const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(
                  contour, /*zone_checking*/ false);
              if (!frenet_box_or.ok()) {
                break;
              }
              GetNudgeObjectIdForStep(nudgeState.l_min,
                                      nudgeState.min_dist_object_ptr,
                                      nudgeState.l_min_lon_s, *direction,
                                      *frenet_box_or, av_sl_boxes[k], &traj);
            }
          }
          const double kNudgeBuffer = IsVru(traj) ? 2.0 : 1.2;
          const double trajectory_max_dis =
              result_points.back().s() +
              vehicle_geometry_params.front_edge_to_center() + kSRangeExtend;
          if (nudgeState.l_min < kNudgeBuffer && l_nudge_max.has_value() &&
              nudgeState.l_min_lon_s < trajectory_max_dis) {
            NudgeObjectInfo nudge_object_info;
            nudge_object_info.id =
                std::string(nudgeState.min_dist_object_ptr->object_id());
            nudge_object_info.direction = *direction;
            const auto object_frenet_box = drive_passage.QueryFrenetBoxAt(
                nudgeState.min_dist_object_ptr->bounding_box());
            if (!object_frenet_box.ok()) {
              return absl::OutOfRangeError("Object is not on drive passage.");
            }
            nudge_object_info.arc_dist_to_object = std::max(
                0.0, object_frenet_box->s_min - av_sl_boxes.front().s_max);
            nudge_object_info.type =
                nudgeState.min_dist_object_ptr->object_type();
            nudge_object_info.nudge_state = NudgeObjectInfo::NudgeState::NUDGE;
            if (std::fabs(l_nudge_max.value()) > kBorrowLOffsetThreshold) {
              nudge_object_info.nudge_state =
                  NudgeObjectInfo::NudgeState::BORROW;
            }
            Log2DDS::LogDataV2(
                "init_nudge_object_info",
                absl::StrCat("id: ", std::string(nudge_object_info.id),
                             ", direction: ", *direction,
                             " ,arc_dist_to_object: ",
                             nudge_object_info.arc_dist_to_object,
                             ", l_nudge_max: ", *l_nudge_max));
            nudge_object_infos.addNudgeInfo(std::move(nudge_object_info));
            nudgeState.reset();
          }
        }
        break;
      } else {
        direction.reset();
        l_nudge_max.reset();
        start_nudge_index.reset();
        end_nudge_index.reset();
      }
    }
  }
  return nudge_object_infos;
}
}  // namespace initializer
}  // namespace planning
}  // namespace st
