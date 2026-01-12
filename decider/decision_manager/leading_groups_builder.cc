

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "plan_common/log.h"
#include "absl/types/span.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

#include "plan_common/math/util.h"
#include "plan_common/log_data.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "decider/decision_manager/decision_util.h"
#include "decider/decision_manager/leading_groups_builder.h"

namespace st::planning {

namespace {
constexpr double kFullyEnterTargetLateralThreshold = 1.2;  // m.
constexpr double kObjTrajectoryStep = 0.1;                 // s.
constexpr double kObjMostReliableTrajectoryTimeMax = 1.5;  // s.
constexpr double kObjReliableTrajectoryTimeMax = 3.0;      // s.
constexpr double kEpsilon = 1e-5;

constexpr int kObjMostReliableTrajectoryIndexMax =
    std::max(1, static_cast<int>(std::round(kObjMostReliableTrajectoryTimeMax /
                                            kObjTrajectoryStep)));
constexpr int kObjReliableTrajectoryIndexMax =
    std::max(1, static_cast<int>(std::round(kObjReliableTrajectoryTimeMax /
                                            kObjTrajectoryStep)));
struct LeadingObjectTrajectoryInfo {
  // Project potential leading object trajectory onto drive passage.
  std::string traj_id;
  double front_s, rear_s;
};

bool IsEgoInJunction(const DrivePassage& drive_passage) {
  constexpr double kEgoInJunctionThre = 5.0;  // m
  for (const auto& station : drive_passage.stations()) {
    if (station.accumulated_s() < 0) continue;
    if (station.accumulated_s() > kEgoInJunctionThre) break;
    if (station.is_in_intersection() ||
        (station.turn_type() != ad_byd::planning::NO_TURN)) {
      return true;
    }
  }
  return false;
}

bool HasEnteredSlBoundary(const PathSlBoundary& path_boundary,
                          const FrenetBox& fbox, bool lc_left,
                          bool ego_in_junction,
                          std::pair<double, double> current_boundary) {
  auto [boundary_left_l, boundary_right_l] = current_boundary;
  if (ego_in_junction) {
    std::tie(boundary_left_l, boundary_right_l) =
        CalcSlBoundaries(path_boundary, fbox);
  }

  const double l_center = path_boundary.QueryReferenceCenterL(fbox.center_s());

  constexpr double kLateralEnterThres = 0.5;  // m.
  return lc_left
             ? fbox.l_min < boundary_left_l - kLateralEnterThres &&
                   fbox.l_max > l_center - boundary_left_l + kLateralEnterThres
             : fbox.l_max > boundary_right_l + kLateralEnterThres &&
                   fbox.l_min <
                       l_center - boundary_right_l - kLateralEnterThres;
}

inline Box2d LerpBox2d(const Box2d& box1, const Box2d& box2, double t) {
  // Assume same size.
  return Box2d(box1.half_length(), box1.half_width(),
               Lerp(box1.center(), box2.center(), t),
               LerpAngle(box1.heading(), box2.heading(), t));
}

absl::StatusOr<Box2d> FindPredictedObjectBox(
    absl::Span<const prediction::PredictionObjectState> obj_states,
    double start_time_offset) {
  for (size_t i = 1; i < obj_states.size(); ++i) {
    const auto& obj_state = obj_states[i];
    if (obj_state.traj_point->t() < start_time_offset) continue;

    return LerpBox2d(
        obj_states[i - 1].box, obj_state.box,
        (start_time_offset - obj_states[i - 1].traj_point->t()) /
            (obj_state.traj_point->t() - obj_states[i - 1].traj_point->t()));
  }
  return absl::NotFoundError(
      "Path start point is not covered by predicted trajectory.");
}

bool HasFullyEnteredTargetLane(const double center_l, const double half_width) {
  return std::abs(center_l) < kFullyEnterTargetLateralThreshold + half_width;
}

bool ObjIsLeavingTargetLane(const FrenetBox& obj_cur_box,
                            const st::FrenetCoordinate& obj_preview_pnt,
                            double diff_l_thrd) {
  const double obj_width = obj_cur_box.width();
  const double obj_cur_l = obj_cur_box.center_l();
  const auto obj_preview_l = obj_preview_pnt.l;
  bool obj_preview_not_in_target =
      !HasFullyEnteredTargetLane(obj_preview_l, 0.5 * obj_width);

  const bool obj_lc_left = (obj_preview_l > obj_cur_l);
  const bool obj_cut_out_clearly =
      obj_preview_not_in_target &&
      (std::fabs(obj_preview_l) > std::fabs(obj_cur_l)) &&
      (std::fabs(obj_preview_l - obj_cur_l) > diff_l_thrd);

  return obj_cut_out_clearly;
}

bool HasFullyEnteredTargetLane(const FrenetBox& obj_box,
                               const double half_width) {
  bool corner_has_entered = false;
  bool center_has_entered = false;

  if (obj_box.l_max * obj_box.l_min > 0) {
    double min_dis_target =
        std::min(std::fabs(obj_box.l_max), std::fabs(obj_box.l_min));
    corner_has_entered =
        std::fabs(min_dis_target) < kFullyEnterTargetLateralThreshold * 0.8;
  } else {
    corner_has_entered = true;
  }

  center_has_entered = std::abs(obj_box.center_l()) <
                       kFullyEnterTargetLateralThreshold + half_width;
  return corner_has_entered || center_has_entered;
}

double getLatOverlap(const st::FrenetBox& ego_cur_frenet_box,
                     const st::FrenetBox& obj_cur_frenet_box, double& rel_dl) {
  const bool if_obs_left =
      (ego_cur_frenet_box.l_max < obj_cur_frenet_box.l_min);
  const double lat_overlap =
      std::min(ego_cur_frenet_box.l_max, obj_cur_frenet_box.l_max) -
      std::max(ego_cur_frenet_box.l_min, obj_cur_frenet_box.l_min);
  rel_dl = (lat_overlap > -kEpsilon)
               ? 0
               : (if_obs_left ? -lat_overlap : lat_overlap);
  return lat_overlap;
};

bool ShouldConsiderInteraction(
    const SpacetimeObjectTrajectory& traj, double ego_heading,
    const FrenetBox& ego_frenet_box, const PathSlBoundary& path_boundary,
    const DrivePassage& drive_passage, bool lc_left, bool ego_in_junction,
    std::pair<double, double> current_boundary,
    LeadingObjectTrajectoryInfo* traj_info,
    const st::planning::PlannerSemanticMapManager& psmm,
    const double path_start_time_offset) {
  const auto& states = traj.states();
  if (std::abs(NormalizeAngle(ego_heading -
                              states.front().traj_point->theta())) > M_PI_2) {
    return false;
  }

  // First check the current position.
  ASSIGN_OR_RETURN(const auto fbox,
                   drive_passage.QueryFrenetBoxAt(states.front().box), false);
  if (fbox.s_min <= ego_frenet_box.s_max) return false;
  
  // ============== cutout ==================
  const auto target_lane_path_ext =
      BackwardExtendLanePath(psmm,
                             drive_passage.extend_lane_path().BeforeArclength(
                                 kLaneChangeCheckForwardLength),
                             kLaneChangeCheckBackwardLength);
  auto target_frenet_frame_or = BuildKdTreeFrenetFrame(
      SampleLanePathPoints(psmm, target_lane_path_ext), true);
  if (target_frenet_frame_or.ok()) {
    auto obj_cur_box_or_not =
        FindPredictedObjectBox(states, path_start_time_offset);
    if (obj_cur_box_or_not.ok()) {
      const auto obj_cur_box = obj_cur_box_or_not.value();
      auto obj_cur_frenet_box_or_not =
          target_frenet_frame_or.value().QueryFrenetBoxWithHeading(obj_cur_box,
                                                                   M_PI);

      if (obj_cur_frenet_box_or_not.ok()) {
        const auto obj_cur_frenet_box = obj_cur_frenet_box_or_not.value();

        bool obs_in_target_lane = HasFullyEnteredTargetLane(
            obj_cur_frenet_box, 0.5 * obj_cur_frenet_box.width());

        const auto obj_near_field_preview_pnt =
            target_frenet_frame_or.value().XYToSL(
                states.size() < kObjMostReliableTrajectoryIndexMax
                    ? states.back().traj_point->pos()
                    : (states.begin() + kObjMostReliableTrajectoryIndexMax - 1)
                          ->traj_point->pos());
        bool obj_is_leaving_near_field_target = ObjIsLeavingTargetLane(
            obj_cur_frenet_box, obj_near_field_preview_pnt, 0.15);

        const auto obj_preview_pnt = target_frenet_frame_or.value().XYToSL(
            states.size() < kObjReliableTrajectoryIndexMax
                ? states.back().traj_point->pos()
                : (states.begin() + kObjReliableTrajectoryIndexMax - 1)
                      ->traj_point->pos());
        const double lat_overlap_thrd_for_obj_leaving = 1.8;
        const bool obj_lc_left =
            obj_preview_pnt.l > obj_cur_frenet_box.center_l();
        double rel_dl = 0.0;
        double lat_overlap =
            getLatOverlap(ego_frenet_box, obj_cur_frenet_box, rel_dl);
        // Log2DDS::LogDataV2(
        //     "xxxxx", absl::StrCat("obs_id:", traj.traj_id(),
        //                           ", obs_in_target_lane: ",
        //                           obs_in_target_lane,
        //                           ", obj_near_field_preview_pnt: ",
        //                           obj_near_field_preview_pnt.l,
        //                           ", offset_time: ", path_start_time_offset,
        //                           ", obj_is_leaving_near_field_target: ",
        //                           obj_is_leaving_near_field_target, "lc_left:
        //                           ", lc_left ? obj_cur_frenet_box.l_min > 0
        //                                   : obj_cur_frenet_box.l_max < 0,
        //                           "obj_lc_left:", obj_lc_left));
        if (obs_in_target_lane && obj_is_leaving_near_field_target &&
            (lc_left ? obj_cur_frenet_box.l_min > 0
                     : obj_cur_frenet_box.l_max < 0) &&
            lat_overlap < -lat_overlap_thrd_for_obj_leaving &&
            obj_lc_left == lc_left) {
          return false;
        }
      }
    }
  }

  // ============== cutin  ==================

  if (HasEnteredSlBoundary(path_boundary, fbox, lc_left, ego_in_junction,
                           current_boundary)) {
    traj_info->traj_id = std::string(traj.traj_id());
    traj_info->front_s = fbox.s_max;
    traj_info->rear_s = fbox.s_min;
    return true;
  }

  // Then check the whole trajectory to find possible interaction by ratio.
  constexpr int kEvalStep = 1;
  constexpr int kEnterSRangeStep = static_cast<int>(1.0 / kTrajectoryTimeStep);
  bool is_valid = false;
  int projected_states = 1, along_path_states = 0;
  double front_s{0}, rear_s{0};
  const size_t kMaxTimeStep = 30;
  size_t step_cnt = std::min(kMaxTimeStep, states.size());
  for (int i = 1; i < step_cnt; i += kEvalStep) {
    if (!is_valid && i > kEnterSRangeStep) {
      // Not entering drive passage in the early seconds means the object is too
      // far behind, so we ignore it.
      return false;
    }

    ASSIGN_OR_CONTINUE(const auto fbox,
                       drive_passage.QueryFrenetBoxAt(states[i].box));
    ++projected_states;
    if (!is_valid) {
      // Record the first projectable state onto drive passage.
      is_valid = true;
      front_s = fbox.s_max;
      rear_s = fbox.s_min;
    }
    if (HasEnteredSlBoundary(path_boundary, fbox, lc_left, ego_in_junction,
                             current_boundary)) {
      ++along_path_states;
    }
  }

  constexpr double kMoveAlongPathPercentageThreshold = 0.5;
  const double on_path_ratio = static_cast<double>(along_path_states) /
                               static_cast<double>(projected_states);
  if (on_path_ratio > kMoveAlongPathPercentageThreshold) {
    traj_info->traj_id = std::string(traj.traj_id());
    traj_info->front_s = front_s;
    traj_info->rear_s = rear_s;
    return true;
  }
  return false;
}

}  // namespace

std::vector<LeadingGroup> FindMultipleLeadingGroups(
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    bool lc_left, const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects, double ego_heading,
    const FrenetBox& ego_frenet_box,
    const VehicleGeometryParamsProto& vehicle_geom,
    const st::planning::PlannerSemanticMapManager& psmm,
    const double path_start_time_offset) {
  std::vector<LeadingGroup> leading_groups;

  // If ego not in junction, use ego current boundary
  const bool ego_in_junction = IsEgoInJunction(drive_passage);
  const auto current_boundary = CalcSlBoundaries(path_boundary, ego_frenet_box);

  // If current lateral offset greater than a value, consider we need lane
  // change or lane borrow.
  const auto& considered_trajectories = st_traj_mgr.trajectories();
  std::vector<LeadingObjectTrajectoryInfo> filtered_trajs;
  for (const auto& traj : considered_trajectories) {
    VLOG(3) << "Consider traj: " << traj.traj_id();
    const auto object_type = traj.planner_object().type();
    LeadingObjectTrajectoryInfo traj_info;
    if (object_type == ObjectType::OT_CYCLIST ||
        object_type == ObjectType::OT_TRICYCLIST)
      continue;
    if (IsLeadingObjectType(object_type) &&
        ShouldConsiderInteraction(traj, ego_heading, ego_frenet_box,
                                  path_boundary, drive_passage, lc_left,
                                  ego_in_junction, current_boundary, &traj_info,
                                  psmm, path_start_time_offset)) {
      filtered_trajs.push_back(traj_info);
    }
  }
  std::stable_sort(filtered_trajs.begin(), filtered_trajs.end(),
                   [](const LeadingObjectTrajectoryInfo& traj1,
                      const LeadingObjectTrajectoryInfo& traj2) {
                     return traj1.rear_s < traj2.rear_s;
                   });

  // Group potential leading objects to groups. Separate groups according to
  // sufficient longitudinal gap.
  LeadingGroup traj_group;
  const double min_gap = vehicle_geom.length() * 2.0;
  double previous_s = std::numeric_limits<double>::lowest();
  for (const auto& traj : filtered_trajs) {
    if (traj.front_s >= path_boundary.end_s()) break;

    if (!traj_group.empty()) {
      const double current_gap = traj.rear_s - previous_s;
      if (current_gap >= min_gap) {
        traj_group.swap(leading_groups.emplace_back());
      }
    }

    if (!stalled_objects.contains(
            SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
                traj.traj_id))) {
      traj_group.emplace(
          traj.traj_id,
          CreateLeadingObject(
              *st_traj_mgr.FindTrajectoryById(traj.traj_id), drive_passage,
              ConstraintProto::LeadingObjectProto::LANE_CHANGE_TARGET,
              traj_group.empty()));
    }
    previous_s = traj.front_s;

    if (leading_groups.size() ==
        FLAGS_planner_initializer_max_multi_traj_num - 1) {
      break;
    }
  }
  if (!traj_group.empty()) {
    leading_groups.push_back(std::move(traj_group));
  }
  return leading_groups;
}

}  // namespace st::planning
