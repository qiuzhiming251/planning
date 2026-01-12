

#include "spacetime_planner_object_trajectories_filter.h"

#include <algorithm>
#include <ostream>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/math/linear_interpolation.h"
#include "plan_common/math/util.h"
#include "object_manager/drive_passage_filter.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_object_state.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"

namespace st {
namespace planning {
namespace {

bool IsMaybeCutInVehicleTrajectory(
    const DrivePassage& drive_passage,
    const LaneChangeStateProto& lane_change_state, const Box2d& av_box,
    const FrenetBox& av_sl_box, double av_speed,
    const SpacetimeObjectTrajectory& traj) {
  if (lane_change_state.stage() != LCS_NONE) {
    return false;
  }
  if (traj.object_type() != ObjectType::OT_VEHICLE &&
      traj.object_type() != ObjectType::OT_LARGE_VEHICLE) {
    return false;
  }

  constexpr double kObjectDistThreshold = 0.8;
  const auto& object_box = traj.bounding_box();
  const double dist_to_av_box = object_box.DistanceTo(av_box);
  if (dist_to_av_box < kObjectDistThreshold) {
    VLOG(2) << traj.traj_id() << " dist to close " << dist_to_av_box << "m.";
    return false;
  }
  const auto& object_frenet_box = drive_passage.QueryFrenetBoxAt(object_box);
  if (!object_frenet_box.ok()) {
    return false;
  }
  const auto& states = traj.states();
  const auto& object_start_pt = *states.front().traj_point;
  const auto current_object_sl_pt =
      drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
          object_start_pt.pos());
  if (!current_object_sl_pt.ok()) {
    VLOG(2) << traj.traj_id()
            << " current pose not in drive_passage, skip evaluation.";
    return false;
  }
  const auto current_lane_heading =
      drive_passage.QueryTangentAngleAtS(current_object_sl_pt->s);
  if (!current_lane_heading.ok()) {
    VLOG(2) << traj.traj_id() << " current lane heading query failed.";
    return false;
  }

  constexpr double kLargeLatSpeed = 0.3;  // m/s^2
  const double object_lat_speed =
      object_start_pt.v() *
      std::sin(object_start_pt.theta() - *current_lane_heading);
  const bool lat_speed_large = current_object_sl_pt->l > 0.0
                                   ? object_lat_speed < -kLargeLatSpeed
                                   : object_lat_speed > kLargeLatSpeed;

  constexpr double kObjectSExtent = 2.0;  // m
  const bool cur_lon_overlapped =
      (av_sl_box.s_max > (object_frenet_box->s_min - kObjectSExtent) &&
       av_sl_box.s_max < (object_frenet_box->s_max + kObjectSExtent)) ||
      (av_sl_box.s_min > (object_frenet_box->s_min - kObjectSExtent) &&
       av_sl_box.s_min < (object_frenet_box->s_max + kObjectSExtent));

  if (lat_speed_large && cur_lon_overlapped &&
      dist_to_av_box < kObjectSExtent) {
    VLOG(2) << traj.traj_id() << " may be immoral.";
    return false;
  }

  const auto final_object_sl_pt =
      drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
          states.back().traj_point->pos());

  if (!final_object_sl_pt.ok()) {
    VLOG(2) << traj.traj_id()
            << " final pose not in drive_passage, skip evaluation.";
    return false;
  }
  // Just pay attention to object outside lane.
  // TODO: Use prev info to check cut in state for object inside lane.
  const auto current_lane_boundary_offset =
      drive_passage.QueryNearestBoundaryLateralOffset(current_object_sl_pt->s);
  if (!current_lane_boundary_offset.ok()) {
    VLOG(2) << traj.traj_id() << " current lateral boundary query failed.";
    return false;
  }
  if (current_object_sl_pt->l < current_lane_boundary_offset->second &&
      current_object_sl_pt->l > current_lane_boundary_offset->first) {
    VLOG(2) << traj.traj_id() << " object in lane.";
    return false;
  }
  const auto final_lane_boundary_offset =
      drive_passage.QueryNearestBoundaryLateralOffset(final_object_sl_pt->s);
  if (!final_lane_boundary_offset.ok()) {
    VLOG(2) << traj.traj_id() << " fianl lateral boundary query failed.";
    return false;
  }
  // If traj is slope to center.
  constexpr double kCenterOffsetThreshold = 0.8;  // m
  bool is_traj_slope_to_center = false;

  if (current_object_sl_pt->l < current_lane_boundary_offset->first) {
    is_traj_slope_to_center =
        final_object_sl_pt->l > current_lane_boundary_offset->first &&
        final_object_sl_pt->l > current_object_sl_pt->l;
  } else if (current_object_sl_pt->l > current_lane_boundary_offset->second) {
    is_traj_slope_to_center =
        final_object_sl_pt->l < current_lane_boundary_offset->second &&
        final_object_sl_pt->l < current_object_sl_pt->l;
  }
  if (is_traj_slope_to_center) {
    // Heading matching: If toward center ref line diff is large, object may
    // be cut-in.
    constexpr double kCutInThetaThreshold = 0.05;  // rad
    const double toward_center_theta_diff =
        current_object_sl_pt->l > 0.0
            ? NormalizeAngle(*current_lane_heading - traj.pose().theta())
            : NormalizeAngle(traj.pose().theta() - *current_lane_heading);
    if (toward_center_theta_diff > kCutInThetaThreshold) {
      return true;
    }

    const double toward_center_offset =
        current_object_sl_pt->l > 0.0
            ? current_object_sl_pt->l - final_object_sl_pt->l
            : final_object_sl_pt->l - current_object_sl_pt->l;
    VLOG(3) << traj.traj_id()
            << " toward_center_offset: " << toward_center_offset << " "
            << toward_center_theta_diff;
    constexpr double kMaybeCutInThetaThreshold = 0.01;  // rad
    // If vehicle is predicted to get closer to center line and heading toward
    // center line, object might be cutting in.
    if (toward_center_offset > kCenterOffsetThreshold &&
        toward_center_theta_diff > kMaybeCutInThetaThreshold) {
      VLOG(2) << traj.traj_id() << " prediction toward center offset "
              << toward_center_offset << "m > " << kCenterOffsetThreshold
              << "m, toward center theta diff is " << kMaybeCutInThetaThreshold
              << "rad, maybe cutin traj, refuse to nudge.";
      return true;
    }
  }

  return false;
}

bool IsVru(const SpacetimeObjectTrajectory& traj) {
  return traj.object_type() == ObjectType::OT_MOTORCYCLIST ||
         traj.object_type() == ObjectType::OT_CYCLIST ||
         traj.object_type() == ObjectType::OT_TRICYCLIST ||
         traj.object_type() == ObjectType::OT_PEDESTRIAN;
}

bool IsCrossingTrajectory(const DrivePassage& drive_passage,
                          const PlannerSemanticMapManager& psmm,
                          const SpacetimeObjectTrajectory& traj) {
  if (traj.is_stationary() ||
      prediction::IsStaticObjectType(traj.object_type())) {
    return false;
  }
  const bool is_vru = IsVru(traj);
  const auto cur_lane_id = drive_passage.lane_path().front().lane_id();
  const auto cur_lane_info = psmm.FindCurveLaneByIdOrNull(cur_lane_id);
  bool is_left_turn = false;
  bool is_intersection_straight = false;
  if (cur_lane_info &&
      cur_lane_info->turn_type() == ad_byd::planning::LEFT_TURN) {
    is_left_turn = true;
  }
  bool is_in_intersection =
      cur_lane_info && !(cur_lane_info->junction_id() == 0);
  if (is_in_intersection &&
      cur_lane_info->turn_type() == ad_byd::planning::NO_TURN) {
    is_intersection_straight = true;
  }
  const auto& traj_start_point = *traj.states().front().traj_point;
  const auto frenet_start_point =
      drive_passage.QueryUnboundedFrenetCoordinateAt(traj_start_point.pos());
  if (!frenet_start_point.ok()) {
    return false;
  }
  // ignore obstacles whose trajs keep the same side
  if (!is_vru && is_intersection_straight && traj_start_point.v() < 4.0) {
    const auto frenet_end_point =
        drive_passage.QueryUnboundedFrenetCoordinateAt(
            (*traj.states().back().traj_point).pos());
    if (frenet_end_point.ok() &&
        (frenet_start_point->l * frenet_end_point->l > 0.0 ||
         std::fabs(frenet_end_point->l) < 1.5)) {
      return false;
    }
  }
  const auto lane_theta_at_pose =
      drive_passage.QueryTangentAngleAtS(frenet_start_point->s);
  if (!lane_theta_at_pose.ok()) {
    return false;
  }
  // Compute thetas perpendicular to current lane.
  const double lane_normal_theta_at_pose =
      NormalizeAngle(*lane_theta_at_pose + M_PI_2);
  const double lane_negative_normal_theta_at_pose =
      NormalizeAngle(*lane_theta_at_pose - M_PI_2);

  // tag reverse object
  const bool obs_reverse_flag =
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj_start_point.theta())) < M_PI / 6;

  // modify is_intersection_straight and is_in_intersection logic
  if (obs_reverse_flag) {
    is_intersection_straight = false;
    is_in_intersection = false;
  }

  if (is_vru && (!(obs_reverse_flag ||
                   std::abs(NormalizeAngle(*lane_theta_at_pose -
                                           traj_start_point.theta())) <
                       M_PI / 6 + 1e-3))) {
    return true;
  }
  // If all angles between trajectory points theta and lane
  // theta is less than 1.0rad(60°), most(60%) angles is less
  // than 0.78rad(45°) and a little((40%)) angles is less than
  // 0.52(30°), we think it is a crossing trajectory. Special
  // deal with intersection left turn and straight drive.
  const std::vector<double> intersection_thresholds = {1.57, 1.57, 1.57};
  const std::vector<double> theta_diff_thresholds = {1.0, 0.78, 0.52};
  const std::vector<double> ratio_limits = {0.8, 0.6, 0.4};
  const int level_size = theta_diff_thresholds.size();
  std::vector<int> level_counts(level_size, 0);

  const auto& states = traj.states();
  const int states_size = is_vru ? states.size() / 2 : states.size();
  for (int i = 0; i < states_size; ++i) {
    const auto& state = states[i];
    const double theta = state.traj_point->theta();
    const double theta_diff_to_lane_normal =
        NormalizeAngle(theta - lane_normal_theta_at_pose);
    const double theta_diff_to_negative_lane_normal =
        NormalizeAngle(lane_negative_normal_theta_at_pose - theta);
    for (int j = 0; j < level_size; ++j) {
      if (!is_vru && is_left_turn && 0.0 < theta_diff_to_lane_normal &&
          theta_diff_to_lane_normal < intersection_thresholds[j]) {
        level_counts[j]++;
      } else if (!is_vru && is_intersection_straight &&
                 0.0 < theta_diff_to_negative_lane_normal &&
                 theta_diff_to_negative_lane_normal <
                     intersection_thresholds[j]) {
        level_counts[j]++;
      } else if (std::abs(theta_diff_to_lane_normal) <
                     theta_diff_thresholds[j] ||
                 std::abs(theta_diff_to_negative_lane_normal) <
                     theta_diff_thresholds[j]) {
        level_counts[j]++;
      }
    }
  }
  for (int j = 0; j < level_size; ++j) {
    const double ratio =
        static_cast<double>(level_counts[j]) / static_cast<double>(states_size);
    if (ratio < ratio_limits[j]) {
      return false;
    }
  }
  return true;
}

bool IsTrajectoryBeyondStopLine(const DrivePassage& drive_passage,
                                double first_stop_line_s,
                                const SpacetimeObjectTrajectory& traj) {
  if (std::isinf(first_stop_line_s)) {
    return false;
  }
  if (traj.is_stationary()) {
    const auto frenet_box = drive_passage.QueryFrenetBoxAt(traj.bounding_box());
    if (!frenet_box.ok()) {
      return false;
    }
    if (frenet_box->s_min < first_stop_line_s) {
      return false;
    }
  } else {
    const auto& states = traj.states();
    for (const auto& state : states) {
      const auto frenet_box = drive_passage.QueryFrenetBoxAt(state.box);
      if (!frenet_box.ok()) {
        continue;
      }
      if (frenet_box->s_min < first_stop_line_s) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

bool IsFardistObjectTrajectory(const DrivePassage& drive_passage,
                               const bool is_lane_change, const double av_speed,
                               const FrenetBox& av_sl_box,
                               const SpacetimeObjectTrajectory& traj) {
  const double kHeadway_lk = 5.5;
  const double kUpperdist_lk = 80.0;
  const double kLowerdist_lk = 50.0;
  const double kHeadway_lc = 5.5;
  const double kUpperdist_lc = 80.0;
  const double kLowerdist_lc = 50.0;
  const double kTTCThreshold = 10.0;
  const double kTTCValidDist = 40.0;
  const double kHeadway_vru = 3.0;
  const double kUpperdist_vru = 50.0;
  const double kLowerdist_vru = 30.0;

  const bool is_vru = IsVru(traj);

  const double dist_threshold =
      is_vru ? std::fmin(std::fmax(av_speed * kHeadway_vru, kLowerdist_vru),
                         kUpperdist_vru)
      : is_lane_change
          ? std::fmin(std::fmax(av_speed * kHeadway_lc, kLowerdist_lc),
                      kUpperdist_lc)
          : std::fmin(std::fmax(av_speed * kHeadway_lk, kLowerdist_lk),
                      kUpperdist_lk);
  const auto& object_box = traj.bounding_box();
  const auto& obj_frenet_box = drive_passage.QueryFrenetBoxAt(object_box);
  if (!obj_frenet_box.ok()) {
    return false;
  }
  const double dist2ego = obj_frenet_box->s_min - av_sl_box.s_max;
  if (dist2ego > dist_threshold) {
    return true;
  }

  if (dist2ego > kTTCValidDist) {
    double obj_speed = traj.planner_object().pose().v();
    const auto& tangent_unit_or =
        drive_passage.QueryTangentAtS(obj_frenet_box->center_s());
    if (tangent_unit_or.ok()) {
      obj_speed = tangent_unit_or.value().Dot(traj.planner_object().velocity());
    }

    const double ttc = std::abs(av_speed - obj_speed) > 1e-5
                           ? (dist2ego / (av_speed - obj_speed))
                           : DBL_MAX;
    if (ttc < 0 || ttc > kTTCThreshold) {
      return true;
    }
  }

  return false;
}

bool IsCutInObjectTrajectory(const DrivePassage& drive_passage,
                             const bool is_lane_change, const double av_speed,
                             const FrenetBox& av_sl_box,
                             const SpacetimeObjectTrajectory& traj) {
  // std::cout << "IsCutInObjectTrajectory - obs id: " << traj.object_id() <<
  // std::endl; lane change state condition
  if (is_lane_change) return false;

  // obstacle type condition
  if (traj.object_type() != ObjectType::OT_VEHICLE &&
      traj.object_type() != ObjectType::OT_LARGE_VEHICLE) {
    return false;
  }

  // speed condition
  const double obj_speed = traj.pose().v();
  if (av_speed - obj_speed < 1.5 && av_speed > 2.8) {
    return false;
  }

  // theta condition
  const auto frenet_start_point =
      drive_passage.QueryUnboundedFrenetCoordinateAt(
          traj.states().front().traj_point->pos());
  if (!frenet_start_point.ok()) {
    return false;
  }
  const auto lane_theta_at_pose =
      drive_passage.QueryTangentAngleAtS(frenet_start_point->s);
  if (!lane_theta_at_pose.ok()) {
    return false;
  }
  const bool obs_reverse_flag =
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj.states().front().traj_point->theta())) <
      M_PI / 3;
  if (obs_reverse_flag) {
    return false;
  }

  // longitudinal condition
  const double kObjectDistThreshold = 0.8;
  const double kObjectDistThreshold_lowspeed = 0.4;
  double ObjectDistThreshold =
      av_speed < 2.8 ? kObjectDistThreshold_lowspeed : kObjectDistThreshold;
  const double kBrakeAcc = 3.0;
  const double min_brake_dist =
      (av_speed - obj_speed) * (av_speed - obj_speed) / (kBrakeAcc * 2.0);
  const auto& object_box = traj.bounding_box();
  const auto& obj_frenet_box = drive_passage.QueryFrenetBoxAt(object_box);
  if (!obj_frenet_box.ok()) {
    return false;
  }
  if (obj_frenet_box->s_min - av_sl_box.s_max <
      std::fmax(ObjectDistThreshold, min_brake_dist)) {
    return false;
  }

  // latitudinal condition
  const auto& object_frenet_box = drive_passage.QueryFrenetBoxAt(object_box);
  if (!object_frenet_box.ok()) {
    return false;
  }
  // std::cout << "IsCutInObjectTrajectory - 1 " << traj.object_id()
  //           << " | object_frenet_box->l_min: " << object_frenet_box->l_min
  //           << " | object_frenet_box->l_max: " << object_frenet_box->l_max <<
  //           std::endl;
  const double center_range = 1.5;
  if (object_frenet_box->l_min >
          0.6 * ad_byd::planning::Constants::DEFAULT_LANE_WIDTH ||
      object_frenet_box->l_max <
          -0.6 * ad_byd::planning::Constants::DEFAULT_LANE_WIDTH ||
      std::fabs(object_frenet_box->center_l()) < center_range) {
    return false;
  }

  // check cutin
  const double cutin_l_threshold = 1.0;
  const double delta_l_threshold = 0.5;
  // double prev_l = object_frenet_box->l_min >
  // ad_byd::planning::Constants::ZERO
  //                     ? object_frenet_box->l_min
  //                     : object_frenet_box->l_max;
  double start_t = 0.0;
  if (traj.states().size() > 0) {
    start_t = traj.states()[0].traj_point->t();
  }
  for (const auto& obj_state : traj.states()) {
    const auto pred_box = drive_passage.QueryFrenetBoxAt(obj_state.box);
    if (!pred_box.ok()) {
      break;
    }
    const auto& nearest_station =
        drive_passage.FindNearestStationAtS(pred_box->center_s());
    bool is_virtual = nearest_station.is_virtual();
    double rel_time = obj_state.traj_point->t() - start_t;
    if ((is_virtual && rel_time > 4.0) || rel_time > 5.0) {
      break;
    }
    if (object_frenet_box->center_l() > ad_byd::planning::Constants::ZERO) {
      // if (pred_box->l_min > prev_l) return false;
      if (pred_box->l_min < cutin_l_threshold &&
          object_frenet_box->l_min - pred_box->l_min > delta_l_threshold) {
        std::string cutin_filter_msg = " cutin filter: ";
        cutin_filter_msg += std::string(traj.object_id());
        Log2DDS::LogDataV2("cutin_filter", cutin_filter_msg);
        return true;
      }
      //  prev_l = pred_box->l_min;
    } else if (object_frenet_box->center_l() <
               -ad_byd::planning::Constants::ZERO) {
      // std::cout << "pred_box->l_max: " << pred_box->l_max << std::endl;
      // if (pred_box->l_max < prev_l) return false;
      if (pred_box->l_max > -cutin_l_threshold &&
          pred_box->l_max - object_frenet_box->l_max > delta_l_threshold) {
        // std::cout << "IsCutInObjectTrajectory - cutin check obs id: " <<
        // traj.object_id() << std::endl;
        std::string cutin_filter_msg = " cutin filter: ";
        cutin_filter_msg += std::string(traj.object_id());
        Log2DDS::LogDataV2("cutin_filter", cutin_filter_msg);
        return true;
      }
      //  prev_l = pred_box->l_max;
    }
  }

  return false;
}

CutInSpacetimePlannerObjectTrajectoriesFilter::
    CutInSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        const LaneChangeStateProto* lane_change_state, const Box2d& av_box,
        double av_speed)
    : drive_passage_(drive_passage),
      lane_change_state_(lane_change_state),
      av_box_(av_box),
      av_speed_(av_speed) {
  const auto av_sl_box_or = drive_passage_->QueryFrenetBoxAt(av_box_);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
}

bool CutInSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (!av_sl_box_.has_value()) {
    VLOG(2) << "AV box can't be mapped on drive passage, skip.";
    return false;
  }
  const bool is_lane_change =
      ((*lane_change_state_).stage() == LaneChangeStage::LCS_EXECUTING ||
       (*lane_change_state_).stage() == LaneChangeStage::LCS_RETURN ||
       (*lane_change_state_).stage() == LaneChangeStage::LCS_PAUSE);
  if (IsFardistObjectTrajectory(*drive_passage_, is_lane_change, av_speed_,
                                *av_sl_box_, traj)) {
    std::string fardist_filter_msg = " fardist filter: ";
    fardist_filter_msg += std::string(traj.object_id());
    Log2DDS::LogDataV2("fardist_filter", fardist_filter_msg);
    return true;
  }
  return IsCutInObjectTrajectory(*drive_passage_, is_lane_change, av_speed_,
                                 *av_sl_box_, traj);
}

bool TargetBoundarySpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  const bool use_out_boundary = false;
  const bool use_decay_buffer = true;

  const double obj_buffer = GetDistanceBuffer(traj.planner_object());
  if (traj.is_stationary()) {
    if (IsContourWithBufferOverlapsBoundary(
            traj.contour(), obj_buffer, *sl_boundary_, *drive_passage_,
            ego_box_, *lc_state_, use_out_boundary, use_decay_buffer,
            &ego_v_)) {
      return false;
    }
    return true;
  } else {
    constexpr double kTrajectoryConsiderTime = 8.0;
    for (const auto& pred_state : traj.states()) {
      if (pred_state.traj_point->t() > kTrajectoryConsiderTime) break;

      if (IsContourWithBufferOverlapsBoundary(
              pred_state.contour, obj_buffer, *sl_boundary_, *drive_passage_,
              ego_box_, *lc_state_, use_out_boundary, use_decay_buffer,
              &ego_v_)) {
        return false;
      }
    }
    return true;
  }
}

CutInVehicleSpacetimePlannerObjectTrajectoriesFilter::
    CutInVehicleSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        const LaneChangeStateProto* lane_change_state, const Box2d& av_box,
        double av_speed)
    : drive_passage_(drive_passage),
      lane_change_state_(lane_change_state),
      av_box_(av_box),
      av_speed_(av_speed) {
  const auto av_sl_box_or = drive_passage_->QueryFrenetBoxAt(av_box_);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
}

bool CutInVehicleSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (!av_sl_box_.has_value()) {
    VLOG(2) << "AV box can't be mapped on drive passage, skip.";
    return false;
  }
  return IsMaybeCutInVehicleTrajectory(*drive_passage_, *lane_change_state_,
                                       av_box_, *av_sl_box_, av_speed_, traj);
}

CrossingSpacetimePlannerObjectTrajectoriesFilter::
    CrossingSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        const PlannerSemanticMapManager* psmm)
    : drive_passage_(drive_passage), psmm_(psmm) {}

bool CrossingSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (IsCrossingTrajectory(*drive_passage_, *psmm_, traj)) {
    Log2DDS::LogDataV2("Cross_filter", absl::StrCat(traj.traj_id()));
    return true;
  }
  return false;
}

BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter::
    BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        absl::Span<const ConstraintProto::StopLineProto> stop_lines)
    : drive_passage_(drive_passage) {
  for (const auto& stop_line : stop_lines) {
    first_stop_line_s_ =
        std::min(first_stop_line_s_, stop_line.s() - stop_line.standoff());
  }
}

bool BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  return IsTrajectoryBeyondStopLine(*drive_passage_, first_stop_line_s_, traj);
}

ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter::
    ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        const PlannerSemanticMapManager* psmm,
        const VehicleGeometryParamsProto* vehicle_geometry_params,
        const PathSlBoundary* sl_boundary,
        const NudgeObjectInfo* nudge_object_info, Box2d av_box, double av_speed)
    : drive_passage_(drive_passage),
      psmm_(psmm),
      vehicle_geometry_params_(vehicle_geometry_params),
      sl_boundary_(sl_boundary),
      nudge_object_info_(nudge_object_info),
      av_box_(std::move(av_box)),
      av_speed_(av_speed) {}

bool ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  const bool is_vru = IsVru(traj);
  if (traj.object_type() != ObjectType::OT_VEHICLE &&
      traj.object_type() != ObjectType::OT_LARGE_VEHICLE && !is_vru) {
    return false;
  }

  const auto ego_frenet_box_outut = drive_passage_->QueryFrenetBoxAt(av_box_);
  if (!ego_frenet_box_outut.ok()) return false;
  const auto& ego_frenet_box = ego_frenet_box_outut.value();

  const auto object_frenet_box_output =
      drive_passage_->QueryFrenetBoxAtContour(traj.contour());
  if (!object_frenet_box_output.ok()) return false;
  const auto& object_frenet_box = object_frenet_box_output.value();
  const double object_s = object_frenet_box.center_s();

  const auto cur_lane_id = drive_passage_->lane_path().front().lane_id();
  const auto cur_lane_info = psmm_->FindCurveLaneByIdOrNull(cur_lane_id);
  const bool is_in_intersection =
      cur_lane_info && !(cur_lane_info->junction_id() == 0);
  bool is_left_turn = false;
  if (cur_lane_info &&
      cur_lane_info->turn_type() == ad_byd::planning::LEFT_TURN) {
    is_left_turn = true;
  }

  const auto lane_theta_at_pose =
      drive_passage_->QueryTangentAngleAtS(object_s);
  if (!lane_theta_at_pose.ok()) {
    return false;
  }

  const auto& ego_nearest_station = drive_passage_->FindNearestStationAtS(
      ego_frenet_box.center_s() + std::clamp(5 * av_speed_, 10.0, 100.0));

  // current left_turn and ignore reverse car
  if (ego_nearest_station.is_virtual() && is_left_turn && is_in_intersection &&
      std::abs(NormalizeAngle(*lane_theta_at_pose + M_PI -
                              traj.pose().theta())) < M_PI / 2 &&
      !is_vru) {
    Log2DDS::LogDataV2(
        "reverse_filter",
        absl::StrCat("ego_turn_left,ignore:", traj.object_id(),
                     ",far_distance:", std::clamp(5 * av_speed_, 10.0, 100.0)));
    return true;
  }
  const auto& nearest_station = drive_passage_->FindNearestStationAtS(object_s);
  if (nearest_station.is_virtual()) {
    return false;
  }

  if (object_frenet_box.s_min < ego_frenet_box.s_max) return false;

  constexpr double kMaxHalfLaneWidth = 4.5;  // m
  constexpr double kMinHalfLaneWidth = 1.2;  // m
  constexpr double kSampleStepAlongS = 1.0;  // m
  double initial_left_boundary = std::numeric_limits<double>::infinity();
  double initial_right_boundary = -std::numeric_limits<double>::infinity();

  constexpr bool use_out_boundary = true;
  for (double sample_s = object_frenet_box.s_min;
       sample_s <= object_frenet_box.s_max; sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] =
        use_out_boundary ? sl_boundary_->QueryBoundaryL(sample_s)
                         : sl_boundary_->QueryTargetBoundaryL(sample_s);
    initial_left_boundary = std::min(initial_left_boundary, left_l);
    initial_right_boundary = std::max(initial_right_boundary, right_l);
  }

  const double left_boundary =
      std::clamp(initial_left_boundary, kMinHalfLaneWidth, kMaxHalfLaneWidth);
  const double right_boundary = std::clamp(
      initial_right_boundary, -kMaxHalfLaneWidth, -kMinHalfLaneWidth);

  constexpr double kThetaThreshold = 0.25;  // rad
  const double kLatSafeDistanceThreshold =
      nudge_object_info_ && (nudge_object_info_->nudge_state ==
                                 NudgeObjectInfo::NudgeState::NUDGE ||
                             nudge_object_info_->nudge_state ==
                                 NudgeObjectInfo::NudgeState::BORROW)
          ? 0.2
          : 0.4;  // m

  const double ego_obs_delta_lon_dis =
      object_frenet_box.s_min - ego_frenet_box.s_max;
  auto const lon_dis_buffer = ad_byd::planning::math::lerp(
      0.0, 10.0, 0.6, 60.0, ego_obs_delta_lon_dis, true);
  const bool object_overlap_with_safe_boundary =
      object_frenet_box.center_l() > 0.0
          ? object_frenet_box.l_min >
                right_boundary + vehicle_geometry_params_->width() +
                    kLatSafeDistanceThreshold - lon_dis_buffer
          : object_frenet_box.l_max <
                left_boundary - vehicle_geometry_params_->width() -
                    kLatSafeDistanceThreshold + lon_dis_buffer;

  const double object_relative_tangent_opposite = std::fabs(NormalizeAngle(
      nearest_station.tangent().FastAngle() + M_PI - traj.pose().theta()));

  // reverse fast car in right side ignore
  if (object_frenet_box.center_l() > right_boundary &&
      object_frenet_box.center_l() < 0.0 && traj.pose().v() > Kph2Mps(15.0) &&
      object_relative_tangent_opposite < kThetaThreshold && !is_vru) {
    Log2DDS::LogDataV2("reverse_filter",
                       absl::StrCat("fast right:", traj.object_id()));
    return true;
  }

  if (!object_overlap_with_safe_boundary &&
      object_relative_tangent_opposite < kThetaThreshold &&
      !traj.is_stationary()) {
    Log2DDS::LogDataV2("reverse_filter",
                       absl::StrCat("no space:", traj.object_id()));
    return true;
  }

  return false;
}

}  // namespace planning
}  // namespace st
