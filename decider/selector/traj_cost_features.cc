#include "decider/selector/traj_cost_features.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/util.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/drive_passage.h"
#include "plan_common/log_data.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/path_sl_boundary.h"
#include "router/route_sections_util.h"
#include "decider/selector/cost_feature_util.h"
#include "decider/selector/speed_cost_estimator.h"
#include "decider/selector/route/merge_lane_cost.h"
#include "decider/selector/route/bus_lane_cost.h"
#include "plan_common/util/format_numeric_string.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/status_macros.h"
#include "decider/selector/lane_change_signal_frame.h"

#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"

#define btoa(x) ((x) ? "true" : "false")
#define btof(x) ((x) ? 1.0 : 0.0)

namespace st::planning {
namespace {
constexpr double kMinStandardProgressDiff = 50.0;  // m.

constexpr double kRadicalPorgressFactor = 2.0;
// constexpr double kConservativePorgressFactor = 0.5;
// normal lane change when Conservative mode
constexpr double kConservativePorgressFactor = 1.0;
constexpr double kLaneSpeedDiffToLimitRatio = 0.4;  // s.
constexpr double kMinFollowLeaderTime = 2.0;        // s.
constexpr double kMinFollowDistance = 10.0;         // m.

constexpr double kFollowDistanceFactor = 0.8;

constexpr double kMaxOppositeLcTimeBound = 50.0;             // s.
constexpr double kMinOppositeLcTimeBound = 30.0;             // s.
constexpr double kMinPassedSplitOppositeLcTimeBound = 30.0;  // s.
constexpr double kPassedSplitForwardDis = 10.0;              // s.
constexpr double kPassedSplitBackwardDis = -50.0;            // s.
constexpr double kRedLightSpeedBase = 6.0;                   // m/s.
constexpr double kRedLightWaitingTimeBase = 20.0;            // s.
constexpr double kRedLightWaitingLeaderFactor = 0.2;
constexpr double kMaxRedLightDistance = 90.0;  // m.
constexpr double kStandardAverageTrajDiff = 1.6 * kDefaultHalfLaneWidth;
constexpr double kCrossBoundaryFactor = 0.6;
constexpr double kLcEffectBase = 2.0;                     // m/s^2
constexpr double kReachDestinationCutOffDist = 70.0;      // m.
constexpr double kRouteLengthCutOffDist = 2000.0;         // m.
constexpr double kBeginRouteLengthLcForHighWay = 1500.0;  // m.
constexpr double kForceRouteLengthLcForHighWay = 100.0;   // m.
constexpr double kBeginRouteLcCostFactorForHighWay = 0.35;
constexpr double kForceRouteLcCostFactorForHighWay = 1.3;
constexpr double kBeginRouteTtcForHighWay = 40.0;        // s.
constexpr double kForceRouteTtcForHighWay = 20.0;        // s.
constexpr double kConsiderTtcSpeedLowerBound = 5.0;      // m/s.
constexpr double kConsiderPrevTrajTime = 4.0;            // s.
constexpr double kBackCheckCurveDistance = 20.0;         // m.
constexpr double kCalculateCurvatureStep = 4.0;          // m.
constexpr double kLengthAlongRouteBaseForLeft = 800.0;   // m.
constexpr double kLengthAlongRouteBaseForRight = 700.0;  // m.
constexpr double kMergeLaneLengthBaseHighWay = 4500.0;   // m.
constexpr double kMergeLaneLengthBase = 600.0;           // m.
constexpr double kStalledObjectLengthBase = 60.0;        // m.
constexpr double kStalledObjectLengthBaseHighWay = 100.0;  // m.
constexpr double kShortLengthAlongRouteBase = 100.0;     // m.
constexpr double kLengthAlongRouteObviousThreshold = 0.5;
constexpr double kLengthAlongMergedAreaThreshold = 0.5;
constexpr double kLengthCutOffObviousThreshold = 0.4;
constexpr double kLengthBeforeMergeLaneObviousThreshold = 0.85;
constexpr double kBehindStalledObjObviousThreshold = 0.5;
constexpr double kLatAwayObviousThreshold = 0.5;
constexpr double kInvadeRiskObviousThreshold = 0.3;
constexpr double kSlowWorkingObviousThreshold = 0.3;
constexpr double kFollowSlowObviousThreshold = 0.3;
constexpr double kBeginLcLengthAlongRouteThreshold = 0.3;
constexpr double kBeginLcForPreviewThreshold = 0.9;
constexpr double kBeginLcForDiscourageRightMostThreshold = 0.6;
constexpr double kBeginLcForExitLaneThreshold = 0.6;
constexpr double kBeginLcForMergeLaneThreshold = 0.5;
constexpr double kBeginLcForMergeLaneThresholdHighway = 0.8;
constexpr double kDiscourageRightMostSpeedLowerBound = Kph2Mps(20.0);  // m/s.
constexpr double kDiscourageRightMostSpeedUpperBound = Kph2Mps(80.0);  // m/s.
constexpr double kCurveSpeedConstraint = 0.2;
constexpr double kConsiderMinLcNumToTarget = 2;
constexpr double kPreviewJunctionCostForLC = 5.0;
constexpr double kDistanceEpsilon = 1.0;                    // m.
constexpr double kLaneSpeedDiffToLimitRatioHighWay = 0.25;  // s.
constexpr double kEncourageLeftNudgeFactor = 1.2;
constexpr double kLengthAlongRouteBaseForHighWay = 3800.0;       // m.
constexpr double kMinLenForLaneChange = 500.0;                   // m.
constexpr double kMinLenForTwiceLaneChangeCity = 800.0;          // m.
constexpr double kMinLenForLaneChangeHighWay = 500.0;            // m.
constexpr double kConfortbleDecelerationAcc = -0.5;              // m/s^2
constexpr double kMaxDecelerationAcc = -1.0;                     // m/s^2
constexpr double kMinDecelerationAcc = -3.0;                     // m/s^2
constexpr double kInvalidLength = 10000.0;                       // m.
constexpr double kMinLaneSpeedDiffStandard = 8.0;                // m.
constexpr double kJunctionSmoothnessBase = std::sin(M_PI / 12);  // m.
constexpr double kCloseToJunctionBase = 150.0;                   // m.
constexpr double kLowSpeedLaneChangeDiscardCurveBase = 8.3;      // m/s(30kph)

// constexpr double kLengthAlongRouteBaseForLeft = 900.0;      // m.
// constexpr double kLengthAlongRouteBaseForRight = 1100.0;    // m.
// constexpr double kCurveSpeedConstraint = 0.2;
// constexpr double kLcEffectBase = 2.5;                       // m/s^2
// constexpr double kReachDestinationCutOffDist = 200.0;       // m.
// constexpr double kMaxOppositeLcTimeBound = 10.0;   // s
constexpr double kAvoidConesObviousThreshold = 0.8;
constexpr double kBeginAvoidConesChangeTheshold = 0.5;
constexpr double kBeginAvoidBusLaneChangeTheshold = 0.5;
constexpr double kExtendChangeLaneSolidBoundaryLineLength = 4.0;  // m.

constexpr double kCrossSolidStartDis = 2.0;               // m.
constexpr double kCrossSolidMinEndDis = 10.0;             // m.
constexpr double kCrossSolidPreviewTimeForCity = 2.0;     // s.
constexpr double kCrossSolidPreviewTimeForHighway = 3.5;  // s.
constexpr double kMaxTrafficJamLength = 800.0;            // m.
constexpr double kDistToTrafficJamThreshlod = 200.0;      // m.
constexpr int kTrafficJamStatusThreshlod = 3;
constexpr double kTrafficJamLengthThreshlod = 100.0;    // m.
constexpr double kNearNaviEndDistance = 1500.0;         // m
constexpr double kNearNaviEndDistanceMapless = 1000.0;  // m
// DefensiveDrivng constexpr
constexpr double kMinDefensiveBehindTrunkDist = 40.0;  // m
constexpr double kDefensiveTruckSpdDiffThrd = 1.0;     // m/s
constexpr double kDefensiveTruckTHWThrd = 3.0;         // s
constexpr double kTimetoLCThrd = 8.0;                  // s
constexpr double kTimetoLFThrd = 16.0;                 // s
constexpr double kHighwayRightLaneChangeBaseCost = 0.03;
// Defense lc derease, better use DrivePassage speed limit.
double CalcDefaultRouteDecreaseFactor(double length_along_route,
                                      double av_speed, double speed_limit,
                                      int lc_num_to_targets, bool is_highway) {
  constexpr double kMaxAllowSpeedBias = 5.0;  // m/s.
  const double expect_speed = speed_limit - av_speed > kMaxAllowSpeedBias
                                  ? av_speed
                                  : std::max(speed_limit, av_speed);
  constexpr double kBegConsiderRouteTimeSecs = 10.0;  // secs.
  const double looking_ahead_time =
      kBegConsiderRouteTimeSecs + lc_num_to_targets * 5.0;  // secs.
  const double force_lc_dist =
      is_highway ? kForceRouteLengthLcForHighWay
                 : kForceRouteLengthLcForHighWay * 0.25;  // m.
  const double looking_ahead_dist =
      std::max(expect_speed * looking_ahead_time, force_lc_dist);
  if (length_along_route < force_lc_dist) {
    return 0.0;
  }
  const auto ratio = std::clamp(
      (length_along_route - force_lc_dist) / looking_ahead_dist, 0.0, 1.0);
  return Sqr(ratio);
}

bool IsStandardJunction(const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info) {
  if (lane_seq_info == nullptr) {
    return false;
  }
  const auto junction_lane = lane_seq_info->junction_lane;
  if (junction_lane == nullptr) {
    return false;
  }
  return junction_lane->type() == LaneType::LANE_VIRTUAL_COMMON ||
         junction_lane->type() == LaneType::LANE_VIRTUAL_JUNCTION;
}

double GetMinAccelerateOrZero(
    const std::vector<ApolloTrajectoryPointProto>& traj_pts,
    int max_check_pts_num) {
  int num_pts = std::min(static_cast<int>(traj_pts.size()), max_check_pts_num);
  auto min_it = std::min_element(
      traj_pts.begin(), traj_pts.begin() + num_pts,
      [](const auto& lhs, const auto& rhs) { return lhs.a() < rhs.a(); });
  return min_it != traj_pts.begin() + num_pts ? min_it->a() : 0.0;
}

absl::StatusOr<bool> CheckNearDisIsSolidLane(
    const LaneChangeStateProto& lc_state, const LaneChangeStage& change_stage,
    const Vec2d ego_pos, const double ego_v, const double ego_front_to_ra,
    const DrivePassage& drive_passage, const bool in_high_way) {
  if (drive_passage.lane_seq_info() == nullptr ||
      (change_stage != st::LaneChangeStage::LCS_EXECUTING &&
       change_stage != st::LaneChangeStage::LCS_PAUSE))
    return false;
  bool is_near_solid_lane = false;
  const auto& lane_seq_ptr =
      drive_passage.lane_seq_info()->lane_seq;  // LaneSequencePtr
  bool check_left_solid_dashed =
      !lc_state.lc_left() && !lc_state.entered_target_lane();
  bool check_right_dashed_solid =
      lc_state.lc_left() && !lc_state.entered_target_lane();
  if (lane_seq_ptr == nullptr) return false;
  const auto& lanes =
      lane_seq_ptr->lanes();  // const std::vector<LaneConstPtr> &
  ASSIGN_OR_RETURN(
      const auto ego_sl,
      drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(ego_pos),
      _ << "Ego pos not in drive passage.");
  for (auto lane_ptr : lanes) {
    if (lane_ptr == nullptr) continue;
    const auto lane_bound_ptr = lc_state.lc_left() ? lane_ptr->right_boundary()
                                                   : lane_ptr->left_boundary();
    if (lane_bound_ptr == nullptr) continue;
    for (const auto& lane_bound : lane_bound_ptr->lane_boundaries()) {
      if (lane_bound == nullptr) continue;
      const std::vector<Vec2d>& points = lane_bound->line_curve().points();
      if (points.empty()) continue;
      ASSIGN_OR_RETURN(const auto lane_first_point_sl,
                       drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                           points.front()),
                       _ << "Boundary lane front point not on drive passage.");
      ASSIGN_OR_RETURN(const auto lane_end_point_sl,
                       drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                           points.back()),
                       _ << "Boundary lane back point not on drive passage.");
      if (lane_end_point_sl.s <
          (ego_sl.s + ego_front_to_ra + kCrossSolidStartDis))
        continue;
      const double preview_cross_solid_dis =
          in_high_way ? std::max(ego_v * kCrossSolidPreviewTimeForHighway,
                                 kCrossSolidMinEndDis)
                      : std::max(ego_v * kCrossSolidPreviewTimeForCity,
                                 kCrossSolidMinEndDis);
      if (lane_first_point_sl.s >
          ego_sl.s + ego_front_to_ra + preview_cross_solid_dis)
        break;
      bool is_solid_and_dashed = false;
      if (check_left_solid_dashed)
        is_solid_and_dashed = lane_bound->type().line_type ==
                              ad_byd::planning::LineType::SOLID_DASHED;
      if (check_right_dashed_solid)
        is_solid_and_dashed = lane_bound->type().line_type ==
                              ad_byd::planning::LineType::DASHED_SOLID;
      if (lane_bound->type().line_type == ad_byd::planning::LineType::SOLID ||
          lane_bound->type().line_type ==
              ad_byd::planning::LineType::SOLID_SOLID ||
          is_solid_and_dashed) {
        is_near_solid_lane = true;
        break;
      }
    }
  }
  return is_near_solid_lane;
}

/// @return <cross road boundary, cross solid line>
absl::StatusOr<std::pair<bool, bool>> IsCrossSolidByTraj(
    const DrivePassage& drive_passage,
    const std::vector<ApolloTrajectoryPointProto>& traj_pts,
    const LaneChangeStateProto& lc_state, double ego_width, double ego_length,
    double ego_front_to_ra, const bool target_task_switched) {
  // TODO(xiang): Refactor it!
  if (traj_pts.empty()) {
    return std::pair(false, false);
  }
  constexpr int kCheckEveryNPt = 5;
  constexpr double kTrajectorySExtension = 10.0;  // m.

  bool check_left_solid_dashed =
      !lc_state.lc_left() && !lc_state.entered_target_lane();
  bool check_right_dashed_solid =
      lc_state.lc_left() && !lc_state.entered_target_lane();
  const auto& change_stage = lc_state.stage();
  // Only check a first small part of trajectory on lane change pause
  // to avoid end of trajectory cross line
  const int check_first_n =
      change_stage == LaneChangeStage::LCS_PAUSE
          ? std::min<int>(CeilToInt(0.3 * traj_pts.size()) + 1, traj_pts.size())
          : traj_pts.size();
  const auto last_pt_in_rear_center =
      Vec2dFromApolloTrajectoryPointProto(traj_pts[check_first_n - 1]);
  const auto heading = traj_pts[check_first_n - 1].path_point().theta();
  const auto unit = Vec2d::UnitFromAngle(heading);
  const auto last_pt_in_front_center =
      last_pt_in_rear_center + unit * ego_front_to_ra;

  // 2. get solid boundary from drive passage
  ASSIGN_OR_RETURN(const auto last_pt_in_front_center_frenet,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       last_pt_in_front_center),
                   _ << "Last considered traj point not on drive passage.");
  ASSIGN_OR_RETURN(const auto first_point_sl,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(traj_pts.front())),
                   _ << "First traj point not on drive passage.");
  const auto solid_boundaries = FindSolidBoundaryIntervals(
      drive_passage, first_point_sl,
      last_pt_in_front_center_frenet.s + kTrajectorySExtension,
      kRouteStationUnitStep);

  // 3. generate ego car trajectory box
  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(
      CeilToInt(check_first_n / static_cast<float>(kCheckEveryNPt)));
  const double rear_to_real_center = ego_front_to_ra - 0.5 * ego_length;
  for (int i = 0; i < check_first_n; i += kCheckEveryNPt) {
    const auto& traj_pt = traj_pts[i];
    const double heading = traj_pt.path_point().theta();
    Box2d ego_box(Vec2dFromApolloTrajectoryPointProto(traj_pt), heading,
                  ego_length, ego_width);
    ego_box.Shift(Vec2d::UnitFromAngle(heading) * rear_to_real_center);
    ego_boxes.emplace_back(std::move(ego_box));
  }

  // get road boudary
  std::vector<Segment2d> left_road_boundary_point_segs,
      right_road_boundary_point_segs, left_bound_point_segs_solid_white,
      right_bound_point_segs_solid_white;
  if (drive_passage.lane_seq_info() != nullptr &&
      (change_stage == st::LaneChangeStage::LCS_EXECUTING ||
       change_stage == st::LaneChangeStage::LCS_RETURN ||
       change_stage == st::LaneChangeStage::LCS_PAUSE ||
       target_task_switched)) {
    const auto& lane_seq_ptr =
        drive_passage.lane_seq_info()->lane_seq;  // LaneSequencePtr
    if (lane_seq_ptr != nullptr) {
      const auto& lanes =
          lane_seq_ptr->lanes();  // const std::vector<LaneConstPtr> &
      for (auto lane_ptr : lanes) {
        if (lane_ptr == nullptr) continue;
        auto left_bound_ptr = lane_ptr->left_boundary();
        auto right_bound_ptr = lane_ptr->right_boundary();

        auto left_road_bound_ptr =
            lane_ptr->left_road_boundary();  // const RoadBoundariesConstPtr &
        auto right_road_bound_ptr =
            lane_ptr->right_road_boundary();  // const RoadBoundariesConstPtr &
        // left
        if (left_road_bound_ptr != nullptr) {
          // const std::vector<RoadBoundaryConstPtr> &
          for (const auto& road_bound :
               left_road_bound_ptr->road_boundaries()) {
            if (road_bound != nullptr) {
              const std::vector<Vec2d>& points =
                  road_bound->line_curve().points();
              for (int i = 1; i < points.size(); i++) {
                left_road_boundary_point_segs.emplace_back(points[i - 1],
                                                           points[i]);
              }
            }
          }
        }
        // rignt
        if (right_road_bound_ptr != nullptr) {
          // const std::vector<RoadBoundaryConstPtr> &
          for (const auto& road_bound :
               right_road_bound_ptr->road_boundaries()) {
            if (road_bound != nullptr) {
              const std::vector<Vec2d>& points =
                  road_bound->line_curve().points();
              for (int i = 1; i < points.size(); i++) {
                right_road_boundary_point_segs.emplace_back(points[i - 1],
                                                            points[i]);
              }
            }
          }
        }

        // left bound
        size_t solid_index = 0;
        if (left_bound_ptr) {
          for (const auto& l_bound : left_bound_ptr->lane_boundaries()) {
            if (l_bound == nullptr) continue;
            if (l_bound->type().line_type ==
                    ad_byd::planning::LineType::SOLID ||
                l_bound->type().line_type ==
                    ad_byd::planning::LineType::SOLID_SOLID ||
                (check_left_solid_dashed &&
                 l_bound->type().line_type ==
                     ad_byd::planning::LineType::SOLID_DASHED)) {
              const std::vector<Vec2d>& points = l_bound->line_curve().points();
              for (int i = 1; i < points.size(); i++) {
                left_bound_point_segs_solid_white.emplace_back(points[i - 1],
                                                               points[i]);
              }
              if (solid_index <
                  (left_bound_ptr->lane_boundaries().size() - 1)) {
                const auto& next_l_bound =
                    left_bound_ptr->lane_boundaries().at(solid_index + 1);
                if (next_l_bound == nullptr) continue;
                const std::vector<Vec2d>& next_lane_points =
                    next_l_bound->line_curve().points();

                // neighbor point check,distance distance < 0.1m
                if (next_lane_points.size() > 0 && points.size() > 0) {
                  const auto lane_cross_point_diff_vec =
                      points.back() - next_lane_points.front();
                  if (lane_cross_point_diff_vec.Length() > 0.1) break;
                }

                // extend left boundry solid line distance
                double extend_acc_dis = 0.0;
                for (int i = 1; i < next_lane_points.size(); i++) {
                  const auto next_point_vec =
                      next_lane_points[i - 1] - next_lane_points[i];
                  extend_acc_dis = extend_acc_dis + next_point_vec.Length();
                  if (extend_acc_dis > kExtendChangeLaneSolidBoundaryLineLength)
                    break;
                  left_bound_point_segs_solid_white.emplace_back(
                      next_lane_points[i - 1], next_lane_points[i]);
                }
              }
            }
            solid_index++;
          }
        }

        // right bound
        solid_index = 0;
        if (right_bound_ptr) {
          for (const auto& r_bound : right_bound_ptr->lane_boundaries()) {
            if (r_bound == nullptr) continue;
            if (r_bound->type().line_type ==
                    ad_byd::planning::LineType::SOLID ||
                r_bound->type().line_type ==
                    ad_byd::planning::LineType::SOLID_SOLID ||
                (check_right_dashed_solid &&
                 r_bound->type().line_type ==
                     ad_byd::planning::LineType::DASHED_SOLID)) {
              const std::vector<Vec2d>& points = r_bound->line_curve().points();
              for (int i = 1; i < points.size(); i++) {
                right_bound_point_segs_solid_white.emplace_back(points[i - 1],
                                                                points[i]);
              }
              if (solid_index <
                  (right_bound_ptr->lane_boundaries().size() - 1)) {
                const auto& next_r_bound =
                    right_bound_ptr->lane_boundaries().at(solid_index + 1);
                if (next_r_bound == nullptr) continue;
                const std::vector<Vec2d>& next_lane_points =
                    next_r_bound->line_curve().points();

                // neighbor point check,distance distance < 0.1m
                if (next_lane_points.size() > 0 && points.size() > 0) {
                  const auto lane_cross_point_diff_vec =
                      points.back() - next_lane_points.front();
                  if (lane_cross_point_diff_vec.Length() > 0.1) break;
                }

                // extend right boundry solid line distance
                double extend_acc_dis = 0.0;
                for (int i = 1; i < next_lane_points.size(); i++) {
                  const auto next_point_vec =
                      next_lane_points[i - 1] - next_lane_points[i];
                  extend_acc_dis = extend_acc_dis + next_point_vec.Length();
                  if (extend_acc_dis > kExtendChangeLaneSolidBoundaryLineLength)
                    break;
                  right_bound_point_segs_solid_white.emplace_back(
                      next_lane_points[i - 1], next_lane_points[i]);
                }
              }
            }
            solid_index++;
          }
        }
      }
    }
  }
  // check if collide with road boudaries when performing lane change
  bool has_overlap = false, has_overlap_solid_white(false);
  if (change_stage == st::LaneChangeStage::LCS_EXECUTING ||
      change_stage == st::LaneChangeStage::LCS_RETURN ||
      change_stage == st::LaneChangeStage::LCS_PAUSE || target_task_switched) {
    for (int i = 0; i < ego_boxes.size(); i++) {
      const auto ego_box = ego_boxes[i];
      // check left
      for (const auto& segment : left_road_boundary_point_segs) {
        has_overlap = ego_box.HasOverlapWithBuffer(segment, 0.1, 0.2);
        if (has_overlap) {
          Log2DDS::LogDataV0("cross solid,segment.start.x",
                             segment.start().x());
          Log2DDS::LogDataV0("cross solid,segment.start.y",
                             segment.start().y());
          Log2DDS::LogDataV0("cross solid,segment.end.x", segment.end().x());
          Log2DDS::LogDataV0("cross solid,segment.end.y", segment.end().y());
          break;
        }
      }
      if (has_overlap) break;

      // check right
      for (const auto& segment : right_road_boundary_point_segs) {
        has_overlap = ego_box.HasOverlapWithBuffer(segment, 0.1, 0.2);
        if (has_overlap) {
          Log2DDS::LogDataV0("cross solid,segment.start.x",
                             segment.start().x());
          Log2DDS::LogDataV0("cross solid,segment.start.y",
                             segment.start().y());
          Log2DDS::LogDataV0("cross solid,segment.end.x", segment.end().x());
          Log2DDS::LogDataV0("cross solid,segment.end.y", segment.end().y());
          break;
        }
      }
      if (has_overlap) break;

      // check left bound
      for (const auto& segment : left_bound_point_segs_solid_white) {
        has_overlap_solid_white =
            ego_box.HasOverlapWithBuffer(segment, 0.1, 0.2);

        if (has_overlap_solid_white) {
          Log2DDS::LogDataV0("cross solid,segment.start.x",
                             segment.start().x());
          Log2DDS::LogDataV0("cross solid,segment.start.y",
                             segment.start().y());
          Log2DDS::LogDataV0("cross solid,segment.end.x", segment.end().x());
          Log2DDS::LogDataV0("cross solid,segment.end.y", segment.end().y());
          break;
        }
      }
      if (has_overlap_solid_white) break;
      // check right bound
      for (const auto& segment : right_bound_point_segs_solid_white) {
        has_overlap_solid_white =
            ego_box.HasOverlapWithBuffer(segment, 0.1, 0.2);
        if (has_overlap_solid_white) {
          Log2DDS::LogDataV0("cross solid,segment.start.x",
                             segment.start().x());
          Log2DDS::LogDataV0("cross solid,segment.start.y",
                             segment.start().y());
          Log2DDS::LogDataV0("cross solid,segment.end.x", segment.end().x());
          Log2DDS::LogDataV0("cross solid,segment.end.y", segment.end().y());
          break;
        }
      }
      if (has_overlap_solid_white) break;
    }
  }
  return std::pair(has_overlap, has_overlap_solid_white);
}

double CalcPassedSplitTimeFactor(double time) {
  static const PiecewiseLinearFunction<double, double>
      kSplitSuppressTimeFactorPlf = {{0.0, 10.0, 15.0}, {1.0, 0.66, 0.0}};
  return kSplitSuppressTimeFactorPlf(time);
}

}  // namespace

double CalcuNaviLengthAfterRoutePreiew(const SelectorCostInput& cost_input,
                                       const int idx, const bool lc_ongoing,
                                       const double len_before_intersection,
                                       const double ego_v,
                                       const double navi_length_along_route) {
  constexpr double kPreviewTime = 5.0;
  constexpr double kMinPreviewDist = 80.0;
  const double kMaxPreviewDistBeforeJunction = 300.0;
  const bool is_route_preview =
      len_before_intersection < kMaxPreviewDistBeforeJunction;
  if (!is_route_preview || cost_input.last_selected_idx != idx || !lc_ongoing) {
    return navi_length_along_route;
  }
  const PiecewiseLinearFunction<double, double> kDampingFactor = {
      {0.0, 100.0, 200.0, 250.0, 300.0},  // length error
      {1.0, 1.0, 1.0, 0.3, 0.0}};         // factor
  const double decrease_factor = kDampingFactor(len_before_intersection);
  const double preivew_dist =
      std::max(ego_v * kPreviewTime, kMinPreviewDist) * decrease_factor;
  return std::max(navi_length_along_route - preivew_dist, 10.0);
}

double GetPreviewDistanceThreshold(int lc_num, bool in_high_way) {
  constexpr double kHighwayPreviewMinDistanceThreshold = 2000.0;  // m.
  constexpr double kHighwayPreviewMaxDistanceThreshold = 2500.0;  // m.
  constexpr double kPreviewMinDistanceThreshold = 900.0;          // m.
  constexpr double kPreviewMaxDistanceThreshold = 1200.0;         // m.
  if (lc_num < kConsiderMinLcNumToTarget) {
    return 0.0;
  }

  if (lc_num == kConsiderMinLcNumToTarget) {
    return in_high_way ? kHighwayPreviewMinDistanceThreshold
                       : kPreviewMinDistanceThreshold;
  }

  return in_high_way ? kHighwayPreviewMaxDistanceThreshold
                     : kPreviewMaxDistanceThreshold;
}

byd::msg::orin::routing_map::NaviActionInfo FindNearestNonStraightActionInfo(
    const byd::msg::orin::routing_map::MapEvent& map_event) {
  byd::msg::orin::routing_map::NaviActionInfo navi_aciton_info;
  navi_aciton_info.set_main_action(byd::msg::orin::routing_map::NMA_NONE);
  navi_aciton_info.set_action_dis(DBL_MAX);
  for (const auto& action : map_event.navi_action()) {
    if (action.action_dis() < 100.0) continue;
    if (action.main_action() != byd::msg::orin::routing_map::NMA_NONE &&
        action.main_action() != ns_routing_map::NMA_CONTINUE &&
        action.main_action() != byd::msg::orin::routing_map::NMA_STRAIGHT &&
        action.main_action() != byd::msg::orin::routing_map::NAA_UNKNOWN) {
      navi_aciton_info = action;
      break;
    }
  }
  return navi_aciton_info;
}

double TrafficJamSmoothFactor(const double lower, const double upper,
                              const double x) {
  return 1.0 - std::pow(Lerp(0.0, lower, 1.0, upper, x, true), 2);
}

bool IsInTrafficJamArea(const SelectorCommonFeature& common_feature) {
  const double dist_to_start_traffic_jam = static_cast<double>(
      common_feature.traffic_jam_info.dist_to_start_traffic_jam());
  const int traffic_jam_status =
      static_cast<int>(common_feature.traffic_jam_info.traffic_jam_status());
  const double traffic_jam_dist =
      static_cast<double>(common_feature.traffic_jam_info.traffic_jam_dist());
  return !common_feature.in_high_way &&
         dist_to_start_traffic_jam < kDistToTrafficJamThreshlod &&
         traffic_jam_dist >= kTrafficJamLengthThreshlod &&
         traffic_jam_status >= kTrafficJamStatusThreshlod;
}

bool ShouldFixNaviLengthAlongRoute(const bool in_traffic_jam,
                                   const bool is_ego_on_pref_lane,
                                   const bool lc_ongoing,
                                   const bool is_curr_lane_pref) {
  return (in_traffic_jam && !is_ego_on_pref_lane && !is_curr_lane_pref) ||
         (in_traffic_jam && is_ego_on_pref_lane && lc_ongoing &&
          !is_curr_lane_pref);
}

absl::StatusOr<CostVec> TrajProgressCost::ComputeCostV2(
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CostVec cost_vec(4);
  CHECK_NOTNULL(extra_info);
  // const auto st_traj_mgr = cost_input.st_traj_manager;
  const uint64_t start_lane_id =
      planner_output.scheduler_output.drive_passage.lane_path()
          .front()
          .lane_id();
  auto start_lane_info = psmm_->FindCurveLaneByIdOrNull(start_lane_id);
  auto lane_seq_info =
      planner_output.scheduler_output.drive_passage.lane_seq_info();

  const double driving_dist =
      lane_seq_info ? lane_seq_info->dist_to_navi_end : kInvalidLength;
  const bool on_highway =
      common_feature()->in_high_way || common_feature()->preview_in_high_way;

  const auto lc_stage =
      planner_output.scheduler_output.lane_change_state.stage();
  const bool lc_left =
      planner_output.scheduler_output.lane_change_state.lc_left();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_RETURN ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  // 1. Compute the trajectory length.
  const auto& scheduler_output = planner_output.scheduler_output;
  if (!common_feature()->lane_feature_infos.contains(idx)) {
    return absl::NotFoundError("Invalid lane_feature_infos.");
  }
  const auto& lane_feature_info =
      FindOrDieNoPrint(common_feature()->lane_feature_infos, idx);
  const auto s = planner_output.traj_points.empty()
                     ? 0.0
                     : planner_output.traj_points.back().path_point().s();
  const double standard_progress =
      std::max(kMinStandardProgressDiff, kTrajectoryTimeHorizon * ego_v_);

  cost_vec[0] = std::max(0.0, standard_progress - s) / standard_progress;

  if (!lane_speed_map_.contains(idx)) {
    return absl::NotFoundError("Invalid lane_speed_map_.");
  }
  // 2. Regularization is related to lane speed limit.
  const auto [init_leader_speed, max_leader_speed, lane_speed_limit_by_leader,
              lane_speed_limit, block_id, nearest_obj_s] =
      FindOrDieNoPrint(lane_speed_map_, idx);

  const auto& slow_working_obj = FindOrDieNoPrint(slow_working_objs_map_, idx);
  const auto ref_speed_diff =
      on_highway
          ? std::max(kMinLaneSpeedDiffStandard,
                     kLaneSpeedDiffToLimitRatioHighWay * lane_speed_limit)
          : std::max(kMinLaneSpeedDiffStandard,
                     kLaneSpeedDiffToLimitRatio * lane_speed_limit);
  const auto& ego_box =
      planner_output.scheduler_output.av_frenet_box_on_drive_passage;
  // Consider different driving style.
  const auto driving_style_factor =
      GetDrivingStyleFactor(driving_style_gear_, on_highway);
  double driving_dist_factor = std::min(1.0, driving_dist / 300.0);
  driving_dist_factor = driving_dist_factor * driving_dist_factor;

  double turn_factor = 1.0;
  if (start_lane_info &&
      (start_lane_info->turn_type() == ad_byd::planning::LEFT_TURN ||
       start_lane_info->turn_type() == ad_byd::planning::RIGHT_TURN)) {
    turn_factor = 0.05;
  }

  constexpr double kDisFactorTCC = 6.0;  // s.
  const double dis_factor_base =
      on_highway ? std::clamp(ego_v_ * kDisFactorTCC, 60.0, 150.0) : 60.0;
  double obs_dis_factor =
      nearest_obj_s - ego_box.s_max > dis_factor_base
          ? 0.0
          : 1.0 - Sqr((nearest_obj_s - ego_box.s_max) / dis_factor_base);
  double dist_to_virtual_lane = std::min(
      kInvalidLength, common_feature()->road_horizon_info.dist_to_cross);

  const bool is_conversative_style =
      lane_change_style_ == LaneChangeStyle::LC_STYLE_CONSERVATIVE;

  const auto speed_limit_cost_output =
      CalculateSpeedLimitCost(SpeedLimitCostInput{
          .is_highway = on_highway,
          .driving_style_factor = driving_style_factor,
          .max_lane_speed = max_lane_speed_,
          .lane_speed_limit = lane_speed_limit,
          .max_init_leader_speed = max_init_leader_speed_,
          .init_leader_speed = init_leader_speed,
          .lane_speed_limit_by_leader = lane_speed_limit_by_leader,
          .ego_v = ego_v_,
          .dist_to_virtual_lane = dist_to_virtual_lane,
          .is_conversative_style = is_conversative_style,
      });
  double speed_limit_cost = speed_limit_cost_output.speed_limit_cost();
  const double driving_speed_diff = ego_v_ - lane_speed_limit_by_leader;
  if (on_highway) obs_dis_factor = 1.0;
  speed_limit_cost *= obs_dis_factor;

  // History follow cost
  double history_follow_cost = speed_limit_cost;
  // Add right lane change base cost
  if (!lc_left && lc_stage == LaneChangeStage::LCS_EXECUTING)
    history_follow_cost += kHighwayRightLaneChangeBaseCost;

  const double ego_v_ratio = ego_v_ / std::max(Kph2Mps(20.0), lane_speed_limit);
  const bool is_valid_history_follow_ego_v =
      (ego_v_ratio > 0.5 || ego_v_ > Kph2Mps(60.0)) && ego_v_ratio < 1.0;
  const bool is_valid_speed_diff =
      (lane_speed_limit_by_leader != max_lane_speed_);
  const double leader_v_ratio =
      lane_speed_limit_by_leader / std::max(Kph2Mps(20.0), max_lane_speed_);
  const double init_leader_speed_diff = max_lane_speed_ - init_leader_speed;
  const bool is_slow_driving = on_highway
                                   ? init_leader_speed_diff >= Kph2Mps(5.0)
                                   : init_leader_speed_diff >= Kph2Mps(3.0);
  if (block_id.has_value() && block_id != "stopline" &&
      lane_feature_info.nearest_leader.has_value()) {
    // Only calc the cost, but Not deploy yet.
    auto last_follow_cost_info =
        traj_feature_output->object_cost_map[*block_id];
    auto follow_cost_info = ComputeFollowCost(
        lane_feature_info.nearest_leader->obj_type, speed_limit_cost,
        last_follow_cost_info.integration, is_valid_history_follow_ego_v,
        is_valid_speed_diff, is_slow_driving);
    history_follow_cost = follow_cost_info.final_follow_cost;
    follow_cost_info.obj_id = *block_id;
    follow_cost_info.ts = common_feature()->plan_time;
    follow_cost_info.count = last_follow_cost_info.count + 1;
    traj_feature_output->object_cost_map[*block_id] =
        std::move(follow_cost_info);
  }

  constexpr double kHighwayProgressFactor = 1.5;
  history_follow_cost =
      std::clamp(history_follow_cost, 0.0,
                 1.0 * (on_highway ? kHighwayProgressFactor : 1.0));

  cost_vec[1] = FLAGS_planner_enable_selector_cost_history ? history_follow_cost
                                                           : speed_limit_cost;
  if (is_conversative_style && on_highway) {
    cost_vec[1] = speed_limit_cost;
  }
  constexpr double kLookAheadDistance = 150.0;  // m.
  auto valid_lane_num_op = FindMinValidLaneNumAtMost(
      common_feature()->mpp_section, kLookAheadDistance);
  traj_feature_output->overtake_begin_frame = CalcOvertakeFrame(
      OvertakeFrameInput{
          .leader_speed_diff = init_leader_speed - lane_keep_init_leader_speed_,
          .ego_v = ego_v_,
          .valid_lane_num = lane_feature_info.valid_lane_num,
          .lc_left = lc_left,
          .on_highway = common_feature()->in_high_way,
      },
      is_conversative_style);
  const int is_curr_lane_pref = scheduler_output.lc_num == 0;
  // when near the navi end, decay the follow slow cost of pref lane, push it to
  // pref lane
  double route_decrease_factor = 1.0;
  if (!cost_input.is_ego_on_pref_lane && is_curr_lane_pref &&
      !planner_output.traj_points.empty()) {
    route_decrease_factor = CalcDefaultRouteDecreaseFactor(
        cost_input.ego_lane_dist_to_navi_end, planner_output.traj_points[0].v(),
        lane_feature_info.speed_limit,
        /*lc_num_to_targets*/ 1, on_highway);
    route_decrease_factor = pow(route_decrease_factor, 2);
    cost_vec[1] *= route_decrease_factor;
  }

  traj_feature_output->progress_factor =
      speed_limit_cost_output.progress_factor();
  traj_feature_output->has_progress_cost = (cost_vec[1] > 0.0) ? true : false;
  extra_info->emplace_back(absl::StrFormat("in_high_way: %d", on_highway));
  extra_info->emplace_back(DebugFormat(speed_limit_cost_output));
  extra_info->emplace_back(absl::StrCat(
      "has_progress_cost: ", traj_feature_output->has_progress_cost));
  extra_info->emplace_back(
      absl::StrCat("is_conversative_style: ", btoa(is_conversative_style)));

  constexpr double kEpsilon = 1.0;  // m.
  if (slow_working_obj.has_value() &&
      slow_working_obj->is_slow_working_object &&
      slow_working_obj->is_large_vehicle &&
      slow_working_obj->object_s < min_slow_working_object_s_ + kEpsilon) {
    cost_vec[2] = slow_working_obj->probability;
  }

  const double preview_s = 10.0;
  // Using Dis To Junction
  const bool ego_in_intersection =
      common_feature()->ego_in_tl_controlled_intersection;
  bool preview_in_intersection = ego_in_intersection;
  const auto& drive_passage = planner_output.scheduler_output.drive_passage;
  if (drive_passage.lane_seq_info() != nullptr) {
    const double dist_to_intersection =
        common_feature()->road_horizon_info.dist_to_cross;
    preview_in_intersection =
        preview_in_intersection || (dist_to_intersection < preview_s);
  }

  double weight_lat = 0.04;
  double weight_lon = 0.027777778;
  // add object lateral and longitudinal cost
  double objects_dist_cost_sum = 0.0;
  if (cost_input.preview_in_intersection) {
    int points_num =
        std::min(30, static_cast<int>(planner_output.traj_points.size()));
    for (int i = 0; i < points_num; i += 2) {
      double object_dist_cost_sum = 0.0;
      for (const auto& obj_traj : cost_input.stm->trajectories()) {
        double lat_dis = 0.0;
        double lon_dis = 0.0;
        // find bypass time at object trajectory
        double bypass_time = 15.0;
        size_t bypass_traj_point_idx = 0;
        const auto& traj_point = planner_output.traj_points[i];
        if (!obj_traj.trajectory().points().empty()) {
          size_t next_traj_point_idx =
              obj_traj.trajectory().points().size() - 1;
          while (next_traj_point_idx - bypass_traj_point_idx > 1) {
            size_t mid_idx = (next_traj_point_idx + bypass_traj_point_idx) >> 1;
            if (obj_traj.trajectory().points().at(mid_idx).t() <
                traj_point.relative_time()) {
              bypass_traj_point_idx = mid_idx;
            } else {
              next_traj_point_idx = mid_idx;
            }
          }
        }
        const auto& obj_point =
            obj_traj.trajectory().points().at(bypass_traj_point_idx);
        const double dx = obj_point.pos().x() - traj_point.path_point().x();
        const double dy = obj_point.pos().y() - traj_point.path_point().y();
        const double sint = std::sin(traj_point.path_point().theta());
        const double cost = std::cos(traj_point.path_point().theta());
        const double dist_sqr = dx * dx + dy * dy;
        if (dist_sqr > 36.0) {
          continue;
        }
        lat_dis = std::abs(-dx * sint + dy * cost);
        if (lat_dis > 5.0) {
          continue;
        }
        lon_dis = std::abs(dx * cost + dy * sint);
        double object_dist_cost =
            weight_lon * (6.0 - lon_dis) * (6.0 - lon_dis) +
            weight_lat * (5.0 - lat_dis) * (5.0 - lat_dis);
        object_dist_cost_sum += object_dist_cost;
      }
      objects_dist_cost_sum += object_dist_cost_sum;
    }
  }
  if (objects_dist_cost_sum > 1e-2) {
    *(cost_input.is_nearby_obs) = true;
    traj_feature_output->lane_change_for_intersection_obs = true;
  }
  cost_vec[3] = 0.0;

  // Generate lane change reason.
  if ((max_lane_speed_ - lane_speed_limit) / max_lane_speed_ >
      kCutOffLaneSpeedDiffRatio) {
    traj_feature_output->lane_change_for_road_speed_limit = true;
  }

  if (lane_feature_info.nearest_leader.has_value()) {
    const auto& nearest_leader = *lane_feature_info.nearest_leader;
    if (nearest_leader.is_stationary) {
      if (nearest_leader.obj_type == ObjectType::OT_LARGE_VEHICLE ||
          nearest_leader.obj_type == ObjectType::OT_VEHICLE) {
        traj_feature_output->lane_change_for_stationary_vehicle = true;
      } else if (IsStaticObjectType(nearest_leader.obj_type)) {
        traj_feature_output->lane_change_for_stationary_obj = true;
      }
    }
  }

  if (slow_working_obj.has_value() &&
      !traj_feature_output->lane_change_for_stationary_obj &&
      cost_vec[2] > kSlowWorkingObviousThreshold) {  // todo(xxx) adapt block
    traj_feature_output->lane_change_for_moving_obj = true;
  }
  if (block_id.has_value() && *block_id != "stopline" &&
      !traj_feature_output->lane_change_for_stationary_obj &&
      cost_vec[1] > kFollowSlowObviousThreshold) {
    traj_feature_output->lane_change_for_moving_obj = true;
  }
  if (block_id.has_value() && *block_id == "stopline") {
    // Consider route cost for merge lane.
    traj_feature_output->lane_change_for_route_cost = true;
  }

  const auto& traj_pts = planner_output.traj_points;
  constexpr int kMaxConsiserAccelPointsNum = 20;
  if (traj_pts.size() > kMaxConsiserAccelPointsNum) {
    const auto [min_a_traj, max_a_traj] = std::minmax_element(
        traj_pts.begin(), traj_pts.begin() + kMaxConsiserAccelPointsNum,
        [](const auto& pt1, const auto& pt2) { return pt1.a() < pt2.a(); });
    constexpr double kMinNonDecel = -1E-6;
    constexpr double kMinAccelAccel = 0.8;
    if (min_a_traj->a() > kMinNonDecel && max_a_traj->a() > kMinAccelAccel) {
      traj_feature_output->is_accel_traj_start = true;
    }
  }
  extra_info->emplace_back(
      absl::StrFormat("Progress: %.2f / %.2f", s, standard_progress));
  extra_info->emplace_back(absl::StrFormat(
      "lane_speed_limit: %.2f,  lane_limit_leader: %.2f, ref_speed_diff: %.2f",
      lane_speed_limit, lane_speed_limit_by_leader, ref_speed_diff));
  extra_info->back() += (absl::StrFormat(
      " = Max_lane_speed: %.2f * Ratio: %.2f", max_lane_speed_,
      common_feature()->in_high_way ? kLaneSpeedDiffToLimitRatioHighWay
                                    : kLaneSpeedDiffToLimitRatio));
  if (block_id.has_value()) {
    extra_info->emplace_back(
        absl::StrFormat("Leader percep v: %.2f, max predict v: %.2f "
                        "m/s,follow_cost_info.count:%d ",
                        init_leader_speed, max_leader_speed,
                        traj_feature_output->object_cost_map[*block_id].count));
    extra_info->back() += absl::StrFormat(" from %s", *block_id);
  }
  extra_info->emplace_back(absl::StrFormat(
      "route_decrease_factor: %.2f, speed_limit_cost: %.2f, "
      "history_follow_cost: %.2f",
      route_decrease_factor, speed_limit_cost, history_follow_cost));
  extra_info->emplace_back(
      absl::StrFormat("Factor: drive_dist: %.2f, turn: %.2f ,obs: %.2f",
                      driving_dist_factor, turn_factor, obs_dis_factor));
  extra_info->emplace_back(absl::StrFormat(
      "History: is_valid_ego_v: %s, is_valid_speed_diff: %s,ego_v_ratio: "
      "%.2f,ego_v_: "
      "%.2f ,lane_speed_limit: %.2f",
      btoa(is_valid_history_follow_ego_v), btoa(is_valid_speed_diff),
      ego_v_ratio, ego_v_, lane_speed_limit));
  extra_info->emplace_back(
      absl::StrFormat("Overtake_frame: init_leader_speed: %.2f, "
                      "lane_keep_init_leader_speed: %.2f,ego_v:%.2f",
                      init_leader_speed, lane_keep_init_leader_speed_, ego_v_));

  extra_info->emplace_back(absl::StrFormat(
      "History: is_slow_driving: %s, leader_v_ratio: "
      "%.2f,lane_speed_limit_by_leader: %.2f ,max_lane_speed: "
      "%.2f,min_lane_speed: %.2f",
      btoa(is_slow_driving), leader_v_ratio, lane_speed_limit_by_leader,
      max_lane_speed_, min_lane_speed_));

  if (slow_working_obj.has_value()) {
    extra_info->emplace_back(absl::StrFormat(
        "Id: %s, Slow Prob: %.2f, Leader count: %d, Large: %s",
        slow_working_obj->object_id, slow_working_obj->probability,
        slow_working_obj->leader_count,
        btoa(slow_working_obj->is_large_vehicle)));
  }

  extra_info->emplace_back(absl::StrFormat(
      "valid_lane_num: %d, right_index: %d, preivew_valid_lane_num: "
      "%d, preivew_right_index: %d",
      lane_feature_info.valid_lane_num, lane_feature_info.right_index,
      lane_feature_info.preview_valid_lane_num,
      lane_feature_info.preivew_right_index));

  extra_info->emplace_back(
      absl::StrFormat("objects_dist_cost_sum: %.2f", objects_dist_cost_sum));
  extra_info->emplace_back(absl::StrFormat(
      "Emergency overtake: Most: %s, Normal: "
      "%s,driving_speed_diff: %.2f,lane_speed_limit_by_leader: %.2f,ego_v_: "
      "%.2f",
      btoa(traj_feature_output->lane_change_for_most_emergency_overtake),
      btoa(traj_feature_output->lane_change_for_normal_emergency_overtake),
      driving_speed_diff, lane_speed_limit_by_leader, ego_v_));
  return cost_vec;
}

absl::StatusOr<CostVec> TrajMaxJerkCost::ComputeCostV2(
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CostVec cost_vec(5);
  CHECK_NOTNULL(extra_info);

  const auto lc_stage =
      planner_output.scheduler_output.lane_change_state.stage();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_RETURN ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  const auto& traj_pts = planner_output.traj_points;
  const int n_pts = traj_pts.size();
  int max_accel_idx = -1, max_decel_idx = -1, max_lat_idx = -1,
      max_deacc_idx = -1;

  double accel_jerk_cost = 0.0, decel_jerk_cost = 0.0, deacc_cost = 0.0;
  for (int i = 0; i < n_pts; ++i) {
    if (i >= kTrajectorySteps) break;
    const auto lon_jerk_cost = traj_pts[i].j() * coeffs_[i];
    if (lon_jerk_cost > accel_jerk_cost) {
      accel_jerk_cost = lon_jerk_cost;
      max_accel_idx = i;
    }
    if (lon_jerk_cost < decel_jerk_cost) {
      decel_jerk_cost = lon_jerk_cost;
      max_decel_idx = i;
    }

    if (traj_pts[i].a() > kMaxDecelerationAcc) {
      continue;
    }
    const auto lon_acc_cost = coeffs_[i] *
                              (kMaxDecelerationAcc - traj_pts[i].a()) /
                              (kMaxDecelerationAcc - kMinDecelerationAcc);
    if (deacc_cost < lon_acc_cost) {
      deacc_cost = lon_acc_cost;
      max_deacc_idx = i;
    }
  }

  constexpr int kCheckPointsNum = 30;  // ~3s.
  const double min_a = GetMinAccelerateOrZero(traj_pts, kCheckPointsNum);

  if (min_a < kConfortbleDecelerationAcc) {
    traj_feature_output->uncomfortable_decel = true;
  }

  traj_feature_output->min_a = min_a;

  // a < -3 m/s^2
  if (min_a < kMinDecelerationAcc) {
    traj_feature_output->lane_change_for_emergency = true;
  }

  cost_vec[0] = std::max(accel_jerk_cost / accel_jerk_constraint_,
                         decel_jerk_cost / decel_jerk_constraint_);

  double max_lat_jerk_cost = 0.0;
  if (n_pts > 1) {
    std::vector<double> psi(n_pts);
    psi[0] = CalcPsi(traj_pts[0], traj_pts[1]);
    for (int i = 1; i < n_pts - 1; ++i) {
      psi[i] = CalcPsi(traj_pts[i - 1], traj_pts[i + 1]);
    }
    psi[n_pts - 1] = CalcPsi(traj_pts[n_pts - 1], traj_pts[n_pts - 2]);
    for (int i = 0; i < n_pts; ++i) {
      double lat_jerk_coeff = i >= kTrajectorySteps ? 0.0 : coeffs_lat_[i];
      const double lat_jerk_cost =
          CalcLatJerk(traj_pts[i], psi[i]) * lat_jerk_coeff;
      if (lat_jerk_cost > max_lat_jerk_cost) {
        max_lat_jerk_cost = lat_jerk_cost;
        max_lat_idx = i;
      }
    }
  }
  cost_vec[1] = max_lat_jerk_cost / lat_jerk_constraint_;

  // increase lat_jerk punish for split task change
  const auto lane_seq_info =
      planner_output.scheduler_output.drive_passage.lane_seq_info();
  if (lane_seq_info != nullptr && !lc_ongoing &&
      cost_input.last_selected_idx != -1 &&
      idx != cost_input.last_selected_idx) {
    const double dist_fornt = lane_seq_info->dist_to_nearest_split.second;
    const double dist_back = lane_seq_info->dist_to_nearest_split.first;
    const double dist_to_split = std::fabs(dist_back) < 10.0
                                     ? std::fabs(dist_back)
                                     : std::min(dist_fornt, kInvalidLength);
    constexpr double kMaxDistCheckSplitTaskSwitch = 30.0;  // m
    constexpr double kMinLatJerkVaild = 0.05;
    constexpr double kMaxLatJerkBase = 1.5;
    constexpr double kSplitLatJerkCostFactor = 10.0;
    const PiecewiseLinearFunction<double, double> kDecayFactor = {
        {0.0, 5.0, 10.0, 15.0, 20.0, 30.0},  // length error
        {1.0, 0.95, 0.85, 0.65, 0.3, 0.0}};  // factor
    const double split_lat_jerk_cost =
        dist_to_split > kMaxDistCheckSplitTaskSwitch ||
                std::fabs(max_lat_jerk_cost) < kMinLatJerkVaild
            ? 0.0
            : std::min(1.0, std::fabs(max_lat_jerk_cost) / kMaxLatJerkBase) *
                  kDecayFactor(dist_to_split) * kSplitLatJerkCostFactor;
    cost_vec[1] += split_lat_jerk_cost;
    extra_info->emplace_back(absl::StrFormat(
        "split lat jerk cost: %.2f, max_jerk: %.2f, "
        "max_idx: %d, dist_to_split: %.2f",
        split_lat_jerk_cost, max_lat_jerk_cost, max_lat_idx, dist_to_split));
  }

  const bool in_high_way = common_feature()->in_high_way;
  cost_vec[2] = in_high_way ? 0.0 : deacc_cost;

  //   if (max_accel_idx != -1) {
  //     extra_info->emplace_back(absl::StrFormat(
  //         "Max accel_j cost %.2f at %d",
  //         accel_jerk_cost / accel_jerk_constraint_, max_accel_idx));
  //   }
  //   if (max_decel_idx != -1) {
  //     extra_info->emplace_back(absl::StrFormat(
  //         "Max decel_j cost %.2f at %d",
  //         decel_jerk_cost / decel_jerk_constraint_, max_decel_idx));
  //   }
  //   if (max_lat_idx != -1) {
  //     extra_info->emplace_back(
  //         absl::StrFormat("Max lat_j cost %.2f at %d",
  //                         max_lat_jerk_cost / lat_jerk_constraint_,
  //                         max_lat_idx));
  //   }

  //   if (max_deacc_idx != -1) {
  //     extra_info->emplace_back(
  //         absl::StrFormat("Max deacc cost %.2f when deacc is %.2f",
  //         deacc_cost,
  //                         traj_pts[max_deacc_idx].a()));
  //   }

  cost_vec[3] = 0.0;
  std::optional<double> lat_away_start_point_s;
  std::optional<double> lat_away_end_point_s;
  double max_lat_l = 0.0;
  constexpr double kAllowCrossLineDist = 0.1;
  const double kLatAwayThreshold =
      (kDefaultLaneWidth - ego_width_) * 0.5 + kAllowCrossLineDist;
  double navi_factor = 1.2;
  const double dist_to_navi_end_for_lc =
      common_feature()->road_horizon_info.dist_to_navi_end_for_lc;
  const auto& drive_passage = planner_output.scheduler_output.drive_passage;
  if (!common_feature()->lane_feature_infos.contains(idx)) {
    return absl::NotFoundError("Invalid lane_feature_infos.");
  }
  const auto& lane_feature_info =
      FindOrDieNoPrint(common_feature()->lane_feature_infos, idx);
  for (const auto& obj : lane_feature_info.block_obj_ids) {
    traj_feature_output->traj_block_obj_ids.emplace(obj);
  }

  if (!lc_ongoing) {
    const auto& path_points = planner_output.path;
    const auto& traj_pts = planner_output.traj_points;

    for (const auto& pt : traj_pts) {
      const auto vec_pt = Vec2dFromApolloTrajectoryPointProto(pt);
      ASSIGN_OR_BREAK(const auto sl_pt,
                      drive_passage.QueryFrenetCoordinateAt(vec_pt));
      if (std::fabs(sl_pt.l) > std::fabs(max_lat_l)) {
        max_lat_l = sl_pt.l;
      }
      if (std::fabs(sl_pt.l) > kLatAwayThreshold) {
        if (!lat_away_start_point_s.has_value()) {
          lat_away_start_point_s = sl_pt.s;
        }
        lat_away_end_point_s = sl_pt.s;
      }
    }

    // Get lat away for which lc type
    const auto st_traj_mgr = cost_input.stm;
    bool lat_away_for_obstacle = false;
    if (st_traj_mgr == nullptr) {
      lat_away_for_obstacle = false;
    } else if (lat_away_start_point_s.has_value()) {
      absl::flat_hash_set<std::string> has_checked_set;
      absl::string_view lat_away_nearest_obs_id;
      double nearest_dis = DBL_MAX;
      constexpr double kObjInvadeDis = 0.3;
      for (const auto& traj : st_traj_mgr->trajectories()) {
        // Remove duplicated object id
        if (has_checked_set.contains(traj.object_id())) continue;
        has_checked_set.emplace(traj.object_id());
        if (lane_feature_info.block_obj_ids.contains(traj.object_id())) {
          continue;
        }
        if (!traj.is_stationary()) {
          continue;
        }
        ASSIGN_OR_CONTINUE(
            const auto aabbox,
            drive_passage.QueryFrenetBoxAtContour(traj.contour()));
        if (aabbox.s_max < (*lat_away_start_point_s) ||
            aabbox.s_min > (*lat_away_end_point_s)) {
          continue;
        }
        if (aabbox.l_max > (-kDefaultLaneWidth * 0.5 + kObjInvadeDis) &&
            aabbox.l_min < (kDefaultLaneWidth * 0.5 - kObjInvadeDis)) {
          lat_away_for_obstacle = true;
          if (aabbox.s_min < nearest_dis) {
            nearest_dis = aabbox.s_min;
            lat_away_nearest_obs_id = traj.object_id();
          }
          traj_feature_output->lat_away_obj_ids.emplace(traj.object_id());
        }
      }
      if (lat_away_for_obstacle) {
        extra_info->emplace_back(absl::StrFormat("lat away for nearest obs: %s",
                                                 lat_away_nearest_obs_id));
      }
    }

    if (lat_away_start_point_s.has_value() && lat_away_for_obstacle) {
      constexpr double kHighWayMinLcLengthForBackNavi = 500.0;
      constexpr double kCityMinLcLengthForBackNavi = 120.0;
      const double back_to_navi_base_length =
          in_high_way ? kHighWayMinLcLengthForBackNavi
                      : kCityMinLcLengthForBackNavi;
      double remain_distance_to_navi =
          std::fmax(0.0, dist_to_navi_end_for_lc - (*lat_away_start_point_s));
      if (drive_passage.lane_seq_info() != nullptr &&
          drive_passage.lane_seq_info()->lc_num == 0) {
        navi_factor =
            std::clamp(Sqr(remain_distance_to_navi / back_to_navi_base_length),
                       0.0, navi_factor);
      }
      const PiecewiseLinearFunction<double, double> kLatAwayCrossLineCost = {
          /*crossline_distance=*/{0.0, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5},
          /*cost=*/{0.0, 0.2, 0.4, 0.8, 1.1, 1.3, 1.5}};
      const PiecewiseLinearFunction<double, double> kLatAwayLonDisCost = {
          /*lon_distance=*/{20.0, 50.0, 80.0, 120.0, 160.0},
          /*cost=*/{1.0, 0.8, 0.6, 0.2, 0.0}};

      cost_vec[3] =
          kLatAwayLonDisCost(*lat_away_start_point_s) *
          kLatAwayCrossLineCost(std::fabs(max_lat_l) - kLatAwayThreshold) *
          navi_factor;
    }
    traj_feature_output->lane_change_for_stationary_obj |=
        (cost_vec[3] > kLatAwayObviousThreshold);
  }

  cost_vec[4] = 0.0;
  if (drive_passage.lane_seq_info() != nullptr &&
      drive_passage.lane_seq_info()->split_task_state !=
          ad_byd::planning::SplitTasksState::None &&
      common_feature()->min_split_angle.has_value()) {
    const double min_split_angle = common_feature()->min_split_angle.value();
    const double split_angle =
        drive_passage.lane_seq_info()->nearest_split_angle;
    const double angle_error = std::fabs(split_angle) - min_split_angle;
    constexpr double kAngleThreshold = 0.5 * M_PI / 180.0;
    cost_vec[4] =
        in_high_way || angle_error < kAngleThreshold
            ? 0.0
            : std::clamp(angle_error / (0.3 * min_split_angle), 0.0, 1.0);
    extra_info->emplace_back(
        absl::StrFormat("min_split_angle: %.3f, split_angle: %.3f",
                        min_split_angle, split_angle));
  }

  if (lat_away_start_point_s.has_value()) {
    extra_info->emplace_back(absl::StrFormat(
        "lat away cost: %.3f, max_lat_l: %.2f, lat_away_start_point_s: %.2f, "
        "Navi_factor: %.2f",
        cost_vec[3], max_lat_l, *lat_away_start_point_s, navi_factor));
  } else {
    extra_info->emplace_back(absl::StrFormat(
        "lat away cost: %.2f, max_lat_l: %.2f, check_lat_l: %.2f", cost_vec[3],
        max_lat_l, kLatAwayThreshold));
  }
  extra_info->emplace_back(FormatNumericString("Distance to navi end for lc: ",
                                               dist_to_navi_end_for_lc));
  extra_info->emplace_back(absl::StrCat(
      "emergency: ", btoa(traj_feature_output->lane_change_for_emergency)));
  return cost_vec;
}

// absl::StatusOr<CostVec> TrajLaneChangeCost::ComputeCost(
//     const EstPlannerOutput& planner_output,
//     std::vector<std::string>* extra_info) const {
//   CostVec cost_vec(8);
//   CHECK_NOTNULL(extra_info);

//   const auto& drive_passage = planner_output.scheduler_output.drive_passage;
//   const bool target_switched = prev_lp_from_current_->front().lane_id() !=
//                                drive_passage.lane_path().front().lane_id();
//   cost_vec[0] = btof(target_switched);

//   const auto lc_stage =
//       planner_output.scheduler_output.lane_change_state.stage();
//   const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
//                           lc_stage == LaneChangeStage::LCS_RETURN ||
//                           lc_stage == LaneChangeStage::LCS_PAUSE;
//   ASSIGN_OR_RETURN(const auto ego_sl,
//                    drive_passage.QueryFrenetCoordinateAt(ego_pos_),
//                    _ << "Ego pos not in drive passage.");
//   const double lat_offset = std::abs(ego_sl.l);
//   cost_vec[1] = lc_ongoing ? lat_offset / kDefaultLaneWidth : 0.0;

//   double avg_dist = 0.0;
//   int valid_traj_size = 0;
//   if (prev_traj_ff_or_.ok()) {
//     const auto& traj_pts = planner_output.traj_points;
//     for (const auto& pt : traj_pts) {
//       if (pt.relative_time() > kConsiderPrevTrajTime) {
//         break;
//       }
//       avg_dist += std::abs(
//           prev_traj_ff_or_->XYToSL(Vec2dFromApolloTrajectoryPointProto(pt)).l);
//       valid_traj_size++;
//     }
//     if (valid_traj_size != 0) {
//       // if valid traj size = 0, avg_dist = 0
//       avg_dist /= valid_traj_size;
//     }
//   }
//   cost_vec[2] = avg_dist / kStandardAverageTrajDiff;

//   constexpr double kLcPreviewTime = 5.0;  // s.
//   const double preview_t = (lat_offset / kDefaultLaneWidth) * kLcPreviewTime;
//   const double preview_s = ego_sl.s + std::max(ego_v_, kMinLCSpeed) *
//   preview_t; const bool ego_in_intersection =
//       common_feature()->ego_in_tl_controlled_intersection;
//   const bool preview_in_intersection =
//       ego_in_intersection ||
//       IsInTlControlledIntersection(*psmm_, drive_passage, preview_s);

//   // avoid lc in curve road
//   std::vector<double> headings;
//   std::vector<double> factors;
//   for (double accum_s = ego_sl.s - kBackCheckCurveDistance; accum_s <
//   preview_s;
//        accum_s += kCalculateCurvatureStep) {
//     const auto heading_or = drive_passage.QueryTangentAngleAtS(accum_s);
//     if (!heading_or.ok()) {
//       continue;
//     }
//     factors.push_back(2.0 - std::max(0.0, accum_s - ego_sl.s) /
//                                 (preview_s - ego_sl.s));
//     headings.push_back(*heading_or);
//   }

//   double average_curvature = 0.0;
//   if (headings.size() > 1) {
//     for (int i = 0; i < headings.size() - 1; ++i) {
//       average_curvature +=
//           std::fabs(AngleDifference(headings.at(i), headings.at(i + 1))) *
//           factors.at(i);
//     }
//     average_curvature =
//         average_curvature / (kCalculateCurvatureStep * (headings.size() -
//         1));
//   }

//   cost_vec[3] =
//       std::min(1.0, btof(lc_ongoing && !preview_in_intersection) *
//                         average_curvature * ego_v_ / kCurveSpeedConstraint);

//   const double forbidden_lc_in_intersection =
//       planner_enable_lane_change_in_intersection_ ? 1.0 :
//       kForbidBehaviorCost;
//   cost_vec[4] = btof(lc_ongoing && preview_in_intersection) *
//                 forbidden_lc_in_intersection;

//   const double length_before_intersection =
//       common_feature()->length_before_intersection;
//   const double time_since_last_red_light =
//       common_feature()->time_since_last_red_light;
//   // Need more patience when the line is long.
//   const double valid_redlight_distance =
//       std::min(length_before_intersection, kMaxRedLightDistance);
//   const double time_waiting =
//       std::max(time_since_last_red_light -
//                    valid_redlight_distance * kRedLightWaitingLeaderFactor,
//                0.0);

//   // Avoid lc in redlight with low speed
//   cost_vec[5] =
//       (lc_ongoing && !ego_in_intersection)
//           ? std::max(0.0, 1.0 - time_waiting / kRedLightWaitingTimeBase) *
//                 std::max(0.0, 1.0 - ego_v_ / kRedLightSpeedBase)
//           : 0.0;

//   const auto time_since_last_lane_change =
//       common_feature()->time_since_last_lane_change;
//   const double opposite_lc_factor = std::max(
//       0.0, 1.0 - time_since_last_lane_change / kMaxOppositeLcTimeBound);
//   cost_vec[6] =
//       btof(lc_ongoing &&
//            last_lc_info_.lc_left() !=
//                planner_output.scheduler_output.lane_change_state.lc_left()) *
//       opposite_lc_factor;

//   // Prevent too radical lane change.
//   cost_vec[7] = lc_ongoing
//                     ? std::min(1.0, Sqr(planner_output.follower_max_decel /
//                                         kLcEffectBase))
//                     : 0.0;

//   constexpr double kDblMaxDisplay = 1e6;
//   std::string follower_set_ids;
//   for (const auto& id : planner_output.follower_set) {
//     follower_set_ids.append(id + ", ");
//   }
//   extra_info->emplace_back(absl::StrFormat("Has switched target lane path:
//   %s",
//                                            btoa(target_switched)));
//   extra_info->emplace_back(
//       absl::StrFormat("Is performing lane change: %s", btoa(lc_ongoing)));
//   extra_info->emplace_back(absl::StrFormat("Ego lat offset: %.2f",
//   lat_offset)); extra_info->emplace_back(
//       absl::StrFormat("Average dist to prev traj: %.2f", avg_dist));
//   extra_info->emplace_back(absl::StrFormat("Preview in intersection: %s",
//                                            btoa(preview_in_intersection)));
//   extra_info->emplace_back(absl::StrFormat(
//       "Average curv: %.3f, Average curv with speed: %.3f, pure cost: %.3f",
//       average_curvature, average_curvature * ego_v_, cost_vec[3]));
//   extra_info->emplace_back(
//       absl::StrFormat("Enable lc in intersection: %s",
//                       btoa(planner_enable_lane_change_in_intersection_)));
//   extra_info->emplace_back(absl::StrFormat(
//       "Time since last lc: %.3f s, opposite factor: %.3f",
//       std::clamp(time_since_last_lane_change, 0.0, kDblMaxDisplay),
//       opposite_lc_factor));
//   extra_info->emplace_back(absl::StrFormat(
//       "Time since last redlight: %.2f s, waiting: %.2f",
//       std::clamp(time_since_last_red_light, 0.0, kDblMaxDisplay),
//       std::clamp(time_waiting, 0.0, kDblMaxDisplay)));
//   extra_info->emplace_back(absl::StrFormat("Follower max decel: %.3f",
//                                            planner_output.follower_max_decel));
//   extra_info->emplace_back(
//       absl::StrFormat("Follower obstacle ids: %s", follower_set_ids));
//   return cost_vec;
// }

absl::StatusOr<CostVec> TrajLaneChangeCost::ComputeCostV2(
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CostVec cost_vec(13);
  CHECK_NOTNULL(extra_info);
  const bool in_high_way =
      common_feature()->in_high_way || common_feature()->preview_in_high_way;

  const auto& drive_passage = planner_output.scheduler_output.drive_passage;
  const bool target_switched = prev_lp_from_current_->front().lane_id() !=
                               drive_passage.lane_path().front().lane_id();
  const auto lc_stage =
      planner_output.scheduler_output.lane_change_state.stage();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_RETURN ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  const bool idx_switched =
      common_feature()->lane_feature_infos.count(idx) > 0 &&
      common_feature()->lane_feature_infos.at(idx).target_switched;
  double idx_switched_cost = 0.0;
  if (idx_switched && drive_passage.lane_seq_info() != nullptr) {
    const auto lane_seq_info = drive_passage.lane_seq_info();
    if (!lc_ongoing && lane_seq_info->split_task_state !=
                           ad_byd::planning::SplitTasksState::None) {
      const double dist_to_split =
          std::max(lane_seq_info->dist_to_nearest_split.second, 0.0);
      constexpr double kMinDistIdxSwitchVaild = 50.0;  // m
      idx_switched_cost =
          1.0 - std::min(dist_to_split / kMinDistIdxSwitchVaild, 1.0);
    } else {
      idx_switched_cost = 1.0;
    }
  }
  cost_vec[0] = target_switched ? 1.0 : idx_switched_cost;

  traj_feature_output->is_perform_lane_change = lc_ongoing;
  traj_feature_output->lane_change_left =
      planner_output.scheduler_output.lane_change_state.lc_left();
  ASSIGN_OR_RETURN(const auto ego_sl,
                   drive_passage.QueryFrenetCoordinateAt(ego_pos_),
                   _ << "Ego pos not in drive passage.");
  const double lat_offset = std::abs(ego_sl.l);
  if (in_high_way && lat_offset > 0.5 * kDefaultLaneWidth) {
    cost_vec[1] = lc_ongoing ? 1.0 : 0.0;
  } else {
    cost_vec[1] = lc_ongoing ? lat_offset / kDefaultLaneWidth : 0.0;
  }

  double avg_dist = 0.0;
  int valid_traj_size = 0;
  if (prev_traj_ff_or_.ok()) {
    const auto& traj_pts = planner_output.traj_points;
    for (const auto& pt : traj_pts) {
      if (pt.relative_time() > kConsiderPrevTrajTime) {
        break;
      }
      avg_dist += std::abs(
          prev_traj_ff_or_->XYToSL(Vec2dFromApolloTrajectoryPointProto(pt)).l);
      valid_traj_size++;
    }
    if (valid_traj_size != 0) {
      // if valid traj size = 0, avg_dist = 0
      avg_dist /= valid_traj_size;
    }
  }
  cost_vec[2] = avg_dist / kStandardAverageTrajDiff;

  constexpr double kLcPreviewTime = 5.0;     // s.
  constexpr double kLcMinPreviewTime = 3.0;  // s.
  const double preview_t = std::max(
      kLcMinPreviewTime, (lat_offset / kDefaultLaneWidth) * kLcPreviewTime);
  const double preview_s = ego_sl.s + std::max(ego_v_, kMinLCSpeed) * preview_t;
  const bool ego_in_intersection =
      common_feature()->ego_in_tl_controlled_intersection;

  // Using Dis To Junction v2
  bool preview_in_intersection = ego_in_intersection;
  bool ego_close_junction = ego_in_intersection;
  if (drive_passage.lane_seq_info() != nullptr) {
    const double dist_to_intersection =
        common_feature()->road_horizon_info.dist_to_cross;
    preview_in_intersection =
        preview_in_intersection || (dist_to_intersection < preview_s);
    extra_info->emplace_back(
        FormatNumericString("dist_to_intersection: ", dist_to_intersection));
  }

  double distance_before_turn = 0.0;
  for (const auto& station : drive_passage.stations()) {
    if (station.accumulated_s() < 0) {
      continue;
    }
    if (station.is_in_intersection() ||
        (station.turn_type() != ad_byd::planning::NO_TURN)) {
      break;
    }

    distance_before_turn = station.accumulated_s();
  }

  ego_close_junction =
      (distance_before_turn < kCloseToJunctionBase && !in_high_way) ? true
                                                                    : false;
  //   IsInTlControlledIntersection(*psmm_, drive_passage, preview_s);

  // avoid lc in curve road
  std::vector<double> headings;
  std::vector<double> factors;
  for (double accum_s = ego_sl.s - kBackCheckCurveDistance; accum_s < preview_s;
       accum_s += kCalculateCurvatureStep) {
    const auto heading_or = drive_passage.QueryTangentAngleAtS(accum_s);
    if (!heading_or.ok()) {
      continue;
    }
    if (accum_s < ego_sl.s) {
      factors.push_back(2.0 - (ego_sl.s - accum_s) / kBackCheckCurveDistance);
    } else {
      factors.push_back(2.0 - (accum_s - ego_sl.s) / (preview_s - ego_sl.s));
    }
    headings.push_back(*heading_or);
  }

  double average_curvature = 0.0;
  if (headings.size() > 1) {
    for (int i = 0; i < headings.size() - 1; ++i) {
      average_curvature +=
          std::fabs(AngleDifference(headings.at(i), headings.at(i + 1))) *
          factors.at(i);
    }
    average_curvature =
        average_curvature / (kCalculateCurvatureStep * (headings.size() - 1));
  }

  if (average_curvature * ego_v_ > 0.09) {
    traj_feature_output->is_obvous_curvy_road = true;
  }
  const auto& traj_pts = planner_output.traj_points;
  double curve_factor = 1.0;
  const double ego_lane_length_along_route =
      common_feature()->ego_lane_dist_to_navi_end;
  if (common_feature()->lane_feature_infos.contains(idx)) {
    const auto& lane_feature_info =
        common_feature()->lane_feature_infos.at(idx);
    curve_factor = CalcDefaultRouteDecreaseFactor(
        ego_lane_length_along_route, ego_v_, lane_feature_info.speed_limit,
        /*lc_num_to_targets*/ 1, common_feature()->in_high_way);
  }

  cost_vec[3] =
      std::min(1.0, btof(lc_ongoing && !preview_in_intersection) *
                        average_curvature * ego_v_ / kCurveSpeedConstraint) *
      curve_factor;
  if (average_curvature < 0.005 || ego_close_junction) {
    cost_vec[3] = 0.0;
  }

  const bool pre_lc_going =
      btof(pre_lc_stage_ == LaneChangeStage::LCS_EXECUTING ||
           pre_lc_stage_ == LaneChangeStage::LCS_PAUSE ||
           pre_lc_stage_ == LaneChangeStage::LCS_RETURN);
  const double forbidden_lc_in_intersection =
      planner_enable_lane_change_in_intersection_ ? 1.0
                                                  : kPreviewJunctionCostForLC;
  const bool is_standard_junc =
      IsStandardJunction(drive_passage.lane_seq_info());
  cost_vec[4] = btof(lc_ongoing && preview_in_intersection &&
                     is_standard_junc && !pre_lc_going) *
                forbidden_lc_in_intersection;

  const double length_before_intersection = std::min(
      kInvalidLength, common_feature()->road_horizon_info.dist_to_cross);

  Log2DDS::LogDataV2(
      "lane_seq_map",
      FormatNumericString("dist_to_junction: ", length_before_intersection));

  const double time_since_last_red_light =
      common_feature()->time_since_last_red_light;
  // Need more patience when the line is long.
  const double valid_redlight_distance =
      std::min(length_before_intersection, kMaxRedLightDistance);
  const double time_waiting =
      std::max(time_since_last_red_light -
                   valid_redlight_distance * kRedLightWaitingLeaderFactor,
               0.0);

  // Avoid lc in redlight with low speed
  cost_vec[5] =
      (lc_ongoing && !ego_in_intersection)
          ? std::max(0.0, 1.0 - time_waiting / kRedLightWaitingTimeBase) *
                std::max(0.0, 1.0 - ego_v_ / kRedLightSpeedBase)
          : 0.0;

  const auto time_since_last_lane_change =
      common_feature()->time_since_last_lane_change;
  const double opposite_lc_factor = std::max(
      0.0, 1.0 - time_since_last_lane_change / kMinOppositeLcTimeBound);
  cost_vec[6] =
      btof(lc_ongoing &&
           last_lc_info_.lc_left() !=
               planner_output.scheduler_output.lane_change_state.lc_left()) *
      opposite_lc_factor;
  if (cost_vec[6] > 0.0) {
    traj_feature_output->opposite_lc_interval_secs =
        time_since_last_lane_change;
  }

  // Prevent too radical lane change.
  // cost_vec[7] = lc_ongoing
  //                 ? std::min(1.0, Sqr(planner_output.follower_max_decel /
  //                                     kLcEffectBase))
  //                 : 0.0;
  double trajectory_max_decel = 0.0;
  if (planner_output.traj_points.size() > 0) {
    for (const auto& p : planner_output.traj_points) {
      if (p.has_a() && p.a() < trajectory_max_decel) {
        trajectory_max_decel = p.a();
      }
    }
  }

  const double use_follower_max_decel = fabs(trajectory_max_decel);
  const auto lane_seq_info =
      planner_output.scheduler_output.drive_passage.lane_seq_info();
  double use_dist_to_navi_end = common_feature()->ego_lane_dist_to_navi_end;

  double decel_distance_factor = 1.0;
  constexpr double kMinDecelDecreaseHighwayDist = 800.0;
  constexpr double kMaxDecelDecreaseHighwayDist = 1000.0;
  constexpr double kMinDecelDecreaseCityDist = 200.0;
  constexpr double kMaxDecelDecreaseCityDist = 300.0;
  double kMinDecelDecreaseDist =
      in_high_way ? kMinDecelDecreaseHighwayDist : kMinDecelDecreaseCityDist;
  double kMaxDecelDecreaseDist =
      in_high_way ? kMaxDecelDecreaseHighwayDist : kMaxDecelDecreaseCityDist;
  decel_distance_factor =
      Lerp(0.0, 1.0,
           (use_dist_to_navi_end - kMinDecelDecreaseDist) /
               (kMaxDecelDecreaseDist - kMinDecelDecreaseDist));
  decel_distance_factor = std::clamp(decel_distance_factor, 0.0, 1.0);

  constexpr double kCurrentLaneDriveDecelDecreaeFactor = 0.5;
  cost_vec[7] =
      lc_ongoing ? std::min(1.0, Sqr(use_follower_max_decel / kLcEffectBase) *
                                     decel_distance_factor)
                 : std::min(1.0, Sqr(use_follower_max_decel / kLcEffectBase) *
                                     decel_distance_factor *
                                     kCurrentLaneDriveDecelDecreaeFactor);
  const bool is_split_right =
      lane_seq_info && lane_seq_info->split_task_state ==
                           ad_byd::planning::SplitTasksState::Right_Task;
  const bool is_split_left =
      lane_seq_info && lane_seq_info->split_task_state ==
                           ad_byd::planning::SplitTasksState::Left_Task;

  const bool already_turn_on_lc_signal =
      (last_turn_signal_ != TurnSignal::TURN_SIGNAL_NONE) &&
      (last_turn_signal_reason_ == TurnSignalReason::LANE_CHANGE_TURN_SIGNAL ||
       last_turn_signal_reason_ ==
           TurnSignalReason::PREPARE_LANE_CHANGE_TURN_SIGNAL);
  const bool already_turn_on_fork_left_signal =
      (last_turn_signal_ == TurnSignal::TURN_SIGNAL_LEFT) &&
      (last_turn_signal_reason_ == TurnSignalReason::FORK_TURN_SIGNAL);
  const bool already_turn_on_fork_right_signal =
      (last_turn_signal_ == TurnSignal::TURN_SIGNAL_RIGHT) &&
      (last_turn_signal_reason_ == TurnSignalReason::FORK_TURN_SIGNAL);
  const bool is_opposite_to_turn_signal =
      !lc_ongoing && (already_turn_on_lc_signal ||
                      (is_split_right && already_turn_on_fork_left_signal) ||
                      (is_split_left && already_turn_on_fork_right_signal));
  cost_vec[8] = btof(is_opposite_to_turn_signal);

  double cur_dot_pre = 0.0;
  if (!common_feature()->in_high_way && drive_passage.lane_seq_info()) {
    for (const auto& lane : drive_passage.lane_seq_info()->lane_seq->lanes()) {
      bool current_junction = lane->type() == LaneType::LANE_VIRTUAL_JUNCTION;
      bool pre_junction = false;
      if (lane && current_junction) {
        const auto& pre_lane_ids = lane->pre_lane_ids();
        for (const auto& lane_pre_id : pre_lane_ids) {
          auto lane_pre = psmm_->FindLaneByIdOrNull(lane_pre_id);
          if (lane_pre) {
            if (drive_passage.lane_seq_info()->lane_seq->IsOnLaneSequence(
                    lane_pre) &&
                lane_pre->type() != LaneType::LANE_VIRTUAL_JUNCTION) {
              pre_junction = false;
              const auto cur_lane_vec = lane->center_line().end_point() -
                                        lane->center_line().begin_point();
              const auto pre_lane_vec = lane_pre->center_line().end_point() -
                                        lane_pre->center_line().begin_point();
              if (cur_lane_vec.Length() > 1.0 && pre_lane_vec.Length() > 1.0) {
                extra_info->emplace_back(absl::StrFormat(
                    "cur_vec: %.2f , %.2f ,pre_vec: %.2f , %.2f",
                    lane->center_line().end_point().x(),
                    lane->center_line().end_point().y(),
                    lane->center_line().begin_point().x(),
                    lane->center_line().begin_point().x()));
                cur_dot_pre = cur_lane_vec.InnerProd(pre_lane_vec) /
                              (cur_lane_vec.Length() * pre_lane_vec.Length());
                break;
              }
            }
          }
        }
      }

      // for (const auto& lane :
      // drive_passage.lane_seq_info()->lane_seq->lanes()) {
      //   bool current_junction = lane->type() !=
      //   LaneType::LANE_VIRTUAL_JUNCTION; bool pre_junction = false; if (lane
      //   && current_junction) {
      //     const auto& pre_lane_ids = lane->pre_lane_ids();
      //     for (const auto& lane_pre_id : pre_lane_ids) {
      //       auto lane_pre = psmm_->FindLaneByIdOrNull(lane_pre_id);
      //       if (lane_pre) {
      //         if (drive_passage.lane_seq_info()->lane_seq->IsOnLaneSequence(
      //                 lane_pre) &&
      //             lane_pre->type() == LaneType::LANE_VIRTUAL_JUNCTION) {

      //           pre_junction = false;
      //           const auto cur_lane_vec = lane->center_line().end_point() -
      //                                     lane->center_line().begin_point();
      //           const auto pre_lane_vec = lane_pre->center_line().end_point()
      //           -
      //                                     lane_pre->center_line().begin_point();
      //           if (cur_lane_vec.Length() > 1.0 && pre_lane_vec.Length()
      //           > 1.0) {
      //             extra_info->emplace_back(absl::StrFormat(
      //                 "cur_vec: %.2f , %.2f ,pre_vec: %.2f , %.2f",
      //                 lane->center_line().end_point().x(),
      //                 lane->center_line().end_point().y(),
      //                 lane->center_line().begin_point().x(),
      //                 lane->center_line().begin_point().x()));
      //             cur_dot_pre = cur_lane_vec.InnerProd(pre_lane_vec) /
      //                           (cur_lane_vec.Length() *
      //                           pre_lane_vec.Length());
      //             break;
      //           }
      //         }
      //       }
      //     }
      //   }
      if (cur_dot_pre > 0.0) {
        break;
      }
    }
    if (cur_dot_pre > 0.0) {
      cost_vec[9] = std::clamp(
          std::sqrt(1 - std::pow(cur_dot_pre, 2)) / kJunctionSmoothnessBase,
          0.0, 1.0);
    } else {
      cost_vec[9] = 0.0;
    }
  }

  const auto time_since_last_passed_split =
      common_feature()->time_since_last_passed_split;
  const double passed_split_opposite_lc_factor =
      CalcPassedSplitTimeFactor(time_since_last_passed_split);
  const bool is_valid_passed_split_time =
      time_since_last_passed_split < kMinPassedSplitOppositeLcTimeBound;

  const bool is_valid_avoid_split_dis =
      common_feature()->is_valid_avoid_split_dis;
  bool is_passed_split_lane_change = cost_input.is_passed_split_lane_change;
  if (is_valid_passed_split_time &&
      pre_lc_stage_ == LaneChangeStage::LCS_EXECUTING) {
    is_passed_split_lane_change = true;
  } else if (cost_input.is_passed_split_lane_change &&
             !is_valid_passed_split_time) {
    is_passed_split_lane_change = false;
  }

  cost_vec[10] =
      common_feature()->in_high_way
          ? btof(target_switched) * passed_split_opposite_lc_factor *
                btof(is_valid_avoid_split_dis || is_valid_passed_split_time) *
                btof(!is_passed_split_lane_change)
          : 0.0;
  traj_feature_output->is_passed_split_lane_change =
      is_passed_split_lane_change;

  cost_vec[11] = 0.0;
  std::optional<double> lc_total_length;
  if (lc_ongoing && traj_pts.size() > 0) {
    constexpr double kLcFinishLatCheck = 0.3;
    for (const auto& pt : traj_pts) {
      const auto vec_pt = Vec2dFromApolloTrajectoryPointProto(pt);
      ASSIGN_OR_CONTINUE(const auto sl_pt,
                         drive_passage.QueryFrenetCoordinateAt(vec_pt));
      if (std::fabs(sl_pt.l) < kLcFinishLatCheck) {
        break;
      }
      lc_total_length = sl_pt.s;
    }
  }

  if (!lc_ongoing || !lc_total_length.has_value()) {
    cost_vec[11] = 0.0;
  } else {
    double min_remain_width = DBL_MAX;
    double lane_width = kDefaultLaneWidth;
    double max_static_invade_lon = 0.0;
    double lc_enable_check_s = DBL_MAX;
    bool lc_pause = (lc_stage == LaneChangeStage::LCS_PAUSE);
    if (common_feature()->lane_feature_infos.contains(idx)) {
      const auto& lane_feature_info =
          common_feature()->lane_feature_infos.at(idx);
      const auto& invade_obj_map = lane_feature_info.invade_static_obj_map;
      for (const auto& [obj_id, obj_info] : invade_obj_map) {
        if (obj_info.obj_s_max < 0.0 ||
            obj_info.obj_s_min > (*lc_total_length)) {
          continue;
        }
        if (IsConstructionObject(obj_info.obj_type)) {
          continue;
        }
        if (lc_pause && !obj_info.is_stalled) {
          continue;
        }
        double max_obj_remain_width =
            std::fmax(obj_info.left_remain_width, obj_info.right_remain_width);
        if (max_obj_remain_width > 1e-2 &&
            max_obj_remain_width < min_remain_width) {
          min_remain_width = max_obj_remain_width;
          lane_width = (obj_info.boundary_left_l - obj_info.boundary_right_l);
        }
        if (obj_info.obj_s_max > max_static_invade_lon) {
          max_static_invade_lon = obj_info.obj_s_max;
          lc_enable_check_s = obj_info.obj_s_min;
        }
      }
    }
    lane_width = std::fmax(lane_width, 1e-2);
    const PiecewiseLinearFunction<double, double> kLatSafeBufferWithSpeed = {
        /*speed=*/{Kph2Mps(30.0), Kph2Mps(40.0), Kph2Mps(50.0), Kph2Mps(60.0),
                   Kph2Mps(70.0), Kph2Mps(80.0)},
        /*buffer=*/{0.6, 0.7, 0.9, 1.1, 1.4, 1.6}};
    const double buffer_width = ego_width_ + kLatSafeBufferWithSpeed(ego_v_);
    const double lat_weight = std::sqrt(
        std::fmax(0.0, buffer_width - min_remain_width) / buffer_width);
    const double ttc = max_static_invade_lon / std::fmax(ego_v_, Kph2Mps(10.0));
    const double lon_weight =
        (ttc < 1.0) ? std::pow(ttc, 2) : std::fmin(ttc, 2.0);
    constexpr double kInvadeLcEnableDis = -1.0;
    cost_vec[11] = (lc_enable_check_s < kInvadeLcEnableDis)
                       ? 0.0
                       : (lat_weight * lon_weight);
  }
  const double dist_to_navi_end_for_lc =
      common_feature()->road_horizon_info.dist_to_navi_end_for_lc;
  constexpr double kMinLcTime = 3.0;
  const double min_navi_dist_for_lc = std::fmax(50.0, ego_v_ * kMinLcTime);
  traj_feature_output->has_obvious_invade_risk =
      (cost_vec[11] > kInvadeRiskObviousThreshold) &&
      (dist_to_navi_end_for_lc > min_navi_dist_for_lc);

  const auto& lane_feature_info =
      FindOrDieNoPrint(common_feature()->lane_feature_infos, idx);
  cost_vec[12] =
      !common_feature()->in_high_way && lc_ongoing
          ? CalcLaneChangeToBusLaneCost(lane_seq_info, lane_feature_info,
                                        common_feature()->max_navi_dist_keep)
          : 0.0;

  constexpr double kDblMaxDisplay = 1e6;
  std::string follower_set_ids;
  for (const auto& id : planner_output.follower_set) {
    follower_set_ids.append(id + ", ");
  }
  extra_info->emplace_back(
      absl::StrFormat("target_switched: %s, idx_switched: %s",
                      btoa(target_switched), btoa(idx_switched)));
  extra_info->emplace_back(
      absl::StrFormat("has_lane_changed_intersection: %s",
                      btoa(cost_input.has_lane_changed_intersection)));
  extra_info->emplace_back(
      absl::StrFormat("Performing lc: %s", btoa(lc_ongoing)));
  extra_info->emplace_back(absl::StrFormat("Ego lat offset: %.2f", lat_offset));
  extra_info->emplace_back(
      absl::StrFormat("Average dist to prev traj: %.2f", avg_dist));
  extra_info->emplace_back(absl::StrFormat("Preview in intersection: %s",
                                           btoa(preview_in_intersection)));
  extra_info->emplace_back(absl::StrFormat(
      "Average curv: %.3f, with speed: %.3f, is_obvous_curvy_road: %s,"
      "curve_factor: %.3f",
      average_curvature, average_curvature * ego_v_,
      btoa(traj_feature_output->is_obvous_curvy_road), curve_factor));
  extra_info->emplace_back(absl::StrFormat(
      "Enable lc in intersection: %s , vec angle diff %.2f",
      btoa(planner_enable_lane_change_in_intersection_), cur_dot_pre));
  extra_info->emplace_back(absl::StrFormat(
      "Time since last lc: %.3f s, opposite factor: %.3f",
      std::clamp(time_since_last_lane_change, 0.0, kDblMaxDisplay),
      opposite_lc_factor));
  extra_info->emplace_back(absl::StrFormat(
      "Time since last passed split: %.3f s, passed split opposite "
      "factor:%.3f,target switch: "
      "%s,is_valid_avoid_split_dis:%s,is_valid_passed_split_time:%s,is_passed_"
      "split_lane_change:%s",
      std::clamp(time_since_last_passed_split, 0.0, kDblMaxDisplay),
      passed_split_opposite_lc_factor, btoa(target_switched),
      btoa(is_valid_avoid_split_dis), btoa(is_valid_passed_split_time),
      btoa(is_passed_split_lane_change)));
  extra_info->emplace_back(absl::StrFormat(
      "Time since last redlight: %.2f s, waiting: %.2f",
      std::clamp(time_since_last_red_light, 0.0, kDblMaxDisplay),
      std::clamp(time_waiting, 0.0, kDblMaxDisplay)));
  // extra_info->emplace_back(absl::StrFormat("Follower max decel: %.3f",
  //                                          planner_output.follower_max_decel));
  // extra_info->emplace_back(
  //     absl::StrFormat("Is near navi end: %s", btoa(is_near_navi_end)));
  extra_info->emplace_back(
      absl::StrFormat("Follower max decel: %.3f", use_follower_max_decel));
  extra_info->emplace_back(
      absl::StrFormat("Trajectory max decel: %.3f", trajectory_max_decel));
  extra_info->emplace_back(
      absl::StrFormat("Planner output follower max decel: %.3f",
                      planner_output.follower_max_decel));
  extra_info->emplace_back(absl::StrFormat(
      "in_high_way: %s, use_dist_to_navi_end:%.1f, "
      "decel_distance_factor: %.2f",
      btoa(in_high_way), use_dist_to_navi_end, decel_distance_factor));

  extra_info->emplace_back(
      absl::StrFormat("Follower obstacle ids: %s", follower_set_ids));
  extra_info->emplace_back(absl::StrFormat(
      "Already turn on pre turn signal: %s", btoa(is_opposite_to_turn_signal)));
  extra_info->emplace_back(absl::StrFormat(
      "ego_close_junction: %s, distance_before_turn: %.3f, "
      "ego_pos_s: %.3f",
      btoa(ego_close_junction), distance_before_turn, ego_sl.s));
  return cost_vec;
}
// absl::StatusOr<CostVec> TrajCrossSolidBoundaryCost::ComputeCost(
//     const EstPlannerOutput& planner_output,
//     std::vector<std::string>* extra_info) const {
//   CHECK_NOTNULL(extra_info);

//   // 1. get variables from input
//   CostVec cost_vec(4);
//   constexpr int kCheckEveryNPt = 5;
//   constexpr double kTrajectorySExtension = 10.0;  // m.

//   const auto& traj_pts = planner_output.traj_points;
//   const auto& drive_passage = planner_output.scheduler_output.drive_passage;
//   const auto& change_stage =
//       planner_output.scheduler_output.lane_change_state.stage();

//   // Only check a first small part of trajectory on lane change pause
//   // to avoid end of trajectory cross line
//   const int check_first_n =
//       change_stage == LaneChangeStage::LCS_PAUSE
//           ? std::min<int>(CeilToInt(0.3 * traj_pts.size()) + 1,
//           traj_pts.size()) : traj_pts.size();
//   const auto last_pt_in_rear_center =
//       Vec2dFromApolloTrajectoryPointProto(traj_pts[check_first_n - 1]);
//   const auto heading = traj_pts[check_first_n - 1].path_point().theta();
//   const auto unit = Vec2d::UnitFromAngle(heading);
//   const auto last_pt_in_front_center =
//       last_pt_in_rear_center + unit * ego_front_to_ra_;

//   // 2. get solid boundary from drive passage
//   ASSIGN_OR_RETURN(const auto last_pt_in_front_center_frenet,
//                    drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
//                        last_pt_in_front_center),
//                    _ << "Last considered traj point not on drive passage.");
//   ASSIGN_OR_RETURN(const auto first_point_sl,
//                    drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
//                        Vec2dFromApolloTrajectoryPointProto(traj_pts.front())),
//                    _ << "First traj point not on drive passage.");
//   const auto solid_boundaries = FindSolidBoundaryIntervals(
//       drive_passage, first_point_sl,
//       last_pt_in_front_center_frenet.s + kTrajectorySExtension,
//       kRouteStationUnitStep);

//   // 3. generate ego car trajectory box
//   std::vector<Box2d> ego_boxes;
//   ego_boxes.reserve(
//       CeilToInt(check_first_n / static_cast<float>(kCheckEveryNPt)));
//   const double rear_to_real_center = ego_front_to_ra_ - 0.5 * ego_length_;
//   for (int i = 0; i < check_first_n; i += kCheckEveryNPt) {
//     const auto& traj_pt = traj_pts[i];
//     const double heading = traj_pt.path_point().theta();
//     Box2d ego_box(Vec2dFromApolloTrajectoryPointProto(traj_pt), heading,
//                   ego_length_, ego_width_);
//     ego_box.Shift(Vec2d::UnitFromAngle(heading) * rear_to_real_center);
//     ego_boxes.emplace_back(std::move(ego_box));
//   }
//   // For low speed condition before a stop line.
//   const Segment2d last_pt_to_ref_center_seg(
//       last_pt_in_front_center,
//       *drive_passage.QueryPointXYAtS(last_pt_in_front_center_frenet.s));
//   ego_boxes.emplace_back(last_pt_to_ref_center_seg, ego_width_);

//   const auto start_l_or =
//       drive_passage.QueryFrenetLatOffsetAt(ego_boxes.front().center());
//   const auto end_l_or =
//       drive_passage.QueryFrenetLatOffsetAt(ego_boxes.back().center());
//   const double ego_half_width = ego_width_ * 0.5;

//   // 4. calculate cost for different solid boudary type
//   const double forbidden_cross_solid_line_cost =
//       planner_enable_cross_solid_boundary_ ? 1.0 : kForbidBehaviorCost;

//   cost_vec[0] = CalculateCrossingBoundary(
//                     drive_passage, solid_boundaries, ego_boxes, change_stage,
//                     {StationBoundaryType::SOLID_WHITE,
//                      StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE,
//                      StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE},
//                     start_l_or, end_l_or, ego_half_width, *psmm_) *
//                 forbidden_cross_solid_line_cost;
//   cost_vec[1] = CalculateCrossingBoundary(
//                     drive_passage, solid_boundaries, ego_boxes, change_stage,
//                     {StationBoundaryType::SOLID_YELLOW}, start_l_or,
//                     end_l_or, ego_half_width, *psmm_) *
//                 forbidden_cross_solid_line_cost;
//   cost_vec[2] = CalculateCrossingBoundary(
//                     drive_passage, solid_boundaries, ego_boxes, change_stage,
//                     {StationBoundaryType::SOLID_DOUBLE_YELLOW}, start_l_or,
//                     end_l_or, ego_half_width, *psmm_) *
//                 forbidden_cross_solid_line_cost;
//   cost_vec[3] =
//       CalculateCrossingBoundary(drive_passage, solid_boundaries, ego_boxes,
//                                 change_stage, {StationBoundaryType::CURB},
//                                 start_l_or, end_l_or, ego_half_width,
//                                 *psmm_);

//   extra_info->emplace_back(absl::StrFormat("Solid white: %.2f",
//   cost_vec[0])); extra_info->emplace_back(absl::StrFormat("Solid yellow:
//   %.2f", cost_vec[1])); extra_info->emplace_back(
//       absl::StrFormat("Solid double yellow: %.2f", cost_vec[2]));
//   extra_info->emplace_back(absl::StrFormat("Curb: %.2f", cost_vec[3]));
//   extra_info->emplace_back(
//       absl::StrFormat("Enable lc cross solid boundary: %s",
//                       btoa(planner_enable_cross_solid_boundary_)));

//   return cost_vec;
// }
absl::StatusOr<CostVec> TrajCrossSolidBoundaryCost::ComputeCostV2(
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CHECK_NOTNULL(extra_info);

  // 1. get variables from input
  CostVec cost_vec(4);
  bool is_cross_solid = false;
  const auto& drive_passage = planner_output.scheduler_output.drive_passage;
  const bool target_switched = prev_lp_from_current_->front().lane_id() !=
                               drive_passage.lane_path().front().lane_id();
  const auto& traj_pts = planner_output.traj_points;

  const bool in_high_way = common_feature()->in_high_way;
  const auto& lc_state = planner_output.scheduler_output.lane_change_state;
  const auto& change_stage = lc_state.stage();

  const auto is_near_solid_line =
      CheckNearDisIsSolidLane(lc_state, change_stage, ego_pos_, ego_v_,
                              ego_front_to_ra_, drive_passage, in_high_way);
  bool is_near_solid_line_check =
      is_near_solid_line.ok() && *is_near_solid_line;
  bool has_overlap_solid_white = false, has_overlap = false;

  const bool target_task_switched =
      common_feature()->lane_feature_infos.count(idx) > 0 &&
      common_feature()->lane_feature_infos.at(idx).target_switched;
  auto cross_solid_or =
      IsCrossSolidByTraj(drive_passage, traj_pts, lc_state, ego_width_,
                         ego_length_, ego_front_to_ra_, target_task_switched);

  if (cross_solid_or.ok()) {
    has_overlap = cross_solid_or->first;
    has_overlap_solid_white = cross_solid_or->second;
  }

  const bool is_overtake_lc_pause =
      change_stage == st::LaneChangeStage::LCS_PAUSE &&
      cost_input.overtake_lc_pause_successive_count > 0;

  extra_info->emplace_back(absl::StrFormat(
      "overtake_lc_pause_successive_count: %d, is_overtake_lc_pause : %s",
      cost_input.overtake_lc_pause_successive_count,
      btoa(is_overtake_lc_pause)));

  cost_vec[3] = btof(has_overlap);
  is_cross_solid = (is_near_solid_line_check || has_overlap_solid_white);
  cost_vec[0] = is_cross_solid ? 1.0 : 0.0;

  if (is_cross_solid) {
    traj_feature_output->cross_solid_boundary = true;
    const auto& scheduler_output = planner_output.scheduler_output;
    const double length_along_route =
        common_feature()->ego_lane_dist_to_navi_end;
    if (common_feature()->lane_feature_infos.contains(idx)) {
      const auto& lane_feature_info =
          common_feature()->lane_feature_infos.at(idx);
      const double factor = CalcDefaultRouteDecreaseFactor(
          length_along_route, ego_v_, lane_feature_info.speed_limit,
          /*lc_num_to_targets*/ 1, common_feature()->in_high_way);
      VLOG(3) << "factor: " << factor << ", " << length_along_route << ", "
              << ego_v_ << ", " << lane_feature_info.speed_limit;
      extra_info->emplace_back(absl::StrFormat(
          "factor: %f, length_along_route: %f, "
          "lane_feature_info.speed_limit: %f,start_v:%f",
          factor, length_along_route, lane_feature_info.speed_limit, ego_v_));
      cost_vec[0] *= factor;
    }
  }

  // Ingore cross solid lane cost when near to merge
  constexpr double kIgnoreMergeDisHighWay = 80.0;  // m.
  constexpr double kIgnoreMergeDisCity = 60.0;     // m.
  if ((in_high_way &&
       common_feature()->min_dist_to_merge < kIgnoreMergeDisHighWay) ||
      (!in_high_way &&
       common_feature()->min_dist_to_merge < kIgnoreMergeDisCity))
    cost_vec[0] = 0;

  // CalculateCrossingBoundary(
  //                   drive_passage, solid_boundaries, ego_boxes,
  //                   change_stage, {StationBoundaryType::SOLID_WHITE,
  //                    StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE,

  //                    StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE},
  //                   start_l_or, end_l_or, ego_half_width, *psmm_) *
  //               forbidden_cross_solid_line_cost;
  cost_vec[1] = 0.0;
  //   CalculateCrossingBoundary(
  //                     drive_passage, solid_boundaries, ego_boxes,
  //                     change_stage, {StationBoundaryType::SOLID_YELLOW},
  //                     start_l_or, end_l_or, ego_half_width, *psmm_) *
  //                 forbidden_cross_solid_line_cost;
  cost_vec[2] = 0.0;
  //   CalculateCrossingBoundary(
  //                     drive_passage, solid_boundaries, ego_boxes,
  //                     change_stage,
  //                     {StationBoundaryType::SOLID_DOUBLE_YELLOW}, start_l_or,
  //                     end_l_or, ego_half_width, *psmm_) *
  //                 forbidden_cross_solid_line_cost;
  // cost_vec[3] = 0.0;
  //   CalculateCrossingBoundary(drive_passage, solid_boundaries, ego_boxes,
  //                             change_stage, {StationBoundaryType::CURB},
  //                             start_l_or, end_l_or, ego_half_width,
  //                             *psmm_);

  extra_info->emplace_back(
      absl::StrFormat("Enable lc cross solid boundary: %s",
                      btoa(planner_enable_cross_solid_boundary_)));
  extra_info->emplace_back(
      absl::StrFormat("target switch: %s", btoa(target_switched)));
  extra_info->emplace_back(absl::StrFormat("has_overlap_solid_white: %s",
                                           btoa(has_overlap_solid_white)));
  extra_info->emplace_back(absl::StrFormat("is_near_solid_line_check: %s",
                                           btoa(is_near_solid_line_check)));
  extra_info->emplace_back(
      absl::StrFormat("dist to merge: %.2f",
                      std::min(10000.0, common_feature()->min_dist_to_merge)));

  return cost_vec;
}

// absl::StatusOr<CostVec> TrajRouteLookAheadCost::ComputeCost(
//     const EstPlannerOutput& planner_output,
//     std::vector<std::string>* extra_info) const {
//   CHECK_NOTNULL(extra_info);

//   CostVec cost_vec(7);
//   constexpr double kEpsilon = 1.0;  // m.

//   const auto start_lane_id =
//       planner_output.scheduler_output.drive_passage.lane_path()
//           .front()
//           .lane_id();
//   const auto scheduler_hash = planner_output.scheduler_output.Hash();
//   const double len_before_merge_lane =
//       FindWithDefault(len_before_merge_lane_map_, start_lane_id, 0.0);
//   const double length_along_route =
//       FindWithDefault(len_along_route_map_, scheduler_hash, 0.0);
//   const double raw_len_along_route =
//       FindWithDefault(raw_len_along_route_map_, scheduler_hash, 0.0);
//   const auto front_stalled_obj_info =
//       FindOrDieNoPrint(front_stalled_obj_map_, scheduler_hash);
//   const double driving_dist = FindOrDie(driving_dist_map_, start_lane_id);
//   const double len_before_intersection =
//       FindWithDefault(len_before_intersection_map_, start_lane_id, 0.0);
//   const int lc_num_to_targets =
//       FindOrDie(lc_num_to_targets_map_, start_lane_id);
//   const int lc_num_within_driving_dist =
//       FindOrDie(lc_num_within_driving_dist_map_, start_lane_id);
//   const bool is_right_most_lane =
//       FindOrDie(is_right_most_lane_map_, start_lane_id);
//   const bool is_max_len_along_route =
//       length_along_route >= max_len_along_route_ - kEpsilon;
//   const bool is_min_len_along_route =
//       length_along_route <= min_len_along_route_ + kEpsilon;
//   const bool is_min_lc_num = lc_num_to_targets == min_lc_num_;
//   const bool in_high_way =
//       common_feature()->in_high_way || common_feature()->preview_in_high_way;

//   // 1. length along route cost
//   cost_vec[0] = 0.0;
//   const double length_consider_congestion =
//       length_along_route * (1 - traffic_congestion_factor_);
//   if (in_high_way) {
//     // Need to lc early in high-way.
//     if (!is_max_len_along_route && !is_min_lc_num) {
//       cost_vec[0] =
//           Sqr(1.0 - std::min(1.0, length_consider_congestion /
//                                       kLengthAlongRouteBaseForHighWay));
//     }
//   } else {
//     const double length_along_route_base = is_right_turn_
//                                                ?
//                                                kLengthAlongRouteBaseForRight
//                                                :
//                                                kLengthAlongRouteBaseForLeft;
//     if (!is_max_len_along_route && !is_min_lc_num) {
//       cost_vec[0] = Sqr(
//           1.0 - std::min(1.0, length_along_route / length_along_route_base));
//     }
//   }
//   cost_vec[0] = std::min(1.0, cost_vec[0]);

//   // 2. avoid too short driving dist.
//   cost_vec[1] = is_min_lc_num || is_max_len_along_route
//                     ? 0.0
//                     : Sqr(1.0 - std::min(1.0, driving_dist /
//                                                   kReachDestinationCutOffDist));

//   // 3. preview beyond the local map horizon for lane changes
//   // that are far away.
//   const double length_for_one_lc =
//       in_high_way ? kMinLenForLaneChangeHighWay : kMinLenForLaneChange;
//   cost_vec[2] =
//       btof(!is_min_lc_num &&
//            lc_num_within_driving_dist * length_for_one_lc > driving_dist &&
//            lc_num_within_driving_dist >= kConsiderMinLcNumToTarget);

//   // 4. discourage right most lane cost
//   const bool discourage_right_most = !planner_is_bus_model_ &&
//                                      is_right_most_lane &&
//                                      enable_discourage_right_most_cost_;
//   cost_vec[3] = btof(discourage_right_most);

//   // 5. generate prohibited stalled obj cost
//   // when there is only one single lane ,we can borrow lane in other
//   direction if (front_stalled_obj_info.has_value()) {
//     constexpr double kFollowDistance = 5.0;  // m.
//     const double distance_factor = std::clamp(
//         1.0 - (front_stalled_obj_info->stalled_obj_s - kFollowDistance) /
//                   kStalledObjectLengthBase,
//         0.0, 1.0);
//     cost_vec[4] = (is_max_len_along_route && !is_min_len_along_route)
//                       ? 0.0
//                       : front_stalled_obj_info->punish_factor *
//                       distance_factor;
//   }
//   // 6. for merge lane cost
//   const double merge_lane_length_base =
//       in_high_way ? kMergeLaneLengthBaseHighWay : kMergeLaneLengthBase;
//   cost_vec[5] =
//       Sqr(1.0 - std::min(1.0, len_before_merge_lane /
//       merge_lane_length_base));

//   // 7. encourage right most lane cost for bus mode.
//   cost_vec[6] = btof(enable_encourage_right_most_cost_ &&
//                      planner_is_bus_model_ && !is_right_most_lane);

//   // generate debug information
//   extra_info->emplace_back(
//       absl::StrFormat("Length along route: %.2f", length_along_route));
//   extra_info->emplace_back(
//       absl::StrFormat("Raw Length along route: %.2f", raw_len_along_route));
//   extra_info->emplace_back(absl::StrFormat("Length before merge lane: %.2f.",
//                                            len_before_merge_lane));
//   extra_info->emplace_back(
//       absl::StrFormat("Length along route before intersection: %.2f.",
//                       len_before_intersection));
//   extra_info->emplace_back(
//       absl::StrFormat("Target lane congestion factor %.2f, length: %.2f.",
//                       traffic_congestion_factor_,
//                       length_consider_congestion));
//   extra_info->emplace_back(
//       absl::StrFormat("Remaining Driving distance: %.2f.", driving_dist));
//   extra_info->emplace_back(
//       absl::StrFormat("Lane change num to target: %d", lc_num_to_targets));
//   extra_info->emplace_back(absl::StrFormat(
//       "Lane change num within driving dist: %d",
//       lc_num_within_driving_dist));
//   extra_info->emplace_back(absl::StrFormat(
//       "Highway: %s, Right most: %s, Discourage: %s", btoa(in_high_way),
//       btoa(is_right_most_lane), btoa(enable_discourage_right_most_cost_)));
//   extra_info->emplace_back(absl::StrFormat(
//       "Encourage right most: %s, Bus: %s",
//       btoa(enable_encourage_right_most_cost_), btoa(planner_is_bus_model_)));
//   extra_info->emplace_back(absl::StrFormat("Turn left: %s, Turn right: %s",
//                                            btoa(is_left_turn_),
//                                            btoa(is_right_turn_)));
//   if (front_stalled_obj_info.has_value()) {
//     extra_info->emplace_back(absl::StrFormat(
//         "Stalled id: %s, Factor: %.2f",
//         front_stalled_obj_info->stalled_obj_id,
//         front_stalled_obj_info->punish_factor));
//   } else {
//     extra_info->emplace_back("No front stalled object found.");
//   }
//   return cost_vec;
// }
absl::StatusOr<CostVec> TrajRouteLookAheadCost::ComputeCostV2(
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CHECK_NOTNULL(extra_info);
  constexpr int kMergeLaneCostIndex = 5;

  CostVec cost_vec(10);
  const double kEpsilon = std::max(100.0, 0.05 * max_len_along_route_);  // m.

  const auto lane_seq_info =
      planner_output.scheduler_output.drive_passage.lane_seq_info();

  const auto scheduler_hash = planner_output.scheduler_output.Hash();
  const auto start_lane_id =
      planner_output.scheduler_output.drive_passage.lane_path()
          .front()
          .lane_id();
  const auto& cutoff_info = planner_output.scheduler_output.drive_passage
                                .traffic_static_obstacles_info();

  const auto lc_stage =
      planner_output.scheduler_output.lane_change_state.stage();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_RETURN ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  const auto lc_left =
      planner_output.scheduler_output.lane_change_state.lc_left();

  const auto& lane_feature_info =
      FindOrDieNoPrint(common_feature()->lane_feature_infos, idx);
  const bool is_lc_against_navi = lane_feature_info.is_lc_against_navi;
  double len_before_merge_lane = kInvalidLength;
  double dist_to_split = kInvalidLength;
  double len_before_intersection = kInvalidLength;
  double dist_to_virtual_lane = kInvalidLength;
  double driving_dist = kInvalidLength;
  int lc_num_to_targets = 0;
  int lc_num_within_driving_dist = 0;
  bool is_right_most_lane = FindOrDie(is_right_most_lane_map_, idx);
  bool is_cur_lane_seq_belonging_navi = false;
  double raw_lane_seq_length = kInvalidLength;  // dist_to_navi_end, used when
                                                // dist_to_navi_end_v2 invalid
  int origin_lc_num_to_targets = 0;
  if (lane_seq_info) {
    driving_dist = lane_seq_info->dist_to_navi_end;
    len_before_merge_lane =
        std::min(len_before_merge_lane, lane_seq_info->dist_to_merge);
    dist_to_split =
        std::min(dist_to_split, lane_seq_info->dist_to_nearest_split.second);
    dist_to_virtual_lane =
        std::min(dist_to_virtual_lane,
                 common_feature()->road_horizon_info.dist_to_cross);
    len_before_intersection = std::max(dist_to_virtual_lane, 0.0);
    origin_lc_num_to_targets = lane_seq_info->lc_num;
    lc_num_to_targets = lane_seq_info->lc_num;
    lc_num_within_driving_dist = lane_seq_info->lc_num;
    raw_lane_seq_length =
        std::min(raw_lane_seq_length, lane_seq_info->dist_to_navi_end);
    is_cur_lane_seq_belonging_navi = lane_seq_info->lane_seq_connect_navi_end;
  }

  // process map_event, fix the lc_num_to_targets and preview
  // navi_length_along_routeS
  const auto map_event = cost_input.map_event;
  const double kMapEventEnableDistUpper = 3000.0;
  const double kMapEventEnableDistLower = 1500.0;
  const double kLcPreviewDist = 200.0;
  double lc_preview_dist = 0.0;
  auto is_in_interval = [](const double min, const double max,
                           const double value) -> bool {
    return value >= min && value <= max;
  };
  const bool control_point_merge_right =
      map_event != nullptr &&
      map_event->control_waypoint_type() ==
          byd::msg::orin::routing_map::MapEvent::COMING_RIGHT_ONTO_THE_RAMP &&
      is_in_interval(kMapEventEnableDistLower, kMapEventEnableDistUpper,
                     map_event->control_waypoint_dis());
  byd::msg::orin::routing_map::NaviActionInfo first_non_straight_action;
  if (map_event != nullptr) {
    first_non_straight_action = FindNearestNonStraightActionInfo(*map_event);
  }
  const bool navi_action_merge_right =
      map_event != nullptr &&
      first_non_straight_action.main_action() ==
          byd::msg::orin::routing_map::NMA_MERGE_RIGHT &&
      is_in_interval(kMapEventEnableDistLower, kMapEventEnableDistUpper,
                     first_non_straight_action.action_dis());
  const auto& task_id_lane_index_map = cost_input.task_id_lane_index_map;
  const int curr_lane_index =
      task_id_lane_index_map.find(idx) != task_id_lane_index_map.end()
          ? task_id_lane_index_map.at(idx)
          : -1;
  const int lane_index_median =
      static_cast<int>((cost_input.valid_lane_num + 1) / 2);
  const bool is_fix_lc_num =
      common_feature()->in_high_way &&
      common_feature()->last_selected_stage == LaneChangeStage::LCS_NONE &&
      lc_left && lc_stage == LaneChangeStage::LCS_EXECUTING &&
      (control_point_merge_right || navi_action_merge_right) &&
      cost_input.is_all_lanes_pref && curr_lane_index + 1 > lane_index_median;

  if (is_fix_lc_num) {
    lc_num_to_targets = curr_lane_index;
    lc_preview_dist = kLcPreviewDist;
  }

  const double length_along_route =
      FindWithDefault(len_along_route_map_, idx, 0.0);
  const double raw_len_along_route =
      FindWithDefault(raw_len_along_route_map_, idx, 0.0);
  const auto front_stalled_obj_info =
      FindOrDieNoPrint(front_stalled_obj_map_, idx);
  traj_feature_output->feasible_length_along_route = length_along_route;

  const bool is_max_len_along_route =
      raw_len_along_route >= max_len_along_route_ - kEpsilon;
  const bool is_min_len_along_route =
      raw_len_along_route <= min_len_along_route_ + kEpsilon;
  const bool is_min_lc_num = lc_num_to_targets == min_lc_num_;
  const bool is_curr_lane_pref = lc_num_to_targets == 0;
  const bool in_high_way =
      common_feature()->in_high_way || common_feature()->preview_in_high_way;

  double dist_to_exit_junction =
      !in_high_way && psmm_ != nullptr && psmm_->map_ptr() != nullptr &&
              psmm_->map_ptr()->route() != nullptr
          ? psmm_->map_ptr()->route()->GetDistToJunctionEndOnNavi()
          : kInvalidLength;
  if (dist_to_exit_junction > kInvalidLength) {
    dist_to_exit_junction = 0.0;
  }
  const double length_after_junction =
      std::max(raw_lane_seq_length - dist_to_exit_junction, 0.0);
  constexpr double kMaxDistanceEnterJunction = 120.0;  // m.
  const bool is_enter_junction =
      len_before_intersection < kMaxDistanceEnterJunction &&
      dist_to_exit_junction < raw_lane_seq_length;

  bool idx_switched_against_navi = false;
  if (cost_input.last_selected_idx != -1) {
    const auto& lane_info_last = FindOrDieNoPrint(
        common_feature()->lane_feature_infos, cost_input.last_selected_idx);
    int lc_num_last =
        lane_info_last.lc_num.has_value() ? lane_info_last.lc_num.value() : 0;
    idx_switched_against_navi =
        lane_feature_info.target_switched && lc_num_to_targets > lc_num_last;
  }
  //   // 1. length along route cost
  //   cost_vec[0] = 0.0;
  //   const double length_consider_congestion =
  //       length_along_route * (1 - traffic_congestion_factor_);
  //   if (in_high_way) {
  //     // Need to lc early in high-way.
  //     if (!is_max_len_along_route && !is_min_lc_num) {
  //       cost_vec[0] =
  //           Sqr(1.0 - std::min(1.0, length_consider_congestion /
  //                                       kLengthAlongRouteBaseForHighWay));
  //     }
  //   } else {
  //     const double length_along_route_base = is_right_turn_
  //                                                ?
  //                                                kLengthAlongRouteBaseForRight
  //                                                :
  //                                                kLengthAlongRouteBaseForLeft;
  //     if (!is_max_len_along_route && !is_min_lc_num) {
  //       cost_vec[0] = Sqr(
  //           1.0 - std::min(1.0, length_along_route /
  //           length_along_route_base));
  //     }
  //   }
  //   cost_vec[0] = std::min(1.0, cost_vec[0]);

  // 1. length along route cost (navigation cost, avoiding wrong route)
  cost_vec[0] = 0.0;
  double length_consider_congestion = 0.0;
  bool cost_reset_by_navigation_flag = false;
  extra_info->emplace_back(absl::StrFormat(
      "length_along_route: %.2f, max_len_along_route_: %.2f, "
      "min_len_along_route_: %.2f, is_max_len_along_route: %d, "
      "is_min_len_along_route: %d",
      length_along_route, max_len_along_route_, min_len_along_route_,
      is_max_len_along_route, is_min_len_along_route));
  extra_info->emplace_back(
      absl::StrFormat("is_min_lc_num: %d, lc_num_to_targets: %d, "
                      "min_lc_num_: %d, origin_lc_num = %d, is_fix_lc_num: %s",
                      is_min_lc_num, lc_num_to_targets, min_lc_num_,
                      origin_lc_num_to_targets, btoa(is_fix_lc_num)));
  extra_info->emplace_back(absl::StrFormat(
      "control_point_merge_right: %d, navi_action_merge_right: %d, "
      "is_all_lanes_pref: %d, lane_index_median = %d, "
      "valid_lane_num: %d,  curr_lane_index: %d",
      control_point_merge_right, navi_action_merge_right,
      cost_input.is_all_lanes_pref, lane_index_median,
      cost_input.valid_lane_num, curr_lane_index));
  auto smooth_transition = [](double lower, double upper, const double x) {
    return std::pow(Lerp(0.0, lower, 1.0, upper, x, true), 3);
  };

  if (common_feature()->in_high_way) {  // Need to lc early in high-way.
    // if (!is_min_lc_num) {
    // double dist_to_ramp = DBL_MAX;
    // if (lane_seq_info && psmm_ && psmm_->map_ptr()) {
    //   dist_to_ramp = std::fmin(psmm_->map_ptr()->v2_info().dist_to_ramp,
    //                            lane_seq_info->dist_to_navi_end);
    // }
    // Log2DDS::LogDataV0("dist_to_ramp",dist_to_ramp);
    // double navi_length_along_route =
    //     lc_num_to_targets > 0
    //         ? std::fmax(
    //               dist_to_ramp - (lc_num_to_targets - 1) *
    //               kMinLenForLaneChange, 10.0)
    //         : dist_to_ramp;

    //   const double speed = std::max(kConsiderTtcSpeedLowerBound, ego_v_);
    //   const double force_route_ttc = kForceRouteTtcForHighWay;
    //   const double begin_route_ttc = kBeginRouteTtcForHighWay;
    //   const double force_route_lc_length =
    //       std::clamp(force_route_ttc * speed, kForceRouteLengthLcForHighWay,
    //                  kBeginRouteLengthLcForHighWay);
    //   const double begin_route_lc_length =
    //       std::clamp(begin_route_ttc * speed, force_route_lc_length + 1.0,
    //                  kBeginRouteLengthLcForHighWay);
    //   if (navi_length_along_route > kRouteLengthCutOffDist) {
    //     cost_vec[0] = 0.0;
    //   } else if (navi_length_along_route > kBeginRouteLengthLcForHighWay) {
    //     cost_vec[0] = LinearInterpolate(
    //         0.0, kBeginRouteLcCostFactorForHighWay, kRouteLengthCutOffDist,
    //         kBeginRouteLengthLcForHighWay, navi_length_along_route);
    //   } else if (navi_length_along_route > begin_route_lc_length) {
    //     cost_vec[0] = kBeginRouteLcCostFactorForHighWay;
    //   } else if (navi_length_along_route > force_route_lc_length) {
    //     cost_vec[0] = LinearInterpolate(
    //         kBeginRouteLcCostFactorForHighWay,
    //         kForceRouteLcCostFactorForHighWay, begin_route_lc_length,
    //         force_route_lc_length, navi_length_along_route);
    //   } else {
    //     cost_vec[0] = kForceRouteLcCostFactorForHighWay;
    //   }
    // }

    /// length_before_intersection is used for turning in intersection cases.
    /// driving_dist is used for turning in ramp cases.

    double dist_to_navi_end = DBL_MAX;
    if (lane_seq_info && psmm_ && psmm_->map_ptr()) {
      // dist_to_navi_end =
      // std::fmin(psmm_->map_ptr()->v2_info().dist_to_ramp,
      //                          lane_seq_info->dist_to_navi_end);
      dist_to_navi_end = lane_seq_info->dist_to_navi_end;
    }
    Log2DDS::LogDataV0("dist_to_navi_end", dist_to_navi_end);
    double navi_length_along_route =
        lc_num_to_targets > 0 ? std::fmax(dist_to_navi_end -
                                              (lc_num_to_targets - 1) *
                                                  kMinLenForLaneChangeHighWay -
                                              lc_preview_dist,
                                          10.0)
                              : dist_to_navi_end;
    auto calculate_validity = [&smooth_transition](double dist_to_navi_end,
                                                   int lc_num_to_targets) {
      double validity = 0.0;
      if (lc_num_to_targets <= 1) {
        validity = 1.0 - smooth_transition(1500, 1700, dist_to_navi_end);
      } else if (lc_num_to_targets == 2) {
        validity = 1.0 - smooth_transition(1900, 2150, dist_to_navi_end);
      } else if (lc_num_to_targets > 2) {
        validity = 1.0 - smooth_transition(2400, 2700, dist_to_navi_end);
      }
      return validity;
    };
    const double validity =
        calculate_validity(dist_to_navi_end, lc_num_to_targets);
    cost_vec[0] =
        !(is_min_lc_num && is_max_len_along_route) && validity > 0.0
            ? Sqr(1.0 - std::min(1.0, navi_length_along_route /
                                          kLengthAlongRouteBaseForHighWay)) *
                  validity
            : 0.0;
  } else {
    raw_lane_seq_length =
        common_feature()->mapless ? driving_dist : raw_lane_seq_length;
    double navi_length_along_route = raw_lane_seq_length;
    if (lc_num_to_targets == 2) {
      navi_length_along_route =
          std::fmax(raw_lane_seq_length - kMinLenForLaneChange, 10.0);
    } else if (lc_num_to_targets == 3) {
      navi_length_along_route =
          std::fmax(raw_lane_seq_length - kMinLenForTwiceLaneChangeCity, 10.0);
    } else if (lc_num_to_targets > 3) {
      navi_length_along_route = std::fmax(
          raw_lane_seq_length - (lc_num_to_targets - 2) * kMinLenForLaneChange,
          10.0);
    }

    // minus the length of traffic jam
    bool fixed_navi_length_along_route = false;
    const double dist_to_start_traffic_jam = static_cast<double>(
        common_feature()->traffic_jam_info.dist_to_start_traffic_jam());
    double traffic_jam_length =
        static_cast<double>(
            common_feature()->traffic_jam_info.traffic_jam_dist()) *
        TrafficJamSmoothFactor(100, 200, dist_to_start_traffic_jam);
    traffic_jam_length = std::fmin(kMaxTrafficJamLength, traffic_jam_length);
    const bool in_traffic_jam = IsInTrafficJamArea(*common_feature());
    if (ShouldFixNaviLengthAlongRoute(in_traffic_jam,
                                      cost_input.is_ego_on_pref_lane,
                                      lc_ongoing, is_curr_lane_pref)) {
      fixed_navi_length_along_route = true;
    }

    const double length_along_route_base = is_right_turn_
                                               ? kLengthAlongRouteBaseForRight
                                               : kLengthAlongRouteBaseForLeft;
    // cal preview dist, route change early
    navi_length_along_route = CalcuNaviLengthAfterRoutePreiew(
        cost_input, idx, lc_ongoing, len_before_intersection, ego_v_,
        navi_length_along_route);

    if (is_enter_junction) {
      constexpr double kLengthAfterJunctionForLC1 = 100.0;
      constexpr double kLengthAfterJunctionForLC2 = 150.0;
      constexpr double kLengthAfterJunctionForLC3 = 200.0;
      double len_threshlod_after_junction = 1000.0;
      if (lc_num_to_targets <= 1) {
        len_threshlod_after_junction = kLengthAfterJunctionForLC1;
      } else if (lc_num_to_targets == 2) {
        len_threshlod_after_junction = kLengthAfterJunctionForLC2;
      } else if (lc_num_to_targets > 2) {
        len_threshlod_after_junction = kLengthAfterJunctionForLC3;
      }
      const PiecewiseLinearFunction<double, double> kDampingFactor = {
          {0.0, 20.0, 50.0, 80.0, 100.0, 120.0},  // length error
          {1.0, 0.5, 0.2, 0.1, 0.0, 0.0}};        // factor
      const double decrease_factor =
          (is_lc_against_navi || idx_switched_against_navi ||
           length_after_junction < len_threshlod_after_junction)
              ? 1.0
              : kDampingFactor(length_after_junction -
                               len_threshlod_after_junction);
      cost_vec[0] =
          is_min_lc_num && is_max_len_along_route
              ? 0.0
              : Sqr(1.0 - std::min(1.0, navi_length_along_route /
                                            length_along_route_base)) *
                    decrease_factor;
    } else {
      auto calculate_validity = [&smooth_transition](double raw_lane_seq_length,
                                                     int lc_num_to_targets) {
        double validity = 0.0;
        if (lc_num_to_targets <= 1) {
          validity = 1.0 - smooth_transition(400, 500, raw_lane_seq_length);
        } else if (lc_num_to_targets == 2) {
          validity = 1.0 - smooth_transition(900, 1050, raw_lane_seq_length);
        } else if (lc_num_to_targets > 2) {
          validity = 1.0 - smooth_transition(1100, 1300, raw_lane_seq_length);
        }
        return validity;
      };
      const double validity =
          fixed_navi_length_along_route
              ? 1.0
              : calculate_validity(raw_lane_seq_length, lc_num_to_targets);
      if (fixed_navi_length_along_route) {
        navi_length_along_route =
            std::fmax(navi_length_along_route - traffic_jam_length, 10.0);
      }
      cost_vec[0] =
          !(is_min_lc_num && is_max_len_along_route) && validity > 0.0
              ? Sqr(1.0 - std::min(1.0, navi_length_along_route /
                                            length_along_route_base)) *
                    validity
              : 0.0;
    }
    if (cost_vec[0] > 0.9 && lc_num_to_targets > 1) {
      constexpr double kMaxFactor = 0.6;
      constexpr double kStepFactor = 0.15;
      const double factor_increase =
          1.0 + std::min((lc_num_to_targets - 1) * kStepFactor, kMaxFactor);
      cost_vec[0] *= factor_increase;
    }
    extra_info->emplace_back(absl::StrFormat(
        "length_along_route_base: %.2f, length_after_junction: %.2f, "
        "dist_to_virtual: %.2f, dist_to_exit: %.2f",
        length_along_route_base, length_after_junction, dist_to_virtual_lane,
        dist_to_exit_junction));
    extra_info->emplace_back(
        absl::StrFormat("fixed_navi_length_along_route: %s",
                        btoa(fixed_navi_length_along_route)));
  }
  auto speed_limit = std::max(lane_feature_info.speed_limit, ego_v_);
  constexpr double kMaxDecel = 0.9;            // m/s^2
  constexpr double kLowSpeed = Kph2Mps(30.0);  // m/s.
  if (ego_v_ < kLowSpeed || 2 * ego_v_ < lane_feature_info.speed_limit) {
    speed_limit = std::max(ego_v_, kLowSpeed);
  }

  bool has_curb_cutoff =
      (cutoff_info.enable_stop || cutoff_info.enable_slow_down) &&
      (cutoff_info.block_reason == BlockReason::CURB_CROSS);
  bool has_cross_curb = FindOrDie(has_cross_curb_map_, idx);
  if (has_curb_cutoff ||
      (!front_stalled_obj_info.has_value() && has_cross_curb)) {
    constexpr double kMinLengthForLcBack = 20.0;
    const double dist_to_navi_end_for_lc =
        common_feature()->road_horizon_info.dist_to_navi_end_for_lc;
    double cutoff_along_route =
        has_curb_cutoff ? cutoff_info.stop_s : length_along_route;
    const double remain_length_for_lc =
        (dist_to_navi_end_for_lc - cutoff_along_route);
    if (remain_length_for_lc > kMinLengthForLcBack) {
      constexpr double kFollowDistance = 5.0;  // m.
      const auto length_base =
          std::fmax(kShortLengthAlongRouteBase,
                    0.5 * speed_limit * speed_limit / kMaxDecel);
      double short_length_cost = std::clamp(
          1.0 - Sqr((cutoff_along_route - kFollowDistance) / length_base), 0.0,
          1.0);
      cost_vec[0] = std::fmax(short_length_cost, cost_vec[0]);
      if (short_length_cost > kLengthCutOffObviousThreshold) {
        traj_feature_output->has_obvious_cutoff_cost = true;
        traj_feature_output->lane_change_for_length_cutoff = true;
        extra_info->emplace_back(absl::StrFormat(
            "has_obvious_cutoff_cost, value is: %.2f", short_length_cost));
      }
    }
  }
  /*
  useful when task 0 fails
  desired cases: ego corss solid line, task 1 (return to current lane) and
                  fallback task (continue lane changing) exist
  desired: choose task 1
  With map existing, reset the navigation clc_numost to zero if current junction
  is drivable
  */
  // if (func_id_ == st::Behavior_FunctionId_CITY_NOA && lane_seq_info &&
  //     lane_seq_info->lane_seq && lc_num_to_targets <= 1 && !is_min_lc_num) {
  //   double junc_dist_from_func = 0.0;
  //   bool cur_junc_drivable = false;
  //   if (navi_start_.section_id != "Invalid") {
  //     cur_junc_drivable = lane_seq_info->lane_seq->CanPassJunction(
  //         navi_start_, &junc_dist_from_func, false);
  //   }
  //   if (cur_junc_drivable && (std::abs(lane_seq_info->dist_to_junction -
  //                                      junc_dist_from_func) <= 10.0)) {
  //     cost_vec[0] = 0.0;
  //     cost_reset_by_navigation_flag = true;
  //   }
  // }

  // 2. avoid too short driving dist.
  // cost_vec[1] = 0.0;
  // if (!is_max_len_along_route &&
  //     (len_before_intersection < 350.0 || driving_dist < 350.0)) {
  //   cost_vec[1] = Sqr(
  //       1.0 - std::min(1.0, length_along_route /
  //       kReachDestinationCutOffDist));
  // }
  cost_vec[1] =
      is_min_lc_num && is_max_len_along_route
          ? 0.0
          : 1.0 -
                Sqr(std::min(1.0, driving_dist / kReachDestinationCutOffDist));

  //   // 2. avoid too short driving dist.
  //   cost_vec[1] = is_min_lc_num || is_max_len_along_route
  //                     ? 0.0
  //                     : Sqr(1.0 - std::min(1.0, driving_dist /
  //                                                   kReachDestinationCutOffDist));

  // 3. preview beyond the local map horizon for lane changes
  // that are far away. (navi cost)
  const double preview_dist_threshold =
      GetPreviewDistanceThreshold(lc_num_within_driving_dist, in_high_way);
  cost_vec[2] =
      in_high_way
          ? btof(!is_min_lc_num && preview_dist_threshold > driving_dist &&
                 lc_num_within_driving_dist >= kConsiderMinLcNumToTarget)
          : 0.0;

  // 4. discourage right most lane cost
  const bool discourage_right_most =
      CalculateDiscourageRightMost(planner_is_bus_model_, is_right_most_lane,
                                   enable_discourage_right_most_cost_);
  double discourage_right_most_factor = CalcDiscourageRightCost(ego_v_);
  double discourage_dist_factor =
      CalculateDiscourageDistFactor(in_high_way, dist_to_virtual_lane);
  double straight_preview_factor =
      CalculateStraightPreviewfactor(in_high_way, common_feature());
  const double highway_factor = in_high_way ? 1.5 : 1.0;
  cost_vec[3] = btof(discourage_right_most) * discourage_right_most_factor *
                discourage_dist_factor * straight_preview_factor *
                highway_factor;
  extra_info->emplace_back(
      absl::StrFormat("discourage_right_most: %s, discourage_right_most_factor:"
                      "%.2f, straight_preview_factor: "
                      "%.2f, highway_factor: %.2f",
                      btoa(discourage_right_most), discourage_right_most_factor,
                      straight_preview_factor, highway_factor));
  // 5. generate prohibited stalled obj cost
  // when there is only one single lane ,we can borrow lane in other direction
  bool has_obs_cutoff =
      (cutoff_info.enable_stop || cutoff_info.enable_slow_down) &&
      (cutoff_info.block_reason == BlockReason::OBS_CENTER_OCCUPATION ||
       cutoff_info.block_reason == BlockReason::OBS_CENTER_TOO_NEAR ||
       cutoff_info.block_reason == BlockReason::OBS_CENTER_SINGLE ||
       cutoff_info.block_reason == BlockReason::OBS_NEIGHBOR_TREND ||
       cutoff_info.block_reason == BlockReason::OBS_NEIGHBOR_TRIANGLE ||
       cutoff_info.block_reason == BlockReason::OBS_CENTER_CURB ||
       cutoff_info.block_reason == BlockReason::OBS_BOUNDARY_CURB ||
       cutoff_info.block_reason == BlockReason::OBS_ROW_OBSTACLES);

  if (has_obs_cutoff || front_stalled_obj_info.has_value()) {
    // The cost should match with STOPLINE, route look ahead cost
    double cutoff_obj_s = has_obs_cutoff
                              ? cutoff_info.stop_s
                              : front_stalled_obj_info->stalled_obj_s;
    double punish_factor =
        has_obs_cutoff ? 1.0 : front_stalled_obj_info->punish_factor;
    constexpr double kFollowDistance = 5.0;  // m.
    constexpr double kIntersectionFollowDistance = 11.0;
    const double dist_to_intersection =
        common_feature()->road_horizon_info.dist_to_cross;
    if (dist_to_intersection > 1e-2 &&
        dist_to_intersection < cutoff_obj_s - kIntersectionFollowDistance) {
      punish_factor = 0.0;
    }
    const auto length_base =
        std::max((in_high_way ? kStalledObjectLengthBaseHighWay
                              : kStalledObjectLengthBase),
                 0.5 * speed_limit * speed_limit / kMaxDecel);
    double distance_factor =
        1.0 - Sqr((cutoff_obj_s - kFollowDistance) / length_base);
    distance_factor = std::clamp(distance_factor, 0.0, 1.0);
    traj_feature_output->has_obvious_stalled_object = true;
    cost_vec[4] = punish_factor * distance_factor;
    traj_feature_output->lane_change_for_stalled_vehicle =
        (cost_vec[4] > kBehindStalledObjObviousThreshold);
  }

  // 6. for merge lane cost
  bool is_max_merge_lane =
      max_length_before_merge_lane_ - len_before_merge_lane <
      std::min(0.05 * max_length_before_merge_lane_, 30.0);
  bool is_valid_merge_lenth = len_before_merge_lane < raw_lane_seq_length;
  if (is_max_merge_lane || !is_valid_merge_lenth) {
    cost_vec[kMergeLaneCostIndex] = 0.0;
  } else {
    cost_vec[kMergeLaneCostIndex] = CalceMergeLaneCost(MergeLaneCostInput{
        .on_highway = in_high_way,
        .dist_to_merge = len_before_merge_lane,
        .lane_seq_info = lane_seq_info,
        .dist_to_split = dist_to_split < kInvalidLength
                             ? std::make_optional(dist_to_split)
                             : std::nullopt,
        .dist_to_exit_junction = dist_to_exit_junction < kInvalidLength
                                     ? std::make_optional(dist_to_exit_junction)
                                     : std::nullopt,
    });
  }
  // 7. encourage right most lane cost for bus mode.
  cost_vec[6] = btof(enable_encourage_right_most_cost_ &&
                     planner_is_bus_model_ && !is_right_most_lane);

  // 8. avoid cones
  cost_vec[7] = 0.0;
  int left_cones = 0;
  int right_cones = 0;
  int blocking_cones = 0;
  std::string left_invade_cone_id = "Invalid";
  std::string right_invade_cone_id = "Invalid";
  int left_cones_invade_count = 0;
  int right_cones_invade_count = 0;
  int left_attribute_satisfied_count = 0;
  int right_attribute_satisfied_count = 0;
  int lc_left_direction_cones_count = 0;
  int lc_right_direction_cones_count = 0;

  double last_selected_length_along_route = length_along_route;
  for (const auto& [index, lane_feature] :
       common_feature()->lane_feature_infos) {
    if (lane_feature.target_switched) {
      continue;
    }
    last_selected_length_along_route =
        FindWithDefault(raw_len_along_route_map_, index, length_along_route);
    break;
  }
  double cones_decrease_factor =
      is_min_lc_num ? CalcDefaultRouteDecreaseFactor(
                          last_selected_length_along_route, ego_v_,
                          lane_feature_info.speed_limit, lc_num_to_targets,
                          common_feature()->in_high_way)
                    : 1.0;
  constexpr int kMinNeighborConesCount = 3;
  left_cones = lane_feature_info.lane_cones_info.left_cones_count;
  right_cones = lane_feature_info.lane_cones_info.right_cones_count;
  blocking_cones = lane_feature_info.lane_cones_info.blocking_cones_count;
  left_invade_cone_id = lane_feature_info.lane_cones_info.left_invade_cone_id;
  right_invade_cone_id = lane_feature_info.lane_cones_info.right_invade_cone_id;
  left_cones_invade_count =
      lane_feature_info.lane_cones_info.left_cones_invade_count;
  right_cones_invade_count =
      lane_feature_info.lane_cones_info.right_cones_invade_count;
  left_attribute_satisfied_count =
      lane_feature_info.lane_cones_info.left_attribute_satisfied_count;
  right_attribute_satisfied_count =
      lane_feature_info.lane_cones_info.right_attribute_satisfied_count;
  lc_left_direction_cones_count =
      lane_feature_info.lane_cones_info.lc_left_direction_cones_count;
  lc_right_direction_cones_count =
      lane_feature_info.lane_cones_info.lc_right_direction_cones_count;

  if (!lc_ongoing) {
    if (left_cones >= kMinNeighborConesCount ||
        right_cones >= kMinNeighborConesCount) {
      cost_vec[7] = std::clamp(0.3 * (std::max(left_cones, right_cones) -
                                      kMinNeighborConesCount + 1),
                               0.0, 0.65);
    }
  } else {
    if (lc_left) {
      if (lc_left_direction_cones_count > 0 || right_cones_invade_count > 0) {
        cost_vec[7] = std::clamp(0.3 * (std::max(lc_left_direction_cones_count,
                                                 right_cones_invade_count)),
                                 0.0, 0.65);
      }
    } else {
      if (left_cones_invade_count > 0 || lc_right_direction_cones_count > 0) {
        cost_vec[7] =
            std::clamp(0.3 * (std::max(left_cones_invade_count,
                                       lc_right_direction_cones_count)),
                       0.0, 0.65);
      }
    }
  }
  constexpr int kBlockingConseCount = 2;
  if (blocking_cones > kBlockingConseCount) {
    cost_vec[7] += 0.0;
  } else if (blocking_cones == kBlockingConseCount) {
    cost_vec[7] += 0.0;
  } else if (blocking_cones > 0) {
    cost_vec[7] += 0.0;
  }
  constexpr double kMaxJuncDistForInhbConesCost = 120.0;  // m.
  const bool is_inhibitConesCost =
      len_before_intersection < kMaxJuncDistForInhbConesCost;
  cost_vec[7] =
      in_high_way ? std::min(cost_vec[7], 1.0) * cones_decrease_factor : 0.0;

  cost_vec[8] = !in_high_way
                    ? CalcRouteBusLaneCost(lane_seq_info, lane_feature_info,
                                           common_feature()->max_navi_dist_keep,
                                           extra_info)
                    : 0.0;

  constexpr double kEnableAvoidMergedAreaTimeThreshold = 20.0;  // s
  cost_vec[9] = in_high_way && common_feature()->time_since_ego_leave_ramp >
                                   kEnableAvoidMergedAreaTimeThreshold
                    ? CalceMergedAreaCost(lane_seq_info, extra_info)
                    : 0.0;

  if (cost_vec[0] > kLengthAlongRouteObviousThreshold ||
      cost_vec[4] > kBehindStalledObjObviousThreshold ||
      cost_vec[kMergeLaneCostIndex] > kLengthBeforeMergeLaneObviousThreshold ||
      cost_vec[7] > kAvoidConesObviousThreshold
      /* || cost_vec[8] > kBeginLcForExitLaneThreshold*/) {
    traj_feature_output->has_obvious_route_cost = true;
  }

  if (cost_vec[8] > kLengthAlongRouteObviousThreshold) {
    traj_feature_output->lane_change_for_avoid_bus_lane = true;
  }

  if (cost_vec[9] > kLengthAlongMergedAreaThreshold) {
    traj_feature_output->lane_change_for_avoid_merge_area = true;
  }

  // Generate lane change reason.
  if (cost_vec[7] >= kBeginAvoidConesChangeTheshold) {
    traj_feature_output->lane_change_for_avoid_cones = true;
  }
  if (cost_vec[3] > kBeginLcForDiscourageRightMostThreshold) {
    traj_feature_output->lane_change_for_right_most_lane = true;
  }

  double route_change_continuity_factor = 1.0;
  if (cost_input.last_selected_idx == idx &&
      cost_input.begin_route_change_left.has_value()) {
    route_change_continuity_factor = 0.5;
  }

  if (cost_vec[0] >
      kBeginLcLengthAlongRouteThreshold * route_change_continuity_factor) {
    // Ignore short length along route caused by front stalled object.
    if (!front_stalled_obj_info.has_value()) {
      traj_feature_output->lane_change_for_navi_cost = true;
    }
  }
  if (cost_vec[2] > kBeginLcForPreviewThreshold) {
    traj_feature_output->lane_change_for_route_cost = true;
  }
  if (cost_vec[kMergeLaneCostIndex] >
      (in_high_way ? kBeginLcForMergeLaneThresholdHighway
                   : kBeginLcForMergeLaneThreshold) *
          route_change_continuity_factor) {
    traj_feature_output->lane_change_for_merge_lane = true;
  }
  traj_feature_output->has_begin_route_change =
      traj_feature_output->lane_change_for_navi_cost ||
      traj_feature_output->lane_change_for_merge_lane;

  // generate debug information
  extra_info->emplace_back(
      absl::StrFormat("last_selected_idx: %d", cost_input.last_selected_idx));
  extra_info->emplace_back(
      absl::StrFormat("has_begin_route_change: %d",
                      cost_input.begin_route_change_left.has_value()));
  extra_info->emplace_back(absl::StrFormat(
      "route_change_continuity_factor: %.2f", route_change_continuity_factor));
  extra_info->emplace_back(
      absl::StrFormat("has_obvious_route_cost: %s",
                      btoa(traj_feature_output->has_obvious_route_cost)));
  extra_info->emplace_back(
      absl::StrFormat("length_along_route: %.2f, raw: %.2f", length_along_route,
                      raw_len_along_route));
  extra_info->emplace_back(
      absl::StrFormat("dist_to_merge: %.2f, Len before intersection: %.2f.",
                      len_before_merge_lane, len_before_intersection));
  extra_info->emplace_back(
      absl::StrFormat("traffic_congestion_factor: %.2f, length: %.2f.",
                      traffic_congestion_factor_, length_consider_congestion));
  extra_info->emplace_back(absl::StrFormat(
      "Remain drive dist: %.2f, dist_to_navi_end (lane_level):%.2f.",
      driving_dist, raw_lane_seq_length));
  extra_info->emplace_back(
      absl::StrFormat("Lc num to target: %d", lc_num_to_targets));
  extra_info->emplace_back(absl::StrFormat("Lc num within driving dist: %d",
                                           lc_num_within_driving_dist));
  extra_info->emplace_back(absl::StrFormat(
      "Highway: %s, Right most: %s, Discourage: %s", btoa(in_high_way),
      btoa(is_right_most_lane), btoa(enable_discourage_right_most_cost_)));
  extra_info->emplace_back(absl::StrFormat(
      "Encourage right most: %s, Bus: %s",
      btoa(enable_encourage_right_most_cost_), btoa(planner_is_bus_model_)));
  extra_info->emplace_back(absl::StrFormat("Turn left: %s, Turn right: %s",
                                           btoa(is_left_turn_),
                                           btoa(is_right_turn_)));
  if (has_obs_cutoff) {
    extra_info->emplace_back(
        absl::StrFormat("Has obs cutoff at s: %.2f", cutoff_info.stop_s));
  }
  if (front_stalled_obj_info.has_value()) {
    extra_info->emplace_back(
        absl::StrFormat("Stalled id: %s, Factor: %.2f, Stalled_obj_s: %.2f",
                        front_stalled_obj_info->stalled_obj_id,
                        front_stalled_obj_info->punish_factor,
                        front_stalled_obj_info->stalled_obj_s));
  } else {
    extra_info->emplace_back("No front stalled obj.");
  }

  extra_info->emplace_back(
      absl::StrFormat("Borrow case: %s", btoa(is_borrow_case_)));
  extra_info->emplace_back(
      absl::StrFormat("Navi lane sequence: %s, navigation cost reset:%s",
                      btoa(is_cur_lane_seq_belonging_navi),
                      btoa(cost_reset_by_navigation_flag)));

  extra_info->emplace_back(absl::StrFormat(
      "Navi start section id: %s, navi start s offset:%.2f, function id:%d",
      navi_start_.section_id, navi_start_.s_offset, func_id_));
  extra_info->emplace_back(absl::StrFormat(
      "Left_Cones:(left: %d, left_invade_cone_id: %s, left_invade_counts:%d, "
      "left_attribute_satisfied_count:%d, lc_left_direction_cones_count:%d)",
      left_cones, left_invade_cone_id, left_cones_invade_count,
      left_attribute_satisfied_count, lc_left_direction_cones_count));

  extra_info->emplace_back(absl::StrFormat(
      "Right_Cones:(right: %d, right_invade_cone_id:%s, "
      "right_invade_counts:%d, "
      "right_attribute_satisfied_count:%d,lc_right_direction_cones_count:%d)",
      right_cones, right_invade_cone_id, right_cones_invade_count,
      right_attribute_satisfied_count, lc_right_direction_cones_count));

  extra_info->emplace_back(
      absl::StrFormat("Cones:(blocking: %d, decrease_factor: %.2f)",
                      blocking_cones, cones_decrease_factor));
  return cost_vec;
}

absl::StatusOr<CostVec> TrajBoundaryExpansionCost::ComputeCostV2(
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CostVec cost_vec(1);
  CHECK_NOTNULL(extra_info);

  ASSIGN_OR_RETURN(
      const double ego_l,
      planner_output.scheduler_output.drive_passage.QueryFrenetLatOffsetAt(
          ego_pos_),
      _ << "Plan start point is not on drive passage!");

  const auto& path_boundary = planner_output.scheduler_output.sl_boundary;
  const auto& left_l_vec = path_boundary.target_left_l_vector();
  const auto& right_l_vec = path_boundary.target_right_l_vector();
  const double left_offset =
      std::accumulate(left_l_vec.begin(), left_l_vec.end(), 0.0) /
      path_boundary.size();
  const double right_offset =
      std::accumulate(right_l_vec.begin(), right_l_vec.end(), 0.0) /
      path_boundary.size();

  const double dist = left_offset > -right_offset
                          ? std::abs(left_offset - ego_l)
                          : std::abs(ego_l - right_offset);

  constexpr double kDefaultOneAndHalfLaneWidth = 1.5 * kDefaultLaneWidth;
  cost_vec[0] = dist / kDefaultOneAndHalfLaneWidth;

  extra_info->emplace_back(absl::StrFormat("left: %.2f", left_offset));
  extra_info->emplace_back(absl::StrFormat("right: %.2f", -right_offset));

  return cost_vec;
}

absl::StatusOr<CostVec> PrepareIntentionCost::ComputeCostV2(
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CHECK_NOTNULL(extra_info);
  CostVec cost_vec(1);
  const auto lc_stage =
      planner_output.scheduler_output.lane_change_state.stage();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_RETURN ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  const auto lc_left =
      planner_output.scheduler_output.lane_change_state.lc_left();
  // for the convenience of code implementation, it is set to a negative value
  constexpr int kRouteChangeThresold = 20;
  if (traj_feature_output->cur_frame_has_begin_route_change && lc_ongoing &&
      ((cost_input.begin_route_change_left.has_value() &&
        lc_left == cost_input.begin_route_change_left) ||
       (cost_input.is_going_force_route_change_left.has_value() &&
        lc_left == cost_input.is_going_force_route_change_left))) {
    int max_route_change_successive_count =
        std::max(cost_input.begin_route_change_successive_count,
                 cost_input.force_route_change_successive_count);
    cost_vec[0] = std::max(
        -1.0 * max_route_change_successive_count / kRouteChangeThresold, -1.0);
    traj_feature_output->begin_route_change_successive_count =
        cost_input.begin_route_change_successive_count;
    traj_feature_output->force_route_change_successive_count =
        cost_input.force_route_change_successive_count;
  }

  extra_info->emplace_back(
      absl::StrFormat("last_has_begin_route_change: %d",
                      cost_input.begin_route_change_left.has_value()
                          ? cost_input.begin_route_change_left.value()
                          : -1));
  extra_info->emplace_back(
      absl::StrFormat("begin_route_change_successive_count: %d",
                      cost_input.begin_route_change_successive_count));

  extra_info->emplace_back(
      absl::StrFormat("force_route_change_successive_count: %d",
                      cost_input.force_route_change_successive_count));
  return cost_vec;
}

absl::StatusOr<CostVec> DefensiveDrivingCost::ComputeCostV2(
    const EstPlannerOutput& planner_output, const int idx,
    const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CHECK_NOTNULL(extra_info);
  CostVec cost_vec(1);
  if (!common_feature()->lane_feature_infos.contains(idx)) {
    return absl::NotFoundError("Invalid lane_feature_infos.");
  }
  const auto& lane_feature_info =
      FindOrDieNoPrint(common_feature()->lane_feature_infos, idx);
  const bool is_in_high_way = common_feature()->in_high_way;
  const double length_along_route = common_feature()->ego_lane_dist_to_navi_end;
  const double force_lc_dist =
      is_in_high_way ? kForceRouteLengthLcForHighWay
                     : kForceRouteLengthLcForHighWay * 0.25;  // m.
  const double deltaSpd = lane_feature_info.nearest_leader->obj_v - ego_v_;
  const double safety_efficient_lc_dist = std::max(
      std::max(ego_v_ * kDefensiveTruckTHWThrd, kMinDefensiveBehindTrunkDist),
      kDefensiveTruckTHWThrd * ego_v_ -
          deltaSpd * (kTimetoLCThrd + kTimetoLFThrd));

  const auto lc_stage =
      planner_output.scheduler_output.lane_change_state.stage();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_RETURN ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  if (lane_feature_info.nearest_leader.has_value()) {
    if (is_in_high_way && length_along_route > force_lc_dist && lc_ongoing &&
        lane_feature_info.nearest_leader->obj_type == OT_LARGE_VEHICLE) {
      if ((lane_feature_info.nearest_leader->obj_s > 0.0 &&
           lane_feature_info.nearest_leader->obj_s <
               safety_efficient_lc_dist) ||
          deltaSpd <= kDefensiveTruckSpdDiffThrd) {
        cost_vec[0] =
            std::clamp(1 - Sqr(lane_feature_info.nearest_leader->obj_s /
                               safety_efficient_lc_dist),
                       0.0, 1.0);
      }
    }
  } else {
    cost_vec[0] = 0.0;
  }

  if (cost_vec[0] > 0.0) {
    traj_feature_output->defensive_driving_lc_behind_truck = true;
  }

  extra_info->emplace_back(absl::StrFormat(
      "defensive_driving_lc_behind_truck: %s",
      btoa(traj_feature_output->defensive_driving_lc_behind_truck)));
  extra_info->emplace_back(absl::StrFormat(
      "defensive_driving_lc_behind_truck_cost: %.2f", cost_vec[0] * 6.0));
  extra_info->emplace_back(absl::StrFormat(
      "LargeVehType_satisfied: %s",
      btoa(lane_feature_info.nearest_leader->obj_type == OT_LARGE_VEHICLE)));
  extra_info->emplace_back(absl::StrFormat(
      "LargeVehType: %d", lane_feature_info.nearest_leader.has_value()
                              ? lane_feature_info.nearest_leader->obj_type
                              : -1));
  extra_info->emplace_back(absl::StrFormat(
      "LeaderVeh_ID: %s", lane_feature_info.nearest_leader.has_value()
                              ? lane_feature_info.nearest_leader->obj_id
                              : "-1"));
  extra_info->emplace_back(
      absl::StrFormat("SpdRange_satisfied: %s",
                      btoa((lane_feature_info.nearest_leader->obj_s > 0.0 &&
                            lane_feature_info.nearest_leader->obj_s <
                                safety_efficient_lc_dist) ||
                           deltaSpd <= kDefensiveTruckSpdDiffThrd)));
  extra_info->emplace_back(absl::StrFormat("Spddiff: %.2f", deltaSpd));
  extra_info->emplace_back(
      absl::StrFormat("Range: %.2f", lane_feature_info.nearest_leader->obj_s));
  return cost_vec;
}

}  // namespace st::planning
