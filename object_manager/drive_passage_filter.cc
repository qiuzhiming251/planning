

#include "drive_passage_filter.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "predictor/prediction_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/log_data.h"

namespace st {
namespace planning {

namespace {

// Returns the right most lateral shift and left most lateral shift on the sl
// boundary from min_s to max_s.
std::pair<double, double> FindMaxRangeOfL(
    const PathSlBoundary& sl_boudnary, const DrivePassage& drive_passage,
    const LaneChangeStateProto& lc_state, const Box2d& ego_box,
    const double min_s, const double max_s, const bool use_out_boundary,
    const bool is_extend) {
  constexpr double kDefaultLaneWidth = 3.75;
  static constexpr double kSampleStep = 1.0;
  double min_lat = std::numeric_limits<double>::infinity();
  double max_lat = -std::numeric_limits<double>::infinity();
  const auto [r, l] = use_out_boundary
                          ? sl_boudnary.QueryOptBoundaryL(max_s)
                          : sl_boudnary.QueryTargetBoundaryL(max_s);
  min_lat = Min(min_lat, r, l);
  max_lat = Max(max_lat, r, l);
  for (double s = min_s; s < max_s; s += kSampleStep) {
    const auto [r, l] = use_out_boundary ? sl_boudnary.QueryOptBoundaryL(s)
                                         : sl_boudnary.QueryTargetBoundaryL(s);
    min_lat = Min(min_lat, r);
    max_lat = Max(max_lat, l);
  }

  // Expand boundary if opt_boundary_width not enough
  if (is_extend && use_out_boundary &&
      lc_state.stage() == st::LaneChangeStage::LCS_NONE) {
    const double bound_width_expand =
        (kDefaultLaneWidth + ego_box.width()) - (max_lat - min_lat);
    if (bound_width_expand > 0.0) {
      min_lat -= 0.5 * bound_width_expand;
      max_lat += 0.5 * bound_width_expand;
    }
  }
  return {min_lat, max_lat};
}

// Check if the polygon has lateral overlap.
bool HasLateralOverlap(const DrivePassage& drive_passage,
                       const Polygon2d& contour, double boundary_min_l,
                       double boundary_max_l) {
  CHECK_GE(boundary_max_l, boundary_min_l);

  double contour_min_l = std::numeric_limits<double>::infinity();
  double contour_max_l = -std::numeric_limits<double>::infinity();
  bool has_projection = false;
  for (const auto& pt : contour.points()) {
    ASSIGN_OR_CONTINUE(const auto offset,
                       drive_passage.QueryFrenetLatOffsetAt(pt));
    has_projection = true;
    UpdateMin(offset, &contour_min_l);
    UpdateMax(offset, &contour_max_l);
  }
  if (!has_projection) return false;

  return contour_min_l <= boundary_max_l && contour_max_l >= boundary_min_l;
}

bool TrajectoryMaybeHasOverlap(const prediction::PredictedTrajectory& traj,
                               const PlannerObject& object,
                               const double obj_buffer,
                               const DrivePassage& drive_passage,
                               const PathSlBoundary& sl_boundary,
                               const Box2d& ego_box,
                               const LaneChangeStateProto& lc_state) {
  constexpr double kTrajectoryConsiderTime = 8.0;

  const auto& obj_pose = object.pose().pos();
  const auto& obj_contour = object.contour();
  const auto& obj_heading = object.bounding_box().heading();

  for (const auto& traj_pt : traj.points()) {
    if (traj_pt.t() > kTrajectoryConsiderTime) break;

    const Vec2d rotation =
        Vec2d::FastUnitFromAngle(traj_pt.theta() - obj_heading);
    const Polygon2d traj_pt_contour = obj_contour.Transform(
        obj_pose, rotation.x(), rotation.y(), traj_pt.pos() - obj_pose);

    if (IsContourWithBufferOverlapsBoundary(traj_pt_contour, obj_buffer,
                                            sl_boundary, drive_passage, ego_box,
                                            lc_state)) {
      return true;
    }
  }
  return false;
}

}  // namespace

// Returns the object distance buffer in meters. If the object's distance to sl
// boundary is larger than this value, we may ignore the object.
double GetDistanceBuffer(const PlannerObject& object) {
  switch (object.type()) {
    case OT_UNKNOWN_STATIC:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.6;
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return 0.8;
    case OT_LARGE_VEHICLE:
      return object.is_stationary() ? 0.8 : 1.0;
    case OT_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_ROW_OBSTACLES:
      return object.is_stationary() ? 0.6 : 0.8;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

bool IsContourWithBufferOverlapsBoundary(
    const Polygon2d& contour, const double buffer,
    const PathSlBoundary& sl_boundary, const DrivePassage& drive_passage,
    const Box2d& ego_box, const LaneChangeStateProto& lc_state,
    const bool use_out_boundary, const bool use_decay_buffer,
    const double* ego_v) {
  constexpr double kDefaultLaneWidth = 3.75;                    // m
  constexpr double kLaneChangeBoundaryAdjustDistFromEgo = 0.3;  // m

  ASSIGN_OR_RETURN(const auto ego_frenet_box,
                   drive_passage.QueryFrenetBoxAt(ego_box), true);

  ASSIGN_OR_RETURN(const auto obj_frenet_box,
                   drive_passage.QueryFrenetBoxAtContour(contour), false);

  double decay_buffer = buffer;
  if (use_decay_buffer) {
    const double dist2ego =
        obj_frenet_box.center_s() - ego_frenet_box.center_s();
    std::vector<double> decay_dist = {-40, -20, 60, 100};
    std::vector<double> decay_factor = {0.25, 1.0, 1.0, 0.3};
    if (ego_v) {
      decay_dist = {Min(-40, -3 * (*ego_v)), Min(-20, -2 * (*ego_v)),
                    Max(60, 4 * (*ego_v)), Max(100, 5 * (*ego_v))};
    }
    const PiecewiseLinearFunction<double, double> decay_plf = {decay_dist,
                                                               decay_factor};
    decay_buffer = buffer * decay_plf(dist2ego);
  }

  if (obj_frenet_box.s_max + decay_buffer < sl_boundary.start_s() ||
      obj_frenet_box.s_min - decay_buffer > sl_boundary.end_s())
    return false;

  auto [min_right_l, max_left_l] = FindMaxRangeOfL(
      sl_boundary, drive_passage, lc_state, ego_box, obj_frenet_box.s_min,
      obj_frenet_box.s_max, use_out_boundary, true);
  if (min_right_l >= max_left_l) return true;

  if (lc_state.stage() == st::LaneChangeStage::LCS_EXECUTING) {
    if (lc_state.lc_left()) {
      min_right_l =
          Min(ego_frenet_box.l_min - kLaneChangeBoundaryAdjustDistFromEgo,
              max_left_l - kDefaultLaneWidth);
    } else {
      max_left_l =
          Max(ego_frenet_box.l_max + kLaneChangeBoundaryAdjustDistFromEgo,
              min_right_l + kDefaultLaneWidth);
    }
  }

  return obj_frenet_box.l_min - decay_buffer <= max_left_l &&
         obj_frenet_box.l_max + decay_buffer >= min_right_l;
}

bool Is_Onpath(const Box2d& ego_box, const Polygon2d& contour,
               const PathSlBoundary& sl_boundary,
               const DrivePassage& drive_passage,
               const LaneChangeStateProto& lc_state,
               const PlannerSemanticMapManager* psmm) {
  const auto ego_frenet_box = drive_passage.QueryFrenetBoxAt(ego_box);
  if (!ego_frenet_box.ok() || !psmm) return false;
  const auto obj_frenet_box_raw =
      drive_passage.QueryFrenetBoxAtContour(contour);
  if (!obj_frenet_box_raw.ok()) return false;
  const auto obj_frenet_box = obj_frenet_box_raw.value();
  // is near split
  const auto& nearest_station =
      drive_passage.FindNearestStationAtS(ego_frenet_box->s_min);
  const auto lane_id = nearest_station.lane_id();
  const auto lane_info = psmm->FindCurveLaneByIdOrNull(lane_id);

  bool is_split = false;
  // to recognize split earlier
  double dist_to_split = 0.0;
  const double kMax_to_split = 120.0;
  if (drive_passage.lane_seq_info() != nullptr) {
    dist_to_split = drive_passage.lane_seq_info()->dist_to_nearest_split.second;
  }
  if (dist_to_split > 0.0 && dist_to_split < kMax_to_split) {
    is_split = true;
  }

  if (lane_info && lane_info->pre_lane_ids().size() == 1) {
    const auto pre_lane_info =
        psmm->FindCurveLaneByIdOrNull(lane_info->pre_lane_ids()[0]);
    if (pre_lane_info && pre_lane_info->next_lane_ids().size() >= 2) {
      is_split = true;
    }
  }

  if (obj_frenet_box.s_max < sl_boundary.start_s() ||
      obj_frenet_box.s_min > sl_boundary.end_s())
    return false;
  auto [min_right_l, max_left_l] =
      FindMaxRangeOfL(sl_boundary, drive_passage, lc_state, ego_box,
                      obj_frenet_box.s_min, obj_frenet_box.s_max, false, false);

  // overlap
  std::pair<double, double> overlap_bound;
  overlap_bound.first = std::min(max_left_l, obj_frenet_box.l_max);
  overlap_bound.second = std::max(min_right_l, obj_frenet_box.l_min);
  // double overlap_center = 0.5 * (overlap_bound.first + overlap_bound.second);
  double overlap = overlap_bound.first - overlap_bound.second;
  double objwidth = obj_frenet_box.l_max - obj_frenet_box.l_min;

  if (is_split &&
      (overlap > 0.3 ||
       (overlap > objwidth - 0.01 && obj_frenet_box.l_max < max_left_l - 0.1 &&
        obj_frenet_box.l_min > min_right_l + 0.1)) &&
      (obj_frenet_box.s_min > dist_to_split || dist_to_split > 1e3))
    return true;
  return false;
}

FilterReason::Type DrivePassageFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  const double obj_buffer = GetDistanceBuffer(object);

  // special type filter
  std::string object_debug = " obj_id: " + object.id() +
                             ", obj_type: " + ObjectType_Name(object.type());
  if (object.type() == OT_ROW_OBSTACLES) {
    Log2DDS::LogDataV2("drivepassage_filter",
                       object_debug + " Filtered when ROW_OBSTACLES");
    return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
  }

  auto lane_attr_type =
      object.object_proto().vision_attribute().lane_attr_type();
  bool is_lane_change = lc_state_->stage() == LaneChangeStage::LCS_EXECUTING ||
                        lc_state_->stage() == LaneChangeStage::LCS_PAUSE ||
                        lc_state_->stage() == LaneChangeStage::LCS_RETURN;
  if (object.type() == st::ObjectType::OT_BARRIER ||
      object.type() == st::ObjectType::OT_CONE ||
      object.type() == st::ObjectType::OT_WARNING_TRIANGLE) {
    // is real on path
    bool is_real_onpath = Is_Onpath(ego_box_, object.contour(), *sl_boundary_,
                                    *drive_passage_, *lc_state_, psmm_);
    object_debug =
        absl::StrCat(object_debug, ", lane_attr_type: ", lane_attr_type,
                     ", is_real_onpath: ", is_real_onpath);
    if (lc_state_->stage() == LaneChangeStage::LCS_NONE &&
        (lane_attr_type == LaneAttrType::LANEATTR_LEFT ||
         lane_attr_type == LaneAttrType::LANEATTR_RIGHT) &&
        !is_real_onpath) {
      Log2DDS::LogDataV2(
          "drivepassage_filter",
          object_debug + " Filtered LANEATTR_LEFT/RIGHT when no lane change");
      return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
    } else if (is_lane_change && lc_state_->lc_left() &&
               lane_attr_type == LaneAttrType::LANEATTR_RIGHT) {
      Log2DDS::LogDataV2(
          "drivepassage_filter",
          object_debug + " Filtered LANEATTR_RIGHT when lane change left");
      return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
    } else if (is_lane_change && !lc_state_->lc_left() &&
               lane_attr_type == LaneAttrType::LANEATTR_LEFT) {
      Log2DDS::LogDataV2(
          "drivepassage_filter",
          object_debug + " Filtered LANEATTR_LEFT when lane change right");
      return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
    } else if (is_on_highway_ &&
               lane_attr_type == LaneAttrType::LANEATTR_OTHER) {
      Log2DDS::LogDataV2(
          "drivepassage_filter",
          object_debug + " Filtered all LANEATTR_OTHER when on highway");
      return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
    }
  }

  if (prediction::IsStationaryTrajectory(traj)) {
    if (!IsContourWithBufferOverlapsBoundary(object.contour(), obj_buffer,
                                             *sl_boundary_, *drive_passage_,
                                             ego_box_, *lc_state_)) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_SL_BOUNDARY;
    }
  } else {
    // Filtering moving objects.
    if (!TrajectoryMaybeHasOverlap(traj, object, obj_buffer, *drive_passage_,
                                   *sl_boundary_, ego_box_, *lc_state_)) {
      return FilterReason::TRAJECTORY_NOT_ON_SL_BOUNDARY;
    }
  }
  return FilterReason::NONE;
}
}  // namespace planning
}  // namespace st
