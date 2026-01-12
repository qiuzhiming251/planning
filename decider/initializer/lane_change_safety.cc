

#include "decider/initializer/lane_change_safety.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <array>
#include <cmath>
#include <numeric>
#include <ostream>
#include <float.h>
#include <algorithm>

#include "gflags/gflags.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "plan_common/log_data.h"
#include "plan_common/gflags.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/math/vec.h"
#include "plan_common/math/util.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "plan_common/util/spatial_search_util.h"

#include "predictor/prediction.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_object_state.h"

#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "decider/scheduler/path_boundary_builder_helper.h"
#include "decider/initializer/initializer_util.h"
#include "alternative_gaming/speed_gaming/gaming_lane_change_check.h"

// #pragma GCC push_options
// #pragma GCC optimize("O0")
namespace st::planning {
constexpr double kEnterTargetLateralThreshold = 1.5;        // m.
constexpr double kFullyEnterTargetLateralThreshold = 1.2;   // m.
constexpr double kStaticEnterTargetLateralThreshold = 0.6;  // m.

constexpr double kMaxAllowedDecelForObject = 1.5;  // m/s^2
constexpr double kMaxAllowedDecelForEgo = 2.5;     // m/s^2

constexpr double kMinLonBufferToFront = 2.0;          // m.
constexpr double kMinLatBufferToCloseBlocking = 0.8;  // m.
constexpr double kLatThresholdToCloseBlocking = 0.5;  // m.

// The following params could be tuned.
constexpr double kFollowerStandardResponseTime = 0.6;  // s
constexpr double kEgoResponseTime = 0.4;               // s
constexpr double kEgoFollowTimeBufferPrepare = 0.5;    // s.
constexpr double kEgoLeadTimeBufferPrepare = 0.5;      // s.
constexpr double kEgoFollowTimeBufferProcess = 0.4;    // s.
constexpr double kEgoLeadTimeBufferProcess = 0.4;      // s.
constexpr double kLondisignore = 10.0;
constexpr double kObjTrajectoryStep = 0.1;             // s.
constexpr double kObjCutoutSoonPreviewTime = 3.0;      // s.
constexpr double kObjReliableTrajectoryTimeMax = 3.0;  // s.
constexpr double kObjMostReliableTrajectoryTimeMax = 1.5;  // s.
constexpr int kObjCutoutSoonPreviewIndex =
    std::max(1, static_cast<int>(std::round(kObjCutoutSoonPreviewTime /
                                            kObjTrajectoryStep)));
constexpr int kObjReliableTrajectoryIndexMax =
    std::max(1, static_cast<int>(std::round(kObjReliableTrajectoryTimeMax /
                                            kObjTrajectoryStep)));
constexpr int kObjMostReliableTrajectoryIndexMax =
    std::max(1, static_cast<int>(std::round(kObjMostReliableTrajectoryTimeMax /
                                            kObjTrajectoryStep)));

constexpr double kBackTTCRelaxDistanceThresholdHighway = 40.0;  // m
constexpr double kBackTTCRelaxDistanceThresholdCity = 30.0;     // m

constexpr double kResponseStyleConserToNormalFactor = 0.8;
constexpr double kResponseStyleConserToRadicalFactor = 0.7;
constexpr double kResponseStyleNormalToRadicalFactor = 0.875;

constexpr double kEgoLowSpeedMpsThreshold = 8.333;  // m/s. 30.0kph

constexpr double kEpsilon = 1e-5;
// 根据速度查表调整目标安全判断的最小阈值
const std::vector<double> MIN_FRONT_DISTANCE_VEC = {1.2, 2.0, 2.5, 3.0,
                                                    4.0, 5.0, 10.0};
const std::vector<double> MIN_BACK_DISTANCE_VEC = {0.8, 1.0, 1.5, 2.5,
                                                   3.8, 5.0, 10.0};
const std::vector<double> V_EGO_VEHICLE_VEC = {0.0, 15, 20, 30, 60, 90, 120};

// 根据速度查表动态调整head_way_time计算的系数
const std::vector<double> K_FRONT_HEAD_WAY_VEC = {0.20, 0.50, 0.40, 0.36,
                                                  0.30, 0.44, 0.45};
const std::vector<double> K_BACK_HEAD_WAY_VEC = {0.20, 0.24, 0.18, 0.60,
                                                 0.78, 0.72, 0.66};
const std::vector<double> V_HEAD_WAY_VEC = {0.0, 15, 20, 30, 60, 90, 120};
const std::vector<double> F_DV_FRONT_HEAD_WAY_VEC = {0.75, 0.8,  1.0,
                                                     1.0,  1.15, 1.2};
const std::vector<double> F_DV_BACK_HEAD_WAY_VEC = {0.85, 0.95, 1.0,
                                                    1.05, 1.2,  1.25};
const std::vector<double> DV_HEAD_WAY_VEC = {-20, -10, 0.0, 10, 20, 30};

const std::vector<double> ENTER_TARGET_TIME_VEC = {0.5, 1.0, 1.5, 2.0,
                                                   3.0, 4.0, 5.0};
const std::vector<double> ENTER_TARGET_PREVIEW_FACTOR_VEC = {1.0, 1.2, 1.3, 1.4,
                                                             1.5, 1.6, 1.7};

const double kMinCompensationFactor = 0.38;
const double kMaxCompensationFactor = 1.0;

const double kRearObjIgnoreLatOverlapFactor = 0.2;
const double kLaneChangeMinLimitTTC = 2.0;
double kExtremelySlowVehicleSpeedkph = 7.0;
double kStationaryVehicleSpeedkph = 1.5;

// 根据相对速度差来动态调整舒适制动减速度
const std::vector<double> F_DV_FRONT_DEC_VEC = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
const std::vector<double> F_DV_STATIC_DEC_VEC = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
const std::vector<double> F_DV_BACK_DEC_VEC = {0.6, 1.2, 1.5, 1.8, 2.0, 3.0};
const std::vector<double> DV_TTC_VEC = {0.0, 10, 20, 30, 40, 120};

const std::vector<double> kEenterTargetLaneLatDis = {0.6, 0.5,  0.4, 0.3,
                                                     0.0, -0.3, -0.5};
const std::vector<double> kEgoVehSpdKphVct = {0.0, 15, 20, 30, 60, 90, 120};

std::vector<double> occupied_width_vec = {0.9, 0.8, 0.7, 0.6, 0.5};
std::vector<double> rel_distance_vec = {0.0, 30, 60, 80, 120};

bool HasEnteredTargetLane(const double center_l, const double half_width) {
  return std::abs(center_l) < kEnterTargetLateralThreshold + half_width;
}

bool HasEnteredTargetLane(const FrenetBox& obj_box, const double lane_width) {
  bool has_entered = false;

  if (obj_box.l_max * obj_box.l_min > 0) {
    double min_dis_target =
        std::min(std::fabs(obj_box.l_max), std::fabs(obj_box.l_min));
    double intrusion_dis_thrd =
        std::clamp((0.38 * (lane_width / kDefaultLaneWidth)), 0.2, 0.5);
    has_entered = min_dis_target < lane_width * 0.5 - intrusion_dis_thrd;
  } else {
    has_entered = true;
  }
  return has_entered;
}

bool HasFullyEnteredTargetLane(const double center_l, const double half_width) {
  return std::abs(center_l) < kFullyEnterTargetLateralThreshold + half_width;
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

bool HasFullyCenteredInTargetLane(const FrenetBox& obj_box,
                                  const double half_width) {
  bool corner_has_centered = false;
  bool center_has_entered = false;

  if (obj_box.l_max * obj_box.l_min < 0) {
    corner_has_centered = true;
  } else {
    corner_has_centered = false;
  }

  center_has_entered =
      std::abs(obj_box.center_l()) < kFullyEnterTargetLateralThreshold * 0.5;
  return corner_has_centered && center_has_entered;
}

bool isEgoFullyOccupyTargetLane(const FrenetBox& ego_frenet_box,
                                const double ego_v_kph, const bool lc_left) {
  double ref_center_l = 0;
  const double ref_l_min = ego_frenet_box.l_min - ref_center_l;
  const double ref_l_max = ego_frenet_box.l_max - ref_center_l;
  double enter_target_lane_dis_thrd = ad_byd::planning::math::interp1_inc(
      kEgoVehSpdKphVct, kEenterTargetLaneLatDis, ego_v_kph);

  bool ego_entered_target_lane =
      (lc_left ? ref_l_max > enter_target_lane_dis_thrd
               : ref_l_min < -enter_target_lane_dis_thrd);

  return ego_entered_target_lane;
}

bool isStaticObsRideLine(const FrenetBox& ego_box, const FrenetBox& obj_box,
                         const bool lc_left, const double rel_ds) {
  double occupied_width_limit = ad_byd::planning::math::interp1_inc(
      rel_distance_vec, occupied_width_vec, rel_ds);

  bool ride_line = lc_left ? (obj_box.l_max <= -1.3 * occupied_width_limit &&
                              obj_box.l_min >= ego_box.l_max)
                           : (obj_box.l_min >= 1.3 * occupied_width_limit &&
                              obj_box.l_max <= ego_box.l_min);
  return ride_line;
}

inline bool IsVehicle(
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory) {
    return false;
  }
  auto obs_type = obstacle_trajectory->object_type();
  bool is_vehicle = (obs_type == OT_VEHICLE || obs_type == OT_LARGE_VEHICLE ||
                     obs_type == OT_TRICYCLIST);
  return is_vehicle;
}

bool isStaticObsOccupyTargetLane(
    const SpacetimeObjectTrajectory* const obstacle_trajectory,
    const FrenetBox& obj_box, const FrenetBox& ego_box, const double half_width,
    const bool lc_left, const double rel_ds) {
  const bool is_vehicle = IsVehicle(obstacle_trajectory);
  double occupied_width_limit = ad_byd::planning::math::interp1_inc(
      rel_distance_vec, occupied_width_vec, rel_ds);

  bool corner_has_entered = false;
  bool center_has_entered = false;
  double min_dis_target =
      std::min(std::fabs(obj_box.l_max), std::fabs(obj_box.l_min));
  double max_dis_target =
      std::max(std::fabs(obj_box.l_max), std::fabs(obj_box.l_min));

  bool big_heading = (max_dis_target - min_dis_target) / (2 * half_width) > 1.2;

  bool far_away_ego_obs =
      (obj_box.l_max * obj_box.l_min > 0) &&
      (lc_left ? (obj_box.l_min > 0.3) : (obj_box.l_max < -0.3));

  if (is_vehicle && far_away_ego_obs) {
    return false;
  }

  if (obj_box.l_max * obj_box.l_min > 0) {
    bool close_ego_side_obs =
        lc_left ? (obj_box.l_max < 0) : (obj_box.l_min > 0);
    corner_has_entered =
        std::fabs(min_dis_target) <
        occupied_width_limit * (close_ego_side_obs ? 1.0 : 0.8);
  } else {
    corner_has_entered = true;
  }

  center_has_entered = std::abs(obj_box.center_l()) <
                       kStaticEnterTargetLateralThreshold + half_width;
  return corner_has_entered || center_has_entered;
}

bool isStaticConstructionObsOccupyTargetLane(const FrenetBox& obj_box,
                                             const double half_lane_width,
                                             const bool lc_left) {
  const double kStaticConstructionObsOccupyBuffer = 1.05;
  return lc_left ? (obj_box.l_min < kStaticConstructionObsOccupyBuffer &&
                    obj_box.l_max > -half_lane_width)
                 : (obj_box.l_max > -kStaticConstructionObsOccupyBuffer &&
                    obj_box.l_min < half_lane_width);
}

namespace {
inline bool IsBigVehicle(
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory) {
    return false;
  }
  auto obs_type = obstacle_trajectory->object_type();
  double obs_length = !obstacle_trajectory->states().empty()
                          ? obstacle_trajectory->states().front().box.length()
                          : 5.0;
  double obs_width = !obstacle_trajectory->states().empty()
                         ? obstacle_trajectory->states().front().box.width()
                         : 2.0;
  bool is_big_vehicle =
      (obs_type == OT_LARGE_VEHICLE &&
       (obs_length > 6.0 || obs_width > 2.6 || obs_length * obs_width > 11.6));
  return is_big_vehicle;
}

inline bool IsVRU(const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory) {
    return false;
  }
  auto obs_type = obstacle_trajectory->object_type();
  bool is_vru = (obs_type == OT_MOTORCYCLIST || obs_type == OT_PEDESTRIAN ||
                 obs_type == OT_CYCLIST || obs_type == OT_TRICYCLIST);
  return is_vru;
}

inline bool IsConstructionObs(
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory) {
    return false;
  }
  auto obs_type = obstacle_trajectory->object_type();
  bool is_cst = (obs_type == OT_UNKNOWN_STATIC ||
                 obs_type == OT_UNKNOWN_MOVABLE || obs_type == OT_CONE ||
                 obs_type == OT_BARRIER || obs_type == OT_WARNING_TRIANGLE);
  return is_cst;
}

inline bool IsSizeQualified(
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  if (!obstacle_trajectory || obstacle_trajectory->states().empty()) {
    return false;
  }

  auto obs_length = obstacle_trajectory->states().front().box.length();
  auto obs_width = obstacle_trajectory->states().front().box.width();
  auto obs_height = 1.0;
  // auto& obs_height = obstacle_trajectory->states().front().box.height();

  bool size_qualified = false;
  if (obs_length >= 0.3 || obs_width >= 0.3 || obs_height >= 0.3 ||
      (obs_length * obs_width * obs_height >= 0.003)) {
    size_qualified = true;
  }

  return size_qualified;
}

// Check for moving objects.
// 根据速度查表调整目标安全判断的最小阈值
inline double getMinDisCompensation(bool if_obs_front, bool is_big_vehicle,
                                    const Box2d& obj_cur_box) {
  bool frt_big_veh = is_big_vehicle && if_obs_front;
  double min_dis_compensation = 0.0;

  if (frt_big_veh) {
    double ego_veh_length = 5;
    double factor =
        std::floor(obj_cur_box.length() / ((2 * ego_veh_length) + kEpsilon));
    min_dis_compensation = std::clamp((factor * 0.5), 0.0, 3.0);
  }

  return min_dis_compensation;
}

inline double QuadraticLerp(double x0, double t0, double x1, double t1,
                            double t, bool clamp = true) {
  if (std::abs(t1 - t0) <= 0.0001) {
    return x0;
  }
  if (t0 > t1) {
    std::swap(t0, t1);
    std::swap(x0, x1);
  }
  if (clamp) {
    if (t0 <= t1) {
      if (t <= t0) {
        return x0;
      }
      if (t >= t1) {
        return x1;
      }
    }
  }

  if (t0 > t1) {
    std::swap(t0, t1);
    std::swap(x0, x1);
  }

  const double a = (x0 - x1) / pow(t0 - t1, 2);
  const double b = x1;

  const double x = a * pow(t - t1, 2) + b;
  return x;
}

double getLongOverlap(const st::FrenetBox& ego_cur_frenet_box,
                      const st::FrenetBox& obj_cur_frenet_box, double& rel_ds) {
  const bool if_obs_front =
      (ego_cur_frenet_box.s_max < obj_cur_frenet_box.s_min);
  const double long_overlap =
      std::min(ego_cur_frenet_box.s_max, obj_cur_frenet_box.s_max) -
      std::max(ego_cur_frenet_box.s_min, obj_cur_frenet_box.s_min);
  rel_ds = (long_overlap > -kEpsilon)
               ? 0
               : (if_obs_front ? -long_overlap : long_overlap);
  return long_overlap;
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

double getLatOverlapWithLcDirection(const st::FrenetBox& ego_cur_frenet_box,
                                    const st::FrenetBox& obj_cur_frenet_box,
                                    double& rel_dl) {
  const double kLcDirectionMaxSpacingDis = 3.75;
  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  double lat_overlap_with_direction = 0;
  const bool if_obs_left =
      (ego_cur_frenet_box.l_max < obj_cur_frenet_box.l_min);
  const double lat_overlap =
      std::min(ego_cur_frenet_box.l_max, obj_cur_frenet_box.l_max) -
      std::max(ego_cur_frenet_box.l_min, obj_cur_frenet_box.l_min);
  rel_dl = (lat_overlap > -kEpsilon)
               ? 0
               : (if_obs_left ? -lat_overlap : lat_overlap);

  if (lc_left && (lat_overlap > -kEpsilon ||
                  (rel_dl > kEpsilon &&
                   obj_cur_frenet_box.l_min - ego_cur_frenet_box.l_max <
                       kLcDirectionMaxSpacingDis))) {
    lat_overlap_with_direction =
        obj_cur_frenet_box.l_max - ego_cur_frenet_box.l_min;
  } else if (!lc_left && (lat_overlap > -kEpsilon ||
                          (rel_dl < -kEpsilon &&
                           ego_cur_frenet_box.l_min - obj_cur_frenet_box.l_max <
                               kLcDirectionMaxSpacingDis))) {
    lat_overlap_with_direction =
        ego_cur_frenet_box.l_max - obj_cur_frenet_box.l_min;
  } else {
    lat_overlap_with_direction = 0;
  }

  return lat_overlap_with_direction;
};

inline double getObjPredictedTrajectoryTime(
    const SpacetimeObjectTrajectory* const obstacle_trajectory,
    bool if_obs_front) {
  double obj_trj_time = 6.0;
  auto obs_type = obstacle_trajectory->object_type();
  bool is_pedestrian = (obs_type == OT_PEDESTRIAN);

  if (is_pedestrian) {
    obj_trj_time = 3.0;
  } else {
    obj_trj_time = 7.0;  // 5.0;
  }
  return obj_trj_time;
}

inline double getObjTypeFactor(
    const SpacetimeObjectTrajectory* const obstacle_trajectory,
    bool if_obs_front) {
  double obj_type_factor = 1.0;
  if (IsBigVehicle(obstacle_trajectory)) {
    if (if_obs_front) {
      obj_type_factor = 1.2;
    } else {
      obj_type_factor = 1.5;
    }
  } else if (IsVRU(obstacle_trajectory)) {
    if (if_obs_front) {
      obj_type_factor = 1.8;
    } else {
      obj_type_factor = 1.38;
    }
  } else {
    obj_type_factor = 1.0;
  }
  return obj_type_factor;
}

double getLatOffsetFactor(const st::FrenetBox& ego_cur_frenet_box,
                          double ego_half_width, double intrusion_dis,
                          double lane_width) {
  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  const double closet_corner_lat_offset =
      lc_left ? std::abs(std::min(0.0, ego_cur_frenet_box.l_max))
              : std::abs(std::max(0.0, ego_cur_frenet_box.l_min));

  double lat_offset_factor =
      QuadraticLerp(kMinCompensationFactor, (0.5 * lane_width - intrusion_dis),
                    kMaxCompensationFactor, (lane_width - ego_half_width),
                    closet_corner_lat_offset, true);
  return lat_offset_factor;
}

double getLatOffsetFactor(const st::FrenetBox& ego_cur_frenet_box,
                          double ego_half_width, double min_compensation_factor,
                          double intrusion_dis, double lane_width) {
  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  const double closet_corner_lat_offset =
      lc_left ? std::abs(std::min(0.0, ego_cur_frenet_box.l_max))
              : std::abs(std::max(0.0, ego_cur_frenet_box.l_min));

  double lat_offset_factor =
      QuadraticLerp(min_compensation_factor, (0.5 * lane_width - intrusion_dis),
                    kMaxCompensationFactor, (lane_width - ego_half_width),
                    closet_corner_lat_offset, true);
  return lat_offset_factor;
}

inline bool IsLatClosestBlocking(const FrenetBox& ego_box,
                                 const FrenetBox& obj_box, double obj_lat_ext) {
  double rel_dl = 0;
  double lat_overlap = getLatOverlap(ego_box, obj_box, rel_dl);
  const bool lc_left = ego_box.center_l() < 0.0;

  return !HasFullyEnteredTargetLane(obj_box, 0.5 * obj_box.width()) &&
         (obj_box.center_l() * ego_box.center_l() > 0.0) &&
         (std::abs(ego_box.center_l()) > std::abs(obj_box.center_l()) ||
          std::fabs(rel_dl) < ego_box.width() * 0.6) &&
         ((!lc_left &&
           ego_box.center_l() - ego_box.width() * 0.5 - obj_box.width() * 0.5 -
                   std::max(kMinLatBufferToCloseBlocking,
                            kLatThresholdToCloseBlocking + obj_lat_ext) <
               obj_box.center_l()) ||
          (lc_left &&
           obj_box.center_l() - ego_box.width() * 0.5 - obj_box.width() * 0.5 -
                   std::max(kMinLatBufferToCloseBlocking,
                            kLatThresholdToCloseBlocking - obj_lat_ext) <
               ego_box.center_l()));
}

inline bool IsBlockingObjectAbreast(const FrenetBox& ego_box,
                                    const FrenetBox& obj_box,
                                    const double obj_front_ext,
                                    const double obj_lat_ext,
                                    const double min_lon_buffer,
                                    const bool obj_willenter_tarlane,
                                    const bool can_not_return) {
  return (((HasFullyEnteredTargetLane(obj_box, 0.5 * obj_box.width()) ||
            IsLatClosestBlocking(ego_box, obj_box, obj_lat_ext)) &&
           !can_not_return) ||
          obj_willenter_tarlane) &&
         ego_box.s_max + std::max(min_lon_buffer, obj_front_ext) >
             obj_box.s_min &&
         obj_box.s_max + std::max(min_lon_buffer, obj_front_ext) >
             ego_box.s_min;
}

inline double GetLaneChangeStyleFactor(LaneChangeStyle lc_style) {
  switch (lc_style) {
    case LC_STYLE_NORMAL:
      return FLAGS_planner_lc_safety_normal_factor;
    case LC_STYLE_RADICAL:
      return FLAGS_planner_lc_safety_radical_factor;
    case LC_STYLE_CONSERVATIVE:
      return FLAGS_planner_lc_safety_conservative_factor;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

inline double GetResponseStyleFactor(
    LaneChangeStyle lc_style,
    const std::pair<PathResponseStyle, SpeedResponseStyle>& response_style) {
  if (lc_style == LC_STYLE_NORMAL &&
      (response_style.first == PathResponseStyle::PATH_RESPONSE_RADICAL ||
       response_style.second == SpeedResponseStyle::SPEED_RESPONSE_RADICAL)) {
    return kResponseStyleNormalToRadicalFactor;
  }
  if (lc_style == LC_STYLE_CONSERVATIVE &&
      (response_style.first == PathResponseStyle::PATH_RESPONSE_NORMAL ||
       response_style.second == SpeedResponseStyle::SPEED_RESPONSE_NORMAL)) {
    return kResponseStyleConserToNormalFactor;
  }
  if (lc_style == LC_STYLE_CONSERVATIVE &&
      (response_style.first == PathResponseStyle::PATH_RESPONSE_RADICAL ||
       response_style.second == SpeedResponseStyle::SPEED_RESPONSE_RADICAL)) {
    return kResponseStyleConserToRadicalFactor;
  }
  return 1.0;
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

double EstimateObjectSpeed(const PlannerObject& object, double preview_time) {
  double obj_v = object.pose().v();
  const auto& accel_hist = object.long_term_behavior().accel_history;
  if (!accel_hist.empty()) {
    constexpr int kMaxConsideredAccelHistoryItem = 5;  // One record per second.
    constexpr std::array<double, 5> kAccelWeights{8.0, 6.0, 4.0, 2.0, 1.0};

    int idx = 0;
    double avg_accel = 0.0;
    for (auto it = accel_hist.rbegin(); it != accel_hist.rend(); ++it) {
      avg_accel += *it * kAccelWeights[idx++];
      if (idx >= kMaxConsideredAccelHistoryItem) break;
    }
    avg_accel /= std::accumulate(kAccelWeights.begin(),
                                 kAccelWeights.begin() + idx, 0.0);
    obj_v += avg_accel * preview_time;
  }
  return obj_v;
}

bool IsLeavingTargetLanePath(
    const FrenetFrame& target_frenet_frame, bool ego_lc_left,
    const FrenetBox& ego_cur_box, const FrenetBox& obj_cur_box,
    absl::Span<const prediction::PredictionObjectState> obj_states,
    double lon_safe_min_dist, double min_lon_buffer) {
  const double obj_width = obj_cur_box.width();
  const auto obj_preview_l =
      target_frenet_frame
          .XYToSL(obj_states.size() < kObjMostReliableTrajectoryIndexMax
                      ? obj_states.back().traj_point->pos()
                      : (obj_states.begin() +
                         kObjMostReliableTrajectoryIndexMax - 1)
                            ->traj_point->pos())
          .l;

  if (HasFullyEnteredTargetLane(obj_preview_l, 0.5 * obj_width)) return false;

  const double obj_cur_l = obj_cur_box.center_l();
  const bool obj_lc_left = (obj_preview_l > obj_cur_l);
  const bool obj_cut_out_clearly =
      (std::fabs(obj_preview_l) > std::fabs(obj_cur_l)) &&
      (std::fabs(obj_preview_l - obj_cur_l) > 0.15);

  if ((ego_lc_left == obj_lc_left && ego_cur_box.s_max < obj_cur_box.s_min) ||
      (ego_lc_left != obj_lc_left &&
       ego_cur_box.s_min < obj_cur_box.s_max + min_lon_buffer) ||
      (std::fabs(ego_cur_box.center_s() - obj_cur_box.center_s()) <
       lon_safe_min_dist + ego_cur_box.length() * 0.5 +
           obj_cur_box.length() * 0.5)) {
    return false;
  }

  constexpr double kLeaveLaneLatRatio = 0.15;
  return (ego_lc_left && obj_lc_left && obj_cut_out_clearly &&
          obj_cur_box.l_min > -kLeaveLaneLatRatio * obj_width) ||
         (!ego_lc_left && !obj_lc_left && obj_cut_out_clearly &&
          obj_cur_box.l_max < kLeaveLaneLatRatio * obj_width);
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

bool PathHasOverlap(
    absl::Span<const Box2d> ego_boxes, double obj_v,
    absl::Span<const prediction::PredictionObjectState> obj_states) {
  constexpr double kFrontExtensionTime = 2.0;    // s.
  constexpr double kLateralExtension = 2 * 0.5;  // m.
  // First check the current state.
  auto obj_cur_ext_box =
      obj_states[0].box.ExtendedAtFront(obj_v * kFrontExtensionTime);
  obj_cur_ext_box.LateralExtend(kLateralExtension);
  if (ego_boxes[0].HasOverlap(obj_cur_ext_box)) return true;

  // Check the whole path.
  for (const auto& obj_state : obj_states) {
    Box2d obj_ext_box = obj_state.box;
    obj_ext_box.LateralExtend(kLateralExtension);
    for (const auto& ego_box : ego_boxes) {
      if (ego_box.HasOverlap(obj_ext_box)) {
        return true;
      }
    }
  }
  return false;
}

double ComputeEnterTargetTime(
    const FrenetFrame& target_frenet_frame,
    absl::Span<const prediction::PredictionObjectState> obj_states) {
  const double obj_half_width = obj_states[0].box.width() * 0.5;
  for (const auto& state : obj_states) {
    if (state.traj_point->t() > kObjReliableTrajectoryTimeMax) {
      break;
    }
    const auto obj_sl = target_frenet_frame.XYToSL(state.traj_point->pos());
    if (HasFullyEnteredTargetLane(obj_sl.l, obj_half_width)) {
      return state.traj_point->t() - obj_states.front().traj_point->t();
    }
  }
  return DBL_MAX;
}

double ComputeEnterTargetTimeByLatV(const FrenetBox& frenet_box,
                                    const double lat_v) {
  const double min_abs_l =
      std::min(std::abs(frenet_box.l_min), std::abs(frenet_box.l_max));

  if (frenet_box.l_min * frenet_box.l_max < 0.0 ||
      min_abs_l < kFullyEnterTargetLateralThreshold) {
    return 0.0;
  }

  if (frenet_box.center_l() * lat_v > 0) {
    return DBL_MAX;
  }

  const double abs_lat_v = std::abs(lat_v) + kEpsilon;

  return std::max((min_abs_l - kFullyEnterTargetLateralThreshold) / abs_lat_v,
                  0.0);
}

double ComputeMinLonBufferSimilarSpeedFactor(
    double lead_v, double follow_v,
    const SpacetimeObjectTrajectory* const obstacle_trajectory) {
  const std::vector<double> K_SPEED_FACTOR_AGGR_VEC = {
      1.2, 1.1, 1.0, 0.95, 0.9, 0.8, 0.78, 0.75, 0.7};
  const std::vector<double> K_SPEED_FACTOR_CNSV_VEC = {
      2.0, 1.7, 1.5, 1.2, 1.0, 0.95, 0.9, 0.85, 0.75};
  const std::vector<double> V_FOLLOW_DV_VEC = {-30, -15, -10, -5, 0,
                                               5,   10,  15,  30};

  auto obs_type = obstacle_trajectory->object_type();
  bool is_motor = (obs_type == OT_MOTORCYCLIST || obs_type == OT_CYCLIST);

  double dv_to_follow = (lead_v - follow_v);

  double k_speed_factor = ad_byd::planning::math::interp1_inc(
      V_FOLLOW_DV_VEC,
      (!is_motor ? K_SPEED_FACTOR_AGGR_VEC : K_SPEED_FACTOR_CNSV_VEC),
      Mps2Kph(dv_to_follow));

  return std::clamp(k_speed_factor, 0.7, 2.0);
}

double ComputeSimilarSpeedFactor(double lead_v, double follow_v, double ref_v,
                                 bool is_on_highway) {
  double kMaxSpeedDiffThres = is_on_highway ? 5.0 : 2.5;  // m/s.
  constexpr double kSimilarSpeedThresRatio = 0.25;

  double SpeedDiffThres =
      std::min(kMaxSpeedDiffThres, kSimilarSpeedThresRatio * std::fabs(ref_v));

  return std::clamp((follow_v - lead_v + SpeedDiffThres) / SpeedDiffThres, 0.0,
                    1.0);
}

double ComputeEgoLeadTime(double speed_limit, double ego_v, double obj_v,
                          LaneChangeStage lc_state, double min_lon_buffer,
                          bool is_on_highway) {
  double k_speed_factor = ad_byd::planning::math::interp1_inc(
      V_HEAD_WAY_VEC, K_BACK_HEAD_WAY_VEC, Mps2Kph(obj_v));
  double dv_kph = Mps2Kph(obj_v - ego_v);
  double k_dv_factor = ad_byd::planning::math::interp1_inc(
      DV_HEAD_WAY_VEC, F_DV_BACK_HEAD_WAY_VEC, dv_kph);
  constexpr double kMinSimilarSpeedFactor = 0.0;
  const double similar_speed_factor =
      std::max(ComputeSimilarSpeedFactor(ego_v, obj_v, obj_v, is_on_highway),
               kMinSimilarSpeedFactor);

  constexpr double kExceedSpeedLimitRatio = 0.95;
  constexpr double kExceedSpeedLimitSlope = 10.0;

  const bool is_lane_change_state =
      (lc_state == LaneChangeStage::LCS_EXECUTING) ||
      (lc_state == LaneChangeStage::LCS_PAUSE) ||
      (lc_state == LaneChangeStage::LCS_RETURN);

  double ego_stationary_time_compensation = 0;
  if (ego_v < kEpsilon) {
    ego_stationary_time_compensation = is_lane_change_state ? 0.5 : 1.0;
  } else if (ego_v < 1) {
    ego_stationary_time_compensation = is_lane_change_state ? 0.2 : 0.5;
  } else if (ego_v < 2) {
    ego_stationary_time_compensation = is_lane_change_state ? 0.1 : 0.2;
  } else {
    ego_stationary_time_compensation = 0;
  }

  double kEgoLeadTimeBuffer =
      is_lane_change_state
          ? (kEgoLeadTimeBufferProcess + ego_stationary_time_compensation)
          : (kEgoLeadTimeBufferPrepare + ego_stationary_time_compensation);

  const double min_lead_time = (obj_v > kEpsilon) ? min_lon_buffer / obj_v : 0;

  const double speed_limit_ =
      std::max(speed_limit, std::max(Kph2Mps(30.0), ego_v));

  const std::vector<double> BEYOND_SPD_FACTOR_VEC = {0.0, 0.2, 0.5, 1.0, 1.0};
  const std::vector<double> V_BYOND_DV_VEC = {0.0, 0.1, 0.2, 0.5, 1};
  double k_beyond_spd_factor = ad_byd::planning::math::interp1_inc(
      V_BYOND_DV_VEC, BEYOND_SPD_FACTOR_VEC,
      std::max(0.0, obj_v / speed_limit_ - 1));

  const double speed_limit_factor =
      1.0 +
      kExceedSpeedLimitSlope * k_beyond_spd_factor *
          Sqr(std::max(0.0, obj_v / speed_limit_ - kExceedSpeedLimitRatio));

  return std::max(min_lead_time, kEgoLeadTimeBuffer * similar_speed_factor *
                                     speed_limit_factor * k_speed_factor *
                                     k_dv_factor);
}

double ComputeEgoFollowTime(double obj_v, double ego_v,
                            LaneChangeStage lc_state, double min_lon_buffer) {
  // 根据速度查表动态调整head_way_time计算的系数
  double k_speed_factor = ad_byd::planning::math::interp1_inc(
      V_HEAD_WAY_VEC, K_FRONT_HEAD_WAY_VEC, Mps2Kph(ego_v));

  // 根据速度差查表动态调整head_way_time计算的系数
  double dv_kph = Mps2Kph(ego_v - obj_v);
  double k_dv_factor = ad_byd::planning::math::interp1_inc(
      DV_HEAD_WAY_VEC, F_DV_FRONT_HEAD_WAY_VEC, dv_kph);

  constexpr double kHigherSpeedThresRatio = 0.95;
  double PauseFactor = 1.0;
  const bool is_lane_change_state =
      (lc_state == LaneChangeStage::LCS_EXECUTING) ||
      (lc_state == LaneChangeStage::LCS_PAUSE) ||
      (lc_state == LaneChangeStage::LCS_RETURN);
  double kEgoFollowTimeBuffer = is_lane_change_state
                                    ? kEgoFollowTimeBufferProcess
                                    : kEgoFollowTimeBufferPrepare;
  if (lc_state == LCS_PAUSE) {
    PauseFactor = 1.25;
  }

  const double min_follow_time =
      (std::fabs(ego_v) > kEpsilon) ? min_lon_buffer / ego_v : 0;

  // if (obj_v > ego_v * kHigherSpeedThresRatio && std::fabs(ego_v) > kEpsilon)
  // {
  //   return min_follow_time * PauseFactor;
  // }

  return std::max(min_follow_time, kEgoFollowTimeBuffer
                  /* ComputeSimilarSpeedFactor(obj_v, ego_v, ego_v)*/) *
         PauseFactor * k_speed_factor * k_dv_factor;
}

absl::Status CheckDeceleration(
    double lon_dist, std::string name_lead, std::string name_follow,
    double v_lead, double v_follow, double response_time, double lead_time,
    double max_allowed_decel, double min_lon_buffer,
    double acc_compensation_dis, std::string& ok_debug_info,
    double* hypo_deceleration, double* max_decel = nullptr) {
  const double v_diff = std::max(0.0, v_follow - v_lead);
  const double buffered_lon_dist =
      lon_dist - v_diff * (lead_time - (v_lead / max_allowed_decel)) -
      min_lon_buffer - acc_compensation_dis;
  if (buffered_lon_dist <= kEpsilon) {
    return absl::CancelledError(absl::StrFormat(
        "No space left for %s to decelerate behind %s. "
        "lon_dist:%.2f lead_time:%.2f "
        "v_lead:%.2f v_follow:%.2f v_diff:%.2f "
        "max_ald_decel:%.2f MinLonBuffer:%.2f acc_comp_dis:%.2f",
        name_follow, name_lead, lon_dist, lead_time, v_lead, v_follow, v_diff,
        max_allowed_decel, min_lon_buffer, acc_compensation_dis));
  }

  const double hypo_decel =
      (v_diff * (v_follow + v_lead)) / (2.0 * buffered_lon_dist);
  if (hypo_deceleration != nullptr) {
    *hypo_deceleration = hypo_decel;
  }

  bool decel_danger = hypo_decel > max_allowed_decel;

  auto debug_str = absl::StrFormat(
      "decel_danger:%d. (hypo_decel:-%.2f<max_allowed_decel:-%.2f) for %s "
      "behind %s. "
      "lon_dist:%.2f lead_time:%.2f "
      "v_lead:%.2f v_follow:%.2f v_diff:%.2f "
      "MinLonBuffer:%.2f acc_comp_dis:%.2f",
      decel_danger, hypo_decel, max_allowed_decel, name_follow, name_lead,
      lon_dist, lead_time, v_lead, v_follow, v_diff, min_lon_buffer,
      acc_compensation_dis);

  if (decel_danger) {
    return absl::CancelledError(debug_str);
  }
  if (max_decel != nullptr && hypo_decel > *max_decel) *max_decel = hypo_decel;

  ok_debug_info.clear();
  ok_debug_info = debug_str;

  return absl::OkStatus();
}

double getAccelerationDistanceCompensation(double offset_factor, double ego_acc,
                                           double obs_acc, double rel_vs,
                                           bool if_obs_front,
                                           bool is_lane_change_state,
                                           bool is_distance_shortening) {
  double rel_acc = ego_acc - obs_acc;
  double rel_abs_acc = std::fabs(ego_acc - obs_acc);
  double rel_acc_temp = std::clamp(rel_acc, -5.0, 2.0);
  double acc_safe_dis = 0;
  double pred_time = 3;
  int res_sign = 0;
  double non_change_pred_time = 1.414;
  double pred_time_min = 0.8;
  double pred_time_max = 1.0;

  if (if_obs_front) {
    if (rel_acc_temp > 0) {
      res_sign = 1;
    } else if (rel_acc_temp < 0 && ego_acc < 0.5) {
      res_sign = -1;
    } else {
      res_sign = 0;
    }
  } else {
    if (rel_acc_temp < 0) {
      res_sign = 1;
    } else if (rel_acc_temp > 0 && ego_acc > -0.5) {
      res_sign = -1;
    } else {
      res_sign = 0;
    }
  }

  if (res_sign == 1) {
    if (is_distance_shortening && !if_obs_front &&
        ((rel_vs > 3 && (ego_acc < -0.5 || obs_acc > 0.8)) || ego_acc < -1.5)) {
      non_change_pred_time = 1.6;
      pred_time_max = 1.414;
      pred_time_min = 1.26;
    } else {
      non_change_pred_time = 1.414;
      pred_time_max = 1.26;
      pred_time_min = 1.1;
    }

    pred_time = is_lane_change_state
                    ? Lerp(pred_time_max, kMaxCompensationFactor, pred_time_min,
                           kMinCompensationFactor, offset_factor, true)
                    : non_change_pred_time;
  } else if (res_sign == -1) {
    if (!if_obs_front) {
      non_change_pred_time = 1.51;
      pred_time_min = 1.6;
      pred_time_max = 1.68;
    } else {
      non_change_pred_time = 1.0;
      pred_time_min = 1.1;
      pred_time_max = 1.26;
    }
    pred_time = is_lane_change_state
                    ? Lerp(pred_time_min, kMaxCompensationFactor, pred_time_max,
                           kMinCompensationFactor, offset_factor, true)
                    : non_change_pred_time;
  } else {
    pred_time = 0;
  }

  acc_safe_dis =
      0.5 * res_sign * rel_abs_acc * pred_time * pred_time * offset_factor;

  return acc_safe_dis;
}

bool isLaneChanging(const st::LaneChangeStage& lc_state) {
  const bool is_lane_change_state =
      (lc_state == LaneChangeStage::LCS_EXECUTING) ||
      (lc_state == LaneChangeStage::LCS_PAUSE) ||
      (lc_state == LaneChangeStage::LCS_RETURN);
  return is_lane_change_state;
}

// check whether the object is on the specific lane
bool IsObjectOnLanePathByPose(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePath& lane_path,
                              const Vec2d& query_point, double* arc_len,
                              double lateral_error_buffer = 2.0) {
  if (lane_path.IsEmpty()) {
    return false;
  }
  return IsPointOnLanePathAtLevel(psmm, query_point, lane_path, arc_len,
                                  lateral_error_buffer);
}

// categorize all the moving vehicles on the target lane and sort them
bool SortMovingVehiclesOnTargetLane(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& target_lane_path,
    const SpacetimeTrajectoryManager& st_traj_mgr, const Vec2d& ego_pos,
    const double target_lane_half_width,
    std::vector<std::pair<std::string, double>>* object_ids_on_target_lane,
    std::string* nearest_follower_id) {
  // find ego_s on the target lane
  double ego_s = 0.0;
  if (!IsObjectOnLanePathByPose(psmm, target_lane_path, ego_pos, &ego_s,
                                10.0)) {
    return false;
  }

  // select all the moving vehicles on the target lane
  std::vector<std::pair<std::string, double>> target_object_ids_vec;
  double arc_len = 0.0;
  for (const auto* traj_ptr : st_traj_mgr.moving_object_trajs()) {
    if (IsVRU(traj_ptr)) {
      continue;
    }
    const auto& obj_proto = traj_ptr->planner_object().object_proto();
    double arc_len = 0.0;
    if (IsObjectOnLanePathByPose(psmm, target_lane_path,
                                 Vec2dFromProto(obj_proto.pos()), &arc_len,
                                 target_lane_half_width)) {
      target_object_ids_vec.emplace_back(
          std::make_pair(std::string(traj_ptr->object_id()), arc_len));
    }
  }

  // sort the moving vehicles by S in descending order
  std::sort(target_object_ids_vec.begin(), target_object_ids_vec.end(),
            [&](const auto& a, const auto& b) { return a.second > b.second; });

  // find the nearest follower on the target lane
  if (nearest_follower_id != nullptr) {
    *nearest_follower_id = "";
    for (const auto& obj_info : target_object_ids_vec) {
      if (obj_info.second < ego_s) {
        *nearest_follower_id = obj_info.first;
        break;
      }
    }
  }

  // output the vector
  if (object_ids_on_target_lane != nullptr) {
    object_ids_on_target_lane->clear();
    *object_ids_on_target_lane = std::move(target_object_ids_vec);
  }

  return true;
}

std::string GetLeadingObjTrajId(const SpacetimeTrajectoryManager& st_mgr,
                                const FrenetFrame& target_frenet_frame,
                                const std::vector<std::string>& leading_objs,
                                double ego_front_to_ra) {
  double min_s = std::numeric_limits<double>::max();
  std::string leading_traj_id = "";
  for (const auto& lead_obj : leading_objs) {
    const auto obj_traj = st_mgr.FindTrajectoryById(lead_obj);
    const auto states = obj_traj->states();
    const auto fbox =
        obj_traj->is_stationary()
            ? target_frenet_frame.QueryFrenetBoxAt(states.front().box)
            : target_frenet_frame.QueryFrenetBoxAt(states.back().box);
    if (!fbox.ok()) {
      continue;
    }
    if (fbox->s_min < min_s) {
      min_s = fbox->s_min;
      leading_traj_id = lead_obj;
    }
  }
  return leading_traj_id;
}

bool ObjectHasLCIntentionInDiffSide(
    const FrenetFrame& target_frenet_frame,
    const SpacetimeObjectTrajectory& obj_traj,
    const ObjectHistoryManager& obs_history_mgr,
    const st::FrenetBox& ego_cur_frenet_box,
    const st::FrenetBox& obj_cur_frenet_box, const double ego_lon_v,
    const double obj_lon_v, const double obj_target_normal_cos,
    const double target_lane_width, const double ego_enter_target_time,
    const double obj_front_extension, const double min_lon_buffer,
    const bool ego_lc_left, std::string& debug_str) {
  const double kObjPushToBoundaryMargin = 0.2;
  const double kObjPushToBoundaryHeadingThreshold = 0.021;  // 1.2deg
  const unsigned int kObjPushToBoundaryCntMax = 8;
  const unsigned int kObjPushToBoundaryCntReduceByTurnLight = 4;
  const double kObjPushToBoundaryProbability = 0.6;
  const double kObjEnterTargetLaneTimeMax = 2.0;
  const double half_target_lane_width = 0.5 * target_lane_width;

  auto check_obj_push_to_boundary = [&](const int idx,
                                        const st::FrenetBox& obj_frenet_box,
                                        const double obj_sin_heading) -> bool {
    const bool obj_is_in_diff_side =
        !HasFullyEnteredTargetLane(obj_frenet_box,
                                   0.5 * obj_frenet_box.width()) &&
        (obj_frenet_box.center_l() * ego_cur_frenet_box.center_l() < 0.0);
    const double obj_abs_l_min =
        std::min(fabs(obj_frenet_box.l_max), fabs(obj_frenet_box.l_min));
    const bool obj_has_enough_offset =
        obj_abs_l_min < (0.5 * target_lane_width + kObjPushToBoundaryMargin);
    const bool obj_is_heading_to_target =
        obj_frenet_box.center_l() > 0.0
            ? (obj_sin_heading < kObjPushToBoundaryHeadingThreshold)
            : (obj_sin_heading > -kObjPushToBoundaryHeadingThreshold);
    // Log2DDS::LogDataV2(
    //     "safety_lc_intention_debug",
    //     absl::StrFormat("idx:%d, res:%d,%d,%d, L:%.3f,%.3f, sin:%.3f", idx,
    //                     obj_is_in_diff_side, obj_has_enough_offset,
    //                     obj_is_heading_to_target, obj_frenet_box.l_min,
    //                     obj_frenet_box.l_max, obj_sin_heading));
    return obj_is_in_diff_side && obj_has_enough_offset &&
           obj_is_heading_to_target;
  };

  debug_str = absl::StrFormat(
      "obj_id:%s, ego_s:%.3f,%.3f, ego_l:%.3f,%.3f, obj_s:%.3f,%.3f, "
      "obj_l:%.3f,%.3f",
      std::string(obj_traj.object_id()).c_str(), ego_cur_frenet_box.s_min,
      ego_cur_frenet_box.s_max, ego_cur_frenet_box.l_min,
      ego_cur_frenet_box.l_max, obj_cur_frenet_box.s_min,
      obj_cur_frenet_box.s_max, obj_cur_frenet_box.l_min,
      obj_cur_frenet_box.l_max);

  // Determine the counts threshold to check the history of the object
  const bool obj_has_lc_light = ego_lc_left ? obj_traj.planner_object()
                                                  .object_proto()
                                                  .obstacle_light()
                                                  .right_turn_lights()
                                            : obj_traj.planner_object()
                                                  .object_proto()
                                                  .obstacle_light()
                                                  .left_turn_lights();
  int kObjPushToBoundaryCntThreashold = kObjPushToBoundaryCntMax;
  if (obj_has_lc_light) {
    kObjPushToBoundaryCntThreashold -= kObjPushToBoundaryCntReduceByTurnLight;
    kObjPushToBoundaryCntThreashold =
        std::max(0, kObjPushToBoundaryCntThreashold);
  }
  debug_str += absl::StrFormat(", light:%d, cnt_thr:%d", obj_has_lc_light,
                               kObjPushToBoundaryCntThreashold);

  unsigned int total_his_cnt = 1, valid_his_cnt = 0;
  double valid_ratio = 0.0;

  // Check if the object intends to change lane in the opposite direction now
  bool current_obj_push_to_boundary = false;
  if (check_obj_push_to_boundary(total_his_cnt - 1, obj_cur_frenet_box,
                                 obj_target_normal_cos)) {
    ++valid_his_cnt;
    current_obj_push_to_boundary = true;
  }
  debug_str += absl::StrFormat(", curr:%d", current_obj_push_to_boundary);

  if (kObjPushToBoundaryCntThreashold > 0) {
    // Check if the historical objects had the intention
    const auto* his_obj_ptr =
        obs_history_mgr.GetObjHistory(obj_traj.planner_object().id());
    if (his_obj_ptr == nullptr || his_obj_ptr->Empty()) {
      debug_str = absl::StrFormat("Intention(0): ") + debug_str;
      return false;
    }
    const auto& his_obj = his_obj_ptr->GetFrames();
    for (auto riter = his_obj.rbegin(); riter != his_obj.rend(); ++riter) {
      ++total_his_cnt;
      const auto& obj_box = st::Box2d(riter->object_proto.bounding_box());
      const auto obj_frenet_box_or_not =
          target_frenet_frame.QueryFrenetBoxWithHeading(obj_box, M_PI);
      if (!obj_frenet_box_or_not.ok()) {
        continue;
      }
      const auto obj_frenet_box = obj_frenet_box_or_not.value();
      const auto obj_sin_heading = obj_box.tangent().Dot(
          target_frenet_frame.InterpolateTangentByS(obj_frenet_box.center_s())
              .Rotate(M_PI_2));
      if (check_obj_push_to_boundary(total_his_cnt - 1, obj_frenet_box,
                                     obj_sin_heading)) {
        ++valid_his_cnt;
      }
      if (total_his_cnt > kObjPushToBoundaryCntThreashold) {
        break;
      }
    }
  }
  if (total_his_cnt > kObjPushToBoundaryCntThreashold) {
    valid_ratio = total_his_cnt == 0
                      ? 0.0
                      : static_cast<double>(valid_his_cnt) / total_his_cnt;
  }
  debug_str += absl::StrFormat(", his_valid:%.2f=%d/%d", valid_ratio,
                               valid_his_cnt, total_his_cnt);
  if (valid_ratio + kEpsilon < kObjPushToBoundaryProbability) {
    debug_str = absl::StrFormat("Intention(0): ") + debug_str;
    return false;
  }

  // Check if the current lon dist is unsafe
  bool cur_obj_enter_target_lane_lon_ignore =
      ego_cur_frenet_box.s_max + obj_front_extension <
          obj_cur_frenet_box.s_min ||
      obj_cur_frenet_box.s_max + obj_front_extension < ego_cur_frenet_box.s_min;
  if (!cur_obj_enter_target_lane_lon_ignore) {
    debug_str = absl::StrFormat("Intention(1): ") + debug_str;
    return true;
  }

  // Estimate the time for the object entering the target lane
  const double obj_enter_target_time = std::max(
      0.0, kObjEnterTargetLaneTimeMax *
               (fabs(obj_cur_frenet_box.center_l()) - half_target_lane_width) /
               half_target_lane_width);
  const double min_enter_target_time =
      std::min(obj_enter_target_time, ego_enter_target_time);

  // Estimate obj preview frenet_s
  const double obj_lon_travelling_dist = min_enter_target_time * obj_lon_v;
  const double obj_preview_s_max =
      obj_cur_frenet_box.s_max + obj_lon_travelling_dist;
  const double obj_preview_s_min =
      obj_cur_frenet_box.s_min + obj_lon_travelling_dist;

  // Estimate ego preview frenet_s
  const double ego_lon_travelling_dist = min_enter_target_time * ego_lon_v;
  const double ego_preview_s_max =
      ego_cur_frenet_box.s_max + ego_lon_travelling_dist;
  const double ego_preview_s_min =
      ego_cur_frenet_box.s_min + ego_lon_travelling_dist;

  // Check if the intention can be ignored
  double preview_factor = ad_byd::planning::math::interp1_inc(
      ENTER_TARGET_TIME_VEC, ENTER_TARGET_PREVIEW_FACTOR_VEC,
      min_enter_target_time);
  double front_extension_threshold =
      std::fmax(min_lon_buffer, obj_front_extension * preview_factor);
  bool preview_obj_enter_target_lane_lon_ignore =
      ego_preview_s_max + front_extension_threshold < obj_preview_s_min ||
      obj_preview_s_max + front_extension_threshold < ego_preview_s_min;

  debug_str += absl::StrFormat(
      ", obj_et:%.3f, min_et:%.3f, ego_prs:%.3f,%.3f, obj_prs:%.3f,%.3f, "
      "prf:%.3f, Fthre:%.3f(%.3f,%.3f)",
      obj_enter_target_time, min_enter_target_time, ego_preview_s_min,
      ego_preview_s_max, obj_preview_s_min, obj_preview_s_max, preview_factor,
      front_extension_threshold, obj_front_extension, min_lon_buffer);
  bool obj_lc_intention_res = !preview_obj_enter_target_lane_lon_ignore;
  debug_str =
      absl::StrFormat("Intention(%d): ", obj_lc_intention_res) + debug_str;

  return obj_lc_intention_res;
}

void CheckForConesRidingMiddleLine(
    const FrenetFrame& target_frenet_frame,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const st::FrenetBox& ego_cur_frenet_box, const bool lc_left,
    std::vector<std::string>& cone_ids_list, double& rel_ds_to_ego,
    std::string& debug_str) {
  cone_ids_list.clear();
  rel_ds_to_ego = DBL_MAX;
  debug_str = "ride_line_uu:";
  std::pair<double, double> cones_s_range = std::make_pair(DBL_MAX, DBL_MIN);
  std::string sub_debug_str = absl::StrFormat(
      "|EgoL:%.3f,%.3f", ego_cur_frenet_box.l_min, ego_cur_frenet_box.l_max);
  for (const auto* traj_ptr : st_traj_mgr.stationary_object_trajs()) {
    // 1. it must be construction obstacle
    bool is_construction =
        IsConstructionObs(traj_ptr) && IsSizeQualified(traj_ptr);
    if (!is_construction) {
      continue;
    }
    // 2. it must be riding the middle line
    ASSIGN_OR_CONTINUE(
        const auto obj_cur_frenet_box,
        target_frenet_frame.QueryFrenetBoxAtContour(traj_ptr->contour()));
    double rel_ds = 0.0;
    getLongOverlap(ego_cur_frenet_box, obj_cur_frenet_box, rel_ds);
    bool ride_middle_line = isStaticObsRideLine(
        ego_cur_frenet_box, obj_cur_frenet_box, lc_left, rel_ds);
    std::string obj_id = std::string(traj_ptr->object_id());
    sub_debug_str +=
        absl::StrFormat("|Obj:%s,ds:%.3f,L:%.3f,%.3f", obj_id.c_str(), rel_ds,
                        obj_cur_frenet_box.l_min, obj_cur_frenet_box.l_max);
    if (!ride_middle_line) {
      continue;
    }
    // 3. it is the cone riding middle line
    debug_str += (cone_ids_list.empty() ? "" : "->");
    debug_str += obj_id;
    cone_ids_list.emplace_back(obj_id);
    cones_s_range.first =
        std::min(cones_s_range.first, obj_cur_frenet_box.s_min);
    cones_s_range.second =
        std::max(cones_s_range.second, obj_cur_frenet_box.s_max);
  }
  if (!cone_ids_list.empty()) {
    double lon_overlap_with_cones =
        std::min(ego_cur_frenet_box.s_max, cones_s_range.second) -
        std::max(ego_cur_frenet_box.s_min, cones_s_range.first);
    bool is_cones_front = cones_s_range.first > ego_cur_frenet_box.s_max;
    rel_ds_to_ego = lon_overlap_with_cones > -kEpsilon
                        ? 0.0
                        : (is_cones_front ? -lon_overlap_with_cones
                                          : lon_overlap_with_cones);
    sub_debug_str += absl::StrFormat("|DS:%.3f", rel_ds_to_ego);
  }
  debug_str += sub_debug_str;
}

}  // namespace

absl::Status CheckLaneChangeSafety(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const PathSlBoundary& sl_boundary,
    const ApolloTrajectoryPointProto& start_point,
    const std::vector<ApolloTrajectoryPointProto>& ego_traj_pts,
    const std::vector<std::string>& leading_traj_ids,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const FrenetFrame& target_frenet_frame, double speed_limit,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ObjectHistoryManager& obs_history_mgr,
    const mapping::LanePath& target_lane_path_ext,
    const VehicleParamsProto& vehicle_params, LaneChangeStyle lc_style,
    const std::pair<PathResponseStyle, SpeedResponseStyle>& prev_resp_style,
    const st::LaneChangeStage& lc_state,
    const st::LaneChangeStage& prev_lc_stage, const bool is_congestion_scene,
    absl::Duration path_look_ahead_duration, int plan_id,
    TrajEvalInfo* eval_info,
    LaneChangeStylePostDeciderSceneInfo* target_front_obj_scene_info,
    LaneChangeStylePostDeciderSceneInfos* scene_infos,
    int* scene_cones_riding_line_frames_result,
    absl::flat_hash_set<std::string>& gaming_lc_obs_set, bool is_closed_ramp,
    bool is_on_highway, const std::vector<double>* stop_s_vec) {
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive = vehicle_params.vehicle_drive_params();
  auto& follower_set = eval_info->follower_set;
  auto& leader_set = eval_info->leader_set;
  auto& follower_max_decel = eval_info->follower_max_decel;
  auto& leader_max_decel = eval_info->leader_max_decel;
  auto& unsafe_object_id = eval_info->unsafe_object_id;
  auto& status_code = eval_info->status_code;

  auto& follower_debug_info = eval_info->follower_debug_info;
  auto& leader_debug_info = eval_info->leader_debug_info;

  // After the scene is detected (with a non-zero frame count), the frame count
  // will be accumulated.
  // If it is detected again, the count will be reset to one.
  // Otherwise, this count may be invalid due to a timeout and be reset to zero.
  auto& cones_riding_line_frames = *scene_cones_riding_line_frames_result;

  double target_front_obj_scene_s_min = DBL_MAX;
  LaneChangeStylePostDeciderSceneInfo* target_front_obj_scene_info_ptr =
      target_front_obj_scene_info;
  if (target_front_obj_scene_info == nullptr) {
    target_front_obj_scene_info_ptr =
        std::make_shared<LaneChangeStylePostDeciderSceneInfo>().get();
  }
  LaneChangeStylePostDeciderSceneInfos* scene_infos_ptr = scene_infos;
  if (scene_infos == nullptr) {
    scene_infos_ptr =
        std::make_shared<LaneChangeStylePostDeciderSceneInfos>().get();
  }

  double ego_slow_front_obj_scene_s_min = DBL_MAX;
  LaneChangeStylePostDeciderSceneInfo ego_slow_front_obj_scene_info;
  double ego_static_front_obj_scene_s_min = DBL_MAX;
  LaneChangeStylePostDeciderSceneInfo ego_static_front_obj_scene_info;

  const bool is_lane_change_state = isLaneChanging(lc_state);

  const bool is_lc_state_prev = isLaneChanging(prev_lc_stage);

  double kEgoLeadTimeBuffer =
      is_lc_state_prev ? kEgoLeadTimeBufferProcess : kEgoLeadTimeBufferPrepare;
  double kEgoFollowTimeBuffer = is_lc_state_prev ? kEgoFollowTimeBufferProcess
                                                 : kEgoFollowTimeBufferPrepare;

  const double preview_time = 0;
  const double ego_v = start_point.v() + start_point.a() * preview_time;
  const double ego_speed_kph = Mps2Kph(ego_v);

  double min_lon_buffer = kMinLonBufferToFront;

  int ego_traj_size = ego_traj_pts.size();

  // Get the lane width
  double cur_lane_width = kDefaultLaneWidth;
  double cur_half_lane_width = kDefaultHalfLaneWidth;
  if (ego_traj_size > 0) {
    const auto lane_ptr = psmm.FindLaneByIdOrNull(
        drive_passage.lane_path().ArclengthToLanePoint(0.0).lane_id());
    if (lane_ptr) {
      cur_lane_width = lane_ptr->GetWidthAtAccumS(0.0);
      cur_half_lane_width = 0.5 * cur_lane_width;
    }
  }

  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(ego_traj_size);
  int ego_enter_target_idx = -1;
  Box2d ego_enter_target_box;
  double ego_half_width = vehicle_geom.width() * 0.5;
  double ego_half_length = vehicle_geom.length() * 0.5;
  std::ostringstream dynamic_obs_debug;
  dynamic_obs_debug.str("");

  std::ostringstream static_obs_debug;
  static_obs_debug.str("");

  std::vector<PathPoint> raw_path_points;
  raw_path_points.reserve(ego_traj_size);
  for (int i = 0; i < ego_traj_size; ++i) {
    const auto& traj_pt = ego_traj_pts[i];
    const Vec2d traj_pos = Vec2dFromApolloTrajectoryPointProto(traj_pt);
    const auto ego_sl = target_frenet_frame.XYToSL(traj_pos);
    raw_path_points.push_back(traj_pt.path_point());
    ego_boxes.push_back(
        ComputeAvBox(traj_pos, traj_pt.path_point().theta(), vehicle_geom));
    if (ego_enter_target_idx == -1 &&
        HasFullyEnteredTargetLane(ego_sl.l, ego_half_width)) {
      ego_enter_target_idx = i;
      ego_enter_target_box = ego_boxes.back();
    }
  }
  const Vec2d start_box = Vec2dFromApolloTrajectoryPointProto(start_point);
  Box2d start_pos =
      ComputeAvBox(start_box, start_point.path_point().theta(), vehicle_geom);
  ASSIGN_OR_RETURN(
      const auto ego_start_frenet_box,
      target_frenet_frame.QueryFrenetBoxAt(start_pos),
      _ << "Cannot project the current ego box onto drive passage.");
  const auto ego_start_target_normal_cos = start_pos.tangent().Dot(
      target_frenet_frame.InterpolateTangentByS(ego_start_frenet_box.center_s())
          .Rotate(M_PI_2));
  dynamic_obs_debug << "/***/"
                    << absl::StrFormat("lane_width:%.2f", cur_lane_width);
  static_obs_debug << "/***/"
                   << absl::StrFormat("enter_target_idx:%d",
                                      ego_enter_target_idx);

  constexpr int kMustEnterTargetStep = 0.95 * kTrajectorySteps;
  if (ego_enter_target_idx == -1 ||
      ego_enter_target_idx > kMustEnterTargetStep) {
    double min_stop_s = DBL_MAX;
    if (stop_s_vec && !stop_s_vec->empty()) {
      min_stop_s = *std::min_element(stop_s_vec->begin(), stop_s_vec->end());
    }
    const auto fake_traj_pts_or = GenerateLatQuinticLonConstAccTrajToRefL(
        drive_passage, sl_boundary, st_traj_mgr, leading_traj_ids, start_point,
        vehicle_geom.front_edge_to_center(), min_stop_s, plan_id,
        "Init traj not entering the target lane!");
    if (!fake_traj_pts_or.ok()) {
      // If the trajectory does not enter the target lane in time, consider as
      // unsafe to enter now.
      status_code = PlannerStatusProto::TRAJECTORY_NOT_ENTERING_TARGET_LANE;
      unsafe_object_id = GetLeadingObjTrajId(
          st_traj_mgr, target_frenet_frame, leading_traj_ids,
          vehicle_geom.front_edge_to_center());
      return absl::CancelledError(absl::StrFormat(
          "Trajectory not entering the target lane "
          "(enter_target_idx:%d, leading_id:%s). Fake failed:%s",
          ego_enter_target_idx, unsafe_object_id,
          fake_traj_pts_or.status().message()));
    } else {
      const auto& fake_traj_pts = fake_traj_pts_or.value();
      ego_traj_size = fake_traj_pts.size();
      ego_boxes.clear();
      ego_enter_target_idx = -1;
      for (int i = 0; i < ego_traj_size; ++i) {
        const auto& traj_pt = fake_traj_pts[i];
        const Vec2d traj_pos = Vec2dFromApolloTrajectoryPointProto(traj_pt);
        const auto ego_sl = target_frenet_frame.XYToSL(traj_pos);
        ego_boxes.push_back(
            ComputeAvBox(traj_pos, traj_pt.path_point().theta(), vehicle_geom));
        if (ego_enter_target_idx == -1 &&
            HasFullyEnteredTargetLane(ego_sl.l, ego_half_width)) {
          ego_enter_target_idx = i;
          ego_enter_target_box = ego_boxes.back();
        }
      }
      if (ego_enter_target_idx == -1 ||
          ego_enter_target_idx > kMustEnterTargetStep) {
        // If the fake trajectory does not enter the target lane in time,
        // consider as
        // unsafe to enter now.
        status_code = PlannerStatusProto::TRAJECTORY_NOT_ENTERING_TARGET_LANE;
        unsafe_object_id = GetLeadingObjTrajId(
            st_traj_mgr, target_frenet_frame, leading_traj_ids,
            vehicle_geom.front_edge_to_center());
        return absl::CancelledError(
            absl::StrFormat("Fake trajectory not entering the target lane "
                            "(enter_target_idx:%d, leading_id:%s).",
                            ego_enter_target_idx, unsafe_object_id));
      }
    }
  }
  auto ego_cur_box = ego_boxes.front();
  ASSIGN_OR_RETURN(
      const auto ego_cur_frenet_box,
      target_frenet_frame.QueryFrenetBoxAt(ego_cur_box),
      _ << "Cannot project the current ego box onto drive passage.");
  ASSIGN_OR_RETURN(
      const auto ego_enter_target_frenet_box,
      target_frenet_frame.QueryFrenetBoxAt(ego_enter_target_box),
      _ << "Cannot project the enter-target ego box onto drive passage.");

  auto ego_corner = ego_boxes.front().GetCornersCounterClockwise();
  auto ego_front_vct =
      std::vector<st::Vec2d>{ego_corner.front(), ego_corner.back()};
  ASSIGN_OR_RETURN(
      const auto ego_front_frenet_box,
      target_frenet_frame.QueryFrenetBoxAtPoints(ego_front_vct),
      _ << "Cannot project the current ego box onto drive passage.");

  const auto ego_target_tangent_cos = ego_cur_box.tangent().Dot(
      target_frenet_frame.InterpolateTangentByS(ego_cur_frenet_box.center_s()));
  const auto ego_target_normal_cos = ego_cur_box.tangent().Dot(
      target_frenet_frame.InterpolateTangentByS(ego_cur_frenet_box.center_s())
          .Rotate(M_PI_2));

  const double ego_lon_v = ego_v * ego_target_tangent_cos;
  const double ego_lat_v = ego_v * ego_target_normal_cos;
  const double ego_lon_v_kph = Mps2Kph(ego_lon_v);
  const double ego_acc = start_point.a();
  const double ego_lon_acc = ego_acc * ego_target_tangent_cos;
  const double ego_lat_acc = ego_acc * ego_target_normal_cos;

  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  const double path_start_time_offset =
      absl::ToDoubleSeconds(path_look_ahead_duration);
  double lc_style_factor = FLAGS_planner_enable_lc_style_params
                               ? GetLaneChangeStyleFactor(lc_style)
                               : 1.0;
  double response_style_factor =
      is_lc_state_prev ? GetResponseStyleFactor(lc_style, prev_resp_style)
                       : 1.0;
  lc_style_factor *= response_style_factor;
  const bool is_radical_lowspeed_congestion_scene =
      is_lc_state_prev && is_congestion_scene &&
      ego_v < kEgoLowSpeedMpsThreshold;
  double congestion_factor = is_radical_lowspeed_congestion_scene ? 0.5 : 1.0;
  double conserv_base = lc_style_factor * congestion_factor *
                        getLatOffsetFactor(ego_cur_frenet_box, ego_half_width,
                                           0.2, cur_lane_width);

  dynamic_obs_debug << absl::StrFormat(",style_ftr:%.2f,t_offset:%.2f",
                                       lc_style_factor, path_start_time_offset);

  double conserv = conserv_base;

  bool ego_occupy_target =
      isEgoFullyOccupyTargetLane(ego_cur_frenet_box, ego_speed_kph, lc_left);
  bool ego_centered_in_target_lane = HasFullyCenteredInTargetLane(
      ego_cur_frenet_box, 0.5 * ego_cur_frenet_box.width());

  // Create ego info for lc style decider
  LaneChangeStyleDeciderObjectInfo ego_lcs_decider_info(
      "ego", ego_lon_v, ego_lat_v, ego_cur_frenet_box);

  // find the nearest moving leader
  // & the nearest stationary leader
  // & the nearest follower on the target lane
  std::string nearest_moving_leader_id = "";
  std::string nearest_stationary_leader_id = "";
  for (const auto& leading_traj_id : leading_traj_ids) {
    const auto leading_traj_ptr =
        st_traj_mgr.FindTrajectoryById(leading_traj_id);
    if (leading_traj_ptr != nullptr) {
      if (nearest_moving_leader_id == "" &&
          !leading_traj_ptr->is_stationary()) {
        nearest_moving_leader_id = std::string(leading_traj_ptr->object_id());
      } else if (nearest_stationary_leader_id == "" &&
                 leading_traj_ptr->is_stationary()) {
        nearest_stationary_leader_id =
            std::string(leading_traj_ptr->object_id());
      }
      if (nearest_moving_leader_id != "" &&
          nearest_stationary_leader_id != "") {
        break;
      }
    }
  }
  std::vector<std::pair<std::string, double>> sorted_object_ids_on_target_lane;
  std::string nearest_follower_id = "";
  SortMovingVehiclesOnTargetLane(psmm, target_lane_path_ext, st_traj_mgr,
                                 ego_cur_box.center(), cur_half_lane_width,
                                 &sorted_object_ids_on_target_lane,
                                 &nearest_follower_id);
  if (nearest_follower_id != "") {
    follower_set.insert(nearest_follower_id);
  }

  // get box and frenet-box of the leading
  bool is_moving_leader_box_ready = false,
       is_stationary_leader_box_ready = false;
  Box2d moving_leader_box, stationary_leader_box;
  FrenetBox moving_leader_frenet_box, stationary_leader_frenet_box;
  if (nearest_moving_leader_id != "") {
    leader_set.insert(nearest_moving_leader_id);
    const auto leading_trajs =
        st_traj_mgr.FindTrajectoriesByObjectId(nearest_moving_leader_id);
    if (leading_trajs.size() > 0) {
      const auto leading_traj = *leading_trajs[0];
      const auto obj_cur_box =
          FindPredictedObjectBox(leading_traj.states(), path_start_time_offset);
      if (obj_cur_box.ok()) {
        moving_leader_box = *obj_cur_box;
        const auto obj_cur_frenet_box =
            target_frenet_frame.QueryFrenetBoxWithHeading(*obj_cur_box);
        if (obj_cur_frenet_box.ok()) {
          moving_leader_frenet_box = *obj_cur_frenet_box;
          is_moving_leader_box_ready = true;
        }
      }
    }
  }
  if (nearest_stationary_leader_id != "") {
    const auto leading_trajs =
        st_traj_mgr.FindTrajectoriesByObjectId(nearest_stationary_leader_id);
    if (leading_trajs.size() > 0) {
      const auto leading_traj = *leading_trajs[0];
      const auto obj_cur_box =
          FindPredictedObjectBox(leading_traj.states(), path_start_time_offset);
      if (obj_cur_box.ok()) {
        stationary_leader_box = *obj_cur_box;
        const auto obj_cur_frenet_box =
            target_frenet_frame.QueryFrenetBoxWithHeading(*obj_cur_box);
        if (obj_cur_frenet_box.ok()) {
          stationary_leader_frenet_box = *obj_cur_frenet_box;
          is_stationary_leader_box_ready = true;
        }
      }
    }
  }

  // Estimate the target lateral offset if lc pause
  constexpr double kComfortLaneChangeCancelLatAccel = 0.25;  // m/s^2.
  constexpr double kLCPauseDelaySec = 0.4;                   // s.
  double kinematic_lc_pause_offset =
      ego_cur_frenet_box.center_l() +
      std::copysign(Sqr(ego_lat_v) * 0.5 / kComfortLaneChangeCancelLatAccel,
                    ego_lat_v);
  double sim_lc_pause_offset = ego_cur_frenet_box.center_l();
  KinematicPauseOffsetSimulation(drive_passage, start_point, vehicle_geom,
                                 sim_lc_pause_offset, lc_left);
  double geom_lc_pause_offset =
      ego_cur_frenet_box.center_l() +
      ego_v * (1.0 - ego_target_tangent_cos) / ego_target_normal_cos +
      ego_v * ego_target_normal_cos * kLCPauseDelaySec;
  double lc_pause_offset =
      lc_left ? std::max(kinematic_lc_pause_offset,
                         std::max(sim_lc_pause_offset, geom_lc_pause_offset))
              : std::min(kinematic_lc_pause_offset,
                         std::min(sim_lc_pause_offset, geom_lc_pause_offset));
  dynamic_obs_debug << absl::StrFormat(",lcp_offset:%.3f", lc_pause_offset);

  // Create the instance of LaneChangeSafetyChecker gaming
  LaneChangeSafetyChecker gaming_lc_safety_checker(&vehicle_geom,
                                                   &vehicle_drive);
  const DiscretizedPath raw_discretized_path(raw_path_points);
  const LaneChangeSafetyGamingInput gaming_lc_safety_input = {
      .traj_mgr = &st_traj_mgr,
      .ego_path = &raw_discretized_path,
      .plan_start_v = start_point.v(),
      .plan_start_a = start_point.a(),
      .plan_start_j = start_point.j(),
      .drive_passage = &drive_passage,
      .speed_limit = speed_limit,
      .plan_id = plan_id,
  };

  // **********************Check for moving objects.************************
  double temp_min_front_dist = ad_byd::planning::math::interp1_inc(
      V_EGO_VEHICLE_VEC, MIN_FRONT_DISTANCE_VEC, ego_lon_v_kph);
  double temp_min_back_dist = ad_byd::planning::math::interp1_inc(
      V_EGO_VEHICLE_VEC, MIN_BACK_DISTANCE_VEC, ego_lon_v_kph);

  const double ego_enter_target_time =
      ego_enter_target_idx * kTrajectoryTimeStep;

  const double closet_corner_lat_offset =
      lc_left ? std::abs(std::min(0.0, ego_cur_frenet_box.l_max))
              : std::abs(std::max(0.0, ego_cur_frenet_box.l_min));

  // Prevent frequent jumps in the lane change status
  // caused by outer pockets during the pause process
  double pause_factor =
      (prev_lc_stage == LaneChangeStage::LCS_PAUSE && ego_enter_target_idx > 10)
          ? 1.1
          : 1.0;

  st::FrenetBox frnt_static_filter_obs_box{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  std::string frnt_static_filter_obs_id = "";

  st::FrenetBox frnt_obs_box{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  st::FrenetBox rear_obs_box{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  std::string frnt_obs_id = "";
  std::string rear_obs_id = "";

  // add can not lc return scene
  // bool theta_can_not_return =
  //     (std::fabs(ego_cur_frenet_box.center_l() +
  //                (ego_v / ego_target_normal_cos -
  //                 ego_target_tangent_cos * ego_v / ego_target_normal_cos) +
  //                (prev_lc_stage == LaneChangeStage::LCS_EXECUTING
  //                     ? (ego_v * ego_target_normal_cos * 0.4)
  //                     : 0))) < (cur_lane_width * 0.3 + ego_half_width);
  // bool dis_can_not_return =
  //     std::fmin(std::fabs(ego_cur_frenet_box.l_min),
  //               std::fabs(ego_cur_frenet_box.l_max)) < cur_lane_width / 3;
  // const bool can_not_return = theta_can_not_return || dis_can_not_return;

  // disable can not lc return scene
  const bool theta_can_not_return = false;
  const bool dis_can_not_return = false;
  const bool can_not_return = false;

  for (const auto* traj_ptr : st_traj_mgr.moving_object_trajs()) {
    dynamic_obs_debug << "-->Mobj:" << traj_ptr->object_id();

    auto obj_cur_box_or_not =
        FindPredictedObjectBox(traj_ptr->states(), path_start_time_offset);
    if (!obj_cur_box_or_not.ok()) {
      dynamic_obs_debug << "-boxfail";
      continue;
    }
    const auto obj_cur_box = obj_cur_box_or_not.value();
    if (traj_ptr->states().empty()) {
      dynamic_obs_debug << "-notraj";
    }
    CHECK(!traj_ptr->states().empty())
        << "No prediction state for trajectory " << traj_ptr->traj_id();
    auto obj_cur_frenet_box_or_not =
        target_frenet_frame.QueryFrenetBoxWithHeading(obj_cur_box, M_PI);
    if (!obj_cur_frenet_box_or_not.ok()) {
      dynamic_obs_debug << "-fnboxfail";
      continue;
    }
    const auto obj_cur_frenet_box = obj_cur_frenet_box_or_not.value();

    const bool is_vehicle = IsVehicle(traj_ptr);
    const bool is_big_vehicle = IsBigVehicle(traj_ptr);
    const bool is_vru = IsVRU(traj_ptr);

    const bool if_obs_front =
        (ego_cur_frenet_box.s_max < obj_cur_frenet_box.s_min);
    const bool if_obs_rear =
        (obj_cur_frenet_box.s_max < ego_cur_frenet_box.s_min);

    double rel_ds, rel_dl = 0;
    double long_overlap =
        getLongOverlap(ego_cur_frenet_box, obj_cur_frenet_box, rel_ds);
    double lat_overlap =
        getLatOverlap(ego_cur_frenet_box, obj_cur_frenet_box, rel_dl);
    double lat_overlap_lc = getLatOverlapWithLcDirection(
        ego_front_frenet_box, obj_cur_frenet_box, rel_dl);

    const bool if_obs_left =
        (ego_cur_frenet_box.l_max < obj_cur_frenet_box.l_min);

    bool obs_in_target_lane = HasFullyEnteredTargetLane(
        obj_cur_frenet_box, 0.5 * obj_cur_frenet_box.width());
    bool obs_centered_in_target_lane = HasFullyCenteredInTargetLane(
        obj_cur_frenet_box, 0.5 * obj_cur_frenet_box.width());

    const auto obj_preview_pnt = target_frenet_frame.XYToSL(
        traj_ptr->states().size() < kObjReliableTrajectoryIndexMax
            ? traj_ptr->states().back().traj_point->pos()
            : (traj_ptr->states().begin() + kObjReliableTrajectoryIndexMax - 1)
                  ->traj_point->pos());
    bool obj_is_leaving_target =
        ObjIsLeavingTargetLane(obj_cur_frenet_box, obj_preview_pnt, 0.3);
    const auto obj_near_field_preview_pnt = target_frenet_frame.XYToSL(
        traj_ptr->states().size() < kObjMostReliableTrajectoryIndexMax
            ? traj_ptr->states().back().traj_point->pos()
            : (traj_ptr->states().begin() + kObjMostReliableTrajectoryIndexMax -
               1)
                  ->traj_point->pos());
    bool obj_is_leaving_near_field_target = ObjIsLeavingTargetLane(
        obj_cur_frenet_box, obj_near_field_preview_pnt, 0.15);
    const bool obj_lc_left =
        (obj_preview_pnt.l > obj_cur_frenet_box.center_l());
    const auto obj_shorter_preview_pnt = target_frenet_frame.XYToSL(
        traj_ptr->states().size() < kObjCutoutSoonPreviewIndex
            ? traj_ptr->states().back().traj_point->pos()
            : (traj_ptr->states().begin() + kObjCutoutSoonPreviewIndex - 1)
                  ->traj_point->pos());
    bool obj_is_cutout_soon =
        !HasFullyEnteredTargetLane(obj_shorter_preview_pnt.l,
                                   0.5 * obj_cur_frenet_box.width()) &&
        ((lc_left &&
          obj_shorter_preview_pnt.l > obj_cur_frenet_box.center_l()) ||
         (!lc_left &&
          obj_shorter_preview_pnt.l < obj_cur_frenet_box.center_l()));

    const auto obj_target_tangent_cos =
        obj_cur_box.tangent().Dot(target_frenet_frame.InterpolateTangentByS(
            obj_cur_frenet_box.center_s()));
    const auto obj_target_normal_cos = obj_cur_box.tangent().Dot(
        target_frenet_frame.InterpolateTangentByS(obj_cur_frenet_box.center_s())
            .Rotate(M_PI_2));

    const double obj_v =
        ego_cur_frenet_box.s_max < obj_cur_frenet_box.s_min
            ? traj_ptr->planner_object().pose().v()
            : EstimateObjectSpeed(traj_ptr->planner_object(), preview_time);
    const double obj_lon_v = obj_v * obj_target_tangent_cos;
    const double obj_lat_v = obj_v * obj_target_normal_cos;
    const double rel_vs = (obj_lon_v - ego_lon_v);
    const double abs_rel_vs_kph = std::fabs(Mps2Kph(rel_vs));
    bool is_distance_shortening = rel_ds * rel_vs < 0;
    bool is_rear_approaching_vehicle =
        (is_distance_shortening && !if_obs_front);
    const double obj_acc = traj_ptr->planner_object().pose().a();
    const double obj_lon_acc = obj_acc * obj_target_tangent_cos;
    const double obj_lat_acc = obj_acc * obj_target_normal_cos;
    bool is_ego_stationary = (ego_speed_kph < kStationaryVehicleSpeedkph);
    bool is_ego_extra_slow = (ego_speed_kph < kExtremelySlowVehicleSpeedkph);

    const auto leader_vel = if_obs_front ? obj_v : ego_v;
    const auto leader_box = if_obs_front ? obj_cur_box : ego_cur_box;
    const auto leader_target_tangent_cos =
        if_obs_front ? obj_target_tangent_cos : ego_target_tangent_cos;
    const auto leader_target_normal_cos =
        if_obs_front ? obj_target_normal_cos : ego_target_normal_cos;

    const auto follower_vel = if_obs_front ? ego_v : obj_v;
    const auto follower_box = if_obs_front ? ego_cur_box : obj_cur_box;
    const auto follower_target_tangent_cos =
        if_obs_front ? ego_target_tangent_cos : obj_target_tangent_cos;
    const auto follower_target_normal_cos =
        if_obs_front ? ego_target_normal_cos : obj_target_normal_cos;
    bool is_obj_oncoming = obj_target_tangent_cos < -0.5;

    // Check for time to enter target lane.
    const double obj_enter_target_time = std::clamp(
        ComputeEnterTargetTime(target_frenet_frame, traj_ptr->states()), 0.0,
        255.0);
    const double obj_enter_target_time_by_lat_v =
        std::clamp(ComputeEnterTargetTimeByLatV(obj_cur_frenet_box, obj_lat_v),
                   0.0, 255.0);

    bool obj_pred_to_target =
        !(obj_enter_target_time > 1.2 * ego_enter_target_time &&
          obj_enter_target_time >
              getObjPredictedTrajectoryTime(traj_ptr, if_obs_front) *
                  conserv_base) &&
        !obj_is_leaving_target;

    conserv =
        conserv_base * pause_factor * getObjTypeFactor(traj_ptr, if_obs_front);
    const double follow_obj_resp_time = conserv * kFollowerStandardResponseTime;

    double min_lon_buffer_factor = std::clamp(
        conserv * ComputeMinLonBufferSimilarSpeedFactor(
                      leader_vel * leader_target_tangent_cos,
                      follower_vel * follower_target_tangent_cos, traj_ptr),
        0.38, 2.0);

    min_lon_buffer =
        min_lon_buffer_factor *
        ((if_obs_front ? temp_min_front_dist : temp_min_back_dist) +
         getMinDisCompensation(if_obs_front, is_big_vehicle, obj_cur_box));

    bool veh_will_stop = (Mps2Kph(obj_v) < kExtremelySlowVehicleSpeedkph) ||
                         ((obj_acc < -kEpsilon) && (-obj_v / obj_acc < 2.0) &&
                          (-obj_v / obj_acc >= 0));

    const double lat_overlap_thrd_temp = conserv * kEgoFollowTimeBufferPrepare *
                                         (ego_lat_v - obj_lat_v) *
                                         (lc_left ? 1.0 : -1.0);
    const double lat_overlap_thrd = obs_centered_in_target_lane
                                        ? std::min(-lat_overlap_thrd_temp, 0.0)
                                        : kRearObjIgnoreLatOverlapFactor *
                                              conserv_base *
                                              obj_cur_frenet_box.width();
    bool obj_overlap_ignore = (lat_overlap >= lat_overlap_thrd);

    bool is_obj_enter_target_in_same_side =
        obj_cur_frenet_box.center_l() * ego_cur_frenet_box.center_l() >
            kEpsilon &&
        (std::max(std::fabs(obj_cur_frenet_box.l_max),
                  std::fabs(obj_cur_frenet_box.l_min)) >
         (cur_half_lane_width + 0.2)) &&
        (obj_pred_to_target ||
         obj_enter_target_time - kEpsilon < kObjReliableTrajectoryTimeMax);
    bool is_rear_obj_enter_target_in_same_side =
        !if_obs_front && is_obj_enter_target_in_same_side;

    // Create obj info for lc style decider
    LaneChangeStyleDeciderObjectInfo obj_lcs_decider_info(
        (std::string)traj_ptr->object_id(), obj_lon_v, obj_lat_v,
        obj_cur_frenet_box);

    if (is_ego_stationary && !if_obs_front && is_rear_approaching_vehicle &&
        obs_in_target_lane && !obj_overlap_ignore) {
      min_lon_buffer += 5.0;
      dynamic_obs_debug << "-buf+";
    } else if (!is_lc_state_prev && !obs_in_target_lane &&
               ego_enter_target_idx != 0 && is_vehicle && if_obs_front &&
               std::fabs(rel_dl) < kEpsilon && veh_will_stop) {
      // When changing lanes, consider the safe distance of the low speed
      // vehicle in front of the current lane to prevent the risk of
      // collision when changing lanes.
      min_lon_buffer = 5.0 * conserv_base;

      double compensation_heading_dis =
          ego_start_target_normal_cos * ego_cur_box.half_length() * 2;
      double heading_factor = std::fabs(
          (compensation_heading_dis /
           (std::fabs(lat_overlap_lc) > kEpsilon ? lat_overlap_lc : kEpsilon)));
      double nudge_heading_factor = std::clamp(1 - heading_factor, 0.0, 1.0);

      std::vector<double> lat_threshold = {0.2, 1.0, 1.5, 2.0, 4.0};
      std::vector<double> lon_dis = {0.5, 3.0, 4.8, 6.0, 10.0};
      auto lat_overlap_safe_dis = ad_byd::planning::math::interp1_inc(
          lat_threshold, lon_dis, lat_overlap_lc * nudge_heading_factor);

      double lat_overlap_factor =
          std::clamp(lat_overlap_lc / vehicle_geom.width(), 0.0, 2.0);

      double state_debounce_factor =
          (std::fabs(ego_target_normal_cos) < 0.06 ? 1.0 : 0.8);

      auto front_safe_dis =
          std::max(lat_overlap_safe_dis,
                   ego_lon_v * 2.5 * conserv_base * lat_overlap_factor *
                       nudge_heading_factor * state_debounce_factor);
      if (prev_lc_stage == LaneChangeStage::LCS_EXECUTING) {
        front_safe_dis = front_safe_dis * 0.8;
      }
      bool obj_danger = rel_ds < front_safe_dis;
      dynamic_obs_debug << "-frtSta:" << obj_danger;
      if (obj_danger) {
        unsafe_object_id = std::string(traj_ptr->object_id());
        status_code = PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_CURRENT_LANE;
        leader_debug_info << static_obs_debug.str();
        follower_debug_info << dynamic_obs_debug.str();
        return absl::CancelledError(
            absl::StrFormat("front low speed vehicle %s currently too "
                            "close.lat_overlap:%.3f lat_overlap_lc:%.3f "
                            "front_safe_dis:%.3f rel_ds:%.3f conserv_base:%.3f"
                            "lat_overlap_safe_dis:%.3f ego_v:%.3f "
                            "nudge_heading_factor:%.3f",
                            traj_ptr->object_id(), lat_overlap, lat_overlap_lc,
                            front_safe_dis, rel_ds, conserv_base,
                            lat_overlap_safe_dis, ego_v, nudge_heading_factor));
      }

      // temporarily add scene[FRONT_SLOW_OBJ_ON_EGO_LANE] for lc style decider
      // the closest obj will be added at the end
      if (obj_cur_frenet_box.s_min < ego_slow_front_obj_scene_s_min) {
        ego_slow_front_obj_scene_s_min = obj_cur_frenet_box.s_min;
        ego_slow_front_obj_scene_info.set_obj_info(obj_lcs_decider_info,
                                                   ego_lcs_decider_info);
        ego_slow_front_obj_scene_info.set_scene_type(
            LaneChangeStylePostDeciderSceneType::
                SCENE_FRONT_SLOW_OBJ_ON_EGO_LANE);
        ego_slow_front_obj_scene_info.add_safety_checking_info(
            LaneChangeSafetyCheckingInfo(
                LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
                std::fabs(rel_ds), front_safe_dis));
      }
    }

    const std::vector<double> lon_safety_dis = {2.0, 2.5, 3.0, 3.5, 4.0};
    const std::vector<double> safe_speed_diff = {0.0, 10.0, 20.0, 30.0, 40.0};
    const auto lon_safe_min_dist = ad_byd::planning::math::interp1_inc(
        safe_speed_diff, lon_safety_dis, abs_rel_vs_kph);

    // Consider the impact of the acceleration of the own vehicle and the target
    // vehicle on the safe distance.
    double acc_safe_dis = 0;
    if (!(obj_lon_v < -kEpsilon) && !(ego_lon_v < -kEpsilon) &&
        !(ego_lon_v < kEpsilon && if_obs_front)) {
      acc_safe_dis = getAccelerationDistanceCompensation(
          conserv_base, ego_lon_acc, obj_lon_acc, rel_vs, if_obs_front,
          is_lc_state_prev, is_distance_shortening);
    }

    const double rel_head_way_time =
        if_obs_front
            ? ComputeEgoFollowTime(obj_lon_v, ego_lon_v, prev_lc_stage,
                                   min_lon_buffer)
            : ComputeEgoLeadTime(speed_limit, ego_lon_v, obj_lon_v,
                                 prev_lc_stage, min_lon_buffer, is_on_highway);

    // Check if the object is currently abreast of the ego vehicle.
    double obj_front_extension = 1.0;
    if (is_radical_lowspeed_congestion_scene) {
      obj_front_extension =
          std::max(0.0, conserv * rel_head_way_time * follower_vel *
                                follower_target_tangent_cos +
                            acc_safe_dis);
    } else {
      double min_safe_dis = ad_byd::planning::math::interp1_inc(
          std::vector<double>{0.0, 10.0, 20.0, 30.0, 60.0, 90.0, 120.0},
          std::vector<double>{1.0, 1.0, 1.2, 1.9, 2.8, 4.0, 6.0},
          Mps2Kph(follower_vel));
      constexpr double min_follow_safe_dis = 1.5;
      obj_front_extension =
          std::max(min_safe_dis, (conserv * rel_head_way_time * follower_vel *
                                      follower_target_tangent_cos +
                                  acc_safe_dis + min_follow_safe_dis));
    }
    if (is_big_vehicle) {
      if (if_obs_front) {
        obj_front_extension *= 1.3;
      } else {
        const double ext_factor_by_v = ad_byd::planning::math::interp1_inc(
            std::vector<double>{20.0, 30.0, 60.0, 90.0},
            std::vector<double>{1.0, 1.2, 1.3, 1.5}, Mps2Kph(obj_lon_v));
        const double ext_factor_by_dv = ad_byd::planning::math::interp1_inc(
            std::vector<double>{-10.0, 0.0, 10.0, 30.0},
            std::vector<double>{0.7, 1.0, 1.0, 1.2}, Mps2Kph(rel_vs));
        const double ext_factor =
            std::clamp(ext_factor_by_v * ext_factor_by_dv, 1.0, 1.5);
        obj_front_extension *= ext_factor;
      }
    }
    // if (is_rear_obj_enter_target_in_same_side) {
    //   obj_front_extension *= 1.5;
    // }

    const double obj_lat_extension =
        conserv * rel_head_way_time *
        (leader_vel * leader_target_normal_cos -
         follower_vel * follower_target_normal_cos);

    DLOG(INFO) << "object_id()" << traj_ptr->object_id();

    // obj will enter target lane from diff side scene
    double obj_box_l_min = std::fmin(std::fabs(obj_cur_frenet_box.l_min),
                                     std::fabs(obj_cur_frenet_box.l_max));
    double min_enter_time =
        std::clamp(Min(obj_enter_target_time, ego_enter_target_time,
                       obj_enter_target_time_by_lat_v),
                   0.0, 255.0);
    int obj_preview_index = std::clamp(
        static_cast<int>(std::round(min_enter_time / kObjTrajectoryStep)), 0,
        static_cast<int>(traj_ptr->states().size()) - 1);
    int ego_preview_index = std::clamp(
        static_cast<int>(std::round(min_enter_time / kTrajectoryTimeStep)), 0,
        ego_traj_size - 1);
    ASSIGN_OR_CONTINUE(const auto obj_preview_frenet_box,
                       target_frenet_frame.QueryFrenetBoxWithHeading(
                           traj_ptr->states()[obj_preview_index].box, M_PI));
    ASSIGN_OR_CONTINUE(const auto ego_preview_frenet_box,
                       target_frenet_frame.QueryFrenetBoxWithHeading(
                           ego_boxes[ego_preview_index], M_PI));
    double obj_previewbox_l_min =
        std::fmin(std::fabs(obj_preview_frenet_box.l_min),
                  std::fabs(obj_preview_frenet_box.l_max));
    double ego_previewbox_l_min =
        std::fmin(std::fabs(ego_preview_frenet_box.l_min),
                  std::fabs(ego_preview_frenet_box.l_max));
    double preview_factor = ad_byd::planning::math::interp1_inc(
        ENTER_TARGET_TIME_VEC, ENTER_TARGET_PREVIEW_FACTOR_VEC, min_enter_time);
    double type_factor = std::max(1.0, conserv);
    double front_extension_threshold =
        std::fmax(min_lon_buffer, obj_front_extension * preview_factor);
    double preview_rel_ds = 0.0;
    double preview_lon_overlap = getLongOverlap(
        ego_preview_frenet_box, obj_preview_frenet_box, preview_rel_ds);
    bool obj_enter_target_lon_ignore =
        ego_preview_frenet_box.s_max + front_extension_threshold <
            obj_preview_frenet_box.s_min ||
        obj_preview_frenet_box.s_max + front_extension_threshold <
            ego_preview_frenet_box.s_min;
    bool obj_enter_target_lat_ignore =
        obj_previewbox_l_min >
        std::fmin(
            cur_lane_width * type_factor * 1.5,
            std::fmax(cur_lane_width * type_factor * 0.5,
                      std::fabs(obj_lat_extension) - ego_previewbox_l_min));
    bool obj_enter_target_ignore =
        obj_enter_target_lon_ignore || obj_enter_target_lat_ignore;
    bool obj_not_leaving_heading = obj_cur_frenet_box.center_l() > 0
                                       ? (obj_target_normal_cos < 0.01)
                                       : (obj_target_normal_cos > -0.01);
    std::string obj_lc_intention_debug_str = "";
    const bool obj_will_enter_targetlane =
        !obs_in_target_lane &&
        obj_cur_frenet_box.center_l() * ego_cur_frenet_box.center_l() < 0 &&
        ((obj_enter_target_time < 5.0 && !obj_enter_target_ignore) ||
         (obj_cur_frenet_box.center_l() * obj_target_normal_cos < 0 &&
          std::fabs(obj_target_normal_cos) > 0.05 && !obj_enter_target_ignore &&
          std::fabs(obj_v * obj_target_normal_cos) > 1.0 &&
          obj_box_l_min < cur_lane_width - 0.3) ||
         (obj_box_l_min < cur_half_lane_width - 0.3 &&
          obj_not_leaving_heading) ||
         (!is_lc_state_prev && Mps2Kph(ego_v) > 20.0 &&
          ObjectHasLCIntentionInDiffSide(
              target_frenet_frame, *traj_ptr, obs_history_mgr,
              ego_cur_frenet_box, obj_cur_frenet_box, ego_lon_v, obj_lon_v,
              obj_target_normal_cos, cur_lane_width, ego_enter_target_time,
              obj_front_extension, min_lon_buffer, lc_left,
              obj_lc_intention_debug_str)));
    if (obj_lc_intention_debug_str != "") {
      Log2DDS::LogDataV2("safety_lc_intention_debug",
                         obj_lc_intention_debug_str);
    }

    if (!if_obs_front && !(obj_pred_to_target || obs_in_target_lane) &&
        (std::fabs(obj_cur_frenet_box.center_l()) >
         std::fabs(ego_cur_frenet_box.center_l()) - ego_half_width) &&
        !obj_will_enter_targetlane) {
      dynamic_obs_debug << "-SkpRr";
      continue;
    }

    if (if_obs_front && !(obj_pred_to_target || obs_in_target_lane) &&
        (std::fabs(obj_cur_frenet_box.center_l()) >
         std::fabs(ego_cur_frenet_box.center_l()) + ego_half_width) &&
        !obj_will_enter_targetlane) {
      dynamic_obs_debug << "-SkpFf";
      continue;
    }

    bool obj_is_abreast = IsBlockingObjectAbreast(
        ego_cur_frenet_box, obj_cur_frenet_box, obj_front_extension,
        obj_lat_extension, min_lon_buffer, obj_will_enter_targetlane,
        can_not_return);
    dynamic_obs_debug << "-abr:" << obj_is_abreast
                      << "-e:" << obs_in_target_lane
                      << "-p:" << obj_pred_to_target
                      << "-ss:" << is_rear_obj_enter_target_in_same_side
                      << "-ot:" << obj_enter_target_time
                      << "-otv:" << obj_enter_target_time_by_lat_v
                      << "-et:" << ego_enter_target_time
                      << "-w:" << obj_will_enter_targetlane
                      << "-eprs:" << ego_preview_frenet_box.center_s()
                      << "-oprs:" << obj_preview_frenet_box.center_s()
                      << "-otig:" << obj_enter_target_ignore
                      << "-oprl:" << obj_previewbox_l_min
                      << "-cts:" << obj_is_cutout_soon;
    auto obj_abreast_str = absl::StrFormat(
        "Object:%s abreast:%d. "
        "front_ext:%.3f lat_ext:%.3f "
        "conserv:%.3f conserv_base:%.3f MinLonBuf:%.3f "
        "MinLonBuf_fct:%.3f  head_way:%.3f head_vel:%.3f "
        "rel_ds:%.3f rel_dl:%.3f ego_v:%.3f obs_v:%.3f "
        "ego_acc:%.3f obj_acc:%.3f acc_dis:%.3f "
        "ego_cos:%.3f obj_cos:%.3f ego_sin:%.3f obj_sin:%.3f "
        "ego_lmax:%.3f ego_lmin:%.3f obj_lmax:%.3f obj_lmin:%.3f "
        "ego_pr_s:%.3f %.3f obj_pr_s:%.3f %.3f "
        "ego_pr_lmin:%.3f obj_pr_lmin:%.3f ",
        traj_ptr->object_id(), obj_is_abreast, obj_front_extension,
        obj_lat_extension, conserv, conserv_base, min_lon_buffer,
        min_lon_buffer_factor, rel_head_way_time, follower_vel, rel_ds, rel_dl,
        ego_v, obj_v, ego_acc, obj_acc, acc_safe_dis, ego_target_tangent_cos,
        obj_target_tangent_cos, ego_target_normal_cos, obj_target_normal_cos,
        ego_cur_frenet_box.l_max, ego_cur_frenet_box.l_min,
        obj_cur_frenet_box.l_max, obj_cur_frenet_box.l_min,
        ego_preview_frenet_box.s_max, ego_preview_frenet_box.s_min,
        obj_preview_frenet_box.s_max, obj_preview_frenet_box.s_min,
        ego_previewbox_l_min, obj_previewbox_l_min);

    if (if_obs_front && (obj_pred_to_target || obs_in_target_lane) &&
        (frnt_obs_id.empty() ||
         obj_cur_frenet_box.s_min < frnt_obs_box.s_min)) {
      frnt_obs_id = std::string(traj_ptr->object_id());
      frnt_obs_box = obj_cur_frenet_box;
      leader_debug_info.clear();
      leader_debug_info.str("");
      // if (obj_is_abreast) {
      //   leader_debug_info << "Object:" << traj_ptr->object_id()
      //                     << " is abreast.";
      // } else {
      //   leader_debug_info << obj_abreast_str;
      // }
      leader_debug_info << obj_abreast_str;
    } else if (if_obs_rear && (obj_pred_to_target || obs_in_target_lane) &&
               (rear_obs_id.empty() ||
                obj_cur_frenet_box.s_max > rear_obs_box.s_max)) {
      rear_obs_id = std::string(traj_ptr->object_id());
      rear_obs_box = obj_cur_frenet_box;
      follower_debug_info.clear();
      follower_debug_info.str("");
      // if (obj_is_abreast) {
      //   follower_debug_info << "Object:" << traj_ptr->object_id()
      //                       << " is abreast.";
      // } else {
      //   follower_debug_info << obj_abreast_str;
      // }
      follower_debug_info << obj_abreast_str;
    }

    if (!obs_in_target_lane && obj_enter_target_ignore &&
        obj_enter_target_time < 5.0 &&
        obj_cur_frenet_box.center_l() * ego_cur_frenet_box.center_l() < 0 &&
        ego_preview_frenet_box.s_min > obj_preview_frenet_box.s_max) {
      if (!if_obs_front && obj_enter_target_lon_ignore) {
        // add scene[REAR_OBJ_IN_OPPOSITE_DIR] for lc style decider
        LaneChangeStylePostDeciderSceneInfo lc_scene_info;
        lc_scene_info.set_obj_info(obj_lcs_decider_info, ego_lcs_decider_info);
        lc_scene_info.set_scene_type(LaneChangeStylePostDeciderSceneType::
                                         SCENE_REAR_OBJ_IN_OPPOSITE_DIR);
        lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
            LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
            std::fabs(rel_ds), obj_front_extension));
        lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
            LaneChangeSafetyCheckingCondition::CHECK_COND_PRV_LON_DS,
            std::fabs(preview_rel_ds), front_extension_threshold));
        scene_infos_ptr->emplace_back(lc_scene_info);
      } else if (if_obs_front &&
                 obj_cur_frenet_box.s_min < target_front_obj_scene_s_min) {
        target_front_obj_scene_s_min = obj_cur_frenet_box.s_min;
        // add scene[FRONT_OBJ_ON_TARGET_LANE] for lc style decider
        target_front_obj_scene_info_ptr->set_obj_info(obj_lcs_decider_info,
                                                      ego_lcs_decider_info);
        target_front_obj_scene_info_ptr->set_scene_type(
            LaneChangeStylePostDeciderSceneType::
                SCENE_FRONT_OBJ_ON_TARGET_LANE);
        target_front_obj_scene_info_ptr->ClearSafetyCheckingInfo();
        target_front_obj_scene_info_ptr->add_safety_checking_info(
            LaneChangeSafetyCheckingInfo(
                LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
                std::fabs(rel_ds), obj_front_extension));
        target_front_obj_scene_info_ptr->add_safety_checking_info(
            LaneChangeSafetyCheckingInfo(
                LaneChangeSafetyCheckingCondition::CHECK_COND_PRV_LON_DS,
                std::fabs(preview_rel_ds), front_extension_threshold));
      }
      dynamic_obs_debug << "-DSIG";
      continue;
    }
    if ((if_obs_front && obs_in_target_lane) &&
        (frnt_static_filter_obs_id.empty() ||
         obj_cur_frenet_box.s_min < frnt_static_filter_obs_box.s_min)) {
      frnt_static_filter_obs_id = std::string(traj_ptr->object_id());
      frnt_static_filter_obs_box = obj_cur_frenet_box;
    }

    // Ignore objects that are currently leaving the target lane path behind
    // or abreast the ego vehicle.
    const double lat_overlap_thrd_for_obj_leaving = 1.8;
    if (!is_vru && obs_in_target_lane && obj_is_leaving_near_field_target &&
        (lc_left ? obj_cur_frenet_box.l_min > 0
                 : obj_cur_frenet_box.l_max < 0) &&
        lat_overlap < -lat_overlap_thrd_for_obj_leaving &&
        obj_lc_left == lc_left && !is_obj_oncoming) {
      if (if_obs_front &&
          obj_cur_frenet_box.s_min < target_front_obj_scene_s_min) {
        target_front_obj_scene_s_min = obj_cur_frenet_box.s_min;
        // add scene[FRONT_OBJ_ON_TARGET_LANE] for lc style decider
        target_front_obj_scene_info_ptr->set_obj_info(obj_lcs_decider_info,
                                                      ego_lcs_decider_info);
        target_front_obj_scene_info_ptr->set_scene_type(
            LaneChangeStylePostDeciderSceneType::
                SCENE_FRONT_OBJ_ON_TARGET_LANE);
        target_front_obj_scene_info_ptr->ClearSafetyCheckingInfo();
        target_front_obj_scene_info_ptr->add_safety_checking_info(
            LaneChangeSafetyCheckingInfo(
                LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
                std::fabs(rel_ds), obj_front_extension));
      } else if (!if_obs_front &&
                 nearest_follower_id == (std::string)traj_ptr->object_id()) {
        // add scene[REAR_OBJ_ON_TARGET_LANE] for lc style decider
        LaneChangeStylePostDeciderSceneInfo lc_scene_info;
        lc_scene_info.set_obj_info(obj_lcs_decider_info, ego_lcs_decider_info);
        lc_scene_info.set_scene_type(
            LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_ON_TARGET_LANE);
        lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
            LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
            std::fabs(rel_ds), obj_front_extension));
        scene_infos_ptr->emplace_back(lc_scene_info);
      }
      dynamic_obs_debug << "-Lea1";
      continue;
    }

    if (IsLeavingTargetLanePath(target_frenet_frame, lc_left,
                                ego_cur_frenet_box, obj_cur_frenet_box,
                                traj_ptr->states(), lon_safe_min_dist,
                                min_lon_buffer) &&
        !is_obj_oncoming) {
      dynamic_obs_debug << "-Lea2";
      continue;
    } else {
      dynamic_obs_debug << "-LvT:" << obj_is_leaving_near_field_target;
    }

    // if (!is_vru && obs_centered_in_target_lane && obj_overlap_ignore &&
    //     ego_enter_target_idx < 10) {
    //   dynamic_obs_debug << "-SkpTr:" << lat_overlap_thrd;
    //   continue;
    // }

    double rel_ttc = 0.0;
    double rel_ttc_preview = 0.0;
    bool impending_collision_scene = true;
    // check for obtaining the lane-right from the rear object
    if (obj_cur_frenet_box.s_min <= ego_cur_frenet_box.s_max) {
      rel_ttc =
          (obj_lon_v > ego_lon_v)
              ? std::fabs(rel_ds) / std::max(obj_lon_v - ego_lon_v, kEpsilon)
              : DBL_MAX;
      // check if it is safe for the obj to decelerate to ego_v
      double impending_collision_scene_ttc_factor = 1.0;
      if (obj_lon_v > ego_lon_v) {
        const double impending_collision_scene_follower_max_decel = 2.0;
        double obj_brake_time =
            rel_vs / impending_collision_scene_follower_max_decel;
        double obj_travel_dist = obj_lon_v * follow_obj_resp_time +
                                 0.5 * (obj_lon_v + ego_lon_v) * obj_brake_time;
        double ego_travel_dist =
            ego_lon_v * (follow_obj_resp_time + obj_brake_time);
        if (ego_travel_dist - rel_ds - acc_safe_dis - min_lon_buffer >
            obj_travel_dist) {
          impending_collision_scene_ttc_factor = 0.6;
        }
      }

      rel_ttc_preview = rel_ttc;
      if (min_enter_time > kEpsilon) {
        const double ego_lon_v_preview =
            ego_lon_v + ego_lon_acc * min_enter_time;
        const double obj_lon_v_preview =
            obj_lon_v + obj_lon_acc * min_enter_time;

        double rel_ds_preview = 0;
        getLongOverlap(ego_preview_frenet_box, obj_preview_frenet_box,
                       rel_ds_preview);

        if (rel_ds_preview >= 0) {
          rel_ttc_preview = 0.0;
        } else {
          rel_ttc_preview =
              (obj_lon_v_preview > ego_lon_v_preview)
                  ? std::fabs(rel_ds_preview) /
                        std::max(obj_lon_v_preview - ego_lon_v_preview,
                                 kEpsilon)
                  : DBL_MAX;
        }
      }

      const double ttc_thrd = conserv_base *
                              impending_collision_scene_ttc_factor *
                              (is_lc_state_prev ? 6.0 : 7.0);
      impending_collision_scene =
          rel_ttc < ttc_thrd || rel_ttc_preview < ttc_thrd;
      dynamic_obs_debug << "-(ttc:" << rel_ttc << ",ttcp:" << rel_ttc_preview
                        << ",thrd:" << ttc_thrd << ")";
      // if (obj_overlap_ignore && !is_rear_obj_enter_target_in_same_side &&
      //     (!obs_centered_in_target_lane ? !impending_collision_scene : true))
      //     {
      //   if (!obs_centered_in_target_lane) {
      //     // add scene[REAR_OBJ_IN_SAME_DIR] for lc style decider
      //     LaneChangeStylePostDeciderSceneInfo lc_scene_info;
      //     lc_scene_info.set_obj_info(obj_lcs_decider_info,
      //                                ego_lcs_decider_info);
      //     lc_scene_info.set_scene_type(
      //         LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_IN_SAME_DIR);
      //     lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
      //         LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
      //         std::fabs(rel_ds), obj_front_extension));
      //     lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
      //         LaneChangeSafetyCheckingCondition::CHECK_COND_LAT_DL,
      //         lat_overlap, lat_overlap_thrd));
      //     if (obj_lon_v > ego_lon_v) {
      //       lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
      //           LaneChangeSafetyCheckingCondition::CHECK_COND_TTC, rel_ttc,
      //           ttc_thrd));
      //     }
      //     scene_infos_ptr->emplace_back(lc_scene_info);
      //   } else if (nearest_follower_id == (std::string)traj_ptr->object_id())
      //   {
      //     // add scene[REAR_OBJ_ON_TARGET_LANE] for lc style decider
      //     LaneChangeStylePostDeciderSceneInfo lc_scene_info;
      //     lc_scene_info.set_obj_info(obj_lcs_decider_info,
      //                                ego_lcs_decider_info);
      //     lc_scene_info.set_scene_type(LaneChangeStylePostDeciderSceneType::
      //                                      SCENE_REAR_OBJ_ON_TARGET_LANE);
      //     lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
      //         LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
      //         std::fabs(rel_ds), obj_front_extension));
      //     scene_infos_ptr->emplace_back(lc_scene_info);
      //   }
      //   dynamic_obs_debug << "-Olap";
      //   continue;
      // }
    }

    // Check deceleration in advance, but make it effective in subsequent steps
    double max_braking_deceleration = 0;
    double hypo_braking_deceleration = 0;
    double debounce_braking_deceleration = 0;
    std::string check_deceleration_str = "";
    absl::Status check_deceleration_status;
    if (obj_cur_frenet_box.s_min <= ego_cur_frenet_box.s_max) {
      double max_decel_for_rear_object = ad_byd::planning::math::interp1_inc(
          DV_TTC_VEC, F_DV_BACK_DEC_VEC, abs_rel_vs_kph);

      if (is_lc_state_prev) {
        double acc_factor =
            impending_collision_scene
                ? ad_byd::planning::math::interp1_inc(
                      std::vector<double>{-2.0, -1.0, 0, 1.0, 1.5, 2.0, 3.0},
                      std::vector<double>{1.6, 1.3, 1.0, 0, -0.3, -0.6, -1.2},
                      obj_lon_acc)
                : 1.0;
        // double acc_factor = Lerp(1.0, 0.5, 0.0, 0.8, obs_acc, true);
        debounce_braking_deceleration =
            std::clamp(((1.0 - conserv_base) * 2.5 * acc_factor), -1.2, 1.5);

        max_decel_for_rear_object += debounce_braking_deceleration;
      }

      if (is_big_vehicle) {
        dynamic_obs_debug << "-f:0.6";
        max_decel_for_rear_object *= 0.6;
      } else if (obj_is_leaving_target) {
        double obs_abs_center_l = std::fabs(obj_cur_frenet_box.center_l());
        double obs_offset_factor = ad_byd::planning::math::interp1_inc(
            std::vector<double>{0.0, 0.3, 0.5, 1.5, 2.0},
            std::vector<double>{1.1, 1.2, 1.3, 1.5, 2.0}, obs_abs_center_l);
        dynamic_obs_debug << "-f:" << obs_offset_factor;
        max_decel_for_rear_object *= obs_offset_factor;
      } else if ((obj_lon_acc > std::max(1.5, 1.2 * ego_lon_acc)) &&
                 impending_collision_scene && !obj_is_leaving_target) {
        dynamic_obs_debug << "-f:0.7";
        max_decel_for_rear_object *= 0.7;
      } else if ((ego_lon_acc < -0.5 || obj_lon_acc < -2.5) &&
                 !obj_is_leaving_target && impending_collision_scene) {
        dynamic_obs_debug << "-f:0.8";
        max_decel_for_rear_object *= 0.8;
      } else {
        dynamic_obs_debug << "-f:1.0";
      }

      if (max_decel_for_rear_object < -obj_lon_acc) {
        dynamic_obs_debug << "-avr";
        max_decel_for_rear_object =
            (max_decel_for_rear_object - obj_lon_acc) * 0.5;
      }

      max_decel_for_rear_object *= (1.0 / congestion_factor);

      max_decel_for_rear_object = std::clamp(
          max_decel_for_rear_object / pause_factor, 0.6, is_vru ? 3.0 : 5.0);

      // if (is_rear_obj_enter_target_in_same_side) {
      //   max_decel_for_rear_object *= 0.67;
      // }

      max_braking_deceleration = max_decel_for_rear_object;

      check_deceleration_status = CheckDeceleration(
          std::fabs(rel_ds), "ego", std::string(traj_ptr->object_id()),
          ego_lon_v, obj_lon_v, follow_obj_resp_time,
          conserv * rel_head_way_time, max_decel_for_rear_object,
          min_lon_buffer, acc_safe_dis, check_deceleration_str,
          &hypo_braking_deceleration, &follower_max_decel);
    } else {
      double max_decel_for_front_object = ad_byd::planning::math::interp1_inc(
          DV_TTC_VEC, F_DV_FRONT_DEC_VEC, abs_rel_vs_kph);

      rel_ttc =
          (ego_lon_v > obj_lon_v) ? rel_ds / (ego_lon_v - obj_lon_v) : DBL_MAX;
      impending_collision_scene =
          rel_ttc < conserv_base * (is_lc_state_prev ? 5.0 : 6.0);

      if (is_lc_state_prev) {
        double acc_factor =
            impending_collision_scene
                ? ad_byd::planning::math::interp1_inc(
                      std::vector<double>{-3.0, -2.0, -1.5 - 1.0, 0, 0.5, 1.0,
                                          2.0},
                      std::vector<double>{-1.0, -0.8, -0.5, 0, 1.0, 1.2, 1.3,
                                          1.6},
                      obj_lon_acc)
                : 1.0;

        debounce_braking_deceleration =
            std::clamp(((1.0 - conserv_base) * 2.0 * acc_factor), -1.5, 2.0);

        max_decel_for_front_object += debounce_braking_deceleration;
      }

      bool front_obj_is_leaving =
          obj_is_leaving_target && !obs_centered_in_target_lane;

      if (is_vru) {
        dynamic_obs_debug << "-f:0.6";
        max_decel_for_front_object *= 0.6;
      } else if (ego_lon_acc > 0.5 && !front_obj_is_leaving &&
                 impending_collision_scene) {
        dynamic_obs_debug << "-f:0.7";
        max_decel_for_front_object *= 0.7;
      } else if ((obj_lon_acc < std::min(-1.0, ego_lon_acc) ||
                  ego_lon_acc > 0.1) &&
                 !front_obj_is_leaving && impending_collision_scene) {
        dynamic_obs_debug << "-f:0.8";
        max_decel_for_front_object *= 0.8;
      } else if (front_obj_is_leaving) {
        dynamic_obs_debug << "-f:1.2";
        max_decel_for_front_object *= 1.2;
      } else {
        dynamic_obs_debug << "-f:1.0";
      }

      max_decel_for_front_object =
          std::clamp(max_decel_for_front_object / pause_factor, 1.0, 5.0);
      max_braking_deceleration = max_decel_for_front_object;
      max_decel_for_front_object = is_vru ? 3.0 : 5.0;

      check_deceleration_status = CheckDeceleration(
          std::fabs(rel_ds), std::string(traj_ptr->object_id()), "ego",
          obj_lon_v, ego_lon_v, kEgoResponseTime, conserv * rel_head_way_time,
          max_decel_for_front_object, min_lon_buffer, acc_safe_dis,
          check_deceleration_str, &hypo_braking_deceleration,
          &leader_max_decel);
    }
    dynamic_obs_debug << absl::StrFormat("-dec:%d(-%.3f/-%.3f)",
                                         !check_deceleration_status.ok(),
                                         hypo_braking_deceleration,
                                         max_braking_deceleration)
                      << "-ttc:" << rel_ttc;

    // For rear objects in the target lane
    if (obs_in_target_lane &&
        ego_cur_frenet_box.s_min > obj_cur_frenet_box.s_max) {
      // If checking deceleration is safe and there is a lateral overlap while
      // lc pausing, force the lane change to be safe
      const double kLCPauseTime = 1.0;  // s.
      const double obj_lat_dist_lc_pause = obj_lat_v * kLCPauseTime;
      double lc_pause_lat_overlap = 0.0;
      if (lc_left) {
        lc_pause_lat_overlap =
            lc_pause_offset + ego_half_width -
            (obj_cur_frenet_box.l_min + obj_lat_dist_lc_pause);
      } else {
        lc_pause_lat_overlap = obj_cur_frenet_box.l_max +
                               obj_lat_dist_lc_pause -
                               (lc_pause_offset - ego_half_width);
      }
      const bool lc_force_safe =
          (prev_lc_stage == LaneChangeStage::LCS_EXECUTING) &&
          (!is_rear_obj_enter_target_in_same_side ||
           ego_enter_target_idx == 0) &&
          ((check_deceleration_status.ok() &&
            lc_pause_lat_overlap > -kEpsilon) ||
           lc_pause_lat_overlap > 0.5 * obj_cur_box.width());
      if (lc_force_safe) {
        if (is_rear_obj_enter_target_in_same_side) {
          // add scene[REAR_OBJ_IN_SAME_DIR] for lc style decider
          LaneChangeStylePostDeciderSceneInfo lc_scene_info;
          lc_scene_info.set_obj_info(obj_lcs_decider_info,
                                     ego_lcs_decider_info);
          lc_scene_info.set_scene_type(
              LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_IN_SAME_DIR);
          lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
              LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
              std::fabs(rel_ds), obj_front_extension));
          lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
              LaneChangeSafetyCheckingCondition::CHECK_COND_LAT_DL, lat_overlap,
              lat_overlap_thrd));
          lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
              LaneChangeSafetyCheckingCondition::CHECK_COND_FOLLOW_DECEL,
              std::fabs(hypo_braking_deceleration),
              std::fabs(max_braking_deceleration)));
          scene_infos_ptr->emplace_back(lc_scene_info);
        } else {
          if (nearest_follower_id == (std::string)traj_ptr->object_id()) {
            // add scene[REAR_OBJ_ON_TARGET_LANE] for lc style decider
            LaneChangeStylePostDeciderSceneInfo lc_scene_info;
            lc_scene_info.set_obj_info(obj_lcs_decider_info,
                                       ego_lcs_decider_info);
            lc_scene_info.set_scene_type(LaneChangeStylePostDeciderSceneType::
                                             SCENE_REAR_OBJ_ON_TARGET_LANE);
            lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
                LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
                std::fabs(rel_ds), obj_front_extension));
            lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
                LaneChangeSafetyCheckingCondition::CHECK_COND_FOLLOW_DECEL,
                std::fabs(hypo_braking_deceleration),
                std::fabs(max_braking_deceleration)));
            scene_infos_ptr->emplace_back(lc_scene_info);
          }
        }
        dynamic_obs_debug << "-ForceSafe";
        continue;
      }
    }

    // Ignore objs that are lane changing in the same side but have not caught
    // up laterally with the ego
    bool obj_lat_further_than_ego =
        lc_left ? ego_cur_frenet_box.l_min > obj_cur_frenet_box.l_max
                : obj_cur_frenet_box.l_min > ego_cur_frenet_box.l_max;
    if (is_obj_enter_target_in_same_side && obj_lat_further_than_ego) {
      dynamic_obs_debug << "-sslf";
      continue;
    }

    // const bool is_gaming_lc_scenario = is_congestion_scene && (ego_speed_kph
    // < 45.0);
    const bool is_gaming_lc_scenario = false;
    if (obj_is_abreast) {
      const std::string obj_id = std::string(traj_ptr->object_id());
      double abreast_dist_hard_thrd = 0.8 * obj_front_extension;
      if (ego_cur_frenet_box.s_min - obj_cur_frenet_box.s_max >
          abreast_dist_hard_thrd) {
        follower_set.insert(obj_id);
      } else if (obj_cur_frenet_box.s_min - ego_cur_frenet_box.s_max >
                 abreast_dist_hard_thrd) {
        leader_set.insert(obj_id);
      }
      if (frnt_obs_id != "") {
        leader_set.insert(frnt_obs_id);
      }
      if (rear_obs_id != "") {
        follower_set.insert(rear_obs_id);
      }
      if (obj_will_enter_targetlane && !if_obs_front && is_gaming_lc_scenario) {
        // Ignore the object if lc_safety_checker considers it safe
        auto gaming_lc_safety_status = gaming_lc_safety_checker.Execute(
            gaming_lc_safety_input, traj_ptr->traj_id());
        if (gaming_lc_safety_status.ok()) {
          gaming_lc_obs_set.insert(gaming_lc_safety_status.value());
          dynamic_obs_debug << "-GamingSafe";
          continue;
        }
      }
      unsafe_object_id = obj_id;
      status_code =
          obs_in_target_lane
              ? (if_obs_front
                     ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                     : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_TARGET_LANE)
              : (if_obs_front
                     ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                     : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
      leader_debug_info << static_obs_debug.str();
      follower_debug_info << dynamic_obs_debug.str();
      return absl::CancelledError(obj_abreast_str);
    }
    if (can_not_return && !is_obj_oncoming) {
      dynamic_obs_debug << "-can not return";
      continue;
    }
    if ((ego_enter_target_idx == 0 && ego_occupy_target) ||
        ego_centered_in_target_lane) {
      // If already entered target lane, only check for very dangrerous
      // situation where some vehicle is too close.
      dynamic_obs_debug << "-OcT";
      continue;
    }

    if ((prev_lc_stage != LaneChangeStage::LCS_PAUSE) &&
        !(obj_pred_to_target || obs_in_target_lane) &&
        (std::fabs(obj_cur_frenet_box.center_l()) >
             std::fabs(ego_cur_frenet_box.center_l()) - ego_half_width ||
         (obj_cur_frenet_box.center_l() * ego_cur_frenet_box.center_l() < 0)) &&
        !PathHasOverlap(ego_boxes, obj_v, traj_ptr->states())) {
      dynamic_obs_debug << "-Nol";
      continue;
    }

    // Check for time to enter target lane.
    if (!(obj_pred_to_target || obs_in_target_lane) &&
        (std::fabs(obj_cur_frenet_box.center_l()) >
             std::fabs(ego_cur_frenet_box.center_l()) - ego_half_width ||
         (obj_cur_frenet_box.center_l() * ego_cur_frenet_box.center_l() < 0))) {
      dynamic_obs_debug << "-Net"
                        << "-t:" << obj_enter_target_time;
      continue;
    }

    // Check for obj's deceleration.
    if (obj_cur_frenet_box.s_min <=
        ego_cur_frenet_box.s_max) {  // Ego at front.
      // Record all considered objects that should follow the ego vehicle.
      follower_set.insert(std::string(traj_ptr->object_id()));

      // ignore if it is on the target lane but not the nearest follower
      std::string obj_id = std::string(traj_ptr->object_id());
      bool is_obj_on_target_lane =
          sorted_object_ids_on_target_lane.end() !=
          std::find_if(sorted_object_ids_on_target_lane.begin(),
                       sorted_object_ids_on_target_lane.end(),
                       [&](const auto& a) { return a.first == obj_id; });
      if (!is_vru && is_obj_on_target_lane && obj_id != nearest_follower_id) {
        dynamic_obs_debug << "-Nnf";
        continue;
      }

      if (ego_lon_v >= obj_lon_v &&
          ego_lon_v + std::min(ego_lon_acc, 0.0) * 3.0 * conserv_base >=
              obj_lon_v + obj_lon_acc * 3.0 * conserv_base) {
        if (is_rear_obj_enter_target_in_same_side) {
          // add scene[REAR_OBJ_IN_SAME_DIR] for lc style decider
          LaneChangeStylePostDeciderSceneInfo lc_scene_info;
          lc_scene_info.set_obj_info(obj_lcs_decider_info,
                                     ego_lcs_decider_info);
          lc_scene_info.set_scene_type(
              LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_IN_SAME_DIR);
          lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
              LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
              std::fabs(rel_ds), obj_front_extension));
          lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
              LaneChangeSafetyCheckingCondition::CHECK_COND_LAT_DL, lat_overlap,
              lat_overlap_thrd));
          scene_infos_ptr->emplace_back(lc_scene_info);
        } else if (nearest_follower_id == (std::string)traj_ptr->object_id()) {
          // add scene[REAR_OBJ_ON_TARGET_LANE] for lc style decider
          LaneChangeStylePostDeciderSceneInfo lc_scene_info;
          lc_scene_info.set_obj_info(obj_lcs_decider_info,
                                     ego_lcs_decider_info);
          lc_scene_info.set_scene_type(LaneChangeStylePostDeciderSceneType::
                                           SCENE_REAR_OBJ_ON_TARGET_LANE);
          lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
              LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
              std::fabs(rel_ds), obj_front_extension));
          scene_infos_ptr->emplace_back(lc_scene_info);
        }
        dynamic_obs_debug << "-Rsml";
        continue;
      }

      if (rear_obs_id == traj_ptr->object_id()) {
        if (check_deceleration_status.ok()) {
          follower_debug_info << "/***/CheckDeceleration:"
                              << check_deceleration_str;
        }
      }

      if (!check_deceleration_status.ok()) {
        // Ignore the object if lc_safety_checker considers it safe
        if (is_gaming_lc_scenario) {
          auto gaming_lc_safety_status = gaming_lc_safety_checker.Execute(
              gaming_lc_safety_input, traj_ptr->traj_id());
          if (gaming_lc_safety_status.ok()) {
            gaming_lc_obs_set.insert(gaming_lc_safety_status.value());
            dynamic_obs_debug << "-GamingSafe";
            continue;
          }
        }
        unsafe_object_id = std::string(traj_ptr->object_id());
        status_code =
            obs_in_target_lane
                ? (if_obs_front
                       ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                       : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_TARGET_LANE)
                : (if_obs_front ? PlannerStatusProto::
                                      FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                                : PlannerStatusProto::
                                      REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
        leader_debug_info << static_obs_debug.str();
        follower_debug_info << dynamic_obs_debug.str();
        return absl::CancelledError(check_deceleration_status.ToString());
      }
    } else {  // Obj at front.
      // Record all considered objects that should lead the ego vehicle.
      leader_set.insert(std::string(traj_ptr->object_id()));

      // ignore if it is farther than the nearest leader
      if (nearest_moving_leader_id != std::string(traj_ptr->object_id()) &&
          is_moving_leader_box_ready &&
          obj_cur_frenet_box.s_min > moving_leader_frenet_box.s_min) {
        dynamic_obs_debug << "-Nnl";
        continue;
      }

      // Before lane-changing,
      // check if there is a closer blocking object between ego and leader
      // if no leading, there must be no any front objects
      double front_dist_thr = ad_byd::planning::math::interp1_inc(
          std::vector<double>{0.0, 20.0, 50.0, 80.0, 120.0},
          std::vector<double>{30.0, 40.0, 50.0, 60.0, 70.0}, ego_speed_kph);
      bool obj_current_in_target_lane = HasFullyEnteredTargetLane(
          obj_cur_frenet_box.center_l(), 0.5 * obj_cur_frenet_box.width());
      if (!is_lc_state_prev && obj_current_in_target_lane && !is_vru &&
          rel_ds < front_dist_thr &&
          ((nearest_moving_leader_id == "" && !obj_is_cutout_soon) ||
           (nearest_moving_leader_id != std::string(traj_ptr->object_id()) &&
            is_moving_leader_box_ready &&
            obj_cur_frenet_box.s_min < moving_leader_frenet_box.s_min))) {
        unsafe_object_id = std::string(traj_ptr->object_id());
        status_code =
            obs_in_target_lane
                ? (if_obs_front
                       ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                       : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_TARGET_LANE)
                : (if_obs_front ? PlannerStatusProto::
                                      FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                                : PlannerStatusProto::
                                      REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
        leader_debug_info << static_obs_debug.str();
        follower_debug_info << dynamic_obs_debug.str();
        return absl::CancelledError(absl::StrFormat(
            "obj(%s) is between ego and leading(%s), rel_ds:%.3f, "
            "dist_thr:%3.f",
            traj_ptr->object_id(), nearest_moving_leader_id.c_str(), rel_ds,
            front_dist_thr));
      }

      if (ego_lon_v <= obj_lon_v &&
          ego_lon_v + std::max(ego_lon_acc, 0.0) * 3.0 * conserv_base <=
              obj_lon_v + obj_lon_acc * 3.0 * conserv_base) {
        if (obj_cur_frenet_box.s_min < target_front_obj_scene_s_min) {
          target_front_obj_scene_s_min = obj_cur_frenet_box.s_min;
          // add scene[FRONT_OBJ_ON_TARGET_LANE] for lc style decider
          target_front_obj_scene_info_ptr->set_obj_info(obj_lcs_decider_info,
                                                        ego_lcs_decider_info);
          target_front_obj_scene_info_ptr->set_scene_type(
              LaneChangeStylePostDeciderSceneType::
                  SCENE_FRONT_OBJ_ON_TARGET_LANE);
          target_front_obj_scene_info_ptr->ClearSafetyCheckingInfo();
          target_front_obj_scene_info_ptr->add_safety_checking_info(
              LaneChangeSafetyCheckingInfo(
                  LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
                  std::fabs(rel_ds), obj_front_extension));
        }
        dynamic_obs_debug << "-Flrg";
        continue;
      }

      if (frnt_obs_id == traj_ptr->object_id()) {
        if (check_deceleration_status.ok()) {
          leader_debug_info << "/***/CheckDeceleration:"
                            << check_deceleration_str;
        }
      }
      if (!check_deceleration_status.ok()) {
        unsafe_object_id = std::string(traj_ptr->object_id());
        status_code =
            obs_in_target_lane
                ? (if_obs_front
                       ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                       : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_TARGET_LANE)
                : (if_obs_front ? PlannerStatusProto::
                                      FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                                : PlannerStatusProto::
                                      REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
        leader_debug_info << static_obs_debug.str();
        follower_debug_info << dynamic_obs_debug.str();
        return absl::CancelledError(check_deceleration_status.ToString());
      }
    }

    // TTC is used to make safety judgments for large vehicles at the rear.
    double ttc_threshold_for_scene = 0.0;
    double ttc_for_scene = 0.0;
    bool ttc_for_scene_valid = false;
    if (is_distance_shortening) {
      const std::vector<double> K_FRONT_TTC_VEC = {0.83, 0.86, 0.90, 0.93,
                                                   0.96, 1.0,  1.2};
      const std::vector<double> K_BACK_TTC_VEC = {0.83, 0.86, 0.90, 0.93,
                                                  0.96, 1.0,  1.2};
      const std::vector<double> V_TTC_VEC = {0.0, 15, 20, 30, 60, 90, 120};
      const std::vector<double> TTC_VEC = {3.0, 3.0, 3.0, 3.0, 3.5, 4.0, 4.5};

      const std::vector<double> DV_TTC_VEC = {0.0, 5.0, 10.0, 20.0};
      const std::vector<double> TTC_VEC_FAC = {0.0, 0.3, 0.8, 1.0};
      double ttc_factor = ad_byd::planning::math::interp1_inc(
          DV_TTC_VEC, TTC_VEC_FAC, abs_rel_vs_kph);

      double front_ttc = ad_byd::planning::math::interp1_inc(V_TTC_VEC, TTC_VEC,
                                                             ego_lon_v_kph) +
                         ttc_factor - 1.5;
      double back_ttc = ad_byd::planning::math::interp1_inc(V_TTC_VEC, TTC_VEC,
                                                            ego_lon_v_kph) +
                        ttc_factor;

      bool quick_response_type = false;
      bool is_jam_scen = false;
      double spd_diff_kph_threshold = 100;
      if (is_distance_shortening) {
        if (if_obs_front) {
          if (quick_response_type) {
            spd_diff_kph_threshold = 100;
          } else {
            spd_diff_kph_threshold = 40;
          }
        } else {
          if (is_ego_stationary) {
            spd_diff_kph_threshold = 8;
          } else if (is_ego_extra_slow) {
            spd_diff_kph_threshold = 10;
          } else {
            if (quick_response_type) {
              spd_diff_kph_threshold = 30;
            } else {
              spd_diff_kph_threshold = 30;
            }
          }
        }
        if (is_lc_state_prev) spd_diff_kph_threshold += 5;
      } else {
        spd_diff_kph_threshold = 100;
      }

      bool large_spd_diff_scene =
          (is_distance_shortening && abs_rel_vs_kph > (spd_diff_kph_threshold));

      bool rear_large_spd_acc_scene =
          (is_distance_shortening &&
           abs_rel_vs_kph > (spd_diff_kph_threshold)*0.8 && obj_lon_acc > 2.0 &&
           !if_obs_front);
      bool is_rear_big_approaching_vehicle =
          (is_big_vehicle && is_rear_approaching_vehicle);

      double k_front_ttc = ad_byd::planning::math::interp1_inc(
          V_TTC_VEC, K_FRONT_TTC_VEC, ego_lon_v_kph);

      double k_back_ttc = ad_byd::planning::math::interp1_inc(
          V_TTC_VEC, K_BACK_TTC_VEC, Mps2Kph(obj_lon_v));

      // 根据相对速度差来动态调整距离缩小目标的ttc计算的系数
      std::vector<double> f_dv_front_ttc_vec = {
          (front_ttc - 1.0), (front_ttc + 0.0), (front_ttc + 0.5),
          (front_ttc + 0.8), (front_ttc + 1.0), (front_ttc + 1.5)};
      std::vector<double> f_dv_back_ttc_vec = {
          (back_ttc - 0.0), (back_ttc + 0.5), (back_ttc + 1.0),
          (back_ttc + 1.5), (back_ttc + 2.0), (back_ttc + 2.5)};
      std::vector<double> dv_ttc_vec = {0.0, 20, 30, 45, 60, 120};

      double f_dv_front_ttc =
          std::max(ad_byd::planning::math::interp1_inc(
                       dv_ttc_vec, f_dv_front_ttc_vec, abs_rel_vs_kph),
                   front_ttc);

      double f_dv_back_ttc =
          std::max(ad_byd::planning::math::interp1_inc(
                       dv_ttc_vec, f_dv_back_ttc_vec, abs_rel_vs_kph),
                   back_ttc);

      // adjust the TTC factor by scenes for rear obstacles
      double back_ttc_factor_by_scenes = 1.0;
      double kBackTTCRelaxDistanceThreshold =
          is_on_highway ? kBackTTCRelaxDistanceThresholdHighway
                        : kBackTTCRelaxDistanceThresholdCity;
      std::string ttc_factor_scene_debug_str = "none";
      if (!if_obs_front) {
        if (std::fabs(rel_ds) > kBackTTCRelaxDistanceThreshold) {
          back_ttc_factor_by_scenes = Lerp(1.0, kBackTTCRelaxDistanceThreshold,
                                           0.5, 60.0, std::fabs(rel_ds), true);
          ttc_factor_scene_debug_str = "far_follower";
        } else if (is_rear_big_approaching_vehicle) {
          back_ttc_factor_by_scenes = 1.0;
          ttc_factor_scene_debug_str = "big_appr_veh";
        } else if (!is_vru && obj_is_leaving_target) {
          double obs_lc_offset =
              obj_lc_left
                  ? std::abs(std::max(obj_cur_frenet_box.center_l(), 0.0))
                  : std::abs(std::min(obj_cur_frenet_box.center_l(), 0.0));
          double obs_lc_offset_factor =
              Lerp(1.0, 0.4, 0.7, 1.6, obs_lc_offset, true);
          back_ttc_factor_by_scenes = obs_lc_offset_factor;
          ttc_factor_scene_debug_str =
              absl::StrFormat("obs_lc_offset:%.3f", obs_lc_offset);
        } else {
          double lat_offset =
              lc_left ? std::abs(std::min(0.0, ego_cur_frenet_box.l_max))
                      : std::abs(std::max(0.0, ego_cur_frenet_box.l_min));
          double obs_dl_factor = Lerp(0.5, 0.3, 1.0, 2.0, lat_offset, true);
          back_ttc_factor_by_scenes = obs_dl_factor;
          ttc_factor_scene_debug_str =
              absl::StrFormat("ego_lat_offset:%.3f", lat_offset);
        }
      }
      k_back_ttc *= back_ttc_factor_by_scenes;

      double ttc_debounce_factor = is_lc_state_prev ? 0.8 : 1.0;
      k_front_ttc *= ttc_debounce_factor;
      k_back_ttc *= ttc_debounce_factor;

      front_ttc =
          std::max(k_front_ttc * f_dv_front_ttc, kLaneChangeMinLimitTTC);
      back_ttc = std::max(k_back_ttc * f_dv_back_ttc, kLaneChangeMinLimitTTC);

      if (is_jam_scen || large_spd_diff_scene) {
        if (!is_lc_state_prev) {
          front_ttc += (1.5 * conserv_base);
          back_ttc += (2.0 * conserv_base);
        } else {
          front_ttc += (0.5 * conserv_base);
          back_ttc += (0.5 * conserv_base);
        }
      } else {
        if (!is_lc_state_prev) {
          front_ttc += (0.7 * conserv_base);
          back_ttc += (1.0 * conserv_base);
        } else {
          front_ttc += (0.2 * conserv_base);
          back_ttc += (0.2 * conserv_base);
        }
      }

      double ttc_thrd = (if_obs_front ? front_ttc : back_ttc);
      double ttc_rel = 0.0;
      bool ttc_safe = false;
      std::string ttc_debug_str = "";
      if (if_obs_front) {
        double ttc_distance = std::fabs(rel_vs) * ttc_thrd;
        ttc_rel = std::fabs(rel_ds) / (std::fabs(rel_vs) + kEpsilon);
        bool ttc_dis_danger = std::fabs(rel_ds) < ttc_distance;
        ttc_threshold_for_scene = ttc_thrd;
        ttc_for_scene = ttc_rel;
        ttc_for_scene_valid = true;

        // Advanced Check for preview state safety
        bool preview_ttc_dis_danger = ttc_dis_danger;
        double danger_preview_time = 0.0;
        double total_preview_time = 0.0;
        double follower_preview_acc = 0.0;
        if (!ttc_dis_danger) {
          double ego_lat_offset =
              lc_left ? std::abs(std::min(0.0, ego_cur_frenet_box.l_max))
                      : std::abs(std::max(0.0, ego_cur_frenet_box.l_min));
          const double ttc_preview_t_step = 0.3;
          const double ttc_preview_time_max = 1.5;
          total_preview_time =
              Lerp(ttc_preview_t_step, (cur_half_lane_width - ego_half_width),
                   ttc_preview_time_max, (cur_lane_width - ego_half_width),
                   ego_lat_offset, true);
          double obj_preview_acc = obj_lon_acc;
          double ego_preview_acc =
              is_vru ? 0.0
                     : std::min(ego_lon_acc, Lerp(-1.5, 1.2, -2.5, 0.5,
                                                  lc_style_factor, true));
          follower_preview_acc = ego_preview_acc;
          for (double t = ttc_preview_t_step;
               t < total_preview_time + 0.3 * ttc_preview_t_step;
               t += ttc_preview_t_step) {
            double ttc_thrd_relax_factor =
                Lerp(0.1, ttc_preview_t_step, (is_lc_state_prev ? 0.3 : 0.2),
                     ttc_preview_time_max, t, true);
            double preview_ttc_thrd = ttc_thrd * (1.0 - ttc_thrd_relax_factor);
            double ego_preview_v = ego_lon_v + ego_preview_acc * t;
            double obj_preview_v = obj_lon_v + obj_preview_acc * t;
            double preview_rel_vs = obj_preview_v - ego_preview_v;
            double preview_rel_ds =
                rel_ds + 0.5 * (rel_vs + preview_rel_vs) * t;
            danger_preview_time = t;
            if (preview_rel_ds * preview_rel_vs < 0.0) {
              double preview_ttc_dist =
                  std::fabs(preview_rel_vs) * preview_ttc_thrd;
              if (std::fabs(preview_rel_ds) < preview_ttc_dist) {
                preview_ttc_dis_danger = true;
                break;
              }
            }
          }
        }
        ttc_safe = !ttc_dis_danger && !preview_ttc_dis_danger;

        dynamic_obs_debug << "-ttc:" << ttc_dis_danger
                          << ",preview_ttc:" << preview_ttc_dis_danger;
        ttc_debug_str = absl::StrFormat(
            "/***/Check_ttc:%d %d. obstacle %s will collide with ego after "
            "%.3f seconds (ttc_thrd:%.3f ttc distance:%.3f preview_t:%.2f/%.2f "
            "preview_flr_a:%.2f rel_vs:%.3f conserv_base:%.3f "
            "back_scene_fct:%.3f scene_fct_str:%s).",
            ttc_dis_danger, preview_ttc_dis_danger, traj_ptr->object_id(),
            ttc_rel, ttc_thrd, ttc_distance, danger_preview_time,
            total_preview_time, follower_preview_acc, rel_vs, conserv_base,
            back_ttc_factor_by_scenes, ttc_factor_scene_debug_str.c_str());
      } else {
        // if (is_rear_obj_enter_target_in_same_side) {
        //   ttc_thrd *= 1.5;
        // }
        // get preview time for ttc
        // constexpr double max_ttc_preview_time = 1.0;  // s
        double max_ttc_preview_time = ad_byd::planning::math::interp1_inc(
            std::vector<double>{5.0, 10.0, 30.0},
            std::vector<double>{0.0, 0.5, 1.0}, std::abs(rel_ds));
        if (is_big_vehicle) {
          max_ttc_preview_time = 0.0;
          double big_vehicle_acc_ttc_gain = ad_byd::planning::math::interp1_inc(
              std::vector<double>{0.0, 0.5, 1.5},
              std::vector<double>{1.0, 1.2, 1.5}, obj_lon_acc);
          ttc_thrd *= 1.2 * big_vehicle_acc_ttc_gain;
        }
        double ttc_preview_time =
            std::min(ego_enter_target_time, max_ttc_preview_time);
        // get avbox in ttc preview time
        int tgt_index =
            std::min(static_cast<int>(ttc_preview_time / kTrajectoryTimeStep),
                     ego_traj_size - 1);
        auto max_ttc_preview_ego_box = ego_boxes[tgt_index];
        ASSIGN_OR_RETURN(
            const auto max_ttc_preview_ego_frenet_box,
            target_frenet_frame.QueryFrenetBoxAt(max_ttc_preview_ego_box),
            _ << "Cannot project the max_ttc_preview_ego_box onto drive "
                 "passage.");

        double obj_preview_acc =
            // is_rear_obj_enter_target_in_same_side
            //     ? 0.0
            //     :
            is_vru
                ? 0.0
                : std::max(-hypo_braking_deceleration,
                           Lerp(-0.8, 1.2, -1.5, 0.5, lc_style_factor, true));
        obj_preview_acc = std::min(obj_preview_acc, obj_lon_acc);
        double obj_preview_travel_dist = 0.0;
        double obj_preview_v = obj_lon_v;
        if (std::fabs(obj_preview_acc) < kEpsilon) {
          obj_preview_travel_dist = obj_lon_v * ttc_preview_time;
        } else {
          double obj_brake_time =
              std::min(ttc_preview_time, -obj_lon_v / obj_preview_acc);
          obj_preview_v = obj_lon_v + obj_preview_acc * obj_brake_time;
          obj_preview_travel_dist =
              0.5 * (obj_lon_v + obj_preview_v) * obj_brake_time;
        }
        double preview_rel_ds = max_ttc_preview_ego_frenet_box.s_min -
                                obj_cur_frenet_box.s_max -
                                obj_preview_travel_dist;
        double preview_rel_vs = obj_preview_v - ego_lon_v;
        if (preview_rel_ds < kEpsilon) {
          ttc_rel = 0.0;
          ttc_safe = false;
        } else {
          ttc_rel = preview_rel_vs < kEpsilon ? DBL_MAX
                                              : preview_rel_ds / preview_rel_vs;
          ttc_safe = ttc_rel > ttc_thrd;
        }

        if (preview_rel_vs > kEpsilon) {
          ttc_threshold_for_scene = ttc_thrd;
          ttc_for_scene = ttc_rel;
          ttc_for_scene_valid = true;
        }

        dynamic_obs_debug << "-prv_ttc:" << ttc_rel;
        std::ostringstream ttc_str;
        ttc_str << ttc_rel;
        ttc_debug_str = absl::StrFormat(
            "/***/Check_ttc:%d. obstacle %s will collide with ego after %s "
            "seconds (ttc_thrd:%.3f prv_t:%.2f prv_ego_s_min:%.3f "
            "cur_obj_s_max:%.3f prv_obj_ds:%.3f prv_obj_a:%.2f prv_rel_ds:%.3f "
            "prv_rel_vs:%.3f conserv_base:%.3f back_scene_fct:%.3f "
            "scene_fct_str:%s).",
            !ttc_safe, traj_ptr->object_id(), ttc_str.str(), ttc_thrd,
            ttc_preview_time, ego_enter_target_frenet_box.s_min,
            obj_cur_frenet_box.s_max, obj_preview_travel_dist, obj_preview_acc,
            preview_rel_ds, preview_rel_vs, conserv_base,
            back_ttc_factor_by_scenes, ttc_factor_scene_debug_str.c_str());
      }

      if (if_obs_front && frnt_obs_id == traj_ptr->object_id()) {
        leader_debug_info << ttc_debug_str;
      } else if (if_obs_rear && rear_obs_id == traj_ptr->object_id()) {
        follower_debug_info << ttc_debug_str;
      }

      if (!ttc_safe) {
        if (!if_obs_front && is_gaming_lc_scenario) {
          // Ignore the object if lc_safety_checker considers it safe
          auto gaming_lc_safety_status = gaming_lc_safety_checker.Execute(
              gaming_lc_safety_input, traj_ptr->traj_id());
          if (gaming_lc_safety_status.ok()) {
            gaming_lc_obs_set.insert(gaming_lc_safety_status.value());
            dynamic_obs_debug << "-GamingSafe";
            continue;
          }
        }
        unsafe_object_id = std::string(traj_ptr->object_id());
        status_code =
            obs_in_target_lane
                ? (if_obs_front
                       ? PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_TARGET_LANE
                       : PlannerStatusProto::REAR_DANGEROUS_VEHICLE_TARGET_LANE)
                : (if_obs_front ? PlannerStatusProto::
                                      FRONT_DANGEROUS_VEHICLE_CURRENT_LANE
                                : PlannerStatusProto::
                                      REAR_DANGEROUS_VEHICLE_CURRENT_LANE);
        leader_debug_info << static_obs_debug.str();
        follower_debug_info << dynamic_obs_debug.str();
        return absl::CancelledError(ttc_debug_str);
      }
    }

    if (is_rear_obj_enter_target_in_same_side) {
      // add scene[REAR_OBJ_IN_SAME_DIR] for lc style decider
      LaneChangeStylePostDeciderSceneInfo lc_scene_info;
      lc_scene_info.set_obj_info(obj_lcs_decider_info, ego_lcs_decider_info);
      lc_scene_info.set_scene_type(
          LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_IN_SAME_DIR);
      lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
          LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
          std::fabs(rel_ds), obj_front_extension));
      lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
          LaneChangeSafetyCheckingCondition::CHECK_COND_LAT_DL, lat_overlap,
          lat_overlap_thrd));
      lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
          LaneChangeSafetyCheckingCondition::CHECK_COND_FOLLOW_DECEL,
          std::fabs(hypo_braking_deceleration),
          std::fabs(max_braking_deceleration)));
      if (is_distance_shortening && ttc_for_scene_valid) {
        lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
            LaneChangeSafetyCheckingCondition::CHECK_COND_TTC, ttc_for_scene,
            ttc_threshold_for_scene));
      }
      scene_infos_ptr->emplace_back(lc_scene_info);
    } else if (if_obs_front &&
               obj_cur_frenet_box.s_min < target_front_obj_scene_s_min) {
      target_front_obj_scene_s_min = obj_cur_frenet_box.s_min;
      // add scene[FRONT_OBJ_ON_TARGET_LANE] for lc style decider
      target_front_obj_scene_info_ptr->set_obj_info(obj_lcs_decider_info,
                                                    ego_lcs_decider_info);
      target_front_obj_scene_info_ptr->set_scene_type(
          LaneChangeStylePostDeciderSceneType::SCENE_FRONT_OBJ_ON_TARGET_LANE);
      target_front_obj_scene_info_ptr->ClearSafetyCheckingInfo();
      target_front_obj_scene_info_ptr->add_safety_checking_info(
          LaneChangeSafetyCheckingInfo(
              LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
              std::fabs(rel_ds), obj_front_extension));
      target_front_obj_scene_info_ptr->add_safety_checking_info(
          LaneChangeSafetyCheckingInfo(
              LaneChangeSafetyCheckingCondition::CHECK_COND_FOLLOW_DECEL,
              std::fabs(hypo_braking_deceleration),
              std::fabs(max_braking_deceleration)));
      if (is_distance_shortening && ttc_for_scene_valid) {
        target_front_obj_scene_info_ptr->add_safety_checking_info(
            LaneChangeSafetyCheckingInfo(
                LaneChangeSafetyCheckingCondition::CHECK_COND_TTC,
                ttc_for_scene, ttc_threshold_for_scene));
      }
    } else if (!if_obs_front) {
      // add scene[REAR_OBJ_ON_TARGET_LANE] for lc style decider
      LaneChangeStylePostDeciderSceneInfo lc_scene_info;
      lc_scene_info.set_obj_info(obj_lcs_decider_info, ego_lcs_decider_info);
      lc_scene_info.set_scene_type(
          LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_ON_TARGET_LANE);
      lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
          LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
          std::fabs(rel_ds), obj_front_extension));
      lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
          LaneChangeSafetyCheckingCondition::CHECK_COND_FOLLOW_DECEL,
          std::fabs(hypo_braking_deceleration),
          std::fabs(max_braking_deceleration)));
      if (is_distance_shortening && ttc_for_scene_valid) {
        lc_scene_info.add_safety_checking_info(LaneChangeSafetyCheckingInfo(
            LaneChangeSafetyCheckingCondition::CHECK_COND_TTC, ttc_for_scene,
            ttc_threshold_for_scene));
      }
      scene_infos_ptr->emplace_back(lc_scene_info);
    }
  }

  if (ego_slow_front_obj_scene_info.scene_type() ==
      LaneChangeStylePostDeciderSceneType::SCENE_FRONT_SLOW_OBJ_ON_EGO_LANE) {
    // add scene[FRONT_SLOW_OBJ_ON_EGO_LANE] for lc style decider
    scene_infos_ptr->emplace_back(ego_slow_front_obj_scene_info);
  }

  // Make sure the vehicle closest to the target lane is set as leader_set to
  // ensure lane change safety
  follower_set.insert(rear_obs_id);
  leader_set.insert(frnt_obs_id);

  // log the reason why ego can't lc return
  std::string can_not_return_debug =
      absl::StrCat("-theta_can_not_return:", theta_can_not_return,
                   "-dist_can_not_return:", dis_can_not_return);
  Log2DDS::LogDataV2("lanechange return scene:", can_not_return_debug);

  // **********************Check for stationary objects.************************
  constexpr double kStationaryBuffer = 0.3;  // m.
  st::FrenetBox frnt_static_current{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  st::FrenetBox frnt_static_target{DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  std::string frnt_static_current_id = "";
  std::string frnt_static_target_id = "";

  static_obs_debug << " frnt_stc_fltr:" << frnt_static_filter_obs_id << " ";

  // Check for cones riding the middle line
  std::vector<std::string> ride_line_uu_ids_list;
  double ride_line_uu_ds = DBL_MAX;
  std::string ride_line_uu_debug_str = "";
  CheckForConesRidingMiddleLine(
      target_frenet_frame, st_traj_mgr, ego_cur_frenet_box, lc_left,
      ride_line_uu_ids_list, ride_line_uu_ds, ride_line_uu_debug_str);
  static_obs_debug << "<" << (is_on_highway ? "HW," : "CY,")
                   << ride_line_uu_debug_str << ">";
  double abs_ride_line_uu_ds = std::fabs(ride_line_uu_ds);
  int ride_line_uu_cnt = ride_line_uu_ids_list.size();
  const double high_confi_ride_line_uu_dist_thrd_for_hw = 10.0;
  const double midd_confi_ride_line_uu_dist_thrd_for_hw = 30.0;
  const double confi_ride_line_uu_dist_thrd_for_city = 50.0;
  if ((is_on_highway &&
       (ride_line_uu_cnt >= 3 ||
        (ride_line_uu_cnt >= 2 &&
         abs_ride_line_uu_ds < midd_confi_ride_line_uu_dist_thrd_for_hw) ||
        (ride_line_uu_cnt >= 1 &&
         abs_ride_line_uu_ds < high_confi_ride_line_uu_dist_thrd_for_hw))) ||
      (!is_on_highway && ride_line_uu_cnt >= 3 &&
       abs_ride_line_uu_ds < confi_ride_line_uu_dist_thrd_for_city)) {
    unsafe_object_id = ride_line_uu_ids_list.front();
    status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
    leader_debug_info << static_obs_debug.str();
    // The scene is detected and the count is reset to one.
    cones_riding_line_frames = 1;
    return absl::CancelledError(
        absl::StrFormat("Many uu or cone static obstacles ride line. %s",
                        ride_line_uu_debug_str.c_str()));
  }
  // If the cones-riding-line frame count is less than the threshold, it is
  // considered that the timeout has not occurred and the scene will remain
  // valid.
  int cones_riding_line_frame_thrd = -1;
  constexpr double kConesDistanceFramesThrd = 100.0;  // =cones_dist*plan_freq
  if (ego_lon_v_kph > kStationaryVehicleSpeedkph) {
    cones_riding_line_frame_thrd = static_cast<int>(
        std::round(kConesDistanceFramesThrd / std::fabs(ego_lon_v)));
  }
  if (cones_riding_line_frames > 0 &&
      (cones_riding_line_frame_thrd < 0 ||
       cones_riding_line_frames <= cones_riding_line_frame_thrd)) {
    unsafe_object_id =
        ride_line_uu_cnt > 0
            ? ride_line_uu_ids_list.front()
            : (st_traj_mgr.stationary_object_trajs().empty()
                   ? ""
                   : std::string(st_traj_mgr.stationary_object_trajs()
                                     .front()
                                     ->object_id()));
    status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
    leader_debug_info << static_obs_debug.str();
    return absl::CancelledError(absl::StrFormat(
        "Keep static obstacles ride line (frames:%d,thrd:%d). %s",
        cones_riding_line_frames, cones_riding_line_frame_thrd,
        ride_line_uu_debug_str.c_str()));
  }
  // The frame count will be reset to zero due to a timeout
  else {
    cones_riding_line_frames = 0;
  }

  for (const auto* traj_ptr : st_traj_mgr.stationary_object_trajs()) {
    static_obs_debug << "-->Sobj:" << traj_ptr->object_id();

    conserv = conserv_base * getObjTypeFactor(traj_ptr, true);
    // ASSIGN_OR_CONTINUE(
    //     const auto obj_cur_box,
    //     FindPredictedObjectBox(traj_ptr->states(), path_start_time_offset));

    // Use contour rather than bounding box for stationary objects since their
    // heading might be largely deviated.l
    const bool is_vehicle = IsVehicle(traj_ptr);
    const bool is_vru = IsVRU(traj_ptr);
    const auto& obj_contour =
        !is_vehicle
            ? traj_ptr->contour()
            : Polygon2d(traj_ptr->bounding_box().GetCornersCounterClockwise());
    auto obj_frenet_box_or_not =
        target_frenet_frame.QueryFrenetBoxAtContour(obj_contour);
    if (!obj_frenet_box_or_not.ok()) {
      static_obs_debug << "-fnboxfail";
      continue;
    }
    const auto obj_frenet_box = obj_frenet_box_or_not.value();

    // if (lc_state == LaneChangeStage::LCS_RETURN && is_vehicle &&
    //     stalled_objects.contains(traj_ptr->object_id())) {
    //   static_obs_debug << "|stall";
    //   continue;
    // }

    const bool if_obs_front = (ego_cur_frenet_box.s_max < obj_frenet_box.s_min);
    const bool if_obs_rear = (obj_frenet_box.s_max < ego_cur_frenet_box.s_min);

    double rel_ds, rel_dl = 0;
    double long_overlap =
        getLongOverlap(ego_cur_frenet_box, obj_frenet_box, rel_ds);
    double lat_overlap =
        getLatOverlap(ego_cur_frenet_box, obj_frenet_box, rel_dl);
    const auto obj_cur_box = traj_ptr->states().front().box;
    const auto obj_target_tangent_cos = obj_cur_box.tangent().Dot(
        target_frenet_frame.InterpolateTangentByS(obj_frenet_box.center_s()));
    const auto obj_target_normal_cos = obj_cur_box.tangent().Dot(
        target_frenet_frame.InterpolateTangentByS(obj_frenet_box.center_s())
            .Rotate(M_PI_2));

    // Create obj info for lc style decider
    LaneChangeStyleDeciderObjectInfo obj_lcs_decider_info(
        (std::string)traj_ptr->object_id(), 0.0, 0.0, obj_frenet_box);

    if (rel_ds < -6.0 || rel_ds > 135 ||
        (lc_left ? rel_dl < -0.5 : rel_dl > 0.5) || std::fabs(rel_dl) > 6.0) {
      static_obs_debug << "|skp";
      continue;
    }
    // When changing lanes, consider the safe distance of the stationary vehicle
    // in front of the current lane to prevent the risk of collision when
    // changing lanes.
    double lat_overlap_lc = getLatOverlapWithLcDirection(
        ego_front_frenet_box, obj_frenet_box, rel_dl);
    const double obj_v = traj_ptr->planner_object().pose().v();
    const double obj_lon_v = obj_v * obj_target_tangent_cos;
    const double obj_lat_v = obj_v * obj_target_normal_cos;
    bool obs_in_target_lane =
        HasFullyEnteredTargetLane(obj_frenet_box, 0.5 * obj_frenet_box.width());

    double ttc_thrd = is_lc_state_prev ? 6.0 : 8.0;
    double ttc_dis_danger =
        rel_ds <
        std::min(135.0, std::fmax(ego_lon_v * ttc_thrd * conserv_base, 5.0));

    bool is_construction =
        IsConstructionObs(traj_ptr) && IsSizeQualified(traj_ptr);

    conserv = std::clamp(conserv, 0.5, 1.0);

    bool static_occupy_target =
        is_construction ? isStaticConstructionObsOccupyTargetLane(
                              obj_frenet_box, cur_half_lane_width, lc_left)
                        : isStaticObsOccupyTargetLane(
                              traj_ptr, obj_frenet_box, ego_cur_frenet_box,
                              0.5 * obj_frenet_box.width(), lc_left, rel_ds);

    bool target_lane_construction = is_construction && if_obs_front &&
                                    static_occupy_target && ttc_dis_danger;

    double frt_obj_rel_ds, frt_obj_rel_dl = 0;

    // Update the nearest vehicle information in front of the target lane
    if ((is_vehicle && if_obs_front && obs_in_target_lane) &&
        (frnt_static_filter_obs_id.empty() ||
         obj_frenet_box.s_min < frnt_static_filter_obs_box.s_min)) {
      frnt_static_filter_obs_id = std::string(traj_ptr->object_id());
      frnt_static_filter_obs_box = obj_frenet_box;
    }

    if (!frnt_static_filter_obs_id.empty() &&
        frnt_static_filter_obs_id != traj_ptr->object_id()) {
      auto frt_obj_long_overlap = getLongOverlap(
          frnt_static_filter_obs_box, obj_frenet_box, frt_obj_rel_ds);
      auto frt_obj_lat_overlap = getLatOverlap(frnt_static_filter_obs_box,
                                               obj_frenet_box, frt_obj_rel_dl);
    }

    // Filter static targets with external false detection of the nearest
    // dynamic target in front of the target lane
    if (!frnt_static_filter_obs_id.empty() && target_lane_construction &&
        frt_obj_rel_ds > -kEpsilon && std::fabs(frt_obj_rel_dl) < 0.3) {
      static_obs_debug << "|frt_obj:" << frnt_static_filter_obs_id
                       << "-rel_ds:" << frt_obj_rel_ds
                       << "-rel_dl:" << frt_obj_rel_dl;
      // static_obs_debug << "|obj-smax:" << obj_frenet_box.s_max
      //                  << "-smin:" << obj_frenet_box.s_min
      //                  << "-lmax:" << obj_frenet_box.l_max
      //                  << "-lmin:" << obj_frenet_box.l_min;
      // static_obs_debug << "|ftr-smax:" << frnt_static_filter_obs_box.s_max
      //                  << "-smin:" << frnt_static_filter_obs_box.s_min
      //                  << "-lmax:" << frnt_static_filter_obs_box.l_max
      //                  << "-lmin:" << frnt_static_filter_obs_box.l_min;
      static_obs_debug << "|FDt";
      continue;
    }

    bool ride_middle_line = isStaticObsRideLine(
        ego_cur_frenet_box, obj_frenet_box, lc_left, rel_ds);

    bool ride_line_condition =
        is_construction && ride_middle_line && rel_ds >= -6.0;

    bool occupy_current_condition =
        !static_occupy_target && ego_enter_target_idx != 0 &&
        (is_vehicle || is_construction) && if_obs_front &&
        std::fabs(rel_dl) < kEpsilon &&
        Mps2Kph(obj_v) < kStationaryVehicleSpeedkph;

    static_obs_debug << "|c:" << occupy_current_condition
                     << "-r:" << ride_middle_line
                     << "-o:" << static_occupy_target;

    static_obs_debug << "|cc:" << occupy_current_condition
                     << "-rc:" << ride_line_condition
                     << "-tc:" << target_lane_construction;

    // if (ride_line_condition) {
    //   ride_line_uu_num++;
    //   ride_line_debug << traj_ptr->object_id() << "->";
    // }

    if (occupy_current_condition || ride_line_condition) {
      min_lon_buffer = 5.0 * 1.0;

      double compensation_heading_dis =
          ego_start_target_normal_cos * ego_cur_box.half_length() * 2;
      double heading_factor = std::fabs(
          (compensation_heading_dis /
           (std::fabs(lat_overlap_lc) > kEpsilon ? lat_overlap_lc : kEpsilon)));
      double nudge_heading_factor = std::clamp(1 - heading_factor, 0.0, 1.0);
      std::vector<double> lat_threshold = {0.2, 1.0, 1.5, 2.0, 4.0};
      std::vector<double> lon_dis = {0.5, 3.0, 4.8, 6.0, 10.0};
      auto lat_overlap_safe_dis = ad_byd::planning::math::interp1_inc(
          lat_threshold, lon_dis, lat_overlap_lc * nudge_heading_factor);

      double lat_overlap_factor =
          std::clamp(lat_overlap_lc <= 1.5 * vehicle_geom.width()
                         ? (lat_overlap_lc / vehicle_geom.width())
                         : (lat_overlap_lc + vehicle_geom.width()) /
                               (2 * vehicle_geom.width()),
                     0.0, 2.5);

      double state_debounce_factor =
          ((prev_lc_stage != LaneChangeStage::LCS_EXECUTING ||
            std::fabs(ego_target_normal_cos) < 0.06)
               ? 1.0
               : 0.8);

      auto front_safe_dis =
          std::max(lat_overlap_safe_dis,
                   ego_lon_v * ((is_vehicle && if_obs_front) ? 2.5 : 2.0) *
                       conserv_base * lat_overlap_factor *
                       nudge_heading_factor * state_debounce_factor);
      if (prev_lc_stage == LaneChangeStage::LCS_EXECUTING) {
        front_safe_dis = front_safe_dis * 0.8;
      }
      bool front_obs_too_close = rel_ds < front_safe_dis;

      auto front_veh_debug_str = absl::StrFormat(
          "stationary obs %s currently "
          "close:%d.lat_overlap:%.3f lat_overlap_lc:%.3f "
          "front_safe_dis:%.3f rel_ds:%.3f conserv_base:%.3f"
          "conserv:%.3f lat_overlap_safe_dis:%.3f ego_v:%.3f "
          "nudge_heading_factor:%.3f",
          traj_ptr->object_id(), front_obs_too_close, lat_overlap,
          lat_overlap_lc, front_safe_dis, rel_ds, conserv_base, conserv,
          lat_overlap_safe_dis, ego_v, nudge_heading_factor);

      // static_obs_debug << front_veh_debug_str;

      if (front_obs_too_close) {
        unsafe_object_id = std::string(traj_ptr->object_id());
        if (is_construction) {
          status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
        } else {
          status_code =
              PlannerStatusProto::FRONT_DANGEROUS_VEHICLE_CURRENT_LANE;
        }

        leader_debug_info << static_obs_debug.str();
        return absl::CancelledError(front_veh_debug_str);
      }

      // temporarily add scene[FRONT_STATIC_OBJ_ON_EGO_LANE] for lc style
      // decider the closest obj will be added at the end
      if (obj_frenet_box.s_min < ego_static_front_obj_scene_s_min) {
        ego_static_front_obj_scene_s_min = obj_frenet_box.s_min;
        ego_static_front_obj_scene_info.set_obj_info(obj_lcs_decider_info,
                                                     ego_lcs_decider_info);
        ego_static_front_obj_scene_info.set_scene_type(
            LaneChangeStylePostDeciderSceneType::
                SCENE_FRONT_STATIC_OBJ_ON_EGO_LANE);
        ego_static_front_obj_scene_info.add_safety_checking_info(
            LaneChangeSafetyCheckingInfo(
                LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
                std::fabs(rel_ds), front_safe_dis));
      }
    }
    /* else if (target_lane_construction) {
      if (static_occupy_target) {
        unsafe_object_id = std::string(traj_ptr->object_id());
        status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
        leader_debug_info << static_obs_debug.str();
        return absl::CancelledError(
            absl::StrFormat("UU or cone %s currently occupy target lane.",
                            traj_ptr->object_id()));
      }
    } */
    /* else if (ride_line_uu_num >= 3 && ride_line_condition) {
       unsafe_object_id = std::string(traj_ptr->object_id());
       status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
       leader_debug_info << static_obs_debug.str();
       return absl::CancelledError(
           absl::StrFormat("Many uu or cone static obstacles ride line %s. %s",
                           traj_ptr->object_id(), ride_line_debug.str()));
    } */
    else {
      min_lon_buffer = kMinLonBufferToFront;
    }

    // Ignore stationary objects behind ego or not on the target lane.
    if (obj_frenet_box.s_max < ego_cur_frenet_box.s_min ||
        std::abs(obj_frenet_box.center_l()) >
            0.5 * obj_frenet_box.width() + ego_half_width + kStationaryBuffer) {
      static_obs_debug << "|skp2";
      continue;
    }

    for (int i = 0; i < ego_traj_size; ++i) {
      if (static_occupy_target || obj_contour.HasOverlap(ego_boxes[i])) {
        static_obs_debug << "|overlap:" << i << "-occ:" << static_occupy_target;
        const double rel_head_way_time =
            if_obs_front ? ComputeEgoFollowTime(obj_lon_v, ego_lon_v,
                                                prev_lc_stage, min_lon_buffer)
                         : ComputeEgoLeadTime(speed_limit, ego_lon_v, obj_lon_v,
                                              prev_lc_stage, min_lon_buffer,
                                              is_on_highway);
        const auto follower_lon_vel = if_obs_front ? ego_lon_v : obj_lon_v;
        const auto follower_lat_vel = if_obs_front ? ego_lat_v : obj_lat_v;
        // Check if the object is currently abreast of the ego vehicle.
        const double obj_front_extension =
            conserv * follower_lon_vel * rel_head_way_time;
        const double obj_lat_extension =
            -conserv * follower_lat_vel * rel_head_way_time;

        bool is_abreast = IsBlockingObjectAbreast(
            ego_cur_frenet_box, obj_frenet_box, obj_front_extension,
            obj_lat_extension, min_lon_buffer, false, false);
        static_obs_debug << "|abr:" << is_abreast;

        if (is_abreast) {
          // Not safe if colliding object lies abreast of the ego vehicle.
          unsafe_object_id = std::string(traj_ptr->object_id());
          status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
          leader_debug_info << static_obs_debug.str();
          return absl::CancelledError(absl::StrFormat(
              "Object %s currently abreast.", traj_ptr->object_id()));
        }

        bool ego_is_entered = (ego_enter_target_idx == 0 && ego_occupy_target);
        static_obs_debug << "|etr:" << ego_is_entered;
        if (ego_is_entered || ego_centered_in_target_lane) {
          // If already entered target lane, only check for very dangrerous
          // situation where some vehicle is too close.
          break;
        }

        // // ignore if it is not the nearest leader or a closer blocking object
        // if (nearest_stationary_leader_id !=
        //         std::string(traj_ptr->object_id()) &&
        //     is_stationary_leader_box_ready &&
        //     obj_frenet_box.s_min > stationary_leader_frenet_box.s_min) {
        //   static_obs_debug << "|Nnl";
        //   break;
        // }

        // // check if there is a closer blocking object between ego and leader
        // // if no leading, there must be no any front objects
        // double front_dist_thr = ad_byd::planning::math::interp1_inc(
        //     std::vector<double>{0.0, 20.0, 50.0, 80.0, 120.0},
        //     std::vector<double>{30.0, 40.0, 50.0, 60.0, 70.0},
        //     ego_speed_kph);
        // if (obs_in_target_lane && rel_ds < front_dist_thr &&
        //     (nearest_stationary_leader_id == "" ||
        //      (nearest_stationary_leader_id !=
        //           std::string(traj_ptr->object_id()) &&
        //       is_stationary_leader_box_ready &&
        //       obj_frenet_box.s_min < stationary_leader_frenet_box.s_min))) {
        //   unsafe_object_id = std::string(traj_ptr->object_id());
        //   status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
        //   leader_debug_info << static_obs_debug.str();
        //   return absl::CancelledError(absl::StrFormat(
        //       "obj(%s) is between ego and leading(%s), rel_ds:%.3f, "
        //       "dist_thr:%.3f",
        //       traj_ptr->object_id(), nearest_stationary_leader_id.c_str(),
        //       rel_ds, front_dist_thr));
        // }

        const double lon_dist = obj_frenet_box.s_min - ego_cur_frenet_box.s_max;

        double max_decel_for_front_object = ad_byd::planning::math::interp1_inc(
            DV_TTC_VEC, F_DV_STATIC_DEC_VEC, ego_lon_v_kph);

        if (is_lc_state_prev) {
          double debounce_braking_deceleration =
              std::clamp(((1.0 - conserv_base) * 2.0), 0.0, 1.5);
          max_decel_for_front_object += debounce_braking_deceleration;
          max_decel_for_front_object =
              std::clamp(max_decel_for_front_object, 0.0, 5.0);
        }

        if (is_vru) {
          max_decel_for_front_object *= 0.6;
        }
        std::string dec_str = "";
        double hypo_braking_deceleration = 0.0;
        max_decel_for_front_object = is_vru ? 3.0 : 5.0;
        auto check_status = CheckDeceleration(
            lon_dist, std::string(traj_ptr->object_id()), "ego",
            /*v_lead=*/0.0, ego_lon_v,
            /*response_time=*/0.0, kEgoFollowTimeBuffer,
            max_decel_for_front_object, min_lon_buffer,
            /*acc_compensation_dis=*/0.0, dec_str, &hypo_braking_deceleration,
            &leader_max_decel);
        static_obs_debug << "|dec:" << check_status.ok()
                         << "-max_dec:" << max_decel_for_front_object
                         << "-hpy_dec:" << leader_max_decel << " ";
        if (!check_status.ok()) {
          unsafe_object_id = std::string(traj_ptr->object_id());
          status_code = PlannerStatusProto::DANGEROUS_STATIONARY_OBSTACLE;
          leader_debug_info << static_obs_debug.str();
          return check_status;
        }

        if (obj_frenet_box.s_min < target_front_obj_scene_s_min) {
          target_front_obj_scene_s_min = obj_frenet_box.s_min;
          // add scene[FRONT_OBJ_ON_TARGET_LANE] for lc style decider
          target_front_obj_scene_info_ptr->set_obj_info(obj_lcs_decider_info,
                                                        ego_lcs_decider_info);
          target_front_obj_scene_info_ptr->set_scene_type(
              LaneChangeStylePostDeciderSceneType::
                  SCENE_FRONT_OBJ_ON_TARGET_LANE);
          target_front_obj_scene_info_ptr->ClearSafetyCheckingInfo();
          target_front_obj_scene_info_ptr->add_safety_checking_info(
              LaneChangeSafetyCheckingInfo(
                  LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS,
                  std::fabs(lon_dist), obj_front_extension));
          target_front_obj_scene_info_ptr->add_safety_checking_info(
              LaneChangeSafetyCheckingInfo(
                  LaneChangeSafetyCheckingCondition::CHECK_COND_FOLLOW_DECEL,
                  std::fabs(hypo_braking_deceleration),
                  std::fabs(max_decel_for_front_object)));
        }

        break;
      }
    }
    static_obs_debug << "|sf:";
  }

  if (ego_static_front_obj_scene_info.scene_type() ==
      LaneChangeStylePostDeciderSceneType::SCENE_FRONT_STATIC_OBJ_ON_EGO_LANE) {
    // add scene[FRONT_STATIC_OBJ_ON_EGO_LANE] for lc style decider
    scene_infos_ptr->emplace_back(ego_static_front_obj_scene_info);
  }

  status_code = PlannerStatusProto::OK;
  leader_debug_info << static_obs_debug.str();
  follower_debug_info << dynamic_obs_debug.str();
  return absl::OkStatus();
}

}  // namespace st::planning

// #pragma GCC pop_options