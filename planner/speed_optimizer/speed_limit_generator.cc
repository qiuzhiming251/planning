

#include "planner/speed_optimizer/speed_limit_generator.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "plan_common/log_data.h"

//#include "global/buffered_logger.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/constants.h"
#include "plan_common/log_data.h"
#include "plan_common/speed/ad_byd_speed/constant_acc_builder.h"
//#include "decision/constraint_manager.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "plan_common/plan_common_defs.h"
#include "planner/planner_manager/planner_flags.h"
#include "planner/speed_optimizer/speed_finder_flags.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/speed_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st::planning {
namespace {
using SpeedLimitProto = SpeedFinderParamsProto::SpeedLimitParamsProto;
using SpeedLimitRange = SpeedLimit::SpeedLimitRange;
using V2TurnInfo = ad_byd::planning::V2TurnInfo;
using V2DetailType = ad_byd::planning::V2TurnInfo::V2DetailTurnType;
// Speed limit diff less than this value will be ignored.
constexpr double kApproxSpeedLimitEps = 0.5;
constexpr double kExtendJunction = 20.0;

constexpr double kLaneSpeedLimitRaiseRatio = 1.1;

constexpr double kEps = 0.001;

const PiecewiseLinearFunction<double> kMaxComfortDecelToSpeedDiff = {
    {1.0, 7.0}, {-0.2, -1.1}};
const PiecewiseLinearFunction<double, double> kAvSpeedRaiseRatioPlf = {
    {25.0, 30.0}, {1.1, 1.0}};
const PiecewiseLinearFunction<double> kComfortableBrakeAccPlf = {
    {80.0, 100.0, 130.0}, {-1.5, -0.8, -0.5}};

struct DrivingTopoRoad {
  double start_s = 0.0;
  double end_s = 0.0;
  double real_start_s = 0.0;
  double real_end_s = 0.0;
  ad_byd::planning::TurnType turn_type = ad_byd::planning::TurnType::NO_TURN;
  std::string left_lane_id = "";
  std::string right_lane_id = "";
};

enum SpeedLimitLevel {
  WEAKEST = 0,
  WEAKER = 1,
  MIDDLE = 2,
  STRONG = 3,
};

struct V2SpeedInfo {
  double max_speed = 0.0;
  double start_s = 0.0;
  double end_s = 0.0;
  double max_acc = 0.0;
};

#define DEBUG_CROSS_CURB_SPEED_LIMIT (0)
bool IsEgoCrossCurbLeft2Right(const DiscretizedPath& path_points,
                              const std::vector<DistanceInfo>&
                                  distance_info_to_impassable_path_boundaries,
                              double* dis_corss_piont) {
  // path kappa +---->>> -----
  double max_kappa = std::numeric_limits<double>::min();
  double s_max_kappa = std::numeric_limits<double>::min();
  int max_kappa_index = 0;
  double min_kappa = std::numeric_limits<double>::max();
  double s_min_kappa = std::numeric_limits<double>::max();
  int min_kappa_index = path_points.size() - 1;
  for (int i = 0; i < path_points.size(); ++i) {
    if (max_kappa < path_points[i].kappa()) {
      max_kappa_index = i;
      s_max_kappa = path_points[i].s();
      max_kappa = path_points[i].kappa();
    }
    if (min_kappa > path_points[i].kappa()) {
      min_kappa_index = i;
      min_kappa = path_points[i].kappa();
      s_min_kappa = path_points[i].s();
    }
  }
  Log2DDS::LogDataV0(
      "IsEgoCrossCurbLeft2Right",
      absl::StrCat("max_kappa_index ", max_kappa_index, " max_kappa ",
                   max_kappa, " s_max_kappa ", s_max_kappa));
  Log2DDS::LogDataV0(
      "IsEgoCrossCurbLeft2Right",
      absl::StrCat("min_kappa_index ", min_kappa_index, " min_kappa ",
                   min_kappa, " s_min_kappa ", s_min_kappa));
#if DEBUG_CROSS_CURB_SPEED_LIMIT
  std::cout << "max_kappa_index  " << max_kappa_index << "  max_kappa "
            << max_kappa << "s_max_kappa  " << s_max_kappa << std::endl;
  std::cout << "min_kappa_index  " << min_kappa_index << "  min_kappa "
            << min_kappa << "s_min_kappa  " << s_min_kappa << std::endl;
#endif
  if (min_kappa_index > max_kappa_index && min_kappa * max_kappa < 0.0 &&
      std::fabs(min_kappa) > 0.01 && std::fabs(max_kappa) > 0.01) {
#if DEBUG_CROSS_CURB_SPEED_LIMIT
    std::cout << "S path" << std::endl;
#endif
    Log2DDS::LogDataV0("IsEgoCrossCurbLeft2Right", "  S path  ");
  } else {
    return false;
  }
  //
  constexpr int kCheckExpend = 50;
  const int path_size_before_collision =
      distance_info_to_impassable_path_boundaries.size();
  bool have_left_curb = false;
  bool have_right_curb = false;
  double left_curb_max_s = 0;
  if (min_kappa_index + kCheckExpend < path_size_before_collision) {
    // 1: right curb
    for (int i = std::min(min_kappa_index + kCheckExpend,
                          path_size_before_collision - 1);
         i > min_kappa_index; --i) {
      if (distance_info_to_impassable_path_boundaries[i].dist < 0 &&
          distance_info_to_impassable_path_boundaries[i].info != "|None.") {
        have_right_curb = true;
        // left_curb_max_index = i - 1;
        // left_curb_max_s = distance_info_to_impassable_path_boundaries[i -
        // 1].s;
        break;
      }
    }
    // 1: left curb
    for (int i = std::max(max_kappa_index - kCheckExpend, 0);
         i < max_kappa_index; ++i) {
      if (distance_info_to_impassable_path_boundaries[i].dist > 0 &&
          distance_info_to_impassable_path_boundaries[i].info != "|None.") {
        have_left_curb = true;
        // left_curb_max_index = i - 1;
        left_curb_max_s = distance_info_to_impassable_path_boundaries[i].s;
        break;
      }
    }
    if (have_left_curb && have_right_curb) {
      Log2DDS::LogDataV0("IsEgoCrossCurbLeft2Right",
                         absl::StrCat("CrossCurb ", left_curb_max_s));
#if DEBUG_CROSS_CURB_SPEED_LIMIT

      std::cout << "S path CrossCurb" << std::endl;
#endif
      *dis_corss_piont = left_curb_max_s;
      return true;
    }
  } else {
    // todo find curb
  }
  return false;
}

std::vector<DrivingTopoRoad> SeparateAvDrivingTopoRoad(
    const PlannerSemanticMapManager* psmm, const DrivePassage& drive_passage,
    const PathPoint av_pos) {
  std::vector<DrivingTopoRoad> driving_topo_road_seq;
  if (psmm == nullptr) {
    return driving_topo_road_seq;
  }

  const auto& lane_path = drive_passage.lane_path();
  bool find_start = false;
  double accum_s = 0.0;
  for (const auto& lane_id : lane_path.lane_ids()) {
    double start_fraction = 0.0;
    double end_fraction = 1.0;
    const auto& lane = psmm->FindLaneByIdOrNull(lane_id);
    if (!lane) continue;

    // find first av position in lane path
    if (!find_start) {
      const auto ff = BuildBruteForceFrenetFrame(
          lane->points(), /*down_sample_raw_points=*/false);
      if (!ff.ok()) continue;
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha;
      ff.value().XYToSL({av_pos.x(), av_pos.y()}, &sl, &normal, &index_pair,
                        &alpha);
      start_fraction = sl.s / std::fmax(lane->curve_length(), 1e-2);
      if (start_fraction < 0.0 - kEps || start_fraction > 1.0 - kEps) {
        continue;
      } else {
        find_start = true;
      }
    }

    // calculate start s, end s and accum s
    const double process_length =
        std::fmax(0.0, (end_fraction - start_fraction) * lane->curve_length());
    DrivingTopoRoad driving_topo_road;
    driving_topo_road.start_s = accum_s;
    driving_topo_road.end_s = driving_topo_road.start_s + process_length;
    driving_topo_road.real_start_s = driving_topo_road.start_s;
    driving_topo_road.real_end_s = driving_topo_road.end_s;

    accum_s = driving_topo_road.end_s;

    // set zone type
    driving_topo_road.turn_type = lane->turn_type();
    driving_topo_road.left_lane_id = lane->left_lane_id();
    driving_topo_road.right_lane_id = lane->right_lane_id();

    // merge same zone by type
    driving_topo_road_seq.push_back(driving_topo_road);
  }

  // extend driving_topo_road_seq end_s and start_s if not NO_TURN
  if (driving_topo_road_seq.size() < 2) {
    return driving_topo_road_seq;
  }
  for (int i = driving_topo_road_seq.size() - 2; i >= 0; i--) {
    if (driving_topo_road_seq[i + 1].turn_type !=
        ad_byd::planning::TurnType::NO_TURN) {
      driving_topo_road_seq[i + 1].start_s -= kExtendJunction;
    }
    driving_topo_road_seq[i].end_s = std::min(
        driving_topo_road_seq[i].end_s, driving_topo_road_seq[i + 1].start_s);
    driving_topo_road_seq[i].start_s = std::min(
        driving_topo_road_seq[i].start_s, driving_topo_road_seq[i].end_s);
  }
  for (int i = 0; i < driving_topo_road_seq.size() - 1; i++) {
    if (driving_topo_road_seq[i].turn_type !=
        ad_byd::planning::TurnType::NO_TURN) {
      driving_topo_road_seq[i].end_s += kExtendJunction;
    }
    driving_topo_road_seq[i + 1].start_s = std::max(
        driving_topo_road_seq[i + 1].start_s, driving_topo_road_seq[i].end_s);
    driving_topo_road_seq[i + 1].end_s =
        std::max(driving_topo_road_seq[i + 1].start_s,
                 driving_topo_road_seq[i + 1].end_s);
  }

  driving_topo_road_seq.erase(
      std::remove_if(driving_topo_road_seq.begin(), driving_topo_road_seq.end(),
                     [](const DrivingTopoRoad& topo_road) {
                       return std::fabs(topo_road.end_s - topo_road.start_s) <
                              1e-2;
                     }),
      driving_topo_road_seq.end());

  return driving_topo_road_seq;
}

std::vector<double> GetMaxValuesNearbyPathPoint(
    const DiscretizedPath& discretized_points, double forward_radius,
    double backward_radius,
    const std::function<double(const PathPoint&)>& get_value) {
  const auto num_of_points = discretized_points.size();
  // Deque element: [s, v].
  std::deque<std::pair<double, double>> dq;
  std::vector<double> res;
  res.reserve(num_of_points);
  int k = 0;
  for (int i = 0; i < num_of_points; ++i) {
    const auto& path_point = discretized_points[i];
    const double start_s = path_point.s() - backward_radius;
    const double end_s = path_point.s() + forward_radius;
    while (!dq.empty() && dq.front().first < start_s) {
      dq.pop_front();
    }
    while (k < num_of_points && discretized_points[k].s() <= end_s) {
      const double value = get_value(discretized_points[k]);
      while (!dq.empty() && dq.back().second < std::fabs(value)) {
        dq.pop_back();
      }
      dq.emplace_back(discretized_points[k++].s(), value);
    }
    res.push_back(dq.front().second);
  }
  return res;
}

void GetFirstValidV2TurnInfo(V2TurnInfo* res_turn_info, int* idx,
                             const std::vector<V2TurnInfo>& v2_turn_info) {
  if (nullptr == res_turn_info || nullptr == idx) return;
  res_turn_info->is_valid = false;
  *idx = -10000;
  if (v2_turn_info.empty()) return;

  for (int i = 0; i < v2_turn_info.size(); i++) {
    auto turn_info = v2_turn_info[i];
    if (V2TurnInfo::V2TurnType::LEFT != turn_info.turn_type &&
        V2TurnInfo::V2TurnType::RIGHT != turn_info.turn_type &&
        V2TurnInfo::V2TurnType::MERGE_LEFT != turn_info.turn_type &&
        V2TurnInfo::V2TurnType::MERGE_RIGHT != turn_info.turn_type &&
        V2TurnInfo::V2TurnType::RAMP_LEFT != turn_info.turn_type &&
        V2TurnInfo::V2TurnType::RAMP_RIGHT != turn_info.turn_type) {
      continue;
    }
    *res_turn_info = turn_info;
    *idx = i;
    break;
  }

  return;
}

std::optional<int> GetFirstV2TurnInfoIndex(
    const std::vector<V2TurnInfo>& v2_turn_info) {
  if (v2_turn_info.empty()) return std::nullopt;
  for (int i = 0; i < v2_turn_info.size(); i++) {
    const auto detail_turn_type = v2_turn_info[i].detail_turn_type;
    if (detail_turn_type == V2DetailType::TURN_LEFT ||
        detail_turn_type == V2DetailType::TURN_RIGHT ||
        detail_turn_type == V2DetailType::SLIGHT_LEFT ||
        detail_turn_type == V2DetailType::SLIGHT_RIGHT ||
        detail_turn_type == V2DetailType::TURN_RIGHT_ONLY ||
        detail_turn_type == V2DetailType::UTURN ||
        detail_turn_type == V2DetailType::UTURN_RIGHT) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<double> CalculateMinDistanceDecelToTargetSpeed(
    const V2TurnInfo& curr_turn_info, double av_speed, double v2_max_decel,
    const double max_application_distance) {
  constexpr double kEhpV2TurnMaxSpeedLimit =
      12.92;  // 50km/h -> 12.92     display->real
  constexpr double kEhpV2TurnMidSpeedLimit =
      6.5;  // 25km/h -> 6.5  display->real
  constexpr double kEhpV2TurnMinSpeedLimit = Kph2Mps(10.0);
  // (consider actor delay set min decel distance )
  constexpr double kV2TurnMinSpeedLimitApplicationDistance = 100.0;  // m
  // constexpr double kV2TurnTargetSpeedStartBuffer = 20.0;  // m
  // constexpr double kV2TurnActorDelayBuffer = 20.0;        // m

  auto calculate_decel_distance =
      [av_speed, v2_max_decel, kV2TurnMinSpeedLimitApplicationDistance,
       max_application_distance](
          const double speed_limit) -> std::optional<double> {
    if (av_speed <= speed_limit) return std::nullopt;
    double decel_to_target_speed_distance =
        0.5 * (Sqr(speed_limit) - Sqr(av_speed)) / v2_max_decel;
    decel_to_target_speed_distance = std::clamp(
        decel_to_target_speed_distance, kV2TurnMinSpeedLimitApplicationDistance,
        max_application_distance);
    return decel_to_target_speed_distance;
  };

  const auto& type = curr_turn_info.detail_turn_type;
  if (type == V2DetailType::TURN_LEFT || type == V2DetailType::TURN_RIGHT ||
      type == V2DetailType::TURN_RIGHT_ONLY) {
    return calculate_decel_distance(kEhpV2TurnMidSpeedLimit);
  } else if (type == V2DetailType::UTURN || type == V2DetailType::UTURN_RIGHT) {
    return calculate_decel_distance(kEhpV2TurnMinSpeedLimit);
  } else {
    return std::nullopt;
  }
}

// --- On hold. was used in CURVATURE speed limit.
std::pair<double, double> MatchingLowClassRoadByV2(
    const PlannerSemanticMapManager* psmm_ptr,
    const std::vector<DrivingTopoRoad>& topo_roads,
    bool enable_more_middle_limit, int* road_class) {
  constexpr double kV2MatchRange = 5.0;

  std::pair<double, double> low_class_road_range =
      std::make_pair<double, double>(DBL_MAX, DBL_MAX);
  if (nullptr == psmm_ptr) return low_class_road_range;
  if (nullptr == psmm_ptr->map_ptr()) return low_class_road_range;

  const auto& ehp_v2_info = psmm_ptr->map_ptr()->v2_info();

  const auto& v2_road_class = ehp_v2_info.road_class;
  if (!v2_road_class.empty()) {
    if (v2_road_class.front().start_s < 40.0) {
      *road_class = static_cast<int>(v2_road_class.front().type);
    }
  }

  if (!ehp_v2_info.has_navigation) return low_class_road_range;

  V2TurnInfo curr_turn_info, next_turn_info;
  int idx;

  GetFirstValidV2TurnInfo(&curr_turn_info, &idx, ehp_v2_info.turn_info);

  if (!curr_turn_info.is_valid) return low_class_road_range;

  int road_type = 0;
  if (!curr_turn_info.infos.empty()) {
    if (!curr_turn_info.infos.front().empty()) {
      road_type = atoi(curr_turn_info.infos.front().c_str());
    }
  }

  for (double i = -1.0; i <= 1.0 + 1e-2; i += 1.0) {
    const double dist = curr_turn_info.dist + i * kV2MatchRange;
    const auto v2_match_road = std::lower_bound(
        topo_roads.begin(), topo_roads.end(), dist,
        [](const DrivingTopoRoad& driving_topo_road, double av_s) {
          return driving_topo_road.end_s < av_s;
        });
    if (v2_match_road != topo_roads.end()) {
      if ((curr_turn_info.turn_type == V2TurnInfo::V2TurnType::LEFT ||
           curr_turn_info.turn_type == V2TurnInfo::V2TurnType::RIGHT) &&
          curr_turn_info.after_turn.lane_num == 1 &&
          curr_turn_info.after_turn.road_class == 6 &&
          curr_turn_info.before_turn.lane_num == 1 &&
          curr_turn_info.before_turn.road_class == 6 && road_type == 8) {
        low_class_road_range.first = v2_match_road->start_s;
        low_class_road_range.second = v2_match_road->end_s;
        // std::cout << "matching road , start_s : " <<
        // low_class_road_range.first
        //           << "   end_s : " << low_class_road_range.second;
        return low_class_road_range;
      }
    }
    if (v2_match_road != topo_roads.end()) {
      if ((curr_turn_info.turn_type == V2TurnInfo::V2TurnType::LEFT ||
           curr_turn_info.turn_type == V2TurnInfo::V2TurnType::RIGHT) &&
          curr_turn_info.after_turn.lane_num == 1 &&
          curr_turn_info.after_turn.road_class == 6 &&
          curr_turn_info.before_turn.lane_num == 1 && road_type == 2) {
        low_class_road_range.first = v2_match_road->start_s;
        low_class_road_range.second = v2_match_road->end_s;
        // std::cout << "matching road , start_s : " <<
        // low_class_road_range.first
        //           << "   end_s : " << low_class_road_range.second;
        return low_class_road_range;
      }
    }
  }
  return low_class_road_range;
}

// --- On hold. was used in CURVATURE speed limit.
SpeedLimitLevel GetSpeedLimitLevel(
    const std::pair<double, double>& low_class_road_range,
    const std::vector<DrivingTopoRoad>& topo_roads, double max_kappa,
    double pos_s, int road_class) {
  constexpr double kNoTurnSmallKappaThres = 0.025;  // m^-1.

  // low class turn use MIDDLE speed limit
  if (pos_s > low_class_road_range.first &&
      pos_s < low_class_road_range.second &&
      std::fabs(max_kappa) > kNoTurnSmallKappaThres) {
    return SpeedLimitLevel::MIDDLE;
  }

  const auto driving_road = std::lower_bound(
      topo_roads.begin(), topo_roads.end(), pos_s,
      [](const DrivingTopoRoad& driving_topo_road, double av_s) {
        return driving_topo_road.end_s < av_s;
      });

  // low class and large kappa road use STRONG speed limit
  if (driving_road != topo_roads.end() &&
      driving_road->turn_type == ad_byd::planning::TurnType::NO_TURN &&
      std::fabs(max_kappa) > kNoTurnSmallKappaThres && road_class == 6) {
    return SpeedLimitLevel::STRONG;
  }

  // single road use STRONG speed limit
  if (driving_road != topo_roads.end() &&
      driving_road->turn_type == ad_byd::planning::TurnType::NO_TURN &&
      std::fabs(max_kappa) > kNoTurnSmallKappaThres &&
      driving_road->left_lane_id.empty() &&
      driving_road->right_lane_id.empty()) {
    return SpeedLimitLevel::STRONG;
  }

  // right turn use WEAKER speed limit
  if (driving_road != topo_roads.end() &&
      driving_road->turn_type == ad_byd::planning::TurnType::RIGHT_TURN &&
      std::fabs(max_kappa) > kNoTurnSmallKappaThres) {
    return SpeedLimitLevel::WEAKER;
  }

  // others use WEAKEST speed limit
  return SpeedLimitLevel::WEAKEST;
}

// --- On hold. was used in CURVATURE speed limit.
void ExtendSpeedLimit(const std::vector<std::pair<double, double>>& need_extend,
                      std::vector<SpeedLimitRange>& speed_limit_ranges) {
  if (speed_limit_ranges.empty()) {
    return;
  }

  // forward extend
  for (int i = 0; i < speed_limit_ranges.size(); i++) {
    if (need_extend[i].first < 1e-2) {
      continue;
    }
    double min_speed_limit = speed_limit_ranges[i].speed_limit;
    double accum_s = 0.0;
    for (int j = i; j >= 0; j--) {
      if (speed_limit_ranges[i].start_s - speed_limit_ranges[j].start_s <
          need_extend[i].first) {
        speed_limit_ranges[j].speed_limit =
            std::fmin(min_speed_limit, speed_limit_ranges[j].speed_limit);
      } else {
        break;
      }
    }
  }

  // backward extend
  for (int i = speed_limit_ranges.size() - 1; i >= 0; i--) {
    if (need_extend[i].second < 1e-2) {
      continue;
    }
    double min_speed_limit = speed_limit_ranges[i].speed_limit;
    double accum_s = 0.0;
    for (int j = i; j < speed_limit_ranges.size(); j++) {
      if (speed_limit_ranges[j].start_s - speed_limit_ranges[i].start_s <
          need_extend[i].second) {
        speed_limit_ranges[j].speed_limit =
            std::fmin(min_speed_limit, speed_limit_ranges[j].speed_limit);
      } else {
        break;
      }
    }
  }
  return;
}

SpeedLimit GenerateCurvatureSpeedLimit(
    const DiscretizedPath& path_points,
    const VehicleDriveParamsProto& veh_drive_params,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit,
    double av_speed, const SpeedLimitProto& speed_limit_config,
    const PlannerSemanticMapManager* psmm_ptr,
    const std::vector<DrivingTopoRoad>& topo_roads,
    const double spdlimit_curvature_gain_prev,
    double* const spdlimit_curvature_gain_ptr) {
  const std::function<double(const PathPoint&)> get_kappa =
      [](const PathPoint& pt) { return pt.kappa(); };

  // --- Params preparation
  // The new curvature speed limit equation is
  // \frac{a}{\kappa^{b} + c} + d.
  // The derivative of lat acc is
  // -\frac{2ab\kappa^{b}(a+d(\kappa^{b}+c))}{(\kappa^{b}+c)^{3}}+(\frac{a}{\kappa^{b}+c})^{2}
  // Use the tangent at kSmallKappaThres to calc speed limit for the points
  // whose kappa is less than kSmallKappaThres to avoid hard brake.
  constexpr double kSmallKappaThres = 0.003;  // m^-1.
  constexpr double kCurvatureEps = 0.000001;  // Avoid division by zero.
  const double a = speed_limit_config.curvature_numerator();
  const double b = speed_limit_config.curvature_power();
  const double c = speed_limit_config.curvature_bias1();
  const double d = speed_limit_config.curvature_bias2();
  const double gain_target = speed_limit_config.curvature_lanechange_gain();
  const double gain_prev = spdlimit_curvature_gain_prev;
  double gain = 1.0;
  const double gain_lpf_coeffi = 0.06;  // low pass filter coeffi.
  // Avoid sudden drop after lane change finished.
  gain = gain_prev - gain_target > 0.02
             ? Lerp(gain_prev, gain_target, gain_lpf_coeffi)
             : gain_target;
  *spdlimit_curvature_gain_ptr = gain;

  const double k_power_b = std::pow(kSmallKappaThres, b);
  const double k_power_b_plus_c = k_power_b + c;
  const double small_kappa_speed_limit_sqr = Sqr(a / k_power_b_plus_c + d);
  const double small_kappa_lat_acc_derivative =
      std::max(0.0, -(2.0 * a * b * k_power_b * (a + d * k_power_b_plus_c)) /
                            Cube(k_power_b_plus_c) +
                        small_kappa_speed_limit_sqr);
  const double intercept =
      std::max(0.0, -kSmallKappaThres * small_kappa_lat_acc_derivative +
                        small_kappa_speed_limit_sqr * kSmallKappaThres);

  const double max_allowed_kappa =
      ComputeCenterMaxCurvature(veh_geo_params, veh_drive_params);
  const double av_speed_sqr = Sqr(av_speed);

  // --- get_speed_limit[]
  const auto get_speed_limit = [&](double max_kappa, double s) {
    const double fabs_kappa = std::fabs(max_kappa);
    double speed_limit = 0.0;
    if (fabs_kappa > max_allowed_kappa) {
      speed_limit = speed_limit_config.speed_limit_for_large_curvature();
    } else if (fabs_kappa >= kSmallKappaThres) {
      speed_limit = a / (std::pow(fabs_kappa, b) + c) + d;
    } else {
      const double lat_acc =
          fabs_kappa * small_kappa_lat_acc_derivative + intercept;
      speed_limit = std::sqrt(lat_acc / (fabs_kappa + kCurvatureEps));
    }
    speed_limit = speed_limit * gain;  // speed limit rise by lane change.
    const double comfortable_brake_acc =
        kComfortableBrakeAccPlf(Mps2Kph(av_speed));  // m/ss.
    const double comfortable_brake_speed_sqr =
        av_speed_sqr + 2.0 * comfortable_brake_acc * s;
    if (comfortable_brake_speed_sqr > Sqr(speed_limit)) {
      speed_limit = std::sqrt(comfortable_brake_speed_sqr);
    }
    return speed_limit;
  };

  // --- Main loop.
  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(path_points.size());
  const double radius = speed_limit_config.max_curvature_consider_radius();
  const auto max_kappas =
      GetMaxValuesNearbyPathPoint(path_points, radius, radius, get_kappa);
  const double init_limit = get_speed_limit(max_kappas[0], path_points[0].s());
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), init_limit);
  double debug_max_kappa = 0.0;

  for (int i = 1; i < path_points.size(); ++i) {
    const double curr_speed_limit =
        get_speed_limit(max_kappas[i], path_points[i].s());
    if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
            kApproxSpeedLimitEps ||
        i == path_points.size() - 1) {
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = path_points[i].s(),
           .speed_limit = prev_speed_limit_point.second,
           .info = SpeedLimitTypeProto::Type_Name(
               SpeedLimitTypeProto_Type_CURVATURE)});

      debug_max_kappa = std::max(debug_max_kappa, std::fabs(max_kappas[i]));

      prev_speed_limit_point =
          std::make_pair(path_points[i].s(), curr_speed_limit);
    }
  }

  Log2DDS::LogDataV2("speed_max_kappa", debug_max_kappa);
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit GenerateSteerRateSpeedLimit(
    const std::vector<PathPoint>& path_points,
    const VehicleDriveParamsProto& veh_drive_params,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit,
    const SpeedLimitProto& speed_limit_config) {
  const double wheel_base = veh_geo_params.wheel_base();
  CHECK_GT(wheel_base, 0.0);
  CHECK(!path_points.empty());

  const std::function<double(const PathPoint&)> get_front_wheel_omega_by_v =
      [&wheel_base](const PathPoint& p) {
        const auto lambda = std::fabs(p.lambda());
        const auto kappa = p.kappa();
        constexpr double kKappaThreshold = 0.05;  // for 20m radius.
        if (std::fabs(kappa) < kKappaThreshold) return 0.0;
        // tan(delta) = l * kappa;
        // d(delta)/dt = l / (1 + sqr(l * kappa)) * d(kappa) /ds * v;
        return wheel_base * lambda / (1.0 + Sqr(wheel_base * kappa));
      };

  const auto get_speed_limit = [&speed_limit_config, &veh_drive_params,
                                &max_speed_limit](
                                   double max_front_wheel_omega_by_v) {
    double speed_limit =
        speed_limit_config.max_steer_rate() /
        (veh_drive_params.steer_ratio() * max_front_wheel_omega_by_v + kEps);
    constexpr double kLowSpeedThreshold = 1.0;  // m/s.
    if (speed_limit < kLowSpeedThreshold) {
      speed_limit = speed_limit_config.min_steer_rate_speed_limit();
    }
    return std::min(speed_limit, max_speed_limit);
  };

  const DiscretizedPath discretized_path_points(path_points);
  std::vector<PathPoint> resampled_points;
  resampled_points.reserve(
      CeilToInt(discretized_path_points.length() / kPathSampleInterval) +
      discretized_path_points.size());
  auto iter = discretized_path_points.begin();
  double prev_s = iter->s();
  resampled_points.push_back(*iter);
  while (iter != discretized_path_points.end()) {
    const auto next_iter = iter + 1;
    if (next_iter == discretized_path_points.end()) {
      break;
    }
    // The comparison shall be as the same logic as the operation in the "else"
    // statement because a <= b + c may be not equal to a - b <= c due to
    // numeric error.
    if (next_iter->s() <= prev_s + kPathSampleInterval) {
      resampled_points.push_back(*next_iter);
      prev_s = next_iter->s();
      ++iter;
    } else {
      prev_s += kPathSampleInterval;
      resampled_points.push_back(discretized_path_points.Evaluate(prev_s));
    }
  }

  // Remove outlying lambda.
  constexpr double kOutlierThreshold = 5.0;  // times.
  for (int i = 1; i < resampled_points.size() - 1; ++i) {
    const double cur_lambda = resampled_points[i].lambda();
    const double prev_lambda = resampled_points[i - 1].lambda();
    const double next_lambda = resampled_points[i + 1].lambda();
    const double avg_lambda = 0.5 * (prev_lambda + next_lambda);
    if (resampled_points[i + 1].s() - resampled_points[i - 1].s() <
            kPathSampleInterval &&
        prev_lambda * next_lambda > -kEps &&
        std::fabs(cur_lambda) > kOutlierThreshold * std::fabs(avg_lambda)) {
      resampled_points[i].set_lambda(avg_lambda);
    }
  }

  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(resampled_points.size());
  const auto front_wheel_omega_by_vs = GetMaxValuesNearbyPathPoint(
      DiscretizedPath(resampled_points),
      speed_limit_config.min_steer_rate_forward_consider_radius(),
      speed_limit_config.min_steer_rate_backward_consider_radius(),
      get_front_wheel_omega_by_v);
  const double init_limit = get_speed_limit(front_wheel_omega_by_vs[0]);
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(resampled_points[0].s(), init_limit);
  for (int i = 1; i < resampled_points.size(); ++i) {
    const double curr_speed_limit = get_speed_limit(front_wheel_omega_by_vs[i]);
    if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
            kApproxSpeedLimitEps ||
        i == resampled_points.size() - 1) {
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = resampled_points[i].s(),
           .speed_limit = prev_speed_limit_point.second,
           .info = SpeedLimitTypeProto::Type_Name(
               SpeedLimitTypeProto_Type_STEER_RATE)});
      prev_speed_limit_point =
          std::make_pair(resampled_points[i].s(), curr_speed_limit);
    }
  }
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit GenerateLaneSpeedLimit(const DiscretizedPath& path_points,
                                  double max_speed_limit, double av_speed,
                                  const DrivePassage& drive_passage,
                                  const LaneChangeStage lc_stage,
                                  bool raise_lane_speed_limit) {
  const auto get_speed_limit = [&drive_passage, max_speed_limit, lc_stage,
                                raise_lane_speed_limit,
                                av_speed](const PathPoint& path_point) {
    constexpr double kExceedLimitThreshold = 0.5;  // m/s.
    const double av_speed_sqr = Sqr(av_speed - kExceedLimitThreshold);
    const auto speed_limit =
        drive_passage.QuerySpeedLimitAt(ToVec2d(path_point));
    if (!speed_limit.ok()) {
      return max_speed_limit;
    }
    double raw_speed_limit = *speed_limit;
    if (raise_lane_speed_limit) {
      raw_speed_limit *= kAvSpeedRaiseRatioPlf(av_speed);
    }
    const double comfortable_decel =
        kMaxComfortDecelToSpeedDiff(av_speed - raw_speed_limit);
    const double comfortable_brake_speed_sqr =
        av_speed_sqr + 2.0 * comfortable_decel * path_point.s();
    if (comfortable_brake_speed_sqr > Sqr(raw_speed_limit)) {
      return std::sqrt(comfortable_brake_speed_sqr);
    }
    return raw_speed_limit;
  };

  std::vector<SpeedLimitRange> speed_limit_ranges;
  const int num_points = path_points.size();
  speed_limit_ranges.reserve(num_points);
  // first: s second: v
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), get_speed_limit(path_points[0]));
  double last_sample_s = 0.0;
  for (int i = 1; i < num_points; ++i) {
    // Only check the lane speed limit at every meter to save computation.
    constexpr double kSpeedLimitSampleRange = 1.0;  // Meters.
    if (path_points[i].s() - last_sample_s > kSpeedLimitSampleRange ||
        i == num_points - 1) {
      last_sample_s = path_points[i].s();
      if (const double curr_speed_limit = get_speed_limit(path_points[i]);
          std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
              kApproxSpeedLimitEps ||
          i == num_points - 1) {
        CHECK_GT(path_points[i].s(), prev_speed_limit_point.first);
        speed_limit_ranges.push_back(
            {.start_s = prev_speed_limit_point.first,
             .end_s = path_points[i].s(),
             .speed_limit = prev_speed_limit_point.second,
             .info = SpeedLimitTypeProto::Type_Name(
                 SpeedLimitTypeProto_Type_LANE)});
        prev_speed_limit_point =
            std::make_pair(path_points[i].s(), curr_speed_limit);
      }
    }
  }
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

#define DEBUG_CLOSE_CURB (0)

std::optional<SpeedLimit> GenerateCloseCurbSpeedLimit(
    const DiscretizedPath& path_points, double max_speed_limit,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries,
    const PiecewiseLinearFunction<double, double>&
        hard_curb_clearance_rel_speed_plf,
    double av_speed, double close_curb_max_dec) {
  struct SpeedLimitPoint {
    double s = 0.0;
    double v = 0.0;
    std::string info;
  };
  const auto get_speed_limit_point =
      [&path_points, &distance_info_to_impassable_path_boundaries,
       &hard_curb_clearance_rel_speed_plf,
       max_speed_limit](int index) -> SpeedLimitPoint {
    const double max_hard_curb_clearance =
        hard_curb_clearance_rel_speed_plf.x().back();

    const auto& distance_info =
        distance_info_to_impassable_path_boundaries[index];
    if (std::fabs(distance_info.dist) > max_hard_curb_clearance) {
      return {.s = path_points[index].s(),
              .v = max_speed_limit,
              .info = distance_info.info};
    }
    if (distance_info.dist > kMathEpsilon) {
      return {.s = path_points[index].s(),
              .v = std::min(hard_curb_clearance_rel_speed_plf(
                                std::fabs(distance_info.dist)),
                            max_speed_limit),
              .info = distance_info.info};
    } else {
      return {.s = path_points[index].s(),
              .v = max_speed_limit,
              .info = distance_info.info};
    }
  };

  const auto path_size_before_collision =
      distance_info_to_impassable_path_boundaries.size();

  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(path_size_before_collision);
  // first: s second: v
  auto prev_speed_limit_point = get_speed_limit_point(0);
  for (int i = 1; i < path_size_before_collision; ++i) {
    auto curr_speed_limit_point = get_speed_limit_point(i);

    double tmp_vel_sq =
        Sqr(av_speed) - 2 * close_curb_max_dec * path_points[i].s();
    double consider_dec_speed_limit = 0.0;
    if (tmp_vel_sq > 0) {
      consider_dec_speed_limit = std::sqrt(tmp_vel_sq);
    }
    prev_speed_limit_point.v =
        std::max(prev_speed_limit_point.v, consider_dec_speed_limit);

    if (std::fabs(curr_speed_limit_point.v - prev_speed_limit_point.v) >
            kApproxSpeedLimitEps ||
        i == path_size_before_collision - 1) {
      CHECK_GT(path_points[i].s(), prev_speed_limit_point.s);
#if DEBUG_CLOSE_CURB
      if (prev_speed_limit_point.v < av_speed) {
        std::cout << "[ " << i << " ]: "
                  << " curr_speed_limit_point:[ vel "
                  << Mps2Kph(curr_speed_limit_point.v) << "km/h  s "
                  << curr_speed_limit_point.s << " ] "
                  << " prev_speed_limit_point:[ vel "
                  << Mps2Kph(prev_speed_limit_point.v) << "km/h  s "
                  << prev_speed_limit_point.s << " prev_s "
                  << prev_speed_limit_point.s << " consider_dec_speed_limit "
                  << Mps2Kph(consider_dec_speed_limit) << "km/h "
                  << " s " << path_points[i].s() << std::endl;
        std::cout << "          dist "
                  << distance_info_to_impassable_path_boundaries[i].dist
                  << " info "
                  << distance_info_to_impassable_path_boundaries[i].info
                  << " id " << distance_info_to_impassable_path_boundaries[i].id
                  << " s " << distance_info_to_impassable_path_boundaries[i].s
                  << std::endl;
      }
#endif
      curr_speed_limit_point.v =
          std::max(curr_speed_limit_point.v, consider_dec_speed_limit);

#if DEBUG_CLOSE_CURB
      if (prev_speed_limit_point.v < av_speed) {
        std::cout << "          res: start_s " << prev_speed_limit_point.s
                  << " speed_limit " << Mps2Kph(prev_speed_limit_point.v)
                  << "km/h lat_dist "
                  << distance_info_to_impassable_path_boundaries[i].dist
                  << std::endl;
        std::cout << std::endl;
      }
#endif
      if (ad_byd::planning::BoundaryType::OCC_VEGETATION ==
          distance_info_to_impassable_path_boundaries[i].type.boundary_type) {
        continue;
      }
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.s,
           .end_s = path_points[i].s(),
           .speed_limit = prev_speed_limit_point.v,
           .info = absl::StrCat(SpeedLimitTypeProto::Type_Name(
                                    SpeedLimitTypeProto_Type_CLOSE_CURB),
                                prev_speed_limit_point.info)});
      prev_speed_limit_point = curr_speed_limit_point;
    }
  }
#if DEBUG_CLOSE_CURB
  std::cout << std::endl;
#endif
  if (speed_limit_ranges.empty()) return std::nullopt;
  return SpeedLimit(speed_limit_ranges);
}

#define DEBUG_APPROACH_CURB (0)

std::optional<SpeedLimit> GenerateApproachCurbSpeedLimit(
    const DiscretizedPath& path_points,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries,
    const SpeedLimitProto& speed_limit_config, double max_speed_limit,
    double av_speed, double approach_curb_max_dec,
    const LaneChangeStateProto lane_change_state) {
  // use distance_to_curb and angle_with_curb as two factor to calculate
  // speed_limit
  const PiecewiseLinearFunction<double> approach_curb_angle_plf =
      PiecewiseLinearFunctionFromProto(
          speed_limit_config.approach_curb_angle_plf());
  PiecewiseLinearFunction<double> approach_curb_dist_plf =
      PiecewiseLinearFunctionFromProto(
          speed_limit_config.approach_curb_dist_plf());
  const PiecewiseLinearFunction<double> approach_curb_straight_dist_plf =
      PiecewiseLinearFunctionFromProto(
          speed_limit_config.approach_curb_straight_dist_plf());

  const auto path_size_before_collision =
      distance_info_to_impassable_path_boundaries.size();
  constexpr double kEnableLonDist = 30.0;
  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(path_size_before_collision);
  bool enable_speed_limit = false;
  double min_speed_limit = std::numeric_limits<double>::max();
  constexpr double kMaxCurvature = 0.0033;

  double average_curvature = 0.0;
  int point_nums = 0;
  double max_curve = 0.0;
  for (const auto& point : path_points) {
    average_curvature += std::abs(point.kappa());
    point_nums++;
    max_curve = std::max(max_curve, std::abs(point.kappa()));
    if (point.s() > kEnableLonDist) {
      break;
    }
  }
  if (0 != point_nums) {
    average_curvature /= point_nums;
  }

  if (average_curvature < kMaxCurvature && max_curve < kMaxCurvature &&
      LaneChangeStage::LCS_NONE == lane_change_state.stage()) {
    approach_curb_dist_plf = approach_curb_straight_dist_plf;
  }
#if DEBUG_APPROACH_CURB
  std::cout << " average_curvature " << average_curvature << " max_curve "
            << max_curve << " lane_change_state "
            << static_cast<int>(lane_change_state.stage()) << std::endl;
#endif

  std::vector<double> angle, dist, speed_limit, speed_limit_s;

  for (int i = 0; i < path_size_before_collision - 1; i++) {
    auto curr_point = path_points[i];
    if (i + 8 > path_size_before_collision - 1) break;
    auto next_point = path_points[i + 8];
    const auto& curr_dist_info = distance_info_to_impassable_path_boundaries[i];
    const auto& next_dist_info =
        distance_info_to_impassable_path_boundaries[i + 8];
    double approach_curb_limit = max_speed_limit;
    double hypot = std::fmax(std::hypot(next_point.x() - curr_point.x(),
                                        next_point.y() - curr_point.y()),
                             kMathEpsilon);
    if (curr_dist_info.dist > 20.0) continue;
    if (curr_dist_info.dists.first > 20.0 || next_dist_info.dists.first > 20.0)
      continue;
    if (curr_point.s() < kEnableLonDist) {
      // left side curb
      if (curr_dist_info.dists.first < approach_curb_dist_plf.x().back() &&
          curr_dist_info.dists.first > next_dist_info.dists.first) {
        double delta_l =
            curr_dist_info.dists.first - next_dist_info.dists.first;
        double delta_angle = std::asin(delta_l / hypot) * 180.0 / M_PI;
        if (delta_angle > approach_curb_angle_plf.x().front()) {
#if DEBUG_APPROACH_CURB
          std::cout << "[left]: delta_angle " << delta_angle << " dist "
                    << curr_dist_info.dist << " s " << curr_point.s()
                    << " delta_l " << delta_l << " curr_dist "
                    << curr_dist_info.dists.first << " next_dist "
                    << next_dist_info.dists.first << " hypot " << hypot
                    << " std::asin(delta_l / hypot) "
                    << std::asin(delta_l / hypot) << std::endl;
#endif
          approach_curb_limit =
              approach_curb_angle_plf(delta_angle) *
              approach_curb_dist_plf(std::fabs(curr_dist_info.dist)) *
              max_speed_limit;
          angle.push_back(approach_curb_angle_plf(delta_angle));
          dist.push_back(
              approach_curb_dist_plf(std::fabs(curr_dist_info.dist)));
          speed_limit.push_back(approach_curb_limit);
          speed_limit_s.push_back(path_points[i].s());
        }
      }
      // right side curb
      if (-curr_dist_info.dists.second < approach_curb_dist_plf.x().back() &&
          curr_dist_info.dists.second < next_dist_info.dists.second) {
        double delta_l =
            next_dist_info.dists.second - curr_dist_info.dists.second;
        double delta_angle = std::asin(delta_l / hypot) * 180.0 / M_PI;
        if (delta_angle > approach_curb_angle_plf.x().front()) {
#if DEBUG_APPROACH_CURB
          std::cout << "[right]: delta_angle " << delta_angle << " dist "
                    << curr_dist_info.dist << " s " << curr_point.s()
                    << " delta_l " << delta_l << " curr_dist "
                    << curr_dist_info.dists.first << " next_dist "
                    << next_dist_info.dists.first << " hypot " << hypot
                    << " std::asin(delta_l / hypot) "
                    << std::asin(delta_l / hypot) << std::endl;
#endif
          approach_curb_limit = std::fmin(
              approach_curb_limit,
              approach_curb_angle_plf(delta_angle) *
                  approach_curb_dist_plf(std::fabs(curr_dist_info.dist)) *
                  max_speed_limit);
          angle.push_back(approach_curb_angle_plf(delta_angle));
          dist.push_back(
              approach_curb_dist_plf(std::fabs(curr_dist_info.dist)));
          speed_limit.push_back(Mps2Kph(approach_curb_limit));
          speed_limit_s.push_back(path_points[i].s());
        }
      }
      // combine

      if (ad_byd::planning::BoundaryType::OCC_VEGETATION ==
              curr_dist_info.type.boundary_type ||
          ad_byd::planning::BoundaryType::OCC_VEGETATION ==
              next_dist_info.type.boundary_type) {
        continue;
      }

      double tmp_vel_sq =
          Sqr(av_speed) - 2 * approach_curb_max_dec * path_points[i].s();
      double consider_dec_speed_limit = 0.0;
      if (tmp_vel_sq > 0) {
        consider_dec_speed_limit = std::sqrt(tmp_vel_sq);
      }

      approach_curb_limit =
          std::max(approach_curb_limit, consider_dec_speed_limit);

#if DEBUG_APPROACH_CURB
      std::cout << " start_s " << curr_point.s() << " approach_curb_limit "
                << Mps2Kph(approach_curb_limit) << "km/h curr_boundary_type "
                << static_cast<int>(curr_dist_info.type.boundary_type)
                << " next_boundary_type "
                << static_cast<int>(next_dist_info.type.boundary_type)
                << " max_dec " << approach_curb_max_dec
                << " consider_dec_speed_limit "
                << Mps2Kph(consider_dec_speed_limit) << "km/h" << std::endl;
#endif
      min_speed_limit = std::min(min_speed_limit, approach_curb_limit);
      speed_limit_ranges.push_back(
          {.start_s = curr_point.s(),
           .end_s = next_point.s(),
           .speed_limit = std::fmax(1.0, approach_curb_limit),
           .info = absl::StrCat(SpeedLimitTypeProto::Type_Name(
                                    SpeedLimitTypeProto_Type_APPROACH_CURB),
                                curr_dist_info.info)});
      if (std::fabs(next_dist_info.dist) < approach_curb_dist_plf.x().front()) {
#if DEBUG_APPROACH_CURB
        std::cout << " result: curr_dist " << curr_dist_info.dist
                  << " next_dist " << next_dist_info.dist << std::endl;
#endif
        enable_speed_limit = true;
      }
#if DEBUG_APPROACH_CURB
      std::cout << std::endl;
#endif
    }
  }

#if DEBUG_APPROACH_CURB
  Log2DDS::LogDataV0("angle", angle);
  Log2DDS::LogDataV0("dist", dist);
  Log2DDS::LogDataV0("speed_limit", speed_limit);
  Log2DDS::LogDataV0("speed_limit_s", speed_limit_s);
#endif

  if (enable_speed_limit) {
    return SpeedLimit(speed_limit_ranges);
  } else {
    return std::nullopt;
  }
}

std::optional<SpeedLimit> GenerateCrossCurbSpeedLimit(
    const DiscretizedPath& path_points, double max_speed_limit,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries,
    double min_speed_limit, double av_speed, double corss_curb_max_dec) {
  struct SpeedLimitPoint {
    double s = 0.0;
    double v = 0.0;
    std::string info;
  };
  double dis_corss_piont = path_points.back().s();
  if (!IsEgoCrossCurbLeft2Right(path_points,
                                distance_info_to_impassable_path_boundaries,
                                &dis_corss_piont)) {
    return std::nullopt;
  }
  if (av_speed <= min_speed_limit) return std::nullopt;
  corss_curb_max_dec =
      (Sqr(av_speed) - Sqr(min_speed_limit)) / (2 * dis_corss_piont);
  const auto get_speed_limit_point =
      [&path_points, &distance_info_to_impassable_path_boundaries, &av_speed,
       &corss_curb_max_dec, &dis_corss_piont, &min_speed_limit,
       max_speed_limit](int index) -> SpeedLimitPoint {
    const auto path_size_before_collision =
        distance_info_to_impassable_path_boundaries.size();
    if (index > path_size_before_collision - 1) {
      return {.s = path_points[index].s(),
              .v = max_speed_limit,
              .info = "corss_curb "};
    }
    const auto& distance_info =
        distance_info_to_impassable_path_boundaries[index];
    if (std::fabs(distance_info.s) < dis_corss_piont) {
      double tmp_vel_sq =
          Sqr(av_speed) - 2 * corss_curb_max_dec * path_points[index].s();
      double consider_dec_speed_limit = 0.0;
      if (tmp_vel_sq > 0) {
        consider_dec_speed_limit = std::sqrt(tmp_vel_sq);
      }
      return {.s = path_points[index].s(),
              .v = std::max(min_speed_limit, consider_dec_speed_limit),
              .info = "corss_curb " + distance_info.info};
    } else {
      return {.s = path_points[index].s(),
              .v = max_speed_limit,
              .info = "corss_curb " + distance_info.info};
    }
  };
  const auto path_size_before_collision =
      distance_info_to_impassable_path_boundaries.size();

  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(path_points.size());
  // first: s second: v
  auto prev_speed_limit_point = get_speed_limit_point(0);
  for (int i = 1; i < path_points.size(); ++i) {
    auto curr_speed_limit_point = get_speed_limit_point(i);
    if (std::fabs(curr_speed_limit_point.v - prev_speed_limit_point.v) >
            kApproxSpeedLimitEps ||
        i == path_size_before_collision - 1) {
      CHECK_GT(path_points[i].s(), prev_speed_limit_point.s);
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.s,
           .end_s = path_points[i].s(),
           .speed_limit = prev_speed_limit_point.v,
           .info = absl::StrCat(SpeedLimitTypeProto::Type_Name(
                                    SpeedLimitTypeProto_Type_CROSS_CURB),
                                prev_speed_limit_point.info)});
      prev_speed_limit_point = curr_speed_limit_point;
    }
  }
  if (speed_limit_ranges.empty()) return std::nullopt;

#if DEBUG_CROSS_CURB_SPEED_LIMIT
  std::cout << "CROSS_CURB_SPEED_LIMIT  " << path_size_before_collision
            << std::endl;
#endif

  return SpeedLimit(speed_limit_ranges);
}

std::string TypeCaseToString(const SourceProto::TypeCase type) {
  switch (type) {
    case SourceProto::TypeCase::kCloseObject:
      return "CLOSE_OBJECT";
    case SourceProto::TypeCase::kSpeedBump:
      return "SPEED_BUMP";
    case SourceProto::TypeCase::kIntersection:
      return "INTERSECTION";
    case SourceProto::TypeCase::kLcEndOfCurrentLane:
      return "LC_END_OF_CURRENT_LANE";
    case SourceProto::TypeCase::kBeyondLengthAlongRoute:
      return "BEYOND_LENGTH_ALONE_ROUTE";
    case SourceProto::TypeCase::kPedestrianObject:
      return "PEDESTRIAN_OBJECT";
    case SourceProto::TypeCase::kCrosswalk:
      return "CROSSWALK";
    case SourceProto::TypeCase::kToll:
      return "TOLL";
    case SourceProto::TypeCase::kTrafficLight:
      return "TRAFFIC_LIGHT";
    case SourceProto::TypeCase::kNoBlock:
      return "NO_BLOCK";
    case SourceProto::TypeCase::kEndOfPathBoundary:
      return "END_OF_PATH_BOUNDARY";
    case SourceProto::TypeCase::kEndOfCurrentLanePath:
      return "END_OF_CURRENT_LANE_PATH";
    case SourceProto::TypeCase::kRouteDestination:
      return "ROUTE_DESTINATION";
    case SourceProto::TypeCase::kParkingBrakeRelease:
      return "PARKING_BRAKERELEASE";
    case SourceProto::TypeCase::kBlockingStaticObject:
      return "BLOCKING_STATIC_OBJECT";
    case SourceProto::TypeCase::kStandby:
      return "STANDBY";
    case SourceProto::TypeCase::kStandstill:
      return "STANDSTILL";
    case SourceProto::TypeCase::kPullOver:
      return "PULL_OVER";
    case SourceProto::TypeCase::kBrakeToStop:
      return "BRAKE_TO_STOP";
    case SourceProto::TypeCase::kEndOfLocalPath:
      return "END_OF_LOCAL_PATH";
    case SourceProto::TypeCase::kSolidLineWithinBoundary:
      return "SOLID_LINE_WITHIN_BOUNDARY";
    case SourceProto::TypeCase::kOccludedObject:
      return "OCCLUDED_OBJECT";
    case SourceProto::TypeCase::kDenseTrafficFlow:
      return "DENSE_TRAFFIC_FLOW";
    case SourceProto::TypeCase::kStopPolyline:
      return "STOP_POLYLINE";
    case SourceProto::TypeCase::kLaneMerge:
      return "LANE_MERGE";
    case SourceProto::TypeCase::kConstructionScene:
      return "CONSTRUCTION_SCENE";
    case SourceProto::TypeCase::TYPE_NOT_SET:
      return "TYPE_NOT_SET";
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

std::optional<SpeedLimit> GenerateExternalSpeedLimit(
    const DiscretizedPath& path_points, const ConstraintManager& constraint_mgr,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit) {
  std::vector<Vec2d> points;
  points.reserve(path_points.size());
  for (const auto& path_point : path_points) {
    points.emplace_back(path_point.x(), path_point.y());
  }
  ASSIGN_OR_DIE(const auto frenet_path,
                BuildBruteForceFrenetFrame(points, true));
  constexpr double kSpeedRegionBuffer = 0.5;
  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(constraint_mgr.SpeedRegion().size() +
                             constraint_mgr.PathSpeedRegion().size());

  // Add drive passage speed regions.
  for (const ConstraintProto::SpeedRegionProto& speed_region :
       constraint_mgr.SpeedRegion()) {
    const auto type_case = speed_region.source().type_case();
    // Only consider upper bound speed limit.
    if (type_case == SourceProto::TypeCase::kNoBlock) continue;
    const double start_s =
        frenet_path.XYToSL(Vec2dFromProto(speed_region.start_point())).s -
        veh_geo_params.front_edge_to_center() - kSpeedRegionBuffer;
    const double end_s =
        frenet_path.XYToSL(Vec2dFromProto(speed_region.end_point())).s +
        veh_geo_params.back_edge_to_center() + kSpeedRegionBuffer;
    if (start_s >= end_s) continue;
#if 0
    std::cout << " external: "
              << " start_s " << start_s << " end_s " << end_s << " max_speed "
              << Mps2Kph(std::min(speed_region.max_speed(), max_speed_limit))
              << std::endl;
    std::cout << " info " << TypeCaseToString(speed_region.source().type_case())
              << std::endl;
#endif
    speed_limit_ranges.push_back(
        {.start_s = start_s,
         .end_s = end_s,
         .speed_limit = std::min(speed_region.max_speed(), max_speed_limit),
         .info = TypeCaseToString(speed_region.source().type_case())});
  }

  // Add path speed regions.
  for (const ConstraintProto::PathSpeedRegionProto& path_speed_region :
       constraint_mgr.PathSpeedRegion()) {
    const double start_s = path_speed_region.start_s() -
                           veh_geo_params.front_edge_to_center() -
                           kSpeedRegionBuffer;
    const double end_s = path_speed_region.end_s() +
                         veh_geo_params.back_edge_to_center() +
                         kSpeedRegionBuffer;
    if (start_s >= end_s) continue;
    speed_limit_ranges.push_back(
        {.start_s = start_s,
         .end_s = end_s,
         .speed_limit =
             std::min(path_speed_region.max_speed(), max_speed_limit),
         .info = absl::StrFormat(
             "%s Id: %s",
             TypeCaseToString(path_speed_region.source().type_case()),
             path_speed_region.id())});
  }
  if (speed_limit_ranges.empty()) return std::nullopt;
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit GenerateCombinationSpeedLimit(
    const std::map<SpeedLimitTypeProto::Type, SpeedLimit>& speed_limit_map,
    double max_speed_limit) {
  std::vector<SpeedLimitRange> speed_limit_ranges;
  int cnt = 0;
  for (const auto& [_, speed_limit] : speed_limit_map) {
    cnt += speed_limit.speed_limit_ranges().size();
  }
  speed_limit_ranges.reserve(cnt);
  for (const auto& [_, speed_limit] : speed_limit_map) {
    for (const auto& range : speed_limit.speed_limit_ranges()) {
      speed_limit_ranges.push_back(range);
    }
  }
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

#define DEBUG_TOLL_SPEED_LIMIT (0)

std::optional<SpeedLimit> GenerateTollSpeedLimit(
    const DiscretizedPath& path_points,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit,
    const PlannerSemanticMapManager* planner_semantic_map_manager,
    double av_max_acc, double av_speed) {
  constexpr double kMaxDistToToll = 400.0;
  constexpr double kTargetSpeedToToll = Kph2Mps(30.0);
  constexpr double kTargetDistToToll = 75.0;
  std::vector<SpeedLimitRange> speed_limit_ranges;

  if (nullptr == planner_semantic_map_manager) return std::nullopt;

#if DEBUG_TOLL_SPEED_LIMIT
  std::cout << "------------------toll speed limit------------------"
            << std::endl;
#endif
  const auto& v2_info = planner_semantic_map_manager->map_ptr()->v2_info();

#if DEBUG_TOLL_SPEED_LIMIT
  std::cout << " v2: has_navigation " << v2_info.has_navigation << std::endl;
#endif

  if (!v2_info.has_navigation) return std::nullopt;
  const auto& sub_type = planner_semantic_map_manager->map_ptr()->sub_type();

#if DEBUG_TOLL_SPEED_LIMIT
  std::cout << " v2: sub_type " << sub_type << " dist_to_toll "
            << v2_info.dist_to_toll << std::endl;
#endif

  if (sub_type != ad_byd::planning::NEAR_TOLL &&
      sub_type != ad_byd::planning::NEAR_TOLL_FOR_LCC &&
      sub_type != ad_byd::planning::NEAR_TOLL_FOR_RAMP) {
    return std::nullopt;
  }
  if (v2_info.dist_to_toll >= kMaxDistToToll) return std::nullopt;

  double dist_to_toll = v2_info.dist_to_toll;
  double dist_buf = kTargetDistToToll - veh_geo_params.front_edge_to_center();
  double max_acc = av_max_acc;
  ad_byd::planning::SpeedPoint start(0.0, 0.0, kTargetSpeedToToll, 0.0, 0.0);
  ad_byd::planning::SpeedPoint end(30.0, 0.0, max_speed_limit, max_acc, 0.0);
  auto speed_builder = std::make_shared<ad_byd::planning::ConstantAccBuilder>();
  const auto speed_profile = speed_builder->Build(start, end, 30.0);

  if (!speed_profile || !speed_profile->IsValid()) return std::nullopt;
  double interval = std::fmax(std::fmin(20.0, 0.3 * dist_to_toll), 0.5);
  std::vector<ad_byd::planning::SpeedPoint> speed_points =
      speed_profile->SampleSpeedPointsByS(0.0, dist_to_toll, interval);

  if (speed_points.empty() || speed_points.back().v < av_speed) {
    return std::nullopt;
  }

  std::vector<std::pair<double, double>> speed_limit_point;

  for (const auto& p : speed_points) {
    double dist = p.s + dist_buf;
    if (dist >= dist_to_toll) {
      speed_limit_point.emplace_back(std::pair<double, double>(0.0, p.v));
      break;
    }
    speed_limit_point.emplace_back(
        std::pair<double, double>(dist_to_toll - dist, p.v));
  }

  reverse(speed_limit_point.begin(), speed_limit_point.end());

  for (double d = dist_to_toll - dist_buf + 20.0; d < dist_to_toll; d += 20.0) {
    speed_limit_point.emplace_back(
        std::pair<double, double>(d, kTargetSpeedToToll));
  }

#if DEBUG_TOLL_SPEED_LIMIT
  for (const auto& point : speed_limit_point) {
    std::cout << " point: start_s " << point.first << " speed_limit "
              << Mps2Kph(point.second) << "km/h" << std::endl;
  }
#endif

  std::vector<std::tuple<double, double, double>> start_end_speed_limit_point;

  for (int i = 0; i < speed_limit_point.size(); i++) {
    if (i == (speed_limit_point.size() - 1)) {
      start_end_speed_limit_point.emplace_back(
          std::tuple<double, double, double>(speed_limit_point[i].first,
                                             path_points.length(),
                                             speed_limit_point[i].second));
    } else {
      start_end_speed_limit_point.emplace_back(
          std::tuple<double, double, double>(speed_limit_point[i].first,
                                             speed_limit_point[i + 1].first,
                                             speed_limit_point[i].second));
    }
  }

  /* construct constraints */
#if DEBUG_TOLL_SPEED_LIMIT
  std::cout << " start_end_speed_limit_point: size "
            << start_end_speed_limit_point.size() << std::endl;
#endif
  int32_t toll_sped_limit_id = 0;

  for (const auto& single_result : start_end_speed_limit_point) {
    double start_s = std::get<0>(single_result);
    double end_s = std::get<1>(single_result);
    double curr_max_speed_limit = std::get<2>(single_result);
#if DEBUG_TOLL_SPEED_LIMIT
    std::cout << " toll_limit_result: idx " << toll_sped_limit_id << " start_s "
              << start_s << " end_s " << end_s << " curr_max_speed_limit "
              << Mps2Kph(curr_max_speed_limit) << "km/h" << std::endl;
#endif
    if (start_s >= end_s) continue;
    speed_limit_ranges.push_back(
        {.start_s = start_s,
         .end_s = end_s,
         .speed_limit = std::min(curr_max_speed_limit, max_speed_limit),
         .info =
             absl::StrFormat("toll_speed_limit [ %d ]", toll_sped_limit_id)});
    toll_sped_limit_id++;
  }

  if (speed_limit_ranges.empty()) return std::nullopt;
  return SpeedLimit(speed_limit_ranges);
}

std::optional<SpeedLimit> GenerateBigJunctionSpeedLimit(
    const DiscretizedPath& path_points,
    const SpeedLimitProto& speed_limit_config, double av_speed) {
  const auto get_speed_limit = [av_speed](const PathPoint& path_point) {
    constexpr double kBigJunctionMaxDec = -1.5;  // m/s^2.
    const double comfortable_brake_speed_sqr = std::fmax(
        0.0, av_speed * av_speed + 2.0 * kBigJunctionMaxDec * path_point.s());
    return std::sqrt(comfortable_brake_speed_sqr);
  };

  const double kTargetSpeedKph = 35.0;  // kph
  PiecewiseLinearFunction<double> user_set_speed_bias_plf(
      PiecewiseLinearFunctionFromProto(
          speed_limit_config.user_set_speed_bias_plf()));
  PiecewiseLinearFunction<double> user_set_speed_gain_plf(
      PiecewiseLinearFunctionFromProto(
          speed_limit_config.user_set_speed_gain_plf()));
  const double target_speed_mps = SpeedTransformation(
      user_set_speed_bias_plf, user_set_speed_gain_plf, kTargetSpeedKph);

  std::vector<SpeedLimitRange> speed_limit_ranges;
  const int num_points = path_points.size();
  speed_limit_ranges.reserve(num_points);
  // first: s second: v
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), get_speed_limit(path_points[0]));
  double last_sample_s = 0.0;
  for (int i = 1; i < num_points; ++i) {
    // Only check the speed limit at every meter to save computation.
    constexpr double kSpeedLimitSampleRange = 1.0;  // Meters.
    if (path_points[i].s() - last_sample_s > kSpeedLimitSampleRange ||
        i == num_points - 1) {
      last_sample_s = path_points[i].s();
      if (const double curr_speed_limit = get_speed_limit(path_points[i]);
          std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
              kApproxSpeedLimitEps ||
          i == num_points - 1) {
        CHECK_GT(path_points[i].s(), prev_speed_limit_point.first);
        speed_limit_ranges.push_back(
            {.start_s = prev_speed_limit_point.first,
             .end_s = path_points[i].s(),
             .speed_limit =
                 std::fmax(prev_speed_limit_point.second, target_speed_mps),
             .info = SpeedLimitTypeProto::Type_Name(
                 SpeedLimitTypeProto_Type_BIG_JUNCTION)});
        prev_speed_limit_point =
            std::make_pair(path_points[i].s(), curr_speed_limit);
      }
    }
  }
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

std::optional<SpeedLimit> GeneratePerceptionLossSpeedLimit() {
  // TODO:
  return std::nullopt;
}

std::optional<SpeedLimit> GenerateInJunctionTMapSpeedLimit() {
  // TODO:
  return std::nullopt;
}

std::optional<SpeedLimit> GenerateTJunctionMapSpeedLimit(
    const DiscretizedPath& path_points, double max_speed_limit, double av_speed,
    const SpeedLimitProto& speed_limit_config,
    const PlannerSemanticMapManager* psmm_ptr,
    const std::vector<DrivingTopoRoad>& topo_roads) {
  constexpr double kExtendDistance = 10.0;
  std::optional<double> start_s = std::nullopt;
  std::optional<double> end_s = std::nullopt;
  for (const auto& topo_road : topo_roads) {
    if (topo_road.turn_type != ad_byd::planning::TurnType::LEFT_TURN &&
        topo_road.turn_type != ad_byd::planning::TurnType::RIGHT_TURN) {
      continue;
    } else if (topo_road.real_start_s < 1e-2) {
      continue;
    } else {
      start_s = std::optional<double>(
          std::max(0.0, topo_road.real_start_s - kExtendDistance));
      end_s = std::optional<double>(topo_road.end_s);
      break;
    }
  }
  if ((!start_s.has_value()) || (!end_s.has_value())) {
    return std::nullopt;
  }
  if (start_s.value() > speed_limit_config.t_junction_brake_dis()) {
    return std::nullopt;
  }

  if (nullptr == psmm_ptr) return std::nullopt;
  if (nullptr == psmm_ptr->map_ptr()) return std::nullopt;
  const auto& ehp_v2_info = psmm_ptr->map_ptr()->v2_info();
  const auto& v2_road_class = ehp_v2_info.road_class;
  int road_class = 0;
  if (!v2_road_class.empty()) {
    if (v2_road_class.front().start_s < 40.0) {
      road_class = static_cast<int>(v2_road_class.front().type);
    }
  }
  if (!ehp_v2_info.has_navigation) return std::nullopt;

  V2TurnInfo curr_turn_info, next_turn_info;
  int idx;

  GetFirstValidV2TurnInfo(&curr_turn_info, &idx, ehp_v2_info.turn_info);

  if (!curr_turn_info.is_valid) return std::nullopt;

  int road_type = 0;
  if (!curr_turn_info.infos.empty()) {
    if (!curr_turn_info.infos.front().empty()) {
      road_type = atoi(curr_turn_info.infos.front().c_str());
    }
  }
  bool enable_t_junction_brake =
      (curr_turn_info.turn_type == V2TurnInfo::V2TurnType::LEFT ||
       curr_turn_info.turn_type == V2TurnInfo::V2TurnType::RIGHT) &&
      curr_turn_info.after_turn.lane_num == 1 &&
      curr_turn_info.before_turn.lane_num == 1 &&
      curr_turn_info.before_turn.road_class == 6 &&
      (road_type == 8 || road_type == 4);
  if (!enable_t_junction_brake) return std::nullopt;
  if (curr_turn_info.dist > end_s.value()) return std::nullopt;
  const double speed_limit =
      road_type == 8 ? Kph2Mps(speed_limit_config.t_junction_max_speed())
                     : Kph2Mps(10.0);

  const auto get_speed_limit = [av_speed, &start_s, &speed_limit_config,
                                &speed_limit,
                                max_speed_limit](const PathPoint& path_point) {
    if (path_point.s() < start_s.value()) {
      return max_speed_limit;
    }
    const double kTJunctionMaxDec =
        speed_limit_config.t_junction_brake_acc();  // m/s^2.
    const double comfortable_brake_speed_sqr =
        av_speed * av_speed + 2.0 * kTJunctionMaxDec * path_point.s();
    return std::max(std::sqrt(std::max(comfortable_brake_speed_sqr, 1e-1)),
                    speed_limit);
  };

  std::vector<SpeedLimitRange> speed_limit_ranges;
  const int num_points = path_points.size();
  speed_limit_ranges.reserve(num_points);
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), get_speed_limit(path_points[0]));
  double last_sample_s = 0.0;
  for (int i = 1; i < num_points; ++i) {
    constexpr double kSpeedLimitSampleRange = 0.3;  // Meters.
    if (path_points[i].s() - last_sample_s > kSpeedLimitSampleRange ||
        i == num_points - 1) {
      last_sample_s = path_points[i].s();
      if (const double curr_speed_limit = get_speed_limit(path_points[i]);
          std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
              kApproxSpeedLimitEps ||
          i == num_points - 1) {
        CHECK_GT(path_points[i].s(), prev_speed_limit_point.first);
        speed_limit_ranges.push_back(
            {.start_s = prev_speed_limit_point.first,
             .end_s = path_points[i].s(),
             .speed_limit = prev_speed_limit_point.second,
             .info = SpeedLimitTypeProto::Type_Name(
                 SpeedLimitTypeProto_Type_IN_JUNCTION_T_MAP)});
        prev_speed_limit_point =
            std::make_pair(path_points[i].s(), curr_speed_limit);
      }
    }
  }
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

}  // namespace

#define DEBUG_V2_SPEED_LIMIT (0)

int GetDataFrom(const std::vector<std::string>& infos, std::string type) {
  for (const std::string& info : infos) {
    size_t foundIndex = info.find(type);
    if (foundIndex != std::string::npos) {
      size_t index = info.find(":");
      if (index != std::string::npos && index + 1 < info.size()) {
        std::string str_data = info.substr(index + 1);
        int int_data = atoi(str_data.c_str());
        return int_data;
      }
    }
  }
  return 0;
}

bool IsEgoCloseCurb(const DiscretizedPath& path_points,
                    const std::vector<DistanceInfo>&
                        distance_info_to_impassable_path_boundaries) {
  constexpr double kMinDistToCurb = 0.7;
  constexpr double kMaxSearchDist = 80.0;

  const auto path_size_before_collision =
      distance_info_to_impassable_path_boundaries.size();

  for (int i = 1; i < path_size_before_collision; ++i) {
    if (ad_byd::planning::BoundaryType::OCC_VEGETATION ==
        distance_info_to_impassable_path_boundaries[i].type.boundary_type) {
      continue;
    }
    if (distance_info_to_impassable_path_boundaries[i].s > kMaxSearchDist) {
      break;
    }
    if (std::fabs(distance_info_to_impassable_path_boundaries[i].dist) <
        kMinDistToCurb) {
      return true;
    }
  }

  return false;
}

V2SpeedInfo GetMaxSpeedForV2DetailTurnType(const V2TurnInfo& curr_turn_info,
                                           const V2TurnInfo& next_turn_info,
                                           double dist_to_junction) {
  constexpr double kEhpV2TurnSpeedLimit = Kph2Mps(50.0);  // km/h
  constexpr double kEhpV2TurnLeftSpeedLimit =
      6.5;  // m/s 25 -> 6.5 display-real
  constexpr double kEhpV2TurnRightSpeedLimit = 6.5;            // m/s
  constexpr double kEhpV2TurnUTurnSpeedLimit = Kph2Mps(10.0);  // m/s
  constexpr double kDistLeftToNextTurnThreshold = 50.0;        // m
  constexpr double kV2SpeedLimitLength = 30.0;                 // m
  // constexpr double kReverseLengthToLeftTurn = 30.0;      // m
  constexpr double kReverseLengthToUTurn = 20.0;       // m
  constexpr double kReverseLengthToTurn = 25.0;        // m
  constexpr double kReverseLengthToSlightTurn = 25.0;  // m
  constexpr double kV2SpeedLimitKeepLength = 30.0;     // m
  constexpr double kV2LeftTurnMaxDecel = -1.0;         // m/ss
  constexpr double kV2RightTurnMaxDecel = -0.8;        // m/ss
  constexpr double kV2SoftTurnDecel = -0.5;            // m/ss
  constexpr int kBeforeLaneMinNum = 3;
  constexpr int kTurnOnlyLaneNum = 1;

  double dist_between_next_turn_type = 0.0;  // m                // m
  const auto cur_detail_turn_type = curr_turn_info.detail_turn_type;
  const auto next_detail_turn_type = next_turn_info.detail_turn_type;
  V2SpeedInfo v2_speed_info;
  if (curr_turn_info.is_valid && next_turn_info.is_valid) {
    dist_between_next_turn_type =
        std::fabs(static_cast<double>(curr_turn_info.original_dist) -
                  static_cast<double>(next_turn_info.original_dist));
  }

  auto calculate_turn_speed_info =
      [&v2_speed_info, curr_turn_info, dist_to_junction](
          double speed_limit, double reverse_length, double speed_limit_length,
          double v2_turn_decel) {
        v2_speed_info.max_speed = speed_limit;
        v2_speed_info.start_s = dist_to_junction - reverse_length;
        v2_speed_info.end_s = v2_speed_info.start_s + speed_limit_length;
        v2_speed_info.max_acc = v2_turn_decel;
      };
  switch (cur_detail_turn_type) {
    case V2DetailType::TURN_LEFT:
      calculate_turn_speed_info(kEhpV2TurnLeftSpeedLimit, kReverseLengthToTurn,
                                kV2SpeedLimitKeepLength, kV2LeftTurnMaxDecel);
    case V2DetailType::TURN_RIGHT:
      calculate_turn_speed_info(kEhpV2TurnRightSpeedLimit, kReverseLengthToTurn,
                                kV2SpeedLimitKeepLength, kV2RightTurnMaxDecel);
      break;
    case V2DetailType::SLIGHT_LEFT:
    case V2DetailType::SLIGHT_RIGHT:
      calculate_turn_speed_info(kEhpV2TurnSpeedLimit,
                                kReverseLengthToSlightTurn, kV2SpeedLimitLength,
                                kV2SoftTurnDecel);
      if (next_turn_info.is_valid &&
          next_detail_turn_type == V2DetailType::TURN_LEFT &&
          cur_detail_turn_type == V2DetailType::SLIGHT_LEFT &&
          dist_between_next_turn_type <= kDistLeftToNextTurnThreshold) {
        calculate_turn_speed_info(kEhpV2TurnLeftSpeedLimit,
                                  kReverseLengthToTurn, kV2SpeedLimitKeepLength,
                                  kV2LeftTurnMaxDecel);
      } else if (next_turn_info.is_valid &&
                 next_detail_turn_type == V2DetailType::TURN_RIGHT &&
                 cur_detail_turn_type == V2DetailType::SLIGHT_RIGHT &&
                 dist_between_next_turn_type <= kDistLeftToNextTurnThreshold) {
        calculate_turn_speed_info(kEhpV2TurnRightSpeedLimit,
                                  kReverseLengthToTurn, kV2SpeedLimitLength,
                                  kV2RightTurnMaxDecel);
      } else if (curr_turn_info.before_turn.lane_num >= kBeforeLaneMinNum &&
                 curr_turn_info.after_turn.lane_num == kTurnOnlyLaneNum) {
        calculate_turn_speed_info(kEhpV2TurnRightSpeedLimit,
                                  kReverseLengthToTurn, kV2SpeedLimitLength,
                                  kV2RightTurnMaxDecel);
      }
      break;
    case V2DetailType::UTURN:
    case V2DetailType::UTURN_RIGHT:
      calculate_turn_speed_info(kEhpV2TurnUTurnSpeedLimit,
                                kReverseLengthToUTurn, kV2SpeedLimitLength,
                                kV2RightTurnMaxDecel);
      break;
    case V2DetailType::TURN_RIGHT_ONLY:
      calculate_turn_speed_info(kEhpV2TurnRightSpeedLimit, kReverseLengthToTurn,
                                kV2SpeedLimitLength, kV2RightTurnMaxDecel);
      break;
    case V2DetailType::NONE:
    case V2DetailType::TURN_HARD_LEFT:
    case V2DetailType::TURN_HARD_RIGHT:
    case V2DetailType::CONTINUE:
    case V2DetailType::LEFT_MERGE:
    case V2DetailType::RIGHT_MERGE:
      break;
  }
  return v2_speed_info;
}

void GetMaxSpeedForV2TurnType(double* max_speed, double* start_s, double* end_s,
                              double* max_acc, EgoFrame* curr_ego_frame,
                              const V2TurnInfo curr_turn_info,
                              const V2TurnInfo next_turn_info,
                              const DiscretizedPath& path_points,
                              const std::vector<DistanceInfo>&
                                  distance_info_to_impassable_path_boundaries,
                              const EgoHistory* ego_history,
                              const SpeedLimitProto& speed_limit_config) {
  constexpr double kV2SpeedLimitLength = 18.0;
  constexpr double kEhpV2TurnSpeedLimit = Kph2Mps(50.0);
  constexpr double kEhpV2TurnLeftSpeedLimit = Kph2Mps(40.0);
  constexpr double kEhpV2TurnRightSpeedLimit = Kph2Mps(20.0);
  constexpr double kEhpV2SmallTurnLeftSpeedLimit = Kph2Mps(18.0);

  if (nullptr == max_speed || nullptr == start_s || nullptr == end_s ||
      nullptr == max_acc) {
    return;
  }

  double delta_dist = 100000;
  const auto& next_detail_turn_type = next_turn_info.detail_turn_type;
  if (curr_turn_info.is_valid && next_turn_info.is_valid) {
    delta_dist = std::fabs(static_cast<double>(curr_turn_info.original_dist) -
                           static_cast<double>(next_turn_info.original_dist));
  }

  const auto& cur_turn_type = curr_turn_info.turn_type;
  const auto& cur_detail_turn_type = curr_turn_info.detail_turn_type;

  if (nullptr != curr_ego_frame) {
    curr_ego_frame->v2_speed_limit_info.is_close_curb = IsEgoCloseCurb(
        path_points, distance_info_to_impassable_path_boundaries);
  }

  // int road_type = GetDataFrom(curr_turn_info.infos, "road type");
  int road_type = 0;
  if (!curr_turn_info.infos.empty()) {
    if (!curr_turn_info.infos.front().empty()) {
      road_type = atoi(curr_turn_info.infos.front().c_str());
    }
  }

#if DEBUG_V2_SPEED_LIMIT
  std::cout << "[before_turn]: road_class "
            << curr_turn_info.before_turn.road_class << " lane_num "
            << curr_turn_info.before_turn.lane_num << " road_type " << road_type
            << std::endl;
  std::cout << "[after_turn]: road_class "
            << curr_turn_info.after_turn.road_class << " lane_num "
            << curr_turn_info.after_turn.lane_num << " road_type " << road_type
            << std::endl;
#endif

  if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT ==
      cur_detail_turn_type) {
    if (curr_turn_info.after_turn.lane_num == 1 &&
        curr_turn_info.after_turn.road_class == 6 &&
        curr_turn_info.before_turn.lane_num == 1 &&
        curr_turn_info.before_turn.road_class == 6 && road_type == 4) {
      *start_s =
          curr_turn_info.dist - speed_limit_config.v2_small_left_delta_dist();
      *end_s = *start_s + speed_limit_config.v2_small_left_end_s();
      *max_acc = speed_limit_config.v2_small_left_min_acc();
      *max_speed = Kph2Mps(speed_limit_config.v2_small_left_speed_max());
#if DEBUG_V2_SPEED_LIMIT
      std::cout
          << " [max_speed - 1]: turn right-- small --> small single type: 4"
          << " speed_result: " << Mps2Kph(*max_speed) << " km/h " << std::endl;
#endif
    } else if (curr_turn_info.before_turn.road_class == 6 &&
               curr_turn_info.after_turn.road_class < 6 &&
               (road_type == 4 || road_type == 8)) {
      *start_s = curr_turn_info.dist - 25.0;
      *end_s = *start_s + 18.0;
      *max_acc = -0.8;
      *max_speed = Kph2Mps(20.0);
#if DEBUG_V2_SPEED_LIMIT
      std::cout << " [max_speed - 2]: turn right-- small --> big type: 4 or 8"
                << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                << std::endl;
#endif
    } else {
      *max_speed = kEhpV2TurnRightSpeedLimit;
#if DEBUG_V2_SPEED_LIMIT
      std::cout << " [max_speed - 3]: turn right-- normal "
                << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                << std::endl;
#endif
    }
  } else if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT ==
             cur_detail_turn_type) {
    if (curr_turn_info.after_turn.lane_num == 1 &&
        curr_turn_info.after_turn.road_class == 6 &&
        curr_turn_info.before_turn.lane_num == 1 &&
        curr_turn_info.before_turn.road_class == 6 && road_type == 4) {
      *start_s =
          curr_turn_info.dist - speed_limit_config.v2_small_left_delta_dist();
      *end_s = *start_s + speed_limit_config.v2_small_left_end_s();
      *max_acc = speed_limit_config.v2_small_left_min_acc();
      *max_speed = Kph2Mps(speed_limit_config.v2_small_left_speed_max());
#if DEBUG_V2_SPEED_LIMIT
      std::cout
          << " [max_speed - 4]: turn left-- small --> small single type: 4"
          << " speed_result: " << Mps2Kph(*max_speed) << " km/h " << std::endl;
#endif
    } else if (curr_turn_info.before_turn.road_class < 6 &&
               curr_turn_info.after_turn.road_class == 6) {
      *max_speed = Kph2Mps(30.0);
      *start_s =
          curr_turn_info.dist - speed_limit_config.v2_small_left_delta_dist();
      *end_s = *start_s + speed_limit_config.v2_small_left_end_s();
      *max_acc = speed_limit_config.v2_small_left_min_acc();
#if DEBUG_V2_SPEED_LIMIT
      std::cout << " [max_speed - 5]: turn left-- big --> small"
                << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                << std::endl;
#endif
    } else if (curr_turn_info.before_turn.road_class == 6 &&
               curr_turn_info.after_turn.road_class == 6 && road_type == 8) {
      *max_speed = Kph2Mps(30.0);
      *start_s =
          curr_turn_info.dist - speed_limit_config.v2_small_left_delta_dist();
      *end_s = *start_s + speed_limit_config.v2_small_left_end_s();
      *max_acc = speed_limit_config.v2_small_left_min_acc();
#if DEBUG_V2_SPEED_LIMIT
      std::cout << " [max_speed - 6]: turn left-- small --> small type: 8"
                << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                << std::endl;
#endif
    } else {
      *max_speed = kEhpV2TurnLeftSpeedLimit;
#if DEBUG_V2_SPEED_LIMIT
      std::cout << " [max_speed - 7]: turn left-- normal"
                << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                << std::endl;
#endif
    }
  } else if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT_ONLY ==
             cur_detail_turn_type) {
    *max_speed = Kph2Mps(20.0);
#if DEBUG_V2_SPEED_LIMIT
    std::cout << " [max_speed - 8]: turn right only-- normal"
              << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
              << std::endl;
#endif
  } else if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_HARD_RIGHT ==
             cur_detail_turn_type) {
    *max_speed = Kph2Mps(20.0);
#if DEBUG_V2_SPEED_LIMIT
    std::cout << " [max_speed - 9]: turn right behind-- normal"
              << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
              << std::endl;
#endif
  } else if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_HARD_LEFT ==
             cur_detail_turn_type) {
    *max_speed = Kph2Mps(20.0);
#if DEBUG_V2_SPEED_LIMIT
    std::cout << " [max_speed - 10]: turn left behind-- normal"
              << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
              << std::endl;
#endif
  } else if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::SLIGHT_RIGHT ==
                 cur_detail_turn_type ||
             ad_byd::planning::V2TurnInfo::V2DetailTurnType::RIGHT_MERGE ==
                 cur_detail_turn_type) {
    *max_speed = kEhpV2TurnSpeedLimit;
#if DEBUG_V2_SPEED_LIMIT
    std::cout << " [max_speed - 11]: turn right front-- normal"
              << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
              << std::endl;
    std::cout << " next_turn_info: is_valid " << next_turn_info.is_valid
              << " turn_type:  " << next_detail_turn_type
              << " delta_dist: " << delta_dist << std::endl;
#endif
    if (next_turn_info.is_valid && delta_dist <= 50.0) {
      if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT ==
          next_detail_turn_type) {
        *max_speed = kEhpV2TurnRightSpeedLimit;
        *start_s =
            curr_turn_info.dist - speed_limit_config.v2_small_left_delta_dist();
        *end_s = *start_s + speed_limit_config.v2_small_left_end_s();
        *max_acc = speed_limit_config.v2_small_left_min_acc();
#if DEBUG_V2_SPEED_LIMIT
        std::cout << " [max_speed - 12]: turn right front --> turn right"
                  << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                  << std::endl;
#endif
      }
    } else {
      if (nullptr != ego_history && nullptr != curr_ego_frame &&
          ego_history->Size() >= 2) {
        const EgoFrame* latest_ego_frame = ego_history->GetLatestFrame();
        const EgoFrame* second_latest_ego_frame =
            ego_history->GetSecondLatestFrame();
        if (nullptr != latest_ego_frame && nullptr != second_latest_ego_frame) {
#if DEBUG_V2_SPEED_LIMIT
          std::cout
              << "is_second_last_close_curb: "
              << second_latest_ego_frame->v2_speed_limit_info.is_close_curb
              << " is_latest_close_curb: "
              << latest_ego_frame->v2_speed_limit_info.is_close_curb
              << " is_curr_close_curb: "
              << curr_ego_frame->v2_speed_limit_info.is_close_curb
              << " latest_is_generate_small_limit: "
              << latest_ego_frame->v2_speed_limit_info
                     .is_generate_small_speed_limit
              << std::endl;
#endif
          if (!latest_ego_frame->v2_speed_limit_info
                   .is_generate_small_speed_limit) {
            curr_ego_frame->v2_speed_limit_info.is_generate_small_speed_limit =
                second_latest_ego_frame->v2_speed_limit_info.is_close_curb &&
                latest_ego_frame->v2_speed_limit_info.is_close_curb &&
                curr_ego_frame->v2_speed_limit_info.is_close_curb;
            if (curr_ego_frame->v2_speed_limit_info
                    .is_generate_small_speed_limit) {
              *max_speed = Kph2Mps(30.0);
#if DEBUG_V2_SPEED_LIMIT
              std::cout << " [max_speed - 13]: turn right front --> curb"
                        << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                        << std::endl;
#endif
            }
          } else {
            curr_ego_frame->v2_speed_limit_info.is_generate_small_speed_limit =
                latest_ego_frame->v2_speed_limit_info
                    .is_generate_small_speed_limit;
            *max_speed = Kph2Mps(30.0);
#if DEBUG_V2_SPEED_LIMIT
            std::cout << " [max_speed - 14]: turn right front --> curb"
                      << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                      << std::endl;
#endif
          }
        }
      }
    }
  } else if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::SLIGHT_LEFT ==
             cur_detail_turn_type) {
    *max_speed = kEhpV2TurnSpeedLimit;
#if DEBUG_V2_SPEED_LIMIT
    std::cout << " [max_speed - 15]: turn left front --> normal"
              << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
              << std::endl;
    std::cout << " next_turn_info: is_valid " << next_turn_info.is_valid
              << " turn_type:  " << next_detail_turn_type
              << " delta_dist: " << delta_dist << std::endl;
#endif
    if (next_turn_info.is_valid && delta_dist <= 50.0) {
      if (ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT ==
          next_detail_turn_type) {
        *max_speed = kEhpV2TurnLeftSpeedLimit;
        *start_s = next_turn_info.dist - speed_limit_config.v2_delta_dist();
        *end_s = *start_s + kV2SpeedLimitLength;
#if DEBUG_V2_SPEED_LIMIT
        std::cout << " [max_speed - 16]: turn left front --> turn left"
                  << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                  << std::endl;
#endif
      }
    } else {
      if (nullptr != ego_history && nullptr != curr_ego_frame &&
          ego_history->Size() >= 2) {
        const EgoFrame* latest_ego_frame = ego_history->GetLatestFrame();
        const EgoFrame* second_latest_ego_frame =
            ego_history->GetSecondLatestFrame();
        if (nullptr != latest_ego_frame && nullptr != second_latest_ego_frame) {
#if DEBUG_V2_SPEED_LIMIT
          std::cout
              << "is_second_last_close_curb: "
              << second_latest_ego_frame->v2_speed_limit_info.is_close_curb
              << " is_latest_close_curb: "
              << latest_ego_frame->v2_speed_limit_info.is_close_curb
              << " is_curr_close_curb: "
              << curr_ego_frame->v2_speed_limit_info.is_close_curb
              << " latest_is_generate_small_limit: "
              << latest_ego_frame->v2_speed_limit_info
                     .is_generate_small_speed_limit
              << std::endl;
#endif
          if (!latest_ego_frame->v2_speed_limit_info
                   .is_generate_small_speed_limit) {
            curr_ego_frame->v2_speed_limit_info.is_generate_small_speed_limit =
                second_latest_ego_frame->v2_speed_limit_info.is_close_curb &&
                latest_ego_frame->v2_speed_limit_info.is_close_curb &&
                curr_ego_frame->v2_speed_limit_info.is_close_curb;
            if (curr_ego_frame->v2_speed_limit_info
                    .is_generate_small_speed_limit) {
              *max_speed = Kph2Mps(30.0);
#if DEBUG_V2_SPEED_LIMIT
              std::cout << " [max_speed - 17]: turn left front --> curb"
                        << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                        << std::endl;
#endif
            }
          } else {
            curr_ego_frame->v2_speed_limit_info.is_generate_small_speed_limit =
                latest_ego_frame->v2_speed_limit_info
                    .is_generate_small_speed_limit;
            *max_speed = Kph2Mps(30.0);
#if DEBUG_V2_SPEED_LIMIT
            std::cout << " [max_speed - 18]: turn left front --> curb"
                      << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
                      << std::endl;
#endif
          }
        }
      }
    }
  } else {
    *max_speed = kEhpV2TurnSpeedLimit;
#if DEBUG_V2_SPEED_LIMIT
    std::cout << " [max_speed - 19]: turn normal"
              << " speed_result: " << Mps2Kph(*max_speed) << " km/h "
              << std::endl;
#endif
  }

  return;
}
std::optional<SpeedLimit> GenerateV2DetailTurnTypeSpeedLimit(
    const DiscretizedPath& path_points,
    const PlannerSemanticMapManager* planner_semantic_map_manager,
    const DrivePassage& drive_passage, double max_speed_limit, double av_speed,
    std::optional<double>* v2_trigger_distance) {
  constexpr double kSpeedProfileRanges = 30.0;      // m
  constexpr double kSpeedProfileMinInterval = 0.5;  // m
  constexpr double kDistFactor = 0.1;
  constexpr double kSpeedProfileStep = 1.0;                       // m/s
  constexpr double kDistanceOffset = 0.1;                         // m
  constexpr double kSpeedProfileMinGap = 0.1;                     // m/s
  constexpr double kV2TurnSpeedLimitApplicationDistance = 200.0;  // m
  constexpr double kV2TurnSpeedLimitReverseDistance = 30.0;       // m
  constexpr double kV2TurnInfoDistBuffer = 10.0;                  // m
  constexpr double kV2TurnMaxDecel = -0.8;                        // m/ss
  constexpr double kJunctionV2DistanceGapBuffer = 100.0;          // m
  constexpr double kSwitchToTheVisualRange = 120.0;               // m
  constexpr double kV2TurnSpeedLimitMinDecel = -0.3;              // m/ss
  // constexpr double kV2TurnAvCanAccelerationSpeedBUffer = 3.0;     // m/s

  // using SpeedPoint = ad_byd::planning::SpeedPoint;
  // using ConstantAccBuilder = ad_byd::planning::ConstantAccBuilder;
  using V2RoadClass = ad_byd::planning::V2RoadClass;

  V2TurnInfo curr_turn_info, next_turn_info;

  const auto road_type = planner_semantic_map_manager->GetRoadClass();
  const bool is_on_highway = planner_semantic_map_manager->IsOnHighway() ||
                             road_type == V2RoadClass::HIGH_WAY_ROAD;
  if (is_on_highway) return std::nullopt;

  if (nullptr == planner_semantic_map_manager) return std::nullopt;
  const auto& ehp_v2_info = planner_semantic_map_manager->map_ptr()->v2_info();

  if (!ehp_v2_info.has_navigation) return std::nullopt;

  double dist_to_junction = 0.0;
  if (drive_passage.lane_seq_info() != nullptr) {
    dist_to_junction = drive_passage.lane_seq_info()->dist_to_junction;
  }
  // Get close v2_detail_type & scencrio_recongnize
  const std::optional<int> first_v2_turn_idx =
      GetFirstV2TurnInfoIndex(ehp_v2_info.turn_info);
  if (!first_v2_turn_idx.has_value()) {
    return std::nullopt;
  }

  curr_turn_info = ehp_v2_info.turn_info[*first_v2_turn_idx];
  if (curr_turn_info.dist < kV2TurnInfoDistBuffer ||
      curr_turn_info.dist > kV2TurnSpeedLimitApplicationDistance) {
    *v2_trigger_distance = std::nullopt;
    return std::nullopt;
  }

  if (curr_turn_info.dist <= kV2TurnSpeedLimitApplicationDistance) {
    auto decel_to_target_speed_distance =
        CalculateMinDistanceDecelToTargetSpeed(
            curr_turn_info, av_speed, kV2TurnMaxDecel,
            kV2TurnSpeedLimitApplicationDistance);
    if (!decel_to_target_speed_distance.has_value()) {
      return std::nullopt;
    } else {
      if (!v2_trigger_distance->has_value()) {
        *v2_trigger_distance = decel_to_target_speed_distance;
      } else {
        *v2_trigger_distance =
            std::min(std::max(v2_trigger_distance->value(),
                              decel_to_target_speed_distance.value()),
                     curr_turn_info.dist);
      }
    }
  }

  if (v2_trigger_distance->has_value() &&
      curr_turn_info.dist > v2_trigger_distance->value()) {
    return std::nullopt;
  }

  if (*first_v2_turn_idx + 1 > 0 &&
      *first_v2_turn_idx + 1 < ehp_v2_info.turn_info.size()) {
    next_turn_info = ehp_v2_info.turn_info[*first_v2_turn_idx + 1];
  } else {
    next_turn_info.is_valid = false;
  }

  if (curr_turn_info.dist <= kSwitchToTheVisualRange) {
    dist_to_junction =
        abs(curr_turn_info.dist - dist_to_junction) >
                kJunctionV2DistanceGapBuffer
            ? curr_turn_info.dist - kV2TurnInfoDistBuffer
            : std::min(dist_to_junction,
                       curr_turn_info.dist - kV2TurnInfoDistBuffer);
  } else {
    dist_to_junction = curr_turn_info.dist - kV2TurnInfoDistBuffer;
  }
  if (dist_to_junction < kDistanceOffset) {
    *v2_trigger_distance = std::nullopt;
    return std::nullopt;
  }
  // calculate speed_limit_point;
  V2SpeedInfo v2_speed_info = GetMaxSpeedForV2DetailTurnType(
      curr_turn_info, next_turn_info, dist_to_junction);
  const double start_s = v2_speed_info.start_s;
  const double end_s = v2_speed_info.end_s;
  double max_speed = v2_speed_info.max_speed;
  double max_acc = 0.0;

  // if (max_acc > kV2TurnSpeedLimitMinDecel) return std::nullopt;
  if (av_speed + kSpeedProfileMinGap < max_speed || end_s < kDistanceOffset) {
    *v2_trigger_distance = std::nullopt;
    return std::nullopt;
  }

  double interval = std::clamp(kDistFactor * start_s, kSpeedProfileMinInterval,
                               kSpeedProfileStep);
  int speed_limit_point_size = static_cast<int>(end_s / interval) + 1;

  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.clear();

  for (size_t i = 0; i < speed_limit_point_size; ++i) {
    double current_s = i * interval;
    double target_speed_limit = 0.0;
    if (current_s > end_s) break;
    double decel_current_v = std::max(
        sqrt(std::max((Sqr(av_speed) + 2.0 * current_s * kV2TurnMaxDecel),
                      kEps)),
        max_speed);
    if (start_s > kDistanceOffset) {
      target_speed_limit = current_s < start_s ? decel_current_v : max_speed;
    } else {
      target_speed_limit = max_speed;
    }
    speed_limit_ranges.push_back(
        {.start_s = current_s,
         .end_s = (i + 1) * interval,
         .speed_limit = target_speed_limit,
         .info =
             absl::StrFormat("cautious_ehp_v2_turn_%d", curr_turn_info.id)});
  }
  if (speed_limit_ranges.empty()) {
    *v2_trigger_distance = std::nullopt;
    return std::nullopt;
  }
  return SpeedLimit(speed_limit_ranges);
}

std::optional<SpeedLimit> GenerateV2TurnTypeSpeedLimit(
    const DiscretizedPath& path_points,
    const PlannerSemanticMapManager* planner_semantic_map_manager,
    double max_speed_limit, double av_speed,
    const SpeedLimitProto& speed_limit_config, const Behavior& behavior,
    const EgoHistory* ego_history, EgoFrame* curr_ego_frame,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries) {
  constexpr double kV2SpeedLimitLength = 18.0;
  constexpr double kEhpV2TurnSpeedLimit = Kph2Mps(50.0);
  constexpr double kSpeedProfileStep = 1.0;
  std::vector<SpeedLimitRange> speed_limit_ranges;

  speed_limit_ranges.clear();

#if DEBUG_V2_SPEED_LIMIT
  std::cout << "-----------------v2_speed_limit---------------" << std::endl;
#endif

  if (nullptr == planner_semantic_map_manager) return std::nullopt;
  const auto& ehp_v2_info = planner_semantic_map_manager->map_ptr()->v2_info();

#if DEBUG_V2_SPEED_LIMIT
  std::cout << " has_navigation " << ehp_v2_info.has_navigation << std::endl;
#endif
  bool lcc_ai_driver_mode =
      (behavior.function_id() == Behavior_FunctionId_LKA &&
       behavior.auto_navi_lc_enable_status());

  if ((!lcc_ai_driver_mode && !ehp_v2_info.has_navigation)) {
    return std::nullopt;
  }

  V2TurnInfo curr_turn_info, next_turn_info;
  int idx;

  GetFirstValidV2TurnInfo(&curr_turn_info, &idx, ehp_v2_info.turn_info);
  if (!curr_turn_info.is_valid) return std::nullopt;

#if DEBUG_V2_SPEED_LIMIT
  for (int i = 0; i < ehp_v2_info.turn_info.size(); i++) {
    auto turn_info = ehp_v2_info.turn_info[i];
    std::cout << " turn_Info: "
              << " detail_type " << turn_info.detail_turn_type << " turn_type "
              << turn_info.turn_type << " original_dist "
              << turn_info.original_dist << " dist " << turn_info.dist
              << std::endl;
  }
#endif

  next_turn_info.is_valid = false;

  if ((idx + 1) > 0 && (idx + 1) < ehp_v2_info.turn_info.size()) {
    next_turn_info = ehp_v2_info.turn_info[idx + 1];
#if DEBUG_V2_SPEED_LIMIT
    std::cout << " next_turn_info: "
              << "type " << next_turn_info.detail_turn_type << " original_dist "
              << next_turn_info.original_dist << " idx " << idx << " size "
              << ehp_v2_info.turn_info.size() << std::endl;
#endif
  }

  const auto& cur_turn_type = curr_turn_info.turn_type;
  const auto& cur_detail_turn_type = curr_turn_info.detail_turn_type;

  if (V2TurnInfo::V2TurnType::LEFT != cur_turn_type &&
      V2TurnInfo::V2TurnType::RIGHT != cur_turn_type &&
      V2TurnInfo::V2TurnType::MERGE_LEFT != cur_turn_type &&
      V2TurnInfo::V2TurnType::MERGE_RIGHT != cur_turn_type) {
    return std::nullopt;
  }

  double start_s = curr_turn_info.dist - speed_limit_config.v2_delta_dist();
  double max_acc = speed_limit_config.v2_min_acc();
  double end_s = start_s + kV2SpeedLimitLength;
  double max_speed;

#if DEBUG_V2_SPEED_LIMIT
  std::cout << " turn_info_size " << ehp_v2_info.turn_info.size()
            << " first_is_valid " << curr_turn_info.is_valid
            << " first_turn_type " << curr_turn_info.turn_type
            << " first_v2_dist " << curr_turn_info.dist << std::endl;
#endif

  GetMaxSpeedForV2TurnType(
      &max_speed, &start_s, &end_s, &max_acc, curr_ego_frame, curr_turn_info,
      next_turn_info, path_points, distance_info_to_impassable_path_boundaries,
      ego_history, speed_limit_config);

#if DEBUG_V2_SPEED_LIMIT
  std::cout << "****** find ! start s: " << start_s << " | end s : " << end_s
            << " | speed limit : " << kEhpV2TurnSpeedLimit << " | max_speed "
            << Mps2Kph(max_speed) << " | av_traj_length "
            << path_points.length() << std::endl;
#endif

  if (start_s > 0) {
    double calc_max_acc =
        (max_speed * max_speed - av_speed * av_speed) / (2.0 * start_s);
    max_acc = std::min(max_acc, calc_max_acc);
  }

  ad_byd::planning::SpeedPoint start(0.0, 0.0, av_speed, 0.0, 0.0);
  ad_byd::planning::SpeedPoint end(30.0, 0.0, max_speed, max_acc, 0.0);
  auto speed_builder = std::make_shared<ad_byd::planning::ConstantAccBuilder>();
  const auto speed_profile = speed_builder->Build(start, end, 30.0);

  if (!speed_profile || !speed_profile->IsValid()) return std::nullopt;
  double interval = std::fmax(std::fmin(kSpeedProfileStep, 0.1 * start_s), 0.5);
  std::vector<ad_byd::planning::SpeedPoint> speed_points =
      speed_profile->SampleSpeedPointsByS(0.0, std::max(start_s, interval),
                                          interval);

  std::vector<std::pair<double, double>> speed_limit_point;

  for (const auto& point : speed_points) {
    if (point.v <= (max_speed + 0.1)) {
#if DEBUG_V2_SPEED_LIMIT
      std::cout << " speed_profile_end_s " << point.s << " point.v "
                << Mps2Kph(point.v) << std::endl;
#endif
      if (point.s < start_s) {
        return std::nullopt;
      }
      break;
    }
  }

  for (const auto& p : speed_points) {
    if (p.s >= start_s) {
      break;
    }
    speed_limit_point.emplace_back(std::pair<double, double>(p.s, p.v));
  }

  for (double dist = start_s; dist <= end_s; dist = dist + kSpeedProfileStep) {
    speed_limit_point.emplace_back(std::pair<double, double>(dist, max_speed));
  }

#if DEBUG_V2_SPEED_LIMIT
  for (const auto& point : speed_limit_point) {
    std::cout << " point: start_s " << point.first << " speed_limit "
              << Mps2Kph(point.second) << "km/h" << std::endl;
  }
#endif

  std::vector<std::tuple<double, double, double>> start_end_speed_limit_point;

  for (int i = 0; i < speed_limit_point.size(); i++) {
    if (i == (speed_limit_point.size() - 1)) {
      start_end_speed_limit_point.emplace_back(
          std::tuple<double, double, double>(speed_limit_point[i].first,
                                             path_points.length(),
                                             speed_limit_point[i].second));
    } else {
      start_end_speed_limit_point.emplace_back(
          std::tuple<double, double, double>(speed_limit_point[i].first,
                                             speed_limit_point[i + 1].first,
                                             speed_limit_point[i].second));
    }
  }

  for (const auto& single_result : start_end_speed_limit_point) {
    double start_s = std::get<0>(single_result);
    double end_s = std::get<1>(single_result);
    double curr_max_speed_limit = std::get<2>(single_result);
#if DEBUG_V2_SPEED_LIMIT
    std::cout << " v2_limit_result: idx "
              << " start_s " << start_s << " end_s " << end_s
              << " curr_max_speed_limit " << Mps2Kph(curr_max_speed_limit)
              << "km/h"
              << " max_acc " << max_acc << std::endl;
#endif
    if (start_s >= end_s) continue;
    speed_limit_ranges.push_back(
        {.start_s = start_s,
         .end_s = end_s,
         .speed_limit = std::min(curr_max_speed_limit, max_speed_limit),
         .info =
             absl::StrFormat("cautious_ehp_v2_turn_%d", curr_turn_info.id)});
  }

  if (speed_limit_ranges.empty()) return std::nullopt;

  return SpeedLimit(speed_limit_ranges);
}

#define DEBUG_NEAREST_CLOSE (0)

std::optional<SpeedLimit> GenerateNearestCloseSpeedLimit(
    const std::vector<StCloseTrajectory>& moving_close_trajs,
    double max_speed_limit, double av_speed, const DiscretizedPath& path_points,
    const SpacetimeTrajectoryManager& traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const LaneChangeStateProto lane_change_state,
    const NudgeObjectInfo* nudge_object_info,
    const bool is_narrow_near_large_vehicle) {
  constexpr double kMinDeltaDist = 30.0;
  const std::vector<double> kCloseObjectLatDistRange = {0.0, 0.4, 0.5, 0.7,
                                                        1.0};  // m.

  const PiecewiseLinearFunction<double, double> kLatNearestCloseRelVPlf = {
      kCloseObjectLatDistRange, {-1.0, -1.0, 3.0, 4.0, 4.0}};  // m/s.

  const std::vector<double> kCloseObjectLonDistRange = {0.0, 8.0, 10.0, 12.0,
                                                        16.0};  // m.
  const PiecewiseLinearFunction<double, double> kLonNearestCloseRelVPlf = {
      kCloseObjectLonDistRange, {-1.0, -0.5, -0.5, -0.5, 0.0}};  // m/s.

  constexpr double kMinVelocity = Kph2Mps(20.0);
  constexpr double kTtcThreshold = 3.5;
  constexpr double kTtcThresholdSmallDeltaV = 5.0;
  constexpr double kDeltaVel = Kph2Mps(10.0);
  constexpr double kMaxDist = 60.0;
  constexpr double kMaxDeltaTheta = ad_byd::planning::Constants::DEG2RAD * 40.0;
  constexpr double kMinDeltaTheta = ad_byd::planning::Constants::DEG2RAD * 2.0;
  std::vector<SpeedLimitRange> speed_limit_ranges;
  constexpr double kVehicleComfortableAcc = -0.5;
  constexpr double kCyclistComfortableAcc = -0.8;
  constexpr double kMaxBrakeAcc = -1.5;
  constexpr double kCyclistLatDistThreshold = 0.8;
  constexpr double kNudgeCyclistLatDistThreshold = 0.6;

#if DEBUG_NEAREST_CLOSE
  std::cout << "--------------------------" << std::endl;
  std::cout << "   nearest close obj   " << std::endl;
  std::cout << " lc_state: " << lane_change_state.stage() << " lc_left "
            << lane_change_state.lc_left() << std::endl;
#endif

  for (const auto& moving_close_traj : moving_close_trajs) {
#if DEBUG_NEAREST_CLOSE
    std::cout << "-----------------" << std::endl;
    std::cout << " obj_id: " << moving_close_traj.object_id()
              << " st_nearest_points_size: "
              << moving_close_traj.st_nearest_points().size() << std::endl;
#endif
    if (moving_close_traj.st_nearest_points().size() == 0) continue;

    const PlannerObject* obj =
        traj_mgr.FindObjectByObjectId(moving_close_traj.object_id());

#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id() << " ]:"
              << " object_vel " << Mps2Kph(obj->pose().v()) << "km/h "
              << " av_vel " << Mps2Kph(av_speed) << "km/h "
              << " delta_vel " << Mps2Kph(obj->pose().v()) - Mps2Kph(av_speed)
              << "km/h " << std::endl;
#endif

    if ((obj->pose().v() - av_speed) > kDeltaVel) continue;
    double ds = std::numeric_limits<double>::max();
    double dl = std::numeric_limits<double>::max();
    double delta_theta = 0.0;
    if (nullptr != obj && 0 < path_points.size()) {
      const Vec2d ego_dir_unit =
          Vec2d::FastUnitFromAngle((path_points)[0].theta());
      bool is_back_obj = true;
      for (const auto& corner_point : obj->contour().points()) {
        if (ego_dir_unit.Dot(corner_point - ToVec2d((path_points).front())) >
            0.0) {
          is_back_obj = false;
          break;
        }
      }
      delta_theta = std::fabs(ad_byd::planning::math::AngleDiff(
          (path_points)[0].theta(), obj->pose().theta()));

#if DEBUG_NEAREST_CLOSE
      std::cout << "[ " << moving_close_traj.object_id() << " ]:"
                << " ego_theta "
                << ad_byd::planning::Constants::RAD2DEG *
                       (path_points)[0].theta()
                << " obj_theta "
                << ad_byd::planning::Constants::RAD2DEG * obj->pose().theta()
                << " delta_theta "
                << ad_byd::planning::Constants::RAD2DEG * delta_theta
                << " is_back_obj " << is_back_obj << std::endl;
#endif
      if (delta_theta > kMaxDeltaTheta) continue;
      if (is_back_obj) continue;
      FrenetPolygon obj_frenet_polygon;
      obj_frenet_polygon =
          path_points.XYToSL(obj->contour(), obj->pose().pos());
      ds = std::max(
          obj_frenet_polygon.s_min - vehicle_geo_params.front_edge_to_center(),
          0.0);
      const double ego_half_width = vehicle_geo_params.width() * 0.5;
      if (obj_frenet_polygon.l_min > ego_half_width) {
        dl = obj_frenet_polygon.l_min - ego_half_width;
      } else if (obj_frenet_polygon.l_max < (-1.0 * ego_half_width)) {
        dl = obj_frenet_polygon.l_max + ego_half_width;
      } else {
        dl = 0.0;
      }

      double ego_obj_cross_prod =
          Vec2d::FastUnitFromAngle((path_points)[0].theta())
              .CrossProd(Vec2d::FastUnitFromAngle(obj->pose().theta()));
      bool is_obj_moving_faraway = false;
      if (dl > 0) {
        if (ego_obj_cross_prod > 0) {
          is_obj_moving_faraway = true;
        }
      } else if (dl < 0) {
        if (ego_obj_cross_prod < 0) {
          is_obj_moving_faraway = true;
        }
      }
#if DEBUG_NEAREST_CLOSE
      std::cout << "[ " << moving_close_traj.object_id() << " ]:"
                << " dl " << dl << " ego_obj_cross_prod " << ego_obj_cross_prod
                << " is_obj_moving_faraway " << is_obj_moving_faraway
                << std::endl;
#endif
      double deg_threshold_car = ad_byd::planning::Constants::DEG2RAD * 2.0;
      double deg_threshold_vru = ad_byd::planning::Constants::DEG2RAD * 5.0;
      double deg_threshold = deg_threshold_vru;
      if (StBoundaryProto::VEHICLE == moving_close_traj.object_type()) {
        deg_threshold = deg_threshold_car;
      }
      if (is_obj_moving_faraway && (delta_theta >= deg_threshold)) continue;
      dl = std::fabs(dl);
      double relative_v = std::fabs(obj->pose().v() - av_speed);
      double ttc = ds / (relative_v + kEps);

#if DEBUG_NEAREST_CLOSE
      std::cout << "[ " << moving_close_traj.object_id() << " ]:"
                << " ds " << ds << " relative_v " << relative_v << " ttc "
                << ttc << " min_l " << obj_frenet_polygon.l_min << " max_l "
                << obj_frenet_polygon.l_max << std::endl;
#endif
      if ((av_speed - obj->pose().v()) <= kDeltaVel) {
        if (ttc > kTtcThresholdSmallDeltaV) {
          continue;
        }
      } else {
        if (ttc > kTtcThreshold) {
          continue;
        }
      }

      if (obj_frenet_polygon.s_max <
          vehicle_geo_params.front_edge_to_center()) {
        continue;
      }

      if (LaneChangeStage::LCS_NONE != lane_change_state.stage()) {
        if ((lane_change_state.lc_left() && obj_frenet_polygon.l_min < 0) ||
            (!lane_change_state.lc_left() && obj_frenet_polygon.l_max > 0)) {
          continue;
        }
      }
    }

    double start_s = moving_close_traj.st_nearest_points().front().s;
    double end_s;

    if (moving_close_traj.st_nearest_points().size() == 1) {
      end_s = start_s + kMinDeltaDist;
    } else {
      end_s = std::max(moving_close_traj.st_nearest_points().back().s,
                       start_s + kMinDeltaDist);
    }

    double speed_limit_v = moving_close_traj.st_nearest_points().front().v;
    double lat_dist = std::numeric_limits<double>::max();
    double lat_time = std::numeric_limits<double>::max();
    for (const auto& single_nearest_point :
         moving_close_traj.st_nearest_points()) {
      lat_dist = std::min(std::fabs(single_nearest_point.lat_dist), lat_dist);
      lat_time = std::min(std::fabs(single_nearest_point.lat_t), lat_time);
    }
    if (std::numeric_limits<double>::max() == dl) {
      dl = lat_dist;
    }
    lat_dist = lat_dist + 0.1;
    lat_dist = std::min(lat_dist, dl);
#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id() << " ]:"
              << " lat_dist " << lat_dist << " dl " << dl << " lat_time "
              << lat_time << std::endl;
    if (nullptr != nudge_object_info) {
      std::cout << " nudge_object_info->id " << nudge_object_info->id
                << std::endl;
    }
#endif
    double cyclist_dist_lat_threshold = kCyclistLatDistThreshold;
    if (nullptr != nudge_object_info &&
        moving_close_traj.object_id() == nudge_object_info->id &&
        StBoundaryProto::CYCLIST == moving_close_traj.object_type()) {
      cyclist_dist_lat_threshold = kNudgeCyclistLatDistThreshold;
      if ((av_speed - obj->pose().v()) <= kDeltaVel) continue;
    }

    if (StBoundaryProto::CYCLIST == moving_close_traj.object_type()) {
      if (lat_dist > cyclist_dist_lat_threshold) continue;
      if (std::fabs(dl) > cyclist_dist_lat_threshold &&
          delta_theta < kMinDeltaTheta) {
        continue;
      }
    } else if (StBoundaryProto::VEHICLE == moving_close_traj.object_type()) {
      if (obj->is_large_vehicle() && is_narrow_near_large_vehicle) {
        if (std::fabs(dl) > 0.55) continue;
      } else {
        if (lat_dist > 1.5) continue;
        if (lat_time > 3.0) continue;
        if (std::fabs(dl) > 0.4 && delta_theta < kMinDeltaTheta) continue;
      }

    } else if (StBoundaryProto::PEDESTRIAN == moving_close_traj.object_type()) {
      if (lat_dist > 0.4) continue;
    } else {
      continue;
    }

#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id() << " ]:"
              << " speed_limit_init " << Mps2Kph(speed_limit_v) << "km/h"
              << std::endl;
#endif
    speed_limit_v += kLatNearestCloseRelVPlf(lat_dist);
#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id() << " ]:"
              << " speed_limit_lat " << Mps2Kph(speed_limit_v) << "km/h"
              << std::endl;
#endif
    speed_limit_v += kLonNearestCloseRelVPlf(ds);
#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id() << " ]:"
              << " speed_limit_lon " << Mps2Kph(speed_limit_v) << "km/h"
              << std::endl;
#endif

    std::string info = absl::StrFormat("nearest_close_speed_limit_%d",
                                       moving_close_traj.object_id());
#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id() << " ]:"
              << " lat_dist " << lat_dist << " object_v "
              << Mps2Kph(moving_close_traj.st_nearest_points().front().v)
              << " speed_limit_v " << Mps2Kph(speed_limit_v) << " start_s "
              << start_s << " end_s " << end_s << std::endl;
#endif

    if (speed_limit_v > av_speed) continue;
    if (start_s >= kMaxDist) continue;

    if (StBoundaryProto::VEHICLE == moving_close_traj.object_type()) {
      if (speed_limit_v > moving_close_traj.st_nearest_points().front().v) {
        continue;
      }
    }

    speed_limit_v = std::max(kMinVelocity, speed_limit_v);

    if (StBoundaryProto::PEDESTRIAN == moving_close_traj.object_type()) {
      speed_limit_v = std::max(speed_limit_v, av_speed * 0.8);
    }

    if (start_s > end_s) continue;

    double comfortable_acc;

    if (StBoundaryProto::CYCLIST == moving_close_traj.object_type()) {
      comfortable_acc = kCyclistComfortableAcc;
    } else {
      comfortable_acc = kVehicleComfortableAcc;
    }

    double max_dec_vel = std::sqrt(-2 * comfortable_acc * start_s);
    double comfortable_expect_vel;
    if (av_speed < max_dec_vel) {
      comfortable_expect_vel = 0.0;
    } else {
      comfortable_expect_vel =
          std::sqrt(Sqr(av_speed) + 2 * comfortable_acc * start_s);
    }
    comfortable_expect_vel = std::max(comfortable_expect_vel, speed_limit_v);

#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id()
              << " ]: comfortable_expect_vel "
              << Mps2Kph(comfortable_expect_vel) << "km/h"
              << " dl " << dl << std::endl;
#endif
    double tmp_lat_dist = std::min(std::max(dl, 0.0), 0.5);
    double speed_limit_lat_dist =
        (comfortable_expect_vel - speed_limit_v) / 0.5 * tmp_lat_dist +
        speed_limit_v;

    speed_limit_lat_dist = std::max(speed_limit_lat_dist, speed_limit_v);

#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id()
              << " ]: speed_limit_lat_dist " << Mps2Kph(speed_limit_lat_dist)
              << "km/h" << std::endl;
#endif

    double tmp_lat_time = std::min(std::max(lat_time, 0.0), 2.0);
    double speed_limit_lat_time;
    if (tmp_lat_time >= 0.0 && tmp_lat_time <= 0.2) {
      speed_limit_lat_time = speed_limit_v;
    } else if (tmp_lat_time < 1.4) {
      speed_limit_lat_time = comfortable_expect_vel * 0.7 + speed_limit_v * 0.3;
    } else {
      speed_limit_lat_time = comfortable_expect_vel;
    }

    speed_limit_lat_time = std::max(speed_limit_lat_time, speed_limit_v);

#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id()
              << " ]: speed_limit_lat_time " << Mps2Kph(speed_limit_lat_time)
              << "km/h" << std::endl;
#endif
    speed_limit_v = std::min(speed_limit_lat_time, speed_limit_lat_dist);

    max_dec_vel = std::sqrt(-2 * kMaxBrakeAcc * start_s);
    double max_brake_expect_vel;
    if (av_speed < max_dec_vel) {
      max_brake_expect_vel = 0.0;
    } else {
      max_brake_expect_vel =
          std::sqrt(Sqr(av_speed) + 2 * kMaxBrakeAcc * start_s);
    }
    if (ds > 5.0) {
      speed_limit_v = std::max(speed_limit_v, max_brake_expect_vel);
    }

#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id()
              << " ]: max_brake_expect_vel " << Mps2Kph(max_brake_expect_vel)
              << "km/h" << std::endl;
#endif

    if (speed_limit_v > av_speed) continue;
    constexpr double kLargeVehicleSpeedBuffer = 0.5;
    if (obj->is_large_vehicle() && is_narrow_near_large_vehicle) {
      speed_limit_v = obj->pose().v() - kLargeVehicleSpeedBuffer;
    }

#if DEBUG_NEAREST_CLOSE
    std::cout << "[ " << moving_close_traj.object_id()
              << " ]: generate_speed_limit " << Mps2Kph(speed_limit_v) << "km/h"
              << std::endl;
#endif
    speed_limit_ranges.push_back(
        {.start_s = start_s,
         .end_s = end_s,
         .speed_limit = std::min(speed_limit_v, max_speed_limit),
         .info = info});
  }

  if (speed_limit_ranges.empty()) return std::nullopt;

  return SpeedLimit(speed_limit_ranges);
}

std::map<SpeedLimitTypeProto::Type, SpeedLimit> GetSpeedLimitMap(
    const DiscretizedPath& discretized_points,
    const std::vector<PathPoint>& st_path_points,
    const PathSlBoundary& path_sl_boundary, double max_speed_limit,
    double av_speed, const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const DrivePassage& drive_passage, const ConstraintManager& constraint_mgr,
    const SpeedLimitProto& speed_limit_config,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries,
    const PlannerSemanticMapManager* planner_semantic_map_manager,
    double av_max_acc, const Behavior& behavior,
    const std::vector<StCloseTrajectory>& moving_close_trajs,
    const SpacetimeTrajectoryManager& traj_mgr,
    const LaneChangeStateProto lane_change_state,
    const NudgeObjectInfo* nudge_object_info, const EgoHistory* ego_history,
    EgoFrame* curr_ego_frame, double is_narrow_near_large_vehicle,
    bool raise_lane_speed_limit, double spdlimit_curvature_gain_prev,
    double* spdlimit_curvature_gain_ptr,
    std::optional<double>* v2_trigger_distance) {
  // ("GetSpeedLimitMap");
  const auto& map_func_id = behavior.function_id();
  std::vector<DrivingTopoRoad> driving_topo_road_seq =
      SeparateAvDrivingTopoRoad(planner_semantic_map_manager, drive_passage,
                                discretized_points[0]);
  const auto& lc_stage = lane_change_state.stage();
  std::map<SpeedLimitTypeProto::Type, SpeedLimit> speed_limit_map;
  SpeedLimit lane_speed_limit =
      GenerateLaneSpeedLimit(discretized_points, max_speed_limit, av_speed,
                             drive_passage, lc_stage, raise_lane_speed_limit);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_LANE,
                          std::move(lane_speed_limit));

  SpeedLimit curvature_speed_limit = GenerateCurvatureSpeedLimit(
      discretized_points, veh_drive_params, veh_geo_params, max_speed_limit,
      av_speed, speed_limit_config, planner_semantic_map_manager,
      driving_topo_road_seq, spdlimit_curvature_gain_prev,
      spdlimit_curvature_gain_ptr);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_CURVATURE,
                          std::move(curvature_speed_limit));

  if (speed_limit_config.enable_steer_rate_limit()) {
    SpeedLimit steer_rate_speed_limit = GenerateSteerRateSpeedLimit(
        st_path_points, veh_drive_params, veh_geo_params, max_speed_limit,
        speed_limit_config);
    speed_limit_map.emplace(SpeedLimitTypeProto_Type_STEER_RATE,
                            std::move(steer_rate_speed_limit));
  }

  constexpr double min_speed_limit = Kph2Mps(15.0);
  constexpr double corss_curb_max_dec = 1.5;

  auto corss_curb_speed_limit = GenerateCrossCurbSpeedLimit(
      discretized_points, max_speed_limit,
      distance_info_to_impassable_path_boundaries, min_speed_limit, av_speed,
      corss_curb_max_dec);
  if (corss_curb_speed_limit.has_value()) {
    speed_limit_map.emplace(SpeedLimitTypeProto_Type_CROSS_CURB,
                            std::move(*corss_curb_speed_limit));
  }

  auto close_curb_speed_limit = GenerateCloseCurbSpeedLimit(
      discretized_points, max_speed_limit,
      distance_info_to_impassable_path_boundaries,
      PiecewiseLinearFunctionFromProto(
          speed_limit_config.hard_curb_clearance_rel_speed_plf()),
      av_speed, speed_limit_config.close_curb_max_dec());
  if (close_curb_speed_limit.has_value()) {
    speed_limit_map.emplace(SpeedLimitTypeProto_Type_CLOSE_CURB,
                            std::move(*close_curb_speed_limit));
  }

  auto approach_curb_speed_limit = GenerateApproachCurbSpeedLimit(
      discretized_points, distance_info_to_impassable_path_boundaries,
      speed_limit_config, max_speed_limit, av_speed,
      speed_limit_config.approach_curb_max_dec(), lane_change_state);
  if (approach_curb_speed_limit.has_value()) {
    speed_limit_map.emplace(SpeedLimitTypeProto_Type_APPROACH_CURB,
                            std::move(*approach_curb_speed_limit));
  }

  if (auto external_speed_limit = GenerateExternalSpeedLimit(
          discretized_points, constraint_mgr, veh_geo_params, max_speed_limit);
      external_speed_limit.has_value()) {
    speed_limit_map.emplace(SpeedLimitTypeProto_Type_EXTERNAL,
                            std::move(*external_speed_limit));
  }

  auto toll_speed_limit = GenerateTollSpeedLimit(
      discretized_points, veh_geo_params, max_speed_limit,
      planner_semantic_map_manager, av_max_acc, av_speed);
  if (toll_speed_limit.has_value()) {
    speed_limit_map.emplace(SpeedLimitTypeProto_Type_TOLL_SPEED_LIMIT,
                            std::move(*toll_speed_limit));
  }
  if (map_func_id == Behavior_FunctionId_LKA) {
    const auto& sub_type = planner_semantic_map_manager->map_ptr()->sub_type();
    if (sub_type == ad_byd::planning::BIG_JUNCTION) {
      // generate BIG_JUNCTION speed limit
      auto map_sub_type_speed_limit = GenerateBigJunctionSpeedLimit(
          discretized_points, speed_limit_config, av_speed);
      if (map_sub_type_speed_limit.has_value()) {
        speed_limit_map.emplace(SpeedLimitTypeProto_Type_BIG_JUNCTION,
                                std::move(*map_sub_type_speed_limit));
      }
    } else if (sub_type == ad_byd::planning::PERCEPTION_LOSS) {
      // TODO: generate PERCEPTION_LOSS speed limit
      auto map_sub_type_speed_limit = GeneratePerceptionLossSpeedLimit();
      if (map_sub_type_speed_limit.has_value()) {
        speed_limit_map.emplace(SpeedLimitTypeProto_Type_PERCEPTION_LOSS,
                                std::move(*map_sub_type_speed_limit));
      }
    } else if (sub_type == ad_byd::planning::IN_JUNCTION_T_MAP) {
      // TODO: generate IN_JUNCTION_T_MAP speed limit
      auto map_sub_type_speed_limit = GenerateInJunctionTMapSpeedLimit();
      if (map_sub_type_speed_limit.has_value()) {
        speed_limit_map.emplace(SpeedLimitTypeProto_Type_IN_JUNCTION_T_MAP,
                                std::move(*map_sub_type_speed_limit));
      }
    }
  } else if (map_func_id == Behavior_FunctionId_MAPLESS_NOA) {
    const auto& sub_type = planner_semantic_map_manager->map_ptr()->sub_type();
    if (sub_type == ad_byd::planning::IN_JUNCTION_T_MAP ||
        sub_type == ad_byd::planning::JUNCTION_T_MAP) {
      auto t_junction_speed_limit = GenerateTJunctionMapSpeedLimit(
          discretized_points, max_speed_limit, av_speed, speed_limit_config,
          planner_semantic_map_manager, driving_topo_road_seq);
      if (t_junction_speed_limit.has_value()) {
        speed_limit_map.emplace(SpeedLimitTypeProto_Type_IN_JUNCTION_T_MAP,
                                std::move(*t_junction_speed_limit));
      }
    }
  }

  auto v2_turn_type_speed_limit = GenerateV2DetailTurnTypeSpeedLimit(
      discretized_points, planner_semantic_map_manager, drive_passage,
      max_speed_limit, av_speed, v2_trigger_distance);
  if (v2_turn_type_speed_limit.has_value()) {
    speed_limit_map.emplace(SpeedLimitTypeProto_Type_V2_TURN_TYPE,
                            std::move(*v2_turn_type_speed_limit));
  }

  SpeedLimit combination_speed_limit =
      GenerateCombinationSpeedLimit(speed_limit_map, max_speed_limit);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_COMBINATION,
                          std::move(combination_speed_limit));

  if (speed_limit_config.enable_nearest_close_limit()) {
    auto nearest_close_speed_limit = GenerateNearestCloseSpeedLimit(
        moving_close_trajs, max_speed_limit, av_speed, discretized_points,
        traj_mgr, veh_geo_params, lane_change_state, nudge_object_info,
        is_narrow_near_large_vehicle);
    if (nearest_close_speed_limit.has_value()) {
      speed_limit_map.emplace(SpeedLimitTypeProto_Type_NEAREST_CLOSE,
                              std::move(*nearest_close_speed_limit));
    }
  }

  return speed_limit_map;
}

VtSpeedLimit GetExternalVtSpeedLimit(const ConstraintManager& constraint_mgr,
                                     int traj_steps, double time_step) {
  VtSpeedLimit vt_speed_limit;
  const auto& vt_speed_profiles = constraint_mgr.SpeedProfiles();
  if (vt_speed_profiles.empty()) return vt_speed_limit;
  vt_speed_limit.reserve(traj_steps + 1);
  // Convert speed profiles to map of piecewise linear func.
  std::map<std::string, PiecewiseLinearFunction<double>> vt_speed_map;
  for (const auto& speed_profile : vt_speed_profiles) {
    vt_speed_map.emplace(
        TypeCaseToString(speed_profile.source().type_case()),
        PiecewiseLinearFunctionFromProto(speed_profile.vt_upper_constraint()));
  }
  for (double t = 0.0; t < traj_steps * kTrajectoryTimeStep; t += time_step) {
    double min_speed_upper_bound = std::numeric_limits<double>::max();
    std::string min_type;
    for (const auto& [type, speed_profile] : vt_speed_map) {
      if (speed_profile(t) < min_speed_upper_bound) {
        min_speed_upper_bound = speed_profile(t);
        min_type = type;
      }
    }
    vt_speed_limit.emplace_back(min_speed_upper_bound, std::move(min_type));
  }
  return vt_speed_limit;
}

}  // namespace st::planning
