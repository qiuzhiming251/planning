

#include "planner/speed_optimizer/decider/close_object_slowdown_decider.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "plan_common/log_data.h"
#include "plan_common/log.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "planner/speed_optimizer/speed_finder_flags.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/path_util.h"

namespace st {
namespace planning {

namespace {

// TODO: Move to params file.
constexpr double kStationarySStartLength = 10.0;  // m
constexpr double kStationarySEndLength = 3.0;     // m
constexpr double kStationaryMaxTime = 5.0;

constexpr double kMovingSStartLength = 10.0;  // m
constexpr double kMovingSEndLength = 3.0;     // m

constexpr double kMaxDecelByCloseObject = -0.8;  // m/s^2.

const std::vector<double> kCloseObjectDistanceRange = {0.0, 0.5, 1.0, 1.5,
                                                       2.0};  // m

const std::vector<double> kCloseObjectDistanceRangeHighway = {0.0, 0.3, 0.6,
                                                              0.9, 1.2};  // m

const std::vector<double> kStationaryUnknownObjectMaxSpeed = {
    5.0, 8.0, 12.0, 14.0, 18.0};  // m/s
const std::vector<double> kStationaryVehicleMaxSpeed = {5.0, 8.0, 12.0, 14.0,
                                                        18.0};  // m/s
const std::vector<double> kStationaryCyclistMaxSpeed = {2.0, 3.0, 4.0, 6.0,
                                                        8.0};  // m/s
const std::vector<double> kStationaryPedestrianMaxSpeed = {2.0, 3.0, 4.0, 6.0,
                                                           8.0};  // m/s
const std::vector<double> kStationaryStaticMaxSpeed = {10.0, 15.0, 20.0, 25.0,
                                                       30.0};  // m/s

// NOTE: Consider the case when change line and the sl boundary is wide.
const std::vector<double> kStationaryInsideSlBoundaryUnknownObjectMaxSpeed = {
    4.0, 6.0, 8.0, 10.0, 15.0};  // m/s
const std::vector<double> kStationaryInsideSlBoundaryVehicleMaxSpeed = {
    4.0, 6.0, 8.0, 10.0, 15.0};  // m/s
const std::vector<double> kStationaryInsideSlBoundaryCyclistMaxSpeed = {
    2.0, 3.0, 4.0, 6.0, 8.0};  // m/s
const std::vector<double> kStationaryInsideSlBoundaryPedestrianMaxSpeed = {
    2.0, 3.0, 4.0, 6.0, 8.0};  // m/s
// Copy to AddAggregateStaticObjectCost in trajectory_optizmier, please modify
// at the same time.
const std::vector<double> kStationaryInsideSlBoundaryStaticMaxSpeed = {
    10.0, 15.0, 20.0, 25.0, 30.0};  // m/s

const std::vector<double> kMovingUnknownObjectMaxSpeed = {4.0, 8.0, 12.0, 14.0,
                                                          18.0};  // m/s
const std::vector<double> kMovingVehicleMaxSpeed = {4.0, 8.0, 12.0, 14.0,
                                                    18.0};  // m/s
const std::vector<double> kMovingCyclistMaxSpeed = {3.0, 4.0, 6.0, 8.0,
                                                    10.0};  // m/s
const std::vector<double> kMovingPedestrianMaxSpeed = {3.0, 4.0, 6.0, 8.0,
                                                       10.0};  // m/s
const std::vector<double> kMovingStaticMaxSpeed = {4.0, 10.0, 15.0, 20.0,
                                                   25.0};  // m/s

const std::vector<double> kMovingAwayUnknownObjectMaxSpeed = {
    8.0, 16.0, 24.0, 28.0, 36.0};  // m/s
const std::vector<double> kMovingAwayVehicleMaxSpeed = {8.0, 16.0, 24.0, 28.0,
                                                        36.0};  // m/s
const std::vector<double> kMovingAwayCyclistMaxSpeed = {6.0, 8.0, 12.0, 16.0,
                                                        20.0};  // m/s
const std::vector<double> kMovingAwayPedestrianMaxSpeed = {6.0, 8.0, 12.0, 16.0,
                                                           20.0};  // m/s
const std::vector<double> kMovingAwayStaticMaxSpeed = {4.0, 10.0, 15.0, 20.0,
                                                       25.0};  // m/s

std::optional<double> DecideStationaryMaxSpeed(
    StBoundaryProto::ObjectType object_type,
    const StDistancePoint& st_distance_point, const bool is_on_highway) {
  static const auto kMap = absl::flat_hash_map<
      StBoundaryProto::ObjectType, PiecewiseLinearFunction<double, double>>{
      {StBoundaryProto::UNKNOWN_OBJECT,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryUnknownObjectMaxSpeed)},
      {StBoundaryProto::VEHICLE,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryVehicleMaxSpeed)},
      {StBoundaryProto::CYCLIST,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryCyclistMaxSpeed)},
      {StBoundaryProto::PEDESTRIAN,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryPedestrianMaxSpeed)},
      {StBoundaryProto::STATIC,
       PiecewiseLinearFunction(is_on_highway ? kCloseObjectDistanceRangeHighway
                                             : kCloseObjectDistanceRange,
                               kStationaryStaticMaxSpeed)}};
  const auto* res = FindOrNull(kMap, object_type);
  if (res != nullptr) {
    return res->Evaluate(st_distance_point.distance);
  } else {
    return std::nullopt;
  }
}

std::optional<double> DecideStationaryInsideSlBoundaryMaxSpeed(
    StBoundaryProto::ObjectType object_type,
    const StDistancePoint& st_distance_point) {
  static const auto kMap = absl::flat_hash_map<
      StBoundaryProto::ObjectType, PiecewiseLinearFunction<double, double>>{
      {StBoundaryProto::UNKNOWN_OBJECT,
       PiecewiseLinearFunction(
           kCloseObjectDistanceRange,
           kStationaryInsideSlBoundaryUnknownObjectMaxSpeed)},
      {StBoundaryProto::VEHICLE,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryInsideSlBoundaryVehicleMaxSpeed)},
      {StBoundaryProto::CYCLIST,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryInsideSlBoundaryCyclistMaxSpeed)},
      {StBoundaryProto::PEDESTRIAN,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryInsideSlBoundaryPedestrianMaxSpeed)},
      {StBoundaryProto::STATIC,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryInsideSlBoundaryStaticMaxSpeed)}};
  const auto* res = FindOrNull(kMap, object_type);
  if (res != nullptr) {
    return res->Evaluate(st_distance_point.distance);
  } else {
    return std::nullopt;
  }
}

std::optional<double> DecideMovingMaxSpeed(
    StBoundaryProto::ObjectType object_type,
    const StDistancePoint& st_distance_point) {
  static const auto kMap =
      absl::flat_hash_map<StBoundaryProto::ObjectType,
                          PiecewiseLinearFunction<double, double>>{
          {StBoundaryProto::UNKNOWN_OBJECT,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingUnknownObjectMaxSpeed)},
          {StBoundaryProto::VEHICLE,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingVehicleMaxSpeed)},
          {StBoundaryProto::CYCLIST,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingCyclistMaxSpeed)},
          {StBoundaryProto::PEDESTRIAN,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingPedestrianMaxSpeed)},
          {StBoundaryProto::STATIC,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingStaticMaxSpeed)}};
  const auto* res = FindOrNull(kMap, object_type);
  if (res != nullptr) {
    return res->Evaluate(st_distance_point.distance) +
           std::max(st_distance_point.relative_v, 0.0);
  } else {
    return std::nullopt;
  }
}

std::optional<double> DecideMovingAwayMaxSpeed(
    StBoundaryProto::ObjectType object_type,
    const StDistancePoint& st_distance_point) {
  static const auto kMap =
      absl::flat_hash_map<StBoundaryProto::ObjectType,
                          PiecewiseLinearFunction<double, double>>{
          {StBoundaryProto::UNKNOWN_OBJECT,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingAwayUnknownObjectMaxSpeed)},
          {StBoundaryProto::VEHICLE,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingAwayVehicleMaxSpeed)},
          {StBoundaryProto::CYCLIST,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingAwayCyclistMaxSpeed)},
          {StBoundaryProto::PEDESTRIAN,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingAwayPedestrianMaxSpeed)},
          {StBoundaryProto::STATIC,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kMovingAwayStaticMaxSpeed)}};
  const auto* res = FindOrNull(kMap, object_type);
  if (res != nullptr) {
    return res->Evaluate(st_distance_point.distance) +
           st_distance_point.relative_v;
  } else {
    return std::nullopt;
  }
}

void ComputeStationaryPathS(const StDistancePoint& st_distance_point,
                            double max_s, double* start_path_s,
                            double* end_path_s) {
  *start_path_s =
      std::max(st_distance_point.path_s - kStationarySStartLength, 0.0);
  *end_path_s =
      std::min(st_distance_point.path_s + kStationarySEndLength, max_s);
}

void ComputeMovingPathS(const StDistancePoint& st_distance_point, double max_s,
                        double* start_path_s, double* end_path_s) {
  *start_path_s = std::max(st_distance_point.path_s - kMovingSStartLength, 0.0);
  *end_path_s = std::min(st_distance_point.path_s + kMovingSEndLength, max_s);
}

bool CheckInsidePathSlBoundary(const Polygon2d& contour,
                               const PathSlBoundary& path_sl_boundary,
                               const DrivePassage& drive_passage) {
  const auto fbox = drive_passage.QueryFrenetBoxAtContour(contour);
  if (fbox.ok()) {
    const auto [right_l_s_min, left_l_s_min] =
        path_sl_boundary.QueryBoundaryL(fbox->s_min);
    VLOG(2) << "fbox_l_min: " << fbox->l_min << ", fbox_l_max: " << fbox->l_max
            << ", right_l_s_min: " << right_l_s_min
            << ", left_l_s_min: " << left_l_s_min;
    if ((right_l_s_min <= fbox->l_min && fbox->l_min <= left_l_s_min) ||
        (right_l_s_min <= fbox->l_max && fbox->l_max <= left_l_s_min)) {
      return true;
    }
    const auto [right_l_s_max, left_l_s_max] =
        path_sl_boundary.QueryBoundaryL(fbox->s_max);
    VLOG(2) << "right_l_s_max: " << right_l_s_max
            << ", left_l_s_max: " << left_l_s_max;
    return (right_l_s_max <= fbox->l_min && fbox->l_min <= left_l_s_max) ||
           (right_l_s_max <= fbox->l_max && fbox->l_max <= left_l_s_max);
  } else {
    VLOG(2) << "Failed to project to frenet for contour: "
            << contour.DebugString();
    return false;
  }
  // Should never be here.
  return false;
}

std::optional<double> DecideStationarySoftAccMaxSpeed(
    const CloseSpaceTimeObject& close_space_time_object,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const DrivePassage& drive_passage, const PathPoint& base_point,
    double av_speed, double end_path_s) {
  const double kCloseLatDistThres = 1.0;
  const double kCloseLonDistThres = 8.0;
  bool lon_direction_close = false;
  Vec2d start_point(base_point.x(), base_point.y());
  const auto& st_distance_point = close_space_time_object.st_distance_points[0];
  const auto ego_fpos = drive_passage.QueryFrenetCoordinateAt(start_point);
  const auto obj_fbox =
      drive_passage.QueryFrenetBoxAtContour(close_space_time_object.contour);
  if (obj_fbox.ok() && ego_fpos.ok()) {
    double ds = obj_fbox->s_min - ego_fpos->s -
                vehicle_geometry_params.front_edge_to_center();
    lon_direction_close = st_distance_point.distance < kCloseLatDistThres &&
                          ds > -kMathEpsilon && ds < kCloseLonDistThres;
  }
  if (close_space_time_object.is_stationary && lon_direction_close) {
    const std::vector<double> kDistanceRange = {0.5, 0.8};  // m
    const std::vector<double> KSoftAcc = {1.2, 1.5};        // m/ss
    const auto& soft_acc_plf =
        PiecewiseLinearFunction(kDistanceRange, KSoftAcc);
    return std::sqrt(Sqr(av_speed) +
                     2.0 * end_path_s *
                         soft_acc_plf(st_distance_point.distance));
  }
  return std::nullopt;
}

}  // namespace

std::vector<ConstraintProto::PathSpeedRegionProto>
MakeCloseObjectSlowdownDecision(
    const std::vector<CloseSpaceTimeObject>& close_space_time_objects,
    const DrivePassage& drive_passage, const DiscretizedPath& path_points,
    double av_speed, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const bool is_on_highway) {
  std::vector<ConstraintProto::PathSpeedRegionProto> res;

  for (auto close_space_time_object = close_space_time_objects.begin();
       close_space_time_object != close_space_time_objects.end();
       ++close_space_time_object) {
    CHECK_EQ(close_space_time_object->st_distance_points.size(), 1);
    const auto& st_distance_point =
        close_space_time_object->st_distance_points[0];

    VLOG(3) << "------- ID: " << close_space_time_object->id << " ---------";

    std::optional<double> max_speed;
    double start_path_s = 0.0;
    double end_path_s = 0.0;

    if (close_space_time_object->is_stationary) {
      const auto obj_type = close_space_time_object->object_type;
      if (FLAGS_planner_enable_static_object_close_speed_limit &&
          obj_type == StBoundaryProto::STATIC)
        continue;
      const bool inside_path_sl_boundary = CheckInsidePathSlBoundary(
          close_space_time_object->contour, path_sl_boundary, drive_passage);
      if (st_distance_point.path_s / std::max(av_speed, 1e-6) >
          kStationaryMaxTime)
        continue;
      if (inside_path_sl_boundary) {
        max_speed = DecideStationaryInsideSlBoundaryMaxSpeed(
            close_space_time_object->object_type, st_distance_point);
      } else {
        max_speed =
            DecideStationaryMaxSpeed(close_space_time_object->object_type,
                                     st_distance_point, is_on_highway);
      }
      VLOG(3) << "Inside_sl_boundary: "
              << (inside_path_sl_boundary ? "true" : "false");

      if (!max_speed.has_value()) continue;
      ComputeStationaryPathS(st_distance_point, path_points.length(),
                             &start_path_s, &end_path_s);
    } else {
      if (close_space_time_object->is_away_from_traj) {
        max_speed = DecideMovingAwayMaxSpeed(
            close_space_time_object->object_type, st_distance_point);
      } else {
        max_speed = DecideMovingMaxSpeed(close_space_time_object->object_type,
                                         st_distance_point);
      }
      if (!max_speed.has_value()) continue;
      ComputeMovingPathS(st_distance_point, path_points.length(), &start_path_s,
                         &end_path_s);
    }

    VLOG(3) << "Stationary: "
            << (close_space_time_object->is_stationary ? "true" : "false")
            << ", away from traj: "
            << (close_space_time_object->is_away_from_traj ? "true" : "false")
            << ", start_path_s: " << start_path_s
            << ", end_path_s: " << end_path_s
            << ", path_s: " << st_distance_point.path_s
            << ", relative_v: " << st_distance_point.relative_v
            << ", max_speed: " << *max_speed
            << ", distance: " << st_distance_point.distance
            << ", type: " << close_space_time_object->object_type;

    const auto start_point = ToVec2d(path_points.Evaluate(start_path_s));
    const auto end_point = ToVec2d(path_points.Evaluate(end_path_s));

    // Avoid hard brake caused by close object speed limit.
    const double decel =
        (Sqr(*max_speed) - Sqr(av_speed)) / ((2.0 * start_path_s) + 1e-3);
    if (decel < kMaxDecelByCloseObject) {
      max_speed = std::sqrt(std::max(
          0.0, 2.0 * kMaxDecelByCloseObject * start_path_s + Sqr(av_speed)));
    }
    // Avoid dash before pass close object
    std::optional<double> soft_acc_speed = DecideStationarySoftAccMaxSpeed(
        *close_space_time_object, vehicle_geometry_params, drive_passage,
        path_points.front(), av_speed, end_path_s);
    if (soft_acc_speed.has_value()) {
      Log2DDS::LogDataV0(
          "soft_acc_speed ",
          absl::StrCat(close_space_time_object->id, ": ", *soft_acc_speed));
      max_speed = std::fmin(*max_speed, *soft_acc_speed);
    }

    ConstraintProto::PathSpeedRegionProto close_object_speed_region;
    close_object_speed_region.set_start_s(start_path_s);
    close_object_speed_region.set_end_s(end_path_s);
    start_point.ToProto(close_object_speed_region.mutable_start_point());
    end_point.ToProto(close_object_speed_region.mutable_end_point());
    close_object_speed_region.set_max_speed(*max_speed);
    close_object_speed_region.set_id(close_space_time_object->id);
    close_object_speed_region.mutable_source()->mutable_close_object()->set_id(
        close_space_time_object->id);
    res.emplace_back(std::move(close_object_speed_region));
  }

  return res;
}

}  // namespace planning
}  // namespace st
