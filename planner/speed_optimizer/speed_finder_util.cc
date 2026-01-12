

#include "planner/speed_optimizer/speed_finder_util.h"

#include <algorithm>
#include <limits>
#include <ostream>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "plan_common/log_data.h"
#include "plan_common/util/vehicle_geometry_util.h"

// #include "global/trace.h"
// #include "lite/check.h"
// #include "lite/logging.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/vec.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/path_util.h"

#include "modules/cnoa_pnc/planning/proto/aabox3d.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st {
namespace planning {

namespace {

constexpr double kEpsilon = 0.01;

}  // namespace

void KeepNearestStationarySpacetimeTrajectoryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  double nearest_stationary_s = std::numeric_limits<double>::max();
  const std::string* nearest_stationary_id_ptr = nullptr;
  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    const auto& st_boundary = *st_boundary_with_decision.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary()) continue;
    if (st_boundary.min_s() < kEpsilon) continue;
    if (const double follow_s =
            st_boundary.min_s() -
            st_boundary_with_decision.follow_standstill_distance();
        follow_s < nearest_stationary_s) {
      nearest_stationary_s = follow_s;
      nearest_stationary_id_ptr = &st_boundary.id();
    }
  }
  if (nearest_stationary_id_ptr != nullptr) {
    st_boundaries_with_decision->erase(
        std::remove_if(
            st_boundaries_with_decision->begin(),
            st_boundaries_with_decision->end(),
            [nearest_stationary_id_ptr](
                const StBoundaryWithDecision& st_boundary_with_decision) {
              const auto& st_boundary =
                  *st_boundary_with_decision.raw_st_boundary();
              if (st_boundary.source_type() !=
                  StBoundarySourceTypeProto::ST_OBJECT) {
                return false;
              }
              if (!st_boundary.is_stationary()) return false;
              if (st_boundary.min_s() < kEpsilon) return false;
              return st_boundary.id() != *nearest_stationary_id_ptr;
            }),
        st_boundaries_with_decision->end());
  }
}
std::optional<std::string> GetNearestStationaryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  double nearest_stationary_s = std::numeric_limits<double>::max();
  std::optional<std::string> nearest_stationary_id = std::nullopt;
  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    const auto& st_boundary = *st_boundary_with_decision.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary() &&
        st_boundary_with_decision.obj_pose_info().v() > Kph2Mps(2.0))
      continue;
    if (st_boundary.min_s() < kEpsilon) continue;
    if (const double follow_s = st_boundary.min_s();
        follow_s < nearest_stationary_s) {
      nearest_stationary_s = follow_s;
      nearest_stationary_id = st_boundary.id();
    }
  }
  return nearest_stationary_id;
}

void SetStBoundaryDebugInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    SpeedFinderDebugProto* speed_finder_proto) {
  CHECK_NOTNULL(speed_finder_proto);
  speed_finder_proto->mutable_st_boundaries()->clear();
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    StBoundaryProto st_boundary_proto;
    const auto* st_boundary = boundary_with_decision.st_boundary();
    st_boundary_proto.set_decision_type(boundary_with_decision.decision_type());
    st_boundary_proto.set_decision_reason(
        boundary_with_decision.decision_reason());
    st_boundary_proto.set_decision_info(
        std::string(boundary_with_decision.decision_info()));
    st_boundary_proto.set_follow_standstill_distance(
        boundary_with_decision.follow_standstill_distance());
    st_boundary_proto.set_lead_standstill_distance(
        boundary_with_decision.lead_standstill_distance());
    st_boundary_proto.set_object_type(st_boundary->object_type());
    st_boundary_proto.set_probability(st_boundary->probability());
    st_boundary_proto.set_min_s(st_boundary->min_s());
    st_boundary_proto.set_max_s(st_boundary->max_s());
    st_boundary_proto.set_min_t(st_boundary->min_t());
    st_boundary_proto.set_max_t(st_boundary->max_t());
    *st_boundary_proto.mutable_decision_prob() =
        boundary_with_decision.decision_prob();
    st_boundary_proto.set_is_stationary(st_boundary->is_stationary());

    speed_finder_proto->mutable_st_boundaries()->insert(
        {st_boundary->id(), st_boundary_proto});
  }
}

std::vector<StBoundaryWithDecision> InitializeStBoundaryWithDecision(
    std::vector<StBoundaryRef> raw_st_boundaries) {
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.reserve(raw_st_boundaries.size());
  for (auto& st_boundary : raw_st_boundaries) {
    // Initialize with unknown-decision.
    st_boundaries_with_decision.emplace_back(std::move(st_boundary));
  }
  return st_boundaries_with_decision;
}

int GetSpeedFinderTrajectorySteps(double init_v, double default_speed_limit) {
  constexpr double kMaxSpeedOfFixedTrajectorySteps = 22.23;  // m/s.->80km/h
  if (default_speed_limit <= kMaxSpeedOfFixedTrajectorySteps) {
    return kTrajectorySteps;
  }

  const PiecewiseLinearFunction<double> plf = {
      {kMaxSpeedOfFixedTrajectorySteps, default_speed_limit},
      {kTrajectorySteps, kSpeedFinderMaxTrajectorySteps}};
  return static_cast<int>(plf(init_v));
}

void PostProcessSpeedByFullStop(
    const SpeedFinderParamsProto& speed_finder_params,
    SpeedVector* speed_data) {
  CHECK_NOTNULL(speed_data);
  // TODO： Tune the full stop threshold down after control
  // performance improves.
  bool force_brake = true;
  int i = 0;
  while ((*speed_data)[i].t() <
         speed_finder_params.full_stop_traj_time_threshold()) {
    if ((*speed_data)[i].v() >
        speed_finder_params.full_stop_speed_threshold()) {
      force_brake = false;
      break;
    }
    ++i;
  }
  if (speed_data->TotalLength() >=
      speed_finder_params.full_stop_traj_length_threshold()) {
    force_brake = false;
  }
  if (force_brake) {
    for (int i = 0; i < speed_data->size(); ++i) {
      (*speed_data)[i].set_s(0.0);
      (*speed_data)[i].set_v(0.0);
      (*speed_data)[i].set_a(0.0);
      (*speed_data)[i].set_j(0.0);
    }
  }
}

std::vector<Box2d> BuildAvHeadBoxes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<VehicleShapeBasePtr>& av_shapes) {
  // stand for distance from head center to geo_center
  const double hc_to_gc =
      0.5 * (vehicle_geom.back_edge_to_center() + vehicle_geom.wheel_base());
  const double half_length = 0.5 * vehicle_geom.length() - hc_to_gc;
  const double half_width = vehicle_geom.width() * 0.5;
  std::vector<Box2d> av_head_boxes;
  av_head_boxes.reserve(av_shapes.size());
  for (const auto& av_shape : av_shapes) {
    const auto heading = av_shape->heading();
    const Vec2d tangent = Vec2d::FastUnitFromAngle(heading);
    const Vec2d center = av_shape->center() + tangent * hc_to_gc;
    av_head_boxes.push_back(
        Box2d(half_length, half_width, center, heading, tangent));
  }
  return av_head_boxes;
}

std::vector<VehicleShapeBasePtr> BuildAvShapes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path_points) {
  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  const double rac_to_center = half_length - vehicle_geom.back_edge_to_center();
  std::vector<VehicleShapeBasePtr> av_shapes;
  const int num_points = path_points.size();
  av_shapes.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const auto& pt = path_points[i];
    const double theta = pt.theta();
    const Vec2d rac(pt.x(), pt.y());
    const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
    const Vec2d center = rac + tangent * rac_to_center;
    av_shapes.push_back(std::make_unique<VehicleBoxShape>(
        vehicle_geom, rac, center, tangent, theta, half_length, half_width));
  }
  return av_shapes;
}

std::unique_ptr<SegmentMatcherKdtree> BuildPathKdTree(
    const DiscretizedPath& path_points) {
  std::vector<Vec2d> points;
  points.reserve(path_points.size());
  for (const auto& point : path_points) {
    points.emplace_back(point.x(), point.y());
  }
  return std::make_unique<SegmentMatcherKdtree>(points);
}

std::optional<PathApprox> BuildPathApproxForMirrors(
    const PathApprox& path_approx,
    const VehicleGeometryParamsProto& vehicle_geom) {
  if (!vehicle_geom.has_left_mirror() || !vehicle_geom.has_right_mirror()) {
    return std::nullopt;
  }
  // left_mirror().y() is positive.
  const auto& mirror = vehicle_geom.left_mirror();
  if (!mirror.has_width() || !mirror.has_length()) {
    return std::nullopt;
  }
  std::vector<PathSegment> mirror_segments;
  mirror_segments.reserve(path_approx.segments().size());
  const double mirrors_center_offset = 0.5 * vehicle_geom.length() -
                                       vehicle_geom.front_edge_to_center() +
                                       mirror.x();
  for (const auto& segment : path_approx.segments()) {
    const Vec2d mirrors_center =
        segment.center() + segment.tangent() * mirrors_center_offset;
    const double mirror_box_length =
        std::max(segment.length() - vehicle_geom.length(), 0.0) +
        mirror.width();
    Box2d path_segment_box(0.5 * mirror_box_length,
                           0.5 * mirror.length() + mirror.y(), mirrors_center,
                           segment.heading(), segment.tangent());
    mirror_segments.emplace_back(segment.first_index(), segment.last_index(),
                                 segment.first_ra(), segment.last_ra(),
                                 segment.first_s(), segment.last_s(),
                                 std::move(path_segment_box));
  }
  return PathApprox(std::move(mirror_segments), path_approx.path_kd_tree());
}

std::vector<PartialSpacetimeObjectTrajectory> GetConsideredStObjects(
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& obj_mgr,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>
        processed_st_objects) {
  std::unordered_map<std::string, PartialSpacetimeObjectTrajectory>
      considered_st_objects_map;
  considered_st_objects_map.reserve(st_boundaries_with_decision.size());
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    const auto decision_type = st_boundary_with_decision.decision_type();
    if (decision_type == StBoundaryProto::IGNORE ||
        decision_type == StBoundaryProto::UNKNOWN ||
        st_boundary_with_decision.st_boundary()->source_type() !=
            StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    const auto& traj_id = st_boundary_with_decision.traj_id();
    CHECK(traj_id.has_value());
    const auto* raw_st_boundary = st_boundary_with_decision.raw_st_boundary();
    if (considered_st_objects_map.find(*traj_id) ==
        considered_st_objects_map.end()) {
      // Judge whether the spacetime_object corresponding to current st-boundary
      // has been processed.
      if (processed_st_objects.find(*traj_id) != processed_st_objects.end()) {
        // Use st_object from processed_st_objects.
        considered_st_objects_map.emplace(
            *traj_id, PartialSpacetimeObjectTrajectory(std::move(
                          FindOrDie(processed_st_objects, *traj_id))));
      } else {
        // Use st_object from obj_mgr.
        considered_st_objects_map.emplace(
            *traj_id, PartialSpacetimeObjectTrajectory(*CHECK_NOTNULL(
                          obj_mgr.FindTrajectoryById(*traj_id))));
      }
    }
    CHECK(decision_type == StBoundaryProto::OVERTAKE ||
          decision_type == StBoundaryProto::YIELD ||
          decision_type == StBoundaryProto::FOLLOW);
    FindOrDie(considered_st_objects_map, *traj_id)
        .AppendTimeRangeAndDecisonType(
            raw_st_boundary->min_t(), raw_st_boundary->max_t(),
            decision_type == StBoundaryProto::OVERTAKE
                ? PartialSpacetimeObjectTrajectory::DecisionType::LEAD
                : PartialSpacetimeObjectTrajectory::DecisionType::FOLLOW);
  }
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  considered_st_objects.reserve(considered_st_objects_map.size());
  for (auto& [_, st_obj] : considered_st_objects_map) {
    considered_st_objects.push_back(std::move(st_obj));
  }
  return considered_st_objects;
}

void CutoffSpeedByTimeHorizon(SpeedVector* speed_data) {
  CHECK_NOTNULL(speed_data);
  constexpr double kTimeHorizon = kTrajectoryTimeStep * kTrajectorySteps;
  speed_data->erase(
      std::lower_bound(speed_data->begin(), speed_data->end(), kTimeHorizon,
                       [](const auto& pt, double t) { return pt.t() < t; }),
      speed_data->end());
}

SpeedLimit GenerateRawLaneSpeedLimit(const DrivePassage* drive_passage,
                                     const DiscretizedPath& path_points,
                                     double allowed_max_speed) {
  const auto get_speed_limit =
      [drive_passage, allowed_max_speed](const PathPoint& path_point) {
        if (drive_passage == nullptr) return allowed_max_speed;
        const auto lane_speed_limit =
            drive_passage->QuerySpeedLimitAt(ToVec2d(path_point));
        return lane_speed_limit.ok() ? *lane_speed_limit : allowed_max_speed;
      };
  constexpr double kSpeedLimitSampleRange = 1.0;  // m.
  constexpr double kApproxSpeedLimitEps = 0.5;    // m/s.
  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
  double last_sample_s = 0.0;
  const int num_points = path_points.size();
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), get_speed_limit(path_points[0]));
  for (int i = 0; i < num_points; ++i) {
    if (path_points[i].s() - last_sample_s > kSpeedLimitSampleRange ||
        i == num_points - 1) {
      last_sample_s = path_points[i].s();
      const double curr_speed_limit = get_speed_limit(path_points[i]);
      if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
              kApproxSpeedLimitEps ||
          i == num_points - 1) {
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

SpeedVector GenerateReferenceSpeed(
    const std::vector<SpeedBoundWithInfo>& min_speed_limit,
    const std::optional<SpeedLimit>& raw_lane_speed_limit, double init_v,
    double ref_speed_bias, double ref_speed_static_limit_bias, double max_accel,
    double max_decel, double total_time, double delta_t) {
  CHECK_GT(delta_t, 0.0);
  constexpr double kMaxComfortAcc = 1.4;  // m/s^2.
  const double max_speed_limit = init_v + kMaxComfortAcc * total_time;
  const double init_v_ref = std::max(init_v, 0.0) + ref_speed_bias;
  std::vector<double> reference_speed;
  reference_speed.reserve(min_speed_limit.size());

  double init_bound = min_speed_limit[0].bound + ref_speed_static_limit_bias;
  if (raw_lane_speed_limit.has_value()) {
    const auto lane_speed_limit = raw_lane_speed_limit->GetSpeedLimitByS(0.0);
    if (lane_speed_limit.has_value()) {
      init_bound = std::max(*lane_speed_limit, init_v);
    }
  }

  reference_speed.push_back(std::min(init_v_ref, init_bound));
  for (int i = 1; i < min_speed_limit.size(); ++i) {
    double bound = min_speed_limit[i].bound + ref_speed_static_limit_bias;
    if (raw_lane_speed_limit.has_value()) {
      const auto lane_speed_limit =
          raw_lane_speed_limit->GetSpeedLimitByS(i * delta_t);
      if (lane_speed_limit.has_value()) {
        bound = std::max(*lane_speed_limit, init_v);
      }
    }
    bound = std::min(bound, max_speed_limit);
    reference_speed.push_back(
        std::min(reference_speed.back() + max_accel * delta_t, bound));
  }
  for (int i = reference_speed.size() - 2; i >= 0; --i) {
    reference_speed[i] = std::min(reference_speed[i],
                                  reference_speed[i + 1] - max_decel * delta_t);
  }
  SpeedVector speed_points;
  speed_points.reserve(reference_speed.size());
  double s = 0.0;
  double t = 0.0;
  for (int i = 0; i < reference_speed.size() - 1; ++i) {
    speed_points.emplace_back(
        /*t=*/t, /*s=*/s,
        /*v=*/reference_speed[i],
        /*a=*/(reference_speed[i + 1] - reference_speed[i]) / delta_t,
        /*j=*/0.0);
    t += delta_t;
    s += (reference_speed[i + 1] + reference_speed[i]) * 0.5 * delta_t;
  }
  speed_points.emplace_back(/*t=*/t, /*s=*/s,
                            /*v=*/reference_speed.back(),
                            /*a=*/0.0,
                            /*j=*/0.0);
  return speed_points;
}

std::vector<std::pair<double, double>> GenerateAccelerationBound(
    const st::MotionConstraintParamsProto& motion_constraint_params,
    const std::optional<double> acc_target_a,
    const std::optional<double> dec_target_a, double start_a, double delta_t,
    int knot_num) {
  std::vector<std::pair<double, double>> accel_bound;
  accel_bound.clear();
  accel_bound.reserve(knot_num);
  constexpr double kComfortAccJerk = 0.5;
  constexpr double reach_target_time = 2.0;
  const double upper_bound = motion_constraint_params.max_acceleration();
  const double lower_bound = motion_constraint_params.max_deceleration();
  const double max_decel =
      acc_target_a.has_value() ? acc_target_a.value() : lower_bound;
  double expect_jerk = kComfortAccJerk;
  double modified_target_a =
      std::min(max_decel, start_a + kComfortAccJerk * reach_target_time);
  bool reach_target = false;
  double prev_a = start_a;
  for (int i = 0; i < knot_num; i++) {
    const double t = i * delta_t;
    if (t < reach_target_time) {
      double curr_a = prev_a + expect_jerk * delta_t;
      double modified_lower_bound = std::min(curr_a, modified_target_a);
      prev_a = curr_a;
      if (modified_lower_bound > 0.8 * modified_target_a) {
        reach_target = true;
      }
      accel_bound.emplace_back(std::make_pair(
          upper_bound, std::max(lower_bound, modified_lower_bound)));
      if (reach_target) {
        expect_jerk -= kComfortAccJerk * delta_t;
      }
    } else {
      accel_bound.emplace_back(std::make_pair(upper_bound, lower_bound));
    }
  }
  return accel_bound;
}

bool IsConsiderOncomingObs(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const StBoundary& st_boundary, const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params, double current_v,
    const DiscretizedPath& path) {
  const auto& overlap_info = st_boundary.overlap_infos().front();
  const int overlap_av_idx =
      (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;
  // CHECK_GT(path.size(), overlap_av_idx);
  // const auto& overlap_av_path_point = path[overlap_av_idx];

  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto obj_pose = traj->planner_object().pose().pos();
  const auto obj_vel = traj->planner_object().pose().v();
  const auto& nearest_station = drive_passage.FindNearestStation(obj_pose);
  const auto lane_id = nearest_station.lane_id();
  // const auto& lane = psmm.FindLaneByIdOrNull(lane_id);
  // if (!lane) return false;
  const auto& overlap_meta = *st_boundary.overlap_meta();
  if (st_boundary.object_type() == StBoundaryProto::VEHICLE &&
      overlap_meta.pattern() == StOverlapMetaProto::LEAVE) {
    return false;
  }
  const auto& curr_path_point = path.front();
  const auto av_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), vehicle_geometry_params);
  const auto av_frenet_box_or = drive_passage.QueryFrenetBoxAt(av_box);

  const auto obj_frenet_box_or =
      drive_passage.QueryFrenetBoxAtContour(traj->contour(), false);
  if (!obj_frenet_box_or.ok() || !av_frenet_box_or.ok()) {
    return false;
  }
  const auto& obj_frenet_box = *obj_frenet_box_or;
  const auto& av_frenet_box = *av_frenet_box_or;

  constexpr double kBufferlat = 0.35;
  constexpr double kTTC_buffer = 1.5;
  bool is_safte_lon =
      obj_frenet_box.s_min - av_frenet_box.s_max >
      std::min(35.0, std::max(15.0, kTTC_buffer * (obj_vel + current_v)));
  const auto lane_boundary_info =
      drive_passage.QueryEnclosingLaneBoundariesAtS(obj_frenet_box.center_s());
  const double lane_width = lane_boundary_info.left->lat_offset -
                            lane_boundary_info.right->lat_offset;
  std::string debug = "";
  debug = absl::StrCat(
      "traj_id: ", traj_id, " [ ", is_safte_lon, " ] ", " lat: ",
      std::min(std::fabs(obj_frenet_box.l_max),
               std::fabs(obj_frenet_box.l_min)),
      " lon ", std::fabs(obj_frenet_box.s_min - av_frenet_box.s_max),
      " obj_lon ", std::fabs(obj_frenet_box.s_min), " av_lon ",
      std::fabs(av_frenet_box.s_max), " lon_th ",
      std::fabs(kTTC_buffer * (obj_vel + current_v)), "lane_width", lane_width);
  Log2DDS::LogDataV0("IsConsiderOncomingObs", debug);
  double lat_threshold = 0.5 * vehicle_geometry_params.width() + kBufferlat;
  constexpr double DEFAULT_LANE_WIDTH = 3.75;  // m
  if (st_boundary.object_type() == StBoundaryProto::VEHICLE) {
    // get curren lane width
    lat_threshold = std::max(
        0.0, vehicle_geometry_params.width() + kBufferlat - lane_width * 0.5);
  } else if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN ||
             st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    lat_threshold = 0.5 * vehicle_geometry_params.width() + kBufferlat;
  }
  if (!is_safte_lon &&
      ((std::min(std::fabs(obj_frenet_box.l_max),
                 std::fabs(obj_frenet_box.l_min)) < lat_threshold) ||
       obj_frenet_box.l_max * obj_frenet_box.l_min < 0.0)) {
    Log2DDS::LogDataV0("IsConsiderOncomingObs", traj_id + "consider");
    return true;
  }
  return false;
}

SpeedBoundMapType EstimateSpeedBound(
    const SpeedLimitProvider& speed_limit_provider,
    const SpeedVector& preliminary_speed, double init_v,
    double allowed_max_speed, int knot_num, double delta_t) {
  const auto fill_speed_bound =
      [](const std::optional<SpeedLimit::SpeedLimitInfo>& speed_limit_info,
         SpeedBoundWithInfo* speed_bound, double allowed_max_speed) {
        CHECK_NOTNULL(speed_bound);
        speed_bound->bound = speed_limit_info.has_value()
                                 ? speed_limit_info->speed_limit
                                 : allowed_max_speed;
        speed_bound->info =
            speed_limit_info.has_value() ? speed_limit_info->info : "NO_TYPE";
      };

  SpeedBoundMapType speed_upper_bound_map;

  std::vector<double> estimated_s;
  estimated_s.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    const double t = i * delta_t;
    const auto speed_point = preliminary_speed.EvaluateByTime(t);
    const double s = speed_point.has_value() ? speed_point->s() : init_v * t;
    estimated_s.push_back(s);
  }

  // Emplace default speed limit.
  speed_upper_bound_map.emplace(
      SpeedLimitTypeProto_Type_DEFAULT,
      std::vector<SpeedBoundWithInfo>(
          knot_num,
          SpeedBoundWithInfo{.bound = allowed_max_speed, .info = "Default."}));
  // Estimate speed bound for static speed limit.
  for (const auto& [type, speed_limit] :
       speed_limit_provider.static_speed_limit_map()) {
    std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
    speed_bounds_with_info.reserve(knot_num);
    for (int i = 0; i < knot_num; ++i) {
      const auto speed_limit_info =
          speed_limit.GetSpeedLimitInfoByS(estimated_s[i]);
      fill_speed_bound(speed_limit_info, &speed_bounds_with_info.emplace_back(),
                       allowed_max_speed);
    }
    speed_upper_bound_map.emplace(type, std::move(speed_bounds_with_info));
  }

  // Estimate speed bound for dynamic speed limit.
  std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
  speed_bounds_with_info.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    const double t = i * delta_t;
    const auto dynamic_speed_limit_info =
        speed_limit_provider.GetDynamicSpeedLimitInfoByTimeAndS(t,
                                                                estimated_s[i]);
    SpeedBoundWithInfo speed_bound;
    fill_speed_bound(dynamic_speed_limit_info, &speed_bound, allowed_max_speed);
    speed_bounds_with_info.push_back(std::move(speed_bound));
  }
  speed_upper_bound_map.emplace(SpeedLimitTypeProto_Type_MOVING_CLOSE_TRAJ,
                                std::move(speed_bounds_with_info));

  // Get speed bound for vt speed limit.
  for (const auto& [type, _] : speed_limit_provider.vt_speed_limit_map()) {
    const bool has_type =
        speed_upper_bound_map.find(type) != speed_upper_bound_map.end();

    for (int i = 0; i < knot_num; ++i) {
      const double t = i * delta_t;
      const auto vt_speed_limit_info =
          speed_limit_provider.GetVtSpeedLimitInfoByTypeAndTime(type, t);
      if (!has_type) {
        SpeedBoundWithInfo speed_bound = {
            .bound = vt_speed_limit_info.has_value()
                         ? vt_speed_limit_info->speed_limit
                         : allowed_max_speed,
            .info = vt_speed_limit_info.has_value()
                        ? absl::StrCat(vt_speed_limit_info->info, "-vt")
                        : "NO_TYPE"};
        speed_upper_bound_map[type].push_back(std::move(speed_bound));
      } else {
        // Merge vt speed limit to a particular type.
        if (!vt_speed_limit_info.has_value()) continue;
        SpeedBoundWithInfo& origin = speed_upper_bound_map[type][i];
        if (vt_speed_limit_info->speed_limit < origin.bound) {
          origin.bound = vt_speed_limit_info->speed_limit;
          origin.info = absl::StrCat(vt_speed_limit_info->info, "-vt");
        }
      }
    }
  }

  return speed_upper_bound_map;
}

std::vector<SpeedBoundWithInfo> GenerateMinSpeedLimitWithLaneAndCurvature(
    const std::vector<SpeedBoundWithInfo>& lane_speed_limit,
    const std::vector<SpeedBoundWithInfo>& curvature_speed_limit) {
  std::vector<SpeedBoundWithInfo> min_speed_limit;
  CHECK_EQ(lane_speed_limit.size(), curvature_speed_limit.size());
  int knot_num = static_cast<int>(lane_speed_limit.size());
  min_speed_limit.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    min_speed_limit.push_back(SpeedBoundWithInfo{
        .bound =
            std::min(lane_speed_limit[i].bound, curvature_speed_limit[i].bound),
        .info = "Min speed limit between lane and curvature."});
  }
  return min_speed_limit;
}

std::vector<std::string> MakeStBoundaryDebugInfo(
    const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeTrajectoryManager& traj_mgr) {
  std::vector<std::string> debug_info_list;
  constexpr int kDebugInfoNum = 23;
  debug_info_list.reserve(kDebugInfoNum);
  const StBoundary* st_boundary = st_boundary_wd.st_boundary();
  CHECK_NOTNULL(st_boundary);
  auto push = [&debug_info_list](const auto& name, const auto& value) {
    debug_info_list.push_back(absl::StrFormat("%s:%s", name, value));
  };
  auto to_str = [](double val) { return absl::StrFormat("%.3f", val); };
  push("source_type", StBoundary::SourceTypeName(st_boundary->source_type()));
  push("object_type",
       StBoundaryProto::ObjectType_Name(st_boundary->object_type()));
  push("is_stationary", st_boundary->is_stationary() ? "true" : "false");
  push("protective_type",
       StBoundaryProto::ProtectionType_Name(st_boundary->protection_type()));
  push("is_large_vehicle", st_boundary->is_large_vehicle() ? "true" : "false");
  push("first_overlap_v",
       st_boundary->speed_points().empty()
           ? "None"
           : to_str(st_boundary->speed_points().front().v()) + "m/s");
  push("first_overlap_s", to_str(st_boundary->bottom_left_point().s()));
  push("first_overlap_t", to_str(st_boundary->bottom_left_point().t()));

  const auto& object_id = st_boundary->object_id();
  double obj_v = 0.0;
  double obj_a = 0.0;
  if (object_id.has_value()) {
    const auto& pose =
        CHECK_NOTNULL(traj_mgr.FindObjectByObjectId(*object_id))->pose();
    obj_v = pose.v();
    obj_a = pose.a();
  }
  push("object_speed", to_str(obj_v));
  push("object_accel", to_str(obj_a));

  push("follow_standstill",
       to_str(st_boundary_wd.follow_standstill_distance()));
  push("lead_standstill", to_str(st_boundary_wd.lead_standstill_distance()));
  push("pass_time", to_str(st_boundary_wd.pass_time()));
  push("yield_time", to_str(st_boundary_wd.yield_time()));
  push("decision_info", st_boundary_wd.decision_info());
  push("modifier_type", StBoundaryModifierProto::ModifierType_Name(
                            st_boundary_wd.modifier().modifier_type()));
  switch (st_boundary->obj_scenario_info().relationship) {
    case Relationship::OnComing:
      push("relationship", "OnComing");
      break;
    case Relationship::NotRelevant:
      push("relationship", "NotRelevant");
      break;
    case Relationship::SameDir:
      push("relationship", "SameDir");
      break;
    case Relationship::Merge:
      push("relationship", "Merge");
      break;
    case Relationship::Cross:
      push("relationship", "Cross");
      break;
    case Relationship::Static:
      push("relationship", "Static");
      break;
    default:
      push("relationship", "Unknown");
      break;
  }

  // overlap_meta
  const auto& overlap_meta = st_boundary->overlap_meta();
  const bool has_meta = overlap_meta.has_value();

  push("is_oncoming_obj",
       has_meta ? (overlap_meta->is_oncoming() ? "true" : "false") : "false");
  push("meta_pattern", has_meta ? StOverlapMetaProto::OverlapPattern_Name(
                                      overlap_meta->pattern())
                                : "None");
  push("meta_source",
       has_meta ? StOverlapMetaProto::OverlapSource_Name(overlap_meta->source())
                : "None");
  push("meta_priority", has_meta ? StOverlapMetaProto::OverlapPriority_Name(
                                       overlap_meta->priority())
                                 : "None");
  push("meta_modification_type",
       has_meta ? StOverlapMetaProto::ModificationType_Name(
                      overlap_meta->modification_type())
                : "None");
  push("meta_obj_lane_direction", has_meta
                                      ? StOverlapMetaProto::LaneDirection_Name(
                                            overlap_meta->obj_lane_direction())
                                      : "None");
  return debug_info_list;
}

void DumpStGraphBoundary(
    int plan_id,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const SpeedFinderDebugProto& speed_finder_proto) {
  const auto get_stb_points = [](const StBoundary* st_boundary) {
    std::vector<StPoint> st_points;
    if (st_boundary == nullptr) return st_points;
    const auto& lower_points = st_boundary->lower_points();
    const auto& upper_points = st_boundary->upper_points();
    if (lower_points.empty() || upper_points.empty()) {
      return st_points;
    }
    st_points.reserve(lower_points.size() + upper_points.size());
    for (int i = 0; i < lower_points.size(); ++i) {
      st_points.push_back(lower_points[i]);
    }
    for (int i = upper_points.size() - 1; i >= 0; --i) {
      st_points.push_back(upper_points[i]);
    }
    return st_points;
  };

  const auto get_decision_suffix = [](StBoundaryProto::DecisionType type) {
    switch (type) {
      case StBoundaryProto::FOLLOW:
        return "_F";
      case StBoundaryProto::YIELD:
        return "_Y";
      case StBoundaryProto::OVERTAKE:
        return "_O";
      case StBoundaryProto::IGNORE:
        return "_I";
      case StBoundaryProto::UNKNOWN:
        return "_U";
    }
    return "_U";
  };

  const auto get_soft_bound_idx = [](const auto& times, double t) {
    return std::distance(times.begin(),
                         std::lower_bound(times.begin(), times.end(), t));
  };

  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    // Dump st_boundary points.
    const auto decision = st_boundary_with_decision.decision_type();
    const auto decision_suffix = get_decision_suffix(decision);
    const auto color =
        st_boundary_with_decision.decision_type() == StBoundaryProto::IGNORE
            ? Log2DDS::kTiffanyBlue
            : Log2DDS::kOrange;

    const auto& st_boundary = st_boundary_with_decision.st_boundary();
    const auto st_points = get_stb_points(st_boundary);
    if (!st_points.empty()) {
      Log2DDS::LogChartV2(
          Log2DDS::TaskPrefix(plan_id) + "st",
          st_boundary->id() + decision_suffix, color, true, st_points,
          [](const StPoint& p) -> double { return p.t(); },
          [](const StPoint& p) -> double { return p.s(); });
    }

    // Dump debug_info.
    Log2DDS::LogDataV2(
        Log2DDS::TaskPrefix(plan_id) + "stbinfo_" + st_boundary->id(),
        MakeStBoundaryDebugInfo(st_boundary_with_decision, traj_mgr));

    // Dump st_boundary soft bound.
    if (decision == StBoundaryProto::UNKNOWN ||
        decision == StBoundaryProto::IGNORE) {
      continue;
    }
    const auto id =
        GetStBoundaryIntegrationId(*st_boundary_with_decision.st_boundary());
    const auto& speed_opt_debug = speed_finder_proto.speed_optimizer();
    CHECK(decision == StBoundaryProto::YIELD ||
          decision == StBoundaryProto::OVERTAKE ||
          decision == StBoundaryProto::FOLLOW);
    const auto* soft_bound_data =
        (decision == StBoundaryProto::YIELD ||
         decision == StBoundaryProto::FOLLOW)
            ? FindOrNull(speed_opt_debug.soft_s_upper_bound(), id)
            : FindOrNull(speed_opt_debug.soft_s_lower_bound(), id);
    if (!soft_bound_data || soft_bound_data->time_size() < 2) continue;

    const double min_t = st_boundary->min_t();
    const double max_t = st_boundary->max_t();
    const int start_idx = get_soft_bound_idx(soft_bound_data->time(), min_t);
    const int soft_bound_size = soft_bound_data->time_size();
    std::vector<StPoint> stb_st;
    stb_st.reserve(soft_bound_size);

    // First point.
    const auto stb_start_pt =
        st_boundary->GetBoundarySRange(soft_bound_data->time(start_idx));
    if (!stb_start_pt.has_value()) continue;
    const double stb_start_point_s = (decision == StBoundaryProto::YIELD ||
                                      decision == StBoundaryProto::FOLLOW)
                                         ? stb_start_pt->second
                                         : stb_start_pt->first;
    stb_st.emplace_back(stb_start_point_s, soft_bound_data->time(start_idx));

    int end_idx = 0.0;
    for (int i = start_idx; i < soft_bound_size; ++i) {
      const double curr_time = soft_bound_data->time(i);
      if (!InRange(curr_time, min_t, max_t)) break;
      end_idx = i;
      stb_st.emplace_back(soft_bound_data->value(i), curr_time);
    }

    // Back point.
    const auto stb_end_pt =
        st_boundary->GetBoundarySRange(soft_bound_data->time(end_idx));
    if (!stb_end_pt.has_value()) return;
    const double stb_end_point_s = (decision == StBoundaryProto::YIELD ||
                                    decision == StBoundaryProto::FOLLOW)
                                       ? stb_end_pt->second
                                       : stb_end_pt->first;
    stb_st.emplace_back(stb_end_point_s, soft_bound_data->time(end_idx));

    const auto group_name = Log2DDS::TaskPrefix(plan_id) + "st_softbound";
    Log2DDS::LogChartV2(
        group_name, st_boundary->id() + "_softbound", Log2DDS::kGray, false,
        stb_st, [](const StPoint& p) -> double { return p.t(); },
        [](const StPoint& p) -> double { return p.s(); });
  }
}

void DumpStGraphSpeed(int plan_id, double path_length, int trajectory_steps,
                      const SpeedVector& preliminary_speed,
                      const SpeedVector& optimized_speed,
                      const SpeedVector& comfortable_brake_speed,
                      const SpeedVector& max_brake_speed) {
  const auto group_name = Log2DDS::TaskPrefix(plan_id) + "st_speed";

  // end_of_path
  SpeedVector end_of_path(
      {SpeedPoint(0.0, path_length, 0.0, 0.0, 0.0),
       SpeedPoint((trajectory_steps - 1) * kTrajectoryTimeStep, path_length,
                  0.0, 0.0, 0.0)});
  Log2DDS::LogChartV2(
      group_name, "end_of_path", Log2DDS::kGray, false, end_of_path,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.s(); });

  // preliminary_speed
  Log2DDS::LogChartV2(
      group_name, "preliminary_speed", Log2DDS::kOrange, false,
      preliminary_speed, [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.s(); });

  // optimized_speed
  Log2DDS::LogChartV2(
      group_name, "optimized_speed", Log2DDS::kGreen, false, optimized_speed,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.s(); });

  // comfortable_brake_speed
  Log2DDS::LogChartV2(
      group_name, "comfortable_brake_speed", Log2DDS::kLightGray, false,
      comfortable_brake_speed,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.s(); });

  // max_brake_speed
  Log2DDS::LogChartV2(
      group_name, "max_brake_speed", Log2DDS::kLightGray, false,
      max_brake_speed, [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.s(); });
}

void DumpVtGraphSpeedLimit(int plan_id,
                           const SpeedFinderDebugProto& speed_finder_proto) {
  const std::unordered_map<SpeedLimitTypeProto::Type, Log2DDS::Color> types = {
      {SpeedLimitTypeProto_Type_LANE, Log2DDS::kBlack},
      {SpeedLimitTypeProto_Type_CURVATURE, Log2DDS::kRed},
      {SpeedLimitTypeProto_Type_EXTERNAL, Log2DDS::kBlue},
      {SpeedLimitTypeProto_Type_MOVING_CLOSE_TRAJ, Log2DDS::kDarkGreen},
      {SpeedLimitTypeProto_Type_UNCERTAIN_PEDESTRAIN, Log2DDS::kLightBlue},
      {SpeedLimitTypeProto_Type_UNCERTAIN_VEHICLE, Log2DDS::kViolet},
      {SpeedLimitTypeProto_Type_STEER_RATE, Log2DDS::kAqua},
      {SpeedLimitTypeProto_Type_NEAR_PARALLEL_VEHICLE, Log2DDS::kHotpink},
      {SpeedLimitTypeProto_Type_IGNORE_OBJECT, Log2DDS::kMiddleBlueGreen},
      {SpeedLimitTypeProto_Type_CLOSE_CURB, Log2DDS::kCoral},
      {SpeedLimitTypeProto_Type_CROSS_CURB, Log2DDS::kTiffanyBlue},
      {SpeedLimitTypeProto_Type_DEFAULT, Log2DDS::kTiffanyBlue},
      {SpeedLimitTypeProto_Type_SOFT_ACC, Log2DDS::kBrown},
      {SpeedLimitTypeProto_Type_APPROACH_CURB, Log2DDS::kLime},
      {SpeedLimitTypeProto_Type_TOLL_SPEED_LIMIT, Log2DDS::kPurple},
      {SpeedLimitTypeProto_Type_V2_TURN_TYPE, Log2DDS::kDarkGreen},
      {SpeedLimitTypeProto_Type_RIGHT_TURN_CLOSE, Log2DDS::kDarkGreen},
      {SpeedLimitTypeProto_Type_BIG_JUNCTION, Log2DDS::kDarkBlue},
      {SpeedLimitTypeProto_Type_FAST_SPEED_LIMIT, Log2DDS::kDarkRed},
      {SpeedLimitTypeProto_Type_NEAREST_CLOSE, Log2DDS::kGrassGreen},
      {SpeedLimitTypeProto_Type_IN_JUNCTION_T_MAP, Log2DDS::kDarkBlue},
      {SpeedLimitTypeProto_Type_TRAFFIC_ACC_GAP, Log2DDS::kPink},
      {SpeedLimitTypeProto_Type_CREEP_INTERACTION, Log2DDS::kTiffanyBlue},
      {SpeedLimitTypeProto_Type_PERCEPTION_LOSS, Log2DDS::kDarkBlue},
      {SpeedLimitTypeProto_Type_GAP_REF_SPEED, Log2DDS::kPurple},
      {SpeedLimitTypeProto_Type_COMBINATION, Log2DDS::kRed},
      {SpeedLimitTypeProto_Type_DEFENSIVE, Log2DDS::kBrown}};
  auto group_name = Log2DDS::TaskPrefix(plan_id) + "vt_limit";
  for (const auto& [type, color] : types) {
    const auto type_name = SpeedLimitTypeProto::Type_Name(type);
    const auto* data = FindOrNull(
        speed_finder_proto.speed_optimizer().speed_limit(), type_name);
    if (data == nullptr) {
      continue;
    }
    CHECK_EQ(data->time_size(), data->value_size());
    constexpr double kMaxPlotSpeedLimit = 40.0;  // m/s
    constexpr double kEpsilon = 1.0e-6;
    SpeedVector speed_limit;
    speed_limit.reserve(data->time_size());
    std::vector<Log2DDS::ChartInfos> infos;
    infos.reserve(data->time_size());
    for (int i = 0; i < static_cast<int>(data->time_size()); ++i) {
      const auto time = data->time(i);
      const auto value = data->value(i);
      speed_limit.emplace_back(time, 0.0, std::min(value, kMaxPlotSpeedLimit),
                               0.0, 0.0);
      Log2DDS::ChartInfos info;
      info.set_name("info");
      info.set_value(data->info(i));
      infos.emplace_back(info);
    }
    Log2DDS::LogChartV2(
        group_name, type_name, color, false, speed_limit,
        [](const SpeedPoint& p) -> double { return p.t(); },
        [](const SpeedPoint& p) -> double { return p.v(); }, infos);
  }
}

void DumpVtGraphSpeed(int plan_id, const SpeedVector& preliminary_speed,
                      const SpeedVector& optimized_speed,
                      const SpeedVector& ref_speed) {
  auto group_name = Log2DDS::TaskPrefix(plan_id) + "vt_speed";

  // preliminary_speed
  Log2DDS::LogChartV2(
      group_name, "preliminary_speed", Log2DDS::kOrange, false,
      preliminary_speed, [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.v(); }, /*infos=*/{});

  // optimized_speed
  Log2DDS::LogChartV2(
      group_name, "optimized_speed", Log2DDS::kGreen, false, optimized_speed,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.v(); }, /*infos=*/{});

  // ref_speed
  Log2DDS::LogChartV2(
      group_name, "ref_speed", Log2DDS::kMagenta, false, ref_speed,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.v(); }, /*infos=*/{});
}

void DumpAccPath(int plan_id, const DiscretizedPath& path,
                 const Box2d& av_box) {
  const std::string group_name = Log2DDS::TaskPrefix(plan_id) + "DrivePassage";
  std::vector<Vec2d> center_points;
  center_points.reserve(path.size());
  for (const auto& pt : path) {
    center_points.push_back(ToVec2d(pt));
  }
  Log2DDS::LogChartV2(
      group_name, "center", Log2DDS::kGray, false, center_points,
      [](const auto& p) -> double { return p.x(); },
      [](const auto& p) -> double { return p.y(); });

  const std::string box_group_name =
      Log2DDS::TaskPrefix(plan_id) + "PathContour";
  Log2DDS::LogChartV2(
      box_group_name, "ego_contour", Log2DDS::kGray, false,
      av_box.GetCornersWithBufferCounterClockwise(0.0, 0.0),
      [](const auto& p) -> double { return p.x(); },
      [](const auto& p) -> double { return p.y(); });
}

void DumpPredictionTrajectories(
    int plan_id,
    const std::unordered_map<std::string, const SpacetimeObjectTrajectory*>&
        st_trajs_map) {
  for (const auto& [traj_id, traj] : st_trajs_map) {
    const std::vector<prediction::PredictedTrajectoryPoint>& pts =
        traj->trajectory().points();
    Log2DDS::LogChartV2(
        Log2DDS::TaskPrefix(plan_id) + "pred-traj_vt", traj_id, Log2DDS::kGray,
        false, pts, [](const auto& p) -> double { return p.t(); },
        [](const auto& p) -> double { return p.v(); });
    Log2DDS::LogChartV2(
        Log2DDS::TaskPrefix(plan_id) + "pred-traj_at", traj_id, Log2DDS::kGray,
        false, pts, [](const auto& p) -> double { return p.t(); },
        [](const auto& p) -> double { return p.a(); });
  }
}

void DumpPrimaryBrakingTarget(const SpeedVector& optimized_speed,
                              const SpeedFinderDebugProto& speed_finder_proto,
                              int plan_id) {
  if (optimized_speed.empty()) return;
  double max_decel = std::numeric_limits<double>::max();
  for (const auto& speed_point : optimized_speed) {
    max_decel = std::min(max_decel, speed_point.a());
  }
  constexpr double kBrakingDecelThres = -0.3;  // m/ss.
  if (max_decel > kBrakingDecelThres) return;
  const auto query_soft_bound_idx = [](const auto& times, double t) {
    return std::distance(times.begin(),
                         std::lower_bound(times.begin(), times.end(), t));
  };
  const PiecewiseLinearFunction<double, double> time_atten_plf = {{5.0, 8.0},
                                                                  {1.0, 0.5}};
  constexpr double kTimeStep = 0.2;  // s.
  const auto& upper_bounds_map =
      speed_finder_proto.speed_optimizer().soft_s_upper_bound();
  double max_intrusion_value = std::numeric_limits<double>::lowest();
  std::string max_intrusion_target_id;
  for (double t = 0.0; t < optimized_speed.back().t(); t += kTimeStep) {
    const auto speed_pt = optimized_speed.EvaluateByTime(t);
    const double time_atten = time_atten_plf(t);
    if (!speed_pt.has_value()) continue;
    for (const auto& pair : upper_bounds_map) {
      const auto& id = pair.first;
      const auto& data = pair.second;
      const int data_size = data.time_size();
      if (data_size == 0 ||
          !InRange(t, data.time(0), data.time(data_size - 1))) {
        continue;
      }
      const int idx = query_soft_bound_idx(data.time(), t);
      const double intrusion_value =
          time_atten * (speed_pt->s() - data.value(idx));
      if (intrusion_value > 0.0) {
        max_intrusion_value = std::max(intrusion_value, max_intrusion_value);
        max_intrusion_target_id = id;
      }
    }
  }

  if (max_intrusion_value > 0.0) {
    Log2DDS::LogDataV2(
        absl::StrCat(Log2DDS::TaskPrefix(plan_id), "primary_braking_target"),
        max_intrusion_target_id);
  }
}

CipvObjectInfo ComputeCipvObjectInfo(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SegmentMatcherKdtree& path_kd_tree,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  CipvObjectInfo cipv_object_info;
  double nearest_object_s = std::numeric_limits<double>::infinity();
  double nearest_stay_object_s = std::numeric_limits<double>::infinity();
  double second_nearest_stay_object_s = std::numeric_limits<double>::infinity();
  for (const StBoundaryWithDecision& st_boundary_wd : *st_boundaries_wd) {
    const StBoundary* st_boundary = st_boundary_wd.raw_st_boundary();
    CHECK_NOTNULL(st_boundary);
    if (st_boundary->source_type() != StBoundarySourceTypeProto::ST_OBJECT ||
        st_boundary_wd.decision_type() == StBoundaryProto::IGNORE ||
        st_boundary->min_t() > 0.0) {
      continue;
    }

    CHECK(st_boundary->traj_id().has_value());
    CHECK(st_boundary->object_id().has_value());
    const auto& object_id = *st_boundary->object_id();
    const auto& traj_id = *st_boundary->traj_id();
    if (ContainsKey(cipv_object_info.object_distance_map, object_id)) {
      continue;
    }
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
    Vec2d pos = traj->pose().pos();
    if (!traj->states().empty()) {
      pos = traj->states().front().traj_point->pos();
    }
    double s = 0.0;
    double l = 0.0;
    if (!path_kd_tree.GetProjection(pos.x(), pos.y(), /*is_clamp=*/false, &s,
                                    &l)) {
      continue;
    }
    const auto& overlap_meta = st_boundary->overlap_meta();
    if ((overlap_meta.has_value() &&
         overlap_meta->pattern() == StOverlapMetaProto::STAY) ||
        st_boundary->is_stationary()) {
      auto& nearest_stay_id = cipv_object_info.nearest_stay_object_id;
      if (!nearest_stay_id.has_value()) {
        nearest_stay_id = object_id;
        nearest_stay_object_s = s;
      } else if (s < nearest_stay_object_s) {
        cipv_object_info.second_nearest_stay_object_id = nearest_stay_id;
        second_nearest_stay_object_s = nearest_stay_object_s;
        cipv_object_info.nearest_stay_object_id = object_id;
        nearest_stay_object_s = s;
      }
    }
    if (!cipv_object_info.nearest_object_id.has_value() ||
        s < nearest_object_s) {
      cipv_object_info.nearest_object_id = object_id;
      nearest_object_s = s;
    } else if (s > nearest_stay_object_s &&
               (!cipv_object_info.second_nearest_stay_object_id.has_value() ||
                s < second_nearest_stay_object_s)) {
      cipv_object_info.second_nearest_stay_object_id = object_id;
      second_nearest_stay_object_s = s;
    }
    cipv_object_info.object_distance_map[object_id] = s;
  }

  const auto& cipv = cipv_object_info.nearest_object_id;
  const auto& stay_cipv = cipv_object_info.nearest_stay_object_id;
  if (cipv.has_value() || stay_cipv.has_value()) {
    for (auto& st_boundary_wd : *st_boundaries_wd) {
      const auto& object_id = st_boundary_wd.object_id();
      if (!object_id.has_value()) continue;
      if (cipv.has_value() && *cipv == *object_id) {
        st_boundary_wd.set_is_cipv(true);
      }
      if (stay_cipv.has_value() && *stay_cipv == *object_id) {
        st_boundary_wd.set_is_stay_cipv(true);
      }
    }
  }

  return cipv_object_info;
}

std::vector<std::string> MakeCandidateProfileDebugInfo(
    const CandidateProfile& candidate_profile_proto) {
  std::vector<std::string> debug_info_list;
  const auto& interaction_costs = candidate_profile_proto.interaction_costs();
  int kDebugInfoNum = 2 * interaction_costs.interaction_cost_size() + 3;
  debug_info_list.reserve(kDebugInfoNum);
  auto push = [&debug_info_list](const auto& name, const auto& value) {
    debug_info_list.push_back(absl::StrFormat("%s:%s", name, value));
  };
  auto to_str = [](double val) { return absl::StrFormat("%.3f", val); };
  push("total_cost", to_str(candidate_profile_proto.total_cost()));
  push("decision_changed_cost",
       to_str(candidate_profile_proto.decision_changed_cost()));
  push("consistency_cost", to_str(candidate_profile_proto.consistency_cost()));
  for (const auto& interaction_cost : interaction_costs.interaction_cost()) {
    if (interaction_cost.has_id()) {
      push(absl::StrCat("interaction_cost_", interaction_cost.id()),
           to_str(interaction_cost.cost()));
      push(absl::StrCat("interaction_result_", interaction_cost.id()),
           interaction_cost.interactive_result());
    }
  }
  return debug_info_list;
}

void DumpInteractiveSetDebugInfoToDebugFrame(
    const InteractiveSpeedDebugProto& interactive_speed_debug, int plan_id,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision) {
  std::vector<std::string> interactive_speed_debug_details;
  interactive_speed_debug_details.reserve(2);

  // interactive_set
  std::string interactive_set_str = "[interactive_set]:";
  const auto& interactive_set_ids =
      interactive_speed_debug.interactive_set().id();
  for (int i = 0; i < interactive_set_ids.size(); ++i) {
    if (i > 0) {
      interactive_set_str += ", ";
    }
    interactive_set_str += interactive_set_ids[i];
  }
  interactive_speed_debug_details.emplace_back(interactive_set_str);

  // interactive_result
  interactive_speed_debug_details.emplace_back(absl::StrFormat(
      "[interactive_result]: is_non_interactive:%d, reason:%s",
      interactive_speed_debug.interactive_result().is_non_interactive(),
      interactive_speed_debug.interactive_result().reason()));

  Log2DDS::LogDataV0(
      absl::StrCat(Log2DDS::TaskPrefix(plan_id), "interactive_debug"),
      interactive_speed_debug_details);

  // st_boundary interactive_info
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    const auto& st_boundary = st_boundary_with_decision.st_boundary();
    const auto& non_interactive_set =
        interactive_speed_debug.non_interactive_set();
    bool isInteractive = true;
    std::string reason = "None";
    if (non_interactive_set.st_boundary_size() > 0) {
      for (const auto& non_interactive_st : non_interactive_set.st_boundary()) {
        std::string non_interactive_id = non_interactive_st.id();
        std::string st_boundary_id = st_boundary->id();

        // remove suffixes (e.g., |raw, |m).
        auto non_interactive_parts = absl::StrSplit(non_interactive_id, '|');
        auto st_boundary_parts = absl::StrSplit(st_boundary_id, '|');
        if (non_interactive_parts.begin() != non_interactive_parts.end() &&
            st_boundary_parts.begin() != st_boundary_parts.end()) {
          auto non_interactive_prefix = *non_interactive_parts.begin();
          auto st_boundary_prefix = *st_boundary_parts.begin();
          if (non_interactive_prefix == st_boundary_prefix) {
            isInteractive = false;
            reason = non_interactive_st.reason();
            break;
          }
        }
      }
    }
    std::vector<std::string> debug_info_list = {
        absl::StrFormat("%s:%s", "is_interactive",
                        isInteractive ? "true" : "false"),
        absl::StrFormat("%s:%s", "non_interactive_reason",
                        isInteractive ? "None" : reason)};
    Log2DDS::LogDataV2(
        Log2DDS::TaskPrefix(plan_id) + "stbinfo_" + st_boundary->id(),
        debug_info_list);
  }

  // interactive candidate speed_profile & cost
  const auto group_name = Log2DDS::TaskPrefix(plan_id) + "st_speed";
  if (interactive_speed_debug.has_candidate_profiles()) {
    const CandidateProfilesProto& candidate_profiles =
        interactive_speed_debug.candidate_profiles();

    for (int i = 0; i < candidate_profiles.candidate_profile_size(); ++i) {
      // speed_profile
      SpeedVector candidate_profile;
      const auto& candidate_profile_proto =
          candidate_profiles.candidate_profile(i).speed_profile();
      candidate_profile.reserve(candidate_profile_proto.speed_points_size());
      for (const auto& speed_pt : candidate_profile_proto.speed_points()) {
        candidate_profile.emplace_back().FromProto(speed_pt);
      }

      std::string curve_name = "candidate_profile_" + std::to_string(i);
      Log2DDS::LogChartV2(
          group_name, curve_name, Log2DDS::kLightGray, false, candidate_profile,
          [](const SpeedPoint& p) -> double { return p.t(); },
          [](const SpeedPoint& p) -> double { return p.s(); });

      // cost
      Log2DDS::LogDataV2(
          absl::StrCat(Log2DDS::TaskPrefix(plan_id), "candidate_cost_", i),
          MakeCandidateProfileDebugInfo(
              candidate_profiles.candidate_profile(i)));
    }
  }
}
}  // namespace planning
}  // namespace st
