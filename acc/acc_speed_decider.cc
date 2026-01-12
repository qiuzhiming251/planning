#include "acc/acc_speed_decider.h"

#include <functional>

#include "absl/status/status.h"
#include "plan_common/async/async_util.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/timer.h"
#include "plan_common/util/status_macros.h"
#include "planner/speed_optimizer/decider/pre_st_boundary_modifier.h"
#include "planner/speed_optimizer/interactive_speed_decision.h"
#include "planner/speed_optimizer/speed_finder_flags.h"
#include "planner/speed_optimizer/speed_finder_util.h"
#include "planner/speed_optimizer/st_graph.h"
#include "planner/speed_optimizer/st_overlap_analyzer.h"
#include "planner/speed_optimizer/standstill_distance_decider.h"
#include "planner/speed_optimizer/time_buffer_decider.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st::planning {
namespace {

constexpr double kCrossingThreshold = M_PI / 6.0;
constexpr double kSameDirAngleThreshold = M_PI / 6.0;
constexpr double kSameDirectionCutinTimeThreshold = 1.5;  // s.
constexpr double kPedestrianCutinTimeThreshold = 4.0;     // s.
constexpr double kCutinTimeThreshold = 2.5;               // s.
constexpr double kOncomingAngleThreshold = M_PI_2;
constexpr double kPedestrianOncomingThreshold = 0.5;  // s.
constexpr double kOncomingTimeThreshold = 0.2;        // s.
constexpr double kMaxOncomingRatio = 0.3;             // m
constexpr double kMinOncomingS = 4.0;                 // m
const PiecewiseLinearFunction<double, double> kMaxLatAccPlf(
    {5.0, 10.0, 15.0, 30.0}, {2.0, 1.8, 1.4, 1.2});
const PiecewiseLinearFunction<double> kMaxComfortDecelToSpeedDiff = {
    {1.0, 7.0}, {-0.2, -1.1}};

double ComputeFirstOverlapThetaDiff(const StBoundaryRef& st_boundary,
                                    const DiscretizedPath& path,
                                    const SpacetimeObjectTrajectory& st_traj) {
  const auto& overlap_infos = st_boundary->overlap_infos();
  CHECK(!overlap_infos.empty());
  const OverlapInfo& first_overlap_info = overlap_infos.front();
  const int first_overlap_path_index =
      (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2;
  const int first_overlap_traj_state_index = first_overlap_info.obj_idx;
  const double av_heading = path[first_overlap_path_index].theta();
  const double obj_heading =
      st_traj.states()[first_overlap_traj_state_index].traj_point->theta();
  return AngleDifference(obj_heading, av_heading);
}

StOverlapMetaProto AccAnalyzeStOverlap(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  StOverlapMetaProto overlap_meta;

  const double theta_diff =
      ComputeFirstOverlapThetaDiff(st_boundary, path, st_traj);
  const auto overlap_pattern = AnalyzeOverlapPattern(st_boundary, st_traj, path,
                                                     vehicle_geometry_params);
  overlap_meta.set_theta_diff(theta_diff);
  overlap_meta.set_pattern(overlap_pattern);

  if (overlap_meta.pattern() == StOverlapMetaProto::CROSS &&
      std::fabs(theta_diff) > kCrossingThreshold) {
    overlap_meta.set_source(StOverlapMetaProto::LANE_CROSS);
  } else {
    overlap_meta.set_source(StOverlapMetaProto::OBJECT_CUTIN);
  }
  overlap_meta.set_priority(StOverlapMetaProto::LOW);
  overlap_meta.set_priority_reason("ACC Cutin");
  const auto& object_type = st_boundary->object_type();
  if (object_type == StBoundaryProto::VEHICLE ||
      object_type == StBoundaryProto::CYCLIST) {
    overlap_meta.set_modification_type(StOverlapMetaProto::LON_MODIFIABLE);
  } else {
    overlap_meta.set_modification_type(StOverlapMetaProto::NON_MODIFIABLE);
  }

  return overlap_meta;
}

void RunAccAnalyzeStOverlaps(
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    std::vector<StBoundaryRef>* st_boundaries) {
  CHECK_NOTNULL(st_boundaries);
  for (StBoundaryRef& st_boundary : *st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) {
      continue;
    }
    CHECK(st_boundary->traj_id().has_value());
    const auto& traj_id = *st_boundary->traj_id();
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
    auto overlap_meta =
        AccAnalyzeStOverlap(st_boundary, *traj, path, vehicle_geometry_params);
    st_boundary->set_overlap_meta(std::move(overlap_meta));
  }
}

std::vector<StBoundaryRef> FilterStBoundariesByFirstOverlapTime(
    std::vector<StBoundaryRef> st_boundaries,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& path) {
  std::vector<StBoundaryRef> filtered_st_boundaries;
  for (auto& st_boundary : st_boundaries) {
    if (st_boundary->source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    double theta_diff = 0.0;
    const auto& overlap_meta = st_boundary->overlap_meta();
    if (overlap_meta.has_value()) {
      theta_diff = overlap_meta->theta_diff();
    } else {
      const auto* st_traj = CHECK_NOTNULL(
          st_traj_mgr.FindTrajectoryById(*st_boundary->traj_id()));
      theta_diff = ComputeFirstOverlapThetaDiff(st_boundary, path, *st_traj);
    }
    if (std::fabs(theta_diff) < kSameDirAngleThreshold &&
        st_boundary->min_t() > kSameDirectionCutinTimeThreshold) {
      continue;
    }
    const double min_t_thres =
        st_boundary->object_type() == StBoundaryProto::PEDESTRIAN
            ? kPedestrianCutinTimeThreshold
            : kCutinTimeThreshold;
    if (st_boundary->min_t() > min_t_thres) {
      continue;
    }

    if (std::fabs(theta_diff) > kOncomingAngleThreshold) {
      const double oncoming_min_t_thres =
          st_boundary->object_type() == StBoundaryProto::PEDESTRIAN
              ? kPedestrianOncomingThreshold
              : kOncomingTimeThreshold;
      if (st_boundary->min_t() > oncoming_min_t_thres) {
        continue;
      }
    }
    filtered_st_boundaries.push_back(std::move(st_boundary));
  }
  return filtered_st_boundaries;
}

// Avoid hard braking caused by oncoming object.
void ModifyReverseStBoundaries(std::vector<StBoundaryRef>* st_boundaries) {
  CHECK_NOTNULL(st_boundaries);
  for (StBoundaryRef& st_boundary : *st_boundaries) {
    auto upper_points = st_boundary->upper_points();
    auto lower_points = st_boundary->lower_points();
    double first_overlap_s = lower_points.front().s();
    double allowed_ds =
        -std::max(first_overlap_s * kMaxOncomingRatio, kMinOncomingS);
    std::vector<std::pair<StPoint, StPoint>> point_pairs;
    point_pairs.reserve(lower_points.size());
    bool st_boundary_modified = false;
    const double allowed_s = first_overlap_s + allowed_ds;
    for (int i = 0; i < lower_points.size(); ++i) {
      auto& lower_pt = upper_points[i];
      auto& upper_pt = upper_points[i];
      if (lower_pt.s() <= allowed_s) {
        const double upper_lower_diff = upper_pt.s() - lower_pt.s();
        lower_pt.set_s(allowed_s);
        upper_pt.set_s(lower_pt.s() + upper_lower_diff);
        st_boundary_modified = true;
      }
      point_pairs.emplace_back(lower_pt, upper_pt);
    }
    if (st_boundary_modified) {
      st_boundary->Init(point_pairs);
    }
  }
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

SpeedLimit ComputeLaneSpeedLimit(double user_speed_limit, bool is_standwait,
                                 double path_length, double max_speed_limit,
                                 const DiscretizedPath& path_points,
                                 double av_speed) {
  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
  constexpr double kExceedLimitThreshold = 0.5;  // m/s.
  const double av_speed_sqr = Sqr(av_speed - kExceedLimitThreshold);
  const double comfortable_decel =
      kMaxComfortDecelToSpeedDiff(av_speed - user_speed_limit);
  const double comfortable_brake_speed_sqr =
      av_speed_sqr + 2.0 * comfortable_decel * path_points[0].s();
  if (comfortable_brake_speed_sqr > Sqr(user_speed_limit)) {
    user_speed_limit = std::sqrt(comfortable_brake_speed_sqr);
  }

  speed_limit_ranges.push_back(
      {.start_s = 0.0,
       .end_s = path_length,
       .speed_limit =
           is_standwait ? 0.0 : std::min(user_speed_limit, max_speed_limit),
       .info = SpeedLimitTypeProto_Type_Name(SpeedLimitTypeProto_Type_LANE)});
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit ComputeCurvatureSpeedLimit(
    const DiscretizedPath& path, double av_speed, double max_speed_limit,
    double loaded_map_dist,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpeedFinderParamsProto& speed_finder_params) {
  const double max_allowed_kappa =
      ComputeCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params);

  const double av_speed_sqr = Sqr(av_speed);
  const auto& speed_limit_config = speed_finder_params.speed_limit_params();
  const double a = speed_limit_config.curvature_numerator();
  const double b = speed_limit_config.curvature_power();
  const double c = speed_limit_config.curvature_bias1();
  const double d = speed_limit_config.curvature_bias2();
  const auto compute_speed_limit = [&](double max_kappa, double s) {
    const double fabs_kappa = std::fabs(max_kappa);
    constexpr double kAccModeGain = 1.2;
    constexpr double kLoadedMapDistBuffer = 10.0;
    double speed_limit = 0.0;
    if (fabs_kappa > max_allowed_kappa) {
      speed_limit = speed_limit_config.speed_limit_for_large_curvature();
    } else if (s > loaded_map_dist - kLoadedMapDistBuffer) {
      speed_limit = max_speed_limit;
    } else {
      speed_limit = a / (std::pow(fabs_kappa, b) + c) + d;
    }
    speed_limit *= kAccModeGain;
    constexpr double kComfortableBrakeAcc = -1.5;  // m/ss.
    const double comfortable_brake_speed_sqr =
        av_speed_sqr + 2.0 * kComfortableBrakeAcc * s;
    if (comfortable_brake_speed_sqr > Sqr(speed_limit)) {
      speed_limit = std::sqrt(comfortable_brake_speed_sqr);
    }
    return FLAGS_planner_enable_acc_curvature_speed_limit
               ? std::min(speed_limit, max_speed_limit)
               : max_speed_limit;
  };
  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
  const double radius = speed_limit_config.max_curvature_consider_radius();
  const std::function<double(const PathPoint&)> get_kappa =
      [](const PathPoint& pt) { return pt.kappa(); };
  const auto max_kappas =
      GetMaxValuesNearbyPathPoint(path, radius, radius, get_kappa);
  constexpr double kApproxSpeedLimitEps = 0.5;
  const double init_speed_limit =
      compute_speed_limit(max_kappas.front(), path.front().s());
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path[0].s(), init_speed_limit);
  for (int i = 0; i < path.size(); ++i) {
    const double curr_speed_limit =
        compute_speed_limit(max_kappas[i], path[i].s());
    if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
            kApproxSpeedLimitEps ||
        i == path.size() - 1) {
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = path[i].s(),
           .speed_limit = prev_speed_limit_point.second,
           .info = SpeedLimitTypeProto::Type_Name(
               SpeedLimitTypeProto_Type_CURVATURE)});
      prev_speed_limit_point = std::make_pair(path[i].s(), curr_speed_limit);
    }
  }
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit GenerateCombinationSpeedLimit(
    const std::map<SpeedLimitTypeProto::Type, SpeedLimit>& speed_limit_map) {
  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
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

std::map<SpeedLimitTypeProto::Type, SpeedLimit> GenerateAccSpeedLimitMap(
    const DiscretizedPath& path, double av_speed, double user_speed_limit,
    double max_speed_limit, bool is_standwait, double loaded_map_dist,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SpeedFinderParamsProto& speed_finder_params) {
  std::map<SpeedLimitTypeProto::Type, SpeedLimit> speed_limit_map;
  auto lane_speed_limit =
      ComputeLaneSpeedLimit(user_speed_limit, is_standwait, path.length(),
                            max_speed_limit, path, av_speed);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_LANE,
                          std::move(lane_speed_limit));

  auto curvature_speed_limit = ComputeCurvatureSpeedLimit(
      path, av_speed, max_speed_limit, loaded_map_dist, vehicle_geometry_params,
      vehicle_drive_params, speed_finder_params);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_CURVATURE,
                          std::move(curvature_speed_limit));

  auto combination_speed_limit = GenerateCombinationSpeedLimit(speed_limit_map);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_COMBINATION,
                          std::move(combination_speed_limit));
  return speed_limit_map;
}

}  // namespace

absl::StatusOr<AccSpeedDeciderOutput> RunAccSpeedDecider(
    const AccSpeedDeciderInput& input) {
  Timer timer(__FUNCTION__);

  const auto& vehicle_geometry_params = *input.vehicle_geometry_params;
  const auto& vehicle_drive_params = *input.vehicle_drive_params;
  const auto& motion_constraint_params = *input.motion_constraint_params;
  const auto& speed_finder_params = *input.speed_finder_params;

  // Build st-graph.
  const auto av_shapes = BuildAvShapes(vehicle_geometry_params, *input.path);
  std::optional<PathApprox> path_approx_for_mirrors;
  PathApprox* path_approx_for_mirrors_ptr = nullptr;
  if (FLAGS_planner_use_path_approx_based_st_mapping) {
    path_approx_for_mirrors =
        BuildPathApproxForMirrors(*input.path_approx, vehicle_geometry_params);
    path_approx_for_mirrors_ptr = path_approx_for_mirrors.has_value()
                                      ? &(*path_approx_for_mirrors)
                                      : nullptr;
  }

  std::unique_ptr<StGraph> st_graph = std::make_unique<StGraph>(
      /*plan_id=*/0, input.path, input.trajectory_steps, input.plan_start_v,
      input.plan_start_a, motion_constraint_params.max_deceleration(),
      &vehicle_geometry_params, &speed_finder_params.st_graph_params(),
      &av_shapes, input.path_kd_tree, input.path_approx,
      path_approx_for_mirrors_ptr);

  ConstraintManager constraint_manager;  // not use.
  auto mapping_output = st_graph->GetStBoundaries(
      *input.traj_mgr, /*leading_objs=*/{},
      /*consider_lane_change_gap=*/false, constraint_manager,
      /*psman_mgr=*/nullptr,
      /*drive_passage=*/nullptr,
      /*path_sl_boundary=*/nullptr,
      /*nudge_object_info=*/nullptr,
      /*thread_pool=*/nullptr);
  auto st_boundaries = std::move(mapping_output.st_boundaries);

  // Filter and modify st-boundaries.
  st_boundaries = FilterStBoundariesByFirstOverlapTime(
      std::move(st_boundaries), *input.traj_mgr, *input.path);
  ModifyReverseStBoundaries(&st_boundaries);

  // Analyze overlap.
  RunAccAnalyzeStOverlaps(*input.path, *input.traj_mgr, vehicle_geometry_params,
                          &st_boundaries);

  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));

  // Set standstill and pass/yield time.
  absl::flat_hash_set<std::string> stalled_objects = {};
  const StandstillDistanceDeciderInput standstill_distance_decider_input{
      .speed_finder_params = &speed_finder_params,
      .stalled_object_ids = &stalled_objects,
      .planner_semantic_map_manager = nullptr,
      .lane_path = nullptr,
      .st_traj_mgr = input.traj_mgr,
      .constraint_mgr = &constraint_manager,
      .plan_start_v = input.plan_start_v};
  for (auto& st_boundary_wd : st_boundaries_with_decision) {
    DecideStandstillDistanceForStBoundary(standstill_distance_decider_input,
                                          &st_boundary_wd, /*opengap*/ false,
                                          /*cipv*/ nullptr);
  }

  // Set standstill and pass/yield time.
  absl::flat_hash_set<std::string> disable_pass_time_buffer_set;
  for (const auto& st_boundary_wd : st_boundaries_with_decision) {
    if (!st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }
    const auto& protected_st_boundary_id =
        st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
    if (!protected_st_boundary_id.has_value()) {
      continue;
    }
    switch (st_boundary_wd.raw_st_boundary()->protection_type()) {
      case StBoundaryProto::SMALL_ANGLE_CUT_IN:
      case StBoundaryProto::LANE_CHANGE_GAP: {
        disable_pass_time_buffer_set.insert(*protected_st_boundary_id);
        break;
      }
      case StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT:
      case StBoundaryProto::NON_PROTECTIVE:
        break;
    }
  }
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    const bool disable_pass_time_buffer = ContainsKey(
        disable_pass_time_buffer_set, st_boundary_with_decision.id());
    const auto* st_boundary = st_boundary_with_decision.raw_st_boundary();
    if (st_boundary->object_type() == StBoundaryProto::VEHICLE ||
        st_boundary->object_type() == StBoundaryProto::CYCLIST ||
        st_boundary->object_type() == StBoundaryProto::PEDESTRIAN) {
      DecideTimeBuffersForStBoundary(&st_boundary_with_decision,
                                     input.plan_start_v, *input.path,
                                     *input.vehicle_geometry_params,
                                     *input.traj_mgr, disable_pass_time_buffer);
    }
  }

  // Build speed limit provider.
  auto speed_limit_map = GenerateAccSpeedLimitMap(
      *input.path, input.plan_start_v, input.user_speed_limit,
      input.max_allowed_speed, input.is_standwait, input.loaded_map_dist,
      vehicle_geometry_params, vehicle_drive_params, speed_finder_params);
  SpeedLimitProvider speed_limit_provider(
      std::move(speed_limit_map), /*dynamic_speed_limit=*/{},
      /*vt_speed_limit_map=*/{}, kSpeedLimitProviderTimeStep);

  // Run pre st-boundary modifier.
  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_objs;
  const PreStboundaryModifierInput pre_st_boundary_modifier_input{
      .vehicle_geom = input.vehicle_geometry_params,
      .st_graph = st_graph.get(),
      .st_traj_mgr = input.traj_mgr,
      .current_v = input.plan_start_v,
      .current_a = input.plan_start_a,
      .path = input.path,
      .lat_modification_info_map = nullptr,
      .path_semantics = nullptr,
      .drive_passage = nullptr,
      .path_sl_boundary = nullptr,
      .leading_objs = &leading_objs};

  if (FLAGS_planner_enable_acc_pre_st_boundary_modifier) {
    OpenLoopSpeedLimit open_loop_speed_limit;
    PreModifyStBoundaries(pre_st_boundary_modifier_input,
                          &st_boundaries_with_decision, &processed_st_objects,
                          &open_loop_speed_limit);
  }

  // Run interactive speed decision.
  SpeedVector sampling_dp_speed;
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_obj;
  InteractiveSpeedDebugProto interactive_speed_debug;
  std::vector<DrivingProcess> driving_process_seq;
  double dist_to_merge = std::numeric_limits<double>::max();
  RETURN_IF_ERROR(MakeInteractiveSpeedDecision(
      input.base_name, *input.vehicle_geometry_params,
      *input.motion_constraint_params, *st_graph, *input.traj_mgr, *input.path,
      input.plan_start_v, input.plan_start_a, *input.speed_finder_params,
      input.user_speed_limit, input.trajectory_steps, leading_obj,
      /*nudge_object_info=*/nullptr, &speed_limit_provider, driving_process_seq,
      /*is_on_highway=*/false, dist_to_merge, &sampling_dp_speed,
      &st_boundaries_with_decision, &processed_st_objects,
      &interactive_speed_debug, /*thread_pool=*/nullptr,
      /*lc_stage=*/st::LaneChangeStage::LCS_NONE, /*follower_set=*/nullptr,
      /*active_speed_response_style=*/SPEED_RESPONSE_NORMAL));
  DestroyContainerAsyncMarkSource(std::move(st_graph), std::string{});

  return AccSpeedDeciderOutput{
      .speed_limit_provider = std::move(speed_limit_provider),
      .st_boundaries_with_decision = std::move(st_boundaries_with_decision),
      .sampling_dp_speed = std::move(sampling_dp_speed),
  };
}

}  // namespace st::planning
