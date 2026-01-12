

#include "decider/initializer/search_motion.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include "Eigen/Core"

#include <algorithm>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "plan_common/log.h"

#include "absl/time/time.h"
#include "absl/types/span.h"
#include "absl/time/clock.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer_config.pb.h"
#include "modules/msg/st_msgs/sm_behavior.pb.h"

#include "plan_common/timer.h"
#include "plan_common/gflags.h"
#include "plan_common/log_data.h"
#include "plan_common/drive_passage.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/async/async_util.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/util/time_util.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/math/vec.h"
#include "plan_common/math/util.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/planning_macros.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/planner_semantic_map_util.h"

#include "object_manager/planner_object.h"
#include "object_manager/st_inference/decider_input.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"

#include "decider/scheduler/path_boundary_builder.h"
#include "decider/decision_manager/decision_util.h"
#include "decider/decision_manager/leading_object.h"
#include "decider/decision_manager/constraint_builder.h"
#include "decider/decision_manager/leading_groups_builder.h"
#include "decider/initializer/initializer_util.h"
#include "decider/initializer/collision_checker.h"
#include "decider/initializer/motion_searcher.h"
#include "decider/initializer/reference_line_searcher.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "decider/initializer/geometry/geometry_form_builder.h"
#include "decider/initializer/geometry/geometry_graph_builder.h"
#include "decider/initializer/geometry/geometry_graph_cache.h"
#include "decider/initializer/obstacle_offset_decider.h"

namespace st::planning {
namespace {

constexpr double kHighestSpeedThreshold = 18.0;           // m/s ~ 60 km/h
constexpr double kHighSpeedThreshold = 14.0;              // m/s ~ 50 km/h.
constexpr double kMediumSpeedThreshold = 8.0;             // m/s ~ 30 km/h.
constexpr double kLowSpeedThreshold = 2.0;                // m/s ~ 7 km/h.
constexpr double kSpeedHysteresis = 1.0;                  // m/s.
constexpr double kStationaryObjectCollisionBuffer = 0.3;  // m on each side.
constexpr double kMovingObjectCollisionBuffer = 0.5;      // m on each side.
constexpr double kReduceStaticBufferThresholdS = 15.0;    // m.
constexpr double kMaxSamplingDistance = 350.0;            // meters.
constexpr double kMaxSamplingLookForwardTime = 15.0;      // seconds.
constexpr double kMinSamplingDistance = 60.0;
constexpr double kInitializerLKTimeConsumptionReportThreshold = 30;  // ms.
constexpr double kInitializerLCTimeConsumptionReportThreshold = 50;  // ms.

GeometryGraphSamplingStrategy ParseStrategy(
    const InitializerConfig::InitializerSamplePattern& sample_pattern,
    bool is_lane_change) {
  GeometryGraphSamplingStrategy strategy;

  strategy.is_lane_change = is_lane_change;
  const int config_len = sample_pattern.config().range().size();
  strategy.range_list.reserve(config_len);
  strategy.layer_gap_list.reserve(config_len);
  strategy.lateral_resolution_list.reserve(config_len);
  strategy.cross_layer_connection_list.reserve(config_len);
  strategy.unit_length_lateral_span_list.reserve(config_len);

  for (const auto& val : sample_pattern.config().range()) {
    strategy.range_list.push_back(val);
  }
  for (const auto& val : sample_pattern.config().layer_gap()) {
    strategy.layer_gap_list.push_back(val);
  }
  for (const auto& val : sample_pattern.config().lateral_resolution()) {
    strategy.lateral_resolution_list.push_back(val);
  }
  for (const auto& val : sample_pattern.config().cross_layer_connection()) {
    strategy.cross_layer_connection_list.push_back(val);
  }
  for (const auto& val : sample_pattern.config().unit_length_lateral_span()) {
    strategy.unit_length_lateral_span_list.push_back(val);
  }
  return strategy;
}

GeometryGraphSamplingStrategy FindStrategy(
    InitializerSamplePatternConfig sample_pattern_config,
    const InitializerConfig& initializer_config, bool is_lane_change) {
  const auto scenario =
      is_lane_change
          ? InitializerConfig::InitializerSamplePattern::SCENARIO_LANE_CHANGE
          : InitializerConfig::InitializerSamplePattern::SCENARIO_LANE_KEEPING;

  for (const auto& sample_pattern : initializer_config.sample_patterns()) {
    if (sample_pattern.config_name() == sample_pattern_config &&
        sample_pattern.scenario() == scenario) {
      return ParseStrategy(sample_pattern, is_lane_change);
    }
  }
  // Cannot find a sample pattern, qcheck failed.
  CHECK(false);
  return GeometryGraphSamplingStrategy();
}

InitializerSamplePatternConfig FindPattern(
    double cur_v, InitializerSamplePatternConfig prev_sample_config) {
  double highest_speed_threshold = kHighestSpeedThreshold;
  double high_speed_threshold = kHighSpeedThreshold;
  double medium_speed_threshold = kMediumSpeedThreshold;
  double low_speed_threshold = kLowSpeedThreshold;
  switch (prev_sample_config) {
    case InitializerSamplePatternConfig::ISC_NONE:
      break;
    case InitializerSamplePatternConfig::ISC_HIGH_SPEED:
      high_speed_threshold = high_speed_threshold - kSpeedHysteresis;
      break;
    case InitializerSamplePatternConfig::ISC_HIGHEST_SPEED:
      highest_speed_threshold = highest_speed_threshold - kSpeedHysteresis;
      break;
    case InitializerSamplePatternConfig::ISC_MEDIUM_SPEED:
      medium_speed_threshold = medium_speed_threshold - kSpeedHysteresis;
      break;
    case InitializerSamplePatternConfig::ISC_LOW_SPEED:
      break;
    case InitializerSamplePatternConfig::ISC_CREEP_SPEED:
      // Hysteresis in the opposite direction here.
      low_speed_threshold = low_speed_threshold + kSpeedHysteresis;
      break;
  }
  if (cur_v > highest_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_HIGHEST_SPEED;
  } else if (cur_v > high_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_HIGH_SPEED;
  } else if (cur_v > medium_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_MEDIUM_SPEED;
  } else if (cur_v > low_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_LOW_SPEED;
  } else {
    return InitializerSamplePatternConfig::ISC_CREEP_SPEED;
  }
}

// Get corresponding sample pattern based on current speed.
std::pair<InitializerSamplePatternConfig, GeometryGraphSamplingStrategy>
GetSamplingStrategy(const InitializerConfig& config, bool is_lane_change,
                    double cur_v,
                    InitializerSamplePatternConfig prev_sample_config) {
  InitializerSamplePatternConfig cur_pattern =
      FindPattern(cur_v, prev_sample_config);
  GeometryGraphSamplingStrategy strategy =
      FindStrategy(cur_pattern, config, is_lane_change);
  return std::make_pair(cur_pattern, std::move(strategy));
}

std::vector<double> ConvertStoplineToStopS(
    absl::Span<const ConstraintProto::StopLineProto> stoplines,
    double front_to_ra) {
  std::vector<double> stop_s;
  stop_s.reserve(stoplines.size());
  for (const auto& stop_line : stoplines) {
    stop_s.push_back(std::max(stop_line.s() - front_to_ra, 0.0));
  }
  return stop_s;
}

bool EnableInitializerLaneChangeTargetDecision(
    const LaneChangeStateProto& lc_state, const FrenetBox& av_frenet_box) {
  return (lc_state.stage() == LaneChangeStage::LCS_EXECUTING ||
          lc_state.stage() == LaneChangeStage::LCS_RETURN) &&
         !lc_state.entered_target_lane();
}

absl::Status CheckIsReturnScene(
    const InitializerInput& initializer_input, const std::string& prefix,
    const absl::flat_hash_set<std::string>& unsafe_object_ids,
    TaskSafetyEvaluationProto* task_safety_evaluation_result) {
  // check nullptr
  if (!task_safety_evaluation_result || !initializer_input.st_traj_mgr) {
    return absl::InvalidArgumentError("Nullptr check failed!");
  }

  task_safety_evaluation_result->set_task_is_init_return_scene(false);
  const auto& st_traj_mgr = *initializer_input.st_traj_mgr;

  // get previous info
  const auto& prev_return = initializer_input.pre_task_safety_evaluation_result
                                .task_is_init_return_scene();

  // for debug
  Log2DDS::LogDataV2(
      "return_scene",
      absl::StrCat(" prefix:", prefix, " prev_return:", prev_return));

  // check large vehicle
  for (const auto& traj : st_traj_mgr.trajectories()) {
    if (traj.planner_object().is_large_vehicle()) {
      const auto& obj_id = traj.planner_object().id();
      if (unsafe_object_ids.contains(obj_id)) {
        task_safety_evaluation_result->set_task_is_init_return_scene(true);
        Log2DDS::LogDataV2("return_scene",
                           absl::StrCat("Unsafe large vehicle! id:", obj_id));
        return absl::OkStatus();
      }
    }
  }

  return absl::NotFoundError("Not return scene!");
}

// same as traj_init, archived
// void DumpInitSpeedProfileToDebugFrame(const MotionSearchOutput&
// motion_output,
//                                       int plan_id) {
//   const auto& prefix = Log2DDS::TaskPrefix(plan_id);
//   auto name = prefix + "init_speed-profile";
//   const auto& traj = motion_output.traj_points;
//   const int size = traj.size();
//   std::vector<double> speed_profile_rt;
//   std::vector<double> speed_profile_v;
//   std::vector<double> speed_profile_a;
//   speed_profile_rt.reserve(size);
//   speed_profile_v.reserve(size);
//   speed_profile_a.reserve(size);
//   for (const auto& point : traj) {
//     speed_profile_rt.emplace_back(point.relative_time());
//     speed_profile_v.emplace_back(point.v());
//     speed_profile_a.emplace_back(point.a());
//   }
//   Log2DDS::LogDataV2(name + "_rt", std::move(speed_profile_rt));
//   Log2DDS::LogDataV2(name + "_v", std::move(speed_profile_v));
//   Log2DDS::LogDataV2(name + "_a", std::move(speed_profile_a));
// }
}  // namespace

absl::StatusOr<MotionSearchOutput> SearchMotion(const MotionSearchInput& input,
                                                ThreadPool* thread_pool,
                                                int plan_id) {
  TIMELINE("SearchMotion");
  absl::StatusOr<MotionSearchOutput> output_or;
  std::ostringstream lc_safety_debug;
  output_or = SearchForRawTrajectory(input, thread_pool, plan_id);
  lc_safety_debug.str("");
  lc_safety_debug << Log2DDS::TaskPrefix(plan_id)
                  << " SearchMotion result:" << output_or.ok()
                  << " msg:" << output_or.status().message();
  Log2DDS::LogDataV2("lc_safety", lc_safety_debug.str());
  return output_or;
}

// NOLINTNEXTLINE(readability-function-size)
absl::StatusOr<InitializerOutput> RunInitializer(
    const InitializerInput& initializer_input,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    SchedulerOutput* scheduler_output, DeciderOutput* decider_output,
    InitializerDebugProto* debug_proto, ThreadPool* thread_pool,
    std::map<std::string, bool>* obj_leading,
    PlannerStatusProto::PlannerStatusCode* lc_status_code,
    LaneChangeStyleDeciderResultProto* lc_style_decider_result,
    TaskSafetyEvaluationProto* task_safety_evaluation_result,
    int* scene_cones_riding_line_frames_result) {
  TIMELINE("RunInitializer");
  std::string function_name = Log2DDS::TaskPrefix(initializer_input.plan_id) +
                              std::string(__FUNCTION__);
  SCOPED_TRACE(function_name.c_str());
  const auto plan_time = initializer_input.path_start_point_info->plan_time;
  const auto& lane_change_state = *initializer_input.lane_change_state;
  const auto& prev_lane_change_stage = initializer_input.prev_lc_stage;

  const auto& drive_passage = *initializer_input.drive_passage;
  const auto& st_traj_mgr = *initializer_input.st_traj_mgr;
  const auto& vehicle_geom =
      initializer_input.vehicle_params->vehicle_geometry_params();
  const auto& vehicle_drive =
      initializer_input.vehicle_params->vehicle_drive_params();
  auto path_sl_boundary = *initializer_input.sl_boundary;
  const auto& initializer_state = *initializer_input.prev_initializer_state;
  const auto& decision_constraint_config =
      *initializer_input.decision_constraint_config;
  const auto& initializer_params = *initializer_input.initializer_params;
  const auto& motion_constraint_params =
      *initializer_input.motion_constraint_params;
  const auto plan_id = initializer_input.plan_id;
  const auto& stalled_objects = *initializer_input.stalled_objects;
  const auto& st_planner_object_traj =
      *initializer_input.st_planner_object_traj;
  const auto& planner_semantic_map_manager =
      *initializer_input.planner_semantic_map_manager;
  const auto& av_frenet_box = *initializer_input.av_frenet_box;
  ad_byd::planning::TrafficLightStatusMap temp_tl_status_map;
  const auto& tl_status_map =
      (nullptr == initializer_input.traffic_light_status_map)
          ? temp_tl_status_map
          : *initializer_input.traffic_light_status_map;

  //  ScopedMultiTimer initializer_timer("initializer_debug");
  //  initializer_timer.Mark("initializer start");

  auto mutable_start_point =
      initializer_input.path_start_point_info->start_point;  // Copy
  mutable_start_point.mutable_path_point()->set_theta(NormalizeAngle(
      mutable_start_point.path_point().theta()));  // Fix heading angle.
  const auto& path_start_point = mutable_start_point;

  const bool is_lane_change =
      (lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING ||
       lane_change_state.stage() == LaneChangeStage::LCS_RETURN ||
       lane_change_state.stage() == LaneChangeStage::LCS_PAUSE);

  // Create collision checker.
  // std::unique_ptr<CollisionChecker> collision_checker =
  //     std::make_unique<BoxGroupCollisionChecker>(
  //         &st_planner_object_traj, &vehicle_geom,
  //         MotionForm::kConstTimeIntervalSampleStep,
  //         kStationaryObjectCollisionBuffer, kMovingObjectCollisionBuffer);

  // // Should only happen if lc pushing
  // if (initializer_input.push_dir == PushDirection::Push_Normal_Left) {
  //   scheduler_output->lane_change_state.set_push_state(PushState::LEFT_PUSH);
  // } else if (initializer_input.push_dir == PushDirection::Push_Normal_Right)
  // {
  //   scheduler_output->lane_change_state.set_push_state(PushState::RIGHT_PUSH);
  // } else if (initializer_input.push_dir ==
  //            PushDirection::Push_Congestion_Left) {
  //   scheduler_output->lane_change_state.set_push_state(
  //       PushState::CONGESTION_LEFT_PUSH);
  // } else if (initializer_input.push_dir ==
  //            PushDirection::Push_Congestion_Right) {
  //   scheduler_output->lane_change_state.set_push_state(
  //       PushState::CONGESTION_RIGHT_PUSH);
  // } else {
  //   scheduler_output->lane_change_state.set_push_state(PushState::NONE_PUSH);
  // }
  // //
  // scheduler_output->lane_change_state.set_push_state(PushState::NONE_PUSH);
  // if (scheduler_output->lane_change_state.push_state() !=
  //     PushState::NONE_PUSH) {
  //   // Modify scheduler output for lc pushing.
  //   ASSIGN_OR_RETURN(
  //       scheduler_output->sl_boundary,
  //       BuildPathBoundaryFromPose(
  //           planner_semantic_map_manager, drive_passage,
  //           initializer_input.start_point_info->start_point, vehicle_geom,
  //           st_traj_mgr, scheduler_output->lane_change_state,
  //           *initializer_input.smooth_result_map,
  //           scheduler_output->borrow_lane, scheduler_output->should_smooth),
  //       _ << " Lc_pushing Rebuilding path boundary failed.");
  //   //  initializer sl_boundary
  //   path_sl_boundary = scheduler_output->sl_boundary;
  // }

  const InitializerSamplePatternConfig prev_config =
      initializer_state.has_sample_pattern_config()
          ? initializer_state.sample_pattern_config()
          : InitializerSamplePatternConfig::ISC_NONE;
  const auto [sample_pattern, sample_strategy] = GetSamplingStrategy(
      initializer_params, is_lane_change, path_start_point.v(), prev_config);
  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(path_start_point);
  ASSIGN_OR_RETURN(const auto ego_sl,
                   drive_passage.QueryFrenetCoordinateAt(ego_pos),
                   _ << "Failed to project ego position on drive passage.");

  debug_proto->set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));

  // Get the speed_limit
  double speed_limit = 0.0;
  for (const auto& station : drive_passage.stations()) {
    speed_limit += station.speed_limit();
  }
  speed_limit /= std::max(1, drive_passage.stations().size());
  const double passage_speed_limit =
      std::fmax(speed_limit, kMinSpeedForFinalCost);

  // Get smooth reference line max length.
  // const double speed_dist =
  //     path_start_point.v() > passage_speed_limit
  //         ? path_start_point.v() * kMaxSamplingLookForwardTime
  //         : (passage_speed_limit - path_start_point.v() >
  //                    kMaxSamplingLookForwardTime
  //                ? path_start_point.v() * kMaxSamplingLookForwardTime +
  //                      0.5 * kMaxSamplingLookForwardTime *
  //                          kMaxSamplingLookForwardTime
  //                : 0.5 * (passage_speed_limit * passage_speed_limit -
  //                         path_start_point.v() * path_start_point.v()) +
  //                      passage_speed_limit *
  //                          (kMaxSamplingLookForwardTime - passage_speed_limit
  //                          +
  //                           path_start_point.v()));
  const double sampling_dist_by_speed =
      std::max(std::max(path_start_point.v(), passage_speed_limit) *
                   kMaxSamplingLookForwardTime,
               kMinSamplingDistance);
  const double max_sampling_acc_s = std::min(
      drive_passage.end_s(),
      std::min(kMaxSamplingDistance, sampling_dist_by_speed) + ego_sl.s);

  // Get s_from_start.
  double s_from_start = 0.0;
  if (!initializer_input.path_start_point_info->reset) {
    // On async mode, `prev_start_point` is not the actual prev pos, so skip.
    if (initializer_state.has_s_from_start() &&
        initializer_state.has_prev_start_point()) {
      const auto prev_pos = Vec2dFromApolloTrajectoryPointProto(
          initializer_state.prev_start_point());
      ASSIGN_OR_RETURN(const auto prev_sl,
                       drive_passage.QueryFrenetCoordinateAt(prev_pos),
                       _ << "Failed to project previous ego position ("
                         << prev_pos.transpose() << ") on drive passage.");
      s_from_start = initializer_state.s_from_start() - prev_sl.s + ego_sl.s;
    }
  }
  InitializerStateProto new_state;
  new_state.set_sample_pattern_config(sample_pattern);
  new_state.set_s_from_start(s_from_start);
  *new_state.mutable_prev_start_point() = path_start_point;

  XYGeometryGraph geom_graph;
  auto graph_cache = std::make_unique<GeometryGraphCache>();
  const double s_from_start_with_diff = s_from_start - ego_sl.s;
  // Not const, to be destroyed asynchronously.
  auto form_builder = std::make_unique<GeometryFormBuilder>(
      &drive_passage, max_sampling_acc_s, s_from_start_with_diff);

  const auto stop_s_vec =
      ConvertStoplineToStopS(decider_output->constraint_manager.StopLine(),
                             vehicle_geom.front_edge_to_center());
  const double nearest_stop_s = stop_s_vec.empty() ? 200 : stop_s_vec.front();

  // Leading decider
  const bool enable_lc_multi_traj = EnableInitializerLaneChangeTargetDecision(
      lane_change_state, av_frenet_box);
  std::vector<LeadingGroup> leading_groups;
  bool is_first_lead = false;
  bool must_borrow = false;
  if (enable_lc_multi_traj) {
    // Generate multiple leading group candidates for initializer's
    // multiple trajectories.
    VLOG(3) << "lc_multiple_traj true: constructing leading object groups";
    const double path_start_time_offset = absl::ToDoubleSeconds(
        initializer_input.path_start_point_info->plan_time -
        initializer_input.start_point_info->plan_time);
    leading_groups = FindMultipleLeadingGroups(
        drive_passage, path_sl_boundary, lane_change_state.lc_left(),
        st_traj_mgr, stalled_objects, path_start_point.path_point().theta(),
        av_frenet_box, vehicle_geom, planner_semantic_map_manager,
        path_start_time_offset);
    if (!leading_groups.empty()) {
      leading_groups.resize(1);
      scheduler_output->leading_id = leading_groups.front().begin()->first;
    } else {
      leading_groups.push_back({});  // Take no trajectory as leading.
    }
  } else {
    // Do normal leading objects extraction and generate one single group.
    auto leading_trajs = FindLeadingObjects(
        tl_status_map, planner_semantic_map_manager, drive_passage,
        path_sl_boundary, lane_change_state.stage(),
        *initializer_input.scene_reasoning, st_traj_mgr, stalled_objects,
        path_start_point, vehicle_geom, av_frenet_box,
        initializer_input.borrow_lane, initializer_input.obs_history,
        *obj_leading, initializer_input.nudge_object_info, is_lane_change,
        nearest_stop_s, &is_first_lead, must_borrow);
    auto& traj_group = leading_groups.emplace_back();
    for (auto& leading_traj : leading_trajs) {
      traj_group.emplace(leading_traj.traj_id(), std::move(leading_traj));
    }
  }

  // viz obj
  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  auto name = prefix + "init_obj";
  std::stringstream obj_ss;
  obj_ss << "lc:" << enable_lc_multi_traj << leading_groups.size()
         << " stalled:";
  for (const auto& obj : stalled_objects) {
    obj_ss << obj << ",";
  }
  obj_ss << " leading:";
  for (const auto& g : leading_groups) {
    for (const auto& t : g) {
      obj_ss << t.first << "-" << t.second.reason() << ",";
    }
    obj_ss << ";";
  }
  obj_ss << "stopline-(" << absl::StrCat(ego_sl.s).substr(0, 4) << ","
         << absl::StrCat(passage_speed_limit).substr(0, 4) << "):";
  for (const auto& ss : stop_s_vec) {
    obj_ss << absl::StrCat(ss).substr(0, 4) << ",";
  }
  obj_ss << "traffic:";
  for (const auto& traffic_waiting_queue :
       initializer_input.scene_reasoning->traffic_waiting_queue()) {
    for (const auto& object_id : traffic_waiting_queue.object_id()) {
      obj_ss << object_id << ",";
    }
    obj_ss << ";";
  }
  Log2DDS::LogDataV2(name, obj_ss.str());

  // Create collision checker.
  std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_planner_object_traj, &vehicle_geom,
          MotionForm::kConstTimeIntervalSampleStep,
          kStationaryObjectCollisionBuffer, kMovingObjectCollisionBuffer,
          path_start_point, must_borrow);

  // Translate center line decider
  LargeVehicleAvoidStateProto cur_large_vehicle_avoid_state;
  if ((initializer_input.behavior->function_id() ==
           Behavior_FunctionId::Behavior_FunctionId_HW_NOA ||
       initializer_input.behavior->function_id() ==
           Behavior_FunctionId::Behavior_FunctionId_LKA) &&
      FLAGS_planner_enable_large_vehicle_avoid) {
    ObstacleOffsetDecider obstacle_offset_decider;  // Create an instance
    bool target_switched = true;
    if (initializer_input.prev_target_lane_path_from_start != nullptr) {
      target_switched =
          initializer_input.prev_target_lane_path_from_start->front()
              .lane_id() != drive_passage.lane_path().front().lane_id();
    }
    if (!target_switched &&
        (initializer_input.pre_large_vehicle_avoid_state != nullptr)) {
      cur_large_vehicle_avoid_state =
          *initializer_input.pre_large_vehicle_avoid_state;
    }
    if (!target_switched) {
      const Box2d ego_box = ComputeAvBox(
          ego_pos, path_start_point.path_point().theta(), vehicle_geom);
      const auto ego_frenet_box = drive_passage.QueryFrenetBoxAt(ego_box);
      if (ego_frenet_box.ok()) {
        auto res_or = obstacle_offset_decider.OffsetDecide(
            drive_passage, st_planner_object_traj, st_traj_mgr, vehicle_geom,
            leading_groups, ego_sl, ego_frenet_box.value(), path_start_point,
            &cur_large_vehicle_avoid_state, &path_sl_boundary,
            initializer_input.prev_lc_stage,
            initializer_input.lane_change_state->lc_left(),
            initializer_input.obs_history);
        if (res_or.ok() && res_or.value() == true) {
          scheduler_output->sl_boundary = path_sl_boundary;
        }
      }
    }
  }
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &drive_passage,
      .sl_boundary = &path_sl_boundary,
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &path_start_point,
      .s_from_start = s_from_start,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sample_strategy,
      .vehicle_drive = &vehicle_drive,
      .form_builder = form_builder.get(),
      .lc_multiple_traj = enable_lc_multi_traj};
  ASSIGN_OR_RETURN(
      geom_graph,
      BuildCurvyGeometryGraph(geom_graph_builder_input,
                              /*retry_collision_checker=*/false,
                              graph_cache.get(), thread_pool, debug_proto),
      [&]() {
        Log2DDS::LogDataV2(
            absl::StrCat(prefix, "init_msg"),
            absl::StrCat("1:", _.JoinMessageToStatus().message()));
        return MakeAebInitializerOutput(
            std::move(mutable_start_point), std::move(new_state),
            std::string(_.JoinMessageToStatus().message()), debug_proto);
      }());

  // If the constructed graph is short & blocked by static obj, we try a
  // smaller buffer.
  const auto& geom_end_info = geom_graph.GetGeometryGraphEndInfo();
  if (geom_end_info.end_reason() == GeometryGraphProto::END_STATIC_OBJ &&
      geom_end_info.end_accumulated_s() - ego_sl.s <
          kReduceStaticBufferThresholdS) {
    VLOG(2) << "Failed to construct graph, try a smaller stationary object "
               "buffer.";
    collision_checker->UpdateStationaryObjectBuffer(
        0.5 * kStationaryObjectCollisionBuffer);
    ASSIGN_OR_RETURN(
        geom_graph,
        BuildCurvyGeometryGraph(geom_graph_builder_input,
                                /*retry_collision_checker=*/true,
                                graph_cache.get(), thread_pool, debug_proto),
        [&]() {
          Log2DDS::LogDataV2(
              absl::StrCat(prefix, "init_msg"),
              absl::StrCat("2:", _.JoinMessageToStatus().message()));
          return MakeAebInitializerOutput(
              std::move(mutable_start_point), std::move(new_state),
              std::string(_.JoinMessageToStatus().message()), debug_proto);
        }());
  }
  // initializer_timer.Mark("build curvy xy geometry graph");
  VLOG(3) << "Done build geometry graph";

  // Add blocking object stop line to constraint manager if the geometry graph
  // is blocked by some object.
  std::unique_ptr<ConstraintProto::LeadingObjectProto> blocking_static_traj =
      nullptr;
  if (geom_end_info.end_reason() == GeometryGraphProto::END_STATIC_OBJ &&
      !stalled_objects.contains(geom_end_info.object_id())) {
    const auto blocking_frenet_box_or = drive_passage.QueryFrenetBoxAt(
        st_traj_mgr.FindObjectByObjectId(geom_end_info.object_id())
            ->bounding_box());
    if (blocking_frenet_box_or.ok() &&
        blocking_frenet_box_or->s_min > av_frenet_box.s_max) {
      const auto trajs =
          st_traj_mgr.FindTrajectoriesByObjectId(geom_end_info.object_id());
      CHECK_GT(trajs.size(), 0);
      blocking_static_traj =
          std::make_unique<ConstraintProto::LeadingObjectProto>(
              CreateLeadingObject(
                  *trajs[0], drive_passage,
                  ConstraintProto::LeadingObjectProto::BLOCKING_STATIC));
    }
  }
  if (nullptr != blocking_static_traj) {
    Log2DDS::LogDataV2(
        name, absl::StrCat("static", blocking_static_traj->traj_id(), "-",
                           blocking_static_traj->reason()));
  }

  auto* graph_proto = debug_proto->mutable_geom_graph();
  graph_proto->Clear();

  if (FLAGS_planner_initializer_only_activate_nodes_near_capnet_traj) {
    // RETURN_IF_ERROR(DeactivateFarGeometries(
    //     initializer_input.captain_net_output->traj_points, path_sl_boundary,
    //     &geom_graph));
  } else if (!is_lane_change &&
             FLAGS_planner_initializer_only_activate_nodes_near_refline) {
    TIMELINE("SearchReferenceLine");
    ReferenceLineSearcherInput ref_line_search_input{
        .geometry_graph = &geom_graph,
        .drive_passage = &drive_passage,
        .sl_boundary = &path_sl_boundary,
        .initializer_params = &initializer_params,
        .vehicle_geom = &vehicle_geom,
        .vehicle_drive = &vehicle_drive,
        .st_planner_object_traj = &st_planner_object_traj,
    };
    const auto ref_line_output_or =
        SearchReferenceLine(ref_line_search_input, debug_proto, thread_pool);

    if (ref_line_output_or.ok()) {
      RETURN_IF_ERROR(DeactivateFarGeometries(*ref_line_output_or,
                                              path_sl_boundary, &geom_graph));
      if (FLAGS_planner_initializer_debug_level >= 1) {
        ParseReferenceLineResultToProto(*ref_line_output_or, graph_proto);
      }
    }
    // initializer_timer.Mark("search reference line");
  }

  if (FLAGS_planner_initializer_debug_level >= 1) {
    geom_graph.ToProto(graph_proto);
    form_builder->FillSmoothDrivePassage(graph_proto);
    for (const auto& edge : graph_proto->edges()) {
      std::vector<double> xss, yss;
      xss.reserve(edge.states_size());
      yss.reserve(edge.states_size());
      for (const auto& state : edge.states()) {
        xss.emplace_back(state.x());
        yss.emplace_back(state.y());
      }
      Log2DDS::LogLineV3(absl::StrCat(prefix, "init-graph"), Log2DDS::kOrange,
                         {}, xss, yss, 1.0);
    }

    std::vector<double> xs, ys;
    xs.reserve(graph_proto->nodes_size());
    ys.reserve(graph_proto->nodes_size());
    for (const auto& pt : graph_proto->nodes()) {
      xs.emplace_back(pt.x());
      ys.emplace_back(pt.y());
    }
    Log2DDS::LogPointsV3(absl::StrCat(prefix, "init-graph"), Log2DDS::kGreen,
                         {}, xs, ys, 10.0);
  }

  const bool eval_safety =
      !FLAGS_planner_est_scheduler_seperate_lc_pause && is_lane_change &&
      lane_change_state.stage() != LaneChangeStage::LCS_PAUSE &&
      !(lane_change_state.force_merge() ||
        lane_change_state.entered_target_lane());
  // viz safety info
  auto safety_name = prefix + "safety";

  std::string safety_info =
      absl::StrCat("check-lc-merge-enter-lead:", eval_safety, is_lane_change,
                   lane_change_state.force_merge(),
                   lane_change_state.entered_target_lane(), is_first_lead);
  Log2DDS::LogDataV2(safety_name, safety_info);
  // const ml::captain_net::CaptainNetOutput empty_captain_net_output;
  InitializerSceneType init_scene_type = InitializerSceneType::INIT_SCENE_NONE;
  if (is_lane_change) {
    init_scene_type = InitializerSceneType::INIT_SCENE_LANE_CHANGE;
  } else if (is_first_lead) {
    init_scene_type = InitializerSceneType::INIT_SCENE_FOLLOW;
  } else {
    init_scene_type = initializer_input.borrow_lane
                          ? InitializerSceneType::INIT_SCENE_BORROW
                          : InitializerSceneType::INIT_SCENE_NUDGE;
  }

  MotionSearchInput motion_search_input{
      .planner_semantic_map_manager = &planner_semantic_map_manager,
      .start_point = &path_start_point,
      .ego_sl = ego_sl,
      .path_look_ahead_duration = initializer_input.path_look_ahead_duration,
      .plan_time = plan_time,
      .drive_passage = &drive_passage,
      .sl_boundary = &path_sl_boundary,
      .st_traj_mgr = &st_traj_mgr,
      .st_planner_object_traj = &st_planner_object_traj,
      .initializer_params = &initializer_params,
      .motion_constraint_params = &motion_constraint_params,
      .vehicle_params = initializer_input.vehicle_params,
      .geom_graph = &geom_graph,
      .form_builder = form_builder.get(),
      .collision_checker = collision_checker.get(),
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .blocking_static_traj = blocking_static_traj.get(),
      .stalled_objects = &stalled_objects,
      .obs_history = initializer_input.obs_history,
      .standard_congestion_factor =
          scheduler_output->standard_congestion_factor,
      .traffic_congestion_factor = scheduler_output->traffic_congestion_factor,
      // .captain_net_output =
      //     FLAGS_planner_use_ml_trajectory_as_initializer_ref_traj
      //         ? initializer_input.captain_net_output
      //         : &empty_captain_net_output,
      .passage_speed_limit = passage_speed_limit,
      .init_scene_type = init_scene_type,
      .is_lane_change = is_lane_change,
      .eval_safety = eval_safety,
      .lc_style = initializer_input.lane_change_style,
      //.log_av_trajectory = initializer_input.log_av_trajectory,
      .lc_state = lane_change_state.stage(),
      .lc_left = lane_change_state.lc_left(),
      .prev_lc_stage = prev_lane_change_stage,
      .push_dir = initializer_input.push_dir,
      .borrow_lane = initializer_input.borrow_lane,
      .pre_lc_style_decider_result =
          initializer_input.pre_lc_style_decider_result,
      .pre_task_safety_evaluation_result =
          initializer_input.pre_task_safety_evaluation_result,
      .pre_scene_cones_riding_line_frames_result =
          initializer_input.pre_scene_cones_riding_line_frames_result};
  // initializer_timer.Mark("create initializer input");

  ASSIGN_OR_RETURN(
      auto motion_output,
      SearchMotion(motion_search_input, thread_pool, plan_id), [&]() {
        Log2DDS::LogDataV2(
            absl::StrCat(prefix, "init_msg"),
            absl::StrCat("3:", _.JoinMessageToStatus().message()));
        return MakeAebInitializerOutput(
            std::move(mutable_start_point), std::move(new_state),
            std::string(_.JoinMessageToStatus().message()), debug_proto);
      }());
  if (!motion_output.result_status.ok()) {
    return absl::CancelledError(
        absl::StrCat("Motion search output invalid: ",
                     motion_output.result_status.message()));
  }
  if (lc_style_decider_result != nullptr) {
    *lc_style_decider_result = motion_output.lc_style_decider_result;
  }
  if (task_safety_evaluation_result != nullptr) {
    *task_safety_evaluation_result =
        motion_output.task_safety_evaluation_result;
  }
  if (scene_cones_riding_line_frames_result != nullptr) {
    *scene_cones_riding_line_frames_result =
        motion_output.scene_cones_riding_line_frames_result;
  }

  std::string lc_lead_obj_id = "none";
  std::string lc_lead_info = "info:";
  if (enable_lc_multi_traj) {
    double min_traj_cost = std::numeric_limits<double>::max();
    for (const auto& traj : motion_output.multi_traj_candidates) {
      if (traj.total_cost < min_traj_cost - 1.0) {
        min_traj_cost = traj.total_cost;
        lc_lead_obj_id =
            traj.leading_traj_ids.empty() ? "" : traj.leading_traj_ids.front();
      }
      lc_lead_info += absl::StrCat(
          "(",
          (traj.leading_traj_ids.empty() ? "" : traj.leading_traj_ids.front()),
          ":", absl::StrCat(traj.total_cost).substr(0, 5), ")");
    }
    if ("none" != lc_lead_obj_id && "" != lc_lead_obj_id) {
      const auto found = lc_lead_obj_id.find("-idx");
      lc_lead_obj_id =
          (std::string::npos == found) ? "" : lc_lead_obj_id.substr(0, found);
    }
  }
  Log2DDS::LogDataV2(
      safety_name, absl::StrCat("lc_lead:", lc_lead_obj_id, ",", lc_lead_info));

  // Should only happen if lc safety check has been applied.
  PausePushSavedOffsetProto cur_saved_offset;
  if (motion_output.is_lc_pause) {
    scheduler_output->lane_change_state.set_stage(LaneChangeStage::LCS_PAUSE);
    const auto start_lane_id = drive_passage.lane_path().front().lane_id();
    *unsafe_object_ids = std::move(motion_output.unsafe_object_ids);
    std::string danger_obs = "danger: ";
    for (const auto& obs_str : *unsafe_object_ids) {
      danger_obs += obs_str;
    }
    Log2DDS::LogDataV2(safety_name, danger_obs);
    if (!(initializer_input.prev_target_lane_path_from_start->IsEmpty() ||
          initializer_input.prev_target_lane_path_from_start->front()
                  .lane_id() != start_lane_id)) {
      bool must_borrow = false;
      // Modify scheduler output for lc pause.
      if (initializer_input.saved_offset != nullptr) {
        cur_saved_offset.set_pre_pause_offset(
            initializer_input.saved_offset->pre_pause_offset());
        cur_saved_offset.set_pre_pause_scene(
            initializer_input.saved_offset->pre_pause_scene());
      }

      // 拥堵场景判断
      constexpr double kCongestionLowSpeedMpsThrd = 30.0 / 3.6;
      bool in_low_speed = path_start_point.v() < kCongestionLowSpeedMpsThrd;
      const bool is_congestion_scene =
          lc_style_decider_result != nullptr
              ? (lc_style_decider_result->congestion_scene() && in_low_speed)
              : false;

      ASSIGN_OR_RETURN(
          scheduler_output->sl_boundary,
          BuildPathBoundaryFromPose(
              planner_semantic_map_manager, drive_passage,
              initializer_input.start_point_info->start_point, vehicle_geom,
              st_traj_mgr, scheduler_output->lane_change_state,
              *initializer_input.smooth_result_map,
              scheduler_output->borrow_lane, scheduler_output->should_smooth,
              scheduler_output->lane_path_before_lc, &cur_saved_offset,
              unsafe_object_ids, is_congestion_scene,
              initializer_input.obs_history),
          _ << "Rebuilding path boundary failed.");

      // Find the corresponding leading trajectories.
      auto leading_trajs = FindLeadingObjects(
          tl_status_map, planner_semantic_map_manager, drive_passage,
          scheduler_output->sl_boundary,
          scheduler_output->lane_change_state.stage(),
          *initializer_input.scene_reasoning, st_traj_mgr, stalled_objects,
          path_start_point, vehicle_geom, av_frenet_box,
          scheduler_output->borrow_lane, initializer_input.obs_history,
          *obj_leading, initializer_input.nudge_object_info, is_lane_change,
          nearest_stop_s, nullptr, must_borrow);
      for (auto& leading_traj : leading_trajs) {
        motion_output.leading_trajs.emplace(leading_traj.traj_id(),
                                            std::move(leading_traj));
      }

      // TODO: Calculate route_target_info for lc_pause
      std::optional<RouteTargetInfo> route_target_info = std::nullopt;
      if (FLAGS_planner_enable_pause_gap_cal) {
        const Box2d ego_box = ComputeAvBox(
            ego_pos, path_start_point.path_point().theta(), vehicle_geom);

        route_target_info = [plan_id, &scheduler_output,
                             &planner_semantic_map_manager, &st_traj_mgr,
                             &ego_box]() -> std::optional<RouteTargetInfo> {
          const auto target_lane_path_ext = BackwardExtendLanePath(
              planner_semantic_map_manager,
              scheduler_output->drive_passage.extend_lane_path()
                  .BeforeArclength(kLaneChangeCheckForwardLength),
              kLaneChangeCheckBackwardLength);
          auto target_frenet_frame_or = BuildKdTreeFrenetFrame(
              SampleLanePathPoints(planner_semantic_map_manager,
                                   target_lane_path_ext),
              true);
          if (!target_frenet_frame_or.ok()) return std::nullopt;

          ASSIGN_OR_RETURN(const auto ego_frenet_box,
                           target_frenet_frame_or->QueryFrenetBoxAt(ego_box),
                           std::nullopt);

          return RouteTargetInfo{
              .plan_id = plan_id,
              .frenet_frame = std::move(target_frenet_frame_or).value(),
              .ego_frenet_box = ego_frenet_box,
              .drive_passage = scheduler_output->drive_passage,
              .sl_boundary = scheduler_output->sl_boundary,
              .st_traj_mgr = st_traj_mgr};
        }();
      }

      // Rerun constraint builder for lc pause case.
      DeciderInput decider_input{
          .plan_id = 99,
          .vehicle_geometry_params = &vehicle_geom,
          .motion_constraint_params = &motion_constraint_params,
          .config = &decision_constraint_config,
          .planner_semantic_map_manager = &planner_semantic_map_manager,
          .lc_state = &scheduler_output->lane_change_state,
          .plan_start_point = &initializer_input.start_point_info->start_point,
          .lane_path_before_lc = &scheduler_output->lane_path_before_lc,
          .passage = &drive_passage,
          .sl_boundary = &scheduler_output->sl_boundary,
          .borrow_lane_boundary = scheduler_output->borrow_lane,
          .obj_mgr = initializer_input.obj_mgr,
          .st_traj_mgr = &st_traj_mgr,
          // .tl_info_map = initializer_input.tl_info_map,
          .traffic_light_status_map =
              initializer_input.traffic_light_status_map,
          .pre_decider_state = initializer_input.prev_decider_state,
          // .parking_brake_release_time =
          //     initializer_input.parking_brake_release_time,
          //.teleop_enable_traffic_light_stop =
          //    initializer_input.enable_traffic_light_stopping,
          //.enable_pull_over = initializer_input.enable_pull_over,
          //.brake_to_stop = initializer_input.brake_to_stop,
          .max_reach_length = initializer_input.left_navi_dist,
          .lc_num = initializer_input.cur_lc_num,
          .leading_id = lc_lead_obj_id,
          // .vehicle_model =
          //     initializer_input.vehicle_params->vehicle_params().model(),
          .plan_time = initializer_input.start_point_info->plan_time,
          .route_target_info = route_target_info.has_value()
                                   ? &route_target_info.value()
                                   : nullptr,
          .scene_reasoning = initializer_input.scene_reasoning,
          .behavior = initializer_input.behavior,
          .speed_state = initializer_input.speed_state,
          .stalled_objects = initializer_input.stalled_objects};
      // Log2DDS::LogDataV2("pre_dec",
      // Log2DDS::TaskPrefix(decider_input.plan_id)
      // +
      //                                 "in search motion");
      // Log2DDS::LogDataV2("pre_dec",
      // Log2DDS::TaskPrefix(decider_input.plan_id)
      // +
      //                                 absl::StrCat("lc num",
      //                                 scheduler_output->lc_num));
      // Log2DDS::LogDataV2("pre_dec",
      // Log2DDS::TaskPrefix(decider_input.plan_id)
      // +
      //                                 absl::StrCat("max_reach_length",
      //                                 scheduler_output->max_reach_length));
      ASSIGN_OR_RETURN(auto lcp_decider_output, BuildConstraints(decider_input),
                       _ << "Rebuilding decision constraints failed.");
      *decider_output = std::move(lcp_decider_output);

      const double target_l =
          scheduler_output->sl_boundary.QueryReferenceCenterL(ego_sl.s);
      ASSIGN_OR_RETURN(auto lcp_traj,
                       GenerateConstLateralAccelConstSpeedTraj(
                           drive_passage, vehicle_geom.front_edge_to_center(),
                           target_l, motion_output.leading_trajs,
                           decider_output->constraint_manager.StopLine(),
                           path_start_point, kInitializerTrajectorySteps),
                       _ << "Generating lc pause trajectory failed.");
      motion_output.multi_traj_candidates.insert(
          motion_output.multi_traj_candidates.begin(),
          MotionSearchOutput::MultiTrajCandidate{.trajectory = lcp_traj});
      motion_output.traj_points = std::move(lcp_traj);

      // check is_return_scene
      const auto return_status =
          CheckIsReturnScene(initializer_input, prefix, *unsafe_object_ids,
                             task_safety_evaluation_result);
      if (!return_status.ok()) {
        Log2DDS::LogDataV2("return_scene",
                           absl::StrCat(" prefix:", prefix, " not ok! msg:",
                                        return_status.message()));
      }
    }
  }
  Log2DDS::LogDataV2(
      absl::StrCat(prefix, "lcs_stage"),
      LaneChangeStage_Name(scheduler_output->lane_change_state.stage()));

  ParseMotionSearchOutputToInitializerResult(motion_output, debug_proto);
  ParseMotionSearchOutputToMultiTrajDebugProto(
      motion_output, debug_proto->mutable_multi_traj_debug());
  if (FLAGS_planner_initializer_debug_level >= 1) {
    if (!motion_output.is_lc_pause) {
      ParseMotionSearchOutputToMotionSearchDebugProto(
          motion_output, debug_proto->mutable_motion_search_debug());
      const auto search_proto = debug_proto->motion_search_debug();
      std::vector<std::string> traj_costs;
      // std::vector<Log2DDS::Color> traj_colors{
      //     Log2DDS::kYellow, Log2DDS::kCoral,     Log2DDS::kOrange,
      //     Log2DDS::kPink,   Log2DDS::kPurple,    Log2DDS::kHotpink,
      //     Log2DDS::kWhite,  Log2DDS::kDarkkhaki, Log2DDS::kLime,
      //     Log2DDS::kAqua};
      std::vector<Log2DDS::Color> traj_colors{
          Log2DDS::kYellow, Log2DDS::kPink,   Log2DDS::kPink,
          Log2DDS::kPink,   Log2DDS::kPurple, Log2DDS::kPurple,
          Log2DDS::kPurple, Log2DDS::kPurple, Log2DDS::kPurple,
          Log2DDS::kPurple};
      int motion_idx = 0;
      // Log top k trajectories and their costs.
      for (int i = 0; i < search_proto.top_k_trajs().size(); i++) {
        if (3 < i && 0 != i % 9 && search_proto.top_k_trajs().size() - 1 != i) {
          continue;
        }
        const auto& traj_info = search_proto.top_k_trajs(i);
        std::string traj_cost = absl::StrCat(
            prefix, "init_traj_", i, ", total cost: ", traj_info.total_cost());
        for (int j = 0; j < search_proto.cost_names().size(); j++) {
          traj_cost += absl::StrCat(", ", search_proto.cost_names(j), ": ",
                                    traj_info.costs(j));
        }
        traj_costs.push_back(traj_cost);
        std::vector<double> xss, yss;
        xss.reserve(traj_info.traj_points().size());
        yss.reserve(traj_info.traj_points().size());
        for (const auto& point : traj_info.traj_points()) {
          xss.emplace_back(point.pos().x());
          yss.emplace_back(point.pos().y());
        }
        double motion_z = i == 0 ? 2.0 : 1.5;
        // Log2DDS::Color motion_color =
        //     i == 0 ? Log2DDS::Color(255, 182, 193, 1)
        //            : Log2DDS::Color(0, 0, 200 - 10 * i, 1);
        Log2DDS::Color motion_color =
            (search_proto.top_k_trajs().size() - 1 == i)
                ? Log2DDS::kTiffanyBlue
                : traj_colors[motion_idx % traj_colors.size()];
        Log2DDS::LogLineV3(absl::StrCat(prefix, "init-motion_", i),
                           motion_color, {}, xss, yss, motion_z);
        motion_idx++;
      }
      // viz the ref_speed_info
      std::string ref_speed_info = "ref_speed:";
      const std::array<double, 9> sample_times = {0.0, 0.9, 1.9, 2.9, 3.9,
                                                  4.9, 5.9, 6.9, 7.9};
      const RefSpeedTable& ref_speed_table = *motion_output.ref_speed_table;
      for (const double sample_time : sample_times) {
        for (const auto& point : motion_output.traj_points) {
          if (point.relative_time() > sample_time) {
            double sample_s = point.path_point().s();
            auto ref_v =
                ref_speed_table.LookUpRefSpeed(sample_time, sample_s).second;
            ref_speed_info +=
                absl::StrCat(absl::StrCat(ref_v).substr(0, 4), "/",
                             absl::StrCat(point.v()).substr(0, 4), ",");
            break;
          }
        }
      }
      traj_costs.push_back(ref_speed_info);
      Log2DDS::LogDataV3(absl::StrCat(prefix, "init_motion_cost"), traj_costs);
    } else {
      std::vector<double> xs, ys;
      xs.reserve(motion_output.traj_points.size());
      ys.reserve(motion_output.traj_points.size());
      for (const auto& point : motion_output.traj_points) {
        xs.push_back(point.path_point().x());
        ys.push_back(point.path_point().y());
      }
      Log2DDS::LogLineV3(absl::StrCat(prefix, "init-motion_pause"),
                         Log2DDS::kYellow, {}, xs, ys, 2.0);
    }
  }
  // DumpInitSpeedProfileToDebugFrame(motion_output, plan_id);

  DestroyContainerAsyncMarkSource(std::move(form_builder), "form_builder");
  DestroyContainerAsyncMarkSource(std::move(graph_cache),
                                  "geometry_graph_cache");

  if (FLAGS_planner_dumping_initializer_features) {
    ParseFeaturesDumpingProto(motion_output,
                              debug_proto->mutable_expert_evaluation(),
                              debug_proto->mutable_candidates_evaluation());
  }

  // Collect corresponding leading trajectory information to output.
  auto& leading_trajs = motion_output.leading_trajs;
  // Non-stalled blocking static should also be a leading trajectory.
  if (blocking_static_traj != nullptr &&
      leading_trajs.find(blocking_static_traj->traj_id()) ==
          leading_trajs.end()) {
    const auto obj_id = SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
        blocking_static_traj->traj_id());
    if (!stalled_objects.contains(obj_id)) {
      (*obj_leading)[obj_id] = true;
      leading_trajs.emplace(blocking_static_traj->traj_id(),
                            *blocking_static_traj);
    }
  }
  for (const auto& [_, leading_proto] : leading_trajs) {
    *debug_proto->add_leading_objects() = leading_proto;
  }

  return InitializerOutput{
      .follower_set = std::move(motion_output.follower_set),
      .leader_set = std::move(motion_output.leader_set),
      .follower_max_decel = motion_output.follower_max_decel,
      .is_lc_pause = motion_output.is_lc_pause,
      .traj_points = std::move(motion_output.traj_points),
      .initializer_state = std::move(new_state),
      .leading_trajs = std::move(leading_trajs),
      .nudge_info = std::move(motion_output.nudge_info),
      .speed_response_style = std::move(motion_output.speed_response_style),
      .lc_status_code = std::move(motion_output.lc_status_code),
      .is_init_follow_scene =
          (InitializerSceneType::INIT_SCENE_FOLLOW == init_scene_type),
      .lc_lead_obj_id = lc_lead_obj_id,
      .pre_large_vehicle_avoid_state = std::move(cur_large_vehicle_avoid_state),
      .saved_offset = std::move(cur_saved_offset),
      .gaming_lc_obs_set = std::move(motion_output.gaming_lc_obs_set),
  };
}

}  // namespace st::planning
