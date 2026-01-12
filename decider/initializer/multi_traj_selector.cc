

#include "decider/initializer/multi_traj_selector.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <ostream>
#include <utility>

#include <float.h>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "plan_common/async/parallel_for.h"
#include "plan_common/log_data.h"
#include "plan_common/log.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/vec.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/util/status_macros.h"

#include "decider/initializer/lane_change_style_decider.h"

namespace st::planning {

// Debounce for lc style decision
const uint64_t gLcStyleDebounceBuffer = 2;

// Congestion factor
const double kCongestionFactorThreshold = 2.0;
const double kCongestionFactorDebounceThreshold = 1.5;

const double kEgoLowSpeedMpsThreshold = 8.333;  // m/s. 30.0kph

namespace {

absl::Status EvaluateSingleTrajectory(const SingleTrajInfo& traj_info,
                                      TrajEvalInfo* eval_info) {
  eval_info->eval_cost = traj_info.total_cost;
  return absl::OkStatus();
}

absl::Status EvaluateSingleTrajectoryAndCheckSafety(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const PathSlBoundary& sl_boundary,
    const ApolloTrajectoryPointProto& start_point,
    const SingleTrajInfo& traj_info, const FrenetFrame& target_frenet_frame,
    double speed_limit, const SpacetimeTrajectoryManager& st_traj_mgr,
    const mapping::LanePath& target_lane_path_ext,
    const VehicleParamsProto& vehicle_params, LaneChangeStyle lc_style,
    const std::pair<PathResponseStyle, SpeedResponseStyle>& prev_resp_style,
    const st::LaneChangeStage& lc_state,
    const st::LaneChangeStage& prev_lc_stage, const bool is_congestion_scene,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ObjectHistoryManager& obs_history_mgr,
    absl::Duration path_look_ahead_duration, int plan_id,
    TrajEvalInfo* eval_info,
    LaneChangeStylePostDeciderSceneInfo* target_front_obj_scene_info,
    LaneChangeStylePostDeciderSceneInfos* scene_infos,
    int* scene_cones_riding_line_frames_result, bool is_closed_ramp,
    bool is_on_highway, absl::flat_hash_set<std::string>& gaming_lc_obs_set,
    const std::vector<double>* stop_s_vec = nullptr) {
  auto lc_safety_status = CheckLaneChangeSafety(
      psmm, drive_passage, sl_boundary, start_point, traj_info.traj_points,
      traj_info.leading_trajs, stalled_objects, target_frenet_frame,
      speed_limit, st_traj_mgr, obs_history_mgr, target_lane_path_ext,
      vehicle_params, lc_style, prev_resp_style, lc_state, prev_lc_stage,
      is_congestion_scene, path_look_ahead_duration, plan_id, eval_info,
      target_front_obj_scene_info, scene_infos,
      scene_cones_riding_line_frames_result, gaming_lc_obs_set, is_closed_ramp,
      is_on_highway, stop_s_vec);
  eval_info->eval_cost = traj_info.total_cost;

  return lc_safety_status;
}

}  // namespace

SpeedResponseStyle MappingLongResponseLevel(LaneChangeStage prev_lc_stage,
                                            double follower_max_decel,
                                            double leader_max_decel,
                                            const std::string& prefix) {
  SpeedResponseStyle leader_level = SPEED_RESPONSE_NORMAL;
  static SpeedResponseStyle output_level = SPEED_RESPONSE_NORMAL;

  if (leader_max_decel == 0.0) {
    leader_level = SPEED_RESPONSE_NORMAL;
  } else if (leader_max_decel < 0.8) {
    leader_level = SPEED_RESPONSE_NORMAL;
  } else if (leader_max_decel < 2.0) {
    leader_level = SPEED_RESPONSE_FAST;
  } else {
    leader_level = SPEED_RESPONSE_RADICAL;
  }

  auto follower_level = SPEED_RESPONSE_NORMAL;
  if (follower_max_decel == 0.0) {
    follower_level = SPEED_RESPONSE_NORMAL;
  } else if (follower_max_decel < 0.3) {
    follower_level = SPEED_RESPONSE_NORMAL;
  } else if (follower_max_decel < 0.8) {
    follower_level = SPEED_RESPONSE_FAST;
  } else {
    follower_level = SPEED_RESPONSE_RADICAL;
  }

  auto res_level = std::max(leader_level, follower_level);

  if (prev_lc_stage == LaneChangeStage::LCS_EXECUTING) {
    output_level = std::max(res_level, output_level);
  } else {
    output_level = SPEED_RESPONSE_NORMAL;
  }

  Log2DDS::LogDataV2(
      "lc_safety",
      absl::StrCat(prefix, absl::StrFormat(
                               ":leader_max_decel:%.3f follower_max_decel:%.3f",
                               leader_max_decel, follower_max_decel)));
  Log2DDS::LogDataV2(
      "lc_safety",
      absl::StrCat(prefix,
                   absl::StrFormat(":output_level:%.1f  res_level:%.1f "
                                   "leader_level:%.1f follower_level:%.1f ",
                                   output_level, res_level, leader_level,
                                   follower_level)));
  return output_level;
}

absl::StatusOr<int> EvaluateMultiTrajs(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const PathSlBoundary& sl_boundary,
    const ApolloTrajectoryPointProto& start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<SingleTrajInfo>& multi_trajs,
    const VehicleParamsProto& vehicle_params, double speed_limit,
    bool eval_safety, const std::pair<double, double>& congestion_factor,
    LaneChangeStyle lc_style,
    const LaneChangeStyleDeciderResultProto& pre_lc_style_decider_result,
    const TaskSafetyEvaluationProto& pre_task_safety_evaluation_result,
    const int pre_scene_cones_riding_line_frames_result,
    const st::LaneChangeStage& lc_state, const bool lc_left,
    const st::LaneChangeStage& prev_lc_stage,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ObjectHistoryManager& obs_history_mgr,
    absl::Duration path_look_ahead_duration,
    absl::flat_hash_set<std::string>* follower_set,
    absl::flat_hash_set<std::string>* leader_set, double* follower_max_decel,
    double* leader_max_decel,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    PlannerStatusProto::PlannerStatusCode* status_code, ThreadPool* thread_pool,
    int plan_id, bool is_borrow, int* c_idx,
    LaneChangeStyleDeciderResultProto* lc_style_decider_result,
    TaskSafetyEvaluationProto* task_safety_evaluation_result,
    int* scene_cones_riding_line_frames_result,
    const std::vector<std::string>& leading_trajs,
    absl::flat_hash_set<std::string>* gaming_lc_obs_set,
    const std::vector<double>* stop_s_vec) {
  TIMELINE("EvaluateMultiTrajs");

  // After the ConesRidingLine scene is detected (with a non-zero frame count),
  // the frame count will be accumulated.
  *scene_cones_riding_line_frames_result =
      pre_scene_cones_riding_line_frames_result;
  if (*scene_cones_riding_line_frames_result > 0) {
    ++(*scene_cones_riding_line_frames_result);
  }

  const auto& prefix = Log2DDS::TaskPrefix(plan_id);
  if (multi_trajs.empty()) {
    *status_code = PlannerStatusProto::TRAJECTORY_NOT_ENTERING_TARGET_LANE;
    *lc_style_decider_result = pre_lc_style_decider_result;
    task_safety_evaluation_result->set_task_safe_counter(0);
    task_safety_evaluation_result->set_task_danger_counter(
        pre_task_safety_evaluation_result.task_danger_counter() + uint64_t(1));
    Log2DDS::LogDataV2(
        "safe_zone",
        absl::StrCat("----", (lc_left ? "lc_left" : "lc_right"), prefix, -1,
                     "-lc_state:", lc_state, "-borrow:", is_borrow,
                     "-prev_stage:", prev_lc_stage, "-danger_res:", 1,
                     "-safe_res:", 0, "-res:", 0, "----"));
    Log2DDS::LogDataV2(
        "safe_zone",
        absl::StrCat("traj_status:", "num:", -1, ",status:", 0, ",dngr_cnt:",
                     pre_task_safety_evaluation_result.task_danger_counter() +
                         uint64_t(1),
                     ",safe_cnt:", 0, ",message:",
                     "CANCEELED: Astar search failed ,no traj found"));
    std::string unsafe_obj_id =
        leading_trajs.empty() ? "" : leading_trajs.front();
    unsafe_object_ids->insert(unsafe_obj_id);
    return absl::CancelledError(absl::StrFormat(
        "Trajectory not entering the target lane (leading_id:%s).",
        unsafe_obj_id));
  }
  VLOG(3) << "Evaluating " << multi_trajs.size() << " trajectories.";

  // Log2DDS::LogDataV2("lc_safety", "EvaluateMultiTrajs:");
  // Log2DDS::LogDataV2(
  //     "lc_safety",
  //     absl::StrFormat("lc_stage:%.d prev_lc_stage:%.d lc_style:%.d ",
  //                     lc_state, prev_lc_stage, lc_style));

  bool prev_lane_changing =
      (prev_lc_stage == st::LaneChangeStage::LCS_EXECUTING ||
       prev_lc_stage == st::LaneChangeStage::LCS_PAUSE ||
       prev_lc_stage == st::LaneChangeStage::LCS_RETURN);
  bool is_lane_change = (lc_state == st::LaneChangeStage::LCS_EXECUTING ||
                         lc_state == st::LaneChangeStage::LCS_PAUSE ||
                         lc_state == st::LaneChangeStage::LCS_RETURN);

  // Make lc style radical if it is congested && low_speed
  LaneChangeStyle inside_lc_style = lc_style;
  bool is_congestion_scene = pre_lc_style_decider_result.congestion_scene();
  if (is_lane_change) {
    double congestion_factor_thr =
        pre_lc_style_decider_result.congestion_scene()
            ? kCongestionFactorDebounceThreshold
            : kCongestionFactorThreshold;
    is_congestion_scene = congestion_factor.second >
                          congestion_factor_thr * congestion_factor.first;
    if (inside_lc_style != LC_STYLE_RADICAL && is_congestion_scene &&
        start_point.v() < kEgoLowSpeedMpsThreshold) {
      inside_lc_style = LC_STYLE_RADICAL;
    }
    Log2DDS::LogDataV2(
        prefix + "lc_style_decider_debug",
        absl::StrFormat("congestion:%d (std:%.3f, traffic:%.3f, ego_v:%.3f), "
                        "last_cong:%d, lc style:%s->%s",
                        is_congestion_scene, congestion_factor.first,
                        congestion_factor.second, start_point.v(),
                        pre_lc_style_decider_result.congestion_scene(),
                        GetLaneChangeStyleName(lc_style),
                        GetLaneChangeStyleName(inside_lc_style)));
  }

  *lc_style_decider_result = pre_lc_style_decider_result;
  PathResponseStyle default_path_response_style =
      GetPathResponseStyleByLcStyle(inside_lc_style);
  SpeedResponseStyle default_speed_response_style =
      GetSpeedResponseStyleByLcStyle(inside_lc_style);

  std::vector<absl::Status> traj_status_list(multi_trajs.size());
  std::vector<TrajEvalInfo> traj_eval_info_list(multi_trajs.size());

  std::vector<LaneChangeStylePostDeciderSceneInfo>
      target_front_obj_scene_info_list(multi_trajs.size());
  std::vector<LaneChangeStylePostDeciderSceneInfos> scene_infos_list(
      multi_trajs.size());
  std::vector<std::pair<PathResponseStyle, SpeedResponseStyle>>
      post_lc_style_list(
          multi_trajs.size(),
          std::make_pair(GetPathResponseStyleByLcStyle(inside_lc_style),
                         GetSpeedResponseStyleByLcStyle(inside_lc_style)));
  std::vector<std::string> post_lc_style_decider_debug_str_list(
      multi_trajs.size());
  std::map<int, absl::flat_hash_set<std::string>> gaming_lc_obs_sets;
  std::vector<int> scene_cones_riding_line_frames_result_list(
      multi_trajs.size(), *scene_cones_riding_line_frames_result);
  uint64_t danger_counter =
      pre_task_safety_evaluation_result.task_danger_counter();
  uint64_t safe_counter = pre_task_safety_evaluation_result.task_safe_counter();
  bool traj_danger_res = false;
  bool traj_safe_res = false;
  bool is_on_highway = psmm.IsOnHighway();
  std::vector<double> ego_speed_kph = {0, 5, 30, 120};
  std::vector<double> counter_thrd = {1.0, 1.0, 2.0, 2.0};
  double spd_counter_thrd_db = ad_byd::planning::math::interp1_inc(
      ego_speed_kph, counter_thrd, Mps2Kph(start_point.v()));

  uint64_t SPD_COUNTER_THRD =
      static_cast<uint64_t>(std::round(spd_counter_thrd_db));

  bool has_status_ok = false;
  int min_cost_idx = 0;
  double min_cost = DBL_MAX;
  if (!eval_safety) {
    for (int i = 0; i < multi_trajs.size(); ++i) {
      traj_status_list[i] =
          EvaluateSingleTrajectory(multi_trajs[i], &traj_eval_info_list[i]);
    }
  } else {
    const auto target_lane_path_ext =
        BackwardExtendLanePath(psmm,
                               drive_passage.extend_lane_path().BeforeArclength(
                                   kLaneChangeCheckForwardLength),
                               kLaneChangeCheckBackwardLength);
    ASSIGN_OR_RETURN(
        const auto target_frenet_frame,
        BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path_ext),
                               /*down_sample_raw_points=*/true));

    constexpr double kLaneSpeedLimitPreviewTime = 6.0;  // s.
    // const double speed_limit = psmm.QueryLaneSpeedLimitById(
    //     drive_passage.lane_path()
    //         .ArclengthToLanePoint(multi_trajs[0].traj_points[0].v() *
    //                               kLaneSpeedLimitPreviewTime)
    //         .lane_id());

    const auto ego_sl = target_frenet_frame.XYToSL(
        Vec2dFromApolloTrajectoryPointProto(start_point));

    // judgement is closed ramp
    bool is_closed_ramp = false;
    if (psmm.map_ptr() && drive_passage.lane_seq_info()) {
      is_closed_ramp = (psmm.map_ptr()->v2_info().dist_to_ramp < 1500) &&
                       (drive_passage.lane_seq_info()->lc_num == 0);
    }

    const auto prev_response_style = std::make_pair(
        pre_lc_style_decider_result.active_path_response_style(),
        pre_lc_style_decider_result.active_speed_response_style());

    // do not use the thread pool
    ParallelFor(0, multi_trajs.size(), thread_pool, [&](int i) {
      traj_status_list[i] = EvaluateSingleTrajectoryAndCheckSafety(
          psmm, drive_passage, sl_boundary, start_point, multi_trajs[i],
          target_frenet_frame, speed_limit, st_traj_mgr, target_lane_path_ext,
          vehicle_params, inside_lc_style, prev_response_style, lc_state,
          prev_lc_stage, is_congestion_scene, stalled_objects, obs_history_mgr,
          path_look_ahead_duration, plan_id, &traj_eval_info_list[i],
          &target_front_obj_scene_info_list[i], &scene_infos_list[i],
          &scene_cones_riding_line_frames_result_list[i], is_closed_ramp,
          is_on_highway, gaming_lc_obs_sets[i], stop_s_vec);
      if ((prev_lc_stage == st::LaneChangeStage::LCS_EXECUTING &&
           lc_state == st::LaneChangeStage::LCS_EXECUTING) ||
          (prev_lc_stage == st::LaneChangeStage::LCS_RETURN &&
           lc_state == st::LaneChangeStage::LCS_RETURN)) {
        post_lc_style_list[i] = prev_response_style;
        post_lc_style_decider_debug_str_list[i] =
            "keep styles due to lc executing";
      } else if (!traj_status_list[i].ok()) {
        post_lc_style_list[i].first = default_path_response_style;
        post_lc_style_list[i].second = default_speed_response_style;
        post_lc_style_decider_debug_str_list[i] =
            "reset styles due to unsafety";
      } else {
        auto status = RunLaneChangeStylePostDecider(
            inside_lc_style, &target_front_obj_scene_info_list[i],
            scene_infos_list[i], &post_lc_style_list[i],
            post_lc_style_decider_debug_str_list[i]);
      }
    });
    for (size_t i = 0; i < traj_status_list.size(); ++i) {
      if (traj_status_list[i].ok()) {
        has_status_ok = true;
        if (traj_eval_info_list[i].eval_cost < min_cost) {
          min_cost = traj_eval_info_list[i].eval_cost;
          min_cost_idx = i;
        }
      }
    }

    // Log2DDS::LogDataV2("safe_zone_multi",
    //                    "----------------" + prefix + "----------------");
    // for (size_t i = 0; i < traj_status_list.size(); ++i) {
    //   Log2DDS::LogDataV2("safe_zone_multi",
    //                      absl::StrFormat("num:%d,status:%d,message:%s ", i,
    //                                      traj_status_list[i].ok(),
    //                                      traj_status_list[i].ToString()));

    //   Log2DDS::LogDataV2("safe_zone_multi",
    //                      traj_eval_info_list[i].leader_debug_info.str());

    //   Log2DDS::LogDataV2("safe_zone_multi",
    //                      traj_eval_info_list[i].follower_debug_info.str());
    // }
  }
  const int choice = has_status_ok
                         ? min_cost_idx
                         : std::min_element(traj_eval_info_list.begin(),
                                            traj_eval_info_list.end(),
                                            [&](const auto& a, const auto& b) {
                                              return a.eval_cost < b.eval_cost;
                                            }) -
                               traj_eval_info_list.begin();
  *scene_cones_riding_line_frames_result =
      scene_cones_riding_line_frames_result_list[choice];
  *c_idx = choice;
  bool pre_safety = pre_task_safety_evaluation_result.task_safe();
  bool res = pre_safety;
  if (eval_safety) {
    if (traj_status_list[choice].ok()) {
      safe_counter++;
      danger_counter = 0;
    } else {
      safe_counter = 0;
      danger_counter++;
    }
    task_safety_evaluation_result->set_task_safe_counter(safe_counter);
    task_safety_evaluation_result->set_task_danger_counter(danger_counter);

    traj_danger_res =
        !traj_status_list[choice].ok() &&
        (prev_lane_changing ? danger_counter > SPD_COUNTER_THRD : true);
    traj_safe_res =
        traj_status_list[choice].ok() &&
        (prev_lane_changing ? safe_counter > SPD_COUNTER_THRD : true);

    if (pre_safety && traj_danger_res) {
      res = false;
    } else if (!pre_safety && traj_safe_res) {
      res = true;
    }
    task_safety_evaluation_result->set_task_safe(res);

    Log2DDS::LogDataV2(
        "safe_zone",
        absl::StrCat("----", (lc_left ? "lc_left" : "lc_right"), prefix, choice,
                     "-lc_state:", lc_state, "-borrow:", is_borrow,
                     "-prev_stage:", prev_lc_stage,
                     "-danger_res:", traj_danger_res,
                     "-safe_res:", traj_safe_res, "-res:", res, "----"));
    Log2DDS::LogDataV2(
        "safe_zone",
        absl::StrCat("traj_status:", "num:", choice,
                     ",status:", traj_status_list[choice].ok(),
                     ",dngr_cnt:", danger_counter, "-(", SPD_COUNTER_THRD, ")",
                     ",safe_cnt:", safe_counter, ",cone_ride_line_cnt:",
                     *scene_cones_riding_line_frames_result,
                     ",message:", traj_status_list[choice].ToString()));
    // if (lc_state != st::LaneChangeStage::LCS_RETURN) {
    Log2DDS::LogDataV2(
        "safe_zone",
        absl::StrCat("front_debug:",
                     traj_eval_info_list[choice].leader_debug_info.str()));

    Log2DDS::LogDataV2(
        "safe_zone",
        absl::StrCat("rear_debug:",
                     traj_eval_info_list[choice].follower_debug_info.str()));
    // }

    auto post_lc_style_decision = post_lc_style_list[choice];
    auto post_lc_style_decider_debug_str =
        post_lc_style_decider_debug_str_list[choice];
    auto lat_lc_style_counter =
        pre_lc_style_decider_result.prepared_path_response_style_counter();
    auto lon_lc_style_counter =
        pre_lc_style_decider_result.prepared_speed_response_style_counter();
    // While lane changing, keep the styles
    if ((prev_lc_stage == st::LaneChangeStage::LCS_EXECUTING &&
         lc_state == st::LaneChangeStage::LCS_EXECUTING) ||
        (prev_lc_stage == st::LaneChangeStage::LCS_RETURN &&
         lc_state == st::LaneChangeStage::LCS_RETURN)) {
      lc_style_decider_result->set_active_path_response_style(
          pre_lc_style_decider_result.active_path_response_style());
      lc_style_decider_result->set_prepared_path_response_style(
          pre_lc_style_decider_result.active_path_response_style());
      lc_style_decider_result->set_prepared_path_response_style_counter(
          gLcStyleDebounceBuffer);
      lc_style_decider_result->set_active_speed_response_style(
          pre_lc_style_decider_result.active_speed_response_style());
      lc_style_decider_result->set_prepared_speed_response_style(
          pre_lc_style_decider_result.active_speed_response_style());
      lc_style_decider_result->set_prepared_speed_response_style_counter(
          gLcStyleDebounceBuffer);
    } else {
      // While it is not lane changing but it is safe now,
      // modify the styles if their counters reach the threshold
      if (traj_status_list[choice].ok()) {
        if (!pre_safety && res) {
          lat_lc_style_counter = gLcStyleDebounceBuffer;
          lc_style_decider_result->set_active_path_response_style(
              post_lc_style_decision.first);
          lc_style_decider_result->set_prepared_path_response_style(
              post_lc_style_decision.first);
          lc_style_decider_result->set_prepared_path_response_style_counter(
              lat_lc_style_counter);
          lon_lc_style_counter = gLcStyleDebounceBuffer;
          lc_style_decider_result->set_active_speed_response_style(
              post_lc_style_decision.second);
          lc_style_decider_result->set_prepared_speed_response_style(
              post_lc_style_decision.second);
          lc_style_decider_result->set_prepared_speed_response_style_counter(
              lon_lc_style_counter);
        } else {
          if (post_lc_style_decision.first ==
              pre_lc_style_decider_result.prepared_path_response_style()) {
            ++lat_lc_style_counter;
            if (lat_lc_style_counter >= gLcStyleDebounceBuffer) {
              lat_lc_style_counter = gLcStyleDebounceBuffer;
              lc_style_decider_result->set_active_path_response_style(
                  post_lc_style_decision.first);
            }
          } else {
            lat_lc_style_counter = 1;
            lc_style_decider_result->set_prepared_path_response_style(
                post_lc_style_decision.first);
          }
          lc_style_decider_result->set_prepared_path_response_style_counter(
              lat_lc_style_counter);
          if (post_lc_style_decision.second ==
              pre_lc_style_decider_result.prepared_speed_response_style()) {
            ++lon_lc_style_counter;
            if (lon_lc_style_counter >= gLcStyleDebounceBuffer) {
              lon_lc_style_counter = gLcStyleDebounceBuffer;
              lc_style_decider_result->set_active_speed_response_style(
                  post_lc_style_decision.second);
            }
          } else {
            lon_lc_style_counter = 1;
            lc_style_decider_result->set_prepared_speed_response_style(
                post_lc_style_decision.second);
          }
          lc_style_decider_result->set_prepared_speed_response_style_counter(
              lon_lc_style_counter);
        }
      }
      // While lane keeping and unsafe now, reset the styles to default
      else {
        SetDefaultResponseStyle(inside_lc_style, lc_style_decider_result);
      }
    }
    Log2DDS::LogDataV2(
        prefix + "lc_style_decider_debug",
        absl::StrFormat(
            "style:%s,%s cnt:%d,%d, message:%s",
            GetLaneChangeStyleName(
                lc_style_decider_result->active_path_response_style()),
            GetLaneChangeStyleName(
                lc_style_decider_result->active_speed_response_style()),
            lat_lc_style_counter, lon_lc_style_counter,
            post_lc_style_decider_debug_str));

    std::stringstream info_ss;
    for (size_t i = 0; i < traj_status_list.size(); ++i) {
      info_ss << absl::StrFormat(
          "num:%d,status:%d,style:[%s,%s],message:%s ", i,
          traj_status_list[i].ok(),
          GetLaneChangeStyleName(
              lc_style_decider_result->active_path_response_style()),
          GetLaneChangeStyleName(
              lc_style_decider_result->active_speed_response_style()),
          traj_status_list[i].ToString());
    }
    Log2DDS::LogDataV2(absl::StrCat(prefix, "safety"), info_ss.str());
  } else {
    traj_danger_res = !traj_status_list[choice].ok();
    traj_safe_res = traj_status_list[choice].ok();
    danger_counter = 0;
    safe_counter = 0;

    if (pre_safety && traj_danger_res) {
      res = false;
    } else if (!pre_safety && traj_safe_res) {
      res = true;
    }
    task_safety_evaluation_result->set_task_safe(res);
    task_safety_evaluation_result->set_task_safe_counter(safe_counter);
    task_safety_evaluation_result->set_task_danger_counter(danger_counter);

    Log2DDS::LogDataV2(
        "safe_zone",
        absl::StrCat(
            "----", is_lane_change ? (lc_left ? "lc_left" : "lc_right") : "lk",
            prefix, choice, "-lc_state:", lc_state, "-borrow:", is_borrow,
            "-prev_stage:", prev_lc_stage, "-danger_res:", traj_danger_res,
            "-safe_res:", traj_safe_res, "-res:", res, "----"));

    Log2DDS::LogDataV2(
        "safe_zone",
        absl::StrCat(
            "traj_status:", "num:", choice,
            ",status:", traj_status_list[choice].ok(),
            ",dngr_cnt:", danger_counter, ",safe_cnt:", safe_counter,
            ",cone_ride_line_cnt:", *scene_cones_riding_line_frames_result,
            ",message:", traj_status_list[choice].ToString()));
  }

  lc_style_decider_result->set_congestion_scene(is_congestion_scene);

  *follower_max_decel = traj_eval_info_list[choice].follower_max_decel;
  *leader_max_decel = traj_eval_info_list[choice].leader_max_decel;

  if (!res) {
    std::string fail_msg;
    for (int i = 0; i < traj_status_list.size(); ++i) {
      fail_msg += absl::StrFormat("%d. %s ", i, traj_status_list[i].message());
      if (!traj_eval_info_list[i].unsafe_object_id.empty()) {
        unsafe_object_ids->insert(traj_eval_info_list[i].unsafe_object_id);
      }
    }
    *status_code = traj_eval_info_list[choice].status_code;
    return absl::CancelledError(fail_msg);
  }
  *follower_set = std::move(traj_eval_info_list[choice].follower_set);
  *leader_set = std::move(traj_eval_info_list[choice].leader_set);
  *gaming_lc_obs_set = std::move(gaming_lc_obs_sets[choice]);

  // follower_set and leader_set debug info
  {
    std::ostringstream obj_set_debug;
    obj_set_debug.str("");
    for (const auto& obj_str : *follower_set) {
      obj_set_debug << obj_str << "-";
    }
    Log2DDS::LogDataV2("safe_zone",
                       absl::StrCat("follower_set:", obj_set_debug.str()));

    obj_set_debug.str("");
    for (const auto& obj_str : *leader_set) {
      obj_set_debug << obj_str << "-";
    }
    Log2DDS::LogDataV2("safe_zone",
                       absl::StrCat("leader_set:", obj_set_debug.str()));
  }

  *status_code = PlannerStatusProto::OK;
  return choice;
}

}  // namespace st::planning
