

#include "node_manager/task_runner/city_planner/city_planner.h"

#include <memory>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "plan_common/async/async_util.h"
#include "cyber/time/time.h"
#include "dump_result_util.h"
#include "gflags/gflags.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/glog.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "plan_common/av_history.h"
#include "plan_common/gflags.h"
#include "plan_common/git_version.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "plan_common/planning_macros.h"
#include "plan_common/timer.h"
#include "planner/planner_manager/planner_flags.h"
#include "planner/planner_manager/planner_params_builder.h"
#include "planner/planner_manager/planner_util.h"
#include "node_manager/task_runner/acc_planner.h"
#include "node_manager/task_runner/multi_tasks_cruise_planner.h"
#include "node_manager/task_runner/multi_tasks_cruise_planner_input.h"
#include "node_manager/task_runner/planner_main_loop_internal.h"
#include "plan_common/util/file_util.h"
#include "plan_common/util/status_macros.h"

#include <cereal/archives/json.hpp>
#include "plan_common/serialization/planner_state_serialization.h"

using ::google::protobuf::RepeatedPtrField;
namespace st {
namespace planning {

namespace {

constexpr double kDefaultVehicleLength = 7.0;
constexpr double kDefaultMainTargetS = 9999.0;
constexpr int kMaxLaneWidthHoldNum = 2;
constexpr double kLaneWidthFilterLastWeight = 0.7;
constexpr double kLaneWidthFilterCurWeight = 0.3;
constexpr double kMaxLaneWidthDiff = 0.4;
#define DEG2RAD(x) (3.1415926 * (x) / 180.0)
#define RAD2DEG(x) (180.0 * (x) / 3.1415926)

bool ShouldRunAccIfFailed(Behavior::FunctionId func_id) {
  switch (func_id) {
    case Behavior::APA:
    case Behavior::NONE:
    case Behavior::ACC:
      return false;
    case Behavior::LKA:
    case Behavior::LKA_PLUS:

    case Behavior::HW_NOA:
    case Behavior::MAPLESS_NOA:
    case Behavior::CITY_NOA:
      return true;
  }
  return false;
}

inline bool IsAccOnly() { return FLAGS_planner_task_init_type == 2; }

}  // namespace

CityPlanner::CityPlanner(int num_workers)
    : thread_pool_(new ThreadPool(num_workers, "CityPlanner")),
      planner_params_(new PlannerParamsProto()),
      vehicle_params_(new VehicleParamsProto()),
      planner_state_(new PlannerState()),
      planning_result_msg_(new ad_byd::planning::planning_result_type()) {}

bool CityPlanner::Init(const std::string& params_dir) {
  if (!InitParams(params_dir)) {
    LOG_ERROR << "init params from " << params_dir << " failed";
    return false;
  }
  // Init planner_state.
  planner_state_->previous_autonomy_state.set_autonomy_state(
      AutonomyStateProto::NOT_READY);
  return true;
}

bool CityPlanner::InitParams(const std::string& params_dir) {
  if (!file_util::FileToProto(FLAGS_ad_byd_vehicle_params_path,
                              vehicle_params_.get())) {
    LOG_ERROR << "failed to load vehicle params";
    return false;
  }

  LOG_INFO << "loaded vehicle params from " << FLAGS_ad_byd_vehicle_params_path;

  const auto glog_params_file = params_dir + "/glog_params.pb.txt";
  GlogParamsProto glog_params;
  if (!file_util::FileToProto(glog_params_file, &glog_params)) {
    LOG_ERROR << "failed to load glog params";
    return false;
  }
  FLAGS_v = glog_params.v();
  FLAGS_vmodule = glog_params.vmodule();

  auto ret = BuildPlannerParams(params_dir,
                                vehicle_params_->vehicle_geometry_params());
  if (!ret.ok()) {
    LOG_ERROR << "load planner params failed:" << ret.status();
    return false;
  }
  *planner_params_ = ret.value();
  LOG_INFO << "loaded planner params from " << params_dir;
  for (const auto& circle : planner_params_->vehicle_models_params()
                                .trajectory_optimizer_vehicle_model_params()
                                .circles()) {
    LOG_INFO << "circle: dist_to_rac: " << circle.dist_to_rac()
             << ", angle_to_axis: " << circle.angle_to_axis()
             << ", radius: " << circle.radius() << ", name: " << circle.name()
             << ", enable_pnp_route_selector: "
             << planner_params_->enable_pnp_route_selector();
  }
#ifdef DEBUG_X86_ENABLED
  // inject the first frame planner_state_->previous_trajectory
  const auto logsim_prevtraj_file = "./prev_traj.data";
  if (!file_util::FileToProto(logsim_prevtraj_file,
                              &planner_state_->previous_trajectory)) {
    LOG_ERROR << "[logsim] failed to load prev_traj";
    // continue with no prev_trajectory, won't return false
  } else {
    LOG_INFO << "[logsim] load prev_traj success";
  }

  // inject the first frame planner_state_->object_stop_time_map
  ObjectsPredictionProto tmp_pred_proto;
  const auto logsim_stoptime_file = "./obs_stop_time.data";
  if (!file_util::FileToProto(logsim_stoptime_file, &tmp_pred_proto)) {
    LOG_ERROR << "[logsim] failed to load obs_stop_time";
    // continue with no obs_stop_time, won't return false
  } else {
    absl::flat_hash_map<std::string, PlannerState::ObjectStopTimeResult>
        prev_stop_time_map;
    for (const auto& obs : tmp_pred_proto.objects()) {
      PlannerState::ObjectStopTimeResult obs_stoptime;
      obs_stoptime.time_duration_since_stop =
          obs.stop_time().time_duration_since_stop();
      obs_stoptime.previous_stop_time_duration =
          obs.stop_time().previous_stop_time_duration();
      obs_stoptime.last_move_time_duration =
          obs.stop_time().last_move_time_duration();
      obs_stoptime.last_time =
          static_cast<double>(tmp_pred_proto.header().timestamp()) / 1e6;
      prev_stop_time_map[obs.id()] = std::move(obs_stoptime);
    }
    planner_state_->object_stop_time_map = std::move(prev_stop_time_map);
    LOG_INFO << "[logsim] load obs_stop_time success";
  }
#endif

  return true;
}

void CityPlanner::Run(const ad_byd::planning::PlanningInputFrame* f) {
  SCOPED_TRACE("CityPlanner::Run");
  TIMELINE("CityPlanner::Run");
  // 0 = city, 1 = highway
  Log2DDS::LogDataV0("planner", 0);
  if (!vehicle_params_) {
    LOG_ERROR << "vehicle_params_ is nullptr";
  } else {
    Log2DDS::LogDataV0("platform",
                       vehicle_params_->vehicle_geometry_params().platform());
  }
  if (!f) {
    LOG_ERROR << "PlanningInputFrame is nullptr";
    return;
  } else {
    if (!f->behavior) {
      LOG_ERROR << "f->behavior is nullptr";
    }
    if (!f->env_map) {
      LOG_ERROR << "f->env_map is nullptr";
    }
    if (!f->prediction) {
      LOG_ERROR << "f->prediction is nullptr";
    }
    if (!f->vehicle_status) {
      LOG_ERROR << "f->vehicle_status is nullptr";
    }
    if (!f->odometry) {
      LOG_ERROR << "f->odometry is nullptr";
    }
  }

  const absl::Time current_time = FromUnixDoubleSeconds(f->last_msg_timestamp);
  const absl::Time predicted_plan_time =
      current_time + absl::Milliseconds(FLAGS_planner_lookforward_time_ms);

  Log2DDS::LogDataV2(
      "time_shift",
      absl::StrFormat(
          " last_msg_timestamp:%.3f, planner_lookforward_time_ms:%.3f, "
          "predicted_plan_time:%.3f",
          f->last_msg_timestamp, FLAGS_planner_lookforward_time_ms,
          ToUnixDoubleSeconds(predicted_plan_time)));

  const std::unique_ptr<MultiTasksCruisePlannerInput> planner_input =
      AdaptPlannerInput(f, *planner_params_, *vehicle_params_,
                        predicted_plan_time, thread_pool_.get(),
                        planner_state_.get());  // to do

  if (planner_input == nullptr) {
    if (planning_result_msg_) {
      planning_result_msg_->mutable_trajectory()->set_timestamp(
          f->last_msg_timestamp);
      Log2DDS::LogDataV2(
          "planner_status",
          planner_state_->planner_status_code == PlannerStatusProto::OK
              ? "OK"
              : absl::StrCat("planner_status:",
                             PlannerStatusProto::PlannerStatusCode_Name(
                                 planner_state_->planner_status_code)));
      if (ComputerFailCodeHoldCounter(planner_state_->planner_status_code)) {
        planning_result_msg_->mutable_state()->set_result(
            st::planning::Result::RESULT_OK);
        planning_result_msg_->mutable_state()->set_fail_reason(
            byd::msg::planning::PlannerStatusCode::OK);
      } else {
        Log2DDS::LogDataV2("planner_status",
                           absl::StrCat("fail_number_:", fail_number_));
        if (fail_number_ > 0) {
          --fail_number_;
          planning_result_msg_->mutable_state()->set_result(
              st::planning::Result::RESULT_OK);
        } else {
          planning_result_msg_->mutable_state()->set_result(
              st::planning::Result::RESULT_FAIL);
          planning_result_msg_->mutable_state()->set_fail_reason(
              static_cast<byd::msg::planning::PlannerStatusCode>(
                  planner_state_->planner_status_code));
          planning_result_msg_->mutable_trajectory()->clear_points();
          fail_reason_ = PlannerStatus(planner_state_->planner_status_code, "");
        }
      }
    }
    return;
  }

  FLAGS_planner_mapless_status = !(f->use_hd_map);
  PathBoundedEstPlannerOutput planner_output;
  // new upgrade plan
  const auto driver_select_function_id =
      planner_input->behavior->driver_select_function_id();
  const auto actual_function_id = planner_input->behavior->function_id();
  is_enable_to_upgrade_ = true;
  // case1: driver_select_function_id == Behavior_FunctionId_ACC, run pure acc
  if (driver_select_function_id == Behavior_FunctionId_ACC) {
    if (FLAGS_planner_enable_acc) {
      planner_status_ =
          RunAccPlanner(*planner_input, *planner_state_, &planner_output);
      fail_reason_ = planner_status_;
      is_enable_to_upgrade_ = false;
    }
     Log2DDS::LogDataV2("planner_running_case: ","case1:pure acc");
  } else {  // driver_select_function_id ==  LKA||HW_NOA||CITY_NOA
    // case2:run pure estplanner
    if (actual_function_id == Behavior_FunctionId_LKA ||
        actual_function_id == Behavior_FunctionId_HW_NOA ||
        actual_function_id == Behavior_FunctionId_CITY_NOA) {
      if (planner_state_->planner_semantic_map_manager != nullptr &&
          planner_state_->planner_semantic_map_manager->map_ptr() != nullptr &&
          planner_state_->planner_semantic_map_manager->map_ptr()->IsValid()) {
        if (planner_state_->planner_status_code !=
            PlannerStatusProto::EGO_LANE_LOSS_FAIL) {
          planner_status_ =
              RunMultiTasksCruisePlanner(*planner_input, *planner_state_,
                                         &planner_output, thread_pool_.get());
          est_planner_status_ = planner_status_;
        } else {
          planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                          "No ego lane found.");
        }
      } else {
        planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                        "No valid lane input from env.");
      }
      fail_reason_ = planner_status_;
      if (ComputerFailCodeHoldCounter(planner_status_.status_code())) {
        planner_status_debounce = planner_status_;
      } else {
        Log2DDS::LogDataV2("planner_status",
                           absl::StrCat("fail_number_:", fail_number_));
        if (fail_number_ > 0) {
          LOG_ERROR << "planner failed number: "
                    << static_cast<int>(fail_number_);
          --fail_number_;
          planner_status_debounce = PlannerStatus(PlannerStatusProto::OK, "");
        } else {
          LOG_ERROR << "planner failed debounce finished!";
          planner_status_debounce = planner_status_;
          is_enable_to_upgrade_ = false;
        }
      }
      Log2DDS::LogDataV2("planner_running_case: ","case2:pure est");
    }
    // case3:run only est to check whether NONE-> ICC/NOA satisfied , avoiding
    // "function_none" condition.
    if (actual_function_id == Behavior_FunctionId_NONE) {
      if (planner_state_->planner_semantic_map_manager != nullptr &&
          planner_state_->planner_semantic_map_manager->map_ptr() != nullptr &&
          planner_state_->planner_semantic_map_manager->map_ptr()->IsValid()) {
        if (planner_state_->planner_status_code !=
            PlannerStatusProto::EGO_LANE_LOSS_FAIL) {
          planner_status_ =
              RunMultiTasksCruisePlanner(*planner_input, *planner_state_,
                                         &planner_output, thread_pool_.get());
          est_planner_status_ = planner_status_;  // need to check
        } else {
          planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                          "No ego lane found.");
        }
      } else {
        planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                        "No valid lane input from env.");
      }
      fail_reason_ = planner_status_;
      is_enable_to_upgrade_ =
          CheckIfEstTrajMeettoUpgrade(f, planner_input.get(), &planner_output);
      Log2DDS::LogDataV2("planner_running_case: ","case3:wait upgrade");
    }
    // case4: run both est and acc, check est traj to determine whether acc->
    // icc/noa satisfied.
    if (actual_function_id == Behavior_FunctionId_ACC) {
      // check est first
      PathBoundedEstPlannerOutput planner_output_check;
      if (planner_state_->planner_semantic_map_manager != nullptr &&
          planner_state_->planner_semantic_map_manager->map_ptr() != nullptr &&
          planner_state_->planner_semantic_map_manager->map_ptr()->IsValid()) {
        if (planner_state_->planner_status_code !=
            PlannerStatusProto::EGO_LANE_LOSS_FAIL) {
          planner_status_ = RunMultiTasksCruisePlanner(
              *planner_input, *planner_state_, &planner_output_check,
              thread_pool_.get());
          // est_planner_status_ = planner_status_;  // need to check
        } else {
          planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                          "No ego lane found.");
        }
      } else {
        planner_status_ = PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                                        "No valid lane input from env.");
      }
      fail_reason_ = planner_status_;
      is_enable_to_upgrade_ = CheckIfEstTrajMeettoUpgrade(
          f, planner_input.get(), &planner_output_check);
      // run acc
      if (FLAGS_planner_enable_acc) {
        planner_status_ =
            RunAccPlanner(*planner_input, *planner_state_, &planner_output);
      }
      Log2DDS::LogDataV2("planner_running_case: ","case4:degrade to acc");
    }
  }

  Log2DDS::LogDataV2("planner_status",
                     planner_status_.ok()
                         ? "OK"
                         : "planner_status:" + planner_status_.ToString());
  {
    SCOPED_TRACE("DumpPlanningResult");
    UpdatePlannerState(planner_input.get(), &planner_output,
                       predicted_plan_time);
    DumpPlanningResult(f, planner_input.get(), &planner_output);
    DestroyContainerAsyncMarkSource(std::move(planner_output), "");
  }
}

void CityPlanner::UpdatePlannerState(
    const MultiTasksCruisePlannerInput* planner_input,
    const PathBoundedEstPlannerOutput* planner_output,
    const absl::Time predicted_plan_time) {
  // Temporarily store prev_target_lane_path
  const auto prev_tgt_lp = planner_state_->prev_target_lane_path;
  const auto prev_lc_state = planner_state_->lane_change_state;
  // Clear some fields of planner state, should be filled by the active planner.
  planner_state_->Clear();
  planner_state_->selector_state = std::move(planner_output->selector_state);
  planner_state_->acc_state = planner_output->acc_state;
  planner_state_->prev_ego_pose_frenet = planner_output->ego_pose_frenet;
  planner_state_->prev_est_planner_status_code =
      est_planner_status_.status_code();
  if (planner_state_->planner_semantic_map_manager != nullptr &&
      planner_state_->planner_semantic_map_manager->map_ptr() != nullptr) {
    planner_state_->prev_map_type =
        planner_state_->planner_semantic_map_manager->map_ptr()->type();
  } else {
    planner_state_->prev_map_type = ad_byd::planning::MapType::UNKNOWN_MAP;
  }

  UpdatePreLaneChangeSafetyInfo(planner_output,
                                &planner_state_->pre_lane_change_safety_info);

  UpdatePrePushStatus(planner_output, &planner_state_->pre_push_status);

  UpdateSavedOffset(planner_output, &planner_state_->saved_offset);

  planner_state_->pre_funciton_id = planner_input->behavior.has_value()
                                        ? planner_input->behavior->function_id()
                                        : Behavior_FunctionId_NONE;

  if (planner_output->acc_state == AccState::ACC_ON &&
      !planner_output->est_planner_output_list.empty()) {
    // The trjectory result is based on ACC.
    ParseEstPlannerOutputToPlannerState(
        planner_output->est_planner_output_list[0], planner_state_.get());
    planner_state_->ego_history.UpdateEgoHistory(
        planner_output->curr_selected_ego_frame);
    // covert trajectory to output

    const auto past_points = CreatePastPointsList(
        planner_input->start_point_info->plan_time,
        planner_state_->previous_trajectory,
        planner_input->start_point_info->reset_reason !=
                ResetReasonProto::SPEED_ONLY &&
            planner_input->start_point_info->reset_reason !=
                ResetReasonProto::FIRST_ENGAGE,
        kMaxPastPointNum);

    planner_state_->previous_trajectory.Clear();

    FillTrajectoryProto(
        planner_input->start_point_info->plan_time,
        planner_output->est_planner_output_list[0].traj_points, past_points,
        /*target_lane_path_from_current=*/{}, LaneChangeStateProto(),
        TurnSignal::TURN_SIGNAL_NONE,
        planner_output->est_planner_debug_list[0].traj_validation_result,
        &planner_state_->previous_trajectory);
    return;
  }

  if (planner_output->selector_output.has_value()) {
    planner_state_->is_going_force_route_change_left = std::move(
        planner_output->selector_output->is_going_force_route_change_left);
    planner_state_->begin_route_change_left =
        planner_output->selector_output->begin_route_change_left;
  }
  planner_state_->nudge_object_info = planner_output->nudge_object_info;
  planner_state_->if_continuous_lc = planner_output->if_continuous_lc;
  planner_state_->lc_push_dir = planner_output->lc_push_dir;
  planner_state_->last_lc_style = planner_output->last_lc_style;
  planner_state_->eie_time_counter_prev = planner_output->eie_time_counter_prev;
  planner_state_->eie_choice_type_prev = planner_output->eie_choice_type_prev;
  planner_state_->prev_dist_to_junction = planner_output->prev_dist_to_junction;
  planner_state_->cur_max_dist_to_junction =
      planner_output->cur_max_dist_to_junction;
  // Move out all scheduler_outputs since they should live longer than
  // planner_outputs (for filling debug proto).
  // std::vector<SchedulerOutput> scheduler_outputs;
  // if (IsNonEmptyPlannerResult(planner_status_)) {
  //   scheduler_outputs.reserve(planner_output->est_planner_output_list.size());
  //   for (auto& est_planner_output : planner_output->est_planner_output_list)
  //   {
  //     scheduler_outputs.emplace_back(est_planner_output.scheduler_output);
  //   }
  // }
  // if (IsNonEmptyPlannerResult(planner_status_)) {
  //   scheduler_outputs.emplace_back(
  //       planner_output->est_planner_output_list.at(0).scheduler_output);
  // }
  planner_state_->last_gaming_result.mutable_driveline_result()
      ->set_driveline_status(DrivelineStatus::GENERATE_FAILED);

  // If planner status is incorrect, keep target_lane_path and map_type of
  // previous successful frame
  if (planner_status_.ok()) {
    planner_state_->prev_success_map_type = planner_state_->prev_map_type;
  } else if (planner_status_.status_code() ==
                 PlannerStatusProto::INPUT_INCORRECT ||
             planner_status_.status_code() ==
                 PlannerStatusProto::TARGET_LANE_JUMPED_FAIL ||
             planner_status_.status_code() ==
                 PlannerStatusProto::EGO_LANE_LOSS_FAIL ||
             planner_status_.status_code() ==
                 PlannerStatusProto::SCHEDULER_UNAVAILABLE) {
    if (fail_number_ > 0) {
      planner_state_->prev_target_lane_path = prev_tgt_lp;
      planner_state_->lane_change_state = prev_lc_state;
      LOG_INFO << "keep previous target lane path";
    }
  }
  if (planner_status_.ok() &&
      !planner_output->est_planner_output_list.empty()) {
    const auto& selected_scheduler_output =
        planner_output->est_planner_output_list[0].scheduler_output;
    // planner_state_->prev_low_freq_psmm = planner_output->low_freq_psmm;
    // planner_state_->previous_trajectory_plan_counter = 0;
    // planner_state_->previous_st_path_global_including_past =
    //     planner_output->st_path_points_global_including_past;
    const auto& psmm = planner_state_->planner_semantic_map_manager;
    if (psmm != nullptr && psmm->map_ptr() != nullptr) {
      ParseSchedulerOutputToPlannerState(
          selected_scheduler_output, planner_state_.get(),
          planner_state_->planner_semantic_map_manager->map_ptr());
    }
    ParseEstPlannerOutputToPlannerState(
        planner_output->est_planner_output_list[0], planner_state_.get());
    UpdateObjectsHistory(
        planner_state_->object_history_manager, planner_input->objects_proto,
        planner_output->est_planner_output_list[0].st_boundaries_with_decision,
        planner_output->est_planner_output_list[0]
            .st_planner_object_trajectories,
        planner_output->est_planner_output_list[0].obj_lead,
        *planner_input->stalled_objects, planner_output->nudge_object_info);

    planner_state_->ego_history.UpdateEgoHistory(
        planner_output->curr_selected_ego_frame);

    // covert trajectory to output
    const auto past_points = CreatePastPointsList(
        planner_input->start_point_info->plan_time,
        planner_state_->previous_trajectory,
        planner_input->start_point_info->reset, kMaxPastPointNum);

    planner_state_->previous_trajectory.Clear();
    FillTrajectoryProto(
        planner_input->start_point_info->plan_time,
        planner_output->est_planner_output_list[0].traj_points, past_points,
        selected_scheduler_output.drive_passage.lane_path(),
        selected_scheduler_output.lane_change_state,
        TurnSignal::TURN_SIGNAL_NONE,
        // planner_state_->previously_triggered_aeb,
        planner_output->est_planner_debug_list[0].traj_validation_result,
        &planner_state_->previous_trajectory);
    planner_state_->last_gaming_result =
        planner_output->est_planner_output_list[0].gaming_result;

    // set the lc_lead_obj_id
    for (const auto& est_output : planner_output->est_planner_output_list) {
      if ("none" != est_output.lc_lead_obj_id &&
          planner_state_->lc_lead_obj_ids.end() ==
              std::find(planner_state_->lc_lead_obj_ids.begin(),
                        planner_state_->lc_lead_obj_ids.end(),
                        est_output.lc_lead_obj_id)) {
        planner_state_->lc_lead_obj_ids.push_back(est_output.lc_lead_obj_id);
      }
    }
    planner_state_->truncated_back_traj_horizon =
        planner_output->est_planner_output_list[0].truncated_back_traj_horizon;
  }
  // ---------------------- Update PLC result ------------------------
  Log2DDS::LogDataV2("mlc_debug", "result");
  if (!planner_state_->preferred_lane_path.IsEmpty() && !planner_status_.ok()) {
    planner_state_->preferred_lane_path = mapping::LanePath();
    planner_state_->alc_state = ALC_STANDBY_ENABLE;
    planner_state_->lane_change_command = DriverAction::LC_CMD_NONE;
    planner_state_->plc_prepare_start_time = std::nullopt;
    LOG_ERROR << "Teleop lane change failed: all branches failed.";
    Log2DDS::LogDataV2("mlc_debug", "result: reset");
  }
  if (planner_output->plc_result.has_value()) {
    planner_state_->preferred_lane_path =
        std::move(planner_output->plc_result->preferred_lane_path);
    planner_state_->alc_state = planner_output->alc_state;
    planner_state_->lane_change_command =
        planner_output->plc_result->lane_change_command;
    Log2DDS::LogDataV2("mlc_debug", "result: set");

    if (planner_state_->alc_state == ALC_PREPARE) {
      if (!planner_state_->plc_prepare_start_time.has_value()) {
        planner_state_->plc_prepare_start_time = predicted_plan_time;
      }
    } else if (planner_state_->plc_prepare_start_time.has_value()) {
      planner_state_->plc_prepare_start_time = std::nullopt;
    }
  }
  if (planner_status_.ok()) {
    // set traffic light interface
    planner_state_->tl_stop_interface = planner_output->tl_stop_interface;
    planner_state_->tl_ind_info = planner_output->tl_ind_info;
    // set speed state
    planner_state_->speed_state = planner_output->speed_state;
    Log2DDS::LogDataV2("lcc_keep_brake",
                       planner_output->speed_state.lcc_keep_brake);
    Log2DDS::LogDataV2("yield_to_vru",
                       planner_output->speed_state.yield_to_vru);
    Log2DDS::LogDataV2("keep_yield_time",
                       planner_output->speed_state.vru_interact_timer);
    for (const auto& id : planner_output->speed_state.infront_vru_ids) {
      Log2DDS::LogDataV2("infront_vru_id", id);
    }
  }
  // turn light decider
  const auto prev_turn_signal =
      TurnSignalResult{planner_state_->selector_state.last_turn_signal,
                       planner_state_->selector_state.last_turn_signal_reason};
  const auto& map_func_id = planner_input->behavior.has_value()
                                ? planner_input->behavior->function_id()
                                : Behavior_FunctionId_NONE;
  if (planner_status_.ok() && map_func_id != Behavior_FunctionId_NONE) {
    const auto& scheduler_output =
        planner_output->est_planner_output_list.at(0).scheduler_output;
    planner_state_->turn_signal_result = DecideTurnSignal(
        *planner_state_->planner_semantic_map_manager,
        planner_output->selector_state.pre_turn_signal,
        scheduler_output.drive_passage.lane_path(),
        planner_output->est_planner_output_list[0].redlight_lane_id,
        scheduler_output.lane_change_state, scheduler_output.drive_passage,
        scheduler_output.av_frenet_box_on_drive_passage,
        TurnSignalResult{.signal = scheduler_output.planner_turn_signal,
                         .reason = scheduler_output.turn_signal_reason},
        planner_input->pose_proto, planner_output->if_continuous_lc,
        scheduler_output.drive_passage.lane_seq_info(),
        planner_output->turn_type_signal, planner_output->selector_output,
        prev_turn_signal);
    planner_state_->selector_state.last_turn_signal =
        planner_state_->turn_signal_result.signal;
    planner_state_->selector_state.last_turn_signal_reason =
        planner_state_->turn_signal_result.reason;
    Log2DDS::LogDataV0("light_reason",
                       planner_state_->turn_signal_result.reason);

    if (map_func_id == Behavior_FunctionId_LKA &&
        !planner_output->icc_lc_enable) {
      if (!planner_input->behavior->auto_navi_lc_enable_status() ||
          planner_state_->turn_signal_result.reason != TURNING_TURN_SIGNAL) {
        planner_state_->turn_signal_result =
            TurnSignalResult{TURN_SIGNAL_NONE, TURN_SIGNAL_OFF};
      }
    }
  }
  planner_state_->lc_style_decider_results =
      planner_output->lc_style_decider_results;
  planner_state_->task_safety_evaluation_results =
      planner_output->task_safety_evaluation_results;
  planner_state_->scene_cones_riding_line_frames_results =
      planner_output->scene_cones_riding_line_frames_results;

  UpdateIsConstructionScene(planner_output, planner_state_->construction_info);
  // DestroyContainerAsyncMarkSource(std::move(scheduler_outputs), "");
}

void CityPlanner::Reset() {
  Timer timer("CityPlanner::Reset::planner_state_");
  auto prev_est_planner_code = planner_state_->prev_est_planner_status_code;
  auto pre_object_stop_time_map = planner_state_->object_stop_time_map;
  planner_state_.reset(new PlannerState());
  planner_state_->prev_est_planner_status_code =
      std::move(prev_est_planner_code);
  planner_state_->object_stop_time_map = std::move(pre_object_stop_time_map);
  planning_result_msg_.reset(new ad_byd::planning::planning_result_type());
}

void CityPlanner::ResetforAccEnterandExit() {
  Timer timer("CityPlanner::Reset::planner_state_");
  auto prev_est_planner_code = planner_state_->prev_est_planner_status_code;
  auto pre_object_stop_time_map = planner_state_->object_stop_time_map;
  planner_state_->ClearforAccEnterandExit();
  planner_state_->prev_est_planner_status_code =
      std::move(prev_est_planner_code);
  planner_state_->object_stop_time_map = std::move(pre_object_stop_time_map);
  planning_result_msg_.reset(new ad_byd::planning::planning_result_type());
  fail_number_ = 5;
}

void CityPlanner::DumpPlanningResult(
    const PlanningInputFrame* f,
    const MultiTasksCruisePlannerInput* planner_input,
    const PathBoundedEstPlannerOutput* planner_output) {
  auto msg_plan_result =
      std::make_shared<ad_byd::planning::planning_result_type>();
  msg_plan_result->mutable_header()->set_publish_timestamp(
      apollo::cyber::Time::Now().ToSecond());
  msg_plan_result->mutable_header()->set_measurement_timestamp(
      f->last_msg_timestamp);
  // msg_plan_result->mutable_header()->set_frame_id(
  //     absl::StrCat("NOA|", git_branch, "|", git_hash));

  // flag is_lane_short (true) if length of ego lane is too short
  double min_Length_threshold = 0.0;
  bool is_lane_short = false;
  // check NOA or ICC
  bool is_noa = false, is_icc = false, is_noa_icc = false;
  auto function_id = Behavior_FunctionId_NONE;
  if (planner_input->behavior.has_value()) {
    function_id = planner_input->behavior->function_id();
    is_noa = (function_id == Behavior_FunctionId_CITY_NOA) ||
             (function_id == Behavior_FunctionId_HW_NOA);
    is_icc = function_id == Behavior_FunctionId_LKA;
    is_noa_icc = is_noa || is_icc;
  }
  // get signal of headlight
  bool is_light_on = false;
  if (f->vehicle_status != nullptr) {
    const auto& left_bcm_0x38a = f->vehicle_status->left_bcm_0x38a();
    const auto dipped_headlight_38a_s = left_bcm_0x38a.dipped_headlight_38a_s();
    const auto high_beam_light_38a_s = left_bcm_0x38a.high_beam_light_38a_s();
    is_light_on = (dipped_headlight_38a_s == 1) || (high_beam_light_38a_s == 1);
  }

  if (is_noa_icc && planner_output->acc_state != AccState::ACC_ON) {
    if (planner_status_.ok() &&
        !planner_output->est_planner_output_list.empty()) {
      // check if has cipv
      const auto& cipv_obj_id =
          planner_output->est_planner_output_list[0].cipv_obj_id;
      bool has_cipv = cipv_obj_id.has_value();
      // get ego speed
      auto ego_v = 0.0;
      if (planner_input->start_point_info.has_value()) {
        ego_v = planner_input->start_point_info->start_point.v();
      }
      if (is_noa && has_cipv) {
        min_Length_threshold = ego_v * 1.0;
      } else if ((is_noa && !has_cipv) || is_icc) {
        if (is_light_on && ego_v > 27.78) {
          min_Length_threshold = ego_v * 1.8;
        } else {
          min_Length_threshold = ego_v * 2.6;
        }
      }

      const auto& selected_lane_path =
          planner_output->est_planner_output_list[0]
              .scheduler_output.drive_passage.lane_path();
      if (!selected_lane_path.IsEmpty()) {
        const auto remaining_len = selected_lane_path.length();
        if (remaining_len < min_Length_threshold) {
          is_lane_short = true;
        }
      }
    } else if (planner_status_.status_code() ==
                   PlannerStatusProto::INPUT_INCORRECT ||
               planner_status_.status_code() ==
                   PlannerStatusProto::TARGET_LANE_JUMPED_FAIL ||
               planner_status_.status_code() ==
                   PlannerStatusProto::EGO_LANE_LOSS_FAIL ||
               planner_status_.status_code() ==
                   PlannerStatusProto::SCHEDULER_UNAVAILABLE) {
      is_lane_short = true;
    }
  }
  msg_plan_result->mutable_state()->set_is_lane_short(is_lane_short);

  // trajectory
  msg_plan_result->mutable_trajectory()->set_timestamp(f->last_msg_timestamp);
  msg_plan_result->mutable_trajectory()->set_coordinate_type(1);  // odom
  // LOG_ERROR << std::fixed << std::setprecision(3)<< "odom time: "
  // <<f->start_timestamp; get final trajectory
  if (!planner_state_->previous_trajectory.past_points().empty()) {
    int start_index =
        planner_state_->previous_trajectory.past_points_size() - 1;
    for (int i = planner_state_->previous_trajectory.past_points_size(); i > 0;
         --i) {
      if (planner_state_->previous_trajectory.trajectory_start_timestamp() +
              planner_state_->previous_trajectory.past_points(i - 1)
                  .relative_time() <
          f->last_msg_timestamp - FLAGS_planner_extend_traj_behind_ego_by_time) {
        start_index = i - 1;
        break;
      }
      start_index = i - 1;
    }
    for (int i = start_index;
         i < planner_state_->previous_trajectory.past_points_size(); ++i) {
      auto point = planner_state_->previous_trajectory.past_points(i);
      auto out_point = msg_plan_result->mutable_trajectory()->add_points();
      out_point->set_x(point.path_point().x());
      out_point->set_y(point.path_point().y());
      out_point->set_theta(point.path_point().theta());
      out_point->set_kappa(point.path_point().kappa());
      out_point->set_t(
          planner_state_->previous_trajectory.trajectory_start_timestamp() +
          point.relative_time());
      out_point->set_v(point.v());
      out_point->set_a(point.a());
      out_point->set_yaw_rate(point.yaw_rate());
      out_point->set_steering_angle(point.path_point().steer_angle());
      out_point->set_is_extend(point.is_extend());
      // msg_plan_result->trajectory().points().push_back(out_point);
    }
  }
  std::string main_target_id = "";
  double main_target_speed = 0.0;
  double main_target_acc = 0.0;
  double main_target_ds = 0.0;
  double main_target_dl = 0.0;
  if (!planner_output->est_planner_output_list.empty()) {
    double s_min = std::numeric_limits<double>::max();
    for (const auto& traj :
         planner_output->est_planner_output_list[0].leading_trajs) {
      if (ConstraintProto::LeadingObjectProto::AFTER_STOPLINE !=
              traj.second.reason() &&
          traj.second.st_constraints_size() > 0 &&
          traj.second.st_constraints().begin()->s() < s_min) {
        s_min = traj.second.st_constraints().begin()->s();
        main_target_id =
            SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(traj.first);
      }
    }
    if (!main_target_id.empty()) {
      for (const auto& st_boundary : planner_output->est_planner_output_list[0]
                                         .st_boundaries_with_decision) {
        if (st_boundary.object_id().has_value() &&
            st_boundary.object_id().value() == main_target_id &&
            st_boundary.raw_st_boundary() != nullptr &&
            st_boundary.raw_st_boundary()->obj_sl_info().has_value()) {
          main_target_speed = st_boundary.obj_pose_info().v();
          main_target_acc = st_boundary.obj_pose_info().a();
          const auto& obj_sl_info =
              st_boundary.raw_st_boundary()->obj_sl_info();
          main_target_ds = obj_sl_info->ds;
          main_target_dl = obj_sl_info->dl;
          break;
        }
      }
    }
    msg_plan_result->mutable_state()->set_main_target_id(main_target_id);
    msg_plan_result->mutable_debug()->mutable_main_target()->set_id(
        main_target_id);
    msg_plan_result->mutable_debug()->mutable_main_target()->set_v(
        main_target_speed);
    msg_plan_result->mutable_debug()->mutable_main_target()->set_a(
        main_target_acc);
    msg_plan_result->mutable_debug()->mutable_main_target()->set_ds(
        main_target_ds);
    msg_plan_result->mutable_debug()->mutable_main_target()->set_dl(
        main_target_dl);
    // for nudge
    if (planner_output->nudge_object_info.has_value() &&
        planner_output->nudge_object_info->id != main_target_id) {
      msg_plan_result->mutable_state()->add_nudge_obstacles(
          planner_output->nudge_object_info->id);
      msg_plan_result->mutable_state()->set_nudge_obstacle_id(
          planner_output->nudge_object_info->id);
      if (planner_output->nudge_object_info->direction == 1) {
        if (planner_output->nudge_object_info->nudge_state ==
            NudgeObjectInfo::NudgeState::NUDGE) {
          msg_plan_result->mutable_state()->set_nudge_state(
              st::planning::NudgeState::NUDGE_LEFT);
        } else {
          msg_plan_result->mutable_state()->set_lane_borrow_state(
              st::planning::LaneBorrowState::BORROW_LEFT);
          msg_plan_result->mutable_state()->set_lane_borrow_reason(
              st::planning::LaneBorrowReason::LC_REASON_FOR_GENERAL);
        }
      } else {
        if (planner_output->nudge_object_info->nudge_state ==
            NudgeObjectInfo::NudgeState::NUDGE) {
          msg_plan_result->mutable_state()->set_nudge_state(
              st::planning::NudgeState::NUDGE_RIGHT);
        } else {
          msg_plan_result->mutable_state()->set_lane_borrow_state(
              st::planning::LaneBorrowState::BORROW_RIGHT);
          msg_plan_result->mutable_state()->set_lane_borrow_reason(
              st::planning::LaneBorrowReason::LC_REASON_FOR_GENERAL);
        }
      }
    }

    double main_target_s = kDefaultMainTargetS;
    const auto& st_boundaries_with_decision =
        planner_output->est_planner_output_list[0].st_boundaries_with_decision;
    if (!main_target_id.empty()) {
      for (const auto& st_boundary : st_boundaries_with_decision) {
        if ((st_boundary.decision_type() == StBoundaryProto::YIELD ||
             st_boundary.decision_type() == StBoundaryProto::FOLLOW) &&
            st_boundary.object_id().has_value() &&
            st_boundary.object_id().value() == main_target_id &&
            st_boundary.st_boundary() &&
            st_boundary.st_boundary()->GetBoundarySRange(0).has_value()) {
          main_target_s =
              st_boundary.st_boundary()->GetBoundarySRange(0).value().second;
          break;
        }
      }
    }
    // std::vector<pnc_idls::idls::YieldObstacles> yield_obstacles;
    double main_target_length = kDefaultVehicleLength;
    const auto* main_target_car =
        planner_input->object_manager->FindObjectById(main_target_id);
    if (main_target_car) {
      main_target_length = main_target_car->bounding_box().length();
    }

    for (const auto& st_boundary : st_boundaries_with_decision) {
      // pnc_idls::idls::YieldObstacles yield_obstacle;
      if ((st_boundary.decision_type() == StBoundaryProto::YIELD ||
           st_boundary.decision_type() == StBoundaryProto::FOLLOW) &&
          st_boundary.object_id().has_value() &&
          st_boundary.object_id().value() != main_target_id &&
          st_boundary.st_boundary() &&
          st_boundary.st_boundary()->GetBoundarySRange(0).has_value() &&
          st_boundary.st_boundary()->GetBoundarySRange(0).value().first <
              main_target_length + main_target_s) {
        msg_plan_result->mutable_state()->add_yield_obstacles(
            st_boundary.object_id().value());
        // yield_obstacle.id(st_boundary.object_id().value());
        // yield_obstacles.push_back(yield_obstacle);
      }
    }
    // msg_plan_result->state().yield_obstacles(yield_obstacles);
    if (!msg_plan_result->state().yield_obstacles().empty()) {
      msg_plan_result->mutable_state()->set_yield_obstacle_id(
          msg_plan_result->state().yield_obstacles(0));
    };

    // std::vector<std::string> overtake_obstacles;
    for (const auto& st_boundary : st_boundaries_with_decision) {
      if (st_boundary.st_boundary() &&
          st_boundary.decision_type() == StBoundaryProto::OVERTAKE &&
          st_boundary.object_id().has_value()) {
        msg_plan_result->mutable_state()->add_overtake_obstacles(
            st_boundary.object_id().value());
        // overtake_obstacles.push_back(st_boundary.object_id().value());
      }
    }
    // msg_plan_result->state().overtake_obstacles(overtake_obstacles);
  }

  const RepeatedPtrField<ApolloTrajectoryPointProto>& pts_vec =
      planner_state_->previous_trajectory.trajectory_point();
  for (auto iter = pts_vec.begin(); iter != pts_vec.end(); ++iter) {
    auto out_point = msg_plan_result->mutable_trajectory()->add_points();
    out_point->set_x(iter->path_point().x());
    out_point->set_y(iter->path_point().y());
    out_point->set_theta(iter->path_point().theta());
    out_point->set_kappa(iter->path_point().kappa());
    out_point->set_t(
        planner_state_->previous_trajectory.trajectory_start_timestamp() +
        iter->relative_time());
    out_point->set_v(iter->v());
    out_point->set_a(iter->a());
    out_point->set_yaw_rate(iter->yaw_rate());
    out_point->set_steering_angle(iter->path_point().steer_angle());
    out_point->set_is_extend(iter->is_extend());
    // msg_plan_result->trajectory().points().push_back(out_point);
  }
  AddDebugInfo(f->last_msg_timestamp, msg_plan_result->trajectory());
  // 1:fsd 2:st
  msg_plan_result->mutable_trajectory()->set_trajectory_type(
      static_cast<uint8_t>(2));
  LOG_INFO << "fail_number_ is :"<< static_cast<int>(fail_number_);
  if (planner_status_debounce.ok() || planner_status_.ok()) {
    msg_plan_result->mutable_state()->set_result(
        st::planning::Result::RESULT_OK);
    msg_plan_result->mutable_state()->set_fail_reason(
        static_cast<byd::msg::planning::PlannerStatusCode>(
            fail_reason_.status_code()));
    if (planner_state_->selector_state.is_deviate_navi) {
      msg_plan_result->mutable_state()->set_fail_reason(
          byd::msg::planning::PlannerStatusCode::DEVIATE_NAVI);
    }
  } else {
    if(fail_number_ > 0){//All functions needs to exit, including ACC.
      msg_plan_result->mutable_state()->set_result(
          st::planning::Result::RESULT_FAIL);
    }else{ //downgrade to ACC.
      msg_plan_result->mutable_state()->set_result(
        st::planning::Result::RESULT_OK);
    }
    msg_plan_result->mutable_state()->set_fail_reason(
        static_cast<byd::msg::planning::PlannerStatusCode>(
            planner_status_.status_code()));
    //msg_plan_result->mutable_trajectory()->clear_points();
  }

  // set stop hold action: control lock steering wheel
  constexpr double kepsilon = 1.0e-5;
  constexpr double v_threshold = 1.0 / 3.6;
  const auto& points = msg_plan_result->trajectory().points();
  if (!points.empty()) {
    double front_velocity = points.begin()->v();
    double back_velocity = points.rbegin()->v();
    if ((front_velocity < kepsilon && back_velocity < kepsilon) ||
        front_velocity < 1e-5) {
      msg_plan_result->mutable_trajectory()->set_action(
          st::planning::Trajectory::ACTION_EMERGENCY_STOP);
    }
  }

  // planning result : path info
  if (!planner_output->est_planner_output_list.empty() &&
      !planner_output->est_planner_output_list[0].st_path_points.empty()) {
    const auto& ddp_path_points_vec =
        planner_output->est_planner_output_list[0].st_path_points;
    for (auto iter = ddp_path_points_vec.begin();
         iter != ddp_path_points_vec.end(); ++iter) {
      if (iter == ddp_path_points_vec.begin()) {
        auto ego_position_ptr =
            msg_plan_result->mutable_path_info()->mutable_ego_position();
        ego_position_ptr->set_x(iter->x());
        ego_position_ptr->set_y(iter->y());
        ego_position_ptr->set_theta(iter->theta());
      }
      auto temp_p = msg_plan_result->mutable_path_info()->add_points();
      temp_p->set_x(iter->x());
      temp_p->set_y(iter->y());
      // msg_plan_result->path_info().points().emplace_back(temp_p);
    }
  } else {
    for (auto iter = pts_vec.begin(); iter != pts_vec.end(); ++iter) {
      if (iter == pts_vec.begin()) {
        auto ego_position_ptr =
            msg_plan_result->mutable_path_info()->mutable_ego_position();
        ego_position_ptr->set_x(iter->path_point().x());
        ego_position_ptr->set_y(iter->path_point().y());
        ego_position_ptr->set_theta(iter->path_point().theta());
      }
      auto temp_p = msg_plan_result->mutable_path_info()->add_points();
      temp_p->set_x(iter->path_point().x());
      temp_p->set_y(iter->path_point().y());
    }
  }

  // planning result: driving_boundary
  if (!planner_output->est_planner_output_list.empty()) {
    DumpSlBoundaryToDrivingBoundary(
        planner_output->est_planner_output_list[0].scheduler_output.sl_boundary,
        msg_plan_result->mutable_driving_boundary(), /*sample_interval=*/2);
  }

  // planning result: backup_path_info
  int backup_path_idx = -1;
  for (size_t i = 0; i < planner_output->est_planner_output_list.size(); ++i) {
    if (0 == i) continue;
    if (!planner_output->est_status_list[i].ok()) continue;
    backup_path_idx = i;
    if (!planner_output->est_planner_output_list[i]
             .scheduler_output.is_fallback)
      break;
  }
  if (-1 != backup_path_idx) {
    const auto& traj_points =
        planner_output->est_planner_output_list[backup_path_idx].traj_points;
    for (int i = 0; i < traj_points.size(); ++i) {
      if (0 == i) {
        auto ego_position_ptr =
            msg_plan_result->mutable_backup_path_info()->mutable_ego_position();
        ego_position_ptr->set_x(traj_points[i].path_point().x());
        ego_position_ptr->set_y(traj_points[i].path_point().y());
        ego_position_ptr->set_theta(traj_points[i].path_point().theta());
        // msg_plan_result->backup_path_info().ego_position(
        //     {traj_points[i].path_point().x(),
        //     traj_points[i].path_point().y(),
        //      traj_points[i].path_point().theta()});
      }
      auto temp_p = msg_plan_result->mutable_backup_path_info()->add_points();
      temp_p->set_x(traj_points[i].path_point().x());
      temp_p->set_y(traj_points[i].path_point().y());
      // msg_plan_result->backup_path_info().points().emplace_back(
      //     traj_points[i].path_point().x(), traj_points[i].path_point().y(),
      //     0, 0);
    }
  }

  // unmovable and queue obstacles
  for (const auto& stalled_obj : planner_state_->stalled_cars) {
    msg_plan_result->mutable_state()
        ->mutable_obstacles_intention()
        ->add_obstacles_unmovable(stalled_obj);
  }
  for (const auto& queue_obj : planner_state_->in_queue_cars) {
    msg_plan_result->mutable_state()
        ->mutable_obstacles_intention()
        ->add_obstacles_in_queue(queue_obj);
  }

  // add turn light
  // auto turn_light_req = st::planning::TurnLight::TURN_LIGHT_NONE;
  auto turn_light_req = byd::msg::planning::TurnLightRequest::TURN_LIGHT_NONE;
  auto turn_light_reason = byd::msg::planning::TurnLightReason::NONE;

  if (planner_state_->turn_signal_result.signal ==
      TurnSignal::TURN_SIGNAL_LEFT) {
    turn_light_req = byd::msg::planning::TurnLightRequest::TURN_LIGHT_LEFT;
  } else if (planner_state_->turn_signal_result.signal ==
             TurnSignal::TURN_SIGNAL_RIGHT) {
    turn_light_req = byd::msg::planning::TurnLightRequest::TURN_LIGHT_RIGHT;
  }
  Log2DDS::LogDataV0(
      "light_reason",
      absl::StrCat("reason: ", planner_state_->turn_signal_result.reason,
                   " turn_light_req: ", turn_light_req));
  switch (planner_state_->turn_signal_result.reason) {
    case TurnSignalReason::CONTINUE_LANE_CHANGE_SIGNAL:
      Log2DDS::LogDataV0("light_reason", "continue lc");
      turn_light_reason = byd::msg::planning::TurnLightReason::CONTINUE_LC;
      break;
    case TurnSignalReason::PREPARE_LANE_CHANGE_TURN_SIGNAL:
      Log2DDS::LogDataV0("light_reason", "prepare");
      turn_light_reason = byd::msg::planning::TurnLightReason::TURN;
      break;
    case TurnSignalReason::TURNING_TURN_SIGNAL:
    case TurnSignalReason::LANE_CHANGE_TURN_SIGNAL:
      Log2DDS::LogDataV0("light_reason", "turning");
      turn_light_reason = byd::msg::planning::TurnLightReason::TURN;
      break;
    case TurnSignalReason::MERGE_TURN_SIGNAL:
      Log2DDS::LogDataV0("light_reason", "merge");
      turn_light_reason = byd::msg::planning::TurnLightReason::MERGE;
      break;
    case TurnSignalReason::FORK_TURN_SIGNAL:
      Log2DDS::LogDataV0("light_reason", "split");
      turn_light_reason = byd::msg::planning::TurnLightReason::SPLIT;
      break;
    case TurnSignalReason::OPEN_GAP_TURN_SIGNAL:
      Log2DDS::LogDataV0("light_reason", "push");
      turn_light_reason = byd::msg::planning::TurnLightReason::PUSH;
      break;
    default:
      break;
  }
  Log2DDS::LogDataV0("light_reason",
                     absl::StrCat("final reason: ", turn_light_reason));
  double front_velocity = 0.0;
  if (!points.empty()) {
    front_velocity = points.begin()->v();
  }
  if (front_velocity < 0.3) {
    turn_light_req = byd::msg::planning::TurnLightRequest::TURN_LIGHT_NONE;
    turn_light_reason = byd::msg::planning::TurnLightReason::NONE;
  }
  msg_plan_result->mutable_state()->set_turn_light_request(turn_light_req);
  msg_plan_result->mutable_state()->set_turn_light_reason(turn_light_reason);
  // auto mode
  auto auto_mode = st::planning::AutoMode::Auto_None;
  if (planner_state_->previous_autonomy_state.autonomy_state() ==
      AutonomyStateProto::AUTO_DRIVE) {
    auto_mode = st::planning::AutoMode::Auto_LaneChange;
  }
  msg_plan_result->mutable_state()->set_auto_mode(auto_mode);
  // add target_traffic_velocity
  msg_plan_result->mutable_state()->set_target_traffic_velocity(
      planner_output->target_traffic_velocity);
  // add lc_state
  auto lc_state = byd::msg::planning::LaneChangeState::Lane_Keeping;
  if (planner_state_->lane_change_state.stage() != LaneChangeStage::LCS_NONE) {
    if (planner_state_->lane_change_state.lc_left()) {
      lc_state = byd::msg::planning::LaneChangeState::Lane_ChangeLeft;
    } else {
      lc_state = byd::msg::planning::LaneChangeState::Lane_ChangeRight;
    }
    if (planner_state_->lane_change_state.stage() ==
        LaneChangeStage::LCS_RETURN) {
      lc_state = byd::msg::planning::LaneChangeState::Lane_ChangeCancel;
    }
  } else {
    lc_state = byd::msg::planning::LaneChangeState::Lane_Keeping;
    if (planner_state_->selector_state.lane_change_prepare_state ==
            LaneChangePrepareState::Lane_PrepareLeft ||
        (turn_light_req ==
             byd::msg::planning::TurnLightRequest::TURN_LIGHT_LEFT &&
         turn_light_reason == byd::msg::planning::TurnLightReason::PUSH)) {
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lane_change_prepare_state ==
            LaneChangePrepareState::Lane_PrepareRight ||
        (turn_light_req ==
             byd::msg::planning::TurnLightRequest::TURN_LIGHT_RIGHT &&
         turn_light_reason == byd::msg::planning::TurnLightReason::PUSH)) {
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareRight;
    }
    Log2DDS::LogDataV2("mlc_debug", absl::StrCat("planner_output_cancel_1: ",
                                                 planner_output->if_cancel_lc));
    if (planner_output->if_cancel_lc) {
      lc_state = byd::msg::planning::LaneChangeState::Lane_ChangeCancel;
      Log2DDS::LogDataV2("mlc_debug", "cancel_seted ");
    }
  }
  Log2DDS::LogDataV2("lc_unable_reason",
                     absl::StrCat("planner_state_->lc_command_number",
                                  planner_state_->lc_command_number));
  Log2DDS::LogDataV2(
      "lc_unable_reason",
      absl::StrCat("planner_state_->lc_command", planner_output->input_lc_cmd));
  Log2DDS::LogDataV2("lc_unable_reason",
                     absl::StrCat("planner_state_->lane_change_state.stage()",
                                  planner_state_->lane_change_state.stage()));
  msg_plan_result->mutable_state()->set_lc_state(lc_state);
  bool is_push_state =
      planner_state_->lc_push_dir ==
          ad_byd::planning::PushDirection::Push_Normal_Left ||
      planner_state_->lc_push_dir ==
          ad_byd::planning::PushDirection::Push_Normal_Right ||
      planner_state_->lc_push_dir ==
          ad_byd::planning::PushDirection::Push_Congestion_Left ||
      planner_state_->lc_push_dir ==
          ad_byd::planning::PushDirection::Push_Congestion_Right;
  if (is_push_state) {
    msg_plan_result->mutable_state()->set_lc_sub_state(
        byd::msg::planning::LaneChangeSubState::Push);
  } else {
    msg_plan_result->mutable_state()->set_lc_sub_state(
        byd::msg::planning::LaneChangeSubState::None);
  }
  // add lc reason
  if (planner_state_->last_lc_reason == LaneChangeReason::MANUAL_CHANGE &&
      planner_state_->lane_change_state.stage() == LaneChangeStage::LCS_NONE) {
    Log2DDS::LogDataV2(
        "mlc_debug",
        "last lane change reason is manual and lane change stage is keeping");
    planner_state_->last_manual_lc_time++;
  }
  if (planner_state_->last_lc_reason != LaneChangeReason::MANUAL_CHANGE) {
    planner_state_->last_manual_lc_time = 0;
  }
  auto lc_reason = byd::msg::planning::LcReason::LC_REASON_NONE;
  if (planner_state_->lane_change_state.stage() != LaneChangeStage::LCS_NONE ||
      planner_state_->selector_state.lane_change_prepare_state !=
          LaneChangePrepareState::Lane_Keeping) {
    if (planner_state_->selector_state.last_lane_change_reason ==
        LaneChangeReason::NO_CHANGE) {
      Log2DDS::LogDataV2("last_lane_change_reason", "new loop");
      planner_state_->selector_state.last_lane_change_reason =
          planner_state_->selector_state.lane_change_reason;
    }
    if (planner_state_->selector_state.last_lane_change_reason !=
            planner_state_->selector_state.lane_change_reason &&
        planner_state_->selector_state.last_lane_change_reason !=
            LaneChangeReason::MANUAL_CHANGE &&
        planner_output->input_lc_cmd != DriverAction::LC_CMD_LEFT &&
        planner_output->input_lc_cmd != DriverAction::LC_CMD_RIGHT) {
      Log2DDS::LogDataV2("last_lane_change_reason",
                         "lc_reason_is_different_from_last");
      Log2DDS::LogDataV2(
          "last_lane_change_reason",
          absl::StrCat(
              "last: ", planner_state_->selector_state.last_lane_change_reason,
              "lane_change_reason raw: ",
              planner_state_->selector_state.lane_change_reason));
      planner_state_->selector_state.lane_change_reason =
          planner_state_->selector_state.last_lane_change_reason;
      Log2DDS::LogDataV2(
          "last_lane_change_reason",
          absl::StrCat("lane_change_reason after set",
                       planner_state_->selector_state.lane_change_reason));
    }

    Log2DDS::LogDataV2("mlc_debug",
                       absl::StrCat("city——planner lane change stage",
                                    planner_state_->lane_change_state.stage()));
    if (planner_state_->lane_change_state.stage() ==
            LaneChangeStage::LCS_NONE &&
        planner_state_->selector_state.lane_change_prepare_state ==
            LaneChangePrepareState::Lane_Keeping) {
      planner_state_->selector_state.last_lane_change_reason ==
          LaneChangeReason::NO_CHANGE;
    }
    if (planner_state_->selector_state.lane_change_reason == PROGRESS_CHANGE) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_OVERTAKE;
    }
    if (planner_state_->selector_state.lane_change_reason == ROUTE_CHANGE ||
        planner_state_->selector_state.lane_change_reason == LCC_ROUTE_CHANGE) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_FOR_NAVI;
    }
    if (planner_state_->selector_state.lane_change_reason == MANUAL_CHANGE) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_MANUAL;
    }
    if (planner_state_->selector_state.lane_change_reason == CENTER_CHANGE) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_CENTER;
    }
    if (planner_state_->selector_state.lane_change_reason == MERGE_CHANGE) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_FOR_NAVI;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        AVOID_STATIC_OBJECT) {
      lc_reason =
          byd::msg::planning::LcReason::LC_REASON_FOR_AVOID_STATIC_OBJECT;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        AVOID_VEHICLE_CHANGE) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_FOR_AVOID_VEHICLE;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        AVOID_ROADWORK_CHANGE) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_FOR_AVOID_ROADWORK;
    }
    if (planner_state_->selector_state.lane_change_reason == AVOID_LANE_BUS) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_FOR_AVOID_LANE_BUS;
    }
    if (planner_state_->selector_state.lane_change_reason == AVOID_MERGE_AREA) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_FOR_AVOID_MERGE_AREA;
    }
    if (planner_state_->selector_state.lane_change_reason == DEFAULT_CHANGE) {
      lc_reason = byd::msg::planning::LcReason::LC_REASON_FOR_AVOID_GENERAL;
    }
  }
  if (planner_state_->lane_change_state.stage() != LaneChangeStage::LCS_NONE) {
    planner_state_->last_lc_reason =
        planner_state_->selector_state.lane_change_reason;
    if (planner_output->input_lc_cmd == DriverAction::LC_CMD_LEFT ||
        planner_output->input_lc_cmd == DriverAction::LC_CMD_RIGHT) {
      planner_state_->last_lc_reason = LaneChangeReason::MANUAL_CHANGE;
    }
  }
  Log2DDS::LogDataV2("mlc_debug",
                     absl::StrCat("city——planner last lane change reason",
                                  planner_state_->last_lc_reason));
  Log2DDS::LogDataV2("lc_type", lc_reason);
  msg_plan_result->mutable_state()->set_lc_reason(lc_reason);
  // add lc state cannot cancel reason
  auto lc_cannot_cancel_reason =
      byd::msg::planning::LcStateReasonCannotCancel::CANNOT_CANCEL_NONE;
  if (planner_output->if_allow_cancel_lc == false ||
      planner_state_->lane_change_state.crossed_boundary()) {
    lc_cannot_cancel_reason =
        byd::msg::planning::LcStateReasonCannotCancel::CANNOT_CANCEL_CROSS_LINE;
  }
  msg_plan_result->mutable_state()->set_lc_state_reason_cannot_cancel(
      lc_cannot_cancel_reason);
  // add gap info
  msg_plan_result->mutable_state()->mutable_lc_gap_obstacles()->set_gap_front(
      planner_state_->selector_state.gap_front_id);
  msg_plan_result->mutable_state()->mutable_lc_gap_obstacles()->set_gap_back(
      planner_state_->selector_state.gap_back_id);
  double lane_width = -1.0;
  if (!planner_output->est_planner_output_list.empty() &&
      planner_output->ego_pose_frenet.has_value()) {
    const auto boundary =
        planner_output->est_planner_output_list[0]
            .scheduler_output.drive_passage.QueryNearestBoundaryLateralOffset(
                planner_output->ego_pose_frenet->s);
    if (boundary.ok()) {
      double boundary_left = boundary.value().second,
             boundary_right = boundary.value().first;
      lane_width = boundary_left - boundary_right;
    }
  }
  msg_plan_result->mutable_debug()->set_lane_width(lane_width);
  double ori_last_lane_width = planner_state_->saved_lane_width;
  if (lane_width < 0) {
    if (planner_state_->lane_width_invalid_cnt < kMaxLaneWidthHoldNum) {
      lane_width = planner_state_->saved_lane_width;
    } else {
      planner_state_->saved_lane_width = lane_width;
    }
    ++planner_state_->lane_width_invalid_cnt;
  } else {
    planner_state_->saved_lane_width =
        kLaneWidthFilterLastWeight * planner_state_->saved_lane_width +
        kLaneWidthFilterCurWeight * lane_width;
    if (ori_last_lane_width > 0) {
      const std::vector<double> speed_vec = {30, 80, 120};
      const std::vector<double> offset_limit_vec = {0.2, 0.5, 0.8};
      double lane_width_change_limit = ad_byd::planning::math::interp1_inc(
          speed_vec, offset_limit_vec,
          Mps2Kph(planner_input->start_point_info->start_point.v()));
      double delta_lane_width =
          planner_state_->saved_lane_width - ori_last_lane_width;
      delta_lane_width = std::clamp(delta_lane_width, -lane_width_change_limit,
                                    lane_width_change_limit);
      planner_state_->saved_lane_width = ori_last_lane_width + delta_lane_width;
      planner_state_->lane_width_invalid_cnt = 0;
    }
  }
  msg_plan_result->mutable_debug()->set_lane_width(
      planner_state_->saved_lane_width);

  msg_plan_result->mutable_debug()
      ->mutable_lane_change_debug()
      ->set_gap_id_front(planner_state_->selector_state.gap_front_id);
  msg_plan_result->mutable_debug()
      ->mutable_lane_change_debug()
      ->set_gap_id_back(planner_state_->selector_state.gap_back_id);
  // add lc unable reason
  auto lc_left_unable_reason = byd::msg::planning::LcUnableReason::REASON_NONE;
  auto lc_right_unable_reason = byd::msg::planning::LcUnableReason::REASON_NONE;
  if ((planner_output->input_lc_cmd == DriverAction::LC_CMD_LEFT ||
       planner_state_->selector_state.lane_change_prepare_state ==
           LaneChangePrepareState::Lane_PrepareLeft) &&
      planner_state_->lane_change_state.stage() == LaneChangeStage::LCS_NONE) {
    if (planner_state_->selector_state.lc_unable_reason ==
            LcFeasibility::FEASIBILITY_NO_LANE ||
        planner_output->lc_unable_reason ==
            LcFeasibility::FEASIBILITY_NO_LANE) {
      lc_left_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_NO_LANE;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
            LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT ||
        planner_output->lc_unable_reason ==
            LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT) {
      lc_left_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_OBSTACLE_FRONT;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        LcFeasibility::FEASIBILITY_OBS_TARGET_BACK) {
      lc_left_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_OBSTACLE_REAR;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        LcFeasibility::FEASIBILITY_CURVATURE) {
      lc_left_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_CURVATURE;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
            LcFeasibility::FEASIBILITY_LINE_TYPE ||
        planner_output->lc_unable_reason ==
            LcFeasibility::FEASIBILITY_LINE_TYPE) {
      lc_left_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_LINE_SOLID;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        LcFeasibility::FEASIBILITY_CONES) {
      lc_left_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_SATIC_OBSTACLE;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        LcFeasibility::FEASIBILITY_GENERAL) {
      lc_left_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_GENERAL_REASON;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareLeft;
    }
  }
  msg_plan_result->mutable_state()->set_lc_state(lc_state);
  Log2DDS::LogDataV1("lc_unable_reason", absl::StrCat("lc_unable_reason_left",
                                                      lc_left_unable_reason));
  msg_plan_result->mutable_state()->set_lc_unable_reason_left(
      lc_left_unable_reason);
  if ((planner_output->input_lc_cmd == DriverAction::LC_CMD_RIGHT ||
       planner_state_->selector_state.lane_change_prepare_state ==
           LaneChangePrepareState::Lane_PrepareRight) &&
      planner_state_->lane_change_state.stage() == LaneChangeStage::LCS_NONE) {
    if (planner_state_->selector_state.lc_unable_reason ==
            LcFeasibility::FEASIBILITY_NO_LANE ||
        planner_output->lc_unable_reason ==
            LcFeasibility::FEASIBILITY_NO_LANE) {
      lc_right_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_NO_LANE;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
            LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT ||
        planner_output->lc_unable_reason ==
            LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT) {
      lc_right_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_OBSTACLE_FRONT;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        LcFeasibility::FEASIBILITY_OBS_TARGET_BACK) {
      lc_right_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_OBSTACLE_REAR;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        LcFeasibility::FEASIBILITY_CURVATURE) {
      lc_right_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_CURVATURE;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
            LcFeasibility::FEASIBILITY_LINE_TYPE ||
        planner_output->lc_unable_reason ==
            LcFeasibility::FEASIBILITY_LINE_TYPE) {
      lc_right_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_LINE_SOLID;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        LcFeasibility::FEASIBILITY_CONES) {
      lc_right_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_SATIC_OBSTACLE;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        LcFeasibility::FEASIBILITY_GENERAL) {
      lc_right_unable_reason =
          byd::msg::planning::LcUnableReason::REASON_GENERAL_REASON;
      lc_state = byd::msg::planning::LaneChangeState::Lane_PrepareRight;
    }
  }
  if ((planner_output->input_lc_cmd == DriverAction::LC_CMD_LEFT ||
       planner_output->input_lc_cmd == DriverAction::LC_CMD_RIGHT) &&
      (planner_state_->lane_change_state.stage() !=
       LaneChangeStage::LCS_EXECUTING) &&
      planner_state_->lc_command_number > 273) {
    Log2DDS::LogDataV2("lc_unable_reason", "lane change cancel!");
    lc_state = byd::msg::planning::LaneChangeState::Lane_ChangeCancel;
  }
  if (planner_state_->lane_change_state.stage() ==
      LaneChangeStage::LCS_EXECUTING) {
    planner_state_->lc_command_number = 0;
  }
  // add 3 frames delay when lc state is cancel or keeping
  // make 3 frames keeping after 3 frames cancel
  if (planner_state_->selector_state.prev_lc_stage ==
          st::LaneChangeStage::LCS_EXECUTING &&
      planner_state_->lane_change_state.stage() == LaneChangeStage::LCS_NONE &&
      lc_state != byd::msg::planning::LaneChangeState::Lane_ChangeCancel) {
    lc_state = byd::msg::planning::LaneChangeState::Lane_Keeping;
    planner_state_->selector_state.pre_lane_change_state =
        LaneChangeState::Lc_Keeping;
    planner_state_->lc_keeping_delay_number = 1;
  } else if ((planner_state_->selector_state.prev_lc_stage ==
                  st::LaneChangeStage::LCS_RETURN &&
              planner_state_->lane_change_state.stage() ==
                  LaneChangeStage::LCS_NONE) ||
             lc_state ==
                 byd::msg::planning::LaneChangeState::Lane_ChangeCancel) {
    lc_state = byd::msg::planning::LaneChangeState::Lane_ChangeCancel;
    planner_state_->selector_state.pre_lane_change_state =
        LaneChangeState::Lc_ChangeCancel;
    planner_state_->lc_cancel_delay_number = 1;
  } else {
    if (planner_state_->lc_keeping_delay_number > 0 &&
        planner_state_->lc_keeping_delay_number <= 2 &&
        (planner_state_->selector_state.pre_lane_change_state ==
             LaneChangeState::Lc_Keeping ||
         planner_state_->selector_state.pre_lane_change_state ==
             LaneChangeState::Lc_ChangeCancel)) {
      lc_state = byd::msg::planning::LaneChangeState::Lane_Keeping;
      planner_state_->selector_state.pre_lane_change_state =
          LaneChangeState::Lc_Keeping;
      planner_state_->lc_keeping_delay_number++;
    } else {
      planner_state_->lc_keeping_delay_number = 0;
    }

    if (planner_state_->lc_cancel_delay_number > 0 &&
        planner_state_->lc_cancel_delay_number <= 2 &&
        planner_state_->selector_state.pre_lane_change_state ==
            LaneChangeState::Lc_ChangeCancel) {
      lc_state = byd::msg::planning::LaneChangeState::Lane_ChangeCancel;
      planner_state_->selector_state.pre_lane_change_state =
          LaneChangeState::Lc_ChangeCancel;
      planner_state_->lc_cancel_delay_number++;
    } else if (planner_state_->lc_cancel_delay_number == 3) {
      planner_state_->lc_keeping_delay_number = 1;
      planner_state_->lc_cancel_delay_number = 0;
    }
  }
  Log2DDS::LogDataV2(
      "lc_state_debug",
      absl::StrCat("prev_lc_stage: ",
                   planner_state_->selector_state.pre_lane_change_state));
  Log2DDS::LogDataV2("lc_state_debug",
                     absl::StrCat("lane_change_stage: ",
                                  planner_state_->lane_change_state.stage()));
  Log2DDS::LogDataV2("lc_state_debug", absl::StrCat("lc_state: ", lc_state));
  Log2DDS::LogDataV2("lc_state_debug",
                     absl::StrCat("lc_stage_cancel_number: ",
                                  planner_state_->lc_cancel_delay_number));
  Log2DDS::LogDataV2("lc_state_debug",
                     absl::StrCat("lc_stage_keeping_number: ",
                                  planner_state_->lc_keeping_delay_number));

  msg_plan_result->mutable_state()->set_lc_state(lc_state);
  Log2DDS::LogDataV1("lc_unable_reason", absl::StrCat("lc_unable_reason_right",
                                                      lc_right_unable_reason));
  msg_plan_result->mutable_state()->set_lc_unable_reason_right(
      lc_right_unable_reason);
  Log2DDS::LogDataV1(
      "lc_debug",
      absl::StrFormat("raw state:%d,reason:%d,raw reason:%d",
                      planner_state_->lane_change_state.stage(), lc_reason,
                      planner_state_->selector_state.lane_change_reason));
  Log2DDS::LogDataV1("lc_stage", planner_state_->lane_change_state.stage());

  // lc_notice
  planner_state_->lc_notice = planner_output->lc_notice;
  auto lc_notice = planner_state_->lc_notice;
  auto lc_notice_idl =
      static_cast<byd::msg::planning::LaneChangeNoticeType>(lc_notice);
  msg_plan_result->mutable_state()->set_lc_notice(lc_notice_idl);
  if (planner_output->has_passed_this_junction) {
    planner_state_->has_triggered_lc_notice = false;
  }
  // set traffic light interface
  msg_plan_result->mutable_state()->set_stop_line(
      planner_state_->tl_stop_interface);

  // set traffic light indication information
  switch (planner_state_->tl_ind_info.ind_color()) {
    case TlColor::TL_COLOR_GREEN:
      msg_plan_result->mutable_state()->set_ind_color(
          byd::msg::planning::TlColor::TL_COLOR_GREEN);
      break;
    case TlColor::TL_COLOR_YELLOW:
      msg_plan_result->mutable_state()->set_ind_color(
          byd::msg::planning::TlColor::TL_COLOR_YELLOW);
      break;
    case TlColor::TL_COLOR_RED:
      msg_plan_result->mutable_state()->set_ind_color(
          byd::msg::planning::TlColor::TL_COLOR_RED);
      break;
    case TlColor::TL_COLOR_BLOCK:
      msg_plan_result->mutable_state()->set_ind_color(
          byd::msg::planning::TlColor::TL_COLOR_BLOCK);
      break;
    case TlColor::TL_COLOR_NOLIGHT:
      msg_plan_result->mutable_state()->set_ind_color(
          byd::msg::planning::TlColor::TL_COLOR_NOLIGHT);
      break;
    case TlColor::TL_COLOR_UNKNOWN:
      msg_plan_result->mutable_state()->set_ind_color(
          byd::msg::planning::TlColor::TL_COLOR_UNKNOWN);
      break;
  }

  // set traffic light indication turn type infomation
  switch (planner_state_->tl_ind_info.ind_color_turn_type()) {
    case TrafficLightDirection::UNMARKED:
    case TrafficLightDirection::STRAIGHT:
      msg_plan_result->mutable_state()->set_ind_turn_type(
          byd::msg::planning::TlTurnType::TL_NO_TURN);
      break;
    case TrafficLightDirection::LEFT:
      msg_plan_result->mutable_state()->set_ind_turn_type(
          byd::msg::planning::TlTurnType::TL_LEFT_TURN);
      break;
    case TrafficLightDirection::RIGHT:
      msg_plan_result->mutable_state()->set_ind_turn_type(
          byd::msg::planning::TlTurnType::TL_RIGHT_TURN);
      break;
    case TrafficLightDirection::UTURN:
      msg_plan_result->mutable_state()->set_ind_turn_type(
          byd::msg::planning::TlTurnType::TL_U_TURN);
      break;
  }

  // set indication infomation
  switch (planner_state_->tl_ind_info.ind_status()) {
    case IndStatus::TL_GREEN_LIGHT_PASS:
      msg_plan_result->mutable_state()->set_ind_status(
          byd::msg::planning::PassFlag::GREEN_LIGHT_PASS);
      break;
    case IndStatus::TL_YELLOW_LIGHT_STOP:
      msg_plan_result->mutable_state()->set_ind_status(
          byd::msg::planning::PassFlag::YELLOW_LIGHT_STOP);
      break;
    case IndStatus::TL_RAD_LIGHT_STOP:
      msg_plan_result->mutable_state()->set_ind_status(
          byd::msg::planning::PassFlag::RAD_LIGHT_STOP);
      break;
    case IndStatus::TL_GREEN_BLINK_STOP:
      msg_plan_result->mutable_state()->set_ind_status(
          byd::msg::planning::PassFlag::GREEN_BLINK_STOP);
      break;
    case IndStatus::TL_PAY_ATTENTION_TO_TRAFFICLIGHT:
      msg_plan_result->mutable_state()->set_ind_status(
          byd::msg::planning::PassFlag::PAY_ATTENTION_TO_TRAFFICLIGHT);
      break;
    case IndStatus::TL_NO_MESSAGE:
      msg_plan_result->mutable_state()->set_ind_status(
          byd::msg::planning::PassFlag::NO_MESSAGE);
      break;
    default:
      msg_plan_result->mutable_state()->set_ind_status(
          byd::msg::planning::PassFlag::NO_MESSAGE);
      break;
  }
  // vru_for_suppressing_auto_start
  msg_plan_result->mutable_state()->set_is_vru_for_suppressing_auto_start(
      planner_input->is_vru_for_suppressing_auto_start);
  // ignore cross line
  msg_plan_result->mutable_state()->set_ignore_cross_line_state(
      st::planning::IgnoreCrossLineCheck::IGNORE_BOTH);

  // reference line
  if (!planner_output->est_planner_output_list.empty()) {
    msg_plan_result->mutable_state()->set_is_construction_scene(
        planner_output->est_planner_output_list[0]
            .scheduler_output.drive_passage.traffic_static_obstacles_info()
            .is_construction_scene &&
        planner_output->speed_state.is_construction_scene_speed_plan);
    DumpSlBoundaryToReferenceLine(
        planner_output->est_planner_output_list[0].scheduler_output.sl_boundary,
        msg_plan_result->mutable_reference_line(), /*sample_interval=*/2);
  }
  // start point
  const auto& start_p = planner_input->start_point_info->start_point;
  st::planning::StateDebug point;
  auto debug_s_point = msg_plan_result->mutable_debug()->mutable_start_point();
  debug_s_point->set_t(
      ToUnixDoubleSeconds(planner_input->start_point_info->plan_time));
  debug_s_point->set_x(start_p.path_point().x());
  debug_s_point->set_y(start_p.path_point().y());
  if (planner_output->start_point_frenet) {
    debug_s_point->set_s(planner_output->start_point_frenet->s);
    debug_s_point->set_l(planner_output->start_point_frenet->l);
  } else {
    debug_s_point->set_s(start_p.path_point().s());
    debug_s_point->set_l(0);  // no data: l
  }
  debug_s_point->set_v(start_p.v());
  debug_s_point->set_a(start_p.a());
  debug_s_point->set_theta(start_p.path_point().theta());
  debug_s_point->set_kappa(start_p.path_point().kappa());
  // msg_plan_result->debug().start_point(point);
  // reserved_0
  if (planner_output->ego_pose_frenet) {
    msg_plan_result->mutable_debug()->set_reserved_0(
        planner_output->ego_pose_frenet->l);
  }
  // reserved_1
  double heading_diff = M_PI;
  if (!planner_output->est_planner_output_list.empty()) {
    const auto start_point_lane_theta =
        planner_output->est_planner_output_list[0]
            .scheduler_output.drive_passage.QueryTangentAngleAtS(
                debug_s_point->s());
    if (start_point_lane_theta.ok()) {
      heading_diff =
          NormalizeAngle(debug_s_point->theta() - *start_point_lane_theta);
    }
  }
  msg_plan_result->mutable_debug()->set_reserved_1(heading_diff);
  // obstacles_debug
  absl::flat_hash_map<std::string, byd::msg::planning::ObstacleDebug>
      obstacle_debug_map;
  obstacle_debug_map.clear();
  // set default stop line
  std::string stopline_object_id = "stopline";
  byd::msg::planning::ObstacleDebug stopline_default_debug;
  stopline_default_debug.set_id(stopline_object_id);
  stopline_default_debug.set_ds(200.0);
  obstacle_debug_map[stopline_object_id] = stopline_default_debug;

  if (planner_output && planner_output->est_planner_output_list.size() > 0) {
    for (const auto& boundary_with_decision :
         planner_output->est_planner_output_list[0]
             .st_boundaries_with_decision) {
      int int_variable = 0;
      double double_variable = 0.0;
      byd::msg::planning::ObstacleDebug obstacle_debug;
      st::planning::MultiTrajDebug multi_traj_debug;
      std::string object_id = "";
      if (boundary_with_decision.raw_st_boundary() &&
          boundary_with_decision.raw_st_boundary()->object_type() ==
              StBoundaryProto::VIRTUAL &&
          boundary_with_decision.decision_type() == StBoundaryProto::YIELD) {
        object_id = stopline_object_id;
        obstacle_debug.set_id(stopline_object_id);
        obstacle_debug.set_ds(
            boundary_with_decision.raw_st_boundary()->bottom_left_point().s());
        obstacle_debug_map[object_id] = obstacle_debug;
        continue;
      }
      if (boundary_with_decision.object_id().has_value()) {
        object_id = boundary_with_decision.object_id().value();
      } else {
        continue;
      }
      if (obstacle_debug_map.find(object_id) == obstacle_debug_map.end()) {
        obstacle_debug.set_id(object_id);
        const auto& obj_sl_info =
            boundary_with_decision.raw_st_boundary()->obj_sl_info();
        if (obj_sl_info.has_value()) {
          obstacle_debug.set_ds(obj_sl_info->ds);
          obstacle_debug.set_dl(obj_sl_info->dl);
        }
        obstacle_debug.set_t(double_variable);
        obstacle_debug.set_v(boundary_with_decision.obj_pose_info().v());
        obstacle_debug.set_a(boundary_with_decision.obj_pose_info().a());
        obstacle_debug.set_v_l(double_variable);
        obstacle_debug.set_posterior_a(double_variable);
        obstacle_debug.set_intention(int_variable);
        obstacle_debug.set_lon_decision(boundary_with_decision.decision_type());
        obstacle_debug.set_lat_decision(int_variable);
        obstacle_debug.set_time_buffer(double_variable);
        int variable_to_int =
            static_cast<int>(boundary_with_decision.st_boundary()
                                 ->obj_scenario_info()
                                 .relationship);
        obstacle_debug.set_relationship(variable_to_int);
        obstacle_debug.set_priority(int_variable);
        obstacle_debug_map[object_id] = obstacle_debug;
      }

      if (boundary_with_decision.traj_id().has_value()) {
        multi_traj_debug.set_traj_id(boundary_with_decision.traj_id().value() +
                                     " | " + boundary_with_decision.id());
      } else {
        continue;
      }

      const auto& overlap_meta =
          *boundary_with_decision.st_boundary()->overlap_meta();
      std::string overlap_info =
          " pattern: " +
          StOverlapMetaProto::OverlapPattern_Name(overlap_meta.pattern()) +
          " source: " +
          StOverlapMetaProto::OverlapSource_Name(overlap_meta.source());

      multi_traj_debug.set_lat_decision(int_variable);
      multi_traj_debug.set_lon_decision(boundary_with_decision.decision_type());
      multi_traj_debug.set_traj_decision_reason(absl::StrCat(
          boundary_with_decision.decision_reason(), " | ignore_reason ",
          boundary_with_decision.ignore_reason(), "\n| info | ",
          boundary_with_decision.decision_info(), "\n overlap_info",
          overlap_info));
      if (obstacle_debug_map.find(object_id) != obstacle_debug_map.end()) {
        obstacle_debug_map[object_id].add_multi_traj_debug()->CopyFrom(
            multi_traj_debug);
      } else {
        obstacle_debug_map[object_id] = obstacle_debug;
        obstacle_debug_map[object_id].add_multi_traj_debug()->CopyFrom(
            multi_traj_debug);
      }
    }

    // add st_planner_object_trajectories debug
    const auto st_planner_obj_trjs = planner_output->est_planner_output_list[0]
                                         .st_planner_object_trajectories;
    for (size_t i = 0; i < st_planner_obj_trjs.trajectory_size(); ++i) {
      auto obj_id = st_planner_obj_trjs.trajectory(i).id();
      if (obstacle_debug_map.find(obj_id) == obstacle_debug_map.end()) {
        byd::msg::planning::ObstacleDebug obstacle_debug;
        obstacle_debug.set_id(obj_id);
        obstacle_debug.set_lat_decision(
            st_planner_obj_trjs.trajectory(i).reason());
        auto it =
            planner_output->est_planner_output_list[0].obj_sl_map.find(obj_id);
        if (it != planner_output->est_planner_output_list[0].obj_sl_map.end()) {
          // ds dl infos
          obstacle_debug.set_ds(it->second.ds);
          obstacle_debug.set_dl(it->second.dl);
        }
        obstacle_debug_map[obj_id] = obstacle_debug;
      } else {
        auto& obs_map = obstacle_debug_map[obj_id];
        obs_map.set_lat_decision(st_planner_obj_trjs.trajectory(i).reason());
      }
    }
  }
  for (const auto& [id, obstacle_debug] : obstacle_debug_map) {
    msg_plan_result->mutable_debug()->add_obstacles_debug()->CopyFrom(
        obstacle_debug);
  }

  // Set evaluate result
  auto* planning_state = msg_plan_result->mutable_state();
  if (planner_output->acc_state == AccState::ACC_ON) {
    auto* ref_line = msg_plan_result->mutable_reference_line();
    DumpSlBoundaryToReferenceLine(planner_output->acc_sl_boundary,
                                  msg_plan_result->mutable_reference_line(),
                                  /*sample_interval=*/2);
    DumpSlBoundaryToDrivingBoundary(planner_output->acc_sl_boundary,
                                    msg_plan_result->mutable_driving_boundary(),
                                    /*sample_interval=*/2);
  }
  if (is_enable_to_upgrade_) {
    planning_state->set_plan_evaluate_result(
        byd::msg::planning::PlanEvaluateResult::PLAN_EVALUATE_RESULT_LON_LAT);
  } else {
    planning_state->set_plan_evaluate_result(
        byd::msg::planning::PlanEvaluateResult::PLAN_EVALUATE_RESULT_LON_ONLY);
  }
  // if (!planner_status_debounce.ok() && !planner_status_.ok()) {
  //   planning_state->set_plan_evaluate_result(
  //       byd::msg::planning::PlanEvaluateResult::PLAN_EVALUATE_RESULT_NONE);
  //   msg_plan_result->mutable_state()->set_result(
  //       st::planning::Result::RESULT_FAIL);
  // } else if (planner_output->acc_state == AccState::ACC_ON) {
  //   auto* ref_line = msg_plan_result->mutable_reference_line();
  //   DumpSlBoundaryToReferenceLine(planner_output->acc_sl_boundary,
  //                                 msg_plan_result->mutable_reference_line(),
  //                                 /*sample_interval=*/2);
  //   DumpSlBoundaryToDrivingBoundary(planner_output->acc_sl_boundary,
  //                                   msg_plan_result->mutable_driving_boundary(),
  //                                   /*sample_interval=*/2);
  //   if (!fail_reason_.ok()) {
  //     planning_state->set_fail_reason(
  //         static_cast<byd::msg::planning::PlannerStatusCode>(
  //             fail_reason_.status_code()));
  //   }
  //   if (planner_output->acc_path_corridor_type == AccPathCorridorType::MAP) {
  //     planning_state->set_plan_evaluate_result(
  //         byd::msg::planning::PlanEvaluateResult::PLAN_EVALUATE_RESULT_LON_LAT);
  //   } else {
  //     planning_state->set_plan_evaluate_result(
  //         byd::msg::planning::PlanEvaluateResult::
  //             PLAN_EVALUATE_RESULT_LON_ONLY);
  //   }
  // } else {
  //   planning_state->set_plan_evaluate_result(
  //       byd::msg::planning::PlanEvaluateResult::PLAN_EVALUATE_RESULT_LON_LAT);
  // }
  planning_result_msg_.swap(msg_plan_result);
}

std::shared_ptr<ad_byd::planning::planning_result_type>
CityPlanner::GetPlanningResult() const {
  return planning_result_msg_;
}

std::shared_ptr<ad_byd::planning::debug_frame_type> CityPlanner::GetDebugFrame()
    const {
  return Log2DDS::Dump();
}

void CityPlanner::AddDebugInfo(double current_timestamp,
                               const planning_trajectory_type& traj) const {
  // Find control look ahead point, 150ms.
  constexpr double kControlLookAheadTime = 150.0 / 1e3;
  const double control_point_time = current_timestamp + kControlLookAheadTime;
  int points_size = traj.points_size();
  if (points_size == 0 || control_point_time < traj.points(0).t() ||
      control_point_time > traj.points(points_size - 1).t()) {
    return;
  }
  for (int i = 1; i < points_size; ++i) {
    if (control_point_time < traj.points(i).t()) {
      const double ratio = std::clamp(
          1.0 - (traj.points(i).t() - control_point_time) / kTrajectoryTimeStep,
          0.0, 1.0);
      const double last_yaw_rate = traj.points(i - 1).yaw_rate();
      const double control_yaw_rate =
          last_yaw_rate + ratio * (traj.points(i).yaw_rate() - last_yaw_rate);
      Log2DDS::LogDataV0("control_yaw_rate", control_yaw_rate);
      const double last_steer_angle = traj.points(i - 1).steering_angle();
      const double control_steer_angle =
          last_steer_angle +
          ratio * (traj.points(i).steering_angle() - last_steer_angle);
      Log2DDS::LogDataV0("control_steer_angle", control_steer_angle);
      break;
    }
  }
}

bool CityPlanner::ComputerFailCodeHoldCounter(
    const PlannerStatusProto::PlannerStatusCode& status_code) const {
  static PlannerStatusProto::PlannerStatusCode last_status_code =
      PlannerStatusProto::OK;
  if (status_code == PlannerStatusProto::OK) {
    last_status_code = status_code;
    return true;
  } else if (last_status_code == PlannerStatusProto::OK) {
    switch (status_code) {
      case PlannerStatusProto::INPUT_INCORRECT: {
        fail_number_ = 4;
        break;
      }
      case PlannerStatusProto::TARGET_LANE_JUMPED_FAIL: {
        fail_number_ = 4;
        break;
      }
      case PlannerStatusProto::EGO_LANE_LOSS_FAIL: {
        fail_number_ = 4;
        break;
      }
      case PlannerStatusProto::SCHEDULER_UNAVAILABLE: {
        fail_number_ = 4;
        break;
      }
      default: {
        fail_number_ = 0;
      }
    }
    last_status_code = status_code;
    return false;
  }
  return false;
}

const double CityPlanner::LatDiffLookUpTable(const double speed) const {
  const std::vector<double> speed_table = {30.0, 50.0, 60.0,  70.0,
                                           80.0, 90.0, 100.0, 110.0};
  const std::vector<double> lat_diff_table = {0.8, 0.8, 0.8, 0.7,
                                              0.7, 0.6, 0.5, 0.5};
  const double ego_pose_lat_diff_threshold = 0.45;
  if (speed_table.empty() || lat_diff_table.empty() ||
      speed_table.size() != lat_diff_table.size()) {
    return ego_pose_lat_diff_threshold;
  }
  int size_num = speed_table.size();
  if (size_num == 1) {
    return lat_diff_table[0];
  }
  if (speed <= speed_table[0]) {
    return lat_diff_table[0];
  }
  if (speed >= speed_table[(size_num - 1)]) {
    return lat_diff_table[size_num - 1];
  }
  int left = 0;
  int right = size_num;
  while (left < (right - 1)) {
    int middle = (left + right) / 2;
    if (speed == speed_table[middle]) {
      return lat_diff_table[middle];
    } else if (speed < speed_table[middle]) {
      right = middle;
    } else {
      left = middle;
    }
  }
  int idx = std::min(size_num - 1, std::max(left, right));
  return lat_diff_table[idx];
}

const double CityPlanner::HeadingDiffLookUpTable(const double speed) const {
  const std::vector<double> speed_table = {30.0, 40.0, 50.0, 60.0,
                                           70.0, 80.0, 90.0};
  const std::vector<double> degree_table = {1.5, 1.35, 1.2, 1.1, 1.0, 0.9, 0.8};
  const double ego_pose_heading_diff_degree_threshold = 1.2;
  if (speed_table.empty() || degree_table.empty() ||
      speed_table.size() != degree_table.size()) {
    return ego_pose_heading_diff_degree_threshold;
  }
  int size_num = speed_table.size();
  if (size_num == 1) {
    return degree_table[0];
  }
  if (speed <= speed_table[0]) {
    return degree_table[0];
  }
  if (speed >= speed_table[(size_num - 1)]) {
    return degree_table[size_num - 1];
  }

  int left = 0;
  int right = size_num;
  while (left < (right - 1)) {
    int middle = (left + right) / 2;
    if (speed == speed_table[middle]) {
      return degree_table[middle];
    } else if (speed < speed_table[middle]) {
      right = middle;
    } else {
      left = middle;
    }
  }
  int idx = std::min(size_num - 1, std::max(left, right));
  return degree_table[idx];
}

bool CityPlanner::CheckIfEstTrajMeettoUpgrade(
    const ad_byd::planning::PlanningInputFrame* f,
    const MultiTasksCruisePlannerInput* planner_input,
    const PathBoundedEstPlannerOutput* planner_output) {
  if (planner_status_.ok() &&
      !planner_output->est_planner_output_list.empty() &&
      f->vehicle_status != nullptr) {
    const auto orignal_traj = planner_state_->previous_trajectory;
    const auto past_points = CreatePastPointsList(
        planner_input->start_point_info->plan_time,
        planner_state_->previous_trajectory,
        planner_input->start_point_info->reset, kMaxPastPointNum);

    planner_state_->previous_trajectory.Clear();
    const auto& selected_scheduler_output =
        planner_output->est_planner_output_list[0].scheduler_output;

    FillTrajectoryProto(
        planner_input->start_point_info->plan_time,
        planner_output->est_planner_output_list[0].traj_points, past_points,
        selected_scheduler_output.drive_passage.lane_path(),
        selected_scheduler_output.lane_change_state,
        TurnSignal::TURN_SIGNAL_NONE,
        // planner_state_->previously_triggered_aeb,
        planner_output->est_planner_debug_list[0].traj_validation_result,
        &planner_state_->previous_trajectory);

    auto traj_result =
        std::make_shared<ad_byd::planning::planning_result_type>();
    if (!planner_state_->previous_trajectory.past_points().empty()) {
      int start_index =
          planner_state_->previous_trajectory.past_points_size() - 1;
      for (int i = planner_state_->previous_trajectory.past_points_size();
           i > 0; --i) {
        if (planner_state_->previous_trajectory.trajectory_start_timestamp() +
                planner_state_->previous_trajectory.past_points(i - 1)
                    .relative_time() <
            f->last_msg_timestamp) {
          start_index = i - 1;
          break;
        }
        start_index = i - 1;
      }
      for (int i = start_index;
           i < planner_state_->previous_trajectory.past_points_size(); ++i) {
        auto point = planner_state_->previous_trajectory.past_points(i);
        auto out_point = traj_result->mutable_trajectory()->add_points();
        out_point->set_theta(point.path_point().theta());
        out_point->set_kappa(point.path_point().kappa());
        out_point->set_steering_angle(point.path_point().steer_angle());
      }
    }
    const RepeatedPtrField<ApolloTrajectoryPointProto>& pts_vec =
        planner_state_->previous_trajectory.trajectory_point();
    for (auto iter = pts_vec.begin(); iter != pts_vec.end(); ++iter) {
      auto out_point = traj_result->mutable_trajectory()->add_points();
      out_point->set_theta(iter->path_point().theta());
      out_point->set_kappa(iter->path_point().kappa());
      out_point->set_steering_angle(iter->path_point().steer_angle());
    }
    planner_state_->previous_trajectory = orignal_traj;
    // reserved_0
    if (planner_output->ego_pose_frenet) {
      traj_result->mutable_debug()->set_reserved_0(
          planner_output->ego_pose_frenet->l);
    }
    // reserved_1
    double heading_diff = M_PI;
    double start_s = 0;
    const auto& start_p = planner_input->start_point_info->start_point;
    if (planner_output->start_point_frenet) {
      start_s = planner_output->start_point_frenet->s;
    } else {
      start_s = start_p.path_point().s();
    }

    const auto start_point_lane_theta =
        planner_output->est_planner_output_list[0]
            .scheduler_output.drive_passage.QueryTangentAngleAtS(start_s);
    if (start_point_lane_theta.ok()) {
      heading_diff = NormalizeAngle(start_p.path_point().theta() -
                                    *start_point_lane_theta);
    }

    const auto veh_dis_spd =
        f->vehicle_status->left_bcm_0x151().speed_signal_151_s();
    const auto current_steer_angle =
        DEG2RAD(f->vehicle_status->eps_0x06d().eps_steerwheelag());
    auto lat_l = traj_result->debug().reserved_0();
    auto heading_diff_with_lane = heading_diff;  // radian
    double max_diff_steer_angle = 0.0;
    constexpr double KCheckPlanTrajPointTime = 0.5;
    constexpr double KPlanTrajSteerAngleThreshold = 10.0;
    constexpr double plan_step = 0.1;
    int check_point_num = static_cast<int>(KCheckPlanTrajPointTime / plan_step);
    if (!traj_result->trajectory().points().empty()) {
      auto point_num =
          std::min(traj_result->trajectory().points().size(), check_point_num);
      double steer_angle_diff = 0.0;
      for (int i = 0; i < point_num; i++) {
        steer_angle_diff = std::fabs(
            traj_result->trajectory().points().at(i).steering_angle() -
            heading_diff_with_lane - current_steer_angle);
        max_diff_steer_angle =
            std::fmax(max_diff_steer_angle, steer_angle_diff);
      }
    }
    double lat_diff_check = LatDiffLookUpTable(veh_dis_spd);
    double heading_diff_check = HeadingDiffLookUpTable(veh_dis_spd);
    Log2DDS::LogDataV2(
        "traj_check", absl::StrCat("LAT_CHECK: ",
                                  std::fabs(lat_l)," ",lat_diff_check,
                                  " HEADING_CHECK: ",
                                  std::fabs(RAD2DEG(heading_diff_with_lane))," ",heading_diff_check,
                                  " STEER_CHECK: ",
                                  std::fabs(RAD2DEG(max_diff_steer_angle))," ",KPlanTrajSteerAngleThreshold));
    return (std::fabs(lat_l) <= lat_diff_check &&
            std::fabs(RAD2DEG(heading_diff_with_lane)) <= heading_diff_check &&
            std::fabs(RAD2DEG(max_diff_steer_angle)) <=
                KPlanTrajSteerAngleThreshold);
  } else {
    return false;
  }
}

}  // namespace planning
}  // namespace st
