#include "byd_planning_component.h"

#include <execinfo.h>

#include <csignal>
#include <string>

#include "cyber/event/trace.h"
#include "cyber/event/trace_delay.h"
#include "glog/logging.h"
#include "plan_common/gflags.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "plan_common/timer.h"
#include "plan_common/util/hr_timer.h"
#include "plan_common/util/utility.h"
#include "version.h"

namespace ad_byd {
namespace planning {

void sigusr2_handler(int sig) {
  void *buffer[64];
  int layers = backtrace(buffer, 64);
  auto names = backtrace_symbols(buffer, layers);
  if (names) {
    AERROR << "fsd-planning, ------------backtrace start---------------";
    for (int i = 0; i < layers; i++) {
      AERROR << "fsd-planning, " << names[i];
    }
    AERROR << "fsd-planning, ------------backtrace end---------------";
  } else {
    AERROR << "fsd-planning, failed to get backtrace of callback thread";
  }
  AFATAL << "fsd-planning, callback did not finish in time. force stop it";
  throw std::runtime_error("planning callback did not return after 1 sec");
}

bool BydPlanningComponent::Init() {
#ifdef PLATFORM_X86
  google::ShutdownGoogleLogging();
  google::InitGoogleLogging("byd_planning_component");
  google::SetLogDestination(google::INFO, "");
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::FATAL, "");
  AERROR << "x86 mode: use new log file.";
#elif defined(PLATFORM_ARM)
  AERROR << "real car mode: use default log direction.";
#else
  AERROR << "unknown mode: use default log direction.";
#endif
  core_.reset(new BydPlanningCore());
  signal(SIGUSR2, sigusr2_handler);
  if (ComponentBase::ConfigFilePath().empty()) {
    AERROR << "Unable to load planning vehicle config file path: "
           << ComponentBase::ConfigFilePath();
    return false;
  } else {
    AINFO << "Load planning vehicle config file path: "
          << ComponentBase::ConfigFilePath();
  }
  if (!core_->Init(ComponentBase::ConfigFilePath())) {
    return false;
  };

  msg_manager_ = std::make_unique<CyberMsgManager>(node_);
  msg_manager_->Init();

  monitor_timer_.reset(new apollo::cyber::Timer(
      50, [this]() { this->monitor_callback(); }, false, false));
  monitor_timer_->Start();

  env_var = apollo::cyber::common::GetEnv(kDeterministicSimModeEnvName,
                                          kDefaultCyberSimFlag);
  return true;
}

bool BydPlanningComponent::Proc() {
  PERFORMANCE_TRACE_START(planning);
  PlanningCallback();
  return true;
}

void BydPlanningComponent::Clear() {
  msg_manager_->Exit();
  core_->Exit();
}

void BydPlanningComponent::monitor_callback() {
  pthread_t latest_tid{0};
  int64_t latest_start_ms{0};
  {
    std::scoped_lock<std::mutex> lock(thread_info_mutex_);
    latest_tid = planning_callback_tid_;
    latest_start_ms = planning_callback_start_ms_;
  }
  if (latest_tid == 0) {
    AWARN << "fsd-planning, invalid tid";
    return;
  }
  if (latest_start_ms == 0) {
    AWARN << "fsd-planning, invalid start ms";
    return;
  }
  int64_t current_ms = apollo::cyber::Time::MonoTime().ToMillisecond();
  int64_t dt_ms = current_ms - latest_start_ms;
#ifndef DEBUG_X86_ENABLED
  LOG_INFO << "fsd-planning, tid = " << static_cast<int64_t>(latest_tid)
           << ", dt = " << dt_ms;
  if (dt_ms > 5000 && env_var != kCyberSimActivate) {
    LOG_ERROR
        << "fsd-planning, planning callback did not finish.restarting node ";

    pthread_kill(latest_tid, SIGUSR2);
  }
#endif
}

void BydPlanningComponent::PlanningCallback() {
  TIMELINE("BydPlanningComponent::PlanningCallback");
  {
    std::scoped_lock<std::mutex> lock(thread_info_mutex_);
    planning_callback_start_ms_ =
        apollo::cyber::Time::MonoTime().ToMillisecond();
    planning_callback_tid_ = pthread_self();
  }
  TASK_REGISTER_MACRO(kTaskName, TaskType::TIMER);
  auto input_frame = msg_manager_->GetPlanningInputFrame();
  TASK_CONTROL_MACRO(kTaskName, void());
  auto [planning_result, debug_frame] = core_->PlanningCallback(input_frame);
  if (planning_result) {
    TIMELINE("BydPlanningComponent::PublishPlanRes");
    const PlanningInputFrame *frame = std::get<1>(input_frame).get();
    if (frame != nullptr) {
      const auto &prediction_msg = frame->prediction;
      if (prediction_msg != nullptr &&
          prediction_msg->header().has_camera_timestamp()) {
        DELAY_TRACE_START(
            planning_camera,
            (planning_result->header().publish_timestamp() * 1000.0),
            (prediction_msg->header().camera_timestamp() * 1000.0));
      }
    }
    planning_result->mutable_header()->set_frame_id(GIT_COMMIT_HASH);
    msg_manager_->PublishPlanningResultMsg(planning_result);
    const std::array<double, 3> ego_pos = msg_manager_->GetVehPos();
    double cos_yaw = std::cos(ego_pos[2]), sin_yaw = std::sin(ego_pos[2]);
    byd::msg::pnc::PlanTrajInfo plan_traj_info;
    auto &result_traj = planning_result->trajectory();
    for (int i = 0; i < result_traj.points_size(); i++) {
      auto point_ptr = plan_traj_info.add_pt_bus();
      const auto &res_point = result_traj.points(i);
      double base_traj_x = (res_point.x() - ego_pos[0]) * cos_yaw +
                           (res_point.y() - ego_pos[1]) * sin_yaw;
      double base_traj_y = (res_point.y() - ego_pos[1]) * cos_yaw -
                           (res_point.x() - ego_pos[0]) * sin_yaw;
      point_ptr->mutable_path_point()->set_x(base_traj_x);
      point_ptr->mutable_path_point()->set_y(base_traj_y);
    }
    plan_traj_info.set_pt_num(plan_traj_info.pt_bus_size());
    plan_traj_info.mutable_header()->set_publish_timestamp(
        planning_result->header().publish_timestamp());
    plan_traj_info.mutable_header()->set_measurement_timestamp(
        planning_result->header().publish_timestamp());
    plan_traj_info.mutable_header()->set_sequence_num(
        planning_result->header().sequence_num());
    plan_traj_info.mutable_header()->set_frame_id(GIT_COMMIT_HASH);
    msg_manager_->PublishPlanningTrajMsg(
        std::make_shared<byd::msg::pnc::PlanTrajInfo>(plan_traj_info));
  }
  if (debug_frame) {
    debug_frame->mutable_header()->set_frame_id(GIT_COMMIT_HASH);
    msg_manager_->PublishDebugFrameMsg(debug_frame);
  }
}
}  // namespace planning
}  // namespace ad_byd
