#pragma once

#include <chrono>
#include <memory>

#include "cyber/timer/timer.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/class_loader/class_loader.h"

#include "node_manager/msg_adapter/cyber_msg_manager.h"
#include "node_manager/task_runner/byd_planning_core.h"
#include "node_manager/task_runner/city_planner/city_planner.h"
#include "modules/simulator/deterministic_scheduler/deterministic_scheduler.h"

namespace ad_byd {
namespace planning {

using byd::simulator::deterministic_scheduler::SchedulerUnitManager;
using byd::simulator::deterministic_scheduler::TaskType;

static const std::string kDeterministicSimModeEnvName = "CYBER_SIM";
static const std::string kDefaultCyberSimFlag = "0";
static const std::string kCyberSimActivate = "1";

class BydPlanningComponent final : public apollo::cyber::TimerComponent {
 public:
  BydPlanningComponent() = default;
  virtual ~BydPlanningComponent() = default;

  virtual bool Init() override;
  virtual bool Proc() override;
  virtual void Clear() override;

 private:
  void PlanningCallback();
  void monitor_callback();

  int64_t planning_callback_start_ms_{0};
  pthread_t planning_callback_tid_{0};
  std::mutex thread_info_mutex_;
  BydPlanningCore::UPtr core_;
  CyberMsgManager::UPtr msg_manager_;
  std::unique_ptr<apollo::cyber::Timer> monitor_timer_;

  const std::string kTaskName = "timer_planning";
  std::string env_var = "0";
};
CYBER_REGISTER_COMPONENT(BydPlanningComponent)
}  // namespace planning
}  // namespace ad_byd
