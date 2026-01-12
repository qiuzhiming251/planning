#ifndef AD_BYD_PLANNING_NODES_PLANNER_CITY_CITY_PLANNER_H
#define AD_BYD_PLANNING_NODES_PLANNER_CITY_CITY_PLANNER_H
#include "plan_common/async/thread_pool.h"
#include "node_manager/msg_adapter/msg_manager.h"
#include "node_manager/task_runner/city_planner/msg_proxy.h"
#include "node_manager/task_runner/path_bounded_est_planner_output.h"
#include "node_manager/task_runner/planner.h"
#include "node_manager/task_runner/planner_state.h"

namespace st {

namespace planning {
using namespace ad_byd::planning;

class CityPlanner : public ad_byd::planning::Planner {
 public:
  DECLARE_PTR(CityPlanner);

  CityPlanner(int num_workers);

  bool Init(const std::string& params_dir);

  virtual void Run(
      const ad_byd::planning::PlanningInputFrame* input_frame) override;
  virtual void Reset() override;
  void ResetforAccEnterandExit();
  virtual std::shared_ptr<ad_byd::planning::planning_result_type>
  GetPlanningResult() const override;
  virtual std::shared_ptr<ad_byd::planning::debug_frame_type> GetDebugFrame()
      const override;

 private:
  void UpdatePlannerState(const MultiTasksCruisePlannerInput* planner_input,
                          const PathBoundedEstPlannerOutput* planner_output,
                          const absl::Time predicted_plan_time);
  void DumpPlanningResult(const ad_byd::planning::PlanningInputFrame* f,
                          const MultiTasksCruisePlannerInput* planner_input,
                          const PathBoundedEstPlannerOutput* planner_output);
  bool InitParams(const std::string& params_dir);
  void AddDebugInfo(double current_timestamp,
                    const planning_trajectory_type& traj) const;
  bool ComputerFailCodeHoldCounter(
      const PlannerStatusProto::PlannerStatusCode& status_code) const;
  const double LatDiffLookUpTable(const double speed) const;
  const double HeadingDiffLookUpTable(const double speed) const;
  bool CheckIfEstTrajMeettoUpgrade(const ad_byd::planning::PlanningInputFrame* f,
                                 const MultiTasksCruisePlannerInput* planner_input,
                                 const PathBoundedEstPlannerOutput* planner_output);

  std::unique_ptr<ThreadPool> thread_pool_;
  std::unique_ptr<PlannerParamsProto> planner_params_;
  std::unique_ptr<VehicleParamsProto> vehicle_params_;

  PlannerStatus planner_status_debounce;
  PlannerStatus planner_status_;
  PlannerStatus est_planner_status_;
  PlannerStatus fail_reason_ = PlannerStatus();
  std::unique_ptr<PlannerState> planner_state_;

  std::shared_ptr<ad_byd::planning::planning_result_type> planning_result_msg_;
  mutable uint8_t fail_number_ = 5;  // 10->5 2025.3.17
  bool is_enable_to_upgrade_ = true;
};

}  // namespace planning

}  // namespace st

#endif  // AD_BYD_PLANNING_NODES_PLANNER_CITY_CITY_PLANNER_H
