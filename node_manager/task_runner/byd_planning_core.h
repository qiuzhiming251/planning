
#ifndef AD_BYD_PLANNING_NODES_BYD_PLANNING_CORE_H
#define AD_BYD_PLANNING_NODES_BYD_PLANNING_CORE_H

#include <chrono>
#include <memory>

#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
#include "modules/cnoa_pnc/planning/proto/planning_config.pb.h"
#include "node_manager/msg_adapter/msg_manager.h"
#include "plan_common/gflags.h"
#include "node_manager/task_runner/city_planner/city_planner.h"

namespace ad_byd {
namespace planning {

class BydPlanningCore {
 public:
  DECLARE_PTR(BydPlanningCore);
  BydPlanningCore() = default;
  virtual ~BydPlanningCore() = default;

  bool Init(std::string vehicle_config_file_path);
  bool Exit();

  std::pair<std::shared_ptr<planning_result_type>,
            std::shared_ptr<debug_frame_type>>
  PlanningCallback(
      const std::pair<st::planning::PlannerStatusProto::PlannerStatusCode,
                      PlanningInputFrame::Ptr>& input_frame);
  //   std::shared_ptr<prediction_type> PredictionCallback(
  //       const absl::StatusOr<PredictionInputFrame::Ptr>& input_frame);

  bool planning_enabled() const { return planning_enabled_; }
  bool prediction_enabled() const { return prediction_enabled_; }
  void set_planning_enabled(bool b) { planning_enabled_ = b; }
  void set_prediction_enabled(bool b) { prediction_enabled_ = b; }

 private:
  bool InitGflags() const;

  pred_config::PredictionConfig prediction_config_;

  // Prediction Component
  //   Component::UPtr component_;

  // Planners
  st::planning::CityPlanner::UPtr city_planner_;
  //   HighwayPlanner::UPtr highway_planner_;

  bool is_city_{true};
  bool planning_enabled_{true};
  bool prediction_enabled_{true};
  byd::msg::planning::FunctionId prev_func_id_ =
      byd::msg::planning::FunctionId::FUNCTION_NONE;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_BYD_PLANNING_CORE_H
