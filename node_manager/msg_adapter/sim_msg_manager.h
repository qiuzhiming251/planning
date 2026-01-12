#ifndef AD_BYD_PLANNING_NODES_SIM_MSG_MANAGER_H
#define AD_BYD_PLANNING_NODES_SIM_MSG_MANAGER_H

#include "node_manager/msg_adapter/msg_manager.h"

namespace ad_byd {
namespace planning {

class SimMsgManager : public MsgManager {
 public:
  DECLARE_PTR(SimMsgManager);

  SimMsgManager() {
    prediction_pub_.SetUserCallback(
        [this](const std::shared_ptr<prediction_type>& m) {
          prediction_ = *m;
        });
    planning_result_pub_.SetUserCallback(
        [this](const std::shared_ptr<planning_result_type>& m) {
          planning_result_ = *m;
        });
    debug_frame_pub_.SetUserCallback(
        [this](const std::shared_ptr<debug_frame_type>& m) {
          debug_frame_ = *m;
        });

    odometry_sub_.DisableCheckDdl();
    vehicle_status_sub_.DisableCheckDdl();
    vmc_debug_sub_.DisableCheckDdl();
    obstacle_sub_.DisableCheckDdl();
    bev_map_sub_.DisableCheckDdl();
    env_map_sub_.DisableCheckDdl();
    noa_map_sub_.DisableCheckDdl();
    behavior_sub_.DisableCheckDdl();
    prediction_sub_.DisableCheckDdl();
#if defined(BYD_X2B) || defined(BYD_VCPB)
    top_state_sub_.DisableCheckDdl();
#endif
    highway_prediction_sub_.DisableCheckDdl();
    scenario_intention_sub_.DisableCheckDdl();
  }

  prediction_type GetPrediction() const { return prediction_; }

  planning_result_type GetPlanningResult() const { return planning_result_; }

  debug_frame_type GetDebugFrame() const { return debug_frame_; }

 private:
  prediction_type prediction_;
  planning_result_type planning_result_;
  debug_frame_type debug_frame_;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_SIM_MSG_MANAGER_H
