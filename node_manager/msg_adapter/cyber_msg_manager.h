#ifndef AD_BYD_PLANNING_NODES_CYBER_MSG_MANAGER_H
#define AD_BYD_PLANNING_NODES_CYBER_MSG_MANAGER_H

#include "cyber/node/node.h"
#include "node_manager/msg_adapter/msg_manager.h"
#include "modules/simulator/deterministic_scheduler/deterministic_scheduler.h"

namespace ad_byd {
namespace planning {

using byd::simulator::deterministic_scheduler::SchedulerUnitManager;
using byd::simulator::deterministic_scheduler::TaskType;

#define CYBER_SUB_PTR(MSG) std::shared_ptr<apollo::cyber::Reader<MSG>>
#define CYBER_PUB_PTR(MSG) std::shared_ptr<apollo::cyber::Writer<MSG>>

class CyberMsgManager : public MsgManager {
 public:
  DECLARE_PTR(CyberMsgManager);

  CyberMsgManager(std::shared_ptr<apollo::cyber::Node> node);
  virtual void Init() override;
  virtual void Exit() override;
  virtual void PullOdoMsg() override;

 private:
  template <typename Msg>
  static CYBER_SUB_PTR(Msg)
      CreateCyberSub(Subscriber<Msg>& sub,
                     std::shared_ptr<apollo::cyber::Node> node) {
    return node->CreateReader<Msg>(
        sub.GetTopic(),
        [&sub](const std::shared_ptr<Msg>& msg) { sub.OnNewMessage(*msg); });
  }

  template <typename Msg>
  static CYBER_SUB_PTR(Msg)
      CreateOdoCyberSub(Subscriber<Msg>& sub,
                        std::shared_ptr<apollo::cyber::Node> node) {
    return node->CreateReader<Msg>(sub.GetTopic());
  }

  template <typename Msg>
  static CYBER_PUB_PTR(Msg)
      CreateCyberPub(Publisher<Msg>& pub,
                     std::shared_ptr<apollo::cyber::Node> node) {
    return node->CreateWriter<Msg>(pub.GetTopic());
  }

  CYBER_SUB_PTR(odometry_type) odometry_cyber_sub_;
  CYBER_SUB_PTR(vehicle_status_type) vehicle_status_cyber_sub_;
  CYBER_SUB_PTR(vmc_msg_type) vmc_debug_cyber_sub_;
  // CYBER_SUB_PTR(obstacle_type) obstacle_cyber_sub_;
  // CYBER_SUB_PTR(bev_map_type) bev_map_cyber_sub_;
  CYBER_SUB_PTR(env_map_type) env_map_cyber_sub_;
  CYBER_SUB_PTR(env_map_type) noa_map_cyber_sub_;
  CYBER_SUB_PTR(behavior_type) behavior_cyber_sub_;
  CYBER_SUB_PTR(prediction_type) prediction_cyber_sub_;
  CYBER_SUB_PTR(map_event_type) map_event_cyber_sub_;
#if defined(BYD_X2B) || defined(BYD_VCPB)
  CYBER_SUB_PTR(top_state_type) top_state_cyber_sub_;
#endif
  // CYBER_SUB_PTR(scenario_intention_type) scenario_intention_cyber_sub_;
  // CYBER_SUB_PTR(prediction_type) highway_prediction_cyber_sub_;
  // CYBER_SUB_PTR(road_horizon_type) road_horizon_cyber_sub_;

  // CYBER_PUB_PTR(prediction_type) prediction_cyber_pub_;
  CYBER_PUB_PTR(planning_result_type) planning_result_cyber_pub_;
  CYBER_PUB_PTR(debug_frame_type) debug_frame_cyber_pub_;
  CYBER_PUB_PTR(byd::msg::pnc::PlanTrajInfo) planning_traj_cyber_pub_;

  const std::string kTaskName = "timer_planning";
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_CYBER_MSG_MANAGER_H
