#include "node_manager/msg_adapter/cyber_msg_manager.h"

namespace ad_byd {
namespace planning {

CyberMsgManager::CyberMsgManager(std::shared_ptr<apollo::cyber::Node> node)
    : odometry_cyber_sub_(CreateOdoCyberSub(odometry_sub_, node)),
      vehicle_status_cyber_sub_(CreateCyberSub(vehicle_status_sub_, node)),
      vmc_debug_cyber_sub_(CreateCyberSub(vmc_debug_sub_, node)),
      // obstacle_cyber_sub_(CreateCyberSub(obstacle_sub_, node)),
      // bev_map_cyber_sub_(CreateCyberSub(bev_map_sub_, node)),
      env_map_cyber_sub_(CreateCyberSub(env_map_sub_, node)),
      noa_map_cyber_sub_(CreateCyberSub(noa_map_sub_, node)),
      behavior_cyber_sub_(CreateCyberSub(behavior_sub_, node)),
      prediction_cyber_sub_(CreateCyberSub(prediction_sub_, node)),
      map_event_cyber_sub_(CreateCyberSub(map_event_sub_, node)),
#if defined(BYD_X2B) || defined(BYD_VCPB)
      top_state_cyber_sub_(CreateCyberSub(top_state_sub_, node)),
#endif
      // scenario_intention_cyber_sub_(
      //     CreateCyberSub(scenario_intention_sub_, node)),
      // highway_prediction_cyber_sub_(
      //     CreateCyberSub(highway_prediction_sub_, node)),
      // road_horizon_cyber_sub_(CreateCyberSub(road_horizon_sub_, node)),
      // prediction_cyber_pub_(CreateCyberPub(prediction_pub_, node)),
      planning_result_cyber_pub_(CreateCyberPub(planning_result_pub_, node)),
      debug_frame_cyber_pub_(CreateCyberPub(debug_frame_pub_, node)),
      planning_traj_cyber_pub_(CreateCyberPub(planning_traj_frame_pub_, node)) {
  // prediction_pub_.SetUserCallback(
  //     [this](const std::shared_ptr<prediction_type>& m) {
  //       prediction_cyber_pub_->Publish(m);
  //     });
  planning_result_pub_.SetUserCallback(
      [this](const std::shared_ptr<planning_result_type>& m) {
        SchedulerUnitManager::Instance()->WriteAndCacheDetOutput(
            kTaskName, *m, planning_result_cyber_pub_);
        // planning_result_cyber_pub_->Write(m);
      });
  debug_frame_pub_.SetUserCallback(
      [this](const std::shared_ptr<debug_frame_type>& m) {
        if (m) {
          SchedulerUnitManager::Instance()->WriteAndCacheDetOutput(
              kTaskName, *m, debug_frame_cyber_pub_);
          // AWARN << "before publish planning_debug_frame time ["
          //       << apollo::cyber::Time::Now().ToSecond() << "]";
          // debug_frame_cyber_pub_->Write(m);
          // AWARN << "after publish planning_debug_frame time ["
          //       << apollo::cyber::Time::Now().ToSecond() << "]";
        }
      });
  planning_traj_frame_pub_.SetUserCallback(
      [this](const std::shared_ptr<byd::msg::pnc::PlanTrajInfo>& m) {
        SchedulerUnitManager::Instance()->WriteAndCacheDetOutput(
            kTaskName, *m, planning_traj_cyber_pub_);
        // planning_traj_cyber_pub_->Write(m);
      });
}

void CyberMsgManager::Init() {
  MsgManager::Init();
  // odometry_cyber_sub_->Subscribe();
  // vehicle_status_cyber_sub_->Subscribe();
  // obstacle_cyber_sub_->Subscribe();
  // bev_map_cyber_sub_->Subscribe();
  // env_map_cyber_sub_->Subscribe();
  // behavior_cyber_sub_->Subscribe();
  // prediction_cyber_sub_->Subscribe();
  // scenario_intention_cyber_sub_->Subscribe();
  // highway_prediction_cyber_sub_->Subscribe();
  // road_horizon_cyber_sub_->Subscribe();
}

void CyberMsgManager::Exit() {
  odometry_cyber_sub_->Shutdown();
  vehicle_status_cyber_sub_->Shutdown();
  vmc_debug_cyber_sub_->Shutdown();
  // obstacle_cyber_sub_->Shutdown();
  // bev_map_cyber_sub_->Shutdown();
  env_map_cyber_sub_->Shutdown();
  noa_map_cyber_sub_->Shutdown();
  behavior_cyber_sub_->Shutdown();
  prediction_cyber_sub_->Shutdown();
  map_event_cyber_sub_->Shutdown();
#if defined(BYD_X2B) || defined(BYD_VCPB)
  top_state_cyber_sub_->Shutdown();
#endif
  // scenario_intention_cyber_sub_->Shutdown();
  // highway_prediction_cyber_sub_->Shutdown();
  // road_horizon_cyber_sub_->Shutdown();
}

void CyberMsgManager::PullOdoMsg() {
  odometry_cyber_sub_->Observe();
  auto odo_msg = odometry_cyber_sub_->GetLatestObserved();
  if (odo_msg != nullptr) {
    odometry_sub_.OnNewMessage(*odo_msg);
  } else {
    LOG(ERROR) << "get odo_msg fail!";
  }
}

}  // namespace planning
}  // namespace ad_byd