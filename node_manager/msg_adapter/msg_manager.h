
#ifndef AD_BYD_PLANNING_NODES_MSG_MANAGER_H
#define AD_BYD_PLANNING_NODES_MSG_MANAGER_H
#include <map>
#include <mutex>
#include <variant>
#include <unordered_set>

#ifdef BYD_J6
#include "cyber/event/trace_delay.h"
#endif

#include "plan_common/type_def.h"
#include "plan_common/input_frame.h"
#include "plan_common/planning_macros.h"
#include "plan_common/util/time_util.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/map_container.h"
#include "plan_common/math/pose_transform.h"

#include "node_manager/msg_adapter/publisher.h"
#include "node_manager/msg_adapter/subscriber.h"
#include "node_manager/msg_adapter/behavior_container.h"
#include "modules/simulator/deterministic_scheduler/deterministic_scheduler.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"

namespace ad_byd {
namespace planning {

using byd::simulator::deterministic_scheduler::SchedulerUnitManager;
using byd::simulator::deterministic_scheduler::TaskType;

class MsgManager {
 public:
  DECLARE_PTR(MsgManager);

  MsgManager();
  virtual ~MsgManager() = default;
  virtual void Init() { map_container_.Init(); }
  virtual void Exit() {}
  virtual void PullOdoMsg() {}

  // absl::StatusOr<PredictionInputFrame::Ptr> GetPredictionInputFrame();
  std::pair<st::planning::PlannerStatusProto::PlannerStatusCode,
            PlanningInputFrame::Ptr>
  GetPlanningInputFrame();
  /*
    // for offline extraction and test
    void OnOdometryMsg(const odometry_type& m);
    void OnVehicleStatusMsg(const vehicle_status_type& m);
    void OnObstacleMsg(const obstacle_type& m);
    void OnBevMapMsg(const bev_map_type& m);
    void OnEnvMapMsg(const env_map_type& m);
    void OnBehaviorMsg(const behavior_type& m);
    void OnPredictionMsg(const prediction_type& m);
    void OnScenarioIntentionMsg(const scenario_intention_type& m);
    void OnRoadHorizonMsg(const road_horizon_type& m);
  */
  // void PublishPredictionMsg(const std::shared_ptr<prediction_type>& m);
  void OnBehaviorMsg(const behavior_type& m);
  void OnPredictionMsg(const prediction_type& m);
  void PublishPlanningResultMsg(const std::shared_ptr<planning_result_type>& m);
  void PublishDebugFrameMsg(const std::shared_ptr<debug_frame_type>& m);
  void PublishPlanningTrajMsg(
      const std::shared_ptr<byd::msg::pnc::PlanTrajInfo>& m);
  const std::array<double, 3>& GetVehPos() const { return veh_pos_; }

  const std::string& GetOdometryTopic() const {
    return odometry_sub_.GetTopic();
  }
  const std::string& GetVehicleStatusTopic() const {
    return vehicle_status_sub_.GetTopic();
  }
  const std::string& GetVmcDebugTopic() const {
    return vmc_debug_sub_.GetTopic();
  }
  // const std::string& GetObstacleTopic() const {
  //   return obstacle_sub_.GetTopic();
  // }
  // const std::string& GetBevMapTopic() const { return bev_map_sub_.GetTopic();
  // }
  const std::string& GetEnvMapTopic() const { return env_map_sub_.GetTopic(); }
  const std::string& GetE2eMapTopic() const { return noa_map_sub_.GetTopic(); }
  const std::string& GetBehaviorTopic() const {
    return behavior_sub_.GetTopic();
  }
  const std::string& GetPredictionTopic() const {
    return prediction_sub_.GetTopic();
  }
  const std::string& GetMapEventTopic() const {
    return map_event_sub_.GetTopic();
  }
#if defined(BYD_X2B) || defined(BYD_VCPB)
  const std::string& GetTopStateTopic() const {
    return top_state_sub_.GetTopic();
  }
#endif
  // const std::string& GetScenarioIntentionTopic() const {
  //   return scenario_intention_sub_.GetTopic();
  // }
  // const std::string& GetRoadHorizonTopic() const {
  //   return road_horizon_sub_.GetTopic();
  // }

  const std::string& GetPlanningResultTopic() const {
    return planning_result_pub_.GetTopic();
  }
  const std::string& GetDebugFrameTopic() const {
    return debug_frame_pub_.GetTopic();
  }

 protected:
  void OdometryMsgUserCallback(const odometry_type& m);
  void VehicleStatusMsgUserCallback(const vehicle_status_type& m);
  void VmcDebugMsgUserCallback(const vmc_msg_type& m);
  // void ObstacleMsgUserCallback(const obstacle_type& m);
  // void BevMapMsgUserCallback(const bev_map_type& m);
  void EnvMapMsgUserCallback(const env_map_type& m);
  void NoaMapMsgUserCallback(const env_map_type& m);
  void BehaviorMsgUserCallback(const behavior_type& m);
  void PredictionMsgUserCallback(const prediction_type& m);
  void MapEventMsgUserCallback(const map_event_type& m);
#if defined(BYD_X2B) || defined(BYD_VCPB)
  void TopStateMsgUserCallback(const top_state_type& m);
#endif
  // void ScenarioIntentionMsgUserCallback(const scenario_intention_type& m);
  // void RoadHorizonMsgUserCallback(const road_horizon_type& m);

  MapInfo CreateMapInfo(const env_map_type& m, bool is_on_highway = false);
  // static EgoInfo CreateEgoInfo(const odometry_type& odom,
  //                              const vehicle_status_type& vehicle_status);
  // static ObstacleInfo CreateObstacleInfo(const obstacle_type& m);
  // static NavigationInfo CreateNavigationInfo(const road_horizon_type& m);

  QueueSubscriber<odometry_type> odometry_sub_;
  QueueSubscriber<vehicle_status_type> vehicle_status_sub_;
  QueueSubscriber<vmc_msg_type> vmc_debug_sub_;
  // QueueSubscriber<obstacle_type> obstacle_sub_;

  // SingleSubscriber<bev_map_type> bev_map_sub_;
  SingleSubscriber<env_map_type> env_map_sub_;
  SingleSubscriber<env_map_type> noa_map_sub_;
  SingleSubscriber<behavior_type> behavior_sub_;
  SingleSubscriber<prediction_type> prediction_sub_;
  SingleSubscriber<map_event_type> map_event_sub_;
#if defined(BYD_X2B) || defined(BYD_VCPB)
  SingleSubscriber<top_state_type> top_state_sub_;
#endif
  // SingleSubscriber<prediction_type> highway_prediction_sub_;
  // SingleSubscriber<scenario_intention_type> scenario_intention_sub_;
  // SingleSubscriber<road_horizon_type> road_horizon_sub_;

  // Publisher<prediction_type> prediction_pub_;
  Publisher<planning_result_type> planning_result_pub_;
  Publisher<debug_frame_type> debug_frame_pub_;
  Publisher<byd::msg::pnc::PlanTrajInfo> planning_traj_frame_pub_;

  BehaviorContainer behavior_container_;
  MapContainer map_container_;
  std::array<double, 3> veh_pos_;
  std::atomic<bool> is_in_highway_scene_{false};
  static constexpr size_t kPredictionCounter = 0;
  static constexpr size_t kPlanningCounter = 1;
#ifdef BYD_J6
  byd::msg::basic::Header canbus_header_;
  std::mutex header_mutex_;
#endif
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_MSG_MANAGER_H
