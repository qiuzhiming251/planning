
#ifndef AD_BYD_PLANNING_COMMON_INPUT_FRAME_H
#define AD_BYD_PLANNING_COMMON_INPUT_FRAME_H
#include <list>
#include <memory>

#include "plan_common/maps/ld_lite_map.h"
#include "plan_common/maps/map_element.h"
#include "plan_common/planning_macros.h"
#include "plan_common/type_def.h"
#include "modules/msg/drivers_msgs/canbus_uplink.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/orin_msgs/map_event.pb.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"
#include "modules/msg/orin_msgs/vmc_msgs.pb.h"
#include "modules/msg/planning_msgs/plan_traj_info.pb.h"
#include "modules/msg/prediction_msgs/pred_obj_info_v2.pb.h"
#include "modules/msg/st_msgs/map_info.pb.h"
#include "modules/msg/st_msgs/planning_debug_frame.pb.h"
#include "modules/msg/st_msgs/planning_result.pb.h"
#include "modules/msg/st_msgs/sm_behavior.pb.h"
#if defined(BYD_X2B) || defined(BYD_VCPB)
#include "modules/msg/state_machine_msgs/top_state.pb.h"
#endif

namespace ad_byd {
namespace planning {

using odometry_type = byd::msg::localization::LocalizationEstimate;
using vehicle_status_type = byd::msg::drivers::CanbusUpLink;
// using env_map_type = byd::msg::planning::MapInfo;
using env_map_type = byd::msg::orin::routing_map::RoutingMap;
using env_lane_type = byd::msg::orin::routing_map::LaneInfo;
using behavior_type = byd::msg::planning::SMBehavior;
using planning_result_type = byd::msg::planning::PLanningResultProto;
using debug_frame_type = byd::msg::planning::DebugFrameProto;
using planning_trajectory_type = byd::msg::planning::Trajectory;
using prediction_type = byd::msg::prediction::ObjectsPrediction;
using vmc_msg_type = byd::msg::orin::vmc_msgs::MsgVmcDebug;
using map_event_type = byd::msg::orin::routing_map::MapEvent;
#if defined(BYD_X2B) || defined(BYD_VCPB)
using top_state_type = byd::msg::state_machine::TopState;
#endif
struct EgoInfo {
  int64_t seq_num{0};
  double stamp{0};
  Point2d pos;
  Point2d vel;
  Point2d acc;
  double heading{0};
  double steering_angle{0};
  double length{0};
  double width{0};
  double height{0};
};

struct DynamicObstacleInfo {
  int64_t seq_num{0};
  std::string id;
  ObstacleType type{OBJECT_UNKNOWN};
  double stamp{0};
  double length{0};
  double width{0};
  double height{0};
  double heading{0};
  double yaw_rate{0};
  double life_time{0};
  double confidence{0};
  Point2d pos;
  Point2d vel;
  Point2d acc;
  std::vector<Point2d> polygon;
  int8_t obstacle_state;
  int8_t fusion_type{0};
  std::vector<ObstacleLightType>
      obstacle_lights;  // [left_turn_lights, right_turn_lights, brake_lights,
                        // hazard_lights]
  std::vector<double> obstacle_lights_conf;
};

struct StaticObstacleInfo {
  double stamp{0};
  int64_t seq_num{0};
  std::string id;
  Point2d pos;
  Point2d vel;
  Point2d acc;
  double heading{0};
  double length{0};
  double width{0};
  double height{0};
  ObstacleType type{OBJECT_UNKNOWN};
  int8_t obstacle_state;
  int8_t fusion_type{0};
  std::vector<Point2d> polygon;
};

struct ObstacleInfo {
  int64_t seq_num{0};
  int8_t coordinate_type;
  // env_model_idls::idls::Pose pose_odom;
  // std::vector<env_model_idls::idls::OcculusionArea> occulusion_areas;
  std::vector<DynamicObstacleInfo> dynamic_obs;
  std::vector<StaticObstacleInfo> static_obs;
};

struct NavigationInfo {
  int64_t seq_num{0};
  std::vector<TurnInfo> turn_infos;
  std::vector<RoadClassInfo> road_class_infos;
  std::vector<LaneGroupInfo> lane_group_infos;
  std::vector<FormOfWayInfo> form_of_way_infos;
  bool has_navigation = false;
};

struct PredictionInputFrame {
  DECLARE_PTR(PredictionInputFrame);

  std::vector<EgoInfo> ego_infos;
  std::vector<ObstacleInfo> obstacle_infos;
  EgoInfo latest_ego_info;
  NavigationInfo navigation_infos;
  MapElement map_element;
  double start_time;
  double head_ts;
  int64_t odom_seq;
  int64_t vehicle_seq;
  std::shared_ptr<prediction_type> highway_prediction;
  std::shared_ptr<behavior_type> behavior;
  // std::shared_ptr<env_map_type> env_map;
};

struct PlanningInputFrame {
  DECLARE_PTR(PlanningInputFrame);

  double last_odometry_timestamp{0};
  double last_msg_timestamp{0};
  double estimate_a{0};

  std::shared_ptr<odometry_type> odometry = nullptr;
  std::shared_ptr<vehicle_status_type> vehicle_status = nullptr;
  std::shared_ptr<vmc_msg_type> vmc_debug = nullptr;
  std::shared_ptr<behavior_type> behavior = nullptr;
  std::shared_ptr<prediction_type> prediction = nullptr;
  std::shared_ptr<map_event_type> map_event = nullptr;
  MapPtr map_ptr = nullptr;
  LdLiteMapPtr lite_map_ptr = nullptr;
  std::shared_ptr<env_map_type> env_map = nullptr;
#if defined(BYD_X2B) || defined(BYD_VCPB)
  std::shared_ptr<top_state_type> top_state = nullptr;
#endif
  // std::shared_ptr<scenario_intention_type> scenario_intention = nullptr;
  bool use_hd_map = false;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_COMMON_INPUT_FRAME_H
