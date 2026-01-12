#include "msg/cyber/supported_types_cyber.h"

#include "modules/msg/st_msgs/planning_debug_frame.pb.h"
#include "modules/msg/st_msgs/planning_result.pb.h"
#include "modules/msg/st_msgs/sm_behavior.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/drivers_msgs/canbus_uplink.pb.h"
#include "modules/msg/prediction_msgs/pred_obj_info_v2.pb.h"
#include "modules/msg/perception_msgs/dynamic_common.pb.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"
#include "modules/msg/planning_msgs/plan_traj_info.pb.h"
#include "modules/msg/orin_msgs/vmc_msgs.pb.h"
#include "modules/msg/orin_msgs/map_event.pb.h"
#include "modules/msg/state_machine_msgs/top_state.pb.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"
#include "modules/msg/environment_model_msgs/local_map_info.pb.h"

std::string GetTypeName(const google::protobuf::Descriptor* descriptor) {
  return descriptor->full_name();
}

// #define PROTO_MSG_TYPE_DATA_SUPPORT(NAME) \
//   {GetTypeName(NAME),                     \
//    std::shared_ptr<const google::protobuf::Descriptor>(std::move(NAME))}

std::map<std::string, const google::protobuf::Descriptor*> supported_types_map =
    {
        {GetTypeName(byd::msg::drivers::CanbusUpLink::descriptor()),
         byd::msg::drivers::CanbusUpLink::descriptor()},
        {GetTypeName(byd::msg::drivers::VehInfo::descriptor()),
         byd::msg::drivers::VehInfo::descriptor()},
        {GetTypeName(
             byd::msg::localization::LocalizationEstimate::descriptor()),
         byd::msg::localization::LocalizationEstimate::descriptor()},
        {GetTypeName(byd::msg::orin::vmc_msgs::MsgVmcDebug::descriptor()),
         byd::msg::orin::vmc_msgs::MsgVmcDebug::descriptor()},
        {GetTypeName(byd::msg::orin::routing_map::MapEvent::descriptor()),
         byd::msg::orin::routing_map::MapEvent::descriptor()},
        {GetTypeName(byd::msg::orin::routing_map::RoutingMap::descriptor()),
         byd::msg::orin::routing_map::RoutingMap::descriptor()},
        {GetTypeName(byd::msg::env_model::LocalMapInfo::descriptor()),
         byd::msg::env_model::LocalMapInfo::descriptor()},
        {GetTypeName(byd::msg::perception::PerceptionObstacles::descriptor()),
         byd::msg::perception::PerceptionObstacles::descriptor()},
        {GetTypeName(byd::msg::pnc::PlanTrajInfo::descriptor()),
         byd::msg::pnc::PlanTrajInfo::descriptor()},
        {GetTypeName(byd::msg::prediction::ObjectsPrediction::descriptor()),
         byd::msg::prediction::ObjectsPrediction::descriptor()},
        {GetTypeName(byd::msg::planning::PLanningResultProto::descriptor()),
         byd::msg::planning::PLanningResultProto::descriptor()},
        {GetTypeName(byd::msg::planning::DebugFrameProto::descriptor()),
         byd::msg::planning::DebugFrameProto::descriptor()},
        {GetTypeName(byd::msg::planning::SMBehavior::descriptor()),
         byd::msg::planning::SMBehavior::descriptor()},
        {GetTypeName(byd::msg::state_machine::TopState::descriptor()),
         byd::msg::state_machine::TopState::descriptor()},
};

const std::map<std::string, const google::protobuf::Descriptor*>&
getSupportedCyberTypes() {
  return supported_types_map;
}