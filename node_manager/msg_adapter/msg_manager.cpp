

#include "node_manager/msg_adapter/msg_manager.h"

#include <cmath>
#include <set>
#include <unordered_map>
#include <utility>
#include <cereal/archives/json.hpp>

#include "plan_common/math/math_utils.h"
#include "plan_common/gflags.h"
#include "plan_common/log.h"
#include "plan_common/obstacle.h"
#include "plan_common/timer.h"
#include "plan_common/serialization/map_serialization.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
// #define MAPTOPIC

namespace ad_byd {
namespace planning {

bool IsStatic(uint8_t type) {
  return type == OBJECT_CONE || type == OBJECT_BARREL ||
         type == OBJECT_BARRIER || type == OBJECT_SAFETY_TRIANGLE ||
         type == OBJECT_PARKING_LOCK || type == OBJECT_SPACE_LIMITER ||
         type == OBJECT_UNKNOWN_UNMOVABLE;
}

/*
#define LOG_LEVEL_MAPPING(LEVEL) \
  ((LEVEL) == google::WARNING ? LOG_WARN : \
   (LEVEL) == google::FATAL   ? LOG_FATAL : \
   LOG_##LEVEL)

#define LOG_STAMP(LEVEL) LOG_LEVEL_MAPPING(LEVEL) << std::fixed <<
std::setprecision(4)
*/

MsgManager::MsgManager()
    : odometry_sub_("/localization/dr", 100,
                    [](const odometry_type& msg) {
                      return msg.header().measurement_timestamp();
                    }),
      vehicle_status_sub_("/drivers/canbus/canbus_uplink", 50,
                          [](const vehicle_status_type& msg) {
                            return msg.header().measurement_timestamp();
                          }),
      vmc_debug_sub_("/mpc_interface_drv_vmc_Debug", 100,
                     [](const vmc_msg_type& msg) {
                       return msg.header().measurement_timestamp();
                     }),
// obstacle_sub_("rt/fusion/odometry/obstacles", 10,
//               [](const obstacle_type& msg) {
//                 return msg.header().measurement_timestamp();
//               }),
// bev_map_sub_("rt/vs/fsd_map",
//              [](const bev_map_type& msg) {
//                return msg.header().publish_timestamp();
//              }),
#ifdef MAPTOPIC
      env_map_sub_("/noa_map/routing_map",
                   [](const env_map_type& msg) {
                     return msg.header().measurement_timestamp();
                   }),
#else
      env_map_sub_("/perception/env/routing_map",
                   [](const env_map_type& msg) {
                     return msg.header().measurement_timestamp();
                   }),
#endif
      noa_map_sub_("/noa_map/routing_map",
                   [](const env_map_type& msg) {
                     return msg.header().measurement_timestamp();
                   }),
      behavior_sub_("/st/pnc/sm_behavior",
                    [](const behavior_type& msg) {
                      return msg.header().measurement_timestamp();
                    }),
      prediction_sub_("/prediction/trajectory_v2",
                      [](const prediction_type& msg) {
                        return msg.header().measurement_timestamp();
                      }),
      map_event_sub_("/noa_map/map_event",
                     [](const map_event_type& msg) {
                       return msg.header().measurement_timestamp();
                     }),
#if defined(BYD_X2B) || defined(BYD_VCPB)
      top_state_sub_("/state_machine/top_state/state",
                     [](const top_state_type& msg) {
                       return msg.header().measurement_timestamp();
                     }),
#endif
      // highway_prediction_sub_("rt/prediction/highway_obstacles",
      //                         [](const prediction_type& m) {
      //                           return msg.header().publish_timestamp();
      //                         }),
      // scenario_intention_sub_("Perception/ScenarioIntention",
      //                         [](const scenario_intention_type& m) {
      //                           return m.header().publish_timestamp();
      //                         }),
      // road_horizon_sub_("rt/ehorizon/sd_road_horizon",
      //                   [](const road_horizon_type& m) {
      //                     return msg.header().publish_timestamp();
      //                   }),
      // prediction_pub_(prediction_sub_.GetTopic()),
      planning_result_pub_("/st/pnc/pilot_planning_result"),
      debug_frame_pub_("/st/pnc/planning_debugframe"),
      planning_traj_frame_pub_("/plan/trajectory") {
  // set user callback
  odometry_sub_.SetUserCallback(
      [this](const odometry_type& m) { OdometryMsgUserCallback(m); });
  vehicle_status_sub_.SetUserCallback([this](const vehicle_status_type& m) {
    VehicleStatusMsgUserCallback(m);
  });
  vmc_debug_sub_.SetUserCallback(
      [this](const vmc_msg_type& m) { VmcDebugMsgUserCallback(m); });
  // obstacle_sub_.SetUserCallback(
  //     [this](const obstacle_type& m) { ObstacleMsgUserCallback(m); });
  // bev_map_sub_.SetUserCallback(
  //     [this](const bev_map_type& m) { BevMapMsgUserCallback(m); });
  env_map_sub_.SetUserCallback(
      [this](const env_map_type& m) { EnvMapMsgUserCallback(m); });
  noa_map_sub_.SetUserCallback(
      [this](const env_map_type& m) { NoaMapMsgUserCallback(m); });
  behavior_sub_.SetUserCallback(
      [this](const behavior_type& m) { BehaviorMsgUserCallback(m); });
  prediction_sub_.SetUserCallback([this](const prediction_type& m) {
    PredictionMsgUserCallback(m);
    // byd::log::Warn(
    //     "fsd-planning", "receive prediction [%" PRId64 "] time [%f]",
    //     m.header().seq(), byd::utils::time::NowMilliseconds() / 1000.0);
  });
  map_event_sub_.SetUserCallback(
      [this](const map_event_type& m) { MapEventMsgUserCallback(m); });
#if defined(BYD_X2B) || defined(BYD_VCPB)
  top_state_sub_.SetUserCallback(
      [this](const top_state_type& m) { TopStateMsgUserCallback(m); });
#endif

  // highway_prediction_sub_.SetUserCallback(
  //     [this](const prediction_type& m) { PredictionMsgUserCallback(m); });
  // scenario_intention_sub_.SetUserCallback(
  //     [this](const scenario_intention_type& m) {
  //       ScenarioIntentionMsgUserCallback(m);
  //     });
  // road_horizon_sub_.SetUserCallback(
  //     [this](const road_horizon_type& m) { RoadHorizonMsgUserCallback(m); });
  // prediction counter
  // odometry_sub_.RegisterCounter(kPredictionCounter, 3);
  // vehicle_status_sub_.RegisterCounter(kPredictionCounter, 3);
  // obstacle_sub_.RegisterCounter(kPredictionCounter, 4);
  // highway_prediction_sub_.RegisterCounter(kPredictionCounter, 3);
  // behavior_sub_.RegisterCounter(kPredictionCounter, 3);
  // road_horizon_sub_.RegisterCounter(kPredictionCounter, 3);
  // planning counter
  odometry_sub_.RegisterCounter(kPlanningCounter, 3);
  vehicle_status_sub_.RegisterCounter(kPlanningCounter, 3);
  vmc_debug_sub_.RegisterCounter(kPlanningCounter, 3);
  env_map_sub_.RegisterCounter(kPlanningCounter, 10);
  noa_map_sub_.RegisterCounter(kPlanningCounter, 10);
  prediction_sub_.RegisterCounter(kPlanningCounter, 4);
  behavior_sub_.RegisterCounter(kPlanningCounter, 3);
  map_event_sub_.RegisterCounter(kPlanningCounter, 3);
#if defined(BYD_X2B) || defined(BYD_VCPB)
  top_state_sub_.RegisterCounter(kPlanningCounter, 3);
#endif
  // scenario_intention_sub_.RegisterCounter(kPlanningCounter, 3);
}
/*
void MsgManager::OnOdometryMsg(const odometry_type& m) {
  odometry_sub_.OnNewMessage(m);
}
void MsgManager::OnVehicleStatusMsg(const vehicle_status_type& m) {
  vehicle_status_sub_.OnNewMessage(m);
}
void MsgManager::OnObstacleMsg(const obstacle_type& m) {
  obstacle_sub_.OnNewMessage(m);
}
void MsgManager::OnBevMapMsg(const bev_map_type& m) {
  bev_map_sub_.OnNewMessage(m);
}
void MsgManager::OnEnvMapMsg(const env_map_type& m) {
  env_map_sub_.OnNewMessage(m);
}
void MsgManager::OnBehaviorMsg(const behavior_type& m) {
  behavior_sub_.OnNewMessage(m);
}
void MsgManager::OnPredictionMsg(const prediction_type& m) {
  prediction_sub_.OnNewMessage(m);
}
void MsgManager::OnScenarioIntentionMsg(const scenario_intention_type& m) {
  scenario_intention_sub_.OnNewMessage(m);
}
void MsgManager::OnRoadHorizonMsg(const road_horizon_type& m) {
  road_horizon_sub_.OnNewMessage(m);
}
*/
void MsgManager::OdometryMsgUserCallback(const odometry_type& m) {
  std::string task_name = "plan_localization_reader";
  std::string topic = GetOdometryTopic();
  READER_REGISTER_TASK_INSTANCE(task_name, topic, m, void());
}

// void MsgManager::RoadHorizonMsgUserCallback(const road_horizon_type& m) {}

void MsgManager::VehicleStatusMsgUserCallback(const vehicle_status_type& m) {
  std::string task_name = "plan_canbus_reader";
  std::string topic = GetVehicleStatusTopic();
  READER_REGISTER_TASK_INSTANCE(task_name, topic, m, void());
#ifdef BYD_J6
  {
    std::lock_guard<std::mutex> canbus_lock(header_mutex_);
    canbus_header_ = m.header();
    TRACE_INFO_ADD_TIME_POINT(rv3_planning_begin,
                              apollo::cyber::Time::Now().ToSecond(),
                              canbus_header_);
  }
#endif
}

void MsgManager::VmcDebugMsgUserCallback(const vmc_msg_type& m) {
  std::string task_name = "plan_vmc_reader";
  std::string topic = GetVmcDebugTopic();
  READER_REGISTER_TASK_INSTANCE(task_name, topic, m, void());
}
// void MsgManager::ObstacleMsgUserCallback(const obstacle_type&) {}

// void MsgManager::BevMapMsgUserCallback(const bev_map_type& m) {
//   map_container_.InsertMap(CreateMapInfo(m));
// }
void MsgManager::EnvMapMsgUserCallback(const env_map_type& m) {
  SCOPED_TRACE(__FUNCTION__);
  std::string task_name = "plan_map_reader";
  std::string topic = GetEnvMapTopic();
  READER_REGISTER_TASK_INSTANCE(task_name, topic, m, void());
  auto behavior = behavior_container_.Get();
  bool use_hd_map =
      behavior.function_id == byd::msg::planning::FunctionId::FUNCTION_HW_NOA ||
      behavior.function_id ==
          byd::msg::planning::FunctionId::FUNCTION_CITY_NOA ||
      behavior.function_id == byd::msg::planning::FunctionId::FUNCTION_LKA_PLUS;
#ifdef OFFLINE
  use_hd_map = true;
#endif
  bool is_on_highway = m.is_on_highway();
  is_in_highway_scene_ = is_on_highway;
  // auto map_info = CreateMapInfo(use_hd_map ? m.hd_map() : m.perception_map(),
  //                               is_on_highway);
  // LOG_ERROR << "is_in_highway_scene_1: " << is_in_highway_scene_;
  auto map_info = CreateMapInfo(m, is_on_highway);

#ifdef SERIALIZE_MACRO
  std::ofstream os("map_info.json");
  cereal::JSONOutputArchive archive(os);
  archive(map_info);
#endif

  auto& ehp_v2 = map_info.v2_info;
  // if (use_hd_map) {
  //   ehp_v2.dist_to_ramp = m.hd_map().v2_ramp_info().dist();
  //   LOG_INFO << "DistToRamp: " << ehp_v2.dist_to_ramp;
  // }

  if (m.env_info().v2_valid()) {
    // auto& ehp_v2 = map_info.v2_info;
    ehp_v2.is_valid = true;
    ehp_v2.has_navigation = m.env_info().v2_has_navigation();
    ehp_v2.dist_to_ramp = m.env_info().v2_dist_to_ramp();
    ehp_v2.dist_to_toll = m.env_info().v2_dist_to_toll();
    ehp_v2.dist_to_tunnel = m.env_info().v2_dist_to_tunnel();
    ehp_v2.dist_to_subpath = m.env_info().dist_to_subpath();
    ehp_v2.dist_to_route_split = m.env_info().dist_to_split_routelanenum_dec();
    // road class
    for (const auto& rc : m.env_info().v2_road_classes()) {
      V2RoadClass road_class_tmp;
      road_class_tmp.start_s = rc.start();
      road_class_tmp.end_s = rc.end();
      road_class_tmp.type =
          static_cast<V2RoadClass::V2RoadClassType>(rc.road_class());
      ehp_v2.road_class.push_back(road_class_tmp);
    }

    // traffic flow
    for (const auto& tf : m.env_info().traffic_flows()) {
      V2TrafficFlow traffic_flow_tmp;
      traffic_flow_tmp.start_s = tf.start_s();
      traffic_flow_tmp.end_s = tf.end_s();
      traffic_flow_tmp.type =
          static_cast<V2TrafficFlow::V2TrafficFlowType>(tf.type());
      ehp_v2.traffic_flow.push_back(traffic_flow_tmp);
    }
    // turn type
    for (const auto& ti : m.env_info().v2_turn_info()) {
      // if (ti.id() == "planning") {
      V2TurnInfo turn_info_tmp;
      turn_info_tmp.id = ti.id();
      turn_info_tmp.is_valid = ti.is_valid();
      turn_info_tmp.turn_type =
          static_cast<V2TurnInfo::V2TurnType>(ti.turn_type());
      turn_info_tmp.detail_turn_type =
          static_cast<V2TurnInfo::V2DetailTurnType>(ti.detail_turn_type());
      turn_info_tmp.dist = ti.dist();
      turn_info_tmp.original_dist = ti.v2_dist();
      turn_info_tmp.before_turn.road_class =
          static_cast<int>(ti.before_turn().road_class());
      turn_info_tmp.before_turn.lane_num =
          static_cast<int>(ti.before_turn().lane_num());
      turn_info_tmp.after_turn.road_class =
          static_cast<int>(ti.after_turn().road_class());
      turn_info_tmp.after_turn.lane_num =
          static_cast<int>(ti.after_turn().lane_num());
      // Log2DDS::LogDataV2("test", absl::StrCat("id: ", turn_info_tmp.id));
      // Log2DDS::LogDataV2("test",
      //                    absl::StrCat("dist: ", turn_info_tmp.dist));
      // for (const auto& info : ti.info_add()) {
      //   turn_info_tmp.infos.push_back(info);
      // turn_info_tmp.straight_pass_info.curr_index =
      //     static_cast<int>(ti.straight_pass_info().curr_index());
      // turn_info_tmp.straight_pass_info.index_num =
      //     static_cast<int>(ti.straight_pass_info().index_num());
      ehp_v2.turn_info.push_back(turn_info_tmp);
    }
    // curvature
    for (const auto& curv : m.env_info().v2_curvatures()) {
      V2Curvature curvature_tmp;
      curvature_tmp.curvature = curv.curvature();
      curvature_tmp.distance = curv.distance();
      ehp_v2.v2_curvatures.push_back(curvature_tmp);
    }
  }

  map_container_.InsertMap(map_info);
}

void MsgManager::NoaMapMsgUserCallback(const env_map_type& m) {
  if (!FLAGS_planner_enable_ld_lite_map) return;
  SCOPED_TRACE(__FUNCTION__);
  // if (m.header().sequence_num() % 2 != 0) return;
  LdLiteMapInfo lite_map_info;
  lite_map_info.seq = m.header().sequence_num();
  const auto& sd_route = m.sd_route();
  for (const auto& mpp_section : sd_route.mpp_sections()) {
    LdLiteSectionInfo lite_section;
    lite_section.id = mpp_section.id();
    lite_section.length = mpp_section.length();
    for (const auto& lane_group : mpp_section.lane_group_idx()) {
      LdLiteLaneGroupIndex lane_group_info;
      lane_group_info.id = lane_group.id();
      lane_group_info.start_range = lane_group.start_range_offset();
      lane_group_info.end_range = lane_group.end_range_offset();
      lite_section.lane_group_idx.push_back(lane_group_info);
    }
    lite_map_info.mpp_info.push_back(lite_section);
  }

  lite_map_info.navi_start.section_id = sd_route.navi_start().section_id();
  lite_map_info.navi_start.s_offset = sd_route.navi_start().s_offset();

  for (const auto& sd_lane_group : m.sd_lane_groups()) {
    LdLiteLaneGroupInfo lane_group_info;
    lane_group_info.id = sd_lane_group.id();
    lane_group_info.lane_num = sd_lane_group.lane_num();
    lane_group_info.length = sd_lane_group.length();
    for (const uint64_t next_l_group :
         sd_lane_group.successor_lane_group_ids()) {
      lane_group_info.next_lane_group.push_back(next_l_group);
    }
    for (const uint64_t pre_l_group :
         sd_lane_group.predecessor_lane_group_ids()) {
      lane_group_info.previous_lane_group.push_back(pre_l_group);
    }
    int lane_info_size = sd_lane_group.lane_info().size();
    for (int i = 0; i < lane_info_size; i++) {
      LdLiteLaneInfo lite_lane_info;
      const auto& sd_lane_info = sd_lane_group.lane_info(i);
      lite_lane_info.id = sd_lane_info.id();
      lite_lane_info.lane_group_id = sd_lane_group.id();
      lite_lane_info.lane_seq = i + 1;
      lite_lane_info.type = static_cast<LaneType>(sd_lane_info.type());
      lite_lane_info.turn_type =
          static_cast<LiteTurnType>(sd_lane_info.turn_type());
      for (const auto next_l_id : sd_lane_info.next_lane_ids()) {
        lite_lane_info.next_lanes.push_back(next_l_id);
      }
      for (const auto pre_l_id : sd_lane_info.previous_lane_ids()) {
        lite_lane_info.previous_lanes.push_back(pre_l_id);
      }
      lane_group_info.lane_info.push_back(lite_lane_info);
    }
    lite_map_info.lane_group_info.push_back(lane_group_info);
  }

  map_container_.InsertLdLiteMap(lite_map_info);
}

void MsgManager::BehaviorMsgUserCallback(const behavior_type& m) {
  std::string task_name = "plan_behavior_reader";
  std::string topic = GetBehaviorTopic();
  READER_REGISTER_TASK_INSTANCE(task_name, topic, m, void());
  bool on_high_way = is_in_highway_scene_;
  behavior_container_.OnMsg(m, on_high_way);
}

void MsgManager::PredictionMsgUserCallback(const prediction_type& m) {
  std::string task_name = "plan_prediction_reader";
  std::string topic = GetPredictionTopic();
  READER_REGISTER_TASK_INSTANCE(task_name, topic, m, void());
}

void MsgManager::MapEventMsgUserCallback(const map_event_type& m) {
  std::string task_name = "plan_map_event_reader";
  std::string topic = GetMapEventTopic();
  READER_REGISTER_TASK_INSTANCE(task_name, topic, m, void());
}

#if defined(BYD_X2B) || defined(BYD_VCPB)
void MsgManager::TopStateMsgUserCallback(const top_state_type& m) {
  std::string task_name = "plan_top_state_reader";
  std::string topic = GetTopStateTopic();
  READER_REGISTER_TASK_INSTANCE(task_name, topic, m, void());
}
#endif

// void MsgManager::ScenarioIntentionMsgUserCallback(
//     const scenario_intention_type& m) {}

// void MsgManager::PublishPredictionMsg(
//     const std::shared_ptr<prediction_type>& m) {
//   prediction_pub_.Publish(m);
// }
void MsgManager::PublishPlanningResultMsg(
    const std::shared_ptr<planning_result_type>& m) {
#ifdef BYD_J6
  {
    std::lock_guard<std::mutex> canbus_lock(header_mutex_);
    TRACE_INFO_COPY(canbus_header_, m->mutable_header());
  }
  TRACE_INFO_ADD_TIME_POINT(rv3_planning_end,
                            apollo::cyber::Time::Now().ToSecond(),
                            (*(m->mutable_header())));
#endif
  planning_result_pub_.Publish(m);
}
void MsgManager::PublishDebugFrameMsg(
    const std::shared_ptr<debug_frame_type>& m) {
  debug_frame_pub_.Publish(m);
}
void MsgManager::PublishPlanningTrajMsg(
    const std::shared_ptr<byd::msg::pnc::PlanTrajInfo>& m) {
  planning_traj_frame_pub_.Publish(m);
}

TurnType LaneTurnTypeAdaptor(env_lane_type::TurnType env_lane_type) {
  switch (env_lane_type) {
    case env_lane_type::NO_TURN:
      return TurnType::NO_TURN;
    case env_lane_type::LEFT_TURN:
    case env_lane_type::STRAIGHT_AND_LEFT_TURN:
    case env_lane_type::LEFT_TURN_AND_RIGHT_TURN:
      return TurnType::LEFT_TURN;
    case env_lane_type::RIGHT_TURN:
    case env_lane_type::STRAIGHT_AND_RIGHT_TURN:
      return TurnType::RIGHT_TURN;
    case env_lane_type::U_TURN:
    case env_lane_type::STRAIGHT_AND_U_TURN:
    case env_lane_type::LEFT_TURN_AND_U_TURN:
    case env_lane_type::RIGHT_TURN_AND_U_TURN:
      return TurnType::U_TURN;
    default:
      return TurnType::NO_TURN;
  }
}

MapInfo MsgManager::CreateMapInfo(const env_map_type& msg, bool is_on_highway) {
  MapInfo map_info{};
  // basic information
  map_info.type = static_cast<MapType>(msg.map_type());
  // map_info.sub_type = static_cast<MapSubType>(msg.sub_type());

  map_info.is_on_highway = is_on_highway;
  // Route
  std::unordered_set<uint64_t> route_circle_check;
  route_circle_check.clear();
  map_info.timestamp = msg.header().measurement_timestamp();
  map_info.seq = msg.header().sequence_num();
  map_info.route.id = msg.route().id();

#ifdef MAPTOPIC
  // trsnsform
  auto odom_msg_from_map = odometry_sub_.GetMsgAt(map_info.timestamp);
  auto odom_msg_lastest = odometry_sub_.Back(kPlanningCounter);
  auto odom_msg = odom_msg_lastest;
  if (!odom_msg_from_map.ok()) {
    LOG_ERROR << "****************timestamp: " << map_info.timestamp;
    // return map_info;
  } else {
    odom_msg = odom_msg_from_map;
  }

  if (!odom_msg.ok()) {
    LOG_ERROR << "****************timestamp: " << map_info.timestamp;
    return map_info;
  }

  auto odom_ego = odom_msg->second;

  double yaw = math::QuaternionToYaw(
      odom_ego->pose().orientation().qw(), odom_ego->pose().orientation().qx(),
      odom_ego->pose().orientation().qy(), odom_ego->pose().orientation().qz());
  double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
  // PoseTransform pose_transform(TransformInfo(
  //     odom_ego->pose().position().x(), odom_ego->pose().position().y(),
  //     yaw));
  auto TransPointVCSToDR = [&](double x, double y) {
    double odom_x = odom_ego->pose().position().x() + x * cos_yaw - y * sin_yaw;
    double odom_y = odom_ego->pose().position().y() + x * sin_yaw + y * cos_yaw;
    return Point2d(odom_x, odom_y);
  };
#endif

  // data
  double front_navi_distance = 0.;
  bool navi_start_found = false;
  constexpr double kMaxRouteDistance = 2000.0;
  for (const auto& section_env : msg.route().sections()) {
    SectionInfo section_tmp{};
    std::size_t last_size = route_circle_check.size();
    route_circle_check.insert(section_env.id());
    if (route_circle_check.size() == last_size) {
      Log2DDS::LogDataV2("route_debug",
                         absl::StrCat("Navi route circle! Circle_section: ",
                                      section_env.id()));
      break;
    }
    if (navi_start_found) {
      front_navi_distance += section_env.length();
    } else {
      if (msg.route().navi_start().section_id() == section_env.id()) {
        front_navi_distance = msg.route().navi_start().s_offset();
        navi_start_found = true;
      }
    }
    if (map_info.type == MapType::HD_MAP && !is_on_highway &&
        front_navi_distance > kMaxRouteDistance)
      break;

    section_tmp.id = section_env.id();
    section_tmp.length = section_env.length();
    for (int i = 0; i < section_env.lane_ids_size(); i++) {
      section_tmp.lane_ids.emplace_back(section_env.lane_ids(i));
    }
    section_tmp.road_class =
        static_cast<MapRoadClass>(section_env.road_class());
    map_info.route.sections.emplace_back(std::move(section_tmp));
  }

  int cur_size = map_info.route.sections.size();
  for (int i = 0; i < msg.route().extend_sections().size(); ++i) {
    PathExtend path_extend_tmp{};
    const auto& path_extend = msg.route().extend_sections().at(i);
    int from_section_idx = path_extend.from_section_idx() < 0
                               ? -1
                               : path_extend.from_section_idx();
    int to_section_idx = path_extend.to_section_idx();
    if (to_section_idx <= 0 || to_section_idx >= cur_size ||
        from_section_idx >= to_section_idx) {
      continue;
    }
    path_extend_tmp.from_section_idx = from_section_idx;
    path_extend_tmp.to_section_idx = to_section_idx;
    path_extend_tmp.from_section_id = path_extend.from_section_id();
    for (const auto& extend_section : path_extend.sections()) {
      SectionInfo section_tmp{};
      std::size_t last_size = route_circle_check.size();
      route_circle_check.insert(extend_section.id());
      if (route_circle_check.size() == last_size) {
        LOG_ERROR << "Navi route circle!";
        break;
      }
      section_tmp.id = extend_section.id();
      // LOG(ERROR) << " section_tmp.id: " << section_tmp.id;
      section_tmp.length = extend_section.length();
      for (int i = 0; i < extend_section.lane_ids_size(); i++) {
        // LOG(ERROR) << " extend section lane id: " <<
        // extend_section.lane_ids(i);
        section_tmp.lane_ids.emplace_back(extend_section.lane_ids(i));
      }
      section_tmp.road_class =
          static_cast<MapRoadClass>(extend_section.road_class());
      path_extend_tmp.extend_sections.emplace_back(std::move(section_tmp));
    }
    map_info.route.extend_sections_vec.emplace_back(std::move(path_extend_tmp));
    for (uint64_t j = from_section_idx + 1; j >= 0 && j < to_section_idx; ++j) {
      map_info.route.sections[j].is_neighbor_extend = true;
      map_info.route.sections[j].extend_sections_index.emplace_back(
          map_info.route.extend_sections_vec.size() - 1);
    }
  }

  // RoadBoundaryInfo
  // std::map<std::string, RoadBoundaryInfo> road_boundary_segments;
  const auto& road_boundary_segments_env = msg.road_boundaries();
  for (const auto& road_boundary_env : road_boundary_segments_env) {
    if (road_boundary_env.id() == 0 || road_boundary_env.points().empty()) {
      continue;
    }
    RoadBoundaryInfo road_boundary_segment_info_tmp;
    road_boundary_segment_info_tmp.id = road_boundary_env.id();
    road_boundary_segment_info_tmp.boundary_type =
        static_cast<BoundaryType>(road_boundary_env.boundary_type());
    // road_boundary_segment_info_tmp.width = road_boundary_env.width();
    for (const auto& pt : road_boundary_env.points()) {
      if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) ||
          std::isnan(pt.x()) || std::isnan(pt.y())) {
        continue;
      }
      Point2d new_point(pt.x(), pt.y());
      if (!road_boundary_segment_info_tmp.points.empty() &&
          new_point.DistanceTo(road_boundary_segment_info_tmp.points.back()) <
              FLAGS_ad_byd_planning_map_point_distance_threshold)
        continue;
#ifdef MAPTOPIC
      road_boundary_segment_info_tmp.points.emplace_back(
          TransPointVCSToDR(pt.x(), pt.y()));
#else
      road_boundary_segment_info_tmp.points.emplace_back(std::move(new_point));
#endif
    }
    map_info.all_road_boundaries_vec.push_back(
        std::move(road_boundary_segment_info_tmp));
  }
  // road_boundary_segments.clear();

  // LaneBoudnary
  auto& lane_boundaries = map_info.all_lane_boundaries_vec;
  const auto& boundary_segments_env = msg.lane_boundaries();
  for (const auto& boundary_env : boundary_segments_env) {
    if (boundary_env.id() == 0 || boundary_env.points().empty()) {
      continue;
    }
    LaneBoundaryInfo lane_boundary_info;
    lane_boundary_info.id = boundary_env.id();
    if (LineType temp_line_type =
            static_cast<LineType>(boundary_env.line_type());
        temp_line_type == LineType::FISH_SOLID) {
      lane_boundary_info.boundary_type.line_type = LineType::SOLID;
      Log2DDS::LogDataV2(
          "lane_boundary_type_convert",
          absl::StrCat(LineType::FISH_SOLID, "convert to:", LineType::SOLID));
    } else if (temp_line_type == LineType::FISH_DASH) {
      lane_boundary_info.boundary_type.line_type = LineType::DASHED;
      Log2DDS::LogDataV2(
          "lane_boundary_type_convert",
          absl::StrCat(LineType::FISH_DASH, "convert to:", LineType::DASHED));
    } else {
      lane_boundary_info.boundary_type.line_type = temp_line_type;
    }
    // lane_boundary_info.boundary_type.line_color =
    //     static_cast<LineColor>(boundary_env.line_color());
    lane_boundary_info.boundary_type.line_color = LineColor::COLOR_WHITE;
    for (const auto& pt : boundary_env.points()) {
      Point2d new_point(pt.x(), pt.y());
      if (!lane_boundary_info.points.empty() &&
          new_point.DistanceTo(lane_boundary_info.points.back()) <
              FLAGS_ad_byd_planning_map_point_distance_threshold)
        continue;
#ifdef MAPTOPIC
      lane_boundary_info.points.emplace_back(TransPointVCSToDR(pt.x(), pt.y()));
#else
      lane_boundary_info.points.emplace_back(std::move(new_point));
#endif
    }
    lane_boundaries.push_back(std::move(lane_boundary_info));
  }

  // Lane
  const auto& lanes_env = msg.lanes();
  auto& lanes = map_info.all_lanes_vec;
  for (const auto& lane_env : lanes_env) {
    if (lane_env.id() == 0) {
      continue;
    }
    LaneInfo lane_info_tmp;
    lane_info_tmp.id = lane_env.id();
    lane_info_tmp.section_id = lane_env.section_id();
    lane_info_tmp.junction_id = lane_env.junction_id();

    for (const auto& l_lane_boundary_id : lane_env.left_lane_boundary_ids()) {
      lane_info_tmp.left_lane_boundary_ids.emplace_back(l_lane_boundary_id);
    }
    for (const auto& r_lane_boundary_id : lane_env.right_lane_boundary_ids()) {
      lane_info_tmp.right_lane_boundary_ids.emplace_back(r_lane_boundary_id);
    }
    for (const auto& l_road_boundary_id : lane_env.left_road_boundary_ids()) {
      lane_info_tmp.left_road_boundary_ids.emplace_back(l_road_boundary_id);
    }
    for (const auto& r_road_boundary_id : lane_env.right_road_boundary_ids()) {
      lane_info_tmp.right_road_boundary_ids.emplace_back(r_road_boundary_id);
    }
    lane_info_tmp.left_lane_id = lane_env.left_lane_id();
    lane_info_tmp.right_lane_id = lane_env.right_lane_id();
    for (const auto& next_lane : lane_env.next_lane_ids()) {
      lane_info_tmp.next_lane_ids.emplace_back(next_lane);
    }
    // constexpr const double PointMseThresholdFITExtended = 0.25;
    // // double lastpoints_mse = 0.0;
    for (int i = 0; i < lane_env.points().size(); ++i) {
      auto& pt = lane_env.points()[i];
      if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) ||
          std::isnan(pt.x()) || std::isnan(pt.y())) {
        continue;
      }
      // if ((pt.point_source() == byd::msg::orin::routing_map::PS_BEV) &&
      //     !std::isnan(pt.mse()) && std::isfinite(pt.mse()))
      //   lastpoints_mse = pt.mse();
      // if (pt.point_source() ==
      //         byd::msg::orin::routing_map::SECTION_EXTENTED_FROM_FIT &&
      //     lastpoints_mse >= PointMseThresholdFITExtended)
      //   break;
      Point2d new_point(pt.x(), pt.y());
      if (!lane_info_tmp.points.empty() &&
          new_point.DistanceTo(lane_info_tmp.points.back()) <
              FLAGS_ad_byd_planning_map_point_distance_threshold)
        continue;
#ifdef MAPTOPIC
      lane_info_tmp.points.emplace_back(TransPointVCSToDR(pt.x(), pt.y()));
#else
      lane_info_tmp.points.emplace_back(std::move(new_point));
#endif
      // if (std::isnan(pt.mse()) || !std::isfinite(pt.mse()))
      //   lane_info_tmp.points_mse.emplace_back(lastpoints_mse);
      // else
      //   lane_info_tmp.points_mse.emplace_back(pt.mse());
    }
    if (lane_info_tmp.points.size() < 2) lane_info_tmp.points.clear();
    lane_info_tmp.type = static_cast<LaneType>(lane_env.type());
    if (lane_info_tmp.type == LaneType::LANE_VIRTUAL_COMMON ||
        lane_info_tmp.type == LaneType::LANE_VIRTUAL_JUNCTION) {
      lane_info_tmp.is_virtual = true;
    }
    lane_info_tmp.split_topology =
        static_cast<SplitTopology>(lane_env.split_topology());
    // if (lane_info_tmp.split_topology ==
    // SplitTopology::TOPOLOGY_SPLIT_UNKNOWN)
    //   lane_info_tmp.split_topology = SplitTopology::TOPOLOGY_SPLIT_NONE;
    lane_info_tmp.merge_topology =
        static_cast<MergeTopology>(lane_env.merge_topology());
    if (map_info.type == MapType::BEV_MAP) {
      lane_info_tmp.lane_merge_info.valid = lane_env.merge_info().merge_valid();
      lane_info_tmp.lane_merge_info.merge_source =
          static_cast<MergeSource>(lane_env.merge_info().merge_source());
      lane_info_tmp.lane_merge_info.dist_to_merge =
          lane_env.merge_info().dis_to_merge();
      // if (!lane_info_tmp.lane_merge_info.valid) {
      //   lane_info_tmp.merge_topology = MergeTopology::TOPOLOGY_MERGE_NONE;
      // }
    }
    // if (lane_info_tmp.merge_topology ==
    // MergeTopology::TOPOLOGY_MERGE_UNKNOWN)
    //   lane_info_tmp.merge_topology = MergeTopology::TOPOLOGY_MERGE_NONE;
    lane_info_tmp.length = lane_env.length();
    lane_info_tmp.speed_limit = lane_env.speed_limit();
    // lane_info_tmp.stop_line = lane_env.stop_line();
    lane_info_tmp.stop_line = !lane_env.traffic_stop_lines().empty();

    // lane_info_tmp.turn_type = static_cast<TurnType>(lane_env.turn_type());
    lane_info_tmp.turn_type = LaneTurnTypeAdaptor(lane_env.turn_type());

    lane_info_tmp.light_status =
        static_cast<LightStatus>(lane_env.light_status());
    lane_info_tmp.light_countdown = lane_env.light_countdown();
    lane_info_tmp.traffic_set_reason =
        static_cast<TrafficSetReason>(lane_env.traffic_set_reason());
    lane_info_tmp.pre_has_light = lane_env.light_info().prev_has_light_now_no();
    lane_info_tmp.none_odd_type =
        static_cast<NoneOddType>(lane_env.none_odd_type());
    // lane_info_tmp.lane_operation_type = lane_env.lane_operation_type();
    // lane_info_tmp.arrow_type = lane_env.arrow_type();
    lane_info_tmp.is_virtual_navigation = lane_env.is_virtual();

    // link object
    for (const auto& cross_walk : lane_env.cross_walks()) {
      lane_info_tmp.cross_walks.emplace_back(cross_walk);
    }
    // for (const auto& speed_bump : lane_env.speed_bumps()) {
    //   lane_info_tmp.speed_bumps.emplace_back(speed_bump);
    // }
    // for (const auto& parking_space : lane_env.parking_spaces()) {
    //   lane_info_tmp.parking_spaces.emplace_back(parking_space);
    // }
    // for (const auto& clear_area : lane_env.clear_areas()) {
    //   lane_info_tmp.clear_areas.emplace_back(clear_area);
    // }
    for (const auto& traffic_stop_line : lane_env.traffic_stop_lines()) {
      lane_info_tmp.traffic_stop_lines.emplace_back(traffic_stop_line);
    }

    lane_info_tmp.stopline_angle_flag = lane_env.stopline_angle_flag();
    lane_info_tmp.is_acc_adj_lane = lane_env.is_acc_adj_lane();
    lane_info_tmp.merge_start_dis = lane_env.merge_start_dis();
    lane_info_tmp.merge_end_dis = lane_env.merge_end_dis();
    lane_info_tmp.restricted_info.restricted_type = 
        static_cast<RestrictedType>(lane_env.restricted_info().restricted_type());
    lane_info_tmp.restricted_info.is_passable = lane_env.restricted_info().is_passable();
    lane_info_tmp.restricted_info.passable_env_state = lane_env.restricted_info().passable_env_state();
    lanes.push_back(std::move(lane_info_tmp));
  }

  // stop lines
  auto& stop_lines = map_info.all_stop_lines_vec;
  const auto& stop_lines_env = msg.stop_lines();
  for (const auto& stop_line_env : stop_lines_env) {
    if (stop_line_env.id() == 0 || stop_line_env.points().empty()) continue;
    StopLineInfo stop_line_info_tmp{};
    stop_line_info_tmp.id = stop_line_env.id();
    // stop_line_info_tmp.type =
    //     static_cast<StopLineType>(stop_line_env.stop_line_type());
    stop_line_info_tmp.type = StopLineType::STOPLINETYPE_UNKNOWN;
    stop_line_info_tmp.virtual_type = static_cast<int8_t>(stop_line_env.type());
    // stop_line_info_tmp.sub_type =
    //     static_cast<int8_t>(stop_line_env.stop_line_sub_type());
    // stop_line_info_tmp.light_type =
    //     static_cast<LightStatus>(stop_line_env.stop_line_type());

    for (const auto& pt : stop_line_env.points()) {
#ifdef MAPTOPIC
      stop_line_info_tmp.points.emplace_back(TransPointVCSToDR(pt.x(), pt.y()));
#else
      stop_line_info_tmp.points.emplace_back(pt.x(), pt.y());
#endif
    }
    stop_lines.push_back(std::move(stop_line_info_tmp));
  }
  // junction
  auto& junctions = map_info.all_junctions_vec;
  const auto& junctions_env = msg.junctions();
  for (const auto& junction_env : junctions_env) {
    if (junction_env.id() == 0 || junction_env.points().empty()) continue;
    JunctionInfo junction_info_tmp;
    junction_info_tmp.id = junction_env.id();

    for (const auto& pt : junction_env.points()) {
#ifdef MAPTOPIC
      junction_info_tmp.points.emplace_back(TransPointVCSToDR(pt.x(), pt.y()));
#else
      junction_info_tmp.points.emplace_back(pt.x(), pt.y());
#endif
    }
    junctions.push_back(std::move(junction_info_tmp));
  }
  // cross_walk
  auto& cross_walks = map_info.all_cross_walks_vec;
  const auto& cross_walks_env = msg.cross_walks();
  for (const auto& cross_walk_env : cross_walks_env) {
    if (cross_walk_env.id() == 0 || cross_walk_env.points().empty()) continue;
    CrossWalkInfo cross_walk_info_tmp;
    cross_walk_info_tmp.id = cross_walk_env.id();
    for (const auto& pt : cross_walk_env.points()) {
#ifdef MAPTOPIC
      cross_walk_info_tmp.points.emplace_back(
          TransPointVCSToDR(pt.x(), pt.y()));
#else
      cross_walk_info_tmp.points.emplace_back(pt.x(), pt.y());
#endif
    }
    cross_walks.push_back(std::move(cross_walk_info_tmp));
  }
  /*
      // speed_bump
      auto& speed_bumps = map_info.all_speed_bumps_vec;
      const auto& speed_bumps_env = msg.speed_bumps();
      for (const auto& speed_bump_env : speed_bumps_env) {
        SpeedBumpInfo speed_bump_info;
        speed_bump_info.id = speed_bump_env.id();
        for (const auto& pt : speed_bump_env.points()) {
          speed_bump_info.points.emplace_back(pt.x(), pt.y());
        }
        speed_bumps.emplace_back(std::move(speed_bump_info));
      }
      // parking space
      auto& parking_spaces = map_info.all_parking_spaces_vec;
      const auto& parking_spaces_env = msg.parking_spaces();
      for (const auto& parking_space_env : parking_spaces_env) {
        ParkingSpaceInfo parking_space_info;
        parking_space_info.id = parking_space_env.id();
        for (const auto& pt : parking_space_env.points()) {
          parking_space_info.points.emplace_back(pt.x(), pt.y());
        }
        parking_spaces.emplace_back(std::move(parking_space_info));
      }
      // clear area
      auto& clear_areas = map_info.all_clear_areas_vec;
      const auto& clear_areas_env = msg.clear_areas();
      for (const auto& clear_area_env : clear_areas_env) {
        ClearAreaInfo clear_area_info;
        clear_area_info.id = clear_area_env.id();
        clear_area_info.type =
            static_cast<ImpassableAeraType>(clear_area_env.type());
        for (const auto& pt : clear_area_env.points()) {
          clear_area_info.points.emplace_back(pt.x(), pt.y());
        }
        clear_areas.emplace_back(std::move(clear_area_info));
      }
  */

  // navi start
  auto& navi_start = map_info.route.navi_start;
  const auto& navi_start_temp = msg.route().navi_start();
  if (navi_start_temp.section_id() != 0) {
    navi_start.section_id = navi_start_temp.section_id();
    navi_start.s_offset = navi_start_temp.s_offset();
  }
  // navi end
  auto& navi_end = map_info.route.navi_end;
  // const auto& navi_end_temp = msg.route().navi_end();
  if (!msg.route().sections().empty()) {
    navi_end.section_id = msg.route().sections().rbegin()->id();
    // navi_end.s_offset = navi_end_temp.s_offset();
  }

  // update exp_trajectories input
  const auto& exp_trajectories_env = msg.exp_trajectories();
  auto& exp_trajectories_vec = map_info.all_exp_trajectories_vec;
  for (const auto& exp_trajectory_env : exp_trajectories_env) {
    ExpTrajectoryInfo exp_trajectory;
    exp_trajectory.id = exp_trajectory_env.id();
    exp_trajectory.lane_id = exp_trajectory_env.lane_id();
    exp_trajectory.start_lane_id = exp_trajectory_env.start_lane_id();
    exp_trajectory.end_lane_id = exp_trajectory_env.end_lane_id();
    for (const auto& relative_lane_id : exp_trajectory_env.relative_lane_id()) {
      exp_trajectory.relative_lane_id.emplace_back(relative_lane_id);
    }

    for (int i = 0; i < exp_trajectory_env.points().size(); ++i) {
      const auto& pt = exp_trajectory_env.points()[i];
      Point2d tmp_point(pt.x(), pt.y());
      exp_trajectory.points.emplace_back(std::move(tmp_point));
    }

    exp_trajectories_vec.emplace_back(exp_trajectory);
  }

  return map_info;
}

/*
EgoInfo MsgManager::CreateEgoInfo(const odometry_type& odom,
                                  const vehicle_status_type& vehicle_status) {
  EgoInfo ego;
  ego.pos.set_x(odom.position().x());
  ego.pos.set_y(odom.position().y());
  ego.vel.set_x(odom.velocity().x());
  ego.vel.set_y(odom.velocity().y());
  ego.acc.set_x(odom.ego_acceleration().x());
  ego.acc.set_y(odom.ego_acceleration().y());

  double qx = odom.quaternion().x();
  double qy = odom.quaternion().y();
  double qz = odom.quaternion().z();
  double qw = odom.quaternion().w();

  ego.heading = math::QuaternionToYaw(qw, qx, qy, qz);
  ego.length = 5.222;
  ego.width = 2.046;
  ego.height = 1.5;
  ego.steering_angle =
      vehicle_status.vehicle_control_status().lat_control().steering_angle();
  ego.stamp = odom.header().stamp() / 1000.0;
  return ego;
}

NavigationInfo MsgManager::CreateNavigationInfo(
    const road_horizon_type& road_horizon) {
  NavigationInfo navigation;
  navigation.seq_num = road_horizon.header().seq();
  for (const auto& turn : road_horizon.current_road().junctions()) {
    navigation.has_navigation = true;
    TurnInfo turn_info;
    turn_info.start = turn.start();
    turn_info.angle = turn.angle_of_ego_turn();
    navigation.turn_infos.emplace_back(turn_info);
  }
  for (const auto& form_of_way :
       road_horizon.current_road().road_profile().form_of_ways()) {
    navigation.form_of_way_infos.emplace_back(
        form_of_way.start(), form_of_way.end(),
        static_cast<ad_byd::planning::RoadType>(form_of_way.form_of_way()));
  }
  for (auto& road_class :
       road_horizon.current_road().road_profile().road_classes()) {
    RoadClassInfo road_class_info;
    road_class_info.start = road_class.start();
    road_class_info.end = road_class.end();
    road_class_info.class_type =
        static_cast<ad_byd::planning::RoadClass>(road_class.road_class());
    navigation.road_class_infos.emplace_back(road_class_info);
  }
  for (auto& lane_group : road_horizon.current_road().lane_groups()) {
    LaneGroupInfo lane_group_info;
    lane_group_info.start = lane_group.start();
    lane_group_info.end = lane_group.end();
    lane_group_info.num_of_lanes = lane_group.num_of_lanes();
    for (auto& lane_id : lane_group.lane_ids()) {
      lane_group_info.lane_ids.emplace_back(lane_id);
    }
    navigation.lane_group_infos.emplace_back(lane_group_info);
  }
  return navigation;
}

ObstacleInfo MsgManager::CreateObstacleInfo(const obstacle_type& m) {
  ObstacleInfo obs;
  obs.seq_num = m.header().seq();
  obs.coordinate_type = m.coordinate_type();
  obs.pose_odom = m.pose_odom();
  for (auto& o : m.obstacles()) {
    if (IsStatic(o.type())) {
      StaticObstacleInfo info;
      info.stamp = o.timestamp();
      info.id = o.id();
      info.seq_num = m.header().seq();
      info.pos.set_x(o.position().x());
      info.pos.set_y(o.position().y());
      info.vel.set_x(o.velocity().x());
      info.vel.set_y(o.velocity().y());
      info.acc.set_x(o.acceleration().x());
      info.acc.set_y(o.acceleration().y());
      info.heading = o.heading();
      info.length = o.length();
      info.width = o.width();
      info.type = static_cast<ObstacleType>(o.type());
      info.fusion_type = o.fusion_type();
      info.obstacle_state = o.obstacle_state();
      for (auto& p : o.corner_points()) {
        info.polygon.emplace_back(p.x(), p.y());
      }
      obs.static_obs.push_back(info);
    } else {
      DynamicObstacleInfo info;
      info.seq_num = m.header().seq();
      info.id = o.id();
      info.type = static_cast<ObstacleType>(o.type());
      info.stamp = o.timestamp();
      info.length = o.length();
      info.width = o.width();
      info.height = o.height();
      info.heading = o.heading();
      info.yaw_rate = o.yaw_rate();
      info.life_time = o.life_time();
      info.confidence = o.confidence();
      info.pos.set_x(o.position().x());
      info.pos.set_y(o.position().y());
      info.vel.set_x(o.velocity().x());
      info.vel.set_y(o.velocity().y());
      info.acc.set_x(o.acceleration().x());
      info.acc.set_y(o.acceleration().y());
      for (auto& p : o.corner_points()) {
        info.polygon.emplace_back(p.x(), p.y());
      }
      info.obstacle_state = o.obstacle_state();
      info.fusion_type = o.fusion_type();
      const auto& obstacle_light = o.obstacle_light();
      info.obstacle_lights.emplace_back(
          static_cast<ObstacleLightType>(obstacle_light.left_turn_lights()));
      info.obstacle_lights.emplace_back(
          static_cast<ObstacleLightType>(obstacle_light.right_turn_lights()));
      info.obstacle_lights.emplace_back(
          static_cast<ObstacleLightType>(obstacle_light.brake_lights()));
      info.obstacle_lights.emplace_back(
          static_cast<ObstacleLightType>(obstacle_light.hazard_lights()));
      info.obstacle_lights_conf.emplace_back(
          obstacle_light.left_turn_lights_conf());
      info.obstacle_lights_conf.emplace_back(
          obstacle_light.right_turn_lights_conf());
      info.obstacle_lights_conf.emplace_back(
          obstacle_light.brake_lights_conf());
      info.obstacle_lights_conf.emplace_back(
          obstacle_light.hazard_lights_conf());
      obs.dynamic_obs.push_back(info);
    }
  }

  return obs;
}

absl::StatusOr<PredictionInputFrame::Ptr>
MsgManager::GetPredictionInputFrame() {
  RETURN_IF_NOT_OK(auto obs_msgs, obstacle_sub_.Dump(kPredictionCounter));

  if (obs_msgs.empty()) {
    return absl::UnavailableError("no fusion msg");
  }

  auto res = std::make_shared<PredictionInputFrame>();
  res->start_time = byd::utils::time::NowMilliseconds() / 1000.0;

  for (auto& [stamp_s, obs_msg] : obs_msgs) {
    RETURN_IF_NOT_OK(auto odom_msg, odometry_sub_.GetMsgAt(stamp_s));
    RETURN_IF_NOT_OK(auto vehicle_status,
                     vehicle_status_sub_.GetMsgAt(stamp_s));

    auto ego_info = CreateEgoInfo(*odom_msg.second, *vehicle_status.second);
    ego_info.seq_num = obs_msg->header().seq();
    res->ego_infos.push_back(ego_info);
    res->obstacle_infos.push_back(CreateObstacleInfo(*obs_msg));
  }
  res->head_ts = obs_msgs.back().first;

  RETURN_IF_NOT_OK(auto latest_odom, odometry_sub_.Back(kPredictionCounter));
  RETURN_IF_NOT_OK(auto latest_vehicle_status,
                   vehicle_status_sub_.Back(kPredictionCounter));

  res->latest_ego_info =
      CreateEgoInfo(*latest_odom.second, *latest_vehicle_status.second);
  res->vehicle_seq = latest_vehicle_status.second->header().seq();
  res->odom_seq = latest_odom.second->header().frame_id();

  res->highway_prediction = nullptr;
  if (auto latest = highway_prediction_sub_.Get(kPredictionCounter);
      latest.ok()) {
    res->highway_prediction = latest->second;
  }
  res->behavior = nullptr;
  if (auto latest = behavior_sub_.Get(kPredictionCounter); latest.ok()) {
    res->behavior = latest->second;
  }
  // res->env_map = nullptr;
  // if (auto latest = env_map_sub_.Get(); latest.ok()) {
  //   res->env_map = latest->second;
  // }

  if (auto ret = map_container_.GetMap(&res->map_element);
      ErrorCode::PLANNING_OK != ret) {
    return absl::UnavailableError(absl::StrCat("map unavailable: ",
                                  static_cast<int>(ret)));
  }

#ifndef OFFLINE
  RETURN_IF_NOT_OK(auto ehorizon_msgs,
                   road_horizon_sub_.Get(kPredictionCounter));
  auto navigation_info = CreateNavigationInfo(*ehorizon_msgs.second);
  res->navigation_infos = navigation_info;
#endif

  byd::log::Warn("BYD_PREDICTION",
                  "%s:%d: BYD-Process sequence num %" PRIu64
                  " process map seq num %" PRIu64 "",
                  __FILE__, __LINE__, obs_msgs.back().second->header().seq(),
                  res->map_element.map_ptr->seq());
  byd::log::Warn(
      "BYD_PREDICTION",
      "%s:%d: BYD-Obstacle total num [%" PRIu64 "] in this frame lru cache 100",
      __FILE__, __LINE__, obs_msgs.back().second->obstacles().size());

  return res;
}
*/

std::pair<st::planning::PlannerStatusProto::PlannerStatusCode,
          PlanningInputFrame::Ptr>
MsgManager::GetPlanningInputFrame() {
  TIMELINE("MsgManager::GetPlanningInputFrame");
  PullOdoMsg();
  st::planning::PlannerStatusProto::PlannerStatusCode result =
      st::planning::PlannerStatusProto::OK;
  auto res = std::make_shared<PlanningInputFrame>();
  double prediction_stamp{0.0};

  if (auto latest = odometry_sub_.Back(kPlanningCounter); !latest.ok()) {
    LOG_ERROR << "planning input odometry: " << latest.status().ToString();
    Log2DDS::LogDataV2("odometry_seq", "ODOMETRY_TIMEOUT");
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_MSG_ODOMETRY_TIMEOUT
                  : result);
  } else {
    res->odometry = latest->second;
    Log2DDS::LogDataV2("odometry_seq",
                       absl::StrCat(res->odometry->header().sequence_num()));
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
    res->last_odometry_timestamp = latest->first;

    double yaw = res->odometry->pose().heading() * M_PI / 180.0;
    veh_pos_ = {res->odometry->pose().position().x(),
                res->odometry->pose().position().y(), yaw};
  }

  if (auto latest = vehicle_status_sub_.Back(kPlanningCounter); !latest.ok()) {
    LOG_ERROR << "planning input vehicle_status: "
              << latest.status().ToString();
    Log2DDS::LogDataV2("vehicle_status_seq", "VEHICLE_TIMEOUT");
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_MSG_VEHICLE_TIMEOUT
                  : result);
  } else {
    res->vehicle_status = latest->second;
    Log2DDS::LogDataV2(
        "vehicle_status_seq",
        absl::StrCat(res->vehicle_status->header().sequence_num()));
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);

    // res->estimate_a = latest->second->ego_motion_status().da_in_lgtacc_sg();
    const double delta_time = 0.3;
    auto prev = vehicle_status_sub_.GetMsgAt(latest->first - delta_time);
    // if (prev.ok()) {
    //   const auto dt = latest->first - prev->first;
    //   if (dt > 0.5 * delta_time) {
    //     const double dv =
    //         latest->second->ego_motion_status().da_in_vehspd_sg() -
    //         prev->second->ego_motion_status().da_in_vehspd_sg();
    //     res->estimate_a = dv / dt;
    //   }
    // }
  }

  if (auto latest = vmc_debug_sub_.Back(kPlanningCounter); !latest.ok()) {
    LOG_ERROR << "planning input VMC: " << latest.status().ToString();
    Log2DDS::LogDataV2("vmc_debug_seq", "VMC_TIMEOUT");
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_MSG_VMC_TIMEOUT
                  : result);
  } else {
    res->vmc_debug = latest->second;
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
    res->estimate_a = latest->second->debug_cddaest();
    Log2DDS::LogDataV2("vmc_debug_seq",
                       absl::StrCat(res->vmc_debug->header().sequence_num()));
  }

  if (auto latest = env_map_sub_.Get(kPlanningCounter); !latest.ok()) {
    LOG_ERROR << "planning input env_map: " << latest.status().ToString();
    Log2DDS::LogDataV2("env_map_seq", "MAP_TIMEOUT");
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_MSG_MAP_TIMEOUT
                  : result);
  } else {
    res->env_map = latest->second;
    Log2DDS::LogDataV2("env_map_seq",
                       absl::StrCat(res->env_map->header().sequence_num()));
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
  }

  if (auto latest = map_event_sub_.Get(kPlanningCounter); !latest.ok()) {
    Log2DDS::LogDataV2("map_event_seq", "MAP_EVENT_TIMEOUT");
  } else {
    res->map_event = latest->second;
    Log2DDS::LogDataV2("map_event_seq",
                       absl::StrCat(res->map_event->header().sequence_num()));
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
  }

#if defined(BYD_X2B) || defined(BYD_VCPB)
  if (auto latest = top_state_sub_.Get(kPlanningCounter); !latest.ok()) {
    Log2DDS::LogDataV2("top_state_seq", "TOP_STATE_TIMEOUT");
  } else {
    res->top_state = latest->second;
    Log2DDS::LogDataV2("top_state_seq",
                       absl::StrCat(res->top_state->header().sequence_num()));
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
  }
#endif

  if (auto latest = prediction_sub_.Get(kPlanningCounter); !latest.ok()) {
    LOG_ERROR << "planning input prediction " << latest.status().ToString();
    Log2DDS::LogDataV2("prediction_obstacles_seq", "OBSTACLE_TIMEOUT");
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_MSG_OBSTACLE_TIMEOUT
                  : result);
  } else {
    LOG_ERROR << "planning input prediction " << latest.status().ToString();
    res->prediction = latest->second;
    Log2DDS::LogDataV2("prediction_obstacles_seq",
                       absl::StrCat(res->prediction->header().sequence_num()));
    // res->last_msg_timestamp = std::max(res->last_msg_timestamp,
    // latest->first);

    prediction_stamp = latest->second->objects().empty()
                           ? latest->first
                           : latest->second->header().measurement_timestamp();
  }

  if (auto latest = behavior_sub_.Get(kPlanningCounter); !latest.ok()) {
    LOG_ERROR << "planning input behavior:" << latest.status().ToString();
    Log2DDS::LogDataV2("behavior_seq", "BEHAVIOR_TIMEOUT");
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_MSG_BEHAVIOR_TIMEOUT
                  : result);
  } else {
    res->behavior = latest->second;
    Log2DDS::LogDataV2("behavior_seq",
                       absl::StrCat(res->behavior->header().sequence_num()));
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
    if (is_in_highway_scene_ &&
        res->behavior->function_id() ==
            byd::msg::planning::FunctionId::FUNCTION_CITY_NOA) {
      res->behavior->set_function_id(
          byd::msg::planning::FunctionId::FUNCTION_HW_NOA);
    }
  }
  /*
    if (auto latest = scenario_intention_sub_.Get(kPlanningCounter);
        !latest.ok()) {
      // optional
      // return latest.status();
    } else {
      res->scenario_intention = latest->second;
      res->last_msg_timestamp = std::max(res->last_msg_timestamp,
    latest->first);
    }
  */
  double odom_delay = res->last_msg_timestamp - res->last_odometry_timestamp;
  if (odom_delay > 0.5) {
    LOG_ERROR << std::fixed << std::setprecision(4)
              << "planning input odometry delay: last_msg_timestamp = "
              << res->last_msg_timestamp
              << ", last_odometry_timestamp = " << res->last_odometry_timestamp
              << ", delay " << odom_delay;
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_MSG_ODOMETRY_DELAY
                  : result);
  }

  double prediction_delay = res->last_msg_timestamp - prediction_stamp;
  if (prediction_delay > 1.0) {
    LOG_ERROR << std::fixed << std::setprecision(4)
              << "planning input prediction delay: last_msg_timestamp = "
              << res->last_msg_timestamp
              << ", prediction_stamp = " << prediction_stamp << ", delay "
              << prediction_delay;
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_MSG_OBSTACLE_DELAY
                  : result);
  }

  auto behavior = behavior_container_.Get();
  res->use_hd_map =
      behavior.function_id == byd::msg::planning::FunctionId::FUNCTION_HW_NOA ||
      behavior.function_id ==
          byd::msg::planning::FunctionId::FUNCTION_CITY_NOA ||
      behavior.function_id == byd::msg::planning::FunctionId::FUNCTION_LKA_PLUS;

#ifdef DEBUG_X86_ENABLED
  // sleep for map_ptr construction
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif
  MapElement map_element;
  if (auto ret = map_container_.GetMap(&map_element);
      ErrorCode::PLANNING_OK != ret) {
    result = (result == st::planning::PlannerStatusProto::OK
                  ? st::planning::PlannerStatusProto::PLAN_FAIL_NO_MAP
                  : result);
  }
  res->map_ptr = map_element.map_ptr;
  if (res->map_ptr != nullptr) {
    Log2DDS::LogDataV2("map_ptr_seq", absl::StrCat(res->map_ptr->seq()));
  } else {
    Log2DDS::LogDataV2("map_ptr_seq", "MAP_PTR_NULL");
  }
  if (auto latest = noa_map_sub_.Get(kPlanningCounter); !latest.ok()) {
    res->lite_map_ptr = nullptr;
    Log2DDS::LogDataV2("lite_map_seq", "LITE_MAP_TIMEOUT");
  } else {
    map_container_.GetLdLiteMap(res->lite_map_ptr);
    Log2DDS::LogDataV2("lite_map_seq",
                       absl::StrCat(latest->second->header().sequence_num()));
    if (res->lite_map_ptr) {
      Log2DDS::LogDataV2("ld_lite_debug",
                         absl::StrCat("is_lite_map_valid: ",
                                      res->lite_map_ptr->is_lite_map_valid()));
      Log2DDS::LogDataV2("lite_map_ptr_seq",
                         absl::StrCat(res->lite_map_ptr->seq()));
      if (!res->lite_map_ptr->is_lite_map_valid()) res->lite_map_ptr = nullptr;
    } else {
      Log2DDS::LogDataV2("lite_map_ptr_seq", "LITE_MAP_PTR_NULL");
    }
  }

  return std::pair<st::planning::PlannerStatusProto::PlannerStatusCode,
                   PlanningInputFrame::Ptr>(result, res);
}

}  // namespace planning
}  // namespace ad_byd
