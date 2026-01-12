/**
 * @file speed_gaming_test.cc
 * @author su.lei (su.lei12@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-04-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <fcntl.h>
#include <gtest/gtest.h>
#include <json/json.h>
#include "google/protobuf/text_format.h"

#include <fstream>
#include <iostream>
#include <string>

#include "../simulator/alternative_idm_simulator.h"
#include "../simulator/multi_agent_idm.h"
#include "../simulator/speed_idm_common.h"
#include "../speed_gaming_decider.h"
#include "test_util.h"
static Json::Value g_root;
namespace st::planning {
void SaveUnitTestData(const std::string &id, const GamingSimResult &SimResult,
                      const SpeedVector &ego_speed_data) {
  SpeedVector obj_speed_data = SimResult.obj_speed_data;
  if (ego_speed_data.empty() || obj_speed_data.empty()) {
    return;
  }
  std::string simulation_type;
  if (SimResult.ego_lon_type == LonGamingDecisionType::kYield) {
    simulation_type.append("+Yield");
  } else if (SimResult.ego_lon_type == LonGamingDecisionType::kOvertake) {
    simulation_type.append("Overtake");
  }
  Json::Value data;
  Json::Value ego;
  g_root.clear();
  g_root["type"] = simulation_type;
  g_root["id"] = id;
  for (auto &pt : ego_speed_data) {
    Json::Value node;
    node["t"] = pt.t();
    node["s"] = pt.s();
    node["v"] = pt.v();
    node["a"] = pt.a();
    node["j"] = pt.j();
    ego.append(node);
  }
  g_root["ego_speed_path"] = ego;

  Json::Value obj;
  for (auto &pt : SimResult.obj_speed_data) {
    Json::Value node;
    node["t"] = pt.t();
    node["s"] = pt.s();
    node["v"] = pt.v();
    node["a"] = pt.a();
    node["j"] = pt.j();
    obj.append(node);
  }
  g_root["obj_speed_path"] = obj;
  GamingConflictZone riskfield_conflict_zone_in_ego_view =
      SimResult.sim_conflict_zone_in_ego_view;
  Json::Value conflict_zone_in_ego_view;
  conflict_zone_in_ego_view["ego_cutin_s"] =
      riskfield_conflict_zone_in_ego_view.ego_cutin_s;
  conflict_zone_in_ego_view["ego_cutin_t"] =
      riskfield_conflict_zone_in_ego_view.ego_cutin_time;
  conflict_zone_in_ego_view["ego_cutout_s"] =
      riskfield_conflict_zone_in_ego_view.ego_cutout_s;
  conflict_zone_in_ego_view["ego_cutout_t"] =
      riskfield_conflict_zone_in_ego_view.ego_cutout_time;
  conflict_zone_in_ego_view["ego_cross_angle"] =
      riskfield_conflict_zone_in_ego_view.ego_cross_angle;
  conflict_zone_in_ego_view["agent_cutin_s"] =
      riskfield_conflict_zone_in_ego_view.agent_cutin_s;
  conflict_zone_in_ego_view["agent_cutin_t"] =
      riskfield_conflict_zone_in_ego_view.agent_cutin_time;
  conflict_zone_in_ego_view["agent_cutout_s"] =
      riskfield_conflict_zone_in_ego_view.agent_cutout_s;
  conflict_zone_in_ego_view["agent_cutout_t"] =
      riskfield_conflict_zone_in_ego_view.agent_cutout_time;
  conflict_zone_in_ego_view["agent_cross_angle"] =
      riskfield_conflict_zone_in_ego_view.agent_cross_angle;
  conflict_zone_in_ego_view["risk_field_lat_ratio"] =
      riskfield_conflict_zone_in_ego_view.risk_field_lat_ratio;
  conflict_zone_in_ego_view["is_path_conflict"] =
      riskfield_conflict_zone_in_ego_view.is_path_conflict;
  conflict_zone_in_ego_view["is_risk_field_conflict"] =
      riskfield_conflict_zone_in_ego_view.is_risk_field_conflict;
  g_root["conflict_zone_in_ego_view"] = conflict_zone_in_ego_view;

  Json::StyledWriter sw;
  std::ofstream os;
  std::string path =
      "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
      "test/MultiAgent_UnitTestResult_id" +
      id + "_" + simulation_type + ".json";

  os.open(path, std::ios::out);
  if (!os.is_open()) {
    std::cout << "error:can not find or create the file";
    return;
  }
  os << sw.write(g_root);
  os.close();
  return;
}

TEST(MultiAgentUnitTest, TestProcess)

{
  DiscretizedPath ego_path;
  std::vector<SpacetimeTrajectoryManager> speed_traj_manager;
  std::unordered_map<std::string, GamingConflictZoneInfo> conflict_zone_infos_;
  std::vector<AidmScene> aidm_scenes;
  SpeedVector upper_speed_data;
  SpeedVector normal_yield_speed_data;
  AidmScene aidm_scene;

  // 定义文件路径
  std::string file_path =
      "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
      "test/obs_preprocess_debug.json";

  // 打开文件并读取内容到字符串
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "无法打开文件: " << file_path << std::endl;
    return;  // 提前返回，避免后续操作
  }

  // 读取文件内容到字符串
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string content = buffer.str();

  // 解析JSON内容
  Json::Value debug_config;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream s(content);
  if (!Json::parseFromStream(reader, s, &debug_config, &errs)) {
    std::cerr << "JSON解析错误1: " << errs << std::endl;
    return;  // 解析失败，提前返回
  }

  // 访问特定数据
  if (debug_config.isMember("conflict_zones") &&
      !debug_config["conflict_zones"].isNull()) {
    const Json::Value &conflict_zones_json = debug_config["conflict_zones"];
    for (const auto &zone_json : conflict_zones_json) {
      GamingConflictZoneInfo conflict_zone_info;
      GamingConflictZone &agent_view =
          conflict_zone_info.conflict_zone_in_agent_view;
      if (zone_json.isMember("agent_view") &&
          zone_json["agent_view"].isObject()) {
        const Json::Value &agent_view_json = zone_json["agent_view"];
        agent_view.agent_cutin_s = agent_view_json["agent_cutin_s"].asDouble();
        agent_view.agent_cutin_time =
            agent_view_json["agent_cutin_time"].asDouble();
        agent_view.agent_cutout_s =
            agent_view_json["agent_cutout_s"].asDouble();
        agent_view.agent_cutout_time =
            agent_view_json["agent_cutout_time"].asDouble();
        agent_view.ego_cutin_s = agent_view_json["ego_cutin_s"].asDouble();
        agent_view.ego_cutout_s = agent_view_json["ego_cutout_s"].asDouble();
        agent_view.is_path_conflict =
            agent_view_json["is_path_conflict"].asBool();
        agent_view.is_risk_field_conflict =
            agent_view_json["is_risk_field_conflict"].asBool();
        agent_view.risk_field_lat_ratio =
            agent_view_json["risk_field_lat_ratio"].asDouble();
      }

      GamingConflictZone &ego_view =
          conflict_zone_info.conflict_zone_in_ego_view;
      if (zone_json.isMember("ego_view") && zone_json["ego_view"].isObject()) {
        const Json::Value &ego_view_json = zone_json["ego_view"];
        ego_view.agent_cutin_s = ego_view_json["agent_cutin_s"].asDouble();
        ego_view.agent_cutin_time =
            ego_view_json["agent_cutin_time"].asDouble();
        ego_view.agent_cutout_s = ego_view_json["agent_cutout_s"].asDouble();
        ego_view.agent_cutout_time =
            ego_view_json["agent_cutout_time"].asDouble();
        ego_view.ego_cutin_s = ego_view_json["ego_cutin_s"].asDouble();
        ego_view.ego_cutout_s = ego_view_json["ego_cutout_s"].asDouble();
        ego_view.is_path_conflict = ego_view_json["is_path_conflict"].asBool();
        ego_view.is_risk_field_conflict =
            ego_view_json["is_risk_field_conflict"].asBool();
        ego_view.risk_field_lat_ratio =
            ego_view_json["risk_field_lat_ratio"].asDouble();
      }

      conflict_zone_info.obj_traj_id = zone_json["id"].asString();
      conflict_zone_info.is_pre_gaming = zone_json["is_pre_gaming"].asBool();

      conflict_zone_infos_.emplace(conflict_zone_info.obj_traj_id,
                                   conflict_zone_info);

      int interaction_type = zone_json["type"].asInt();
      if (interaction_type == 1) {
        aidm_scene = AidmScene::kCross;
      } else if (interaction_type == 2 || interaction_type == 3) {
        aidm_scene = AidmScene::kMergeIn;
      }
      aidm_scenes.push_back(aidm_scene);
    }
  } else {
    std::cerr << "JSON中缺少conflict_zones数组或格式不正确" << std::endl;
  }

  if (debug_config.isMember("ego_path") && !debug_config["ego_path"].isNull()) {
    const Json::Value &ego_path_json = debug_config["ego_path"];

    std::vector<PathPoint> path_points;
    for (const auto &point_json : ego_path_json) {
      // std::cout << "自车路径点"
      //           << "; x=" << point_json["x"].asDouble()
      //           << ", y=" << point_json["y"].asDouble()
      //           << ", yaw=" << point_json["yaw"].asDouble() << std::endl;

      PathPoint point;
      point.set_x(point_json["x"].asDouble());
      point.set_y(point_json["y"].asDouble());
      point.set_theta(point_json["yaw"].asDouble());
      path_points.push_back(point);
    }
    ego_path = DiscretizedPath(std::move(path_points));
  } else {
    std::cerr << "JSON中缺少ego_path数组或格式不正确" << std::endl;
  }

  // construct spacetime object manager.
  ObjectPredictionProto obj_pre_proto1;
  ObjectPredictionProto obj_pre_proto2;
  if (!TextFileToProto(
          "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
          "test/cross_object_prediction.proto.txt",
          &obj_pre_proto1) ||
      !TextFileToProto(
          "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
          "test/merge_object_prediction.proto.txt",
          &obj_pre_proto2)) {
    std::cerr << "解析失败" << std::endl;
  }
  prediction::ObjectPrediction object_prediction1(obj_pre_proto1);
  prediction::ObjectPrediction object_prediction2(obj_pre_proto2);
  PlannerObject objects[2] = {PlannerObject(object_prediction1),
                              PlannerObject(object_prediction2)};
  SpacetimeTrajectoryManager traj_mgr(absl::MakeSpan(objects, 2));

  // 配置其他参数
  // construct VehicleGeometryParams
  VehicleGeometryParamsProto vehicle_geo_params;
  vehicle_geo_params.set_front_edge_to_center(4.0);
  vehicle_geo_params.set_back_edge_to_center(1.0);
  vehicle_geo_params.set_left_edge_to_center(1.0);
  vehicle_geo_params.set_right_edge_to_center(1.0);
  vehicle_geo_params.set_length(5.0);
  vehicle_geo_params.set_width(2.0);
  vehicle_geo_params.set_height(1.7);
  vehicle_geo_params.set_wheel_base(3.0);

  // 设置ego_init_point
  TrajectoryPoint ego_init_point;
  ego_init_point.set_pos(Vec2d(ego_path.front().x(), ego_path.front().y()));
  ego_init_point.set_theta(ego_path.front().theta());
  ego_init_point.set_s(ego_path.front().s());
  ego_init_point.set_v(5.0);

  std::vector<double> s_list{0.0, 50.0, 100.0};
  std::vector<double> v_list{20.0, 20.0, 20.0};
  const PiecewiseLinearFunction<double> speed_limit_plf(s_list, v_list);

  // get yield_sim_results
  std::unordered_map<std::string, GamingSimResult> ego_yield_sim_results;
  GamingSimResult ego_yield_sim_result;
  std::string yield_file_path =
      "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
      "test/UnitTestResult_id2_EgoYield+Merge.json";

  // 打开文件并读取内容到字符串
  std::ifstream yield_file(yield_file_path);
  if (!yield_file.is_open()) {
    std::cerr << "无法打开文件: " << yield_file_path << std::endl;
    return;  // 提前返回，避免后续操作
  }

  // 读取文件内容到字符串
  std::stringstream yield_buffer;
  yield_buffer << yield_file.rdbuf();
  std::string yield_content = yield_buffer.str();

  // 解析JSON内容
  Json::Value yield_debug_config;
  Json::CharReaderBuilder yield_reader;
  std::string yield_errs;
  std::istringstream yield_s(yield_content);
  if (!Json::parseFromStream(yield_reader, yield_s, &yield_debug_config,
                             &yield_errs)) {
    std::cerr << "JSON解析错误2: " << yield_errs << std::endl;
    return;  // 解析失败，提前返回
  }
  std::string obj_traj_id1;
  if (yield_debug_config.isMember("iter_time_10") &&
      !yield_debug_config["iter_time_10"].isNull()) {
    const Json::Value &conflict_zones_json = yield_debug_config["iter_time_10"];
    GamingConflictZoneInfo conflict_zone_info;
    GamingConflictZone &conflict_zone_in_ego_view =
        conflict_zone_info.conflict_zone_in_ego_view;
    if (conflict_zones_json.isMember("conflict_zone_in_ego_view") &&
        conflict_zones_json["conflict_zone_in_ego_view"].isObject()) {
      const Json::Value &agent_view_json =
          conflict_zones_json["conflict_zone_in_ego_view"];
      conflict_zone_in_ego_view.agent_cutin_s =
          agent_view_json["agent_cutin_s"].asDouble();
      conflict_zone_in_ego_view.agent_cutin_time =
          agent_view_json["agent_cutin_t"].asDouble();
      conflict_zone_in_ego_view.agent_cutout_s =
          agent_view_json["agent_cutout_s"].asDouble();
      conflict_zone_in_ego_view.agent_cutout_time =
          agent_view_json["agent_cutout_t"].asDouble();
      conflict_zone_in_ego_view.ego_cutin_s =
          agent_view_json["ego_cutin_s"].asDouble();
      conflict_zone_in_ego_view.ego_cutin_time =
          agent_view_json["ego_cutin_t"].asDouble();
      conflict_zone_in_ego_view.ego_cutout_s =
          agent_view_json["ego_cutout_s"].asDouble();
      conflict_zone_in_ego_view.ego_cutout_time =
          agent_view_json["ego_cutout_t"].asDouble();
      conflict_zone_in_ego_view.is_path_conflict =
          agent_view_json["is_path_conflict"].asBool();
      conflict_zone_in_ego_view.is_risk_field_conflict =
          agent_view_json["is_risk_field_conflict"].asBool();
      conflict_zone_in_ego_view.risk_field_lat_ratio =
          agent_view_json["risk_field_lat_ratio"].asDouble();
    } else {
      std::cerr << "JSON中缺少conflict_zone_in_ego_view的结果" << std::endl;
    }
    obj_traj_id1 = yield_debug_config["id"].asString();
    ego_yield_sim_result.sim_conflict_zone_in_ego_view =
        conflict_zone_in_ego_view;
    int interaction_type = yield_debug_config["type"].asInt();
    if (interaction_type == 1) {
      ego_yield_sim_result.interaction_type = InteractionType::kCross;
    } else if (interaction_type == 2) {
      ego_yield_sim_result.interaction_type = InteractionType::kStraightMerge;
    } else if (interaction_type == 3) {
      ego_yield_sim_result.interaction_type = InteractionType::kTurnMerge;
    }
    int ego_lon_type = yield_debug_config["lontype"].asInt();
    if (ego_lon_type == 1) {
      ego_yield_sim_result.ego_lon_type = LonGamingDecisionType::kYield;
    } else if (ego_lon_type == 2) {
      ego_yield_sim_result.ego_lon_type = LonGamingDecisionType::kOvertake;
    } else {
      ego_yield_sim_result.ego_lon_type = LonGamingDecisionType::kNone;
    }
    if (conflict_zones_json.isMember("ego_speed_path")) {
      SpeedVector ego_path_points;
      const Json::Value &ego_path_json = conflict_zones_json["ego_speed_path"];
      for (const auto &point_json : ego_path_json) {
        SpeedPoint point;
        point.set_t(point_json["t"].asDouble());
        point.set_s(point_json["s"].asDouble());
        point.set_v(point_json["v"].asDouble());
        point.set_a(point_json["a"].asDouble());
        point.set_j(point_json["j"].asDouble());
        ego_path_points.push_back(point);
      }
      ego_yield_sim_result.ego_speed_data = ego_path_points;
    } else {
      std::cerr << "JSON中缺少yield_ego_speed_path的结果" << std::endl;
    }
    if (conflict_zones_json.isMember("obj_speed_path")) {
      SpeedVector agent_path_points;
      const Json::Value &ego_path_json = conflict_zones_json["obj_speed_path"];
      for (const auto &point_json : ego_path_json) {
        SpeedPoint point;
        point.set_t(point_json["t"].asDouble());
        point.set_s(point_json["s"].asDouble());
        point.set_v(point_json["v"].asDouble());
        point.set_a(point_json["a"].asDouble());
        point.set_j(point_json["j"].asDouble());
        agent_path_points.push_back(point);
      }
      ego_yield_sim_result.obj_speed_data = agent_path_points;
      std::cout << "agent_path_points_size" << agent_path_points.size();
    } else {
      std::cerr << "JSON中缺少yield_obj_speed_path的结果" << std::endl;
    }
  } else {
    std::cerr << "JSON中缺少迭代十次的结果" << std::endl;
  }
  ego_yield_sim_results.emplace(obj_traj_id1, ego_yield_sim_result);

  // get overtake_sim_results
  std::unordered_map<std::string, GamingSimResult> ego_overtake_sim_results;
  GamingSimResult ego_overtake_sim_result;
  std::string overtake_file_path =
      "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
      "test/UnitTestResult_id1_EgoPass+Cross.json";

  // 打开文件并读取内容到字符串
  std::ifstream overtake_file(overtake_file_path);
  if (!overtake_file.is_open()) {
    std::cerr << "无法打开文件: " << overtake_file_path << std::endl;
    return;  // 提前返回，避免后续操作
  }

  // 读取文件内容到字符串
  std::stringstream overtake_buffer;
  overtake_buffer << overtake_file.rdbuf();
  std::string overtake_content = overtake_buffer.str();

  // 解析JSON内容
  Json::Value overtake_debug_config;
  Json::CharReaderBuilder overtake_reader;
  std::string overtake_errs;
  std::istringstream overtake_s(overtake_content);
  if (!Json::parseFromStream(overtake_reader, overtake_s,
                             &overtake_debug_config, &overtake_errs)) {
    std::cerr << "JSON解析错误3: " << overtake_errs << std::endl;
    return;  // 解析失败，提前返回
  }
  std::string obj_traj_id2;
  if (overtake_debug_config.isMember("iter_time_10") &&
      !overtake_debug_config["iter_time_10"].isNull()) {
    const Json::Value &conflict_zones_json =
        overtake_debug_config["iter_time_10"];

    GamingConflictZoneInfo conflict_zone_info;
    GamingConflictZone &conflict_zone_in_ego_view =
        conflict_zone_info.conflict_zone_in_ego_view;
    if (conflict_zones_json.isMember("conflict_zone_in_ego_view") &&
        conflict_zones_json["conflict_zone_in_ego_view"].isObject()) {
      const Json::Value &agent_view_json =
          conflict_zones_json["conflict_zone_in_ego_view"];
      conflict_zone_in_ego_view.agent_cutin_s =
          agent_view_json["agent_cutin_s"].asDouble();
      conflict_zone_in_ego_view.agent_cutin_time =
          agent_view_json["agent_cutin_t"].asDouble();
      conflict_zone_in_ego_view.agent_cutout_s =
          agent_view_json["agent_cutout_s"].asDouble();
      conflict_zone_in_ego_view.agent_cutout_time =
          agent_view_json["agent_cutout_t"].asDouble();
      conflict_zone_in_ego_view.ego_cutin_s =
          agent_view_json["ego_cutin_s"].asDouble();
      conflict_zone_in_ego_view.ego_cutin_time =
          agent_view_json["ego_cutin_t"].asDouble();
      conflict_zone_in_ego_view.ego_cutout_s =
          agent_view_json["ego_cutout_s"].asDouble();
      conflict_zone_in_ego_view.ego_cutout_time =
          agent_view_json["ego_cutout_t"].asDouble();
      conflict_zone_in_ego_view.is_path_conflict =
          agent_view_json["is_path_conflict"].asBool();
      conflict_zone_in_ego_view.is_risk_field_conflict =
          agent_view_json["is_risk_field_conflict"].asBool();
      conflict_zone_in_ego_view.risk_field_lat_ratio =
          agent_view_json["risk_field_lat_ratio"].asDouble();
    } else {
      std::cerr << "JSON中缺少conflict_zone_in_ego_view的结果" << std::endl;
    }
    obj_traj_id2 = overtake_debug_config["id"].asString();
    ego_overtake_sim_result.sim_conflict_zone_in_ego_view =
        conflict_zone_in_ego_view;
    int interaction_type = overtake_debug_config["type"].asInt();
    if (interaction_type == 1) {
      ego_overtake_sim_result.interaction_type = InteractionType::kCross;
    } else if (interaction_type == 2) {
      ego_overtake_sim_result.interaction_type =
          InteractionType::kStraightMerge;
    } else if (interaction_type == 3) {
      ego_overtake_sim_result.interaction_type = InteractionType::kTurnMerge;
    }
    int ego_lon_type = overtake_debug_config["lontype"].asInt();
    if (ego_lon_type == 1) {
      ego_overtake_sim_result.ego_lon_type = LonGamingDecisionType::kYield;
    } else if (ego_lon_type == 2) {
      ego_overtake_sim_result.ego_lon_type = LonGamingDecisionType::kOvertake;
    } else {
      ego_overtake_sim_result.ego_lon_type = LonGamingDecisionType::kNone;
    }
    if (conflict_zones_json.isMember("ego_speed_path")) {
      SpeedVector ego_path_points;
      const Json::Value &ego_path_json = conflict_zones_json["ego_speed_path"];
      for (const auto &point_json : ego_path_json) {
        SpeedPoint point;
        point.set_t(point_json["t"].asDouble());
        point.set_s(point_json["s"].asDouble());
        point.set_v(point_json["v"].asDouble());
        point.set_a(point_json["a"].asDouble());
        point.set_j(point_json["j"].asDouble());
        ego_path_points.push_back(point);
      }
      ego_overtake_sim_result.ego_speed_data = ego_path_points;
    } else {
      std::cerr << "JSON中缺少overtake_ego_speed_path的结果" << std::endl;
    }

    SpeedVector agent_path_points;
    if (conflict_zones_json.isMember("obj_speed_path")) {
      const Json::Value &ego_path_json = conflict_zones_json["obj_speed_path"];
      for (const auto &point_json : ego_path_json) {
        SpeedPoint point;
        point.set_t(point_json["t"].asDouble());
        point.set_s(point_json["s"].asDouble());
        point.set_v(point_json["v"].asDouble());
        point.set_a(point_json["a"].asDouble());
        point.set_j(point_json["j"].asDouble());
        agent_path_points.push_back(point);
      }
    } else {
      std::cerr << "JSON中缺少overtake_obj_speed_path的结果" << std::endl;
    }
    ego_overtake_sim_result.obj_speed_data = agent_path_points;
  } else {
    std::cerr << "JSON中缺少迭代十次的结果" << std::endl;
  }
  ego_overtake_sim_results.emplace(obj_traj_id2, ego_overtake_sim_result);
  // 调用测试函数
  IDMConfigPlf aggressive_idm_config_plf = {
      PiecewiseLinearFunction<double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3}),
      PiecewiseLinearFunction<double>({0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0})
      //   PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 0.0}),
      //   PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 0.0})
  };
  IDMConfigPlf normal_idm_config_plf = {
      PiecewiseLinearFunction<double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {0.5, 0.5, 0.5, 0.6, 0.8, 1.0, 1.0}),
      PiecewiseLinearFunction<double>({0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {1.2, 1.2, 1.5, 2.0, 2.0, 2.0, 2.0, 2.0})
      // PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 2.0}),
      // PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 1.0})
  };
  // 3.1 用aggressive idm param 走multiagent，只对所有yield障碍物生成上边界

  // 3.2 解冲突
  // 用生成的upper_speed_data，重刷所有overtake的障碍物，完成解冲突（不调用multiagent）
  // 举例：obj id 1 2 3 其中 1 重刷后由超转让了
  // 调用CalcYieldTrajSpeedDataofMultiAgent输入 现有的upper_speed_data 只输入obj
  // 1 新生成的conflict zone，更新upper_speed_data
  SpeedGamingDecider decider(&vehicle_geo_params);
  MultiAgentIdm multi_agent_idm;
  SpeedIdmUpdateParams idm_param;
  upper_speed_data = multi_agent_idm.CalcYieldTrajSpeedDataOfMultiAgent(
      ego_init_point, ego_path, upper_speed_data, aggressive_idm_config_plf,
      aggressive_idm_config_plf, speed_limit_plf, idm_param,
      ego_yield_sim_results);

  bool has_overtake_to_yield = true;
  while (has_overtake_to_yield) {
    has_overtake_to_yield = false;
    std::vector<std::string> overtake_remove_ids;
    for (auto it = ego_overtake_sim_results.begin();
         it != ego_overtake_sim_results.end(); it++) {
      GamingSimResult sim_result;
      sim_result.interaction_type = it->second.interaction_type;
      auto origin_conflict_zone_iter = conflict_zone_infos_.find(it->first);
      if (origin_conflict_zone_iter != conflict_zone_infos_.end()) {
        switch (it->second.interaction_type) {
          case InteractionType::kCross:
            decider.RunCrossGaming(ego_path, ego_init_point, traj_mgr,
                                   origin_conflict_zone_iter->second,
                                   speed_limit_plf, upper_speed_data,
                                   normal_yield_speed_data, &sim_result);
            break;
          case InteractionType::kStraightMerge:
            decider.RunAgentMergeEgoGaming(
                ego_path, ego_init_point, traj_mgr,
                origin_conflict_zone_iter->second, speed_limit_plf,
                upper_speed_data, normal_yield_speed_data, &sim_result);
            std::cout << "agent_cross_angle "
                      << origin_conflict_zone_iter->second
                             .conflict_zone_in_ego_view.agent_cross_angle
                      << std::endl;
            break;
          case InteractionType::kTurnMerge:
            decider.RunEgoMergeAgentGaming(
                ego_path, ego_init_point, traj_mgr,
                origin_conflict_zone_iter->second, speed_limit_plf,
                upper_speed_data, normal_yield_speed_data, &sim_result);
            break;
          default:
            break;
        }
      }
      if (sim_result.ego_lon_type == LonGamingDecisionType::kYield) {
        has_overtake_to_yield = true;
        ego_yield_sim_results.insert({it->first, sim_result});
        overtake_remove_ids.push_back(it->first);
      }
    }
    for (auto &id : overtake_remove_ids) {
      ego_overtake_sim_results.erase(id);
    }
    if (has_overtake_to_yield) {
      upper_speed_data = multi_agent_idm.CalcYieldTrajSpeedDataOfMultiAgent(
          ego_init_point, ego_path, upper_speed_data, aggressive_idm_config_plf,
          aggressive_idm_config_plf, speed_limit_plf, idm_param,
          ego_yield_sim_results);
    }
  }

  // 3.3 生成博弈预测轨迹（不调用multiagent）

  // 3.4 考虑所有结果，先重新生成normal speed data,在基于此生成ref speed
  // data，处理成参考轨迹 upper_speed_data在3.2刷新过了
  normal_yield_speed_data = multi_agent_idm.CalcYieldTrajSpeedDataOfMultiAgent(
      ego_init_point, ego_path, upper_speed_data, normal_idm_config_plf,
      normal_idm_config_plf, speed_limit_plf, idm_param, ego_yield_sim_results);

  auto ego_speed_data = multi_agent_idm.CalcTrajSpeedDataOfMultiAgent(
      ego_init_point, ego_path, upper_speed_data, normal_yield_speed_data,
      speed_limit_plf, idm_param, ego_overtake_sim_results);

  std::cout << ego_speed_data.size() << std::endl;
  for (auto &result : ego_yield_sim_results)
    SaveUnitTestData(result.first, result.second, ego_speed_data);
  std::cout << ego_overtake_sim_results.size() << std::endl;
  for (auto &result : ego_overtake_sim_results)
    SaveUnitTestData(result.first, result.second, ego_speed_data);
}
}  // namespace st::planning