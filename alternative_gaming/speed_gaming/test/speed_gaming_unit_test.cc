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

#include "../simulator/alternative_idm_simulator.h"
#include "../simulator/speed_idm_common.h"
#include "../speed_gaming_decider.h"
#include "test_util.h"

namespace st::planning {

TEST(SpeedGamingUnitTest, TestProcess) {
  DiscretizedPath av_path;
  std::vector<SpacetimeTrajectoryManager> speed_traj_manager;
  std::vector<GamingConflictZoneInfo> conflict_zone_infos;
  std::vector<AidmScene> aidm_scenes;
  SpeedVector upper_speed_data;
  SpeedVector normal_yield_speed_data;
  InteractionType interaction_type;
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
    std::cerr << "JSON解析错误: " << errs << std::endl;
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

      conflict_zone_infos.push_back(conflict_zone_info);

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
      //           << ": s=" << point_json["s"].asDouble()
      //           << ", x=" << point_json["x"].asDouble()
      //           << ", y=" << point_json["y"].asDouble()
      //           << ", yaw=" << point_json["yaw"].asDouble() << std::endl;

      PathPoint point;
      point.set_s(point_json["s"].asDouble());
      point.set_x(point_json["x"].asDouble());
      point.set_y(point_json["y"].asDouble());
      point.set_theta(point_json["yaw"].asDouble());
      path_points.push_back(point);
    }
    av_path = DiscretizedPath(std::move(path_points));
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
  ego_init_point.set_pos(Vec2d(av_path.front().x(), av_path.front().y()));
  ego_init_point.set_theta(av_path.front().theta());
  ego_init_point.set_s(av_path.front().s());
  ego_init_point.set_v(5.0);

  std::vector<double> s_list{0.0, 50.0, 100.0};
  std::vector<double> v_list{20.0, 20.0, 20.0};
  const PiecewiseLinearFunction<double> speed_limit_plf(s_list, v_list);

  //调用测试函数
  SpeedGamingDecider unit_test(&vehicle_geo_params);
  if (!conflict_zone_infos.empty()) {
    int idx = 0;
    for (const auto &conflict_zone_info : conflict_zone_infos) {
      GamingSimResult sim_result;
      unit_test.RunGaming(av_path, ego_init_point, traj_mgr, conflict_zone_info,
                          speed_limit_plf, upper_speed_data,
                          normal_yield_speed_data, aidm_scenes[idx],
                          &sim_result);
      idx++;
    }
  }
}
}  // namespace st::planning