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
      "test/Intergrated_TestResult_id" +
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

TEST(IntegratedModuletest, TestProcess)

{
  std::vector<SpacetimeTrajectoryManager> speed_traj_manager;
  std::unordered_map<std::string, GamingConflictZoneInfo> conflict_zone_infos_;
  std::vector<AidmScene> aidm_scenes;
  SpeedVector upper_speed_data;
  SpeedVector normal_yield_speed_data;
  AidmScene aidm_scene;

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
  std::vector<geometry_path> segments;
  segments.push_back(geometry_path(40, 0.0));
  DiscretizedPath ego_path = construct_path(0.0, 0.0, 0.0, segments);

  std::vector<double> s_list{0.0, 50.0, 100.0};
  std::vector<double> v_list{20.0, 20.0, 20.0};
  const PiecewiseLinearFunction<double> speed_limit_plf(s_list, v_list);
  SpeedGamingDecider decider(&vehicle_geo_params);
  // decider.conflict_zone_infos_ = conflict_zone_infos_;
  SpeedGamingInput input;
  input.traj_mgr = &traj_mgr;
  input.ego_path = &ego_path;
  input.plan_start_v = 5.0;
  input.plan_start_a = 0.0;
  input.plan_start_j = 0.0;
  decider.Execute(input);
}
}  // namespace st::planning