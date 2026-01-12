/**
 * @file test_util.h
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-04-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <fstream>
#include <iostream>

#pragma once
#include <fcntl.h>
#include <json/json.h>
#include "alternative_gaming/speed_gaming/speed_obstacle_processor.h"
#include "google/protobuf/io/zero_copy_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/message.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/json_util.h"

namespace st::planning {
struct geometry_path {
  geometry_path(double _length, double _kappa)
      : length(_length), kappa(_kappa) {}
  double length;
  double kappa;
};

inline void SaveUnitTestData(
    Json::Value &g_root, const int i, const std::string &id,
    const std::string &simulation_type, const SpeedVector &ego_speed_data,
    const SpeedVector &obj_speed_data,
    const GamingConflictZone &riskfield_conflict_zone_in_ego_view) {
  if (ego_speed_data.empty() || obj_speed_data.empty()) {
    return;
  }

  Json::Value data;
  Json::Value ego;
  for (auto &pt : ego_speed_data) {
    Json::Value node;
    node["t"] = pt.t();
    node["s"] = pt.s();
    node["v"] = pt.v();
    node["a"] = pt.a();
    node["j"] = pt.j();
    ego.append(node);
  }
  data["ego_speed_path"] = ego;

  Json::Value obj;
  for (auto &pt : obj_speed_data) {
    Json::Value node;
    node["t"] = pt.t();
    node["s"] = pt.s();
    node["v"] = pt.v();
    node["a"] = pt.a();
    node["j"] = pt.j();
    obj.append(node);
  }
  data["obj_speed_path"] = obj;

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
  data["conflict_zone_in_ego_view"] = conflict_zone_in_ego_view;

  if (i == 0) {
    g_root.clear();
    g_root["type"] = simulation_type;
    g_root["id"] = id;
  }
  g_root["iter_time_" + absl::StrCat(i)] = data;

  Json::StyledWriter sw;
  std::ofstream os;
  std::string path =
      "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
      "test/UnitTestResult_id" +
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

inline void SaveTestData(Json::Value &g_root, const std::string &id,
                         const GamingSimResult &SimResult,
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

inline DiscretizedPath construct_path(
    double init_x, double init_y, double init_theta,
    const std::vector<geometry_path> &segments) {
  std::vector<PathPoint> points;
  points.clear();
  PathPoint p0;
  p0.set_x(init_x);
  p0.set_y(init_y);
  p0.set_theta(init_theta);
  p0.set_kappa(0.0);
  p0.set_s(0.0);
  points.push_back(p0);
  for (auto &segment : segments) {
    points.back().set_kappa(segment.kappa);
    double seg_init_theta = points.back().theta();
    double x, y, theta;
    int point_size = segment.length;  //假设1米1个点
    bool circle = abs(segment.kappa) > 1e-4;
    double c_x, c_y, c_R, c_theta;
    if (circle) {
      c_R = 1.0 / segment.kappa;
      c_x = points.back().x() + c_R * std::cos(seg_init_theta + 0.5 * M_PI);
      c_y = points.back().y() + c_R * std::sin(seg_init_theta + 0.5 * M_PI);
      c_theta = atan2(points.back().y() - c_y, points.back().x() - c_x);
    }
    double theta_p = c_theta;
    while (point_size) {
      if (!circle) {
        x = points.back().x() + std::cos(seg_init_theta);
        y = points.back().y() + std::sin(seg_init_theta);
        theta = seg_init_theta;
      } else {
        theta_p = NormalizeAngle(theta_p + segment.kappa);
        x = c_x + abs(c_R) * std::cos(theta_p);
        y = c_y + abs(c_R) * std::sin(theta_p);
        seg_init_theta = NormalizeAngle(seg_init_theta + segment.kappa);
        theta = seg_init_theta;
      }
      p0.set_x(x);
      p0.set_y(y);
      p0.set_theta(theta);
      p0.set_kappa(segment.kappa);
      p0.set_s(points.back().s() + 1.0);
      points.push_back(p0);
      point_size--;
    }
  }
  return DiscretizedPath(points);
}

inline bool TextFileToProto(const std::string &file_name,
                            google::protobuf::Message *message) {
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int fd = open(file_name.c_str(), O_RDONLY);
  if (fd < 0) {
    return false;
  }
  ZeroCopyInputStream *input = new FileInputStream(fd);
  bool success = google::protobuf::TextFormat::Parse(input, message);
  delete input;
  close(fd);
  return success;
}

inline bool SaveDebugData(
    const std::unordered_map<std::string, GamingConflictZoneInfo>
        &conflict_zone_infos,
    const DiscretizedPath &ego_path,
    const SpacetimeTrajectoryManager &traj_mgr) {
  Json::Value root;

  Json::Value ego;
  for (auto &pt : ego_path) {
    Json::Value node;
    node["x"] = pt.x();
    node["y"] = pt.y();
    node["yaw"] = pt.theta();
    node["s"] = pt.s();
    ego.append(node);
  }
  root["ego_path"] = ego;

  Json::Value obj_infos;
  for (auto &traj : traj_mgr.trajectories()) {
    Json::Value node;
    node["id"] = traj.traj_id();
    node["type"] = traj.planner_object().type();
    node["vx"] = traj.planner_object().velocity().x();
    node["vy"] = traj.planner_object().velocity().y();
    node["length"] = traj.planner_object().bounding_box().length();
    node["width"] = traj.planner_object().bounding_box().width();
    Json::Value traj_points;
    for (auto &pt : traj.states()) {
      Json::Value p;
      p["x"] = pt.traj_point->pos().x();
      p["y"] = pt.traj_point->pos().y();
      p["yaw"] = pt.traj_point->theta();
      p["t"] = pt.traj_point->t();
      p["s"] = pt.traj_point->s();
      p["v"] = pt.traj_point->v();
      p["a"] = pt.traj_point->a();
      traj_points.append(p);
    }
    node["traj_points"] = traj_points;

    //从障碍物预测轨迹抽取path
    const DiscretizedPath obj_path =
        SpeedGamingCommon::GeneratePathBasedTraj(&traj, 100, false);
    for (auto iter = conflict_zone_infos.begin();
         iter != conflict_zone_infos.end(); iter++) {
      if (iter->second.obj_traj_id == traj.traj_id()) {
        node["agent_cutin_x"] =
            obj_path
                .Evaluate(iter->second.conflict_zone_in_ego_view.agent_cutin_s)
                .x();
        node["agent_cutin_y"] =
            obj_path
                .Evaluate(iter->second.conflict_zone_in_ego_view.agent_cutin_s)
                .y();
        node["agent_cutout_x"] =
            obj_path
                .Evaluate(iter->second.conflict_zone_in_ego_view.agent_cutout_s)
                .x();
        node["agent_cutout_y"] =
            obj_path
                .Evaluate(iter->second.conflict_zone_in_ego_view.agent_cutout_s)
                .y();
      }
    }
    obj_infos.append(node);
  }
  root["obj_infos"] = obj_infos;

  Json::Value conflict_zones;
  for (auto iter = conflict_zone_infos.begin();
       iter != conflict_zone_infos.end(); iter++) {
    Json::Value node;
    node["type"] = iter->second.interaction_type;
    node["id"] = iter->second.obj_traj_id;
    node["is_pre_gaming"] = iter->second.is_pre_gaming;
    Json::Value ego_view;
    ego_view["ego_cutin_s"] =
        iter->second.conflict_zone_in_ego_view.ego_cutin_s;
    ego_view["ego_cutout_s"] =
        iter->second.conflict_zone_in_ego_view.ego_cutout_s;
    ego_view["agent_cutin_time"] =
        iter->second.conflict_zone_in_ego_view.agent_cutin_time;
    ego_view["agent_cutin_s"] =
        iter->second.conflict_zone_in_ego_view.agent_cutin_s;
    ego_view["agent_cutout_time"] =
        iter->second.conflict_zone_in_ego_view.agent_cutout_time;
    ego_view["agent_cutout_s"] =
        iter->second.conflict_zone_in_ego_view.agent_cutout_s;
    ego_view["risk_field_lat_ratio"] =
        iter->second.conflict_zone_in_ego_view.risk_field_lat_ratio;
    ego_view["is_path_conflict"] =
        iter->second.conflict_zone_in_ego_view.is_path_conflict;
    ego_view["is_risk_field_conflict"] =
        iter->second.conflict_zone_in_ego_view.is_risk_field_conflict;
    ego_view["ego_cutin_x"] =
        ego_path.Evaluate(iter->second.conflict_zone_in_ego_view.ego_cutin_s)
            .x();
    ego_view["ego_cutin_y"] =
        ego_path.Evaluate(iter->second.conflict_zone_in_ego_view.ego_cutin_s)
            .y();
    ego_view["ego_cutout_x"] =
        ego_path.Evaluate(iter->second.conflict_zone_in_ego_view.ego_cutout_s)
            .x();
    ego_view["ego_cutout_y"] =
        ego_path.Evaluate(iter->second.conflict_zone_in_ego_view.ego_cutout_s)
            .y();
    node["ego_view"] = ego_view;

    Json::Value agent_view;
    agent_view["ego_cutin_s"] =
        iter->second.conflict_zone_in_agent_view.ego_cutin_s;
    agent_view["ego_cutout_s"] =
        iter->second.conflict_zone_in_agent_view.ego_cutout_s;
    agent_view["agent_cutin_time"] =
        iter->second.conflict_zone_in_agent_view.agent_cutin_time;
    agent_view["agent_cutin_s"] =
        iter->second.conflict_zone_in_agent_view.agent_cutin_s;
    agent_view["agent_cutout_time"] =
        iter->second.conflict_zone_in_agent_view.agent_cutout_time;
    agent_view["agent_cutout_s"] =
        iter->second.conflict_zone_in_agent_view.agent_cutout_s;
    agent_view["risk_field_lat_ratio"] =
        iter->second.conflict_zone_in_agent_view.risk_field_lat_ratio;
    agent_view["is_path_conflict"] =
        iter->second.conflict_zone_in_agent_view.is_path_conflict;
    agent_view["is_risk_field_conflict"] =
        iter->second.conflict_zone_in_agent_view.is_risk_field_conflict;
    node["agent_view"] = agent_view;
    Json::Value ego_left_risk_edge;
    for (auto &p : iter->second.ego_left_risk_edge) {
      Json::Value pt;
      pt["x"] = p.first;
      pt["y"] = p.second;
      ego_left_risk_edge.append(pt);
    }
    node["ego_left_risk_edge"] = ego_left_risk_edge;
    Json::Value ego_right_risk_edge;
    for (auto &p : iter->second.ego_right_risk_edge) {
      Json::Value pt;
      pt["x"] = p.first;
      pt["y"] = p.second;
      ego_right_risk_edge.append(pt);
    }
    node["ego_right_risk_edge"] = ego_right_risk_edge;
    Json::Value agent_left_risk_edge;
    for (auto &p : iter->second.agent_left_risk_edge) {
      Json::Value pt;
      pt["x"] = p.first;
      pt["y"] = p.second;
      agent_left_risk_edge.append(pt);
    }
    node["agent_left_risk_edge"] = agent_left_risk_edge;
    Json::Value agent_right_risk_edge;
    for (auto &p : iter->second.agent_right_risk_edge) {
      Json::Value pt;
      pt["x"] = p.first;
      pt["y"] = p.second;
      agent_right_risk_edge.append(pt);
    }
    node["agent_right_risk_edge"] = agent_right_risk_edge;
    conflict_zones.append(node);
  }
  root["conflict_zones"] = conflict_zones;

  Json::StyledWriter sw;
  std::ofstream os;
  std::string path =
      "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
      "test/obs_preprocess_debug.json";
  os.open(path, std::ios::out);
  if (!os.is_open()) {
    std::cout << "error:can not find or create the file";
    return false;
  }
  os << sw.write(root);
  os.close();
  return true;
}
}  // namespace st::planning