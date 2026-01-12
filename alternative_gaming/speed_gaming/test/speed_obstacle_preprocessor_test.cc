/**
 * @file speed_obstacle_preprocessor_test.cc
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-04-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <fcntl.h>
#include <gtest/gtest.h>
#include <json/json.h>

#include <fstream>
#include <iostream>

#include "google/protobuf/io/zero_copy_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/message.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/json_util.h"
#include "../speed_obstacle_processor.h"
#include "test_util.h"

namespace st::planning {

TEST(SpeedObstaclePreprocessorTest, TestProcess) {
  // construct spacetime object manager.
  ObjectPredictionProto obj_pre_proto1;
  ObjectPredictionProto obj_pre_proto2;
  ObjectPredictionProto obj_pre_proto3;
  if (!TextFileToProto(
          "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
          "test/cross_object_prediction.proto.txt",
          &obj_pre_proto1) ||
      !TextFileToProto(
          "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
          "test/merge_object_prediction.proto.txt",
          &obj_pre_proto2) ||
      !TextFileToProto(
          "/apollo/modules/cnoa_pnc/planning/alternative_gaming/speed_gaming/"
          "test/turnmerge_object_prediction.proto.txt",
          &obj_pre_proto3)) {
    std::cerr << "解析失败" << std::endl;
  }
  prediction::ObjectPrediction object_prediction1(obj_pre_proto1);
  prediction::ObjectPrediction object_prediction2(obj_pre_proto2);
  prediction::ObjectPrediction object_prediction3(obj_pre_proto3);
  PlannerObject objects[3] = {PlannerObject(object_prediction1),
                              PlannerObject(object_prediction2),
                              PlannerObject(object_prediction3)};
  SpacetimeTrajectoryManager traj_mgr(absl::MakeSpan(objects, 3));

  // construct ego path
  std::vector<geometry_path> segments;
  // for straightmerge and cross
  segments.push_back(geometry_path(40, 0.0));
  DiscretizedPath ego_path = construct_path(0.0, 0.0, 0.0, segments);

  // for turnmerge
  // segments.push_back(geometry_path(16.0, -0.1));
  // segments.push_back(geometry_path(10.0, 0.0));
  // DiscretizedPath ego_path = construct_path(0.0, 0.0, 0.5 * M_PI, segments);

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

  SpeedObstacleProcessor processor;
  processor.Init(&traj_mgr, &ego_path, &vehicle_geo_params);
  std::unordered_map<std::string, GamingConflictZoneInfo> conflict_zone_infos;
  processor.Process(conflict_zone_infos);

  SaveDebugData(conflict_zone_infos, ego_path, traj_mgr);

  EXPECT_FALSE(conflict_zone_infos.empty());
}
}  // namespace st::planning