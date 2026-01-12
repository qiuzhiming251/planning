#include <algorithm>
#include <string>

#include "absl/strings/str_cat.h"
#include "plan_common/planning_macros.h"
//#include "config/highway/config_manager.h"
#include <memory>

#include "plan_common/log.h"
#include "plan_common/maps/stop_line.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "decider/scene_manager/construction_scene_identification.h"

namespace st::planning {

bool RunConstructionSceneIdentification(
    const ConstructionSceneIdentificationInput& input) {
  SCOPED_TRACE(__FUNCTION__);

  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.plan_start_point);
  CHECK_NOTNULL(input.target_lane_seq);
  CHECK_NOTNULL(input.lane_change_state);
  CHECK_NOTNULL(input.vehicle_param);
  // check construction inhabit condition
  // 0.function mode must lcc mode
  if (input.function_id.has_value() &&
      input.function_id.value() != Behavior_FunctionId_LKA) {
    Log2DDS::LogDataV2(
        "construct_lc_debug",
        absl::StrCat("current function_mode condition:",
                     Behavior_FunctionId_Name(input.function_id.value())));
    return false;
  }

  // 1.lane_change_stage check
  if (input.lane_change_state->stage() != LaneChangeStage::LCS_NONE) {
    Log2DDS::LogDataV2(
        "construct_lc_debug",
        absl::StrCat("lane_change_state condition. current stage:",
                     LaneChangeStage_Name(input.lane_change_state->stage())));
    return false;
  }

  // 2.check in tunnel
  if (input.tunnel_status.has_value() && input.tunnel_status.value() != 0) {
    Log2DDS::LogDataV2("construct_lc_debug", "in tunnel.");
    return false;
  }

  // 3.check vehicle speed
  // if (input.plan_start_point->v() > Kph2Mps(130.0) ||
  //     input.plan_start_point->v() < Kph2Mps(45.0)) {
  //   Log2DDS::LogDataV2("construct_lc_debug",
  //                      absl::StrCat("ego speed condition. current speed:",
  //                                   Mps2Kph(input.plan_start_point->v())));
  //   return false;
  // }

  // 4.check driver_set_speed
  if (input.lcc_cruising_speed_limit.has_value() &&
      input.lcc_cruising_speed_limit.value() < 1.0) {
    Log2DDS::LogDataV2("construct_lc_debug",
                       absl::StrCat("driver_set_speed condition.set speed:",
                                    *input.lcc_cruising_speed_limit));
    return false;
  }

  const auto target_lane_seq = input.target_lane_seq;

  // 5.ramp suppression
  // if (target_lane_seq->nearest_lane->type() == LANE_RAMP ||
  //     target_lane_seq->nearest_lane->type() == LANE_DEC) {
  //   Log2DDS::LogDataV2("blocker_lc_debug", "ego in ramp.");
  //   return false;
  // }

  // 6.junction with 100m condition check
  double stop_line_s = 0.0;
  double start_point_s = 0.0;
  const auto& start_point = input.plan_start_point->path_point();
  if (input.psmm->map_ptr() &&
      !input.psmm->map_ptr()->stop_line_map().empty()) {
    std::shared_ptr<ad_byd::planning::StopLine> stop_line = nullptr;
    for (const auto& [id, stop_line_ptr] :
         input.psmm->map_ptr()->stop_line_map()) {
      if (!(id == 0)) {
        stop_line = stop_line_ptr;
        break;
      }
    }
    if (!(stop_line->id() == 0) && stop_line->points().size() > 1) {
      const double stop_line_x =
          (stop_line->points().front().x() + stop_line->points().back().x()) /
          2.0;
      const double stop_line_y =
          (stop_line->points().front().y() + stop_line->points().back().y()) /
          2.0;
      input.target_lane_seq->GetProjectionDistance({stop_line_x, stop_line_y},
                                                   &stop_line_s);
    }
    input.target_lane_seq->GetProjectionDistance(
        {start_point.x(), start_point.y()}, &start_point_s);
    if (stop_line_s - start_point_s < 100) {
      Log2DDS::LogDataV2(
          "construct_lc_debug",
          absl::StrCat("distance to junction condition,distance:",
                       stop_line_s - start_point_s));
      return false;
    }
  } else {
    LaneConstPtr lane;
    double dist_to_junction = input.target_lane_seq->GetDistanceToJunction(
        start_point.x(), start_point.y(), lane);
    if (dist_to_junction < 100.0) {
      Log2DDS::LogDataV2(
          "construct_lc_debug",
          absl::StrCat("distance to junction condition 2,distance:",
                       dist_to_junction));
      return false;
    }
  }
  // 7.ego in junction
  if (auto const nearstLane =
          input.target_lane_seq
              ->GetNearestLane({start_point.x(), start_point.y()})
              ->type();
      nearstLane == ad_byd::planning::LaneType::LANE_VIRTUAL_COMMON ||
      nearstLane == ad_byd::planning::LaneType::LANE_VIRTUAL_JUNCTION) {
    Log2DDS::LogDataV2("construct_lc_debug", "ego in junction condition!");
    return false;
  }

  // 8. dist to toll
  static double dist_from_toll = DBL_MAX;
  static bool reach_toll = false;
  const auto& v2_info = input.psmm->map_ptr()->v2_info();
  if (!reach_toll && v2_info.dist_to_toll < 20.0) {
    reach_toll = true;
    dist_from_toll = 0.0;
  } else if (!reach_toll && v2_info.dist_to_toll < 200.0) {
    Log2DDS::LogDataV2(
        "construct_lc_debug",
        absl::StrCat("ego close to toll!", v2_info.dist_to_toll));
    return false;
  }
  if (reach_toll || v2_info.dist_to_toll < 20.0) {
    dist_from_toll += input.plan_start_point->v() * 0.1;
    if (dist_from_toll < 100.0) {
      Log2DDS::LogDataV2(
          "construct_lc_debug",
          absl::StrCat("ego just leave from toll!", dist_from_toll));
      return false;
    } else {
      reach_toll = false;
      dist_from_toll = DBL_MAX;
    }
  } else {
    reach_toll = false;
    dist_from_toll = DBL_MAX;
  }
  Log2DDS::LogDataV2("construct_lc_debug",
                     absl::StrCat("dis to toll:", v2_info.dist_to_toll,
                                  ",dis from toll:", dist_from_toll));

  return ConstructionBlockerLcUUTrigger(input);
}

bool RunBlockerSceneIdentification(
    const BlockerSceneIdentificationInput& blockerinput) {
  SCOPED_TRACE(__FUNCTION__);

  CHECK_NOTNULL(blockerinput.psmm);
  CHECK_NOTNULL(blockerinput.plan_start_point);
  // CHECK_NOTNULL(blockerinput.ext_cmd_status);
  CHECK_NOTNULL(blockerinput.target_lane_seq);
  CHECK_NOTNULL(blockerinput.lane_change_state);
  CHECK_NOTNULL(blockerinput.vehicle_param);
  CHECK_NOTNULL(blockerinput.target_lane_seq_blocker);
  // check construction inhabit condition
  // 0.function mode must lcc mode
  if (blockerinput.function_id.has_value() &&
      blockerinput.function_id.value() != Behavior_FunctionId_LKA) {
    Log2DDS::LogDataV2("st_blocker_lc_debug",
                       absl::StrCat("current function_mode blocker:",
                                    Behavior_FunctionId_Name(
                                        blockerinput.function_id.value())));
    return false;
  }

  // 1.lane_change_stage check
  if (blockerinput.lane_change_state->stage() != LaneChangeStage::LCS_NONE) {
    Log2DDS::LogDataV2(
        "st_blocker_lc_debug",
        absl::StrCat(
            "lane_change_state blocker. current stage:",
            LaneChangeStage_Name(blockerinput.lane_change_state->stage())));
    return false;
  }

  // 2.check in tunnel
  if (blockerinput.tunnel_status.has_value() &&
      blockerinput.tunnel_status.value() != 0) {
    Log2DDS::LogDataV2("st_blocker_lc_debug", "in tunnel.");
    return false;
  }

  // 4.check driver_set_speed
  if (blockerinput.function_id == Behavior_FunctionId_LKA) {
    if (blockerinput.lcc_cruising_speed_limit.has_value() &&
        blockerinput.lcc_cruising_speed_limit.value() < 1.0) {
      Log2DDS::LogDataV2("st_blocker_lc_debug",
                         absl::StrCat("driver_set_speed blocker.set speed:",
                                      *blockerinput.lcc_cruising_speed_limit));
      return false;
    }
  }
  // 6.junction with 100m condition check
  double stop_line_s = 0.0;
  double start_point_s = 0.0;
  double dist_to_junction = 0.0;
  const auto& start_point = blockerinput.plan_start_point->path_point();
  if (blockerinput.psmm->map_ptr() &&
      !blockerinput.psmm->map_ptr()->stop_line_map().empty()) {
    std::shared_ptr<ad_byd::planning::StopLine> stop_line = nullptr;
    for (const auto& [id, stop_line_ptr] :
         blockerinput.psmm->map_ptr()->stop_line_map()) {
      if (!(id == 0)) {
        stop_line = stop_line_ptr;
        break;
      }
    }
    if (!(stop_line->id() == 0) && stop_line->points().size() > 1) {
      const double stop_line_x =
          (stop_line->points().front().x() + stop_line->points().back().x()) /
          2.0;
      const double stop_line_y =
          (stop_line->points().front().y() + stop_line->points().back().y()) /
          2.0;
      blockerinput.target_lane_seq->GetProjectionDistance(
          {stop_line_x, stop_line_y}, &stop_line_s);
    }
    blockerinput.target_lane_seq->GetProjectionDistance(
        {start_point.x(), start_point.y()}, &start_point_s);
    dist_to_junction = stop_line_s - start_point_s;
    // if (dist_to_junction < 100) {
    //   Log2DDS::LogDataV2("st_blocker_lc_debug",
    //                      absl::StrCat("distance to junction
    //                      blocker,distance:",
    //                                   dist_to_junction));
    //   return false;
    // }
  } else {
    LaneConstPtr lane;
    dist_to_junction = blockerinput.target_lane_seq->GetDistanceToJunction(
        start_point.x(), start_point.y(), lane);
  }
  // 7.ego in junction
  if (auto const nearstLane =
          blockerinput.target_lane_seq
              ->GetNearestLane({start_point.x(), start_point.y()})
              ->type();
      nearstLane == ad_byd::planning::LaneType::LANE_VIRTUAL_COMMON ||
      nearstLane == ad_byd::planning::LaneType::LANE_VIRTUAL_JUNCTION) {
    Log2DDS::LogDataV2("st_blocker_lc_debug", "ego in junction condition!");
    return false;
  }

  // 8. dist to toll
  static double dist_from_toll = DBL_MAX;
  static bool reach_toll = false;
  const auto& v2_info = blockerinput.psmm->map_ptr()->v2_info();
  if (!reach_toll && v2_info.dist_to_toll < 20.0) {
    reach_toll = true;
    dist_from_toll = 0.0;
  } else if (!reach_toll && v2_info.dist_to_toll < 200.0) {
    Log2DDS::LogDataV2(
        "st_blocker_lc_debug",
        absl::StrCat("ego close to toll!", v2_info.dist_to_toll));
    return false;
  }
  if (reach_toll || v2_info.dist_to_toll < 20.0) {
    dist_from_toll += blockerinput.plan_start_point->v() * 0.1;
    if (dist_from_toll < 100.0) {
      Log2DDS::LogDataV2(
          "st_blocker_lc_debug",
          absl::StrCat("ego just leave from toll!", dist_from_toll));
      return false;
    } else {
      reach_toll = false;
      dist_from_toll = DBL_MAX;
    }
  } else {
    reach_toll = false;
    dist_from_toll = DBL_MAX;
  }
  Log2DDS::LogDataV2("st_blocker_lc_debug",
                     absl::StrCat("dis to toll:", v2_info.dist_to_toll,
                                  ",dis from toll:", dist_from_toll));

  return BlockerLcUUTrigger(blockerinput, dist_to_junction);
}

bool ConstructionBlockerLcUUTrigger(
    const ConstructionSceneIdentificationInput& input) {
  const auto map = input.psmm->map_ptr();
  const auto& lane_seq = input.target_lane_seq;
  // const auto& lanes = lane_seq_info->lanes();
  const auto& start_point = input.plan_start_point->path_point();
  double start_point_s = 0.0;
  lane_seq->GetProjectionDistance({start_point.x(), start_point.y()},
                                  &start_point_s);
  double trust_UU_dist = 150.0;

  // map->sub_type() != MapSubType::HIGHWAY_HD &&
  // map->sub_type() != MapSubType::BEFORE_NEAR_TOLL

  const LaneConstPtr nearest_lane =
      lane_seq->GetNearestLane(Vec2d(input.plan_start_point->path_point().x(),
                                     input.plan_start_point->path_point().y()));
  const auto& coeffs = nearest_lane->lane_info().coeffs;
  if (coeffs.size() >= 4) {
    double trust_max_dis = 150.0;
    lane_seq->GetProjectionDistance(Vec2d(coeffs[2], coeffs[3]),
                                    &trust_max_dis);
    trust_UU_dist = trust_max_dis - start_point_s;
    Log2DDS::LogDataV2("construct_lc_debug",
                       "trust_UU_dist:" + absl::StrCat(trust_UU_dist));
  }

  bool trapezoid_flag = false, rectangle_flag = false;
  int current_cone_count = 0, current_barrel_count = 0;
  int rectangle_cone_count = 0, rectangle_barrel_count = 0;
  const auto& stationary_objects = input.obj_mgr.stationary_objects();
  std::vector<StationaryObstacle> fsd_obs, obstacles;
  obstacles.reserve(stationary_objects.size());
  fsd_obs.reserve(stationary_objects.size());
  ConvertStObstaclesToFsdObs(stationary_objects, fsd_obs);
  if (!ClassifyObstaclesByPositionAndSetSLBoundary(lane_seq, fsd_obs, obstacles,
                                                   start_point_s)) {
    Log2DDS::LogDataV2("construct_lc_debug",
                       "ClassifyObstaclesByPositionAndSetSLBoundary");
    return false;
  }
  // ignore uu in front of leader car
  const auto& all_objects = input.obj_mgr.planner_objects();
  double leader_car_s_offset =
      GetLeaderCarSOffset(lane_seq, all_objects, start_point_s);
  Log2DDS::LogDataV2("construct_lc_debug",
                     absl::StrCat("leader_car_s_offset:", leader_car_s_offset));

  std::string current_front_UU_id;
  if (!obstacles.empty()) {
    const double check_pre_dist = 150.0;
    for (const StationaryObstacle& obstacle : obstacles) {
      LaneConstPtr nearest_lane = lane_seq->GetNearestLane(
          ad_byd::planning::math::Vec2d(obstacle.x(), obstacle.y()));
      if (!nearest_lane) continue;
      double target_half_lane_width =
          nearest_lane->GetWidthAtPoint(obstacle.x(), obstacle.y()) * 0.5;
      target_half_lane_width =
          ad_byd::planning::math::Clamp(target_half_lane_width, 1.2, 2.2);
      if (obstacle.ds() >= leader_car_s_offset) continue;
      if (obstacle.ds() >= 80.0) continue;
      // count cone in rectangle range
      if (obstacle.ds() >= 80.0 && obstacle.ds() <= trust_UU_dist) {
        if (obstacle.sl_boundary().l_max > -target_half_lane_width + 0.5 ||
            obstacle.sl_boundary().l_min < target_half_lane_width - 0.5) {
          if (obstacle.type() == OT_UNKNOWN_STATIC) {
            rectangle_flag = true;
          } else if (obstacle.type() == OT_CONE) {
            rectangle_cone_count += 1;
          } else if (obstacle.type() == OT_BARRIER) {
            rectangle_barrel_count += 1;
          }
        }
      }
      //
      std::vector<Vec2d> points;
      lane_seq->SamplePoints(0.0, &points);
      ad_byd::planning::Path path(points);
      if (path.points().empty()) {
        return false;
      }
      SLBoundary sl_boundary;
      double l_min = DBL_MIN;
      double l_max = DBL_MAX;
      bool if_ignore = false;
      const double l_buffer = ad_byd::planning::math::interp1_inc(
          {0.0, trust_UU_dist}, {0.5, 1.0}, obstacle.ds());

      if (obstacle.type() == OT_UNKNOWN_STATIC) {
        ad_byd::planning::TrajectoryPoint obs_pt;
        obs_pt.set_x(obstacle.x());
        obs_pt.set_y(obstacle.y());
        obs_pt.theta = obstacle.theta();
        path.XYToSL(obstacle.GetCornerPoints(obs_pt), &sl_boundary);
      } else {
        ad_byd::planning::math::Box2d obs_box(
            ad_byd::planning::math::Vec2d(obstacle.x(), obstacle.y()),
            obstacle.theta(), obstacle.length(), obstacle.width());
        path.XYToSL(obs_box, &sl_boundary);
      }
      l_min = sl_boundary.l_min;
      l_max = sl_boundary.l_max;
      if_ignore = l_max < -target_half_lane_width + l_buffer ||
                  l_min > target_half_lane_width - l_buffer ||
                  std::fabs(l_max - l_min) > 30.0;

      if (if_ignore) {
        const double half_width = 1.0;
        // highway::GetConfigManager()->vehicle_config().width() * 0.5;
        if (std::fabs(l_min) < half_width + 0.2 ||
            std::fabs(l_max) < half_width + 0.2) {
          if_ignore = false;
          Log2DDS::LogDataV2(
              "construct_lc_debug",
              absl::StrCat("id", obstacle.id(), " near center line."));
        }
      }

      if (obstacle.ds() < -2.0 || obstacle.ds() > check_pre_dist || if_ignore)
        continue;
      if ((obstacle.type() == OT_UNKNOWN_STATIC || obstacle.type() == OT_CONE ||
           obstacle.type() == OT_BARRIER) &&
          obstacle.ds() < trust_UU_dist) {
        if (current_front_UU_id.empty()) {
          current_front_UU_id = obstacle.id();
        }
        if (obstacle.ds() > 80.0 && obstacle.ds() < trust_UU_dist) {
          if (obstacle.type() == OT_CONE) {
            current_cone_count += 1;
          } else if (obstacle.type() == OT_BARRIER) {
            current_barrel_count += 1;
          } else if (obstacle.type() == OT_UNKNOWN_STATIC) {
            trapezoid_flag = true;
          }
        } else if (obstacle.ds() > 0.0 && obstacle.ds() < 80.0) {
          trapezoid_flag = true;
        }
        Log2DDS::LogDataV2("construct_lc_debug",
                           absl::StrCat("find UU, id:", obstacle.id(),
                                        ", dist:.", obstacle.ds()));
      }
      Log2DDS::LogDataV2(
          "construct_lc_debug",
          absl::StrCat("current_cone_count:", current_cone_count,
                       ",current_barrel_count:", current_barrel_count));

      Log2DDS::LogDataV2(
          "construct_lc_debug",
          absl::StrCat("rectangle_cone_count:", rectangle_cone_count,
                       ",rectangle_barrel_count:", rectangle_barrel_count));
      if (!current_front_UU_id.empty()) {
        if (current_cone_count >= 3 || current_barrel_count >= 2 ||
            current_cone_count + current_barrel_count >= 3) {
          trapezoid_flag = true;
        }
        if (rectangle_barrel_count >= 2 || rectangle_cone_count >= 3 ||
            rectangle_barrel_count + rectangle_cone_count >= 3) {
          rectangle_flag = true;
        }
        if (!trapezoid_flag && !rectangle_flag) {
          current_front_UU_id.clear();
        }
      }
    }
  }
  DLOG(INFO) << "construct_lc_debug" << current_front_UU_id;
  Log2DDS::LogDataV2("construct_lc_debug",
                     "final valid uu:" + current_front_UU_id);

  //
  const double safety_dist = input.plan_start_point->v() * 4.0;
  const auto& obs_itr = std::find_if(obstacles.begin(), obstacles.end(),
                                     [&](const StationaryObstacle& obs) {
                                       return obs.id() == current_front_UU_id;
                                     });
  if (!current_front_UU_id.empty() && obs_itr != obstacles.end() &&
      (*obs_itr).ds() < safety_dist) {
    Log2DDS::LogDataV2("construct_lc_debug", "safety check fail");
    return false;
  }

  return !current_front_UU_id.empty();
}

bool BlockerLcUUTrigger(const BlockerSceneIdentificationInput& blockerinput,
                        double dist_to_junction) {
  // define vector
  const auto& lane_seq = blockerinput.target_lane_seq_blocker;
  const auto& start_point = blockerinput.plan_start_point;
  double start_point_s = 0.0;
  lane_seq->GetProjectionDistance(
      {start_point->path_point().x(), start_point->path_point().y()},
      &start_point_s);
  const auto& stationary_objects = blockerinput.obj_mgr.stationary_objects();
  for (const auto& stall_obj : stationary_objects) {
    Log2DDS::LogDataV2("obstacle_filter",
                       "stationary_objects: " + stall_obj->id());
  }
  std::vector<StationaryObstacle> fsd_obs, obstacles;
  int movable_car_count = 0;
  std::string blocker_id;
  // determin input nullptr or not
  obstacles.reserve(stationary_objects.size());
  fsd_obs.reserve(stationary_objects.size());
  ConvertStObstaclesToFsdObs(stationary_objects, fsd_obs);
  if (!ClassifyObstaclesByPositionAndSetSLBoundary(lane_seq, fsd_obs, obstacles,
                                                   start_point_s)) {
    Log2DDS::LogDataV2("construct_lc_debug",
                       "ClassifyObstaclesByPositionAndSetSLBoundary");
    return false;
  }
  if (!obstacles.empty()) {
    Log2DDS::LogDataV2("obstacle_filter",
                       "stalled objects after set is not nullptr");
    // filter obstacles
    for (const StationaryObstacle& obstacle : obstacles) {
      Log2DDS::LogDataV2("obstacle_filter",
                         "stalled objects after set: " + obstacle.id());
      if (obstacle.ds() < -0.0) {
        continue;
      }
      double pre_dist =
          std::fmax(70.0, std::fmin(100.0, start_point->v() * 5.0));
      if (dist_to_junction > 0.0) {
        pre_dist = std::fmin(dist_to_junction, pre_dist);
      }
      if (obstacle.ds() > pre_dist) {
        break;
      }
      if (dist_to_junction > 0.1) {
        if (obstacle.ds() > dist_to_junction) {
          break;
        }
      }
      SLBoundary sl_boundry = obstacle.sl_boundary();
      // if (obstacle->id() == lc_supplement->last_blocker_id) {
      //   sl_boundry.l_min -= 0.2;
      //   sl_boundry.l_max += 0.2;
      // }
      if (obstacle.type() == OT_UNKNOWN_STATIC) {
        // sl_boundry.l_min -= 0.2;
        // sl_boundry.l_max += 0.2;
        continue;
      }
      Log2DDS::LogDataV2("obstacle_filter",
                         absl::StrCat("stalled objects in: ", obstacle.id(),
                                      ",l_max:", sl_boundry.l_max, ",l_min:",
                                      sl_boundry.l_min, ",ds:", obstacle.ds(),
                                      ",dist_to_junction:", dist_to_junction));
      if (std::fabs(sl_boundry.l_max - sl_boundry.l_min) > 30.0) {
        continue;
      }
      if (sl_boundry.l_max * sl_boundry.l_min > 0.0 &&
          std::fmin(fabs(sl_boundry.l_max), fabs(sl_boundry.l_min)) > 0.4) {
        continue;
      }
      if (obstacle.type() == OT_PEDESTRIAN) {
        continue;
      }
      blocker_id = obstacle.id();
      break;
    }
  }
  return !blocker_id.empty();
}

bool ClassifyObstaclesByPositionAndSetSLBoundary(
    const LaneSequencePtr& lane_seq, std::vector<StationaryObstacle>& fsd_obs,
    std::vector<StationaryObstacle>& current_lane_obstacles,
    const double start_point_s) {
  // clear history
  current_lane_obstacles.clear();

  for (StationaryObstacle& obstacle : fsd_obs) {
    if (!lane_seq) return false;
    LaneConstPtr nearest_lane = lane_seq->GetNearestLane(
        ad_byd::planning::math::Vec2d(obstacle.x(), obstacle.y()));
    if (!nearest_lane) continue;
    const double target_half_lane_width =
        nearest_lane->GetWidthAtPoint(obstacle.x(), obstacle.y()) * 0.5;
    ad_byd::planning::math::Clamp(target_half_lane_width, 1.2, 2.2);

    std::vector<Vec2d> points;
    lane_seq->SamplePoints(0.0, &points);
    ad_byd::planning::Path path(points);
    if (path.points().empty()) {
      return false;
    }
    SLBoundary sl_boundary;
    double l_min = DBL_MIN;
    double l_max = DBL_MAX;
    if (obstacle.type() == OT_UNKNOWN_STATIC) {
      ad_byd::planning::TrajectoryPoint obs_pt;
      obs_pt.set_x(obstacle.x());
      obs_pt.set_y(obstacle.y());
      obs_pt.theta = obstacle.theta();
      path.XYToSL(obstacle.GetCornerPoints(obs_pt), &sl_boundary);
    } else {
      ad_byd::planning::math::Box2d obs_box(
          ad_byd::planning::math::Vec2d(obstacle.x(), obstacle.y()),
          obstacle.theta(), obstacle.length(), obstacle.width());
      path.XYToSL(obs_box, &sl_boundary);
    }
    l_min = sl_boundary.l_min;
    l_max = sl_boundary.l_max;
    // longitudinal distance and position
    double ds = 0.0;
    const double ego_s_min = start_point_s - 1.152;
    // highway::GetConfigManager()->vehicle_config().rear_to_rear_axle();
    const double ego_s_max = start_point_s + 4.070;
    // highway::GetConfigManager()->vehicle_config().front_to_rear_axle();
    if (sl_boundary.s_min > ego_s_max) {
      ds = sl_boundary.s_min - ego_s_max;
    } else if (sl_boundary.s_max < ego_s_min) {
      ds = sl_boundary.s_max - ego_s_min;
    } else {
      ds = 0.0;
    }
    obstacle.mutable_situation().set_lon_distance(ds);
    obstacle.set_sl_boundary(sl_boundary);
    if (fabs(l_max) < target_half_lane_width ||
        fabs(l_min) < target_half_lane_width || l_max * l_min < 0.0) {
      current_lane_obstacles.push_back(obstacle);
    }
  }
  auto comp = [](const StationaryObstacle& p1, const StationaryObstacle& p2) {
    return p1.ds() < p2.ds();
  };
  std::sort(current_lane_obstacles.begin(), current_lane_obstacles.end(), comp);
  return true;
}

void ConvertStObstaclesToFsdObs(
    absl::Span<const PlannerObject* const> stationary_objects,
    std::vector<StationaryObstacle>& fsd_obs) {
  for (const auto& raw_obs : stationary_objects) {
    if (!raw_obs) continue;
    fsd_obs.emplace_back(raw_obs->id(), raw_obs->type(), raw_obs->contour(),
                         raw_obs->pose().pos(), raw_obs->pose().theta(),
                         raw_obs->bounding_box().length(),
                         raw_obs->bounding_box().width());
  }
}

double GetLeaderCarSOffset(const LaneSequencePtr& lane_seq,
                           const ObjectVector<PlannerObject>& obstacles,
                           const double& start_offset) {
  double s_offset = std::numeric_limits<double>::max();
  if (!lane_seq) {
    return s_offset;
  }

  std::vector<Vec2d> points;
  lane_seq->SamplePoints(0.0, &points);
  ad_byd::planning::Path path(points);
  if (path.points().empty()) {
    return s_offset;
  }

  for (const auto& raw_obs : obstacles) {
    // type check
    if (raw_obs.type() == OT_UNKNOWN_STATIC || raw_obs.type() == OT_CONE ||
        raw_obs.type() == OT_BARRIER) {
      continue;
    }
    StationaryObstacle obs{raw_obs.id(),
                           raw_obs.type(),
                           raw_obs.contour(),
                           raw_obs.pose().pos(),
                           raw_obs.pose().theta(),
                           raw_obs.bounding_box().length(),
                           raw_obs.bounding_box().width()};

    LaneConstPtr nearest_lane = lane_seq->GetNearestLane(obs.pos());
    if (!nearest_lane) continue;
    double target_half_lane_width =
        nearest_lane->GetWidthAtPoint(obs.pos().x(), obs.pos().y()) * 0.5;
    target_half_lane_width =
        ad_byd::planning::math::Clamp(target_half_lane_width, 1.2, 2.2);
    SLBoundary sl_boundary;
    ad_byd::planning::TrajectoryPoint obs_pt;
    obs_pt.set_x(obs.pos().x());
    obs_pt.set_y(obs.pos().y());
    obs_pt.theta = obs.theta();
    path.XYToSL(obs.GetCornerPoints(obs_pt), &sl_boundary);

    // lane check
    bool is_current_obs = false;
    if (fabs(sl_boundary.l_max) < target_half_lane_width ||
        fabs(sl_boundary.l_min) < target_half_lane_width ||
        sl_boundary.l_max * sl_boundary.l_min < 0.0) {
      is_current_obs = true;
    }
    if (!is_current_obs) continue;

    // s check
    double ds = 0.0;
    const double ego_s_min = start_offset - 1.152;
    // highway::GetConfigManager()->vehicle_config().rear_to_rear_axle();
    const double ego_s_max = start_offset + 4.070;
    // highway::GetConfigManager()->vehicle_config().front_to_rear_axle();
    if (sl_boundary.s_min > ego_s_max) {
      ds = sl_boundary.s_min - ego_s_max;
    } else if (sl_boundary.s_max < ego_s_min) {
      ds = sl_boundary.s_max - ego_s_min;
    } else {
      ds = 0.0;
    }
    if (ds < 0.0) continue;
    if (ds < s_offset) {
      s_offset = ds;
    }
  }
  return s_offset;
}

}  // namespace st::planning