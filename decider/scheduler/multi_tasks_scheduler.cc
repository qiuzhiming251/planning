

#include "decider/scheduler/multi_tasks_scheduler.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <limits>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "plan_common/async/parallel_for.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/log.h"
#include "object_manager/object_history.h"
#include "plan_common/planning_macros.h"
#include "plan_common/timer.h"
// #include "global/trace.h"
// #include "lite/logging.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/log_data.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/plan_common_defs.h"
// #include "planner/planner_manager/planner_flags.h"
#include "router/drive_passage_builder.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "router/route_sections_util.h"
#include "decider/scheduler/path_boundary_builder.h"
#include "decider/scheduler/scheduler_util.h"
#include "plan_common/util/format_numeric_string.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st::planning {

namespace {

using V2RoadType = ad_byd::planning::V2RoadClass::V2RoadClassType;
constexpr double kEpsilon = 0.1;             // m.
constexpr double kForceMergeMaxSpeed = 2.0;  // m/s.

absl::StatusOr<LanePathInfo> FindNeighbor(
    const PlannerSemanticMapManager& psmm,
    const std::vector<LanePathInfo>& lp_infos, bool lc_left,
    mapping::ElementId start_id) {
  const auto& lane = psmm.map_ptr()->GetLaneById(start_id);
  if (!lane) {
    return absl::NotFoundError("No match lane.");
  }
  const auto neighbor_id =
      !lc_left ? lane->left_lane_id() : lane->right_lane_id();
  for (const auto& lp_info : lp_infos) {
    if (lp_info.start_lane_id() == neighbor_id) return lp_info;
  }
  return absl::NotFoundError("Neighbor lane not viable.");
}

absl::StatusOr<SchedulerOutput> MakeLcPauseSchedulerOutput(
    const PlannerSemanticMapManager& psmm, SchedulerOutput scheduler_output,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const mapping::LanePath& prev_lane_path_before_lc_from_start) {
  scheduler_output.lane_change_state.set_stage(LaneChangeStage::LCS_PAUSE);

  ASSIGN_OR_RETURN(
      scheduler_output.sl_boundary,
      BuildPathBoundaryFromPose(
          psmm, scheduler_output.drive_passage, plan_start_point, vehicle_geom,
          st_traj_mgr, scheduler_output.lane_change_state, smooth_result_map,
          /*borrow_lane_boundary=*/false, scheduler_output.should_smooth,
          prev_lane_path_before_lc_from_start),
      _ << "Fail to build path boundary.");

  return scheduler_output;
}

// mapping::ElementId FindWaitingZoneAhead(const TrafficLightInfoMap&
// tl_info_map,
//                                         const mapping::LanePath& lane_path) {
//   for (const auto& lane_id : lane_path.lane_ids()) {
//     if (tl_info_map.count(lane_id) &&
//         tl_info_map.at(lane_id).tl_control_type() ==
//             TrafficLightControlType::LEFT_WAITING_AREA) {
//       return lane_id;
//     }
//   }
//   return mapping::kInvalidElementId;
// }

// std::pair<int, int> CalculateLeftAndRightLcNum(
//     const mapping::ElementId& lane_id, const PlannerSemanticMapManager& psmm,
//     const RouteNaviInfo& route_navi_info) {
//   int left_lc_num = std::numeric_limits<int>::max();
//   int right_lc_num = std::numeric_limits<int>::max();
//   SMM_ASSIGN_LANE_OR_RETURN(lane_info, psmm, lane_id,
//                             std::make_pair(left_lc_num, right_lc_num));

//   if (!lane_info.left_lane_id().empty()) {
//     const auto* left_lane_navi_info_ptr = FindOrNull(
//         route_navi_info.route_lane_info_map, lane_info.left_lane_id());
//     if (left_lane_navi_info_ptr != nullptr) {
//       left_lc_num = left_lane_navi_info_ptr->min_lc_num_to_target;
//     }
//   }
//   if (!lane_info.right_lane_id().empty()) {
//     const auto* right_lane_navi_info_ptr = FindOrNull(
//         route_navi_info.route_lane_info_map, lane_info.right_lane_id());
//     if (right_lane_navi_info_ptr != nullptr) {
//       right_lc_num = right_lane_navi_info_ptr->min_lc_num_to_target;
//     }
//   }
//   return std::make_pair(left_lc_num, right_lc_num);
// }

// bool CheckNeedSwitchRoute(const PlannerSemanticMapManager& psmm,
//                           const LaneChangeStateProto& lane_change_state,
//                           const RouteNaviInfo& route_navi_info,
//                           const DrivePassage& drive_passage) {
//   const auto lane_id = drive_passage.lane_path().front().lane_id();
//   const auto* lane_navi_info_ptr =
//       FindOrNull(route_navi_info.route_lane_info_map, lane_id);
//   if (lane_navi_info_ptr == nullptr) return false;
//   if (lane_navi_info_ptr->min_lc_num_to_target == 0 ||
//       lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING ||
//       lane_change_state.stage() == LaneChangeStage::LCS_RETURN) {
//     return false;
//   }
//   constexpr double kMinLengthToSwitchAlterRoute = 2.0;  // m.

//   if (lane_navi_info_ptr->max_reach_length > kMinLengthToSwitchAlterRoute) {
//     return false;
//   }

//   const auto boundaries = drive_passage.QueryEnclosingLaneBoundariesAtS(0.0);
//   const auto [left_lc_num, right_lc_num] =
//       CalculateLeftAndRightLcNum(lane_id, psmm, route_navi_info);
//   if (left_lc_num < right_lc_num) {
//     return boundaries.left.has_value() && boundaries.left->IsSolid(0.0);
//   } else if (right_lc_num < left_lc_num) {
//     return boundaries.right.has_value() && boundaries.right->IsSolid(0.0);
//   }
//   return false;
// }

// std::pair<TurnSignal, TurnSignalReason> DecideRoutePrepareLcTurnSignal(
//     const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
//     const RouteNaviInfo& route_navi_info) {
//   TurnSignal planner_turn_signal = TURN_SIGNAL_NONE;
//   TurnSignalReason turn_signal_reason = TURN_SIGNAL_OFF;
//   constexpr double kTurnOnSignalPreviewDist = 300.0;  // m.
//   const auto lane_id = drive_passage.lane_path().front().lane_id();
//   const auto* lane_navi_info_ptr =
//       FindOrNull(route_navi_info.route_lane_info_map, lane_id);
//   if (lane_navi_info_ptr == nullptr ||
//       lane_navi_info_ptr->min_lc_num_to_target == 0 ||
//       lane_navi_info_ptr->max_reach_length > kTurnOnSignalPreviewDist) {
//     return std::make_pair(planner_turn_signal, turn_signal_reason);
//   }
//   const auto [left_lc_num, right_lc_num] =
//       CalculateLeftAndRightLcNum(lane_id, psmm, route_navi_info);
//   const auto curr_lc_num = lane_navi_info_ptr->min_lc_num_to_target;
//   planner_turn_signal =
//       curr_lc_num > left_lc_num
//           ? TURN_SIGNAL_LEFT
//           : (curr_lc_num > right_lc_num ? TURN_SIGNAL_RIGHT :
//           TURN_SIGNAL_NONE);
//   turn_signal_reason = planner_turn_signal == TURN_SIGNAL_NONE
//                            ? TURN_SIGNAL_OFF
//                            : PREPARE_LANE_CHANGE_TURN_SIGNAL;
//   return std::make_pair(planner_turn_signal, turn_signal_reason);
// }

bool IsLaneBlockedByObs(const DrivePassage& passage, const FrenetBox& box,
                        const double lat_buffer) {
  const auto [right_boundary, left_boundary] =
      passage.QueryEnclosingLaneBoundariesAtS(box.center_s());
  const double boundary_right_l =
      right_boundary.has_value()
          ? std::max(right_boundary->lat_offset, -kMaxHalfLaneWidth)
          : -kMaxHalfLaneWidth;
  const double boundary_left_l =
      left_boundary.has_value()
          ? std::min(left_boundary->lat_offset, kMaxHalfLaneWidth)
          : kMaxHalfLaneWidth;
  const double l_max = std::clamp(box.l_max, boundary_right_l, boundary_left_l);
  const double l_min = std::clamp(box.l_min, boundary_right_l, boundary_left_l);

  if (std::min(boundary_left_l - l_min, l_max - boundary_right_l) <
      lat_buffer) {
    return false;
  }
  return true;
}

bool IsValidLaneAttrType(const LaneAttrType& lane_attr_type) {
  return lane_attr_type == LaneAttrType::LANEATTR_SELF ||
         lane_attr_type == LaneAttrType::LANEATTR_ON_LINE;
}

bool IsObjectOverlapLine(double obj_l_min, double obj_l_max,
                         double boundary_line_offset, double left_buffer,
                         double right_buffer) {
  return obj_l_max > boundary_line_offset - right_buffer &&
         obj_l_min < boundary_line_offset + left_buffer;
}

bool IsObjectBlockingCenterLine(double obj_l_min, double obj_l_max,
                                double center_l, double lat_buffer) {
  double too_near_with_center_l_thre = 0.6;
  bool totally_in_center_zone =
      obj_l_max < center_l + lat_buffer && obj_l_min > center_l - lat_buffer;
  bool too_near_with_center_l =
      (obj_l_min > 0.0 && obj_l_min < too_near_with_center_l_thre) ||
      (obj_l_max < 0.0 && std::fabs(obj_l_max) < too_near_with_center_l_thre);
  bool cross_center_l = obj_l_min * obj_l_max < 0.0;
  return totally_in_center_zone || too_near_with_center_l || cross_center_l;
}

std::string BlockLocationInfoToString(
    const BlockLocation& block_location_type, const int& center_cone_num,
    const int& center_barrier_num, const int& center_static_obs_num,
    const double& center_l_occupy_max, const double& center_l_occupy_min,
    const int& right_neighbor_obstacles_num,
    const int& left_neighbor_obstacles_num,
    const int& lane_attr_neighbor_obstacle_num) {
  switch (block_location_type) {
    case BlockLocation::BLOCK_CENTER:
      return absl::StrCat("--block_center, cone_num = ", center_cone_num,
                          ", center_barrier_num = ", center_barrier_num,
                          ", center_static_obs_num = ", center_static_obs_num,
                          ", center_l_occupy_max =  ", center_l_occupy_max,
                          ", center_l_occupy_min = ", center_l_occupy_min,
                          "\n");
    case BlockLocation::BLOCK_LEFT:
      return absl::StrCat("--block_left , num = ", left_neighbor_obstacles_num,
                          "\n");
    case BlockLocation::BLOCK_RIGHT:
      return absl::StrCat("--block_right, num = ", right_neighbor_obstacles_num,
                          "\n");
    case BlockLocation::BLOCK_LANE_ATTR:
      return absl::StrCat(
          "--block_lane_attr, num = ", lane_attr_neighbor_obstacle_num, "\n");
    default:
      return "unknown";
  }
}

std::string ObjectTypeToString(const ObjectType& obj_type) {
  switch (obj_type) {
    case ObjectType::OT_CONE:
      return "cone";
    case ObjectType::OT_BARRIER:
      return "barrier";
    case ObjectType::OT_WARNING_TRIANGLE:
      return "waring_triangle";
    case ObjectType::OT_UNKNOWN_STATIC:
      return "Gen_obj_unknown_static";
    default:
      return "unknown";
  }
}

bool DecideCheckWhichNeighbor(
    const std::set<Obstacle, MyComparator>& left_neighbor_obstacles,
    const std::set<Obstacle, MyComparator>& right_neighbor_obstacles,
    std::set<st::planning::Obstacle, MyComparator>& neighbor_obstacles,
    bool& check_left, NeighborObsInfo& neighbor_obstacle_info) {
  auto need_check_left_have_curb = !left_neighbor_obstacles.empty();
  auto need_check_right_have_curb = !right_neighbor_obstacles.empty();

  auto left_obs_s_max = need_check_left_have_curb
                            ? left_neighbor_obstacles.rbegin()->frenet_box.s_max
                            : 0.0;
  auto right_obs_s_max =
      need_check_right_have_curb
          ? right_neighbor_obstacles.rbegin()->frenet_box.s_max
          : 0.0;

  auto left_obs_s_min = need_check_left_have_curb
                            ? left_neighbor_obstacles.begin()->frenet_box.s_min
                            : 1000.0;
  auto right_obs_s_min =
      need_check_right_have_curb
          ? right_neighbor_obstacles.begin()->frenet_box.s_min
          : 1000.0;

  auto left_obs_s_max_id =
      need_check_left_have_curb ? left_neighbor_obstacles.rbegin()->id : "none";
  auto right_obs_s_max_id = need_check_right_have_curb
                                ? right_neighbor_obstacles.rbegin()->id
                                : "none";

  if (need_check_left_have_curb && need_check_right_have_curb) {
    if (left_obs_s_max < right_obs_s_max) {
      neighbor_obstacle_info.s_max = left_obs_s_max;
      neighbor_obstacle_info.s_min = left_obs_s_min;
      neighbor_obstacle_info.id = left_obs_s_max_id;
      neighbor_obstacles = left_neighbor_obstacles;
      check_left = true;
    } else {
      neighbor_obstacle_info.s_max = right_obs_s_max;
      neighbor_obstacle_info.s_min = right_obs_s_min;
      neighbor_obstacle_info.id = right_obs_s_max_id;
      neighbor_obstacles = right_neighbor_obstacles;
      check_left = false;
    }

    return true;
  } else if (need_check_left_have_curb && !need_check_right_have_curb) {
    neighbor_obstacle_info.s_max = left_obs_s_max;
    neighbor_obstacle_info.s_min = left_obs_s_min;
    neighbor_obstacle_info.id = left_obs_s_max_id;
    neighbor_obstacles = left_neighbor_obstacles;
    check_left = true;
    return true;
  } else if (!need_check_left_have_curb && need_check_right_have_curb) {
    neighbor_obstacle_info.s_max = right_obs_s_max;
    neighbor_obstacle_info.s_min = right_obs_s_min;
    neighbor_obstacle_info.id = right_obs_s_max_id;
    neighbor_obstacles = right_neighbor_obstacles;
    check_left = false;
    return true;
  } else {
    return false;
  }
}

void SetStopLine(const int& task_idx, DrivePassage& drive_passage,
                 const double& final_stop_s,
                 const std::string& block_info_string) {
  std::vector<Vec2d> stop_points;
  auto final_left_point = drive_passage.QueryPointXYAtSL(final_stop_s, 1.5);
  auto final_right_point = drive_passage.QueryPointXYAtSL(final_stop_s, -1.5);

  if (final_left_point.ok() && final_right_point.ok()) {
    stop_points.push_back(final_left_point.value());
    stop_points.push_back(final_right_point.value());
  }
  Log2DDS::LogLineV2(absl::StrCat(Log2DDS::TaskPrefix(task_idx), "stop-",
                                  block_info_string, "-", final_stop_s),
                     Log2DDS::kRed, {}, stop_points);
}

void CheckCurb(DrivePassage& drive_passage, const double& check_s_start,
               const bool& check_center, const bool& check_left,
               int& have_curb_cnt, bool& obs_near_junction,
               std::string& debug_string) {
  constexpr double kBoundaryWidthThre{3.0};
  double check_s_end = check_s_start + 20.0;

  for (double s_step = check_s_start; s_step < check_s_end; s_step += 1.0) {
    auto station_idx = drive_passage.FindNearestStationIndexAtS(s_step);
    const auto& step_station = drive_passage.station(station_idx);
    if (step_station.is_in_intersection()) {
      obs_near_junction = true;
      break;
    }
    const auto [right_boundary, left_boundary] =
        drive_passage.QueryEnclosingLaneBoundariesAtS(s_step);
    double lane_width = left_boundary->lat_offset - right_boundary->lat_offset;
    if (check_center) {
      if ((right_boundary->type == StationBoundaryType::CURB ||
           left_boundary->type == StationBoundaryType::CURB) &&
          lane_width < kBoundaryWidthThre) {
        have_curb_cnt++;
      }
    } else {
      auto tmp_boundary_type =
          check_left ? left_boundary->type : right_boundary->type;
      if (tmp_boundary_type == StationBoundaryType::CURB &&
          lane_width < kBoundaryWidthThre) {
        have_curb_cnt++;
      }
    }
    // auto str = absl::StrCat("left_boundary = ", left_boundary->lat_offset , "
    // , right_boundary = ", right_boundary->lat_offset, " l_bound_type = " ,
    // left_boundary->type, " right_bound_type = ",  right_boundary->type,  ",
    // step_s_width = ", lane_width, ", check_center = " , check_center , ",
    // check_left = " , check_left,  ", have_curb_count = ", have_curb_cnt);
    // absl::StrAppend(&debug_string, str, "\n");
  }
}

bool CheckNeighborObstacleBlockTrend(
    std::set<st::planning::Obstacle, MyComparator> neighbor_obstacles,
    std::string& debug_string) {
  if (neighbor_obstacles.empty()) {
    return false;
  }

  std::vector<st::planning::Obstacle> candidate_obs;
  double first_obs_l_min =
      std::fabs(neighbor_obstacles.begin()->frenet_box.l_min);
  double first_obs_s_min = neighbor_obstacles.begin()->frenet_box.s_min;
  std::string tmp_str = absl::StrCat("first_obs_s_min: ", first_obs_s_min,
                                     ", candidate_obs_l_min : ");

  if (first_obs_s_min < 55.0) {
    return false;
  } else {
    for (const auto& element : neighbor_obstacles) {
      if ((element.frenet_box.s_min - first_obs_s_min < 30.0) ||
          candidate_obs.size() < 3) {
        candidate_obs.push_back(element);
        absl::StrAppend(&tmp_str, std::fabs(element.frenet_box.l_min), ",");
      }
    }

    std::vector<st::planning::Obstacle> trend_obs;
    for (auto element : candidate_obs) {
      if (trend_obs.empty()) {
        trend_obs.push_back(element);
      } else {
        if (std::fabs(trend_obs.back().frenet_box.l_min) >
            std::fabs(element.frenet_box.l_min)) {
          trend_obs.push_back(element);
        }
      }
    }

    absl::StrAppend(&tmp_str, "\n", "trend_res : ");
    for (auto element : trend_obs) {
      absl::StrAppend(&tmp_str, std::fabs(element.frenet_box.l_min), "|",
                      element.lane_attr_type, ",");
    }

    double prob = 0.0;
    bool last_trend_obs_lane_attr_self{false};
    if (trend_obs.size() >= 2 && candidate_obs.size() >= 3) {
      prob = static_cast<double>(trend_obs.size()) / candidate_obs.size();
      last_trend_obs_lane_attr_self =
          (trend_obs.back().lane_attr_type == LaneAttrType::LANEATTR_SELF);
      absl::StrAppend(&tmp_str, ", trend_vec.size() = ", trend_obs.size(),
                      " , l_min_vec.size() = ", candidate_obs.size(),
                      " prob = ", prob);
    }
    absl::StrAppend(&debug_string, tmp_str);

    if (prob >= 0.4 && last_trend_obs_lane_attr_self) {
      return true;
    } else {
      return false;
    }
  }
}

void SetStopInfo(const int& task_idx, DrivePassage& drive_passage,
                 const double& final_stop_s, const bool& enable_stop,
                 const bool& enable_slow_down, const BlockReason& block_reason,
                 const bool& have_avliable_zone,
                 const bool& is_emergence_lane_scene,
                 const std::string& block_info_string,
                 const std::string& obs_id) {
  double stop_s = final_stop_s;
  if (!is_emergence_lane_scene) {
    if (block_reason == BlockReason::CURB_CROSS) {
      stop_s = stop_s - FLAGS_planner_curb_cutoff_buffer;
    } else if (block_reason != BlockReason::NONE) {
      stop_s = stop_s - FLAGS_planner_obstacle_cutoff_buffer;
    }
    SetStopLine(task_idx, drive_passage, stop_s, block_info_string);
  }
  drive_passage.SetTrafficStaticObstaclesInfo(
      enable_stop, enable_slow_down, block_reason, final_stop_s,
      have_avliable_zone, obs_id,
      is_emergence_lane_scene);  // enable slow down or set stop_line
}

bool CheckLaneAttr(const LaneChangeStateProto& lc_state,
                   const st::Behavior_FunctionId& function_id,
                   const double& obj_s_min, const LaneAttrType& lane_attr_type,
                   const bool& check_center) {
  constexpr double kCredibleLengthCityFirst{50.0},
      kCredibleLengthCitySecond{80.0};
  constexpr double kCredibleLengthHighWay{80.0};
  bool is_in_city = function_id == Behavior_FunctionId_CITY_NOA;
  bool is_in_highway = function_id == Behavior_FunctionId_HW_NOA;

  // return true is meaning that we believe the obj_pose
  if (lc_state.stage() != LaneChangeStage::LCS_NONE) {
    return true;
  } else if (is_in_city || is_in_highway) {
    double kCredibleLengthFirst{50.0};
    if (is_in_city) {
      kCredibleLengthFirst = kCredibleLengthCityFirst;
      if (obj_s_min > kCredibleLengthCitySecond) {
        return false;
      }
    } else {
      kCredibleLengthFirst = kCredibleLengthHighWay;
    }

    if (obj_s_min > kCredibleLengthFirst) {
      if (check_center) {
        if (lane_attr_type == LaneAttrType::LANEATTR_SELF ||
            lane_attr_type == LaneAttrType::LANEATTR_UNKNOWN) {
          return true;
        } else {
          return false;
        }
      } else {
        if (lane_attr_type == LaneAttrType::LANEATTR_SELF ||
            lane_attr_type == LaneAttrType::LANEATTR_ON_LINE ||
            lane_attr_type == LaneAttrType::LANEATTR_UNKNOWN) {
          return true;
        } else {
          return false;
        }
      }
    } else {
      return true;
    }

  } else {
    return true;
  }
}

void SetStopTypeAccordingSafeZone(
    const double& final_stop_s, const double& ego_v, const bool& city_going,
    const bool& lka_going,
    const LaneChangeSafetyInfo* pre_lane_change_safety_info,
    const ad_byd::planning::BehaviorCommand intention_dir,
    const bool& have_avliable_zone, const bool& is_emergence_lane_scene,
    const BlockReason& block_reason, bool& enable_stop, bool& enable_slow_down,
    std::string& stop_type) {
  if (lka_going && block_reason != BlockReason::OBS_ROW_OBSTACLES) {
    enable_stop = true;
    enable_slow_down = false;
    stop_type = "lka_stop";
    return;
  }

  constexpr int KBlockCntThre{5};
  if (pre_lane_change_safety_info == nullptr) {
    return;
  } else {
    if (pre_lane_change_safety_info->ego_lane_block_by_obs_cnt() >=
            KBlockCntThre &&
        intention_dir == ad_byd::planning::BehaviorCommand::Command_Invalid &&
        !is_emergence_lane_scene) {
      if (block_reason != BlockReason::OBS_ROW_OBSTACLES) {
        enable_stop = true;
        enable_slow_down = false;
        stop_type = "no_select_stop";
      } else {
        enable_stop = false;
        enable_slow_down = true;
        stop_type = "no_select_obs_slow_down";
      }
    } else {
      if (is_emergence_lane_scene) {
        enable_stop = false;
        enable_slow_down = false;
        stop_type = "emergence_lane_none";
      } else {
        if (have_avliable_zone) {
          enable_stop = false;
          enable_slow_down = true;
          stop_type = "avi_slow_down";
        } else {
          if (block_reason != BlockReason::OBS_ROW_OBSTACLES) {
            enable_stop = true;
            enable_slow_down = false;
            stop_type = "non_avi_stop";
          } else {
            enable_stop = false;
            enable_slow_down = true;
            stop_type = "non_avi_row_obs_slow_down";
          }
        }
      }
    }
  }
}

bool CheckHaveAvliableZone(
    const DrivePassage& drive_passage, const LaneChangeStateProto& lc_state,
    const FrenetBox& ego_frenet_box,
    const std::set<Obstacle, MyComparator>& neighbor_obstacles,
    const bool& check_left,
    const std::set<Obstacle, MyComparator>& center_obstacle,
    const PlannerSemanticMapManager& psmm,
    const LaneChangeSafetyInfo* pre_lane_change_safety_info,
    bool& is_emergence_lane_scene, bool& pre_lane_change_left_is_avliable,
    bool& pre_lane_change_right_is_avliable, std::string& debug_string) {
  if (pre_lane_change_safety_info == nullptr) {
    absl::StrAppend(&debug_string, "pre_lane_change_safety_info == nullptr");
    return true;
  }

  pre_lane_change_left_is_avliable =
      pre_lane_change_safety_info->have_check_lc_left_safe() &&
      pre_lane_change_safety_info->lc_left_safe();
  pre_lane_change_right_is_avliable =
      pre_lane_change_safety_info->have_check_lc_right_safe() &&
      pre_lane_change_safety_info->lc_right_safe();

  bool need_check_left{false};
  bool need_check_right{false};
  bool need_both_check{false};

  bool ego_box_is_right = ego_frenet_box.l_max < 0.0;
  bool ego_box_is_left = ego_frenet_box.l_min > 0.0;

  absl::StrAppend(
      &debug_string,
      "pre_lc_left_is_avliable = ", pre_lane_change_left_is_avliable,
      " pre_lc_right_is_avliable = ", pre_lane_change_right_is_avliable,
      " ego_frenet_box.l_max = ", ego_frenet_box.l_max,
      " ego_frenet_box.l_min = ", ego_frenet_box.l_min,
      " neighbor_obstacles.size = ", neighbor_obstacles.size(),
      " center_obstacle.size = ", center_obstacle.size());

  // step0 : if no obstacles , return true
  if (neighbor_obstacles.empty() && center_obstacle.empty()) {
    return true;
  }

  // step1 : check whether ego_frenet_box is totally left or right
  if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
    if ((ego_box_is_left || ego_box_is_right) &&
        (!neighbor_obstacles.empty() || !center_obstacle.empty())) {
      if (ego_box_is_left) {
        need_check_left = true;
        need_check_right = false;
      } else {
        need_check_left = false;
        need_check_right = true;
      }
    } else {
      // step2 : both neighbor_obstacles and center_obstacle not empty
      if (neighbor_obstacles.size() >= 2 && center_obstacle.size() >= 2) {
        // trend check
        const double neighbor_obs_s_min =
            neighbor_obstacles.begin()->frenet_box.s_min;
        const double center_obs_s_min =
            center_obstacle.begin()->frenet_box.s_min;
        const double neighbor_obs_l_min =
            neighbor_obstacles.begin()->frenet_box.l_min;
        const double center_obs_l_min =
            center_obstacle.begin()->frenet_box.l_min;
        absl::StrAppend(&debug_string,
                        ", neighbor_obs_s_min = ", neighbor_obs_s_min,
                        ", center_obs_s_min = ", center_obs_s_min,
                        ", neighbor_obs_l_min = ", neighbor_obs_l_min,
                        ", center_obs_l_min", ", check_left = ", check_left);
        if (center_obs_s_min > neighbor_obs_s_min) {
          if (check_left) {
            need_check_left = false;
            need_check_right = true;
          } else {
            need_check_left = true;
            need_check_right = false;
          }
        } else {
          if (neighbor_obs_l_min < center_obs_l_min) {
            need_check_left = false;
            need_check_right = true;
          } else {
            need_check_left = false;
            need_check_right = true;
          }
        }

        // bool choose_neighbor = (neighbor_obs_s_min < center_obs_s_min) : true
        // : false;

        // const double obs_l_min = choose_neighbor ?
        // neighbor_obstacles.begin()->frenet_box.l_min :
        // center_obstacle.begin()->frenet_box.l_min; const double obs_l_max =
        // choose_neighbor ? neighbor_obstacles.begin()->frenet_box.l_max :
        // center_obstacle.begin()->frenet_box.l_max;

        // if (obs_l_min > ego_frenet_box.l_max) {
        //   need_check_left = false;
        //   need_check_right = true;
        // } else if (obs_l_max < ego_frenet_box.l_min) {
        //   need_check_left = true;
        //   need_check_right = false;
        // }
      } else if (neighbor_obstacles.size() >= 2) {
        // step3 : neighbor_obstacles is not empty
        // const double obs_l_min =
        // neighbor_obstacles.begin()->frenet_box.l_min; const double obs_l_max
        // = neighbor_obstacles.begin()->frenet_box.l_max; if (obs_l_min >
        // ego_frenet_box.l_max) {
        //   need_check_left = false;
        //   need_check_right = true;
        // } else if (obs_l_max < ego_frenet_box.l_min) {
        //   need_check_left = true;
        //   need_check_right = false;
        // }
        absl::StrAppend(&debug_string, ", chec_left = ", check_left);
        if (check_left) {
          need_check_left = false;
          need_check_right = true;
        } else {
          need_check_left = true;
          need_check_right = false;
        }
      } else if (!center_obstacle.empty()) {
        // step3 : both check
        need_both_check = true;
        absl::StrAppend(&debug_string, ", need_both_check = ", need_both_check);
      }
    }

    absl::StrAppend(&debug_string, ", need_check_left = ", need_check_left,
                    ", need_check_right = ", need_check_right,
                    ", need_both_check = ", need_both_check);
    if (!need_both_check) {
      if (need_check_left || need_check_right) {
        // step4 : check pre_lane_change_safety is avliable
        bool is_safety = need_check_left ? pre_lane_change_left_is_avliable
                                         : pre_lane_change_right_is_avliable;
        if (is_safety) {
          absl::StrAppend(&debug_string, ", is_safety = ", is_safety);
          return true;
        } else {
          if (drive_passage.lane_path().IsEmpty()) {
            absl::StrAppend(&debug_string, "lp is empty");
            return true;
          } else {
            const auto front_lane_id =
                drive_passage.lane_path().front().lane_id();

            const auto curr_lane = psmm.FindLaneByIdOrNull(front_lane_id);
            if (curr_lane) {
              if (curr_lane->type() == LaneType::LANE_NON_MOTOR ||
                  curr_lane->type() == LaneType::LANE_EMERGENCY ||
                  curr_lane->type() == LaneType::LANE_UNKNOWN) {
                is_emergence_lane_scene = true;
                absl::StrAppend(&debug_string, ", is_emergence_lane_scene");
                return true;
              } else {
                const auto check_lane_id = need_check_left
                                               ? curr_lane->left_lane_id()
                                               : curr_lane->right_lane_id();
                const auto check_lane = psmm.FindLaneByIdOrNull(check_lane_id);
                if (check_lane) {
                  if (check_lane->type() == LaneType::LANE_NON_MOTOR ||
                      check_lane->type() == LaneType::LANE_EMERGENCY ||
                      check_lane->type() == LaneType::LANE_UNKNOWN) {
                    absl::StrAppend(&debug_string,
                                    ", check_lane is_emergence_lane_scene");
                    is_emergence_lane_scene = true;
                    return true;
                  } else {
                    absl::StrAppend(&debug_string,
                                    ", check_lane isn't emergence_lane");
                    return false;
                  }
                } else {
                  absl::StrAppend(&debug_string, ", no check_lane");
                  return false;
                }
              }
            } else {
              absl::StrAppend(&debug_string, ", no curr_lane");
              return false;
            }
          }
        }
      } else {
        absl::StrAppend(&debug_string, ", no check");
        return true;
      }
    } else {
      if (pre_lane_change_left_is_avliable ||
          pre_lane_change_right_is_avliable) {
        absl::StrAppend(&debug_string, ", both check, ok");
        return true;
      } else {
        absl::StrAppend(&debug_string, ", both check, fail");
        return false;
      }
    }
  } else {
    absl::StrAppend(&debug_string, ", lc_stage not none");
    return true;
  }
}

void CaculateTotalConstructionObsNum(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const LaneChangeStateProto& lc_state, const bool& highway_road,
    const bool& is_cone, const bool& is_barrier,
    const LaneAttrType& obj_lane_attr, const bool& pre_is_construction_scene,
    const bool& pre_construction_is_left, const Obstacle& obs,
    int& total_cone_num, int& total_barrier_num,
    std::set<st::planning::Obstacle, MyComparator>&
        total_pre_deceleration_obstacles,
    std::string& debug_str) {
  const double KLaneAttrLeftRightDistThre{80.0};
  bool is_right_emergency_lane{false};

  const auto curb_l = drive_passage.QueryCurbOffsetAtS(obs.frenet_box.s_min);

  if (lc_state.stage() == LaneChangeStage::LCS_NONE ||
      lc_state.stage() == LaneChangeStage::LCS_RETURN) {
    if (drive_passage.lane_path().IsEmpty()) {
      absl::StrAppend(&debug_str, "lp is empty");
      is_right_emergency_lane = false;
    } else {
      const auto front_lane_id = drive_passage.lane_path().front().lane_id();
      const auto curr_lane = psmm.FindLaneByIdOrNull(front_lane_id);
      if (curr_lane) {
        const auto right_lane_id = curr_lane->right_lane_id();
        const auto right_lane = psmm.FindLaneByIdOrNull(right_lane_id);
        if (right_lane) {
          if (right_lane->type() == LaneType::LANE_NON_MOTOR ||
              right_lane->type() == LaneType::LANE_EMERGENCY ||
              right_lane->type() == LaneType::LANE_UNKNOWN) {
            is_right_emergency_lane = true;
          }
        }
      } else {
        is_right_emergency_lane = false;
      }
    }
  }

  bool ignore_emergence_lane_curb_obs = false;
  double right_l{0.0};
  if (curb_l.ok()) {
    right_l = curb_l->first;
    ignore_emergence_lane_curb_obs =
        (is_right_emergency_lane) &&
        (obj_lane_attr == LaneAttrType::LANEATTR_RIGHT) &&
        (obs.frenet_box.l_max < curb_l->first + 0.8);
  } else {
    absl::StrAppend(&debug_str, "id: ", obs.id, "curb fail");
  }

  if (ignore_emergence_lane_curb_obs) {
    absl::StrAppend(&debug_str, "id: ", obs.id, "-attr: ", obj_lane_attr,
                    "-l_max: ", obs.frenet_box.l_max, "-r_curb: ", right_l,
                    "-s_min: ", obs.frenet_box.s_min, ", ");
  }

  if (highway_road && (is_cone || is_barrier)) {
    if (pre_is_construction_scene) {
      bool is_left = obs.frenet_box.l_max > 0.0;
      bool is_same_location = (is_left == pre_construction_is_left);
      if (!is_same_location) {
        return;
      } else {
        if (is_cone) {
          total_cone_num++;
          total_pre_deceleration_obstacles.insert(obs);
        } else if (is_barrier) {
          total_barrier_num++;
          total_pre_deceleration_obstacles.insert(obs);
        }
      }
    } else if (obj_lane_attr != LaneAttrType::LANEATTR_OTHER) {
      if ((obj_lane_attr == LaneAttrType::LANEATTR_RIGHT ||
           obj_lane_attr == LaneAttrType::LANEATTR_LEFT) &&
          obs.frenet_box.s_min > KLaneAttrLeftRightDistThre) {
        return;
      } else if (lc_state.stage() == LaneChangeStage::LCS_EXECUTING ||
                 lc_state.stage() == LaneChangeStage::LCS_PAUSE ||
                 lc_state.stage() == LaneChangeStage::LCS_RETURN) {
        return;
      } else {
        if (is_cone && !ignore_emergence_lane_curb_obs) {
          total_cone_num++;
          total_pre_deceleration_obstacles.insert(obs);
        } else if (is_barrier && !ignore_emergence_lane_curb_obs) {
          total_barrier_num++;
          total_pre_deceleration_obstacles.insert(obs);
        }
      }
    }
  }
}

void UpdatePreBlockInfo(
    const LaneChangeStateProto& lc_state,
    const ad_byd::planning::ConstructionInfo* construction_info,
    double& pre_final_stop_s, std::string& pre_obs_id) {
  if (lc_state.stage() == LaneChangeStage::LCS_NONE ||
      lc_state.stage() == LaneChangeStage::LCS_RETURN) {
    pre_final_stop_s = construction_info->ego_lane_block_info.pre_final_stop_s;
    pre_obs_id = construction_info->ego_lane_block_info.pre_obs_id;
  } else if (lc_state.stage() == LaneChangeStage::LCS_EXECUTING ||
             lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
    if (lc_state.lc_left()) {
      pre_final_stop_s =
          construction_info->left_lane_block_info.pre_final_stop_s;
      pre_obs_id = construction_info->left_lane_block_info.pre_obs_id;
    } else {
      pre_final_stop_s =
          construction_info->right_lane_block_info.pre_final_stop_s;
      pre_obs_id = construction_info->right_lane_block_info.pre_obs_id;
    }
  } else {
    pre_final_stop_s = 1000.0;
    pre_obs_id = "none";
  }
}
bool EgoIsInJunction(const DrivePassage& drive_passage,
                     const int& target_lane_path_num, const bool& city_going,
                     const ApolloTrajectoryPointProto& plan_start_point) {
  const auto cur_station_index =
      drive_passage
          .FindNearestStationIndex(
              Vec2dFromApolloTrajectoryPointProto(plan_start_point))
          .value();
  const auto& cur_station =
      drive_passage.station(StationIndex(cur_station_index));
  if (cur_station.is_in_intersection() && target_lane_path_num == 1 &&
      city_going) {
    return true;
  } else {
    return false;
  }
}

bool IsCenterSingleOccupation(
    const DrivePassage& drive_passage, const bool& obs_center_single_num,
    const LaneChangeStateProto& lc_state,
    const std::set<st::planning::Obstacle, MyComparator>&
        left_neighbor_obstacles,
    const std::set<st::planning::Obstacle, MyComparator>&
        right_neighbor_obstacles,
    const NeighborObsInfo& neighbor_obstacle_info,
    const double& center_obs_s_max, const double& center_obs_s_min,
    const double& center_l_occupy_max, const double& center_l_occupy_min,
    const bool& check_left,
    const bool& is_center_obs_s_too_near_with_neighbor_obs,
    std::string& debug_string) {
  if (obs_center_single_num) {
    if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
      if (!left_neighbor_obstacles.empty() &&
          !right_neighbor_obstacles.empty()) {
        absl::StrAppend(&debug_string, "lane_keep, l_r_c");
        return true;
      } else {
        bool right_cannot_aviliable{false};
        bool left_cannot_aviliable{false};
        const auto curb_l = drive_passage.QueryCurbOffsetAtS(center_obs_s_min);
        if (!curb_l.ok()) {
          absl::StrAppend(&debug_string, "curb_l fail");
        } else {
          double right_curb{curb_l->first}, left_curb{curb_l->second};
          right_cannot_aviliable = std::fabs(curb_l->first) < 2.0;
          left_cannot_aviliable = std::fabs(curb_l->second) < 2.0;
          absl::StrAppend(
              &debug_string, ", lc_keep, center_obs_s_min = ", center_obs_s_min,
              ", center_obs_s_max = ", center_obs_s_max,
              ", neighbor_.s_max = ", neighbor_obstacle_info.s_max,
              ", neighbor_.id = ", neighbor_obstacle_info.id,
              ", right_curb = ", right_curb, ", left_curb = ", left_curb,
              ", check_left = ", check_left);
        }
        if (((center_obs_s_max > neighbor_obstacle_info.s_max) ||
             ((center_obs_s_max - neighbor_obstacle_info.s_max) < 1.0)) &&
            neighbor_obstacle_info.id != "none") {
          if (check_left) {
            if (!right_cannot_aviliable) {
              return false;
            } else {
              return true;
            }
          } else {
            if (!left_cannot_aviliable) {
              return false;
            } else {
              return true;
            }
          }
        } else if (neighbor_obstacle_info.id != "none") {
          absl::StrAppend(&debug_string, ", lc_keep , n > c");
          return is_center_obs_s_too_near_with_neighbor_obs;
        } else {
          absl::StrAppend(&debug_string, ", lc_keep , neightbor none");
          return false;
        }
      }
    } else {
      if (lc_state.lc_left()) {
        absl::StrAppend(
            &debug_string,
            ", lc_change_left, center_l_occupy_min = ", center_l_occupy_min);
        if (center_l_occupy_min > 0.0) {
          return false;
        } else {
          return true;
        }
      } else {
        absl::StrAppend(
            &debug_string,
            ", lc_change_right, center_l_occupy_min = ", center_l_occupy_max);
        if (center_l_occupy_max < 0.0) {
          return false;
        } else {
          return true;
        }
      }
    }
  } else {
    absl::StrAppend(&debug_string, ", not single num");
    return false;
  }
}

bool StaticObsOccupation(const DrivePassage& drive_passage,
                         const bool& city_going, const bool& lka_going,
                         const bool& static_obs_scene,
                         const LaneChangeStateProto& lc_state,
                         const double& center_obs_s_min,
                         const double& center_l_occupy_max,
                         const double& center_l_occupy_min,
                         std::string& static_obs_debug_str) {
  if (city_going || lka_going) {
    if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
      bool right_cannot_aviliable = false;
      bool left_cannot_aviliable = false;
      bool allow_detour_left = true;
      bool allow_detour_right = true;
      const auto curb_l = drive_passage.QueryCurbOffsetAtS(center_obs_s_min);
      if (curb_l.ok()) {
        double right_curb{curb_l->first}, left_curb{curb_l->second};

        right_cannot_aviliable = std::fabs(right_curb) < 3.0;
        left_cannot_aviliable = std::fabs(left_curb) < 3.0;
        bool allow_detour_left =
            center_l_occupy_max < 0.5 && right_cannot_aviliable;
        bool allow_detour_right =
            center_l_occupy_min > -0.5 && left_cannot_aviliable;
        absl::StrAppend(&static_obs_debug_str,
                        "city_lane_keep, static_obs_scene = ", static_obs_scene,
                        ", center_obs_s_min = ", center_obs_s_min,
                        ", right_cannot_aviliable = ", right_cannot_aviliable,
                        ", left_cannot_aviliable = ", left_cannot_aviliable,
                        ", allow_detour_left = ", allow_detour_left,
                        ", allow_detour_right = ", allow_detour_right,
                        ", static_obs_scene = ", static_obs_scene,
                        ", center_l_occupy_max = ", center_l_occupy_max,
                        ", center_l_occupy_min = ", center_l_occupy_min);
      } else {
        absl::StrAppend(&static_obs_debug_str, "curb_l query fail");
      }

      if (static_obs_scene) {
        if (allow_detour_left || allow_detour_right) {
          return false;
        } else {
          return (center_l_occupy_max > 0.0 && center_l_occupy_min < 0.0);
        }
      } else {
        return false;
      }
    } else {
      absl::StrAppend(&static_obs_debug_str,
                      "city_lane_change, static_obs_scene = ", static_obs_scene,
                      ", center_l_occupy_max = ", center_l_occupy_max,
                      ", center_l_occupy_min = ", center_l_occupy_min);
      return static_obs_scene &&
             (center_l_occupy_max > 0.0 && center_l_occupy_min < 0.0);
    }

  } else {
    absl::StrAppend(&static_obs_debug_str,
                    "hw, static_obs_scene = ", static_obs_scene,
                    ", center_l_occupy_max = ", center_l_occupy_max,
                    ", center_l_occupy_min = ", center_l_occupy_min);
    return static_obs_scene &&
           (center_l_occupy_max > 0.0 && center_l_occupy_min < 0.0);
  }
}

bool CheckDrivePassageBlockByTrafficObsOrCurbCross(
    const int& task_idx, const PlannerSemanticMapManager& psmm,
    DrivePassage& drive_passage, const SpacetimeTrajectoryManager& st_traj_mgr,
    const FrenetBox& ego_frenet_box,
    const ApolloTrajectoryPointProto& plan_start_point,
    const bool& is_on_highway, const st::Behavior_FunctionId& function_id,
    const int& target_lane_path_num, const LaneChangeStateProto& lc_state,
    const ad_byd::planning::ConstructionInfo* construction_info,
    const LaneChangeSafetyInfo* pre_lane_change_safety_info,
    const ad_byd::planning::BehaviorCommand intention_dir) {
  absl::flat_hash_set<std::string> has_checked_set;

  constexpr double kOutofCredibleLength = 100.0;
  constexpr double kOutofPreDecelerationLength = 120.0;
  constexpr double kTooCloseLength = 10.0;
  constexpr double kBehindEgoMinS = -5.0;
  constexpr double kObjMaxDiffS = 30.0;
  constexpr double kObjMaxDiffL = 10.0;
  constexpr double kObjectConfidenceThreshold = 0.7;
  constexpr double kBlockingBuffer = kDefaultHalfLaneWidth - 0.7;
  constexpr double kInInvasionBuffer = 0.7;   // m.
  constexpr double kOutInvasionBuffer = 0.2;  // m.
  int center_cone_num{0}, warning_triangle_num{0}, center_cone_num_thre{2},
      center_barrier_num{0}, center_static_obs_num{0},
      center_barrier_num_thre{1}, total_cone_num{0}, total_barrier_num{0},
      row_obstacle_num{0};
  double center_l_occupy_max{-10.0}, center_l_occupy_min{10.0};
  int right_neighbor_cone_num{0}, right_neighbor_barrier_num{0};
  int left_neighbor_cone_num{0}, left_neighbor_barrier_num{0};
  int lane_attr_neighbor_obstacle_num{0};
  std::unordered_map<st::planning::BlockLocation, std::vector<std::string>>
      cone_debug_strings;
  std::set<st::planning::Obstacle, MyComparator> center_obstacle;
  std::set<st::planning::Obstacle, MyComparator> left_neighbor_obstacles;
  std::set<st::planning::Obstacle, MyComparator> right_neighbor_obstacles;
  std::set<st::planning::Obstacle, MyComparator> lane_attr_obstacles;
  std::set<st::planning::Obstacle, MyComparator>
      total_pre_deceleration_obstacles;

  const auto road_type = psmm.GetRoadClass();
  const bool highway_road = psmm.IsOnHighway() ||
                            road_type == V2RoadType::HIGH_WAY_ROAD ||
                            road_type == V2RoadType::EXPRESS_WAY_ROAD;

  bool highway_going =
      /*is_on_highway || */ (function_id == Behavior_FunctionId_HW_NOA);
  bool city_going = (function_id == Behavior_FunctionId_CITY_NOA);
  bool lka_going = (function_id == Behavior_FunctionId_LKA);
  bool one_target_path = (target_lane_path_num == 1);
  bool have_curb_cross{false};
  uint64_t cross_curb_id{0};
  uint64_t cross_lane_id{0};
  double curb_stop_s{1000.0};

  bool enable_stop{false};
  bool enable_slow_down{false};
  double ego_v = plan_start_point.v();
  std::string stop_type;
  bool pre_is_construction_scene{false};
  int none_construction_scene_cnt{-1};
  bool pre_construction_is_left{false};
  bool pre_is_emergence_lane_scene{false};
  double pre_final_stop_s{1000.0};
  std::string pre_obs_id{"none"};
  std::string pre_deceleration_debug_str{"pre_decel : "};

  if (construction_info != nullptr) {
    pre_is_construction_scene = construction_info->pre_is_construction_scene;
    none_construction_scene_cnt =
        construction_info->none_construction_scene_cnt;
    pre_construction_is_left = construction_info->pre_construction_is_left;
    pre_is_emergence_lane_scene =
        construction_info->pre_is_emergence_lane_scene;
    UpdatePreBlockInfo(lc_state, construction_info, pre_final_stop_s,
                       pre_obs_id);
  }

  // step1 : check if drive_passage have curb cross
  for (const auto& station : drive_passage.stations()) {
    if (station.accumulated_s() > ego_frenet_box.s_max &&
        station.has_cross_curb()) {
      curb_stop_s = station.accumulated_s();
      cross_curb_id = station.cross_curb_id();
      cross_lane_id = station.lane_id();
      have_curb_cross = true;
      break;
    }
  }

  // step2 : check if traffic static cone/barrier/warning_triangle
  for (const auto& traj : st_traj_mgr.trajectories()) {
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());

    const auto& obj_type = traj.object_type();
    const auto& obj_lane_attr = traj.planner_object()
                                    .object_proto()
                                    .vision_attribute()
                                    .lane_attr_type();
    bool is_cone = obj_type == ObjectType::OT_CONE;
    bool is_warning_triangle = obj_type == ObjectType::OT_WARNING_TRIANGLE;
    bool is_barrier = obj_type == ObjectType::OT_BARRIER;
    bool is_unknown_static_obs = obj_type == ObjectType::OT_UNKNOWN_STATIC;
    bool is_row_obstacle = obj_type == ObjectType::OT_ROW_OBSTACLES;

    if (is_cone || is_barrier || is_warning_triangle || is_unknown_static_obs ||
        is_row_obstacle) {
      ASSIGN_OR_CONTINUE(const auto obj_aabox_or,
                         drive_passage.QueryFrenetBoxAtContour(traj.contour()));

      if (is_row_obstacle) {
        if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
          ++row_obstacle_num;
          continue;
        } else {
          continue;
        }
      }

      if (obj_aabox_or.s_min > kOutofPreDecelerationLength) {
        continue;
      } else {
        auto obs = Obstacle{std::string(traj.object_id()), obj_aabox_or,
                            obj_lane_attr};
        CaculateTotalConstructionObsNum(
            psmm, drive_passage, lc_state, highway_road, is_cone, is_barrier,
            obj_lane_attr, pre_is_construction_scene, pre_construction_is_left,
            obs, total_cone_num, total_barrier_num,
            total_pre_deceleration_obstacles, pre_deceleration_debug_str);
      }
      if ((obj_aabox_or.s_min > kOutofCredibleLength ||
           obj_aabox_or.s_max < kBehindEgoMinS) 
           /*|| (obj_aabox_or.s_max < ego_frenet_box.s_max + kTooCloseLength)*/)// ignore too far
      {
        continue;
      }
      if ((obj_aabox_or.s_max - obj_aabox_or.s_min > kObjMaxDiffS) || 
          (obj_aabox_or.l_max - obj_aabox_or.l_min > kObjMaxDiffL)) {
        continue;
      }

      // consider center
      if (IsObjectBlockingCenterLine(obj_aabox_or.l_min, obj_aabox_or.l_max,
                                     /*center_l*/ 0.0, kBlockingBuffer)) {
        if (!CheckLaneAttr(lc_state, function_id, obj_aabox_or.s_min,
                           obj_lane_attr, true)) {
          cone_debug_strings[BlockLocation::BLOCK_CENTER].emplace_back(
              absl::StrCat("center_lane_attr_false, id: ",
                           static_cast<std::string>(traj.object_id()),
                           " type: ", ObjectTypeToString(obj_type), " s_min: ",
                           obj_aabox_or.s_min, " s_max: ", obj_aabox_or.s_max,
                           " l_min: ", obj_aabox_or.l_min, " l_max: ",
                           obj_aabox_or.l_max, " v: ", traj.pose().v(),
                           " , obj_lane_attr: ", obj_lane_attr));

          continue;
        }

        if (is_cone) {
          ++center_cone_num;
        } else if (is_warning_triangle) {
          ++warning_triangle_num;
        } else if (is_barrier) {
          ++center_barrier_num;
        } else if (is_unknown_static_obs) {
          ++center_static_obs_num;
        }

        center_l_occupy_max =
            std::min(std::max(center_l_occupy_max, obj_aabox_or.l_max), 1.0);
        center_l_occupy_min =
            std::max(std::min(center_l_occupy_min, obj_aabox_or.l_min), -1.0);

        center_obstacle.insert(Obstacle{std::string(traj.object_id()),
                                        obj_aabox_or, obj_lane_attr});

        cone_debug_strings[BlockLocation::BLOCK_CENTER].emplace_back(
            absl::StrCat(
                "id: ", static_cast<std::string>(traj.object_id()),
                " type: ", ObjectTypeToString(obj_type),
                " s_min: ", obj_aabox_or.s_min, " s_max: ", obj_aabox_or.s_max,
                " l_min: ", obj_aabox_or.l_min, " l_max: ", obj_aabox_or.l_max,
                " v: ", traj.pose().v(), ", lane_attr_type = ", obj_lane_attr));
        continue;
      }

      // consider vision_attribute
      if (traj.planner_object()
                  .object_proto()
                  .vision_attribute()
                  .lane_attr_conf() >= kObjectConfidenceThreshold &&
          IsValidLaneAttrType(obj_lane_attr)) {
        lane_attr_neighbor_obstacle_num++;
        lane_attr_obstacles.insert(Obstacle{std::string(traj.object_id()),
                                            obj_aabox_or, obj_lane_attr});
        cone_debug_strings[BlockLocation::BLOCK_LANE_ATTR].emplace_back(
            absl::StrCat(
                "id: ", static_cast<std::string>(traj.object_id()),
                " type: ", ObjectTypeToString(obj_type),
                " s_min: ", obj_aabox_or.s_min, " s_max: ", obj_aabox_or.s_max,
                " l_min: ", obj_aabox_or.l_min, " l_max: ", obj_aabox_or.l_max,
                " v: ", traj.pose().v(), ", lane_attr_type = ", obj_lane_attr));
      }

      const auto [right_boundary, left_boundary] =
          drive_passage.QueryEnclosingLaneBoundariesAtS(obj_aabox_or.s_min);
      // Exclude very wide lane and unknown_static_obs
      if (std::fabs(left_boundary->lat_offset - right_boundary->lat_offset) >
              2 * kMaxHalfLaneWidth ||
          is_unknown_static_obs) {
        continue;
      }

      // consider right
      if (right_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
          IsObjectOverlapLine(obj_aabox_or.l_min, obj_aabox_or.l_max,
                              right_boundary->lat_offset, kInInvasionBuffer,
                              kOutInvasionBuffer)) {
        if (!CheckLaneAttr(lc_state, function_id, obj_aabox_or.s_min,
                           obj_lane_attr, false)) {
          cone_debug_strings[BlockLocation::BLOCK_RIGHT].emplace_back(
              absl::StrCat("right_lane_attr_false, id: ",
                           static_cast<std::string>(traj.object_id()),
                           " type: ", ObjectTypeToString(obj_type), " s_min: ",
                           obj_aabox_or.s_min, " s_max: ", obj_aabox_or.s_max,
                           " l_min: ", obj_aabox_or.l_min, " l_max: ",
                           obj_aabox_or.l_max, " v: ", traj.pose().v(),
                           " , obj_lane_attr: ", obj_lane_attr));

          continue;
        }
        if (is_cone) {
          ++right_neighbor_cone_num;
        }
        else if (is_barrier) {
          ++right_neighbor_barrier_num; 
        }
        else if (is_warning_triangle) {
          ++warning_triangle_num;
        }
        right_neighbor_obstacles.insert(Obstacle{std::string(traj.object_id()),
                                                 obj_aabox_or, obj_lane_attr});
        cone_debug_strings[BlockLocation::BLOCK_RIGHT].emplace_back(
            absl::StrCat(
                "id: ", static_cast<std::string>(traj.object_id()), " type: ",
                ObjectTypeToString(obj_type), " s_min: ", obj_aabox_or.s_min,
                " s_max: ", obj_aabox_or.s_max, " l_min: ", obj_aabox_or.l_min,
                " l_max: ", obj_aabox_or.l_max, " v: ", traj.pose().v(),
                " l_offset: ", left_boundary->lat_offset, " r_offset: ",
                right_boundary->lat_offset, " l_type = ", left_boundary->type,
                " r_type = ", right_boundary->type,
                " , obj_lane_attr: ", obj_lane_attr));
      }

      // consider left
      if (left_boundary->type != StationBoundaryType::VIRTUAL_CURB &&
          IsObjectOverlapLine(obj_aabox_or.l_min, obj_aabox_or.l_max,
                              left_boundary->lat_offset, kOutInvasionBuffer,
                              kInInvasionBuffer)) {
        if (!CheckLaneAttr(lc_state, function_id, obj_aabox_or.s_min,
                           obj_lane_attr, false)) {
          cone_debug_strings[BlockLocation::BLOCK_LEFT].emplace_back(
              absl::StrCat("left_lane_attr_false, id: ",
                           static_cast<std::string>(traj.object_id()),
                           " type: ", ObjectTypeToString(obj_type), " s_min: ",
                           obj_aabox_or.s_min, " s_max: ", obj_aabox_or.s_max,
                           " l_min: ", obj_aabox_or.l_min, " l_max: ",
                           obj_aabox_or.l_max, " v: ", traj.pose().v(),
                           " , obj_lane_attr: ", obj_lane_attr));

          continue;
        }

        if (is_cone) {
          ++left_neighbor_cone_num;
        } 
        else if (is_barrier) {
          ++left_neighbor_barrier_num; 
        }
        else if (is_warning_triangle) {
          ++warning_triangle_num;
        }
        left_neighbor_obstacles.insert(Obstacle{std::string(traj.object_id()),
                                                obj_aabox_or, obj_lane_attr});
        cone_debug_strings[BlockLocation::BLOCK_LEFT].emplace_back(absl::StrCat(
            "id: ", static_cast<std::string>(traj.object_id()),
            " type: ", ObjectTypeToString(obj_type),
            " s_min: ", obj_aabox_or.s_min, " s_max: ", obj_aabox_or.s_max,
            " l_min: ", obj_aabox_or.l_min, " l_max: ", obj_aabox_or.l_max,
            " v: ", traj.pose().v(), " l_offset: ", left_boundary->lat_offset,
            " r_offset: ", right_boundary->lat_offset, " l_type = ",
            left_boundary->type, " r_type = ", right_boundary->type,
            " , obj_lane_attr: ", obj_lane_attr));
      }

    } else {
      continue;
    }
  }

  std::string string_res;
  for (const auto& [block_location, vec_debug_string] : cone_debug_strings) {
    auto block_location_string = BlockLocationInfoToString(
        block_location, center_cone_num, center_barrier_num,
        center_static_obs_num, center_l_occupy_max, center_l_occupy_min,
        right_neighbor_obstacles.size(), left_neighbor_obstacles.size(),
        lane_attr_neighbor_obstacle_num);
    absl::StrAppend(&string_res, block_location_string);

    for (const auto& str : vec_debug_string) {
      absl::StrAppend(&string_res, str, "\n");
    }

    absl::StrAppend(&string_res, "--------\n");
  }

  // step2.1 set pre_deceleration
  if(lc_state.stage() == LaneChangeStage::LCS_NONE && highway_road)
  {
    total_cone_num = std::max({total_cone_num, center_cone_num,right_neighbor_cone_num,left_neighbor_cone_num});
    total_barrier_num = std::max({total_barrier_num, center_barrier_num, right_neighbor_barrier_num, left_neighbor_barrier_num});
  }

  int total_cone_num_thre{5}, total_barrier_num_thre{4};
  bool have_row_obstacle = (row_obstacle_num >= 1);
  if (pre_is_construction_scene) {
    total_cone_num_thre = 2;
    total_barrier_num_thre = 2;
    if ((total_cone_num <= 4 && total_cone_num > 0) ||
        (total_barrier_num <= 4 && total_barrier_num > 0)) {
      for (auto obs_tmp : total_pre_deceleration_obstacles) {
        if (obs_tmp.frenet_box.s_min >= 50.0) {
          if (total_cone_num <= 4 && total_cone_num > 0) {
            total_cone_num--;
          } else if (total_barrier_num <= 4 && total_barrier_num > 0) {
            total_barrier_num--;
          }
        }
      }
    }
  }

  if (total_cone_num >= total_cone_num_thre ||
      total_barrier_num >= total_barrier_num_thre || have_row_obstacle) {
    drive_passage.SetTrafficIsConstruction();
    if (!pre_is_construction_scene) {
      if (!total_pre_deceleration_obstacles.empty()) {
        bool obs_location_left =
            total_pre_deceleration_obstacles.begin()->frenet_box.s_max > 0.0;
        drive_passage.SetTrafficConstructionLocation(obs_location_left);
      }
    }
  } else if (pre_is_construction_scene && none_construction_scene_cnt < 15) {
    drive_passage.SetTrafficIsConstructionDebouncing();
  }

  // step3 : check center_zone have curb behind center_obs_s_max
  bool need_check_center_have_curb = !center_obstacle.empty();
  double center_obs_s_min = need_check_center_have_curb
                                ? center_obstacle.begin()->frenet_box.s_min
                                : 1000.0;
  double center_obs_s_max = need_check_center_have_curb
                                ? center_obstacle.rbegin()->frenet_box.s_max
                                : 1000.0;
  int center_have_curb_count{0};
  std::string center_check_curb_str{"center_check_curb_info : "};
  for (auto tmp : center_obstacle) {
    absl::StrAppend(&center_check_curb_str, tmp.frenet_box.s_max, ", ");
  }
  absl::StrAppend(&center_check_curb_str, center_obs_s_max, "\n");

  bool center_obs_near_junction{false};
  if (need_check_center_have_curb) {
    CheckCurb(drive_passage, center_obs_s_max, /*check_center*/ true,
              /*check_left*/ false, center_have_curb_count,
              center_obs_near_junction, center_check_curb_str);
    absl::StrAppend(&string_res, center_check_curb_str, "\n");
  }

  // step4 : check if occupation_cross_center / l_too_near with center_line /
  // s_too_near with neighbor_cone
  //  step4.1 check if occupation_cross_center
  bool cone_occupation_cross_center =
      (center_l_occupy_max > 0.0 && center_l_occupy_min < 0.0) &&
      (center_cone_num >= 2);
  bool barrier_occupation_cross_center =
      (center_l_occupy_max > 0.0 && center_l_occupy_min < 0.0) &&
      (center_barrier_num >= 1);
  double too_near_dist_thre = 0.95;
  double most_near_dist = 10.0;
  bool not_cross_but_l_too_near{false};

  //  step4.2 set some varialbe about neighbor
  bool need_check_neighbor{true}, check_left{true};
  double neighbor_query_start = 0.0;
  NeighborObsInfo neighbor_obstacle_info;
  std::set<st::planning::Obstacle, MyComparator> neighbor_obstacles;
  need_check_neighbor = DecideCheckWhichNeighbor(
      left_neighbor_obstacles, right_neighbor_obstacles, neighbor_obstacles,
      check_left, neighbor_obstacle_info);

  //  step4.3 check l_too_near with center_line
  bool both_obs_on_left =
      center_l_occupy_min > 0.0 && center_l_occupy_min != 10.0;
  bool both_obs_on_right =
      center_l_occupy_max < 0.0 && center_l_occupy_min != -10.0;
  if (!cone_occupation_cross_center || !barrier_occupation_cross_center) {
    most_near_dist =
        both_obs_on_left ? center_l_occupy_min : center_l_occupy_max;
    not_cross_but_l_too_near =
        (std::fabs(most_near_dist) < too_near_dist_thre) &&
        (center_cone_num >= 2 || center_barrier_num >= 2) && highway_going;
  }

  // step4.4 check s_too_near with neighbor_cone
  bool is_center_obs_s_too_near_with_neighbor_obs{false};
  if (need_check_neighbor) {
    is_center_obs_s_too_near_with_neighbor_obs =
        std::fabs(center_obs_s_min - neighbor_obstacle_info.s_min) < 40.0 &&
        (center_obs_s_min != neighbor_obstacle_info.s_min);
  }

  // step4.5 check if lane_keep drive_passage block but there is no avliable
  // lane or avliable lane is emergence_lane

  bool have_avliable_zone{true};
  bool is_emergence_lane_scene{false};
  bool pre_lc_left_is_avliable{true};
  bool pre_lc_right_is_avliable{true};
  bool pre_lane_change_left_is_avliable{true};
  bool pre_lane_change_right_is_avliable{true};
  std::string have_avliable_zone_debug_string{"aviliable_zone: "};
  have_avliable_zone = CheckHaveAvliableZone(
      drive_passage, lc_state, ego_frenet_box, neighbor_obstacles, check_left,
      center_obstacle, psmm, pre_lane_change_safety_info,
      is_emergence_lane_scene, pre_lane_change_left_is_avliable,
      pre_lane_change_right_is_avliable, have_avliable_zone_debug_string);

  // step4.6 ego is in junction and target_lane_path = 1 check
  bool ego_is_in_junction = EgoIsInJunction(drive_passage, target_lane_path_num,
                                            city_going, plan_start_point);

  // step5 : center check
  bool static_obs_scene = (center_static_obs_num >= 1);
  bool special_case =
      one_target_path && (both_obs_on_left || both_obs_on_right);
  std::string static_obs_debug_str{"static_obs_debug: "};
  bool static_obs_occupation_crosss_center = StaticObsOccupation(
      drive_passage, city_going, lka_going, static_obs_scene, lc_state,
      center_obs_s_min, center_l_occupy_max, center_l_occupy_min,
      static_obs_debug_str);
  bool obs_center_block =
      ((center_have_curb_count >= 2 && !static_obs_scene) ||
       (warning_triangle_num >= 1) || cone_occupation_cross_center ||
       barrier_occupation_cross_center ||
       static_obs_occupation_crosss_center) &&
      !ego_is_in_junction && !special_case;
  bool obs_l_too_near = ((center_cone_num >= 2 || center_barrier_num >= 2 ||
                          center_static_obs_num >= 2) &&
                         not_cross_but_l_too_near);

  bool obs_center_single_num =
      (center_cone_num == 1 || center_barrier_num == 1);
  std::string obs_center_single_debug_str{"center_single_debug_str : "};
  bool obs_center_single = IsCenterSingleOccupation(
      drive_passage, obs_center_single_num, lc_state, left_neighbor_obstacles,
      right_neighbor_obstacles, neighbor_obstacle_info, center_obs_s_max,
      center_obs_s_min, center_l_occupy_max, center_l_occupy_min, check_left,
      is_center_obs_s_too_near_with_neighbor_obs, obs_center_single_debug_str);

  obs_center_single = obs_center_single && !special_case;

  if (have_curb_cross || obs_center_block || obs_l_too_near ||
      obs_center_single || have_row_obstacle) {
    auto it_from_center_set =
        std::find_if(center_obstacle.begin(), center_obstacle.end(),
                     [&pre_obs_id](const st::planning::Obstacle& obs) {
                       return obs.id == pre_obs_id;
                     });
    auto it_from_lane_attr_set =
        std::find_if(lane_attr_obstacles.begin(), lane_attr_obstacles.end(),
                     [&pre_obs_id](const st::planning::Obstacle& obs) {
                       return obs.id == pre_obs_id;
                     });
    double center_obs_s = center_obs_s_min;
    std::string found_obs_id{"none"};
    if (it_from_center_set != center_obstacle.end()) {
      center_obs_s = it_from_center_set->frenet_box.s_min;
      found_obs_id = it_from_center_set->id;
    } else if (it_from_lane_attr_set != lane_attr_obstacles.end()) {
      center_obs_s = it_from_lane_attr_set->frenet_box.s_min;
      found_obs_id = it_from_lane_attr_set->id;
    }
    double final_stop_s = center_obs_s;
    std::string debug_string;
    BlockReason block_reason;
    std::string block_info_string;
    std::string obs_id{"none"};
    if (have_curb_cross && final_stop_s > curb_stop_s) {
      enable_stop = true;
      enable_slow_down = false;
      stop_type = "stop";
      block_reason = BlockReason::CURB_CROSS;
      final_stop_s = curb_stop_s;
      obs_id = std::to_string(cross_curb_id);
      debug_string = absl::StrCat(
          Log2DDS::TaskPrefix(task_idx),
          ":true, curb_cross, cross_lane_id: ", cross_lane_id,
          ", cross_curb_id: ", cross_curb_id, ", final_stop_s = ", final_stop_s,
          ", ego_v = ", ego_v, ", have_avliable_zone = ", have_avliable_zone,
          ", ego_is_in_junction = ", ego_is_in_junction,
          ", special_case = ", special_case,
          ", intention_dir = ", static_cast<int>(intention_dir));
      Log2DDS::LogDataV2("schedule_obstacle_debug", debug_string);
    } else {
      if (obs_center_block) {
        block_reason = (center_have_curb_count >= 2)
                           ? BlockReason::OBS_CENTER_CURB
                           : BlockReason::OBS_CENTER_OCCUPATION;
        if (!is_emergence_lane_scene && pre_is_emergence_lane_scene) {
          is_emergence_lane_scene = pre_is_emergence_lane_scene;
        }
        if (block_reason == BlockReason::OBS_CENTER_CURB &&
            !is_emergence_lane_scene) {
          enable_stop = true;
          enable_slow_down = false;
          stop_type = "stop";
        } else {
          SetStopTypeAccordingSafeZone(
              final_stop_s, ego_v, city_going, lka_going,
              pre_lane_change_safety_info, intention_dir, have_avliable_zone,
              is_emergence_lane_scene, block_reason, enable_stop,
              enable_slow_down, stop_type);
        }
        debug_string = absl::StrCat(
            Log2DDS::TaskPrefix(task_idx),
            ":true, block_center, curb_count = ", center_have_curb_count,
            " , triangle_num = ", warning_triangle_num,
            ", cone_occupation_cross_center = ", cone_occupation_cross_center,
            ", barrier_occupation_cross_center = ",
            barrier_occupation_cross_center,
            ", static_obs_occupation_crosss_center = ",
            static_obs_occupation_crosss_center,
            ", row_obstacle_num = ", row_obstacle_num,
            ", total_cone_num = ", total_cone_num,
            ", total_barrier_num = ", total_barrier_num,
            ", pre_is_construction_scene = ", pre_is_construction_scene,
            ", none_construction_scene_cnt = ", none_construction_scene_cnt,
            ", have_avliable_zone = ", have_avliable_zone,
            ", ego_is_in_junction = ", ego_is_in_junction,
            ", special_case = ", special_case,
            ", found_obs_id = ", found_obs_id,
            ", intention_dir = ", static_cast<int>(intention_dir),
            " ,final_stop_s = ", final_stop_s, "\n",
            have_avliable_zone_debug_string, "\n", obs_center_single_debug_str,
            "\n", static_obs_debug_str);
      } else if (obs_l_too_near) {
        if (!is_emergence_lane_scene && pre_is_emergence_lane_scene) {
          is_emergence_lane_scene = pre_is_emergence_lane_scene;
        }
        block_reason = BlockReason::OBS_CENTER_TOO_NEAR;
        SetStopTypeAccordingSafeZone(final_stop_s, ego_v, city_going, lka_going,
                                     pre_lane_change_safety_info, intention_dir,
                                     have_avliable_zone,
                                     is_emergence_lane_scene, block_reason,
                                     enable_stop, enable_slow_down, stop_type);
        debug_string = absl::StrCat(
            Log2DDS::TaskPrefix(task_idx),
            ":true, block_center_too_near, cone_num = ", center_cone_num,
            ", center_barrier_num = ", center_barrier_num,
            ", center_static_obs_num = ", center_static_obs_num,
            " , not_cross_but_l_too_near = ", not_cross_but_l_too_near,
            ", most_near_dist = ", most_near_dist,
            " final_stop_s = ", final_stop_s,
            ", have_avliable_zone = ", have_avliable_zone,
            ", ego_is_in_junction = ", ego_is_in_junction,
            ", special_case = ", special_case,
            ", found_obs_id = ", found_obs_id,
            ", intention_dir = ", static_cast<int>(intention_dir), "\n",
            have_avliable_zone_debug_string, "\n", obs_center_single_debug_str,
            "\n", static_obs_debug_str, "\n", pre_deceleration_debug_str);
      } else if (obs_center_single) {
        if (!is_emergence_lane_scene && pre_is_emergence_lane_scene) {
          is_emergence_lane_scene = pre_is_emergence_lane_scene;
        }
        block_reason = BlockReason::OBS_CENTER_SINGLE;
        SetStopTypeAccordingSafeZone(final_stop_s, ego_v, city_going, lka_going,
                                     pre_lane_change_safety_info, intention_dir,
                                     have_avliable_zone,
                                     is_emergence_lane_scene, block_reason,
                                     enable_stop, enable_slow_down, stop_type);
        debug_string = absl::StrCat(
            Log2DDS::TaskPrefix(task_idx), ":true, block_center_sigle_cone ",
            ",center_obs_s_min = ", center_obs_s_min,
            ", neighbor_obs_s_min = ", neighbor_obstacle_info.s_min,
            ", total_cone_num = ", total_cone_num,
            ", total_barrier_num = ", total_barrier_num,
            ", pre_is_construction_scene = ", pre_is_construction_scene,
            ", none_construction_scene_cnt = ", none_construction_scene_cnt,
            ", have_avliable_zone = ", have_avliable_zone,
            " final_stop_s = ", final_stop_s,
            ", ego_is_in_junction = ", ego_is_in_junction,
            ", special_case = ", special_case,
            ", found_obs_id = ", found_obs_id,
            ", intention_dir = ", static_cast<int>(intention_dir), "\n",
            have_avliable_zone_debug_string, "\n", obs_center_single_debug_str,
            "\n", static_obs_debug_str, "\n", pre_deceleration_debug_str);
      } else if (have_row_obstacle) {
        block_reason = BlockReason::OBS_ROW_OBSTACLES;
        final_stop_s = 50.0;
        found_obs_id = "row_obstacle";
        SetStopTypeAccordingSafeZone(final_stop_s, ego_v, city_going, lka_going,
                                     pre_lane_change_safety_info, intention_dir,
                                     have_avliable_zone,
                                     is_emergence_lane_scene, block_reason,
                                     enable_stop, enable_slow_down, stop_type);
        debug_string = absl::StrCat(
            Log2DDS::TaskPrefix(task_idx), ":true, block_row_obstacle ",
            ",center_obs_s_min = ", center_obs_s_min,
            ", neighbor_obs_s_min = ", neighbor_obstacle_info.s_min,
            ", total_cone_num = ", total_cone_num,
            ", total_barrier_num = ", total_barrier_num,
            ", pre_is_construction_scene = ", pre_is_construction_scene,
            ", none_construction_scene_cnt = ", none_construction_scene_cnt,
            ", have_avliable_zone = ", have_avliable_zone,
            " final_stop_s = ", final_stop_s,
            ", ego_is_in_junction = ", ego_is_in_junction,
            ", special_case = ", special_case,
            ", found_obs_id = ", found_obs_id,
            ", intention_dir = ", static_cast<int>(intention_dir), "\n",
            have_avliable_zone_debug_string, "\n", obs_center_single_debug_str,
            "\n", static_obs_debug_str, "\n", pre_deceleration_debug_str);
      }

      if (found_obs_id != "none") {
        obs_id = found_obs_id;
      } else {
        obs_id =
            !center_obstacle.empty() ? center_obstacle.begin()->id : "none";
      }

      absl::StrAppend(&debug_string, "\n", string_res);
      Log2DDS::LogDataV2("schedule_obstacle_debug", debug_string);
    }

    block_info_string = absl::StrCat(BlockReason_Name(block_reason), "-",
                                     obs_id, "-", stop_type);
    SetStopInfo(task_idx, drive_passage, final_stop_s, enable_stop,
                enable_slow_down, block_reason, have_avliable_zone,
                is_emergence_lane_scene, block_info_string, obs_id);
    return true;
  }

  // step6 : check left / right if have curb
  std::string neighbor_debug_str{"neighbor_info : "};

  for (auto tmp : neighbor_obstacles) {
    absl::StrAppend(&neighbor_debug_str, tmp.frenet_box.s_max, ", ");
  }
  absl::StrAppend(
      &neighbor_debug_str, "\n",
      "neighbor_query_start = ", neighbor_obstacle_info.s_max,
      ", need_check_neighbor = ", need_check_neighbor,
      ", neighbor_obstacle_info.s_min = ", neighbor_obstacle_info.s_min, "\n");

  int have_curb_count = 0;
  bool neighbor_obs_near_junction{false};
  if (need_check_neighbor && neighbor_obstacle_info.s_min > 50.0) {
    CheckCurb(drive_passage, neighbor_obstacle_info.s_max,
              /*check_center*/ false, check_left, have_curb_count,
              neighbor_obs_near_junction, neighbor_debug_str);
  }

  // step7 : check neighbor obstacle block trend
  bool neighbor_block_trend{false};
  if (highway_going && target_lane_path_num >= 2) {
    neighbor_block_trend =
        CheckNeighborObstacleBlockTrend(neighbor_obstacles, neighbor_debug_str);
  }

  std::string false_debug_string = absl::StrCat(
      "false, center_have_curb_count = ", center_have_curb_count,
      " , warning_triangle_num = ", warning_triangle_num,
      ", cone_occupation_cross_center = ", cone_occupation_cross_center,
      ", barrier_occupation_cross_center = ", barrier_occupation_cross_center,
      ", static_obs_occupation_crosss_center = ",
      static_obs_occupation_crosss_center,
      ", center_cone_num = ", center_cone_num,
      " , not_cross_but_l_too_near = ", not_cross_but_l_too_near,
      ", most_near_dist = ", most_near_dist,
      ", center_obs_s_min = ", center_obs_s_min,
      ", neighbor_obs_s_min = ", neighbor_obstacle_info.s_min,
      ", have_curb_count = ", have_curb_count,
      ", neighbor_obstacles.size() = ", neighbor_obstacles.size(),
      ", check_left = ", check_left, ", highway_going = ", highway_going,
      ", is_center_obs_s_too_near_neighbor = ",
      is_center_obs_s_too_near_with_neighbor_obs,
      ", target_lane_path_num = ", target_lane_path_num,
      ", center_obs_near_junction = ", center_obs_near_junction,
      ", neighbor_obs_near_junction = ", neighbor_obs_near_junction,
      ", total_cone_num = ", total_cone_num,
      ", total_barrier_num = ", total_barrier_num,
      ", pre_is_construction_scene = ", pre_is_construction_scene,
      ", none_construction_scene_cnt = ", none_construction_scene_cnt,
      ", have_avliable_zone = ", have_avliable_zone,
      ", ego_is_in_junction = ", ego_is_in_junction,
      ", special_case = ", special_case, ", highway_road = ", highway_road,
      ", intention_dir = ", static_cast<int>(intention_dir), "\n",
      neighbor_debug_str, "\n", have_avliable_zone_debug_string, "\n",
      obs_center_single_debug_str, "\n", static_obs_debug_str, "\n",
      pre_deceleration_debug_str);

  // step8 : check left / right
  if ((neighbor_obstacles.size() >= 2) || warning_triangle_num >= 1) {
    if (have_curb_count >= 2 && !neighbor_obs_near_junction) {
      BlockReason block_reason = BlockReason::OBS_BOUNDARY_CURB;
      std::string obs_id = neighbor_obstacle_info.id;
      double final_stop_s = neighbor_obstacle_info.s_max;
      enable_stop = false;
      enable_slow_down = true;
      stop_type = "slow_down";
      std::string block_info_string = absl::StrCat(
          BlockReason_Name(block_reason), "-", obs_id, "-", stop_type);
      SetStopInfo(task_idx, drive_passage, final_stop_s, enable_stop,
                  enable_slow_down, block_reason, have_avliable_zone,
                  is_emergence_lane_scene, block_info_string, obs_id);
      std::string debug_string = absl::StrCat(
          Log2DDS::TaskPrefix(task_idx),
          ":true, block_boundary_curb, have_curb_count =  ", have_curb_count,
          ",final_stop_s = ", final_stop_s,
          ", total_cone_num = ", total_cone_num,
          ", total_barrier_num = ", total_barrier_num,
          ", pre_is_construction_scene = ", pre_is_construction_scene,
          ", none_construction_scene_cnt = ", none_construction_scene_cnt,
          ", have_avliable_zone = ", have_avliable_zone,
          ", ego_is_in_junction = ", ego_is_in_junction,
          ", special_case = ", special_case,
          ", intention_dir = ", static_cast<int>(intention_dir), "\n",
          obs_center_single_debug_str, "\n", static_obs_debug_str, "\n",
          pre_deceleration_debug_str);
      absl::StrAppend(&debug_string, "\n", neighbor_debug_str, "\n", string_res,
                      "\n", have_avliable_zone_debug_string);
      Log2DDS::LogDataV2("schedule_obstacle_debug", debug_string);
      return true;
    } else if (neighbor_block_trend) {
      BlockReason block_reason = BlockReason::OBS_NEIGHBOR_TREND;
      std::string obs_id = neighbor_obstacle_info.id;
      double final_stop_s = neighbor_obstacle_info.s_max;
      enable_stop = false;
      enable_slow_down = true;
      stop_type = "slow_down";
      std::string block_info_string = absl::StrCat(
          BlockReason_Name(block_reason), "-", obs_id, "-", stop_type);
      SetStopInfo(task_idx, drive_passage, final_stop_s, enable_stop,
                  enable_slow_down, block_reason, have_avliable_zone,
                  is_emergence_lane_scene, block_info_string, obs_id);
      std::string debug_string = absl::StrCat(
          Log2DDS::TaskPrefix(task_idx),
          ":true, block_trend ,final_stop_s = ", final_stop_s,
          ", total_cone_num = ", total_cone_num,
          ", total_barrier_num = ", total_barrier_num,
          ", pre_is_construction_scene = ", pre_is_construction_scene,
          ", none_construction_scene_cnt = ", none_construction_scene_cnt,
          ", have_avliable_zone = ", have_avliable_zone,
          ", ego_is_in_junction = ", ego_is_in_junction,
          ", intention_dir = ", static_cast<int>(intention_dir), "\n",
          obs_center_single_debug_str, "\n", static_obs_debug_str, "\n",
          pre_deceleration_debug_str);
      absl::StrAppend(&debug_string, "\n", neighbor_debug_str, "\n", string_res,
                      "\n", have_avliable_zone_debug_string);
      Log2DDS::LogDataV2("schedule_obstacle_debug", debug_string);
      return true;
    } else if (warning_triangle_num >= 1) {
      BlockReason block_reason = BlockReason::OBS_NEIGHBOR_TRIANGLE;
      std::string obs_id = neighbor_obstacle_info.id;
      double final_stop_s = neighbor_obstacle_info.s_max;
      enable_stop = true;
      enable_slow_down = false;
      stop_type = "stop";
      std::string block_info_string = absl::StrCat(
          BlockReason_Name(block_reason), "-", obs_id, "-", stop_type);
      SetStopInfo(task_idx, drive_passage, final_stop_s, enable_stop,
                  enable_slow_down, block_reason, have_avliable_zone,
                  is_emergence_lane_scene, block_info_string, obs_id);
      std::string debug_string =
          absl::StrCat(Log2DDS::TaskPrefix(task_idx),
                       ":true, block_triangle ,final_stop_s = ", final_stop_s,
                       ", have_avliable_zone = ", have_avliable_zone,
                       ", ego_is_in_junction = ", ego_is_in_junction,
                       ", intention_dir = ", static_cast<int>(intention_dir),
                       "\n", obs_center_single_debug_str, "\n",
                       static_obs_debug_str, "\n", pre_deceleration_debug_str);
      absl::StrAppend(&debug_string, "\n", neighbor_debug_str, "\n", string_res,
                      "\n", have_avliable_zone_debug_string);
      Log2DDS::LogDataV2("schedule_obstacle_debug", debug_string);
      return true;
    } else {
      drive_passage.ReSetIsEmergencyLaneScene();
      absl::StrAppend(&false_debug_string, "\n", string_res);
      Log2DDS::LogDataV2("schedule_obstacle_debug",
                         absl::StrCat(Log2DDS::TaskPrefix(task_idx), ": ",
                                      false_debug_string));
      return false;
    }
  } else {
    drive_passage.ReSetIsEmergencyLaneScene();
    absl::StrAppend(&false_debug_string, "\n", string_res);
    Log2DDS::LogDataV2(
        "schedule_obstacle_debug",
        absl::StrCat(Log2DDS::TaskPrefix(task_idx), ": ", false_debug_string));
    return false;
  }
}

bool IsTargetLaneBlockerdByStaticObs(
    const DrivePassage& passage, const SpacetimeTrajectoryManager& st_traj_mgr,
    double check_range) {
  absl::flat_hash_set<std::string> has_checked_set;
  std::vector<std::pair<const SpacetimeObjectTrajectory*, double>>
      filtered_obstacles;

  for (const auto& traj : st_traj_mgr.trajectories()) {
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());
    ASSIGN_OR_CONTINUE(const auto aabbox,
                       passage.QueryFrenetBoxAtContour(traj.contour()));
    if (aabbox.center_s() > check_range || aabbox.center_s() < 0) {
      continue;
    }
    if (aabbox.width() < 0.3 && aabbox.center_l() < 1.5 &&
        aabbox.center_l() > 1.0) {
      continue;
    }
    filtered_obstacles.push_back({&traj, aabbox.center_s()});
  }

  std::sort(filtered_obstacles.begin(), filtered_obstacles.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

  if (!filtered_obstacles.empty()) {
    const auto& traj = filtered_obstacles.front().first;
    const auto& aabbox = passage.QueryFrenetBoxAtContour(traj->contour());
    Log2DDS::LogDataV2("block_debug",
                       absl::StrCat(" filtered_obs: ",
                                    static_cast<std::string>(traj->object_id()),
                                    " v: ", traj->pose().v()));

    if (aabbox->center_l() < 1.5 && aabbox->center_l() > -1.5 &&
        traj->is_stationary() && traj->pose().v() < 1e-10 &&
        traj->planner_object()
                .prediction()
                .perception_object()
                .obstacle_light()
                .brake_lights() == ObstacleLightType::LIGHT_OFF) {
      Log2DDS::LogDataV2(
          "block_debug",
          " blocked obs id: " + static_cast<std::string>(traj->object_id()));
      return true;
    }
  }

  return false;
}

std::tuple<double, double, std::map<double, LaneObsInfo>>
CalculateTrafficCongestion(const SpacetimeTrajectoryManager& st_traj_mgr,
                           const DrivePassage& passage,
                           const FrenetBox& ego_frenet_box,
                           const Vec2d& ego_pos) {
  constexpr double kObsLatInLaneBuffer = 1.0;     // m.
  constexpr double kLongiOccupancyBuffer = 5.0;   // m.
  constexpr double kMinConsiderDistance = 100.0;  // m.
  constexpr double kFollowTime = 2.0;             // m.

  std::vector<FrenetBox> objs_on_target;
  absl::flat_hash_set<std::string> has_checked_set;
  std::map<double, LaneObsInfo> lane_obstacles_info;
  // Find all obs in current lane.
  if (IsLaneBlockedByObs(passage, ego_frenet_box, kObsLatInLaneBuffer)) {
    objs_on_target.push_back(ego_frenet_box);
  }
  for (const auto& traj : st_traj_mgr.trajectories()) {
    // Remove duplicated object id.
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());

    ASSIGN_OR_CONTINUE(const auto aabbox,
                       passage.QueryFrenetBoxAtContour(traj.contour()));
    if (!IsLaneBlockedByObs(passage, aabbox, kObsLatInLaneBuffer)) {
      continue;
    }
    objs_on_target.push_back(aabbox);
    auto s_min = aabbox.s_min;
    if ((s_min < kMinConsiderDistance) && (s_min > 0.0) &&
        (traj.object_type() == ObjectType::OT_VEHICLE ||
         traj.object_type() == ObjectType::OT_LARGE_VEHICLE)) {
      lane_obstacles_info[s_min].id =
          static_cast<std::string>(traj.object_id());
      lane_obstacles_info[s_min].frenet_box = aabbox;
      lane_obstacles_info[s_min].obs_v = traj.pose().v();
    }
  }

  // Calculate standard congestion factor
  double standard_congestion_factor = 0.0, traffic_congestion_factor = 0.0;
  const auto speed_limit_or = passage.QuerySpeedLimitAt(ego_pos);
  if (speed_limit_or.ok()) {
    standard_congestion_factor =
        ego_frenet_box.length() * 2 /
        std::max(1.0, *speed_limit_or * kFollowTime + kLongiOccupancyBuffer);
  }

  if (objs_on_target.size() <= 1) {
    traffic_congestion_factor = 0.0;
    return {standard_congestion_factor, traffic_congestion_factor,
            lane_obstacles_info};
  }

  // Sort all obstacle on target.
  std::stable_sort(
      objs_on_target.begin(), objs_on_target.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.s_min > rhs.s_min; });
  const double consider_max_length =
      std::max(objs_on_target.front().s_max - objs_on_target.back().s_min,
               kMinConsiderDistance / objs_on_target.size());
  double occupancy_length = objs_on_target.front().length();
  for (int i = 1; i < objs_on_target.size(); ++i) {
    occupancy_length +=
        std::min(kLongiOccupancyBuffer,
                 objs_on_target.at(i - 1).s_min - objs_on_target.at(i).s_max) +
        objs_on_target.at(i).length();
  }

  traffic_congestion_factor =
      std::min(1.0, occupancy_length / std::max(1.0, consider_max_length));

  return {standard_congestion_factor, traffic_congestion_factor,
          lane_obstacles_info};
}

void DumpDrivePassageToDebugFrame(const DrivePassage& drive_passage,
                                  int plan_id) {
  if (FLAGS_log2dds_verbosity_level < 3) return;

  // VLOG(3) << "draw drive passage...";
  std::vector<Vec2d> centers;
  std::vector<std::string> labels;
  std::vector<Vec2d> max_dis_left;
  std::vector<Vec2d> max_dis_right;

  const int size = drive_passage.stations().size();
  const std::vector<std::string> point_names{"broken_white", "other_white",
                                             "yellow",       "curb",
                                             "virtual_curb", "virtual_lane"};
  std::unordered_map<std::string, std::vector<Vec2d>> points;
  points.reserve(point_names.size());
  for (const auto& name : point_names) {
    points.emplace(name, std::vector<Vec2d>());
    points[name].reserve(size);
  }
  constexpr double kEpsilon = 1e-6;
  for (const auto& station : drive_passage.stations()) {
    centers.emplace_back(station.xy());
    std::stringstream ss;
    ss << "task" << plan_id
       << "_drive_passage_center\naccumulated_s:" << station.accumulated_s()
       << "\nxy:(" << station.xy().x() << "," << station.xy().y()
       << ")\nangle:" << station.tangent().FastAngle();
    labels.emplace_back(ss.str());
    double left_max_dis = 0;
    double right_max_dis = 0;
    for (const auto& boundary : station.boundaries()) {
      auto offset = boundary.lat_offset;
      if (offset > left_max_dis) left_max_dis = offset;
      if (offset < right_max_dis) right_max_dis = offset;
      switch (boundary.type) {
        case StationBoundaryType::BROKEN_WHITE:
          points["broken_white"].emplace_back(station.lat_point(offset));
          break;
        case StationBoundaryType::SOLID_WHITE:
        case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
        case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
          points["other_white"].emplace_back(station.lat_point(offset));
          break;
        case StationBoundaryType::BROKEN_YELLOW:
        case StationBoundaryType::SOLID_YELLOW:
        case StationBoundaryType::SOLID_DOUBLE_YELLOW:
          points["yellow"].emplace_back(station.lat_point(offset));
          break;
        case StationBoundaryType::CURB:
          points["curb"].emplace_back(station.lat_point(offset));
          break;
        case StationBoundaryType::VIRTUAL_CURB:
          points["virtual_curb"].emplace_back(station.lat_point(offset));
          break;
        case StationBoundaryType::VIRTUAL_LANE:
          points["virtual_lane"].emplace_back(station.lat_point(offset));
          break;
        case StationBoundaryType::UNKNOWN_TYPE:
          break;
      }
    }
    max_dis_right.emplace_back(station.lat_point(right_max_dis));
    max_dis_left.emplace_back(station.lat_point(left_max_dis));
  }
  const auto& prefix = Log2DDS::TaskPrefix(plan_id) + "drive-passage_";
  Log2DDS::LogLineV3(prefix + "center", Log2DDS::kAqua, labels, centers);
  Log2DDS::LogLineV3(prefix + "max-dis-right", Log2DDS::kGray, {},
                     max_dis_right);
  Log2DDS::LogLineV3(prefix + "max-dis-left", Log2DDS::kGray, {}, max_dis_left);

  Log2DDS::LogPointsV3(prefix + "broken-white", Log2DDS::kLightGray, {},
                       points["broken_white"]);
  Log2DDS::LogPointsV3(prefix + "other-white", Log2DDS::kWhite, {},
                       points["other_white"]);
  Log2DDS::LogPointsV3(prefix + "yellow", Log2DDS::kYellow, {},
                       points["yellow"]);
  Log2DDS::LogPointsV3(prefix + "curb", Log2DDS::kRed, {}, points["curb"]);
  Log2DDS::LogPointsV3(prefix + "virtual-curb", Log2DDS::kDarkRed, {},
                       points["virtual_curb"]);
  Log2DDS::LogPointsV3(prefix + "virtual-lane", Log2DDS::kLightGray, {},
                       points["virtual_lane"]);
}
}  // namespace

// bool ShouldSmoothRefLane(const TrafficLightInfoMap& tl_info_map,
//                          const DrivePassage& dp, bool prev_smooth_state) {
//   const auto& ego_station = dp.FindNearestStationAtS(0.0);
//   if (ego_station.is_in_intersection()) {
//     // Keep the previous choice once entered intersection.
//     return prev_smooth_state;
//   }

//   const auto waiting_zone_lane_id =
//       FindWaitingZoneAhead(tl_info_map, dp.lane_path());
//   if (waiting_zone_lane_id == mapping::kInvalidElementId) {
//     // No waiting zone, apply smooth regardless of ego pose or traffic light.
//     return true;
//   }
//   // Has waiting zone, decide from traffic light state.
//   return tl_info_map.at(waiting_zone_lane_id)
//              .tls()
//              .at(TrafficLightDirection::LEFT)
//              .tl_state == TrafficLightState::TL_STATE_GREEN;
// }

absl::StatusOr<SchedulerOutput> MakeSchedulerOutput(
    const int& task_idx, const PlannerSemanticMapManager& psmm,
    const std::vector<LanePathInfo>& lp_infos, DrivePassage drive_passage,
    const LanePathInfo& lp_info, const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlannerObjectManager& obj_mgr,
    const ObjectHistoryManager& obj_history_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const mapping::LanePath& preferred_lane_path,
    const LaneChangeStateProto& prev_lc_state, bool borrow, bool should_smooth,
    bool planner_is_l4_mode, Behavior behavior, const int& target_lane_path_num,
    bool is_miss_navi, bool is_continuous_lc,
    const st::DriverAction::LaneChangeCommand lc_cmd_state,
    ad_byd::planning::LcReason lc_reason, std::string leading_id,
    ad_byd::planning::PushDirection lc_push_dir,
    std::optional<bool> is_going_force_route_change_left,
    const PushStatusProto* pre_push_status,
    const absl::flat_hash_set<std::string>* stalled_objects,
    const PausePushSavedOffsetProto* saved_offset,
    const ad_byd::planning::ConstructionInfo* construction_info,
    const LaneChangeSafetyInfo* pre_lane_change_safety_info,
    const ad_byd::planning::BehaviorCommand& intention_dir) {
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const Box2d ego_box = ComputeAvBox(
      ego_pos, plan_start_point.path_point().theta(), vehicle_geom);
  ASSIGN_OR_RETURN(
      auto ego_frenet_box, drive_passage.QueryFrenetBoxAt(ego_box),
      _ << "Ego box " << ego_box.DebugString() << " is out of drive passage!");

  ASSIGN_OR_RETURN(
      auto lc_state,
      MakeLaneChangeState(
          psmm, drive_passage, plan_start_point, ego_frenet_box,
          prev_target_lane_path_from_start, prev_lane_path_before_lc_from_start,
          preferred_lane_path, prev_lc_state, smooth_result_map, should_smooth,
          is_miss_navi, is_continuous_lc, behavior, target_lane_path_num),
      _ << "Making lane change state failed.");

  MakePushState(drive_passage, st_traj_mgr, ego_frenet_box, psmm.map_ptr(),
                &obj_mgr, lp_infos, lp_info, plan_start_point, is_miss_navi,
                borrow, is_going_force_route_change_left, lc_push_dir,
                pre_push_status, stalled_objects, &lc_state);

  bool request_help_lane_change_by_route = false;
  int cur_lc_num = 0;
  double current_lane_navi_dist = 0.0;
  PausePushSavedOffsetProto cur_saved_offset;
  if (saved_offset != nullptr &&
      lc_state.push_state() != PushState::NONE_PUSH) {
    cur_saved_offset.set_pre_push_offset(saved_offset->pre_push_offset());
  }
  // for (const auto& each_lp_info : lp_infos) {
  //   if (each_lp_info.lane_seq_info() &&
  //       each_lp_info.lane_seq_info()->is_current) {
  //     cur_lc_num = each_lp_info.lane_seq_info()->lc_num;
  //     current_lane_navi_dist =
  //     each_lp_info.lane_seq_info()->dist_to_navi_end;
  //   }
  // }
  cur_lc_num = lp_info.lane_seq_info()->lc_num;
  current_lane_navi_dist = lp_info.lane_seq_info()->dist_to_navi_end;
  // const auto request_help_lane_change_by_route_or =
  // CheckNeedHelpToRouteChange(
  //     drive_passage, lc_state, route_navi_info, planner_is_l4_mode, lc_num);
  // if (request_help_lane_change_by_route_or.ok()) {
  //   request_help_lane_change_by_route =
  //   *request_help_lane_change_by_route_or;
  // } else {
  //   LOG_WARN << request_help_lane_change_by_route_or.status().message();
  // }
  // const bool switch_alternate_route =
  //     CheckNeedSwitchRoute(psmm, lc_state, route_navi_info, drive_passage);
  // const auto request_help_lane_change_by_route_or =
  // CheckNeedHelpToRouteChange(
  //     drive_passage, lc_state, route_navi_info, planner_is_l4_mode);
  // if (request_help_lane_change_by_route_or.ok()) {
  //   request_help_lane_change_by_route =
  //   *request_help_lane_change_by_route_or;
  // } else {
  //   LOG_WARN << request_help_lane_change_by_route_or.status().message();
  // }
  const bool switch_alternate_route = false;
  // CheckNeedSwitchRoute(psmm, lc_state, route_navi_info, drive_passage);
  const auto traffic_congestion_state = CalculateTrafficCongestion(
      st_traj_mgr, drive_passage, ego_frenet_box, ego_pos);
  const auto standard_congestion_factor = std::get<0>(traffic_congestion_state);
  const auto traffic_congestion_factor = std::get<1>(traffic_congestion_state);
  const auto lane_obstacles_info = std::get<2>(traffic_congestion_state);
  // bool if_check_static_obs = false;
  // if (drive_passage.lane_seq_info() &&
  //     !drive_passage.lane_seq_info()->is_current) {
  //   for (const auto temp_lp_info : lp_infos) {
  //     double dis_to_junction_min =
  //         temp_lp_info.lane_seq_info()->dist_to_junction;
  //     if (behavior.function_id() == Behavior_FunctionId_CITY_NOA) {
  //       dis_to_junction_min = temp_lp_info.lane_seq_info()->dist_to_junction;
  //     }

  //     Log2DDS::LogDataV2(
  //         "block_debug",
  //         absl::StrCat("temp lp front id: ", temp_lp_info.start_lane_id(),
  //             FormatNumericString(
  //                 " dist_to_navi_end: ",
  //                 temp_lp_info.lane_seq_info()->dist_to_navi_end),
  //             FormatNumericString(
  //                 " dist_to_junction: ",
  //                 temp_lp_info.lane_seq_info()->dist_to_junction),
  //             FormatNumericString(" dist_to_junction_min: ",
  //                                 dis_to_junction_min)));
  //     if (temp_lp_info.lane_seq_info()) {
  //       if (!temp_lp_info.lane_seq_info()->is_current &&
  //           temp_lp_info.lane_seq_info()->dist_to_navi_end > 130.0 &&
  //           dis_to_junction_min > 130.0) {
  //         Log2DDS::LogDataV2(
  //             "block_debug",
  //             absl::StrCat("temp lp front id: ",
  //             temp_lp_info.start_lane_id(),
  //                 " distance is expected ",
  //                 FormatNumericString("dist_to_junction_min: ",
  //                                     dis_to_junction_min)));
  //         if_check_static_obs = true;
  //         break;
  //       }
  //     }
  //   }
  // }
  // double check_range = std::max(9 * plan_start_point.v(), 90.0);
  // if (lc_state.stage() != LaneChangeStage::LCS_NONE && if_check_static_obs &&
  //     lc_cmd_state == DriverAction::LC_CMD_NONE &&
  //     IsTargetLaneBlockerdByStaticObs(drive_passage, st_traj_mgr,
  //                                     check_range)) {
  //   Log2DDS::LogDataV2("block_debug", "return error code ");
  //   return absl::CancelledError("target lane blocked by static obs");
  // }
  TurnSignal planner_turn_signal = TURN_SIGNAL_NONE;
  TurnSignalReason turn_signal_reason = TURN_SIGNAL_OFF;
  // const auto [planner_turn_signal, turn_signal_reason] =
  //     DecideRoutePrepareLcTurnSignal(psmm, drive_passage, route_navi_info);

  // // discourage lc when closing to front obs with low speed
  // std::vector<std::string> tl_output_debug;
  // bool find_still_valid_obs = false;
  // for (const auto& obs : obj_mgr.planner_objects()) {
  //   ASSIGN_OR_CONTINUE(const auto aabbox,
  //                      drive_passage.QueryFrenetBoxAtContour(obs.contour()));
  //   double ds = aabbox.s_min - ego_frenet_box.s_max;
  //   double dl = aabbox.center_l() - ego_frenet_box.center_l();
  //   if (ds > 6.0 || ds < 0.0) continue;
  //   if (std::fabs(dl) > 1.0) continue;
  //   if (!obs.is_stationary() && ds > 3.0) continue;
  //   // kappa check
  //   if (lc_state.stage() != LaneChangeStage::LCS_NONE) {
  //     const double l_buffer = 1.0;
  //     const double kappa_buffer = 0.025;
  //     const auto ego_theta = plan_start_point.path_point().theta();
  //     const Vec2d ego_pt(plan_start_point.path_point().x(),
  //                        plan_start_point.path_point().y());
  //     const Vec2d unit_vec{cos(ego_theta), std::sin(ego_theta)};
  //     const Vec2d vec_1 =
  //         unit_vec * 0.5 * (aabbox.s_min - ego_frenet_box.center_s());
  //     if (lc_state.lc_left()) {
  //       const auto end_pt = drive_passage.QueryPointXYAtSL(
  //           aabbox.s_min, aabbox.l_max + l_buffer);
  //       const auto vec_2 = end_pt.value() - ego_pt;
  //       const double kappa = vec_1.CrossProd(vec_2) / vec_1.Length() /
  //                            vec_2.Length() / (0.5 * vec_2.Length());
  //       tl_output_debug.emplace_back("disable task cause of close obs left
  //       k:" +
  //                                    absl::StrCat(kappa));
  //       if (kappa < kappa_buffer) continue;
  //     } else {
  //       const auto end_pt = drive_passage.QueryPointXYAtSL(
  //           aabbox.s_min, aabbox.l_min - l_buffer);
  //       const auto vec_2 = end_pt.value() - ego_pt;
  //       const double kappa = vec_1.CrossProd(vec_2) / vec_1.Length() /
  //                            vec_2.Length() / (0.5 * vec_2.Length());
  //       tl_output_debug.emplace_back(
  //           "disable task cause of close obs right k:" +
  //           absl::StrCat(kappa));
  //       if (kappa > -kappa_buffer) continue;
  //     }
  //   }
  //   tl_output_debug.emplace_back("disable task cause of close obs:" +
  //   obs.id()); find_still_valid_obs = true; break;
  // }
  // if (false && find_still_valid_obs && drive_passage.lane_seq_info() &&
  //     !drive_passage.lane_seq_info()->is_current &&
  //     (lc_state.stage() != LaneChangeStage::LCS_NONE || borrow) &&
  //     lc_cmd_state == DriverAction::LC_CMD_NONE) {
  //   tl_output_debug.emplace_back(
  //       absl::StrCat("disable task cause of close obs:",
  //                    lc_state.stage() != LaneChangeStage::LCS_NONE, "/",
  //                    borrow));
  //   Log2DDS::LogDataV2("disable_task", tl_output_debug);
  //   return absl::CancelledError("target lane blocked by static obs");
  // }

  if (CheckDrivePassageBlockByTrafficObsOrCurbCross(
          task_idx, psmm, drive_passage, st_traj_mgr, ego_frenet_box,
          plan_start_point, psmm.map_ptr()->is_on_highway(),
          behavior.function_id(), target_lane_path_num, lc_state,
          construction_info, pre_lane_change_safety_info, intention_dir)) {
    LOG(INFO) << " schedule_obstacle_debug , "
              << absl::StrCat(Log2DDS::TaskPrefix(task_idx)) << ", true";
    // Log2DDS::LogDataV0("schedule_obstacle_debug",
    // absl::StrCat(Log2DDS::TaskPrefix(task_idx), " : True"));
    // SetDrivePassageDistToNaviEndAndStopLine(drive_passage);
  }

  if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
    ASSIGN_OR_RETURN(auto path_boundary,
                     BuildPathBoundaryFromPose(
                         psmm, drive_passage, plan_start_point, vehicle_geom,
                         st_traj_mgr, lc_state, smooth_result_map, borrow,
                         should_smooth, prev_lane_path_before_lc_from_start,
                         &cur_saved_offset, nullptr, false, &obj_history_mgr),
                     _ << "Fail to build path boundary.");

    return SchedulerOutput{
        .drive_passage = std::move(drive_passage),
        .sl_boundary = std::move(path_boundary),
        .lane_change_state = std::move(lc_state),
        .lc_reason = lc_reason,
        .length_along_route = lp_info.length_along_route(),
        .max_reach_length = current_lane_navi_dist,
        .lc_num = cur_lc_num,
        .leading_id = leading_id,
        .standard_congestion_factor = standard_congestion_factor,
        .traffic_congestion_factor = traffic_congestion_factor,
        .should_smooth = should_smooth,
        .borrow_lane = borrow,
        .av_frenet_box_on_drive_passage = ego_frenet_box,
        .request_help_lane_change_by_route = request_help_lane_change_by_route,
        .switch_alternate_route = switch_alternate_route,
        .planner_turn_signal = planner_turn_signal,
        .turn_signal_reason = turn_signal_reason,
        .miss_navi_scenario = is_miss_navi,
        .saved_offset = cur_saved_offset,
        .lane_obstacles_info = std::move(lane_obstacles_info)};
  }

  // const auto neighbor_lp_info_or =
  //     FindNeighbor(psmm, lp_infos, lc_state.lc_left(),
  //     lp_info.start_lane_id());

  const bool target_switched =
      prev_target_lane_path_from_start.front().lane_id() !=
      drive_passage.lane_path().front().lane_id();
  // single line need check safe
  // #if 0
  //   if (!neighbor_lp_info_or.ok() ||
  //       (neighbor_lp_info_or->max_reach_length() < kEpsilon &&
  //        plan_start_point.v() < kForceMergeMaxSpeed)) {
  //     // No lane path before lc can be found, have to force merge.
  //     LOG(WARNING) << "Applying force merge to lane " <<
  //     lp_info.start_lane_id()
  //                  << ", no safety check is applied!";
  //     lc_state.set_force_merge(true);
  //   }
  // #endif
  ASSIGN_OR_RETURN(auto path_boundary,
                   BuildPathBoundaryFromPose(
                       psmm, drive_passage, plan_start_point, vehicle_geom,
                       st_traj_mgr, lc_state, smooth_result_map, borrow,
                       should_smooth, prev_lane_path_before_lc_from_start,
                       &cur_saved_offset, nullptr, false, &obj_history_mgr),
                   _ << "Fail to build path boundary.");

  mapping::LanePath lane_path_before_lc;
  if (prev_lc_state.stage() == LaneChangeStage::LCS_NONE &&
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING) {
    // lane_path_before_lc = neighbor_lp_info_or.ok()
    //                           ? neighbor_lp_info_or->lane_path()
    //                           : mapping::LanePath();
    lane_path_before_lc = prev_target_lane_path_from_start;
  } else {
    lane_path_before_lc = prev_lane_path_before_lc_from_start;
  }

  return SchedulerOutput{
      .drive_passage = std::move(drive_passage),
      .sl_boundary = std::move(path_boundary),
      .lane_change_state = std::move(lc_state),
      .lc_reason = lc_reason,
      .lane_path_before_lc = std::move(lane_path_before_lc),
      .length_along_route = lp_info.length_along_route(),
      .max_reach_length = lp_info.max_reach_length(),
      .lc_num = cur_lc_num,
      .standard_congestion_factor = standard_congestion_factor,
      .traffic_congestion_factor = traffic_congestion_factor,
      .should_smooth = should_smooth,
      .borrow_lane = borrow,
      .av_frenet_box_on_drive_passage = ego_frenet_box,
      .request_help_lane_change_by_route = request_help_lane_change_by_route,
      .switch_alternate_route = switch_alternate_route,
      .planner_turn_signal = planner_turn_signal,
      .turn_signal_reason = turn_signal_reason,
      .saved_offset = cur_saved_offset,
      .lane_obstacles_info = std::move(lane_obstacles_info)};
}

st::mapping::LanePath PreProcessLanePathInfo(
    const MultiTasksSchedulerInput& input, const LanePathInfo& lp_info) {
  // std::set<std::string> navi_start_lanes;
  std::vector<st::mapping::ElementId> lane_id_infos;
  // auto& navi_start = input.psmm->map_ptr()->route()->navi_start();
  // ad_byd::planning::SectionPtr section;
  // if (!navi_start.section_id.empty()) {
  //   const auto& navi_section =
  //       input.psmm->map_ptr()->GetSectionById(navi_start.section_id);
  //   if (navi_section) {
  //     for (auto& id : navi_section->lanes()) {
  //       navi_start_lanes.insert(id);
  //     }
  //   }
  // }

  // bool find_start = false;
  // for (auto lane_id : lp_info.lane_path().lane_ids()) {
  //   if (find_start ||
  //       navi_start_lanes.find(lane_id) != navi_start_lanes.end() ||
  //       !input.is_navi) {
  //     lane_id_infos.emplace_back(lane_id);
  //     find_start = true;
  //   }
  // }
  const auto& nearest_lane = lp_info.lane_seq()->GetNearestLane(input.ego_pos);
  if (nearest_lane) {
    bool find_nearest = false;
    for (auto lane_id : lp_info.lane_path().lane_ids()) {
      if (find_nearest || nearest_lane->id() == lane_id) {
        lane_id_infos.emplace_back(lane_id);
        find_nearest = true;
      }
    }
  }

  // align ego_pos to lane_id_infos
  if (!lane_id_infos.empty()) {
    std::vector<ad_byd::planning::LaneConstPtr> lane_vector;
    for (const auto& lane_id : lane_id_infos) {
      const ad_byd::planning::LaneConstPtr lane_ptr =
          input.psmm->map_ptr()->GetLaneById(lane_id);
      // if (lane_ptr->IsValid()) lane_vector.emplace_back(lane_ptr);
      if (lane_ptr && lane_ptr->center_line().IsValid())
        lane_vector.emplace_back(lane_ptr);
    }
    auto target_lane_seq =
        std::make_shared<ad_byd::planning::LaneSequence>(lane_vector);

    std::vector<st::mapping::ElementId> lane_ids;
    const auto& nearest_lane = target_lane_seq->GetNearestLane(input.ego_pos);
    if (nearest_lane) {
      bool find_nearest = false;
      for (const auto& lane : target_lane_seq->lanes()) {
        if (lane && nearest_lane && nearest_lane->id() == lane->id()) {
          find_nearest = true;
        }
        if (!find_nearest) continue;
        // if (!lane || lane->id().empty() ||
        // !lane->center_line().IsValid()) break;
        lane_ids.emplace_back(lane->id());
      }
      lane_id_infos = lane_ids;
    }
  }

  double start_fraction = 0.0;
  if (!lane_id_infos.empty()) {
    const ad_byd::planning::LaneConstPtr start_lane =
        input.psmm->map_ptr()->GetLaneById(lane_id_infos.at(0));
    const auto ff = BuildBruteForceFrenetFrame(
        start_lane->points(), /*down_sample_raw_points=*/false);
    if (ff.ok()) {
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha;
      ff.value().XYToSL(input.ego_pos, &sl, &normal, &index_pair, &alpha);
      sl.s = std::max(0.0, sl.s);
      start_fraction = std::clamp(
          sl.s / std::fmax(start_lane->curve_length(), 1e-2), 0.0, 1.0 - 1e-2);
    }
  }

  st::mapping::LanePath lane_path(input.psmm->map_ptr(), lane_id_infos,
                                  start_fraction, 1.0);
  return lane_path;
}

absl::StatusOr<std::vector<SchedulerOutput>> ScheduleMultiplePlanTasks(
    const MultiTasksSchedulerInput& input,
    const std::vector<LanePathInfo>& target_lp_infos, ThreadPool* thread_pool) {
  SCOPED_TRACE(__FUNCTION__);
  TIMELINE("ScheduleMultiplePlanTasks");

  // DLOG(INFO) << "Input target lane path num for scheduler: "
  //            << target_lp_infos.size();
  Log2DDS::LogDataV0("schedule_debug",
                     absl::StrCat("lp info size: ", target_lp_infos.size()));

  std::vector<SchedulerOutput> multi_tasks;
  std::string output_fail_message;
  // for debug
  for (int i = 0; i < target_lp_infos.size(); i++) {
    Log2DDS::LogDataV2(
        "lane_path",
        target_lp_infos[i].lane_path().DebugString() +
            (target_lp_infos[i].lane_seq_info()
                 ? target_lp_infos[i].lane_seq_info()->DebugString()
                 : " lane_seq_info is nullptr!"));
  }
  if (target_lp_infos.size() == 1) {
    const auto& lp_info = target_lp_infos.front();
    const auto lane_seq_info = lp_info.lane_seq_info();
    const auto backward_extended_lane_path = lp_info.lane_path();
    const auto lane_path = PreProcessLanePathInfo(input, lp_info);
    ad_byd::planning::LcReason lc_reason = ad_byd::planning::LC_REASON_NONE;
    if (lane_seq_info) {
      lc_reason = lane_seq_info->lc_reason;
    }
    bool has_borrow_task = false;
    auto lane_ptr = lane_path.IsValid(input.psmm->map_ptr())
                        ? input.psmm->FindLaneByIdOrNull(lane_path.lane_id(0))
                        : nullptr;
    if (lane_ptr) {
      if (lane_ptr->right_lane_id() == 0) {
        if (lane_ptr->type() != ad_byd::planning::LANE_VIRTUAL_JUNCTION &&
            lane_ptr->type() != ad_byd::planning::LANE_VIRTUAL_COMMON) {
          has_borrow_task = true;
        } else if (lane_ptr->turn_type() ==
                   ad_byd::planning::TurnType::RIGHT_TURN) {
          has_borrow_task = true;
        }
      } else {
        const auto& right_lane =
            input.psmm->map_ptr()->GetLaneById(lane_ptr->right_lane_id());
        if (right_lane &&
            right_lane->type() == ad_byd::planning::LANE_EMERGENCY) {
          has_borrow_task = true;
        }
      }
    }

    auto drive_passage_or = BuildDrivePassageV2(
        *input.psmm, lane_path, backward_extended_lane_path,
        *input.station_anchor, input.planning_horizon, *input.destination,
        FLAGS_planner_consider_all_lanes_virtual, input.cruising_speed_limit,
        FrenetFrameType::kQtfmKdTree, lane_seq_info);
    if (!drive_passage_or.ok()) {
      Log2DDS::LogDataV0(
          "schedule_debug",
          absl::StrCat("fail", ", start lane: ", lp_info.start_lane_id(),
                       ", info: ", drive_passage_or.status().ToString()));

      return absl::NotFoundError(absl::StrCat(
          "Fail to build drive passage on single target lane path.",
          " info: ", drive_passage_or.status().ToString()));
    }
    DumpDrivePassageToDebugFrame(*drive_passage_or, 0);
    const bool should_smooth = true;
    //  ShouldSmoothRefLane(
    //     *input.tl_info_map, drive_passage, input.prev_smooth_state);

    const std::vector<bool> borrow_branches =
        (FLAGS_planner_est_scheduler_allow_borrow && has_borrow_task)
            ? std::vector<bool>{false, true}
            : std::vector<bool>{false};
    int i = 0;
    for (bool borrow : borrow_branches) {
      auto output_or = MakeSchedulerOutput(
          i, *input.psmm, *input.lane_path_infos, drive_passage_or.value(),
          lp_info, *input.vehicle_geom, *input.st_traj_mgr, *input.obj_mgr,
          *input.obj_history_mgr, *input.plan_start_point,
          *input.smooth_result_map, *input.prev_target_lane_path_from_start,
          *input.prev_lane_path_before_lc_from_start,
          *input.preferred_lane_path, *input.prev_lc_state, borrow,
          should_smooth, input.planner_is_l4_mode, *input.behavior,
          target_lp_infos.size(), input.miss_navi_scenario,
          input.continuous_lc_scenario, input.lc_cmd_state, lc_reason,
          input.leading_id, input.lc_push_dir,
          input.is_going_force_route_change_left, input.pre_push_status,
          input.stalled_objects, input.saved_offset, input.construction_info,
          input.pre_lane_change_safety_info, input.intention_dir);

      i++;
      // auto ClassifyConstructionObstaclesByPosition =
      //     [](const ApolloTrajectoryPointProto* plan_start_point,
      //        const PlannerObjectManager* obj_mgr,
      //        const ad_byd::planning::LaneSequencePtr& target_lane_seq) {

      //     };
      if (output_or.ok()) {
        if (input.lc_cmd_state == DriverAction::LC_CMD_NONE &&
            output_or.value().lane_change_state.stage() !=
                LaneChangeStage::LCS_NONE &&
            !CheckRoadBoundaryBySInterval(
                input, output_or.value(),
                input.prev_lane_path_before_lc_from_start->lane_seq(),
                lane_path.lane_seq())) {
          Log2DDS::LogDataV0(
              "schedule_debug",
              absl::StrCat("lc check roadboundary fail",
                           ", start lane before lc: ",
                           input.prev_lane_path_before_lc_from_start->lane_seq()
                               ->lanes()
                               .front()
                               ->id(),
                           ", start lane: ", lp_info.start_lane_id(),
                           ", is borrow: ", borrow));
          output_fail_message += "lc check roadboundary fail.";
          continue;
        }
        Log2DDS::LogDataV0(
            "schedule_debug",
            absl::StrCat("ok", ", start lane: ", lp_info.start_lane_id()));
        // Log2DDS::LogDataV0("schedule_debug",
        //                    "start lane: " + lp_info.start_lane_id());
        multi_tasks.emplace_back(std::move(output_or).value());
      } else {
        Log2DDS::LogDataV0(
            "schedule_debug",
            absl::StrCat("fail", ", start lane: ", lp_info.start_lane_id(),
                         ", is borrow: ", borrow,
                         ", info: ", output_or.status().ToString()));

        output_fail_message += output_or.status().ToString();
      }
    }
  } else {
    // if vehicle in right_most and pnp top1-first-lane_id is equal to
    // top2-first-lane_id
    bool has_borrow_task = false;
    size_t borrow_task_idx = 0;
    // if (!input.prev_target_lane_path_from_start->IsEmpty() &&
    //     target_lp_infos.size() == 2) {
    //   const auto lane_path =
    //       PreProcessLanePathInfo(input, target_lp_infos.front());
    //   if (lane_path.IsValid(input.psmm->map_ptr()) &&
    //       input.prev_target_lane_path_from_start->front().lane_id() ==
    //           lane_path.lane_id(0)) {
    //     auto lane_ptr = input.psmm->FindLaneByIdOrNull(lane_path.lane_id(0));
    //     if (lane_ptr && lane_ptr->right_lane_id() == 0 &&
    //         lane_ptr->type() != ad_byd::planning::LANE_VIRTUAL_JUNCTION &&
    //         lane_ptr->type() != ad_byd::planning::LANE_VIRTUAL_COMMON) {
    //       has_borrow_task = true;
    //     }
    //   }
    // }
    // if (!has_borrow_task && target_lp_infos.size() == 2) {
    //   const auto first_lane_path =
    //       PreProcessLanePathInfo(input, target_lp_infos.front());
    //   const auto second_lane_path =
    //       PreProcessLanePathInfo(input, target_lp_infos.back());
    //   if (first_lane_path.IsValid(input.psmm->map_ptr()) &&
    //       second_lane_path.IsValid(input.psmm->map_ptr()) &&
    //       first_lane_path.lane_id(0) == second_lane_path.lane_id(0)) {
    //     auto lane_ptr =
    //         input.psmm->FindLaneByIdOrNull(first_lane_path.lane_id(0));
    //     if (lane_ptr && lane_ptr->right_lane_id() == 0 &&
    //         lane_ptr->type() != ad_byd::planning::LANE_VIRTUAL_JUNCTION &&
    //         lane_ptr->type() != ad_byd::planning::LANE_VIRTUAL_COMMON) {
    //       has_borrow_task = true;
    //     }
    //   }
    // }

    if (target_lp_infos.size() == 2) {
      auto check_borrow = [&](size_t idx) {
        const auto lane_path =
            PreProcessLanePathInfo(input, target_lp_infos.at(idx));
        if (lane_path.IsValid(input.psmm->map_ptr())) {
          auto lane_ptr = input.psmm->FindLaneByIdOrNull(lane_path.lane_id(0));
          if (!lane_ptr) return;
          bool need_borrow = false;
          if (lane_ptr->right_lane_id() == 0) {
            if (lane_ptr->type() != ad_byd::planning::LANE_VIRTUAL_JUNCTION &&
                lane_ptr->type() != ad_byd::planning::LANE_VIRTUAL_COMMON) {
              need_borrow = true;
            } else if (lane_ptr->turn_type() ==
                       ad_byd::planning::TurnType::RIGHT_TURN) {
              need_borrow = true;
            }
          } else {
            const auto& right_lane =
                input.psmm->map_ptr()->GetLaneById(lane_ptr->right_lane_id());
            if (right_lane &&
                right_lane->type() == ad_byd::planning::LANE_EMERGENCY) {
              need_borrow = true;
            }
          }

          if (need_borrow) {
            has_borrow_task = true;
            borrow_task_idx = idx;
          }
        }
      };

      if (input.psmm && input.psmm->IsOnHighway()) {
        if (input.prev_lc_state && input.prev_lc_state->stage() == LCS_NONE) {
          // when lane keep status, the first lane path is the current lane
          // path, no matter in borrow status or not. If the current lane is the
          // rightmost lane, then generate a borrow task.
          check_borrow(0);
        }
      } else {
        check_borrow(0);
        if (!has_borrow_task) {
          check_borrow(1);
        }
      }
    }

    // has_borrow_task =
    //     false;  // disable borrow task when target_lp_infos size == 2

    std::vector<absl::StatusOr<SchedulerOutput>> outputs(
        target_lp_infos.size());
    if (has_borrow_task) {
      outputs.resize(target_lp_infos.size() + 1);
    }
    std::set<int> road_check_fail_idx;
    ParallelFor(0, target_lp_infos.size(), thread_pool, [&](int i) {
      const auto& lp_info = target_lp_infos[i];
      const auto lane_seq_info = lp_info.lane_seq_info();
      const auto backward_extended_lane_path = &lp_info.lane_path();
      const auto lane_path = PreProcessLanePathInfo(input, lp_info);
      ad_byd::planning::LcReason lc_reason = ad_byd::planning::LC_REASON_NONE;
      if (lane_seq_info) {
        lc_reason = lane_seq_info->lc_reason;
      }

      auto drive_passage_or = BuildDrivePassageV2(
          *input.psmm, lane_path, *backward_extended_lane_path,
          *input.station_anchor, input.planning_horizon, *input.destination,
          FLAGS_planner_consider_all_lanes_virtual, input.cruising_speed_limit,
          FrenetFrameType::kQtfmKdTree, lane_seq_info);
      if (!drive_passage_or.ok()) {
        outputs[i] = absl::NotFoundError(drive_passage_or.status().ToString());
        return;
      }
      DumpDrivePassageToDebugFrame(*drive_passage_or, i);
      const bool should_smooth = true;
      //  ShouldSmoothRefLane(
      //     *input.tl_info_map, *drive_passage_or,
      //     input.prev_smooth_state);
      outputs[i] = MakeSchedulerOutput(
          i, *input.psmm, *input.lane_path_infos,
          std::move(drive_passage_or).value(), lp_info, *input.vehicle_geom,
          *input.st_traj_mgr, *input.obj_mgr, *input.obj_history_mgr,
          *input.plan_start_point, *input.smooth_result_map,
          *input.prev_target_lane_path_from_start,
          *input.prev_lane_path_before_lc_from_start,
          *input.preferred_lane_path, *input.prev_lc_state,
          /*borrow=*/false, should_smooth, input.planner_is_l4_mode,
          *input.behavior, target_lp_infos.size(), input.miss_navi_scenario,
          input.continuous_lc_scenario, input.lc_cmd_state, lc_reason,
          input.leading_id, input.lc_push_dir,
          input.is_going_force_route_change_left, input.pre_push_status,
          input.stalled_objects, input.saved_offset, input.construction_info,
          input.pre_lane_change_safety_info, input.intention_dir);
      if (outputs[i].ok()) {
        if (input.lc_cmd_state == DriverAction::LC_CMD_NONE &&
            outputs[i].value().lane_change_state.stage() !=
                LaneChangeStage::LCS_NONE &&
            !CheckRoadBoundaryBySInterval(
                input, outputs[i].value(),
                input.prev_lane_path_before_lc_from_start->lane_seq(),
                lane_path.lane_seq())) {
          Log2DDS::LogDataV0(
              "schedule_debug",
              absl::StrCat(Log2DDS::TaskPrefix(i), "lc check roadboundary fail",
                           ", start lane before lc: ",
                           input.prev_lane_path_before_lc_from_start->lane_seq()
                               ->lanes()
                               .front()
                               ->id(),
                           ", start lane: ", lp_info.start_lane_id(),
                           ", no borrow."));
          output_fail_message +=
              Log2DDS::TaskPrefix(i) + "lc check roadboundary fail.";
          road_check_fail_idx.insert(i);
        }
      }
    });

    if (has_borrow_task) {
      const auto& lp_info = target_lp_infos.at(borrow_task_idx);
      const auto lane_seq_info = lp_info.lane_seq_info();
      const auto backward_extended_lane_path = &lp_info.lane_path();
      const auto lane_path = PreProcessLanePathInfo(input, lp_info);
      ad_byd::planning::LcReason lc_reason = ad_byd::planning::LC_REASON_NONE;
      if (lane_seq_info) {
        lc_reason = lane_seq_info->lc_reason;
      }
      auto drive_passage_or = BuildDrivePassageV2(
          *input.psmm, lane_path, *backward_extended_lane_path,
          *input.station_anchor, input.planning_horizon, *input.destination,
          FLAGS_planner_consider_all_lanes_virtual, input.cruising_speed_limit,
          FrenetFrameType::kQtfmKdTree, lane_seq_info);

      if (!drive_passage_or.ok()) {
        outputs[target_lp_infos.size()] =
            absl::NotFoundError(drive_passage_or.status().ToString());
      } else {
        DumpDrivePassageToDebugFrame(*drive_passage_or, target_lp_infos.size());
        const bool should_smooth = true;
        //  ShouldSmoothRefLane(
        //     *input.tl_info_map, *drive_passage_or, input.prev_smooth_state);

        outputs[target_lp_infos.size()] = MakeSchedulerOutput(
            target_lp_infos.size(), *input.psmm, *input.lane_path_infos,
            std::move(drive_passage_or).value(), lp_info, *input.vehicle_geom,
            *input.st_traj_mgr, *input.obj_mgr, *input.obj_history_mgr,
            *input.plan_start_point, *input.smooth_result_map,
            *input.prev_target_lane_path_from_start,
            *input.prev_lane_path_before_lc_from_start,
            *input.preferred_lane_path, *input.prev_lc_state, /*borrow=*/true,
            should_smooth, input.planner_is_l4_mode, *input.behavior,
            target_lp_infos.size(), input.miss_navi_scenario,
            input.continuous_lc_scenario, input.lc_cmd_state, lc_reason,
            input.leading_id, input.lc_push_dir,
            input.is_going_force_route_change_left, input.pre_push_status,
            input.stalled_objects, input.saved_offset, input.construction_info,
            input.pre_lane_change_safety_info, input.intention_dir);
        if (outputs[target_lp_infos.size()].ok()) {
          if (input.lc_cmd_state == DriverAction::LC_CMD_NONE &&
              !CheckRoadBoundaryBySInterval(
                  input, outputs[target_lp_infos.size()].value(),
                  input.prev_lane_path_before_lc_from_start->lane_seq(),
                  lane_path.lane_seq())) {
            Log2DDS::LogDataV0(
                "schedule_debug",
                absl::StrCat(
                    Log2DDS::TaskPrefix(target_lp_infos.size()),
                    "lc check roadboundary fail", ", start lane before lc: ",
                    input.prev_lane_path_before_lc_from_start->lane_seq()
                        ->lanes()
                        .front()
                        ->id(),
                    ", start lane: ", lp_info.start_lane_id(), ", borrow."));
            output_fail_message += Log2DDS::TaskPrefix(target_lp_infos.size()) +
                                   "lc check roadboundary fail.";
            road_check_fail_idx.insert(target_lp_infos.size());
          }
        }
      }
    }

    for (int i = 0; i < outputs.size(); ++i) {
      if (outputs[i].ok()) {
        if (road_check_fail_idx.count(i)) continue;
        Log2DDS::LogDataV0(
            "schedule_debug",
            absl::StrCat(
                Log2DDS::TaskPrefix(i), "ok", ", start lane: ",
                (i < target_lp_infos.size()
                     ? target_lp_infos[i].start_lane_id()
                     : target_lp_infos[borrow_task_idx].start_lane_id())));

        multi_tasks.emplace_back(std::move(outputs[i]).value());
        // DLOG(INFO) << "(In scheduler) task " << i << " build succeeded.";
      } else {
        Log2DDS::LogDataV0(
            "schedule_debug",
            absl::StrCat(
                Log2DDS::TaskPrefix(i), "fail", ", start lane: ",
                (i < target_lp_infos.size()
                     ? target_lp_infos[i].start_lane_id()
                     : target_lp_infos[borrow_task_idx].start_lane_id()),
                ", info: ", outputs[i].status().ToString()));
        output_fail_message +=
            Log2DDS::TaskPrefix(i) + outputs[i].status().ToString();
      }
    }
  }

  if (multi_tasks.empty()) {
    LOG_ERROR << "Unable to schedule multiple tasks." << output_fail_message;
    return absl::NotFoundError(
        std::string("Fail to build drive passage on each lane path.") +
        output_fail_message);
  }

  DLOG(INFO) << "(In scheduler) scheduled tasks size before seperate lc_pause."
             << multi_tasks.size();

  if (FLAGS_planner_est_scheduler_seperate_lc_pause &&
      input.prev_lc_state->stage() != LaneChangeStage::LCS_NONE) {
    absl::StatusOr<SchedulerOutput> lc_pause_scheduler;
    for (const auto& output : multi_tasks) {
      if (!output.borrow_lane &&
          output.lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING) {
        lc_pause_scheduler = MakeLcPauseSchedulerOutput(
            *input.psmm, output, *input.vehicle_geom, *input.st_traj_mgr,
            *input.plan_start_point, *input.smooth_result_map,
            *input.prev_lane_path_before_lc_from_start);
        break;
      }
    }
    if (lc_pause_scheduler.ok()) {
      multi_tasks.push_back(std::move(lc_pause_scheduler).value());
    }
  }

  return multi_tasks;
}
bool CheckRoadBoundaryBySInterval(
    const MultiTasksSchedulerInput& input, const SchedulerOutput& output,
    const ad_byd::planning::LaneSequencePtr& current_lane_seq,
    const ad_byd::planning::LaneSequencePtr& target_lane_seq) {
  if (!current_lane_seq || !target_lane_seq) {
    return true;
  }

  const ad_byd::planning::BehaviorCommand& lc_command =
      output.lane_change_state.lc_left()
          ? ad_byd::planning::Command_LaneChangeLeft
          : ad_byd::planning::Command_LaneChangeRight;
  const auto& psmm = *input.psmm;
  const auto& smm = *psmm.map_ptr();
  const auto& map = psmm.map_ptr();
  const auto& plan_start_point = *input.plan_start_point;
  const auto& pre_lc_state = input.prev_lc_state;
  if (pre_lc_state->stage() == LaneChangeStage::LCS_EXECUTING &&
      output.lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING &&
      pre_lc_state->lc_left() == output.lane_change_state.lc_left()) {
    return true;
  }
  const Vec2d start_point =
      Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  // 1. Get lane_width.
  double current_lane_width = ad_byd::planning::Constants::DEFAULT_LANE_WIDTH;
  ad_byd::planning::LaneConstPtr nearest_lane =
      current_lane_seq->GetNearestLane(
          ad_byd::planning::math::Vec2d(start_point.x(), start_point.y()));
  if (nearest_lane) {
    current_lane_width =
        nearest_lane->GetWidthAtPoint(start_point.x(), start_point.y());
  }
  double target_lane_width = ad_byd::planning::Constants::DEFAULT_LANE_WIDTH;
  const auto last_nearest_lane = nearest_lane;
  nearest_lane = target_lane_seq->GetNearestLane(
      ad_byd::planning::math::Vec2d(start_point.x(), start_point.y()));
  if (nearest_lane) {
    target_lane_width =
        nearest_lane->GetWidthAtPoint(start_point.x(), start_point.y());
  }
  if (nearest_lane && last_nearest_lane &&
      (last_nearest_lane->id() == nearest_lane->id()))
    return true;
  // 2. Compute lanechange lat_l and preview_time
  const double kPreviewDist = 30.0;  // m
  double target_s_offset = 0.0;
  target_lane_seq->GetProjectionDistance(start_point, &target_s_offset);
  const auto target_pt =
      target_lane_seq->GetPointAtS(target_s_offset + kPreviewDist);
  const double l_dist =
      current_lane_seq->GetProjectionDistance(target_pt, nullptr);
  const double preview_time =  // lerp(t_min, lat_min, t_max, lat_max, lat_l)
      Lerp(2.0, 3.0, 5.0, 7.0, std::fabs(l_dist), true);
  // 3. Sample current lane center points.
  const double kCheckInterval = 1.0, kBehindDist = 2.0;
  double distance = std::fmax(plan_start_point.v() * preview_time, 10.0);
  if (output.lane_change_state.stage() != LaneChangeStage::LCS_NONE) {
    distance -= 2.0;  // distance hysteresis.
  }
  std::vector<ad_byd::planning::Point2d> sample_target_points,
      sample_current_points;
  target_lane_seq->SamplePoints(std::fmax(target_s_offset - kBehindDist, 0.0),
                                &sample_target_points);
  ad_byd::planning::math::LineCurve2d target_line(sample_target_points);
  std::vector<Vec2d> current_center_pts, target_center_pts;
  target_line.SamplePoints(0.0, distance + kBehindDist + 2.0,
                           &target_center_pts, kCheckInterval);
  double current_s_offset = 0.0;
  current_lane_seq->GetProjectionDistance(start_point, &current_s_offset);

  current_lane_seq->SamplePoints(std::fmax(current_s_offset - kBehindDist, 0.0),
                                 &sample_current_points);
  ad_byd::planning::math::LineCurve2d current_line(sample_current_points);

  current_line.SamplePoints(0.0, distance + kBehindDist, &current_center_pts,
                            kCheckInterval);
  // Log2DDS::LogLineV1("current_center_pts", Log2DDS::kOrange, {},
  //                    current_center_pts);
  // Log2DDS::LogLineV1("target_center_pts", Log2DDS::kRed, {},
  // target_center_pts);
  // 4. Get current lane RoadBoundary segments.
  const double kCurrentLatBuffer = 1.5;
  std::pair<double, double> lat_dist_range = {0.0, 0.0};
  std::vector<ad_byd::planning::math::LineSegment2d> road_segs;
  if (output.lane_change_state.stage() != LaneChangeStage::LCS_NONE &&
      lc_command == ad_byd::planning::BehaviorCommand::Command_LaneChangeLeft) {
    lat_dist_range = {0.5 * current_lane_width + kCurrentLatBuffer, 0.0};
    current_lane_seq->GetRoadSegments(true, &road_segs);
  } else if (output.lane_change_state.stage() != LaneChangeStage::LCS_NONE &&
             lc_command ==
                 ad_byd::planning::BehaviorCommand::Command_LaneChangeRight) {
    lat_dist_range = {0.0, -0.5 * current_lane_width - kCurrentLatBuffer};
    current_lane_seq->GetRoadSegments(false, &road_segs);
  }
  // road_segs); Get ClearArea Segments
  for (const auto& lane : current_lane_seq->lanes()) {
    if (!lane) continue;
    for (const uint64_t id : lane->overlap_clear_areas()) {
      const ad_byd::planning::ClearAreaConstPtr clear_area =
          map->GetClearAreaById(id);
      if (!clear_area ||
          clear_area->type() ==
              ad_byd::planning::ImpassableAeraType::LINEAR_SAFE_ISLAND) {
        continue;
      }
      const auto area_segs = clear_area->polygon().line_segments();
      road_segs.insert(road_segs.end(), area_segs.begin(), area_segs.end());
    }
  }
  std::vector<ad_byd::planning::Point2d> road_pts;
  if (ad_byd::planning::RoadBoundary::GetRoadBoundaryPoints(
          current_center_pts, road_segs, lat_dist_range, &road_pts)) {
    // Log2DDS::LogLineV1("lc_forbid_road_pts", Log2DDS::kOrange, {}, road_pts);
    return false;
  }
  // 5. Check target lane RoadBoundary segments.
  const double kTargetLatBuffer = 1.5;
  road_segs.clear();
  if (output.lane_change_state.stage() != LaneChangeStage::LCS_NONE &&
      lc_command == ad_byd::planning::BehaviorCommand::Command_LaneChangeLeft) {
    lat_dist_range = {0.0, -0.5 * target_lane_width - kTargetLatBuffer};
    target_lane_seq->GetRoadSegments(false, &road_segs);
  } else if (output.lane_change_state.stage() != LaneChangeStage::LCS_NONE &&
             lc_command ==
                 ad_byd::planning::BehaviorCommand::Command_LaneChangeRight) {
    lat_dist_range = {0.5 * target_lane_width + kTargetLatBuffer, 0.0};
    target_lane_seq->GetRoadSegments(true, &road_segs);
  }
  // Get ClearArea Segments
  for (const auto& lane : target_lane_seq->lanes()) {
    if (!lane) continue;
    for (const uint64_t id : lane->overlap_clear_areas()) {
      const ad_byd::planning::ClearAreaConstPtr clear_area =
          map->GetClearAreaById(id);
      if (!clear_area ||
          clear_area->type() ==
              ad_byd::planning::ImpassableAeraType::LINEAR_SAFE_ISLAND) {
        continue;
      }
      const auto area_segs = clear_area->polygon().line_segments();
      road_segs.insert(road_segs.end(), area_segs.begin(), area_segs.end());
    }
  }
  if (ad_byd::planning::RoadBoundary::GetRoadBoundaryPoints(
          target_center_pts, road_segs, lat_dist_range, &road_pts)) {
    // LOGVIZ("lc_forbid_road_pts", 0, Pink, road_pts);
    return false;
  }
  return true;
}
}  // namespace st::planning
