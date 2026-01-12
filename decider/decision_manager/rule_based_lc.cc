#include "plan_common/log_data.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
#include "object_manager/planner_object_manager.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/trajectory_util.h"
//#include "decider/scene_manager/construction_scene_identification.h"
//#include "decider/scheduler/driving_map_topo_builder.h"
#include "modules/cnoa_pnc/planning/proto/selector_debug.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "decider/decision_manager/rule_based_lc.h"
//#include "decider/scheduler/candidate_lane_sequences.h"
//#include "decider/selector/selector.h"
#include "object_manager/st_inference/selector_input.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st::planning {
bool IsValidLane(const LaneConstPtr &lane) {
  if (!lane || lane->type() == LaneType::LANE_NON_MOTOR ||
      lane->type() == LaneType::LANE_EMERGENCY) {
    return false;
  }
  return true;
}

void CountNaviAndEfficiencyInfo(const Vec2d &ego_pos,
                                const LaneSequencePtr &laneseq,
                                const PlannerObjectManager &obj_mgr,
                                int *navi_section_cnt, double *average_speed) {
  if (!laneseq) return;
  // navigation
  int navi_cnt = 0;
  for (const auto &lane : laneseq->lanes()) {
    if (!lane->is_navigation()) break;
    navi_cnt++;
  }
  if (navi_section_cnt != nullptr) *navi_section_cnt = navi_cnt;
  // efficiency
  double avg_speed = 0.0;
  int valid_obs_cnt = 0;
  double ego_s = 0.0;
  laneseq->GetProjectionDistance(ego_pos, &ego_s);
  for (const auto &obs : obj_mgr.planner_objects()) {
    const auto obs_pos = obs.pose().pos();
    double obs_s = 0.0;
    double obs_l = laneseq->GetProjectionDistance(obs_pos, &obs_s);
    if (obs_l > 2.0 || obs_s < ego_s) continue;
    avg_speed += obs.pose().v();
    valid_obs_cnt++;
  }
  if (valid_obs_cnt > 0 && average_speed != nullptr) {
    *average_speed = avg_speed / valid_obs_cnt;
  }
}

bool IfOnSameLaneSequence(const ad_byd::planning::MapPtr &map,
                          const ad_byd::planning::LaneConstPtr &start,
                          const ad_byd::planning::LaneConstPtr &target) {
  std::unordered_set<uint64_t> lanes;
  lanes.clear();
  return IfOnSameLaneSequence(map, start, target, lanes);
}

bool IfOnSameLaneSequence(const ad_byd::planning::MapPtr &map,
                          const ad_byd::planning::LaneConstPtr &start,
                          const ad_byd::planning::LaneConstPtr &target,
                          std::unordered_set<uint64_t> &lanes) {
  if (!map || !start || !target) {
    return false;
  }
  if (start->id() == target->id()) {
    return true;
  }
  for (const auto &lane_id : start->next_lane_ids()) {
    if (lanes.count(lane_id) > 0) {
      break;
    }
    lanes.insert(lane_id);
    const auto &lane = map->GetLaneById(lane_id);
    if (lane && IfOnSameLaneSequence(map, lane, target, lanes)) {
      return true;
    }
  }
  return false;
}

bool IfLaneSeqWillMergeCurLane(
    const ad_byd::planning::LaneSequencePtr &lane_seq,
    const ad_byd::planning::LaneConstPtr &current_lane,
    const Vec2d &start_point, const ad_byd::planning::MapPtr &map,
    double &merge_dist) {
  if (!lane_seq || !current_lane || !map) {
    merge_dist = std::numeric_limits<double>::max();
    return false;
  }
  double s_offset = 0.0;
  const double l = lane_seq->GetProjectionDistance(
      {start_point.x(), start_point.y()}, &s_offset);
  const auto projection_point = lane_seq->GetPointAtS(s_offset);
  ad_byd::planning::LaneConstPtr merge_lane = nullptr;
  ad_byd::planning::LaneConstPtr merged_lane = nullptr;
  merge_dist = lane_seq->GetDistanceToPOI(ad_byd::planning::Poi_Merge,
                                          projection_point.x(),
                                          projection_point.y(), merge_lane);
  if (merge_lane) {
    if (merge_lane->merge_topology() == ad_byd::planning::TOPOLOGY_MERGE_LEFT) {
      merged_lane = map->GetLeftLane(merge_lane);
    } else if (merge_lane->merge_topology() ==
               ad_byd::planning::TOPOLOGY_MERGE_RIGHT) {
      merged_lane = map->GetRightLane(merge_lane);
    }
  }
  return IfOnSameLaneSequence(map, current_lane, merged_lane);
}

bool IfSectionSizeLargerThanTwo(
    double check_range, const ad_byd::planning::MapPtr &map,
    const ad_byd::planning::NaviPosition &navi_start) {
  double dist_to_section_size_two = 0.0;
  if (navi_start.section_id == 0) {
    return false;
  }
  bool find_nearest = false;
  const auto &route_section_seq = map->GetRouteSectionSeq();
  for (const auto &section_id : route_section_seq) {
    if (section_id == 0) break;
    // const ad_byd::planning::SectionInfo& section : sections;
    ad_byd::planning::SectionConstPtr section_ptr =
        map->GetSectionById(section_id);
    if (dist_to_section_size_two > check_range) {
      break;
    }
    if (!find_nearest && section_ptr->id() == navi_start.section_id) {
      dist_to_section_size_two = -navi_start.s_offset;
      find_nearest = true;
    }
    if (!find_nearest) {
      continue;
    }
    dist_to_section_size_two += section_ptr->topo_length();
    if (map->GetNormalLaneSize(section_ptr) <= 2) {
      return false;
    }
  }
  return true;
}

bool IfLcByVariableTurn(
    const ad_byd::planning::LaneSequencePtr &cur_lane_sequence,
    const ad_byd::planning::LaneSequencePtr &left_lane_seq,
    const ad_byd::planning::LaneSequencePtr &right_lane_seq,
    const ad_byd::planning::LaneSeqInfo &cur_lane_seq_info, int &lc_direction,
    double &dist_to_target_lane, LaneType lane_type,
    const ad_byd::planning::MapPtr &map) {
  bool if_near_variable_turn = false;
  // 1. Check if cur lane seq exists bus lane
  if (!cur_lane_sequence) {
    return false;
  }
  ad_byd::planning::LaneConstPtr variable_turn_cur_seq = nullptr;
  std::unordered_set<LaneType> lane_type_set{lane_type};
  double dist_to_variable_turn = cur_lane_sequence->GetDistanceToTargetLaneType(
      variable_turn_cur_seq, lane_type_set, map->route()->navi_start());
  dist_to_target_lane = dist_to_variable_turn;
  // LOGDATA("center_intent",
  //         absl::StrCat("[IfLcByVariableTurn]: dist_to_variable_turn: ",
  //                      dist_to_variable_turn));
  // LOGDATA("center_intent",
  //         absl::StrCat("[IfLcByVariableTurn]:
  //         map_supplement.dist_to_junction: ",
  //                      map_supplement.dist_to_junction));
  double variable_turn_lc_threshold = 250.0;
  if (variable_turn_cur_seq &&
      dist_to_variable_turn < variable_turn_lc_threshold &&
      cur_lane_seq_info.dist_to_junction > 80) {
    if_near_variable_turn = true;
  } else {
    return false;  // no need lc
  }
  //-----------------------------------------------
  // 2. Check variable turn position
  const auto cur_section_ptr = cur_lane_seq_info.cur_section_ptr;
  int start_index = 0;
  const auto &route_section_seq = map->GetRouteSectionSeq();
  if (cur_section_ptr) {
    for (int i = 0; i < route_section_seq.size(); i++) {
      const auto &section_ptr = map->GetSectionById(route_section_seq.at(i));
      if (section_ptr && section_ptr->id() == cur_section_ptr->id()) {
        start_index = i;
      }
    }
  }
  double total_length = 0.0;
  int variable_turn_pos = 0;
  int variable_turn_section_size = 0;
  int special_lane_number = 0;
  bool if_found = false;
  std::vector<ad_byd::planning::LaneConstPtr> legal_lanes;
  while (start_index < route_section_seq.size() &&
         total_length < variable_turn_lc_threshold && !if_found) {
    // For each section
    // const auto check_section = map->route()->sections().at(start_index);
    const auto check_section_ptr =
        map->GetSectionById(route_section_seq.at(start_index));
    int lanes_number = check_section_ptr->lanes().size();
    int pos_count = 0;
    special_lane_number = 0;
    legal_lanes.clear();
    // For each lane in cur checking section
    for (auto iter = check_section_ptr->lanes().begin();
         iter != check_section_ptr->lanes().end(); iter++) {
      pos_count += 1;
      // cur section lane type check
      LaneConstPtr check_lane = map->GetLaneById((*iter));
      if (!check_lane || check_lane->type() == ad_byd::planning::LANE_UNKNOWN ||
          check_lane->type() == ad_byd::planning::LANE_NON_MOTOR ||
          !check_lane->center_line().IsValid()) {
        lanes_number -= 1;
        pos_count -= 1;
        continue;
      }
      if (check_lane->type() == ad_byd::planning::LANE_VIRTUAL_JUNCTION) {
        return false;
      }
      if (check_lane->type() == ad_byd::planning::LANE_BUS_NORMAL ||
          check_lane->type() == ad_byd::planning::LANE_EMERGENCY ||
          check_lane->type() == ad_byd::planning::LANE_HOV_NORMAL ||
          check_lane->type() == ad_byd::planning::LANE_REVERSIBLE ||
          check_lane->type() == ad_byd::planning::LANE_VARIABLE_TURN) {
        special_lane_number += 1;
        if (check_lane->id() == variable_turn_cur_seq->id()) {
          if_found = true;
          variable_turn_pos = pos_count;
        }
      }
      legal_lanes.emplace_back(check_lane);
    }  // end for
    if (lanes_number < 2) {
      return false;  // can not lc
    }
    if (lanes_number == special_lane_number) {
      return false;  // can not lc
    }
    total_length += check_section_ptr->topo_length();
    start_index += 1;
    if (if_found) {
      variable_turn_section_size = lanes_number;
      break;
    }
  }  // end while

  // lc direction decider
  if (variable_turn_pos == 1) {  // at left 1
    lc_direction = -1;           // go right
    return true;
  }
  if (variable_turn_pos == variable_turn_section_size) {  // at right 1
    lc_direction = 1;                                     // go left
    return true;
  }
  // ego at center lane
  if (!cur_lane_seq_info.nearest_lane) {
    return false;
  }
  LaneConstPtr right_lane = map->GetRightLane(cur_lane_seq_info.nearest_lane);
  LaneConstPtr left_lane = map->GetLeftLane(cur_lane_seq_info.nearest_lane);

  if (right_lane_seq && left_lane_seq && right_lane && left_lane) {
    // both lane low prior
    if ((left_lane->type() == ad_byd::planning::LANE_EMERGENCY ||
         left_lane->type() == ad_byd::planning::LANE_REVERSIBLE ||
         left_lane->type() == ad_byd::planning::LANE_VARIABLE_TURN) &&
        (right_lane->type() == ad_byd::planning::LANE_EMERGENCY ||
         right_lane->type() == ad_byd::planning::LANE_REVERSIBLE ||
         right_lane->type() == ad_byd::planning::LANE_VARIABLE_TURN)) {
      lc_direction = 0;
      return false;
    }
    // left lane highest prior
    if (left_lane->type() == ad_byd::planning::LANE_NORMAL) {
      lc_direction = 1;
      return true;
    }
    // left higher than right
    if ((left_lane->type() == ad_byd::planning::LANE_BUS_NORMAL ||
         left_lane->type() == ad_byd::planning::LANE_HOV_NORMAL) &&
        (right_lane->type() == ad_byd::planning::LANE_REVERSIBLE ||
         right_lane->type() == ad_byd::planning::LANE_EMERGENCY ||
         right_lane->type() == ad_byd::planning::LANE_VARIABLE_TURN)) {
      lc_direction = 1;
      return true;
    }
    // right lane high prior
    if (right_lane->type() == ad_byd::planning::LANE_NORMAL) {
      lc_direction = -1;
      return true;
    }
    // right higher than right
    if ((right_lane->type() == ad_byd::planning::LANE_BUS_NORMAL ||
         right_lane->type() == ad_byd::planning::LANE_HOV_NORMAL) &&
        (left_lane->type() == ad_byd::planning::LANE_REVERSIBLE ||
         left_lane->type() == ad_byd::planning::LANE_EMERGENCY ||
         left_lane->type() == ad_byd::planning::LANE_VARIABLE_TURN)) {
      lc_direction = -1;
      return true;
    }
  }

  if (left_lane_seq && left_lane) {
    if (left_lane->type() == ad_byd::planning::LANE_NORMAL ||
        left_lane->type() == ad_byd::planning::LANE_BUS_NORMAL ||
        left_lane->type() == ad_byd::planning::LANE_HOV_NORMAL) {
      lc_direction = 1;
      return true;
    }
  }

  if (right_lane_seq && right_lane) {
    if (right_lane->type() == ad_byd::planning::LANE_NORMAL ||
        right_lane->type() == ad_byd::planning::LANE_BUS_NORMAL ||
        right_lane->type() == ad_byd::planning::LANE_HOV_NORMAL) {
      lc_direction = -1;
      return true;
    }
  }
  return false;
}

bool IfLcByBusLane(const ad_byd::planning::LaneSequencePtr &cur_lane_sequence,
                   const ad_byd::planning::LaneSequencePtr &left_lane_seq,
                   const ad_byd::planning::LaneSequencePtr &right_lane_seq,
                   const ad_byd::planning::LaneSeqInfo &cur_lane_seq_info,
                   int &lc_direction, const ad_byd::planning::MapPtr &map) {
  lc_direction = 0;
  bool if_near_bus_lane = false;
  // 1. Check if cur lane seq exists bus lane
  if (!cur_lane_sequence) {
    return false;
  }
  LaneConstPtr bus_lane_cur_seq = nullptr;
  std::unordered_set<LaneType> lane_type_set{ad_byd::planning::LANE_BUS_NORMAL,
                                             ad_byd::planning::LANE_HOV_NORMAL};
  double dist_to_bus_lane = cur_lane_sequence->GetDistanceToTargetLaneType(
      bus_lane_cur_seq, lane_type_set, map->route()->navi_start());

  // LOGDATA("center_intent", absl::StrCat("[IfLcByBusLane]: dist_to_bus_lane:
  // ",
  //                                       dist_to_bus_lane));
  // LOGDATA("center_intent",
  //         absl::StrCat("[IfLcByBusLane]: map_supplement.dist_to_junction: ",
  //                      map_supplement.dist_to_junction));
  // intent threshold
  double bus_lane_lc_threshold = 200.0;
  if (bus_lane_cur_seq && dist_to_bus_lane < bus_lane_lc_threshold &&
      cur_lane_seq_info.dist_to_junction > 80) {
    if_near_bus_lane = true;
  } else {
    return false;  // no need lc
  }
  // 2. Check if LaneSection size is valid for center
  const auto cur_section_ptr = cur_lane_seq_info.cur_section_ptr;
  const auto route_section_seq = map->GetRouteSectionSeq();
  int start_index = 0;
  if (cur_section_ptr) {
    for (int i = 0; i < route_section_seq.size(); i++) {
      const auto &section_ptr = map->GetSectionById(route_section_seq.at(i));
      if (section_ptr && section_ptr->id() == cur_section_ptr->id()) {
        start_index = i;
      }
    }
  }
  double total_length = 0.0;
  int bus_lane_pos = 0;
  int bus_lane_section_size = 0;
  int special_lane_number = 0;
  bool if_found = false;
  std::vector<LaneConstPtr> legal_lanes;
  while (start_index < route_section_seq.size() &&
         total_length < bus_lane_lc_threshold && !if_found) {
    // For each section reset legal lanes, pos count ,lanes number
    const auto check_section_ptr =
        map->GetSectionById(route_section_seq.at(start_index));
    int lanes_number = check_section_ptr->lanes().size();
    int pos_count = 0;
    special_lane_number = 0;
    legal_lanes.clear();
    // For each lane in cur checking section
    for (auto iter = check_section_ptr->lanes().begin();
         iter != check_section_ptr->lanes().end(); iter++) {
      pos_count += 1;
      // cur section lane type check
      LaneConstPtr check_lane = map->GetLaneById((*iter));
      if (!check_lane || check_lane->type() == ad_byd::planning::LANE_UNKNOWN ||
          check_lane->type() == ad_byd::planning::LANE_NON_MOTOR ||
          !check_lane->center_line().IsValid()) {
        lanes_number -= 1;
        pos_count -= 1;
        continue;
      }
      if (check_lane->type() == ad_byd::planning::LANE_VIRTUAL_JUNCTION) {
        return false;
      }
      if (check_lane->type() == ad_byd::planning::LANE_BUS_NORMAL ||
          check_lane->type() == ad_byd::planning::LANE_EMERGENCY ||
          check_lane->type() == ad_byd::planning::LANE_HOV_NORMAL ||
          check_lane->type() == ad_byd::planning::LANE_REVERSIBLE ||
          check_lane->type() == ad_byd::planning::LANE_VARIABLE_TURN) {
        special_lane_number += 1;
        if (check_lane->id() == bus_lane_cur_seq->id()) {
          if_found = true;
          bus_lane_pos = pos_count;
        }
      }
      legal_lanes.emplace_back(check_lane);
    }  // end for
    if (lanes_number < 2) {
      return false;  // can not lc
    }

    if (lanes_number == special_lane_number) {
      return false;  // can not lc
    }

    total_length += check_section_ptr->topo_length();
    start_index += 1;
    if (if_found) {
      bus_lane_section_size = lanes_number;
      break;
    }

  }  // end while

  // lc direction decider
  if (bus_lane_pos == 1) {
    lc_direction = -1;
    return true;
  }
  if (bus_lane_pos == bus_lane_section_size) {
    lc_direction = 1;
    return true;
  }

  // ego at center lane
  if (!cur_lane_seq_info.nearest_lane) {
    return false;
  }
  LaneConstPtr right_lane = map->GetRightLane(cur_lane_seq_info.nearest_lane);
  LaneConstPtr left_lane = map->GetLeftLane(cur_lane_seq_info.nearest_lane);

  if (right_lane_seq && left_lane_seq && right_lane && left_lane) {
    // both lane low prior
    if ((left_lane->type() == ad_byd::planning::LANE_EMERGENCY ||
         left_lane->type() == ad_byd::planning::LANE_REVERSIBLE ||
         left_lane->type() == ad_byd::planning::LANE_VARIABLE_TURN) &&
        (right_lane->type() == ad_byd::planning::LANE_EMERGENCY ||
         right_lane->type() == ad_byd::planning::LANE_REVERSIBLE ||
         right_lane->type() == ad_byd::planning::LANE_VARIABLE_TURN)) {
      lc_direction = 0;
      return false;
    }
    // left lane high prior
    if (left_lane->type() == ad_byd::planning::LANE_NORMAL) {
      lc_direction = 1;
      return true;
    }
    // right lane high prior
    if (right_lane->type() == ad_byd::planning::LANE_NORMAL) {
      lc_direction = -1;
      return true;
    }
  }

  if (left_lane_seq && left_lane) {
    if (left_lane->type() == ad_byd::planning::LANE_NORMAL) {
      lc_direction = 1;
      return true;
    }
  }

  if (right_lane_seq && right_lane) {
    if (right_lane->type() == ad_byd::planning::LANE_NORMAL) {
      lc_direction = -1;
      return true;
    }
  }

  return false;
}

TargetLaneInfo SelectTargetLaneSeqForCityHd(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos, bool is_miss_navi,
    const ApolloTrajectoryPointProto plan_start_point,
    const VehicleParamsProto *vehicle_param,
    const PlannerSemanticMapManager *psmm,
    const std::optional<Behavior_FunctionId> function_id,
    const std::optional<int> tunnel_status, const PlannerObjectManager &obj_mgr,
    const LaneSequencePtr &pre_target_laneseq,
    const LaneChangeStateProto *lane_change_state,
    const LaneChangeReason &lane_change_reason,
    const DriverAction::LaneChangeCommand &lane_change_command,
    const LaneConstPtr &nearest_lane,
    const std::optional<double> cruising_speed_limit, std::string leading_id) {
  TargetLaneInfo target_lane_info;

  // intent for centered lane
  TargetLaneInfo target_lane_center;
  target_lane_center =
      SelectLaneCenter(lane_seq_infos, map, ego_pos, plan_start_point);

  // intent for blocker lane
  TargetLaneInfo target_lane_blocker;
  target_lane_blocker = SelectLaneBlocker(
      lane_seq_infos, map, ego_pos, &plan_start_point, vehicle_param, psmm,
      function_id, tunnel_status, obj_mgr, pre_target_laneseq,
      lane_change_state, lane_change_reason, lane_change_command, nearest_lane);

  // intent for navi lane
  TargetLaneInfo target_lane_navi;
  target_lane_navi = SelectLaneNavi(lane_seq_infos, map, ego_pos, is_miss_navi,
                                    &plan_start_point);

  // intent for merge lane
  TargetLaneInfo target_lane_merge;
  target_lane_merge =
      SelectLaneMerge(lane_seq_infos, map, ego_pos, plan_start_point);

  // intent for overtake lane
  TargetLaneInfo target_lane_overtake;
  target_lane_overtake =
      SelectLaneOverTake(lane_seq_infos, map, ego_pos, cruising_speed_limit,
                         obj_mgr, plan_start_point, leading_id);

  // final intention
  if (target_lane_blocker.target_lane_seq) {
    target_lane_info = target_lane_blocker;
  } else if (target_lane_merge.target_lane_seq) {
    target_lane_info = target_lane_merge;
  } else if (target_lane_navi.target_lane_seq) {
    target_lane_info = target_lane_navi;
  } else if (target_lane_overtake.target_lane_seq) {
    target_lane_info = target_lane_overtake;
  } else if (target_lane_center.target_lane_seq) {
    target_lane_info = target_lane_center;
  }

  return target_lane_info;
}

TargetLaneInfo SelectLaneNavi(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos, bool is_miss_navi,
    const ApolloTrajectoryPointProto *plan_start_point) {
  ad_byd::planning::LaneSeqInfo cur_lane_seq_info;
  ad_byd::planning::LaneSeqInfo left_lane_seq_info;
  ad_byd::planning::LaneSeqInfo right_lane_seq_info;
  ad_byd::planning::LaneSeqInfo tgt_lane_seq_info;
  for (const auto &candidate_seq_info : lane_seq_infos) {
    if (candidate_seq_info.is_current &&
        candidate_seq_info.lc_dir == ad_byd::planning::Command_Invalid) {
      cur_lane_seq_info = candidate_seq_info;
    } else if (candidate_seq_info.lc_dir ==
               ad_byd::planning::Command_LaneChangeLeft) {
      left_lane_seq_info = candidate_seq_info;
    } else if (candidate_seq_info.lc_dir ==
               ad_byd::planning::Command_LaneChangeRight) {
      right_lane_seq_info = candidate_seq_info;
    }
  }
  auto cur_lane_seq = cur_lane_seq_info.lane_seq;
  if (!cur_lane_seq) {
    Log2DDS::LogDataV2("navi_lc_intent", "[navi] no cur lane seq");
    return {};
  }
  std::bitset<32> navi_intent_code;
  if (cur_lane_seq_info.lane_seq_connect_navi_end) {
    Log2DDS::LogDataV2("navi_lc_intent", "[navi] cur to end");
    navi_intent_code.set(0, true);
  }
  double dist_to_lc_solidline = std::numeric_limits<double>::infinity();
  ad_byd::planning::LaneSequencePtr target_lane_seq = nullptr;
  if (cur_lane_seq_info.navi_lc_command ==
          ad_byd::planning::Command_LaneChangeLeft &&
      left_lane_seq_info.lane_seq) {
    target_lane_seq = left_lane_seq_info.lane_seq;
    tgt_lane_seq_info = left_lane_seq_info;
    dist_to_lc_solidline = cur_lane_seq_info.dist_to_left_solid_line;
  } else {
    Log2DDS::LogDataV2("navi_lc_intent", "[navi] no left lane seq");
  }

  if (cur_lane_seq_info.navi_lc_command ==
          ad_byd::planning::Command_LaneChangeRight &&
      right_lane_seq_info.lane_seq) {
    target_lane_seq = right_lane_seq_info.lane_seq;
    tgt_lane_seq_info = right_lane_seq_info;
    dist_to_lc_solidline = cur_lane_seq_info.dist_to_right_solid_line;
  } else {
    Log2DDS::LogDataV2("navi_lc_intent", "[navi] no right lane seq");
  }

  if (!target_lane_seq) {
    Log2DDS::LogDataV2("navi_lc_intent", "[navi] no tgt lane seq");
    navi_intent_code.set(2, true);
    return {};
  }

  // check navi_lc dist
  const auto pri_lane_relation = cur_lane_seq_info.lc_num;
  double navi_lc_dist_thr = 2500.0;
  switch (std::abs(pri_lane_relation)) {
    case 0:
      navi_lc_dist_thr = -2500.0;
      break;
    case 1:
      navi_lc_dist_thr = 300.0;
      break;
    case 2:
      navi_lc_dist_thr = 900.0;
      break;
    case 3:
      navi_lc_dist_thr = 1100.0;
      break;
    default:
      break;
  }

  if (cur_lane_seq_info.dist_to_navi_end > navi_lc_dist_thr) {
    Log2DDS::LogDataV2("navi_lc_intent", "[navi] navi dist too far");
    navi_intent_code.set(1, true);
    return {};
  }

  auto junction_lane = cur_lane_seq_info.junction_lane;
  double dis_to_junction = cur_lane_seq_info.dist_to_junction;
  bool navi_virtual_lane = true;
  if (!junction_lane || !junction_lane->is_navigation()) {
    navi_virtual_lane = false;
  }

  // check taget lane type
  if (target_lane_seq) {
    const auto nearest_lane =
        target_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
    if (nearest_lane &&
        (nearest_lane->type() == ad_byd::planning::LaneType::LANE_UNKNOWN ||
         nearest_lane->type() == ad_byd::planning::LaneType::LANE_NON_MOTOR ||
         nearest_lane->type() == ad_byd::planning::LaneType::LANE_EMERGENCY)) {
      Log2DDS::LogDataV2("navi_lc_intent", "[navi] lane type illegal");
      navi_intent_code.set(4, true);
      return {};
    }
  }
  // check junction
  if (target_lane_seq) {
    const auto nearest_lane =
        target_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
    if (nearest_lane && !(nearest_lane->junction_id() == 0)) {
      Log2DDS::LogDataV2("navi_lc_intent", "[navi] in junction");
      navi_intent_code.set(5, true);
      return {};
    }
    if (navi_virtual_lane && dis_to_junction < 35.0) {
      navi_intent_code.set(6, true);
      Log2DDS::LogDataV2("navi_lc_intent", "[navi] can pass junction");
      return {};
    }
  }
  // check solid line
  if (!is_miss_navi &&
      dist_to_lc_solidline < 15.0 + plan_start_point->v() * 3.0) {
    Log2DDS::LogDataV2("navi_lc_intent", "[navi] solidline check fail");
    navi_intent_code.set(7, true);
    return {};
  }

  // compute can pass current junction
  bool can_pass_junction =
      cur_lane_seq->CanPassJunction(map->route()->navi_start(), NULL, true);
  // compute dist between current junction and next junction
  ad_byd::planning::LaneConstPtr next_virtual_lane = nullptr;
  double dist_between_junctions = DBL_MAX;
  if (junction_lane) {
    dist_between_junctions = cur_lane_seq->GetDistanceBetweenJunction(
        junction_lane->junction_id(), next_virtual_lane);
  }
  // compute can pass next junction
  bool can_pass_next_junction = false;
  if (next_virtual_lane) {
    can_pass_next_junction = next_virtual_lane->is_navigation();
  }
  if (can_pass_junction && can_pass_next_junction) {
    Log2DDS::LogDataV2("navi_lc_intent", "[navi] can pass two junction 1");
    navi_intent_code.set(8, true);
    return {};
  }
  if (can_pass_junction && !can_pass_next_junction) {
    const double dist_between_junctions_threshold = 185.0;
    if (std::abs(pri_lane_relation) <= 1 &&
        dist_between_junctions > dist_between_junctions_threshold &&
        dist_between_junctions < 500.0) {
      Log2DDS::LogDataV2("navi_lc_intent", "[navi] can pass two junction 2");
      navi_intent_code.set(9, true);
      return {};
    }
    if (std::abs(pri_lane_relation) <= 2 && dist_between_junctions >= 500.0) {
      navi_intent_code.set(10, true);
      Log2DDS::LogDataV2("navi_lc_intent", "[navi] can pass two junction 3");
      return {};
    }
  }
  // Avoid bus_lane.
  if (target_lane_seq) {
    ad_byd::planning::LaneConstPtr bus_lane = nullptr;
    const std::unordered_set<ad_byd::planning::LaneType> lane_type_set{
        ad_byd::planning::LANE_BUS_NORMAL};
    double dist_to_bus_lane = target_lane_seq->GetDistanceToTargetLaneType(
        bus_lane, lane_type_set, map->route()->navi_start());
    Log2DDS::LogDataV2(
        "navi_lc_intent",
        absl::StrCat("[navi] dist to bus lane: ", dist_to_bus_lane));
    if (cur_lane_seq_info.dist_to_navi_end > 160.0 &&
        std::abs(pri_lane_relation) <= 1 &&
        dist_to_bus_lane < cur_lane_seq_info.dist_to_navi_end &&
        dist_to_bus_lane < std::max(5.0 * plan_start_point->v(), 50.0)) {
      navi_intent_code.set(13, true);
      Log2DDS::LogDataV2("navi_lc_intent", "[navi] avoid bus lane");
      return {};
    }
  }
  Log2DDS::LogDataV2(
      "navi_lc_intent",
      absl::StrCat("navi_disable_code: ", navi_intent_code.to_ulong()));

  // final dicision
  bool navi_intent_flag = navi_intent_code.none();

  TargetLaneInfo target_lane_info;
  if (navi_intent_flag || is_miss_navi) {
    Log2DDS::LogDataV2("navi_lc_intent", "navi intent");
    target_lane_info.lc_reason = ad_byd::planning::LcReason::LC_REASON_FOR_NAVI;
    target_lane_info.lc_command = cur_lane_seq_info.navi_lc_command;
    target_lane_info.target_lane_seq = target_lane_seq;
  }
  return target_lane_info;
}

TargetLaneInfo SelectLaneCenter(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos,
    const ApolloTrajectoryPointProto plan_start_point) {
  ad_byd::planning::LaneSeqInfo cur_lane_seq_info;
  ad_byd::planning::LaneSeqInfo left_lane_seq_info;
  ad_byd::planning::LaneSeqInfo right_lane_seq_info;
  ad_byd::planning::LaneSeqInfo tgt_lane_seq_info;
  for (const auto &candidate_seq_info : lane_seq_infos) {
    if (candidate_seq_info.is_current &&
        candidate_seq_info.lc_dir == ad_byd::planning::Command_Invalid) {
      cur_lane_seq_info = candidate_seq_info;
    } else if (candidate_seq_info.lc_dir ==
               ad_byd::planning::Command_LaneChangeLeft) {
      left_lane_seq_info = candidate_seq_info;
    } else if (candidate_seq_info.lc_dir ==
               ad_byd::planning::Command_LaneChangeRight) {
      right_lane_seq_info = candidate_seq_info;
    }
  }
  auto cur_lane_seq = cur_lane_seq_info.lane_seq;
  if (!cur_lane_seq) {
    Log2DDS::LogDataV2("center_lc_intent", "[center] no cur lane seq");
    return {};
  }

  std::bitset<32> center_disable_code;
  const auto cur_lane = cur_lane_seq_info.nearest_lane;
  if (!cur_lane) {
    Log2DDS::LogDataV2("center_lc_intent", "[center] no cur lane");
    return {};
  }
  // 0. Check current lane type
  if (cur_lane->type() != ad_byd::planning::LANE_NORMAL &&
      cur_lane->type() != ad_byd::planning::LANE_BUS_NORMAL &&
      cur_lane->type() != ad_byd::planning::LANE_HOV_NORMAL) {
    center_disable_code.set(0, true);
    Log2DDS::LogDataV2("center_lc_intent", "cur_lane illegal");
    return {};
  }

  LaneConstPtr target_lane = nullptr;
  LaneSequencePtr center_tgt_lane_seq = nullptr;
  ad_byd::planning::BehaviorCommand center_command =
      ad_byd::planning::Command_Invalid;

  // Check if occupied.
  bool is_occupied_lc = false;
  bool is_bus_lc = false;
  bool is_variable_lc = false;
  bool is_composite_lc = false;
  bool is_emergency_lc = false;
  bool is_reversible_lc = false;
  bool is_edge_lc = false;

  const auto left_target_lane_seq = left_lane_seq_info.lane_seq;
  double left_seq_navi_end = left_lane_seq_info.dist_to_navi_end;

  const auto right_target_lane_seq = right_lane_seq_info.lane_seq;
  double right_seq_navi_end = right_lane_seq_info.dist_to_navi_end;

  double dist_to_occupied = DBL_MAX;
  if (left_target_lane_seq && dist_to_occupied < left_seq_navi_end &&
      dist_to_occupied < 300.0) {
    target_lane = map->GetLeftLane(cur_lane);
    center_tgt_lane_seq = left_target_lane_seq;
    center_command = ad_byd::planning::Command_LaneChangeLeft;
    is_occupied_lc = true;
  } else if (!left_target_lane_seq && right_target_lane_seq &&
             dist_to_occupied < right_seq_navi_end &&
             dist_to_occupied < 300.0) {
    target_lane = map->GetRightLane(cur_lane);
    center_tgt_lane_seq = right_target_lane_seq;
    center_command = ad_byd::planning::Command_LaneChangeRight;
    is_occupied_lc = true;
  } else {
    ad_byd::planning::BehaviorCommand composite_intent =
        ad_byd::planning::Command_Invalid;
    // check cur_lane_position.
    const auto &cur_lane_position = cur_lane_seq_info.cur_lane_position;
    const auto &cur_section_lane_num = cur_lane_seq_info.cur_section_lane_num;
    double check_section_size_distance = 200.0;
    bool section_size_legal = true;
    if (map && map->route()) {
      section_size_legal = IfSectionSizeLargerThanTwo(
          check_section_size_distance, map, map->route()->navi_start());
    }
    if (section_size_legal && cur_section_lane_num > 2 &&
        cur_lane_position == 1) {
      // target_lane Right if in left 1.
      // check safe
      bool is_faster = true;
      bool is_safe = true;
      if (is_faster && is_safe) {
        target_lane = map->GetRightLane(cur_lane);
        center_tgt_lane_seq = right_target_lane_seq;
        tgt_lane_seq_info = right_lane_seq_info;
        center_command = ad_byd::planning::Command_LaneChangeRight;
        is_edge_lc = true;
        Log2DDS::LogDataV2("center_lc_intent", "edge lc right");
      }
    } else if (section_size_legal && cur_section_lane_num > 2 &&
               cur_lane_position == cur_section_lane_num) {
      // target_lane Left if in right 1.
      // check safe
      bool is_faster = true;
      bool is_safe = true;
      if (is_faster && is_safe) {
        target_lane = map->GetLeftLane(cur_lane);
        center_tgt_lane_seq = left_target_lane_seq;
        tgt_lane_seq_info = left_lane_seq_info;
        center_command = ad_byd::planning::Command_LaneChangeLeft;
        is_edge_lc = true;
        Log2DDS::LogDataV2("center_lc_intent", "edge lc left");
      }
    } else if (composite_intent == ad_byd::planning::Command_LaneChangeLeft) {
      target_lane = map->GetLeftLane(cur_lane);
      center_tgt_lane_seq = left_target_lane_seq;
      tgt_lane_seq_info = left_lane_seq_info;
      center_command = ad_byd::planning::Command_LaneChangeLeft;
      is_composite_lc = true;
      Log2DDS::LogDataV2("center_lc_intent", "composite lc left");
    } else if (composite_intent == ad_byd::planning::Command_LaneChangeRight) {
      target_lane = map->GetRightLane(cur_lane);
      center_tgt_lane_seq = right_target_lane_seq;
      tgt_lane_seq_info = right_lane_seq_info;
      center_command = ad_byd::planning::Command_LaneChangeRight;
      is_composite_lc = true;
      Log2DDS::LogDataV2("center_lc_intent", "composite lc right");
    }
    // Check lc triggered by variable turn
    double dist_to_variable_turn = std::numeric_limits<double>::max();
    int lc_direction = 0;
    if (IfLcByVariableTurn(cur_lane_seq, left_target_lane_seq,
                           right_target_lane_seq, cur_lane_seq_info,
                           lc_direction, dist_to_variable_turn,
                           ad_byd::planning::LANE_VARIABLE_TURN, map)) {
      if (lc_direction == 1) {
        target_lane = map->GetLeftLane(cur_lane);
        if (target_lane) {
          center_tgt_lane_seq = left_target_lane_seq;
          if (center_tgt_lane_seq) {
            bool variable_turn_can_pass_junction =
                center_tgt_lane_seq->CanPassJunction(map->route()->navi_start(),
                                                     nullptr, false);
            if (variable_turn_can_pass_junction ||
                left_lane_seq_info.dist_to_junction > 300) {
              center_command = ad_byd::planning::Command_LaneChangeLeft;
              tgt_lane_seq_info = left_lane_seq_info;
              is_variable_lc = true;
              Log2DDS::LogDataV2("center_lc_intent", "variable turn lc left");
            } else {
              center_tgt_lane_seq = nullptr;
            }
          }
        }
      }
      if (lc_direction == -1) {
        target_lane = map->GetRightLane(cur_lane);
        if (target_lane) {
          center_tgt_lane_seq = right_target_lane_seq;
          if (center_tgt_lane_seq) {
            bool variable_turn_can_pass_junction =
                center_tgt_lane_seq->CanPassJunction(map->route()->navi_start(),
                                                     nullptr, false);
            if (variable_turn_can_pass_junction ||
                right_lane_seq_info.dist_to_junction > 300) {
              center_command = ad_byd::planning::Command_LaneChangeRight;
              tgt_lane_seq_info = right_lane_seq_info;
              is_variable_lc = true;
              Log2DDS::LogDataV2("center_lc_intent", "variable turn lc right");
            } else {
              center_tgt_lane_seq = nullptr;
            }
          }
        }
      }
    }
    // Check lc triggered by LANE_EMERGENCY
    double dist_to_emergency = std::numeric_limits<double>::max();
    lc_direction = 0;
    if (IfLcByVariableTurn(cur_lane_seq, left_target_lane_seq,
                           right_target_lane_seq, cur_lane_seq_info,
                           lc_direction, dist_to_emergency,
                           ad_byd::planning::LANE_EMERGENCY, map)) {
      if (lc_direction == 1) {
        target_lane = map->GetLeftLane(cur_lane);
        if (target_lane) {
          center_tgt_lane_seq = left_target_lane_seq;
          if (center_tgt_lane_seq) {
            bool variable_turn_can_pass_junction =
                center_tgt_lane_seq->CanPassJunction(map->route()->navi_start(),
                                                     nullptr, false);
            if (variable_turn_can_pass_junction ||
                left_lane_seq_info.dist_to_junction > 300) {
              center_command = ad_byd::planning::Command_LaneChangeLeft;
              tgt_lane_seq_info = left_lane_seq_info;
              is_emergency_lc = true;
              Log2DDS::LogDataV2("center_lc_intent", "emergency lane lc left");
            } else {
              center_tgt_lane_seq = nullptr;
            }
          }
        }
      }
      if (lc_direction == -1) {
        target_lane = map->GetRightLane(cur_lane);
        if (target_lane) {
          center_tgt_lane_seq = right_target_lane_seq;
          if (center_tgt_lane_seq) {
            bool variable_turn_can_pass_junction =
                center_tgt_lane_seq->CanPassJunction(map->route()->navi_start(),
                                                     nullptr, false);
            if (variable_turn_can_pass_junction ||
                right_lane_seq_info.dist_to_junction > 300) {
              center_command = ad_byd::planning::Command_LaneChangeRight;
              tgt_lane_seq_info = right_lane_seq_info;
              is_emergency_lc = true;
              Log2DDS::LogDataV2("center_lc_intent", "emergency lane lc right");
            } else {
              center_tgt_lane_seq = nullptr;
            }
          }
        }
      }
    }
    // Check lc triggered by LANE_REVERSIBLE
    double dist_to_reversible = std::numeric_limits<double>::max();
    lc_direction = 0;
    if (IfLcByVariableTurn(cur_lane_seq, left_target_lane_seq,
                           right_target_lane_seq, cur_lane_seq_info,
                           lc_direction, dist_to_reversible,
                           ad_byd::planning::LANE_REVERSIBLE, map)) {
      if (lc_direction == 1) {
        target_lane = map->GetLeftLane(cur_lane);
        if (target_lane) {
          center_tgt_lane_seq = left_target_lane_seq;
          if (center_tgt_lane_seq) {
            bool variable_turn_can_pass_junction =
                center_tgt_lane_seq->CanPassJunction(map->route()->navi_start(),
                                                     nullptr, false);
            if (variable_turn_can_pass_junction ||
                left_lane_seq_info.dist_to_junction > 300) {
              center_command = ad_byd::planning::Command_LaneChangeLeft;
              tgt_lane_seq_info = left_lane_seq_info;
              is_reversible_lc = true;
              Log2DDS::LogDataV2("center_lc_intent", "reversible lane lc left");
            } else {
              center_tgt_lane_seq = nullptr;
            }
          }
        }
      }
      if (lc_direction == -1) {
        target_lane = map->GetRightLane(cur_lane);
        if (target_lane) {
          center_tgt_lane_seq = right_target_lane_seq;
          if (center_tgt_lane_seq) {
            bool variable_turn_can_pass_junction =
                center_tgt_lane_seq->CanPassJunction(map->route()->navi_start(),
                                                     nullptr, false);
            if (variable_turn_can_pass_junction ||
                right_lane_seq_info.dist_to_junction > 300) {
              center_command = ad_byd::planning::Command_LaneChangeRight;
              tgt_lane_seq_info = right_lane_seq_info;
              is_reversible_lc = true;
              Log2DDS::LogDataV2("center_lc_intent",
                                 "reversible lane lc right");
            } else {
              center_tgt_lane_seq = nullptr;
            }
          }
        }
      }
    }
    // Check lc triggered by bus lane
    int lc_bus_lane_direction = 0;
    if (IfLcByBusLane(cur_lane_seq, left_target_lane_seq, right_target_lane_seq,
                      cur_lane_seq_info, lc_bus_lane_direction, map)) {
      if (lc_bus_lane_direction == 1) {
        target_lane = map->GetLeftLane(cur_lane);
        if (target_lane) {
          center_tgt_lane_seq = left_target_lane_seq;
          if (center_tgt_lane_seq) {
            bool bus_lane_can_pass_junction =
                center_tgt_lane_seq->CanPassJunction(map->route()->navi_start(),
                                                     nullptr, false);
            if (bus_lane_can_pass_junction ||
                cur_lane_seq_info.dist_to_junction > 300) {
              center_command = ad_byd::planning::Command_LaneChangeLeft;
              tgt_lane_seq_info = left_lane_seq_info;
              is_bus_lc = true;
              Log2DDS::LogDataV2("center_lc_intent", "bus lane lc left");
            } else {
              center_tgt_lane_seq = nullptr;
            }
          }
        }
      }
      if (lc_bus_lane_direction == -1) {
        target_lane = map->GetRightLane(cur_lane);
        if (target_lane) {
          center_tgt_lane_seq = right_target_lane_seq;
          if (center_tgt_lane_seq) {
            bool bus_lane_can_pass_junction =
                center_tgt_lane_seq->CanPassJunction(map->route()->navi_start(),
                                                     nullptr, false);
            if (bus_lane_can_pass_junction ||
                cur_lane_seq_info.dist_to_junction > 300) {
              center_command = ad_byd::planning::Command_LaneChangeRight;
              tgt_lane_seq_info = right_lane_seq_info;
              is_bus_lc = true;
              Log2DDS::LogDataV2("center_lc_intent", "bus lane lc right");
            } else {
              center_tgt_lane_seq = nullptr;
            }
          }
        }
      }
    }
    LaneConstPtr virtual_lane = nullptr;
    LaneConstPtr next_virtual_lane = nullptr;
    bool can_pass_junction = false;
    bool can_pass_next_junction = false;
    double dist_between_junctions = DBL_MAX;
    double dist_to_junction = DBL_MAX;
    double target_dist_to_occupied = DBL_MAX;
    if (center_tgt_lane_seq) {
      // compute dist to current junction
      dist_to_junction = center_tgt_lane_seq->GetDistanceToJunction(
          virtual_lane, map->route()->navi_start(), true);
      // compute dist_between_junctions
      can_pass_junction = center_tgt_lane_seq->CanPassJunction(
          map->route()->navi_start(), nullptr, true);
      // compute dist between current junction and next junction
      if (virtual_lane) {
        dist_between_junctions =
            center_tgt_lane_seq->GetDistanceBetweenJunction(
                virtual_lane->junction_id(), next_virtual_lane);
      }
      // compute can pass next junction
      if (next_virtual_lane) {
        can_pass_next_junction = next_virtual_lane->is_navigation();
      }
    }
    // 2. Check !can_pass_junction and near to junction.
    double center_lc_dist_thr = 300;
    center_lc_dist_thr += 100.0;
    if (!can_pass_junction && dist_to_junction < center_lc_dist_thr) {
      center_disable_code.set(2, true);
    }
    // 3. Check can_pass_junction and !can_pass_next_junction.
    // Check if near to next_junction
    if (can_pass_junction && !can_pass_next_junction && virtual_lane &&
        (dist_to_junction + virtual_lane->topo_length() +
             dist_between_junctions <
         center_lc_dist_thr)) {
      center_disable_code.set(3, true);
    }

    // 5. Check if need to navi.
    double center_navi_end_dist = cur_lane_seq_info.dist_to_navi_end;
    const auto &target_lane_relation = tgt_lane_seq_info.lc_num;
    const auto &current_lane_relation = cur_lane_seq_info.lc_num;

    double navi_lc_dist_thr = 2500.0;
    switch (std::abs(target_lane_relation)) {
      case 0:
        navi_lc_dist_thr = -2500.0;
        break;
      case 1:
        navi_lc_dist_thr = 300.0;
        break;
      case 2:
        navi_lc_dist_thr = 900;
        break;
      case 3:
        navi_lc_dist_thr = 1100;
        break;
      default:
        break;
    }
    if (target_lane_relation > current_lane_relation &&
        center_navi_end_dist <
            navi_lc_dist_thr + 100.0 + plan_start_point.v() * 5) {
      center_disable_code.set(5, true);
    }
    Log2DDS::LogDataV2(
        "center_lc_intent",
        absl::StrCat("target_lane_relation: ", target_lane_relation));
    Log2DDS::LogDataV2(
        "center_lc_intent",
        absl::StrCat("center_navi_end_dist: ", center_navi_end_dist));

    // 6. Check if no leader car.

    // const auto &leader_car = planning_context.decision().leader_car();
    // if (!leader_car || (leader_car && leader_car->ds() > 100.0)) {
    //   center_disable_code.set(6, true);
    // }
  }

  // 10. Check if center_tgt_lane_seq is nullptr.
  if (!center_tgt_lane_seq) {
    center_disable_code.set(10, true);
  }
  // 11. Check target_lane type.
  if (!target_lane ||
      (target_lane && target_lane->type() != ad_byd::planning::LANE_NORMAL &&
       cur_lane->type() != ad_byd::planning::LANE_BUS_NORMAL &&
       cur_lane->type() != ad_byd::planning::LANE_HOV_NORMAL)) {
    center_disable_code.set(11, true);
  }
  // 12. Route end check
  const double navi_remain_dist_thr = 100.0;
  if (cur_lane_seq_info.dist_to_navi_end < navi_remain_dist_thr) {
    center_disable_code.set(12, true);
    Log2DDS::LogDataV2("center_lc_intent", "dist to navi < 100");
    return {};
  }

  // 13. dist to navi split
  if (cur_lane_seq_info.dist_to_navi_end < 300.0 &&
      (cur_lane_seq_info.navi_lc_command ==
           ad_byd::planning::Command_LaneChangeLeft ||
       cur_lane_seq_info.navi_lc_command ==
           ad_byd::planning::Command_LaneChangeRight)) {
    center_disable_code.set(13, true);
    Log2DDS::LogDataV2("center_lc_intent", "dist to navi < 300");
    return {};
  }
  // 14. Check dist to current junction
  if (cur_lane_seq_info.dist_to_junction < 50.0) {
    center_disable_code.set(14, true);
    Log2DDS::LogDataV2("center_lc_intent", "dist to junction < 50");
    return {};
  }

  // 15. dist_from_junction
  const auto &nearest_lane = cur_lane_seq_info.nearest_lane;
  if (nearest_lane && nearest_lane->junction_id() == 0 &&
      !nearest_lane->pre_lane_ids().empty()) {
    const auto &pre_lane =
        map->GetLaneById(nearest_lane->pre_lane_ids().front());
    if (pre_lane && !(pre_lane->junction_id() == 0)) {
      double dist_from_junction = std::numeric_limits<double>::max();
      nearest_lane->center_line().GetDistance({ego_pos.x(), ego_pos.y()},
                                              nullptr, &dist_from_junction);
      const double dist_from_junction_threshold = is_occupied_lc ? 10.0 : 30.0;
      if (dist_from_junction < dist_from_junction_threshold) {
        center_disable_code.set(15, true);
      }
    }
  }

  // 17. speed limit
  if (plan_start_point.v() < 10.0 * ad_byd::planning::Constants::KPH2MPS ||
      plan_start_point.v() > 120.0 * ad_byd::planning::Constants::KPH2MPS) {
    center_disable_code.set(17, true);
    Log2DDS::LogDataV2("center_lc_intent", "out speed limit");
    return {};
  }
  double dist_to_lc_solidline = DBL_MAX;
  if (center_command == ad_byd::planning::Command_LaneChangeLeft) {
    dist_to_lc_solidline = cur_lane_seq_info.dist_to_left_solid_line;
  }
  if (center_command == ad_byd::planning::Command_LaneChangeRight) {
    dist_to_lc_solidline = cur_lane_seq_info.dist_to_right_solid_line;
  }
  // 20. Check line type
  if (dist_to_lc_solidline < 15.0 + plan_start_point.v() * 3.0) {
    Log2DDS::LogDataV2("center_lc_intent", "[center] solidline check fail");
    center_disable_code.set(11, true);
    return {};
  }
  // 21. If target lane will merge to current lane
  if (center_tgt_lane_seq) {
    double merge_dist = std::numeric_limits<double>::max();
    const bool lane_will_merge = IfLaneSeqWillMergeCurLane(
        center_tgt_lane_seq, cur_lane, ego_pos, map, merge_dist);
    if (lane_will_merge && merge_dist < 300.0) {
      center_disable_code.set(21, true);
    }
  }

  if (center_tgt_lane_seq) {
    double s_offset = 0.0;
    const double l = center_tgt_lane_seq->GetProjectionDistance(
        {ego_pos.x(), ego_pos.y()}, &s_offset);
    const auto projection_point = center_tgt_lane_seq->GetPointAtS(s_offset);
    const double be_merged_dist = center_tgt_lane_seq->GetDistanceToPOI(
        ad_byd::planning::Poi_To_Be_Merged, projection_point.x(),
        projection_point.y());
    if (be_merged_dist < 300.0) {
      center_disable_code.set(22, true);
    }
  }

  // final decision

  Log2DDS::LogDataV2(
      "center_lc_intent",
      absl::StrCat("center_lc_disable_code: ", center_disable_code.to_ulong()));
  bool center_intent_flag = center_disable_code.none();
  //
  TargetLaneInfo target_lane_info;
  if (center_intent_flag) {
    Log2DDS::LogDataV2("center_lc_intent", "[center] center intent!");
    // if (is_occupied_lc) {
    //   decision->set_output_center_lc_type(LC_TYPE_FOR_NAVI);
    // }
    // if (is_edge_lc) {
    //   decision->set_output_center_lc_type(LC_TYPE_CENTER);
    // }
    // if (is_composite_lc) {
    //   decision->set_output_center_lc_type(LC_REASON_FOR_AVOID_MULTI_DIR);
    // }
    // if (is_variable_lc) {
    //   decision->set_output_center_lc_type(LC_REASON_FOR_AVOID_VARIABLE);
    // }
    // if (is_emergency_lc) {
    //   decision->set_output_center_lc_type(LC_TYPE_FOR_NAVI);
    // }
    // if (is_reversible_lc) {
    //   decision->set_output_center_lc_type(LC_REASON_FOR_AVOID_LANE_REVERSIBLE);
    // }
    // if (is_bus_lc) {
    //   decision->set_output_center_lc_type(LC_REASON_FOR_AVOID_LANE_BUS);
    // }
    target_lane_info.lc_reason = ad_byd::planning::LC_REASON_CENTER;
    target_lane_info.lc_command = center_command;
    target_lane_info.target_lane_seq = center_tgt_lane_seq;
  }
  //
  return target_lane_info;
}

TargetLaneInfo SelectLaneMerge(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos,
    ApolloTrajectoryPointProto plan_start_point) {
  ad_byd::planning::LaneSeqInfo cur_lane_seq_info;
  ad_byd::planning::LaneSeqInfo left_lane_seq_info;
  ad_byd::planning::LaneSeqInfo right_lane_seq_info;
  ad_byd::planning::LaneSeqInfo tgt_lane_seq_info;
  for (const auto &candidate_seq_info : lane_seq_infos) {
    if (candidate_seq_info.is_current &&
        candidate_seq_info.lc_dir == ad_byd::planning::Command_Invalid) {
      cur_lane_seq_info = candidate_seq_info;
    } else if (candidate_seq_info.lc_dir ==
               ad_byd::planning::Command_LaneChangeLeft) {
      left_lane_seq_info = candidate_seq_info;
    } else if (candidate_seq_info.lc_dir ==
               ad_byd::planning::Command_LaneChangeRight) {
      right_lane_seq_info = candidate_seq_info;
    }
  }
  auto cur_lane_seq = cur_lane_seq_info.lane_seq;
  if (!cur_lane_seq) {
    Log2DDS::LogDataV2("merge_lc_intent", "[merge] no cur lane seq");
    return {};
  }

  std::bitset<32> merge_intent_code;

  // check merge dist
  if (cur_lane_seq_info.dist_to_merge > 200.0) {
    merge_intent_code.set(1, true);
    Log2DDS::LogDataV2("merge_lc_intent", "[merge] too far");
    return {};
  }
  if (cur_lane_seq_info.dist_to_merge < 50.0) {
    merge_intent_code.set(2, true);
    Log2DDS::LogDataV2("merge_lc_intent", "[merge] too close");
    return {};
  }
  if (!cur_lane_seq_info.merge_lane) {
    merge_intent_code.set(3, true);
    Log2DDS::LogDataV2("merge_lc_intent", "[merge] no merge lane");
    return {};
  }

  // check if beside lane seq is target lane seq
  ad_byd::planning::LaneConstPtr beside_lane = nullptr;
  ad_byd::planning::LaneConstPtr merged_lane = nullptr;
  if (cur_lane_seq_info.merge_command ==
      ad_byd::planning::Command_LaneChangeLeft) {
    beside_lane = map->GetLeftLane(cur_lane_seq_info.nearest_lane);
    merged_lane = map->GetLeftLane(cur_lane_seq_info.merge_lane);
  } else if (cur_lane_seq_info.merge_command ==
             ad_byd::planning::Command_LaneChangeRight) {
    beside_lane = map->GetRightLane(cur_lane_seq_info.nearest_lane);
    merged_lane = map->GetRightLane(cur_lane_seq_info.merge_lane);
  }
  if (!IfOnSameLaneSequence(map, beside_lane, merged_lane)) {
    merge_intent_code.set(4, true);
    Log2DDS::LogDataV2("merge_lc_intent", "[merge] not on same laneseq");
    return {};
  }
  // target lane seq
  double dist_to_lc_solidline = std::numeric_limits<double>::infinity();
  ad_byd::planning::LaneSequencePtr target_lane_seq = nullptr;
  if (cur_lane_seq_info.merge_command ==
      ad_byd::planning::Command_LaneChangeLeft) {
    target_lane_seq = left_lane_seq_info.lane_seq;
    tgt_lane_seq_info = left_lane_seq_info;
    dist_to_lc_solidline = cur_lane_seq_info.dist_to_left_solid_line;
  } else if (cur_lane_seq_info.merge_command ==
             ad_byd::planning::Command_LaneChangeRight) {
    target_lane_seq = right_lane_seq_info.lane_seq;
    tgt_lane_seq_info = right_lane_seq_info;
    dist_to_lc_solidline = cur_lane_seq_info.dist_to_right_solid_line;
  }
  if (!target_lane_seq) {
    merge_intent_code.set(5, true);
    Log2DDS::LogDataV2("merge_lc_intent", "[merge] no tgt lane seq");
    return {};
  }

  // if target lane will merge to current lane
  if (target_lane_seq) {
    double merge_dist = std::numeric_limits<double>::max();
    const bool lane_will_merge = IfLaneSeqWillMergeCurLane(
        target_lane_seq, cur_lane_seq_info.nearest_lane, ego_pos, map,
        merge_dist);
    if (lane_will_merge && merge_dist < 200.0) {
      merge_intent_code.set(8, true);
      Log2DDS::LogDataV2("merge_lc_intent", "[merge] will merge to cur");
      return {};
    }
  }
  // check taget lane type
  if (target_lane_seq) {
    const auto nearest_lane =
        target_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
    if (nearest_lane &&
        nearest_lane->type() == ad_byd::planning::LaneType::LANE_UNKNOWN) {
      merge_intent_code.set(9, true);
      Log2DDS::LogDataV2("merge_lc_intent", "[merge] wrong lane type");
      return {};
    }
  }

  // check junction
  if (target_lane_seq) {
    const auto nearest_lane =
        target_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
    if (nearest_lane && !(nearest_lane->junction_id() == 0)) {
      merge_intent_code.set(10, true);
    } else if (nearest_lane && !nearest_lane->pre_lane_ids().empty()) {
      const auto parent_lane =
          map->GetLaneById(nearest_lane->pre_lane_ids().front());
      double dist_from_junction = std::numeric_limits<double>::max();
      nearest_lane->center_line().GetDistance({ego_pos.x(), ego_pos.y()},
                                              nullptr, &dist_from_junction);
      if ((!(parent_lane->junction_id() == 0) &&
           parent_lane->turn_type() == ad_byd::planning::LEFT_TURN) ||
          (!(parent_lane->junction_id() == 0) &&
           parent_lane->turn_type() == ad_byd::planning::RIGHT_TURN)) {
        if (cur_lane_seq_info.dist_to_navi_end < 50.0) {
          if (dist_from_junction < 10.0) {
            merge_intent_code.set(10, true);
          }
        }
        if (cur_lane_seq_info.dist_to_navi_end > 50.0) {
          if (dist_from_junction < 10.0) {
            merge_intent_code.set(10, true);
          } else if (dist_from_junction < 20.0) {
            merge_intent_code.set(10, true);
          }
        }
      }
      if (!(parent_lane->junction_id() == 0) &&
          parent_lane->turn_type() == ad_byd::planning::NO_TURN) {
        if (cur_lane_seq_info.dist_to_navi_end > 50.0 &&
            dist_from_junction < 10.0) {
          merge_intent_code.set(10, true);
        }
      }
    }

    if (cur_lane_seq_info.dist_to_junction < 35.0) {
      merge_intent_code.set(10, true);
    }
  }

  // check solid line
  if (dist_to_lc_solidline < 15.0 + plan_start_point.v() * 3.0) {
    Log2DDS::LogDataV2("merge_lc_intent", "[merge] solidline check fail");
    merge_intent_code.set(11, true);
    return {};
  }

  Log2DDS::LogDataV2("merge_lc_intent",
                     absl::StrCat("[merge] merge intent disable code: ",
                                  merge_intent_code.to_ulong()));
  // final dicision
  bool merge_intent_flag = merge_intent_code.none();
  TargetLaneInfo target_lane_info;
  if (merge_intent_flag) {
    Log2DDS::LogDataV2("merge_lc_intent", "[merge] merge intent ");
    target_lane_info.lc_reason = ad_byd::planning::LC_REASON_FOR_MERGE;
    target_lane_info.lc_command = cur_lane_seq_info.merge_command;
    target_lane_info.target_lane_seq = target_lane_seq;
  }
  return target_lane_info;
}

TargetLaneInfo SelectLaneBlocker(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos,
    const st::ApolloTrajectoryPointProto *plan_start_point,
    const VehicleParamsProto *vehicle_param,
    const PlannerSemanticMapManager *psmm,
    const std::optional<Behavior_FunctionId> function_id,
    const std::optional<int> tunnel_status, const PlannerObjectManager &obj_mgr,
    const LaneSequencePtr &pre_target_laneseq,
    const LaneChangeStateProto *lane_change_state,
    const LaneChangeReason &lane_change_reason,
    const DriverAction::LaneChangeCommand &lane_change_command,
    const LaneConstPtr &nearest_lane) {
  TargetLaneInfo target_lane_info;

  const bool blocker_bool =
      lane_change_state->stage() != LaneChangeStage::LCS_NONE &&
      lane_change_command == DriverAction::LC_CMD_NONE &&
      lane_change_reason == AVOID_VEHICLE_CHANGE;
  Log2DDS::LogDataV2("blocker_bool", absl::StrCat(blocker_bool));

  if (blocker_bool) {
    target_lane_info.target_lane_seq = pre_target_laneseq;
    target_lane_info.lc_reason =
        ad_byd::planning::LcReason::LC_REASON_FOR_AVOID_VEHICLE;
    return target_lane_info;
  }
  // 3. Select target laneseq
  LaneConstPtr left_lane = map->GetLeftLane(nearest_lane);
  LaneSequencePtr left_laneseq = nullptr;
  if (IsValidLane(left_lane)) {
    left_laneseq = map->GetLaneSequence(left_lane, true);
  }
  LaneConstPtr right_lane = map->GetRightLane(nearest_lane);
  LaneSequencePtr right_laneseq = nullptr;
  if (IsValidLane(right_lane)) {
    right_laneseq = map->GetLaneSequence(right_lane, true);
  }
  if (blocker_bool && (left_laneseq || right_laneseq)) {
    Log2DDS::LogDataV2("target_lane_seq", "target lane seq is seted");
    int left_navi_section_cnt = 0;
    double left_efficiency = 0.0;
    CountNaviAndEfficiencyInfo(ego_pos, left_laneseq, obj_mgr,
                               &left_navi_section_cnt, &left_efficiency);
    int right_navi_section_cnt = 0;
    double right_efficiency = 0.0;
    CountNaviAndEfficiencyInfo(ego_pos, right_laneseq, obj_mgr,
                               &right_navi_section_cnt, &right_efficiency);
    target_lane_info.target_lane_seq =
        left_efficiency > right_efficiency ? left_laneseq : right_laneseq;
    if (blocker_bool) {
      target_lane_info.lc_reason =
          ad_byd::planning::LcReason::LC_REASON_FOR_AVOID_VEHICLE;
    }
  }
  // final decision
  return target_lane_info;
}

TargetLaneInfo SelectLaneOverTake(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos,
    const std::optional<double> cruising_speed_limit,
    const PlannerObjectManager &obj_mgr,
    ApolloTrajectoryPointProto plan_start_point,
    const std::string &leading_id) {
  ad_byd::planning::LaneSeqInfo cur_lane_seq_info;
  ad_byd::planning::LaneSeqInfo left_lane_seq_info;
  ad_byd::planning::LaneSeqInfo right_lane_seq_info;
  ad_byd::planning::LaneSeqInfo tgt_lane_seq_info;
  const auto left_target_lane_seq = left_lane_seq_info.lane_seq;
  const auto right_target_lane_seq = right_lane_seq_info.lane_seq;
  for (const auto &candidate_seq_info : lane_seq_infos) {
    if (candidate_seq_info.is_current &&
        candidate_seq_info.lc_dir == ad_byd::planning::Command_Invalid) {
      cur_lane_seq_info = candidate_seq_info;
    } else if (candidate_seq_info.lc_dir ==
               ad_byd::planning::Command_LaneChangeLeft) {
      left_lane_seq_info = candidate_seq_info;
    } else if (candidate_seq_info.lc_dir ==
               ad_byd::planning::Command_LaneChangeRight) {
      right_lane_seq_info = candidate_seq_info;
    }
  }
  auto cur_lane_seq = cur_lane_seq_info.lane_seq;
  if (!cur_lane_seq) {
    Log2DDS::LogDataV2("overtake_lc_intent", "[overtake] no cur lane seq");
    return {};
  }
  std::bitset<32> overtake_motive_code;
  std::bitset<32> overtake_disable_code;
  // 1. overtake motive check
  // 1.1 leading check
  // q1:leading检查
  // 代码架构和顺序的问题，没办法在scheduler之前拿到leading障碍物，且上一帧的也没存拿下来，通过标签拿到的leading障碍物不唯一
  // 想拿到准确的leading障碍物的话需要把search
  // motion模块的output重新增加一个变量存到input里
  const st::planning::PlannerObject *leading_object = nullptr;
  if (!leading_id.empty()) {
    leading_object = obj_mgr.FindObjectById(leading_id);
  } else {
    overtake_motive_code.set(0, true);
    Log2DDS::LogDataV2("overtake_lc_intent", "[overtake] no leading");
    return {};
  }
  double dist_to_junction = cur_lane_seq_info.dist_to_junction;
  double s_object_offset = 0.0;
  double s_ego_offset = 0.0;
  if (cur_lane_seq) {
    const double l = cur_lane_seq->GetProjectionDistance(
        {leading_object->aabox().center_x(),
         leading_object->aabox().center_y()},
        &s_object_offset);
    const double l1 = cur_lane_seq->GetProjectionDistance(
        {ego_pos.x(), ego_pos.y()}, &s_ego_offset);
  }
  double leading_ds = s_object_offset - s_ego_offset;

  if (leading_object &&
      leading_ds >
          std::fmin(
              dist_to_junction,
              std::fmax(50.0, std::fmin(120.0, plan_start_point.v() * 4.0)))) {
    overtake_motive_code.set(1, true);
    Log2DDS::LogDataV2("overtake_lc_intent",
                       "[overtake] far away from leading");
    return {};
  }
  // 1.2 junction check
  if (dist_to_junction < 100.0) {
    overtake_motive_code.set(2, true);
    Log2DDS::LogDataV2("overtake_lc_intent", "[overtake] near junction");
    return {};
  }
  // 1.3 delta v check
  const auto &driver_set_speed = cruising_speed_limit;
  double speed_delta_thr_kph = 15.0;
  if (leading_object && driver_set_speed &&
      (*driver_set_speed - leading_object->pose().v() * 3.6 <
       speed_delta_thr_kph)) {
    overtake_motive_code.set(3, true);
    Log2DDS::LogDataV2("overtake_lc_intent",
                       "[overtake] delta v is not high enough");
    return {};
  }

  if (leading_object &&
      plan_start_point.v() <
          leading_object->pose().v() - 2.0) {  // consider delete TODO::
    Log2DDS::LogDataV2("overtake_lc_intent",
                       "[overtake] delta v is lower than 2m/s");
    overtake_motive_code.set(4, true);
    return {};
  }
  // 2. overtake feasibility check
  // 2.1 tgt lane seq check

  double overtake_enable_dist_to_split = 100.0;
  int left_navi_section_cnt = 0;
  double left_efficiency = 0.0;
  int right_navi_section_cnt = 0;
  double right_efficiency = 0.0;
  const auto &cur_lane_position = cur_lane_seq_info.cur_lane_position;
  const auto &cur_section_lane_num = cur_lane_seq_info.cur_section_lane_num;
  const auto cur_lane = cur_lane_seq_info.nearest_lane;
  LaneConstPtr overtake_target_lane = nullptr;
  LaneSequencePtr overtake_tgt_lane_seq = nullptr;
  ad_byd::planning::BehaviorCommand overtake_command =
      ad_byd::planning::Command_Invalid;
  CountNaviAndEfficiencyInfo(ego_pos, left_target_lane_seq, obj_mgr,
                             &left_navi_section_cnt, &left_efficiency);
  CountNaviAndEfficiencyInfo(ego_pos, right_target_lane_seq, obj_mgr,
                             &right_navi_section_cnt, &right_efficiency);
  if (cur_section_lane_num >= 2 && cur_lane_position != 0) {
    if (cur_lane_position == cur_section_lane_num) {
      overtake_target_lane = map->GetLeftLane(cur_lane);
      overtake_tgt_lane_seq = left_target_lane_seq;
      overtake_command = ad_byd::planning::Command_LaneChangeLeft;
    } else if (cur_lane_position == 1) {
      overtake_target_lane = map->GetRightLane(cur_lane);
      overtake_tgt_lane_seq = right_target_lane_seq;
      overtake_command = ad_byd::planning::Command_LaneChangeRight;

    } else {
      if (cur_lane_seq_info.dist_to_navi_end > overtake_enable_dist_to_split) {
        if (right_efficiency - left_efficiency >= 1.40) {
          overtake_target_lane = map->GetRightLane(cur_lane);
          overtake_tgt_lane_seq = right_target_lane_seq;
          overtake_command = ad_byd::planning::Command_LaneChangeRight;
        } else {
          overtake_target_lane = map->GetLeftLane(cur_lane);
          overtake_tgt_lane_seq = left_target_lane_seq;
          overtake_command = ad_byd::planning::Command_LaneChangeLeft;
        }
      } else {
        if (cur_lane_seq_info.navi_lc_command ==
            ad_byd::planning::Command_LaneChangeRight) {
          overtake_target_lane = map->GetRightLane(cur_lane);
          overtake_tgt_lane_seq = right_target_lane_seq;
        } else if (cur_lane_seq_info.navi_lc_command ==
                   ad_byd::planning::Command_LaneChangeLeft) {
          overtake_target_lane = map->GetLeftLane(cur_lane);
          overtake_tgt_lane_seq = left_target_lane_seq;
        }
      }
    }
  }

  if (!overtake_target_lane || !overtake_tgt_lane_seq) {
    overtake_disable_code.set(0, true);
    Log2DDS::LogDataV2("overtake_lc_intent", "[overtake] no target lane!");
  }
  // 2.2
  double target_navi_end_dist = DBL_MAX;
  bool is_overtake_VRU =
      false;  // A closed function ifovertakevru in origional code
  if (overtake_tgt_lane_seq) {
    LaneConstPtr poi_lane;
    target_navi_end_dist =
        std::fmin(cur_lane_seq_info.dist_to_split,
                  overtake_tgt_lane_seq->GetDistanceToPOI(
                      ad_byd::planning::Poi_NaviEnd, ego_pos.x(), ego_pos.y(),
                      poi_lane, &map->route()->navi_start()));
  }
  const auto pri_lane_relation = cur_lane_seq_info.lc_num;
  if (overtake_command != cur_lane_seq_info.navi_lc_command) {
    double navi_lc_dist_thr = 2500.0;
    switch (std::abs(pri_lane_relation)) {
      case 0:
        navi_lc_dist_thr = -2500.0;
        break;
      case 1:
        navi_lc_dist_thr =
            300.0;  // city fence（wangjing）is seted in origional code
        break;
      case 2:
        navi_lc_dist_thr = 900;
        break;
      case 3:
        navi_lc_dist_thr = 1100;
        break;
      default:
        break;
    }
    if (target_navi_end_dist < navi_lc_dist_thr + 200.0) {
      Log2DDS::LogDataV2("overtake_lc_debug", "near navi lc point");
      if (!is_overtake_VRU) {
        overtake_disable_code.set(1, true);
      }
    } else {
      Log2DDS::LogDataV2(
          "overtake_lc_debug",
          absl::StrCat("target_navi_end_dist:", target_navi_end_dist,
                       " navi_lc_dist_thr:", navi_lc_dist_thr));
    }
  }

  // 2.3 near merge point
  if (overtake_tgt_lane_seq) {
    const auto nearest_lane =
        overtake_tgt_lane_seq->GetNearestLane({ego_pos.x(), ego_pos.y()});
    double merge_dist = std::numeric_limits<double>::max();
    const bool will_merge_current = IfLaneSeqWillMergeCurLane(
        overtake_tgt_lane_seq, cur_lane_seq_info.nearest_lane, ego_pos, map,
        merge_dist);
    if (will_merge_current && merge_dist < 500.0) {
      overtake_disable_code.set(2, true);
      Log2DDS::LogDataV2("overtake_lc_debug", "near merge point");
      return {};
    }
  }

  // 2.4 tgt lane exit vru
  //  if (overtake_tgt_lane_seq) {
  //    std::vector<ObstaclePtr> target_lane_obs;
  //    double target_lane_width = Constants::MIN_HALF_LANE_WIDTH;
  //    if (cur_lane_seq_info.overtake_lc_command ==
  //        ad_byd::planning::Command_LaneChangeLeft) {
  //      target_lane_obs = GetSelectedLaneObstacles(decision, 1);
  //      target_lane_width = map_supplement.left_lane_width;
  //    } else if (cur_lane_seq_info.overtake_lc_command ==
  //               ad_byd::planning::Command_LaneChangeRight) {
  //      target_lane_obs = GetSelectedLaneObstacles(decision, -1);
  //      target_lane_width = map_supplement.right_lane_width;
  //    }
  //    for (const ObstaclePtr obstacle : target_lane_obs) {
  //      if (obstacle->type() != OBJECT_BICYCLE &&
  //          obstacle->type() != OBJECT_PEDESTRIAN &&
  //          obstacle->type() != OBJECT_TRICYCLE &&
  //          obstacle->type() != OBJECT_MOTORCYCLE) {
  //        continue;
  //      }
  //      if (obstacle->ds() > std::max(20.0, start_point.v * 8.0)) break;
  //      if (obstacle->ds() > 0.0 &&
  //          obstacle->v() < start_point.v + 10.0 * Constants::KPH2MPS &&
  //          std::fmin(std::fabs(obstacle->sl_boundary().l_max),
  //                    std::fabs(obstacle->sl_boundary().l_min)) <
  //              target_lane_width * 0.5 - 0.8) {
  //        LOGDATA("overtake_lc_debug", "target lane VRU exits");
  //        overtake_disable_code.set(3, true);
  //      }
  //    }
  //  }

  // 2.5 bus lane
  if (overtake_tgt_lane_seq) {
    ad_byd::planning::LaneConstPtr bus_lane = nullptr;
    const std::unordered_set<ad_byd::planning::LaneType> lane_type_set{
        ad_byd::planning::LANE_BUS_NORMAL};
    double dist_to_bus_lane =
        overtake_tgt_lane_seq->GetDistanceToTargetLaneType(
            bus_lane, lane_type_set, map->route()->navi_start());
    if (overtake_command == ad_byd::planning::Command_LaneChangeRight &&
        bus_lane &&
        dist_to_bus_lane < std::max(5.0 * plan_start_point.v(), 50.0)) {
      Log2DDS::LogDataV2("overtake_lc_debug",
                         "[overtake]right target lane near bus lane");
      overtake_disable_code.set(4, true);
    }
  }
  // 2.6 not normal lane type
  auto left_lane = map->GetLeftLane(cur_lane);
  auto right_lane = map->GetRightLane(cur_lane);
  if (overtake_command == ad_byd::planning::Command_LaneChangeLeft &&
      left_lane->type() != ad_byd::planning::LANE_NORMAL &&
      left_lane->type() != ad_byd::planning::LANE_HOV_NORMAL) {
    Log2DDS::LogDataV2("overtake_lc_debug",
                       "[overtake]target lane type not normal");
    overtake_disable_code.set(5, true);
    return {};
  }
  if (overtake_command == ad_byd::planning::Command_LaneChangeRight &&
      right_lane->type() != ad_byd::planning::LANE_NORMAL &&
      right_lane->type() != ad_byd::planning::LANE_HOV_NORMAL) {
    Log2DDS::LogDataV2("overtake_lc_debug",
                       "[overtake]target lane type not normal");
    overtake_disable_code.set(5, true);
    return {};
  }
  // 2.7 special lane id
  //  if (target_lane_seq && (overtake_tgt_lane->id() == "3456262518588109122"
  //  ||
  //                          overtake_tgt_lane->id() ==
  //                          "16280910268474439483")) {
  //    overtake_disable_code.set(6, true);
  //  }

  // 2.8 route check
  const double navi_remain_dist_thr = 50.0;
  if (cur_lane_seq_info.dist_to_navi_end < navi_remain_dist_thr) {
    overtake_disable_code.set(7, true);
    Log2DDS::LogDataV2("overtake_lc_debug", "[overtake]route check failed");
  }
  // 2.9 区分functionid
  double navi_lc_dist_thr = 2500.0;
  switch (std::abs(pri_lane_relation)) {
    case 0:
      navi_lc_dist_thr = -2500.0;
      break;
    case 1:
      navi_lc_dist_thr = 1000.0;
      break;
    case 2:
      navi_lc_dist_thr = 1500.0;
      break;
    case 3:
      navi_lc_dist_thr = 2000;
      break;
    default:
      break;
  }
  double extra_dist = 0.0;
  if (overtake_command != cur_lane_seq_info.navi_lc_command) {
    extra_dist = plan_start_point.v() * 10.0;
  }
  if (dist_to_junction < 400.0 &&
      (overtake_tgt_lane_seq && !overtake_tgt_lane_seq->CanPassJunction(
                                    map->route()->navi_start(), NULL, true))) {
    Log2DDS::LogDataV2("overtake_lc_debug",
                       "[overtake]target lane sequence is not navi");
    overtake_disable_code.set(8, true);
    return {};
  } else {
    Log2DDS::LogDataV2("overtake_lc_debug",
                       "[overtake]target lane sequence is navi");
  }
  // 2.10

  // dist to navi split
  if (cur_lane_seq_info.dist_to_navi_end <= overtake_enable_dist_to_split &&
      overtake_command != cur_lane_seq_info.navi_lc_command) {
    Log2DDS::LogDataV2("overtake_lc_debug", "near split point");
    overtake_disable_code.set(8, true);
    return {};
  }
  // 2.11 effenciency check
  if (overtake_command == ad_byd::planning::Command_LaneChangeLeft) {
    if (plan_start_point.v() < 5.0 / 3.6) {
      if (cur_lane_position == 2) {
        if (left_efficiency - plan_start_point.v() < 4.78) {
          overtake_disable_code.set(9, true);
          return {};
        }
      } else {
        if (left_efficiency - plan_start_point.v() < 3.78) {
          overtake_disable_code.set(9, true);
          return {};
        }
      }
    } else {
      if (cur_lane_position == 2) {
        if (left_efficiency - plan_start_point.v() < 3.78) {
          overtake_disable_code.set(9, true);
          return {};
        }
      } else {
        if (left_efficiency - plan_start_point.v() < 2.78) {
          overtake_disable_code.set(9, true);
          return {};
        }
      }
    }
  } else if (overtake_command == ad_byd::planning::Command_LaneChangeRight) {
    if (plan_start_point.v() < 5.0 / 3.6) {
      if (cur_lane_position == cur_section_lane_num - 1) {
        if (right_efficiency - plan_start_point.v() < 4.78) {
          overtake_disable_code.set(9, true);
          return {};
        }
      } else {
        if (right_efficiency - plan_start_point.v() < 3.78) {
          overtake_disable_code.set(9, true);
          return {};
        }
      }
    } else {
      if (cur_lane_position == cur_section_lane_num - 1) {
        if (right_efficiency - plan_start_point.v() < 3.78) {
          overtake_disable_code.set(9, true);
          return {};
        }
      } else {
        if (right_efficiency - plan_start_point.v() < 2.78) {
          overtake_disable_code.set(9, true);
          return {};
        }
      }
    }
  }
  //
  if (overtake_command == ad_byd::planning::Command_LaneChangeRight) {
    const auto &right_lane = map->GetRightLane(cur_lane);
    const auto &r_r_lane = map->GetRightLane(cur_lane);
    if (r_r_lane && r_r_lane->type() == ad_byd::planning::LANE_ACC) {
      overtake_disable_code.set(10, true);
      return {};
    }
  }

  // 2.14 target lane under construction
  //  if (overtake_lc_command == Command_LaneChangeLeft &&
  //      map_supplement.left_under_construction) {
  //    overtake_disable_code.set(12, true);
  //  } else if (overtake_lc_command == Command_LaneChangeRight &&
  //             map_supplement.right_under_construction) {
  //    overtake_disable_code.set(12, true);
  //  }

  // 2.15 if target lane will merge to current lane
  if (overtake_command == ad_byd::planning::Command_LaneChangeRight &&
      overtake_tgt_lane_seq) {
    double s_offset = 0.0;
    const double l = overtake_tgt_lane_seq->GetProjectionDistance(
        {ego_pos.x(), ego_pos.y()}, &s_offset);
    const auto projection_point = overtake_tgt_lane_seq->GetPointAtS(s_offset);
    const double be_merged_dist = overtake_tgt_lane_seq->GetDistanceToPOI(
        ad_byd::planning::Poi_To_Be_Merged, projection_point.x(),
        projection_point.y());
    if (be_merged_dist < 80.0) {
      Log2DDS::LogDataV2("overtake_lc_debug",
                         "[overtake]near to be merge point");
      overtake_disable_code.set(14, true);
    }
  }

  // 2.16 near split near merge
  if (overtake_target_lane) {
    double merge_dist = std::numeric_limits<double>::max();
    const bool lane_will_merge = IfLaneSeqWillMergeCurLane(
        overtake_tgt_lane_seq, cur_lane_seq_info.nearest_lane, ego_pos, map,
        merge_dist);
    if (lane_will_merge && merge_dist < 100.0) {
      Log2DDS::LogDataV2("overtake_lc_debug", "[overtake]near merge point");
      overtake_disable_code.set(15, true);
      return {};
    }
    if (cur_lane_seq_info.dist_to_split < 100.0) {
      Log2DDS::LogDataV2("overtake_lc_debug", "[overtake]near split point");
      overtake_disable_code.set(15, true);
      return {};
    }
  }

  //  2.17 check line type
  double dist_to_lc_solidline = std::numeric_limits<double>::infinity();
  if (overtake_command == ad_byd::planning::Command_LaneChangeLeft &&
      left_lane_seq_info.lane_seq) {
    dist_to_lc_solidline = cur_lane_seq_info.dist_to_left_solid_line;
  } else {
    Log2DDS::LogDataV2("overtake_lc_intent", "[overtake] no left lane seq");
  }
  if (overtake_command == ad_byd::planning::Command_LaneChangeRight &&
      right_lane_seq_info.lane_seq) {
    dist_to_lc_solidline = cur_lane_seq_info.dist_to_right_solid_line;
  } else {
    Log2DDS::LogDataV2("overtake_lc_intent", "[overtake] no right lane seq");
  }
  if (dist_to_lc_solidline < 15.0 + plan_start_point.v() * 3.0) {
    overtake_disable_code.set(16, true);
    Log2DDS::LogDataV2("overtake_lc_intent", "[overtake] solidline check fail");
  }

  // 3. final dicision
  bool overtake_motive_flag = overtake_motive_code.none();
  bool overtake_enable_flag = overtake_disable_code.none();
  TargetLaneInfo target_lane_info;
  if (overtake_motive_flag && overtake_enable_flag) {
    target_lane_info.lc_reason = ad_byd::planning::LC_REASON_OVERTAKE;
    target_lane_info.lc_command = overtake_command;
    ad_byd::planning::LaneSequencePtr target_lane_seq = nullptr;
    target_lane_info.target_lane_seq = target_lane_seq;
  }
  return target_lane_info;
}

}  // namespace st::planning
