#include "decider/scheduler/fsd_lane_selector.h"

#include "plan_common/math/util.h"
#include "plan_common/log_data.h"
#include "plan_common/planning_macros.h"
namespace ad_byd {
namespace planning {
inline bool IsBusLane(LaneConstPtr lane) {
  if (!lane) return false;
  if (lane->type() == LaneType::LANE_BUS_NORMAL ||
      lane->type() == LaneType::LANE_HARBOR_STOP ||
      lane->type() == LaneType::LANE_BRT) {
    return true;
  }
  return false;
}
void CalcuSplitState(const LaneConstPtr extend_split_lane,
                     const uint64_t lane_id, const SplitDirection &dir,
                     SplitTasksState &seqs_split_state) {
  constexpr int none_split_task = 0;
  constexpr int left_split_task = 1;
  constexpr int right_split_task = 2;
  if (extend_split_lane && extend_split_lane->id() == lane_id) {
    if (dir == SplitDirection::Left) {
      seqs_split_state = SplitTasksState::Right_Task;
    } else if (dir == SplitDirection::Right) {
      seqs_split_state = SplitTasksState::Left_Task;
    } else {
      seqs_split_state = SplitTasksState::None;
    }
  } else {
    seqs_split_state = SplitTasksState::None;
  }
}

bool IsPrebuildLeftSeqOnRightTurn(const Vec2d &ego_pos,
                                  const LaneSequencePtr current_seq,
                                  const LaneConstPtr tgt_lane,
                                  uint64_t &build_lane_id) {
  constexpr double kDistToLeftStartLane = 30.0;
  bool find_start_lane = false;
  bool is_exist_left_lane = false;
  double dist_to_left_start_lane = 0.0;
  if (!tgt_lane || !tgt_lane->center_line().IsValid()) {
    return false;
  }
  for (auto lane : current_seq->lanes()) {
    if (!lane || !lane->center_line().IsValid()) continue;
    if (lane->turn_type() == ad_byd::planning::TurnType::RIGHT_TURN) {
      is_exist_left_lane = true;
    }
    if (!find_start_lane && lane->id() == tgt_lane->id()) {
      find_start_lane = true;
    }
    if (!find_start_lane) continue;
    if (dist_to_left_start_lane > kDistToLeftStartLane) break;
    if (lane->id() == tgt_lane->id()) {
      double s, l;
      if (lane->center_line().GetProjection(ego_pos, &s, &l)) {
        dist_to_left_start_lane = lane->center_line().length() - s;
      }
    } else {
      dist_to_left_start_lane += lane->topo_length();
    }
    if (is_exist_left_lane && lane->left_lane_id() != 0) {
      build_lane_id = lane->left_lane_id();
      break;
    }
  }
  return is_exist_left_lane;
}

LaneConstPtr GetNextLRLane(const LaneConstPtr &lane, const int lane_change_sign,
                           const MapPtr &map) {
  if (!lane) {
    return nullptr;
  }
  const auto &next_lanes = map->GetNextLanes(lane);
  if (next_lanes.empty()) {
    return nullptr;
  }
  if (next_lanes.size() == 1) {
    return next_lanes.front();
  }

  // if there is a split, get the leftmost or rightmost lane according to the
  // lane change direction. 1, change left; 2, change right.
  if (lane_change_sign == 1) return map->GetLeftmostValidNextLanes(lane, true);
  return map->GetRightmostValidNextLanes(lane, true);
}

void GenerateCurrentLaneSequences(const Vec2d &start_point,
                                  const double start_point_v,
                                  const LaneConstPtr &start_lane,
                                  const st::Behavior_FunctionId function_id,
                                  const MapPtr &map, int lane_change_sign,
                                  LaneSequencePtr &cur_lane_sequence,
                                  const uint64_t split_lane_id = 0) {
  const auto &nearest_lane = start_lane;
  if (!nearest_lane) {
    LOG_ERROR << "cannot generate lane sequence: nearest_lane donot exist!!!";
    return;
  }

  if (!cur_lane_sequence) {
    std::vector<LaneConstPtr> t_lanes;
    t_lanes.emplace_back(nearest_lane);
    cur_lane_sequence = std::make_shared<LaneSequence>(t_lanes);
  }

  const auto &last_lanes = cur_lane_sequence;
  if (!last_lanes) {
    LOG_ERROR
        << "cannot generate lane sequence: cur_lane sequence donot exist!!!";
    return;
  }

  // step 1. get lanes-seq from last_lanes
  auto lanes = last_lanes->lanes();
  bool find_nearest_lane = false;
  bool find_far_lane = false;
  // double split_point_min_range = std::fmax(start_point_v * 2.0, 7.0);
  double split_point_min_range = 2.0;

  int save_start_idx = -1;
  int save_end_idx = lanes.size();
  if (function_id == st::Behavior_FunctionId_HW_NOA) {
    for (size_t idx = 0; idx < lanes.size(); idx++) {
      auto it = &lanes.at(idx);
      if (!find_nearest_lane && (*it)->id() == nearest_lane->id()) {
        find_nearest_lane = true;
        save_start_idx = idx;
      }
      if (find_nearest_lane) {
        if (!find_far_lane && (*it)->center_line().end_point().DistanceTo(
                                  start_point) > split_point_min_range) {
          find_far_lane = true;
          save_end_idx = idx;
          break;
        }
      }
    }

    if (save_start_idx > 0) {
      lanes.erase(lanes.begin(), lanes.begin() + save_start_idx);
    }
    if (save_end_idx < lanes.size()) {
      lanes.erase(lanes.begin() + (save_end_idx - save_start_idx + 1),
                  lanes.end());
    }
  }

  // step 2. when can't get lane-seq from last lanes, get it from nearest_lane.
  if (!find_nearest_lane) {
    lanes.clear();
    lanes.emplace_back(nearest_lane);
  }

  // step 3. get next lane forward until end
  LaneConstPtr next_lane = lanes.back();

  int iter_time = 0;
  std::unordered_set<uint64_t> check_lanes;
  while (true) {
    if (lane_change_sign == 0) {
      if (map->is_on_highway() ||
          function_id == st::Behavior_FunctionId_HW_NOA) {
        next_lane = map->GetOptimalNextLane(next_lane, true, split_lane_id);
      } else if (function_id == st::Behavior_FunctionId_MAPLESS_NOA) {
        next_lane = map->GetOptimalNextLaneMaplessNoa(next_lane, true);
      } else if (function_id == st::Behavior_FunctionId_LKA_PLUS) {
        next_lane = map->GetSmoothNextLane(next_lane->id());
      } else {
        next_lane = map->GetContinueNextLane(next_lane->id());
      }
    } else {
      next_lane = GetNextLRLane(next_lane, lane_change_sign, map);
    }

    ++iter_time;
    if ((!next_lane) || next_lane->id() == 0 ||
        check_lanes.count(next_lane->id()) > 0 || iter_time > 10000) {
      break;
    }

    check_lanes.insert(next_lane->id());
    lanes.emplace_back(next_lane);
  };

  cur_lane_sequence = std::make_shared<LaneSequence>(lanes);
}

bool CalcuSucceedLaneIndex(const MapPtr &map, LaneConstPtr &suc_lane,
                           int &suc_left_index, int &suc_right_index) {
  if (!map || !suc_lane) {
    return false;
  }

  const std::vector<LaneType> black_list = {LANE_NON_MOTOR, LANE_UNKNOWN,
                                            LANE_EMERGENCY};

  // index from left in quit junction section
  auto get_same_split_branches =
      [map](const ad_byd::planning::LaneConstPtr &lane)
      -> std::unordered_set<uint64_t> {
    std::unordered_set<uint64_t> same_split_branches;
    for (const auto &pre_lane : map->GetPrecedeLanes(lane)) {
      for (const auto &branch : map->GetNextLanes(pre_lane)) {
        if (branch) same_split_branches.insert(branch->id());
      }
    }
    return same_split_branches;
  };

  auto same_split_branches = get_same_split_branches(suc_lane);
  LaneConstPtr suc_left_lane = map->GetLeftLane(suc_lane);
  while (suc_left_lane) {
    if (std::find(black_list.begin(), black_list.end(),
                  suc_left_lane->type()) != black_list.end()) {
      break;
    }
    // If this lane and last valid index lane aren't in the same split branches,
    // add index count.
    if (same_split_branches.count(suc_left_lane->id()) == 0) {
      suc_left_index++;
      // update the same_split_branches.
      same_split_branches = get_same_split_branches(suc_left_lane);
    }
    // continue to check next neighbor lane.
    suc_left_lane = map->GetLeftLane(suc_left_lane);
  }

  // index from right in quit junction section
  same_split_branches = get_same_split_branches(suc_lane);
  LaneConstPtr suc_right_lane = map->GetRightLane(suc_lane);
  while (suc_right_lane) {
    if (std::find(black_list.begin(), black_list.end(),
                  suc_right_lane->type()) != black_list.end()) {
      break;
    }
    // If this lane and last valid index lane aren't in the same split branches,
    // add index.
    if (same_split_branches.count(suc_right_lane->id()) == 0) {
      suc_right_index++;
      // update the same_split_branches.
      same_split_branches = get_same_split_branches(suc_right_lane);
    }
    suc_right_lane = map->GetRightLane(suc_right_lane);
  }

  return true;
}

bool FsdCityScheduleCandidateLaneSequences(
    const Vec2d &ego_pos, const double ego_heading, const MapPtr &map,
    const LaneConstPtr &start_lane, double ego_v,
    const st::LaneChangeStateProto &lane_change_state,
    const ad_byd::planning::LaneSequencePtr &pre_target_lane_seq,
    const ad_byd::planning::LaneSequencePtr &pre_target_lane_seq_before_lc,
    std::vector<LaneSequencePtr> *candidate_lane_seqs,
    const st::DriverAction::LaneChangeCommand &new_lc_cmd,
    const BehaviorCommand &intention_dir) {
  SCOPED_TRACE(__FUNCTION__);
  if (!candidate_lane_seqs) return false;
  LaneConstPtr cur_lane = nullptr;
  LaneConstPtr tgt_lane = nullptr;
  LaneConstPtr left_lane = nullptr;
  LaneConstPtr right_lane = nullptr;
  LaneSequencePtr current_seq = nullptr;
  LaneSequencePtr extend_split_seq = nullptr;
  LaneSequencePtr left_seq = nullptr;
  LaneSequencePtr right_seq = nullptr;
  uint64_t split_lane_id = 0;
  uint64_t extend_split_lane_id = 0;
  SplitDirection split_direction = SplitDirection::NotSpliting;
  double split_lane_dis = 0.0;
  double extend_split_lane_dis = 0.0;
  BehaviorCommand ahead_direction = BehaviorCommand::Command_Invalid;

  bool if_change =
      (lane_change_state.stage() == st::LaneChangeStage::LCS_EXECUTING ||
       lane_change_state.stage() == st::LaneChangeStage::LCS_PAUSE)
          ? true
          : false;
  bool if_left = lane_change_state.lc_left();
  tgt_lane = (pre_target_lane_seq)
                 ? pre_target_lane_seq->GetNearestLane(ego_pos)
                 : nullptr;

  if (!tgt_lane) {
    if (start_lane) {
      tgt_lane = start_lane;
    } else {
      return false;
    }
  }

  // switch into CNOA in the go straight intersection, reset target to be the
  // pre junction lane when it's needed.
  if ((!pre_target_lane_seq) && (tgt_lane->junction_id() != 0) &&
      (tgt_lane->turn_type() == NO_TURN)) {
    // 1. get the pre_lane of the target.
    std::vector<LaneConstPtr> pre_lanes = map->GetPrecedeLanes(tgt_lane);
    LaneConstPtr pre_lane = pre_lanes.empty() ? nullptr : pre_lanes.front();

    // 2. get all branches of this plit in junction.
    std::vector<LaneConstPtr> branches =
        (pre_lane) ? map->GetNextLanes(pre_lane) : std::vector<LaneConstPtr>{};

    // 3. check whether need to reset target to be pre junction lane.
    bool need_to_reset_tgt_lane = true;
    if (branches.size() < 2) {
      // 3.1 if the branch is single, no need.
      need_to_reset_tgt_lane = false;
    } else {
      // 3.2 if ego is far from any branch, no need.
      for (const auto &branch : branches) {
        // only consider going straight branches.
        if (branch && branch->turn_type() == NO_TURN) {
          constexpr double lat_max = 0.8;
          double ds = 0, dl = 10.0;
          branch->center_line().GetProjection(ego_pos, &ds, &dl);
          if (fabs(dl) > lat_max) {
            need_to_reset_tgt_lane = false;
            break;
          }
        }
      }
    }

    // 4. reset the tgt_lane to be pre junction lane.
    if (need_to_reset_tgt_lane) {
      tgt_lane = pre_lane;
    }
  }

  // get left and right lane
  left_lane = map->GetLaneById(tgt_lane->left_lane_id());
  right_lane = map->GetLaneById(tgt_lane->right_lane_id());
  LaneConstPtr tgt_lane_pre =
      tgt_lane->pre_lane_ids().empty()
          ? nullptr
          : map->GetLaneById(tgt_lane->pre_lane_ids().front());
  LaneConstPtr tgt_lane_suc =
      tgt_lane->next_lane_ids().empty()
          ? nullptr
          : map->GetLaneById(tgt_lane->next_lane_ids().front());
  if (!(tgt_lane->junction_id() == 0) && tgt_lane_pre && tgt_lane_suc) {
    // merge
    while (left_lane && !left_lane->next_lane_ids().empty() &&
           left_lane->next_lane_ids().front() == tgt_lane_suc->id()) {
      left_lane = map->GetLaneById(left_lane->left_lane_id());
    }
    while (right_lane && !right_lane->next_lane_ids().empty() &&
           right_lane->next_lane_ids().front() == tgt_lane_suc->id()) {
      right_lane = map->GetLaneById(right_lane->right_lane_id());
    }

    // split
    LaneConstPtr left_lane_tmp = left_lane;
    while (left_lane_tmp && !left_lane_tmp->pre_lane_ids().empty() &&
           left_lane_tmp->pre_lane_ids().front() == tgt_lane_pre->id()) {
      left_lane_tmp = map->GetLaneById(left_lane_tmp->left_lane_id());
    }
    if (left_lane_tmp && !left_lane_tmp->next_lane_ids().empty() &&
        left_lane_tmp->next_lane_ids().front() ==
            tgt_lane_suc->left_lane_id()) {
      left_lane = left_lane_tmp;
    }

    LaneConstPtr right_lane_tmp = right_lane;
    while (right_lane_tmp && !right_lane_tmp->pre_lane_ids().empty() &&
           right_lane_tmp->pre_lane_ids().front() == tgt_lane_pre->id()) {
      right_lane_tmp = map->GetLaneById(right_lane_tmp->right_lane_id());
    }
    if (right_lane_tmp && !right_lane_tmp->next_lane_ids().empty() &&
        right_lane_tmp->next_lane_ids().front() ==
            tgt_lane_suc->right_lane_id()) {
      right_lane = right_lane_tmp;
    }
  }
  auto uturn_filter_result = UturnFilter(map, tgt_lane);
  if (if_change) {
    if (if_left) {
      cur_lane = right_lane;
    } else {
      cur_lane = left_lane;
    }

    if (if_left) {
      if (cur_lane && cur_lane->center_line().IsValid()) {
        current_seq =
            GetOptimalLaneSeq(ego_pos, ego_heading, map, cur_lane, ego_v,
                              /*navi_is_priority=*/true,
                              /*split_is_navi_priority*/ true, split_lane_id,
                              pre_target_lane_seq_before_lc);
      }
      if (tgt_lane && tgt_lane->center_line().IsValid() &&
          !IsNonMotorLane(map, tgt_lane)) {
        // left_seq = map->GetLeftLaneSequence(tgt_lane, current_seq);
        // if (!left_seq) {
        left_seq = GetOptimalLaneSeq(ego_pos, ego_heading, map, tgt_lane, ego_v,
                                     /*navi_is_priority=*/true,
                                     /*split_is_navi_priority*/ true,
                                     split_lane_id, pre_target_lane_seq);
        // }
      }
    } else {
      if (cur_lane && cur_lane->center_line().IsValid()) {
        current_seq =
            GetOptimalLaneSeq(ego_pos, ego_heading, map, cur_lane, ego_v,
                              /*navi_is_priority=*/true,
                              /*split_is_navi_priority*/ true, split_lane_id,
                              pre_target_lane_seq_before_lc);
      }
      if (tgt_lane && tgt_lane->center_line().IsValid() &&
          !IsNonMotorLane(map, tgt_lane)) {
        // right_seq = map->GetRightLaneSequence(tgt_lane, current_seq);
        // if (!right_seq) {
        right_seq = GetOptimalLaneSeq(ego_pos, ego_heading, map, tgt_lane,
                                      ego_v, /*navi_is_priority=*/true,
                                      /*split_is_navi_priority*/ true,
                                      split_lane_id, pre_target_lane_seq);
        // }
      }
    }
  } else {
    // choose seq when lk
    cur_lane = tgt_lane;
    if (tgt_lane && tgt_lane->center_line().IsValid()) {
      current_seq = GetOptimalLaneSeq(
          ego_pos, ego_heading, map, tgt_lane, ego_v, /*navi_is_priority=*/true,
          /*split_is_navi_priority*/ true, split_lane_id, pre_target_lane_seq);
    }

    if (left_lane && left_lane->center_line().IsValid() &&
        uturn_filter_result.find(UturnFilterDirection::left) ==
            uturn_filter_result.end() &&
        !IsNonMotorLane(map, left_lane)) {
      // auto left_seq_from_cur =
      //     map->GetLeftLaneSequence(left_lane, current_seq, true);

      left_seq = GetOptimalLaneSeq(ego_pos, ego_heading, map, left_lane, ego_v,
                                   /*navi_is_priority=*/true,
                                   /*split_is_navi_priority*/ true,
                                   split_lane_id, nullptr);
    }

    if (right_lane && right_lane->center_line().IsValid() &&
        uturn_filter_result.find(UturnFilterDirection::right) ==
            uturn_filter_result.end() &&
        !IsNonMotorLane(map, right_lane)) {
      // auto right_seq_from_cur =
      //     map->GetRightLaneSequence(right_lane, current_seq, true);

      right_seq = GetOptimalLaneSeq(ego_pos, ego_heading, map, right_lane,
                                    ego_v, /*navi_is_priority=*/true,
                                    /*split_is_navi_priority*/ true,
                                    split_lane_id, nullptr);
    }
  }

  if (FLAGS_planner_enable_split_tasks &&
      new_lc_cmd != st::DriverAction::LC_CMD_LEFT &&
      new_lc_cmd != st::DriverAction::LC_CMD_RIGHT) {
    if (if_change) {
      if (if_left) {
        extend_split_seq = CalcuExtendSplitLaneSequence(
            ego_pos, ego_heading, map, tgt_lane, ego_v, true, true,
            split_lane_id, extend_split_lane_id, left_seq, pre_target_lane_seq);
        split_direction = SplitDirection::Right;
      } else {
        extend_split_seq = CalcuExtendSplitLaneSequence(
            ego_pos, ego_heading, map, tgt_lane, ego_v, true, true,
            split_lane_id, extend_split_lane_id, right_seq,
            pre_target_lane_seq);
        split_direction = SplitDirection::Left;
      }
    } else {
      if (intention_dir == BehaviorCommand::Command_Invalid) {
        ahead_direction = CalcuAheadDirection(map, ego_pos, ego_v, current_seq,
                                              left_seq, right_seq);
      }
      if ((intention_dir == BehaviorCommand::Command_LaneChangeLeft ||
           ahead_direction == BehaviorCommand::Command_LaneChangeLeft) &&
          left_seq) {
        auto left_seq_from_cur =
            map->GetLeftLaneSequence(left_lane, current_seq, true);
        extend_split_seq = CalcuExtendSplitLaneSequence(
            ego_pos, ego_heading, map, left_lane, ego_v, true, true,
            split_lane_id, extend_split_lane_id, left_seq, left_seq_from_cur);
        split_direction = SplitDirection::Right;
      } else if ((intention_dir == BehaviorCommand::Command_LaneChangeRight ||
                  ahead_direction ==
                      BehaviorCommand::Command_LaneChangeRight) &&
                 right_seq) {
        auto right_seq_from_cur =
            map->GetRightLaneSequence(right_lane, current_seq, true);
        extend_split_seq = CalcuExtendSplitLaneSequence(
            ego_pos, ego_heading, map, right_lane, ego_v, true, true,
            split_lane_id, extend_split_lane_id, right_seq, right_seq_from_cur);
        split_direction = SplitDirection::Left;
      }
    }

    if (!extend_split_seq) {
      if (if_change) {
        extend_split_seq = CalcuExtendSplitLaneSequence(
            ego_pos, ego_heading, map, cur_lane, ego_v, true, true,
            split_lane_id, extend_split_lane_id, current_seq,
            pre_target_lane_seq_before_lc);
      } else {
        extend_split_seq = CalcuExtendSplitLaneSequence(
            ego_pos, ego_heading, map, cur_lane, ego_v, true, true,
            split_lane_id, extend_split_lane_id, current_seq,
            pre_target_lane_seq);
      }
      if (extend_split_seq) {
        if (split_direction == SplitDirection::NotSpliting) {
          if (left_seq && right_seq) {
            split_direction =
                CalcuSplitDirectionByNavigation(map, left_lane, right_lane);
            if (split_direction == SplitDirection::NotSpliting) {
              split_direction =
                  CalcuSplitDirection(map, split_lane_id, extend_split_lane_id);
            }
          } else {
            split_direction = SplitDirection::Mid;
          }
        }
        auto direction =
            CalcuSplitDirection(map, split_lane_id, extend_split_lane_id);
        if (direction == SplitDirection::Left) {
          current_seq->SetSplitTaskState(SplitTasksState::Right_Task);
          extend_split_seq->SetSplitTaskState(SplitTasksState::Left_Task);
        } else if (direction == SplitDirection::Right) {
          current_seq->SetSplitTaskState(SplitTasksState::Left_Task);
          extend_split_seq->SetSplitTaskState(SplitTasksState::Right_Task);
        }
      } else {
        split_direction = SplitDirection::NotSpliting;
      }
    } else {
      auto direction =
          CalcuSplitDirection(map, split_lane_id, extend_split_lane_id);
      if (direction == SplitDirection::Left) {
        extend_split_seq->SetSplitTaskState(SplitTasksState::Left_Task);
        if (right_seq && split_direction == SplitDirection::Left) {
          right_seq->SetSplitTaskState(SplitTasksState::Right_Task);
        } else if (left_seq && split_direction == SplitDirection::Right) {
          left_seq->SetSplitTaskState(SplitTasksState::Right_Task);
        }
      } else if (direction == SplitDirection::Right) {
        extend_split_seq->SetSplitTaskState(SplitTasksState::Right_Task);
        if (right_seq && split_direction == SplitDirection::Left) {
          right_seq->SetSplitTaskState(SplitTasksState::Left_Task);
        } else if (left_seq && split_direction == SplitDirection::Right) {
          left_seq->SetSplitTaskState(SplitTasksState::Left_Task);
        }
      }
    }
  }

  bool is_build_left_seq = false;
  if (current_seq && !current_seq->lanes().empty() && !left_seq) {
    uint64_t build_lane_id = 0;
    if (IsPrebuildLeftSeqOnRightTurn(ego_pos, current_seq, tgt_lane,
                                     build_lane_id) &&
        build_lane_id != 0) {
      LaneConstPtr left_start_lane = map->GetLaneById(build_lane_id);
      left_seq =
          GetOptimalLaneSeq(ego_pos, ego_heading, map, left_start_lane, ego_v,
                            /*navi_is_priority=*/true,
                            /*split_is_navi_priority*/ true, 0, nullptr);
      is_build_left_seq = true;
    }
  }

  // if (current_seq) {
  //   (*candidate_lane_seqs).emplace_back(current_seq->lanes());
  //   for (int i = 0; i < current_seq->lanes().size(); ++i) {
  //     LOG_ERROR << "current seq: " << current_seq->lanes().at(i);
  //   }
  // }
  // if (left_seq) {
  //   (*candidate_lane_seqs).emplace_back(left_seq->lanes());
  //   for (int i = 0; i < left_seq->lanes().size(); ++i) {
  //     LOG_ERROR << "left seq: " << left_seq->lanes().at(i);
  //   }
  // }
  // if (right_seq) {
  //   (*candidate_lane_seqs).emplace_back(right_seq->lanes());
  //   for (int i = 0; i < right_seq->lanes().size(); ++i) {
  //     LOG_ERROR << "right seq: " << right_seq->lanes().at(i);
  //   }
  // }

  if (current_seq && !current_seq->lanes().empty() &&
      current_seq->lanes().front()) {
    map->GetContinuePreLaneseqs(ego_pos, current_seq);
  }
  if (extend_split_seq && !extend_split_seq->lanes().empty() &&
      extend_split_seq->lanes().front()) {
    map->GetContinuePreLaneseqs(ego_pos, extend_split_seq);
  }
  if (left_seq && !left_seq->lanes().empty() && left_seq->lanes().front()) {
    map->GetContinuePreLaneseqs(ego_pos, left_seq, is_build_left_seq);
  }
  if (right_seq && !right_seq->lanes().empty() && right_seq->lanes().front()) {
    map->GetContinuePreLaneseqs(ego_pos, right_seq);
  }

  if (current_seq) {
    current_seq->SetSequenceDirection(SequenceDirection::Cur);
    candidate_lane_seqs->emplace_back(current_seq);
  }
  // if (extend_split_seq && (!split_direction.empty() || if_change))
  if (extend_split_seq && split_direction != SplitDirection::NotSpliting) {
    extend_split_seq->SetSequenceDirection(SequenceDirection::Extend);
    candidate_lane_seqs->emplace_back(extend_split_seq);
  }
  if (left_seq && split_direction != SplitDirection::Left) {
    left_seq->SetSequenceDirection(SequenceDirection::Left);
    candidate_lane_seqs->emplace_back(left_seq);
  }
  if (right_seq && split_direction != SplitDirection::Right) {
    right_seq->SetSequenceDirection(SequenceDirection::Right);
    candidate_lane_seqs->emplace_back(right_seq);
  }

  // keep these codes, maight use them in the future.
  // // process straight junction lc num
  // // 1. get the best straight junction seq
  // std::unordered_set<int> has_straight_junction_set;
  // has_straight_junction_set.reserve(candidate_lane_seqs->size());
  // int min_succeed_straight_junction_lc_num = std::numeric_limits<int>::max();
  // LaneConstPtr priority_base_lane = nullptr;
  // for (size_t i = 0; i < candidate_lane_seqs->size(); i++) {
  //   auto &seq = candidate_lane_seqs->at(i);
  //   if (seq && seq->HasStraightJunction()) {
  //     int lc_num = seq->GetSucceedStraightJunctionLcNum();
  //     if (lc_num < min_succeed_straight_junction_lc_num) {
  //       min_succeed_straight_junction_lc_num = lc_num;
  //       priority_base_lane = seq->GetBaseLane();
  //     }
  //     has_straight_junction_set.insert(i);
  //   }
  // }
  // // 2. get the base section (ego section)
  // SectionConstPtr base_section = nullptr;
  // if (priority_base_lane) {
  //   base_section = map->GetSectionById(priority_base_lane->section_id());
  // }
  // // 3. calculate other none-straight-junction seq's lc num
  // if (base_section && !has_straight_junction_set.empty() &&
  //     has_straight_junction_set.size() < candidate_lane_seqs->size()) {
  //   auto calcLcNum = [base_section,
  //                     priority_base_lane](const LaneConstPtr lane) -> int {
  //     int d = 0;
  //     if (!lane) return d;
  //     auto it0 =
  //         std::find(base_section->lanes().begin(),
  //         base_section->lanes().end(),
  //                   priority_base_lane->id());
  //     auto it1 = std::find(base_section->lanes().begin(),
  //                          base_section->lanes().end(), lane->id());
  //     if (it0 != base_section->lanes().end() &&
  //         it1 != base_section->lanes().end()) {
  //       d = static_cast<int>(std::distance(it0, it1));
  //     }
  //     return d;
  //   };
  //   for (size_t i = 0; i < candidate_lane_seqs->size(); i++) {
  //     if (has_straight_junction_set.count(i) != 0) continue;
  //     auto &seq = candidate_lane_seqs->at(i);
  //     if (seq) {
  //       const auto &base_lane = seq->GetBaseLane();
  //       int lc_num =
  //           calcLcNum(base_lane) + min_succeed_straight_junction_lc_num;
  //       seq->SetHasStraightJunction(true);
  //       seq->SetSucceedStraightJunctionLcNum(lc_num);
  //     }
  //   }
  // }

  std::vector<std::string> lane_seqs_ids{};
  std::ostringstream stream;
  if (current_seq) {
    stream << "current: ";
    for (auto l : current_seq->lanes()) {
      if (!l->center_line().points().empty()) {
        stream << l->id() << ", ";
      }
    }
    lane_seqs_ids.push_back(stream.str());
    stream.str("");
  }
  if (left_seq) {
    stream << "left: ";
    for (auto l : left_seq->lanes()) {
      if (!l->center_line().points().empty()) {
        stream << l->id() << ", ";
      }
    }
    lane_seqs_ids.push_back(stream.str());
    stream.str("");
  }
  if (right_seq) {
    stream << "right: ";
    for (auto l : right_seq->lanes()) {
      if (!l->center_line().points().empty()) {
        stream << l->id() << ", ";
      }
    }
    lane_seqs_ids.push_back(stream.str());
  }
  if (extend_split_seq) {
    for (auto ls : extend_split_seq->lanes()) {
      if (!ls->center_line().points().empty()) {
        lane_seqs_ids.push_back(absl::StrCat(ls->id()));
      }
    }
  }
  Log2DDS::LogDataV0("lane_seqs", lane_seqs_ids);

  return true;
}

bool FsdHighwayScheduleCandidateLaneSequences(
    const Vec2d &ego_pos, const double ego_heading, const MapPtr &map,
    const LaneConstPtr &start_lane, double ego_v,
    const st::LaneChangeStateProto &lane_change_state,
    const st::Behavior_FunctionId &function_id,
    const ad_byd::planning::LaneSequencePtr &pre_target_lane_seq,
    const ad_byd::planning::LaneSequencePtr &pre_target_lane_seq_before_lc,
    bool is_borrow, bool is_nudge,
    std::vector<LaneSequencePtr> *candidate_lane_seqs,
    const st::DriverAction::LaneChangeCommand &new_lc_cmd,
    const BehaviorCommand &intention_dir,
    const ad_byd::planning::EIEChoiceType &eie_choice_type) {
  SCOPED_TRACE(__FUNCTION__);
  if (!candidate_lane_seqs) return false;
  if (!map) return false;
  LaneConstPtr tgt_lane = nullptr;
  LaneConstPtr before_lc_lane = nullptr;
  LaneConstPtr cur_lane = nullptr;
  LaneConstPtr left_lane = nullptr;
  LaneConstPtr right_lane = nullptr;
  LaneConstPtr extend_start_lane = nullptr;
  LaneSequencePtr cur_seq = nullptr;
  LaneSequencePtr left_seq = nullptr;
  LaneSequencePtr right_seq = nullptr;
  LaneSequencePtr extend_split_seq = nullptr;
  LaneSequencePtr split_seq = nullptr;
  std::vector<std::string> debug_fsd_highway_scheduler_string;

  constexpr const double LThresholdCrossLine = 2.0;
  constexpr const double LThresholdDeviateLanceCenter = 2.0;
  constexpr const double HeadingThreshold = 5.0 / 180.0 * M_PI;
  constexpr double kPreSplitTime = 10.0;
  constexpr double kMinSplitCheckDis = 200;

  uint64_t split_lane_id = 0;
  static uint64_t last_split_id = 0;
  double dis_to_split;
  double distance_limit = std::numeric_limits<double>::infinity();
  static double last_distance = std::numeric_limits<double>::infinity();
  uint64_t extend_split_lane_id = 0;
  SplitDirection split_direction = SplitDirection::NotSpliting;
  SplitTasksState split_task_state = SplitTasksState::None;

  bool gen_left_flag = false;
  bool gen_right_flag = false;

  bool if_change =
      (lane_change_state.stage() == st::LaneChangeStage::LCS_EXECUTING ||
       lane_change_state.stage() == st::LaneChangeStage::LCS_PAUSE)
          ? true
          : false;
  bool if_left = lane_change_state.lc_left();

  // debug info: pre_target_lane_seq
  if (pre_target_lane_seq && !pre_target_lane_seq->lanes().empty()) {
    std::vector<LaneConstPtr> lanes = pre_target_lane_seq->lanes();
    std::ostringstream tmp_stream;
    tmp_stream << "pre_target_lane_seq_lane_ids : ";
    for (const auto &lane : lanes) {
      if (lane) tmp_stream << lane->id() << " ";
    }
    debug_fsd_highway_scheduler_string.emplace_back(tmp_stream.str());
    // LOG_ERROR << tmp_stream.str();
  }

  // debug info: pre_target_lane_seq_before_lc
  if (pre_target_lane_seq_before_lc &&
      !pre_target_lane_seq_before_lc->lanes().empty()) {
    std::vector<LaneConstPtr> lanes = pre_target_lane_seq_before_lc->lanes();
    std::ostringstream tmp_stream;
    tmp_stream << "pre_target_lane_seq_before_lc_lane_ids : ";
    for (const auto &lane : lanes) {
      if (lane) tmp_stream << lane->id() << " ";
    }
    debug_fsd_highway_scheduler_string.emplace_back(tmp_stream.str());
    // LOG_ERROR << tmp_stream.str();
  }

  // debug info: lane_change
  std::ostringstream tmp_stream;
  tmp_stream << absl::StrCat(
      "lane_change_state.stage = ", lane_change_state.stage(),
      " , if_change = ", if_change, " , if_left = ", if_left,
      " , intention_dir = ", intention_dir);
  // LOG_ERROR << tmp_stream.str();
  debug_fsd_highway_scheduler_string.emplace_back(tmp_stream.str());
  tmp_stream.str("");

  // step 1. get target lane
  if (pre_target_lane_seq) {
    tgt_lane = pre_target_lane_seq->GetNearestLane(ego_pos);
  }

  // when tgt_lane be lost, use the cloest lane as target lane.
  if (!tgt_lane) {
    if (start_lane) {
      tgt_lane = start_lane;
    } else {
      LOG_ERROR << "None tgt_lane and start_lane!!!";
      return false;
    }
  }

  // step 2. get orin lane
  if (pre_target_lane_seq_before_lc) {
    before_lc_lane = pre_target_lane_seq_before_lc->GetNearestLane(ego_pos);
  }

  // step 3. check whether human override steer
  // ignore the override checking:
  // 1. ego car is on the split or merge lane;
  // 2. borrow lane;
  // 3. nudge;
  bool is_split = false;
  bool is_merge = false;
  bool is_start_target_same = false;
  bool is_in_virtual_lane = false;
  if (start_lane != nullptr) {
    LaneConstPtr pre_lane = map->GetContinuePreLane(start_lane->id());
    if (pre_lane != nullptr) {
      if (pre_lane->next_lane_ids().size() > 1) {
        is_split = true;
      }
    }

    LaneConstPtr next_lane = map->GetContinueNextLane(start_lane->id());
    if (next_lane != nullptr) {
      if (next_lane->pre_lane_ids().size() > 1) {
        is_merge = true;
      }
    }

    // Notice: tgt_lane isn't nullptr.
    // If human lat override, reset tgt_lane to be start_lane. So if tgt_lane is
    // start_lane, it's not need to check human override.
    if (tgt_lane->id() == start_lane->id()) {
      is_start_target_same = true;
    } else {
      // check ego in virtual lane or not. if in virtual lane, lat diff maybe
      // large.
      if (map->type() != ad_byd::planning::MapType::BEV_MAP) {
        if (tgt_lane->IsBoundaryVirtual()) {
          is_in_virtual_lane = true;
        }
      }
    }
  }

  bool need_check_override = true;
  if (is_start_target_same || is_split || is_merge || is_borrow || is_nudge ||
      is_in_virtual_lane) {
    need_check_override = false;
  }

  // override steer conditions:
  // 1. when lane keep, ego car divides tgt_lane too much
  // 2. when lane change, ego car against the lane change direction;
  bool human_override_steer = false;
  if (need_check_override) {
    if ((lane_change_state.stage() == st::LaneChangeStage::LCS_NONE) &&
        (tgt_lane)) {
      double ego_s, ego_l;
      if (tgt_lane->center_line().GetProjection(ego_pos, &ego_s, &ego_l)) {
        // If cann't GetHeadingFromS, use the default value ego_heading, then
        // diff_angle will be zero.
        double ref_heading = ego_heading;
        tgt_lane->center_line().GetHeadingFromS(ego_s, &ref_heading);
        double diff_angle = st::NormalizeAngle(ego_heading - ref_heading);
        if (((ego_l > LThresholdCrossLine) &&
             (diff_angle > HeadingThreshold)) ||
            ((ego_l < -LThresholdCrossLine) &&
             (diff_angle < -HeadingThreshold))) {
          human_override_steer = true;
          LOG_INFO << "human_override_steer: when lane keep, ego car divides "
                      "tgt_lane too much";
        }
      }
    } else if (if_change && before_lc_lane) {
      double ego_s, ego_l;
      if (before_lc_lane->center_line().GetProjection(ego_pos, &ego_s,
                                                      &ego_l)) {
        double ego_tgt_s = ego_s, ego_tgt_l = ego_l;
        tgt_lane->center_line().GetProjection(ego_pos, &ego_tgt_s, &ego_tgt_l);
        // when need change to left, but ego car move to right.
        // ego car divides far from  before_lc_lane and tgt_lane.
        if (if_left && (ego_l < -LThresholdDeviateLanceCenter) &&
            (ego_tgt_l <
             -(LThresholdDeviateLanceCenter + Constants::DEFAULT_LANE_WIDTH))) {
          human_override_steer = true;
          LOG_INFO << "human_override_steer: when lane keep, when need change "
                      "to left, but ego car move to right";
        }
        // when need change to right, but ego car move to left
        else if ((!if_left) && (ego_l > LThresholdDeviateLanceCenter) &&
                 (ego_tgt_l > (LThresholdDeviateLanceCenter +
                               Constants::DEFAULT_LANE_WIDTH))) {
          human_override_steer = true;
          LOG_INFO << "human_override_steer: when lane keep, when need change "
                      "to right, but ego car move to left";
        }
      }
    } else if (!if_change && start_lane &&
               (start_lane->type() != LANE_EMERGENCY) && tgt_lane &&
               (tgt_lane->type() == LANE_EMERGENCY)) {
      double ego_s, ego_l;
      if (tgt_lane->center_line().GetProjection(ego_pos, &ego_s, &ego_l)) {
        if (ego_l > LThresholdCrossLine) {
          human_override_steer = true;
          LOG_INFO << "human_override_steer: target lane is EMERGENCY";
        }
      }
    }
  }

  tmp_stream << absl::StrCat(
      "need_check_override = ", need_check_override,
      " , human_override_steer = ", human_override_steer);
  debug_fsd_highway_scheduler_string.emplace_back(tmp_stream.str());
  tmp_stream.str("");

  // LOG_INFO << "human_override_steer:" << human_override_steer;

  // step 4. compute cur_lane, left_lane, right_lane

  int lane_change_sign = 0;  // 0, keep, 1, left, 2, right;

  // when human driver override the steer, reset cur_lane to start_lane and none
  // left,right;
  if (human_override_steer) {
    cur_lane = start_lane;
    left_lane = nullptr;
    right_lane = nullptr;
    cur_seq = pre_target_lane_seq;
  } else {
    // get current, left and right lane based on target lane.
    if (if_change) {
      if (if_left) {
        cur_lane = map->GetLaneById(tgt_lane->right_lane_id());
        left_lane = tgt_lane;
        lane_change_sign = 1;
      } else {
        cur_lane = map->GetLaneById(tgt_lane->left_lane_id());
        right_lane = tgt_lane;
        lane_change_sign = 2;
      }
      // on the lane change state, cur lane seq is before lane_lc,
      // other time,  cur lane seq is target.
      cur_seq = pre_target_lane_seq_before_lc;
    } else {
      cur_lane = tgt_lane;
      left_lane = map->GetLaneById(tgt_lane->left_lane_id());
      right_lane = map->GetLaneById(tgt_lane->right_lane_id());
      cur_seq = pre_target_lane_seq;
    }
  }
  // LOG_ERROR << "lane_change_sign:" << lane_change_sign;

  // step 5. compute lane seq
  if (cur_lane) {
    // if current lane be found, find left and right based on current seq.
    GenerateCurrentLaneSequences(ego_pos, ego_v, cur_lane, function_id, map,
                                 lane_change_sign, cur_seq);

    if (left_lane) {
      left_seq = map->GetLeftLaneSequence(left_lane, cur_seq);
    }

    if (right_lane) {
      right_seq = map->GetRightLaneSequence(right_lane, cur_seq, false,
                                            eie_choice_type);
    }
  } else {
    // if current lane can't be found, clear cur_seq and find left and right by
    // self.
    cur_seq.reset();

    if (left_lane && left_lane->type() != LaneType::LANE_DIVERSION) {
      GenerateCurrentLaneSequences(ego_pos, ego_v, left_lane, function_id, map,
                                   0, left_seq);
    }

    if (right_lane && right_lane->type() != LaneType::LANE_DIVERSION) {
      GenerateCurrentLaneSequences(ego_pos, ego_v, right_lane, function_id, map,
                                   0, right_seq);
    }
  }

  if (lane_change_state.stage() == st::LaneChangeStage::LCS_NONE) {
    if (left_seq && intention_dir == BehaviorCommand::Command_LaneChangeLeft) {
      split_lane_id = CalcuSplitLaneId(ego_pos, left_seq, dis_to_split);
      extend_start_lane = left_lane;
      gen_left_flag = true;
      split_seq = left_seq;
    } else if (right_seq &&
               intention_dir == BehaviorCommand::Command_LaneChangeRight) {
      split_lane_id = CalcuSplitLaneId(ego_pos, right_seq, dis_to_split);
      extend_start_lane = right_lane;
      gen_right_flag = true;
      split_seq = right_seq;
    } else if (cur_seq) {
      split_lane_id = CalcuSplitLaneId(ego_pos, cur_seq, dis_to_split);
      extend_start_lane = cur_lane;
      split_seq = cur_seq;
    }
  } else {
    if (left_seq && left_lane && tgt_lane &&
        left_lane->id() == tgt_lane->id()) {
      split_lane_id = CalcuSplitLaneId(ego_pos, left_seq, dis_to_split);
      gen_left_flag = true;
      split_seq = left_seq;
    } else if (right_seq && right_lane && tgt_lane &&
               right_lane->id() == tgt_lane->id()) {
      split_lane_id = CalcuSplitLaneId(ego_pos, right_seq, dis_to_split);
      gen_right_flag = true;
      split_seq = right_seq;
    } else if (cur_seq && cur_lane && tgt_lane &&
               cur_lane->id() == tgt_lane->id()) {
      split_lane_id = CalcuSplitLaneId(ego_pos, cur_seq, dis_to_split);
      split_seq = cur_seq;
    }
    extend_start_lane = tgt_lane;
  }

  if (0 != split_lane_id && FLAGS_planner_enable_split_tasks &&
      new_lc_cmd != st::DriverAction::LC_CMD_LEFT &&
      new_lc_cmd != st::DriverAction::LC_CMD_RIGHT &&
      function_id == st::Behavior_FunctionId_HW_NOA) {
    if (last_split_id != split_lane_id) {
      distance_limit = std::max(ego_v * kPreSplitTime, kMinSplitCheckDis);
    } else {
      distance_limit = last_distance;
    }
    if (dis_to_split < distance_limit) {
      GenerateCurrentLaneSequences(ego_pos, ego_v, extend_start_lane,
                                   function_id, map, lane_change_sign,
                                   extend_split_seq, split_lane_id);
      extend_split_lane_id =
          CalcuSplitLaneId(ego_pos, extend_split_seq, dis_to_split);
      split_direction =
          CalcuSplitDirection(map, split_lane_id, extend_split_lane_id);
      split_seq->SetSplitLaneId(split_lane_id);
      split_seq->SetSplitLcNum(std::abs(
          map->GetPriorityLaneRelation(map->GetLaneById(split_lane_id))));
      extend_split_seq->SetSplitLaneId(extend_split_lane_id);
      extend_split_seq->SetSplitLcNum(std::abs(map->GetPriorityLaneRelation(
          map->GetLaneById(extend_split_lane_id))));
      if (split_seq->GetSplitLcNum() > extend_split_seq->GetSplitLcNum()) {
        split_seq->SetSplitLcNumIsMax(true);
      } else {
        extend_split_seq->SetSplitLcNumIsMax(true);
      }
      last_split_id = split_lane_id;
      last_distance = distance_limit;
      split_lane_id = 0;
    }
  }

  // get pre lane seq behind ego car
  if (cur_seq && !cur_seq->lanes().empty() && cur_seq->lanes().front()) {
    map->GetContinuePreLaneseqs(ego_pos, cur_seq);
  }
  if (left_seq && !left_seq->lanes().empty() && left_seq->lanes().front()) {
    map->GetContinuePreLaneseqs(ego_pos, left_seq);
  }
  if (right_seq && !right_seq->lanes().empty() && right_seq->lanes().front()) {
    map->GetContinuePreLaneseqs(ego_pos, right_seq);
  }
  if (extend_split_seq && !extend_split_seq->lanes().empty() &&
      extend_split_seq->lanes().front()) {
    map->GetContinuePreLaneseqs(ego_pos, extend_split_seq);
  }
  // step 6. make result data
  auto split_direction_tmp = split_direction;
  if (!left_seq || !right_seq) split_direction = SplitDirection::Mid;
  if (cur_seq && !cur_seq->lanes().empty()) {
    cur_seq->SetSequenceDirection(SequenceDirection::Cur);
    CalcuSplitState(extend_start_lane, cur_lane->id(), split_direction_tmp,
                    split_task_state);
    cur_seq->SetSplitTaskState(split_task_state);
    candidate_lane_seqs->emplace_back(cur_seq);
  }
  if (left_seq && !left_seq->lanes().empty() &&
      (extend_split_lane_id == 0 || gen_left_flag ||
       (!gen_right_flag && split_direction != SplitDirection::Left))) {
    left_seq->SetSequenceDirection(SequenceDirection::Left);
    CalcuSplitState(extend_start_lane, left_lane->id(), split_direction_tmp,
                    split_task_state);
    left_seq->SetSplitTaskState(split_task_state);
    candidate_lane_seqs->emplace_back(left_seq);
    Log2DDS::LogDataV0("lane_seqs", "has_left");
  }
  if (right_seq && !right_seq->lanes().empty() &&
      (extend_split_lane_id == 0 || gen_right_flag ||
       (!gen_left_flag && split_direction != SplitDirection::Right))) {
    right_seq->SetSequenceDirection(SequenceDirection::Right);
    CalcuSplitState(extend_start_lane, right_lane->id(), split_direction_tmp,
                    split_task_state);
    right_seq->SetSplitTaskState(split_task_state);
    candidate_lane_seqs->emplace_back(right_seq);
    Log2DDS::LogDataV0("lane_seqs", "has_right");
  }
  if (extend_split_seq && !extend_split_seq->lanes().empty() &&
      (gen_left_flag || gen_right_flag ||
       split_direction != SplitDirection::NotSpliting)) {
    extend_split_seq->SetSequenceDirection(SequenceDirection::Extend);
    if (split_direction_tmp == SplitDirection::Left) {
      split_direction_tmp = SplitDirection::Right;
    } else if (split_direction_tmp == SplitDirection::Right) {
      split_direction_tmp = SplitDirection::Left;
    }
    CalcuSplitState(extend_start_lane, extend_start_lane->id(),
                    split_direction_tmp, split_task_state);
    extend_split_seq->SetSplitTaskState(split_task_state);
    candidate_lane_seqs->emplace_back(extend_split_seq);
    Log2DDS::LogDataV0("lane_seqs", "has_split");
  }
  Log2DDS::LogDataV2("schedule_fsd_debug", debug_fsd_highway_scheduler_string);

  return true;
}

LaneSequencePtr GetOptimalLaneSeq(
    const Vec2d &start_point, const double start_theta, const MapPtr &map,
    const LaneConstPtr &lane, double ego_v, const bool &navi_is_priority,
    const bool &split_is_navi_priority, const uint64_t lane_id,
    const ad_byd::planning::LaneSequencePtr pre_lane_seq) {
  if (!map || !map->IsValid() || !lane) {
    return nullptr;
  }

  std::vector<std::pair<LaneSequencePtr, double>> cost_static;
  GetStaticCost(start_point, map, lane, navi_is_priority,
                split_is_navi_priority, lane_id, cost_static);
  if (1 == cost_static.size()) return cost_static.begin()->first;
  std::sort(cost_static.begin(), cost_static.end(),
            [](const std::pair<LaneSequencePtr, double> &lanes_a,
               const std::pair<LaneSequencePtr, double> &lanes_b) {
              return lanes_a.second < lanes_b.second;
            });
  // std::pair<double, int> -> std::pair<cost, sign>
  // sign: -1 occupied, 0 normal, 1 perfect, 2 perfect but occupied
  std::vector<LaneConstPtr> pre_lane_seq_in_range;
  std::vector<std::string> lane_filter_debug_strs;
  constexpr double kPreLaneSeqTime = 2.0;
  if (pre_lane_seq && !pre_lane_seq->lanes().empty()) {
    double dis_pre_lane_seq_kept = std::fmax(ego_v * kPreLaneSeqTime, 10.0);
    bool find_start_lane = false;
    std::stringstream ss;
    ss << "pre_lane_seq_in_range: ";
    for (auto it_lane : pre_lane_seq->lanes()) {
      if (!it_lane) continue;
      if (!find_start_lane && it_lane->id() == lane->id()) {
        find_start_lane = true;
      }
      if (!find_start_lane) continue;
      if (it_lane->id() == lane->id()) {
        double s, l;
        if (it_lane->center_line().GetProjection(start_point, &s, &l)) {
          dis_pre_lane_seq_kept -= (it_lane->curve_length() - s);
        }
      } else {
        dis_pre_lane_seq_kept -= it_lane->topo_length();
      }
      pre_lane_seq_in_range.emplace_back(it_lane);
      ss << it_lane->id() << ", ";

      if (dis_pre_lane_seq_kept < 0.0) break;
    }
    lane_filter_debug_strs.emplace_back(ss.str());
  }

  std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> cost_all;
  LaneseqFilter(map, cost_static, pre_lane_seq_in_range, cost_all,
                lane_filter_debug_strs);
  {
    std::stringstream ss;
    ss << "after filter seq size=" << cost_all.size();
    lane_filter_debug_strs.push_back(ss.str());
    Log2DDS::LogDataV2("target-filter", lane_filter_debug_strs);
  }
  GetHumanlikeCost(start_point, start_theta, map, cost_all);

  LaneSequencePtr best_lane_seq = nullptr;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto &seq : cost_all) {
    if (seq.second.first < best_cost) {
      best_cost = seq.second.first;
      best_lane_seq = seq.first;
    }
  }
  if (best_lane_seq) best_lane_seq->SetBaseLane(lane);
  return best_lane_seq;
}

bool LaneseqFilter(
    const MapPtr &map,
    const std::vector<std::pair<LaneSequencePtr, double>> &cost_static,
    std::vector<LaneConstPtr> &pre_lane_seq_in_range,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all,
    std::vector<std::string> &logstrs) {
  if (cost_static.empty()) {
    return false;
  }
  std::vector<std::pair<LaneSequencePtr, double>> cost_static_filtered;
  cost_static_filtered.reserve(cost_static.size());
  if (!pre_lane_seq_in_range.empty()) {
    for (const auto &seq : cost_static) {
      bool choose_seq = true;
      const auto lanes = seq.first->lanes();
      for (int i = 0; i < lanes.size(); i++) {
        if (!lanes[i]) continue;
        if (i < pre_lane_seq_in_range.size() && pre_lane_seq_in_range[i] &&
            pre_lane_seq_in_range[i]->id() != lanes[i]->id()) {
          choose_seq = false;
        }
      }
      if (choose_seq) cost_static_filtered.emplace_back(seq);
    }
  }

  if (cost_static_filtered.empty()) cost_static_filtered = cost_static;

  std::stringstream ss;
  ss << "aft pre_seq filt. size=" << cost_static_filtered.size();
  logstrs.push_back(ss.str());
  ss.str("");

  if (cost_static_filtered.size() == 1) {
    cost_all.emplace_back(cost_static_filtered[0].first,
                          std::make_pair(0.0, 0));
    return true;
  }
  const double shortest_length =
      cost_static_filtered.back().first->GetTrueLength();
  constexpr double need_length_after_junction = 500.0;
  constexpr double consider_farthest_junction_distance = 500.0;
  constexpr double consider_nearest_junction_distance = 200.0;

  int remain_opt = 3;
  double first_seq_cost = cost_static_filtered.front().second;
  std::unordered_set<uint64_t> split_junction_lane_id_set{};
  uint64_t before_junction_lane_id = 0;
  double first_navi_dist = -Constants::ZERO;
  bool is_uturn_junction = false;

  for (int idx = 0; idx < cost_static_filtered.size(); idx++) {
    const auto &seq = cost_static_filtered.at(idx);
    const auto &lanes = seq.first->lanes();

    if (lanes.size() < 2) {
      continue;
    }

    // only search junctions on the first lane seq.
    if (remain_opt == 3) {
      double junc_distance = 0.0;
      for (int i = 0; i < lanes.size() - 1; i++) {
        if (!lanes[i] || !lanes[i + 1]) break;
        if (i > 0) junc_distance += lanes[i]->topo_length();
        if (lanes[i]->next_lane_ids().size() >= 2 &&
            ((!(lanes[i + 1]->junction_id() == 0) &&
              lanes[i + 1]->type() != LaneType::LANE_NORMAL) ||
             lanes[i + 1]->turn_type() == TurnType::U_TURN)) {
          split_junction_lane_id_set.insert(lanes[i + 1]->id());
          before_junction_lane_id = lanes[i]->id();
          is_uturn_junction = (lanes[i + 1]->turn_type() == TurnType::U_TURN);
          ss << "junc " << before_junction_lane_id << ", dis " << junc_distance
             << ", ";
        }
        // if there is none junction in range of 500 meters, quit.
        if (before_junction_lane_id == 0) {
          if (junc_distance > consider_farthest_junction_distance) break;
        } else {
          // if there is a junction between 200 and 500 meters and the shortest
          // lane seq after the junction is long enough, don't consider this
          // junction.
          if ((junc_distance > consider_nearest_junction_distance) &&
              ((shortest_length - junc_distance) >
               need_length_after_junction)) {
            before_junction_lane_id = 0;
          }
          break;
        }
      }
    } else {
      // It's not the first lane seq, there must be a junction, because none
      // junciton scene has break. Then use junction filter conditions.

      bool has_same_split_lane = false, has_same_junction_entrance = false;
      LaneConstPtr succeed_junc_lane = nullptr;

      for (size_t i = 0; i < lanes.size() - 2; i++) {
        const auto &maybe_pre_junc = lanes.at(i);
        const auto &maybe_junc = lanes.at(i + 1);
        const auto &maybe_suc_junc = lanes.at(i + 2);
        if (!maybe_pre_junc || !maybe_junc || !maybe_suc_junc) break;
        if (maybe_pre_junc->id() == before_junction_lane_id) {
          has_same_junction_entrance = true;
          if (maybe_junc->junction_id() != 0 ||
              maybe_junc->turn_type() == TurnType::U_TURN) {
            if (split_junction_lane_id_set.count(maybe_junc->id()) != 0) {
              has_same_split_lane = true;
            } else {
              split_junction_lane_id_set.insert(maybe_junc->id());
            }
            succeed_junc_lane = maybe_suc_junc;
          }
          break;
        }
      }

      // drop conditions:
      // 1. drop the same succeed junction lane;
      // 2. only remain the seqs lead to the same pre junction lane.
      //    If don't have the same pre junction lane, drop it.
      if (has_same_split_lane || (!has_same_junction_entrance)) {
        continue;
      }

      if ((!succeed_junc_lane) || (!succeed_junc_lane->is_navigation())) {
        // If none succeed_junc_lane, that means this lane seq is too short,
        // drop it. If succeed_junc_lane isn't navi, drop it.
        continue;
      } else if (!is_uturn_junction) {
        // To calculate the lc_num according to succeed_junc_lane, it's more
        // accurate than according to junction_lane, because there are many
        // splits in the junction.
        int lc_num = abs(map->GetPriorityLaneRelation(succeed_junc_lane));
        // The navi distance need need to keep is related to lc_num:
        // lc_num = 1, need_navi_distance = 100;
        // lc_num = 2, need_navi_distance = 150;
        // lc_num = 3, need_navi_distance = 200;
        // etc.
        const double need_navi_distance = 50.0 + lc_num * 50.0;
        if ((lc_num != 0) &&
            ((succeed_junc_lane->navi_distance() +
              succeed_junc_lane->topo_length()) < need_navi_distance)) {
          continue;
        }
      }
    }

    cost_all.emplace_back(seq.first, std::make_pair(0.0, 0));
    ss << "add " << idx << ", ";

    remain_opt--;
    // 1. If there is a junction, keep up to 3 seqs;
    // 2. If there is none junction, only keep the first lane seq to save
    // computing cost.
    if (remain_opt <= 0 || before_junction_lane_id == 0) {
      break;
    }
  }

  logstrs.push_back(ss.str());
  ss.str("");

  return true;
}

void UpdateSignInLaneseq(
    const Vec2d &start_point,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all) {
  std::vector<std::string> debug_strings{};
  // TODO: reopen
  //   for (auto &seq : cost_all) {
  //     std::ostringstream stream;
  //     stream << "Lanes:";
  //     for (const auto &lane : seq.first->lanes()) {
  //       size_t pos = lane->id().size() > 3 ? (lane->id().size() - 3) : 0;
  //       stream << lane->id().substr(pos) << " , ";
  //     }

  //     bool perfect_check = false;
  //     bool occupied_check = false;
  //     LanePtr junction_lane = nullptr;
  //     LanePtr quit_junction_lane = nullptr;
  //     bool check_quit = false;
  //     std::vector<LanePtr> quit_lanes;
  //     for (const auto &lane : seq.first->lanes()) {
  //       if (lane && junction_lane && !quit_junction_lane) {
  //         quit_junction_lane = lane;
  //         check_quit = true;
  //       }
  //       if (check_quit) {
  //         quit_lanes.emplace_back(lane);
  //         continue;
  //       }
  //       if (lane && !lane->junction_id().empty() &&
  //           lane->type() != LANE_LEFT_WAIT &&
  //           !map->CheckIfValidCommonLane(lane)) {
  //         junction_lane = lane;
  //       }
  //     }
  //     if (junction_lane && junction_lane->is_priority() &&
  //         junction_lane->is_navigation()) {
  //       perfect_check = true;
  //     }
  //     if (quit_junction_lane) {
  //       stream << "..  quit_junction_lane:" << quit_junction_lane->id();
  //     }
  //     if (quit_junction_lane && quit_junction_lane->is_occupied()) {
  //       stream << "..  is occupied lane:" << quit_junction_lane->id();
  //       occupied_check = true;
  //     }
  //     if (perfect_check && occupied_check) {
  //       seq.second.second = 2;
  //     } else if (perfect_check) {
  //       seq.second.second = 1;
  //     } else if (occupied_check) {
  //       seq.second.second = -1;
  //     }

  //     stream << ", Sign:" << seq.second.second;
  //     debug_strings.push_back(stream.str());
  //   }
  //   Log2DDS::LogDataV2("target-dynamic-sign", debug_strings);
  return;
}

bool GetHumanlikeCost(
    const Vec2d &start_point, const double start_theta, const MapPtr &map,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all) {
  if (cost_all.empty()) {
    return false;
  } else if (cost_all.size() == 1) {
    // If there is only one seq after the filter, it's not need to add
    // human-like cost any more. Return directly to save computing cost.
    return true;
  }

  // UTurn
  if (UTurnCost(map, cost_all)) {
    std::vector<std::string> debug_strings{};
    for (const auto &seq : cost_all) {
      std::ostringstream stream;
      stream << "Lanes:";
      for (const auto &lane : seq.first->lanes()) {
        stream << lane->id() << " , ";
      }
      stream << "..  Cost:" << seq.second.first
             << ", Sign:" << seq.second.second;
      debug_strings.push_back(stream.str());
    }
    Log2DDS::LogDataV2("uturn-target-dynamic-cost", debug_strings);
    return true;
  }
  // route info
  TurnType first_junction_turn_type = NO_TURN;
  uint64_t first_junction_section_id;
  double dis_between_junction = std::numeric_limits<double>::max();
  SerialJunctionType navi_turn = map->route()->NextTwoJunctionOnNavi(
      dis_between_junction, first_junction_turn_type,
      first_junction_section_id);
  // sign info
  UpdateSignInLaneseq(start_point, cost_all);
  // occupied and perfect
  // 1 perfect cost
  bool has_perfect_lane = false;
  for (auto &seq : cost_all) {
    if (seq.second.second == 1) {
      has_perfect_lane = true;
      seq.second.first -= 10.0;
    }
  }
  if (has_perfect_lane) {
    return true;
  }
  // 2 occupied cost
  for (auto &seq : cost_all) {
    if (seq.second.second == -1 || seq.second.second == 2) {
      seq.second.first += 10.0;
    }
  }
  // other cost
  if (first_junction_turn_type == NO_TURN) {
    GoStraightCost(start_point, start_theta, map, cost_all);
  } else if (first_junction_turn_type == LEFT_TURN ||
             first_junction_turn_type == U_TURN) {
    TurnLeftCost(start_point, map, navi_turn, first_junction_section_id,
                 dis_between_junction, cost_all);
  } else if (first_junction_turn_type == RIGHT_TURN) {
    TurnRightCost(start_point, map, navi_turn, first_junction_section_id,
                  cost_all);
  }
  std::vector<std::string> debug_strings{};
  for (const auto &seq : cost_all) {
    std::ostringstream stream;
    stream << "Lanes:";
    for (const auto &lane : seq.first->lanes()) {
      stream << lane->id() << " , ";
    }
    stream << "..  Cost:" << seq.second.first << ", Sign:" << seq.second.second;
    // LOG_ERROR << stream.str();
    debug_strings.push_back(stream.str());
  }
  Log2DDS::LogDataV2("target-dynamic-cost", debug_strings);
  return true;
}

void GoStraightCost(
    const Vec2d &start_point, const double start_theta, const MapPtr &map,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all) {
  bool has_calcu_angle = false;  // all seq use same angle
  double same_angle = 0.0;
  if (!map) return;

  for (auto &lanseq_pair : cost_all) {
    // non-junction check
    const auto &lanes = lanseq_pair.first->lanes();
    if (lanes.empty()) return;
    LaneConstPtr before_split_lane = lanes.front();
    for (int i = 0; i < lanes.size() - 1; i++) {
      if (lanes[i] && lanes[i]->next_lane_ids().size() >= 2 && lanes[i + 1] &&
          !(lanes[i + 1]->junction_id() == 0) &&
          lanes[i + 1]->type() != LaneType::LANE_NORMAL) {
        before_split_lane = lanes[i];
        break;
      }
    }
    if (before_split_lane) {
      for (const auto &lane_id : before_split_lane->next_lane_ids()) {
        LaneConstPtr lane = map->GetLaneById(lane_id);
        if (lane && lane->junction_id() == 0) {
          return;
        }
      }
    }

    double straight_human_cost = 2.0;
    // 1 get junction virtual lane and and its predecessor and successor lanes
    uint64_t first_junction_lane_id = 0;
    uint64_t pre_junction_lane_id = 0;
    uint64_t succeed_junction_lane_id = 0;
    int suc_lane_idx = -1;
    std::vector<Point2d> suc_lane_points;
    int idx = 0;
    bool first_junction = false, before_junction_lane_found = false;
    LaneConstPtr first_lane = lanseq_pair.first->lanes().front();
    for (auto lane : lanseq_pair.first->lanes()) {
      if (!before_junction_lane_found) {
        if (lane && before_split_lane &&
            before_split_lane->id() == lane->id()) {
          before_junction_lane_found = true;
        }
        idx++;
        continue;
      }

      if (!(lane->junction_id() == 0) && lane->turn_type() == NO_TURN) {
        first_junction_lane_id = lane->id();
        if (idx > 0 && idx < lanseq_pair.first->lanes().size() - 1) {
          pre_junction_lane_id = lanseq_pair.first->lanes().at(idx - 1)->id();
          succeed_junction_lane_id =
              lanseq_pair.first->lanes().at(idx + 1)->id();
          if (!(succeed_junction_lane_id == 0)) {
            idx++;
            while (idx < lanseq_pair.first->lanes().size()) {
              const auto suc_lane = lanseq_pair.first->lanes().at(idx);
              if (suc_lane && suc_lane->center_line().IsValid()) {
                suc_lane_points.insert(suc_lane_points.end(),
                                       suc_lane->center_line().points().begin(),
                                       suc_lane->center_line().points().end());
              } else {
                break;
              }
              idx++;
            }
          }
        }
        break;
      }
      auto first_junction_lane = map->GetLaneById(first_junction_lane_id);
      if (!first_junction_lane &&
          (lane->split_topology() == TOPOLOGY_SPLIT_LEFT ||
           lane->split_topology() == TOPOLOGY_SPLIT_RIGHT)) {
        lanseq_pair.second.first += 2.0;
        first_junction = true;
        break;
      }
      idx++;
    }
    if (first_junction) continue;
    auto first_junction_lane = map->GetLaneById(first_junction_lane_id);
    if (first_junction_lane &&
        map->CheckIfValidCommonLane(first_junction_lane)) {
      continue;
    }
    // 2 calculate smooth
    auto pre_junction_lane = map->GetLaneById(pre_junction_lane_id);
    auto succeed_junction_lane = map->GetLaneById(succeed_junction_lane_id);
    if (first_junction_lane && pre_junction_lane && succeed_junction_lane) {
      std::ostringstream stream;
      int pre_left_index = 0, pre_right_index = 0;
      int suc_left_index = 0, suc_right_index = 0;
      bool index_valid = CalcuLaneIndex(
          map, pre_junction_lane, first_junction_lane, succeed_junction_lane,
          pre_left_index, pre_right_index, suc_left_index, suc_right_index);
      int pre_lane_cnt = pre_left_index + pre_right_index + 1;
      int suc_lane_cnt = suc_left_index + suc_right_index + 1;
      bool is_first_left = (pre_left_index == 0);
      bool is_first_right = (pre_right_index == 0);
      int left_index_diff = std::abs(pre_left_index - suc_left_index);
      int right_index_diff = std::abs(pre_right_index - suc_right_index);
      stream << "pre_lane:" << pre_junction_lane->id()
             << ", suc_lane:" << succeed_junction_lane->id()
             << ", p_cnt:" << pre_lane_cnt << ", s_cnt:" << suc_lane_cnt
             << ", pre_left_index:" << pre_left_index
             << ", suc_left_index:" << suc_left_index;

      // set has_straight_junction
      lanseq_pair.first->SetHasStraightJunction(true);
      // set suc_junction_lane_lc_num
      const int suc_junc_lc_num =
          map->GetPriorityLaneRelation(succeed_junction_lane);
      lanseq_pair.first->SetSucceedStraightJunctionLcNum(abs(suc_junc_lc_num));
      // calculate suc_junc_navi_dis
      const double suc_junc_navi_dis = succeed_junction_lane->navi_distance() +
                                       succeed_junction_lane->topo_length();

      // if there are not enough valid lane points, only topological cost.
      if (!first_junction_lane->center_line().IsValid() ||
          !pre_junction_lane->center_line().IsValid() ||
          !succeed_junction_lane->center_line().IsValid()) {
        int left_crowd_diff = std::max(pre_left_index - suc_left_index, 0);
        int right_crowd_diff = std::max(pre_right_index - suc_right_index, 0);
        int crowd_cost = std::max(left_crowd_diff / (pre_left_index + 1),
                                  right_crowd_diff / (suc_right_index + 1));
        lanseq_pair.second.first += crowd_cost;
        stream << ", topo_cost:" << crowd_cost;
        Log2DDS::LogDataV2("target-dynamic-debug", stream.str());
        continue;
      }
      // enter junction vector
      Vec2d pre_vec =
          pre_junction_lane->center_line().end_point() -
          pre_junction_lane->center_line().GetPointAtS(
              std::fmax(pre_junction_lane->center_line().length() - 15.0, 0.0));

      // quit junction vector
      Path suc_path(suc_lane_points);
      std::vector<PathPoint> points;
      suc_path.SamplePoints(0.0, suc_path.length(), 1.0, &points);
      const double check_s_range = 15.0;
      Vec2d suc_begin_point = points.front();
      Vec2d suc_end_point = suc_path.GetPrecisionPointAtS(
          std::fmin(suc_path.length(), check_s_range));
      // drop follow codes, use suc_begin_point as fixed.
      // const double check_dk = 0.0004;
      // bool kappa_out = false;
      // double last_kappa = points.front().kappa;
      // for (const auto &point : points) {
      //   if (point.accum_s > check_s_range - 1.0 || point == points.back() ||
      //       point == suc_end_point) {
      //     break;
      //   }
      //   if (point == points.front()) {
      //     last_kappa = points.front().kappa;
      //     continue;
      //   }
      //   if (kappa_out) {
      //     suc_begin_point = point;
      //     kappa_out = false;
      //   }
      //   if (std::fabs(point.kappa - last_kappa) > check_dk) {
      //     kappa_out = true;
      //   }
      //   last_kappa = point.kappa;
      // }
      math::Vec2d suc_vec = suc_end_point - suc_begin_point;

      // virtual lane vector
      math::Vec2d virtual_vec =
          suc_begin_point - first_junction_lane->center_line().begin_point();

      // calculate turn
      double pre_to_suc_trans =
          (pre_vec.CrossProd(suc_vec)) /
          (std::sqrt(pre_vec.Sqr()) * std::sqrt(suc_vec.Sqr()));
      if (has_calcu_angle) {
        pre_to_suc_trans = same_angle;
      } else {
        same_angle = pre_to_suc_trans;
        has_calcu_angle = true;
      }

      // check obs position
      //   std::vector<ObstaclePtr> left_overlap_obs;
      //   for (const auto &obs : planning_context.traffic_info().obstacles()) {
      //     if (!obs) continue;
      //     if (obs->type() == OBJECT_UNKNOWN_UNMOVABLE ||
      //         obs->type() == OBJECT_CONE || obs->type() == OBJECT_BARREL ||
      //         obs->type() == OBJECT_BARRIER) {
      //       continue;
      //     }
      //     if (obs->type() == OBJECT_PEDESTRIAN || obs->type() ==
      //     OBJECT_BICYCLE ||
      //         obs->type() == OBJECT_MOTORCYCLE) {
      //       continue;
      //     }
      //     const double back_s_buffer = std::fmax(obs->v() * 3.0, 5.0);
      //     if (obs->ds() < -back_s_buffer) continue;

      //     const double front_s_buffer = std::fmax(start_point.v * 3.0, 5.0);
      //     if (obs->ds() > front_s_buffer) continue;

      //     if ((obs->dl() > Constants::ZERO) && (obs->dl() < 3.0)) {
      //       left_overlap_obs.emplace_back(obs);
      //     }
      //   }
      // check ego position
      // LaneConstPtr nearest_lane =
      //     map->GetNearestLane(Point2d(start_point.x(), start_point.y()),
      //                         start_theta, Constants::DEFAULT_LANE_WIDTH,
      //                         true);
      // LaneConstPtr left_lane = map->GetLeftLane(nearest_lane);
      // const std::vector<LaneType> black_list = {LANE_NON_MOTOR, LANE_UNKNOWN,
      //                                           LANE_EMERGENCY};
      // bool is_valid_left_lane =
      //     left_lane && std::find(black_list.begin(), black_list.end(),
      //                            left_lane->type()) == black_list.end();

      // bool right_1st_allow_cutin = false; /*TODO: reopen xxx xiecha*/
      //   pre_lane_cnt == 1 || !is_valid_left_lane ||
      //   (is_valid_left_lane && left_overlap_obs.empty());

      bool succeed_lane_has_right_turn = false;
      for (const auto &pre_lane_id : succeed_junction_lane->pre_lane_ids()) {
        LaneConstPtr pre_lane = map->GetLaneById(pre_lane_id);
        if (pre_lane && pre_lane->turn_type() == TurnType::RIGHT_TURN) {
          succeed_lane_has_right_turn = true;
          break;
        }
      }
      double junction_lane_legth = std::max(first_junction_lane->topo_length(),
                                            Constants::DEFAULT_LANE_WIDTH);
      double pre_to_virtual_trans =
          (pre_vec.CrossProd(virtual_vec)) /
          (std::sqrt(pre_vec.Sqr()) * std::sqrt(virtual_vec.Sqr()));

      stream << ", p2s_tr:" << pre_to_suc_trans
             << ", p2v_tr:" << pre_to_virtual_trans;

      // cost
      bool navi_is_first = false;
      const double navi_dis_min = abs(suc_junc_lc_num) * 200.0 + 100.0;
      if (suc_junc_navi_dis < navi_dis_min) {
        bool angle_enough =
            (suc_junc_lc_num > 0 &&
             pre_to_suc_trans > std::sin(5.0 * Constants::DEG2RAD)) ||
            (suc_junc_lc_num < 0 &&
             pre_to_suc_trans < -std::sin(5.0 * Constants::DEG2RAD));
        bool junction_length_enough = abs(junction_lane_legth) > 50.0;
        if (angle_enough && junction_length_enough) {
          navi_is_first = true;
        }
      }
      if (navi_is_first) {
        straight_human_cost += abs(suc_junc_lc_num);
        stream << ", lc cost:" << abs(suc_junc_lc_num);
      } else if (index_valid && suc_path.length() > 5.0 && pre_lane_cnt > 1 &&
                 std::fabs(pre_to_suc_trans) >
                     std::sin(10.0 *
                              Constants::DEG2RAD)) {  // scen 1: bend road 0.173
        double index_diff =
            pre_to_suc_trans > 0.0 ? left_index_diff : right_index_diff;
        straight_human_cost = 2.0 * (index_diff / 5.0);
        straight_human_cost = math::Clamp(straight_human_cost, 0.0, 2.0);
      } else if (index_valid && (pre_lane_cnt == suc_lane_cnt) &&
                 pre_lane_cnt <= 3
                 /*(is_first_left || pre_lane_cnt == 2)*/) {
        // scen 2: aligned junction for left-first
        double index_diff = left_index_diff;
        straight_human_cost = 2.0 * (index_diff / 5.0);
        straight_human_cost = math::Clamp(straight_human_cost, 0.0, 2.0);
      } else if (index_valid && (pre_lane_cnt < suc_lane_cnt) &&
                 is_first_right && false) {
        // scen 3: unaligned junction for right-first
        double index_diff = std::abs(right_index_diff - 1);
        straight_human_cost = 2.0 * (index_diff / 5.0);
        straight_human_cost = math::Clamp(straight_human_cost, 0.0, 2.0);
        if (pre_lane_cnt == 1) {  // scen 3.2: 1 to n
          // 1 to n: navi cost
          double navi_distance = 0.0;
          bool navi_dist_after_junc = false;
          for (const auto &lane : lanseq_pair.first->lanes()) {
            if (!lane) continue;
            if (lane->id() == succeed_junction_lane->id()) {
              navi_dist_after_junc = true;
            }
            if (!navi_dist_after_junc) continue;
            if (lane->is_navigation()) {
              navi_distance += lane->topo_length();
            } else {
              break;
            }
          }
          int pri_lane_relation =
              std::abs(map->GetPriorityLaneRelation(succeed_junction_lane));
          if (navi_distance < pri_lane_relation * 30.0) {
            straight_human_cost += 2.0;
          }
          // 1 to n: merge cost...TODO: merge to right
        }
      } else {  // scen 4: normal straight junction
        // discourage first lane when pre lane is no first lane
        if (index_valid && (pre_lane_cnt <= suc_lane_cnt) && !is_first_left &&
            !is_first_right && (0 == suc_left_index || 0 == suc_right_index)) {
          straight_human_cost += 1.0;
        }
        if (index_valid && suc_lane_cnt > pre_lane_cnt &&
            suc_left_index > pre_left_index) {
          double pre2vir_trans_base =
              -1.0 * Constants::DEFAULT_LANE_WIDTH / junction_lane_legth;
          if (is_first_right && succeed_lane_has_right_turn) {
            pre2vir_trans_base *= 0.3;
            stream << ", p2v_tr_base1:";
          } else if (is_first_left) {
            if (suc_right_index <= pre_right_index) {
              pre2vir_trans_base *= 0.3;
              stream << ", p2v_tr_base2:";
            } else {
              pre2vir_trans_base *= 0.5;
              stream << ", p2v_tr_base3:";
            }
          } else if (suc_lane_cnt - pre_lane_cnt > 1 && !is_first_right &&
                     suc_left_index - pre_left_index > 1) {
            pre2vir_trans_base *= 0.3;
            stream << ", p2v_tr_base4:";
          } else {
            pre2vir_trans_base *= 0.5;
            stream << ", p2v_tr_base5:";
          }
          stream << pre2vir_trans_base;
          if (pre_to_virtual_trans < pre2vir_trans_base)
            straight_human_cost += 1.0;
        }
        if (index_valid && suc_lane_cnt == pre_lane_cnt &&
            suc_right_index == pre_right_index) {
          double same_index_cost =
              0.5 * Constants::DEFAULT_LANE_WIDTH / junction_lane_legth;
          same_index_cost =
              std::sqrt(std::max(0.0, 1 - same_index_cost * same_index_cost)) -
              1;
          straight_human_cost += same_index_cost;
          stream << ", same_index_cost:" << same_index_cost;
        }
        double pre_transvection =
            (pre_vec.InnerProd(virtual_vec)) /
            (std::sqrt(pre_vec.Sqr()) * std::sqrt(virtual_vec.Sqr()));
        straight_human_cost -= pre_transvection * 0.2;
        stream << ", pre_trans:" << pre_transvection;

        if (suc_path.length() > 5.0) {
          double succeed_transvection =
              (suc_vec.InnerProd(virtual_vec)) /
              (std::sqrt(suc_vec.Sqr()) * std::sqrt(virtual_vec.Sqr()));
          straight_human_cost -= succeed_transvection * 0.8;
          stream << ", suc_trans:" << succeed_transvection;
        }
      }

      // succeed lane's lat difference ratio.
      const double lat_diff_ratio = pre_to_virtual_trans * junction_lane_legth /
                                    Constants::DEFAULT_LANE_WIDTH;

      // add short merge lane cost, conditions:
      // 1. pre junction lanes number is less than succeed junciton lanes
      //    number.
      // 2. succeed lane is leftmost and not deviates right too much, or succeed
      //    lane is rightmost and not deviates left too much.
      // 3. succeed lane's merge point is shorter than 120 meters.
      if ((pre_lane_cnt < suc_lane_cnt) &&
          ((is_first_left && (lat_diff_ratio > -0.5)) ||
           (is_first_right && (lat_diff_ratio < 0.5)))) {
        constexpr double merge_s_max = 120.0;
        constexpr double merge_s_full_cost = 80.0;
        constexpr double merge_cost = 0.5;
        // lambda function: check whether the short merge seq.
        auto is_avoid_merge =
            [is_first_right](const LaneConstPtr &lane) -> bool {
          if (is_first_right)
            return (lane->merge_topology() ==
                    MergeTopology::TOPOLOGY_MERGE_LEFT);
          return (lane->merge_topology() ==
                  MergeTopology::TOPOLOGY_MERGE_RIGHT);
        };
        // check whether there is a merge point in merge_s_max range.
        double merge_distance = 0.0;
        for (int idx = suc_lane_idx; idx < lanes.size(); idx++) {
          const auto &merge_pre_lane = lanes.at(idx);
          if (!merge_pre_lane) break;
          merge_distance += merge_pre_lane->topo_length();
          // only consider the merge point in the range of merge_s_max.
          if (merge_distance > merge_s_max) break;
          // there is a merge point.
          if (is_avoid_merge(merge_pre_lane)) {
            // if m_dis < m_min, cost = merge_cost;
            // if m_dis > m_max, cost = 0;
            // other condition,
            // cost = (1 - (m_dis - m_min) / (m_max - m_min)) * cost;
            double cost = (merge_distance < merge_s_full_cost)
                              ? merge_cost
                              : (1.0 - (merge_distance - merge_s_full_cost) /
                                           (merge_s_max - merge_s_full_cost)) *
                                    merge_cost;
            straight_human_cost += cost;
            stream << ", mg_id:" << merge_pre_lane->id() << ", mg_cst:" << cost;
            // don't find merge point anymore, break.
            break;
          }
        }
      }

      // add bus-lane and bus-station cost, conditions:
      // 1. only consider the cost when the lane before intersection is not bus
      // lane.
      const bool is_from_bus_lane = IsBusLane(pre_junction_lane);
      if (!is_from_bus_lane) {
        bool start_count = false;
        double length_count = 0.0;
        constexpr double consider_bus_lane_s_range = 100.0;
        constexpr double bus_lane_cost_max = 1.0;

        // 2. If the succeed lane deviates too much, decrease or cancle the bus
        // lane cost.
        constexpr double lat_diff_ratio_upper = 0.5;
        constexpr double lat_diff_ratio_lower = 0.3;
        bool deviates_too_much = false;
        if ((suc_right_index == 0 && (lat_diff_ratio > lat_diff_ratio_upper)) ||
            (suc_left_index == 0 && (lat_diff_ratio < -lat_diff_ratio_upper))) {
          stream << ", suc-lane(" << succeed_junction_lane->id()
                 << ")'s lat diff too large, ignore bus lane cost.";
          deviates_too_much = true;
        }

        if (!deviates_too_much) {
          double bus_lane_cost = bus_lane_cost_max;
          if (suc_right_index == 0 && lat_diff_ratio > lat_diff_ratio_lower) {
            bus_lane_cost = (lat_diff_ratio - lat_diff_ratio_lower) /
                            (lat_diff_ratio_upper - lat_diff_ratio_lower) *
                            bus_lane_cost_max;
          } else if (suc_left_index == 0 &&
                     (lat_diff_ratio < -lat_diff_ratio_lower)) {
            bus_lane_cost = abs(lat_diff_ratio + lat_diff_ratio_lower) /
                            (lat_diff_ratio_upper - lat_diff_ratio_lower) *
                            bus_lane_cost_max;
          }

          for (auto lane : lanseq_pair.first->lanes()) {
            if (!lane) continue;
            // only consider the lanes after the intersection.
            if (!start_count && succeed_junction_lane->id() == lane->id()) {
              start_count = true;
            }

            if (start_count) {
              // only consider the bus lanes between this juntion and next
              // junction. If reach the next junciton, don't consider this seq
              // anymore.
              if (lane->junction_id() != 0) {
                stream << ", none bus lane until next junc(" << lane->id()
                       << ").";
                break;
              }

              bool is_bus_lane = IsBusLane(lane);
              if (is_bus_lane) {
                straight_human_cost += bus_lane_cost;
                stream << ", add bus lane(" << lane->id() << ") cost("
                       << bus_lane_cost << ").";
                break;
              }

              // only consider bus lane cost in a certain range.
              length_count += lane->topo_length();
              if (length_count > consider_bus_lane_s_range) {
                break;
              }
            }
          }
        }
      }
      Log2DDS::LogDataV2("target-dynamic-debug", stream.str());
    }

    // 3 update cost
    lanseq_pair.second.first += straight_human_cost;
  }
  return;
}

void TurnLeftCost(
    const Vec2d &start_point, const MapPtr &map,
    const SerialJunctionType &navi_turn, const uint64_t junction_section_id,
    const double &dist_btw_junc,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all) {
  if (cost_all.empty() || !map) {
    return;
  }
  std::stringstream stream;
  // 2 check if is single turn
  // bool is_single_turn = false;
  // SectionInfo section;
  // if (map->route()->GetSectionByIdFromRoute(junction_section_id, section)) {
  //   std::map<uint64_t, bool> parent_lanes;
  //   for (const auto &lane_id : section.lane_ids) {
  //     LaneConstPtr lane = map->GetLaneById(lane_id);
  //     if (lane && !lane->pre_lane_ids().empty()) {
  //       parent_lanes[lane->pre_lane_ids().front()] = true;
  //     }
  //   }
  //   if (parent_lanes.size() == 1) is_single_turn = true;
  // }
  // if (!is_single_turn) {
  //   stream << "junc's sec-id " << junction_section_id << " isn't single
  //   turn."; Log2DDS::LogDataV2("turn-L-dynamic-debug", stream.str()); return;
  // }
  // 3 turn cost
  for (auto &seq : cost_all) {
    // 3.1 find quit junc lane and next junc lane
    uint64_t front_junction_id = 0;
    uint64_t front_junction_lane_id = 0;
    uint64_t quit_lane_id = 0;
    int quit_lane_idx = 0;
    bool flag = false;
    for (const auto &lane : seq.first->lanes()) {
      if (flag && lane && lane->junction_id() == 0) {
        quit_lane_id = lane->id();
        break;
      }
      if (lane && !(lane->junction_id() == 0)) {
        flag = true;
        front_junction_id = lane->junction_id();
        front_junction_lane_id = lane->id();
      }
      quit_lane_idx++;
    }
    LaneConstPtr quit_lane = map->GetLaneById(quit_lane_id);
    if (!flag || !quit_lane) continue;
    // SectionInfo quit_sec;
    // map->route()->GetSectionByIdFromRoute(quit_lane->section_id(), quit_sec);
    SectionConstPtr quit_sec_ptr = map->GetSectionById(quit_lane->section_id());
    LaneConstPtr prio_lane = map->GetNaviPriorityLane(quit_lane->section_id());
    if (!prio_lane) continue;

    LaneConstPtr next_virtual_lane = nullptr;
    seq.first->GetDistanceBetweenJunction(front_junction_id, next_virtual_lane);

    // 3.2 calcu distance to left lane and right lane
    // int left_dis = 1;
    // LaneConstPtr left_lane = map->GetLeftLane(quit_lane);
    // while (left_lane) {
    //   left_dis++;
    //   left_lane = map->GetLeftLane(left_lane);
    // }
    // int right_dis = 1;
    // LaneConstPtr right_lane = map->GetRightLane(quit_lane);
    // while (right_lane) {
    //   right_dis++;
    //   right_lane = map->GetRightLane(right_lane);
    // }

    // 3.3 calcu cost
    if (navi_turn == SerialJunctionType::LEFT_STRAIGHT &&
        dist_btw_junc < 50.0) {
      if (!next_virtual_lane || !next_virtual_lane->is_navigation()) {
        double cost = 0.0;
        if (quit_sec_ptr) {
          const auto &it0 =
              std::find(quit_sec_ptr->lanes().begin(),
                        quit_sec_ptr->lanes().end(), quit_lane->id());
          const auto &it1 =
              std::find(quit_sec_ptr->lanes().begin(),
                        quit_sec_ptr->lanes().end(), prio_lane->id());
          cost =
              2.0 * std::abs(static_cast<int>(std::distance(it0, it1))) / 5.0;
        }
        seq.second.first += cost;
        stream << "navi:" << cost;
      }
    }

    if (front_junction_lane_id != 0) {
      LaneConstPtr front_junction_lane_ptr =
          map->GetLaneById(front_junction_lane_id);
      if (front_junction_lane_ptr && front_junction_lane_ptr->is_exp_traj()) {
        double exp_traj_cost = -1.0;
        seq.second.first += exp_traj_cost;
        stream << ",exp:" << exp_traj_cost;
      }
      if (quit_lane) {
        constexpr double consider_distance = 200.0;
        double accum_s = 0.0;
        for (int idx = quit_lane_idx; idx < seq.first->lanes().size(); idx++) {
          const auto &lane = seq.first->lanes().at(idx);
          if (!lane) break;
          if (IsBusLane(lane)) {
            double bus_cost = 2.0;
            seq.second.first += bus_cost;
            stream << ",bus:" << bus_cost;
            break;
          }
          accum_s += lane->topo_length();
          if (accum_s > consider_distance) break;
        }
      }
    }
    Log2DDS::LogDataV2("turn-L-dynamic-debug", stream.str());
    stream.str("");
  }
  return;
}

bool UTurnCost(
    const MapPtr &map,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all) {
  if (cost_all.empty() || !map) {
    return false;
  }
  bool have_uturn = false;
  for (auto &seq : cost_all) {
    LaneConstPtr uturn_lane = nullptr;
    LaneConstPtr quit_lane = nullptr;
    LaneConstPtr pre_lane = nullptr;
    bool this_laneseq_has_uturn = false;
    for (const auto &lane : seq.first->lanes()) {
      if (!lane) continue;
      if (lane->junction_id() != 0 && lane->turn_type() != TurnType::U_TURN) {
        return false;
      }
      if (this_laneseq_has_uturn && lane->turn_type() != TurnType::U_TURN) {
        quit_lane = lane;
        break;
      }
      if (lane->turn_type() == TurnType::U_TURN) {
        this_laneseq_has_uturn = true;
        have_uturn = true;
        uturn_lane = lane;
        continue;
      }
      pre_lane = lane;
    }
    if (!uturn_lane || !quit_lane || !pre_lane) {
      continue;
    }

    int suc_left_index = 0, suc_right_index = 0;
    bool index_valid =
        CalcuSucceedLaneIndex(map, quit_lane, suc_left_index, suc_right_index);
    bool is_rightmost = (index_valid && suc_right_index == 0) ? true : false;
    constexpr double prio_lat_distance = 10.0;
    double delta_lat_coeff = 10.0;
    if (quit_lane->center_line().IsValid() &&
        pre_lane->center_line().IsValid()) {
      double dist1 = quit_lane->center_line().GetDistance(
          pre_lane->center_line().end_point());
      double dist2 = pre_lane->center_line().GetDistance(
          quit_lane->center_line().begin_point());
      double dist = std::min(dist1, dist2);
      delta_lat_coeff = (dist - prio_lat_distance) / 3.75;
    }
    const double cost1 =
        delta_lat_coeff < 0.0 ? 5.0 * abs(delta_lat_coeff) : delta_lat_coeff;
    const double cost2 = uturn_lane->is_exp_traj() ? -0.5 : 0.0;
    const double cost3 = is_rightmost ? 1.0 : 0.0;
    const double cost4 = (!is_rightmost && IsBusLane(quit_lane)) ? 2.0 : 0.0;
    const double cost = cost1 + cost2 + cost3 + cost4;
    seq.second.first = cost;
    std::stringstream stream;
    stream << "uturn_lane:" << uturn_lane->id()
           << ", suc_right_index:" << suc_right_index
           << ", delta_lat_coeff:" << delta_lat_coeff << ", cost: " << cost;
    Log2DDS::LogDataV2("Uturn-dynamic", stream.str());
  }
  return have_uturn;
}

void TurnRightCost(
    const Vec2d &start_point, const MapPtr &map,
    const SerialJunctionType &navi_turn, const uint64_t junction_section_id,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all) {
  if (cost_all.empty()) {
    return;
  }
  for (auto &seq : cost_all) {
    std::stringstream stream;
    // 1. find  junc lane id and record index
    uint64_t first_junction_lane_id = 0;
    LaneConstPtr pre_lane = nullptr;
    LaneConstPtr suc_lane = nullptr;
    int junc_idx = 0;
    for (const auto &lane : seq.first->lanes()) {
      if (lane && !(lane->junction_id() == 0)) {
        first_junction_lane_id = lane->id();
        // get pre junciton lane.
        if (junc_idx > 0) {
          pre_lane = seq.first->lanes().at(junc_idx - 1);
        }
        // get succeed junciton lane.
        if (junc_idx < seq.first->lanes().size() - 1) {
          suc_lane = seq.first->lanes().at(junc_idx + 1);
        }
        break;
      }
      junc_idx++;
    }

    if (first_junction_lane_id != 0) {
      stream << "junc_lane:" << first_junction_lane_id << ", ";
      LaneConstPtr front_junction_lane_ptr =
          map->GetLaneById(first_junction_lane_id);
      if (front_junction_lane_ptr && front_junction_lane_ptr->is_exp_traj()) {
        double exp_traj_cost = -1.0;
        seq.second.first += exp_traj_cost;
        stream << "exp:" << exp_traj_cost << ", ";
      }
      // calculate the pre junction lane's turn right split branches number.
      int split_branches_num = 0;
      if (pre_lane) {
        for (int lane_id : pre_lane->next_lane_ids()) {
          LaneConstPtr lane_ptr = map->GetLaneById(lane_id);
          if (lane_ptr && lane_ptr->junction_id() != 0 &&
              lane_ptr->turn_type() == RIGHT_TURN) {
            split_branches_num++;
          }
        }
      }
      // Avoid rightmost lane, conditions:
      // 1. succeed lane is the rightmost;
      // 2. pre lane's next turn right branches number is more than one;
      if (suc_lane && (suc_lane->right_lane_id() == 0) &&
          (split_branches_num > 1)) {
        constexpr double avid_R1_cost = 0.5;
        const auto &R2_lane = map->GetLaneById(suc_lane->left_lane_id());
        if (R2_lane) {
          double R2_navi_s = R2_lane->navi_distance() + R2_lane->topo_length();
          // if succeed lane's navi distance is long enough, prioritize it.
          if (R2_navi_s > 100.0) {
            seq.second.first += avid_R1_cost;
            stream << "avoid_R1:" << avid_R1_cost << ", ";
          } else {
            const int R1_lc_num = abs(map->GetPriorityLaneRelation(suc_lane));
            const int R2_lc_num = abs(map->GetPriorityLaneRelation(R2_lane));
            // if succeed lane's lc_num is smaller, prioritize it.
            if (R2_lc_num < R1_lc_num) {
              seq.second.first += avid_R1_cost;
              stream << "avoid_R1:" << avid_R1_cost << ", ";
            }
          }
        }
      }
    }
    Log2DDS::LogDataV2("turn-R-dynamic", stream.str());
  }

  return;
}

bool GetStaticCost(
    const Vec2d &start_point, const MapPtr &map, const LaneConstPtr &lane,
    const bool &navi_is_priority, const bool &split_is_navi_priority,
    const uint64_t lane_id,
    std::vector<std::pair<LaneSequencePtr, double>> &cost_static) {
  cost_static.clear();
  auto cur_lane = lane;
  if (!cur_lane) {
    return false;
  }
  const double kGetAllLaneSeqLength = 1000.0;
  LaneConstPtr last_navi_lane = nullptr;
  // TODO: reopen
  //   if (planning_context.cur_lane_sequence()) {
  //     const auto cur_lanes = planning_context.cur_lane_sequence()->lanes();
  //     for (auto it = cur_lanes.rbegin(); it != cur_lanes.rend(); it++) {
  //       if ((*it) && (*it)->is_navigation()) {
  //         last_navi_lane = (*it);
  //         break;
  //       }
  //     }
  //   }

  LaneConstPtr split_front_lane = nullptr;
  std::vector<std::vector<LaneConstPtr>> all_lanes;
  map->GetAllLaneSequences(cur_lane, kGetAllLaneSeqLength, all_lanes, false);
  std::vector<LaneSequencePtr> all_lane_sequences;
  for (const auto &lanes : all_lanes) {
    bool need_skip = false;
    if (0 != lane_id) {
      for (const auto &lane : lanes) {
        if (lane->id() == lane_id) {
          need_skip = true;
          break;
        }
      }
    }
    if (need_skip) {
      continue;
    }
    LaneSequencePtr laneseq = std::make_shared<LaneSequence>(lanes);
    all_lane_sequences.emplace_back(laneseq);
  }
  std::vector<std::pair<std::string, double>> debug_strings{};
  for (const auto &lane_sequence : all_lane_sequences) {
    if (!lane_sequence || !lane_sequence->IsValid()) {
      continue;
    }
    bool short_continuous_junction = false;
    LaneConstPtr first_junc_lane = nullptr;
    LaneConstPtr next_junc_lane = nullptr;
    for (const auto &lane : lane_sequence->lanes()) {
      if (lane && !(lane->junction_id() == 0) &&
          !map->CheckIfValidCommonLane(lane)) {
        first_junc_lane = lane;
        break;
      }
    }
    if (!lane_sequence->lanes().empty()) {
      for (const auto &lane : lane_sequence->lanes()) {
        if (lane && lane->next_lane_ids().size() > 1) {
          int valid_navi_suc_lane_cnt = 0;
          for (const auto &lane_id : lane->next_lane_ids()) {
            LaneConstPtr suc_lane = map->GetLaneById(lane_id);
            if (suc_lane && suc_lane->is_navigation()) {
              valid_navi_suc_lane_cnt++;
            }
          }
          if (valid_navi_suc_lane_cnt > 1) {
            split_front_lane = lane;
            break;
          }
        }
      }
    }
    if (first_junc_lane) {
      double dis_btw_junction = lane_sequence->GetDistanceBetweenJunction(
          first_junc_lane->junction_id(), next_junc_lane);
      if (next_junc_lane && dis_btw_junction < 300.0) {
        short_continuous_junction = true;
      }
    }
    // cost split
    double cost_split = 0.0;
    LaneConstPtr lane_front = nullptr;
    LaneConstPtr lane_back = nullptr;
    LaneConstPtr lane_quit = nullptr;
    for (const auto &lane : lane_sequence->lanes()) {
      if (lane_back) {
        lane_quit = lane;
        break;
      }
      if (lane_front) {
        lane_back = lane;
        continue;
      }
      if (lane && lane->next_lane_ids().size() > 1) {
        lane_front = lane;
      }
    }
    if (lane_back && lane_back->junction_id() == 0) {
      if (lane_back->split_topology() != TOPOLOGY_SPLIT_NONE) {
        cost_split = 1.0;
      } else if (lane_front && lane_back->type() != lane_front->type()) {
        cost_split = 0.1;
      }
    } else if (lane_back && !(lane_back->junction_id() == 0) &&
               (lane_back->turn_type() == TurnType::LEFT_TURN ||
                lane_back->turn_type() == TurnType::RIGHT_TURN)) {
      cost_split = 1.0;
    } else if (lane_front && lane_back && lane_front->center_line().IsValid() &&
               lane_back->center_line().IsValid()) {
      bool quit_lane_check = false;
      if (lane_quit && lane_quit->center_line().IsValid()) {
        double s_accu = 0.0;
        math::Vec2d start_p = lane_quit->center_line().begin_point();
        double l_start = 0.0;
        lane_front->center_line().GetProjection(start_p, &s_accu, &l_start);
        math::Vec2d end_p = lane_quit->center_line().end_point();
        double l_end = 0.0;
        lane_front->center_line().GetProjection(end_p, &s_accu, &l_end);
        if (l_start * l_end < 0.0) {
          quit_lane_check = true;
        }
      }
      if (quit_lane_check) {
        cost_split = 1.0;
      } else {
        math::Vec2d ref_vec;
        ref_vec = lane_front->center_line().end_point() -
                  lane_front->center_line().GetPointAtS(std::fmax(
                      lane_front->center_line().length() - 20.0, 0.0));
        math::Vec2d split_vec;
        double length = std::fmin(lane_back->center_line().length(), 10.0);
        split_vec = lane_back->center_line().GetPointAtS(length) -
                    lane_back->center_line().begin_point();
        split_vec = math::Vec2d::CreateUnitVec2d(split_vec.Angle()) * 10.0;

        double distance = ref_vec.CrossProd(split_vec) /
                          std::fmax(ref_vec.Length(), Constants::ZERO);
        cost_split = std::fabs(distance / 5.0);
      }
    }
    // cost obstacle
    double cost_obs = 0.0;

    // cost navi
    double avg_kappa_upper_ref = 1.0 / 9.0;
    double cost_navi = 0.0;
    double accu_navi_dist = 0.0;
    LaneConstPtr nearest_lane = lane_sequence->GetNearestLane(
        math::Vec2d(start_point.x(), start_point.y()));
    if (navi_is_priority && nearest_lane) {
      int section_cnt = 0;
      double turn_supplyment = 0;
      bool start_check = false;
      // bool short_continuous_check = false;
      for (const auto &lane : lane_sequence->lanes()) {
        if (!lane || !lane->is_navigation()) {
          break;
        }
        if (lane->type() == LaneType::LANE_NON_MOTOR ||
            lane->type() == LaneType::LANE_EMERGENCY) {
          break;
        }
        // if (short_continuous_check) {
        //   break;
        // }
        if (accu_navi_dist > 500.0) {
          if (!short_continuous_junction && next_junc_lane &&
              next_junc_lane->id() == lane->id()) {
            break;
          }
          if (short_continuous_junction && next_junc_lane &&
              next_junc_lane->id() == lane->id()) {
            section_cnt++;
            break;
          }
        }

        // cut before junction or non-navi lane
        if (((first_junc_lane && first_junc_lane->id() == lane->id()) ||
             (last_navi_lane &&
              last_navi_lane->section_id() == lane->section_id())) &&
            lane_back && lane_back->junction_id() == 0 &&
            !split_is_navi_priority) {
          break;
        }
        if (!start_check && lane->id() == nearest_lane->id()) {
          double l, s;
          nearest_lane->center_line().GetProjection(
              math::Vec2d(start_point.x(), start_point.y()), &s, &l);
          accu_navi_dist += (nearest_lane->curve_length() - s);
          start_check = true;
        } else if (start_check) {
          accu_navi_dist += lane->topo_length();
        }
        section_cnt++;
        if (lane->turn_type() != TurnType::NO_TURN &&
            lane->type() != LaneType::LANE_LEFT_WAIT &&
            lane->type() != LaneType::LANE_RIGHT_TURN_LANE) {
          double dis = GetTurnDistance(map, lane_sequence, lane);
          if (dis < -Constants::ZERO) {
            dis *= 1.5;
          } else if (dis < Constants::ZERO) {
            if (GetAverageKappa(lane) > avg_kappa_upper_ref) {
              dis = 1.0;
            }
          }
          turn_supplyment += std::fabs(dis);
        }
      }
      if (accu_navi_dist > 500.0) {
        cost_navi = -5000.0;
      } else {
        cost_navi = -1.0 * section_cnt * 100.0;
      }
      cost_navi = std::fmin(0.0, cost_navi + turn_supplyment * 10.0);
    }
    // cost merge
    double cost_merge = 0.0;
    accu_navi_dist = 0.0;
    bool start_check = false;
    for (const auto &lane : lane_sequence->lanes()) {
      if (!lane || accu_navi_dist > 500.0) {
        break;
      }
      if (navi_is_priority && !lane->is_navigation()) {
        break;
      }
      if (lane->merge_topology() == TOPOLOGY_TO_BE_MERGED) {
        break;
      }
      if ((lane->merge_topology() == TOPOLOGY_MERGE_LEFT) ||
          (lane->merge_topology() == TOPOLOGY_MERGE_RIGHT)) {
        // TODO: reopen
        // if (map->city_fence() == WANGJING_FENCE) {
        //   cost_merge += 1.0;
        // }
        break;
      }
      if (!start_check && lane->id() == nearest_lane->id()) {
        double l, s;
        nearest_lane->center_line().GetProjection(
            math::Vec2d(start_point.x(), start_point.y()), &s, &l);
        accu_navi_dist += (nearest_lane->curve_length() - s);
        start_check = true;
      } else if (start_check) {
        accu_navi_dist += lane->topo_length();
      } else {
        continue;
      }
    }
    // cost curvature
    double cost_curvature = 0.0;
    for (const auto &lane : lane_sequence->lanes()) {
      if (lane && lane->center_line().IsValid() &&
          lane->turn_type() == TurnType::U_TURN) {
        double avg_kappav2 = GetAverageKappaV2(lane);
        if (avg_kappav2 > 0.22) {
          cost_curvature = avg_kappav2 * 20;
        } else {
          cost_curvature = avg_kappav2;
        }
        break;
      }
      if (lane && lane->center_line().IsValid() &&
          (lane->turn_type() == TurnType::LEFT_TURN ||
           lane->turn_type() == TurnType::RIGHT_TURN)) {
        double avg_kappa = GetAverageKappa(lane);
        if (avg_kappa > 1.0 / 6.0) {
          cost_curvature = 1.0;
        } else if (avg_kappa > avg_kappa_upper_ref) {
          cost_curvature = 1.5 - 1.0 / avg_kappa / 6.0;
        }
        break;
      }
    }
    // cost route
    double cost_route = 0.0;
    if (navi_is_priority && map->route()) {
      double max_deviate_dis = 0.0;
      for (const auto &lane : lane_sequence->lanes()) {
        if (!lane) {
          continue;
        }
        if (!map->GetNaviPriorityLane(lane->section_id())) {
          // SectionInfo section_info;
          // map->route()->GetSectionByIdFromRoute(lane->section_id(),
          // section_info);
          SectionConstPtr section_ptr = map->GetSectionById(lane->section_id());
          if (section_ptr) {
            max_deviate_dis =
                std::fmax(section_ptr->lanes().size(), max_deviate_dis);
          }
        } else {
          double deviate_dis = std::abs(map->GetPriorityLaneRelation(lane));
          max_deviate_dis = std::fmax(deviate_dis, max_deviate_dis);
        }
      }
      if (!lane_sequence->lanes().empty()) {
        cost_route = max_deviate_dis * 0.1;
      } else {
        cost_route = 1.0;
      }
      cost_route = math::Clamp(cost_route, 0.0, 1.0);
      double s_offset = 0.0;
      lane_sequence->GetProjectionDistance({start_point.x(), start_point.y()},
                                           &s_offset);
      const auto projection_point = lane_sequence->GetPointAtS(s_offset);
      const auto &nearest_lane =
          lane_sequence->GetNearestLane(projection_point);
      if (!map->route()->CanDriveToRouteEnd(lane_sequence, nearest_lane)) {
        cost_route *= 10.0;
      }
    }
    // cost lc
    double cost_lc = 0.0;
    LaneConstPtr parent_lane = nullptr;
    if (navi_is_priority && map->route()) {
      for (const auto &lane : lane_sequence->lanes()) {
        if (!lane) {
          continue;
        }
        if (lane->is_navigation() && parent_lane &&
            lane->type() != LANE_VIRTUAL_JUNCTION &&
            parent_lane->next_lane_ids().size() > 1) {
          cost_lc += std::abs(map->GetPriorityLaneRelation(lane));
        }
        parent_lane = lane;
      }
    }
    double cost = cost_navi + 15.0 * cost_merge + 10.0 * cost_split +
                  10.0 * cost_lc + 30.0 * cost_curvature + cost_route +
                  30.0 * cost_obs;

    std::ostringstream stream;
    stream << "Lanes:";
    for (const auto &lane : lane_sequence->lanes()) {
      stream << lane->id() << " , ";
    }
    uint64_t first_junc_id = first_junc_lane ? first_junc_lane->id() : 0;
    uint64_t next_junc_id = next_junc_lane ? next_junc_lane->id() : 0;
    stream << "..  Cost:" << cost << ".  Navi:" << cost_navi
           << ", Merge:" << 15.0 * cost_merge << ", Split:" << 10.0 * cost_split
           << ", Lc:" << 10.0 * cost_lc << ", Curve:" << 30.0 * cost_curvature
           << ", Route:" << cost_route;
    debug_strings.push_back({stream.str(), cost});
    // LOG_ERROR << stream.str();
    cost_static.emplace_back(lane_sequence, cost);
  }
  if (debug_strings.size() > 1) {
    std::sort(debug_strings.begin(), debug_strings.end(),
              [](const std::pair<std::string, double> &a,
                 const std::pair<std::string, double> &b) {
                return a.second < b.second;
              });
  }
  std::vector<std::string> ordered_debug_strings(debug_strings.size());
  for (size_t i = 0; i < debug_strings.size(); i++) {
    ordered_debug_strings.at(i) = std::move(debug_strings.at(i).first);
  }
  Log2DDS::LogDataV2("target-static-cost", ordered_debug_strings);
  return true;
}

double GetTurnDistance(const MapPtr &map, const LaneSequencePtr &laneseq,
                       const LaneConstPtr &turn_lane) {
  double dis = 0.0;
  if (!map->route() || !turn_lane) {
    return dis;
  }

  LaneConstPtr parent_lane =
      turn_lane->pre_lane_ids().empty()
          ? nullptr
          : map->GetLaneById(turn_lane->pre_lane_ids().front());
  LaneConstPtr child_lane =
      turn_lane->next_lane_ids().empty()
          ? nullptr
          : map->GetLaneById(turn_lane->next_lane_ids().front());
  if (!parent_lane || !child_lane) {
    // SectionInfo section;
    // map->route()->GetSectionByIdFromRoute(turn_lane->section_id(), section);
    SectionConstPtr section_ptr = map->GetSectionById(turn_lane->section_id());
    if (section_ptr) {
      const auto &it0 = std::find(section_ptr->lanes().begin(),
                                  section_ptr->lanes().end(), turn_lane->id());
      if (it0 != section_ptr->lanes().end()) {
        if (turn_lane->turn_type() == TurnType::LEFT_TURN ||
            turn_lane->turn_type() == TurnType::U_TURN) {
          dis = std::abs(std::distance(it0, section_ptr->lanes().begin()));
        } else if (turn_lane->turn_type() == TurnType::RIGHT_TURN) {
          dis = std::abs(std::distance(it0, section_ptr->lanes().end() - 1));
        }
      }
    }
    return dis;
  }

  double dis_btw_junction = std::numeric_limits<double>::max();
  TurnType first_turn_type = TurnType::NO_TURN;
  uint64_t section_id = 0;
  SerialJunctionType navi_info = map->route()->NextTwoJunctionOnNavi(
      dis_btw_junction, first_turn_type, section_id);
  if ((navi_info == SerialJunctionType::LEFT_RIGHT ||
       navi_info == SerialJunctionType::RIGHT_LEFT) &&
      dis_btw_junction < 400.0) {
    LaneConstPtr priority_lane =
        map->GetNaviPriorityLane(child_lane->section_id());
    // SectionInfo section_info;
    // map->route()->GetSectionByIdFromRoute(child_lane->section_id(),
    // section_info);
    SectionConstPtr section_ptr = map->GetSectionById(child_lane->section_id());
    std::vector<uint64_t> valid_lane;
    if (section_ptr) {
      for (const auto &lane_id : section_ptr->lanes()) {
        LaneConstPtr lane = map->GetLaneById(lane_id);
        if (lane && lane->type() != LANE_UNKNOWN &&
            lane->type() != LANE_NON_MOTOR && lane->type() != LANE_EMERGENCY) {
          valid_lane.emplace_back(lane_id);
        }
      }
    }
    if (valid_lane.size() > 2 && priority_lane) {
      size_t allow_lc_cnt = 1;
      if (dis_btw_junction < 200.0) {
        allow_lc_cnt = 1;
      } else if (dis_btw_junction < 300.0) {
        allow_lc_cnt = 2;
      } else if (dis_btw_junction < 400.0) {
        allow_lc_cnt = 3;
      }
      allow_lc_cnt = std::min(allow_lc_cnt, valid_lane.size() - 1);
      auto it1 =
          std::find(valid_lane.begin(), valid_lane.end(), child_lane->id());
      auto it_prio =
          std::find(valid_lane.begin(), valid_lane.end(), priority_lane->id());
      // left turn
      if (turn_lane->turn_type() == TurnType::LEFT_TURN) {
        auto it0 = it_prio - allow_lc_cnt;
        if (it0 < valid_lane.begin()) {
          it0 = valid_lane.begin();
        }
        dis = std::abs(static_cast<int>(std::distance(it0, it1)));
      }
      // right turn
      if (turn_lane->turn_type() == TurnType::RIGHT_TURN) {
        auto it0 = it_prio + allow_lc_cnt;
        if (it0 > (valid_lane.end() - 1)) {
          it0 = valid_lane.end() - 1;
        }
        dis = std::abs(static_cast<int>(std::distance(it0, it1)));
      }
      return -dis;
    }
  }

  if (turn_lane->turn_type() == TurnType::LEFT_TURN) {
    LaneConstPtr parent_left_lane = map->GetLeftLane(parent_lane);
    while (parent_left_lane) {
      for (const auto &lane : map->GetNextLanes(parent_left_lane)) {
        if (lane && lane->turn_type() == TurnType::LEFT_TURN) {
          dis += 1.0;
          break;
        }
      }
      parent_left_lane = map->GetLeftLane(parent_left_lane);
    }
    LaneConstPtr child_left_lane = map->GetLeftLane(child_lane);
    while (child_left_lane) {
      dis -= 1.0;
      child_left_lane = map->GetLeftLane(child_left_lane);
    }
    return dis;
  }
  if (turn_lane->turn_type() == TurnType::RIGHT_TURN) {
    LaneConstPtr parent_right_lane = map->GetRightLane(parent_lane);
    while (parent_right_lane) {
      if (parent_right_lane->type() == LaneType::LANE_NON_MOTOR ||
          parent_right_lane->type() == LaneType::LANE_EMERGENCY) {
        break;
      }
      for (const auto &lane : map->GetNextLanes(parent_right_lane)) {
        if (lane && lane->turn_type() == TurnType::RIGHT_TURN) {
          dis += 1.0;
          break;
        }
      }
      parent_right_lane = map->GetRightLane(parent_right_lane);
    }
    if (child_lane->type() == LaneType::LANE_ROUND_ABOUT && dis > 1.5) {
      dis = 0.0;
    }
    LaneConstPtr child_right_lane = map->GetRightLane(child_lane);
    while (child_right_lane) {
      if (child_right_lane->type() == LaneType::LANE_NON_MOTOR ||
          child_right_lane->type() == LaneType::LANE_EMERGENCY) {
        break;
      }
      if (child_right_lane->type() == LANE_BUS_NORMAL ||
          child_right_lane->type() == LANE_HARBOR_STOP ||
          child_right_lane->type() == LANE_BRT) {
        dis -= 1.0;
      } else {
        std::vector<LaneConstPtr> lanes;
        map->GetPrecedeLanes(child_right_lane, &lanes);
        for (const auto &lane : lanes) {
          if (lane && lane->turn_type() == TurnType::RIGHT_TURN) {
            dis -= 1.0;
            break;
          }
        }
      }
      child_right_lane = map->GetRightLane(child_right_lane);
    }
    return dis;
  }
  if (turn_lane->turn_type() == TurnType::U_TURN) {
    LaneConstPtr parent_left_lane = map->GetLeftLane(parent_lane);
    while (parent_left_lane) {
      for (const auto &lane : map->GetNextLanes(parent_left_lane)) {
        if (lane && lane->turn_type() == TurnType::U_TURN) {
          dis += 1.0;
          break;
        }
      }
      parent_left_lane = map->GetLeftLane(parent_left_lane);
    }
    LaneConstPtr child_left_lane = map->GetLeftLane(child_lane);
    while (child_left_lane) {
      dis -= 1.0;
      child_left_lane = map->GetLeftLane(child_left_lane);
    }
    return dis;
  }
  return dis;
}

double GetAverageKappa(const LaneConstPtr &lane) {
  if (!lane || !lane->center_line().IsValid()) {
    return 0.0;
  }
  Path path(lane->center_line().points());
  std::vector<PathPoint> points;
  path.SamplePoints(0.0, path.length(), 1.0, &points);
  double sum_kappa = 0.0;
  double max_kappa = 0.0;
  for (const auto &point : points) {
    sum_kappa += std::fabs(point.kappa);
    if (std::fabs(point.kappa) > max_kappa) {
      max_kappa = std::fabs(point.kappa);
    }
  }
  double avg_kappa = 0.0;
  if (!points.empty()) {
    avg_kappa = sum_kappa / points.size();
  }
  return avg_kappa;
}

double GetAverageKappaV2(const LaneConstPtr &lane) {
  if (!lane || !lane->center_line().IsValid()) {
    return 0.0;
  }
  Path path(lane->center_line().points());
  std::vector<PathPoint> points;
  path.SamplePoints(0.0, path.length(), 1.0, &points);
  double sum_kappa = 0.0;
  double num = 0;
  for (const auto &point : points) {
    if (std::fabs(point.kappa) < 0.01) continue;
    sum_kappa += std::fabs(point.kappa);
    num++;
  }
  double avg_kappa = 0.0;
  if (num != 0) {
    avg_kappa = sum_kappa / num;
  }
  return avg_kappa;
}

bool CalcuLaneIndex(const MapPtr &map, LaneConstPtr &pre_lane,
                    LaneConstPtr &mid_lane, LaneConstPtr &suc_lane,
                    int &pre_left_index, int &pre_right_index,
                    int &suc_left_index, int &suc_right_index) {
  if (!map || !pre_lane || !mid_lane || !suc_lane) {
    return false;
  }

  const std::vector<LaneType> black_list = {LANE_NON_MOTOR, LANE_UNKNOWN,
                                            LANE_EMERGENCY};
  // index from left in enter junction section
  LaneConstPtr pre_left_lane = map->GetLeftLane(pre_lane);
  while (pre_left_lane) {
    if (std::find(black_list.begin(), black_list.end(),
                  pre_left_lane->type()) != black_list.end()) {
      break;
    }
    if (false /*pre_left_lane->type() == LaneType::LANE_REVERSIBLE ||
        pre_left_lane->type() == LaneType::LANE_VARIABLE_TURN*/) {
      pre_left_index++;
    } else {
      for (const auto &child_lane : map->GetNextLanes(pre_left_lane)) {
        if (child_lane && child_lane->turn_type() == mid_lane->turn_type()) {
          pre_left_index++;
          break;
        }
      }
    }
    pre_left_lane = map->GetLeftLane(pre_left_lane);
  }
  // index from left in quit junction section
  auto get_same_split_branches =
      [map](const ad_byd::planning::LaneConstPtr &lane)
      -> std::unordered_set<uint64_t> {
    std::unordered_set<uint64_t> same_split_branches;
    for (const auto &pre_lane : map->GetPrecedeLanes(lane)) {
      for (const auto &branch : map->GetNextLanes(pre_lane)) {
        if (branch) same_split_branches.insert(branch->id());
      }
    }
    return same_split_branches;
  };

  auto same_split_branches = get_same_split_branches(suc_lane);
  LaneConstPtr suc_left_lane = map->GetLeftLane(suc_lane);
  while (suc_left_lane) {
    if (std::find(black_list.begin(), black_list.end(),
                  suc_left_lane->type()) != black_list.end()) {
      break;
    }
    // If this lane and last valid index lane aren't in the same split branches,
    // check and add index count.
    if (same_split_branches.count(suc_left_lane->id()) == 0) {
      for (const auto &precede_lane : map->GetPrecedeLanes(suc_left_lane)) {
        if (precede_lane &&
            precede_lane->turn_type() == mid_lane->turn_type()) {
          suc_left_index++;
          // If this lane is valid index lane, update the same_split_branches.
          same_split_branches = get_same_split_branches(suc_left_lane);
          break;
        }
      }
    }
    // continue to check next neighbor lane.
    suc_left_lane = map->GetLeftLane(suc_left_lane);
  }
  // index from right in enter junction section
  LaneConstPtr pre_right_lane = map->GetRightLane(pre_lane);
  while (pre_right_lane) {
    if (std::find(black_list.begin(), black_list.end(),
                  pre_right_lane->type()) != black_list.end()) {
      break;
    }
    if (false /*pre_right_lane->type() == LaneType::LANE_REVERSIBLE ||
        pre_right_lane->type() == LaneType::LANE_VARIABLE_TURN*/) {
      pre_right_index++;
    } else {
      for (const auto &child_lane : map->GetNextLanes(pre_right_lane)) {
        if (child_lane && child_lane->turn_type() == mid_lane->turn_type()) {
          pre_right_index++;
          break;
        }
      }
    }
    pre_right_lane = map->GetRightLane(pre_right_lane);
  }
  // index from right in quit junction section
  same_split_branches = get_same_split_branches(suc_lane);
  LaneConstPtr suc_right_lane = map->GetRightLane(suc_lane);
  while (suc_right_lane) {
    if (std::find(black_list.begin(), black_list.end(),
                  suc_right_lane->type()) != black_list.end()) {
      break;
    }
    // If this lane and last valid index lane aren't in the same split branches,
    // add index.
    if (same_split_branches.count(suc_right_lane->id()) == 0) {
      for (const auto &precede_lane : map->GetPrecedeLanes(suc_right_lane)) {
        if (precede_lane &&
            precede_lane->turn_type() == mid_lane->turn_type()) {
          suc_right_index++;
          // If this lane is valid index lane, update the same_split_branches.
          same_split_branches = get_same_split_branches(suc_right_lane);
          break;
        }
      }
    }
    suc_right_lane = map->GetRightLane(suc_right_lane);
  }

  return true;
}

uint64_t CalcuSplitLaneId(const Vec2d &start_point,
                          const LaneSequencePtr &lane_seq, double &accum_s) {
  uint64_t split_lane_id = 0;
  bool has_split = false;
  int find_start_lane = 0;

  if (lane_seq == nullptr) {
    return split_lane_id;
  }
  auto lanes = lane_seq->lanes();
  // double accum_s = 0.0;
  SLPoint sl_point(0.0, 0.0);

  for (const auto &lane : lanes) {
    if (!lane || !lane->IsValid()) {
      break;
    }
    if (find_start_lane == 0 &&
    lane->id() == lanes.front()->id() && 
    lane->GetSLWithoutLimit(start_point, &sl_point)) {
      find_start_lane++;
      accum_s = lane->center_line().length() - sl_point.s;
    }
    if (find_start_lane != 0) {
      if (has_split) {
        if (0 != lane->junction_id()) {
          return split_lane_id;
        }
        split_lane_id = lane->id();
        break;
      }
      if (lane->next_lane_ids().size() >= 2) {
        has_split = true;
      }
      if (find_start_lane == 1) {
        find_start_lane++;
      } else {
        accum_s = accum_s + lane->center_line().length();
      }
    }
  }
  return split_lane_id;
}

SplitDirection CalcuSplitDirection(const MapPtr &map, uint64_t split_lane_id,
                                   uint64_t extend_split_lane_id) {
  // std::string direction;
  SplitDirection direction = SplitDirection::NotSpliting;
  if (split_lane_id == 0 || extend_split_lane_id == 0) {
    // direction.clear();
    return direction;
  }
  auto split_lane = map->GetLaneById(split_lane_id);
  auto left_lane = split_lane;
  auto right_lane = split_lane;
  while (direction == SplitDirection::NotSpliting && left_lane) {
    if (left_lane->id() == extend_split_lane_id) {
      // direction = "left";
      direction = SplitDirection::Left;
      continue;
    }
    left_lane = map->GetLeftLane(left_lane);
  }
  while (direction == SplitDirection::NotSpliting && right_lane) {
    if (right_lane->id() == extend_split_lane_id) {
      // direction = "right";
      direction = SplitDirection::Right;
      continue;
    }
    right_lane = map->GetRightLane(right_lane);
  }

  return direction;
}

SplitDirection CalcuSplitDirectionByNavigation(const MapPtr &map,
                                               LaneConstPtr &left_lane,
                                               LaneConstPtr &right_lane) {
  if (!left_lane || !left_lane->center_line().IsValid() || !right_lane ||
      !right_lane->center_line().IsValid()) {
    return SplitDirection::Mid;
  }

  int left_lane_lc_num = map->GetPriorityLaneRelation(left_lane);
  int right_lane_lc_num = map->GetPriorityLaneRelation(right_lane);

  if (std::abs(left_lane_lc_num) > std::abs(right_lane_lc_num)) {
    return SplitDirection::Left;
  } else if (std::abs(left_lane_lc_num) < std::abs(right_lane_lc_num)) {
    return SplitDirection::Right;
  }

  if (left_lane->navi_distance() < right_lane->navi_distance()) {
    return SplitDirection::Left;
  } else if (left_lane->navi_distance() > right_lane->navi_distance()) {
    return SplitDirection::Right;
  }

  return SplitDirection::NotSpliting;
}

LaneSequencePtr CalcuExtendSplitLaneSequence(
    const Vec2d &start_point, const double start_theta, const MapPtr &map,
    const LaneConstPtr &lane, double ego_v, const bool &navi_is_priority,
    const bool &split_is_navi_priority, uint64_t &split_lane_id,
    uint64_t &extend_split_lane_id,
    const ad_byd::planning::LaneSequencePtr current_seq,
    const ad_byd::planning::LaneSequencePtr pre_lane_seq) {
  static uint64_t last_split_id = 0;
  double distance_limit = std::numeric_limits<double>::infinity();
  const double kPreSplitTime = 10.0;
  const double kMinSplitCheckDis = 100;
  double dis_to_split = 0.0;
  static double last_distance = std::numeric_limits<double>::infinity();
  LaneSequencePtr extend_split_seq = nullptr;

  split_lane_id = CalcuSplitLaneId(start_point, current_seq, dis_to_split);

  if (0 != split_lane_id) {
    if (last_split_id != split_lane_id)
      distance_limit = std::max(ego_v * kPreSplitTime, kMinSplitCheckDis);
    else
      distance_limit = last_distance;
    if (dis_to_split < distance_limit) {
      extend_split_seq = GetOptimalLaneSeq(
          start_point, start_theta, map, lane, ego_v, /*navi_is_priority=*/true,
          /*split_is_navi_priority*/ true, split_lane_id, pre_lane_seq);
      extend_split_lane_id =
          CalcuSplitLaneId(start_point, extend_split_seq, dis_to_split);
      auto extend_split_lane = map->GetLaneById(extend_split_lane_id);
      if (!extend_split_lane || !extend_split_lane->is_navigation() ||
          extend_split_lane->lane_info().type == LaneType::LANE_EMERGENCY ||
          extend_split_lane->lane_info().type == LaneType::LANE_BUS_NORMAL ||
          extend_split_lane->lane_info().type == LaneType::LANE_NON_MOTOR ||
          extend_split_lane->lane_info().type == LaneType::LANE_HARBOR_STOP ||
          extend_split_lane->lane_info().turn_type == TurnType::U_TURN) {
        extend_split_seq = nullptr;
      } else {
        current_seq->SetSplitLaneId(split_lane_id);
        current_seq->SetSplitLcNum(std::abs(
            map->GetPriorityLaneRelation(map->GetLaneById(split_lane_id))));
        extend_split_seq->SetSplitLaneId(extend_split_lane_id);
        extend_split_seq->SetSplitLcNum(
            std::abs(map->GetPriorityLaneRelation(extend_split_lane)));
        if (current_seq->GetSplitLcNum() > extend_split_seq->GetSplitLcNum()) {
          current_seq->SetSplitLcNumIsMax(true);
        } else {
          extend_split_seq->SetSplitLcNumIsMax(true);
        }
      }
      last_split_id = split_lane_id;
      last_distance = distance_limit;
      return extend_split_seq;
    }
  }

  return nullptr;
}

double CalcuDistToNaviEnd(const MapPtr &map, const LaneSequencePtr &laneseq,
                          const Vec2d start_point) {
  double dist_to_navi_end = 0.0;
  if (!laneseq) {
    return dist_to_navi_end;
  }
  LaneConstPtr navi_end_lane;
  double s_offset = 0.0;
  const double l = laneseq->GetProjectionDistance(
      {start_point.x(), start_point.y()}, &s_offset);
  const auto projection_point = laneseq->GetPointAtS(s_offset);
  dist_to_navi_end = laneseq->GetDistanceToPOI(
      Poi_NaviEnd, projection_point.x(), projection_point.y(), navi_end_lane,
      &map->route()->navi_start());
  return dist_to_navi_end;
}

BehaviorCommand CalcuAheadDirection(
    const MapPtr &map, const Vec2d start_point, const double ego_v,
    const ad_byd::planning::LaneSequencePtr current_seq,
    const ad_byd::planning::LaneSequencePtr left_seq,
    const ad_byd::planning::LaneSequencePtr right_seq) {
  if (!current_seq) {
    return BehaviorCommand::Command_Invalid;
  }
  BehaviorCommand ahead_dir = BehaviorCommand::Command_Invalid;
  double distance_limit = std::numeric_limits<double>::infinity();
  const double kPreTime = 10.0;
  const double kMinDis = 100.0;
  const double KReleaseDis = 400.0;
  const double kDiffDis = 50.0;
  static double last_distance = std::numeric_limits<double>::infinity();
  static bool in_range = false;

  double current_dist_to_navi_end =
      CalcuDistToNaviEnd(map, current_seq, start_point);
  double left_dist_to_navi_end = CalcuDistToNaviEnd(map, left_seq, start_point);
  double right_dist_to_navi_end =
      CalcuDistToNaviEnd(map, right_seq, start_point);

  if (in_range) {
    if (current_dist_to_navi_end > KReleaseDis) {
      distance_limit = std::max(ego_v * kPreTime, kMinDis);
      in_range = false;
    } else {
      distance_limit = last_distance;
    }
  } else {
    distance_limit = std::max(ego_v * kPreTime, kMinDis);
  }

  if (current_dist_to_navi_end < distance_limit &&
      (std::max(left_dist_to_navi_end, right_dist_to_navi_end) -
           current_dist_to_navi_end >
       kDiffDis)) {
    if (left_dist_to_navi_end > right_dist_to_navi_end) {
      ahead_dir = BehaviorCommand::Command_LaneChangeLeft;
    } else if (right_dist_to_navi_end > left_dist_to_navi_end) {
      ahead_dir = BehaviorCommand::Command_LaneChangeRight;
    }
    in_range = true;
    last_distance = distance_limit;
  }
  Log2DDS::LogDataV2("ahead_dir", ahead_dir);
  return ahead_dir;
}

std::set<UturnFilterDirection> UturnFilter(const MapPtr &map,
                                           const LaneConstPtr &cur_lane) {
  std::set<UturnFilterDirection> result;
  if (!cur_lane) {
    return result;
  }
  std::map<LaneConstPtr, UturnFilterDirection> has_exp_uturn;
  std::map<LaneConstPtr, UturnFilterDirection> no_exp_uturn;
  // get left and right lane
  auto left_lane = map->GetLaneById(cur_lane->left_lane_id());
  auto right_lane = map->GetLaneById(cur_lane->right_lane_id());

  if (cur_lane->lane_info().turn_type == TurnType::U_TURN) {
    if (cur_lane->is_exp_traj()) {
      has_exp_uturn.insert({cur_lane, UturnFilterDirection::current});
    } else {
      no_exp_uturn.insert({cur_lane, UturnFilterDirection::current});
    }
  }

  if (left_lane && left_lane->lane_info().turn_type == TurnType::U_TURN) {
    if (left_lane->is_exp_traj()) {
      has_exp_uturn.insert({left_lane, UturnFilterDirection::left});
    } else {
      no_exp_uturn.insert({left_lane, UturnFilterDirection::left});
    }
  }

  if (right_lane && right_lane->lane_info().turn_type == TurnType::U_TURN) {
    if (right_lane->is_exp_traj()) {
      has_exp_uturn.insert({right_lane, UturnFilterDirection::right});
    } else {
      no_exp_uturn.insert({right_lane, UturnFilterDirection::right});
    }
  }

  for (const auto &[no_exp_lane, value] : no_exp_uturn) {
    for (const auto &[exp_lane, value1] : has_exp_uturn) {
      if (!no_exp_lane->pre_lane_ids().empty() &&
          !exp_lane->pre_lane_ids().empty() &&
          (no_exp_lane->pre_lane_ids().front() ==
           exp_lane->pre_lane_ids().front())) {
        result.insert(value);
        break;
      }
    }
  }

  return result;
}

bool IsNonMotorLane(const MapPtr &map, const LaneConstPtr &lane) {
  if (!lane) {
    return false;
  }
  if (lane->lane_info().type == LaneType::LANE_NON_MOTOR) {
    return true;
  }
  bool all_pre_lanes_is_non_motor = true;
  bool all_next_lanes_is_non_motor = true;

  if (lane->pre_lane_ids().empty()) {
    all_pre_lanes_is_non_motor = false;
  } else {
    for (auto pre_id : lane->pre_lane_ids()) {
      auto pre_lane = map->GetLaneById(pre_id);
      if (pre_lane && pre_lane->lane_info().type != LaneType::LANE_NON_MOTOR) {
        all_pre_lanes_is_non_motor = false;
      }
    }
  }
  if (lane->next_lane_ids().empty()) {
    all_next_lanes_is_non_motor = false;
  } else {
    for (auto next_id : lane->next_lane_ids()) {
      auto next_lane = map->GetLaneById(next_id);
      if (next_lane &&
          next_lane->lane_info().type != LaneType::LANE_NON_MOTOR) {
        all_next_lanes_is_non_motor = false;
      }
    }
  }
  if (all_pre_lanes_is_non_motor && all_next_lanes_is_non_motor) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace ad_byd
