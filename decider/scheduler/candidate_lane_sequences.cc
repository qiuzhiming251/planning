
#include "decider/scheduler/candidate_lane_sequences.h"

#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "plan_common/log_data.h"
#include "plan_common/planning_macros.h"

namespace ad_byd {
namespace planning {
enum class lc_dir { none = 0, left = 1, right = 2 };

size_t GetNormalLanePosition(const MapPtr& map, const uint64_t lane_id,
                             const SectionConstPtr& section_ptr) {
  if (!map || !section_ptr) {
    return 0;
  }
  size_t lane_position = 0;
  for (const auto& section_lane_id : section_ptr->lanes()) {
    const auto& lane = map->GetLaneById(section_lane_id);
    if (!lane) {
      continue;
    }
    if (lane &&
        (lane->type() == LANE_UNKNOWN || lane->type() == LANE_EMERGENCY ||
         lane->type() == LANE_NON_MOTOR)) {
      continue;
    }
    lane_position++;
    if (lane_id == section_lane_id) {
      return lane_position;
    }
  }
  return 0;
}

size_t GetNormalLaneSize(const MapPtr& map,
                         const SectionConstPtr& section_ptr) {
  if (!map || !section_ptr) {
    return 0;
  }
  size_t lane_size = 0;
  for (const auto& section_lane_id : section_ptr->lanes()) {
    const auto& lane = map->GetLaneById(section_lane_id);
    if (!lane) {
      continue;
    }
    if (lane &&
        (lane->type() == LANE_UNKNOWN || lane->type() == LANE_EMERGENCY ||
         lane->type() == LANE_NON_MOTOR)) {
      continue;
    }
    lane_size++;
  }
  return lane_size;
}

bool IfContinuousLcScenario(const ad_byd::planning::LaneSeqInfo* lane_seq_info,
                            st::Behavior_FunctionId func_id,
                            const st::LaneChangeStateProto lc_state_proto,
                            LaneSequencePtr top1_seq, Vec2d ego_pos) {
  if (!lane_seq_info || !top1_seq) {
    Log2DDS::LogDataV2("continue_lc", "no input");
    return false;
  }
  lc_dir top1_dir = lc_dir::none;
  auto nearest_top1_lane = top1_seq->GetNearestLane(ego_pos, nullptr);
  double project_l = std::numeric_limits<double>::max();
  double project_s = std::numeric_limits<double>::max();
  nearest_top1_lane->center_line().GetProjection(ego_pos, &project_s,
                                                 &project_l);
  Log2DDS::LogDataV2("continue_lc", absl::StrCat("project l: ", project_l));
  Log2DDS::LogDataV2("continue_lc",
                     absl::StrCat("pre stage: ", lc_state_proto.stage()));
  if (lc_state_proto.stage() == st::LaneChangeStage::LCS_NONE &&
      std::fabs(project_l) < 1.5) {
    top1_dir = lc_dir::none;
  } else if (lc_state_proto.stage() != st::LaneChangeStage::LCS_NONE &&
             project_l < 0.0) {
    Log2DDS::LogDataV2("continue_lc", "top1_dir: left ");
    top1_dir = lc_dir::left;
  } else if (lc_state_proto.stage() != st::LaneChangeStage::LCS_NONE &&
             project_l > 0.0) {
    Log2DDS::LogDataV2("continue_lc", "top1_dir: right ");
    top1_dir = lc_dir::right;
  }
  lc_dir last_lc_dir = lc_dir::none;
  if (lc_state_proto.stage() != st::LaneChangeStage::LCS_NONE &&
      lc_state_proto.lc_left()) {
    last_lc_dir = lc_dir::left;
    Log2DDS::LogDataV2("continue_lc", "last_lc_dir: left ");
  } else if (lc_state_proto.stage() != st::LaneChangeStage::LCS_NONE &&
             !lc_state_proto.lc_left()) {
    Log2DDS::LogDataV2("continue_lc", "last_lc_dir: right ");
    last_lc_dir = lc_dir::right;
  }
  bool same_dir_last_lc = (last_lc_dir == top1_dir);  //&& (v2_dir == top1_dir)
  if (lc_state_proto.stage() != st::LaneChangeStage::LCS_NONE &&
      !same_dir_last_lc) {
    Log2DDS::LogDataV2("continue_lc", "dir jump");
    return false;
  }

  double min_distance_to_end_or_junction = lane_seq_info->dist_to_navi_end;
  if (lane_seq_info->dist_to_junction < 35.0) {
    Log2DDS::LogDataV2("continue_lc", "near junction");
    return false;
  }
  if (lane_seq_info->lc_num == 0) {
    Log2DDS::LogDataV2("continue_lc", "lc num 0 ");
    return false;
  }

  if (min_distance_to_end_or_junction >
      std::max(120.0, lane_seq_info->lc_num * 80.0)) {
    Log2DDS::LogDataV2("continue_lc", "far to v2 navi end");
    return false;
  }

  Log2DDS::LogDataV2("continue_lc", "ContinuLcScenario!");
  return true;
}

bool BackwardExtendLane(const Vec2d& ego_pos, const MapConstPtr& map,
                        std::vector<std::vector<LaneConstPtr>>* all_lane_seqs) {
  for (auto& laneseq : *all_lane_seqs) {
    if (laneseq.size() < 1) continue;
    double extend_length = 0.0;
    LaneConstPtr front_lane = laneseq.front();
    double accumulate_s = 0.0;
    LaneSequence ls(laneseq);
    if (front_lane && front_lane->IsValid() &&
        ls.GetProjectionDistance(ego_pos, &accumulate_s)) {
      extend_length += accumulate_s;
    }
    while (extend_length < 30.0 && front_lane && front_lane->IsValid()) {
      if (front_lane->pre_lane_ids().empty()) {
        break;
      }
      double best_cost = std::numeric_limits<double>::max();
      LaneConstPtr best_prelane = nullptr;
      for (const auto& pre_id : front_lane->pre_lane_ids()) {
        LaneConstPtr pre_lane = map->GetLaneById(pre_id);
        if (pre_lane && pre_lane->IsValid()) {
          if (pre_lane->is_navigation()) {
            best_prelane = pre_lane;
            break;
          }
          math::Vec2d ref_vec = pre_lane->center_line().end_point() -
                                pre_lane->center_line().GetPointAtS(std::fmax(
                                    pre_lane->curve_length() - 20.0, 0.0));
          math::Vec2d split_vec;
          double length = std::fmin(front_lane->curve_length(), 10.0);
          split_vec = front_lane->center_line().GetPointAtS(length) -
                      front_lane->center_line().begin_point();
          split_vec = math::Vec2d::CreateUnitVec2d(split_vec.Angle()) * 10.0;
          double distance =
              std ::fabs(ref_vec.CrossProd(split_vec) /
                         std::fmax(ref_vec.Length(), Constants::ZERO));
          if (distance < best_cost) {
            best_cost = distance;
            best_prelane = pre_lane;
          }
        }
      }
      if (!best_prelane) break;
      laneseq.insert(laneseq.begin(), best_prelane);
      extend_length += best_prelane->topo_length();
      front_lane = laneseq.front();
    }
  }
  return true;
}

void ForwardExtendLane(const MapConstPtr& map, const LaneConstPtr& start_lane,
                       std::vector<LaneConstPtr>* laneseq) {
  if (!map || !start_lane || !laneseq) return;

  std::unordered_set<uint64_t> circle_check;
  for (int i = 0; i < (*laneseq).size(); i++) {
    const auto& lane = (*laneseq)[i];
    if (!lane) continue;
    circle_check.insert(lane->id());
    if (!(lane->section_id() == 0)) {
      circle_check.insert(lane->section_id());
    }
    if (lane->id() == start_lane->id()) {
      (*laneseq).erase((*laneseq).begin() + i + 1, (*laneseq).end());
      break;
    }
  }

  while ((*laneseq).back()) {
    std::string debug;
    LaneConstPtr next_opt_lane =
        map->GetOptimalSmoothNextLane((*laneseq).back(), true, debug);
    if (!next_opt_lane || circle_check.count(next_opt_lane->id()) != 0 ||
        circle_check.count(next_opt_lane->section_id()) != 0)
      break;
    Log2DDS::LogDataV2("lc_debug", "forward ext:" + debug);
    (*laneseq).emplace_back(next_opt_lane);
    circle_check.insert(next_opt_lane->id());
    if (!(next_opt_lane->section_id() == 0)) {
      circle_check.insert(next_opt_lane->section_id());
    }
  }
  return;
}

bool SelectSmoothCandidateAndMatch(
    const MapConstPtr& map, const std::optional<st::PNPInfos>& pnp_infos,
    const std::vector<std::vector<LaneConstPtr>>& all_lane_seqs,
    const int kTopN,
    std::vector<std::vector<LaneConstPtr>>* candidate_lane_seqs,
    std::vector<int>* candidate_lane_seqs_probabilty, bool* top1_match_failed,
    const st::Behavior_FunctionId& map_func_id) {
  if (!map || !pnp_infos) return false;
  bool has_matched_top1 = false;
  bool top1_match_fail = false;
  for (const auto& pnp_info : pnp_infos->infos()) {
    if (candidate_lane_seqs->size() >= kTopN) break;
    std::vector<uint64_t> pnp_lane_ids(
        pnp_info.target_lane_sequence_ids().begin(),
        pnp_info.target_lane_sequence_ids().end());
    std::vector<uint64_t> pnp_lane_ids_no_junction;
    std::string str = "";
    // remove junction lane in top 1 seq
    for (const auto& lane_id : pnp_lane_ids) {
      auto temp_pnp_lane = map->GetLaneById(lane_id);
      // if (temp_pnp_lane && temp_pnp_lane->junction_id().empty()) {
      if (temp_pnp_lane) {
        str += absl::StrCat(lane_id, ",");
        pnp_lane_ids_no_junction.emplace_back(lane_id);
      } else {
        break;
      }
    }
    // if (map_func_id ==
    //       st::Behavior_FunctionId::Behavior_FunctionId_CITY_NOA) {
    //   pnp_lane_ids_no_junction.clear();
    //   pnp_lane_ids_no_junction.swap(pnp_lane_ids);
    // }
    pnp_lane_ids_no_junction.clear();
    pnp_lane_ids_no_junction.swap(pnp_lane_ids);
    Log2DDS::LogDataV0("lc_debug", "pnp lane seq: " + str);
    std::vector<LaneConstPtr> match_lane_seq;
    if (GetMatchLanePathByIds(map, pnp_lane_ids_no_junction, all_lane_seqs,
                              &match_lane_seq)) {
      LaneConstPtr start_lane =
          map->GetLaneById(pnp_lane_ids_no_junction.back());
      ForwardExtendLane(map, start_lane, &match_lane_seq);
      std::string str = "";
      for (const auto& lane_id : match_lane_seq) {
        str += lane_id->id() + ",";
      }
      Log2DDS::LogDataV0("lc_debug",
                         absl::StrCat("match lane seq: ", str,
                                      " probabilty: ", pnp_info.probabilty()));
      candidate_lane_seqs->emplace_back(match_lane_seq);
      candidate_lane_seqs_probabilty->emplace_back(pnp_info.probabilty());
    }
    if (!has_matched_top1) {
      has_matched_top1 = true;
      top1_match_fail = candidate_lane_seqs->empty();
    }
  }
  if (top1_match_failed) {
    *top1_match_failed = top1_match_fail;
  }
  return true;
}

bool ScheduleCandidateLaneSequences(
    const Vec2d& ego_pos, const double ego_v, const MapPtr& map,
    const LaneConstPtr& start_lane, const LaneConstPtr& nearest_lane,
    const std::optional<st::PNPInfos>& pnp_infos,
    const st::DriverAction::LaneChangeCommand mlc_cmd,
    st::mapping::LanePath* preferred_lane_path,
    std::vector<std::vector<LaneConstPtr>>* candidate_lane_seqs,
    std::vector<int>* candidate_lane_seqs_proprobabilty,
    const st::LaneChangeStage& lane_change_stage,
    const ad_byd::planning::LaneSequencePtr& pre_target_lane_seq,
    const ad_byd::planning::LaneSequencePtr& scene_target_lane_seq,
    const bool is_navi, const bool is_lka,
    const st::Behavior_FunctionId& map_func_id, const double ego_heading,
    const double ego_l_offset, bool& if_continue_lc,
    const st::LaneChangeStateProto lc_state_proto) {
  const int kTopN = 2;

  // 0. get lane ids in current section
  std::vector<uint64_t> navi_start_lanes;
  auto& navi_start = map->route()->navi_start();
  if (!(navi_start.section_id == 0)) {
    const auto& navi_section = map->GetSectionById(navi_start.section_id);
    if (navi_section) {
      for (auto& id : navi_section->lanes()) {
        navi_start_lanes.push_back(id);
      }
    }
  } else if (!is_navi) {
    navi_start_lanes.push_back(nearest_lane->id());
  }

  // 1. Get all lane_squences
  std::vector<std::vector<LaneConstPtr>> all_lane_seqs;
  for (int i = 0; i < navi_start_lanes.size(); i++) {
    const auto& lane = map->GetLaneById(navi_start_lanes[i]);
    if (lane && lane->IsValid() && (lane->is_navigation() || !is_navi)) {
      std::vector<std::vector<LaneConstPtr>> lane_seqs;
      map->GetAllLaneSequences(lane, lane_seqs);
      all_lane_seqs.insert(all_lane_seqs.end(), lane_seqs.begin(),
                           lane_seqs.end());
    }
  }
  if (all_lane_seqs.empty()) {
    return false;
  }

  // map->GetAllLaneSequences(nearest_lane, all_lane_seqs);
  // const auto& left_lane = map->GetLaneById(nearest_lane->left_lane_id());
  // if (left_lane && left_lane->is_navigation() && left_lane->IsValid()) {
  //   std::vector<std::vector<LaneConstPtr>> left_lane_seqs;
  //   map->GetAllLaneSequences(left_lane, left_lane_seqs);
  //   all_lane_seqs.insert(all_lane_seqs.end(), left_lane_seqs.begin(),
  //                        left_lane_seqs.end());
  // }
  // const auto& right_lane = map->GetLaneById(nearest_lane->right_lane_id());
  // if (right_lane && right_lane->is_navigation() && right_lane->IsValid()) {
  //   std::vector<std::vector<LaneConstPtr>> right_lane_seqs;
  //   map->GetAllLaneSequences(right_lane, right_lane_seqs);
  //   all_lane_seqs.insert(all_lane_seqs.end(), right_lane_seqs.begin(),
  //                        right_lane_seqs.end());
  // }
  // if (all_lane_seqs.empty()) {
  //   return false;
  // }

  // 2. Get match lane_squences from pnp infos
  bool top1_match_failed = false;
  SelectSmoothCandidateAndMatch(
      map, pnp_infos, all_lane_seqs, kTopN, candidate_lane_seqs,
      candidate_lane_seqs_proprobabilty, &top1_match_failed, map_func_id);
  Log2DDS::LogDataV2("test", absl::StrCat("ego_l_offset: ", ego_l_offset));
  Log2DDS::LogDataV2("test", absl::StrCat("ego_heading: ", ego_heading));
  SLPoint sl_point(0.0, 0.0);
  bool get_sl = nearest_lane->GetSLWithoutLimit(ego_pos, &sl_point);
  double lane_heading = 0.0;
  bool get_heading = nearest_lane->GetHeadingFromS(sl_point.s, &lane_heading);
  double heading_diff = std::abs(math::AngleDiff(lane_heading, ego_heading));
  bool can_use_new_top1 = false;
  Log2DDS::LogDataV2("test", absl::StrCat("lane_heading: ", lane_heading));
  Log2DDS::LogDataV2("test", absl::StrCat("heading_diff: ", heading_diff));

  // get new top 1 seq
  LaneSequencePtr new_top1_seq = nullptr;
  ad_byd::planning::LaneSeqInfo current_seq_info;
  bool first_check_flag = true;
  for (const auto& candidate_lane_seq : *candidate_lane_seqs) {
    // protect in case null lane
    std::vector<st::mapping::ElementId> lane_id_infos;
    for (const auto& lane : candidate_lane_seq) {
      if (!lane || lane->id() == 0 || !lane->center_line().IsValid() ||
          std::count(lane_id_infos.begin(), lane_id_infos.end(), lane->id())) {
        if (lane) {
          LOG_ERROR << "lane id = " << lane->id()
                    << " valid = " << lane->center_line().IsValid();
        } else {
          LOG_ERROR << "lane is null, fatal error";
        }
        break;
      }
      lane_id_infos.emplace_back(lane->id());
    }
    if (lane_id_infos.empty()) {
      continue;
    }
    if (first_check_flag) {
      first_check_flag = false;
      new_top1_seq =
          std::make_shared<ad_byd::planning::LaneSequence>(candidate_lane_seq);
    }
    // LaneConstPtr temp_nearest_lane = nullptr;
    // ad_byd::planning::LaneSequencePtr temp_lane_seq =
    //   std::make_shared<ad_byd::planning::LaneSequence>(candidate_lane_seq);
    // if(temp_lane_seq){
    //   temp_nearest_lane = temp_lane_seq->GetNearestLane(ego_pos, nullptr);
    // }
    // if(temp_nearest_lane->id() == nearest_lane->id()){
    //     Log2DDS::LogDataV2("continue_lc", "get lane seq info");
    //     bool result = GetLaneSeqInfo(nearest_lane, ego_pos, temp_lane_seq,
    //     map, pnp_infos, &current_seq_info);
    //     if(result){
    //       Log2DDS::LogDataV2("continue_lc", "result ok");
    //     }else{
    //       Log2DDS::LogDataV2("continue_lc", "not ok");
    //     }
    // }
  }

  const auto& cur_lane_seq = map->GetLaneSequence(start_lane, true);
  if (cur_lane_seq) {
    Log2DDS::LogDataV2("continue_lc", "has cur");
    bool result = GetLaneSeqInfo(nearest_lane, ego_pos, ego_v, cur_lane_seq,
                                 map, &current_seq_info);
  }
  LaneConstPtr nearest_new_top1_lane = nullptr;
  if (new_top1_seq) {
    nearest_new_top1_lane = new_top1_seq->GetNearestLane(ego_pos, nullptr);
  }

  double project_l_new = std::numeric_limits<double>::max();
  double project_s_new = std::numeric_limits<double>::max();
  bool new_top1_is_nearest = false;
  if (nearest_new_top1_lane) {
    nearest_new_top1_lane->center_line().GetProjection(ego_pos, &project_s_new,
                                                       &project_l_new);
    new_top1_is_nearest = nearest_new_top1_lane->id() == nearest_lane->id();
  }
  double project_l_pre = std::numeric_limits<double>::max();
  double project_s_pre = std::numeric_limits<double>::max();
  auto nearest_pre_lane = pre_target_lane_seq->GetNearestLane(ego_pos, nullptr);
  if (nearest_pre_lane) {
    nearest_pre_lane->center_line().GetProjection(ego_pos, &project_s_pre,
                                                  &project_l_pre);
  }
  Log2DDS::LogDataV0("lc_pnp_prevent_jump",
                     absl::StrCat("project_l_pre: ", project_l_pre));
  Log2DDS::LogDataV0("lc_pnp_prevent_jump",
                     absl::StrCat("project_l_new: ", project_l_new));
  bool if_jump_dir = true;
  if (project_l_pre * project_l_new > 0.0) {
    if_jump_dir = false;
  }
  Log2DDS::LogDataV0("lc_pnp_prevent_jump",
                     absl::StrCat("if_jump_dir: ", if_jump_dir));
  if_continue_lc = IfContinuousLcScenario(
      &current_seq_info, map_func_id, lc_state_proto, new_top1_seq, ego_pos);
  Log2DDS::LogDataV0("lc_pnp_prevent_jump",
                     absl::StrCat("if_continue: ", if_continue_lc));

  if (get_sl && get_heading) {
    can_use_new_top1 =
        std::abs(heading_diff) < 0.04 && std::abs(ego_l_offset) < 0.25 &&
        map_func_id ==
            st::Behavior_FunctionId::Behavior_FunctionId_MAPLESS_NOA &&
        new_top1_is_nearest;
  }
  Log2DDS::LogDataV0("lc_pnp_prevent_jump",
                     absl::StrCat("can_use_new_top1: ", can_use_new_top1));
  bool is_lanechange =
      (lane_change_stage == st::LaneChangeStage::LCS_EXECUTING ||
       lane_change_stage == st::LaneChangeStage::LCS_RETURN ||
       lane_change_stage == st::LaneChangeStage::LCS_PAUSE);
  if (is_lanechange && !if_continue_lc && !can_use_new_top1) {
    std::vector<uint64_t> lane_ids;
    std::string raw_str = "";
    for (const auto& lane : pre_target_lane_seq->lanes()) {
      lane_ids.push_back(lane->id());
      raw_str += absl::StrCat(lane->id(), ",");
    }
    Log2DDS::LogDataV0("lc_pnp_prevent_jump", "raw pre lane ids: " + raw_str);
    // std::vector<LaneConstPtr> match_lane_seq;
    candidate_lane_seqs->clear();
    candidate_lane_seqs->emplace_back(pre_target_lane_seq->lanes());
    // if (GetMatchLanePathByIds(map, lane_ids, all_lane_seqs, &match_lane_seq))
    // {
    //   candidate_lane_seqs->clear();
    //   std::string str = "";
    //   for (const auto& lane_id : match_lane_seq) {
    //     str += lane_id->id() + ",";
    //   }
    //   Log2DDS::LogDataV0("lc_debug", "match pre lane seq: " + str);
    //   candidate_lane_seqs->emplace_back(match_lane_seq);

    //   Log2DDS::LogDataV0("lc_pnp_prevent_jump", "replace with pre: " + str);
    // } else {
    //   Log2DDS::LogDataV0("lc_pnp_prevent_jump", "replace match fails.");
    // }
  } else if (top1_match_failed) {
    candidate_lane_seqs->insert((candidate_lane_seqs->begin()),
                                pre_target_lane_seq->lanes());
  }

  // 3. Check if match_lanes contain ego_lane.

  bool has_contain_ego_lane = false;
  for (const auto& match_lane_seq : *candidate_lane_seqs) {
    if (std::find_if(match_lane_seq.begin(), match_lane_seq.end(),
                     [&](LaneConstPtr lane) {
                       return lane && lane->id() == nearest_lane->id();
                     }) != match_lane_seq.end()) {
      has_contain_ego_lane = true;
      break;
    }
  }

  // TODO: Get smooth lane_seq
  std::vector<LaneConstPtr> ego_lane_seq;
  const auto& ego_lane_sequence = map->GetLaneSequence(start_lane, true);
  if (ego_lane_sequence) {
    ego_lane_seq = ego_lane_sequence->lanes();
  } else {
    std::vector<uint64_t> ego_lane_id = {start_lane->id()};
    if (!GetMatchLanePathByIds(map, ego_lane_id, all_lane_seqs,
                               &ego_lane_seq)) {
      return false;
    }
  }
  std::string str = "";
  for (const auto& lane_id : ego_lane_seq) {
    str += absl::StrCat(lane_id->id(), ",");
  }
  Log2DDS::LogDataV0("lc_debug", "ego lane seq: " + str);
  if (candidate_lane_seqs->empty()) {
    candidate_lane_seqs->emplace_back(ego_lane_seq);
  } else if (!has_contain_ego_lane) {
    candidate_lane_seqs->insert((candidate_lane_seqs->begin() + 1),
                                ego_lane_seq);
  }

  // 4. Handle manual lc
  bool need_manual_lc = false;
  if ((mlc_cmd == st::DriverAction::LC_CMD_LEFT ||
       mlc_cmd == st::DriverAction::LC_CMD_RIGHT) &&
      preferred_lane_path->lane_seq() &&
      !preferred_lane_path->lane_seq()->lanes().empty()) {
    Log2DDS::LogDataV2("mlc_debug", "handle manual lc.");
    std::vector<uint64_t> raw_lane_ids;
    for (const auto& lane : preferred_lane_path->lane_seq()->lanes()) {
      raw_lane_ids.push_back(lane->id());
    }
    std::vector<LaneConstPtr> match_lane_seq;
    if (GetMatchLanePathByIds(map, raw_lane_ids, all_lane_seqs,
                              &match_lane_seq)) {
      candidate_lane_seqs->clear();
      candidate_lane_seqs->emplace_back(match_lane_seq);
      if (std::find_if(match_lane_seq.begin(), match_lane_seq.end(),
                       [&](LaneConstPtr lane) {
                         return lane && lane->id() == nearest_lane->id();
                       }) == match_lane_seq.end()) {
        candidate_lane_seqs->emplace_back(ego_lane_seq);
      }
      std::vector<st::mapping::ElementId> lane_ids;
      for (const auto& lane : match_lane_seq) {
        if (!lane || lane->id() == 0 || !lane->center_line().IsValid()) break;
        lane_ids.emplace_back(lane->id());
      }
      st::mapping::LanePath lane_path(map, lane_ids, 0.0, 1.0);
      *preferred_lane_path = std::move(lane_path);
      st::planning::Log2DDS::LogDataV2("mlc_debug", "Handle manual lc finish");
      need_manual_lc = true;
    }
  }

  // 5. Handle scene lc
  bool need_scene_lc =
      scene_target_lane_seq && scene_target_lane_seq->IsValid();
  if (!need_manual_lc && need_scene_lc) {
    bool top1_is_curseq = true, top2_is_curseq = true;
    for (int i = 0; i < candidate_lane_seqs->size(); i++) {
      if ((*candidate_lane_seqs)[i].empty()) continue;
      bool is_curseq =
          std::find_if((*candidate_lane_seqs)[i].begin(),
                       (*candidate_lane_seqs)[i].end(), [&](LaneConstPtr lane) {
                         return lane && lane->id() == nearest_lane->id();
                       }) != (*candidate_lane_seqs)[i].end();
      if (i == 0) top1_is_curseq = is_curseq;
      if (i == 1) top2_is_curseq = is_curseq;
      if (i > 1) break;
    }
    if (top1_is_curseq && top2_is_curseq) {
      candidate_lane_seqs->insert(candidate_lane_seqs->begin(),
                                  scene_target_lane_seq->lanes());
    } else if (top1_is_curseq && !top2_is_curseq) {
      candidate_lane_seqs->insert(candidate_lane_seqs->begin(),
                                  *(candidate_lane_seqs->begin() + 1));
    }
  }

  // LKA close normal auto lc
  Log2DDS::LogDataV0(
      "lc_debug",
      absl::StrCat("lka:", is_lka, "/", need_manual_lc, "/", need_scene_lc));
  std::string top1_debug = "";
  for (const auto& lane : (*candidate_lane_seqs)[0]) {
    top1_debug += lane->id() + ",";
  }
  Log2DDS::LogDataV0("lc_debug", "lka raw top1: " + top1_debug);

  if (is_lka && !need_manual_lc && !need_scene_lc) {
    // pnp
    LaneConstPtr valid_lane = nearest_lane;
    if (pre_target_lane_seq) {
      auto lane = pre_target_lane_seq->GetNearestLane(ego_pos);
      if (lane) valid_lane = lane;
    }
    for (int i = 0; i < candidate_lane_seqs->size(); i++) {
      if ((*candidate_lane_seqs)[i].empty()) continue;
      bool is_curseq =
          std::find_if((*candidate_lane_seqs)[i].begin(),
                       (*candidate_lane_seqs)[i].end(), [&](LaneConstPtr lane) {
                         return lane && lane->id() == valid_lane->id();
                       }) != (*candidate_lane_seqs)[i].end();
      if (is_curseq && i == 0) break;
      if (is_curseq) {
        candidate_lane_seqs->insert(candidate_lane_seqs->begin(),
                                    *(candidate_lane_seqs->begin() + i));
        break;
      }
    }
    // // env
    // LaneSequencePtr env_laneseq = nullptr;
    // if (pre_target_lane_seq) {
    //   const auto lane = pre_target_lane_seq->GetNearestLane(ego_pos);
    //   env_laneseq = map->GetLaneSequence(lane, true);
    // } else {
    //   env_laneseq = map->GetLaneSequence(start_lane, true);
    // }
    // if (env_laneseq) {
    //   candidate_lane_seqs->insert(candidate_lane_seqs->begin(),
    //                               env_laneseq->lanes());
    // } else {
    //   Log2DDS::LogDataV0("lc_debug", "lka env laneseq fail");
    // }
  }

  top1_debug = "";
  for (const auto& lane : (*candidate_lane_seqs)[0]) {
    top1_debug += lane->id() + ",";
  }
  Log2DDS::LogDataV0("lc_debug", "new top1: " + top1_debug);

  // 6. Clamp size by kTopN, Backward Extend Lane.
  while (candidate_lane_seqs->size() > kTopN) {
    candidate_lane_seqs->erase(candidate_lane_seqs->end() - 1);
  }
  BackwardExtendLane(ego_pos, map, candidate_lane_seqs);
  return !candidate_lane_seqs->empty();
}

bool GetMatchLanePathByIds(
    const MapConstPtr& map, const std::vector<uint64_t>& lane_ids,
    const std::vector<std::vector<LaneConstPtr>>& all_lane_seqs,
    std::vector<LaneConstPtr>* match_lane_seq) {
  absl::flat_hash_set<uint64_t> input_lanes(lane_ids.begin(), lane_ids.end());
  std::vector<int> repeat_num(all_lane_seqs.size(), 0);
  for (int i = 0; i < all_lane_seqs.size(); ++i) {
    for (const auto& lane : all_lane_seqs[i]) {
      if (input_lanes.contains(lane->id())) {
        repeat_num[i] += 1;
      }
    }
  }
  std::vector<int>::iterator biggest =
      std::max_element(std::begin(repeat_num), std::end(repeat_num));
  const int index = std::distance(std::begin(repeat_num), biggest);
  if (repeat_num[index] < 1) {
    return false;
  }

  std::vector<LaneConstPtr> lanes = all_lane_seqs[index];
  std::set<uint64_t> lane_set;
  for (const auto& lane : lanes) {
    if (lane) lane_set.insert(lane->id());
  }
  while (lanes.back()) {
    const auto next_lanes = map->GetValidNextLanes(lanes.back());
    const auto next_lane = next_lanes.empty() ? nullptr : next_lanes.front();
    if (!next_lane || lane_set.find(next_lane->id()) != lane_set.end()) {
      break;
    }
    lanes.emplace_back(next_lane);
    lane_set.insert(next_lane->id());
  }
  *match_lane_seq = lanes;

  return true;
}

double GetDistanceToSolidLine(const Vec2d& start_point,
                              const LaneSequencePtr& tgt_lane_seq,
                              const LaneConstPtr& nearest_lane, bool is_left,
                              double check_range) {
  double dis_to_solidline = std::numeric_limits<double>::infinity();
  if (!tgt_lane_seq) {
    return dis_to_solidline;
  }

  if (!nearest_lane) {
    return dis_to_solidline;
  }

  const auto nearest_lane_boundary =
      is_left ? nearest_lane->left_boundary() : nearest_lane->right_boundary();
  if (!nearest_lane_boundary) {
    return dis_to_solidline;
  }
  auto target_linetype = is_left ? SOLID_DASHED : DASHED_SOLID;
  const auto line_curve = nearest_lane_boundary->line_curve();
  double dis_from_start = 0.0;
  math::Vec2d nearest_point;
  line_curve.GetDistance(start_point, &nearest_point, &dis_from_start);
  double checked_s = -dis_from_start;
  // search forward
  bool has_found_nearest = false;
  std::vector<LaneBoundaryType> boundary_types;
  for (const auto& target_lane : tgt_lane_seq->lanes()) {
    boundary_types.clear();
    if (target_lane->id() == nearest_lane->id()) {
      has_found_nearest = true;
    }
    if (!has_found_nearest) {
      continue;
    }

    dis_to_solidline = 0.0;
    const auto lane_boundary =
        is_left ? target_lane->left_boundary() : target_lane->right_boundary();
    if (!lane_boundary) {
      // LOGDATA("solid_line_debug", "no boundary ptr");
      continue;
    }
    const auto line_curve = lane_boundary->line_curve();
    boundary_types = lane_boundary->boundary_types();
    for (const auto& boundary_type : boundary_types) {
      auto lane_type = boundary_type.line_type;
      if (lane_type != SOLID && lane_type != SOLID_SOLID &&
          lane_type != SHADED_AREA && lane_type != target_linetype) {
        dis_to_solidline = boundary_type.s;
      } else {
        Log2DDS::LogDataV0("solid_line_debug",
                           absl::StrCat("boundary_type.s: ", boundary_type.s));
        Log2DDS::LogDataV0(
            "solid_line_debug",
            absl::StrCat("dis_to_solidline: ", dis_to_solidline));
        Log2DDS::LogDataV0("solid_line_debug",
                           absl::StrCat("checked_s: ", checked_s));
        Log2DDS::LogDataV0(
            "solid_line_debug",
            absl::StrCat("dist to first: ", dis_to_solidline + checked_s));
        Log2DDS::LogDataV0(
            "solid_line_debug",
            absl::StrCat("dist to next: ", boundary_type.s + checked_s));
        if (boundary_type.s + checked_s < 0) {
          continue;
        }
        Log2DDS::LogDataV0(
            "solid_line_debug",
            absl::StrCat("Found solid line!! lane id:", target_lane->id()));
        return dis_to_solidline + checked_s;
      }
    }
    checked_s += line_curve.length();
    // termination condition
    if (checked_s > check_range) {
      break;
    }
  }
  return std::numeric_limits<double>::infinity();
}

std::vector<double> GetAllDistToMerge(const std::vector<LaneConstPtr>& lanes,
                                      double dist_to_start_lane_end,
                                      int start_index) {
  constexpr double kMaxDistanceCheck = 1500;
  std::vector<double> dist_to_merge_vec;
  if (start_index == -1) {
    return dist_to_merge_vec;
  }
  double dist_to_next_merge = dist_to_start_lane_end;
  bool is_in_intersection = false;
  if (lanes[start_index] && lanes[start_index]->junction_id() != 0) {
    is_in_intersection = true;
  }
  for (int i = start_index + 1; i < lanes.size(); ++i) {
    LaneConstPtr lane = lanes[i];
    if (!lane) {
      continue;
    }
    if (dist_to_next_merge > kMaxDistanceCheck) break;
    if (lane->pre_lane_ids().size() > 1 && !is_in_intersection) {
      dist_to_merge_vec.emplace_back(dist_to_next_merge);
    }
    dist_to_next_merge += lane->topo_length();
    if (lane->junction_id() != 0)
      is_in_intersection = true;
    else
      is_in_intersection = false;
  }
  return dist_to_merge_vec;
}

std::pair<double, double> GetDistanceToSectionForkAndMerge(
    const MapPtr& map, const LaneSequencePtr& tgt_lane_seq,
    double dist_to_start_lane_end, int start_index) {
  double dist_to_section_fork = std::numeric_limits<double>::infinity();
  double dist_to_section_merge = std::numeric_limits<double>::infinity();
  if (!map || !map->route() || !tgt_lane_seq) {
    return {dist_to_section_fork, dist_to_section_merge};
  }
  if (start_index == -1) {
    return {dist_to_section_fork, dist_to_section_merge};
  }
  bool find_nearest = false;
  bool find_fork = false, find_merge = false;
  const auto& lanes = tgt_lane_seq->lanes();
  dist_to_section_fork = 0.0;
  dist_to_section_merge = 0.0;
  for (int index = start_index; index < lanes.size(); index++) {
    if (lanes[index] == nullptr) break;
    if (!find_fork && lanes[index]->next_lane_ids().size() > 1) {
      const auto next_lane_front =
          map->GetLaneById(lanes[index]->next_lane_ids().front());
      for (auto next_iter = lanes[index]->next_lane_ids().rbegin();
           next_iter != lanes[index]->next_lane_ids().rend(); next_iter++) {
        const auto next_lane_tmp = map->GetLaneById((*next_iter));
        if (next_lane_front && next_lane_tmp &&
            next_lane_front->section_id() != next_lane_tmp->section_id()) {
          if (index == start_index)
            dist_to_section_fork = dist_to_start_lane_end;
          else
            dist_to_section_fork += lanes[index]->topo_length();
          find_fork = true;
          break;
        }
      }
    }
    if (!find_merge && index + 1 < lanes.size() &&
        lanes[index + 1]->pre_lane_ids().size() > 1) {
      const auto pre_lane_front =
          map->GetLaneById(lanes[index + 1]->pre_lane_ids().front());
      for (auto pre_iter = lanes[index + 1]->pre_lane_ids().rbegin();
           pre_iter != lanes[index + 1]->pre_lane_ids().rend(); pre_iter++) {
        const auto pre_lane_tmp = map->GetLaneById(*pre_iter);
        if (pre_lane_front && pre_lane_tmp &&
            pre_lane_front->section_id() != pre_lane_tmp->section_id()) {
          if (index == start_index)
            dist_to_section_merge = dist_to_start_lane_end;
          else
            dist_to_section_merge += lanes[index]->topo_length();
          find_merge = true;
          break;
        }
      }
    }
    if (!find_fork) {
      if (index == start_index)
        dist_to_section_fork = dist_to_start_lane_end;
      else
        dist_to_section_fork += lanes[index]->topo_length();
    }
    if (!find_merge) {
      if (index == start_index)
        dist_to_section_merge = dist_to_start_lane_end;
      else
        dist_to_section_merge += lanes[index]->topo_length();
    }
    if (find_fork && find_merge) break;
  }

  dist_to_section_fork = find_fork ? std::max(dist_to_section_fork, 0.0)
                                   : std::numeric_limits<double>::infinity();
  dist_to_section_merge = find_merge ? std::max(dist_to_section_merge, 0.0)
                                     : std::numeric_limits<double>::infinity();
  return {dist_to_section_fork, dist_to_section_merge};
}

std::pair<double, double> GetDistToNearestSplit(
    const MapPtr& map, const std::vector<LaneConstPtr>& lanes,
    const LaneConstPtr& pre_lane, double dist_to_start_lane_end,
    int start_index) {
  if (start_index == -1 || !map) {
    return {std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::max()};
  }
  double dist_to_pre_split =
      lanes[start_index]->topo_length() - dist_to_start_lane_end;
  double dist_to_next_split = dist_to_start_lane_end;
  bool is_find_pre_split = false;
  bool is_find_next_split = false;
  for (int i = start_index - 1; i >= 0; --i) {
    LaneConstPtr lane = lanes[i];
    if (!lane) {
      continue;
    }
    if (lane->next_lane_ids().size() > 1) {
      is_find_pre_split = true;
      break;
    }
    dist_to_pre_split += lane->topo_length();
  }
  if (pre_lane && pre_lane->next_lane_ids().size() > 1) {
    is_find_pre_split = true;
  }
  for (int i = start_index; i < lanes.size(); ++i) {
    LaneConstPtr lane = lanes[i];
    if (!lane) {
      continue;
    }
    if (i != start_index) dist_to_next_split += lane->topo_length();
    if (lane->next_lane_ids().size() > 1) {
      const auto next_lane = map->GetLaneById(lane->next_lane_ids().at(0));
      if (next_lane && next_lane->junction_id() == 0) {
        is_find_next_split = true;
        break;
      }
    }
  }
  if (!is_find_pre_split) {
    dist_to_pre_split = std::numeric_limits<double>::lowest();
  }
  if (!is_find_next_split) {
    dist_to_next_split = std::numeric_limits<double>::max();
  }
  return {-dist_to_pre_split, dist_to_next_split};
}

double GetNearestSplitAngle(const std::vector<LaneConstPtr>& lanes,
                            const double dist_to_split) {
  constexpr double kMaxDistanceCheck = 60.0;           // m.
  constexpr double kQueryDistAfterSplitPoint = 30.0;   // m.
  constexpr double kQueryDistbeforeSplitPoint = 10.0;  // m.
  double nearest_split_angle = 0.0;
  if (dist_to_split <= kMaxDistanceCheck) {
    LaneConstPtr lane_split_front = nullptr;
    LaneConstPtr lane_split_back = nullptr;
    for (const auto& lane : lanes) {
      if (lane_split_front) {
        lane_split_back = lane;
        break;
      }
      if (lane && lane->next_lane_ids().size() > 1) {
        lane_split_front = lane;
      }
    }
    if (lane_split_front != nullptr && lane_split_back != nullptr &&
        lane_split_front->curve_length() > 0.0 &&
        lane_split_back->curve_length() > 0.0) {
      const double query_fraction = std::min(
          1.0, kQueryDistAfterSplitPoint / lane_split_back->curve_length());
      const Vec2d split_back_point =
          lane_split_back->LerpPointFromFraction(query_fraction);
      const Vec2d split_point = lane_split_front->LerpPointFromFraction(1.0);
      const double split_front_query_fraction =
          std::max(0.0, 1.0 - kQueryDistbeforeSplitPoint /
                                  lane_split_front->curve_length());
      const Vec2d split_front_point =
          lane_split_front->LerpPointFromFraction(split_front_query_fraction);
      nearest_split_angle = NormalizeAngle2D(split_point - split_front_point,
                                             split_back_point - split_point);
    }
  }
  return nearest_split_angle;
}

double GetDistToVirtualLane(const std::vector<LaneConstPtr>& lanes,
                            double dist_to_start_lane_end, int start_index) {
  double dist_to_virtual_lane = std::numeric_limits<double>::infinity();
  bool is_find_junction = false;
  if (start_index == -1 || start_index >= lanes.size()) {
    return dist_to_virtual_lane;
  }
  if (lanes[start_index] && lanes[start_index]->junction_id() != 0 &&
      (lanes[start_index]->type() == LaneType::LANE_VIRTUAL_JUNCTION ||
       lanes[start_index]->light_status() != LightStatus::NONE_LIGHT)) {
    return 0.0;
  }
  dist_to_virtual_lane = dist_to_start_lane_end;
  for (int i = start_index + 1; i < lanes.size(); ++i) {
    LaneConstPtr lane = lanes[i];
    if (!lane) {
      continue;
    }
    if (lane->junction_id() != 0 &&
        (lane->type() == LaneType::LANE_VIRTUAL_JUNCTION ||
         lane->light_status() != LightStatus::NONE_LIGHT)) {
      is_find_junction = true;
      break;
    }
    dist_to_virtual_lane += lane->topo_length();
  }
  if (!is_find_junction) return std::numeric_limits<double>::infinity();
  return dist_to_virtual_lane;
}

bool GetLaneSeqInfo(LaneConstPtr current_lane, const Vec2d start_point,
                    const double ego_v, const LaneSequencePtr& tgt_lane_seq,
                    const MapPtr& map, LaneSeqInfo* tgt_seq_info) {
  std::vector<std::pair<uint64_t, double>> nearest_section_lanes;
  return GetLaneSeqInfo(current_lane,
                        st::Behavior_FunctionId::Behavior_FunctionId_CITY_NOA,
                        start_point, ego_v, tgt_lane_seq, map, nullptr,
                        nearest_section_lanes, tgt_seq_info);
}

bool GetLaneSeqInfo(
    LaneConstPtr current_lane, const st::Behavior_FunctionId& function_id,
    const Vec2d start_point, const double ego_v,
    const LaneSequencePtr& tgt_lane_seq, const MapPtr& map,
    const LdLiteMapPtr& lite_map,
    std::vector<std::pair<uint64_t, double>>& nearest_section_lanes,
    LaneSeqInfo* tgt_seq_info) {
  // SCOPED_TRACE(__FUNCTION__);

  if (!tgt_lane_seq) {
    return false;
  }
  if (!current_lane) {
    return false;
  }
  double s_offset = 0.0;
  const double l = tgt_lane_seq->GetProjectionDistance(
      {start_point.x(), start_point.y()}, &s_offset);
  const auto projection_point = tgt_lane_seq->GetPointAtS(s_offset);

  // LANE Supplement
  tgt_seq_info->lane_seq = tgt_lane_seq;
  const auto& nearest_lane = tgt_lane_seq->GetNearestLane(projection_point);
  if (!nearest_lane) {
    return false;
  }

  double dist_to_start_lane_end = std::numeric_limits<double>::infinity();
  int start_index = -1;
  tgt_lane_seq->GetStartDistAndIndex(map->route()->navi_start(), nearest_lane,
                                     s_offset, dist_to_start_lane_end,
                                     start_index);

  // tgt_seq_info->is_current = current_lane->id() == nearest_lane->id();
  tgt_seq_info->is_current = tgt_lane_seq->IsOnLaneSequence(current_lane);
  LaneConstPtr left_lane = map->GetLeftLane(current_lane);
  if (left_lane) {
    if (tgt_lane_seq->IsOnLaneSequence(left_lane)) {
      tgt_seq_info->lc_dir = Command_LaneChangeLeft;
    }
  }
  LaneConstPtr right_lane = map->GetRightLane(current_lane);
  if (right_lane) {
    if (tgt_lane_seq->IsOnLaneSequence(right_lane)) {
      tgt_seq_info->lc_dir = Command_LaneChangeRight;
    }
  }
  tgt_seq_info->nearest_lane = nearest_lane;

  // Navi Supplement
  LaneConstPtr navi_end_lane;
  const auto& ehp_v2_info = map->v2_info();

  constexpr double kPreviewLcNumTime = 8.0;
  constexpr double kMinPreviewLcNumDis = 50.0;
  int lc_num = map->GetPriorityLaneRelation(nearest_lane);
  double preview_lc_num_dis =
      std::max(ego_v * kPreviewLcNumTime, kMinPreviewLcNumDis);
  // LaneConstPtr preview_lane = nullptr;
  bool find_nearest = false, find_preview_lane = false;
  bool split_has_calculated = false;
  for (auto it_lane : tgt_lane_seq->lanes()) {
    if (!it_lane) continue;
    if (!find_nearest && it_lane->id() == nearest_lane->id()) {
      if (it_lane->id() == tgt_lane_seq->GetSplitLaneId()) {
        split_has_calculated = true;
      }
      const auto cur_section_ptr = map->GetSectionById(it_lane->section_id());
      if (cur_section_ptr && cur_section_ptr->outgoing_sections().size() > 1)
        break;

      double s_nearest, l_nearest;
      if (it_lane->center_line().GetProjection(start_point, &s_nearest,
                                               &l_nearest)) {
        preview_lc_num_dis -= (it_lane->curve_length() - s_nearest);
      }
      // preview_lane = it_lane;
      find_nearest = true;
      if (!find_preview_lane && preview_lc_num_dis < 0.0 &&
          it_lane->junction_id() == 0) {
        find_preview_lane = true;
      }
      continue;
    }
    if (!find_nearest) continue;
    if (it_lane->id() == tgt_lane_seq->GetSplitLaneId()) {
      split_has_calculated = true;
    }
    if (!it_lane->is_navigation() || it_lane->turn_type() != TurnType::NO_TURN)
      break;

    preview_lc_num_dis -= it_lane->topo_length();
    // preview_lane = it_lane;
    if (!find_preview_lane) {
      lc_num = map->GetPriorityLaneRelation(it_lane);
    } else {
      int cur_lc_num = map->GetPriorityLaneRelation(it_lane);
      lc_num = std::abs(cur_lc_num) < std::abs(lc_num) ? cur_lc_num : lc_num;
      break;
    }
    const auto cur_section_ptr = map->GetSectionById(it_lane->section_id());
    if (cur_section_ptr && cur_section_ptr->outgoing_sections().size() > 1)
      break;
    if (!find_preview_lane && preview_lc_num_dis < 0.0 &&
        it_lane->junction_id() == 0) {
      find_preview_lane = true;
      continue;
    }
  }

  tgt_seq_info->split_task_state = tgt_lane_seq->GetSplitTasksState();
  tgt_seq_info->lc_num = std::abs(lc_num);
  tgt_seq_info->dist_to_navi_end = tgt_lane_seq->GetDistanceToPOI(
      Poi_NaviEnd, projection_point.x(), projection_point.y(), navi_end_lane,
      dist_to_start_lane_end, start_index);

  if (lc_num < 0) {
    tgt_seq_info->navi_lc_command = BehaviorCommand::Command_LaneChangeRight;
  } else if (lc_num > 0) {
    tgt_seq_info->navi_lc_command = BehaviorCommand::Command_LaneChangeLeft;
  } else {
    tgt_seq_info->navi_lc_command = BehaviorCommand::Command_Invalid;
  }
  tgt_seq_info->navi_end_lane = navi_end_lane;
  tgt_seq_info->lane_seq_connect_navi_end =
      map->route()->CanDriveToRouteEnd(tgt_lane_seq, nearest_lane);
  tgt_seq_info->lc_num =
      tgt_seq_info->lane_seq_connect_navi_end ? 0 : tgt_seq_info->lc_num;
  if (tgt_lane_seq->GetSplitTasksState() != SplitTasksState::None &&
      !split_has_calculated) {
    if (tgt_lane_seq->GetSplitLcNumIsMax()) {
      tgt_seq_info->lc_num =
          std::max(tgt_seq_info->lc_num, tgt_lane_seq->GetSplitLcNum());
    } else {
      tgt_seq_info->lc_num =
          std::min(tgt_seq_info->lc_num, tgt_lane_seq->GetSplitLcNum());
    }
  }
  if (tgt_lane_seq->HasStraightJunction()) {
    tgt_seq_info->lc_num_suc_junc =
        tgt_lane_seq->GetSucceedStraightJunctionLcNum();
  }

  // BoundaryLine
  tgt_seq_info->dist_to_left_solid_line = GetDistanceToSolidLine(
      start_point, tgt_lane_seq, nearest_lane, true, 150.0);
  tgt_seq_info->dist_to_right_solid_line = GetDistanceToSolidLine(
      start_point, tgt_lane_seq, nearest_lane, false, 150.0);

  // Merge
  LaneConstPtr merge_lane = nullptr;
  tgt_seq_info->dist_to_merge = tgt_lane_seq->GetDistanceToPOI(
      Poi_Merge, start_point.x(), start_point.y(), merge_lane,
      dist_to_start_lane_end, start_index);
  tgt_seq_info->merge_lane = merge_lane;
  if (merge_lane) {
    const auto& merge_topo = merge_lane->merge_topology();
    if (merge_topo == TOPOLOGY_MERGE_LEFT) {
      tgt_seq_info->merge_command = Command_LaneChangeLeft;
    } else if (merge_topo == TOPOLOGY_MERGE_RIGHT) {
      tgt_seq_info->merge_command = Command_LaneChangeRight;
    }
  }
  tgt_seq_info->dist_to_merge_vec = GetAllDistToMerge(
      tgt_lane_seq->lanes(), dist_to_start_lane_end, start_index);

  // Section
  // SectionInfo cur_section;
  // bool result =
  //     map->route()->GetSectionByIdFromRoute(nearest_lane->section_id(),
  //     cur_section);
  SectionConstPtr cur_section_ptr =
      map->GetSectionById(nearest_lane->section_id());
  tgt_seq_info->cur_section_ptr = cur_section_ptr;
  tgt_seq_info->cur_lane_position =
      GetNormalLanePosition(map, nearest_lane->id(), cur_section_ptr);
  tgt_seq_info->cur_section_lane_num = GetNormalLaneSize(map, cur_section_ptr);
  // Special lane type
  LaneConstPtr bus_lane = nullptr;
  std::unordered_set<LaneType> lane_type_set{LANE_BUS_NORMAL, LANE_HOV_NORMAL,
                                             LANE_HARBOR_STOP, LANE_BRT};
  tgt_seq_info->dist_to_bus_lane = tgt_lane_seq->GetDistanceToTargetLaneType(
      bus_lane, lane_type_set, dist_to_start_lane_end, start_index);
  // tgt_seq_info->dist_to_bus_lane = tgt_lane_seq->GetDistanceToTargetLaneType(
  //     bus_lane, lane_type_set, map->route()->navi_start());
  tgt_seq_info->dist_to_bus_lane_vec = tgt_lane_seq->GetAllDistanceToTargetLaneType(
    lane_type_set, dist_to_start_lane_end, start_index, tgt_seq_info->bus_lane_passable_mark);

  // Merge && Split
  // tgt_seq_info->dist_to_merge = tgt_lane_seq->GetDistanceToPOI(
  //     Poi_Merge, projection_point.x(), projection_point.y());

  LaneConstPtr split_lane = nullptr;
  double dis_to_split = tgt_lane_seq->GetDistanceToPOI(
      Poi_Split, projection_point.x(), projection_point.y(), split_lane,
      dist_to_start_lane_end, start_index);
  tgt_seq_info->dist_to_split =
      std::fmin(dis_to_split, tgt_seq_info->dist_to_navi_end);

  const auto dist_to_section_fork_merge = GetDistanceToSectionForkAndMerge(
      map, tgt_lane_seq, dist_to_start_lane_end, start_index);
  tgt_seq_info->dist_to_section_fork = dist_to_section_fork_merge.first;
  tgt_seq_info->dist_to_section_merge = dist_to_section_fork_merge.second;

  LaneConstPtr pre_lane = nullptr;
  LaneConstPtr cur_lane = nullptr;
  if (start_index != -1) {
    cur_lane = tgt_seq_info->lane_seq->lanes()[start_index];
  }
  if (start_index == 0 && cur_lane && cur_lane->pre_lane_ids().size() == 1) {
    pre_lane = map->GetLaneById(cur_lane->pre_lane_ids()[0]);
  }
  tgt_seq_info->dist_to_nearest_split =
      GetDistToNearestSplit(map, tgt_seq_info->lane_seq->lanes(), pre_lane,
                            dist_to_start_lane_end, start_index);
  tgt_seq_info->nearest_split_angle =
      GetNearestSplitAngle(tgt_seq_info->lane_seq->lanes(),
                           tgt_seq_info->dist_to_nearest_split.second);
  // merge zone
  tgt_seq_info->dist_to_merged_zone.first =
      std::numeric_limits<double>::lowest();
  tgt_seq_info->dist_to_merged_zone.second = std::numeric_limits<double>::max();
  for (int i = 0; i < tgt_seq_info->lane_seq->lanes().size(); ++i) {
    LaneConstPtr lane = tgt_seq_info->lane_seq->lanes()[i];
    if (!lane) {
      continue;
    }
    if (lane->is_acc_adj_lane()) {
      tgt_seq_info->dist_to_merged_zone.first = lane->merge_start_dis();
      tgt_seq_info->dist_to_merged_zone.second = lane->merge_end_dis();
      break;
    }
  }

  // V2 Information
  // if (!ehp_v2_info.has_navigation) {
  //   return true;
  // }
  bool has_valid_v2_turn_type = false;
  double nearest_v2_turn_dis = 0.0;
  for (const auto& turn_info : ehp_v2_info.turn_info) {
    if (!turn_info.is_valid) break;
    if (turn_info.dist < 0.0) continue;
    tgt_seq_info->nearest_turn_type_v2 = turn_info.detail_turn_type;
    has_valid_v2_turn_type = true;
    nearest_v2_turn_dis = turn_info.dist;
    break;
  }
  for (const auto& turn_info : ehp_v2_info.turn_info) {
    if (!turn_info.is_valid) break;
    if (turn_info.dist < 0.0) continue;
    if ((turn_info.detail_turn_type ==
             ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT ||
         turn_info.detail_turn_type ==
             ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT ||
         turn_info.detail_turn_type ==
             ad_byd::planning::V2TurnInfo::V2DetailTurnType::UTURN ||
         turn_info.detail_turn_type ==
             ad_byd::planning::V2TurnInfo::V2DetailTurnType::CONTINUE ||
         turn_info.detail_turn_type ==
             ad_byd::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT_ONLY)) {
      tgt_seq_info->dist_to_junction_v2 = turn_info.dist;
      break;
    }
  }
  for (const auto& turn_info : ehp_v2_info.turn_info) {
    if (!turn_info.is_valid) break;
    if (turn_info.detail_turn_type ==
            ad_byd::planning::V2TurnInfo::V2DetailTurnType::NONE ||
        turn_info.dist < 0.0) {
      continue;
    }
    tgt_seq_info->dist_to_navi_end_v2 = turn_info.dist;
    tgt_seq_info->last_turn_type_v2 = turn_info.detail_turn_type;
    break;
  }
  LaneConstPtr virtual_lane = nullptr;
  double dist_to_junction = tgt_lane_seq->GetDistanceToJunction(
      dist_to_start_lane_end, start_index, virtual_lane);
  tgt_seq_info->junction_lane = virtual_lane;
  tgt_seq_info->dist_to_junction_id = dist_to_junction;

  if (map->type() == ad_byd::planning::MapType::BEV_MAP) {
    if (lite_map) {
      tgt_seq_info->dist_to_junction = lite_map->dist_to_junction();
      tgt_seq_info->dist_to_virtual_lane = lite_map->dist_to_junction();
    } else {
      tgt_seq_info->dist_to_junction = tgt_seq_info->dist_to_junction_v2;
    }
  } else {
    tgt_seq_info->dist_to_junction = dist_to_junction;
    tgt_seq_info->dist_to_virtual_lane = GetDistToVirtualLane(
        tgt_lane_seq->lanes(), dist_to_start_lane_end, start_index);
  }
  // if (tgt_seq_info->dist_to_junction_v2 !=
  //         std::numeric_limits<double>::infinity() &&
  //     tgt_seq_info->dist_to_junction_v2 - dist_to_junction > 40.0 &&
  //     dist_to_junction > 150.0 &&
  //     map->type() == ad_byd::planning::MapType::BEV_MAP) {
  //   tgt_seq_info->dist_to_junction = tgt_seq_info->dist_to_junction_v2;
  // } else {
  //   tgt_seq_info->dist_to_junction = dist_to_junction;
  // }
  // if dist to junction == dblmax
  // if (tgt_seq_info->dist_to_junction - tgt_seq_info->dist_to_junction_v2 >
  //     2000.0) {
  //   tgt_seq_info->dist_to_junction = tgt_seq_info->dist_to_junction_v2;
  // }

  if (map->type() == ad_byd::planning::MapType::BEV_MAP &&
      ehp_v2_info.is_valid && has_valid_v2_turn_type) {
    bool has_virtual_navi_lane = false;
    for (auto lane_iter = tgt_seq_info->lane_seq->lanes().rbegin();
         lane_iter != tgt_seq_info->lane_seq->lanes().rend(); lane_iter++) {
      if ((*lane_iter)->is_virtual_navigation()) {
        has_virtual_navi_lane = true;
        break;
      }
    }
    tgt_seq_info->dist_to_navi_end =
        std::max(tgt_seq_info->dist_to_navi_end, nearest_v2_turn_dis);
    if (!has_virtual_navi_lane)
      tgt_seq_info->dist_to_navi_end = nearest_v2_turn_dis;
  }

  tgt_seq_info->lc_num_env = tgt_seq_info->lc_num;
  tgt_seq_info->dist_to_navi_end_env = tgt_seq_info->dist_to_navi_end;
  if (map->type() == ad_byd::planning::MapType::BEV_MAP && lite_map) {
    int nearest_section_lane_idx = 0;
    for (int i = 0; i < nearest_section_lanes.size(); i++) {
      if (nearest_section_lanes[i].first == nearest_lane->id()) {
        nearest_section_lane_idx = i + 1;
        break;
      }
    }
    const auto& ld_lite_prio_lane = lite_map->lite_prio_lane();
    double nearest_lane_navi_dis = 0.0;
    if (ld_lite_prio_lane.is_valid && nearest_section_lane_idx > 0 &&
        lite_map->GetNaviDistanceByIndex(nearest_section_lane_idx,
                                         &nearest_lane_navi_dis)) {
      if (tgt_seq_info->split_task_state == SplitTasksState::None) {
        tgt_seq_info->lc_num_lite = std::abs(
            ld_lite_prio_lane.priority_lane_idx - nearest_section_lane_idx);
        tgt_seq_info->dist_to_navi_end_lite = nearest_lane_navi_dis;
        tgt_seq_info->lite_map_match_success = true;
        if (FLAGS_planner_enable_bev_lite_match) {
          tgt_seq_info->lc_num = tgt_seq_info->lc_num_lite;
          tgt_seq_info->dist_to_navi_end = tgt_seq_info->dist_to_navi_end_lite;
        }
      } else {
        int split_task_lc_num = 0;
        bool is_split_left_task =
            (tgt_seq_info->split_task_state == SplitTasksState::Left_Task);
        if (lite_map->GetNaviDistanceBySplitDir(
                nearest_section_lane_idx, is_split_left_task,
                &split_task_lc_num, &nearest_lane_navi_dis)) {
          tgt_seq_info->lc_num_lite = split_task_lc_num;
          tgt_seq_info->dist_to_navi_end_lite = nearest_lane_navi_dis;
          tgt_seq_info->lite_map_match_success = true;
          if (FLAGS_planner_enable_bev_lite_match) {
            tgt_seq_info->lc_num = tgt_seq_info->lc_num_lite;
            tgt_seq_info->dist_to_navi_end =
                tgt_seq_info->dist_to_navi_end_lite;
          }
        }
      }
    }
  }
  // set same lc_num and dist_to_navi_end for icc multi_tasks
  if (function_id == st::Behavior_FunctionId::Behavior_FunctionId_LKA) {
    tgt_seq_info->lc_num = 0;
    tgt_seq_info->dist_to_navi_end = std::numeric_limits<double>::infinity();
  }

  return true;
}
}  // namespace planning
}  // namespace ad_byd
