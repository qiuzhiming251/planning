

#include "decider/decision_manager/turn_signal_decider.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/log_data.h"
// #include "global/logging.h"
// #include "global/trace.h"
// #include "lite/check.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
#include "plan_common/plan_common_defs.h"

namespace st {
namespace planning {

namespace {

using TurnType = ad_byd::planning::TurnType;
using ReactionRule = ad_byd::planning::LaneInteraction::ReactionRule;
using GeometricConfiguration =
    ad_byd::planning::LaneInteraction::GeometricConfiguration;
using SectionDirection = ad_byd::planning::Section::SectionDirection;

constexpr double kPreviewDirectionDist = 15.0;     // m.
constexpr double kDirectionDist = 20.0;            // m.
constexpr double kPreviewMergeForkTime = 2.5;      // s.
constexpr double kPreviewMergeForkMinDist = 20.0;  // m.
constexpr double kPreviewMergeForkMaxDist = 40.0;  // m.

bool HasReachedDestination(const PlannerSemanticMapManager& psmm,
                           const mapping::LanePath& lane_path) {
  // CHECK(!lane_path.lane_ids().empty());
  // if (lane_path.lane_ids().back() != route_sections.destination().lane_id())
  // {
  return false;
  // }

  // constexpr double kEpsilon = 0.5;  // m.
  // const auto& last_sec = route_sections.back();
  // const auto section_info = psmm.map_ptr()->GetSectionById(last_sec.id);
  // if (!section_info) return false;
  // const double last_sec_length = section_info->topo_length();
  // return last_sec_length *
  //            std::abs(last_sec.end_fraction - lane_path.end_fraction()) <
  //        kEpsilon;
}

bool HasMapDictatedTurnSignal(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePath& lane_path,
                              TurnSignal* signal) {
  // const mapping::ElementId cur_lane_id = lane_path.front().lane_id();
  // const double cur_lane_fraction = lane_path.front().fraction();
  // const auto* cur_lane_proto_ptr = psmm.FindCurveLaneByIdOrNull(cur_lane_id);
  // if (cur_lane_proto_ptr == nullptr) return false;

  // const bool left_on =
  //     cur_lane_proto_ptr->has_require_left_turn_signal() &&
  //     (!cur_lane_proto_ptr->require_left_turn_signal().has_start_fraction()
  //     ||
  //      cur_lane_fraction >=
  //          cur_lane_proto_ptr->require_left_turn_signal().start_fraction())
  //          &&
  //     (!cur_lane_proto_ptr->require_left_turn_signal().has_end_fraction() ||
  //      cur_lane_fraction <=
  //          cur_lane_proto_ptr->require_left_turn_signal().end_fraction());
  // const bool right_on =
  //     cur_lane_proto_ptr->has_require_right_turn_signal() &&
  //     (!cur_lane_proto_ptr->require_right_turn_signal().has_start_fraction()
  //     ||
  //      cur_lane_fraction >=
  //          cur_lane_proto_ptr->require_right_turn_signal().start_fraction())
  //          &&
  //     (!cur_lane_proto_ptr->require_right_turn_signal().has_end_fraction() ||
  //      cur_lane_fraction <=
  //          cur_lane_proto_ptr->require_right_turn_signal().end_fraction());

  // if (left_on && right_on) {
  //   *signal = TURN_SIGNAL_EMERGENCY;
  //   return true;
  // } else if (left_on) {
  //   *signal = TURN_SIGNAL_LEFT;
  //   return true;
  // } else if (right_on) {
  //   *signal = TURN_SIGNAL_RIGHT;
  //   return true;
  // }

  return false;
}

std::optional<TurnSignal> ComputeDirectionSignal(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    Vec2d ego_pos) {
  const auto& lane_path = drive_passage.lane_path();
  const auto& extend_lane_path = drive_passage.extend_lane_path();
  st::mapping::ElementId preview_lane_id =
      lane_path.ArclengthToLanePoint(kPreviewDirectionDist * 1.6667).lane_id();
  ad_byd::planning::LaneConstPtr preview_lane_ptr =
      psmm.map_ptr()->GetLaneById(preview_lane_id);
  if (preview_lane_ptr == nullptr) {
    return std::nullopt;
  }
  if (preview_lane_ptr->turn_type() == TurnType::U_TURN) {
    return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
  }

  preview_lane_id =
      lane_path.ArclengthToLanePoint(kPreviewDirectionDist).lane_id();
  preview_lane_ptr = psmm.map_ptr()->GetLaneById(preview_lane_id);
  // There is no direction signal.
  if (preview_lane_ptr == nullptr) {
    return std::nullopt;
  }
  // If in lane_ramp
  if (preview_lane_ptr->type() == LaneType::LANE_RAMP) {
    double dist_to_turn = std::numeric_limits<double>::infinity();
    TurnType turn_type = TurnType::NO_TURN;
    int start_idx = -1;
    const auto& ego_lane_id = lane_path.ArclengthToLanePoint(0.0).lane_id();
    const auto& start_lane = psmm.map_ptr()->GetLaneById(ego_lane_id);
    for (const auto& extend_lane_id : extend_lane_path.lane_ids()) {
      start_idx++;
      if (extend_lane_id == ego_lane_id) {
        break;
      }
    }
    double ego_s = 0.0, ego_l = 0.0;
    if (!start_lane ||
        !start_lane->center_line().GetProjection(ego_pos, &ego_s, &ego_l)) {
      return std::nullopt;
    }
    Log2DDS::LogDataV2(
        "turn_light_debug",
        absl::StrCat("start_lane: ", ego_lane_id, " ego_s: ", ego_s));
    if (start_lane->turn_type() == TurnType::U_TURN) {
      dist_to_turn = 0.0;
      turn_type = TurnType::U_TURN;
    } else if (start_lane->turn_type() != TurnType::NO_TURN) {
      dist_to_turn = ego_s;
      turn_type = start_lane->turn_type();
      for (int i = start_idx - 1; i >= 0; --i) {
        const auto& lane_id = extend_lane_path.lane_id(i);
        const auto& lane = psmm.map_ptr()->GetLaneById(lane_id);
        if (!lane) continue;
        if (lane->turn_type() != start_lane->turn_type()) {
          break;
        }
        dist_to_turn += lane->topo_length();
      }
    } else {
      dist_to_turn = start_lane->center_line().length() - ego_s;
      for (int i = start_idx + 1; i < extend_lane_path.lane_ids().size(); ++i) {
        const auto& lane_id = extend_lane_path.lane_id(i);
        const auto& lane = psmm.map_ptr()->GetLaneById(lane_id);
        if (!lane) continue;
        if (lane->turn_type() != start_lane->turn_type()) {
          turn_type = lane->turn_type();
          break;
        }
        dist_to_turn += lane->topo_length();
        ++start_idx;
      }
    }
    Log2DDS::LogDataV2("turn_light_debug",
                       absl::StrCat("dist_to_turn: ", dist_to_turn,
                                    " turn_type: ", turn_type));
    if (dist_to_turn < kDirectionDist) {
      switch (turn_type) {
        case TurnType::NO_TURN:
          return std::nullopt;
        case TurnType::LEFT_TURN:
          return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
        case TurnType::RIGHT_TURN:
          return std::make_optional<TurnSignal>(TURN_SIGNAL_RIGHT);
        case TurnType::U_TURN:
          return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
      }
    }
    return std::nullopt;
  }
  // If in normal turn lane
  switch (preview_lane_ptr->turn_type()) {
    case TurnType::NO_TURN:
      return std::nullopt;
    case TurnType::LEFT_TURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
    case TurnType::RIGHT_TURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_RIGHT);
    case TurnType::U_TURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
  }
  return std::nullopt;
}

absl::StatusOr<TurnSignal> SignalFromRealLaneAside(
    const PlannerSemanticMapManager& psmm,
    const ad_byd::planning::Lane& lane_info,
    const ad_byd::planning::Lane& target_lane_info) {
  const auto target_sec_id = target_lane_info.section_id();
  const Vec2d target_end_point = target_lane_info.LerpPointFromFraction(1.0);

  double min_sqr_dist = DBL_MAX;
  Vec2d nearest_end_point;
  for (const auto& next_lane_id : lane_info.next_lane_ids()) {
    if (next_lane_id == target_lane_info.id()) continue;
    const auto next_lane_info = psmm.map_ptr()->GetLaneById(next_lane_id);
    if (!next_lane_info) break;
    if (next_lane_info->section_id() != target_sec_id) continue;

    if (!next_lane_info->IsVirtual() && !next_lane_info->IsBoundaryVirtual()) {
      const Vec2d next_end_point = next_lane_info->LerpPointFromFraction(1.0);
      const double sqr_dist = next_end_point.DistanceSquareTo(target_end_point);
      if (sqr_dist < min_sqr_dist) {
        min_sqr_dist = sqr_dist;
        nearest_end_point = next_end_point;
      }
    }
  }
  if (min_sqr_dist < DBL_MAX) {
    const Vec2d fork_point = lane_info.LerpPointFromFraction(1.0);
    return (nearest_end_point - fork_point)
                       .CrossProd(target_end_point - fork_point) > 0.0
               ? TURN_SIGNAL_LEFT
               : TURN_SIGNAL_RIGHT;
  }
  return absl::NotFoundError("No neighboring real lane found.");
}

SectionDirection ComputeSectionDirection(const PlannerSemanticMapManager& psmm,
                                         const mapping::LanePath& lane_path,
                                         uint64_t& sec_lane_id, int lane_idx) {
  const double preview_dist = 150.0;
  const auto far_lane_id =
      lane_path.ArclengthToLanePoint(preview_dist).lane_id();
  Log2DDS::LogDataV2("section_direction_debug",
                     absl::StrCat("far_lane_id: ", far_lane_id));
  for (int i = lane_idx; i < lane_path.lane_ids().size() - 1; ++i) {
    const auto lane_id = lane_path.lane_id(i);
    if (lane_id == far_lane_id) break;

    const auto& lane = psmm.map_ptr()->GetLaneById(lane_id);
    if (!lane) continue;

    const auto& cur_section =
        psmm.map_ptr()->GetSectionById(lane->section_id());
    if (!cur_section) continue;
    const auto outgoing_sections = cur_section->outgoing_sections();
    if (outgoing_sections.size() < 2) continue;
    Log2DDS::LogDataV2("section_direction_debug",
                       absl::StrCat("cur_section: ", lane->section_id()));

    const auto next_lane_id = lane_path.lane_id(i + 1);
    const auto& next_lane = psmm.map_ptr()->GetLaneById(next_lane_id);
    if (next_lane) {
      sec_lane_id = next_lane_id;
      return cur_section->get_outgoing_direction(next_lane->section_id());
    }
  }
  return SectionDirection::NONE;
}

absl::StatusOr<TurnSignal> ComputeForkSignal(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const std::optional<mapping::ElementId>& redlight_lane_id, double ego_v,
    Vec2d ego_pose, const TurnSignalResult& prev_turn_signal) {
  const auto& lane_path = drive_passage.lane_path();
  const auto& extend_lane_path = drive_passage.extend_lane_path();
  const double preview_dist = std::min(
      kPreviewMergeForkMaxDist,
      std::max(kPreviewMergeForkMinDist, ego_v * kPreviewMergeForkTime));
  const auto far_lane_id =
      lane_path.ArclengthToLanePoint(preview_dist).lane_id();
  const auto stop_lane_id = redlight_lane_id.has_value()
                                ? *redlight_lane_id
                                : mapping::kInvalidElementId;
  bool reached_far_lane = false;

  for (int i = 0; i < lane_path.lane_ids().size() - 1; ++i) {
    const auto lane_id = lane_path.lane_id(i);
    if (lane_id == far_lane_id) {
      reached_far_lane = true;
      break;
    }

    const auto next_lane_id = lane_path.lane_id(i + 1);
    if (next_lane_id == stop_lane_id) break;

    SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, lane_id);
    if (lane_info.next_lane_ids().size() < 2) continue;

    Log2DDS::LogDataV2(
        "split_light_debug",
        absl::StrCat("lane id: ", lane_id, " next_lane_id: ", next_lane_id));

    SMM_ASSIGN_LANE_OR_ERROR(next_lane_info, psmm, next_lane_id);
    const auto& map = psmm.map_ptr();
    const auto& lane = map->GetLaneById(lane_id);
    const auto& next_lane = map->GetLaneById(next_lane_id);
    if (lane && next_lane) {
      if (next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_NONE) {
        Log2DDS::LogDataV2("split_light_debug", "no split topolpgy");
        break;
      }
      if (next_lane->is_split_topo_modify()) {
        Log2DDS::LogDataV2("split_light_debug",
                           "ignore modified split topolpgy");
        break;
      }
      // turn off split light in 1to2 junction
      if (!(next_lane->junction_id() == 0) || next_lane->IsVirtual()) {
        Log2DDS::LogDataV2("split_light_debug",
                           "no need to split light in 1to2 junction");
        break;
      }
      // if (!next_lane->IsBoundaryVirtual() &&
      //     next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_NONE)
      //     {
      //   Log2DDS::LogDataV2("split_light_debug",
      //                      "next lane boundary not virtual line");
      //   break;
      // }
      constexpr double kQueryDistAfterForkPoint = 30.0;   // m.
      constexpr double kQueryDistbeforeForkPoint = 10.0;  // m.
      const double query_fraction = std::min(
          1.0, kQueryDistAfterForkPoint / next_lane_info.curve_length());
      const Vec2d next_query_point =
          next_lane_info.LerpPointFromFraction(query_fraction);
      const Vec2d fork_point = lane_info.LerpPointFromFraction(1.0);
      const double before_fork_query_fraction = std::max(
          0.0, 1.0 - kQueryDistbeforeForkPoint / lane_info.curve_length());
      const Vec2d before_fork_point =
          lane_info.LerpPointFromFraction(before_fork_query_fraction);
      double cross_prod_value = (fork_point - before_fork_point)
                                    .CrossProd(next_query_point - fork_point);
      double delta_angle = NormalizeAngle2D(fork_point - before_fork_point,
                                            next_query_point - fork_point);
      Log2DDS::LogDataV2("split_light_debug",
                         absl::StrCat("delta_angle: ", delta_angle));
      // LOG_INFO << "delta_angle: " << delta_angle
      //           << ", cross_prod_value: " << cross_prod_value;

      constexpr double kForkHeadingThreshold = 0.1;  // rad
      bool is_next_ramp = next_lane->type() == LaneType::LANE_RAMP ||
                          next_lane->type() == LaneType::LANE_EXIT ||
                          next_lane->type() == LaneType::LANE_DEC;

      if (is_next_ramp) {
        // return split light signal if other side of split is normal
        for (const auto& other_lane_id : lane_info.next_lane_ids()) {
          if (other_lane_id == next_lane_id) continue;
          const auto other_lane_info =
              psmm.map_ptr()->GetLaneById(other_lane_id);
          if (!other_lane_info) continue;
          if (other_lane_info->type() == LaneType::LANE_NORMAL) {
            return next_lane_id == other_lane_info->left_lane_id()
                       ? TURN_SIGNAL_LEFT
                       : TURN_SIGNAL_RIGHT;
          }
        }
        // if both sides of split are in ramp
        uint64_t temp_lane_id = 0;
        const auto ramp_direction =
            ComputeSectionDirection(psmm, lane_path, temp_lane_id, i);
        Log2DDS::LogDataV2("split_light_debug",
                           absl::StrCat("ramp_direction: ", ramp_direction));
        if (ramp_direction != SectionDirection::NONE) {
          if (std::abs(delta_angle) > kForkHeadingThreshold) {
            if (cross_prod_value > 0 &&
                ramp_direction == SectionDirection::LEFT) {
              return TURN_SIGNAL_LEFT;
            } else if (cross_prod_value <= 0 &&
                       ramp_direction == SectionDirection::RIGHT) {
              return TURN_SIGNAL_RIGHT;
            } else {
              return absl::CancelledError(
                  "Fork direction differ from section direction.");
            }
          } else {
            return absl::CancelledError("Fork angle too small in ramp.");
          }
        }
      }
      if (std::abs(delta_angle) > kForkHeadingThreshold) {
        if (cross_prod_value > 0 &&
            next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_LEFT) {
          return TURN_SIGNAL_LEFT;
        } else if (cross_prod_value <= 0 &&
                   next_lane->split_topology() ==
                       SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
          return TURN_SIGNAL_RIGHT;
        }
      }
    }

    if (next_lane_info.IsVirtual() ||
        (map->is_on_highway() && next_lane_info.IsBoundaryVirtual())) {
      const auto real_lane_or =
          SignalFromRealLaneAside(psmm, lane_info, next_lane_info);
      if (real_lane_or.ok()) {
        return *real_lane_or;
      }
    }
  }

  if (reached_far_lane &&
      prev_turn_signal.reason == TurnSignalReason::FORK_TURN_SIGNAL) {
    const double kBackwardDist = psmm.IsOnHighway() ? 30.0 : 15.0;
    const auto ego_lane_id = lane_path.ArclengthToLanePoint(0.0).lane_id();
    SMM_ASSIGN_LANE_OR_ERROR(ego_lane, psmm, ego_lane_id);
    // calculate the index of kBackwardDist behind ego_pose in extend_lane_path
    int ego_idx = 0, back_idx = 0;
    double ego_s = 0.0, ego_l = 0.0;
    ego_lane.center_line().GetProjection(ego_pose, &ego_s, &ego_l);
    for (int i = 0; i < extend_lane_path.lane_ids().size() - 1; ++i) {
      if (extend_lane_path.lane_id(i) == ego_lane_id) {
        ego_idx = i;
        break;
      }
    }
    double dist = kBackwardDist - ego_s;
    for (int i = ego_idx; i > 0; --i) {
      if (dist <= 0) {
        back_idx = i;
        break;
      }
      const auto back_lane_id = extend_lane_path.lane_id(i - 1);
      SMM_ASSIGN_LANE_OR_ERROR(back_lane, psmm, back_lane_id);
      dist -= back_lane.topo_length();
    }
    // if has split topo within kBackwardDist behind the ego_pose, return
    // previous turn light signal
    for (int i = back_idx; i < ego_idx; ++i) {
      const auto back_lane_id = extend_lane_path.lane_id(i);
      SMM_ASSIGN_LANE_OR_ERROR(back_lane, psmm, back_lane_id);
      if (back_lane.next_lane_ids().size() < 2) {
        continue;
      }
      const auto back_next_id = extend_lane_path.lane_id(i + 1);
      SMM_ASSIGN_LANE_OR_ERROR(back_next_lane, psmm, back_next_id);
      if (back_next_lane.split_topology() ==
              SplitTopology::TOPOLOGY_SPLIT_LEFT ||
          back_next_lane.split_topology() ==
              SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
        return prev_turn_signal.signal;
      }
    }
  }
  return absl::CancelledError("No need to turn fork signal.");
}

absl::StatusOr<TurnSignal> TurnOnMergeSignal(
    const PlannerSemanticMapManager& psmm,
    const ad_byd::planning::Lane& lane_info) {
  mapping::ElementId other_id = mapping::kInvalidElementId;
  for (const auto& interaction : lane_info.interactions()) {
    if (interaction.geometric_configuration != GeometricConfiguration::MERGE) {
      continue;
    }
    if (interaction.reaction_rule == ReactionRule::YIELD_MERGE ||
        interaction.reaction_rule == ReactionRule::YIELD) {
      other_id = mapping::ElementId(interaction.other_lane_id);
      break;
    }
  }
  if (other_id == mapping::kInvalidElementId) {
    return absl::NotFoundError(
        "Can not find the merging lane, please check the map.");
  }

  constexpr double kQueryDistBeforeMergePoint = 10.0;  // m.
  const double query_fraction =
      std::max(0.0, (lane_info.curve_length() - kQueryDistBeforeMergePoint) /
                        lane_info.curve_length());
  const Vec2d this_lane_pos = lane_info.LerpPointFromFraction(query_fraction);

  SMM_ASSIGN_LANE_OR_ERROR(other_lane_info, psmm, other_id);
  const Vec2d other_lane_tangent = other_lane_info.GetTangent(query_fraction);
  const Vec2d other_lane_pos =
      other_lane_info.LerpPointFromFraction(query_fraction);

  return other_lane_tangent.CrossProd(this_lane_pos - other_lane_pos) < 0.0
             ? TURN_SIGNAL_LEFT
             : TURN_SIGNAL_RIGHT;
}

absl::StatusOr<TurnSignal> ComputeMergeSignal(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double ego_v) {
  const double preview_dist = std::min(
      kPreviewMergeForkMaxDist,
      std::max(kPreviewMergeForkMinDist, ego_v * kPreviewMergeForkTime));
  const auto far_lane_id =
      lane_path.ArclengthToLanePoint(preview_dist).lane_id();
  const auto ego_lane_id = lane_path.ArclengthToLanePoint(0.0).lane_id();
  if (far_lane_id == ego_lane_id)
    return absl::CancelledError("No need to turn on merge signal.");

  for (const auto lane_id : lane_path.lane_ids()) {
    SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, lane_id);
    if (lane_info.merge_topology() ==
            ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_LEFT ||
        lane_info.merge_topology() ==
            ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_RIGHT) {
      return TurnOnMergeSignal(psmm, lane_info);
    }
    if (lane_id == far_lane_id) break;
  }
  return absl::CancelledError("No need to turn on merge signal.");
}

}  // namespace

// Planner 3.0 turn signal decider
TurnSignalResult DecideTurnSignal(
    const PlannerSemanticMapManager& psmm, TurnSignal pre_lane_change_signal,
    const mapping::LanePath& current_lane_path,
    const std::optional<mapping::ElementId>& redlight_lane_id,
    const LaneChangeStateProto& lc_state, const DrivePassage& drive_passage,
    const FrenetBox& ego_sl_box, const TurnSignalResult& planned_result,
    const PoseProto& ego_pose, bool if_continue_lc,
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    TurnSignal turn_type_signal, std::optional<SelectorOutput> selector_output,
    const TurnSignalResult& prev_turn_signal) {
  // Teleop override turn signal.
  // always false
  // if (ext_cmd_status.override_emergency_blinker_on ||
  //     (ext_cmd_status.override_left_blinker_on &&
  //      ext_cmd_status.override_right_blinker_on)) {
  //   return {TURN_SIGNAL_EMERGENCY, TELEOP_TURN_SIGNAL};
  // }
  // if (ext_cmd_status.override_left_blinker_on) {
  //   return {TURN_SIGNAL_LEFT, TELEOP_TURN_SIGNAL};
  // }
  // if (ext_cmd_status.override_right_blinker_on) {
  //   return {TURN_SIGNAL_RIGHT, TELEOP_TURN_SIGNAL};
  // }

  // if (route_signal == TURN_SIGNAL_LEFT) {
  //   return {route_signal, STARTING_TURN_SIGNAL};
  // }
  if (psmm.map_ptr() == nullptr) {
    LOG_ERROR << "psmm is null!";
    return {TURN_SIGNAL_NONE, TURN_SIGNAL_OFF};
  }
  // if current_lane_path is in ramp and has split section, print log
  if (!current_lane_path.IsEmpty() && psmm.map_ptr()->is_on_highway()) {
    uint64_t ramp_lane_id = 0;
    const auto ramp_direction =
        ComputeSectionDirection(psmm, current_lane_path, ramp_lane_id, 0);
    if (ramp_lane_id != 0 && ramp_direction != SectionDirection::NONE) {
      Log2DDS::LogDataV2("split_in_ramp",
                         absl::StrCat("ramp_lane id: ", ramp_lane_id));
    }
  }

  // selector decide turn light
  if (selector_output->turn_signal == TurnSignal::TURN_SIGNAL_LEFT ||
      selector_output->turn_signal == TurnSignal::TURN_SIGNAL_RIGHT) {
    Log2DDS::LogDataV2(
        "selector_turn_light_debug",
        absl::StrCat(
            "turn_signal: ", selector_output->turn_signal,
            "  turn_signal_reason:  ", selector_output->turn_signal_reason));
    return {selector_output->turn_signal, selector_output->turn_signal_reason};
  }

  constexpr double kStopSignalDistanceAhead = 30.0;  // m.
  if (HasReachedDestination(psmm, current_lane_path) &&
      current_lane_path.length() < kStopSignalDistanceAhead) {
    return {TURN_SIGNAL_RIGHT, PULL_OVER_TURN_SIGNAL};
  }

  // Map dictated turn signal
  TurnSignal map_signal;
  if (HasMapDictatedTurnSignal(psmm, current_lane_path, &map_signal)) {
    return {map_signal, MAP_DICTATED_TURN_SIGNAL};
  }

  // Direction signal
  const double ego_v =
      Hypot(ego_pose.vel_smooth().x(), ego_pose.vel_smooth().y());
  std::optional<TurnSignal> direction_signal = std::nullopt;
  direction_signal = ComputeDirectionSignal(
      psmm, drive_passage,
      Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()));
  if (direction_signal.has_value() && ego_v > 0.5) {
    return {direction_signal.value(), TURNING_TURN_SIGNAL};
  }

  // Lane change signal
  if (lc_state.stage() != LCS_NONE) {
    if (if_continue_lc) {
      return {lc_state.lc_left() ? TURN_SIGNAL_LEFT : TURN_SIGNAL_RIGHT,
              CONTINUE_LANE_CHANGE_SIGNAL};
    }
    return {lc_state.lc_left() ? TURN_SIGNAL_LEFT : TURN_SIGNAL_RIGHT,
            LANE_CHANGE_TURN_SIGNAL};
  }

  // Planned signal
  // TODO: Merge with lane change signal.
  // disable PrepareLcTurnSignal if pnp_infos.
  if (planned_result.signal != TurnSignal::TURN_SIGNAL_NONE) {
    return planned_result;
  }

  // Merge or fork preview signal
  const auto merge_signal = ComputeMergeSignal(psmm, current_lane_path, ego_v);
  if (merge_signal.ok()) {
    return {merge_signal.value(), MERGE_TURN_SIGNAL};
  }
  const auto fork_signal = ComputeForkSignal(
      psmm, drive_passage, redlight_lane_id, ego_v,
      Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()),
      prev_turn_signal);
  if (fork_signal.ok()) {
    return {fork_signal.value(), FORK_TURN_SIGNAL};
  }

  // Prepare lane change signal
  if (pre_lane_change_signal != TurnSignal::TURN_SIGNAL_NONE) {
    return {pre_lane_change_signal, PREPARE_LANE_CHANGE_TURN_SIGNAL};
  }

  return {TURN_SIGNAL_NONE, TURN_SIGNAL_OFF};
}

}  // namespace planning
}  // namespace st
