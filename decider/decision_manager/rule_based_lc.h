#ifndef RULE_BASED_LC_H_
#define RULE_BASED_LC_H_

#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/type_def.h"
#include "plan_common/maps/map.h"
#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st::planning {
using MapConstPtr = ad_byd::planning::MapConstPtr;
using LaneConstPtr = ad_byd::planning::LaneConstPtr;
using LaneSequencePtr = ad_byd::planning::LaneSequencePtr;
using LaneType = ad_byd::planning::LaneType;

struct TargetLaneInfo {
  ad_byd::planning::LaneSequencePtr target_lane_seq = nullptr;
  ad_byd::planning::LcReason lc_reason =
      ad_byd::planning::LcReason::LC_REASON_NONE;
  ad_byd::planning::BehaviorCommand lc_command =
      ad_byd::planning::BehaviorCommand::Command_Invalid;
};

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
    const std::optional<double> cruising_speed_limit, std::string leading_id);

TargetLaneInfo SelectLaneNavi(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos, bool is_miss_navi,
    const ApolloTrajectoryPointProto *plan_start_point);

TargetLaneInfo SelectLaneOverTake(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos,
    const std::optional<double> cruising_speed_limit,
    const PlannerObjectManager &obj_mgr,
    ApolloTrajectoryPointProto plan_start_point, const std::string &leading_id);

TargetLaneInfo SelectLaneBlocker(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos,
    const ApolloTrajectoryPointProto *plan_start_point,
    const VehicleParamsProto *vehicle_param,
    const PlannerSemanticMapManager *psmm,
    const std::optional<Behavior_FunctionId> function_id,
    const std::optional<int> tunnel_status, const PlannerObjectManager &obj_mgr,
    const LaneSequencePtr &pre_target_laneseq,
    const LaneChangeStateProto *lane_change_state,
    const LaneChangeReason &lane_change_reason,
    const DriverAction::LaneChangeCommand &lane_change_command,
    const LaneConstPtr &nearest_lane);

TargetLaneInfo SelectLaneCenter(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos,
    const ApolloTrajectoryPointProto plan_start_point);

TargetLaneInfo SelectLaneMerge(
    const std::vector<ad_byd::planning::LaneSeqInfo> &lane_seq_infos,
    const ad_byd::planning::MapPtr &map, Vec2d ego_pos,
    ApolloTrajectoryPointProto plan_start_point);

bool IfOnSameLaneSequence(const ad_byd::planning::MapPtr &map,
                          const ad_byd::planning::LaneConstPtr &start,
                          const ad_byd::planning::LaneConstPtr &target);

bool IfOnSameLaneSequence(const ad_byd::planning::MapPtr &map,
                          const ad_byd::planning::LaneConstPtr &start,
                          const ad_byd::planning::LaneConstPtr &target,
                          std::unordered_set<uint64_t> &lanes);

bool IfLaneSeqWillMergeCurLane(
    const ad_byd::planning::LaneSequencePtr &lane_seq,
    const ad_byd::planning::LaneConstPtr &current_lane,
    const Vec2d &start_point, const ad_byd::planning::MapPtr &map,
    double &merge_dist);

bool IfSectionSizeLargerThanTwo(
    double check_range, const ad_byd::planning::MapPtr &map,
    const ad_byd::planning::NaviPosition &navi_start);

bool IfLcByVariableTurn(
    const ad_byd::planning::LaneSequencePtr &cur_lane_sequence,
    const ad_byd::planning::LaneSequencePtr &left_lane_seq,
    const ad_byd::planning::LaneSequencePtr &right_lane_seq,
    const ad_byd::planning::LaneSeqInfo &cur_lane_seq_info, int &lc_direction,
    double &dist_to_target_lane, LaneType lane_type,
    const ad_byd::planning::MapPtr &map);

bool IfLcByBusLane(const ad_byd::planning::LaneSequencePtr &cur_lane_sequence,
                   const ad_byd::planning::LaneSequencePtr &left_lane_seq,
                   const ad_byd::planning::LaneSequencePtr &right_lane_seq,
                   const ad_byd::planning::LaneSeqInfo &cur_lane_seq_info,
                   int &lc_direction, const ad_byd::planning::MapPtr &map);

}  // namespace st::planning

#endif  // RULE_BASED_LC_H_