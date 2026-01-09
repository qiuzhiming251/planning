#ifndef FSD_LANE_SELECTOR
#define FSD_LANE_SELECTOR

#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/map.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "plan_common/path/path.h"
#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {

enum class SplitDirection {
  NotSpliting = 0,
  Left = 1,
  Mid = 2,
  Right,
};

enum class UturnFilterDirection {
  current = 0,
  left = 1,
  right = 2,
};

bool FsdCityScheduleCandidateLaneSequences(
    const Vec2d &ego_pos, const double ego_heading, const MapPtr &map,
    const LaneConstPtr &start_lane, double ego_v,
    const st::LaneChangeStateProto &lane_change_state,
    const ad_byd::planning::LaneSequencePtr &pre_target_lane_seq,
    const ad_byd::planning::LaneSequencePtr &pre_target_lane_seq_before_lc,
    std::vector<LaneSequencePtr> *candidate_lane_seqs,
    const st::DriverAction::LaneChangeCommand &new_lc_cmd,
    const BehaviorCommand &intention_dir);
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
    const ad_byd::planning::EIEChoiceType &eie_choice_type);

/// @brief get optimal lane saquence start from inputting lane
LaneSequencePtr GetOptimalLaneSeq(
    const Vec2d &start_point, const double start_theta, const MapPtr &map,
    const LaneConstPtr &lane, double ego_v, const bool &navi_is_priority,
    const bool &split_is_navi_priority, const uint64_t lane_id,
    const ad_byd::planning::LaneSequencePtr pre_lane_seq = nullptr);

bool GetStaticCost(
    const Vec2d &start_point, const MapPtr &map, const LaneConstPtr &lane,
    const bool &navi_is_priority, const bool &split_is_navi_priority,
    const uint64_t lane_id,
    std::vector<std::pair<LaneSequencePtr, double>> &cost_static);

bool LaneseqFilter(
    const MapPtr &map,
    const std::vector<std::pair<LaneSequencePtr, double>> &cost_static,
    std::vector<LaneConstPtr> &pre_lane_seq_in_range,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all,
    std::vector<std::string> &logstrs);

void UpdateSignInLaneseq(
    const Vec2d &start_point,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all);

bool GetHumanlikeCost(
    const Vec2d &start_point, const double start_theta, const MapPtr &map,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all);

void GoStraightCost(
    const Vec2d &start_point, const double start_theta, const MapPtr &map,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all);

void TurnLeftCost(
    const Vec2d &start_point, const MapPtr &map,
    const SerialJunctionType &navi_turn, const uint64_t junction_section_id,
    const double &dist_btw_junc,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all);

bool UTurnCost(
    const MapPtr &map,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all);

void TurnRightCost(
    const Vec2d &start_point, const MapPtr &map,
    const SerialJunctionType &navi_turn, const uint64_t junction_section_id,
    std::vector<std::pair<LaneSequencePtr, std::pair<double, int>>> &cost_all);

double GetTurnDistance(const MapPtr &map, const LaneSequencePtr &laneseq,
                       const LaneConstPtr &turn_lane);

double GetAverageKappa(const LaneConstPtr &lane);

double GetAverageKappaV2(const LaneConstPtr &lane);

bool CalcuLaneIndex(const MapPtr &map, LaneConstPtr &pre_lane,
                    LaneConstPtr &mid_lane, LaneConstPtr &suc_lane,
                    int &pre_left_index, int &pre_right_index,
                    int &suc_left_index, int &suc_right_index);

uint64_t CalcuSplitLaneId(const Vec2d &start_point,
                          const LaneSequencePtr &lane_seq, double &accum_s);

SplitDirection CalcuSplitDirection(const MapPtr &map, uint64_t split_lane_id,
                                   uint64_t extend_split_lane_id);

SplitDirection CalcuSplitDirectionByNavigation(const MapPtr &map,
                                               LaneConstPtr &left_lane,
                                               LaneConstPtr &right_lane);

LaneSequencePtr CalcuExtendSplitLaneSequence(
    const Vec2d &start_point, const double start_theta, const MapPtr &map,
    const LaneConstPtr &lane, double ego_v, const bool &navi_is_priority,
    const bool &split_is_navi_priority, uint64_t &split_lane_id,
    uint64_t &extend_split_lane_id,
    const ad_byd::planning::LaneSequencePtr current_seq,
    const ad_byd::planning::LaneSequencePtr pre_lane_seq = nullptr);

double CalcuDistToNaviEnd(const MapPtr &map, const LaneSequencePtr &laneseq,
                          const Vec2d start_point);

BehaviorCommand CalcuAheadDirection(
    const MapPtr &map, const Vec2d start_point, const double ego_v,
    const ad_byd::planning::LaneSequencePtr current_seq,
    const ad_byd::planning::LaneSequencePtr left_seq,
    const ad_byd::planning::LaneSequencePtr right_seq);

std::set<UturnFilterDirection> UturnFilter(const MapPtr &map,
                                           const LaneConstPtr &cur_lane);

bool IsNonMotorLane(const MapPtr &map, const LaneConstPtr &lane);
}  // namespace planning
}  // namespace ad_byd
#endif  // FSD_CITY_LANE_SELECTOR
