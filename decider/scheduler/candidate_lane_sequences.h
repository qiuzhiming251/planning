
#ifndef AD_BYD_SCHEDULER_CANDIDATE_LANE_SEQUENCES_H
#define AD_BYD_SCHEDULER_CANDIDATE_LANE_SEQUENCES_H
#include <unordered_map>

#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/ld_lite_map.h"
#include "plan_common/maps/map.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/pnp_info.pb.h"

namespace ad_byd {
namespace planning {
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
    const st::LaneChangeStateProto lc_state_proto);

size_t GetNormalLanePosition(const MapPtr& map, const uint64_t lane_id,
                             const SectionInfo& section);

size_t GetNormalLaneSize(const MapPtr& map, const SectionInfo& section);

bool GetMatchLanePathByIds(
    const MapConstPtr& map, const std::vector<uint64_t>& lane_ids,
    const std::vector<std::vector<LaneConstPtr>>& all_lane_seqs,
    std::vector<LaneConstPtr>* match_lane_seq);

bool GetLaneSeqInfo(LaneConstPtr current_lane, const Vec2d start_point,
                    const double ego_v, const LaneSequencePtr& tgt_lane_seq,
                    const MapPtr& map, LaneSeqInfo* tgt_seq_info);

bool GetLaneSeqInfo(
    LaneConstPtr current_lane, const st::Behavior_FunctionId& function_id,
    const Vec2d start_point, const double ego_v,
    const LaneSequencePtr& tgt_lane_seq, const MapPtr& map,
    const LdLiteMapPtr& lite_map,
    std::vector<std::pair<uint64_t, double>>& nearest_section_lanes,
    LaneSeqInfo* tgt_seq_info);
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_SCHEDULER_CANDIDATE_LANE_SEQUENCES_H
