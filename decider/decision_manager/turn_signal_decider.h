

#ifndef ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_

//#include "plan_common/external_command_info.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/math/frenet_common.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
//#include "modules/cnoa_pnc/planning/proto/pnp_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/route_sections.h"
#include "object_manager/st_inference/selector_output.h"

namespace st {
namespace planning {

struct TurnSignalResult {
  TurnSignal signal = TurnSignal::TURN_SIGNAL_NONE;
  TurnSignalReason reason = TurnSignalReason::TURN_SIGNAL_OFF;
};

// Planner 3.0
TurnSignalResult DecideTurnSignal(
    const PlannerSemanticMapManager& psmm, TurnSignal pre_lane_change_signal,
    const mapping::LanePath& current_lane_path,
    const std::optional<mapping::ElementId>& redlight_lane_id,
    const LaneChangeStateProto& lc_state, const DrivePassage& drive_passage,
    const FrenetBox& ego_sl_box, const TurnSignalResult& planned_result,
    const PoseProto& ego_pose, bool if_continue_lc,
    const ad_byd::planning::LaneSeqInfoPtr& lane_seq_info,
    TurnSignal turn_type_signal, std::optional<SelectorOutput> selector_output,
    const TurnSignalResult& prev_turn_signal);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_
