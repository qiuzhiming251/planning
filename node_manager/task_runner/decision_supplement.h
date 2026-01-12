#ifndef ST_PLANNING_DECISION_SUPPLEMENT
#define ST_PLANNING_DECISION_SUPPLEMENT
#include <chrono>
#include <map>
#include <cereal/cereal.hpp>

#include "plan_common/type_def.h"
namespace st::planning {

// some system default configs
static constexpr double DEFAULT_LANE_WIDTH = 3.75;  // unit m

class DecisionSupplement {
 public:
  virtual void Reset() = 0;
};
typedef std::shared_ptr<DecisionSupplement> DecisionSupplementPtr;

struct LaneChangeProgressInfo {
  // Lane change progress bar info
  double cur_lane_width = DEFAULT_LANE_WIDTH;
  double tgt_lane_width = DEFAULT_LANE_WIDTH;
  double half_cur_lane_width = DEFAULT_LANE_WIDTH * 0.5;
  double half_tgt_lane_width = DEFAULT_LANE_WIDTH * 0.5;
  double dis_to_current_lane = 0.0;
  double dis_to_target_lane = DEFAULT_LANE_WIDTH;
  double lat_offset_factor = 1.0;
  double lc_completion_factor = 0.0;
};

struct LaneChangeSupplement : public DecisionSupplement {
  double last_lc_finish_time = 0.0;
  double pre_start_point_l = 0.0;
  double intention_t = 0.0;
  // emergency intent
  // BehaviorCommand emergency_last_intent = Command_Invalid;
  double emergency_intent_t = 0.0;
  std::string emergency_blocker_id;

  // blocker intent
  bool blocker_lc_intent = false;
  double blocker_intent_t = 0.0;
  double ego_stand_still_time = 0.0;
  std::string last_blocker_id;
  double blocker_lc_speed_limit = 0.0;
  // merge intent
  double merge_intent_t = 0.0;
  // navi intent
  double navi_intent_t = 0.0;
  // overtake intent
  double overtake_motive_t = 0.0;
  double overtake_enable_t = 0.0;
  bool is_overtake_VRU = false;
  std::string current_front_UU_id = "";
  double left_speed_gain = DBL_MAX;
  double right_speed_gain = DBL_MAX;

  // center intent
  double center_intent_t = 0.0;
  //
  double left_lane_efficiency = 0.0;
  double right_lane_efficiency = 0.0;
  double left_lane_efficiency_new = 0.0;
  double right_lane_efficiency_new = 0.0;
  //
  double lc_cancel_check_t = 0.0;
  double lc_finish_check_t = 0.0;
  double prepare_feasible_check_t = 0.0;
  //
  bool lc_offset_reset_flag = false;
  double lc_offset_lat_v = 0.0;
  bool active_offset_compensation = false;
  int offset_compensation_count = 0;
  double last_start_l = 0.0;

  // push
  // BehaviorCommand target_lane_command = Command_Invalid;
  // LaneChangeType lc_type = LC_TYPE_NONE;
  bool lc_push_status = false;
  double lc_push_t = 0.0;
  bool push_start = false;
  bool push_cancel = false;

  bool push_init = false;
  // hold
  bool lc_hold_status = false;
  bool lc_hold_offset_init = false;
  bool lc_resume_offset_init = true;
  double lc_hold_target_offset = 0.0;
  double lc_hold_lat_offset = 0.0;
  double lc_hold_t = 0.0;
  bool hold_start = false;
  bool hold_to_lc = false;
  int lc_direction_last = 0;
  //
  bool if_force_lc = false;
  bool use_lc_qp_config = false;
  // for lka
  bool speed_disable_lc = false;
  double overtake_cone_t = 0.0;
  double center_cone_t = 0.0;
  //
  double min_left_navi_distance = DBL_MAX;  // min left navi distance within
                                            // left, right and current lane seq
                                            // (except seq with non navi ramp)
  // new lc structure
  int overtake_intention = 0;  // for preference lc 1:left  -1:right   0:not lc
  bool if_overtake = false;    // overtake or not
  // time of ego slow speed cause of front leader car
  double ego_low_speed_time_left = 0.0;
  double ego_low_speed_time_right = 0.0;
  double ego_low_speed_threshold = 0.0;
  double preference_lc_intent_t = 0.0;
  double preference_lc_start_time = 0.0;
  bool is_blocker_lc = false;
  int is_blocker_lc_count = 0;
  bool if_check_line_type = true;
  bool left_can_overtake = false;  // can overtake front low speed car or not
  bool right_can_overtake = false;
  bool is_strategy_conflict = false;
  //
  double available_dist = DBL_MAX;
  // TrafficFlowType traffic_flow_type_max = UNKNOWN_FLOW;

  // Lane change progress bar info
  LaneChangeProgressInfo lc_progress = {};

  void Reset() override {
    // emergency intent
    // emergency_last_intent = Command_Invalid;
    emergency_intent_t = 0.0;
    // blocker intent
    blocker_intent_t = 0.0;
    ego_stand_still_time = 0.0;
    blocker_lc_speed_limit = 0.0;

    // merge intent
    merge_intent_t = 0.0;
    // navi intent
    navi_intent_t = 0.0;
    // overtake intent
    overtake_motive_t = 0.0;
    overtake_enable_t = 0.0;
    // center intent
    center_intent_t = 0.0;
    //
    left_lane_efficiency = 0.0;
    right_lane_efficiency = 0.0;
    left_lane_efficiency_new = 0.0;
    right_lane_efficiency_new = 0.0;
    current_front_UU_id = "";
    //
    lc_cancel_check_t = 0.0;
    lc_finish_check_t = 0.0;
    // lc_offset
    lc_offset_reset_flag = true;
    lc_offset_lat_v = 0.0;
    active_offset_compensation = false;
    offset_compensation_count = 0;

    // push
    // target_lane_command = Command_Invalid;
    // lc_type = LC_TYPE_NONE;
    lc_push_status = false;
    lc_push_t = 0.0;
    push_start = false;
    push_cancel = false;
    // hold
    lc_hold_status = false;
    lc_hold_offset_init = false;
    lc_resume_offset_init = true;
    lc_hold_target_offset = 0.0;
    lc_hold_lat_offset = 0.0;
    lc_hold_t = 0.0;
    hold_start = false;
    hold_to_lc = false;
    //
    if_force_lc = false;
    // for lka
    speed_disable_lc = false;
    overtake_cone_t = 0.0;
    center_cone_t = 0.0;
    //
    min_left_navi_distance = DBL_MAX;
    // traffic_flow_type_max = UNKNOWN_FLOW;
    // new lc structure
    overtake_intention = 0;
    if_overtake = false;
    ego_low_speed_time_left = 0.0;
    ego_low_speed_time_right = 0.0;
    is_blocker_lc = false;
    is_blocker_lc_count = 0;
    left_can_overtake = false;
    right_can_overtake = false;
    preference_lc_intent_t = 0.0;
    available_dist = DBL_MAX;
    is_strategy_conflict = false;

    // Lane change progress bar info
    lc_progress = LaneChangeProgressInfo{};
  }
};
}  // namespace st::planning

#endif  // ST_PLANNING_DECISION_SUPPLEMENT
