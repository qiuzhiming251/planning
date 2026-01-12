

#ifndef ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_
#define ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"

#include "modules/cnoa_pnc/planning/proto/acc.pb.h"
#include "modules/cnoa_pnc/planning/proto/alc.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_debug.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"

#include "plan_common/type_def.h"
#include "plan_common/planner_status.h"
#include "plan_common/selector_state.h"
#include "plan_common/plc_internal_result.h"
#include "plan_common/maps/route_sections.h"
#include "object_manager/st_inference/est_planner_output.h"
#include "planner/planner_manager/planner_defs.h"
#include "decider/scheduler/target_lane_path_filter.h"
#include "planner_state.h"

namespace st::planning {

struct PathBoundedEstPlannerOutput {
  std::vector<PlannerStatus> est_status_list;
  std::vector<EstPlannerOutput> est_planner_output_list;
  std::vector<EstPlannerDebug> est_planner_debug_list;

  // EstPlannerDebug async_high_freq_debug;
  std::optional<RouteTargetInfo> route_target_info;  // For async planner.

  PnpTop1History pnp_top1_history;
  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  std::optional<SelectorOutput> selector_output = std::nullopt;
  DriverAction::LaneChangeCommand input_lc_cmd = DriverAction::LC_CMD_NONE;
  LcFeasibility lc_unable_reason = LcFeasibility::FEASIBILITY_OK;
  std::optional<PlcInternalResult> plc_result = std::nullopt;  // For teleop lc.
  ALCState alc_state = ALCState::ALC_OFF;
  AccState acc_state = AccState::ACC_OFF;
  // Optimizer Auto Tuning
  // AutoTuningDataProto auto_tuning_data;

  // ObjectsPredictionProto speed_considered_objects_prediction;

  // int path_start_relative_index = 0;

  // std::shared_ptr<const PlannerSemanticMapManager> low_freq_psmm = nullptr;
  // std::shared_ptr<const DrivingMapTopo> driving_map_topo = nullptr;

  // Alcc only.
  // LaneChangeDirection lc_direction;
  // mapping::LanePath origin_lane_path;
  // mapping::LanePath target_lane_path;
  // std::string online_map_id = kInvalidOnlineMapId;

  // Hmi
  std::optional<NudgeObjectInfo> nudge_object_info;
  // Traffic light interface
  int tl_stop_interface = 0;
  TrafficLightIndicationInfoProto tl_ind_info;
  ad_byd::planning::SpeedState speed_state;
  bool icc_lc_enable = false;
  bool if_lc_to_congestion = false;
  bool has_passed_this_junction = false;
  bool if_cancel_lc = false;
  bool if_allow_cancel_lc = true;
  LaneChangeNotice lc_notice = LaneChangeNotice::Notice_None;
  bool if_continuous_lc = false;
  ad_byd::planning::PushDirection lc_push_dir =
      ad_byd::planning::PushDirection::Push_None;
  LaneChangeStyle last_lc_style = LC_STYLE_CONSERVATIVE;
  std::optional<FrenetCoordinate> start_point_frenet = std::nullopt;
  std::optional<FrenetCoordinate> ego_pose_frenet = std::nullopt;
  EgoFrame curr_selected_ego_frame;
  TurnSignal turn_type_signal = TURN_SIGNAL_NONE;
  double spdlimit_curvature_gain = 1.0;
  // Acc related output.
  PathSlBoundary acc_sl_boundary;
  AccPathCorridorType acc_path_corridor_type = AccPathCorridorType::NO_MAP;
  // Lane change style decider result
  absl::flat_hash_map<std::tuple<st::LaneChangeStage, bool, bool>,
                      LaneChangeStyleDeciderResultProto>
      lc_style_decider_results;
  // Multi tasks safety evaluation result
  absl::flat_hash_map<std::tuple<st::LaneChangeStage, bool, bool>,
                      TaskSafetyEvaluationProto>
      task_safety_evaluation_results;
  // Frames result for the scene of cones riding line
  absl::flat_hash_map<std::tuple<st::LaneChangeStage, bool, bool>, int>
      scene_cones_riding_line_frames_results;
  double eie_time_counter_prev = 0.0;
  ad_byd::planning::EIEChoiceType eie_choice_type_prev =
      ad_byd::planning::EIEChoiceType::CHOICE_NONE;
  double prev_dist_to_junction = std::numeric_limits<double>::max();
  double cur_max_dist_to_junction = std::numeric_limits<double>::max();
  double target_traffic_velocity = 0.0;
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_
