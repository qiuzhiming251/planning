

#ifndef ST_PLANNING_PLANNER_STATE
#define ST_PLANNING_PLANNER_STATE

#include <stdint.h>

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/time/time.h"
#include "decider/decision_manager/turn_signal_decider.h"
#include "object_manager/object_history.h"
#include "plan_common/ego_history.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/math/vec.h"
#include "plan_common/planner_status.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
#include "plan_common/selector_state.h"
#include "plan_common/type_def.h"
#include "planner/planner_manager/planner_defs.h"
#include "node_manager/task_runner/decision.h"
#include "modules/cnoa_pnc/planning/proto/acc.pb.h"
#include "modules/cnoa_pnc/planning/proto/alc.pb.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/lite_common.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {
// The cross iteration states of planner.
enum LaneChangeNotice {
  Notice_None = 0,
  Notice_Left_Waiting = 1,
  Notice_Right_Waiting = 2,
  Notice_Continuous_LC = 3,
  Notice_Miss_Navi = 4,
  Notice_LeTurn_Hard = 5,
  Notice_RiTurn_Hard = 6,
  Notice_Straight_Hard = 7,
  Notice_General_Hard = 8
};
struct PlannerState {
  LiteHeader header;

  st::planning::PlannerStatusProto_PlannerStatusCode planner_status_code =
      PlannerStatusProto::OK;
  st::planning::PlannerStatusProto_PlannerStatusCode
      prev_est_planner_status_code = PlannerStatusProto::OK;

  // int planner_frame_seq_num;

  // TODO: Rename this struct name, after delete tl_history manager.
  // YellowLightObservationsNew yellow_light_observations;

  // Global coordinates of previous planned trajectory.
  struct PosePoint {
    Vec2d pos;
    double theta = 0.0;
  };

  // std::vector<PosePoint> previous_trajectory_global;
  // std::vector<PosePoint> previous_past_trajectory_global;
  // Previous planned trajectory.
  TrajectoryProto previous_trajectory;

  std::optional<FrenetCoordinate> prev_ego_pose_frenet = std::nullopt;
  // Don't clear in cruise_task.
  // std::vector<PathPoint> previous_st_path_global_including_past;

  // Audio playing
  // absl::Time last_audio_alert_time;

  // Parking brake release time.
  // absl::Time parking_brake_release_time;

  LaneChangeStateProto lane_change_state;

  // Traffic light interface
  int tl_stop_interface = 0;
  TrafficLightIndicationInfoProto tl_ind_info;

  // Record how many loops were skipped between this and previous trajectory.
  // int planner_skip_counter = 0;

  // InputSeqNum input_seq_num;

  AutonomyStateProto previous_autonomy_state;

  // int previous_trajectory_plan_counter = 0;

  int version = 4;  // When snapshot upgrade, change version

  // double last_door_override_time = 0.0;

  // absl::Time current_time;

  // int64_t route_update_id = -1;

  void Clear();
  void ClearforAccEnterandExit();

  // Clear trajectory related planner states.
  // void ClearTrajectories();

  // Clear HMI info
  void ClearHMIInfo();

  void FromProto(const PlannerStateProto& proto);

  void ToProto(PlannerStateProto* proto) const;

  bool Upgrade();  // Used for snapshot version compatible

  bool operator==(const PlannerState& other) const;

  bool operator!=(const PlannerState& other) const { return !(*this == other); }

  std::string DebugString() const;

  // Freespace planner state.
  // FreespacePlannerStateProto freespace_planner_state;

  // ------------- Planner 3.0 -----------------
  mapping::LanePath prev_lane_path_before_lc;
  // State of decider
  DeciderStateProto decider_state;
  // State of initializer
  InitializerStateProto initializer_state;
  // Spacetime planner object
  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;

  SpeedFinderStateProto speed_finder_state;

  // ---------------- Planner 3.5 --------------
  // ---------- Multiple trajectories ----------

  // Previous target lane path starting from the position that is slightly
  // behind plan start point: (1) Insure successful projection as plan start
  // point might jump backwards due to resetting. (2) Keep decision
  // consistency.
  ad_byd::planning::MapType prev_success_map_type =
      ad_byd::planning::MapType::UNKNOWN_MAP;
  ad_byd::planning::MapType prev_map_type;
  mapping::LanePath prev_target_lane_path;
  double prev_length_along_route = std::numeric_limits<double>::max();
  double prev_max_reach_length = std::numeric_limits<double>::max();
  // For stabilization of drive passage stations across frames.
  mapping::LanePoint station_anchor;

  // For teleop lane change.
  mapping::LanePath preferred_lane_path;

  // // Route sections starting from the point that is several meters behind
  // plan
  // // start point.
  // RouteSections prev_route_sections;

  // Reference line smooth.
  bool prev_smooth_state = false;

  // MissionStageProto mission_stage;

  // std::deque<PlanTask> plan_task_queue;

  SelectorState selector_state;
  std::optional<bool> is_going_force_route_change_left = std::nullopt;

  // std::optional<TrajectoryEndInfoProto> prev_traj_end_info;
  std::optional<TrajectoryOptimizerStateProto>
      selected_trajectory_optimizer_state_proto;

  // std::optional<safe_invariance::SafeCheckStateProto> safe_check_state_proto;

  SmoothedReferenceLineResultMap smooth_result_map;

  // No need to publish for now.
  // TjaState tja_state;

  // bool stopped_at_route_end = false;

  // AsyncPlannerState async_planner_state;

  // bool previously_triggered_aeb = false;

  // speed error integrator about planner speed and display speed .
  double display_error_integrator = 0.0;

  // ----------------- L2 related ----------------
  // AssistPlanStateProto assist_plan_state;

  // std::shared_ptr<const PlannerSemanticMapManager> prev_low_freq_psmm =
  // nullptr;

  // std::string prev_online_map_id = kInvalidOnlineMapId;

  ObjectHistoryManager object_history_manager;
  EgoHistory ego_history;

  // HMI
  std::vector<std::string> stalled_cars;
  std::vector<std::string> in_queue_cars;

  // from ExternalCmdStatus
  ALCState alc_state = ALCState::ALC_OFF;
  AccState acc_state = AccState::ACC_OFF;
  AccPathCorridorType acc_path_corridor_type = AccPathCorridorType::NO_MAP;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  DriverAction::LaneChangeCommand lane_change_command =
      DriverAction::LC_CMD_NONE;
  std::optional<absl::Time> plc_prepare_start_time = std::nullopt;

  TurnSignalResult turn_signal_result;

  DriverAction::LaneChangeCommand last_lc_command = DriverAction::LC_CMD_NONE;
  LaneChangeReason last_lc_reason = LaneChangeReason::NO_CHANGE;
  std::shared_ptr<PlannerSemanticMapManager> planner_semantic_map_manager =
      nullptr;
  int lc_command_number = 0;
  int lc_cancel_delay_number = 0;
  int lc_keeping_delay_number = 0;
  int last_manual_lc_time = 0;
  struct ObjectStopTimeResult {
    // If object is stationary, the variable is valid and record the stop time
    // duration since lastest stop. Otherwise, the value is 0.0.
    double time_duration_since_stop;
    // Record the previous stop time duration, only refresh when object's motion
    // state changes from stationary to moving. Otherwise, remains constant.
    double previous_stop_time_duration;
    // Record the time duration for last move. If object is stationary, the
    // value remains constant. Otherwise, increases as time progresses.
    double last_move_time_duration;
    double last_time;
    // record brake & hazard light duration time
    double time_duration_brake_light;
    double previous_time_duration_brake_light;
    double time_duration_none_brake_light;
    double last_hazard_light_time;
    double accumulated_high_speed_duration;
  };
  ad_byd::planning::SpeedState speed_state;
  ad_byd::planning::ConstructionInfo construction_info;
  double spdlimit_curvature_gain_prev = 1.0;
  absl::flat_hash_map<std::string, ObjectStopTimeResult> object_stop_time_map;
  std::optional<NudgeObjectInfo> nudge_object_info = std::nullopt;
  bool has_triggered_lc_notice = false;
  int lc_notice_pub_counter = 0;
  LaneChangeNotice lc_notice = LaneChangeNotice::Notice_None;
  bool if_continuous_lc = false;
  ad_byd::planning::PushDirection lc_push_dir =
      ad_byd::planning::PushDirection::Push_None;
  LaneChangeStyle last_lc_style = LC_STYLE_CONSERVATIVE;
  std::vector<std::string> lc_lead_obj_ids;
  Decision decision;
  LargeVehicleAvoidStateProto pre_large_vehicle_avoid_state;
  PushStatusProto pre_push_status;
  LaneChangeSafetyInfo pre_lane_change_safety_info;
  PausePushSavedOffsetProto saved_offset;
  bool has_begin_route_change = false;
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
  std::optional<bool> begin_route_change_left = std::nullopt;
  GamingResultProto last_gaming_result;
  // truncated traj horizon of the obj in the back
  std::unordered_map<std::string, double> truncated_back_traj_horizon;
  double saved_lane_width = 0.0;
  int lane_width_invalid_cnt = 0;
  st::Behavior_FunctionId pre_funciton_id =
      st::Behavior_FunctionId::Behavior_FunctionId_NONE;
  double eie_time_counter_prev = 0.0;
  ad_byd::planning::EIEChoiceType eie_choice_type_prev =
      ad_byd::planning::EIEChoiceType::CHOICE_NONE;
  double prev_dist_to_junction = std::numeric_limits<double>::max();
  double cur_max_dist_to_junction = std::numeric_limits<double>::max();
};

}  // namespace st::planning

#endif  // ST_PLANNING_PLANNER_STATE
