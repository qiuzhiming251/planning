

#ifndef ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_
#define ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/maps/lane_path_info.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
//#include "decider/decision_manager/tl_info.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/route_navi_info.h"
#include "plan_common/maps/route_sections_info.h"
#include "decider/scheduler/scheduler_input.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
namespace st::planning {

absl::StatusOr<std::vector<SchedulerOutput>> ScheduleMultiplePlanTasks(
    const MultiTasksSchedulerInput& input,
    const std::vector<LanePathInfo>& target_lp_infos, ThreadPool* thread_pool);

// bool ShouldSmoothRefLane(const TrafficLightInfoMap& tl_info_map,
//                          const DrivePassage& dp, bool prev_smooth_state);

absl::StatusOr<SchedulerOutput> MakeSchedulerOutput(
    const int& task_idx, const PlannerSemanticMapManager& psmm,
    const std::vector<LanePathInfo>& lp_infos, DrivePassage drive_passage,
    const LanePathInfo& lp_info, const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlannerObjectManager& obj_mgr,
    const ObjectHistoryManager& obj_history_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const mapping::LanePath& preferred_lane_path,
    const LaneChangeStateProto& prev_lc_state, bool borrow, bool should_smooth,
    bool planner_is_l4_mode, Behavior behavior, const int& target_lane_path_num,
    bool is_miss_navi = false, bool is_continuous_lc = false,
    const st::DriverAction::LaneChangeCommand lc_cmd_state =
        DriverAction::LC_CMD_NONE,
    ad_byd::planning::LcReason lc_reason = ad_byd::planning::LC_REASON_NONE,
    std::string leading_id = "",
    ad_byd::planning::PushDirection lc_push_dir =
        ad_byd::planning::PushDirection::Push_None,
    std::optional<bool> is_going_force_route_change_left = std::nullopt,
    const PushStatusProto* pre_push_status = nullptr,
    const absl::flat_hash_set<std::string>* stalled_objects = nullptr,
    const PausePushSavedOffsetProto* saved_offset = nullptr,
    const ad_byd::planning::ConstructionInfo* construction_info = nullptr,
    const LaneChangeSafetyInfo* pre_lane_change_safety_info = nullptr);

bool CheckRoadBoundaryBySInterval(
    const MultiTasksSchedulerInput& input, const SchedulerOutput& output,
    const ad_byd::planning::LaneSequencePtr& current_lane_seq,
    const ad_byd::planning::LaneSequencePtr& target_lane_seq);

enum class BlockLocation {
  BLOCK_CENTER,
  BLOCK_LEFT,
  BLOCK_RIGHT,
  BLOCK_LANE_ATTR,
};

struct Obstacle {
  std::string id;
  FrenetBox frenet_box;
  LaneAttrType lane_attr_type;
  bool operator==(const std::string& other) const { return id == other; }
};

struct NeighborObsInfo {
  std::string id{"none"};
  double s_max{0.0};
  double s_min{1000.0};
};

struct MyComparator {
  bool operator()(const st::planning::Obstacle& lhs,
                  const st::planning::Obstacle& rhs) const {
    return lhs.frenet_box.s_max < rhs.frenet_box.s_max;
  }
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_
