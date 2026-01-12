

#ifndef ONBOARD_PLANNER_PLAN_MULTI_TASKS_CRUISE_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_MULTI_TASKS_CRUISE_PLANNER_H_

#include "plan_common/async/thread_pool.h"
#include "plan_common/planner_status.h"
#include "node_manager/task_runner/multi_tasks_cruise_planner_input.h"
#include "node_manager/task_runner/path_bounded_est_planner_output.h"
#include "node_manager/task_runner/planner_state.h"

namespace st::planning {
using PushDirection = ad_byd::planning::PushDirection;

PlannerStatus RunMultiTasksCruisePlanner(
    const MultiTasksCruisePlannerInput& input,
    const PlannerState& planner_state, PathBoundedEstPlannerOutput* output,
    ThreadPool* thread_pool);

bool IsNonEmptyPlannerResult(const PlannerStatus& status);

void ParseSchedulerOutputToPlannerState(const SchedulerOutput& scheduler_output,
                                        PlannerState* planner_state,
                                        const ad_byd::planning::MapPtr& map);

void ParseEstPlannerOutputToPlannerState(
    const EstPlannerOutput& est_planner_output, PlannerState* planner_state);

PushDirection PushIntention(const ApolloTrajectoryPointProto plan_start_point,
                            const PlannerObjectManager* obj_mgr,
                            const std::vector<SchedulerOutput>& tasks,
                            const PlannerState& planner_state,
                            const bool& if_miss_navi_secnario);

PushDirection ExtractSchedulerPushIntention(
    const std::vector<SchedulerOutput>& multi_tasks);

void UpdatePreLaneChangeSafetyInfo(
    const PathBoundedEstPlannerOutput* planner_output,
    LaneChangeSafetyInfo* pre_lane_change_safety_info);

void UpdatePrePushStatus(const PathBoundedEstPlannerOutput* planner_output,
                         PushStatusProto* pre_push_status);

void UpdateSavedOffset(const PathBoundedEstPlannerOutput* planner_output,
                       PausePushSavedOffsetProto* saved_offset);

void UpdateIsConstructionScene(
    const PathBoundedEstPlannerOutput* planner_output,
    ad_byd::planning::ConstructionInfo& construction_info);

bool IsMergeBetweenLaneSeqs(
    const ad_byd::planning::MapPtr &map, const Vec2d &start_point,
    const ad_byd::planning::LaneSequencePtr first_lane_seq,
    const ad_byd::planning::LaneSequencePtr second_lane_seq);

void CalcuMinMergeLcNum(const ad_byd::planning::MapPtr &map, const Vec2d &start_point,
    std::vector<LanePathInfo> &lp_infos,int &cur_navi_lc_num);
}  // namespace st::planning

#endif  // ONBOARD_PLANNER_PLAN_MULTI_TASKS_CRUISE_PLANNER_H_
