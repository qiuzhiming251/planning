#pragma once

#include "acc/acc_input.h"
#include "acc/acc_output.h"
#include "plan_common/planner_status.h"
#include "node_manager/task_runner/multi_tasks_cruise_planner_input.h"
#include "node_manager/task_runner/path_bounded_est_planner_output.h"
#include "node_manager/task_runner/planner_state.h"

namespace st::planning {

AccInput CreateAccInput(const MultiTasksCruisePlannerInput& input,
                        const PlannerState& planner_state);

PlannerStatus RunAccTask(const AccInput& input, AccOutput* acc_output);

PlannerStatus RunAccPlanner(const MultiTasksCruisePlannerInput& input,
                            const PlannerState& planner_state,
                            PathBoundedEstPlannerOutput* est_planner_output);
}  // namespace st::planning
