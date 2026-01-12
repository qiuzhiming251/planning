

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_H_

#include "absl/status/statusor.h"

#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"

#include "plan_common/planner_status.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/util/planner_status_macros.h"

#include "object_manager/st_inference/initializer_output.h"
#include "object_manager/st_inference/st_path_planner_input.h"
#include "object_manager/st_inference/st_path_planner_output.h"
#include "object_manager/st_inference/trajectory_optimizer_input.h"
#include "object_manager/st_inference/trajectory_optimizer_output.h"

namespace st {
namespace planning {

absl::StatusOr<TrajectoryOptimizerOutput> OptimizeTrajectory(
    const TrajectoryOptimizerInput& input,
    TrajectoryOptimizerDebugProto* optimizer_debug, bool is_compare_weight,
    ThreadPool* thread_pool);

PlannerStatus RunOptimizeTrajectry(StPathPlannerInput& st_path_input,
                                   StPathPlannerOutput* st_path_out,
                                   TrajectoryOptimizerOutput& opt_output,
                                   ThreadPool* thread_pool);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_H_
