

#ifndef ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_

#include <string>
#include <vector>

#include "absl/time/time.h"
#include "plan_common/async/thread_pool.h"
#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/pnp_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"

#include "plan_common/planner_status.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/plan_start_point_info.h"
#include "plan_common/util/decision_info.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

#include "object_manager/object_history.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/st_inference/decider_output.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "object_manager/st_inference/st_path_planner_input.h"
#include "object_manager/st_inference/st_path_planner_output.h"
#include "object_manager/spacetime_planner_object_trajectories.h"

#include "plan_common/ref_line/smooth_reference_line_result.h"

namespace st::planning {
using PushDirection = ad_byd::planning::PushDirection;
using LcReason = ad_byd::planning::LcReason;

// Should not contain pointers since the actual object might have been destroyed
// and cannot be used in speed finder in async planner.

PlannerStatus RunStPathPlanner(StPathPlannerInput& input,
                               StPathPlannerOutput* out,
                               ThreadPool* thread_pool);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_
