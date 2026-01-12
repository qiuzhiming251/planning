

#ifndef ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_
#define ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_

#include <string>

#include "absl/status/statusor.h"
#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"

#include "plan_common/async/thread_pool.h"

#include "object_manager/st_inference/decider_output.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "object_manager/st_inference/initializer_output.h"

#include "decider/initializer/initializer_input.h"
#include "decider/initializer/motion_search_output.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st::planning {

absl::StatusOr<MotionSearchOutput> SearchMotion(
    const MotionSearchInput& motion_input, ThreadPool* thread_pool);

// TODO: Refactor this function to just create initializer input.
absl::StatusOr<InitializerOutput> RunInitializer(
    const InitializerInput& initializer_input,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    SchedulerOutput* scheduler_output, DeciderOutput* decider_output,
    InitializerDebugProto* debug_proto, ThreadPool* thread_pool,
    std::map<std::string, bool>* obj_lead,
    st::planning::PlannerStatusProto::PlannerStatusCode* lc_status_code,
    LaneChangeStyleDeciderResultProto* lc_style_decider_result,
    TaskSafetyEvaluationProto* task_safety_evaluation_result,
    int* scene_cones_riding_line_frames_result);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_
