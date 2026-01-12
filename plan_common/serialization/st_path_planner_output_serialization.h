#ifndef _PLAN_COMMON_SERIALIZATION_st_path_planner_output_SERIALIZE_H_
#define _PLAN_COMMON_SERIALIZATION_st_path_planner_output_SERIALIZE_H_

#include "object_manager/st_inference/st_path_planner_output.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/serialization/base_serialization.h"
#include "plan_common/serialization/constraint_serialization.h"
#include "plan_common/serialization/planner_state_serialization.h"
#include "plan_common/serialization/st_path_planner_input_serialize.h"
#include "plan_common/speed/st_speed/open_loop_speed_limit.h"

#include <cereal/types/map.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>

namespace cereal {

template <typename Archive>
void serialize(Archive& ar,
               st::planning::StPathPlannerOutput& st_path_planner_output) {
  ar(CEREAL_NVP(st_path_planner_output.initializer_output));
  ar(CEREAL_NVP(st_path_planner_output.scheduler_output));
  ar(CEREAL_NVP(st_path_planner_output.path));
  ar(CEREAL_NVP(st_path_planner_output.st_path_points));
  ar(CEREAL_NVP(st_path_planner_output.constraint_manager));
  ar(CEREAL_NVP(st_path_planner_output.follower_set));
  ar(CEREAL_NVP(st_path_planner_output.leader_set));
  ar(CEREAL_NVP(st_path_planner_output.follower_max_decel));
  ar(CEREAL_NVP(st_path_planner_output.unsafe_object_ids));
  ar(CEREAL_NVP(st_path_planner_output.nudge_object_info));
  ar(CEREAL_NVP(st_path_planner_output.leading_trajs));
  ar(CEREAL_NVP(st_path_planner_output.leading_id));
  ar(CEREAL_NVP(st_path_planner_output.st_planner_object_traj));
  //   ar(CEREAL_NVP(st_path_planner_output.initializer_debug_proto));
  //   ar(CEREAL_NVP(st_path_planner_output.optimizer_debug_proto));
  ar(CEREAL_NVP(st_path_planner_output.decider_state));
  ar(CEREAL_NVP(st_path_planner_output.initializer_state));
  ar(CEREAL_NVP(st_path_planner_output.traj_points));
  ar(CEREAL_NVP(st_path_planner_output.captain_traj_points));
  ar(CEREAL_NVP(st_path_planner_output.nudge_info));
  ar(CEREAL_NVP(st_path_planner_output.trajectory_optimizer_state_proto));
  ar(CEREAL_NVP(st_path_planner_output.obs_leading));
  ar(CEREAL_NVP(st_path_planner_output.lc_status_code));
  ar(CEREAL_NVP(st_path_planner_output.is_init_follow_scene));
  ar(CEREAL_NVP(st_path_planner_output.lc_lead_obj_id));
  ar(CEREAL_NVP(st_path_planner_output.speed_response_style));
  ar(CEREAL_NVP(st_path_planner_output.pre_large_vehicle_avoid_state));
  ar(CEREAL_NVP(st_path_planner_output.safety_check_failed_reason));
  ar(CEREAL_NVP(st_path_planner_output.saved_offset));
  ar(CEREAL_NVP(st_path_planner_output.lc_style_decider_result));
  ar(CEREAL_NVP(st_path_planner_output.task_safety_evaluation_result));
}

}  // namespace cereal

namespace st::planning {
template <typename Archive>
void serialize(Archive& ar, InitializerOutput& initializer_output) {
  ar(CEREAL_NVP(initializer_output.follower_set));
  ar(CEREAL_NVP(initializer_output.leader_set));
  ar(CEREAL_NVP(initializer_output.follower_max_decel));
  ar(CEREAL_NVP(initializer_output.is_lc_pause));
  ar(CEREAL_NVP(initializer_output.traj_points));
  ar(CEREAL_NVP(initializer_output.initializer_state));
  ar(CEREAL_NVP(initializer_output.leading_trajs));
  ar(CEREAL_NVP(initializer_output.nudge_info));
  ar(CEREAL_NVP(initializer_output.speed_response_style));
  ar(CEREAL_NVP(initializer_output.lc_status_code));
  ar(CEREAL_NVP(initializer_output.is_init_follow_scene));
  ar(CEREAL_NVP(initializer_output.lc_lead_obj_id));
  ar(CEREAL_NVP(initializer_output.pre_large_vehicle_avoid_state));
  ar(CEREAL_NVP(initializer_output.saved_offset));
  ar(CEREAL_NVP(initializer_output.lc_style_decider_result));
}

template <typename Archive>
void serialize(Archive& ar, NudgeInfos& nudge_info) {
  ar(CEREAL_NVP(nudge_info.nudgeInfos));
}

}  // namespace st::planning

#endif