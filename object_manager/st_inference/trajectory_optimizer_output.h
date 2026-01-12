

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_

#include <string>
#include <vector>

#include "modules/cnoa_pnc/planning/proto/planner.pb.h"

#include "plan_common/trajectory_point.h"
#include "plan_common/trajectory_optimizer_state.h"

namespace st::planning {

struct TrajectoryOptimizerOutput {
  std::vector<TrajectoryPoint> trajectory;
  std::vector<ApolloTrajectoryPointProto> trajectory_proto;

  std::optional<NudgeObjectInfo> nudge_object_info;

  // Optimizer Auto Tuning
  // AutoTuningTrajectoryProto candidate_auto_tuning_traj_proto;
  // AutoTuningTrajectoryProto expert_auto_tuning_traj_proto;

  TrajectoryOptimizerState trajectory_optimizer_state;
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_
