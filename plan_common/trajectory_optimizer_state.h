

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_STATE_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_STATE_H_

#include <vector>

#include "absl/time/time.h"

#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"

#include "plan_common/trajectory_point.h"

namespace st {
namespace planning {

struct TrajectoryOptimizerState {
  TrajectoryOptimizerState() = default;
  explicit TrajectoryOptimizerState(
      const TrajectoryOptimizerStateProto& proto) {
    FromProto(proto);
  }

  void FromProto(const TrajectoryOptimizerStateProto& proto);

  TrajectoryOptimizerStateProto ToProto() const;

  absl::Time last_plan_start_time;
  std::vector<TrajectoryPoint> last_optimized_trajectory;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_STATE_H_
