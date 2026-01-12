

#ifndef ONBOARD_PLANNER_OBJECT_PROCESSED_SPACETIME_TRAJECTORY_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_PROCESSED_SPACETIME_TRAJECTORY_MANAGER_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "plan_common/async/thread_pool.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "predictor/predicted_trajectory.h"
#include "speed_finder_input.h"
#include "plan_common/util/map_util.h"

namespace st {

namespace planning {

// The manager of all the spacetime object trajectories. Each prediction or
// stationary object is modeled as a `SpacetimeObjectTrajectory`.
class ProcessedSpacetimeTrajectoryManager : public SpacetimeTrajectoryManager {
 public:
  ProcessedSpacetimeTrajectoryManager() {}
  ProcessedSpacetimeTrajectoryManager(const SpacetimeTrajectoryManager& other)
      : SpacetimeTrajectoryManager(other) {}

  void ModifySpacetimeTrajectory(
      const SpeedFinderInput& input,
      const VehicleGeometryParamsProto& vehicle_geometry_params,
      const SpeedFinderParamsProto& speed_default_params);
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OBJECT_PROCESSED_SPACETIME_TRAJECTORY_MANAGER_H_
