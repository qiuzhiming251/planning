

#ifndef ONBOARD_PLANNER_OBJECT_TRAJECTORY_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_TRAJECTORY_FILTER_H_

#include <memory>
#include <vector>

#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "planner_object.h"
#include "predictor/predicted_trajectory.h"

namespace st {
namespace planning {

class TrajectoryFilter {
 public:
  virtual FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const = 0;

  virtual ~TrajectoryFilter() {}
};

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_OBJECT_TRAJECTORY_FILTER_H_
