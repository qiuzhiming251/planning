

#ifndef ONBOARD_PLANNER_OBJECT_MOTION_STATE_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_MOTION_STATE_FILTER_H_

#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "planner_object.h"
#include "trajectory_filter.h"
#include "predictor/predicted_trajectory.h"

namespace st {
namespace planning {

class MotionStateFilter : public TrajectoryFilter {
 public:
  MotionStateFilter(const PoseProto& pose,
                    const VehicleGeometryParamsProto& vehicle_geom);

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  double yaw_;
  Vec2d tangent_;
  Vec2d pos_;
  Vec2d velocity_;
  double speed_;
  Vec2d motion_backoff_pos_;
  Vec2d stationary_backoff_pos_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OBJECT_MOTION_STATE_FILTER_H_
