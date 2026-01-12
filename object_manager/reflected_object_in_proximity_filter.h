

#ifndef ONBOARD_PLANNER_OBJECT_REFLECTED_OBJECT_IN_PROXIMITY_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_REFLECTED_OBJECT_IN_PROXIMITY_FILTER_H_

#include "plan_common/math/geometry/box2d.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "planner_object.h"
#include "trajectory_filter.h"
#include "predictor/predicted_trajectory.h"

namespace st {
namespace planning {

// This class filters reflected objects that are in AV proximity. Any objects
// that are considered reflected objects with in SDC's proximity (less than the
// provided `padding` distance) will be filtered.
class ReflectedObjectInProximityFilter : public TrajectoryFilter {
 public:
  ReflectedObjectInProximityFilter(
      const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom,
      double padding);

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  Box2d padded_box_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OBJECT_REFLECTED_OBJECT_IN_PROXIMITY_FILTER_H_
