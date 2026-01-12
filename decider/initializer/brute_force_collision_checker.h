

#ifndef ONBOARD_PLANNER_INITIALIZER_BRUTE_FORCE_COLLISION_CHECKER_H_
#define ONBOARD_PLANNER_INITIALIZER_BRUTE_FORCE_COLLISION_CHECKER_H_

#include <vector>

#include "absl/types/span.h"
#include "decider/initializer/collision_checker.h"
#include "decider/initializer/geometry/geometry_state.h"
#include "decider/initializer/motion_state.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_object_trajectory.h"

namespace st {
namespace planning {

// This class is mainly used to valid and benchmark complicated collision
// checker.
class BruteForceCollisionChecker : public CollisionChecker {
 public:
  BruteForceCollisionChecker(
      absl::Span<const SpacetimeObjectTrajectory> trajectories,
      const VehicleGeometryParamsProto* vehicle_geom, int sample_step,
      double stationary_buffer, double moving_buffer);

  void UpdateStationaryObjectBuffer(double stationary_buffer) override;
  void UpdateMovingObjectBuffer(double moving_buffer) override;

  void CheckCollisionWithTrajectories(double init_t,
                                      absl::Span<const MotionState> states,
                                      const IgnoreTrajMap& ignored_trajs,
                                      CollisionInfo* info) const override;
  void CheckCollisionWithStationaryObjects(
      absl::Span<const GeometryState> states,
      CollisionInfo* info) const override;

  int sample_step() const { return sample_step_; }

 private:
  const VehicleGeometryParamsProto* vehicle_geom_;
  int sample_step_;
  double stationary_buffer_;
  double moving_buffer_;
  std::vector<SampledTrajectory> sampled_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_stationary_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_moving_trajs_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_INITIALIZER_BRUTE_FORCE_COLLISION_CHECKER_H_
