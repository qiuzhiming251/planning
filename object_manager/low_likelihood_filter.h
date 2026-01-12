

#ifndef ONBOARD_PLANNER_OBJECT_LOW_LIKELIHOOD_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_LOW_LIKELIHOOD_FILTER_H_

#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "planner_object.h"
#include "trajectory_filter.h"
#include "predictor/predicted_trajectory.h"

namespace st {
namespace planning {

class LowLikelihoodFilter : public TrajectoryFilter {
 public:
  explicit LowLikelihoodFilter(double min_prob, bool only_use_most_likely_traj)
      : min_prob_(min_prob),
        only_use_most_likely_traj_(only_use_most_likely_traj) {}

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  double min_prob_;
  bool only_use_most_likely_traj_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OBJECT_LOW_LIKELIHOOD_FILTER_H_
