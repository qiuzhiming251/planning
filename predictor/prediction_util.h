

#ifndef ST_PLANNING_PREDICTION_PREDICTION_UTIL
#define ST_PLANNING_PREDICTION_PREDICTION_UTIL

#include <algorithm>
#include <vector>

#include "absl/status/status.h"

#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction.h"
#include "predictor/prediction_defs.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"

namespace st {
namespace prediction {

// Test if a trajectory proto represents a stationary trajectory.
inline bool IsStationaryTrajectory(const PredictedTrajectoryProto& traj) {
  return traj.type() == PredictionType::PT_STATIONARY;
}
// Test if a trajectory represents a stationary trajectory.
inline bool IsStationaryTrajectory(
    const prediction::PredictedTrajectory& traj) {
  return traj.type() == PredictionType::PT_STATIONARY;
}

// Test if a prediction proto represents a stationary trajectory.
inline bool IsStationaryPrediction(const ObjectPredictionProto& pred) {
  return pred.trajectories().size() == 1 &&
         IsStationaryTrajectory(pred.trajectories(0));
}
// Test if a prediction represents a stationary trajectory.
inline bool IsStationaryPrediction(const ObjectPrediction& pred) {
  return pred.trajectories().size() == 1 &&
         IsStationaryTrajectory(pred.trajectories()[0]);
}

bool RefineTrajByAcc(PredictedTrajectory* const trajectory,
                     const double curr_acc, const double acc_ts_sec);

planning::DiscretizedPath PredictedTrajectoryPointsToPathPoints(
    const std::vector<PredictedTrajectoryPoint>& points);

planning::SpeedVector GenerateSpeedProfileByConstAccel(double v, double a,
                                                       int time_horizon,
                                                       double time_step,
                                                       double a_duration);

absl::Status CombinePathAndSpeed(
    const planning::DiscretizedPath& path_data,
    const planning::SpeedVector& speed_data, double time_step,
    std::vector<PredictedTrajectoryPoint>* trajectory);

std::vector<PredictedTrajectoryPoint>
ApolloTrajectoryPointsToPredictionTrajectoryPoints(
    const std::vector<ApolloTrajectoryPointProto>& points);

}  // namespace prediction
}  // namespace st

#endif  // ST_PLANNING_PREDICTION_PREDICTION_UTIL
