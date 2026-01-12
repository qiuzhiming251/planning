

#ifndef ONBOARD_PLANNER_SPEED_PATH_SPEED_COMBINER_H_
#define ONBOARD_PLANNER_SPEED_PATH_SPEED_COMBINER_H_

#include <vector>

#include "absl/status/status.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {

absl::Status CombinePathAndSpeed(
    const DiscretizedPath& path_data, bool forward,
    const SpeedVector& speed_data,
    std::vector<ApolloTrajectoryPointProto>* trajectory);

absl::Status ExtendTrajectoryLength(
    const DiscretizedPath& path_data, bool forward,
    const SpeedVector& speed_data,
    std::vector<ApolloTrajectoryPointProto>* trajectory);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_PATH_SPEED_COMBINER_H_
