#pragma once

#include <string>
#include <vector>

#include "acc/acc_path_corridor.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_validation.pb.h"

namespace st::planning {
struct AccOutput {
  std::vector<AccPathCorridor> acc_path_corridors;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  TrajectoryValidationResultProto traj_validation_result;

  std::string DebugString() const;
};

}  // namespace st::planning
