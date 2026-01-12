#pragma once

#include <vector>

#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {

struct AccSpeedFinderOutput {
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  SpeedFinderDebugProto speed_finder_proto;
};

}  // namespace st::planning
