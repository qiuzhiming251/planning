

#ifndef ST_PLANNING_SPEED_SPEED_FINDER_OUTPUT
#define ST_PLANNING_SPEED_SPEED_FINDER_OUTPUT

#include <optional>
#include <string>
#include <vector>

#include "plan_common/constraint_manager.h"
#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "object_manager/partial_spacetime_object_trajectory.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"

namespace st::planning {

struct SpeedFinderOutput {
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  double spdlimit_curvature_gain = 1.0;
  SpeedFinderDebugProto speed_finder_proto;
  ConstraintManager constraint_mgr;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;
  std::optional<std::string> alerted_front_vehicle;
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  std::map<std::string, ObjectSlInfo> obj_sl_map;
  std::string attention_obj_id = "";
  std::optional<std::string> cipv_obj_id;
  SpeedGamingResultProto speed_gaming_result;
};

}  // namespace st::planning

#endif  // ST_PLANNING_SPEED_SPEED_FINDER_OUTPUT
