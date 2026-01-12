

#ifndef ONBOARD_PLANNER_SPEED_INTERACTIVE_LATERALLY_H_
#define ONBOARD_PLANNER_SPEED_INTERACTIVE_LATERALLY_H_

#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/maps/st_boundary_with_decision.h"

namespace st {
namespace planning {

struct NudgeVector {
  // Index of origin predicted path point that push to nudge.
  int leverage_point_index = 0;

  enum class Direction { LEFT = 1, RIGHT = 2 };
  Direction direction;
  double offset = 0.0;
};

struct StBoundaryLatModificationInfo {
  StBoundaryModifierProto::ModifierType modifier_type =
      StBoundaryModifierProto::UNKNOWN;
  NudgeVector nudge_vector;
};

absl::flat_hash_map<std::string, StBoundaryLatModificationInfo>
GenerateLatModificationInfo(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, double current_a,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision);

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_LATERALLY_H_
