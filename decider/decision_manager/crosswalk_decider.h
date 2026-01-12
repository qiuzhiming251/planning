

#ifndef ONBOARD_PLANNER_DECISION_CROSSWALK_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_CROSSWALK_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/crosswalk_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/planner_object_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

struct CrosswalkDeciderOutput {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  std::vector<ConstraintProto::SpeedRegionProto> speed_regions;
  std::vector<CrosswalkStateProto> crosswalk_states;
};

struct CrosswalkDeciderInput {
  const st::VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const DrivePassage* passage = nullptr;
  const mapping::LanePath* lane_path_from_start = nullptr;
  const PlannerObjectManager* obj_mgr = nullptr;
  const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>*
      last_crosswalk_states = nullptr;

  double now_in_seconds;
  double s_offset;
};

absl::StatusOr<CrosswalkDeciderOutput> BuildCrosswalkConstraints(
    const CrosswalkDeciderInput& input);
}  // namespace planning
}  // namespace st
#endif
