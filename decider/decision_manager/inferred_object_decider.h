

#ifndef ONBOARD_PLANNER_DECISION_INFERRED_OBJECT_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_INFERRED_OBJECT_DECIDER_H_

#include "absl/status/statusor.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st {
namespace planning {

absl::StatusOr<ConstraintProto::SpeedProfileProto>
BuildInferredObjectConstraint(const PlannerSemanticMapManager& psmm,
                              const SceneOutputProto& scene_reasoning,
                              const mapping::LanePath& lane_path_from_start,
                              double ego_init_v);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_INFERRED_OBJECT_DECIDER_H_
