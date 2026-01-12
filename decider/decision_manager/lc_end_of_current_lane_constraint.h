

#ifndef ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_
#define ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_

#include "absl/status/statusor.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

absl::StatusOr<ConstraintProto::SpeedRegionProto>
BuildLcEndOfCurrentLaneConstraints(const DrivePassage& dp,
                                   const mapping::LanePath& lane_path_before_lc,
                                   double ego_v);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_
