

#ifndef ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_
#define ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_

#include <utility>

#include "absl/types/span.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/constraint_manager.h"
#include "plan_common/math/frenet_common.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/speed/st_speed/speed_profile.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

namespace {
// 补齐预测轨迹时长，并通过planner object acc进行重新采样refine
void ExtendPredictionTraj(const SpacetimeObjectTrajectory& traj,
                          ConstraintProto::LeadingObjectProto* leading_obj);
}  // namespace

SpeedProfile CreateSpeedProfile(
    double v_now, const DrivePassage& passage,
    const PlannerSemanticMapManager& psmm,
    const absl::Span<const ConstraintProto::SpeedRegionProto>& speed_zones,
    const absl::Span<const ConstraintProto::StopLineProto>& stop_points);

ConstraintProto::SpeedRegionProto MergeSameElement(
    absl::Span<const ConstraintProto::SpeedRegionProto> elements);

bool IsLeadingObjectType(ObjectType type);

std::pair<double, double> CalcSlBoundaries(const PathSlBoundary& sl_boundary,
                                           const FrenetBox& frenet_box);

ConstraintProto::LeadingObjectProto CreateLeadingObject(
    const SpacetimeObjectTrajectory& traj, const DrivePassage& passage,
    ConstraintProto::LeadingObjectProto::Reason reason,
    bool is_group_tail = false);

bool IsTrafficLightControlledLane(const ad_byd::planning::Lane& lane);
// TODO: implement this function:
// ConstraintProto::LeadingObjectProto ProjectTrajectorySTOnDrivePassage()

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_
