#ifndef ONBOARD_PLANNER_SELECTOR_COST_FEATURE_UTIL_H_
#define ONBOARD_PLANNER_SELECTOR_COST_FEATURE_UTIL_H_

#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/st_inference/est_planner_output.h"
#include "plan_common/assist_util.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections_info.h"
#include "plan_common/plan_common_defs.h"
#include "router/route_sections_util.h"
#include "decider/selector/common_feature.h"

#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {
using PlannerTrajectory = std::vector<ApolloTrajectoryPointProto>;

double LinearInterpolate(double x0, double x1, double t0, double t1, double t);
inline double CalcPsi(const ApolloTrajectoryPointProto& prev_pt,
                      const ApolloTrajectoryPointProto& succ_pt) {
  return (succ_pt.path_point().kappa() - prev_pt.path_point().kappa()) /
         (succ_pt.relative_time() - prev_pt.relative_time());
}

inline double CalcLatJerk(const ApolloTrajectoryPointProto& pt, double psi) {
  // j_lat = 3 * v * a * kappa + v^2 * psi
  return std::abs(3.0 * pt.v() * pt.a() * pt.path_point().kappa() +
                  Sqr(pt.v()) * psi);
}

// Value range (0, 1], `base` for shape and `reg` for scaling along x axis.
inline double ExpDecayCoeffAtStep(double base, double reg, int i) {
  return std::pow(base, -i / (reg * kTrajectorySteps));
}

inline bool IsStaticObjectType(ObjectType type) {
  return type == OT_UNKNOWN_STATIC || type == OT_VEGETATION || type == OT_FOD ||
         type == OT_BARRIER || type == OT_CONE || type == OT_ROW_OBSTACLES ||
         type == OT_WARNING_TRIANGLE;
}

// absl::StatusOr<PointOnRouteSections> FindLastTrajPointOnRouteSections(
//     const PlannerSemanticMapManager& psmm,
//     const RouteSectionsInfo& sections_info, const DrivePassage& dp,
//     const PlannerTrajectory& traj_pts);

// bool IsInTlControlledIntersection(const PlannerSemanticMapManager& psmm,
//                                   const DrivePassage& drive_passage, double
//                                   s);

// double CalculateCrossingBoundary(
//     const DrivePassage& drive_passage,
//     const std::vector<BoundaryInterval>& solid_boundaries,
//     const std::vector<Box2d>& ego_boxes, LaneChangeStage stage,
//     const absl::flat_hash_set<StationBoundaryType>& type_set,
//     const absl::StatusOr<double>& start_l_or,
//     const absl::StatusOr<double>& end_l_or, const double ego_half_width,
//     const PlannerSemanticMapManager& psmm);

absl::flat_hash_set<std::string> FindBlockObjectIds(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const EstPlannerOutput& planner_output,
    const VehicleGeometryParamsProto& vehicle_geom, const bool is_highway,
    const bool is_lane_keep, const bool is_lc_saftey_check_failed,
    const double ego_v);

// not block but invade
std::map<std::string, InvadeStaticObjInfo> FindInvadeStaticObjects(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const EstPlannerOutput& planner_output,
    const absl::flat_hash_set<std::string>& block_obj_ids,
    const absl::flat_hash_set<std::string>& stalled_objects);

TargetLaneStateProto GenerateTargetLaneState(
    const PlannerSemanticMapManager& psmm, const SchedulerOutput& output,
    const double preview_distance);
// int ChooseBestCostTraj(
//     const std::vector<EstPlannerOutput>& results,
//     const absl::flat_hash_map<int, FeatureCostSum>& idx_traj_cost_map);

std::optional<LeaderObjectInfo> CalcNearestLeaderFromBlockObjects(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage,
    const absl::flat_hash_set<std::string>& block_obj_ids);

// Calc by static data, irrelate with the av pos
// @return left_cones_num, right_cones_num

LaneConesInfo CalcNeighborConesOnLane(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage, const FrenetBox& av_frenet_box,
    const absl::flat_hash_set<std::string>& block_obj_ids);

bool CanIgnoreCrossSolidBoundary(LaneChangeType lane_change_type, int lc_num,
                                 double length_along_route, bool lc_left,
                                 bool is_highway, bool in_tunnel,
                                 double lane_keep_dist_to_merge);

std::optional<int> FindMinValidLaneNumAtMost(const MppSectionInfo& mpp_section,
                                             double preview_length);
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SELECTOR_COST_FEATURE_UTIL_H_
