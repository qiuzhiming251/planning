

#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_UTIL_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/vec.h"
#include "plan_common/maps/composite_lane_path.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections.h"
#include "plan_common/maps/route_sections_info.h"

namespace st::planning {

struct PointOnRouteSections {
  double accum_s;
  int section_idx;
  double fraction;
  mapping::ElementId lane_id;
};

// Assume that global sections and local sections have overlaps and global
// sections are longer than local sections.
absl::StatusOr<RouteSections> AlignRouteSections(
    const RouteSections& global_sections, const RouteSections& local_sections);

// NOTE: Concatenate origin sections to target sections, returns from
// the start of origin sections to the end of target sections.
absl::StatusOr<RouteSections> SpliceRouteSections(
    const RouteSections& origin_sections, const RouteSections& target_sections);

// NOTE: Used for dynamic route in NOA mode, similar to
// `SpliceRouteSections`, but assume that origin_sections must be part of the
// global_sections if a matching section is found.
absl::StatusOr<RouteSections> AppendRouteSectionsToTail(
    const RouteSections& origin_sections, const RouteSections& global_sections);

RouteSections RouteSectionsFromCompositeLanePath(
    const ad_byd::planning::Map& smm, const CompositeLanePath& clp);

RouteSections RouteSectionsFromCompositeLanePath(
    const PlannerSemanticMapManager& psmm, const CompositeLanePath& clp);

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByLateralOffset(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info, const Vec2d& query_point,
    double lat_dist_thres = kMaxHalfLaneWidth);

absl::StatusOr<mapping::LanePath>
FindClosestLanePathOnRouteSectionsToSmoothPoint(
    const PlannerSemanticMapManager& psmm, const RouteSections& sections,
    const Vec2d& query_point, double* proj_s = nullptr);

absl::StatusOr<RouteSections> ClampRouteSectionsBeforeArcLength(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& raw_route_sections, double len);

absl::StatusOr<RouteSections> ClampRouteSectionsAfterArcLength(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& raw_route_sections, double len);

absl::StatusOr<std::vector<mapping::LanePath>>
CollectAllLanePathOnRouteSectionsFromStart(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections);

absl::StatusOr<std::vector<mapping::LanePath>>
CollectAllLanePathOnRouteSections(const PlannerSemanticMapManager& psmm,
                                  const RouteSections& route_sections);

RouteSections BackwardExtendRouteSections(const PlannerSemanticMapManager& psmm,
                                          const RouteSections& raw_sections,
                                          double extend_len);

absl::StatusOr<RouteSections> BackwardExtendRouteSectionsFromPos(
    const PlannerSemanticMapManager& psmm, const RouteSections& raw_sections,
    const Vec2d& pos, double extend_len);

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathOnRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const mapping::LanePath& raw_lane_path, double extend_len);

absl::StatusOr<mapping::LanePath> BackwardExtendLanePathOnRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const mapping::LanePath& raw_lane_path, double extend_len);

absl::StatusOr<mapping::LanePath> FindClosestTargetLanePathOnReset(
    const PlannerSemanticMapManager& psmm, const RouteSections& prev_sections,
    const Vec2d& ego_pos);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_UTIL_H_
