

#ifndef ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_UTIL_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"
//#include "planner/planner_manager/planner_defs.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st::planning {

struct SamplePathPointsResult {
  std::vector<Vec2d> points;
  bool is_partial;
  std::string message;
};

bool IsOutgoingLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const ad_byd::planning::Lane& source_lane, mapping::ElementId out_lane_id);

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s);

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id);

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s);

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id);

bool IsLanePathBlockedByBox2d(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const Box2d& box, const mapping::LanePath& lane_path, double lat_thres);

std::vector<Vec2d> SampleLanePathPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path);

// Sample points by smooth coordinates. Can be unified with the above function.
absl::StatusOr<SamplePathPointsResult> SampleLanePathProtoPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePathProto& lane_path);

absl::StatusOr<mapping::LanePath> ClampLanePathFromPos(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, const Vec2d& pos);

absl::StatusOr<mapping::LanePoint> FindOutgoingLanePointWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm, mapping::ElementId id);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_UTIL_H_
