

#ifndef ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_
#define ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/math/vec.h"
#include "plan_common/drive_passage.h"
#include "plan_common/driving_map_topo.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/maps/st_boundary.h"

namespace st {
namespace planning {

// enum class LaneSemantic {
//   NONE = 0,                     // NOLINT
//   ROAD = 1,                     // NOLINT
//   INTERSECTION_LEFT_TURN = 2,   // NOLINT
//   INTERSECTION_RIGHT_TURN = 3,  // NOLINT
//   INTERSECTION_STRAIGHT = 4,    // NOLINT
//   INTERSECTION_UTURN = 5        // NOLINT
// };

struct RouteLaneInfo {
  ad_byd::planning::LaneConstPtr lane = nullptr;
  double start_fraction = 0.0;
  double end_fraction = 1.0;
};

struct PathPointSemantic {
  mapping::LanePoint closest_lane_point;
  Vec2d closest_lane_point_pos;
  LaneSemantic lane_semantic = LaneSemantic::NONE;
  // Keep the lane path id history from path beginning to the current point.
  // Lane path id starts from zero, and is increased by one if there is a left
  // lane change and reduced by one if there is a right lane change.
  // Examples:
  // 1) { 0 } means there is no lane change from path beginning to current
  // point;
  // 2) { 0, 1 } means there is a left lane change from path beginning to
  // current point;
  // 3) { 0, 1, 2 } means there are two successive left lane changes from path
  // beginning to current point;
  // 4) { 0, 1, 0 } means current point returns to the original lane after a
  // left and a right lane change.
  std::vector<int> lane_path_id_history;
  // The distance of the ego vehicle's path deviation from lane center (absolute
  // value).
  double deviation_distance = 0.0;
  ad_byd::planning::LaneConstPtr lane_info = nullptr;
};

absl::StatusOr<std::vector<PathPointSemantic>> AnalyzePathSemantics(
    int plan_id, const DiscretizedPath& path, int max_analyze_path_index,
    const PlannerSemanticMapManager& psmm, const DrivePassage* drive_passage,
    const DrivingMapTopo* driving_map_topo, ThreadPool* thread_pool);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_
