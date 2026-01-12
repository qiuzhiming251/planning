

#ifndef ST_PLANNING_MAPS_SEMANTIC_MAP_UTIL
#define ST_PLANNING_MAPS_SEMANTIC_MAP_UTIL

#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
// IWYU pragma: no_include "onboard/container/strong_int.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/maps/type_defs.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/box2d.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"

namespace st::mapping {
constexpr double kDefaultMapLaneWidth = 3.5;

struct SamplePathPointsResult {
  std::vector<Vec2d> points;
  bool is_partial;
  std::string message;
};

/*
  discomfort = v^2 * kappa * log(T + 1.0)
 */

double GetLaneDiscomfort(double curveture, double speed, double arch_length);

// Recursive traverse message when reassigning semantic map IDs.
void AssignSemanticMapNewId(
    const std::map<std::pair<int, ElementId>, ElementId>& feature_id_table,
    ::google::protobuf::Message* message, int map_index);

bool CheckResampleLanePoints(const ad_byd::planning::Lane& lane,
                             double start_fraction, double end_fraction);

std::vector<Vec2d> ResampleLanePoints(const ad_byd::planning::Lane& lane,
                                      double start_fraction,
                                      double end_fraction,
                                      std::vector<double>* cumulative_lengths);

// ---- Based on lane aligned semantic map
bool IsNeighborLane(bool left, const ad_byd::planning::Lane& source_lane,
                    ElementId target_id);

bool ContainsNeighborLane(bool left, const ad_byd::planning::Lane& source_lane,
                          const std::deque<ElementId>& target_ids);

std::pair<double, double> GetNeighborRange(
    bool left, const LanePath& target,
    const ad_byd::planning::Lane& source_lane);

bool HasNeighborPointOnPath(const ad_byd::planning::Map& map, bool left,
                            const LanePoint& source_point,
                            const LanePath& target,
                            LanePoint* neighbor_point = nullptr);
// -------------------------------------------------------------------

bool IsOutgoingLane(const ad_byd::planning::Lane& source_lane,
                    ElementId out_lane_id);
bool IsRightMostLane(const ad_byd::planning::Map& map,
                     const LanePath& lane_path, double s);
bool IsRightMostLane(const ad_byd::planning::Map& map,
                     const LanePath& lane_path);
bool IsRightMostLane(const ad_byd::planning::Map& map, const ElementId lane_id);
bool IsLeftMostLane(const ad_byd::planning::Map& map, const LanePath& lane_path,
                    double s);
bool IsLeftMostLane(const ad_byd::planning::Map& map, const ElementId lane_id);

// The side of the boundary w.r.t. the considering lane
enum class BoundarySide : int32_t { kAny, kLeft, kRight };

bool IsLanePathBlockedByBox2d(const ad_byd::planning::Map& map,
                              const Box2d& box, const LanePath& lane_path,
                              double lat_thres);

std::vector<Vec2d> SampleLanePathPoints(const ad_byd::planning::Map& map,
                                        const LanePath& lane_path);

// Return the pair [upper_bound pos,inner_fraction] with the dist fraction.
std::pair<int, double> ComputePosition(const std::vector<Vec2d>& points,
                                       double fraction);
std::vector<Vec2d> TruncatePoints(const std::vector<Vec2d>& points,
                                  double start_fraction, double end_fraction,
                                  double epsilon);

// Sample points by smooth coordinates. Can be unified with the above function.
absl::StatusOr<SamplePathPointsResult> SampleLanePathProtoPoints(
    const ad_byd::planning::Map& map, const LanePathProto& lane_path);

std::vector<LanePoint> LanePathToLanePoints(const LanePath& lane_path);

// std::vector<Vec2d> ReSampleLaneOrLaneBoundary(
//     const GeoPolylineProto& control_points, double sample_interval_meters);

std::vector<Polygon2d> SampleLaneBoundaryPolyLine(
    const ad_byd::planning::Lane& left_lane_info,
    const ad_byd::planning::Lane& right_lane_info, double start_fraction,
    double end_fraction);

absl::StatusOr<LanePath> ClampLanePathFromPos(const ad_byd::planning::Map& map,
                                              const LanePath& lane_path,
                                              const Vec2d& pos);

template <typename SmmType>
bool IsOutgoingLane(const SmmType& semantic_map_manager,
                    const ad_byd::planning::Lane& source_lane,
                    ElementId out_lane_id) {
  const auto& lanes = source_lane.next_lane_ids();
  return std::find(lanes.begin(), lanes.end(), out_lane_id) != lanes.end();
}

}  // namespace st::mapping

#endif  // ST_PLANNING_MAPS_SEMANTIC_MAP_UTIL
