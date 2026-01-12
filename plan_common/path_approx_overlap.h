

#ifndef ONBOARD_PLANNER_COMMON_PATH_APPROX_OVERLAP_H_
#define ONBOARD_PLANNER_COMMON_PATH_APPROX_OVERLAP_H_

#include <optional>
#include <vector>

#include "path_approx.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/geometry/polygon2d.h"

namespace st {
namespace planning {

// ********************* Agent Overlap **********************
// A single agent state's overlap.
struct AgentOverlap {
  double first_ra_s = 0.0;
  double last_ra_s = 0.0;
  // The heading of nearest path_segment.
  double ra_heading = 0.0;
  // 0.0 if there is a true overlap. It is negative if the agent state is at the
  // right side of the path, and it is positive if the agent state is at the
  // left side of the overlap.
  double lat_dist = 0.0;
};

std::vector<AgentOverlap> ComputeAgentOverlapsWithBuffer(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius);

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeading(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius, double theta,
    double max_heading_diff, std::pair<int, int>& min_max_index_res);

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeading(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius, double theta,
    double max_heading_diff);

bool HasPathApproxOverlapWithPolygon(const PathApprox& path_approx,
                                     double step_length, int first_index,
                                     int last_index, const Polygon2d& polygon,
                                     double search_radius);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_COMMON_PATH_APPROX_OVERLAP_H_
