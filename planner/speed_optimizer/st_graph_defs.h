

#ifndef ONBOARD_PLANNER_SPEED_ST_GRAPH_DEFS_H_
#define ONBOARD_PLANNER_SPEED_ST_GRAPH_DEFS_H_
#include <string>
#include <vector>

#include "plan_common/math/geometry/polygon2d.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
namespace st::planning {
struct StDistancePoint {
  double t = 0.0;
  double path_s = 0.0;
  double distance = 0.0;
  double relative_v = 0.0;
};

struct CloseSpaceTimeObject {
  std::vector<StDistancePoint> st_distance_points{};
  bool is_stationary = false;
  bool is_away_from_traj = false;
  StBoundaryProto::ObjectType object_type{};
  std::string id = "";
  Polygon2d contour{};
};

struct DistanceInfo {
  double s = 0.0;
  double dist = 0.0;
  std::string id = "";
  std::string info = "";
  // first: dist to left; second: dist to right;
  std::pair<double, double> dists = {0.0,0.0};
  ad_byd::planning::RoadBoundaryType type{};
};
}  // namespace st::planning
#endif  // ONBOARD_PLANNER_SPEED_ST_GRAPH_DEFS_H_
