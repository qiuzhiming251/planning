

#ifndef ONBOARD_PLANNER_DISCRETIZED_PATH_H_
#define ONBOARD_PLANNER_DISCRETIZED_PATH_H_

#include <utility>  // for move
#include <vector>

//#include "global/buffered_logger.h"  // for BufferedLoggerWrapper
//#include "lite/check.h"              // for CHECK_GE
#include "plan_common/log.h"
#include "plan_common/math/frenet_common.h"
#include "geometry/polygon2d.h"
#include "plan_common/math/util.h"  // for CeilToInt
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st {
namespace planning {

// The path points to construct a discretized-path should meet the conditions
// that 1) the s is monotonically increasing and 2) the start s is zero.
class DiscretizedPath : public std::vector<PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<PathPoint> path_points);

  double length() const {
    if (empty()) {
      return 0.0;
    }
    return back().s();
  }

  FrenetCoordinate XYToSL(const Vec2d& pos) const;

  FrenetPolygon XYToSL(const Polygon2d& contour, const Vec2d& pos) const;

  PathPoint Evaluate(double path_s) const;

  PathPoint EvaluateReverse(double path_s) const;

  static DiscretizedPath CreateResampledPath(
      std::vector<PathPoint> raw_path_points, double interval) {
    CHECK_GE(raw_path_points.size(), 2);
    DiscretizedPath raw_path(std::move(raw_path_points));
    double s = 0.0;
    std::vector<PathPoint> path_points;
    path_points.reserve(CeilToInt(raw_path.length() / interval));
    while (s < raw_path.length()) {
      path_points.push_back(raw_path.Evaluate(s));
      s += interval;
    }
    return DiscretizedPath(std::move(path_points));
  }

 protected:
  std::vector<PathPoint>::const_iterator QueryLowerBound(double path_s) const;
  std::vector<PathPoint>::const_iterator QueryUpperBound(double path_s) const;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_DISCRETIZED_PATH_H_
