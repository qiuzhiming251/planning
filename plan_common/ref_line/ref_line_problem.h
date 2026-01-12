
#ifndef AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REFERENCE_PROBLEM_H_
#define AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REFERENCE_PROBLEM_H_
#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {
enum BoundType {
  INVALID_BOUND = 0,
  HARD_BOUND = 1,
  SOFT_BOUND = 2,
  VIRTUAL_BOUND = 3,
  ROAD_BOUND = 4
};

struct RefPoint : public math::Vec2d {
  Point2d center_pt;
  double theta = 0.0;
  double accum_s = 0.0;
  bool reuse = false;
  double l_line_bound = 0.0;
  double r_line_bound = 0.0;
  BoundType l_type = SOFT_BOUND;
  BoundType r_type = SOFT_BOUND;
  double l_road_bound = 0.0;
  double r_road_bound = 0.0;
};

class RefLineProblem {
 public:
  RefLineProblem() = default;
  ~RefLineProblem() = default;

  void set_start_point(const PathPoint &point) { start_point_ = point; }
  void set_points(std::vector<RefPoint> &&points) {
    points_ = std::move(points);
  }

  const PathPoint &start_point() const { return start_point_; }
  const std::vector<RefPoint> &points() const { return points_; }

 private:
  PathPoint start_point_;
  std::vector<RefPoint> points_;
};
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REFERENCE_PROBLEM_H_
