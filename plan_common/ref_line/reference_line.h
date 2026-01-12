
#ifndef AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REFERENCE_LINE_H_
#define AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REFERENCE_LINE_H_
#include "plan_common/path/path.h"
#include "plan_common/type_def.h"
#include "plan_common/maps/map.h"
#include "plan_common/math/box2d.h"
#include "plan_common/math/curve_limits.h"
#include "plan_common/math/vec2d.h"

namespace ad_byd {
namespace planning {
class ReferenceLine {
 public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;

  void Reset();
  double Length() const { return path_.length(); }

  const LaneSequencePtr &lane_sequence() { return lane_sequence_; }

  PlanResult UpdateReferenceLine(const TrajectoryPoint &start_point,
                                 const LaneSequencePtr &lane_sequence,
                                 const MapPtr &map);

  // Sample reference points.
  bool SamplePoints(const double &start_s, const double &length,
                    const double &interval,
                    std::vector<PathPoint> *const path_points) const {
    return path_.SamplePoints(start_s, length, interval, path_points);
  }

  // Get reference point interpolate by s.
  PathPoint GetReferencePoint(const double &s) const {
    return path_.GetPointAtS(s);
  }

  // Convert Frenet SL to XY point.
  bool SLToXY(const SLPoint &sl_point, Point2d *xy_point,
              PathPoint *ref_point) const {
    return path_.SLToXY(sl_point, xy_point, ref_point);
  }
  bool SLToXY(const SLPoint &sl_point, Point2d *xy_point) const {
    return path_.SLToXY(sl_point, xy_point, nullptr);
  }
  bool SLToXY(const FrenetPoint &frenet_point,
              TrajectoryPoint *path_point) const;

  // Convert XY to Frenet SL point.
  bool XYToSL(const Point2d &xy_point, SLPoint *sl_point,
              PathPoint *ref_point) const {
    return path_.XYToSL(xy_point, sl_point, ref_point);
  }
  bool XYToSL(const Point2d &xy_point, SLPoint *sl_point) const {
    return path_.XYToSL(xy_point, sl_point, nullptr);
  }
  bool XYToSL(const TrajectoryPoint &traj_point,
              FrenetPoint *frenet_point) const {
    return path_.XYToSL(traj_point, frenet_point);
  }
  bool XYToSL(std::vector<PathPoint> &path_points) const {
    return path_.XYToSL(path_points);
  }
  bool XYToSL(const math::Box2d &box_2d, SLBoundary *sl_boundary) const {
    return path_.XYToSL(box_2d, sl_boundary);
  };
  bool XYToSL(const std::vector<Point2d> &corners,
              SLBoundary *sl_boundary) const {
    return path_.XYToSL(corners, sl_boundary);
  };

 private:
  PlanResult ReferenceLineFitting(const TrajectoryPoint &start_point,
                                  const LaneSequencePtr &lane_sequence,
                                  const MapPtr &map);

 private:
  LaneSequencePtr lane_sequence_ = nullptr;
  std::vector<Point2d> center_pts_;
  Path path_;
  CurveLimit drive_boundary_;
};
using ReferenceLinePtr = std::shared_ptr<ReferenceLine>;
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_REFERENCE_LINE_REFERENCE_LINE_H_
