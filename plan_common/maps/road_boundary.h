

#ifndef AD_BYD_PLANNING_MAP_ROAD_BOUNDARY_H
#define AD_BYD_PLANNING_MAP_ROAD_BOUNDARY_H
#include <memory>

#include "plan_common/maps/map_def.h"
#include "plan_common/maps/road_boundary.h"
#include "plan_common/math/line_curve2d.h"

#include <cereal/access.hpp>
namespace ad_byd {
namespace planning {
class RoadBoundary {
 public:
  explicit RoadBoundary(const RoadBoundaryInfo& road_boundary_info);
  ~RoadBoundary() = default;

  const uint64_t id() const { return id_; };
  const std::vector<Point2d>& points() const {
    return points_;
  }  // points before interpolation
  const std::vector<Point2d>& curve_points() const {
    return line_curve_.points();
  }
  const math::LineCurve2d& line_curve() const { return line_curve_; }
  const RoadBoundaryType& type() const { return type_; }
  const double& curve_length() const { return curve_length_; }
  const double length() const { return length_; }
  bool IsValid() const { return points_.size() > 1u; }

  void set_section_id(const uint64_t section_id) { section_id_ = section_id; }
  const uint64_t section_id() const { return section_id_; }

  /// @brief Get RoadBoundary points in lat range.
  /// @param center_pts center points for check.
  /// @param road_segs road segments.
  /// @param lat_dist_range <left, right>, left must larget than right.
  /// @return True if has road boundary in lat range.
  static bool GetRoadBoundaryPoints(
      const std::vector<Point2d>& center_pts,
      const std::vector<math::LineSegment2d>& road_segs,
      const std::pair<double, double>& lat_dist_range,
      std::vector<Point2d>* road_pts);

  // default function for st planning
  bool has_height() const { return false; }
  double height() const { return 0.0; }

 private:
  void InterpolatePoints();

 private:
  uint64_t id_;
  std::vector<Point2d> points_;
  math::LineCurve2d line_curve_;
  RoadBoundaryType type_;
  double length_ = 0.0;
  double curve_length_ = 0.0;

  uint64_t section_id_ = 0;

  template <class Archive>
  friend void serialize(Archive& ar, RoadBoundary& road_boundary);
};
using RoadBoundaryPtr = std::shared_ptr<RoadBoundary>;
using RoadBoundaryConstPtr = std::shared_ptr<const RoadBoundary>;
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MAP_ROAD_BOUNDARY_H