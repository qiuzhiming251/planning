
#ifndef AD_BYD_PLANNING_MAP_LANE_BOUNDARY_H
#define AD_BYD_PLANNING_MAP_LANE_BOUNDARY_H

#include <memory>

#include "plan_common/maps/lane_boundary.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/math/line_curve2d.h"

#include <cereal/access.hpp>

namespace ad_byd {
namespace planning {
class LaneBoundary {
 public:
  explicit LaneBoundary(const LaneBoundaryInfo& boundary_info);
  ~LaneBoundary() = default;

  void PushLeftLane(const uint64_t lane_id) {
    left_lanes_.emplace_back(lane_id);
  }
  void PushRightLane(const uint64_t lane_id) {
    right_lanes_.emplace_back(lane_id);
  }

  const uint64_t id() const { return id_; };
  const std::vector<Point2d>& points() const {
    return points_;
  }  // points before interpolation
  const std::vector<Point2d>& curve_points() const {
    return line_curve_.points();
  }
  const math::LineCurve2d& line_curve() const { return line_curve_; }
  const LaneBoundaryType& type() const { return type_; }
  const double& curve_length() const { return curve_length_; }
  const std::vector<uint64_t>& left_lanes() const { return left_lanes_; };
  const std::vector<uint64_t>& right_lanes() const { return right_lanes_; };
  const double length() const { return length_; }
  bool IsValid() const {
    return points_.size() > 1u && line_curve_.points().size() > 1u;
  }

  void set_section_id(const uint64_t section_id) { section_id_ = section_id; }
  const uint64_t section_id() const { return section_id_; }

 private:
  void InterpolatePoints();

 private:
  uint64_t id_;
  std::vector<Point2d> points_;
  std::vector<uint64_t> left_lanes_;
  std::vector<uint64_t> right_lanes_;
  math::LineCurve2d line_curve_;
  LaneBoundaryType type_;
  double length_ = 0.0;
  double curve_length_ = 0.0;

  uint64_t section_id_ = 0;

  template <class Archive>
  friend void serialize(Archive& ar, LaneBoundary& lane_boundary);
};

using LaneBoundaryPtr = std::shared_ptr<LaneBoundary>;
using LaneBoundaryConstPtr = std::shared_ptr<const LaneBoundary>;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_LANE_BOUNDARY_H