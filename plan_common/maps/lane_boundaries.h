
#ifndef AD_BYD_PLANNING_MAP_LANE_BOUNDARIES_H
#define AD_BYD_PLANNING_MAP_LANE_BOUNDARIES_H

#include <memory>

#include <cereal/access.hpp>
#include "plan_common/maps/lane_boundary.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/math/line_curve2d.h"

namespace ad_byd {
namespace planning {
class LaneBoundaries {
 public:
  explicit LaneBoundaries(const std::vector<LaneBoundaryConstPtr> &boundaries);
  ~LaneBoundaries() = default;

  const math::LineCurve2d &line_curve() const { return line_curve_; }

  const std::vector<LaneBoundaryType> &boundary_types() const {
    return boundary_types_;
  }

  const std::vector<LaneBoundaryConstPtr> &lane_boundaries() const {
    return boundaries_;
  };

  /// @brief get segment index by boundary accumulate s
  /// @param s
  /// @return lane boundary segment index
  const int32_t GetBoundarySegmentIndex(const double &s) const;

 private:
  std::vector<LaneBoundaryConstPtr> boundaries_;
  std::vector<LaneBoundaryType> boundary_types_;
  math::LineCurve2d line_curve_;

  template <class Archive>
  friend void serialize(Archive &ar, LaneBoundaries &lane_boundaries);
};

using LaneBoundariesPtr = std::shared_ptr<LaneBoundaries>;
using LaneBoundariesConstPtr = std::shared_ptr<const LaneBoundaries>;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_LANE_BOUNDARIES_H