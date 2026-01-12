
#ifndef AD_BYD_PLANNING_MAP_ROAD_BOUNDARIES_H
#define AD_BYD_PLANNING_MAP_ROAD_BOUNDARIES_H

#include <memory>

#include "plan_common/maps/map_def.h"
#include "plan_common/maps/road_boundary.h"
#include "plan_common/math/line_curve2d.h"

#include <cereal/access.hpp>

namespace ad_byd {
namespace planning {
class RoadBoundaries {
 public:
  explicit RoadBoundaries(const std::vector<RoadBoundaryConstPtr>& boundaries);
  ~RoadBoundaries() = default;

  // const std::vector<RoadBoundaryType> &boundary_types() const { return
  // boundary_types_; }

  const std::vector<RoadBoundaryConstPtr>& road_boundaries() const {
    return boundaries_;
  };

  /// @brief get segment index by boundary accumulate s
  /// @param s
  /// @return road boundary segment index
  // const int32_t GetBoundarySegmentIndex(const double &s) const;

 private:
  std::vector<RoadBoundaryConstPtr> boundaries_;
  // std::vector<RoadBoundaryType> boundary_types_;

  template <class Archive>
  friend void serialize(Archive& ar, RoadBoundaries& road_boundaries);
};

using RoadBoundariesPtr = std::shared_ptr<RoadBoundaries>;
using RoadBoundariesConstPtr = std::shared_ptr<const RoadBoundaries>;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_ROAD_BOUNDARIES_H