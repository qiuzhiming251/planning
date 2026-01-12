#ifndef AD_BYD_PLANNING_MAP_EXP_TRAJECTORY_H
#define AD_BYD_PLANNING_MAP_EXP_TRAJECTORY_H

#include <memory>
#include <unordered_set>
#include "plan_common/maps/map_def.h"
#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {
class ExpTrajectory {
 public:
  ExpTrajectory() = default;
  ExpTrajectory(const ExpTrajectoryInfo& exp_trajectory_info);

  const uint64_t id() const { return id_; }
  const uint64_t lane_id() const { return id_; }
  const uint64_t start_lane_id() const { return start_lane_id_; }
  const uint64_t end_lane_id() const { return end_lane_id_; }
  const std::vector<uint64_t>& relative_lane_id() const {
    return relative_lane_id_;
  }
  const std::vector<Point2d>& points() const { return points_; }

 protected:
  uint64_t id_;
  uint64_t lane_id_;
  uint64_t start_lane_id_;
  uint64_t end_lane_id_;
  std::vector<uint64_t> relative_lane_id_;
  std::vector<Point2d> points_;

  template <typename Archive>
  friend void serialize(Archive& ar, ExpTrajectory& exptrajectory);
};

using ExpTrajectoryPtr = std::shared_ptr<ExpTrajectory>;
using ExpTrajectoryConstPtr = std::shared_ptr<const ExpTrajectory>;

}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_MAP_EXP_TRAJECTORY_H