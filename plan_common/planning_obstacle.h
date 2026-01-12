

#ifndef AD_BYD_PLANNING_COMMON_PLANNING_OBSTACLE_H
#define AD_BYD_PLANNING_COMMON_PLANNING_OBSTACLE_H

#include "obstacle.h"

namespace ad_byd {
namespace planning {

class PlanningObstacle {
  PlanningObstacle(const Obstacle* const obstacle_ptr);
  ~PlanningObstacle() = default;

  void SetLatDecision(const ObstacleDecision& decision) {
    lat_decision_ = decision;
  }
  void SetLonDecision(const ObstacleDecision& decision) {
    lon_decision_ = decision;
  }

  const Obstacle* obstacle_ptr() const { return obstacle_ptr_; }
  const ObstacleDecision& lat_decision() const { return lat_decision_; }
  const ObstacleDecision& lon_decision() const { return lon_decision_; }

 private:
  ObstacleDecision lat_decision_ = ObstacleDecision::Decision_None;
  ObstacleDecision lon_decision_ = ObstacleDecision::Decision_None;
  const Obstacle* obstacle_ptr_ = nullptr;
};

using PlanningObstaclePtr = std::shared_ptr<PlanningObstacle>;
using PlanningObstacleConstPtr = std::shared_ptr<const PlanningObstacle>;

}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_PLANNING_OBSTACLE_H