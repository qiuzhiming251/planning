

#ifndef AD_BYD_PLANNING_COMMON_OBSTACLE_PREDICTION_CONSTRAIN_H
#define AD_BYD_PLANNING_COMMON_OBSTACLE_PREDICTION_CONSTRAIN_H

#include "plan_common/math/vec2d.h"

namespace ad_byd {
namespace planning {
namespace obstacle {
using Vec2d = math::Vec2d;

struct PredictionConstrain {
  PredictionConstrain(const Vec2d& point, const double time, const Vec2d& v) {
    limit_point = point;
    relative_time = time;
    velocity = v;
    heading = velocity.Angle();
  }
  PredictionConstrain() {}

  Vec2d limit_point;
  Vec2d velocity;
  Vec2d acceleration;
  double heading = 0.0;
  double relative_time = 0.0;
};
}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_COMMON_OBSTACLE_PREDICTION_CONSTRAIN_H