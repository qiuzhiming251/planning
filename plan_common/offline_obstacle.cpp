

#include "offline_obstacle.h"

namespace ad_byd {
namespace planning {
bool CalPolygonList(const math::Polygon2d &polygon,
                    const std::vector<TrajectoryPoint> &future,
                    std::vector<math::Polygon2d> *res) {
  if (!res) {
    return false;
  }
  std::vector<math::Vec2d> sdv_corners = polygon.points();
  for (const auto &p : future) {
    math::Vec2d pos(p.x(), p.y());
    auto theta = p.theta;
    std::vector<math::Vec2d> moved_points;
    for (const auto &corner : sdv_corners) {
      moved_points.emplace_back(corner.rotate(theta) + pos);
    }
    math::Polygon2d future_polygon(moved_points);
    res->emplace_back(future_polygon);
  }
  return true;
}

OfflineObstacle::OfflineObstacle(
    const std::string &id, const ObstacleType obstacle_type,
    const math::Polygon2d &polygon,
    const std::vector<TrajectoryPtr> &future_trajectory, const bool is_static) {
  id_ = id;
  polygon_ = polygon;
  future_trajectory_ = future_trajectory;
  is_static_ = is_static;
  obstacle_type_ = obstacle_type;
}

OfflineObstacle::OfflineObstacle(const std::string &id,
                                 const ObstacleType obstacle_type,
                                 const math::Polygon2d &polygon,
                                 const bool is_static) {
  id_ = id;
  polygon_ = polygon;
  is_static_ = is_static;
  obstacle_type_ = obstacle_type;
}

OfflineObstacle::~OfflineObstacle() {}

void OfflineObstacle::GetInfo(std::string &id, ObstacleType &obstacle_type,
                              math::Polygon2d &polygon, bool &is_static) {
  id = id_, polygon = polygon_, is_static = is_static_,
  obstacle_type = obstacle_type_;
}

// void OfflineObstacle::SetInfo(const std::string &id, ObstacleType
// &obstacle_type,
//                          const math::Polygon2d &polygon, const bool
//                          &is_static) {
//   id_ = id, polygon_ = polygon, is_static_ = is_static, obstacle_type_ =
//   obstacle_type;
// }

TrajectoryPtr OfflineObstacle::GetFutureTrajectory() {
  // TODO sdv->multi_traj[top1_idx]
  // 自车一般来说不会调用这个函数；如果调用如何返回？
  // TODO obs->multi_traj[0] 障碍物目前只有1条未来轨迹。
  if (!is_static_) {
    return future_trajectory_[0];
  }
  return nullptr;
}

//
std::vector<TrajectoryPtr> OfflineObstacle::GetFutureMultiTrajectory() {
  return future_trajectory_;
}

void OfflineObstacle::SetDecisionLabel(const DecisionLabel &label) {
  decision_label_ = label;
}
DecisionLabel OfflineObstacle::GetDecisionLabel() { return decision_label_; }
const std::vector<math::Polygon2d> &OfflineObstacle::GetFuturePolygonList() {
  if (polygon_list_.empty()) {
    if (is_static_) {
      polygon_list_.emplace_back(polygon_);
    } else {
      auto future_points = GetFutureTrajectory()->points();
      CalPolygonList(polygon_, future_points, &polygon_list_);
    }
  }
  return polygon_list_;
}
}  // namespace planning
}  // namespace ad_byd
