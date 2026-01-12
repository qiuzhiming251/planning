

#pragma once
#include "plan_common/log.h"
#include "trajectory.h"
#include "plan_common/type_def.h"
#include "plan_common/math/polygon2d.h"

namespace ad_byd {
namespace planning {
bool CalPolygonList(const math::Polygon2d &polygon,
                    const std::vector<TrajectoryPoint> &future,
                    std::vector<math::Polygon2d> *res);
class OfflineObstacle {
 public:
  OfflineObstacle() = delete;
  OfflineObstacle(const std::string &id, const ObstacleType obstacle_type,
                  const math::Polygon2d &polygon,
                  const std::vector<TrajectoryPtr> &future_trajectory,
                  const bool is_static);
  OfflineObstacle(const std::string &id, const ObstacleType obstacle_type,
                  const math::Polygon2d &polygon,
                  const bool is_static);  // TODO 静态障碍物 ERROR
  ~OfflineObstacle();
  void GetInfo(std::string &id, ObstacleType &obstacle_type,
               math::Polygon2d &polygon, bool &is_static);
  const std::string &GetId() const { return id_; };
  ObstacleType GetObsType() const { return obstacle_type_; };
  const math::Polygon2d &GetPolygon() const { return polygon_; };
  // void SetInfo(const std::string &id, ObstacleType &obstacle_type,
  //              const math::Polygon2d &polygon, const bool &is_static);
  bool IsStatic() { return is_static_; }
  TrajectoryPtr GetFutureTrajectory();  // TODO 静态障碍物 ERROR
  std::vector<TrajectoryPtr>
  GetFutureMultiTrajectory();  // TODO 静态障碍物 ERROR
  // TrajectoryPoint GetCurrentPoint() const; // TODO 静态障碍物 ERROR
  // 返回future第一个点；多条future轨迹共享第一个点。
  DecisionLabel GetDecisionLabel();
  const std::vector<math::Polygon2d> &GetFuturePolygonList();
  void SetDecisionLabel(const DecisionLabel &);

 private:
  /* data */
  std::string id_;
  bool is_static_;
  math::Polygon2d polygon_;
  TrajectoryPtr history_trajectory_;
  std::vector<TrajectoryPtr> future_trajectory_;  //! 包含当前时间
  std::vector<math::Polygon2d> polygon_list_;
  DecisionLabel decision_label_;
  ObstacleType obstacle_type_;
};
typedef std::shared_ptr<OfflineObstacle> OfflineObstaclePtr;
}  // namespace planning
}  // namespace ad_byd
