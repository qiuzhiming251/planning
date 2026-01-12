

#ifndef AD_BYD_PLANNING_COMMON_PLANNING_DATA_H
#define AD_BYD_PLANNING_COMMON_PLANNING_DATA_H
#include "model_behavior.h"
#include "plan_common/path/path.h"
#include "planning_obstacle.h"
#include "plan_common/ref_line/reference_line.h"
#include "plan_common/speed/ad_byd_speed/speed_profile.h"
#include "trajectory.h"
#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {

class PlanningData {
 public:
  PlanningData() = default;
  ~PlanningData() = default;

  void Reset() {
    reference_line_ = nullptr;
    planning_obstacles_.clear();
    heuristic_trajectory_.Reset();
    path_.Reset();
    speed_profile_.Reset();
    planning_trajectory_.Reset();
  }

  const TrajectoryPoint &start_point() const { return start_point_; }
  void set_start_point(const TrajectoryPoint &start_point) {
    start_point_ = start_point;
  }

  const ReferenceLinePtr &reference_line() const { return reference_line_; };
  ReferenceLinePtr &mutable_reference_line() { return reference_line_; };

  const std::vector<PlanningObstaclePtr> &obstacles() const {
    return planning_obstacles_;
  }
  std::vector<PlanningObstaclePtr> *mutable_obstacles() {
    return &planning_obstacles_;
  }

  const Path &path() const { return path_; };
  Path &mutable_path() { return path_; };

  const SpeedProfile &speed_profile() const { return speed_profile_; };
  SpeedProfile &mutable_speed_profile() { return speed_profile_; };

  const Trajectory &planning_trajectory() const {
    return planning_trajectory_;
  };
  Trajectory &mutable_planning_trajectory() { return planning_trajectory_; };

  const TaskModelBehavior &task_model_behavior() const {
    return task_model_behavior_;
  }
  void SetTaskModelBehavior(const TaskModelBehavior &task_model_behavior) {
    task_model_behavior_ = task_model_behavior;
  }

 private:
  TrajectoryPoint start_point_;
  ReferenceLinePtr reference_line_;
  std::vector<PlanningObstaclePtr> planning_obstacles_;
  Trajectory heuristic_trajectory_;
  Path path_;
  SpeedProfile speed_profile_;
  Trajectory planning_trajectory_;
  TaskModelBehavior task_model_behavior_;
};

using PlanningDataPtr = std::shared_ptr<PlanningData>;
using PlanningDataConstPtr = std::shared_ptr<const PlanningData>;
using PlanningDataVecPtr = std::shared_ptr<std::vector<PlanningDataPtr>>;
using PlanningDataVecConstPtr =
    std::shared_ptr<const std::vector<PlanningDataPtr>>;

}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_PLANNING_DATA_H
