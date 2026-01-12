

#include "planner/planner_manager/manual_trajectory_util.h"

#include <ostream>

#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st {
namespace planning {

void CompleteTrajectoryPastPoints(double trajectory_time_step,
                                  TrajectoryProto* trajectory) {
  constexpr int kMinPastPointsSize = 3;
  if (trajectory->past_points_size() >= kMinPastPointsSize) return;

  auto first_traj_point = trajectory->trajectory_point()[0];
  for (int i = trajectory->past_points_size(); i < kMinPastPointsSize; ++i) {
    first_traj_point.set_relative_time(-i * trajectory_time_step);
    *trajectory->add_past_points() = first_traj_point;
  }
}

void ShiftPreviousTrajectory(const absl::Duration shift_duration,
                             TrajectoryProto* trajectory) {
  if (trajectory->trajectory_point_size() == 0) {
    // LOG_EVERY_N_SEC(WARNING, 5) << "Reach the end of manual trajectory.";
    return;
  }

  const int past_point_num = trajectory->past_points_size();
  const int traj_point_num = trajectory->trajectory_point_size();

  const auto point = [&](int index) {
    CHECK_GE(index, 0);
    CHECK_LT(index, past_point_num + traj_point_num);
    return index < past_point_num
               ? trajectory->mutable_past_points(index)
               : trajectory->mutable_trajectory_point(index - past_point_num);
  };

  for (int i = 0; i < past_point_num + traj_point_num; ++i) {
    auto* traj_point = point(i);
    const double relative_time_updated =
        traj_point->relative_time() - absl::ToDoubleSeconds(shift_duration);
    traj_point->set_relative_time(relative_time_updated);
  }

  while (trajectory->trajectory_point()[0].relative_time() < 0.0) {
    *trajectory->add_past_points() = trajectory->trajectory_point()[0];
    trajectory->mutable_trajectory_point()->DeleteSubrange(0, 1);
  }
}

void UpdateTrajectoryPointAccel(TrajectoryProto* trajectory) {
  CHECK_GE(trajectory->trajectory_point_size(), 2);
  const int past_point_num = trajectory->past_points_size();
  const int traj_point_num = trajectory->trajectory_point_size();

  const auto point = [&](int index) {
    CHECK_LT(index, past_point_num + traj_point_num);
    return index < past_point_num
               ? trajectory->mutable_past_points(index)
               : trajectory->mutable_trajectory_point(index - past_point_num);
  };

  constexpr int kSmoothFilterCycle = 3;
  for (int j = 0; j < kSmoothFilterCycle; ++j) {
    for (int i = 1; i + 1 < past_point_num + traj_point_num; ++i) {
      const double prev_a = point(i - 1)->a();
      const double curr_a = point(i)->a();
      const double next_a = point(i + 1)->a();

      point(i)->set_a((prev_a + 2 * curr_a + next_a) * 0.25);
    }
  }
}

}  // namespace planning
}  // namespace st
