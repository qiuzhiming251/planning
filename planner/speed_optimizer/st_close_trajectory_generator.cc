

#include <algorithm>
#include <string>
#include <utility>

#include "planner/speed_optimizer/st_close_trajectory_generator.h"

// IWYU pragma: no_include "Eigen/Core"

#include "absl/strings/str_format.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "predictor/predicted_trajectory.h"
#include "plan_common/maps/st_boundary.h"

namespace st::planning {
namespace {
using StNearestPoint = StCloseTrajectory::StNearestPoint;
using CloseTrajPoints = std::vector<StNearestPoint>;
}  // namespace
std::vector<StCloseTrajectory> GenerateMovingStCloseTrajectories(
    const SpacetimeObjectTrajectory& st_traj,
    std::vector<CloseTrajPoints> close_traj_points) {
  std::vector<StCloseTrajectory> st_close_trajs;
  const auto object_type = ToStBoundaryObjectType(st_traj.object_type());
  if (close_traj_points.empty()) return st_close_trajs;

  const int traj_size = close_traj_points.size();
  st_close_trajs.reserve(traj_size);
  for (int i = 0; i < traj_size; ++i) {
    if (close_traj_points[i].empty()) continue;
    auto traj_id = static_cast<std::string>(st_traj.traj_id());
    auto object_id = static_cast<std::string>(st_traj.object_id());
    std::string id =
        traj_size == 1 ? traj_id : absl::StrFormat("%s|%d", traj_id, i);

    st_close_trajs.emplace_back(std::move(close_traj_points[i]), std::move(id),
                                std::move(traj_id), std::move(object_id),
                                object_type, st_traj.trajectory().probability(),
                                st_traj.is_stationary());
  }
  return st_close_trajs;
}

}  // namespace st::planning
