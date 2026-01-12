

#include <algorithm>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "decider/decision_manager/traffic_static_obstacles.h"
#include "plan_common/math/geometry/halfplane.h"

namespace st {
namespace planning {

// This function returns traffic static obstacles constraint.
absl::StatusOr<ConstraintProto::StopLineProto>
BuildTrafficStaticObstaclesConstraint(const DrivePassage& passage) {
  const auto& traffic_static_obstacles_info =
      passage.traffic_static_obstacles_info();
  const BlockReason block_reason =
      passage.traffic_static_obstacles_info().block_reason;
  double end_of_traffic_static_obstacles = traffic_static_obstacles_info.stop_s;

  if (block_reason == BlockReason::CURB_CROSS) {
    end_of_traffic_static_obstacles =
        end_of_traffic_static_obstacles - FLAGS_planner_curb_cutoff_buffer;
  } else if (block_reason != BlockReason::NONE) {
    end_of_traffic_static_obstacles =
        end_of_traffic_static_obstacles - FLAGS_planner_obstacle_cutoff_buffer;
  }

  const auto curbs = passage.QueryCurbPointAtS(end_of_traffic_static_obstacles);
  if (!curbs.ok()) {
    return absl::NotFoundError(
        absl::StrFormat("Curb boundaries not found at s=%.2f.",
                        end_of_traffic_static_obstacles));
  }
  const HalfPlane halfplane(curbs->first, curbs->second);

  ConstraintProto::StopLineProto end_of_traffic_static_obstacles_constraint;
  end_of_traffic_static_obstacles_constraint.set_s(
      end_of_traffic_static_obstacles);
  end_of_traffic_static_obstacles_constraint.set_standoff(3.5);
  end_of_traffic_static_obstacles_constraint.set_time(0.0);
  halfplane.ToProto(
      end_of_traffic_static_obstacles_constraint.mutable_half_plane());
  end_of_traffic_static_obstacles_constraint.set_id(
      "end_of_traffic_static_obstacles");
  end_of_traffic_static_obstacles_constraint.mutable_source()
      ->mutable_traffic_static_obstacles()
      ->set_id(BlockReason_Name(traffic_static_obstacles_info.block_reason));
  return end_of_traffic_static_obstacles_constraint;
}

}  // namespace planning
}  // namespace st
