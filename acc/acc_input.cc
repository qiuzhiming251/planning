#include "acc/acc_input.h"

#include <sstream>
#include <string>

namespace st::planning {

std::string AccInput::DebugString() const {
  std::stringstream ss;
  ss << "front_wheel_angle: " << front_wheel_angle
     << ", acc_standwait: " << acc_standwait << ", start_point_info: "
     << (start_point_info != nullptr ? start_point_info->DebugString() : "")
     << ", prev_trajectory: "
     << (prev_trajectory != nullptr ? prev_trajectory->ShortDebugString()
                                    : "is null ptr")
     << ", prev_lane_change_type: "
     << LaneChangeType_Name(prev_lane_change_type) << ", st_traj_mgr size: "
     << (st_traj_mgr != nullptr ? st_traj_mgr->trajectories().size() : 0)
     << ", time_aligned_prev_traj: "
     << (time_aligned_prev_traj != nullptr ? "Not null." : "")
     << ", psmm is null? " << (psmm == nullptr ? "Y" : "N");

  return ss.str();
}

}  // namespace st::planning
