#pragma once

#include <string>

namespace st::planning {
struct OvertakeFrameInput {
  double leader_speed_diff = 0.0;
  double ego_v = 0.0;
  int valid_lane_num = 0;
  bool lc_left = false;
  bool on_highway = false;
  std::string DebugString() const;
};

int CalcOvertakeFrame(const OvertakeFrameInput &overtake_begin_frame_input,
                      bool is_conversative_style);
}  // namespace st::planning
