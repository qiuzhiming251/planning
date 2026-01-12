

#include "plan_common/obstacle/lane_segment.h"

namespace ad_byd {
namespace planning {
namespace obstacle {

const std::string& LaneSegment::lane_id() const { return lane_id_; }

double LaneSegment::start_s() const { return start_s_; }

double LaneSegment::end_s() const { return end_s_; }

const LaneConstPtr& LaneSegment::lane_ptr() const { return lane_ptr_; }

double LaneSegment::angle_diff() const { return angle_diff_; }

bool LaneSegment::HasLaneId() const { return has_lane_id_; }

bool LaneSegment::HasStartS() const { return has_start_s_; }

bool LaneSegment::HasEndS() const { return has_end_s_; }

bool LaneSegment::HasLanePtr() const { return has_lane_ptr_; }

bool LaneSegment::HasAngleDiff() const { return has_angle_diff_; }

void LaneSegment::SetLaneId(const std::string& lane_id) {
  lane_id_ = lane_id;
  has_lane_id_ = true;
}

void LaneSegment::SetStartS(const double start_s) {
  start_s_ = start_s;
  has_start_s_ = true;
}

void LaneSegment::SetEndS(const double end_s) {
  end_s_ = end_s;
  has_end_s_ = true;
}

void LaneSegment::SetLanePtr(const LaneConstPtr& ptr) {
  lane_ptr_ = ptr;
  has_lane_ptr_ = true;
}

void LaneSegment::SetAngleDiff(const double& angle_diff) {
  has_angle_diff_ = true;
  angle_diff_ = angle_diff;
}

std::string LaneSegment::debug_string() const {
  std::ostringstream oss;
  oss << " lane_id:" << lane_id_ << " start_s:" << start_s_
      << " end_s:" << end_s_;
  return oss.str();
}

}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd