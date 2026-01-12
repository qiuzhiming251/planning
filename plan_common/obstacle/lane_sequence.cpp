

#include "plan_common/obstacle/lane_sequence.h"
#include "plan_common/math/double.h"

namespace ad_byd {
namespace planning {
namespace obstacle {

using Vec2d = ::ad_byd::planning::math::Vec2d;

void LaneSequence::SetSeqId(const int32_t seq_id) {
  seq_id_ = seq_id;
  has_seq_id_ = true;
}
void LaneSequence::PushLaneSegment(const LaneSegment& lane_segment) {
  lane_index_map_.insert(
      std::make_pair(lane_segment.lane_id(), lane_segments_.size()));
  lane_segments_.push_back(lane_segment);
  has_lane_segments_ = true;
}
// void LaneSequence::push_lane_seq_obstacles(const NearbyObstacle&
// lane_obstacle) {
//   lane_seq_obstacles_.push_back(lane_obstacle);
//   has_lane_seq_obstacles_ = true;
// }
void LaneSequence::SetLaneSeqAction(const LateralAction& lane_seq_action) {
  lane_seq_action_ = lane_seq_action;
  has_lane_seq_action_ = true;
}
void LaneSequence::SetLaneSeqTurnType(TurnType turn_type) {
  lane_seq_turn_type_ = turn_type;
  HasLaneSeqTurnType_ = true;
}
void LaneSequence::SetDistanceToTurn(const double& distance) {
  has_distance_to_turn_ = true;
  distance_to_turn_ = distance;
}
void LaneSequence::SetTurnIndex(const std::size_t& turn_index) {
  has_turn_index_ = true;
  turn_index_ = turn_index;
}
void LaneSequence::SetSignalIndex(const std::size_t& signal_index) {
  HasSignalIndex_ = true;
  signal_index_ = signal_index;
}
void LaneSequence::SetTrafficAction(const TrafficAction& traffic_action) {
  has_traffic_action_ = true;
  traffic_action_ = traffic_action;
}
bool LaneSequence::DistanceTo(const Vec2d& pt, double* const dist) const {
  if (lane_segments_.empty()) {
    return false;
  }
  SLPoint sl_point(0.0, 0.0);
  for (const auto& lane_seg : lane_segments_) {
    if (lane_seg.lane_ptr() == nullptr) {
      return false;
    }
    lane_seg.lane_ptr()->GetSLWithoutLimit(pt, &sl_point);
    if (sl_point.s < 0.0) {
      *dist = std::fabs(sl_point.l);
      return true;
    }
    if (math::Double::Compare(sl_point.s, 0.0, 1e-2) !=
            math::Double::CompareType::LESS &&
        math::Double::Compare(sl_point.s, lane_seg.lane_ptr()->topo_length(),
                              1e-2) != math::Double::CompareType::GREATER) {
      *dist = std::fabs(sl_point.l);
      return true;
    }
  }
  if (lane_segments_.back().lane_ptr() == nullptr) {
    return false;
  }
  *dist = std::fabs(sl_point.l);
  return true;
}
std::string LaneSequence::debug_string() const {
  std::ostringstream oss;
  oss << "LaneSequence: "
      << " seq_id: " << seq_id_
      << " lane_seq_action:" << kLateralAction2Name.at(lane_seq_action_);

  oss << "\n    lane_segments: [";
  for (const auto& lane_segment : lane_segments_) {
    oss << lane_segment.debug_string() << ",";
  }
  oss << "]";

  // oss << "\n    lane_seq_obstacles:";
  // for (const auto& lane_seq_obstacle : lane_seq_obstacles_) {
  //   oss << "\n   - " << lane_seq_obstacle.debug_string();
  // }

  return oss.str();
}

}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd