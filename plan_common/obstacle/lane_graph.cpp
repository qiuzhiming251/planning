

#include "plan_common/obstacle/lane_graph.h"

namespace ad_byd {
namespace planning {
namespace obstacle {

const std::vector<LaneSequence>& LaneGraph::lane_sequence() const {
  return lane_sequence_;
}

const std::vector<LaneSequence>& LaneGraph::backward_lane_sequence() const {
  return backward_lane_sequence_;
}

bool LaneGraph::HasLaneSequence() const { return has_lane_sequence_; }

bool LaneGraph::HasBackwardLaneSequence() const {
  return has_backward_lane_sequence_;
}

void LaneGraph::SetLaneSequence(
    const std::vector<LaneSequence>& lane_sequence) {
  lane_sequence_ = lane_sequence;
  for (std::size_t i = 0; i < lane_sequence_.size(); ++i) {
    lane_seq_id_index_map_[lane_sequence_.at(i).seq_id()] = i;
  }
  has_lane_sequence_ = true;
}

void LaneGraph::PushLaneSequence(const LaneSequence& lane_sequence) {
  lane_seq_id_index_map_[lane_sequence.seq_id()] = lane_sequence_.size();
  lane_sequence_.push_back(lane_sequence);
  has_lane_sequence_ = true;
}

void LaneGraph::SetBackwardLaneSequence(
    const std::vector<LaneSequence>& backward_lane_sequence) {
  backward_lane_sequence_ = backward_lane_sequence;
  has_backward_lane_sequence_ = true;
}

void LaneGraph::PushBackwardLaneSequence(
    const LaneSequence& backward_lane_sequence) {
  backward_seq_id_index_map_[backward_lane_sequence.seq_id()] =
      backward_lane_sequence_.size();
  backward_lane_sequence_.push_back(backward_lane_sequence);
  has_backward_lane_sequence_ = true;
}

std::vector<LaneSequence>* LaneGraph::lane_sequence_ptr() {
  return &lane_sequence_;
}

std::vector<LaneSequence>* LaneGraph::backward_lane_sequence_ptr() {
  return &backward_lane_sequence_;
}

bool LaneGraph::GetLaneSequenceAction(
    int32_t seq_id, LateralAction* const lateral_action) const {
  if (lane_seq_id_index_map_.find(seq_id) == lane_seq_id_index_map_.end() ||
      lane_seq_id_index_map_.at(seq_id) >= lane_sequence_.size()) {
    return false;
  }

  *lateral_action =
      lane_sequence_.at(lane_seq_id_index_map_.at(seq_id)).lane_seq_action();
  return true;
}

bool LaneGraph::GetLaneSequenceTurn(int32_t seq_id,
                                    TurnType* const turn_type) const {
  if (lane_seq_id_index_map_.find(seq_id) == lane_seq_id_index_map_.end() ||
      lane_seq_id_index_map_.at(seq_id) >= lane_sequence_.size()) {
    return false;
  }

  *turn_type =
      lane_sequence_.at(lane_seq_id_index_map_.at(seq_id)).lane_seq_turn_type();
  return true;
}

std::size_t LaneGraph::GetLaneSequenceIndex(int32_t seq_id) const {
  if (lane_seq_id_index_map_.find(seq_id) == lane_seq_id_index_map_.end() ||
      lane_seq_id_index_map_.at(seq_id) >= lane_sequence_.size()) {
    return lane_sequence_.size();
  }
  return lane_seq_id_index_map_.at(seq_id);
}

std::size_t LaneGraph::GetBackwardLaneSequenceIndex(int32_t seq_id) const {
  if (backward_seq_id_index_map_.find(seq_id) ==
          backward_seq_id_index_map_.end() ||
      backward_seq_id_index_map_.at(seq_id) >= backward_lane_sequence_.size()) {
    return backward_lane_sequence_.size();
  }
  return backward_seq_id_index_map_.at(seq_id);
}

std::string LaneGraph::debug_string() const {
  std::ostringstream oss;
  oss << "LaneGraph:";
  oss << "\n  forward_lane_sequence:";
  for (const auto& lane_sequence : lane_sequence_) {
    oss << " \n   -" << lane_sequence.debug_string();
  }
  oss << "\n  backward_lane_sequence:";
  for (const auto& lane_sequence : backward_lane_sequence_) {
    oss << " \n   -" << lane_sequence.debug_string();
  }
  return oss.str();
}

}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd