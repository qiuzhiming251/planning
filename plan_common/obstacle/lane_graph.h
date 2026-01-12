

#ifndef AD_BYD_PLANNING_COMMON_OBSTACLELANE_GRAPH_H
#define AD_BYD_PLANNING_COMMON_OBSTACLELANE_GRAPH_H

#include "plan_common/obstacle/lane_sequence.h"

namespace ad_byd {
namespace planning {
namespace obstacle {

class LaneGraph {
 public:
  LaneGraph() = default;
  ~LaneGraph() = default;

  const std::vector<LaneSequence>& lane_sequence() const;
  const std::vector<LaneSequence>& backward_lane_sequence() const;

  bool HasLaneSequence() const;
  bool HasBackwardLaneSequence() const;

  void SetLaneSequence(const std::vector<LaneSequence>& lane_sequence);
  void PushLaneSequence(const LaneSequence& lane_sequence);
  void SetBackwardLaneSequence(
      const std::vector<LaneSequence>& backward_lane_sequence);
  void PushBackwardLaneSequence(const LaneSequence& backward_lane_sequence);

  bool GetLaneSequenceAction(int32_t seq_id,
                             LateralAction* const lateral_action) const;
  bool GetLaneSequenceTurn(int32_t seq_id, TurnType* const turn_type) const;
  std::size_t GetLaneSequenceIndex(int32_t seq_id) const;
  std::size_t GetBackwardLaneSequenceIndex(int32_t seq_id) const;

  std::vector<LaneSequence>* lane_sequence_ptr();
  std::vector<LaneSequence>* backward_lane_sequence_ptr();
  std::string debug_string() const;

 private:
  std::vector<LaneSequence> lane_sequence_;
  std::vector<LaneSequence> backward_lane_sequence_;
  std::unordered_map<int32_t, std::size_t> lane_seq_id_index_map_;
  std::unordered_map<int32_t, std::size_t> backward_seq_id_index_map_;

  bool has_lane_sequence_ = false;
  bool has_backward_lane_sequence_ = false;
};

}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_COMMON_OBSTACLELANE_GRAPH_H