

#ifndef AD_BYD_PLANNING_COMMON_OBSTACLELANE_SEQUENCE_H
#define AD_BYD_PLANNING_COMMON_OBSTACLELANE_SEQUENCE_H

#include <unordered_map>
#include <vector>

#include "plan_common/obstacle/lane_segment.h"
#include "plan_common/type_def.h"
#include "plan_common/math/vec2d.h"
//#include "predictor/container/obstacles/nearby_obstacle.h"

namespace ad_byd {
namespace planning {
namespace obstacle {

enum class LateralAction {
  KEEP_LANE = 1,
  CUTIN_LEFT = 2,
  CUTIN_RIGHT = 3,
  UNKNOWN = 4,
};

enum class TrafficAction { UNKNOWN = 0, GO = 1, STOP = 2 };

static const std::unordered_map<LateralAction, std::string>
    kLateralAction2Name = {{LateralAction::KEEP_LANE, "KEEP_LANE"},
                           {LateralAction::CUTIN_LEFT, "CUTIN_LEFT"},
                           {LateralAction::CUTIN_RIGHT, "CUTIN_RIGHT"},
                           {LateralAction::UNKNOWN, "UNKNOWN"}};

class LaneSequence {
 public:
  LaneSequence() = default;
  ~LaneSequence() = default;

  int32_t seq_id() const { return seq_id_; }
  const std::vector<LaneSegment>& lane_segments() const {
    return lane_segments_;
  }
  // const std::vector<NearbyObstacle>& lane_seq_obstacles() const { return
  // lane_seq_obstacles_; }
  const LateralAction& lane_seq_action() const { return lane_seq_action_; }
  TurnType lane_seq_turn_type() const { return lane_seq_turn_type_; }
  std::size_t turn_index() const { return turn_index_; }
  std::size_t signal_index() const { return signal_index_; }
  const TrafficAction& traffic_action() const { return traffic_action_; }
  double distance_to_turn() const { return distance_to_turn_; }
  std::string debug_string() const;
  bool DistanceTo(const math::Vec2d& pt, double* const dist) const;

  bool HasSeqId() const { return has_seq_id_; }
  bool HasLaneSegments() const { return has_lane_segments_; }
  // bool has_lane_seq_obstacles() const { return has_lane_seq_obstacles_; }
  bool HasLaneSeqAction() const { return has_lane_seq_action_; }
  bool HasLaneSeqTurnType() const { return HasLaneSeqTurnType_; }
  bool HasSignalIndex() const { return HasSignalIndex_; }
  bool HasTrafficAction() const { return has_traffic_action_; }
  bool HasLaneId(const std::string& lane_id, std::size_t* const index) const {
    if (lane_index_map_.find(lane_id) != lane_index_map_.end()) {
      *index = lane_index_map_.at(lane_id);
      return true;
    }
    return false;
  }
  bool HasDistanceToTurn() const { return has_distance_to_turn_; }
  bool HasTurnIndex() const { return has_turn_index_; }

  void SetSeqId(const int32_t seq_id);
  void PushLaneSegment(const LaneSegment& lane_segment);
  // void push_lane_seq_obstacles(const NearbyObstacle& lane_obstacle);
  void SetLaneSeqAction(const LateralAction& lane_sequence_action);
  void SetLaneSeqTurnType(TurnType turn_type);
  void SetDistanceToTurn(const double& distance);
  void SetTurnIndex(const std::size_t& turn_index);
  void SetSignalIndex(const std::size_t& signal_index);
  void SetTrafficAction(const TrafficAction& traffic_action);

 private:
  int32_t seq_id_ = 0;
  std::vector<LaneSegment> lane_segments_;
  // std::vector<NearbyObstacle> lane_seq_obstacles_;  // sorted by s
  std::unordered_map<std::string, std::size_t> lane_index_map_;
  LateralAction lane_seq_action_ = LateralAction::UNKNOWN;
  TurnType lane_seq_turn_type_ = TurnType::NO_TURN;
  double distance_to_turn_ = 0.0;
  std::size_t turn_index_ = 0;
  std::size_t signal_index_ = 0;
  TrafficAction traffic_action_ = TrafficAction::UNKNOWN;

  bool has_seq_id_ = false;
  bool has_lane_segments_ = false;
  // bool has_lane_seq_obstacles_ = false;
  bool has_lane_seq_action_ = false;
  bool HasLaneSeqTurnType_ = false;
  bool has_distance_to_turn_ = false;
  bool has_turn_index_ = false;
  bool HasSignalIndex_ = false;
  bool has_traffic_action_ = false;
};

}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd

#endif  //  AD_BYD_PLANNING_COMMON_OBSTACLELANE_SEQUENCE_H