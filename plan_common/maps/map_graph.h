

#ifndef AD_BYD_PLANNING_MAP_MAP_GRAPH_H
#define AD_BYD_PLANNING_MAP_MAP_GRAPH_H

#include <list>
#include <unordered_map>

#include "plan_common/maps/lane.h"
#include "plan_common/maps/map.h"
#include "plan_common/maps/map_graph.h"

namespace ad_byd {
namespace planning {

class MapGraph {
 public:
  MapGraph(const LaneConstPtr& lane_ptr, const double successor_range,
           const double predecessor_range, const MapConstPtr& map_ptr);
  MapGraph() = default;
  ~MapGraph() = default;

  const std::list<std::list<LaneConstPtr>>& successor_lane_sequence() const {
    return successor_lane_sequences_;
  };
  const std::list<std::list<LaneConstPtr>>& predecessor_lane_sequence() const {
    return predecessor_lane_sequences_;
  };
  const LaneConstPtr& lane_ptr() const { return current_lane_; };

 private:
  void BuildGraph(const LaneConstPtr& lane_ptr, const MapConstPtr& map_ptr);
  void BuildSuccessorLaneSequence(
      const double accumulated_lane_s, const double start_lane_s,
      const MapConstPtr& map_ptr, const LaneConstPtr& lane_ptr,
      std::list<LaneConstPtr>* const history_lane_ptrs, uint32_t uturn_count);
  void BuildPredecessorLaneSequence(
      const double accumulated_lane_s, const double end_lane_s,
      const MapConstPtr& map_ptr, const LaneConstPtr& lane_ptr,
      std::list<LaneConstPtr>* const history_lane_ptrs);

 private:
  double successor_range_ = 0.0;
  double predecessor_range_ = 0.0;
  LaneConstPtr current_lane_ = nullptr;
  std::list<std::list<LaneConstPtr>> successor_lane_sequences_;
  std::list<std::list<LaneConstPtr>> predecessor_lane_sequences_;
};

typedef std::shared_ptr<MapGraph> MapGraphPtr;
typedef std::shared_ptr<const MapGraph> MapGraphConstPtr;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_MAP_GRAPH_H
