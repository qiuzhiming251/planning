

#ifndef ST_PLANNING_COMMON_LANE_PATH_INFO
#define ST_PLANNING_COMMON_LANE_PATH_INFO

#include <vector>

#include <float.h>

#include "lane_path.h"
#include "lane_point.h"
#include "lane_sequence.h"
#include "semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/vec.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st::planning {

class LanePathInfo {
 public:
  LanePathInfo() {}
  LanePathInfo(mapping::LanePath lane_path, double len_along_route,
               double path_cost, const PlannerSemanticMapManager& psmm);

  bool empty() const { return lane_path_.IsEmpty(); }
  const mapping::LanePath& lane_path() const { return lane_path_; }
  mapping::ElementId start_lane_id() const {
    return lane_path_.front().lane_id();
  }

  // If a routed lane change must be executed on this lane path, length
  // decreases by a default lane change distance.
  //
  // [— length_along_route —]
  // ----------------------------------------->
  //
  // [————— length_along_route —————]
  // ----------------------------------------->
  //
  // ------------------------------------------
  //                                          |
  //                                          |
  //                                      destination
  double length_along_route() const { return length_along_route_; }
  double max_reach_length() const { return max_reach_length_; }
  double path_cost() const { return path_cost_; }

  void set_length_along_route(double length_along_route) {
    length_along_route_ = length_along_route;
  }
  void set_max_reach_length(double reach_length) {
    max_reach_length_ = reach_length;
  }

  FrenetCoordinate ProjectionSL(const Vec2d& xy) const;
  FrenetCoordinate ProjectionSLInRange(const Vec2d& xy, double start_s,
                                       double end_s) const;
  Vec2d ProjectionXY(FrenetCoordinate sl) const;

  const ad_byd::planning::LaneSequencePtr& lane_seq() const {
    return lane_path_.lane_seq();
  }

  void set_lane_seq(const ad_byd::planning::LaneSequencePtr& lane_seq) {
    lane_path_.set_lane_seq(lane_seq);
  }
  void set_lane_seq_info(const ad_byd::planning::LaneSeqInfo& lane_seq_info) {
    lane_seq_info_ =
        std::make_shared<ad_byd::planning::LaneSeqInfo>(lane_seq_info);
  }
  const ad_byd::planning::LaneSeqInfoPtr lane_seq_info() const {
    return lane_seq_info_;
  }

  void set_seq_info_lc_reason(ad_byd::planning::LcReason lc_reason) {
    lane_seq_info_->lc_reason = lc_reason;
  }

 private:
  mapping::LanePath lane_path_;
  double length_along_route_ = 0.0;
  double max_reach_length_ = 0.0;
  double path_cost_ = DBL_MAX;
  ad_byd::planning::LaneSeqInfoPtr lane_seq_info_ = nullptr;

  // Projection system
  std::vector<Vec2d> anchor_points_;
  std::vector<double> anchor_s_;
  std::vector<Vec2d> tangents_;
  std::vector<double> segment_len_inv_;

  // TODO: potentially add lane path boundary here.
  // speed_limit
};

}  // namespace st::planning

#endif  // ST_PLANNING_COMMON_LANE_PATH_INFO
