

#ifndef AD_BYD_COMMON_OBSTACLELANE_SEGMENT_H
#define AD_BYD_COMMON_OBSTACLELANE_SEGMENT_H

#include <string>
#include <vector>

#include "plan_common/maps/lane.h"

using LaneConstPtr = std::shared_ptr<const ::ad_byd::planning::Lane>;

namespace ad_byd {
namespace planning {
namespace obstacle {

class LaneSegment {
 public:
  LaneSegment() = default;
  ~LaneSegment() = default;

  const std::string& lane_id() const;
  double start_s() const;
  double end_s() const;
  const LaneConstPtr& lane_ptr() const;
  double angle_diff() const;

  bool HasLaneId() const;
  bool HasStartS() const;
  bool HasEndS() const;
  bool HasLanePtr() const;
  bool HasAngleDiff() const;

  void SetLaneId(const std::string& lane_id);
  void SetStartS(const double start_s);
  void SetEndS(const double end_s);
  void SetLanePtr(const LaneConstPtr& ptr);
  void SetAngleDiff(const double& angle_diff);

  std::string debug_string() const;

 private:
  std::string lane_id_ = "";
  double start_s_ = 0.0;
  double end_s_ = 0.0;
  LaneConstPtr lane_ptr_ = nullptr;
  double angle_diff_ = 0.0;

  bool has_lane_id_ = false;
  bool has_start_s_ = false;
  bool has_end_s_ = false;
  bool has_lane_ptr_ = false;
  bool has_angle_diff_ = false;
};

}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd

#endif  //  AD_BYD_COMMON_OBSTACLELANE_SEGMENT_H
