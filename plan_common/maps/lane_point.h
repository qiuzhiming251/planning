

#ifndef ONBOARD_MAPS_LANE_POINT_H_
#define ONBOARD_MAPS_LANE_POINT_H_

#include <algorithm>
#include <string>
#include <cereal/access.hpp>

#include "absl/strings/str_format.h"
#include "plan_common/maps/map.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/lane_point.pb.h"

namespace st {
namespace mapping {

class LanePoint {
 public:
  LanePoint() = default;
  LanePoint(ElementId lane_id, double fraction)
      : lane_id_(lane_id), fraction_(std::clamp(fraction, 0.0, 1.0)) {}
  explicit LanePoint(const LanePointProto& proto) { FromProto(proto); }

  ElementId lane_id() const { return lane_id_; }
  double fraction() const { return fraction_; }
  void set_fraction(double fraction) { fraction_ = fraction; }

  bool Valid() const {
    return lane_id_ != kInvalidElementId && fraction_ >= 0.0 &&
           fraction_ <= 1.0;
  }

  Vec2d ComputePos(const ad_byd::planning::MapConstPtr& map_ptr) const;
  Vec2d ComputeTangent(const ad_byd::planning::MapConstPtr& map_ptr) const;
  double ComputeLerpTheta(const ad_byd::planning::MapConstPtr& map_ptr) const;
  double ComputeLerpThetaWithSuccessorLane(
      const ad_byd::planning::MapConstPtr& map_ptr,
      ElementId succ_lane_id) const;

  std::string DebugString() const {
    return absl::StrFormat("{%d:%.6f}", lane_id_, fraction_);
  }

  bool operator==(const LanePoint& other) const {
    return lane_id_ == other.lane_id_ && fraction_ == other.fraction_;
  }
  bool operator!=(const LanePoint& other) const { return !(*this == other); }

  void ToProto(LanePointProto* proto) const {
    proto->set_lane_id(lane_id_);
    proto->set_fraction(fraction_);
  }

  void FromProto(const LanePointProto& proto) {
    lane_id_ = ElementId(proto.lane_id());
    fraction_ = std::clamp(proto.fraction(), 0.0, 1.0);
  }

 private:
  ElementId lane_id_ = mapping::kInvalidElementId;
  double fraction_ = 0.0;

  template <typename Archive>
  friend void serialize(Archive& ar, LanePoint& lane_point);
};

}  // namespace mapping
}  // namespace st

#endif  // ONBOARD_MAPS_LANE_POINT_H_
