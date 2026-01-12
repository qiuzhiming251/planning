

#ifndef ST_PLANNING_MAPS_LANE_PATH_DATA
#define ST_PLANNING_MAPS_LANE_PATH_DATA

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <stddef.h>

#include <string>
#include <utility>
#include <vector>

// #include "global/buffered_logger.h"
// #include "lite/check.h"
#include <cereal/access.hpp>
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/log.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"

namespace st {
namespace mapping {

class LanePathData {
 public:
  LanePathData() = default;
  LanePathData(double start_fraction, double end_fraction,
               std::vector<ElementId> lane_ids,
               bool lane_path_in_forward_direction = true)
      : start_fraction_(start_fraction),
        end_fraction_(end_fraction),
        lane_ids_(std::move(lane_ids)),
        lane_path_in_forward_direction_(lane_path_in_forward_direction) {}

  explicit LanePathData(const LanePathProto& lane_path_proto) {
    FromLanePathProto(lane_path_proto);
  }

  explicit LanePathData(const LanePoint& lane_point)
      : start_fraction_(lane_point.fraction()),
        end_fraction_(lane_point.fraction()),
        lane_ids_({lane_point.lane_id()}),
        lane_path_in_forward_direction_(true) {}

  bool empty() const { return lane_ids_.empty(); }
  size_t size() const { return lane_ids_.size(); }

  double start_fraction() const { return start_fraction_; }
  double end_fraction() const { return end_fraction_; }
  bool lane_path_in_forward_direction() const {
    return lane_path_in_forward_direction_;
  }

  const std::vector<ElementId>& lane_ids() const { return lane_ids_; }
  ElementId lane_id(size_t index) const {
    CHECK_LT(index, lane_ids_.size());
    return lane_ids_[index];
  }

  LanePoint start_point() const {
    CHECK(!empty());
    return {lane_ids_.front(), start_fraction_};
  }

  LanePoint end_point() const {
    CHECK(!empty());
    return {lane_ids_.back(), end_fraction_};
  }

  bool operator==(const LanePathData& other) const {
    if (start_fraction_ != other.start_fraction()) return false;
    if (end_fraction_ != other.end_fraction()) return false;
    if (lane_path_in_forward_direction_ !=
        other.lane_path_in_forward_direction())
      return false;

    if (lane_ids_.size() != other.size()) return false;
    for (size_t i = 0; i < lane_ids_.size(); ++i) {
      if (lane_ids_[i] != other.lane_id(i)) return false;
    }
    return true;
  }

  bool operator!=(const LanePathData& other) const { return !(*this == other); }

  void FromLanePathProto(const LanePathProto& lane_path_proto);
  void ToLanePathProto(LanePathProto* lane_path_proto) const;

  void Clear();

  std::string DebugString() const;

  struct LaneSegmentData {
    ElementId lane_id;
    double start_fraction;
    double end_fraction;
  };

  LaneSegmentData lane_segment_data(int index) const;
  LaneSegmentData front() const {
    CHECK_GE(size(), 1) << "LanePathData is empty!";
    return lane_segment_data(0);
  }
  LaneSegmentData back() const {
    CHECK_GE(size(), 1) << "LanePathData is empty!";
    return lane_segment_data(size() - 1);
  }

 private:
  double start_fraction_ = 0.0;
  double end_fraction_ = 0.0;
  std::vector<ElementId> lane_ids_;
  bool lane_path_in_forward_direction_ = true;

  template <class Archive>
  friend void serialize(Archive& ar, LanePathData& lane_path_data);
};

}  // namespace mapping
}  // namespace st

#endif  // ST_PLANNING_MAPS_LANE_PATH_DATA
