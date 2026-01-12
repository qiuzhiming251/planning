

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/maps/lane_path_data.h"

namespace st {
namespace mapping {

void LanePathData::FromLanePathProto(const LanePathProto& lane_path_proto) {
  lane_ids_.clear();
  lane_ids_.reserve(lane_path_proto.lane_ids_size());
  for (const auto& lane_id : lane_path_proto.lane_ids()) {
    lane_ids_.push_back(ElementId(lane_id));
  }
  start_fraction_ = lane_path_proto.start_fraction();
  end_fraction_ = lane_path_proto.end_fraction();
  if (lane_path_proto.has_lane_path_in_forward_direction()) {
    lane_path_in_forward_direction_ =
        lane_path_proto.lane_path_in_forward_direction();
  } else {
    lane_path_in_forward_direction_ = true;
  }
}

void LanePathData::ToLanePathProto(LanePathProto* lane_path_proto) const {
  lane_path_proto->Clear();
  lane_path_proto->mutable_lane_ids()->Reserve(lane_ids_.size());
  for (const auto& lane_id : lane_ids_) {
    lane_path_proto->add_lane_ids(lane_id);
  }
  lane_path_proto->set_start_fraction(start_fraction_);
  lane_path_proto->set_end_fraction(end_fraction_);
  lane_path_proto->set_lane_path_in_forward_direction(
      lane_path_in_forward_direction_);
}

void LanePathData::Clear() {
  start_fraction_ = 0.0;
  end_fraction_ = 0.0;
  lane_ids_.clear();
}

std::string LanePathData::DebugString() const {
  return absl::StrFormat("lane ids: %s, start_fraction: %f, end_fraction: %f.",
                         absl::StrJoin(lane_ids_, ", "), start_fraction_,
                         end_fraction_);
}

LanePathData::LaneSegmentData LanePathData::lane_segment_data(int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, size());
  return LaneSegmentData{
      .lane_id = lane_id(index),
      .start_fraction = index == 0 ? start_fraction_ : 0.0,
      .end_fraction = index + 1 == size() ? end_fraction_ : 1.0,
  };
}

}  // namespace mapping
}  // namespace st
