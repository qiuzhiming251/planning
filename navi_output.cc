

#include "router/navi/navi_output.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <iterator>
#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_util.h"
//#include "semantic_map.pb.h"

namespace st::planning::route {
namespace {
NaviActionProto MakeNavAction(
    const std::optional<NaviActionProto::NaviActionType>& nav_action_type,
    const std::optional<NaviActionProto::SubNaviActionType>& sub_action_type,
    const Vec2d& point, mapping::SectionId section_id) {
  NaviActionProto nav_action_proto;
  if (nav_action_type.has_value()) {
    nav_action_proto.set_navi_action_type(*nav_action_type);
  }
  if (sub_action_type.has_value()) {
    nav_action_proto.set_sub_action_type(*sub_action_type);
  }
  point.ToProto(nav_action_proto.mutable_start_point());
  return nav_action_proto;
}

std::optional<NaviActionProto::NaviActionType> GetNavActionType(
    const ad_byd::planning::Lane& lane_proto,
    const mapping::LaneProto* last_lane_proto_ptr) {
  if (last_lane_proto_ptr != nullptr &&
      lane_proto.direction() == last_lane_proto_ptr->direction()) {
    return std::nullopt;
  }
  switch (lane_proto.direction()) {
    case mapping::LaneProto::STRAIGHT:
      if (lane_proto.is_in_intersection()) {
        return NaviActionProto::STRAIGHT;
      }
      break;
    case mapping::LaneProto::LEFT_TURN:
      return NaviActionProto::TURN_LEFT;

    case mapping::LaneProto::RIGHT_TURN:
      return NaviActionProto::TURN_RIGHT;

    case mapping::LaneProto::UTURN:
      return NaviActionProto::U_TURN;
  }
  return std::nullopt;
}

std::optional<NaviActionProto::SubNaviActionType> GetSubNavActionType(
    const ad_byd::planning::Lane& lane_proto,
    const mapping::LaneProto* last_lane_proto) {
  if ((lane_proto.type() == mapping::LaneProto_Type_RAMP ||
       lane_proto.type() == mapping::LaneProto_Type_RAMP_JCT) &&
      last_lane_proto != nullptr &&
      (last_lane_proto->type() != mapping::LaneProto_Type_RAMP ||
       lane_proto.type() == mapping::LaneProto_Type_RAMP_JCT)) {
    return NaviActionProto::ENTER_RAMP;
  }
  return std::nullopt;
}

}  // namespace

// infos. Now it is just std::transform the object.
template <typename NaviInfoIt>
void MergeNaviAction(
    NaviInfoIt first, NaviInfoIt last,
    google::protobuf::RepeatedPtrField<NaviSegmentProto>* navi_segments) {
  navi_segments->Clear();
  navi_segments->Reserve(std::distance(first, last));
  for (auto it = first; it != last; ++it) {
    auto* navi_segment_ptr = navi_segments->Add();
    navi_segment_ptr->set_length(it->length);
    *navi_segment_ptr->add_navi_actions() = it->navi_action;
  }
}

}  // namespace st::planning::route
