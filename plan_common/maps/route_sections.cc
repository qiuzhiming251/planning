

#include <algorithm>

#include "absl/status/status.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/maps/route_sections.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/plan_common_defs.h"

namespace st::planning {

RouteSections RouteSections::BuildFromLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  std::vector<mapping::SectionId> section_ids;
  section_ids.reserve(lane_path.size());

  for (const auto& seg : lane_path) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, seg.lane_id);
    section_ids.push_back(lane_info.section_id());
  }

  return RouteSections(lane_path.front().fraction(),
                       lane_path.back().fraction(), std::move(section_ids),
                       lane_path.back());
}

RouteSections RouteSections::BuildFromProto(
    const RouteSectionSequenceProto& proto) {
  std::vector<mapping::SectionId> section_ids;

  section_ids.reserve(proto.section_id_size());

  for (const auto& section_id : proto.section_id()) {
    section_ids.push_back(mapping::SectionId(section_id));
  }

  return RouteSections(proto.start_fraction(), proto.end_fraction(),
                       std::move(section_ids),
                       mapping::LanePoint(proto.destination()));
}

RouteSections::RouteSectionSegment RouteSections::route_section_segment(
    int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, size());

  return RouteSectionSegment{
      .id = section_ids_[index],
      .start_fraction = index == 0 ? start_fraction_ : 0.0,
      .end_fraction = index + 1 == size() ? end_fraction_ : 1.0};
}

absl::StatusOr<int> RouteSections::FindSectionSegment(
    const RouteSectionSegment& segment) const {
  for (int i = 0; i < section_ids_.size(); ++i) {
    if (section_ids_[i] == segment.id) {
      const auto this_seg = route_section_segment(i);

      if (segment.start_fraction >= this_seg.start_fraction &&
          segment.end_fraction <= this_seg.end_fraction) {
        return i;
      } else {
        return absl::OutOfRangeError("");
      }
    }
  }
  return absl::NotFoundError("");
}

double RouteSections::planning_horizon(
    const PlannerSemanticMapManager& psmm) const {
  if (!planning_horizon_.has_value()) {
    // Calculate only once.
    double horizon = 0.0;
    double accum_time = 0.0, last_speed_limit{0};
    bool stopped_early = false;
    int idx = 0;
    for (; idx < section_ids_.size(); ++idx) {
      SMM_ASSIGN_SECTION_OR_BREAK(sec_info, psmm, section_ids_[idx]);
      if (const auto& first_lane_ptr =
              psmm.FindLaneByIdOrNull(sec_info.lanes()[0]);
          first_lane_ptr == nullptr) {
        break;
      }

      const double start_frac = (idx == 0 ? start_fraction_ : 0.0);
      const double end_frac =
          (idx + 1 == section_ids_.size() ? end_fraction_ : 1.0);
      const double seg_len = sec_info.curve_length() * (end_frac - start_frac);
      const auto& lane_ids = sec_info.lanes();

      double max_speed_limit = 0.0;
      for (const auto lane_id : lane_ids) {
        max_speed_limit =
            std::max(max_speed_limit, psmm.QueryLaneSpeedLimitById(lane_id));
      }
      last_speed_limit = max_speed_limit;
      const double time_inc = seg_len / max_speed_limit;
      if (accum_time + time_inc > kPlanningTimeHorizon) {
        horizon += (kPlanningTimeHorizon - accum_time) * max_speed_limit;
        stopped_early = true;
        break;
      }
      accum_time += time_inc;
      horizon += seg_len;
    }
    if (!stopped_early) {
      horizon += (kPlanningTimeHorizon - accum_time) * last_speed_limit;
    }
    planning_horizon_ = horizon;
  }
  return *planning_horizon_;
}

double RouteSections::planning_horizon(
    const ad_byd::planning::Map& v2smm) const {
  if (!planning_horizon_.has_value()) {
    // Calculate only once.
    double horizon = 0.0;
    double accum_time = 0.0, last_speed_limit{0};
    bool stopped_early = false;
    int idx = 0;
    for (; idx < section_ids_.size(); ++idx) {
      SMM2_ASSIGN_SECTION_OR_BREAK(sec_info, v2smm, section_ids_[idx]);
      if (sec_info.lanes().empty()) {
        break;
      }
      if (const auto first_lane_ptr =
              v2smm.GetLaneById(mapping::ElementId(sec_info.lanes()[0]));
          first_lane_ptr == nullptr) {
        break;
      }

      const double start_frac = (idx == 0 ? start_fraction_ : 0.0);
      const double end_frac =
          (idx + 1 == section_ids_.size() ? end_fraction_ : 1.0);
      const double seg_len = sec_info.curve_length() * (end_frac - start_frac);
      const auto& lane_ids = sec_info.lanes();

      double max_speed_limit = 0.0;
      for (const auto lane_id : lane_ids) {
        SMM2_ASSIGN_LANE_OR_CONTINUE(lane, v2smm, mapping::ElementId(lane_id));
        max_speed_limit = std::max(max_speed_limit, lane.speed_limit());
      }
      last_speed_limit = max_speed_limit;
      const double time_inc = seg_len / max_speed_limit;
      if (accum_time + time_inc > kPlanningTimeHorizon) {
        horizon += (kPlanningTimeHorizon - accum_time) * max_speed_limit;
        stopped_early = true;
        break;
      }
      accum_time += time_inc;
      horizon += seg_len;
    }
    if (!stopped_early) {
      horizon += (kPlanningTimeHorizon - accum_time) * last_speed_limit;
    }
    planning_horizon_ = horizon;
  }
  return *planning_horizon_;
}

void RouteSections::ToProto(RouteSectionSequenceProto* proto) const {
  proto->Clear();

  proto->set_start_fraction(start_fraction_);

  proto->set_end_fraction(end_fraction_);

  proto->mutable_section_id()->Reserve(section_ids_.size());

  for (const auto& id : section_ids_) {
    proto->add_section_id(id);
  }

  destination_.ToProto(proto->mutable_destination());
}

}  // namespace st::planning
