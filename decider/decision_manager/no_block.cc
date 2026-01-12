

#include <map>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"
#include "plan_common/container/strong_int.h"
#include "decider/decision_manager/decision_util.h"
#include "decider/decision_manager/no_block.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/math/vec.h"

namespace st {
namespace planning {
std::vector<ConstraintProto::SpeedRegionProto> BuildNoBlockConstraints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset) {
  std::map<mapping::ElementId, std::vector<ConstraintProto::SpeedRegionProto>>
      id_element_map;
  for (const auto& seg : lane_path_from_start) {
    const auto lane_info_ptr =
        planner_semantic_map_manager.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) continue;
    if (lane_info_ptr->junction_id() == 0) continue;
    // for (const auto& [intersection_id, intersection_frac] :
    //      lane_info_ptr->Intersections()) {
    // Start point.
    const double start_point_s =
        lane_path_from_start.LaneIndexPointToArclength(
            /*lane_index_point=*/{seg.lane_index, 0.0}) +
        s_offset + passage.lane_path_start_s();
    const auto start_point = lane_info_ptr->LerpPointFromFraction(0.0);
    // End point.
    const double end_point_s = lane_path_from_start.LaneIndexPointToArclength(
                                   /*lane_index_point=*/{seg.lane_index, 1.0}) +
                               s_offset + passage.lane_path_start_s();
    const auto end_point = lane_info_ptr->LerpPointFromFraction(1.0);

    ConstraintProto::SpeedRegionProto no_block_constraint;
    start_point.ToProto(no_block_constraint.mutable_start_point());
    end_point.ToProto(no_block_constraint.mutable_end_point());
    no_block_constraint.set_start_s(start_point_s);
    no_block_constraint.set_end_s(end_point_s);
    no_block_constraint.set_max_speed(
        planner_semantic_map_manager.QueryLaneSpeedLimitById(seg.lane_id));
    constexpr double kIntersectionMinSpeed = 0.5;  // m/s
    no_block_constraint.set_min_speed(kIntersectionMinSpeed);

    no_block_constraint.mutable_source()->mutable_no_block()->set_id(
        absl::StrCat(lane_info_ptr->junction_id()));

    no_block_constraint.set_id(
        absl::StrCat("no_block_", lane_info_ptr->junction_id()));

    id_element_map[lane_info_ptr->junction_id()].push_back(
        std::move(no_block_constraint));
    // }
  }
  std::vector<ConstraintProto::SpeedRegionProto> no_block_constraints;
  no_block_constraints.reserve(id_element_map.size());
  for (const auto& id_element : id_element_map) {
    no_block_constraints.push_back(MergeSameElement(id_element.second));
  }
  return no_block_constraints;
}

}  // namespace planning
}  // namespace st
