

#ifndef ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_
#define ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_

#include <optional>

#include "absl/status/statusor.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st::planning {

// This is the builder function for est planner.
absl::StatusOr<DrivePassage> BuildDrivePassage(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePath& backward_extended_lane_path,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    const mapping::LanePoint& destination, bool all_lanes_virtual = false,
    std::optional<double> override_speed_limit_mps = std::nullopt,
    FrenetFrameType type = FrenetFrameType::kQtfmKdTree,
    ad_byd::planning::LaneSeqInfoPtr lane_seq_info = nullptr);

absl::StatusOr<DrivePassage> BuildDrivePassageFromLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step_s, bool avoid_loop, bool avoid_notcontinuous,
    double backward_extend_len, double required_planning_horizon,
    std::optional<double> override_speed_limit_mps,
    FrenetFrameType type = FrenetFrameType::kQtfmKdTree);

absl::StatusOr<DrivePassage> BuildDrivePassageForBevMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose, double step_s,
    bool avoid_loop, double backward_extend_len,
    double required_planning_horizon,
    std::optional<double> override_speed_limit_mps = std::nullopt,
    FrenetFrameType type = FrenetFrameType::kQtfmKdTree,
    ad_byd::planning::LaneSeqInfoPtr lane_seq_info = nullptr);

absl::StatusOr<DrivePassage> BuildDrivePassageV2(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePath& backward_extended_lane_path,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    const mapping::LanePoint& destination, bool all_lanes_virtual,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type,
    ad_byd::planning::LaneSeqInfoPtr lane_seq_info);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_
