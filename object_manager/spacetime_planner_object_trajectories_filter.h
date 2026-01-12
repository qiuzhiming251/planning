

#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FILTER_H_

#include <limits>

#include "plan_common/path_sl_boundary.h"
#include "plan_common/util/decision_info.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "spacetime_object_trajectory.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
//#include "plan_common/util/hmi_content_util.h"
//#include "object_manager/planner_object.h"

namespace st {
namespace planning {

class SpacetimePlannerObjectTrajectoriesFilter {
 public:
  virtual bool Filter(const SpacetimeObjectTrajectory& traj) const = 0;
  virtual ~SpacetimePlannerObjectTrajectoriesFilter() = default;
};

class CutInSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  CutInSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage,
      const LaneChangeStateProto* lane_change_state, const Box2d& av_box,
      double av_speed);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;              // Not owned.
  const LaneChangeStateProto* lane_change_state_;  // Not owned
  Box2d av_box_;
  std::optional<FrenetBox> av_sl_box_;
  double av_speed_;
};

class TargetBoundarySpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  TargetBoundarySpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage, const PathSlBoundary* sl_boundary,
      const LaneChangeStateProto* lc_state, const Box2d& ego_box,
      const double ego_v)
      : drive_passage_(CHECK_NOTNULL(drive_passage)),
        sl_boundary_(CHECK_NOTNULL(sl_boundary)),
        lc_state_(CHECK_NOTNULL(lc_state)),
        ego_box_(ego_box),
        ego_v_(ego_v) {}
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;     // Not owned.
  const PathSlBoundary* sl_boundary_;     // Not owned.
  const LaneChangeStateProto* lc_state_;  // Not owned
  const Box2d ego_box_;
  const double ego_v_;
};

// Filter cut-in trajs on highway for st planner.
class CutInVehicleSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  CutInVehicleSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage,
      const LaneChangeStateProto* lane_change_state, const Box2d& av_box,
      double av_speed);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;              // Not owned.
  const LaneChangeStateProto* lane_change_state_;  // Not owned
  Box2d av_box_;
  std::optional<FrenetBox> av_sl_box_;
  double av_speed_;
};

// Filter Crossing trajs for st planner.
class CrossingSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  CrossingSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage, const PlannerSemanticMapManager* psmm);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;  // Not owned.
  const PlannerSemanticMapManager* psmm_;
};

// Filter vehicle trajs direction reverse to current lane for st planner.
class ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage, const PlannerSemanticMapManager* psmm,
      const VehicleGeometryParamsProto* vehicle_geometry_params,
      const PathSlBoundary* sl_boundary,
      const NudgeObjectInfo* nudge_object_info, Box2d av_box, double av_speed);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;  // Not owned.
  const PlannerSemanticMapManager* psmm_;
  const VehicleGeometryParamsProto* vehicle_geometry_params_;
  const PathSlBoundary* sl_boundary_;
  const NudgeObjectInfo* nudge_object_info_;
  const Box2d av_box_;
  const double av_speed_;
};

// Filter trajs beyond stop line for st planner.
class BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage,
      absl::Span<const ConstraintProto::StopLineProto> stop_lines);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;  // Not owned.
  double first_stop_line_s_ = std::numeric_limits<double>::infinity();
};

bool IsCutInObjectTrajectory(const DrivePassage& drive_passage,
                             const bool is_lane_change, const double av_speed,
                             const FrenetBox& av_sl_box,
                             const SpacetimeObjectTrajectory& traj);
bool IsFardistObjectTrajectory(const DrivePassage& drive_passage,
                               const bool is_lane_change, const double av_speed,
                               const FrenetBox& av_sl_box,
                               const SpacetimeObjectTrajectory& traj);
}  // namespace planning
}  // namespace st

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FILTER_H_
